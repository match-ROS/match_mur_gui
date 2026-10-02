#!/usr/bin/env python3
"""Plan Home through the running move_group; execute with bounded cancellation."""
import copy
import math
import signal
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from action_msgs.msg import GoalStatus
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from ur_dashboard_msgs.srv import IsProgramRunning


SIDES = {"r": "UR_arm_r", "l": "UR_arm_l"}


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def duration_to_seconds(duration):
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def set_duration_from_seconds(duration, seconds):
    seconds = max(0.0, seconds)
    duration.sec = int(seconds)
    duration.nanosec = int(round((seconds - duration.sec) * 1e9))
    if duration.nanosec >= 1_000_000_000:
        duration.sec += 1
        duration.nanosec -= 1_000_000_000


def scale_joint_trajectory_speed(
    robot_trajectory,
    velocity_scaling,
    reference_state=None,
    hold_duration=0.8,
):
    velocity_scaling = clamp(float(velocity_scaling), 0.01, 1.0)

    trajectory_msg = get_robot_trajectory_msg(robot_trajectory)
    points = trajectory_msg.joint_trajectory.points
    if points:
        if velocity_scaling < 0.999:
            time_scale = 1.0 / velocity_scaling
            for point in points:
                set_duration_from_seconds(
                    point.time_from_start,
                    duration_to_seconds(point.time_from_start) * time_scale,
                )
                point.velocities = [value * velocity_scaling for value in point.velocities]
                point.accelerations = [
                    value * velocity_scaling * velocity_scaling
                    for value in point.accelerations
                ]

        points[0].velocities = [0.0] * len(points[0].positions)
        points[0].accelerations = [0.0] * len(points[0].positions)
        points[-1].velocities = [0.0] * len(points[-1].positions)
        points[-1].accelerations = [0.0] * len(points[-1].positions)

        if hold_duration > 0.0:
            hold_point = copy.deepcopy(points[-1])
            hold_point.velocities = [0.0] * len(hold_point.positions)
            hold_point.accelerations = [0.0] * len(hold_point.positions)
            set_duration_from_seconds(
                hold_point.time_from_start,
                duration_to_seconds(points[-1].time_from_start) + hold_duration,
            )
            points.append(hold_point)

    if hasattr(robot_trajectory, "set_robot_trajectory_msg") and reference_state is not None:
        robot_trajectory.set_robot_trajectory_msg(reference_state, trajectory_msg)
    return robot_trajectory


def get_robot_trajectory_msg(robot_trajectory):
    if hasattr(robot_trajectory, "joint_trajectory"):
        return robot_trajectory
    if hasattr(robot_trajectory, "get_robot_trajectory_msg"):
        return robot_trajectory.get_robot_trajectory_msg()
    raise AttributeError(
        f"Unsupported trajectory type '{type(robot_trajectory).__name__}': "
        "missing joint_trajectory and get_robot_trajectory_msg()"
    )


def named_pose_constraints(srdf, group, pose):
    root = ET.fromstring(srdf)
    state = next((s for s in root.findall("group_state")
                  if s.get("group") == group and s.get("name") == pose), None)
    if state is None:
        raise ValueError(f"Named pose {group}/{pose} is missing in the running MoveIt SRDF")
    constraints = Constraints(name=pose)
    for joint in state.findall("joint"):
        value = float(joint.attrib["value"])
        if not math.isfinite(value):
            raise ValueError("Non-finite named joint position")
        constraints.joint_constraints.append(JointConstraint(
            joint_name=joint.attrib["name"], position=value,
            tolerance_above=0.001, tolerance_below=0.001, weight=1.0))
    if not constraints.joint_constraints:
        raise ValueError("Named pose contains no joints")
    return constraints


class MoveArmToNamedPose(Node):
    def __init__(self):
        super().__init__("move_arm_to_named_pose")
        for name, value in (("robot_name", "mur620"), ("robot_profile", "mur620d"),
                            ("arm", "r"), ("group", ""), ("named_pose", "Home_custom"),
                            ("velocity_scaling", 0.2), ("hold_duration", 0.8)):
            self.declare_parameter(name, value)
        self.robot_name = self.get_parameter("robot_name").value
        self.arm = self.get_parameter("arm").value
        if self.arm not in SIDES:
            raise ValueError("arm must be l or r")
        self.group = self.get_parameter("group").value or SIDES[self.arm]
        if self.group != SIDES[self.arm]:
            raise ValueError("Planning group must match the selected arm")
        self.named_pose = self.get_parameter("named_pose").value
        speed = float(self.get_parameter("velocity_scaling").value)
        hold = float(self.get_parameter("hold_duration").value)
        if not math.isfinite(speed) or not math.isfinite(hold):
            raise ValueError("Speed and hold duration must be finite")
        self.scale = clamp(speed, 0.01, 1.0)
        self.hold = max(0.0, hold)
        ns = '/' + self.robot_name.strip('/')
        self.parameters = self.create_client(GetParameters, ns + '/move_group/get_parameters')
        self.planner = self.create_client(GetMotionPlan, ns + '/plan_kinematic_path')
        self.ready = self.create_client(
            IsProgramRunning, ns + f'/UR10_{self.arm}/dashboard_client/program_running')
        self.executor_client = ActionClient(self, ExecuteTrajectory, ns + '/execute_trajectory')
        self.active_goal = None
        self.interrupted = False
        # Leave time for cancellation before the GUI's outer 90 s process limit.
        self.deadline = time.monotonic() + 70.0

    def wait(self, future, timeout):
        deadline = min(self.deadline, time.monotonic() + timeout)
        while rclpy.ok() and not future.done():
            if self.interrupted or time.monotonic() >= deadline:
                raise RuntimeError("Home interrupted or timed out")
            rclpy.spin_once(self, timeout_sec=0.05)
        result = future.result()
        if result is None:
            raise RuntimeError("ROS request returned no result")
        return result

    def call(self, client, request, timeout=3.0):
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f"Service unavailable: {client.srv_name}")
        return self.wait(client.call_async(request), timeout)

    def check_ready(self):
        status = self.call(self.ready, IsProgramRunning.Request())
        if not status.success or not status.program_running:
            raise RuntimeError("UR External Control program is not running; refusing execution")

    def cancel_active(self):
        if self.active_goal is None:
            return
        future = self.active_goal.cancel_goal_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done() or future.result() is None or not future.result().goals_canceling:
            self.get_logger().error("Execution cancellation was not confirmed; check controller status")
        else:
            self.get_logger().warn("Execution cancellation accepted")
        self.active_goal = None

    def run(self):
        self.check_ready()
        self.get_logger().info(f"Planning {self.group} to {self.named_pose} using running move_group")
        params = self.call(self.parameters, GetParameters.Request(names=['robot_description_semantic']))
        if not params.values or not params.values[0].string_value:
            raise RuntimeError("Running move_group has no robot_description_semantic")
        request = GetMotionPlan.Request()
        request.motion_plan_request.group_name = self.group
        request.motion_plan_request.pipeline_id = 'ompl'
        request.motion_plan_request.allowed_planning_time = 5.0
        request.motion_plan_request.num_planning_attempts = 1
        request.motion_plan_request.start_state.is_diff = True
        request.motion_plan_request.max_velocity_scaling_factor = 1.0
        request.motion_plan_request.max_acceleration_scaling_factor = 1.0
        request.motion_plan_request.goal_constraints = [named_pose_constraints(
            params.values[0].string_value, self.group, self.named_pose)]
        response = self.call(self.planner, request, timeout=15.0).motion_plan_response
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"Planning failed: MoveIt error {response.error_code.val}")
        trajectory = response.trajectory
        if not trajectory.joint_trajectory.points:
            raise RuntimeError("Planner returned an empty trajectory")
        scale_joint_trajectory_speed(trajectory, self.scale, hold_duration=self.hold)
        if not self.executor_client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError("MoveIt execute_trajectory server unavailable")
        # Readiness can change during planning (as in the reported incident).
        self.check_ready()
        self.get_logger().info("Planning succeeded; executing trajectory via running move_group")
        send = self.executor_client.send_goal_async(ExecuteTrajectory.Goal(trajectory=trajectory))
        try:
            self.active_goal = self.wait(send, 5.0)
        except RuntimeError:
            # If acknowledgement arrives late, cancel this goal before shutdown.
            rclpy.spin_until_future_complete(self, send, timeout_sec=3.0)
            if send.done() and send.result() is not None and send.result().accepted:
                self.active_goal = send.result()
            raise
        if not self.active_goal.accepted:
            self.active_goal = None
            raise RuntimeError("MoveIt rejected execution")
        result = self.wait(self.active_goal.get_result_async(), 60.0)
        self.active_goal = None
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"Execution failed: action={result.status}, MoveIt={result.result.error_code.val}")
        self.get_logger().info("Completed trajectory execution with status SUCCEEDED")
        return 0


def main():
    rclpy.init()
    node = None
    code = 1
    try:
        node = MoveArmToNamedPose()
        signal.signal(signal.SIGTERM, lambda *_: setattr(node, 'interrupted', True))
        signal.signal(signal.SIGINT, lambda *_: setattr(node, 'interrupted', True))
        code = node.run()
    except Exception as exc:
        if node is not None:
            node.get_logger().error(f"Home failed: {exc}")
        else:
            print(f"Home initialization failed: {exc}", file=sys.stderr)
    finally:
        if node is not None:
            node.cancel_active()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
