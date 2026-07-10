#!/usr/bin/env python3
"""Keep the current TCP position while aligning tool0 with a base-frame plane."""

import argparse
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mur_control.action import JparseMove
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from match_mur_gui.alignment import (
    PLANE_ALIGNMENT_BY_KEY,
    alignment_quaternion,
    plane_alignment,
)


class AlignArmToPlane(Node):
    def __init__(self, args):
        node_suffix = f"{args.robot_name}_{args.arm}".replace("-", "_")
        super().__init__(f"align_arm_to_plane_{node_suffix}")
        self.args = args
        self.arm_name = f"UR10_{args.arm}"
        self.base_frame = f"{args.robot_name}/{self.arm_name}/base_link"
        self.tip_frame = f"{args.robot_name}/{self.arm_name}/tool0"
        self.action_name = args.action_name or f"/{args.robot_name}/jparse_move_{args.arm}"
        self.alignment = plane_alignment(args.alignment)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, JparseMove, self.action_name)
        self.goal_handle = None
        self.stop_requested = False

    def request_stop(self):
        self.stop_requested = True

    def wait_for_future(self, future, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and not self.stop_requested:
            if time.monotonic() >= deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.done()

    def current_pose(self):
        deadline = time.monotonic() + self.args.tf_timeout
        last_error = None
        while rclpy.ok() and not self.stop_requested and time.monotonic() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.tip_frame,
                    rclpy.time.Time(),
                )
                translation = transform.transform.translation
                return (translation.x, translation.y, translation.z)
            except TransformException as exc:
                last_error = exc
                rclpy.spin_once(self, timeout_sec=0.05)
        if self.stop_requested:
            return None
        raise RuntimeError(
            f"Could not lookup current TCP pose {self.base_frame}->{self.tip_frame}: {last_error}"
        )

    def target_pose(self, position):
        quaternion = alignment_quaternion(self.alignment)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = quaternion
        return pose

    def cancel_active_goal(self):
        if self.goal_handle is None:
            return
        stop_requested = self.stop_requested
        future = self.goal_handle.cancel_goal_async()
        self.stop_requested = False
        self.wait_for_future(future, 1.0)
        self.goal_handle = None
        self.stop_requested = stop_requested

    def cancel_pending_goal(self, send_future):
        stop_requested = self.stop_requested
        self.stop_requested = False
        accepted = self.wait_for_future(send_future, 1.0)
        self.stop_requested = stop_requested
        if not accepted:
            return
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return
        self.goal_handle = goal_handle
        self.cancel_active_goal()

    def run(self):
        position = self.current_pose()
        if self.stop_requested or position is None:
            return 130
        if not self.action_client.wait_for_server(timeout_sec=self.args.server_timeout):
            self.get_logger().error(f"Alignment action is unavailable: {self.action_name}")
            return 2
        if self.stop_requested:
            return 130

        goal = JparseMove.Goal()
        goal.mode = "task_space"
        goal.accuracy = "precision"
        goal.target_pose = self.target_pose(position)
        goal.max_linear_velocity = self.args.max_linear_velocity
        goal.max_angular_velocity = self.args.max_angular_velocity
        goal.timeout = self.args.timeout

        self.get_logger().info(
            f"Aligning {self.args.robot_name}/{self.arm_name} to {self.alignment.plane} "
            f"from {self.alignment.side_label}: {self.alignment.direction_label}; "
            f"holding TCP position ({position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}) "
            f"in {self.base_frame}"
        )
        send_future = self.action_client.send_goal_async(goal)
        if not self.wait_for_future(send_future, self.args.server_timeout):
            if self.stop_requested:
                self.cancel_pending_goal(send_future)
                self.get_logger().warn("Alignment canceled before goal acceptance")
                return 130
            self.get_logger().error("Timed out while sending alignment goal")
            return 3
        self.goal_handle = send_future.result()
        if self.goal_handle is None or not self.goal_handle.accepted:
            self.get_logger().error("Alignment goal was rejected")
            return 4

        result_future = self.goal_handle.get_result_async()
        if not self.wait_for_future(result_future, self.args.timeout + 5.0):
            if self.stop_requested:
                self.cancel_active_goal()
                self.get_logger().warn("Alignment canceled")
                return 130
            self.cancel_active_goal()
            self.get_logger().error("Alignment result timed out")
            return 5
        if self.stop_requested:
            self.cancel_active_goal()
            self.get_logger().warn("Alignment canceled")
            return 130

        wrapped_result = result_future.result()
        result = wrapped_result.result
        level = self.get_logger().info if result.success else self.get_logger().warn
        level(
            f"Alignment finished: success={result.success}, message={result.message}, "
            f"position_error={result.final_position_error:.4f} m, "
            f"orientation_error={result.final_orientation_error:.4f} rad"
        )
        return 0 if result.success else 6


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-name", default="mur620d")
    parser.add_argument("--arm", choices=("l", "r"), required=True)
    parser.add_argument(
        "--alignment",
        choices=tuple(sorted(PLANE_ALIGNMENT_BY_KEY)),
        required=True,
    )
    parser.add_argument("--action-name", default="")
    parser.add_argument("--max-linear-velocity", type=float, default=0.015)
    parser.add_argument("--max-angular-velocity", type=float, default=0.10)
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--server-timeout", type=float, default=3.0)
    parser.add_argument("--tf-timeout", type=float, default=3.0)
    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = AlignArmToPlane(args)

    def request_stop(_signum, _frame):
        node.request_stop()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        exit_code = node.run()
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"Alignment failed: {exc}")
        exit_code = 1
    finally:
        if node.stop_requested:
            node.cancel_active_goal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
