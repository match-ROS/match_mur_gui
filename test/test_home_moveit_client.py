"""Home request construction and timing, with isolated fake ROS servers and no hardware."""
import importlib.util
from pathlib import Path

import pytest
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory

spec = importlib.util.spec_from_file_location(
    'home_client', Path(__file__).parents[1] / 'scripts/move_arm_to_named_pose.py')
home = importlib.util.module_from_spec(spec)
spec.loader.exec_module(home)


def test_named_pose_uses_selected_group_from_server_srdf():
    srdf = '''<robot name="test">
      <group_state name="Home_custom" group="UR_arm_l"><joint name="left" value="1.5"/></group_state>
      <group_state name="Home_custom" group="UR_arm_r"><joint name="right" value="-0.5"/></group_state>
    </robot>'''
    constraints = home.named_pose_constraints(srdf, 'UR_arm_r', 'Home_custom')
    assert len(constraints.joint_constraints) == 1
    assert constraints.joint_constraints[0].joint_name == 'right'
    assert constraints.joint_constraints[0].position == -0.5
    with pytest.raises(ValueError):
        home.named_pose_constraints(srdf, 'wrong_group', 'Home_custom')


@pytest.mark.parametrize('value', ['nan', 'inf'])
def test_nonfinite_targets_rejected(value):
    with pytest.raises(ValueError):
        home.named_pose_constraints(
            f'<robot><group_state name="Home" group="arm"><joint name="j" value="{value}"/></group_state></robot>',
            'arm', 'Home')


def test_twenty_percent_speed_preserves_positions_and_scales_time():
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = ['joint']
    for t, pos in [(0, 0.0), (1, 0.5), (2, 1.0)]:
        point = JointTrajectoryPoint(positions=[pos], velocities=[0.5], accelerations=[0.1])
        point.time_from_start.sec = t
        trajectory.joint_trajectory.points.append(point)
    home.scale_joint_trajectory_speed(trajectory, 0.2, hold_duration=0.8)
    points = trajectory.joint_trajectory.points
    assert list(points[1].positions) == [0.5]
    assert points[1].velocities == pytest.approx([0.1])
    assert points[1].accelerations == pytest.approx([0.004])
    assert home.duration_to_seconds(points[-1].time_from_start) == pytest.approx(10.8)
    assert list(points[-1].positions) == [1.0]


@pytest.mark.parametrize('ready_after_plan, execute_success', [(True, True), (False, True), (True, False)])
def test_home_uses_remote_servers_and_checks_execution_result(ready_after_plan, execute_success, monkeypatch):
    import threading
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.action import ActionServer
    from moveit_msgs.action import ExecuteTrajectory
    from moveit_msgs.srv import GetMotionPlan
    from ur_dashboard_msgs.srv import IsProgramRunning

    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", "/tmp/mur_home_test_logs")
    rclpy.init(domain_id=198)
    server = Node('move_group', namespace='/mur620')
    server.declare_parameter('robot_description_semantic',
        '<robot><group_state name="Home_custom" group="UR_arm_r"><joint name="joint" value="1"/></group_state></robot>')
    calls = {'ready': 0, 'execute': 0}

    def ready(request, response):
        calls['ready'] += 1
        response.success = True
        response.program_running = calls['ready'] == 1 or ready_after_plan
        return response

    def plan(request, response):
        assert request.motion_plan_request.group_name == 'UR_arm_r'
        assert request.motion_plan_request.start_state.is_diff
        response.motion_plan_response.error_code.val = 1
        trajectory = response.motion_plan_response.trajectory.joint_trajectory
        trajectory.joint_names = ['joint']
        for t, p in [(0, 0.0), (1, 1.0)]:
            point = JointTrajectoryPoint(positions=[p])
            point.time_from_start.sec = t
            trajectory.points.append(point)
        return response

    def execute(goal):
        calls['execute'] += 1
        if execute_success:
            goal.succeed()
        else:
            goal.abort()
        result = ExecuteTrajectory.Result()
        result.error_code.val = 1 if execute_success else -4
        return result

    server.create_service(IsProgramRunning, '/mur620/UR10_r/dashboard_client/program_running', ready)
    server.create_service(GetMotionPlan, '/mur620/plan_kinematic_path', plan)
    action = ActionServer(server, ExecuteTrajectory, '/mur620/execute_trajectory', execute)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    client = home.MoveArmToNamedPose()
    try:
        if ready_after_plan and execute_success:
            assert client.run() == 0
        else:
            with pytest.raises(RuntimeError):
                client.run()
        assert calls['ready'] == 2
        assert calls['execute'] == (1 if ready_after_plan else 0)
    finally:
        client.cancel_active()
        client.destroy_node()
        executor.shutdown()
        thread.join()
        action.destroy()
        server.destroy_node()
        rclpy.shutdown()
