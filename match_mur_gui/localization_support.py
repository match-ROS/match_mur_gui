"""Start or reuse local localization inputs; never stop externally owned nodes."""
import argparse
import fcntl
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy


def amcl_launch_arguments(robot, map_present, output):
    return ['ros2', 'launch', 'match_mur_navigation', 'localization.launch.py',
            'robot:='+robot, 'localize:=true', 'stationary_updates:=false',
            'load_map:='+str(not map_present).lower(), 'output_directory:='+str(output)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['mocap', 'amcl'], required=True)
    parser.add_argument('--robot', choices=['mur620a', 'mur620b', 'mur620c', 'mur620d'], default='mur620c')
    args = parser.parse_args()
    directory = Path(os.environ['WS'])/'src/match_mur_gui/logs/localization'
    directory.mkdir(parents=True, exist_ok=True)
    domain = os.environ.get('ROS_DOMAIN_ID', '62')
    lock_path = directory/(args.mode+'_'+(args.robot if args.mode == 'amcl' else 'shared')+'_'+domain+'.lock')
    lock = lock_path.open('a')
    child = None
    def stop(_signal, _frame):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('base_localization_support_'+args.mode+'_'+args.robot)
    alignment = []
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(TFMessage, '/tf_static', lambda m: alignment.extend(
        t for t in m.transforms if (t.header.frame_id, t.child_frame_id) == ('map', 'mocap')), latched)
    client = node.create_client(SetBool, '/qualisys/enable_map_output')
    future = None
    locked = configured = False
    started = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if child is not None and child.poll() is not None:
                raise RuntimeError('Localization child exited with code '+str(child.returncode))
            if configured or time.monotonic()-started < 2.5:
                continue
            if not locked:
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    locked = True
                except BlockingIOError:
                    continue
            if args.mode == 'amcl':
                ns = '/'+args.robot+'/localization'
                if node.count_publishers(ns+'/amcl_pose'):
                    node.get_logger().info('Vorhandene AMCL-Sitzung wird verwendet; GUI übernimmt deren Prozesse nicht.')
                elif node.count_publishers(ns+'/comparison'):
                    raise RuntimeError('AMCL-Prozess fehlt; ein alter Lokalisierungsadapter läuft noch. '
                                       'Die alte Lokalisierung auf ihrem Rechner beenden und erneut starten.')
                else:
                    command = amcl_launch_arguments(args.robot, node.count_publishers(ns+'/map') > 0, directory)
                    child = subprocess.Popen(command, start_new_session=True)
                    node.get_logger().info('AMCL gestartet: '+args.robot)
                configured = True
            elif client.service_is_ready():
                if future is None:
                    future = client.call_async(SetBool.Request(data=True))
                elif future.done():
                    result = future.result()
                    if not result.success:
                        raise RuntimeError(result.message)
                    configured = True
                    node.get_logger().info(result.message)
            elif alignment and ('qualisys_ssh_bridge' in node.get_node_names() or any(
                    node.count_publishers('/qualisys/'+robot+'/pose') for robot in
                    ('mur620a', 'mur620b', 'mur620c', 'mur620d'))):
                # Legacy calibrated bridges have no enable-map service.
                configured = True
                node.get_logger().info('Vorhandene map->mocap-Kalibrierung und Qualisys-Bridge werden verwendet.')
            elif ('qualisys_ssh_bridge' in node.get_node_names() or any(
                    node.count_publishers('/qualisys/'+robot+'/pose') for robot in
                    ('mur620a', 'mur620b', 'mur620c', 'mur620d'))):
                if time.monotonic()-started > 12:
                    raise RuntimeError('Vorhandene Mocap-Bridge liefert keine Map-Kalibrierung. '
                                       'Bridge nach dem Update einmal neu starten.')
            else:
                child = subprocess.Popen([sys.executable, '-m', 'match_mocap_ros2.qualisys_ssh_bridge',
                    '--ros-args', '-p', 'publish_map_pose:=true', '-p', 'publish_robot_tf:=false'],
                    start_new_session=True)
                configured = True
                node.get_logger().info('Qualisys-Empfang mit kalibrierter Map-Ausgabe gestartet.')
    except KeyboardInterrupt:
        pass
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                try:
                    child.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
        node.destroy_node()
        rclpy.shutdown()
        lock.close()


if __name__ == '__main__':
    main()
