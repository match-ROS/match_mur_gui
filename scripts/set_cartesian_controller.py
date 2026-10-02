#!/usr/bin/env python3
"""Explicitly enable/disable Cartesian control; never start a UR program."""
import argparse

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.node import Node
from ur_dashboard_msgs.srv import IsProgramRunning

CONTROLLER = 'integrated_cartesian_admittance_controller'


def call(node, kind, service, request):
    client = node.create_client(kind, service)
    try:
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f'Service unavailable: {service}')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=6.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'Service timed out: {service}')
        return future.result()
    finally:
        node.destroy_client(client)


def switch_request(controllers, disable):
    states = {c.name: c for c in controllers}
    target = states.get(CONTROLLER)
    if target is None or target.state not in ('active', 'inactive'):
        raise RuntimeError('Cartesian controller must be loaded and configured first')
    request = SwitchController.Request()
    request.strictness = SwitchController.Request.STRICT
    request.timeout.sec = 5
    if disable:
        request.deactivate_controllers = [CONTROLLER] if target.state == 'active' else []
    else:
        # Do not take over another motion mode implicitly. A trajectory hold
        # controller from driver startup is the only permitted replacement.
        if states.get('freedrive_mode_controller') is not None and states['freedrive_mode_controller'].state == 'active':
            raise RuntimeError('Disable freedrive before enabling Cartesian control')
        conflicts = [c.name for c in controllers if c.state == 'active' and
                     any(interface.rsplit('/', 1)[-1] in ('position', 'velocity', 'effort')
                         for interface in c.claimed_interfaces) and c.name != CONTROLLER]
        allowed = {'scaled_joint_trajectory_controller', 'joint_trajectory_controller'}
        if any(name not in allowed for name in conflicts):
            raise RuntimeError(f'Another motion controller is active: {conflicts}')
        request.activate_controllers = [CONTROLLER] if target.state != 'active' else []
        request.deactivate_controllers = conflicts
    return request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm-namespace', required=True)
    parser.add_argument('--disable', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = Node('set_cartesian_controller')
    ns = '/' + args.arm_namespace.strip('/')
    try:
        if not args.disable:
            response = call(node, IsProgramRunning, ns + '/dashboard_client/program_running',
                            IsProgramRunning.Request())
            if not response.success or not response.program_running:
                raise RuntimeError('External Control program is not running; refusing activation')
        controllers = call(node, ListControllers, ns + '/controller_manager/list_controllers',
                           ListControllers.Request()).controller
        request = switch_request(controllers, args.disable)
        if request.activate_controllers or request.deactivate_controllers:
            result = call(node, SwitchController, ns + '/controller_manager/switch_controller', request)
            if not result.ok:
                raise RuntimeError('Controller switch rejected')
        print(f'{ns}: Cartesian controller {"disabled" if args.disable else "enabled"}', flush=True)
        return 0
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
