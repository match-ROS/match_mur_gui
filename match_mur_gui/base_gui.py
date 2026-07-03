#!/usr/bin/env python3
"""Reusable PyQt base GUI for MuR robot operation."""

import os
import json
import re
import shlex
import threading
import time
import math
from functools import partial

from PyQt5 import QtCore, QtGui, QtWidgets

import rclpy
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped
from lifecycle_msgs.msg import State, TransitionEvent
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger
from ewellix_interfaces.msg import Command as EwellixCommand
from mir_srvs.srv import ColorRGB


WS = os.environ.get("WS", "/home/rosmatch/colcon_ws")
REMOTE_WS_DEFAULT = os.environ.get("REMOTE_WS", "/home/rosmatch/colcon_ws")
HARDWARE_SCRIPT = os.path.join(
    WS, "src", "match_mobile_robotics_jazzy", "start_mur620_hardware_logged.sh"
)
HARDWARE_LATEST_LOG = os.path.join(
    WS, "src", "match_mobile_robotics_jazzy", "logs", "hardware", "latest.log"
)
GUI_LOG_DIR = os.path.join(WS, "src", "match_mur_gui", "logs", "gui")
GUI_LATEST_LOG = os.path.join(GUI_LOG_DIR, "latest.log")
MIR_POSES_FILE = os.environ.get(
    "MIR_GUI_POSES_FILE",
    os.path.join(WS, "src", "match_mur_gui", "config", "mir_poses.json"),
)
MIR_DEFAULT_NAMESPACE = os.environ.get("MIR_GUI_NAMESPACE", "")
REMOTE_HOST_SETUP_REL = os.path.join(
    "src", "match_mobile_robotics_jazzy", "setup_mur_hardware_host.sh"
)
REMOTE_HOST_DIAG_REL = os.path.join(
    "src", "match_mobile_robotics_jazzy", "diagnose_mur_hardware_host.sh"
)
REMOTE_UR_DASHBOARD_SAFETY_CHECK_REL = os.path.join(
    "src", "match_mur_gui", "scripts", "ur_dashboard_safety_check.py"
)
ROBOTS = ["mur620a", "mur620b", "mur620c", "mur620d"]
SIDES = {"r": "UR10_r", "l": "UR10_l"}
FREEDRIVE_CONTROLLER = "freedrive_mode_controller"
FREEDRIVE_ENABLE_WAIT_SEC = 3.0
FREEDRIVE_ACTIVE_TRANSITION_WAIT_SEC = 4.0
FREEDRIVE_KEEPALIVE_HZ = 10.0
UR_REVERSE_READY_TEXT = "Robot connected to reverse interface. Ready to receive control commands."
UR_REVERSE_WAIT_SEC = 12.0
UR_READY_RETRY_LIMIT = 1
MOTION_CONTROLLERS = [
    "integrated_cartesian_admittance_controller",
    "forward_velocity_controller",
    "scaled_joint_trajectory_controller",
    "joint_trajectory_controller",
    "forward_position_controller",
    "forward_effort_controller",
    "force_mode_controller",
    "passthrough_trajectory_controller",
    "tool_contact_controller",
    FREEDRIVE_CONTROLLER,
]
LOG_LEVELS = ("error", "warning", "info")
ERROR_LOG_RE = re.compile(
    r"\[(error|fatal)\]|"
    r"\b(error|failed|failure|exception|traceback|could not)\b"
)
WARNING_LOG_RE = re.compile(
    r"\[(warn|warning)\]|"
    r"\b(warning|warn|refusing|aborted|unavailable|timeout|timed out)\b"
)


def setup_prefix(ws=WS):
    return (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {shlex.quote(os.path.join(ws, 'install', 'setup.bash'))} && "
        f"export ROS_DOMAIN_ID={shlex.quote(os.environ.get('ROS_DOMAIN_ID', '62'))} && "
        "export ROS2CLI_NO_DAEMON=1 && "
        "export PYTHONUNBUFFERED=1 && "
        "export RCUTILS_LOGGING_BUFFERED_STREAM=0 && "
    )


def normalize_mir_namespace(namespace):
    return (namespace or "").strip().strip("/")


def ros_topic(namespace, suffix):
    suffix = suffix.strip("/")
    namespace = normalize_mir_namespace(namespace)
    if namespace:
        return f"/{namespace}/{suffix}"
    return f"/{suffix}"


def namespaced_frame(namespace, frame):
    namespace = normalize_mir_namespace(namespace)
    frame = frame.strip("/")
    if namespace and frame != "map":
        return f"{namespace}/{frame}"
    return frame


def namespace_from_ros_name(name, suffixes):
    name = (name or "").strip("/")
    if not name:
        return None
    for suffix in suffixes:
        suffix = suffix.strip("/")
        if name == suffix:
            return ""
        marker = "/" + suffix
        if name.endswith(marker):
            return name[: -len(marker)].strip("/")
    return None


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def quaternion_to_yaw(orientation):
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)


class BatteryBadge(QtWidgets.QWidget):
    def __init__(self, label, parent=None):
        super().__init__(parent)
        self.label = label
        self.value = None
        self.setFixedSize(78, 24)
        self.setToolTip(f"{label} battery: no data")

    def set_value(self, value):
        if value is None or not math.isfinite(value):
            self.value = None
            self.setToolTip(f"{self.label} battery: no data")
        else:
            self.value = max(0.0, min(100.0, float(value)))
            self.setToolTip(f"{self.label} battery: {self.value:.0f}%")
        self.update()

    def color(self):
        if self.value is None:
            return QtGui.QColor("#a0aec0")
        if self.value >= 50.0:
            return QtGui.QColor("#48bb78")
        if self.value >= 30.0:
            return QtGui.QColor("#ecc94b")
        if self.value >= 15.0:
            return QtGui.QColor("#ed8936")
        return QtGui.QColor("#e53e3e")

    def paintEvent(self, _event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)

        outline = QtGui.QColor("#4a5568")
        body = QtCore.QRectF(1.5, 5.0, 24.0, 14.0)
        terminal = QtCore.QRectF(25.5, 9.0, 3.0, 6.0)
        painter.setPen(QtGui.QPen(outline, 1.2))
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawRoundedRect(body, 2.0, 2.0)
        painter.drawRect(terminal)

        fill_width = 0.0 if self.value is None else (body.width() - 4.0) * self.value / 100.0
        if fill_width > 0.0:
            fill = QtCore.QRectF(body.left() + 2.0, body.top() + 2.0, fill_width, body.height() - 4.0)
            painter.setPen(QtCore.Qt.NoPen)
            painter.setBrush(self.color())
            painter.drawRoundedRect(fill, 1.2, 1.2)

        text = f"{self.label} --%" if self.value is None else f"{self.label} {self.value:.0f}%"
        painter.setPen(QtGui.QColor("#1a202c"))
        font = painter.font()
        font.setPointSize(8)
        font.setBold(True)
        painter.setFont(font)
        painter.drawText(QtCore.QRectF(32.0, 0.0, 45.0, 24.0), QtCore.Qt.AlignVCenter, text)


class RosWorker(QtCore.QThread):
    log = QtCore.pyqtSignal(str)
    freedrive_status = QtCore.pyqtSignal(str, str, bool, str)
    battery_status = QtCore.pyqtSignal(str, str, float, bool)

    def __init__(self, robot_names=None):
        super().__init__()
        self.robot_names = list(robot_names or ["mur620d"])
        self._node = None
        self._arm_twist_pubs = {}
        self._lift_command_pubs = {}
        self._mir_twist_pubs = {}
        self._mir_goal_pubs = {}
        self._mir_pose_subs = {}
        self._mir_poses = {}
        self._mir_twist_last_warn = {}
        self._joint_positions = {}
        self._joint_state_sub = None
        self._battery_subs = []
        self._previous_freedrive_controllers = {}
        self._freedrive_enable_pubs = {}
        self._freedrive_keepalive = {}
        self._freedrive_keepalive_last = {}
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._lock = threading.Lock()

    def run(self):
        rclpy.init(args=None)
        self._node = rclpy.create_node("mur_gui")
        self._joint_state_sub = self._node.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 50
        )
        self._configure_battery_subscriptions(self.robot_names)
        self._ready.set()
        self.log.emit("[ros] GUI ROS helper started")
        executor = SingleThreadedExecutor()
        executor.add_node(self._node)
        try:
            while rclpy.ok() and not self._stop.is_set():
                executor.spin_once(timeout_sec=0.05)
                self._publish_freedrive_keepalives()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            if rclpy.ok():
                self._stop_all_freedrive_keepalives()
            if self._node is not None:
                executor.remove_node(self._node)
                self._node.destroy_node()
                self._node = None
            executor.shutdown()
            if rclpy.ok():
                rclpy.shutdown()

    def shutdown(self):
        self._stop.set()

    def set_robot_names(self, robot_names):
        self.robot_names = list(robot_names or ["mur620d"])
        if self._ready.wait(timeout=1.0):
            self._configure_battery_subscriptions(self.robot_names)

    def _configure_battery_subscriptions(self, robot_names):
        with self._lock:
            if self._node is None:
                return
            for sub in self._battery_subs:
                self._node.destroy_subscription(sub)
            self._battery_subs = []
            for robot_name in robot_names:
                mur_sub = self._node.create_subscription(
                    Float32,
                    f"/{robot_name}/bms_status/SOC",
                    partial(self._on_mur_battery, robot_name),
                    10,
                )
                mir_sub = self._node.create_subscription(
                    BatteryState,
                    f"/{robot_name}/battery_state",
                    partial(self._on_mir_battery, robot_name),
                    10,
                )
                self._battery_subs.extend([mur_sub, mir_sub])

    def _on_mur_battery(self, robot_name, msg):
        self.battery_status.emit(robot_name, "mur", float(msg.data), True)

    def _on_mir_battery(self, robot_name, msg):
        percentage = float(msg.percentage)
        if 0.0 <= percentage <= 1.0:
            percentage *= 100.0
        self.battery_status.emit(robot_name, "mir", percentage, math.isfinite(percentage))

    def _on_joint_states(self, msg):
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                self._joint_positions[name] = float(position)

    def current_lift_height(self, robot_name, side):
        candidates = [
            "right_lift_joint" if side == "r" else "left_lift_joint",
            f"{robot_name}/right_lift_joint" if side == "r" else f"{robot_name}/left_lift_joint",
            f"{robot_name}/UR10_{side}/right_lift_joint"
            if side == "r"
            else f"{robot_name}/UR10_{side}/left_lift_joint",
        ]
        with self._lock:
            for name in candidates:
                if name in self._joint_positions:
                    return self._joint_positions[name]
        return None

    def call_trigger(self, service_name, label):
        if not self._ready.wait(timeout=1.0):
            self.log.emit(f"[ros] Cannot call {label}: ROS helper not ready")
            return
        with self._lock:
            client = self._node.create_client(Trigger, service_name)
        if not client.wait_for_service(timeout_sec=0.5):
            self.log.emit(f"[ros] {label}: service unavailable: {service_name}")
            return
        future = client.call_async(Trigger.Request())

        def done(done_future):
            try:
                result = done_future.result()
                self.log.emit(
                    f"[ros] {label}: success={result.success}, message='{result.message}'"
                )
            except Exception as exc:  # noqa: BLE001
                self.log.emit(f"[ros] {label}: failed: {exc}")

        future.add_done_callback(done)

    def call_color_rgb(self, service_name, red, green, blue, label):
        if not self._ready.wait(timeout=1.0):
            self.log.emit(f"[ros] Cannot call {label}: ROS helper not ready")
            return
        with self._lock:
            client = self._node.create_client(ColorRGB, service_name)
        if not client.wait_for_service(timeout_sec=0.5):
            self.log.emit(f"[ros] {label}: service unavailable: {service_name}")
            return
        request = ColorRGB.Request()
        request.red = int(max(0, min(255, red)))
        request.green = int(max(0, min(255, green)))
        request.blue = int(max(0, min(255, blue)))
        future = client.call_async(request)

        def done(done_future):
            try:
                result = done_future.result()
                self.log.emit(f"[ros] {label}: success={result.success}")
            except Exception as exc:  # noqa: BLE001
                self.log.emit(f"[ros] {label}: failed: {exc}")

        future.add_done_callback(done)

    def publish_mir_twist(self, namespace, linear_x=0.0, angular_z=0.0):
        if self._node is None:
            return
        namespace = normalize_mir_namespace(namespace)
        linear_x = float(linear_x)
        angular_z = float(angular_z)
        cmd_vel_topic = ros_topic(namespace, "cmd_vel")
        stamped_topic = ros_topic(namespace, "cmd_vel_stamped")

        use_plain_twist = self._topic_has_subscription(cmd_vel_topic, "geometry_msgs/msg/Twist")
        use_stamped_twist = self._topic_has_subscription(
            stamped_topic,
            "geometry_msgs/msg/TwistStamped",
        )
        if use_plain_twist:
            self._publish_mir_plain_twist(cmd_vel_topic, linear_x, angular_z)
            return
        self._publish_mir_stamped_twist(stamped_topic, namespace, linear_x, angular_z)
        if not use_stamped_twist and (abs(linear_x) > 1e-6 or abs(angular_z) > 1e-6):
            now = time.monotonic()
            last_warn = self._mir_twist_last_warn.get(namespace, 0.0)
            if now - last_warn > 2.0:
                self._mir_twist_last_warn[namespace] = now
                self.log.emit(
                    "[ros] MiR jog: no subscriber discovered on "
                    f"{cmd_vel_topic} [Twist] or {stamped_topic} [TwistStamped]"
                )

    def _topic_has_subscription(self, topic, topic_type):
        try:
            infos = self._node.get_subscriptions_info_by_topic(topic)
        except Exception:  # noqa: BLE001
            return False
        return any(getattr(info, "topic_type", "") == topic_type for info in infos)

    def _publish_mir_plain_twist(self, topic, linear_x, angular_z):
        with self._lock:
            publisher = self._mir_twist_pubs.get((topic, "twist"))
            if publisher is None:
                publisher = self._node.create_publisher(Twist, topic, 10)
                self._mir_twist_pubs[(topic, "twist")] = publisher
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        publisher.publish(msg)

    def _publish_mir_stamped_twist(self, topic, namespace, linear_x, angular_z):
        with self._lock:
            publisher = self._mir_twist_pubs.get((topic, "twist_stamped"))
            if publisher is None:
                publisher = self._node.create_publisher(TwistStamped, topic, 10)
                self._mir_twist_pubs[(topic, "twist_stamped")] = publisher
        msg = TwistStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = namespaced_frame(namespace, "base_link")
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        publisher.publish(msg)

    def discover_mir_namespaces(self):
        if not self._ready.wait(timeout=1.0) or self._node is None:
            return []

        topic_suffixes = {
            "cmd_vel",
            "cmd_vel_stamped",
            "robot_pose",
            "scan",
            "f_scan",
            "b_scan",
            "f_raw_scan",
            "b_raw_scan",
        }
        service_suffixes = {
            "RGB_control/rainbow_start",
            "RGB_control/rainbow_stop",
            "RGB_control/match_color",
            "RGB_control/solid_color",
        }
        namespaces = set()
        with self._lock:
            try:
                topics = self._node.get_topic_names_and_types()
                services = self._node.get_service_names_and_types()
            except Exception as exc:  # noqa: BLE001
                self.log.emit(f"[ros] MiR namespace discovery failed: {exc}")
                return []

        for name, _types in topics:
            namespace = namespace_from_ros_name(name, topic_suffixes)
            if namespace is not None:
                namespaces.add(namespace)
        for name, _types in services:
            namespace = namespace_from_ros_name(name, service_suffixes)
            if namespace is not None:
                namespaces.add(namespace)
        return sorted(namespaces)

    def publish_mir_goal(self, namespace, frame_id, x, y, yaw):
        if self._node is None:
            return False, "ROS helper not ready"
        topic = ros_topic(namespace, "move_base_simple/goal")
        with self._lock:
            publisher = self._mir_goal_pubs.get(topic)
            if publisher is None:
                publisher = self._node.create_publisher(PoseStamped, topic, 10)
                self._mir_goal_pubs[topic] = publisher
        msg = PoseStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = frame_id.strip() or "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        publisher.publish(msg)
        if publisher.get_subscription_count() == 0:
            return False, f"published goal on {topic}, no ROS2 subscriber discovered"
        return True, f"published goal on {topic}: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f} deg"

    def ensure_mir_pose_subscription(self, namespace):
        if self._node is None:
            return
        namespace = normalize_mir_namespace(namespace)
        topic = ros_topic(namespace, "robot_pose")
        with self._lock:
            if topic in self._mir_pose_subs:
                return
            sub = self._node.create_subscription(
                Pose,
                topic,
                partial(self._on_mir_pose, namespace),
                10,
            )
            self._mir_pose_subs[topic] = sub

    def _on_mir_pose(self, namespace, msg):
        with self._lock:
            self._mir_poses[namespace] = (
                float(msg.position.x),
                float(msg.position.y),
                quaternion_to_yaw(msg.orientation),
            )

    def current_mir_pose(self, namespace):
        namespace = normalize_mir_namespace(namespace)
        self.ensure_mir_pose_subscription(namespace)
        with self._lock:
            return self._mir_poses.get(namespace)

    def _duration_msg(self, seconds):
        duration = SwitchController.Request().timeout
        duration.sec = int(seconds)
        duration.nanosec = int((seconds - int(seconds)) * 1_000_000_000)
        return duration

    def _wait_for_future(self, future, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            self.msleep(20)
        return future.done()

    def _controller_manager(self, robot_name, side):
        return f"/{robot_name}/{SIDES[side]}/controller_manager"

    def _list_controller_states(self, controller_manager):
        with self._lock:
            client = self._node.create_client(ListControllers, f"{controller_manager}/list_controllers")
        if not client.wait_for_service(timeout_sec=1.0):
            return None, f"service unavailable: {controller_manager}/list_controllers"
        future = client.call_async(ListControllers.Request())
        if not self._wait_for_future(future, 2.0):
            return None, f"timeout listing controllers at {controller_manager}"
        response = future.result()
        if response is None:
            return None, f"empty list_controllers response from {controller_manager}"
        return {controller.name: controller.state for controller in response.controller}, ""

    def _load_controller(self, controller_manager, controller_name):
        with self._lock:
            client = self._node.create_client(LoadController, f"{controller_manager}/load_controller")
        if not client.wait_for_service(timeout_sec=2.0):
            return False, f"service unavailable: {controller_manager}/load_controller"
        request = LoadController.Request()
        request.name = controller_name
        future = client.call_async(request)
        if not self._wait_for_future(future, 3.0):
            return False, f"timeout loading {controller_name}"
        response = future.result()
        if response is None or not response.ok:
            return False, f"failed loading {controller_name}"
        return True, f"loaded {controller_name}"

    def _configure_controller(self, controller_manager, controller_name):
        with self._lock:
            client = self._node.create_client(
                ConfigureController, f"{controller_manager}/configure_controller"
            )
        if not client.wait_for_service(timeout_sec=2.0):
            return False, f"service unavailable: {controller_manager}/configure_controller"
        request = ConfigureController.Request()
        request.name = controller_name
        future = client.call_async(request)
        if not self._wait_for_future(future, 3.0):
            return False, f"timeout configuring {controller_name}"
        response = future.result()
        if response is None or not response.ok:
            return False, f"failed configuring {controller_name}"
        return True, f"configured {controller_name}"

    def _ensure_controller_loaded(self, controller_manager, controller_name):
        states, error = self._list_controller_states(controller_manager)
        if states is None:
            return None, error
        if controller_name not in states:
            ok, message = self._load_controller(controller_manager, controller_name)
            self.log.emit(f"[ros] {controller_name}: {message}")
            if not ok:
                return None, message
            states, error = self._list_controller_states(controller_manager)
            if states is None:
                return None, error

        if states.get(controller_name) == "unconfigured":
            ok, message = self._configure_controller(controller_manager, controller_name)
            self.log.emit(f"[ros] {controller_name}: {message}")
            if not ok:
                return None, message
            states, error = self._list_controller_states(controller_manager)
            if states is None:
                return None, error
        return states, ""

    def _wait_for_controller_state(self, controller_manager, controller_name, target_state, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        last_state = "missing"
        last_error = ""
        while time.monotonic() < deadline:
            states, error = self._list_controller_states(controller_manager)
            if states is None:
                last_error = error
            else:
                last_state = states.get(controller_name, "missing")
                if last_state == target_state:
                    return True, f"{controller_name} is {target_state}"
            self.msleep(50)
        detail = last_error or f"last state was '{last_state}'"
        return False, f"timeout waiting for {controller_name} to become {target_state}: {detail}"

    def _freedrive_enable_topic(self, robot_name, side):
        return f"/{robot_name}/{SIDES[side]}/{FREEDRIVE_CONTROLLER}/enable_freedrive_mode"

    def _freedrive_transition_topic(self, robot_name, side):
        return f"/{robot_name}/{SIDES[side]}/{FREEDRIVE_CONTROLLER}/transition_event"

    def _get_freedrive_enable_publisher(self, robot_name, side):
        topic = self._freedrive_enable_topic(robot_name, side)
        with self._lock:
            publisher = self._freedrive_enable_pubs.get(topic)
            if publisher is None:
                publisher = self._node.create_publisher(Bool, topic, 10)
                self._freedrive_enable_pubs[topic] = publisher
        return topic, publisher

    def _create_freedrive_active_transition_waiter(self, robot_name, side):
        topic = self._freedrive_transition_topic(robot_name, side)
        event = threading.Event()
        details = {"message": "no transition_event received"}

        def callback(msg):
            goal_label = msg.goal_state.label
            goal_id = msg.goal_state.id
            transition_label = msg.transition.label
            details["message"] = (
                f"transition_event {transition_label}: "
                f"{msg.start_state.label}->{goal_label} ({goal_id})"
            )
            if goal_id == State.PRIMARY_STATE_ACTIVE or goal_label == "active":
                event.set()

        with self._lock:
            subscription = self._node.create_subscription(
                TransitionEvent,
                topic,
                callback,
                10,
            )

        def destroy():
            with self._lock:
                self._node.destroy_subscription(subscription)

        return event, details, destroy

    def _publish_freedrive_enable(self, robot_name, side, enabled):
        topic, publisher = self._get_freedrive_enable_publisher(robot_name, side)

        deadline = time.monotonic() + FREEDRIVE_ENABLE_WAIT_SEC
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            self.msleep(20)

        subscription_count = publisher.get_subscription_count()
        msg = Bool()
        msg.data = bool(enabled)
        publisher.publish(msg)

        if subscription_count == 0:
            return (
                False,
                f"Published {msg.data} on {topic}, but no subscriber was discovered "
                f"within {FREEDRIVE_ENABLE_WAIT_SEC:.1f}s",
            )
        return (
            True,
            f"Published {msg.data} once on {topic} (subscribers={subscription_count})",
        )

    def _set_freedrive_keepalive(self, robot_name, side, enabled):
        key = (robot_name, side)
        with self._lock:
            self._freedrive_keepalive[key] = bool(enabled)
            self._freedrive_keepalive_last[key] = 0.0

    def _publish_freedrive_keepalives(self):
        period = 1.0 / FREEDRIVE_KEEPALIVE_HZ
        now = time.monotonic()
        with self._lock:
            items = list(self._freedrive_keepalive.items())

        for (robot_name, side), enabled in items:
            if not enabled:
                continue
            last = self._freedrive_keepalive_last.get((robot_name, side), 0.0)
            if now - last < period:
                continue
            _topic, publisher = self._get_freedrive_enable_publisher(robot_name, side)
            msg = Bool()
            msg.data = True
            publisher.publish(msg)
            self._freedrive_keepalive_last[(robot_name, side)] = now

    def _stop_all_freedrive_keepalives(self):
        with self._lock:
            keys = list(self._freedrive_keepalive.keys())
        for robot_name, side in keys:
            self._set_freedrive_keepalive(robot_name, side, False)
            self._publish_freedrive_enable(robot_name, side, False)

    def _switch_controllers(self, controller_manager, activate, deactivate, label):
        with self._lock:
            client = self._node.create_client(SwitchController, f"{controller_manager}/switch_controller")
        if not client.wait_for_service(timeout_sec=2.0):
            return False, f"{label}: service unavailable: {controller_manager}/switch_controller"

        states, error = self._list_controller_states(controller_manager)
        if states is not None:
            activate = [name for name in activate if states.get(name) != "active"]
            deactivate = [name for name in deactivate if states.get(name) == "active"]
        if not activate and not deactivate:
            return True, f"{label}: controller state already correct"

        request = SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        request.timeout = self._duration_msg(5.0)
        self.log.emit(f"[ros] {label}: activate={activate}, deactivate={deactivate}")
        future = client.call_async(request)
        if not self._wait_for_future(future, 6.0):
            return False, f"{label}: timeout while switching controllers"
        response = future.result()
        if response is None or not response.ok:
            message = "" if response is None else response.message
            return False, f"{label}: switch failed: {message}"
        return True, f"{label}: switch ok"

    def switch_freedrive(self, robot_name, side, enable, fallback_controller):
        thread = threading.Thread(
            target=self._switch_freedrive_worker,
            args=(robot_name, side, enable, fallback_controller),
            daemon=True,
        )
        thread.start()

    def _switch_freedrive_worker(self, robot_name, side, enable, fallback_controller):
        if not self._ready.wait(timeout=1.0):
            self.log.emit(f"[ros] Cannot switch freedrive for {SIDES[side]}: ROS helper not ready")
            return
        controller_manager = self._controller_manager(robot_name, side)
        key = (robot_name, side)
        if enable:
            states, error = self._ensure_controller_loaded(controller_manager, FREEDRIVE_CONTROLLER)
            if states is None:
                self.log.emit(f"[ros] Freedrive {SIDES[side]}: {error}")
                self.freedrive_status.emit(robot_name, side, False, error)
                return
            active_motion = [
                name
                for name in MOTION_CONTROLLERS
                if name != FREEDRIVE_CONTROLLER and states.get(name) == "active"
            ]
            if active_motion:
                self._previous_freedrive_controllers[key] = active_motion
            elif key not in self._previous_freedrive_controllers:
                self._previous_freedrive_controllers[key] = [fallback_controller]
            transition_event, transition_details, destroy_transition_sub = (
                self._create_freedrive_active_transition_waiter(robot_name, side)
            )
            try:
                ok, message = self._switch_controllers(
                    controller_manager,
                    activate=[FREEDRIVE_CONTROLLER],
                    deactivate=active_motion,
                    label=f"Freedrive ON {SIDES[side]}",
                )
                if ok:
                    saw_transition = transition_event.wait(
                        timeout=FREEDRIVE_ACTIVE_TRANSITION_WAIT_SEC
                    )
                    if saw_transition:
                        message += f"; {transition_details['message']}"
                    else:
                        state_ok, active_message = self._wait_for_controller_state(
                            controller_manager,
                            FREEDRIVE_CONTROLLER,
                            "active",
                            timeout_sec=0.5,
                        )
                        if state_ok:
                            message += (
                                "; no active transition_event observed, "
                                f"but {active_message}"
                            )
                        else:
                            ok = False
                            message += (
                                "; timeout waiting for active transition_event "
                                f"on {self._freedrive_transition_topic(robot_name, side)}; "
                                f"{transition_details['message']}; {active_message}"
                            )
                if ok:
                    publish_ok, publish_message = self._publish_freedrive_enable(
                        robot_name, side, True
                    )
                    message += f"; {publish_message}"
                    ok = publish_ok
                if ok:
                    self._set_freedrive_keepalive(robot_name, side, True)
                    message += f"; keepalive started at {FREEDRIVE_KEEPALIVE_HZ:.1f} Hz"
            finally:
                destroy_transition_sub()
            self.log.emit(f"[ros] {message}")
            self.freedrive_status.emit(robot_name, side, ok, message)
            return

        restore = self._previous_freedrive_controllers.get(key) or [fallback_controller]
        restore = [name for name in restore if name and name != FREEDRIVE_CONTROLLER]
        self._set_freedrive_keepalive(robot_name, side, False)
        publish_ok, publish_message = self._publish_freedrive_enable(robot_name, side, False)
        self.log.emit(f"[ros] Freedrive OFF {SIDES[side]}: {publish_message}")
        ok, message = self._switch_controllers(
            controller_manager,
            activate=restore,
            deactivate=[FREEDRIVE_CONTROLLER],
            label=f"Freedrive OFF {SIDES[side]}",
        )
        if publish_ok:
            message += "; enable_freedrive_mode=false sent"
        self.log.emit(f"[ros] {message}")
        self.freedrive_status.emit(robot_name, side, False if ok else True, message)

    def publish_arm_twist(self, robot_name, side, values):
        if self._node is None:
            return
        prefix = SIDES[side]
        topic = (
            f"/{robot_name}/{prefix}/integrated_cartesian_admittance_controller/"
            "equilibrium_twist_cmd"
        )
        with self._lock:
            publisher = self._arm_twist_pubs.get(topic)
            if publisher is None:
                publisher = self._node.create_publisher(TwistStamped, topic, 10)
                self._arm_twist_pubs[topic] = publisher
        msg = TwistStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = f"{prefix}/base_link"
        msg.twist.linear.x = float(values[0])
        msg.twist.linear.y = float(values[1])
        msg.twist.linear.z = float(values[2])
        msg.twist.angular.x = float(values[3])
        msg.twist.angular.y = float(values[4])
        msg.twist.angular.z = float(values[5])
        publisher.publish(msg)

    def publish_lift_target(self, robot_name, side, meters):
        if self._node is None:
            return False, "ROS helper not ready"
        topic = f"/{robot_name}/ewellix_lift_{side}/command"
        with self._lock:
            publisher = self._lift_command_pubs.get(topic)
            if publisher is None:
                publisher = self._node.create_publisher(EwellixCommand, topic, 10)
                self._lift_command_pubs[topic] = publisher
        msg = EwellixCommand()
        msg.ticks = 0
        msg.meters = float(meters)
        publisher.publish(msg)
        if publisher.get_subscription_count() == 0:
            return False, f"sent lift target {meters:.4f} m on {topic}, no subscriber discovered"
        return True, f"sent lift target {meters:.4f} m on {topic}"


class ManipulatorJogDialog(QtWidgets.QDialog):
    def __init__(self, main_window, side, parent=None):
        super().__init__(parent)
        self.main_window = main_window
        self.ros_worker = main_window.ros_worker
        self.side = side
        self.mode = "translation"
        self.active = [0.0] * 6
        self.lift_hold_direction = 0.0
        self.setWindowTitle(f"Jog {SIDES[side]} + {'right' if side == 'r' else 'left'} lift")
        self.setMinimumWidth(520)

        layout = QtWidgets.QVBoxLayout(self)

        robot_row = QtWidgets.QHBoxLayout()
        robot_row.addWidget(QtWidgets.QLabel("Robot"))
        self.robot_combo = QtWidgets.QComboBox()
        for robot in main_window.selected_robots():
            self.robot_combo.addItem(robot)
        if self.robot_combo.count() == 0:
            self.robot_combo.addItem("mur620d")
        robot_row.addWidget(self.robot_combo, 1)
        self.mode_label = QtWidgets.QLabel("Mode: translation")
        robot_row.addWidget(self.mode_label)
        layout.addLayout(robot_row)

        speed_row = QtWidgets.QHBoxLayout()
        self.linear_speed = QtWidgets.QDoubleSpinBox()
        self.linear_speed.setRange(0.001, 0.2)
        self.linear_speed.setDecimals(3)
        self.linear_speed.setSingleStep(0.005)
        self.linear_speed.setValue(0.01)
        self.angular_speed = QtWidgets.QDoubleSpinBox()
        self.angular_speed.setRange(0.01, 1.0)
        self.angular_speed.setDecimals(3)
        self.angular_speed.setSingleStep(0.05)
        self.angular_speed.setValue(0.1)
        speed_row.addWidget(QtWidgets.QLabel("Linear m/s"))
        speed_row.addWidget(self.linear_speed)
        speed_row.addWidget(QtWidgets.QLabel("Angular rad/s"))
        speed_row.addWidget(self.angular_speed)
        layout.addLayout(speed_row)

        arm_box = QtWidgets.QGroupBox("Cartesian Arm Jog")
        arm_grid = QtWidgets.QGridLayout(arm_box)
        self._add_arm_button(arm_grid, "Y+", 0, 1, [0, 1, 0])
        self._add_arm_button(arm_grid, "X-", 1, 0, [-1, 0, 0])
        stop_button = QtWidgets.QPushButton("STOP")
        stop_button.setMinimumHeight(46)
        stop_button.clicked.connect(self.stop)
        arm_grid.addWidget(stop_button, 1, 1)
        self._add_arm_button(arm_grid, "X+", 1, 2, [1, 0, 0])
        self._add_arm_button(arm_grid, "Y-", 2, 1, [0, -1, 0])
        self._add_arm_button(arm_grid, "Z+", 0, 3, [0, 0, 1])
        self._add_arm_button(arm_grid, "Z-", 2, 3, [0, 0, -1])
        layout.addWidget(arm_box)

        lift_box = QtWidgets.QGroupBox("Lift Jog")
        lift_layout = QtWidgets.QGridLayout(lift_box)
        self.lift_label = QtWidgets.QLabel("height: -- m")
        self.lift_label.setAlignment(QtCore.Qt.AlignCenter)
        self.lift_step = QtWidgets.QDoubleSpinBox()
        self.lift_step.setRange(0.001, 0.05)
        self.lift_step.setDecimals(3)
        self.lift_step.setSingleStep(0.001)
        self.lift_step.setValue(0.010)
        self.lift_min = QtWidgets.QDoubleSpinBox()
        self.lift_min.setRange(0.0, 1.0)
        self.lift_min.setDecimals(3)
        self.lift_min.setValue(0.005)
        self.lift_max = QtWidgets.QDoubleSpinBox()
        self.lift_max.setRange(0.0, 1.0)
        self.lift_max.setDecimals(3)
        self.lift_max.setValue(0.260)
        lift_layout.addWidget(self.lift_label, 0, 0, 1, 4)
        lift_layout.addWidget(QtWidgets.QLabel("step"), 1, 0)
        lift_layout.addWidget(self.lift_step, 1, 1)
        lift_layout.addWidget(QtWidgets.QLabel("min/max"), 1, 2)
        minmax = QtWidgets.QHBoxLayout()
        minmax.addWidget(self.lift_min)
        minmax.addWidget(self.lift_max)
        lift_layout.addLayout(minmax, 1, 3)
        lift_down = QtWidgets.QPushButton("Lift -")
        lift_down.setMinimumHeight(42)
        lift_down.pressed.connect(partial(self.start_lift_jog, -1.0))
        lift_down.released.connect(self.stop_lift_jog)
        lift_up = QtWidgets.QPushButton("Lift +")
        lift_up.setMinimumHeight(42)
        lift_up.pressed.connect(partial(self.start_lift_jog, 1.0))
        lift_up.released.connect(self.stop_lift_jog)
        lift_layout.addWidget(lift_down, 2, 0, 1, 2)
        lift_layout.addWidget(lift_up, 2, 2, 1, 2)
        layout.addWidget(lift_box)

        self.lift_hold_timer = QtCore.QTimer(self)
        self.lift_hold_timer.setInterval(250)
        self.lift_hold_timer.timeout.connect(self.repeat_lift_jog)

        hint = QtWidgets.QLabel("Keys: arrows X/Y, PgUp/PgDn Z, M mode, Space stop")
        hint.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(hint)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(50)

    def robot_name(self):
        return self.robot_combo.currentText().strip() or "mur620d"

    def _add_arm_button(self, grid, text, row, column, vector):
        button = QtWidgets.QPushButton(text)
        button.setMinimumHeight(46)
        button.pressed.connect(partial(self._set_axis, vector))
        button.released.connect(self.stop)
        grid.addWidget(button, row, column)

    def _set_axis(self, vector):
        self.active = [0.0] * 6
        offset = 0 if self.mode == "translation" else 3
        speed = self.linear_speed.value() if self.mode == "translation" else self.angular_speed.value()
        for index, value in enumerate(vector):
            self.active[offset + index] = value * speed

    def _tick(self):
        current = self.ros_worker.current_lift_height(self.robot_name(), self.side)
        if current is None:
            self.lift_label.setText("height: -- m")
        else:
            self.lift_label.setText(f"height: {current:.4f} m")
        self.ros_worker.publish_arm_twist(self.robot_name(), self.side, self.active)

    def stop(self):
        self.active = [0.0] * 6
        self.ros_worker.publish_arm_twist(self.robot_name(), self.side, self.active)

    def toggle_mode(self):
        self.mode = "rotation" if self.mode == "translation" else "translation"
        self.mode_label.setText(f"Mode: {self.mode}")
        self.stop()

    def start_lift_jog(self, direction):
        self.lift_hold_direction = direction
        sent = self.step_lift(direction, log_success=True)
        if sent:
            self.lift_hold_timer.start()

    def repeat_lift_jog(self):
        if self.lift_hold_direction == 0.0:
            self.lift_hold_timer.stop()
            return
        sent = self.step_lift(self.lift_hold_direction, log_success=False)
        if not sent:
            self.stop_lift_jog()

    def stop_lift_jog(self):
        if self.lift_hold_direction != 0.0:
            self.main_window.append_log(f"[gui] lift jog stopped: {self.robot_name()}/{SIDES[self.side]}")
        self.lift_hold_direction = 0.0
        self.lift_hold_timer.stop()

    def step_lift(self, direction, log_success=True):
        current = self.ros_worker.current_lift_height(self.robot_name(), self.side)
        if current is None:
            self.main_window.append_log(
                f"[gui] Refusing lift jog {self.robot_name()}/{SIDES[self.side]}: "
                "no lift joint state on /joint_states"
            )
            return False
        target = current + direction * self.lift_step.value()
        target = max(self.lift_min.value(), min(self.lift_max.value(), target))
        if abs(target - current) < 1.0e-4:
            self.main_window.append_log(
                f"[gui] lift jog limit reached: {self.robot_name()}/{SIDES[self.side]} at {current:.4f} m"
            )
            return False
        ok, message = self.ros_worker.publish_lift_target(self.robot_name(), self.side, target)
        level = "ok" if ok else "warn"
        if log_success or not ok:
            self.main_window.append_log(f"[gui] lift jog {level}: {message}")
        return ok

    def keyPressEvent(self, event):
        key = event.key()
        if key == QtCore.Qt.Key_M:
            self.toggle_mode()
            return
        if key in (QtCore.Qt.Key_Space, QtCore.Qt.Key_Period):
            self.stop()
            return
        mapping = {
            QtCore.Qt.Key_Left: [-1, 0, 0],
            QtCore.Qt.Key_Right: [1, 0, 0],
            QtCore.Qt.Key_Up: [0, 1, 0],
            QtCore.Qt.Key_Down: [0, -1, 0],
            QtCore.Qt.Key_PageUp: [0, 0, 1],
            QtCore.Qt.Key_PageDown: [0, 0, -1],
        }
        if key in mapping:
            self._set_axis(mapping[key])
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.key() in (
            QtCore.Qt.Key_Left,
            QtCore.Qt.Key_Right,
            QtCore.Qt.Key_Up,
            QtCore.Qt.Key_Down,
            QtCore.Qt.Key_PageUp,
            QtCore.Qt.Key_PageDown,
        ):
            self.stop()
            return
        super().keyReleaseEvent(event)

    def closeEvent(self, event):
        self.stop_lift_jog()
        self.stop()
        super().closeEvent(event)


def populate_mir_namespace_combo(combo, main_window, keep_current=True):
    current = combo.currentText().strip() if keep_current else ""
    choices = []

    def add_choice(value):
        value = normalize_mir_namespace(value)
        if value not in choices:
            choices.append(value)

    add_choice(current)
    add_choice(MIR_DEFAULT_NAMESPACE)
    for namespace in main_window.ros_worker.discover_mir_namespaces():
        add_choice(namespace)
    for robot in main_window.selected_robots():
        add_choice(robot)

    choices = [choice for choice in choices if choice]
    if not choices:
        choices = ["mur620d"]

    target = normalize_mir_namespace(current or MIR_DEFAULT_NAMESPACE or choices[0])
    if target not in choices:
        choices.insert(0, target)

    combo.blockSignals(True)
    combo.clear()
    combo.addItems(choices)
    combo.setCurrentText(target)
    combo.blockSignals(False)
    return choices


def create_mir_namespace_combo(main_window, placeholder):
    combo = QtWidgets.QComboBox()
    combo.setEditable(True)
    combo.setInsertPolicy(QtWidgets.QComboBox.NoInsert)
    combo.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
    if combo.lineEdit() is not None:
        combo.lineEdit().setPlaceholderText(placeholder)
    populate_mir_namespace_combo(combo, main_window, keep_current=False)
    return combo



class MiRJogDialog(QtWidgets.QDialog):
    def __init__(self, main_window, parent=None):
        super().__init__(parent)
        self.main_window = main_window
        self.ros_worker = main_window.ros_worker
        self.active_linear = 0.0
        self.active_angular = 0.0
        self.setWindowTitle("MiR Jog")
        self.setMinimumWidth(420)

        layout = QtWidgets.QVBoxLayout(self)
        namespace_row = QtWidgets.QHBoxLayout()
        namespace_row.addWidget(QtWidgets.QLabel("MiR"))
        self.namespace_combo = create_mir_namespace_combo(main_window, "z.B. mur620d")
        namespace_row.addWidget(self.namespace_combo, 1)
        refresh_namespaces = QtWidgets.QPushButton("Refresh")
        refresh_namespaces.clicked.connect(
            lambda: populate_mir_namespace_combo(self.namespace_combo, self.main_window)
        )
        namespace_row.addWidget(refresh_namespaces)
        layout.addLayout(namespace_row)

        speed_row = QtWidgets.QHBoxLayout()
        self.linear_speed = QtWidgets.QDoubleSpinBox()
        self.linear_speed.setRange(0.01, 0.8)
        self.linear_speed.setDecimals(2)
        self.linear_speed.setSingleStep(0.01)
        self.linear_speed.setValue(0.10)
        self.angular_speed = QtWidgets.QDoubleSpinBox()
        self.angular_speed.setRange(0.05, 1.5)
        self.angular_speed.setDecimals(2)
        self.angular_speed.setSingleStep(0.05)
        self.angular_speed.setValue(0.30)
        speed_row.addWidget(QtWidgets.QLabel("Linear m/s"))
        speed_row.addWidget(self.linear_speed)
        speed_row.addWidget(QtWidgets.QLabel("Angular rad/s"))
        speed_row.addWidget(self.angular_speed)
        layout.addLayout(speed_row)

        grid = QtWidgets.QGridLayout()
        self._add_button(grid, "Forward", 0, 1, 1.0, 0.0)
        self._add_button(grid, "Turn L", 1, 0, 0.0, 1.0)
        stop_button = QtWidgets.QPushButton("STOP")
        stop_button.setMinimumHeight(50)
        stop_button.clicked.connect(self.stop)
        grid.addWidget(stop_button, 1, 1)
        self._add_button(grid, "Turn R", 1, 2, 0.0, -1.0)
        self._add_button(grid, "Back", 2, 1, -1.0, 0.0)
        layout.addLayout(grid)

        hint = QtWidgets.QLabel("Keys: arrows drive/turn, Space stop")
        hint.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(hint)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(50)

    def namespace(self):
        return self.namespace_combo.currentText().strip()

    def _add_button(self, grid, text, row, column, linear_sign, angular_sign):
        button = QtWidgets.QPushButton(text)
        button.setMinimumHeight(50)
        button.pressed.connect(partial(self._set_motion, linear_sign, angular_sign))
        button.released.connect(self.stop)
        grid.addWidget(button, row, column)

    def _set_motion(self, linear_sign, angular_sign):
        self.active_linear = float(linear_sign) * self.linear_speed.value()
        self.active_angular = float(angular_sign) * self.angular_speed.value()

    def _tick(self):
        self.ros_worker.publish_mir_twist(
            self.namespace(),
            self.active_linear,
            self.active_angular,
        )

    def stop(self):
        self.active_linear = 0.0
        self.active_angular = 0.0
        for _ in range(3):
            self.ros_worker.publish_mir_twist(self.namespace(), 0.0, 0.0)

    def keyPressEvent(self, event):
        key = event.key()
        if key in (QtCore.Qt.Key_Space, QtCore.Qt.Key_Period):
            self.stop()
            return
        mapping = {
            QtCore.Qt.Key_Up: (1.0, 0.0),
            QtCore.Qt.Key_Down: (-1.0, 0.0),
            QtCore.Qt.Key_Left: (0.0, 1.0),
            QtCore.Qt.Key_Right: (0.0, -1.0),
        }
        if key in mapping:
            self._set_motion(*mapping[key])
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.key() in (
            QtCore.Qt.Key_Left,
            QtCore.Qt.Key_Right,
            QtCore.Qt.Key_Up,
            QtCore.Qt.Key_Down,
        ):
            self.stop()
            return
        super().keyReleaseEvent(event)

    def closeEvent(self, event):
        self.stop()
        super().closeEvent(event)


class MiRLightDialog(QtWidgets.QDialog):
    def __init__(self, main_window, parent=None):
        super().__init__(parent)
        self.main_window = main_window
        self.ros_worker = main_window.ros_worker
        self.color = QtGui.QColor(0, 180, 255)
        self.setWindowTitle("MiR Lights")
        self.setMinimumWidth(440)

        layout = QtWidgets.QVBoxLayout(self)
        namespace_row = QtWidgets.QHBoxLayout()
        namespace_row.addWidget(QtWidgets.QLabel("MiR"))
        self.namespace_combo = create_mir_namespace_combo(main_window, "z.B. mur620d")
        namespace_row.addWidget(self.namespace_combo, 1)
        refresh_namespaces = QtWidgets.QPushButton("Refresh")
        refresh_namespaces.clicked.connect(
            lambda: populate_mir_namespace_combo(self.namespace_combo, self.main_window)
        )
        namespace_row.addWidget(refresh_namespaces)
        layout.addLayout(namespace_row)

        trigger_box = QtWidgets.QGroupBox("Effects")
        trigger_layout = QtWidgets.QGridLayout(trigger_box)
        rainbow_start = QtWidgets.QPushButton("Rainbow Start")
        rainbow_start.clicked.connect(lambda: self._trigger("rainbow_start", "MiR rainbow start"))
        rainbow_stop = QtWidgets.QPushButton("Rainbow Stop")
        rainbow_stop.clicked.connect(lambda: self._trigger("rainbow_stop", "MiR rainbow stop"))
        match_color = QtWidgets.QPushButton("MATCH Color")
        match_color.clicked.connect(lambda: self._trigger("match_color", "MiR MATCH color"))
        trigger_layout.addWidget(rainbow_start, 0, 0)
        trigger_layout.addWidget(rainbow_stop, 0, 1)
        trigger_layout.addWidget(match_color, 1, 0, 1, 2)
        layout.addWidget(trigger_box)

        color_box = QtWidgets.QGroupBox("Solid Color")
        color_layout = QtWidgets.QGridLayout(color_box)
        self.color_button = QtWidgets.QPushButton("Choose Color")
        self.color_button.clicked.connect(self.choose_color)
        self.apply_button = QtWidgets.QPushButton("Apply Solid")
        self.apply_button.clicked.connect(self.apply_solid_color)
        self.color_preview = QtWidgets.QLabel()
        self.color_preview.setFixedSize(48, 28)
        self._update_preview()
        color_layout.addWidget(self.color_preview, 0, 0)
        color_layout.addWidget(self.color_button, 0, 1)
        color_layout.addWidget(self.apply_button, 0, 2)
        layout.addWidget(color_box)

    def namespace(self):
        return self.namespace_combo.currentText().strip()

    def _service(self, name):
        return ros_topic(self.namespace(), f"RGB_control/{name}")

    def _trigger(self, service, label):
        self.ros_worker.call_trigger(self._service(service), label)

    def choose_color(self):
        color = QtWidgets.QColorDialog.getColor(self.color, self, "MiR solid color")
        if color.isValid():
            self.color = color
            self._update_preview()

    def _update_preview(self):
        self.color_preview.setStyleSheet(f"background: {self.color.name()}; border: 1px solid #4a5568;")
        self.color_preview.setToolTip(self.color.name())

    def apply_solid_color(self):
        self.ros_worker.call_color_rgb(
            self._service("solid_color"),
            self.color.red(),
            self.color.green(),
            self.color.blue(),
            f"MiR solid color {self.color.name()}",
        )


class MiRGoalDialog(QtWidgets.QDialog):
    def __init__(self, main_window, parent=None):
        super().__init__(parent)
        self.main_window = main_window
        self.ros_worker = main_window.ros_worker
        self.poses = []
        self.setWindowTitle("MiR Goals")
        self.setMinimumSize(620, 420)

        layout = QtWidgets.QVBoxLayout(self)
        top = QtWidgets.QGridLayout()
        self.namespace_combo = create_mir_namespace_combo(main_window, "z.B. mur620d")
        self.namespace_combo.currentTextChanged.connect(lambda _text: self._ensure_pose_subscription())
        if self.namespace_combo.lineEdit() is not None:
            self.namespace_combo.lineEdit().editingFinished.connect(self._ensure_pose_subscription)
        self.frame_edit = QtWidgets.QLineEdit("map")
        self.name_edit = QtWidgets.QLineEdit()
        self.name_edit.setPlaceholderText("pose name")
        top.addWidget(QtWidgets.QLabel("MiR"), 0, 0)
        namespace_cell = QtWidgets.QHBoxLayout()
        namespace_cell.addWidget(self.namespace_combo, 1)
        refresh_namespaces = QtWidgets.QPushButton("Refresh")
        refresh_namespaces.clicked.connect(
            lambda: populate_mir_namespace_combo(self.namespace_combo, self.main_window)
        )
        namespace_cell.addWidget(refresh_namespaces)
        top.addLayout(namespace_cell, 0, 1)
        top.addWidget(QtWidgets.QLabel("Frame"), 0, 2)
        top.addWidget(self.frame_edit, 0, 3)
        top.addWidget(QtWidgets.QLabel("Name"), 1, 0)
        top.addWidget(self.name_edit, 1, 1, 1, 3)
        layout.addLayout(top)

        coord_box = QtWidgets.QGroupBox("Coordinates")
        coord_layout = QtWidgets.QGridLayout(coord_box)
        self.x_spin = self._coord_spin(-1000.0, 1000.0, 0.0, 0.01, 3)
        self.y_spin = self._coord_spin(-1000.0, 1000.0, 0.0, 0.01, 3)
        self.yaw_spin = self._coord_spin(-180.0, 180.0, 0.0, 1.0, 1)
        coord_layout.addWidget(QtWidgets.QLabel("X [m]"), 0, 0)
        coord_layout.addWidget(self.x_spin, 0, 1)
        coord_layout.addWidget(QtWidgets.QLabel("Y [m]"), 0, 2)
        coord_layout.addWidget(self.y_spin, 0, 3)
        coord_layout.addWidget(QtWidgets.QLabel("Yaw [deg]"), 0, 4)
        coord_layout.addWidget(self.yaw_spin, 0, 5)
        send_coords = QtWidgets.QPushButton("Drive Coordinates")
        send_coords.clicked.connect(self.send_current_fields)
        save_coords = QtWidgets.QPushButton("Save Coordinates")
        save_coords.clicked.connect(self.save_fields)
        save_robot = QtWidgets.QPushButton("Save Current Robot Pose")
        save_robot.clicked.connect(self.save_robot_pose)
        coord_layout.addWidget(send_coords, 1, 0, 1, 2)
        coord_layout.addWidget(save_coords, 1, 2, 1, 2)
        coord_layout.addWidget(save_robot, 1, 4, 1, 2)
        layout.addWidget(coord_box)

        saved_box = QtWidgets.QGroupBox("Saved Poses")
        saved_layout = QtWidgets.QGridLayout(saved_box)
        self.pose_list = QtWidgets.QListWidget()
        self.pose_list.currentRowChanged.connect(self._load_selected_into_fields)
        saved_layout.addWidget(self.pose_list, 0, 0, 1, 4)
        drive_selected = QtWidgets.QPushButton("Drive Selected")
        drive_selected.clicked.connect(self.send_selected)
        delete_selected = QtWidgets.QPushButton("Delete Selected")
        delete_selected.clicked.connect(self.delete_selected)
        reload_button = QtWidgets.QPushButton("Reload")
        reload_button.clicked.connect(self.reload_poses)
        saved_layout.addWidget(drive_selected, 1, 0)
        saved_layout.addWidget(delete_selected, 1, 1)
        saved_layout.addWidget(reload_button, 1, 2)
        layout.addWidget(saved_box, 1)

        self.reload_poses()
        self._ensure_pose_subscription()

    def _coord_spin(self, minimum, maximum, value, step, decimals):
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setValue(value)
        spin.setSingleStep(step)
        spin.setDecimals(decimals)
        return spin

    def namespace(self):
        return self.namespace_combo.currentText().strip()

    def frame_id(self):
        return self.frame_edit.text().strip() or "map"

    def _ensure_pose_subscription(self):
        self.ros_worker.ensure_mir_pose_subscription(self.namespace())

    def reload_poses(self):
        self.poses = []
        try:
            with open(MIR_POSES_FILE, "r", encoding="utf-8") as pose_file:
                payload = json.load(pose_file)
        except FileNotFoundError:
            payload = {"poses": []}
        except (OSError, json.JSONDecodeError) as exc:
            self.main_window.append_log(f"[gui] Failed to load MiR poses from {MIR_POSES_FILE}: {exc}")
            payload = {"poses": []}
        if isinstance(payload, list):
            self.poses = payload
        else:
            self.poses = list(payload.get("poses", []))
        self._refresh_pose_list()

    def _write_poses(self):
        try:
            os.makedirs(os.path.dirname(MIR_POSES_FILE), exist_ok=True)
            with open(MIR_POSES_FILE, "w", encoding="utf-8") as pose_file:
                json.dump({"poses": self.poses}, pose_file, indent=2, sort_keys=True)
                pose_file.write("\n")
        except OSError as exc:
            self.main_window.append_log(f"[gui] Failed to write MiR poses to {MIR_POSES_FILE}: {exc}")
            return False
        self.main_window.append_log(f"[gui] Saved MiR poses to {MIR_POSES_FILE}")
        return True

    def _refresh_pose_list(self):
        self.pose_list.clear()
        for pose in self.poses:
            name = pose.get("name", "unnamed")
            frame = pose.get("frame_id", "map")
            self.pose_list.addItem(f"{name}  ({frame}: {pose.get('x', 0.0):.3f}, {pose.get('y', 0.0):.3f}, {pose.get('yaw_deg', 0.0):.1f} deg)")

    def _selected_pose(self):
        row = self.pose_list.currentRow()
        if row < 0 or row >= len(self.poses):
            return None
        return self.poses[row]

    def _load_selected_into_fields(self):
        pose = self._selected_pose()
        if pose is None:
            return
        self.name_edit.setText(pose.get("name", ""))
        self.frame_edit.setText(pose.get("frame_id", "map"))
        self.x_spin.setValue(float(pose.get("x", 0.0)))
        self.y_spin.setValue(float(pose.get("y", 0.0)))
        self.yaw_spin.setValue(float(pose.get("yaw_deg", 0.0)))

    def _pose_from_fields(self):
        return {
            "name": self.name_edit.text().strip() or time.strftime("pose_%Y%m%d_%H%M%S"),
            "frame_id": self.frame_id(),
            "x": float(self.x_spin.value()),
            "y": float(self.y_spin.value()),
            "yaw_deg": float(self.yaw_spin.value()),
        }

    def _upsert_pose(self, pose):
        for index, existing in enumerate(self.poses):
            if existing.get("name") == pose["name"]:
                self.poses[index] = pose
                break
        else:
            self.poses.append(pose)
        if self._write_poses():
            self._refresh_pose_list()

    def save_fields(self):
        self._upsert_pose(self._pose_from_fields())

    def save_robot_pose(self):
        self._ensure_pose_subscription()
        pose = self.ros_worker.current_mir_pose(self.namespace())
        if pose is None:
            self.main_window.append_log(f"[gui] No MiR robot_pose received on {ros_topic(self.namespace(), 'robot_pose')}")
            return
        x, y, yaw = pose
        self.x_spin.setValue(x)
        self.y_spin.setValue(y)
        self.yaw_spin.setValue(math.degrees(yaw))
        self._upsert_pose(self._pose_from_fields())

    def _send_pose(self, pose):
        yaw = math.radians(float(pose.get("yaw_deg", 0.0)))
        ok, message = self.ros_worker.publish_mir_goal(
            self.namespace(),
            pose.get("frame_id", "map"),
            float(pose.get("x", 0.0)),
            float(pose.get("y", 0.0)),
            yaw,
        )
        level = "ok" if ok else "warn"
        self.main_window.append_log(f"[gui] MiR goal {level}: {message}")

    def send_current_fields(self):
        self._send_pose(self._pose_from_fields())

    def send_selected(self):
        pose = self._selected_pose()
        if pose is None:
            self.main_window.append_log("[gui] No saved MiR pose selected")
            return
        self._send_pose(pose)

    def delete_selected(self):
        row = self.pose_list.currentRow()
        if row < 0 or row >= len(self.poses):
            return
        removed = self.poses.pop(row)
        if self._write_poses():
            self._refresh_pose_list()
            self.main_window.append_log(f"[gui] Deleted MiR pose {removed.get('name', 'unnamed')}")


class MurGuiModule:
    """Base class for externally provided GUI modules."""

    def setup_ui(self, context):
        raise NotImplementedError

    def stop_motion_like_actions(self):
        pass

    def on_hardware_start(self):
        self.stop_motion_like_actions()

    def on_robot_selection_changed(self):
        pass

    def on_shutdown(self):
        pass


class MurGuiContext:
    """Narrow integration surface exposed to application-specific modules."""

    def __init__(self, window):
        self.window = window

    @property
    def ros_worker(self):
        return self.window.ros_worker

    def append_log(self, text):
        self.window.append_log(text)

    def start_process(self, name, command, env=None, on_finished=None):
        self.window.start_process(name, command, env=env, on_finished=on_finished)

    def remote_command(self, robot, command):
        return self.window.remote_command(robot, command)

    def remote_ros_command(self, robot, command):
        return self.window.remote_ros_command(robot, command)

    def remote_setup_prefix(self):
        return self.window.remote_setup_prefix()

    def selected_robots(self):
        return self.window.selected_robots()

    def selected_sides(self):
        return self.window.selected_sides()

    def object_host(self):
        return self.window.object_host()

    def robot_arm_pairs(self, robots=None, sides=None):
        return self.window.robot_arm_pairs(robots=robots, sides=sides)

    def process_key(self, robot, name):
        return self.window.process_key(robot, name)

    def fallback_motion_controller(self):
        return self.window.fallback_motion_controller()

    def set_arm_status(self, robot, side, status):
        self.window.update_arm_status(robot, side, status)

    def arm_status(self, robot, side):
        return self.window.arm_status.get((robot, side), "unknown")

    def ur_reverse_ready(self, robot, side):
        return self.window.ur_reverse_ready.get((robot, side), False)

    def add_action_button(self, text, callback, section="Tools"):
        return self.window.add_action_button(text, callback, section=section)

    def add_tool_button(self, text, callback, section="Tools"):
        return self.window.add_tool_button(text, callback, section=section)

    def add_bottom_widget(self, widget):
        self.window.bottom_layout.addWidget(widget)
        return widget

    def add_status_row(self, label_text, widget):
        self.window.status_layout.addRow(label_text, widget)
        return widget

    def ensure_ur_ready(self, sides=None, robots=None, on_success=None, retry_count=0):
        return self.window.ensure_ur_ready(
            sides=sides,
            robots=robots,
            on_success=on_success,
            retry_count=retry_count,
        )


class MurBaseGui(QtWidgets.QMainWindow):
    def __init__(self, modules=None, window_title="General MuR GUI"):
        super().__init__()
        self.modules = list(modules or [])
        self.module_context = MurGuiContext(self)
        self.setWindowTitle(window_title)
        self.resize(1180, 780)
        self.processes = {}
        self.arm_status = {}
        self.ur_reverse_ready = {}
        self.ur_ready_log_scan_start = {}
        self.freedrive_active = {}
        self.connected_robots = set()
        self.connect_status = {}
        self.connect_messages = {}
        self.battery_values = {}
        self.log_entries = []
        self.log_filters = {level: True for level in LOG_LEVELS}
        self.log_filter_buttons = {}
        self.gui_log_path = self._create_gui_log_file()

        self.ros_worker = RosWorker(["mur620d"])
        self.ros_worker.log.connect(self.append_log)
        self.ros_worker.freedrive_status.connect(self.update_freedrive_status)
        self.ros_worker.battery_status.connect(self.update_battery_status)
        self.ros_worker.start()

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        top = QtWidgets.QHBoxLayout()
        root.addLayout(top)
        top.addWidget(self._build_robot_box())
        top.addWidget(self._build_options_box(), 1)
        top.addWidget(self._build_status_box())

        self.section_container = QtWidgets.QWidget()
        self.section_layout = QtWidgets.QGridLayout(self.section_container)
        self.section_layout.setContentsMargins(0, 0, 0, 0)
        self.section_layout.setHorizontalSpacing(10)
        self.section_layout.setVerticalSpacing(8)
        self._sections = {}
        root.addWidget(self.section_container)

        general_buttons = [
            self.add_action_button("Connect", self.connect_selected_robots, section="General"),
            self.add_action_button("Check Host", self.check_selected_hosts, section="General"),
            self.add_action_button("Start Hardware", self.start_hardware, section="General"),
            self.add_action_button("Open RViz", self.open_rviz, section="General"),
            self.add_action_button("Save GUI Log", self.save_gui_log_snapshot, section="General"),
            self.add_action_button("Stop Managed Processes", self.stop_managed_processes, section="General"),
        ]
        self.connect_button = general_buttons[0]
        self.update_connect_button()

        self.add_action_button("MiR Jog", self.open_mir_jog, section="MiR")
        self.add_action_button("MiR Lights", self.open_mir_lights, section="MiR")
        self.add_action_button("MiR Goals", self.open_mir_goals, section="MiR")

        self.add_action_button("Enable URs / Ready", self.ensure_ur_ready, section="UR")
        self.add_action_button("Home L", partial(self.move_home, "l"), section="UR")
        self.add_action_button("Home R", partial(self.move_home, "r"), section="UR")
        self.add_action_button("Jog L", partial(self.open_manipulator_jog, "l"), section="UR")
        self.add_action_button("Jog R", partial(self.open_manipulator_jog, "r"), section="UR")
        self.freedrive_button = self.add_action_button("Freedrive", self.toggle_freedrive, section="UR")
        self.update_freedrive_button()

        self.terminal = QtWidgets.QPlainTextEdit()
        self.terminal.setReadOnly(True)
        self.terminal.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self.terminal.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        root.addWidget(self.terminal, 1)

        self.bottom_layout = QtWidgets.QHBoxLayout()
        root.addLayout(self.bottom_layout)
        self._build_log_filter_controls()
        self.bottom_layout.addStretch(1)

        for module in self.modules:
            module.setup_ui(self.module_context)

        self.append_log(f"[gui] Logging to {self.gui_log_path}")
        self.append_log("[gui] Ready. General MuR controls are active.")

    def _create_gui_log_file(self):
        os.makedirs(GUI_LOG_DIR, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(GUI_LOG_DIR, f"general_mur_gui_{timestamp}.log")
        with open(path, "a", encoding="utf-8") as log_file:
            log_file.write(f"# General MuR GUI log started {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        try:
            tmp_link = GUI_LATEST_LOG + ".tmp"
            if os.path.lexists(tmp_link):
                os.unlink(tmp_link)
            os.symlink(path, tmp_link)
            os.replace(tmp_link, GUI_LATEST_LOG)
        except OSError:
            pass
        return path

    def _build_robot_box(self):
        box = QtWidgets.QGroupBox("Robot")
        layout = QtWidgets.QFormLayout(box)
        robot_checks = QtWidgets.QWidget()
        robot_layout = QtWidgets.QVBoxLayout(robot_checks)
        robot_layout.setContentsMargins(0, 0, 0, 0)
        self.robot_checks = {}
        self.battery_badges = {}
        for robot in ROBOTS:
            row_widget = QtWidgets.QWidget()
            row = QtWidgets.QHBoxLayout(row_widget)
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(6)
            check = QtWidgets.QCheckBox(robot)
            check.setChecked(robot == "mur620d")
            check.toggled.connect(lambda _checked: self.on_robot_selection_changed())
            self.robot_checks[robot] = check
            row.addWidget(check)
            row.addStretch(1)
            mir_badge = BatteryBadge("MiR")
            mur_badge = BatteryBadge("MuR")
            self.battery_badges[(robot, "mir")] = mir_badge
            self.battery_badges[(robot, "mur")] = mur_badge
            row.addWidget(mir_badge)
            row.addWidget(mur_badge)
            robot_layout.addWidget(row_widget)
        layout.addRow("MuRs", robot_checks)
        self.opt_sync_code = QtWidgets.QCheckBox("Sync Code")
        self.opt_sync_code.setChecked(True)
        layout.addRow(self.opt_sync_code)
        self.remote_ws_edit = QtWidgets.QLineEdit(REMOTE_WS_DEFAULT)
        layout.addRow("Remote WS", self.remote_ws_edit)
        self.arm_r = QtWidgets.QCheckBox("UR10_r")
        self.arm_r.setChecked(True)
        self.arm_r.toggled.connect(lambda _checked: self.update_freedrive_button())
        self.arm_l = QtWidgets.QCheckBox("UR10_l")
        self.arm_l.setChecked(True)
        self.arm_l.toggled.connect(lambda _checked: self.update_freedrive_button())
        self.mir_enabled_check = QtWidgets.QCheckBox("mir")
        self.mir_enabled_check.setChecked(False)
        self.mir_enabled_check.setToolTip("Startet den MiR ROS2/ROS1 Bridge-Treiber beim Hardware-Start")
        layout.addRow(self.arm_r)
        layout.addRow(self.arm_l)
        layout.addRow(self.mir_enabled_check)
        return box

    def _build_options_box(self):
        box = QtWidgets.QGroupBox("Launch Options")
        layout = QtWidgets.QGridLayout(box)
        self.opt_build = self._check("Build before launch", True)
        self.opt_integrated = self._check("Integrated controller", True)
        self.opt_ft = self._check("Use FT sensor", True)
        self.opt_require_wrench = self._check("Require wrench", False)
        self.opt_collision = self._check("Collision avoidance", True)
        self.opt_markers = self._check("Collision markers", False)
        self.opt_zero_admittance = self._check("Zero admittance", False)
        self.opt_moveit = self._check("Launch MoveIt", True)
        self.moveit_speed_label = QtWidgets.QLabel("MoveIt speed: 20%")
        self.moveit_speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.moveit_speed_slider.setRange(1, 100)
        self.moveit_speed_slider.setValue(20)
        self.moveit_speed_slider.valueChanged.connect(self.update_moveit_speed_label)
        for idx, widget in enumerate(
            [
                self.opt_build,
                self.opt_integrated,
                self.opt_ft,
                self.opt_require_wrench,
                self.opt_collision,
                self.opt_markers,
                self.opt_zero_admittance,
                self.opt_moveit,
            ]
        ):
            layout.addWidget(widget, idx // 2, idx % 2)
        row = (len([
            self.opt_build,
            self.opt_integrated,
            self.opt_ft,
            self.opt_require_wrench,
            self.opt_collision,
            self.opt_markers,
            self.opt_zero_admittance,
            self.opt_moveit,
        ]) + 1) // 2
        layout.addWidget(self.moveit_speed_label, row, 0)
        layout.addWidget(self.moveit_speed_slider, row, 1)
        return box

    def _build_status_box(self):
        box = QtWidgets.QGroupBox("Arm Status")
        layout = QtWidgets.QFormLayout(box)
        self.status_layout = layout
        self.status_labels = {}
        for robot in ROBOTS:
            for side, prefix in SIDES.items():
                label = QtWidgets.QLabel("unknown | UR reverse missing")
                self.status_labels[(robot, side)] = label
                layout.addRow(f"{robot}/{prefix}", label)
        return box

    def _check(self, text, checked):
        check = QtWidgets.QCheckBox(text)
        check.setChecked(checked)
        return check

    def _button(self, text, callback):
        button = QtWidgets.QPushButton(text)
        button.clicked.connect(callback)
        return button

    def _build_log_filter_controls(self):
        label = QtWidgets.QLabel("Log Filter:")
        self.bottom_layout.addWidget(label)

        filter_specs = [
            ("error", "Errors", "#c53030"),
            ("warning", "Warnings", "#d69e2e"),
            ("info", "Infos", "#2b6cb0"),
        ]
        for level, text, color in filter_specs:
            button = QtWidgets.QPushButton(text)
            button.setCheckable(True)
            button.setChecked(True)
            button.setToolTip(f"{text} im GUI-Log anzeigen/ausblenden")
            button.toggled.connect(partial(self.set_log_filter, level))
            button.setStyleSheet(
                "QPushButton { padding: 4px 10px; }"
                f"QPushButton:checked {{ background: {color}; color: white; font-weight: bold; }}"
            )
            self.log_filter_buttons[level] = button
            self.bottom_layout.addWidget(button)

    def _section_state(self, title):
        title = title or "Tools"
        state = self._sections.get(title)
        if state is not None:
            return state
        box = QtWidgets.QGroupBox(title)
        layout = QtWidgets.QGridLayout(box)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(6)
        index = len(self._sections)
        self.section_layout.addWidget(box, index // 4, index % 4)
        self.section_layout.setColumnStretch(index % 4, 1)
        state = {"box": box, "layout": layout, "count": 0}
        self._sections[title] = state
        return state

    def _add_button_to_section(self, button, section):
        state = self._section_state(section)
        count = state["count"]
        state["layout"].addWidget(button, count // 2, count % 2)
        state["count"] = count + 1
        return button

    def add_action_button(self, text, callback, section="Tools"):
        return self._add_button_to_section(self._button(text, callback), section)

    def add_tool_button(self, text, callback, section="Tools"):
        return self.add_action_button(text, callback, section=section)

    def update_moveit_speed_label(self, value):
        self.moveit_speed_label.setText(f"MoveIt speed: {value}%")

    def moveit_velocity_scaling(self):
        return max(1, min(100, self.moveit_speed_slider.value())) / 100.0

    def remote_ws(self):
        return self.remote_ws_edit.text().strip() or REMOTE_WS_DEFAULT

    def selected_robots(self):
        robots = [
            robot for robot in ROBOTS
            if self.robot_checks.get(robot) is not None and self.robot_checks[robot].isChecked()
        ]
        return robots or ["mur620d"]

    def object_host(self):
        selected = self.selected_robots()
        for robot in ROBOTS:
            if robot in selected:
                return robot
        return selected[0]

    def robot_profile(self, robot=None):
        return robot or self.object_host()

    def robot_name(self):
        return self.object_host()

    def robot_arm_pairs(self, robots=None, sides=None):
        robots = list(robots or self.selected_robots())
        sides = list(sides or self.selected_sides())
        return [(robot, side) for robot in robots for side in sides]

    def process_key(self, robot, name):
        return f"{robot}:{name}"

    def remote_setup_prefix(self):
        return setup_prefix(self.remote_ws())

    def remote_command(self, robot, command):
        remote_shell = "bash -lc " + shlex.quote(command)
        return f"ssh -o BatchMode=yes {shlex.quote(robot)} {shlex.quote(remote_shell)}"

    def remote_ros_command(self, robot, command):
        return self.remote_command(robot, self.remote_setup_prefix() + command)

    def remote_hardware_script(self):
        return os.path.join(
            self.remote_ws(),
            "src",
            "match_mobile_robotics_jazzy",
            "start_mur620_hardware_logged.sh",
        )

    def remote_host_setup_script(self):
        return os.path.join(self.remote_ws(), REMOTE_HOST_SETUP_REL)

    def remote_host_diag_script(self):
        return os.path.join(self.remote_ws(), REMOTE_HOST_DIAG_REL)

    def remote_ur_dashboard_safety_check_script(self):
        return os.path.join(self.remote_ws(), REMOTE_UR_DASHBOARD_SAFETY_CHECK_REL)

    def selected_sides(self):
        sides = []
        if self.arm_r.isChecked():
            sides.append("r")
        if self.arm_l.isChecked():
            sides.append("l")
        return sides

    def launch_mir_enabled(self):
        return getattr(self, "mir_enabled_check", None) is not None and self.mir_enabled_check.isChecked()

    def fallback_motion_controller(self):
        if self.opt_integrated.isChecked():
            return "integrated_cartesian_admittance_controller"
        return "forward_velocity_controller"

    def on_robot_selection_changed(self):
        selected = self.selected_robots()
        self.ros_worker.set_robot_names(selected)
        for module in self.modules:
            module.on_robot_selection_changed()
        for robot in ROBOTS:
            for side in SIDES:
                self.arm_status[(robot, side)] = "unknown"
                self.ur_reverse_ready[(robot, side)] = False
            if robot not in selected:
                for source in ("mir", "mur"):
                    self.update_battery_status(robot, source, 0.0, False)
        self.refresh_status_labels()
        self.update_connect_button()
        self.update_freedrive_button()

    def update_battery_status(self, robot, source, percentage, valid):
        key = (robot, source)
        self.battery_values[key] = percentage if valid else None
        badge = getattr(self, "battery_badges", {}).get(key)
        if badge is not None:
            badge.set_value(percentage if valid else None)

    def update_arm_status(self, robot, side, status):
        self.arm_status[(robot, side)] = status
        self.refresh_status_label(robot, side)

    def set_ur_reverse_ready(self, robot, side, ready, reason):
        key = (robot, side)
        if self.ur_reverse_ready.get(key) == ready:
            return
        self.ur_reverse_ready[key] = ready
        state = "ready" if ready else "not ready"
        self.append_log(f"[gui] {robot}/{SIDES[side]} UR reverse interface {state}: {reason}")
        self.refresh_status_label(robot, side)

    def refresh_status_label(self, robot, side):
        gate_status = self.arm_status.get((robot, side), "unknown")
        reverse_status = (
            "UR reverse OK" if self.ur_reverse_ready.get((robot, side), False)
            else "UR reverse missing"
        )
        label = self.status_labels.get((robot, side))
        if label is not None:
            label.setText(f"{gate_status} | {reverse_status}")

    def refresh_status_labels(self):
        for robot in ROBOTS:
            for side in SIDES:
                self.refresh_status_label(robot, side)

    def update_freedrive_status(self, robot, side, active, message):
        self.freedrive_active[(robot, side)] = active
        self.append_log(
            f"[gui] {robot}/{SIDES[side]} freedrive={'ON' if active else 'OFF'}: {message}"
        )
        self.update_freedrive_button()

    def update_connect_button(self):
        if not hasattr(self, "connect_button"):
            return
        selected = self.selected_robots()
        states = [self.connect_status.get(robot, "idle") for robot in selected]
        details = []
        for robot in selected:
            state = self.connect_status.get(robot, "idle")
            message = self.connect_messages.get(robot, "not connected")
            details.append(f"{robot}: {state} - {message}")
        self.connect_button.setToolTip("\n".join(details))

        if any(state == "running" for state in states):
            self.connect_button.setText("Connect...")
            self.connect_button.setStyleSheet(
                "QPushButton { background: #d69e2e; color: black; font-weight: bold; }"
            )
        elif selected and all(state == "ok" for state in states):
            self.connect_button.setText("Connected")
            self.connect_button.setStyleSheet(
                "QPushButton { background: #1f9d55; color: white; font-weight: bold; }"
            )
        elif any(state == "warn" for state in states):
            self.connect_button.setText("Check Host")
            self.connect_button.setStyleSheet(
                "QPushButton { background: #d69e2e; color: black; font-weight: bold; }"
            )
        elif any(state == "failed" for state in states):
            self.connect_button.setText("Connect Failed")
            self.connect_button.setStyleSheet(
                "QPushButton { background: #c53030; color: white; font-weight: bold; }"
            )
        else:
            self.connect_button.setText("Connect")
            self.connect_button.setStyleSheet("")

    def update_freedrive_button(self):
        if not hasattr(self, "freedrive_button"):
            return
        selected = self.robot_arm_pairs()
        selected_active = [self.freedrive_active.get(pair, False) for pair in selected]
        if selected and all(selected_active):
            self.freedrive_button.setText("Freedrive ON")
            self.freedrive_button.setStyleSheet(
                "QPushButton { background: #d69e2e; color: black; font-weight: bold; }"
            )
        elif any(self.freedrive_active.values()):
            active = ",".join(
                f"{robot}/{SIDES[side]}"
                for (robot, side), value in self.freedrive_active.items()
                if value
            )
            self.freedrive_button.setText(f"Freedrive {active}")
            self.freedrive_button.setStyleSheet(
                "QPushButton { background: #faf089; color: black; font-weight: bold; }"
            )
        else:
            self.freedrive_button.setText("Freedrive")
            self.freedrive_button.setStyleSheet("")

    def append_log(self, text):
        line = text.rstrip()
        level = self._log_level(line)
        self.log_entries.append((level, line))
        terminal = getattr(self, "terminal", None)
        if terminal is not None and self.log_filters.get(level, True):
            terminal.appendPlainText(line)
            terminal.verticalScrollBar().setValue(terminal.verticalScrollBar().maximum())
        self._write_gui_log(line)

    def _log_level(self, line):
        lower = line.lower()
        if ERROR_LOG_RE.search(lower):
            return "error"
        if WARNING_LOG_RE.search(lower):
            return "warning"
        return "info"

    def set_log_filter(self, level, enabled):
        if level not in self.log_filters:
            return
        self.log_filters[level] = enabled
        self.refresh_gui_log()

    def refresh_gui_log(self):
        terminal = getattr(self, "terminal", None)
        if terminal is None:
            return
        scroll_bar = terminal.verticalScrollBar()
        was_at_bottom = scroll_bar.value() == scroll_bar.maximum()
        terminal.setUpdatesEnabled(False)
        terminal.clear()
        terminal.setPlainText(
            "\n".join(
                line
                for level, line in self.log_entries
                if self.log_filters.get(level, True)
            )
        )
        terminal.setUpdatesEnabled(True)
        if was_at_bottom:
            scroll_bar.setValue(scroll_bar.maximum())

    def _write_gui_log(self, line):
        path = getattr(self, "gui_log_path", None)
        if not path:
            return
        try:
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            with open(path, "a", encoding="utf-8") as log_file:
                log_file.write(f"{timestamp} {line}\n")
        except OSError:
            pass

    def save_gui_log_snapshot(self):
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        snapshot_path = os.path.join(GUI_LOG_DIR, f"general_mur_gui_snapshot_{timestamp}.log")
        try:
            os.makedirs(GUI_LOG_DIR, exist_ok=True)
            with open(snapshot_path, "w", encoding="utf-8") as log_file:
                log_file.write(self.terminal.toPlainText())
                log_file.write("\n")
        except OSError as exc:
            self.append_log(f"[gui] Failed to save GUI log snapshot: {exc}")
            return
        self.append_log(f"[gui] Saved GUI log snapshot to {snapshot_path}")

    def start_process(self, name, command, env=None, on_finished=None):
        if name in self.processes and self.processes[name].state() != QtCore.QProcess.NotRunning:
            self.append_log(f"[gui] {name} already running")
            return
        process = QtCore.QProcess(self)
        process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        if env:
            qenv = QtCore.QProcessEnvironment.systemEnvironment()
            for key, value in env.items():
                qenv.insert(key, str(value))
            process.setProcessEnvironment(qenv)
        process.readyReadStandardOutput.connect(
            lambda proc=process, tag=name: self._read_process_output(tag, proc)
        )
        process.finished.connect(
            lambda code, status, tag=name: self.append_log(
                f"[{tag}] finished exit_code={code}, status={int(status)}"
            )
        )
        if on_finished is not None:
            process.finished.connect(on_finished)
        self.processes[name] = process
        self.append_log(f"[{name}] $ {command}")
        process.start("bash", ["-lc", command])

    def start_captured_process(self, name, command, on_finished, env=None):
        if name in self.processes and self.processes[name].state() != QtCore.QProcess.NotRunning:
            self.append_log(f"[gui] {name} already running")
            return
        process = QtCore.QProcess(self)
        process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        if env:
            qenv = QtCore.QProcessEnvironment.systemEnvironment()
            for key, value in env.items():
                qenv.insert(key, str(value))
            process.setProcessEnvironment(qenv)
        output = []

        def read_output():
            data = bytes(process.readAllStandardOutput()).decode(errors="replace")
            output.append(data)
            for line in data.splitlines():
                self.append_log(f"[{name}] {line}")

        process.readyReadStandardOutput.connect(read_output)
        process.finished.connect(
            lambda code, status: self.append_log(
                f"[{name}] finished exit_code={code}, status={int(status)}"
            )
        )
        process.finished.connect(
            lambda code, status: on_finished(code, status, "".join(output))
        )
        self.processes[name] = process
        self.append_log(f"[{name}] $ {command}")
        process.start("bash", ["-lc", command])

    def _read_process_output(self, tag, process):
        data = bytes(process.readAllStandardOutput()).decode(errors="replace")
        for line in data.splitlines():
            self.append_log(f"[{tag}] {line}")
            if tag == "hardware" or tag.endswith(":hardware"):
                robot = tag.split(":", 1)[0] if ":" in tag else self.object_host()
                self._observe_hardware_line(robot, line)

    def _observe_hardware_line(self, robot, line):
        if UR_REVERSE_READY_TEXT in line:
            for side, prefix in SIDES.items():
                if prefix in line:
                    self.set_ur_reverse_ready(robot, side, True, "reverse interface connected")
            return

        if "UR SetMode goal was rejected" in line:
            for side, prefix in SIDES.items():
                if prefix in line:
                    self.set_ur_reverse_ready(robot, side, False, "SetMode goal rejected")

    def _hardware_log_size(self):
        try:
            return os.path.getsize(HARDWARE_LATEST_LOG)
        except OSError:
            return None

    def _scan_latest_hardware_log_for_reverse_ready(self, sides, since_offset=None):
        try:
            with open(HARDWARE_LATEST_LOG, "rb") as log_file:
                log_file.seek(0, os.SEEK_END)
                size = log_file.tell()
                if since_offset is None:
                    start = max(0, size - 512_000)
                else:
                    start = since_offset if since_offset <= size else 0
                log_file.seek(start, os.SEEK_SET)
                text = log_file.read().decode(errors="replace")
        except OSError:
            return

        for line in text.splitlines():
            if UR_REVERSE_READY_TEXT not in line:
                continue
            for side in sides:
                if SIDES[side] in line:
                    self.set_ur_reverse_ready(
                        self.object_host(),
                        side,
                        True,
                        f"reverse interface seen in {HARDWARE_LATEST_LOG}",
                    )

    def connect_selected_robots(self):
        robots = self.selected_robots()
        for robot in robots:
            self.connect_status[robot] = "running"
            self.connect_messages[robot] = (
                "ssh check and rsync running"
                if self.opt_sync_code.isChecked()
                else "ssh check running"
            )
            self.update_connect_button()
            mkdir_cmd = (
                f"mkdir -p {shlex.quote(os.path.join(self.remote_ws(), 'src'))} && "
                "echo connected"
            )
            if self.opt_sync_code.isChecked():
                excludes = [
                    "--exclude=build/",
                    "--exclude=install/",
                    "--exclude=log/",
                    "--exclude=logs/",
                    "--exclude=__pycache__/",
                    "--exclude=.pytest_cache/",
                    "--exclude=.colcon/",
                    "--exclude=.git/",
                ]
                rsync_cmd = " ".join(
                    [
                        "rsync",
                        "-az",
                        "--delete",
                        *excludes,
                        shlex.quote(os.path.join(WS, "src") + "/"),
                        shlex.quote(f"{robot}:{os.path.join(self.remote_ws(), 'src') + '/'}"),
                    ]
                )
                command = self.remote_command(robot, mkdir_cmd) + " && " + rsync_cmd
            else:
                command = self.remote_command(robot, mkdir_cmd)
            check_cmd = (
                f"test -x {shlex.quote(self.remote_host_setup_script())} && "
                f"{shlex.quote(self.remote_host_setup_script())} --check"
            )
            command = command + " && " + self.remote_command(robot, check_cmd)

            def done(exit_code, _status, current_robot=robot):
                if exit_code == 0:
                    self.connected_robots.add(current_robot)
                    self.connect_status[current_robot] = "ok"
                    if self.opt_sync_code.isChecked():
                        self.connect_messages[current_robot] = (
                            f"ssh ok, rsync ok to {self.remote_ws()}/src"
                        )
                    else:
                        self.connect_messages[current_robot] = "ssh ok, sync skipped"
                else:
                    self.connected_robots.discard(current_robot)
                    self.connect_status[current_robot] = "failed"
                    self.connect_messages[current_robot] = (
                        f"ssh/rsync/host-check failed with exit code {exit_code}; "
                        "run Check Host or see terminal log"
                    )
                self.update_connect_button()

            self.start_process(
                self.process_key(robot, "connect"),
                command,
                on_finished=done,
            )

    def check_selected_hosts(self):
        for robot in self.selected_robots():
            self.connect_status[robot] = "running"
            self.connect_messages[robot] = "host preflight check running"
            self.update_connect_button()
            command = self.remote_command(
                robot,
                f"test -x {shlex.quote(self.remote_host_diag_script())} && "
                f"{shlex.quote(self.remote_host_diag_script())}",
            )

            def done(exit_code, _status, current_robot=robot):
                if exit_code == 0:
                    self.connected_robots.add(current_robot)
                    self.connect_status[current_robot] = "ok"
                    self.connect_messages[current_robot] = "host check ok; details in terminal"
                else:
                    self.connected_robots.discard(current_robot)
                    self.connect_status[current_robot] = "failed"
                    self.connect_messages[current_robot] = (
                        f"host check failed with exit code {exit_code}; details in terminal"
                    )
                self.update_connect_button()

            self.start_process(
                self.process_key(robot, "host_check"),
                command,
                on_finished=done,
            )

    def start_hardware(self):
        for module in self.modules:
            module.on_hardware_start()
        for robot in self.selected_robots():
            for side in self.selected_sides():
                self.ur_reverse_ready[(robot, side)] = False
                self.refresh_status_label(robot, side)
            self._start_hardware_after_preflight(robot)

    def _start_hardware_after_preflight(self, robot):
        ur_hosts = []
        if self.arm_l.isChecked():
            ur_hosts.append("UR10_l")
        if self.arm_r.isChecked():
            ur_hosts.append("UR10_r")
        expected_reverse_ip = "192.168.12.69" if robot == "mur620d" else ""
        ur_network_prefix = ""
        if ur_hosts:
            ur_network_prefix = (
                "export MUR_CHECK_UR_NETWORK=true; "
                f"export MUR_UR_HOSTS={shlex.quote(' '.join(ur_hosts))}; "
            )
            if expected_reverse_ip:
                ur_network_prefix += (
                    f"export MUR_EXPECTED_REVERSE_IP={shlex.quote(expected_reverse_ip)}; "
                )
        preflight_cmd = self.remote_command(
            robot,
            ur_network_prefix
            + f"test -x {shlex.quote(self.remote_host_setup_script())} && "
            f"{shlex.quote(self.remote_host_setup_script())} --check",
        )

        def done(exit_code, _status, current_robot=robot):
            if exit_code != 0:
                self.connect_status[current_robot] = "failed"
                self.connect_messages[current_robot] = (
                    f"hardware preflight failed with exit code {exit_code}; "
                    "run remote setup_mur_hardware_host.sh --apply"
                )
                self.update_connect_button()
                self.append_log(
                    f"[gui] Refusing Start Hardware for {current_robot}: "
                    "host preflight failed. Run Connect/Check Host and fix blocking issues."
                )
                return
            self.connect_status[current_robot] = "ok"
            self.connect_messages[current_robot] = "host preflight ok for hardware start"
            self.update_connect_button()
            self._check_ur_safety_before_launch(current_robot)

        self.start_process(
            self.process_key(robot, "hardware_preflight"),
            preflight_cmd,
            on_finished=done,
        )

    def _selected_ur_hosts(self):
        hosts = []
        if self.arm_l.isChecked():
            hosts.append("UR10_l")
        if self.arm_r.isChecked():
            hosts.append("UR10_r")
        return hosts

    def _ur_safety_check_command(self, robot, clear=False):
        hosts = self._selected_ur_hosts()
        script = self.remote_ur_dashboard_safety_check_script()
        args = [
            "python3",
            shlex.quote(script),
            "--json",
        ]
        if clear:
            args.append("--clear")
        for host in hosts:
            args.extend(["--host", shlex.quote(host)])
        return self.remote_command(robot, " ".join(args))

    def _parse_ur_safety_payload(self, output):
        start = output.find("{")
        end = output.rfind("}")
        if start < 0 or end < start:
            raise ValueError("no JSON payload found")
        return json.loads(output[start:end + 1])

    def _format_ur_safety_payload(self, payload):
        lines = []
        note = payload.get("note")
        if note:
            lines.append(note)
            lines.append("")
        for arm in payload.get("arms", []):
            host = arm.get("host", "unknown")
            lines.append(
                f"{host}: reachable={arm.get('reachable')} blocked={arm.get('blocked')}"
            )
            if arm.get("error"):
                lines.append(f"  error: {arm['error']}")
            if arm.get("banner"):
                lines.append(f"  banner: {arm['banner']}")
            for query in arm.get("queries", []):
                lines.append(f"  {query.get('command')}: {query.get('answer')}")
            for command in arm.get("clear", []):
                lines.append(f"  clear {command.get('command')}: {command.get('answer')}")
            for query in arm.get("queries_after_clear", []):
                lines.append(
                    f"  after clear {query.get('command')}: {query.get('answer')}"
                )
            lines.append("")
        return "\n".join(lines).strip()

    def _check_ur_safety_before_launch(self, robot):
        if not self._selected_ur_hosts():
            self._launch_hardware_for_robot(robot)
            return
        command = self._ur_safety_check_command(robot, clear=False)

        def done(exit_code, _status, output, current_robot=robot):
            try:
                payload = self._parse_ur_safety_payload(output)
            except (ValueError, json.JSONDecodeError) as exc:
                self.append_log(
                    f"[gui] UR safety preflight output could not be parsed: {exc}"
                )
                self._confirm_launch_after_safety_check_error(current_robot, output)
                return

            if exit_code == 0 and payload.get("ok", False):
                self.append_log(f"[gui] {current_robot}: UR dashboard safety preflight ok")
                self._launch_hardware_for_robot(current_robot)
                return

            if exit_code == 2:
                self._show_ur_safety_blocking_dialog(current_robot, payload)
                return

            self._confirm_launch_after_safety_check_error(
                current_robot,
                self._format_ur_safety_payload(payload),
            )

        self.start_captured_process(
            self.process_key(robot, "ur_safety_preflight"),
            command,
            on_finished=done,
        )

    def _show_ur_safety_blocking_dialog(self, robot, payload):
        details = self._format_ur_safety_payload(payload)
        self.append_log(f"[gui] {robot}: UR safety blocker detected before hardware launch")
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("UR Safety Stop erkannt")
        dialog.setModal(True)
        dialog.setMinimumSize(760, 520)
        dialog.resize(840, 560)
        dialog.setStyleSheet(
            """
            QDialog {
                background: #fff8db;
            }
            QLabel#SafetyTitle {
                color: #1f2933;
                font-size: 22px;
                font-weight: 700;
            }
            QLabel#SafetySubtitle {
                color: #3f3f46;
                font-size: 13px;
            }
            QFrame#SafetyBanner {
                background: #facc15;
                border: 2px solid #ca8a04;
                border-radius: 6px;
            }
            QPlainTextEdit#SafetyDetails {
                background: #111827;
                color: #f9fafb;
                border: 1px solid #374151;
                border-radius: 4px;
                font-family: monospace;
                font-size: 12px;
            }
            QPushButton#SafetyClearButton {
                background: #ca8a04;
                color: white;
                font-weight: 700;
                padding: 9px 16px;
                border-radius: 4px;
            }
            QPushButton#SafetyAbortButton {
                background: #fef3c7;
                color: #1f2933;
                padding: 9px 16px;
                border: 1px solid #ca8a04;
                border-radius: 4px;
            }
            """
        )

        root = QtWidgets.QVBoxLayout(dialog)
        root.setContentsMargins(18, 18, 18, 18)
        root.setSpacing(14)

        banner = QtWidgets.QFrame(dialog)
        banner.setObjectName("SafetyBanner")
        banner_layout = QtWidgets.QHBoxLayout(banner)
        banner_layout.setContentsMargins(16, 14, 16, 14)
        banner_layout.setSpacing(14)

        icon = QtWidgets.QLabel(banner)
        icon.setPixmap(
            self.style().standardIcon(QtWidgets.QStyle.SP_MessageBoxWarning).pixmap(48, 48)
        )
        banner_layout.addWidget(icon, 0, QtCore.Qt.AlignTop)

        title_column = QtWidgets.QVBoxLayout()
        title = QtWidgets.QLabel(
            "UR Safety-/Protective-Stop erkannt",
            banner,
        )
        title.setObjectName("SafetyTitle")
        subtitle = QtWidgets.QLabel(
            "Der Hardware-Start ist pausiert. Pruefe den Roboterbereich, "
            "bevor du die Fehler quittierst.",
            banner,
        )
        subtitle.setObjectName("SafetySubtitle")
        subtitle.setWordWrap(True)
        title_column.addWidget(title)
        title_column.addWidget(subtitle)
        banner_layout.addLayout(title_column, 1)
        root.addWidget(banner)

        instruction = QtWidgets.QLabel(
            "Dashboard-Meldungen der ausgewaehlten URs:",
            dialog,
        )
        instruction.setStyleSheet("font-weight: 700; color: #1f2933;")
        root.addWidget(instruction)

        detail_box = QtWidgets.QPlainTextEdit(dialog)
        detail_box.setObjectName("SafetyDetails")
        detail_box.setReadOnly(True)
        detail_box.setPlainText(details)
        detail_box.setMinimumHeight(260)
        root.addWidget(detail_box, 1)

        button_row = QtWidgets.QHBoxLayout()
        button_row.addStretch(1)
        abort_button = QtWidgets.QPushButton("Start abbrechen", dialog)
        abort_button.setObjectName("SafetyAbortButton")
        clear_button = QtWidgets.QPushButton("Fehler quittieren und starten", dialog)
        clear_button.setObjectName("SafetyClearButton")
        button_row.addWidget(abort_button)
        button_row.addWidget(clear_button)
        root.addLayout(button_row)

        dialog._clear_requested = False
        clear_button.clicked.connect(lambda: setattr(dialog, "_clear_requested", True))
        clear_button.clicked.connect(dialog.accept)
        abort_button.clicked.connect(dialog.reject)
        abort_button.setDefault(True)

        dialog.exec_()
        if getattr(dialog, "_clear_requested", False):
            self._clear_ur_safety_and_launch(robot)
        else:
            self.append_log(f"[gui] {robot}: hardware start aborted by user after UR safety popup")

    def _confirm_launch_after_safety_check_error(self, robot, details):
        box = QtWidgets.QMessageBox(self)
        box.setIcon(QtWidgets.QMessageBox.Warning)
        box.setWindowTitle("UR Safety Check fehlgeschlagen")
        box.setText("Der UR-Safety-Preflight konnte nicht sauber abgeschlossen werden.")
        box.setInformativeText(
            "Du kannst den Hardware-Start trotzdem fortsetzen oder abbrechen."
        )
        box.setDetailedText(details.strip() or "No safety preflight details available.")
        start_button = box.addButton("Trotzdem starten", QtWidgets.QMessageBox.AcceptRole)
        abort_button = box.addButton("Start abbrechen", QtWidgets.QMessageBox.RejectRole)
        box.setDefaultButton(abort_button)
        box.exec_()
        if box.clickedButton() == start_button:
            self.append_log(
                f"[gui] {robot}: user continued hardware start after safety check failure"
            )
            self._launch_hardware_for_robot(robot)
        else:
            self.append_log(f"[gui] {robot}: hardware start aborted after safety check failure")

    def _clear_ur_safety_and_launch(self, robot):
        command = self._ur_safety_check_command(robot, clear=True)

        def done(exit_code, _status, output, current_robot=robot):
            try:
                payload = self._parse_ur_safety_payload(output)
                details = self._format_ur_safety_payload(payload)
            except (ValueError, json.JSONDecodeError) as exc:
                details = output
                self.append_log(
                    f"[gui] UR safety clear output could not be parsed: {exc}"
                )
            if exit_code == 0:
                self.append_log(
                    f"[gui] {current_robot}: UR safety popups cleared; launching hardware"
                )
                self._launch_hardware_for_robot(current_robot)
                return

            box = QtWidgets.QMessageBox(self)
            box.setIcon(QtWidgets.QMessageBox.Critical)
            box.setWindowTitle("UR Safety Stop nicht quittiert")
            box.setText("Die UR-Safety-Fehler konnten nicht vollstaendig quittiert werden.")
            box.setInformativeText("Der Hardware-Start wurde abgebrochen.")
            box.setDetailedText(details.strip() or "No safety clear details available.")
            box.exec_()
            self.append_log(
                f"[gui] {current_robot}: hardware start aborted; UR safety clear failed"
            )

        self.start_captured_process(
            self.process_key(robot, "ur_safety_clear"),
            command,
            on_finished=done,
        )

    def _launch_hardware_for_robot(self, robot):
        expected_reverse_ip = "192.168.12.69" if robot == "mur620d" else ""
        args = [
            f"robot_name:={robot}",
            f"robot_profile:={robot}",
            f"launch_ur_r:={'true' if self.arm_r.isChecked() else 'false'}",
            f"launch_ur_l:={'true' if self.arm_l.isChecked() else 'false'}",
            f"launch_mir:={'true' if self.launch_mir_enabled() else 'false'}",
            f"integrated_controller_enable_collision_avoidance:={'true' if self.opt_collision.isChecked() else 'false'}",
            f"integrated_controller_publish_collision_markers:={'true' if self.opt_markers.isChecked() else 'false'}",
            f"launch_moveit:={'true' if self.opt_moveit.isChecked() else 'false'}",
            "launch_bms:=true",
            "bms_can_interface:=can0",
            "bms_can_bitrate:=250000",
            "auto_switch_moveit_controllers:=true",
            "launch_moveit_rviz:=false",
        ]
        if self.opt_zero_admittance.isChecked():
            args.extend([
                "integrated_controller_admittance:=0.0 0.0 0.0 0.0 0.0 0.0",
                "integrated_controller_wrench_twist_gain:=0.0 0.0 0.0 0.0 0.0 0.0",
            ])
        if self.opt_integrated.isChecked():
            args.extend([
                "launch_arm_velocity_safety:=false",
                "launch_jparse_idk:=false",
            ])
        env_prefix = " ".join(
            [
                f"export ROS_DOMAIN_ID={shlex.quote(os.environ.get('ROS_DOMAIN_ID', '62'))};",
                f"export ROBOT_PROFILE={shlex.quote(robot)};",
                f"export BUILD_BEFORE_LAUNCH={'true' if self.opt_build.isChecked() else 'false'};",
                "export BUILD_PACKAGES='serial ewellix_driver mur_control mur_moveit_config mur_launch_hardware match_mur_gui mir_launch_hardware mir_driver mir_msgs mir_srvs mir_restapi sdc21x0';",
                f"export MUR_CHECK_UR_NETWORK={'true' if self.selected_sides() else 'false'};",
                f"export MUR_UR_HOSTS={shlex.quote(' '.join(['UR10_l' if side == 'l' else 'UR10_r' for side in self.selected_sides()]))};",
                f"export MUR_EXPECTED_REVERSE_IP={shlex.quote(expected_reverse_ip)};",
                f"export INTEGRATED_CARTESIAN_ACTIVE={'true' if self.opt_integrated.isChecked() else 'false'};",
                f"export INTEGRATED_CARTESIAN_USE_FT={'true' if self.opt_ft.isChecked() else 'false'};",
                f"export INTEGRATED_CARTESIAN_REQUIRE_WRENCH={'true' if self.opt_require_wrench.isChecked() else 'false'};",
                f"export MOVEIT_WITH_INTEGRATED_CARTESIAN={'true' if self.opt_moveit.isChecked() and self.opt_integrated.isChecked() else 'false'};",
                "export MUR_REQUIRE_HOST_PREFLIGHT=true;",
            ]
        )
        launch_cmd = " ".join(
            [shlex.quote(self.remote_hardware_script())]
            + [shlex.quote(arg) for arg in args]
        )
        command = self.remote_command(robot, env_prefix + " " + launch_cmd)
        self.start_process(self.process_key(robot, "hardware"), command)

    def ensure_ur_ready(self, sides=None, robots=None, on_success=None, retry_count=0):
        if isinstance(sides, bool):
            sides = None
        if isinstance(robots, bool):
            robots = None
        selected_robots = list(robots or self.selected_robots())
        selected = list(sides) if sides is not None else self.selected_sides()
        if not selected:
            self.append_log("[gui] Refusing UR ready check: no arm selected")
            return False
        if not selected_robots:
            self.append_log("[gui] Refusing UR ready check: no robot selected")
            return False
        if retry_count == 0:
            for robot in selected_robots:
                self.ur_ready_log_scan_start[robot] = None
                for side in selected:
                    self.set_ur_reverse_ready(
                        robot, side, False, "manual enable/check requested"
                    )

        def retry(reason):
            if retry_count >= UR_READY_RETRY_LIMIT:
                self.append_log(
                    "[gui] UR ready check failed after retry. Not arming motion. "
                    f"Reason: {reason}"
                )
                return
            self.append_log(
                "[gui] UR ready check did not reach reverse-interface-ready; "
                f"retrying once. Reason: {reason}"
            )
            QtCore.QTimer.singleShot(
                1500,
                lambda: self.ensure_ur_ready(
                    sides=selected,
                    robots=selected_robots,
                    on_success=on_success,
                    retry_count=retry_count + 1,
                ),
            )

        self.append_log(
            "[gui] Ensuring selected URs are running their External Control program: "
            + ", ".join(f"{robot}/{SIDES[side]}" for robot in selected_robots for side in selected)
        )
        remaining = set(selected_robots)

        def robot_ready(robot):
            remaining.discard(robot)
            if not remaining and on_success is not None:
                QtCore.QTimer.singleShot(200, on_success)

        for robot in selected_robots:
            commands = []
            for side in selected:
                prefix = SIDES[side]
                commands.append(
                    "ros2 run match_mur_gui ensure_ur_ready.py --ros-args "
                    + f"-p arm_namespace:=/{robot}/{prefix} "
                    + "-p wait_timeout:=30.0 "
                    + "-p target_robot_mode:=7 "
                    + "-p allow_stop_restart:=true"
                )
            command = self.remote_ros_command(robot, " && ".join(commands))

            def done(exit_code, _status, current_robot=robot):
                if exit_code == 0:
                    self._wait_for_ur_reverse_ready(
                        current_robot,
                        selected,
                        on_success=lambda current_robot=current_robot: robot_ready(current_robot),
                        on_timeout=lambda missing, current_robot=current_robot: retry(
                            f"{current_robot}: missing reverse interface for "
                            + ", ".join(SIDES[side] for side in missing)
                        ),
                    )
                else:
                    retry(f"{current_robot}: ensure_ur_ready script exited with failure")

            self.start_process(self.process_key(robot, "ensure_ur_ready"), command, on_finished=done)
        return True

    def _wait_for_ur_reverse_ready(self, robot, sides, on_success=None, on_timeout=None):
        deadline = time.monotonic() + UR_REVERSE_WAIT_SEC

        def poll():
            missing = [
                side for side in sides
                if not self.ur_reverse_ready.get((robot, side), False)
            ]
            if not missing:
                self.append_log(
                    f"[gui] {robot} UR reverse interface ready for: "
                    + ", ".join(SIDES[side] for side in sides)
                )
                if on_success is not None:
                    QtCore.QTimer.singleShot(200, on_success)
                return
            if time.monotonic() >= deadline:
                self.append_log(
                    "[gui] UR ready check timed out waiting for reverse interface: "
                    + ", ".join(SIDES[side] for side in missing)
                )
                if on_timeout is not None:
                    on_timeout(missing)
                return
            QtCore.QTimer.singleShot(250, poll)

        poll()

    def open_manipulator_jog(self, side):
        dialog = ManipulatorJogDialog(self, side, self)
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()
        self._manipulator_jog_dialog = dialog

    def open_mir_jog(self):
        dialog = MiRJogDialog(self, self)
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()
        self._mir_jog_dialog = dialog

    def open_mir_lights(self):
        dialog = MiRLightDialog(self, self)
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()
        self._mir_light_dialog = dialog

    def open_mir_goals(self):
        dialog = MiRGoalDialog(self, self)
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()
        self._mir_goal_dialog = dialog

    def stop_module_motion_like_actions(self):
        for module in self.modules:
            module.stop_motion_like_actions()

    def toggle_freedrive(self):
        pairs = self.robot_arm_pairs()
        if not pairs:
            self.append_log("[gui] Refusing freedrive: no arm selected")
            return
        enable = not all(self.freedrive_active.get(pair, False) for pair in pairs)
        fallback = self.fallback_motion_controller()
        if enable:
            self.append_log(
                "[gui] Enabling freedrive: stopping module motion first"
            )
            self.stop_module_motion_like_actions()
            for robot, side in pairs:
                self.ros_worker.switch_freedrive(robot, side, True, fallback)
            return

        self.append_log("[gui] Disabling freedrive and restoring previous motion controllers")
        for robot, side in pairs:
            self.ros_worker.switch_freedrive(robot, side, False, fallback)

    def open_rviz(self):
        robot = self.object_host()
        config_path = (
            "$(ros2 pkg prefix mur_launch_hardware)"
            "/share/mur_launch_hardware/config/rviz/mur620d_moveit.rviz"
        )
        cmd = (
            setup_prefix()
            + "exec rviz2 "
            + f"-d {config_path} "
            + "--ros-args "
            + f"-r /tf:=/tf "
            + f"-r /tf_static:=/tf_static "
            + f"-p use_sim_time:=false"
        )
        self.append_log(
            f"[gui] Opening RViz for {robot}. MoveIt must be running for MotionPlanning."
        )
        self.start_process("rviz_moveit", cmd)

    def move_home(self, side):
        prefix = SIDES[side]
        for robot in self.selected_robots():
            if self.freedrive_active.get((robot, side), False):
                self.append_log(f"[gui] Home {robot}/{prefix}: disabling freedrive first")
                self.ros_worker.switch_freedrive(
                    robot, side, False, self.fallback_motion_controller()
                )
        if not self.opt_moveit.isChecked():
            self.append_log(
                f"[gui] Home {prefix}: Launch MoveIt is disabled. "
                "Enable it before starting hardware, or make sure MoveIt is already running."
            )
        self.append_log(
            f"[gui] Home {prefix}: stopping module motion first."
        )
        self.stop_module_motion_like_actions()
        QtCore.QTimer.singleShot(500, partial(self._start_home_process, side))

    def _start_home_process(self, side):
        prefix = SIDES[side]
        for robot in self.selected_robots():
            cmd = (
                self.remote_setup_prefix()
                + "exec ros2 run match_mur_gui move_arm_to_named_pose.py --ros-args "
                + f"-p robot_name:={robot} "
                + f"-p robot_profile:={self.robot_profile(robot)} "
                + f"-p arm:={side} "
                + f"-p group:=UR_arm_{side} "
                + "-p named_pose:=Home_custom "
                + f"-p velocity_scaling:={self.moveit_velocity_scaling():.3f}"
            )
            self.start_process(
                self.process_key(robot, f"home_{side}"),
                self.remote_command(robot, cmd),
            )

    def stop_managed_processes(self):
        self.stop_module_motion_like_actions()
        cleanup_patterns = [
            "move_arm_to_named_pose.py",
        ]
        cleanup_cmd = " ; ".join(
            f"pkill -TERM -f {shlex.quote(pattern)} 2>/dev/null || true"
            for pattern in cleanup_patterns
        )
        cleanup_cmd += " ; sleep 0.5 ; "
        cleanup_cmd += " ; ".join(
            f"pkill -KILL -f {shlex.quote(pattern)} 2>/dev/null || true"
            for pattern in cleanup_patterns
        )
        for robot in self.selected_robots():
            self.start_process(
                self.process_key(robot, "remote_cleanup"),
                self.remote_command(robot, cleanup_cmd),
            )
        for name, process in list(self.processes.items()):
            if name == "object_cleanup" or name.endswith(":remote_cleanup"):
                continue
            if process.state() == QtCore.QProcess.NotRunning:
                continue
            self.append_log(f"[gui] terminating {name}")
            process.terminate()
            if not process.waitForFinished(1500):
                process.kill()

    def closeEvent(self, event):
        self.stop_managed_processes()
        for module in self.modules:
            module.on_shutdown()
        self.ros_worker.shutdown()
        self.ros_worker.wait(1500)
        super().closeEvent(event)


class GeneralMuRGui(MurBaseGui):
    def __init__(self):
        super().__init__(modules=[], window_title="General MuR GUI")
