"""Shutdown ordering, failures and GUI selection without contacting hardware."""

import importlib.util
import io
import json
import os
from pathlib import Path
import shlex
from types import SimpleNamespace

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
spec = importlib.util.spec_from_file_location(
    "shutdown_urs", Path(__file__).parents[1] / "scripts/shutdown_urs.py")
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)


class Dashboard:
    def __init__(self, replies):
        self.replies = replies
        self.commands = []

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        pass

    def query(self, command):
        self.commands.append(command)
        reply = self.replies[command]
        if isinstance(reply, list):
            reply = reply.pop(0)
        if isinstance(reply, Exception):
            raise reply
        return reply


def replies(**overrides):
    result = {
        "PolyscopeVersion": "URSoftware 5.12.0.1101319 (Mar 22 2022)",
        "is in remote control": "true", "robotmode": ["Robotmode: RUNNING", "Robotmode: POWER_OFF"],
        "stop": "Stopped", "programState": "STOPPED external_control.urp",
        "power off": "Powering off", "shutdown": "Shutting down",
    }
    result.update(overrides)
    return result


def run(dashboard, **kwargs):
    return helper.shutdown_arm("UR10_r", client_factory=lambda *_args: dashboard, **kwargs)


def test_order_and_state_verification(monkeypatch):
    monkeypatch.setattr(helper.time, "sleep", lambda _seconds: None)
    dashboard = Dashboard(replies(**{
        "programState": ["PLAYING external_control.urp", "STOPPED external_control.urp"],
        "robotmode": ["Robotmode: RUNNING", "Robotmode: IDLE", "Robotmode: POWER_OFF"],
    }))
    assert run(dashboard)["shutdown_accepted"]
    assert dashboard.commands == [
        "PolyscopeVersion", "is in remote control", "robotmode", "stop",
        "programState", "programState", "power off", "robotmode", "robotmode", "shutdown",
    ]


@pytest.mark.parametrize("command,answer", [
    ("stop", "Failed to execute: stop"),
    ("power off", "Command is not allowed due to safety reasons"),
    ("is in remote control", "false"),
    ("is in remote control", "Unknown command"),
    ("PolyscopeVersion", "URSoftware 10.12.0"),
    ("PolyscopeVersion", "garbage"),
])
def test_failed_prerequisite_prevents_shutdown(command, answer):
    dashboard = Dashboard(replies(**{command: answer}))
    result = run(dashboard)
    assert not result["shutdown_accepted"]
    assert result["error"]
    assert "shutdown" not in dashboard.commands
    if command in ("stop", "is in remote control", "PolyscopeVersion"):
        assert "power off" not in dashboard.commands


@pytest.mark.parametrize("state_command,answer", [
    ("programState", "PAUSED external_control.urp"),
    ("robotmode", "Robotmode: RUNNING"),
])
def test_state_timeout_never_advances(monkeypatch, state_command, answer):
    times = iter([0.0, 2.0, 3.0, 5.0])
    monkeypatch.setattr(helper.time, "monotonic", lambda: next(times))
    dashboard = Dashboard(replies(**{state_command: answer}))
    result = run(dashboard, state_timeout=1.0)
    assert not result["shutdown_accepted"]
    assert "Timeout" in result["error"]
    assert "shutdown" not in dashboard.commands
    if state_command == "programState":
        assert "power off" not in dashboard.commands


def test_already_powered_off_is_idempotent():
    dashboard = Dashboard(replies(**{"robotmode": "Robotmode: POWER_OFF"}))
    assert run(dashboard)["shutdown_accepted"]
    assert "stop" not in dashboard.commands
    assert "power off" not in dashboard.commands
    assert dashboard.commands[-2:] == ["programState", "shutdown"]


@pytest.mark.parametrize("version", ["URSoftware 3.15.8.106339", "3.0.15547"])
def test_cb3_does_not_require_remote_mode_query(version):
    dashboard = Dashboard(replies(**{"PolyscopeVersion": version}))
    assert run(dashboard)["shutdown_accepted"]
    assert "is in remote control" not in dashboard.commands


@pytest.mark.parametrize("answer", ["", "Failed to execute: shutdown", OSError("connection lost")])
def test_shutdown_requires_acknowledgement_without_retry(answer):
    dashboard = Dashboard(replies(shutdown=answer))
    result = run(dashboard)
    assert not result["shutdown_accepted"]
    assert dashboard.commands.count("shutdown") == 1


def test_multiple_hosts_are_deduplicated_and_partial_failure_is_reported(monkeypatch, capsys):
    calls = []

    def shutdown(host, *_args, **_kwargs):
        calls.append(host)
        return {"host": host, "shutdown_accepted": host == "UR10_r", "error": "offline"}

    monkeypatch.setattr(helper, "shutdown_arm", shutdown)
    assert helper.main(["--host", "UR10_r", "--host", "UR10_l", "--host", "UR10_r", "--json"]) == 1
    payload = json.loads(capsys.readouterr().out)
    assert sorted(calls) == ["UR10_l", "UR10_r"]
    assert not payload["ok"]
    assert payload["arms"][0]["shutdown_accepted"]


def test_dashboard_closes_socket_when_greeting_is_incomplete(monkeypatch):
    stream = io.BytesIO(b"Connected: Universal Robots Dashboard Server")
    closed = []
    sock = SimpleNamespace(makefile=lambda *_args, **_kwargs: stream,
                           close=lambda: closed.append(True))
    monkeypatch.setattr(helper.socket, "create_connection", lambda *_args: sock)
    with pytest.raises(RuntimeError, match="incomplete"):
        with helper.DashboardClient("unused", 29999, 1.0):
            pass
    assert stream.closed
    assert closed == [True]


@pytest.fixture
def window(monkeypatch, tmp_path):
    from PyQt5 import QtWidgets
    from match_mur_gui import base_gui
    from test_gui_layout import _RosWorker

    monkeypatch.setattr(base_gui, "RosWorker", _RosWorker)
    monkeypatch.setattr(base_gui.MurBaseGui, "_create_gui_log_file", lambda _self: str(tmp_path / "gui.log"))
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    gui = base_gui.MurBaseGui()
    gui.stop_managed_processes = lambda: None
    gui.ros_worker.set_robot_names = lambda *_args: None
    gui.ros_worker.inhibit_arm_motion = lambda *_args: None
    gui._prepare_arm_motion_profile = lambda *_args: None
    gui.disable_cartesian_motion = lambda *_args: None
    yield gui
    gui._ur_shutdown_pending.clear()
    gui.close()
    app.processEvents()


def test_shutdown_targets_snapshot_of_checked_robots_and_sides(window, monkeypatch):
    from PyQt5 import QtWidgets
    calls = []
    for robot, check in window.robot_checks.items():
        check.setChecked(robot in ("mur620a", "mur620c"))
    window.arm_l.setChecked(False)
    monkeypatch.setattr(QtWidgets.QMessageBox, "question", lambda *_args: QtWidgets.QMessageBox.Yes)
    window._start_ur_shutdown = lambda robot, sides: calls.append((robot, sides))
    window.shutdown_selected_urs()
    assert calls == [("mur620a", ("r",)), ("mur620c", ("r",))]
    assert window._ur_shutdown_pairs == {("mur620a", "r"), ("mur620c", "r")}
    assert not window.shutdown_urs_button.isEnabled()
    window.robot_checks["mur620b"].setChecked(True)
    window.arm_l.setChecked(True)
    assert len(calls) == 2
    window.set_ur_reverse_ready("mur620a", "r", True, "stale ready log")
    assert not window.ur_reverse_ready[("mur620a", "r")]


def test_empty_selection_never_uses_default_mur620d(window, monkeypatch):
    from PyQt5 import QtWidgets
    for check in window.robot_checks.values():
        check.setChecked(False)
    monkeypatch.setattr(QtWidgets.QMessageBox, "question", lambda *_args: pytest.fail("Unexpected dialog"))
    window.shutdown_selected_urs()
    assert not window._ur_shutdown_pairs


def test_cancel_does_not_inhibit_or_contact_hardware(window, monkeypatch):
    from PyQt5 import QtWidgets
    monkeypatch.setattr(QtWidgets.QMessageBox, "question", lambda *_args: QtWidgets.QMessageBox.Cancel)
    window._start_ur_shutdown = lambda *_args: pytest.fail("Unexpected shutdown")
    window.shutdown_selected_urs()
    assert not window._ur_shutdown_pairs
    assert not window._ur_shutdown_pending


def test_shutdown_refuses_while_hardware_is_automatically_enabling_ur(window, monkeypatch):
    from PyQt5 import QtWidgets
    window._ur_starting_pairs.add(("mur620d", "r"))
    monkeypatch.setattr(QtWidgets.QMessageBox, "question", lambda *_args: pytest.fail("Unexpected dialog"))
    window.shutdown_selected_urs()
    assert not window._ur_shutdown_pending
    window.set_ur_reverse_ready("mur620d", "r", True, "startup complete")
    assert not window._ur_starting_pairs


def test_repeated_hardware_start_does_not_latch_a_new_startup(window):
    from PyQt5 import QtCore
    window.processes["mur620d:hardware"] = SimpleNamespace(state=lambda: QtCore.QProcess.Running)
    window._launch_hardware_for_robot("mur620d")
    assert not window._ur_starting_pairs


def test_shutdown_rechecks_enable_process_started_during_confirmation(window, monkeypatch):
    from PyQt5 import QtCore, QtWidgets

    def confirm(*_args):
        window.processes["mur620d:ensure_ur_ready"] = SimpleNamespace(state=lambda: QtCore.QProcess.Running)
        return QtWidgets.QMessageBox.Yes

    monkeypatch.setattr(QtWidgets.QMessageBox, "question", confirm)
    window.shutdown_selected_urs()
    assert not window._ur_shutdown_pending


def test_shutdown_command_runs_on_correct_host_with_only_selected_arm(window):
    outer = shlex.split(window._ur_shutdown_command("mur620c", ["l"]))
    assert outer[-2] == "mur620c"
    inner = shlex.split(shlex.split(outer[-1])[-1])
    assert inner[-2:] == ["--host", "UR10_l"]
    assert "UR10_r" not in inner
    assert "100s" in inner


def test_shutdown_blocks_ready_freedrive_home_and_hardware(window):
    window._ur_shutdown_pending.add("mur620d")
    window._ur_shutdown_pairs.add(("mur620d", "r"))
    window.start_process = lambda *_args, **_kwargs: pytest.fail("Unexpected process")
    assert not window.ensure_ur_ready()
    window.start_hardware()
    window.toggle_freedrive()
    window.move_home("r")
    window.open_manipulator_jog("r")
    event = SimpleNamespace(ignore=lambda: None)
    window.closeEvent(event)


def test_delayed_enable_retry_is_invalidated_by_shutdown(window, monkeypatch):
    from PyQt5 import QtCore
    timers = []
    callbacks = []
    monkeypatch.setattr(QtCore.QTimer, "singleShot", lambda _delay, callback: timers.append(callback))
    window.start_process = lambda *_args, **kwargs: callbacks.append(kwargs["on_finished"])
    assert window.ensure_ur_ready(sides=["r"], robots=["mur620d"])
    callbacks[0](1, QtCore.QProcess.NormalExit)
    assert len(timers) == 1
    window._ur_shutdown_generation += 1
    window.ensure_ur_ready = lambda **_kwargs: pytest.fail("Stale retry enabled UR after shutdown")
    timers[0]()


def test_shutdown_result_preserves_each_arm_and_releases_button(window):
    from PyQt5 import QtCore
    window._ur_shutdown_pending.add("mur620d")
    callbacks = []
    process = SimpleNamespace(errorOccurred=SimpleNamespace(connect=lambda _callback: None))

    def capture(name, _command, callback):
        window.processes[name] = process
        callbacks.append(callback)

    window.start_captured_process = capture
    window._start_ur_shutdown("mur620d", ("r", "l"))
    callbacks[0](1, QtCore.QProcess.NormalExit, json.dumps({"arms": [
        {"host": "UR10_r", "shutdown_accepted": True},
        {"host": "UR10_l", "shutdown_accepted": False, "error": "offline"},
    ]}))
    assert window.arm_feedback[("mur620d", "r")][1] is False
    assert window.arm_feedback[("mur620d", "l")][1] is True
    assert "offline" in window.arm_feedback[("mur620d", "l")][0]
    assert not window._ur_shutdown_pending
    assert window.shutdown_urs_button.isEnabled()


def test_in_flight_freedrive_cannot_restart_keepalive_after_shutdown():
    from match_mur_gui.base_gui import RosWorker
    worker = RosWorker([])
    pair = ("mur620a", "l")
    worker._set_freedrive_keepalive(*pair, True)
    worker.inhibit_arm_motion([pair])
    worker._set_freedrive_keepalive(*pair, True)
    assert worker._freedrive_keepalive[pair] is False
    worker.inhibit_arm_motion([pair], False)
    worker._set_freedrive_keepalive(*pair, True)
    assert worker._freedrive_keepalive[pair] is True
