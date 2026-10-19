"""The base GUI pins its chosen source and starts only the required support."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from types import SimpleNamespace
from PyQt5 import QtCore, QtWidgets
import pytest
from match_mur_gui import base_gui
from test_gui_layout import _RosWorker


@pytest.mark.parametrize('source', ['mir', 'mocap', 'amcl'])
def test_source_is_forwarded_at_hardware_start(monkeypatch, tmp_path, source):
    monkeypatch.setattr(base_gui, 'RosWorker', _RosWorker)
    monkeypatch.setattr(base_gui, 'configure_ros_discovery', lambda *_args: 'isolated test')
    monkeypatch.setattr(base_gui.MurBaseGui, '_create_gui_log_file', lambda self: str(tmp_path/'gui.log'))
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = base_gui.MurBaseGui()
    window.stop_managed_processes = lambda: None
    assert window.localization_source.currentData() == 'mir'
    window.mir_enabled_check.setChecked(True)
    assert window.localization_source.isEnabled()
    window.localization_source.setCurrentIndex(window.localization_source.findData(source))
    if source == 'amcl':
        window.resize(1280, 1000)
        window.show()
        app.processEvents()
        window.grab().save(str(tmp_path/'base_localization.png'))
    window.arm_r.setChecked(False)
    window.arm_l.setChecked(False)
    commands, prepared = [], []
    monkeypatch.setattr(window, 'prepare_base_localization', lambda r, s: prepared.append((r, s)) or True)
    monkeypatch.setattr(window, 'remote_command', lambda robot, command: command)
    monkeypatch.setattr(window, 'start_process', lambda name, command, **kwargs: commands.append((name, command)))
    window._launch_hardware_for_robot('mur620c')
    assert prepared == [('mur620c', source)]
    assert 'mir_localization_source:='+source in commands[-1][1]
    assert 'launch_mir:=true' in commands[-1][1]
    assert 'activate_joint_controller:=false' in commands[-1][1]
    window.close()
    app.processEvents()


def test_support_reuses_owned_processes_and_amcl_opens_navigation_view(monkeypatch, tmp_path):
    monkeypatch.setattr(base_gui, 'RosWorker', _RosWorker)
    monkeypatch.setattr(base_gui, 'configure_ros_discovery', lambda *_args: 'isolated test')
    monkeypatch.setattr(base_gui.MurBaseGui, '_create_gui_log_file', lambda self: str(tmp_path/'gui.log'))
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = base_gui.MurBaseGui()
    window.stop_managed_processes = lambda: None
    commands = []
    def capture(name, command, **kwargs):
        commands.append((name, command))
        window.processes[name] = SimpleNamespace(state=lambda: QtCore.QProcess.Running)
    monkeypatch.setattr(window, 'start_process', capture)
    assert window.prepare_base_localization('mur620c', 'mir')
    assert not commands
    assert window.prepare_base_localization('mur620c', 'amcl')
    assert len(commands) == 2
    assert '--mode mocap' in commands[0][1]
    assert '--mode amcl --robot mur620c' in commands[1][1]
    assert window.prepare_base_localization('mur620c', 'amcl')
    assert len(commands) == 2
    window.mir_enabled_check.setChecked(True)
    window.localization_source.setCurrentIndex(window.localization_source.findData('amcl'))
    window.open_rviz()
    assert '--mode localization' in commands[-1][1]
    assert 'moveit.rviz' not in commands[-1][1]
    window.close()
    app.processEvents()
