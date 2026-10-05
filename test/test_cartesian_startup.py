"""Non-actuating controller activation checks."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

spec = importlib.util.spec_from_file_location(
    'set_cartesian_controller', Path(__file__).parents[1] / 'scripts/set_cartesian_controller.py')
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)


def controller(name, state='active', interfaces=()):
    return SimpleNamespace(name=name, state=state, claimed_interfaces=list(interfaces))


def test_enable_replaces_hold_controller_but_preserves_io():
    request = helper.switch_request([
        controller(helper.CONTROLLER, 'inactive'),
        controller('scaled_joint_trajectory_controller', interfaces=['joint/position']),
        controller('io_and_status_controller', interfaces=['gpio/digital_output']),
    ], False)
    assert list(request.activate_controllers) == [helper.CONTROLLER]
    assert list(request.deactivate_controllers) == ['scaled_joint_trajectory_controller']


def test_enable_does_not_take_over_freedrive():
    with pytest.raises(RuntimeError):
        helper.switch_request([
            controller(helper.CONTROLLER, 'inactive'),
            controller('forward_velocity_controller', interfaces=['joint/velocity']),
        ], False)


def test_disable_never_activates_another_controller():
    request = helper.switch_request([controller(helper.CONTROLLER)], True)
    assert not request.activate_controllers
    assert list(request.deactivate_controllers) == [helper.CONTROLLER]


def test_enable_requires_configured_controller():
    with pytest.raises(RuntimeError):
        helper.switch_request([controller(helper.CONTROLLER, 'unconfigured')], False)


def test_vendor_freedrive_interface_blocks_activation():
    with pytest.raises(RuntimeError):
        helper.switch_request([
            controller(helper.CONTROLLER, 'inactive'),
            controller('freedrive_mode_controller', interfaces=['freedrive/enable']),
        ], False)


def test_gui_hardware_start_does_not_activate_motion():
    from match_mur_gui.base_gui import MurBaseGui
    commands = []
    checked = SimpleNamespace(isChecked=lambda: True)
    unchecked = SimpleNamespace(isChecked=lambda: False)
    window = SimpleNamespace(
        arm_r=checked, arm_l=checked, opt_integrated=checked, opt_ft=checked,
        opt_require_wrench=unchecked, opt_zero_admittance=unchecked,
        opt_collision=checked, opt_markers=unchecked, opt_moveit=checked,
        opt_build=checked, mir_camera_check=unchecked,
        launch_mir_enabled=lambda: False, selected_sides=lambda: ['l', 'r'],
        remote_hardware_script=lambda: '/repo/start_mur620_hardware_logged.sh',
        remote_command=lambda robot, command: command,
        process_key=lambda robot, name: name,
        start_process=lambda name, command, **kwargs: commands.append(command),
        _ur_starting_pairs=set(),
    )
    MurBaseGui._launch_hardware_for_robot(window, 'mur620a')
    command = commands[0]
    assert 'INTEGRATED_CARTESIAN_ACTIVE=false;' in command
    assert 'use_integrated_cartesian_admittance_controller:=true' in command
    assert 'integrated_controller_initial_active:=false' in command
    assert 'activate_joint_controller:=false' in command
