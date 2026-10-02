"""User-level desktop entries for the four MuR GUIs."""

import importlib.util
from pathlib import Path
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('mur_gui_desktop', ROOT / 'scripts/install_gui_desktop.py')
INSTALLER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(INSTALLER)


def _install(tmp_path, monkeypatch, apps=tuple(INSTALLER.APPS), shortcut=True):
    original_which = shutil.which
    monkeypatch.setattr(INSTALLER.shutil, 'which',
                        lambda name: None if name == 'gio' else original_which(name))
    desktop = tmp_path / 'Desktop' if shortcut else None
    if desktop:
        desktop.mkdir(exist_ok=True)
    files, untrusted = INSTALLER.install(
        ROOT, apps, tmp_path / 'share', tmp_path / 'bin', shortcut_dir=desktop,
        ros_setup=Path('/opt/ros/jazzy/setup.bash'), workspace=tmp_path / 'workspace',
        validate=False)
    return files, untrusted, desktop


def test_four_entries_icons_and_launchers_are_repeatable(tmp_path, monkeypatch):
    files, untrusted, desktop = _install(tmp_path, monkeypatch)
    assert not untrusted
    for app, (slug, title, _comment, package, executable, _keywords) in INSTALLER.APPS.items():
        entry = tmp_path / 'share/applications' / (slug + '.desktop')
        wrapper = tmp_path / 'bin' / slug
        shortcut = desktop / (slug + '.desktop')
        assert f'Name={title}' in entry.read_text()
        assert f'Exec="{wrapper}"' in entry.read_text()
        icon_128 = tmp_path / 'share/icons/hicolor/128x128/apps' / (slug + '.png')
        assert f'Icon={icon_128}' in entry.read_text()
        assert f'StartupWMClass={slug}' in entry.read_text()
        assert shortcut.read_bytes() == entry.read_bytes()
        assert shortcut.stat().st_mode & 0o111
        assert f'exec ros2 run {package} {executable} "$@"' in wrapper.read_text()
        assert 'export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-62}"' in wrapper.read_text()
        assert wrapper.stat().st_mode & 0o111
        for size, extension in (('scalable', '.svg'), ('48x48', '.png'), ('128x128', '.png')):
            icon = tmp_path / 'share/icons/hicolor' / size / 'apps' / (slug + extension)
            assert icon.is_file()
    if shutil.which('desktop-file-validate'):
        subprocess.run(['desktop-file-validate', *map(str, desktop.glob('*.desktop'))], check=True)
    again, _, _ = _install(tmp_path, monkeypatch)
    assert files == again
    assert len(list((tmp_path / 'share/applications').glob('*.desktop'))) == 4


def test_foreign_launcher_blocks_all_writes(tmp_path, monkeypatch):
    launcher = tmp_path / 'bin/mur-base-gui'
    launcher.parent.mkdir()
    launcher.write_text('foreign')
    with pytest.raises(ValueError, match='Fremde'):
        _install(tmp_path, monkeypatch, ('base',), shortcut=False)
    assert launcher.read_text() == 'foreign'
    assert not (tmp_path / 'share').exists()


def test_workspace_with_spaces_is_quoted_in_launcher(tmp_path, monkeypatch):
    workspace = tmp_path / 'work space'
    original_which = shutil.which
    monkeypatch.setattr(INSTALLER.shutil, 'which',
                        lambda name: None if name == 'gio' else original_which(name))
    INSTALLER.install(ROOT, ('oak',), tmp_path / 'share', tmp_path / 'bin',
                      shortcut_dir=None, ros_setup=Path('/opt/ros/jazzy/setup.bash'),
                      workspace=workspace, validate=False)
    launcher = (tmp_path / 'bin/mur-oak-gui').read_text()
    assert f"source '{workspace}/install/setup.bash'" in launcher
    assert list((tmp_path / 'share/applications').glob('*.desktop')) == [
        tmp_path / 'share/applications/mur-oak-gui.desktop']


def test_snap_data_home_and_gio_trust(tmp_path, monkeypatch):
    home = tmp_path / 'user'
    env = {'SNAP': '/snap/code/249', 'XDG_DATA_HOME': str(home / 'snap/code/249/.local/share')}
    assert INSTALLER.default_data_home(home, env) == home / '.local/share'
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return subprocess.CompletedProcess(command, 0, '', '')

    desktop = tmp_path / 'Desktop'
    desktop.mkdir()
    monkeypatch.setenv('SNAP', '/snap/code/249')
    monkeypatch.setenv('GIO_MODULE_DIR', '/home/test/snap/code/common/.cache/gio-modules')
    monkeypatch.setattr(INSTALLER.shutil, 'which', lambda name: '/usr/bin/gio' if name == 'gio' else None)
    monkeypatch.setattr(INSTALLER.subprocess, 'run', fake_run)
    _, untrusted = INSTALLER.install(ROOT, ('base',), tmp_path / 'share', tmp_path / 'bin',
                                     shortcut_dir=desktop, ros_setup=Path('/opt/ros/jazzy/setup.bash'),
                                     workspace=tmp_path / 'workspace', validate=False)
    assert not untrusted
    command, kwargs = calls[-1]
    assert command[:4] == ['gio', 'set', '--type', 'string']
    assert kwargs['env']['XDG_DATA_HOME'] == str(tmp_path / 'share')
    assert 'GIO_MODULE_DIR' not in kwargs['env']


def test_missing_ros_setup_has_clear_message(tmp_path):
    with pytest.raises(ValueError, match='ROS-Setup fehlt'):
        INSTALLER._validate('base', tmp_path / 'missing.bash', tmp_path)
