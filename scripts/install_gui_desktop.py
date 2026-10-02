#!/usr/bin/env python3
"""Install launchers and matching Ubuntu icons for the four MuR GUIs."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import tempfile


MARKER = 'match_mur_gui/scripts/install_gui_desktop.py'
APPS = {
    'base': ('mur-base-gui', 'MuR Basis-GUI', 'MuR-Roboter bedienen',
             'match_mur_gui', 'general_mur_gui.py', 'MuR;Roboter;Basis;ROS;'),
    'mocap': ('mur-mocap-gui', 'MuR Mocap', 'Qualisys-Posen und MuR-Roboter beobachten',
              'match_mocap_gui', 'mocap_gui', 'MuR;Mocap;Qualisys;ROS;'),
    'cooperative': ('mur-cooperative-gui', 'MuR Cooperative Handling',
                    'Kooperative Handhabung mit MuR-Robotern',
                    'match_cooperative_handling', 'cooperative_handling_gui.py',
                    'MuR;Kooperation;Handling;ROS;'),
    'oak': ('mur-oak-gui', 'MuR OAK Kamera', 'OAK-Kamera an MuR-Robotern bedienen',
            'oak_camera_calibration', 'oak_camera_gui', 'MuR;OAK;Kamera;ROS;'),
}


def default_data_home(home: Path, environ: dict[str, str]) -> Path:
    value = environ.get('XDG_DATA_HOME')
    if not value:
        return home / '.local/share'
    candidate = Path(value).expanduser()
    if not candidate.is_absolute():
        return home / '.local/share'
    # VS Code as a Snap exports a private path invisible to the normal app menu.
    if environ.get('SNAP') and candidate.is_relative_to(home / 'snap'):
        return home / '.local/share'
    return candidate


def desktop_dir(home: Path) -> Path | None:
    if shutil.which('xdg-user-dir'):
        try:
            result = subprocess.run(['xdg-user-dir', 'DESKTOP'], capture_output=True,
                                    text=True, timeout=3)
            candidate = Path(result.stdout.strip())
            if result.returncode == 0 and candidate.is_absolute() and candidate.is_dir():
                return candidate
        except (OSError, subprocess.TimeoutExpired):
            pass
    for name in ('Desktop', 'Schreibtisch'):
        candidate = home / name
        if candidate.is_dir():
            return candidate
    return None


def _desktop_exec(path: Path) -> str:
    value = str(path)
    if not re.fullmatch(r'[A-Za-z0-9_./+@\- ]+', value):
        raise ValueError(f'Launcher-Pfad enthält nicht unterstützte Desktop-Zeichen: {path}')
    return '"' + value + '"'


def _entry(app: str, launcher: Path, icon_path: Path) -> bytes:
    slug, title, comment, _package, _executable, keywords = APPS[app]
    return ('[Desktop Entry]\n'
            'Type=Application\n'
            f'Name={title}\n'
            'GenericName=Roboter-GUI\n'
            f'Comment={comment}\n'
            f'Exec={_desktop_exec(launcher)}\n'
            f'Icon={icon_path}\n'
            f'StartupWMClass={slug}\n'
            'Terminal=false\n'
            'Categories=Utility;\n'
            f'Keywords={keywords}\n'
            'StartupNotify=true\n'
            f'X-MuR-Gui-Managed={MARKER}\n').encode('utf-8')


def _launcher(app: str, ros_setup: Path, workspace: Path) -> bytes:
    _slug, _title, _comment, package, executable, _keywords = APPS[app]
    setup = workspace / 'install/setup.bash'
    return ('#!/usr/bin/env bash\n'
            f'# Managed by {MARKER}\n'
            'set -e\n'
            'export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-62}"\n'
            f'source {shlex.quote(str(ros_setup))}\n'
            f'source {shlex.quote(str(setup))}\n'
            f'exec ros2 run {package} {executable} "$@"\n').encode('utf-8')


def _validate(app: str, ros_setup: Path, workspace: Path) -> None:
    setup = workspace / 'install/setup.bash'
    if not ros_setup.is_file():
        raise ValueError(f'ROS-Setup fehlt: {ros_setup}')
    if not setup.is_file():
        raise ValueError(f'Workspace-Setup fehlt: {setup}. Zuerst den Workspace bauen.')
    package, executable = APPS[app][3:5]
    environment = os.environ.copy()
    for key in ('ROS_DISTRO', 'ROS_VERSION', 'ROS_PYTHON_VERSION', 'ROS_ROOT',
                'ROS_PACKAGE_PATH', 'AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH',
                'CMAKE_PREFIX_PATH', 'LD_LIBRARY_PATH', 'PYTHONPATH', 'PYTHONHOME',
                'GIO_MODULE_DIR'):
        environment.pop(key, None)
    command = ['bash', '--noprofile', '--norc', '-c',
               'source "$1" && source "$2" && ros2 pkg executables "$3"',
               '_', str(ros_setup), str(setup), package]
    try:
        result = subprocess.run(command, capture_output=True, text=True,
                                timeout=20, env=environment)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise ValueError(f'{package}: ROS-Startbefehl konnte nicht geprüft werden: {exc}') from exc
    found = any(line.split() == [package, executable] for line in result.stdout.splitlines())
    if result.returncode != 0 or not found:
        detail = result.stderr.strip().splitlines()[-1] if result.stderr.strip() else 'Executable nicht gefunden'
        raise ValueError(f'{package}/{executable} ist im Workspace nicht startbar ({detail}). '
                         'Zuerst das Paket mit colcon bauen.')


def _atomic_write(path: Path, data: bytes, mode: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile('wb', dir=path.parent, prefix='.mur-gui-', delete=False) as temp:
        temp.write(data)
        temporary = Path(temp.name)
    try:
        temporary.chmod(mode)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def _planned_files(source: Path, apps: tuple[str, ...], data_home: Path,
                   bin_dir: Path, shortcut_dir: Path | None,
                   ros_setup: Path, workspace: Path) -> dict[Path, tuple[bytes, int]]:
    files = {}
    for app in apps:
        slug = APPS[app][0]
        launcher = bin_dir / slug
        icon_path = data_home / 'icons/hicolor/128x128/apps' / (slug + '.png')
        entry = _entry(app, launcher, icon_path)
        files[launcher] = (_launcher(app, ros_setup, workspace), 0o755)
        files[data_home / 'applications' / (slug + '.desktop')] = (entry, 0o644)
        if shortcut_dir is not None:
            files[shortcut_dir / (slug + '.desktop')] = (entry, 0o755)
        for size in ('scalable', '48x48', '128x128'):
            suffix = '.svg' if size == 'scalable' else '.png'
            source_name = slug + ('' if size == 'scalable' else '-' + size.split('x')[0]) + suffix
            asset = source / 'assets/icons' / source_name
            if asset.is_symlink() or not asset.is_file():
                raise ValueError(f'Icon fehlt oder ist ein Symlink: {asset}')
            files[data_home / 'icons/hicolor' / size / 'apps' / (slug + suffix)] = (asset.read_bytes(), 0o644)
    return files


def install(source: Path, apps: tuple[str, ...], data_home: Path, bin_dir: Path,
            *, shortcut_dir: Path | None, ros_setup: Path, workspace: Path,
            validate: bool = True) -> tuple[dict[Path, tuple[bytes, int]], list[Path]]:
    source = Path(source).expanduser().resolve()
    data_home = Path(data_home).expanduser().absolute()
    bin_dir = Path(bin_dir).expanduser().absolute()
    ros_setup = Path(ros_setup).expanduser().absolute()
    workspace = Path(workspace).expanduser().absolute()
    shortcut_dir = Path(shortcut_dir).expanduser().absolute() if shortcut_dir else None
    for app in apps:
        if app not in APPS:
            raise ValueError(f'Unbekannte GUI: {app}')
    files = _planned_files(source, apps, data_home, bin_dir, shortcut_dir, ros_setup, workspace)
    manifest_path = data_home / 'mur-gui-desktop/install.json'
    managed = set()
    if manifest_path.exists() or manifest_path.is_symlink():
        if manifest_path.is_symlink() or not manifest_path.is_file():
            raise ValueError(f'Installationsmanifest ist keine reguläre Datei: {manifest_path}')
        try:
            old = json.loads(manifest_path.read_text(encoding='utf-8'))
        except (OSError, ValueError) as exc:
            raise ValueError(f'Installationsmanifest ist ungültig: {manifest_path}') from exc
        if old.get('managed_by') != MARKER or not isinstance(old.get('paths'), list):
            raise ValueError(f'Fremdes Installationsmanifest nicht überschreiben: {manifest_path}')
        managed = set(old['paths'])
    for path, (data, _mode) in files.items():
        if path.is_symlink() or (path.exists() and not path.is_file()):
            raise ValueError(f'Vorhandene Verknüpfung/Datei nicht überschreiben: {path}')
        if path.exists() and str(path) not in managed and path.read_bytes() != data:
            raise ValueError(f'Fremde Datei nicht überschreiben: {path}')
    if validate:
        for app in apps:
            _validate(app, ros_setup, workspace)
    for path, (data, mode) in files.items():
        if not path.exists() or path.read_bytes() != data or (path.stat().st_mode & 0o777) != mode:
            _atomic_write(path, data, mode)
    manifest = {'managed_by': MARKER,
                'paths': sorted(managed | {str(path) for path in files})}
    _atomic_write(manifest_path, (json.dumps(manifest, ensure_ascii=False, indent=2) + '\n').encode(), 0o644)
    untrusted = []
    if shortcut_dir is not None and shutil.which('gio'):
        environment = os.environ.copy()
        environment['XDG_DATA_HOME'] = str(data_home)
        if environment.get('SNAP'):
            environment.pop('GIO_MODULE_DIR', None)
        for app in apps:
            shortcut = shortcut_dir / (APPS[app][0] + '.desktop')
            try:
                result = subprocess.run(['gio', 'set', '--type', 'string', str(shortcut),
                                         'metadata::trusted', 'true'], capture_output=True,
                                        text=True, timeout=5, env=environment)
                if result.returncode != 0:
                    untrusted.append(shortcut)
            except (OSError, subprocess.TimeoutExpired):
                untrusted.append(shortcut)
    return files, untrusted


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source = Path(__file__).resolve().parents[1]
    home = Path.home()
    parser.add_argument('--app', choices=('all', *APPS), default='all',
                        help='GUI auswählen; Standard: alle vier')
    parser.add_argument('--workspace', type=Path, default=source.parent.parent,
                        help='Colcon-Workspace; Standard: Workspace dieses Repositories')
    parser.add_argument('--ros-setup', type=Path, default=Path('/opt/ros/jazzy/setup.bash'))
    parser.add_argument('--data-home', type=Path, default=default_data_home(home, os.environ))
    parser.add_argument('--bin-dir', type=Path, default=home / '.local/bin')
    parser.add_argument('--desktop-dir', type=Path, help='Desktop-Ordner; Standard: xdg-user-dir DESKTOP')
    parser.add_argument('--no-desktop-shortcut', action='store_true',
                        help='Nur Einträge für die Anwendungssuche anlegen')
    args = parser.parse_args(argv)
    apps = tuple(APPS) if args.app == 'all' else (args.app,)
    shortcut = None if args.no_desktop_shortcut else args.desktop_dir or desktop_dir(home)
    try:
        _files, untrusted = install(source, apps, args.data_home, args.bin_dir,
                                    shortcut_dir=shortcut, ros_setup=args.ros_setup,
                                    workspace=args.workspace)
    except (OSError, ValueError) as exc:
        print(f'Desktop-Installation abgebrochen: {exc}', file=sys.stderr)
        return 1
    for app in apps:
        slug, title = APPS[app][:2]
        print(f'{title}: {Path(args.data_home) / "applications" / (slug + ".desktop")}')
    if shortcut is None:
        print('Kein Desktop-Ordner gewählt oder gefunden; Anwendungssuche ist eingerichtet.')
    elif untrusted:
        print('Bei nicht freigegebenen Desktop-Icons im Dateimanager „Starten erlauben“ wählen.',
              file=sys.stderr)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
