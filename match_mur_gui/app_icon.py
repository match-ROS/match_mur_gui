"""Give standalone MuR GUI windows the icon and desktop identity of their launcher."""

from __future__ import annotations

import os
from pathlib import Path

from PyQt5 import QtGui, QtWidgets


ICON_NAMES = frozenset({
    'mur-base-gui', 'mur-mocap-gui', 'mur-cooperative-gui', 'mur-oak-gui',
})


def configure_gui_icon(app: QtWidgets.QApplication, name: str) -> QtGui.QIcon:
    """Set Qt's window icon and desktop-file identity before creating the window."""
    if name not in ICON_NAMES:
        raise ValueError(f'Unbekannter MuR-GUI-Iconname: {name}')
    app.setApplicationName(name)
    app.setDesktopFileName(name + '.desktop')

    data_homes = []
    configured = os.environ.get('XDG_DATA_HOME')
    if configured and Path(configured).is_absolute():
        data_homes.append(Path(configured))
    data_homes.append(Path.home() / '.local/share')
    for data_home in data_homes:
        candidate = data_home / 'icons/hicolor/128x128/apps' / (name + '.png')
        if candidate.is_file():
            icon = QtGui.QIcon(str(candidate))
            if not icon.isNull():
                app.setWindowIcon(icon)
                return icon

    source_icon = Path(__file__).resolve().parents[1] / 'assets/icons' / (name + '.svg')
    if source_icon.is_file():
        icon = QtGui.QIcon(str(source_icon))
        if not icon.isNull():
            app.setWindowIcon(icon)
            return icon

    icon = QtGui.QIcon.fromTheme(name)
    if not icon.isNull():
        app.setWindowIcon(icon)
    return icon
