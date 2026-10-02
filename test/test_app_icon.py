"""Each standalone MuR window carries its matching desktop identity and icon."""

import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
from PyQt5 import QtWidgets

from match_mur_gui.app_icon import ICON_NAMES, configure_gui_icon


def test_all_gu_is_have_distinct_nonempty_window_icons():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    rendered = set()
    for name in sorted(ICON_NAMES):
        icon = configure_gui_icon(app, name)
        assert app.applicationName() == name
        assert app.desktopFileName() == name + '.desktop'
        assert not app.windowIcon().isNull()
        pixmap = icon.pixmap(48, 48)
        assert not pixmap.isNull()
        image = pixmap.toImage()
        rendered.add(bytes(image.bits().asstring(image.byteCount())))
        window = QtWidgets.QMainWindow()
        window.setWindowIcon(icon)
        assert not window.windowIcon().isNull()
    assert len(rendered) == len(ICON_NAMES)


def test_unknown_icon_name_is_rejected():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    with pytest.raises(ValueError, match='Unbekannter'):
        configure_gui_icon(app, 'unknown-gui')
