"""Jog dialogs must not overwrite driving with stale or idle stop streams."""

import os
from types import SimpleNamespace

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
from PyQt5 import QtCore, QtTest, QtWidgets

from match_mur_gui import base_gui


@pytest.fixture
def gui(monkeypatch):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = QtWidgets.QMainWindow()
    commands = []
    window.ros_worker = SimpleNamespace(
        publish_mir_twist=lambda *args: commands.append(args))

    def combo(_window, _placeholder):
        result = QtWidgets.QComboBox()
        result.addItem('mur620c')
        return result

    monkeypatch.setattr(base_gui, 'create_mir_namespace_combo', combo)
    yield app, window, commands
    window.close()
    window.deleteLater()
    QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
    app.processEvents()


def test_idle_jog_does_not_stream_stop_commands(gui):
    app, window, commands = gui
    dialog = base_gui.MiRJogDialog(window, window)
    dialog.show()
    app.processEvents()
    QtTest.QTest.qWait(120)
    assert commands == []
    dialog._set_motion(1.0, 0.0)
    dialog._tick()
    assert commands[-1] == ('mur620c', 0.1, 0.0)
    dialog.stop()
    assert commands[-3:] == [('mur620c', 0.0, 0.0)] * 3
    count = len(commands)
    QtTest.QTest.qWait(120)
    assert len(commands) == count


@pytest.mark.parametrize('dismiss', ['close', 'hide', 'reject'])
def test_dismissed_jog_stops_timer_and_reopens_idle(gui, dismiss):
    app, window, commands = gui
    dialog = base_gui.MiRJogDialog(window, window)
    dialog.show()
    app.processEvents()
    dialog._set_motion(1.0, 0.0)
    dialog._tick()
    getattr(dialog, dismiss)()
    app.processEvents()
    assert not dialog.timer.isActive()
    assert commands[-3:] == [('mur620c', 0.0, 0.0)] * 3
    count = len(commands)
    QtTest.QTest.qWait(120)
    assert len(commands) == count
    dialog.show()
    app.processEvents()
    assert dialog.timer.isActive()
    QtTest.QTest.qWait(120)
    assert len(commands) == count
    dialog._set_motion(0.0, -1.0)
    dialog._tick()
    assert commands[-1] == ('mur620c', 0.0, -0.3)


def test_open_jog_reuses_one_dialog_even_after_close(gui):
    app, window, _commands = gui
    base_gui.MurBaseGui.open_mir_jog(window)
    first = window._mir_jog_dialog
    base_gui.MurBaseGui.open_mir_jog(window)
    assert window._mir_jog_dialog is first
    assert len(window.findChildren(base_gui.MiRJogDialog)) == 1
    first.close()
    app.processEvents()
    base_gui.MurBaseGui.open_mir_jog(window)
    assert window._mir_jog_dialog is first
    assert first.isVisible()
    assert first.active_linear == first.active_angular == 0.0
