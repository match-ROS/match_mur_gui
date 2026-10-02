"""The shared GUI keeps base actions, module actions and views separate."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5 import QtWidgets

from match_mur_gui import base_gui


class _Signal:
    def connect(self, _callback):
        pass


class _RosWorker:
    def __init__(self, _robots):
        self.log = _Signal()
        self.freedrive_status = _Signal()
        self.battery_status = _Signal()

    def start(self):
        pass

    def shutdown(self):
        pass

    def wait(self, _timeout):
        return True


class _Module(base_gui.MurGuiModule):
    def setup_ui(self, context):
        context.add_action_button("Object action", lambda: None, section="Cooperative")
        context.add_module_tab(QtWidgets.QWidget(), "Mocap")
        context.add_panel(QtWidgets.QLabel("Map view"))


def _tab_titles(tabs):
    return [tabs.tabText(index) for index in range(tabs.count())]


def test_three_panes_and_action_tabs(monkeypatch, tmp_path):
    monkeypatch.setattr(base_gui, "RosWorker", _RosWorker)
    monkeypatch.setattr(
        base_gui.MurBaseGui,
        "_create_gui_log_file",
        lambda self: str(tmp_path / "gui.log"),
    )
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = base_gui.MurBaseGui(modules=[_Module()])
    window.stop_managed_processes = lambda: None
    window.resize(1280, 1000)
    window.show()
    app.processEvents()

    assert _tab_titles(window.section_container) == ["General", "MiR", "UR"]
    assert _tab_titles(window.module_tabs) == ["Cooperative", "Mocap"]
    assert window.work_area_splitter.count() == 3
    assert all(width > 0 for width in window.work_area_splitter.sizes())
    assert window.extension_container.findChild(QtWidgets.QLabel).text() == "Map view"
    general_button = window.section_container.currentWidget().findChildren(
        QtWidgets.QPushButton
    )[0]
    module_button = window.module_tabs.currentWidget().findChildren(
        QtWidgets.QPushButton
    )[0]
    narrow_widths = (general_button.width(), module_button.width())
    assert general_button.height() >= 36
    assert module_button.height() >= 36
    assert general_button.width() >= window.section_container.width() * 0.35
    assert module_button.width() >= window.module_tabs.width() * 0.35

    window.resize(2048, 1152)
    app.processEvents()
    assert general_button.width() > narrow_widths[0]
    assert module_button.width() > narrow_widths[1]
    window.section_container.setCurrentIndex(2)
    window.module_tabs.setCurrentIndex(1)
    assert window.section_container.tabText(window.section_container.currentIndex()) == "UR"
    assert window.module_tabs.tabText(window.module_tabs.currentIndex()) == "Mocap"
    window.close()


def test_base_gui_has_only_base_tabs(monkeypatch, tmp_path):
    monkeypatch.setattr(base_gui, "RosWorker", _RosWorker)
    monkeypatch.setattr(
        base_gui.MurBaseGui,
        "_create_gui_log_file",
        lambda self: str(tmp_path / "gui.log"),
    )
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = base_gui.MurBaseGui()
    window.stop_managed_processes = lambda: None
    window.show()
    app.processEvents()
    assert _tab_titles(window.section_container) == ["General", "MiR", "UR"]
    assert not window.module_tabs.isVisible()
    assert not window.extension_container.isVisible()
    window.close()
