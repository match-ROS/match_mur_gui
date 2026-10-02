#!/usr/bin/env python3
"""General local/remote MuR hardware GUI."""

import signal
import sys

from PyQt5 import QtWidgets

from match_mur_gui.base_gui import MurBaseGui
from match_mur_gui.app_icon import configure_gui_icon


def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtWidgets.QApplication(sys.argv)
    icon = configure_gui_icon(app, "mur-base-gui")
    window = MurBaseGui(window_title="General MuR GUI")
    window.setWindowIcon(icon)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
