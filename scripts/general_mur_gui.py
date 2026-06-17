#!/usr/bin/env python3
"""General local/remote MuR hardware GUI."""

import signal
import sys

from PyQt5 import QtWidgets

from match_mur_gui.base_gui import GeneralMuRGui


def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtWidgets.QApplication(sys.argv)
    window = GeneralMuRGui()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
