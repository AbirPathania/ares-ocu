"""Entry point. Loads the dark theme and opens the main window.

Run with: .venv/bin/python -m ocu.main
"""

from __future__ import annotations

import sys
from pathlib import Path

from PySide6.QtWidgets import QApplication

from ocu.ui.main_window import MainWindow

STYLE_PATH = Path(__file__).parent / "ui" / "style.qss"


#: The size the layout is designed around, and how much of the screen it may
#: take when the screen is smaller than that.
PREFERRED_SIZE = (1600, 960)
SCREEN_FRACTION = 0.92


def opening_size(app: QApplication):
    """The size to open at: the preferred one, or what the screen can take.

    A console that opens larger than the display puts its own controls off
    the edge of it, and the operator's first task is then to find them.
    """
    from PySide6.QtCore import QSize

    screen = app.primaryScreen()
    if screen is None:
        return QSize(*PREFERRED_SIZE)
    available = screen.availableGeometry()
    return QSize(
        int(min(PREFERRED_SIZE[0], available.width() * SCREEN_FRACTION)),
        int(min(PREFERRED_SIZE[1], available.height() * SCREEN_FRACTION)),
    )


def main() -> int:
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE_PATH.read_text())

    window = MainWindow()
    # The feeds are drawn as 3D on the GPU where there is one; without it the
    # numpy renderer is used and the application is none the worse for it.
    window.enable_gpu_cameras()
    # The thermal detector and the infrared tiles need the day's temperature field; start getting it now.
    window.warm_thermal()
    window.resize(opening_size(app))
    window.show()
    window.warm_thermal()  # infrared is ready by the time it is asked for

    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
