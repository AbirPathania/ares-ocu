"""Entry point. Loads the dark theme and opens the main window.

Run with: .venv/bin/python -m ocu.main
"""

from __future__ import annotations

import sys
from pathlib import Path

from PySide6.QtWidgets import QApplication

from ocu.ui.main_window import MainWindow

STYLE_PATH = Path(__file__).parent / "ui" / "style.qss"


def main() -> int:
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE_PATH.read_text())

    window = MainWindow()
    window.show()

    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
