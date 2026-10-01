"""Bottom panel, event log and link strips, section 9.5.

A timestamped line for every mode change, launch and recovery stage
transition, detection, alert, and link drop or restore, worded using the
phase language from section 5 so the log reads like a mission record
rather than a debug console.

The link strips are rolling history, not current state. A bar showing the
present link state duplicates the dot already on each roster card; what
an operator cannot get anywhere else is *when* a link dropped and for how
long, which is exactly what the section 8.5 cut/restore demonstration is
about. Each lane scrolls right to left, so a cut leaves a red band that
visibly drifts into the past after the restore.
"""

from __future__ import annotations

from PySide6.QtCore import QRectF, Qt, QTimer
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QHBoxLayout, QLabel, QListWidget, QVBoxLayout, QWidget

ROW_ORDER = ["rover", "tricopter-1", "tricopter-2", "tricopter-3"]
HISTORY_SAMPLES = 120
SAMPLE_MS = 500
LANE_H = 13

LINK_COLORS = {
    "CONNECTED": QColor("#4F9D69"),
    "DEGRADED": QColor("#C9952F"),
    "LOST": QColor("#C04438"),
}


class LinkHistoryStrip(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setFixedHeight(LANE_H * len(ROW_ORDER) + 18)
        self.setMinimumWidth(240)
        self._states: dict[str, str] = {uid: "CONNECTED" for uid in ROW_ORDER}
        self._history: dict[str, list[str]] = {
            uid: ["CONNECTED"] * HISTORY_SAMPLES for uid in ROW_ORDER
        }

        self._timer = QTimer(self)
        self._timer.setInterval(SAMPLE_MS)
        self._timer.timeout.connect(self._sample)
        self._timer.start()

    def set_link(self, unit_id: str, state: str) -> None:
        if unit_id in self._states:
            self._states[unit_id] = state

    def clear(self) -> None:
        for uid in ROW_ORDER:
            self._states[uid] = "CONNECTED"
            self._history[uid] = ["CONNECTED"] * HISTORY_SAMPLES
        self.update()

    def _sample(self) -> None:
        for uid in ROW_ORDER:
            history = self._history[uid]
            history.append(self._states[uid])
            if len(history) > HISTORY_SAMPLES:
                history.pop(0)
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setPen(QPen(QColor("#7B8794")))
        painter.setFont(QFont("Menlo", 8))
        painter.drawText(QRectF(0, 0, self.width(), 14), Qt.AlignLeft, "LINK HISTORY  ← 60s")

        label_w = 54
        track_x = label_w + 4
        track_w = max(self.width() - track_x - 4, 10)
        sample_w = track_w / HISTORY_SAMPLES

        for i, uid in enumerate(ROW_ORDER):
            y = 16 + i * LANE_H
            painter.setPen(QPen(QColor("#7B8794")))
            painter.setFont(QFont("Menlo", 8))
            painter.drawText(QRectF(0, y, label_w, LANE_H), Qt.AlignLeft | Qt.AlignVCenter, uid)

            painter.setPen(Qt.NoPen)
            for j, state in enumerate(self._history[uid]):
                painter.fillRect(
                    QRectF(track_x + j * sample_w, y + 3, sample_w + 0.6, LANE_H - 6),
                    LINK_COLORS.get(state, QColor("#3A4756")),
                )
        painter.end()


class EventLog(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        log_col = QVBoxLayout()
        log_col.setContentsMargins(0, 0, 0, 0)
        log_col.setSpacing(0)
        header = QLabel("EVENT LOG")
        header.setObjectName("sectionLabel")
        log_col.addWidget(header)

        self.log_list = QListWidget()
        self.log_list.setObjectName("eventLogList")
        log_col.addWidget(self.log_list, stretch=1)
        layout.addLayout(log_col, stretch=1)

        self.link_strip = LinkHistoryStrip()
        layout.addWidget(self.link_strip, alignment=Qt.AlignTop)

    def log(self, mission_clock_s: float, message: str) -> None:
        minutes, seconds = divmod(int(mission_clock_s), 60)
        self.log_list.addItem(f"[{minutes:02d}:{seconds:02d}] {message}")
        self.log_list.scrollToBottom()

    def set_link(self, unit_id: str, state: str) -> None:
        self.link_strip.set_link(unit_id, state)

    def clear(self) -> None:
        self.log_list.clear()
        self.link_strip.clear()
