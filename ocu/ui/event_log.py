"""Bottom panel, event log and link strips, section 9.5.

A timestamped line for every mode change, launch and recovery stage
transition, detection, alert, and link drop or restore, worded so the log
reads like a mission record rather than a debug console. Lines carry a
severity, which is the only thing that colours them, and a subject, so that
a reading climbing on one object rewrites its own line rather than filling
the log with itself.

The link strips are rolling history, not current state. A bar showing the
present link state duplicates the dot already on each roster card; what
an operator cannot get anywhere else is *when* a link dropped and for how
long, which is exactly what the section 8.5 cut/restore demonstration is
about. Each lane scrolls right to left, so a cut leaves a red band that
visibly drifts into the past after the restore.
"""

from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, Qt, QTimer
from PySide6.QtGui import QBrush, QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ocu.ui.labels import short_label
from ocu.ui.theme import FONT_LABEL, HAIRLINE, TEXT_DIM, TEXT_FAINT

#: Severity, and the only reason a log line is ever coloured. An operator
#: scanning the log under pressure finds the alert by colour or not at all.
LEVEL_COLORS = {
    "info": QColor("#7B8794"),
    "good": QColor("#4F9D69"),
    "caution": QColor("#C9952F"),
    "threat": QColor("#C04438"),
}

#: A mission produces more lines than anyone scrolls back through, and an
#: unbounded list is a slow leak in a display meant to run for a whole shift.
MAX_LOG_ENTRIES = 400

ROW_ORDER = ["rover", "tricopter-1", "tricopter-2", "tricopter-3"]
HISTORY_SAMPLES = 120
SAMPLE_MS = 500
LANE_H = 13

#: Only an interruption is coloured. CONNECTED is deliberately absent: the
#: dim rule under each lane already says the link was up.
OUTAGE_COLORS = {
    "DEGRADED": QColor("#C9952F"),
    "LOST": QColor("#C04438"),
    "SILENT": QColor("#3A4756"),
}

LINK_COLORS = {
    "SILENT": QColor("#1E2630"),
    "CONNECTED": QColor("#4F9D69"),
    "DEGRADED": QColor("#C9952F"),
    "LOST": QColor("#C04438"),
}


class LinkHistoryStrip(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setFixedHeight(LANE_H * len(ROW_ORDER) + 26)
        self.setMinimumWidth(300)
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
        """Sixty seconds of link state per unit.

        Drawn as exception rather than as state: a connected link is a dim
        rule, and only a degraded or lost one is given colour and height. A
        strip that paints ninety-five percent of its area solid green to say
        "nothing happened" spends all its contrast on the usual case, and the
        one thing worth seeing -- the moment a link dropped, and for how long
        -- has none left to stand out in.
        """
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, False)

        label_w = 46
        track_x = label_w + 6

        painter.setPen(QPen(TEXT_DIM))
        painter.setFont(QFont(FONT_LABEL, 11, QFont.Bold))
        # The scale below already states the span, so the title does not.
        painter.drawText(QRectF(0, 0, label_w, 13), Qt.AlignLeft, "LINK")
        track_w = max(self.width() - track_x - 10, 10)
        sample_w = track_w / HISTORY_SAMPLES

        # A scale, so "how long was it out" is readable rather than guessed.
        painter.setPen(QPen(HAIRLINE))
        painter.setFont(QFont("Menlo", 7))
        for fraction, caption, align in (
            (0.0, "-60s", Qt.AlignLeft),
            (0.5, "-30s", Qt.AlignHCenter),
            (1.0, "now", Qt.AlignRight),
        ):
            x = track_x + track_w * fraction
            painter.drawLine(QPointF(x, 15), QPointF(x, 18))
            painter.setPen(QPen(TEXT_FAINT))
            # Anchored to its own end of the scale rather than centred on it,
            # so the first and last captions stay inside the strip.
            box = QRectF(x - 34, 4, 34, 10) if align == Qt.AlignRight else QRectF(x, 4, 34, 10)
            if align == Qt.AlignHCenter:
                box = QRectF(x - 17, 4, 34, 10)
            painter.drawText(box, align | Qt.AlignVCenter, caption)
            painter.setPen(QPen(HAIRLINE))

        for row, uid in enumerate(ROW_ORDER):
            y = 22 + row * LANE_H
            mid = y + LANE_H / 2

            painter.setPen(QPen(TEXT_DIM))
            painter.setFont(QFont(FONT_LABEL, 11, QFont.Bold))
            painter.drawText(
                QRectF(0, y, label_w, LANE_H), Qt.AlignLeft | Qt.AlignVCenter, short_label(uid)
            )

            # The baseline: present, and quiet.
            painter.setPen(QPen(HAIRLINE))
            painter.drawLine(QPointF(track_x, mid), QPointF(track_x + track_w, mid))

            painter.setPen(Qt.NoPen)
            for index, state in enumerate(self._history[uid]):
                colour = OUTAGE_COLORS.get(state)
                if colour is None:
                    continue  # connected: the rule already says so
                painter.fillRect(
                    QRectF(track_x + index * sample_w, y + 2, sample_w + 0.6, LANE_H - 4),
                    colour,
                )

            # Where it stands now, at the live edge.
            current = self._states.get(uid, "CONNECTED")
            painter.setBrush(QBrush(LINK_COLORS.get(current, HAIRLINE)))
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(QPointF(track_x + track_w + 5, mid), 2.5, 2.5)
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

    def log(
        self,
        mission_clock_s: float,
        message: str,
        key: str | None = None,
        level: str = "info",
    ) -> None:
        """Add a line, or rewrite the last one if it is about the same thing.

        A confidence climbing on one buried object is one event the operator
        is watching, not five lines of it. Repeating the line for each step
        pushes everything else off the visible log, which is the one place a
        missed alert can still be found. An entry therefore carries the
        subject it is about, and a fresh report on the subject already at the
        bottom rewrites that line instead of adding another.
        """
        minutes, seconds = divmod(int(mission_clock_s), 60)
        text = f"[{minutes:02d}:{seconds:02d}] {message}"
        colour = LEVEL_COLORS.get(level, LEVEL_COLORS["info"])

        last = self.log_list.item(self.log_list.count() - 1)
        if key is not None and last is not None and last.data(Qt.UserRole) == key:
            last.setText(text)
            last.setForeground(colour)
        else:
            item = QListWidgetItem(text)
            item.setData(Qt.UserRole, key)
            item.setForeground(colour)
            self.log_list.addItem(item)
            while self.log_list.count() > MAX_LOG_ENTRIES:
                self.log_list.takeItem(0)
        self.log_list.scrollToBottom()

    def set_link(self, unit_id: str, state: str) -> None:
        self.link_strip.set_link(unit_id, state)

    def clear(self) -> None:
        self.log_list.clear()
        self.link_strip.clear()
