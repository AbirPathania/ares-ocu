"""Threat list, the operator's actual work product.

Detections are keyed by voxel, not by event, so four sensor hits on the
same hole in the ground are one threat with four contributing sensors and
a rising confidence, not four rows to reconcile by eye. That is the same
merge the shared map performs in core/mesh_state.py (section 8.2), shown
where the operator can act on it.

Each card carries the classification, the best fused confidence, which
sensors contributed, the map coordinate, and a status the operator can
change: PENDING until confirmed or rejected, per the section 8.3
human-in-the-loop rule.
"""

from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QFont, QFontMetricsF, QPainter, QPen
from PySide6.QtWidgets import QLabel, QPushButton, QScrollArea, QVBoxLayout, QWidget

from ocu.core.constants import ALERT_CONFIDENCE_THRESHOLD
from ocu.core.mesh_state import voxel_key
from ocu.ui.theme import (
    BACKGROUND,
    CAUTION,
    FONT_LABEL,
    FRIENDLY,
    HAIRLINE,
    INERT,
    PANEL_RAISED,
    TEXT,
    TEXT_DIM,
    TEXT_FAINT,
    THREAT,
)
from ocu.ui.icons import draw_threat_glyph
from ocu.ui.labels import sensor_list
from ocu.ui.motion import FAST_MS, Reveal

CARD_HEIGHT = 64

STATUS_COLORS = {
    "PENDING": CAUTION,
    "CONFIRMED": THREAT,
    "CLEARED": INERT,
}

#: One title size and one small size. Four sizes in a row this small is what
#: makes a list read as a form rather than as a register.
TITLE_PT = 14
SMALL_PT = 11

SELECTED_ROW = PANEL_RAISED

#: Width the CONFIRM / CLEAR pair takes at the right of a pending row.
ACTIONS_WIDTH = 146

def confidence_color(confidence: float) -> QColor:
    if confidence >= ALERT_CONFIDENCE_THRESHOLD:
        return THREAT
    if confidence >= 0.60:
        return CAUTION
    return FRIENDLY


class ThreatCard(QWidget):
    clicked = Signal(object)  # the threat dict
    confirmRequested = Signal(object)
    rejectRequested = Signal(object)

    def __init__(self, threat: dict, parent=None) -> None:
        super().__init__(parent)
        self.threat = threat
        self.setFixedHeight(CARD_HEIGHT)
        self.setCursor(Qt.PointingHandCursor)
        self._selected = False

        # The alert banner's countdown expires. The decision does not: an
        # unresolved threat has to stay actionable from the list afterwards,
        # or a missed banner means the operator can never answer at all.
        self.confirm_btn = QPushButton("CONFIRM", self)
        self.confirm_btn.setObjectName("cardConfirm")
        self.confirm_btn.clicked.connect(lambda: self.confirmRequested.emit(self.threat))
        self.reject_btn = QPushButton("CLEAR", self)
        self.reject_btn.setObjectName("cardReject")
        self.reject_btn.clicked.connect(lambda: self.rejectRequested.emit(self.threat))
        self._sync_actions()

        # A contact arriving in the list is new information. Fading it in
        # over a few frames is what distinguishes it from the rows that were
        # already there; appearing instantly, it reads as having always been.
        # Started last: revealing shows the widget, and showing it lays out
        # the buttons above, which have to exist by then.
        self._arrival = Reveal(self, duration_ms=FAST_MS)
        self._arrival.reveal()

    def _sync_actions(self) -> None:
        pending = self.threat.get("status") == "PENDING"
        for button in (self.confirm_btn, self.reject_btn):
            button.setVisible(pending)
        if pending:
            self.confirm_btn.setGeometry(self.width() - 134, CARD_HEIGHT - 23, 62, 17)
            self.reject_btn.setGeometry(self.width() - 68, CARD_HEIGHT - 23, 58, 17)

    def resizeEvent(self, event) -> None:
        self._sync_actions()
        super().resizeEvent(event)

    def set_selected(self, selected: bool) -> None:
        self._selected = selected
        self.update()

    def mousePressEvent(self, event) -> None:
        if self.childAt(event.position().toPoint()) is not None:
            return
        self.clicked.emit(self.threat)

    def paintEvent(self, event) -> None:
        """One row, drawn as little as it can be and still be read.

        A card used to carry a full border, a filled background, a ring
        around its glyph and four type sizes. That is four enclosures for one
        row of information, and when a dozen of them stack up the operator is
        reading boxes rather than contacts. What is left is the status spine,
        the glyph, the line that names the contact, the confidence against
        its threshold, and a hairline to the next row.
        """
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setOpacity(max(self._arrival.value, 0.0))
        rect = QRectF(self.rect())

        status = self.threat.get("status", "PENDING")
        status_color = STATUS_COLORS.get(status, INERT)
        confidence = self.threat.get("confidence", 0.0)

        # The row owns its background. Without this the widget shows Qt's
        # default, which is white: removing the card's fill as "chrome" is
        # what put a light panel in the middle of a dark console.
        painter.fillRect(rect, SELECTED_ROW if self._selected else BACKGROUND)
        # The spine is the only enclosure: it says status by colour and
        # separates one row from the next by being there at all.
        painter.fillRect(QRectF(rect.left(), rect.top(), 2, rect.height()), status_color)
        painter.setPen(QPen(HAIRLINE))
        painter.drawLine(
            QPointF(rect.left(), rect.bottom() - 0.5), QPointF(rect.right(), rect.bottom() - 0.5)
        )

        glyph_center = QPointF(rect.left() + 22, rect.top() + CARD_HEIGHT / 2)
        painter.setPen(QPen(status_color, 1.4))
        painter.setBrush(Qt.NoBrush)
        draw_threat_glyph(painter, glyph_center, 11, self.threat.get("type", "unknown"))

        text_left = rect.left() + 42
        painter.setPen(QPen(TEXT))
        title_font = QFont(FONT_LABEL, TITLE_PT, QFont.Bold)
        painter.setFont(title_font)
        title_room = rect.width() - text_left - 96
        painter.drawText(
            QRectF(text_left, rect.top() + 7, title_room, 15),
            Qt.AlignLeft | Qt.AlignVCenter,
            QFontMetricsF(title_font).elidedText(
                f'{str(self.threat.get("type", "unknown")).replace("_", " ").upper()}'
                f'   {self.threat["x"]:.0f}, {self.threat["y"]:.0f} m',
                Qt.ElideRight,
                max(title_room, 0.0),
            ),
        )

        painter.setPen(QPen(status_color))
        painter.setFont(QFont(FONT_LABEL, SMALL_PT, QFont.Bold))
        painter.drawText(
            QRectF(rect.right() - 92, rect.top() + 7, 86, 15), Qt.AlignRight | Qt.AlignVCenter, status
        )

        bar_rect = QRectF(text_left, rect.top() + 27, rect.width() - text_left - 100, 4)
        painter.setPen(Qt.NoPen)
        painter.fillRect(bar_rect, HAIRLINE)
        fill = QRectF(bar_rect)
        fill.setWidth(bar_rect.width() * min(max(confidence, 0.0), 1.0))
        painter.fillRect(fill, confidence_color(confidence))

        # Where the decision threshold sits, so a bar can be read against it
        # rather than against a number somewhere else.
        threshold_x = bar_rect.left() + bar_rect.width() * ALERT_CONFIDENCE_THRESHOLD
        painter.setPen(QPen(TEXT_FAINT, 1))
        painter.drawLine(
            QPointF(threshold_x, bar_rect.top() - 2), QPointF(threshold_x, bar_rect.bottom() + 2)
        )

        painter.setPen(QPen(TEXT))
        painter.setFont(QFont("Menlo", SMALL_PT))
        painter.drawText(
            QRectF(rect.right() - 92, rect.top() + 24, 86, 13),
            Qt.AlignRight | Qt.AlignVCenter,
            f"{confidence:.2f}",
        )

        # Clear of the action buttons, which sit at the right of this line
        # while a contact is pending: the sensor list gives way to them, not
        # the other way round.
        painter.setPen(QPen(TEXT_DIM))
        sensor_font = QFont("Menlo", SMALL_PT)
        painter.setFont(sensor_font)
        pending = self.threat.get("status") == "PENDING"
        sensors_room = rect.width() - text_left - (ACTIONS_WIDTH if pending else 8)
        painter.drawText(
            QRectF(text_left, rect.top() + 38, max(sensors_room, 0.0), 13),
            Qt.AlignLeft | Qt.AlignVCenter,
            QFontMetricsF(sensor_font).elidedText(
                sensor_list(self.threat.get("sensors", [])), Qt.ElideRight, max(sensors_room, 0.0)
            ),
        )
        painter.end()


class ThreatPanel(QWidget):
    threatSelected = Signal(object)
    confirmRequested = Signal(object)
    rejectRequested = Signal(object)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._threats: dict[tuple, dict] = {}
        self._cards: dict[tuple, ThreatCard] = {}
        self._selected_key: tuple | None = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.header = QLabel("THREATS  0")
        self.header.setObjectName("sectionLabel")
        layout.addWidget(self.header)

        self.empty_label = QLabel("No detections yet.")
        self.empty_label.setObjectName("mutedNote")
        self.empty_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.empty_label)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        self._container = QWidget()
        self._list_layout = QVBoxLayout(self._container)
        self._list_layout.setContentsMargins(0, 0, 0, 0)
        self._list_layout.setSpacing(3)
        self._list_layout.addStretch(1)
        scroll.setWidget(self._container)
        layout.addWidget(scroll, stretch=1)
        self._refresh_header()

    def add_detection(self, detection: dict) -> dict:
        """Fold one detection into the threat at its voxel, creating it if new."""
        key = voxel_key(detection["x"], detection["y"], 0.0)
        threat = self._threats.get(key)

        if threat is None:
            threat = {
                "key": key,
                "x": detection["x"],
                "y": detection["y"],
                "type": detection.get("type", "unknown"),
                "confidence": detection.get("confidence", 0.0),
                "sensors": [],
                "status": "PENDING",
            }
            self._threats[key] = threat
            card = ThreatCard(threat)
            card.clicked.connect(self._on_card_clicked)
            card.confirmRequested.connect(self.confirmRequested.emit)
            card.rejectRequested.connect(self.rejectRequested.emit)
            self._cards[key] = card
            self._list_layout.insertWidget(self._list_layout.count() - 1, card)

        reported = detection.get("sensors") or (
            [detection["sensor"]] if detection.get("sensor") else []
        )
        for sensor in reported:
            if sensor not in threat["sensors"]:
                threat["sensors"].append(sensor)

        # Highest confidence wins, matching the merge rule in core/mesh_state.py.
        if detection.get("confidence", 0.0) > threat["confidence"]:
            threat["confidence"] = detection["confidence"]
            threat["type"] = detection.get("type", threat["type"])

        self._cards[key].threat = threat
        self._cards[key]._sync_actions()
        self._cards[key].update()
        self._refresh_header()
        return threat

    def set_status(self, x: float, y: float, status: str) -> None:
        key = voxel_key(x, y, 0.0)
        threat = self._threats.get(key)
        if threat is None:
            return
        threat["status"] = status
        self._cards[key]._sync_actions()
        self._cards[key].update()
        self._refresh_header()

    def clear(self) -> None:
        for card in self._cards.values():
            self._list_layout.removeWidget(card)
            card.deleteLater()
        self._threats.clear()
        self._cards.clear()
        self._selected_key = None
        self._refresh_header()

    def _on_card_clicked(self, threat: dict) -> None:
        self._selected_key = threat["key"]
        for key, card in self._cards.items():
            card.set_selected(key == self._selected_key)
        self.threatSelected.emit(threat)

    def _refresh_header(self) -> None:
        pending = sum(1 for t in self._threats.values() if t["status"] == "PENDING")
        confirmed = sum(1 for t in self._threats.values() if t["status"] == "CONFIRMED")
        parts = [f"THREATS  {len(self._threats)}"]
        if pending:
            parts.append(f"{pending} PENDING")
        if confirmed:
            parts.append(f"{confirmed} CONFIRMED")
        self.header.setText("   ".join(parts))
        self.empty_label.setVisible(not self._threats)

        # Only claim the height the list actually needs, so an empty threat
        # list does not hold open a blank third of the right column.
        count = len(self._threats)
        self.setMaximumHeight(56 if count == 0 else min(count * (CARD_HEIGHT + 3) + 34, 430))
