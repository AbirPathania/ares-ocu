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
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QLabel, QPushButton, QScrollArea, QVBoxLayout, QWidget

from ocu.core.constants import ALERT_CONFIDENCE_THRESHOLD
from ocu.core.mesh_state import voxel_key
from ocu.ui.icons import draw_threat_glyph
from ocu.ui.labels import sensor_list

CARD_HEIGHT = 64

STATUS_COLORS = {
    "PENDING": QColor("#C9952F"),
    "CONFIRMED": QColor("#C04438"),
    "CLEARED": QColor("#4F5B68"),
}

def confidence_color(confidence: float) -> QColor:
    if confidence >= ALERT_CONFIDENCE_THRESHOLD:
        return QColor("#C04438")
    if confidence >= 0.60:
        return QColor("#C9952F")
    return QColor("#4A9FD4")


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

    def _sync_actions(self) -> None:
        pending = self.threat.get("status") == "PENDING"
        for button in (self.confirm_btn, self.reject_btn):
            button.setVisible(pending)
        if pending:
            self.confirm_btn.setGeometry(self.width() - 132, CARD_HEIGHT - 24, 62, 18)
            self.reject_btn.setGeometry(self.width() - 66, CARD_HEIGHT - 24, 58, 18)

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
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = QRectF(self.rect()).adjusted(0, 0, -1, -1)

        status = self.threat.get("status", "PENDING")
        status_color = STATUS_COLORS.get(status, QColor("#4F5B68"))
        confidence = self.threat.get("confidence", 0.0)

        painter.fillRect(rect, QColor("#121820") if not self._selected else QColor("#182029"))
        painter.setPen(QPen(QColor("#283340")))
        painter.drawRect(rect)
        painter.fillRect(QRectF(rect.left(), rect.top(), 3, rect.height()), status_color)

        glyph_center = QPointF(rect.left() + 26, rect.top() + 24)
        painter.setPen(QPen(status_color, 1.6))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(glyph_center, 13, 13)
        draw_threat_glyph(painter, glyph_center, 13, self.threat.get("type", "unknown"))

        painter.setPen(QPen(QColor("#C6D0DA")))
        painter.setFont(QFont("DIN Condensed", 11 + 3, QFont.Bold))
        painter.drawText(
            QRectF(rect.left() + 46, rect.top() + 6, rect.width() - 110, 16),
            Qt.AlignLeft | Qt.AlignVCenter,
            f'{str(self.threat.get("type", "unknown")).upper()}'
            f'   {self.threat["x"]:.0f}, {self.threat["y"]:.0f} m',
        )

        painter.setPen(QPen(status_color))
        painter.setFont(QFont("DIN Condensed", 8 + 3, QFont.Bold))
        painter.drawText(
            QRectF(rect.right() - 92, rect.top() + 6, 86, 16), Qt.AlignRight | Qt.AlignVCenter, status
        )

        bar_rect = QRectF(rect.left() + 46, rect.top() + 26, rect.width() - 140, 5)
        painter.setPen(Qt.NoPen)
        painter.fillRect(bar_rect, QColor("#283340"))
        fill = QRectF(bar_rect)
        fill.setWidth(bar_rect.width() * min(max(confidence, 0.0), 1.0))
        painter.fillRect(fill, confidence_color(confidence))

        threshold_x = bar_rect.left() + bar_rect.width() * ALERT_CONFIDENCE_THRESHOLD
        painter.setPen(QPen(QColor("#8c9a9f"), 1, Qt.DotLine))
        painter.drawLine(QPointF(threshold_x, bar_rect.top() - 2), QPointF(threshold_x, bar_rect.bottom() + 2))

        painter.setPen(QPen(QColor("#C6D0DA")))
        painter.setFont(QFont("DIN Condensed", 9 + 3, QFont.Bold))
        painter.drawText(
            QRectF(rect.right() - 92, rect.top() + 22, 86, 14),
            Qt.AlignRight | Qt.AlignVCenter,
            f"{confidence:.2f}",
        )

        sensors = sensor_list(self.threat.get("sensors", []))
        painter.setPen(QPen(QColor("#7B8794")))
        painter.setFont(QFont("Menlo", 8))
        painter.drawText(
            QRectF(rect.left() + 46, rect.top() + 40, rect.width() - 190, 14),
            Qt.AlignLeft | Qt.AlignVCenter,
            sensors,
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
