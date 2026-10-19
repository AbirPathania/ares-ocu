"""Alert and confirm workflow, section 8.3.

Any voxel whose fused confidence crosses 0.85 escalates as an alert,
carrying a type classification and its map coordinates. This is a real
human-in-the-loop decision point in the actual ARES doctrine, not an
interface embellishment, and it belongs in the demo exactly as documented:
an alert banner, the classification and coordinates, two buttons, and a
visible countdown.

The timeout itself is not a number the technical baseline specifies
anywhere ("a configurable window", section 8.3); DEFAULT_ALERT_CONFIRM_TIMEOUT_S
in constants.py is this build's default, not a sourced figure.
"""

from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, QTimer, Qt, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QGraphicsOpacityEffect,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ocu.core.constants import ALERT_CONFIDENCE_THRESHOLD, DEFAULT_ALERT_CONFIRM_TIMEOUT_S
from ocu.ui.icons import draw_threat_glyph
from ocu.ui.motion import Reveal

TICK_MS = 100
BADGE_SIZE = 44


class _ThreatBadge(QWidget):
    """Icon + confidence ring, drawn once per alert."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setFixedSize(BADGE_SIZE, BADGE_SIZE)
        self._threat_type = "unknown"
        self._confidence = 0.0

    def set_detection(self, threat_type: str, confidence: float) -> None:
        self._threat_type = threat_type
        self._confidence = confidence
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        center = QPointF(BADGE_SIZE / 2, BADGE_SIZE / 2)
        radius = BADGE_SIZE / 2 - 4

        track = QPen(QColor("#3a2222"))
        track.setWidth(4)
        painter.setPen(track)
        painter.drawEllipse(center, radius, radius)

        ring = QPen(QColor("#C04438"))
        ring.setWidth(4)
        ring.setCapStyle(Qt.RoundCap)
        painter.setPen(ring)
        span = int(-360 * 16 * self._confidence)
        painter.drawArc(QRectF(center.x() - radius, center.y() - radius, radius * 2, radius * 2), 90 * 16, span)

        painter.setPen(QPen(QColor("#E8EDF2")))
        painter.setBrush(Qt.NoBrush)
        draw_threat_glyph(painter, center, radius * 0.68, self._threat_type)
        painter.end()


class AlertBanner(QWidget):
    confirmed = Signal(dict)
    rejected = Signal(dict)
    timedOut = Signal(dict)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("alertBanner")
        self._detection: dict | None = None
        self._remaining_s = DEFAULT_ALERT_CONFIRM_TIMEOUT_S

        # Arriving and leaving rather than appearing and vanishing. A banner
        # that blinks out between two glances leaves a doubt about whether it
        # was answered or missed. It opens downward from the edge it belongs
        # to, which is the top of the map it is reporting about.
        self._fade = QGraphicsOpacityEffect(self)
        self._fade.setOpacity(0.0)
        self.setGraphicsEffect(self._fade)
        self._reveal = Reveal(self, on_value=self._apply_reveal)

        self._timer = QTimer(self)
        self._timer.setInterval(TICK_MS)
        self._timer.timeout.connect(self._on_tick)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(12, 8, 12, 8)

        self.badge = _ThreatBadge()
        layout.addWidget(self.badge)

        text_col = QVBoxLayout()
        text_col.setSpacing(2)
        self.title_label = QLabel("THREAT ALERT")
        self.title_label.setObjectName("alertTitle")
        text_col.addWidget(self.title_label)
        self.message_label = QLabel()
        self.message_label.setObjectName("alertMessage")
        text_col.addWidget(self.message_label)
        layout.addLayout(text_col, stretch=1)

        self.countdown_label = QLabel()
        self.countdown_label.setObjectName("alertCountdown")
        self.countdown_label.setFixedWidth(48)
        self.countdown_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.countdown_label)

        self.confirm_btn = QPushButton("CONFIRM THREAT")
        self.confirm_btn.setObjectName("confirmButton")
        self.confirm_btn.clicked.connect(self._on_confirm)
        layout.addWidget(self.confirm_btn)

        self.reject_btn = QPushButton("REJECT")
        self.reject_btn.clicked.connect(self._on_reject)
        layout.addWidget(self.reject_btn)

        self.hide()

    def _apply_reveal(self, progress: float) -> None:
        self._fade.setOpacity(progress)
        full = max(self.sizeHint().height(), 1)
        self.setMaximumHeight(max(int(full * progress), 1))

    def show_alert(self, detection: dict) -> None:
        self._detection = detection
        self._remaining_s = DEFAULT_ALERT_CONFIRM_TIMEOUT_S
        threat_type = detection.get("type", "unknown")
        x, y = detection.get("x"), detection.get("y")
        confidence = detection.get("confidence", 0.0)

        # An escalation whose own classification is "inert" is still an
        # escalation, section 8.3, but calling it a threat alert argues with
        # the evidence underneath it and trains the operator to stop reading.
        inert = threat_type == "safe_object"
        self.title_label.setText("CONTACT \u00b7 READS INERT" if inert else "THREAT ALERT")
        self.confirm_btn.setText("CONFIRM THREAT")
        self.reject_btn.setText("CLEAR AS INERT" if inert else "REJECT")
        self.badge.set_detection(threat_type, confidence)
        source = detection.get("source_id", detection.get("source", "unknown"))
        self.message_label.setText(
            f"{threat_type.upper()}   CONF {confidence:.2f}/{ALERT_CONFIDENCE_THRESHOLD:.2f}"
            f"   ({x:.0f}, {y:.0f}) m   SRC {source}"
        )
        self._update_countdown_label()
        self.setMaximumHeight(16777215)  # let it measure itself before revealing
        self._reveal.reveal()
        self._timer.start()

    def dismiss_for(self, detection: dict) -> None:
        """Close the banner if it is counting down on this exact threat.

        The same decision can be answered from the threat list, and a banner
        still ticking on a threat the operator has already resolved would ask
        them to answer it twice.
        """
        current = self._detection
        if current is None:
            return
        if (current.get("x"), current.get("y")) == (detection.get("x"), detection.get("y")):
            self._timer.stop()
            self._detection = None
            self._reveal.dismiss()

    def _on_tick(self) -> None:
        self._remaining_s -= TICK_MS / 1000.0
        self._update_countdown_label()
        if self._remaining_s <= 0:
            self._resolve(self.timedOut)

    def _update_countdown_label(self) -> None:
        self.countdown_label.setText(f"{max(self._remaining_s, 0):.0f}s")

    def _on_confirm(self) -> None:
        self._resolve(self.confirmed)

    def _on_reject(self) -> None:
        self._resolve(self.rejected)

    def _resolve(self, signal: Signal) -> None:
        self._timer.stop()
        detection = self._detection
        self._reveal.dismiss()
        self._detection = None
        if detection is not None:
            signal.emit(detection)
