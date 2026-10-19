"""Left panel, fleet roster, section 9.2.

One card per unit, rover plus three aircraft, each showing control mode
(section 4.2), battery, individual link state, and current task. A table
made the operator read four columns of text to answer "is anything wrong";
these cards answer it by color and bar length before any text is read.
Clicking a card selects that unit and updates the context panel and the
map's focus.
"""

from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QFont, QFontMetricsF, QPainter, QPen
from PySide6.QtWidgets import QVBoxLayout, QWidget

from ocu.ui.labels import short_label
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
    THREAT,
    UNIT_TINTS,
)

CARD_HEIGHT = 52

UNIT_COLORS = dict(UNIT_TINTS)

LINK_COLORS = {
    "CONNECTED": FRIENDLY,
    "DEGRADED": CAUTION,
    "LOST": THREAT,
}

#: The lock and its label, which is the one thing on a row that says an
#: operator is flying this unit by hand.
MANUAL = QColor("#D9B257")
SELECTED_ROW = PANEL_RAISED

ROW_ORDER = ["rover", "tricopter-1", "tricopter-2", "tricopter-3"]


def battery_color(pct: float) -> QColor:
    if pct <= 25:
        return QColor("#C04438")
    if pct <= 50:
        return QColor("#C9952F")
    return QColor("#4A9FD4")


class UnitCard(QWidget):
    clicked = Signal(str)

    def __init__(self, unit_id: str, parent=None) -> None:
        super().__init__(parent)
        self.unit_id = unit_id
        self.setFixedHeight(CARD_HEIGHT)
        self.setCursor(Qt.PointingHandCursor)

        self.control_mode = "AUTO"
        self.battery_pct = 100.0
        self.link_state = "CONNECTED"
        self.current_task = "idle"
        self.manual_holder = False
        self.endurance = ""
        self.docked = unit_id != "rover"
        self.selected = False

    def mousePressEvent(self, event) -> None:
        self.clicked.emit(self.unit_id)

    def paintEvent(self, event) -> None:
        """One unit, laid out from the width it has been given.

        Every position here is derived from the card's width, and every string
        is elided to the room it has, in an order that says what matters: the
        call sign and the battery are never lost, the task gives way before
        the endurance does. The earlier version placed things at fixed
        offsets, which is fine at one width and prints text through other text
        at every other, and a console gets resized.
        """
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = QRectF(self.rect())
        accent = UNIT_COLORS.get(self.unit_id, TEXT_DIM)
        pad = 10.0

        painter.fillRect(rect, SELECTED_ROW if self.selected else BACKGROUND)
        # The spine carries identity; the hairline separates rows. A selected
        # row is lifted, not boxed.
        painter.fillRect(QRectF(rect.left(), rect.top(), 2, rect.height()), accent)
        painter.setPen(QPen(FRIENDLY if self.selected else HAIRLINE))
        painter.drawLine(
            QPointF(rect.left(), rect.bottom() - 0.5), QPointF(rect.right(), rect.bottom() - 0.5)
        )

        # --- row one: who, in what mode, and is the link up ---------------
        title_font = QFont(FONT_LABEL, 14, QFont.Bold)
        painter.setFont(title_font)
        painter.setPen(QPen(TEXT))
        sign = short_label(self.unit_id)
        sign_w = QFontMetricsF(title_font).horizontalAdvance(sign)
        painter.drawText(
            QRectF(rect.left() + pad, rect.top() + 5, sign_w + 2, 16),
            Qt.AlignLeft | Qt.AlignVCenter,
            sign,
        )

        link_x = rect.right() - pad - 7
        painter.setPen(Qt.NoPen)
        painter.setBrush(LINK_COLORS.get(self.link_state, INERT))
        painter.drawEllipse(QRectF(link_x, rect.top() + 10, 7, 7))

        mode_font = QFont(FONT_LABEL, 11, QFont.Bold)
        painter.setFont(mode_font)
        painter.setPen(QPen(MANUAL if self.manual_holder else TEXT_DIM))
        mode_left = rect.left() + pad + sign_w + 10
        mode_room = max(link_x - 8 - mode_left, 0.0)
        mode_text = ("\U0001F512 " if self.manual_holder else "") + self.control_mode
        painter.drawText(
            QRectF(mode_left, rect.top() + 5, mode_room, 16),
            Qt.AlignLeft | Qt.AlignVCenter,
            _elide(mode_text, mode_font, mode_room),
        )

        # --- row two: the battery -------------------------------------------
        pct_font = QFont(FONT_LABEL, 12, QFont.Bold)
        pct_text = f"{self.battery_pct:.0f}%"
        pct_w = QFontMetricsF(pct_font).horizontalAdvance("100%") + 2
        bar_rect = QRectF(rect.left() + pad, rect.top() + 26, rect.width() - pad * 2 - pct_w - 6, 4)
        painter.setPen(Qt.NoPen)
        painter.fillRect(bar_rect, HAIRLINE)
        fill = QRectF(bar_rect)
        fill.setWidth(bar_rect.width() * min(max(self.battery_pct / 100.0, 0.0), 1.0))
        painter.fillRect(fill, battery_color(self.battery_pct))
        painter.setFont(pct_font)
        painter.setPen(QPen(battery_color(self.battery_pct)))
        painter.drawText(
            QRectF(rect.right() - pad - pct_w, rect.top() + 20, pct_w, 16),
            Qt.AlignRight | Qt.AlignVCenter,
            pct_text,
        )

        # --- row three: what it is doing, and for how long it can ------------
        small = QFont("Menlo", 8)
        painter.setFont(small)
        metrics = QFontMetricsF(small)
        # Time remaining at the current draw. The percentage alone barely
        # moves on a three hour platform, which tells an operator nothing.
        endurance_w = metrics.horizontalAdvance(self.endurance) + 2 if self.endurance else 0.0
        if self.endurance:
            painter.setPen(QPen(TEXT_DIM))
            painter.drawText(
                QRectF(rect.right() - pad - endurance_w, rect.top() + 34, endurance_w, 14),
                Qt.AlignRight | Qt.AlignVCenter,
                self.endurance,
            )

        task = "IN BAY" if self.docked else self.current_task.upper()
        task_room = max(rect.width() - pad * 2 - endurance_w - 10, 0.0)
        painter.setPen(QPen(TEXT_DIM))
        painter.drawText(
            QRectF(rect.left() + pad, rect.top() + 34, task_room, 14),
            Qt.AlignLeft | Qt.AlignVCenter,
            _elide(task, small, task_room),
        )
        painter.end()


def _elide(text: str, font: QFont, width: float) -> str:
    """Text cut to fit with an ellipsis, never allowed to run on."""
    return QFontMetricsF(font).elidedText(text, Qt.ElideRight, max(width, 0.0))


class FleetRoster(QWidget):
    unitSelected = Signal(str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(3)

        self.cards: dict[str, UnitCard] = {}
        for unit_id in ROW_ORDER:
            card = UnitCard(unit_id)
            card.clicked.connect(self.unitSelected.emit)
            self.cards[unit_id] = card
            layout.addWidget(card)
        layout.addStretch(1)

    def select_unit(self, unit_id: str) -> None:
        for uid, card in self.cards.items():
            card.selected = uid == unit_id
            card.update()

    def set_docked(self, unit_id: str, docked: bool) -> None:
        card = self.cards.get(unit_id)
        if card is not None:
            card.docked = docked
            card.update()

    def update_unit(
        self,
        unit_id: str,
        control_mode: str,
        battery_pct: float,
        link_state: str,
        current_task: str,
        manual_holder: bool = False,
        endurance: str = "",
    ) -> None:
        card = self.cards.get(unit_id)
        if card is None:
            return
        card.control_mode = control_mode
        card.battery_pct = battery_pct
        card.link_state = link_state
        card.current_task = current_task
        card.manual_holder = manual_holder
        card.endurance = endurance
        card.update()
