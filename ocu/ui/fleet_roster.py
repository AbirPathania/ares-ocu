"""Left panel, fleet roster, section 9.2.

One card per unit, rover plus three aircraft, each showing control mode
(section 4.2), battery, individual link state, and current task. A table
made the operator read four columns of text to answer "is anything wrong";
these cards answer it by color and bar length before any text is read.
Clicking a card selects that unit and updates the context panel and the
map's focus.
"""

from __future__ import annotations

from PySide6.QtCore import QRectF, Qt, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QVBoxLayout, QWidget

from ocu.ui.launch_console import short_label

CARD_HEIGHT = 54

UNIT_COLORS = {
    "rover": QColor("#5ec9c2"),
    "tricopter-1": QColor("#e8b23c"),
    "tricopter-2": QColor("#c97ee8"),
    "tricopter-3": QColor("#7ea8e8"),
}

LINK_COLORS = {
    "CONNECTED": QColor("#5ec9c2"),
    "DEGRADED": QColor("#e8b23c"),
    "LOST": QColor("#e05c4b"),
}

ROW_ORDER = ["rover", "tricopter-1", "tricopter-2", "tricopter-3"]


def battery_color(pct: float) -> QColor:
    if pct <= 25:
        return QColor("#e05c4b")
    if pct <= 50:
        return QColor("#e8b23c")
    return QColor("#5ec9c2")


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
        self.docked = unit_id != "rover"
        self.selected = False

    def mousePressEvent(self, event) -> None:
        self.clicked.emit(self.unit_id)

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = QRectF(self.rect()).adjusted(0, 0, -1, -1)
        accent = UNIT_COLORS.get(self.unit_id, QColor("#cccccc"))

        painter.fillRect(rect, QColor("#1d262b") if self.selected else QColor("#15191d"))
        painter.setPen(QPen(QColor("#5ec9c2") if self.selected else QColor("#222c33")))
        painter.drawRect(rect)
        painter.fillRect(QRectF(rect.left(), rect.top(), 3, rect.height()), accent)

        painter.setPen(QPen(QColor("#e6ecea")))
        painter.setFont(QFont("Menlo", 11, QFont.Bold))
        painter.drawText(
            QRectF(rect.left() + 10, rect.top() + 5, 90, 15),
            Qt.AlignLeft | Qt.AlignVCenter,
            short_label(self.unit_id),
        )

        mode_text = ("\U0001F512 " if self.manual_holder else "") + self.control_mode
        painter.setPen(QPen(QColor("#f4d78a") if self.manual_holder else QColor("#7c8891")))
        painter.setFont(QFont("Menlo", 8, QFont.Bold))
        painter.drawText(
            QRectF(rect.left() + 100, rect.top() + 5, 96, 15), Qt.AlignLeft | Qt.AlignVCenter, mode_text
        )

        link_color = LINK_COLORS.get(self.link_state, QColor("#666666"))
        painter.setPen(Qt.NoPen)
        painter.setBrush(link_color)
        painter.drawEllipse(QRectF(rect.right() - 16, rect.top() + 9, 7, 7))

        bar_rect = QRectF(rect.left() + 10, rect.top() + 25, rect.width() - 62, 5)
        painter.setPen(Qt.NoPen)
        painter.fillRect(bar_rect, QColor("#222c33"))
        fill = QRectF(bar_rect)
        fill.setWidth(bar_rect.width() * min(max(self.battery_pct / 100.0, 0.0), 1.0))
        painter.fillRect(fill, battery_color(self.battery_pct))

        painter.setPen(QPen(battery_color(self.battery_pct)))
        painter.setFont(QFont("Menlo", 8, QFont.Bold))
        painter.drawText(
            QRectF(rect.right() - 46, rect.top() + 21, 40, 14),
            Qt.AlignRight | Qt.AlignVCenter,
            f"{self.battery_pct:.0f}%",
        )

        task = "IN BAY" if self.docked else self.current_task.upper()
        painter.setPen(QPen(QColor("#8fa39e") if self.docked else QColor("#7c8891")))
        painter.setFont(QFont("Menlo", 8))
        painter.drawText(
            QRectF(rect.left() + 10, rect.top() + 36, rect.width() - 20, 14),
            Qt.AlignLeft | Qt.AlignVCenter,
            task,
        )
        painter.end()


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
    ) -> None:
        card = self.cards.get(unit_id)
        if card is None:
            return
        card.control_mode = control_mode
        card.battery_pct = battery_pct
        card.link_state = link_state
        card.current_task = current_task
        card.manual_holder = manual_holder
        card.update()
