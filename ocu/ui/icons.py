"""Vector glyphs for threat classes, shared between the map markers and the
alert banner so a mine reads as the same shape everywhere in the app.
No image assets; everything is drawn with QPainter so it never has to be
mistaken for a photograph of anything.
"""

from __future__ import annotations

import math

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QFont, QPainter


def draw_threat_glyph(painter: QPainter, center: QPointF, radius: float, threat_type: str) -> None:
    """Draws inside a circle of the given radius, centered on center. Caller
    sets the pen/brush color; this only lays out the shape.
    """
    r = radius * 0.55

    if threat_type == "mine":
        painter.drawEllipse(center, r, r)
        for i in range(8):
            angle = math.radians(i * 45)
            p1 = center + QPointF(r * math.cos(angle), r * math.sin(angle))
            p2 = center + QPointF(r * 1.6 * math.cos(angle), r * 1.6 * math.sin(angle))
            painter.drawLine(p1, p2)

    elif threat_type == "ied":
        poly = [
            center + QPointF(0, -r * 1.3),
            center + QPointF(r * 1.3, 0),
            center + QPointF(0, r * 1.3),
            center + QPointF(-r * 1.3, 0),
        ]
        painter.drawPolygon(poly)

    elif threat_type == "person":
        painter.drawEllipse(center + QPointF(0, -r * 0.6), r * 0.45, r * 0.45)
        painter.drawLine(center + QPointF(0, -r * 0.15), center + QPointF(0, r * 1.0))
        painter.drawLine(center + QPointF(0, r * 0.2), center + QPointF(-r * 0.7, r * 1.0))
        painter.drawLine(center + QPointF(0, r * 0.2), center + QPointF(r * 0.7, r * 1.0))

    elif threat_type == "vehicle":
        rect = QRectF(center.x() - r * 1.3, center.y() - r * 0.8, r * 2.6, r * 1.6)
        painter.drawRoundedRect(rect, 2, 2)
        painter.drawLine(rect.topLeft() + QPointF(0, r * 0.5), rect.topRight() + QPointF(0, r * 0.5))

    elif threat_type == "disturbed_soil":
        for dx, dy in ((-r, -r * 0.3), (0, r * 0.4), (r, -r * 0.2), (-r * 0.4, r), (r * 0.5, r * 0.9)):
            painter.drawEllipse(center + QPointF(dx, dy), r * 0.28, r * 0.28)

    elif threat_type == "safe_object":
        p1 = center + QPointF(-r * 0.7, 0)
        p2 = center + QPointF(-r * 0.15, r * 0.6)
        p3 = center + QPointF(r * 0.8, -r * 0.6)
        painter.drawLine(p1, p2)
        painter.drawLine(p2, p3)

    else:  # "unknown", "TBD", or anything not yet classified
        font = QFont("Menlo", int(max(r, 6)))
        font.setBold(True)
        painter.setFont(font)
        painter.drawText(QRectF(center.x() - r, center.y() - r, r * 2, r * 2), Qt.AlignCenter, "?")
