"""Vector glyphs for threat classes, shared between the map markers and the
alert banner so a mine reads as the same shape everywhere in the app.
No image assets; everything is drawn with QPainter so it never has to be
mistaken for a photograph of anything.
"""

from __future__ import annotations

import math

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QFont, QPainter, QPainterPath, QPolygonF


def glyph_key(item: dict) -> str:
    """The glyph a detection or a listed threat is drawn with. The spec's enum has no animal class, so an animal is an
    `unknown` with the label kept; it still gets its own shape rather than the question mark."""
    threat_type = item.get("type", "unknown")
    if threat_type == "unknown" and str(item.get("label", "")).lower() == "animal":
        return "animal"
    return threat_type


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

    elif threat_type == "weapon":
        # a figure with a slung weapon: the barrel up over the shoulder, the stock down past the hip, so that it
        # reads as a rifle rather than as outstretched arms at the size a map marker is drawn
        painter.drawEllipse(center + QPointF(-r * 0.12, -r * 0.74), r * 0.38, r * 0.38)
        painter.drawLine(center + QPointF(-r * 0.12, -r * 0.34), center + QPointF(-r * 0.12, r * 0.92))
        painter.drawLine(center + QPointF(-r * 0.12, r * 0.30), center + QPointF(-r * 0.72, r * 1.05))
        painter.drawLine(center + QPointF(-r * 0.12, r * 0.30), center + QPointF(r * 0.42, r * 1.05))
        muzzle, stock = center + QPointF(r * 1.12, -r * 1.06), center + QPointF(-r * 0.38, r * 0.72)
        painter.drawLine(stock, muzzle)
        painter.drawLine(stock, stock + QPointF(-r * 0.34, r * 0.16))  # the butt
        painter.drawLine(center + QPointF(r * 0.30, -r * 0.30), center + QPointF(r * 0.18, r * 0.02))  # the magazine

    elif threat_type == "animal":
        # A deer in profile, facing right: body, neck, head with an ear, antlers, four legs, a tail.
        body = QRectF(center.x() - r * 1.2, center.y() - r * 0.35, r * 1.8, r * 0.85)
        painter.drawEllipse(body)
        neck_base = QPointF(center.x() + r * 0.55, center.y() - r * 0.15)
        head = QPointF(center.x() + r * 1.05, center.y() - r * 0.85)
        painter.drawLine(neck_base, head)
        painter.drawEllipse(head, r * 0.24, r * 0.2)
        painter.drawLine(head + QPointF(-r * 0.12, -r * 0.18), head + QPointF(-r * 0.3, -r * 0.5))  # ear
        for dx in (0.0, 0.18):  # antlers: a main beam with a tine
            base = head + QPointF(-r * 0.05 + dx * r, -r * 0.2)
            tip = base + QPointF(r * (0.05 + dx), -r * 0.62)
            painter.drawLine(base, tip)
            painter.drawLine(base + (tip - base) * 0.55, base + (tip - base) * 0.55 + QPointF(r * 0.22, -r * 0.1))
        for dx in (-1.0, -0.65, 0.2, 0.5):
            hip = QPointF(center.x() + dx * r, center.y() + r * 0.45)
            painter.drawLine(hip, hip + QPointF(0, r * 0.75))
        painter.drawLine(QPointF(center.x() - r * 1.2, center.y() - r * 0.1), QPointF(center.x() - r * 1.45, center.y() - r * 0.3))  # tail

    elif threat_type == "disturbed_soil":
        for dx, dy in ((-r, -r * 0.3), (0, r * 0.4), (r, -r * 0.2), (-r * 0.4, r), (r * 0.5, r * 0.9)):
            painter.drawEllipse(center + QPointF(dx, dy), r * 0.28, r * 0.28)

    elif threat_type == "safe_object":
        p1 = center + QPointF(-r * 0.7, 0)
        p2 = center + QPointF(-r * 0.15, r * 0.6)
        p3 = center + QPointF(r * 0.8, -r * 0.6)
        painter.drawLine(p1, p2)
        painter.drawLine(p2, p3)

    else:  # "unknown", or anything not yet classified
        font = QFont("Menlo", int(max(r, 6)))
        font.setBold(True)
        painter.setFont(font)
        painter.drawText(QRectF(center.x() - r, center.y() - r, r * 2, r * 2), Qt.AlignCenter, "?")


def draw_friendly_ground(painter: QPainter, center: QPointF, size: float) -> None:
    """MIL-STD-2525 friend, ground: a rectangle, wider than tall."""
    half_w, half_h = size, size * 0.74
    painter.drawRect(QRectF(center.x() - half_w, center.y() - half_h, half_w * 2, half_h * 2))


def draw_friendly_air(painter: QPainter, center: QPointF, size: float) -> None:
    """MIL-STD-2525 friend, air: an arch, flat along the bottom.

    Air tracks are drawn open at the base so a ground symbol and an air
    symbol are still distinguishable when they overlap, which on this map
    they routinely do while an aircraft is over the rover.
    """
    half_w, half_h = size * 0.92, size * 0.9
    path = QPainterPath()
    path.moveTo(center.x() - half_w, center.y() + half_h)
    path.lineTo(center.x() - half_w, center.y())
    path.arcTo(
        QRectF(center.x() - half_w, center.y() - half_h, half_w * 2, half_h * 2), 180.0, -180.0
    )
    path.lineTo(center.x() + half_w, center.y() + half_h)
    painter.drawPath(path)


def draw_unknown_contact(painter: QPainter, center: QPointF, size: float) -> None:
    """MIL-STD-2525 unknown: a quatrefoil, four lobes on the diagonals."""
    path = QPainterPath()
    lobe = size * 0.56
    offset = size * 0.46
    for dx, dy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        path.addEllipse(QPointF(center.x() + dx * offset, center.y() + dy * offset), lobe, lobe)
    painter.drawPath(path.simplified())


def draw_neutral(painter: QPainter, center: QPointF, size: float) -> None:
    """MIL-STD-2525 neutral: a square. A contact the operator has not chosen to treat as a threat."""
    painter.drawRect(QRectF(center.x() - size, center.y() - size, size * 2, size * 2))


def draw_hostile(painter: QPainter, center: QPointF, size: float) -> None:
    """MIL-STD-2525 hostile: a diamond standing on its point."""
    painter.drawPolygon(
        QPolygonF(
            [
                QPointF(center.x(), center.y() - size),
                QPointF(center.x() + size, center.y()),
                QPointF(center.x(), center.y() + size),
                QPointF(center.x() - size, center.y()),
            ]
        )
    )
