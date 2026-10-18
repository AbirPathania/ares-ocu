"""One palette and type scale for the whole interface.

The look this replaces was a dark dashboard: saturated cyan and purple on
near-black, rounded corners, soft glows, decorative colour everywhere. It
reads as a product demo. Ground control software does not look like that,
for reasons that are operational rather than aesthetic:

- **Colour carries meaning or it is not used.** An operator scanning a
  display under stress needs red to mean threat and amber to mean caution
  everywhere, every time. If the chrome is also coloured, the eye has to
  work out which colour is information and which is decoration. So the
  interface is near-greyscale and the status palette is small and fixed.
- **Desaturated, not neon.** These are stared at for a whole shift. The
  status colours here are muted deliberately; a saturated red is harder to
  look at and no more legible than a dull one.
- **Square corners, hairline rules.** Panels are divided, not floated.
  Rounded cards and drop shadows imply a document; a console implies a
  machine.
- **DIN for labels, monospace for numbers.** DIN is the German industrial
  standard typeface, drawn for signage and machinery and legible at small
  sizes and bad angles. Numbers sit in a monospace so columns of telemetry
  line up and a changing digit does not reflow the row.

Unit and threat symbols follow MIL-STD-2525 framing: friendly units are
blue, a ground unit is a rectangle and an air unit is an arch; an unknown
contact is a yellow quatrefoil; a confirmed hostile is a red diamond.
Anyone who has worked near a tactical display will read them without a
legend, and it costs nothing to be correct about it.
"""

from __future__ import annotations

from PySide6.QtGui import QColor

# --- spacing --------------------------------------------------------------
#
# A four pixel baseline. Every gap in the interface is a multiple of one
# number, so the rhythm of the layout is decided once rather than argued out
# again in each panel. Panels that each chose their own padding are what make
# an interface look assembled instead of designed.

UNIT = 4
GUTTER = UNIT  # between items in a stack
PANEL_PAD = UNIT * 2  # inside a panel's edge
SECTION_GAP = UNIT * 3  # between one group of controls and the next

# --- surfaces -------------------------------------------------------------

BACKGROUND = QColor("#0A0E12")
PANEL = QColor("#121820")
PANEL_RAISED = QColor("#182029")
PANEL_SUNKEN = QColor("#0C1116")
HAIRLINE = QColor("#283340")
HAIRLINE_STRONG = QColor("#3A4756")

# --- text -----------------------------------------------------------------

TEXT = QColor("#C6D0DA")
TEXT_DIM = QColor("#7B8794")
TEXT_FAINT = QColor("#4F5B68")
TEXT_BRIGHT = QColor("#E8EDF2")

# --- status. These mean something; nothing else is coloured. --------------

FRIENDLY = QColor("#4A9FD4")  # own forces, MIL-STD-2525 friend
NOMINAL = QColor("#4F9D69")  # within limits
CAUTION = QColor("#C9952F")  # degraded, unresolved, needs attention
THREAT = QColor("#C04438")  # hostile, confirmed, lost
UNKNOWN = QColor("#C2B23F")  # unidentified contact, 2525 unknown
INERT = QColor("#55616E")  # offline, stowed, not applicable

# Per-unit tints stay inside the friendly blue family: these identify which
# aircraft, they do not indicate status, so they must not compete with it.
UNIT_TINTS = {
    "rover": QColor("#5FB3E0"),
    "tricopter-1": QColor("#8FC9E8"),
    "tricopter-2": QColor("#6E9FC4"),
    "tricopter-3": QColor("#A9D4EC"),
}

SELECTION = QColor("#7FC4EC")
GRID = QColor(120, 140, 160, 22)
GRID_MAJOR = QColor(120, 140, 160, 40)
TERRAIN_CONTOUR = QColor(120, 140, 160, 26)

# --- type -----------------------------------------------------------------

FONT_LABEL = "DIN Condensed"  # headers, labels, chips
FONT_DATA = "Menlo"  # numbers, telemetry, logs
FONT_BODY = "Helvetica Neue"  # prose, notes


def unit_color(unit_id: str) -> QColor:
    return UNIT_TINTS.get(unit_id, TEXT_DIM)


def link_color(state: str) -> QColor:
    return {"CONNECTED": NOMINAL, "DEGRADED": CAUTION, "LOST": THREAT}.get(state, INERT)


def battery_color(pct: float) -> QColor:
    if pct <= 25:
        return THREAT
    if pct <= 50:
        return CAUTION
    return NOMINAL


def confidence_color(confidence: float, threshold: float) -> QColor:
    if confidence >= threshold:
        return THREAT
    if confidence >= 0.60:
        return CAUTION
    return UNKNOWN


def hex_of(color: QColor) -> str:
    return color.name()


# --- terrain basemap ------------------------------------------------------
# Deliberately desaturated and dark. This layer is underneath everything
# that carries status, and must never compete with red, amber or blue. It
# answers "what is the ground", not "what should I worry about".

TERRAIN_COLORS = {
    # Open ground sits barely above the background: it reads as "swept" and
    # nothing more. Anything a driver has to care about is lifted out of it.
    "GROUND": QColor("#141A22"),
    "TRACK": QColor("#4A4639"),   # the easiest going, so the brightest
    "SCRUB": QColor("#1E2E1C"),   # low vegetation, passable
    "TREE": QColor("#2C4A2E"),    # canopy: impassable, and it blocks the beam
    "ROCK": QColor("#66727E"),    # impassable, and it blocks the beam
    "POTHOLE": QColor("#2A1F2C"),  # a hole: darker than anything around it
}

TERRAIN_LABELS = {
    "TRACK": "track",
    "GROUND": "open",
    "SCRUB": "scrub",
    "TREE": "tree",
    "ROCK": "rock",
    "POTHOLE": "hole",
}


#: The colour of a strong return off each surface. A painted point is a
#: blend from the dim TERRAIN_COLORS tone toward this, weighted by the
#: measured intensity, so a hard rock face reads bright and grazed ground
#: stays dim without either changing hue.
RETURN_COLORS = {
    "GROUND": QColor("#5E7489"),
    "TRACK": QColor("#A79A74"),
    "SCRUB": QColor("#6E9B5A"),
    "TREE": QColor("#7FD08C"),
    "ROCK": QColor("#D5DEE6"),   # cold pale grey: nothing else on the map is
    "POTHOLE": QColor("#9B6BA8"),  # a hole reads as a hazard, not as terrain
}
