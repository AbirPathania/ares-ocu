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

from ocu.core.camera import CANOPY_RGB, GROUND_RGB, ROCK_RGB, SCRUB_RGB
from ocu.core.terrain import Terrain

_CAM_GROUND = GROUND_RGB[Terrain.GROUND]
_CAM_TRACK = GROUND_RGB[Terrain.TRACK]
_CAM_SCRUB = SCRUB_RGB
_CAM_CANOPY = CANOPY_RGB
_CAM_ROCK = ROCK_RGB

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

# The map's classification colours are derived from the colours the cameras
# render the same ground in, lifted for a dark display. They used to be
# chosen separately and had drifted a long way apart: open ground was blue on
# the map and tan in the camera, rock cold grey against warm. An operator
# looking from one to the other was being asked to match a green patch to a
# different green, and a tan hillside to a blue one.
#
# Hue and saturation come from the camera; only lightness is raised, because
# a map is a sensor display read at a glance and the camera is a window. One
# class is deliberately not natural: a hole is a hazard rather than terrain,
# and is coloured to be found rather than to be recognised.


def _mapped(rgb: tuple[int, int, int], lift: int) -> QColor:
    """The camera's colour for a class, raised to map brightness."""
    return QColor(*rgb).lighter(lift)


TERRAIN_COLORS = {
    # The dim tone a return starts from before its intensity lifts it.
    "GROUND": _mapped(_CAM_GROUND, 34),
    "TRACK": _mapped(_CAM_TRACK, 58),
    "SCRUB": _mapped(_CAM_SCRUB, 62),
    "TREE": _mapped(_CAM_CANOPY, 72),
    "ROCK": _mapped(_CAM_ROCK, 68),
    "POTHOLE": QColor("#2A1F2C"),
    "BUSH": _mapped(_CAM_SCRUB, 96),
}

TERRAIN_LABELS = {
    "TRACK": "track",
    "GROUND": "open",
    "SCRUB": "scrub",
    "TREE": "tree",
    "ROCK": "rock",
    "POTHOLE": "hole",
    "BUSH": "bush",
}

#: blend from the dim TERRAIN_COLORS tone toward this, weighted by the
#: measured intensity, so a hard rock face reads bright and grazed ground
#: stays dim without either changing hue.
# Hue comes from the camera; brightness is the map's own decision, and it
# ranks by what the operator needs to find. Open ground is most of the view
# and stays quiet; a thing that stops the vehicle is the brightest mark on
# the display. Lifting every class equally would be truer to the camera and
# would turn the map into an even wash with no hierarchy in it at all.
RETURN_COLORS = {
    "GROUND": _mapped(_CAM_GROUND, 72),
    "TRACK": _mapped(_CAM_TRACK, 112),
    "SCRUB": _mapped(_CAM_SCRUB, 150),
    "TREE": _mapped(_CAM_CANOPY, 270),
    "ROCK": _mapped(_CAM_ROCK, 185),
    # Not a terrain colour: a hole is a hazard, and is coloured to be found.
    "POTHOLE": QColor("#B07ABE"),
    "BUSH": _mapped(_CAM_SCRUB, 210),
}
