"""What a trained camera detector would make of the things on the ground, from what has actually been measured.

A scout or the rover's camera sees a person, a vehicle or an animal if it is in the frustum and not behind
something (the same geometry as aerial.py). Whether a trained detector then FINDS it is not geometry. It is a
measured property of one model, and it depends mostly on how tall the object looks in the frame. This file
reads those measurements (data/detector_curves.json, built by the perception repository from a model's
detections on images it never trained on) instead of inventing a curve.

Four things the measurements say, and this file keeps:

  * recall climbs with apparent size and is far from 1 for small things; it is sampled, per frame, so a
    detector sometimes misses what it found a moment ago;
  * from the air, people at the sizes a scout's camera usually produces were not found at all (0 of 505), so
    the aerial person curve is that, and above the size it was tested at the ground curve is used and flagged
    as untested;
  * the detector invents people and animals now and then (about one frame in seventy on rover ground), so
    false alarms happen, one frame at a time here, which is an assumption (real ones can stay on one rock);
  * no figure is a vehicle false-alarm rate, because none is known.

A single detection is not shown to an operator as a threat. A track has to be seen K times in N frames at about
the same place first, which is the cheapest reliable way to turn a flickering detector into a stable report,
and it is how the 0.85 escalation line in the spec stays meaningful.

Everything is deterministic: a draw is seeded from the mission seed, the sensor, the thing and the frame, so
replaying a mission replays what the camera found.

No Qt import belongs in this file.
"""

from __future__ import annotations

import json
import math
import zlib
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path

import numpy as np

CURVES_PATH = Path(__file__).resolve().parent.parent / "data" / "detector_curves.json"
WEAPON_CURVES_PATH = Path(__file__).resolve().parent.parent / "data" / "weapon_curves.json"

#: Frames of detector output per second of mission time. An allocation: the real rate depends on hardware that
#: has not been chosen. The measured model ran at several frames a second on a laptop GPU.
PERCEPTION_HZ = 2.0

#: A track is reported after this many hits within this many consecutive frames, within this radius of its last
#: position. Chosen, not measured; see DECISIONS.md.
CONFIRM_HITS, CONFIRM_WINDOW, GATE_RADIUS_M = 3, 5, 3.5

#: A track survives this many frames without a hit before it is dropped, so that a detector that loses a distant
#: object for a few seconds does not start a second track for it. Chosen, not measured.
TRACK_MEMORY_FRAMES = 10

#: Two confirmed tracks of one kind closer than this are one contact. Wider than the position error of a
#: ground-plane fix at 30 m from a camera 1.6 m up, which is metres; a person who walks further than this from
#: where they were first reported is a new report, because the map marker does not move.
MERGE_RADIUS_M = 8.0

#: Spread added to the measured median confidence of a found object, and the floor and ceiling of what is
#: reported. The spread is invented; the floor is the model's operating threshold, and the ceiling is the
#: same MAX_FUSED_CONFIDENCE rule detection.py uses.
SCORE_SPREAD = 0.06
MAX_CONFIDENCE = 0.97

#: One pixel of localisation error in the box's bottom edge, which fixes where on the ground the object is.
POSITION_SIGMA_PX = 2.0

#: Bearing and range at which an invented detection can appear, relative to the camera.
FALSE_RANGE_M = (8.0, 35.0)

# ---------------------------------------------------------------- the weapon stage (decisions 47 and 48)

#: The weapon model was measured on photographs, where a gun fills a good share of the picture. On a whole rover
#: frame a rifle at 20 m is a fraction of a percent, far below anything measured, so running it on the frame is
#: not something any number supports. It runs instead on the crop around a person the first detector has already
#: found, enlarged, which is the size regime that was measured and how a real system would deploy it.
#:
#: Enlarging cannot invent detail: below this many pixels across in the original frame there is nothing in the
#: crop to enlarge. An allocation, not a measurement.
MIN_WEAPON_PIXELS = 12.0

#: A weapon call needs this many hits in the last CONFIRM_WINDOW frames of a confirmed person track, for the same
#: reason a person needs three: one frame of a classifier is not a report. Chosen, not measured.
WEAPON_HITS = 2


@lru_cache(maxsize=1)
def load_curves(path: str = str(CURVES_PATH)) -> dict:
    return json.loads(Path(path).read_text())


@lru_cache(maxsize=1)
def load_weapon_curves(path: str = str(WEAPON_CURVES_PATH)) -> dict:
    return json.loads(Path(path).read_text())


@dataclass(frozen=True)
class CameraSpec:
    """A pinhole camera. Heading in degrees clockwise from +y, pitch down from the horizon, as the renderer."""

    x: float
    y: float
    z: float  # absolute
    heading_deg: float
    pitch_down_deg: float
    hfov_deg: float
    width_px: int
    height_px: int
    aerial: bool

    @property
    def vfov_deg(self) -> float:
        aspect = self.width_px / self.height_px
        return math.degrees(2.0 * math.atan(math.tan(math.radians(self.hfov_deg) / 2.0) / aspect))

    def axes(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        h, p = math.radians(self.heading_deg), math.radians(self.pitch_down_deg)
        forward = np.array([math.sin(h) * math.cos(p), math.cos(h) * math.cos(p), -math.sin(p)])
        right = np.array([math.cos(h), -math.sin(h), 0.0])
        up = np.cross(right, forward)
        return forward, right, up

    def project(self, points: np.ndarray) -> np.ndarray:
        """(n, 3) world points to (n, 3) of (u, v, depth): u and v in [0, 1] across and down the frame."""
        forward, right, up = self.axes()
        rel = np.asarray(points, dtype=float) - np.array([self.x, self.y, self.z])
        depth = rel @ forward
        safe = np.where(depth > 1e-6, depth, np.nan)
        tx, ty = math.tan(math.radians(self.hfov_deg) / 2.0), math.tan(math.radians(self.vfov_deg) / 2.0)
        u = 0.5 + 0.5 * (rel @ right) / safe / tx
        v = 0.5 - 0.5 * (rel @ up) / safe / ty
        return np.stack([u, v, depth], axis=1)


@dataclass(frozen=True)
class Appearance:
    """How a surface entity looks from a camera, when it is in view and nothing is in the way."""

    box: tuple[float, float, float, float]  # u0, v0, u1, v1, clipped to the frame
    share: float  # height of the box as a share of the frame height
    slant_m: float
    ground_range_m: float


def entity_box_corners(x: float, y: float, ground_z: float, heading_deg: float, height_m: float, width_m: float, length_m: float, offset_m: float = 0.0) -> np.ndarray:
    """The eight corners of the box an entity occupies, facing `heading_deg`."""
    h = math.radians(heading_deg)
    fwd, side = np.array([math.sin(h), math.cos(h)]), np.array([math.cos(h), -math.sin(h)])
    pts = []
    for a in (-0.5, 0.5):
        for b in (-0.5, 0.5):
            xy = np.array([x, y]) + (a * length_m + offset_m) * fwd + b * width_m * side
            pts += [[xy[0], xy[1], ground_z], [xy[0], xy[1], ground_z + height_m]]
    return np.array(pts)


def appearance(cam: CameraSpec, entity, pose: tuple[float, float, float], rig) -> Appearance | None:
    """The entity as the camera sees it, or None if it is out of the frame, too near, or hidden.

    `rig` supplies elevation_at(x, y) and line_of_sight(x, y, z, tx, ty, tz), as camera.CameraRig does.
    """
    ex, ey, heading = pose
    gz = rig.elevation_at(ex, ey)
    uvd = cam.project(entity_box_corners(ex, ey, gz, heading, entity.height_m, entity.width_m, entity.length_m, entity.box_offset_m))
    if np.isnan(uvd).any() or (uvd[:, 2] < 0.5).any():
        return None
    u0, v0, u1, v1 = uvd[:, 0].min(), uvd[:, 1].min(), uvd[:, 0].max(), uvd[:, 1].max()
    if u1 < 0.0 or u0 > 1.0 or v1 < 0.0 or v0 > 1.0:
        return None
    centre = np.array([ex, ey, gz + entity.height_m / 2.0])
    if not rig.line_of_sight(cam.x, cam.y, cam.z, ex, ey, gz + entity.height_m * 0.9):
        return None
    slant = float(np.linalg.norm(centre - np.array([cam.x, cam.y, cam.z])))
    # the share of the frame the object would fill, whether or not part of it is cropped by the frame edge
    return Appearance((max(u0, 0.0), max(v0, 0.0), min(u1, 1.0), min(v1, 1.0)), float(v1 - v0), slant, math.hypot(ex - cam.x, ey - cam.y))


# ---------------------------------------------------------------- the measured curves


def _curve_for(kind: str, aerial: bool) -> tuple[list[dict], dict | None]:
    curves = load_curves()["curves"]
    if kind == "person" and aerial:
        c = curves["person_aerial"]
        return c["bins"], {"above": c["fallback_to_person_above_share"], "bins": curves["person"]["bins"]}
    return curves[kind]["bins"], None


def recall_at(bins: list[dict], share: float) -> float:
    """Measured recall at an apparent size, interpolated in log size between bin centres.

    Below the smallest measured bin recall falls away as the square of the size (an extrapolation, not a
    measurement); above the largest it holds the last measured value."""
    if share <= 0.0:
        return 0.0
    centres = [math.sqrt(max(b["share_lo"], 1e-3) * b["share_hi"]) for b in bins]
    recalls = [b["recall"] for b in bins]
    if share <= centres[0]:
        return float(recalls[0] * (share / centres[0]) ** 2)
    if share >= centres[-1]:
        return float(recalls[-1])
    return float(np.interp(math.log(share), [math.log(c) for c in centres], recalls))


def detection_probability(kind: str, share: float, aerial: bool) -> float:
    bins, fallback = _curve_for(kind, aerial)
    if fallback is not None and share >= fallback["above"]:
        bins = fallback["bins"]
    return recall_at(bins, share)


def median_score(kind: str, share: float) -> float:
    """The measured median confidence of a detection of this size, from the nearest bin that recorded one."""
    bins = load_curves()["curves"][kind if kind in load_curves()["curves"] else "animal"]["bins"]
    best = min((b for b in bins if b.get("median_score_when_found")), key=lambda b: abs(math.log(max(share, 1e-3)) - math.log(math.sqrt(max(b["share_lo"], 1e-3) * b["share_hi"]))))
    return float(best["median_score_when_found"])


def weapon_share(weapon_m: tuple[float, float], entity) -> float:
    """A carried weapon's area as a share of the crop around the person, which is the image the model is given.

    Both are taken at their real sizes rather than from the projection: the crop is the person's box, so the ratio
    does not depend on how far away they are. How far away they are decides whether the crop has any detail left
    (MIN_WEAPON_PIXELS), which is a separate question.
    """
    person_area = max(entity.height_m * entity.width_m, 1e-6)
    return float(weapon_m[0] * weapon_m[1] / person_area)


def weapon_probability(weapon_kind: str, share: float) -> float:
    """Measured recall of the weapon model at that share of the crop (decision 48, arm B)."""
    curves = load_weapon_curves()["curves"]
    if weapon_kind not in curves:
        return 0.0
    return recall_at(curves[weapon_kind]["bins"], share)


def weapon_score(weapon_kind: str, share: float) -> float:
    bins = load_weapon_curves()["curves"][weapon_kind]["bins"]
    best = min((b for b in bins if b.get("median_score_when_found")), key=lambda b: abs(math.log(max(share, 1e-3)) - math.log(math.sqrt(max(b["share_lo"], 1e-3) * b["share_hi"]))))
    return float(best["median_score_when_found"])


# ---------------------------------------------------------------- one sensor, one frame


@dataclass(frozen=True)
class Detection:
    kind: str
    score: float
    x: float  # estimated ground position
    y: float
    box: tuple[float, float, float, float]  # u0, v0, u1, v1
    false_alarm: bool = False  # known to the simulation, never shown to an operator
    truth: str = ""
    weapon: str = ""  # what the second stage called on this person's crop, if it fired


def _rng(seed: int, *parts) -> np.random.Generator:
    return np.random.default_rng(zlib.crc32(repr((seed, *parts)).encode()))


def ground_position(cam: CameraSpec, box_bottom_v: float, u_centre: float, ground_z: float, rng: np.random.Generator) -> tuple[float, float]:
    """Where on the flat ground the bottom of a box is, from its pixel row, with one frame's localisation noise.

    A pixel row is a depression angle; with the camera a known height above the ground, range follows. The
    error grows with the square of range, which is why a distant detection is placed loosely."""
    v = box_bottom_v + rng.normal(0.0, POSITION_SIGMA_PX / cam.height_px)
    u = u_centre + rng.normal(0.0, POSITION_SIGMA_PX / cam.width_px)
    forward, right, up = cam.axes()
    tx, ty = math.tan(math.radians(cam.hfov_deg) / 2.0), math.tan(math.radians(cam.vfov_deg) / 2.0)
    ray = forward + right * (2.0 * (u - 0.5) * tx) + up * (2.0 * (0.5 - v) * ty)
    if ray[2] >= -1e-6:
        return cam.x, cam.y  # at or above the horizon: no ground intersection
    t = (ground_z - cam.z) / ray[2]
    return float(cam.x + ray[0] * t), float(cam.y + ray[1] * t)


class PerceptionSensor:
    """One camera's detector, for one mission seed."""

    def __init__(self, seed: int = 20261016) -> None:
        self.seed = seed
        curves = load_curves()
        self.floor = float(curves["operating_threshold"])
        self.false_per_frame = float(curves["false_alarms"]["person_or_animal_per_frame"])
        self.false_scores = curves.get("false_alarm_scores") or [self.floor + 0.05]
        weapons = load_weapon_curves()
        self.weapon_floor = float(weapons["operating_threshold"])
        self.weapon_false_per_crop = float(weapons["false_alarms"]["per_image"])

    def observe(self, sensor_id: str, cam: CameraSpec, entities, poses, rig, frame: int) -> list[Detection]:
        """Everything this camera reports in one frame: found entities, and now and then an invented one."""
        out: list[Detection] = []
        for entity, pose in zip(entities, poses, strict=True):
            ap = appearance(cam, entity, pose, rig)
            if ap is None:
                continue
            rng = _rng(self.seed, sensor_id, entity.name, frame)
            p = detection_probability(entity.kind, ap.share, cam.aerial)
            if rng.random() >= p:
                continue
            score = min(max(median_score(entity.kind, ap.share) + rng.uniform(-SCORE_SPREAD, SCORE_SPREAD), self.floor), MAX_CONFIDENCE)
            gz = rig.elevation_at(pose[0], pose[1])
            gx, gy = ground_position(cam, ap.box[3], (ap.box[0] + ap.box[2]) / 2.0, gz, rng)
            out.append(Detection(entity.kind, float(score), gx, gy, ap.box, False, entity.name, self._weapon_call(entity, ap, cam, rng)))
        rng = _rng(self.seed, sensor_id, "false", frame)
        if rng.random() < self.false_per_frame:
            out.append(self._invented(cam, rig, rng))
        return out

    def _weapon_call(self, entity, ap: Appearance, cam: CameraSpec, rng: np.random.Generator) -> str:
        """The second stage on this person's crop: what it calls, or "" for nothing.

        Only people are cropped, only when the crop still has detail, and the rate at which it calls a weapon on
        someone carrying nothing is the measured rate on photographs of people holding ordinary objects.
        """
        if entity.kind != "person":
            return ""
        carried = getattr(entity, "carrying", "")
        if carried:
            pixels = entity.weapon_size_m[0] / max(ap.slant_m, 1e-3) / math.radians(cam.hfov_deg) * cam.width_px
            if pixels < MIN_WEAPON_PIXELS:
                return ""
            if rng.random() < weapon_probability(carried, weapon_share(entity.weapon_size_m, entity)):
                return carried
            return ""
        # someone carrying nothing: the measured false rate, and what it calls is the class it confuses most
        if rng.random() < self.weapon_false_per_crop:
            return "long_gun"
        return ""

    def _invented(self, cam: CameraSpec, rig, rng: np.random.Generator) -> Detection:
        r = rng.uniform(*FALSE_RANGE_M)
        bearing = math.radians(cam.heading_deg + rng.uniform(-cam.hfov_deg / 3.0, cam.hfov_deg / 3.0))
        x, y = cam.x + r * math.sin(bearing), cam.y + r * math.cos(bearing)
        kind = "person" if rng.random() < 0.5 else "animal"
        size = 1.0 if kind == "animal" else 1.7
        gz = rig.elevation_at(x, y)
        uvd = cam.project(entity_box_corners(x, y, gz, math.degrees(bearing), size, size * 0.4, size * 0.5))
        box = (float(np.clip(uvd[:, 0].min(), 0, 1)), float(np.clip(uvd[:, 1].min(), 0, 1)), float(np.clip(uvd[:, 0].max(), 0, 1)), float(np.clip(uvd[:, 1].max(), 0, 1)))
        score = float(rng.choice(self.false_scores))
        return Detection(kind, min(max(score, self.floor), MAX_CONFIDENCE), x, y, box, True, "false alarm")


# ---------------------------------------------------------------- persistence


@dataclass
class Track:
    id: int
    kind: str
    sensor_id: str
    x: float
    y: float
    hits: list[int] = field(default_factory=list)  # frame numbers
    scores: list[float] = field(default_factory=list)
    confirmed: bool = False
    box: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    truths: list[str] = field(default_factory=list)
    weapons: list[str] = field(default_factory=list)  # what the second stage called on each hit, "" for nothing

    @property
    def confidence(self) -> float:
        return float(np.mean(self.scores[-CONFIRM_WINDOW:])) if self.scores else 0.0

    @property
    def weapon(self) -> str:
        """The weapon class this track is carrying, once enough recent frames agree. "" until then."""
        recent = [w for w in self.weapons[-CONFIRM_WINDOW:] if w]
        if len(recent) < WEAPON_HITS:
            return ""
        return max(set(recent), key=recent.count)

    @property
    def is_false(self) -> bool:
        return bool(self.truths) and all(t == "false alarm" for t in self.truths)


class ConfirmationGate:
    """Turns a flickering detector into stable reports: K hits in the last N frames near one place.

    Associates each detection with the nearest live track of the same kind from the same sensor inside the
    gate radius, or starts a new one. A track that goes N frames without a hit is dropped."""

    def __init__(self, hits: int = CONFIRM_HITS, window: int = CONFIRM_WINDOW, radius_m: float = GATE_RADIUS_M, memory: int = TRACK_MEMORY_FRAMES) -> None:
        if not 1 <= hits <= window:
            raise ValueError("hits must be between 1 and the window")
        if memory < window:
            raise ValueError("a track cannot be forgotten sooner than the window that confirms it")
        self.hits, self.window, self.radius, self.memory = hits, window, radius_m, memory
        self._tracks: list[Track] = []
        self._next = 1

    @property
    def tracks(self) -> list[Track]:
        return list(self._tracks)

    def update(self, sensor_id: str, frame: int, detections: list[Detection]) -> list[Track]:
        """Feed one frame. Returns the tracks that became confirmed in this frame."""
        newly = []
        taken: set[int] = set()
        for d in sorted(detections, key=lambda d: -d.score):
            best, best_dist = None, self.radius
            for t in self._tracks:
                if t.sensor_id != sensor_id or t.kind != d.kind or t.id in taken:
                    continue
                dist = math.hypot(t.x - d.x, t.y - d.y)
                if dist <= best_dist:
                    best, best_dist = t, dist
            if best is None:
                best = Track(self._next, d.kind, sensor_id, d.x, d.y)
                self._next += 1
                self._tracks.append(best)
            taken.add(best.id)
            best.x, best.y, best.box = d.x, d.y, d.box
            best.hits.append(frame)
            best.scores.append(d.score)
            best.truths.append(d.truth)
            best.weapons.append(d.weapon)
            recent = [h for h in best.hits if h > frame - self.window]
            if not best.confirmed and len(recent) >= self.hits:
                best.confirmed = True
                newly.append(best)
        self._tracks = [t for t in self._tracks if t.hits and t.hits[-1] > frame - self.memory]
        return newly


@dataclass
class Contact:
    """One thing on the ground as the operator knows it, however many tracks have confirmed it."""

    id: int
    kind: str
    x: float  # where it was first reported; the marker on the map stays here
    y: float
    reports: int = 1


class ContactRegistry:
    """Folds confirmed tracks into contacts: a re-confirmation near an existing contact of the same kind is that
    contact, not a new one."""

    def __init__(self, radius_m: float = MERGE_RADIUS_M) -> None:
        self.radius = radius_m
        self._contacts: list[Contact] = []
        self._next = 1

    @property
    def contacts(self) -> list[Contact]:
        return list(self._contacts)

    def resolve(self, kind: str, x: float, y: float) -> Contact:
        near = [c for c in self._contacts if c.kind == kind and math.hypot(c.x - x, c.y - y) <= self.radius]
        if near:
            best = min(near, key=lambda c: math.hypot(c.x - x, c.y - y))
            best.reports += 1
            return best
        contact = Contact(self._next, kind, x, y)
        self._next += 1
        self._contacts.append(contact)
        return contact


#: How a detector's kind maps onto the spec's threat_type enum. There is no animal class, and the eighth class is
#: an open decision (DECISIONS.md), so an animal is reported as `unknown` with its label kept.
THREAT_TYPE = {"person": "person", "vehicle": "vehicle", "animal": "unknown"}

#: A person the weapon stage has called on is reported as `weapon`, not `person`: it is one contact, one marker, and
#: the class an operator filters on is the armed one.
WEAPON_THREAT_TYPE = "weapon"
WEAPON_LABELS = {"long_gun": "armed person \u00b7 long gun", "handgun": "armed person \u00b7 handgun"}
