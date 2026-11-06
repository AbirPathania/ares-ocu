"""The route a splat scene was filmed along, in metres, and the camera poses along it.

A reconstruction has no units and no idea which way is up: its camera positions are in whatever scale the first pair
of frames happened to fix, and its axes are the first camera's. A rover driving the trail needs both, so they are found here
from the reconstruction itself:

  * Up. The mean of the cameras' downward axes, reversed. The phone was held a little pitched down, so this is tilted a
    few degrees from true up; the error is small against what it is used for (a camera height).
  * Scale. Under each camera the points below it give the phone's height above the ground in model units
    (`lowest_surface_depth`), and the phone is taken to be held at `REFERENCE_CAMERA_HEIGHT_M`, which fixes the scale. The
    older anchor, the path's length over the time it took to walk it at `WALK_PACE_MS`, is the fallback when too few frames
    have ground under them, and its implied pace is reported either way: it is the number that says whether the height
    can be believed (a walker's pace is between about 0.5 and 1.8 m/s). Pace alone gave phone heights of 1.7-2.0 m on four
    IMG_5108 segments, which cannot be held by hand; heights give paces of 0.74-1.5 m/s, which are walks.

The path is the cameras' positions, smoothed so that the camera's walking sway does not become the rover's.

No Qt and no GL import belongs in this file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy import ndimage

from ocu.render.splats import Intrinsics, Pose

#: A reconstruction from video has no scale of its own. Two things anchor it here, and they disagree by about half
#: again, which is the honest uncertainty: how fast someone walks up a trail (this one), and how high they held the phone
#: (reported, not used). A real measurement of anything in the scene replaces both: see `rescale_to`.
WALK_PACE_MS = 1.25

#: How high the phone was held, measured on the two walks whose ground is certain (IMG_5129 and IMG_5132: sand under almost every camera, the height
#: varying by under 10% along the walk): 1.163 m and 1.154 m at the pace above, which they both agree with. Another walk by the same person with the
#: same phone is taken to hold it at the same height, which is a better anchor than the pace: the pace is the one thing that varies along a hike, and
#: implied phone heights of 1.7-2.0 m (what pace gave for four of the IMG_5108 segments) cannot be held by hand.
REFERENCE_CAMERA_HEIGHT_M = 1.16
#: The lowest-surface depth (`lowest_surface_depth`) reads this much deeper than the plane through the sand on those two walks (1.21 and 1.22 m against
#: 1.16 and 1.15 m): leaves and grass stand above the ground the cameras saw the least of.
LOWEST_SURFACE_BIAS = 1.05
#: A walk with half or fewer of its cameras having enough points under them to say how high they were keeps the pace as its scale (the height is
#: the median over the walk, so up to half of it can be filled in from neighbours).
MIN_GROUND_FRACTION = 0.5
#: Percentile of the depths of the points under a camera taken as the ground: not the deepest (floaters and mistakes lie below it) and not the median
#: (grass and bushes lie above it).
GROUND_PERCENTILE = 75.0


TRAIL_DIR = Path(__file__).resolve().parent.parent.parent / "assets" / "footage" / "trail"


def read_points(path: str | Path) -> tuple[np.ndarray, np.ndarray]:
    """XYZ and RGB (0..255) of every point of a COLMAP text model's points3D.txt."""
    xyz, rgb = [], []
    with open(path) as handle:
        for line in handle:
            if line.startswith("#") or not line.strip():
                continue
            v = line.split(None, 7)
            xyz.append((float(v[1]), float(v[2]), float(v[3])))
            rgb.append((int(v[4]), int(v[5]), int(v[6])))
    return np.array(xyz), np.array(rgb, dtype=np.float32)


def sandy(rgb: np.ndarray) -> np.ndarray:
    """Points coloured like the trail's sand: warm, light and not very saturated (grass is yellower or greener, shade is darker)."""
    r, g, b = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    hi, lo = rgb.max(axis=1), rgb.min(axis=1)
    sat = (hi - lo) / np.maximum(hi, 1.0)
    return (r > g) & (g > b) & (sat > 0.18) & (sat < 0.50) & (hi > 120) & ((g - b) > 0.35 * (r - b))


def _quat(rot: np.ndarray) -> np.ndarray:
    m = rot
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        q = np.array([0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s])
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = np.array([(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s])
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = np.array([(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s])
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = np.array([(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s])
    return q / np.linalg.norm(q)


def _rot(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


@dataclass
class TrailRoute:
    """Camera poses along a path, with the scene's scale and up direction."""

    centres: np.ndarray  # (n, 3) smoothed camera positions, model units
    quats: np.ndarray  # (n, 4) smoothed camera orientations (world to camera), w x y z
    arc_m: np.ndarray  # (n,) distance along the path in metres, starting at 0
    scale: float  # metres per model unit
    up: np.ndarray  # unit vector, model frame
    ground_height: np.ndarray  # (n,) the height of each camera above the ground under it, model units
    camera_height_m: float
    #: (n,) how far each camera was from the smoothed path, metres: a camera pose that the reconstruction put far from where its neighbours say the walk went is a wrong one, and the
    #: splats trained from it are smeared (None for a route made before this was recorded)
    deviation_m: np.ndarray | None = None
    #: (n,) how sharp the splats draw the walk's own view at each camera, from that camera and a little to either side of it (the least of the three), by the variance of the picture's
    #: Laplacian: a stretch where it collapses is smeared, dark or streaked whatever its poses say (None for a route packed without it)
    sharpness: np.ndarray | None = None

    @property
    def length_m(self) -> float:
        return float(self.arc_m[-1])

    def save(self, path: str | Path, k: Intrinsics) -> None:
        np.savez_compressed(
            path, centres=self.centres, quats=self.quats, arc_m=self.arc_m, scale=self.scale, up=self.up,
            ground_height=self.ground_height, camera_height_m=self.camera_height_m, k=np.array([k.width, k.height, k.fx, k.fy, k.cx, k.cy]),
            **({"deviation_m": self.deviation_m} if self.deviation_m is not None else {}),
            **({"sharpness": self.sharpness} if self.sharpness is not None else {}),
        )

    @classmethod
    def load(cls, path: str | Path) -> tuple["TrailRoute", Intrinsics]:
        with np.load(path, allow_pickle=False) as d:
            route = cls(d["centres"], d["quats"], d["arc_m"], float(d["scale"]), d["up"], d["ground_height"], float(d["camera_height_m"]),
                        d["deviation_m"] if "deviation_m" in d.files else None, d["sharpness"] if "sharpness" in d.files else None)
            w, h, fx, fy, cx, cy = d["k"]
        return route, Intrinsics(int(w), int(h), float(fx), float(fy), float(cx), float(cy))

    def rescale_to(self, metres_per_unit: float) -> None:
        """Replace the scale with a measured one, and the distances along the path with it."""
        factor = metres_per_unit / self.scale
        self.arc_m = self.arc_m * factor
        self.camera_height_m *= factor
        if self.deviation_m is not None:
            self.deviation_m = self.deviation_m * factor
        self.scale = metres_per_unit

    def pose_at(self, s_m: float, lateral_m: float = 0.0, yaw_deg: float = 0.0, height_m: float | None = None) -> Pose:
        """The camera at `s_m` metres along the path, moved sideways (positive to the right) and turned about up."""
        s = float(np.clip(s_m, 0.0, self.arc_m[-1]))
        i = int(np.clip(np.searchsorted(self.arc_m, s) - 1, 0, len(self.arc_m) - 2))
        f = (s - self.arc_m[i]) / max(self.arc_m[i + 1] - self.arc_m[i], 1e-9)
        centre = self.centres[i] * (1 - f) + self.centres[i + 1] * f
        q0, q1 = self.quats[i], self.quats[i + 1]
        if float(q0 @ q1) < 0:
            q1 = -q1
        q = q0 * (1 - f) + q1 * f
        rot = _rot(q)
        if yaw_deg:
            a = np.radians(yaw_deg)
            k = self.up
            kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
            turn = np.eye(3) + np.sin(a) * kx + (1 - np.cos(a)) * (kx @ kx)  # a turn about up, in the world
            rot = rot @ turn.T
        if lateral_m:
            right = rot[0]  # the camera's x axis in the world
            right = right - (right @ self.up) * self.up
            right /= max(np.linalg.norm(right), 1e-9)
            centre = centre + right * (lateral_m / self.scale)
        if height_m is not None:
            above = self.ground_height[i] * (1 - f) + self.ground_height[i + 1] * f
            centre = centre + self.up * (height_m / self.scale - above)
        return Pose(rot, -rot @ centre)

def local_ground(centres: np.ndarray, points: np.ndarray, rgb: np.ndarray, up0: np.ndarray, radius: float) -> tuple[np.ndarray, np.ndarray]:
    """Under each camera: the up direction of the trail surface there, and the camera's height above it (model units).

    The sand-coloured points within `radius` (measured across the approximate vertical) and below the camera are the trail surface
    there; a plane through them (the smallest principal axis of the points) gives the local up and the distance."""
    sand = points[sandy(rgb)]
    along = sand @ up0
    ups, heights = np.full((len(centres), 3), np.nan), np.full(len(centres), np.nan)
    for i, c in enumerate(centres):
        d = sand - c
        vertical = along - c @ up0
        horizontal = np.linalg.norm(d - np.outer(vertical, up0), axis=1)
        near = sand[(horizontal < radius) & (vertical < 0.0)]
        if len(near) < 40:
            continue
        centroid = near.mean(axis=0)
        _, _, vt = np.linalg.svd(near - centroid, full_matrices=False)
        normal = vt[2] if vt[2] @ up0 > 0 else -vt[2]
        ups[i], heights[i] = normal, float((c - centroid) @ normal)
    good = np.isfinite(heights) & (heights > 0)
    index = np.arange(len(centres))
    if not good.any():  # no sand under any camera: up stays the cameras' own, and the height is not known
        return np.tile(up0, (len(centres), 1)), heights
    for k in range(3):
        ups[:, k] = np.interp(index, index[good], ups[good, k])
    heights = np.interp(index, index[good], heights[good])
    ups /= np.linalg.norm(ups, axis=1, keepdims=True)
    return ups, heights


def lowest_surface_depth(centres: np.ndarray, points: np.ndarray, up: np.ndarray, radius: float) -> tuple[np.ndarray, float]:
    """The camera's height above the ground at every frame, in model units, from the points under it whatever their colour, and the fraction of frames it could be said for.

    The colour filter of `local_ground` needs sand, and under trees there is leaf litter, grass and shade instead (its estimate along one IMG_5108 segment varied
    by 150% of itself). Here every point within `radius` (measured across `up`) and below the camera counts, and the ground is the `GROUND_PERCENTILE`th of
    their depths. Frames with too few points are filled in from their neighbours."""
    along = points @ up
    depth = np.full(len(centres), np.nan)
    for i, c in enumerate(centres):
        d = points - c
        vertical = along - c @ up
        horizontal = np.linalg.norm(d - np.outer(vertical, up), axis=1)
        below = -vertical[(horizontal < radius) & (vertical < 0.0)]
        if len(below) < 60:
            continue
        below = below[below <= np.percentile(below, 98.0)]
        depth[i] = np.percentile(below, GROUND_PERCENTILE)
    good = np.isfinite(depth)
    if not good.any():
        return depth, 0.0
    index = np.arange(len(centres))
    return np.interp(index, index[good], depth[good]), float(good.mean())


#: A net grade steeper than this over a whole walk is taken to be the camera's pitch mistaken for slope, not the trail: with no gravity sensor,
#: the way a phone is tilted while walking uphill (or down) cannot be told from the slope itself. Beyond it the walk is levelled.
MAX_GRADE = 0.25


def level_if_implausible(centres: np.ndarray, up: np.ndarray) -> tuple[np.ndarray, float, bool]:
    """Return (up, grade, reliable): `up` turned about the walk's own cross axis so that the walk's net rise is zero, if its grade is too steep to be believed."""
    d = centres[-1] - centres[0]
    along = float(d @ up)
    perp = d - along * up
    flat = float(np.linalg.norm(perp))
    grade = along / flat if flat > 1e-9 else 0.0
    if abs(grade) <= MAX_GRADE or flat <= 1e-9:
        return up, grade, True
    theta = math.atan2(-along, flat)  # tan(theta) = -(d . up) / |d_perp| makes d . up' = 0
    w = perp / flat
    levelled = math.cos(theta) * up + math.sin(theta) * w
    return levelled / np.linalg.norm(levelled), grade, False


def build_route(poses: list[Pose], points: np.ndarray, rgb: np.ndarray, seconds_per_frame: float, smoothing_frames: float = 4.0) -> tuple[TrailRoute, dict]:
    """Find up and scale, smooth the path, and say how far the assumptions can be trusted."""
    centres = np.array([p.centre for p in poses])
    down_axes = np.array([p.rotation.T @ np.array([0.0, 1.0, 0.0]) for p in poses])
    up0 = -down_axes.mean(axis=0)
    up0 /= np.linalg.norm(up0)
    spacing = float(np.median(np.linalg.norm(np.diff(centres, axis=0), axis=1)))
    ups, heights = local_ground(centres, points, rgb, up0, radius=6.0 * spacing)
    heights = ndimage.gaussian_filter1d(heights, 3.0, mode="nearest")
    up = ups.mean(axis=0)
    up /= np.linalg.norm(up)
    up, estimated_grade, grade_reliable = level_if_implausible(centres, up)
    sand_height_model = float(np.median(heights))
    sand_spread = float(np.std(heights) / sand_height_model)
    depth, ground_fraction = lowest_surface_depth(centres, points, up, radius=6.0 * spacing)
    by_height = ground_fraction >= MIN_GROUND_FRACTION
    if by_height:
        heights = ndimage.gaussian_filter1d(depth, 3.0, mode="nearest") / LOWEST_SURFACE_BIAS  # what the carved ground is built from, too
    height_model = float(np.median(heights))
    smooth =ndimage.gaussian_filter1d(centres, smoothing_frames, axis=0, mode="nearest")
    quats = np.array([_quat(p.rotation) for p in poses])
    for i in range(1, len(quats)):
        if quats[i] @ quats[i - 1] < 0:
            quats[i] = -quats[i]
    quats = ndimage.gaussian_filter1d(quats, smoothing_frames, axis=0, mode="nearest")
    quats /= np.linalg.norm(quats, axis=1, keepdims=True)
    units = np.linalg.norm(np.diff(smooth, axis=0), axis=1)
    duration = seconds_per_frame * (len(poses) - 1)
    pace_scale = WALK_PACE_MS * duration / float(units.sum())
    scale = REFERENCE_CAMERA_HEIGHT_M / height_model if by_height else pace_scale
    arc = np.concatenate([[0.0], np.cumsum(units * scale)])
    deviation = np.linalg.norm(centres - smooth, axis=1) * scale
    route = TrailRoute(smooth, quats, arc, scale, up, heights, height_model * scale, deviation)
    rise = float((smooth[-1] - smooth[0]) @ up) * scale
    report = {
        "scale_m_per_unit": scale,
        "scale_from": "camera height" if by_height else "walking pace",
        "implied_camera_height_m": height_model * scale,
        "camera_height_spread": float(np.std(heights) / height_model),
        "sand_height_m_at_pace": sand_height_model * pace_scale,
        "sand_height_spread": sand_spread,
        "ground_fraction": ground_fraction,
        "walk_pace_ms": WALK_PACE_MS,
        "implied_pace_ms": WALK_PACE_MS * scale / pace_scale,
        "path_length_m": route.length_m,
        "net_rise_m": rise,
        "up": up.round(3).tolist(),
        "grade_estimated": estimated_grade,
        "grade_reliable": grade_reliable,
    }
    return route, report
