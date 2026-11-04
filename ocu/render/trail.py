"""The route a splat scene was filmed along, in metres, and the camera poses along it.

A reconstruction has no units and no idea which way is up: its camera positions are in whatever scale the first pair
of frames happened to fix, and its axes are the first camera's. A rover driving the trail needs both, so they are found here
from the reconstruction itself:

  * Up. The mean of the cameras' downward axes, reversed. The phone was held a little pitched down, so this is tilted a
    few degrees from true up; the error is small against what it is used for (a camera height).
  * Scale. The path's length in model units over the time it took to walk it, at `WALK_PACE_MS`. That is an
    assumption, stated as one, good to perhaps a quarter either way. Under each camera the trail surface (sand-coloured
    points, fitted by a local plane, because a climbing trail has no single ground plane) gives the phone's height
    above the ground in model units; at the chosen scale that comes out as a height in metres, which is reported as a
    check on the pace (a phone is held between about 0.9 and 1.6 m).

The path is the cameras' positions, smoothed so that the camera's walking sway does not become the rover's.

No Qt and no GL import belongs in this file.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy import ndimage

from ocu.render.splats import Pose

#: A reconstruction from video has no scale of its own. Two things anchor it here, and they disagree by about half
#: again, which is the honest uncertainty: how fast someone walks up a trail (this one), and how high they held the phone
#: (reported, not used). A real measurement of anything in the scene replaces both: see `rescale_to`.
WALK_PACE_MS = 1.25


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

    @property
    def length_m(self) -> float:
        return float(self.arc_m[-1])

    def rescale_to(self, metres_per_unit: float) -> None:
        """Replace the scale with a measured one, and the distances along the path with it."""
        factor = metres_per_unit / self.scale
        self.arc_m = self.arc_m * factor
        self.camera_height_m *= factor
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
    for k in range(3):
        ups[:, k] = np.interp(index, index[good], ups[good, k])
    heights = np.interp(index, index[good], heights[good])
    ups /= np.linalg.norm(ups, axis=1, keepdims=True)
    return ups, heights


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
    height_model = float(np.median(heights))
    smooth = ndimage.gaussian_filter1d(centres, smoothing_frames, axis=0, mode="nearest")
    quats = np.array([_quat(p.rotation) for p in poses])
    for i in range(1, len(quats)):
        if quats[i] @ quats[i - 1] < 0:
            quats[i] = -quats[i]
    quats = ndimage.gaussian_filter1d(quats, smoothing_frames, axis=0, mode="nearest")
    quats /= np.linalg.norm(quats, axis=1, keepdims=True)
    units = np.linalg.norm(np.diff(smooth, axis=0), axis=1)
    duration = seconds_per_frame * (len(poses) - 1)
    scale = WALK_PACE_MS * duration / float(units.sum())
    arc = np.concatenate([[0.0], np.cumsum(units * scale)])
    route = TrailRoute(smooth, quats, arc, scale, up, heights, height_model * scale)
    rise = float((smooth[-1] - smooth[0]) @ up) * scale
    report = {
        "scale_m_per_unit": scale,
        "implied_camera_height_m": height_model * scale,
        "camera_height_spread": float(np.std(heights) / height_model),
        "walk_pace_ms": WALK_PACE_MS,
        "path_length_m": route.length_m,
        "net_rise_m": rise,
        "up": up.round(3).tolist(),
    }
    return route, report
