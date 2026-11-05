"""The simulated world, built round a trail that was reconstructed from the footage.

A reconstruction (render/trail.py) gives the real trail's centreline, in metres, with up and the ground's height under it.
This puts that line into the simulation's world: the deploy point sits on it, it runs north, and everything that follows
from the line is derived from it and not placed by hand:

  * The ground is carved to match. Along the centreline the terrain takes the trail's real rise and fall; a strip about three
    metres wide becomes track, a verge on either side becomes scrub, and the trees, boulders, bushes, pebbles and holes the
    procedural world had put there are taken away, so that what the rover's LiDAR and planner meet is what the
    camera shows.
  * The sweep follows the line. Its waypoints are along the centreline, and `TrailSweep.at` places buried objects and
    people relative to the line, which is how the same scenario code that laid things out across four straight lanes lays them
    out along a bend.
  * The rover's camera is the splat camera. Where the rover is, across and along the trail, and which way it faces,
    choose where on the real walk the picture is drawn from (`camera_state`), so a rover that drives the carved track sees what
    the phone saw there.

The reconstruction only covers what was walked and a little to either side, and only facing the way it was walked, so
`camera_state` says when the rover is outside that and the caller draws the simulated world instead.

Several trails are chained into one patrol by `TrailWorld`: each leg starts where the one before it ended, so the carved path is
continuous, and the rover's camera cuts from one real place to the next as it crosses from one leg into the next.

No Qt and no GL import belongs in this file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

from ocu.core.sweep import LaneSweep
from ocu.core.terrain import Terrain, TerrainWorld
from ocu.render import trail as trail_assets
from ocu.render.splats import Intrinsics

SAMPLE_STEP_M = 0.25
#: Length over which the carved ground's height is smoothed along the path.
GROUND_SMOOTH_M = 3.0
#: Half the width of the track, and how far past it the verge of scrub goes and the ground blends back to the world's own.
HALF_TRACK_M = 1.6
VERGE_M = 2.5
BLEND_M = 7.0
#: Clear of trees, boulders and bushes this much past the verge (their own radius is added).
CLEAR_MARGIN_M = 1.0
#: The reconstruction supports a camera this far to either side of the walk and turned this far from its direction. A rover that is
#: further off or turned further than that still gets the real place, drawn from the nearest camera the walk supports (a picture
#: that is a little wrong about where the rover is, and real, rather than a cut to the simulated world), until it is more than
#: `FALL_BACK_M` from the path.
MAX_LATERAL_M = 1.0
MAX_YAW_DEG = 30.0
FALL_BACK_M = 3.5
#: The first stretch of the walk was seen by few frames and is the poorest; the deploy point is this far along it.
DEFAULT_START_M = 10.0
#: The walk's last metres have the filmer's own shadow in them, and are where the camera slowed to stop; the patrol ends this short of them.
END_MARGIN_M = 9.0
#: Across the trail, in metres, per lane and per metre of offset in the scenario code written for straight lanes four metres apart.
LANE_LATERAL_M = 0.7
OFFSET_SCALE = 0.5


def carve_world(world: TerrainWorld, xy: np.ndarray, ground_z: np.ndarray, tree: cKDTree, base_z: float | None = None) -> float:
    """Cut a centreline (world positions and the ground's height along it) into the world; returns the world's own height at the deploy point."""
    extent, cell, span = world._extent, world.cell_size_m, world._span
    if base_z is None:
        i0 = world._index(world.to_cell(0.0, 0.0))
        base_z = float(world._elevation[i0]) if i0 else 0.0
    axis = (np.arange(span) - extent + 0.5) * cell
    gx, gy = np.meshgrid(axis, axis, indexing="ij")
    d, nearest = tree.query(np.stack([gx.ravel(), gy.ravel()], axis=1))
    d, nearest = d.reshape(gx.shape), nearest.reshape(gx.shape)
    # elevation follows the walk's ground along the line and blends back to the world's own past the verge
    t = np.clip(1.0 - (d - HALF_TRACK_M) / BLEND_M, 0.0, 1.0)
    w = t * t * (3.0 - 2.0 * t)
    world._elevation = (1.0 - w) * world._elevation + w * (base_z + ground_z[nearest])
    near = d <= HALF_TRACK_M + VERGE_M
    world._rock_height[near] = 0.0
    world._pothole_depth[near] = 0.0
    world._grid[near] = int(Terrain.SCRUB)
    world._grid[d <= HALF_TRACK_M] = int(Terrain.TRACK)
    keep_clear = HALF_TRACK_M + VERGE_M + CLEAR_MARGIN_M
    for name, radius_col in (("boulders", 2), ("trees", 3), ("bushes", 2)):
        rows = getattr(world, name)
        if len(rows):
            dist, _ = tree.query(rows[:, :2])
            setattr(world, name, rows[dist > keep_clear + rows[:, radius_col]])
    if len(world.pebbles):
        dist, _ = tree.query(world.pebbles[:, :2])
        world.pebbles = world.pebbles[dist > HALF_TRACK_M + 0.3]
    return base_z


@dataclass(frozen=True)
class TrailState:
    s_m: float  # along the reconstructed walk, from its start
    lateral_m: float  # to the right of the centreline, positive
    yaw_deg: float  # how far the rover is turned from the direction of the walk, positive to the right (clockwise)
    on_trail: bool
    leg: int = 0  # which trail of a chained patrol (always 0 for a single trail)


@dataclass(frozen=True)
class TrailCamera:
    """Where the splat camera is in the simulated world, so that everything else is drawn from the same place."""

    eye: tuple[float, float, float]
    heading_deg: float
    pitch_down_deg: float


@dataclass(frozen=True)
class TrailSweep(LaneSweep):
    """A sweep along the trail's centreline. `at` is the scenario's way of placing things, so it follows the bend."""

    scenario: "TrailScenario" = field(default=None, repr=False)

    def at(self, lane_index: int, along_m: float, offset_m: float = 0.0) -> tuple[float, float]:
        return self.scenario.point(along_m, lane_index * LANE_LATERAL_M + offset_m * OFFSET_SCALE)


@dataclass
class TrailScenario:
    name: str
    route: trail_assets.TrailRoute
    intrinsics: Intrinsics
    start_m: float
    rotation: np.ndarray  # (3, 3): model axes to world, rows are east, north (the way the trail starts) and up
    origin_model: np.ndarray  # the model point that is the deploy point
    arc: np.ndarray  # (n,) metres along the walk, every SAMPLE_STEP_M
    xy: np.ndarray  # (n, 2) world positions of the centreline
    ground_z: np.ndarray  # (n,) ground height relative to the deploy point's ground
    tangent: np.ndarray  # (n, 2) unit direction of travel
    start_camera_height_m: float = 0.0  # how high above the ground the walk's camera was at the deploy point
    offset: np.ndarray = field(default_factory=lambda: np.zeros(3))  # where the deploy point is in the world (x, y, ground height)
    base_z: float = 0.0  # the world's own elevation at the deploy point, set when the ground is carved
    _tree: cKDTree = field(default=None, repr=False)

    # --- building -----------------------------------------------------------------------------------------------------

    @classmethod
    def build(cls, name: str, route: trail_assets.TrailRoute, intrinsics: Intrinsics, start_m: float = DEFAULT_START_M,
              placement: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)) -> "TrailScenario":
        """`placement` is where the deploy point goes in the world and which way the trail leaves it: x, y, ground height, compass bearing."""
        start_m = min(start_m, 0.25 * route.length_m)
        arc = np.arange(0.0, route.length_m, SAMPLE_STEP_M)
        # dense centreline in the model's frame, and the height of the camera above the ground under it
        dense = np.stack([np.interp(arc, route.arc_m, route.centres[:, k]) for k in range(3)], axis=1)
        gh = np.interp(arc, route.arc_m, route.ground_height)
        up = route.up / np.linalg.norm(route.up)
        i0 = int(np.searchsorted(arc, start_m))
        ahead = dense[min(i0 + int(4.0 / SAMPLE_STEP_M), len(arc) - 1)] - dense[i0]
        forward = ahead - (ahead @ up) * up
        forward /= np.linalg.norm(forward)
        east = np.cross(forward, up)  # east x north = up, so the world is right-handed with z up
        rotation = np.stack([east, forward, up])
        origin = dense[i0].copy()
        world = ((dense - origin) * route.scale) @ rotation.T
        ground = world[:, 2] - gh * route.scale
        # the height of the ground under each camera is estimated frame by frame and is noisy (a quarter of the camera's height, on some
        # trails); a trail does not jump a half metre in half a metre, so the ground along it is smoothed before it is carved
        ground = ndimage.gaussian_filter1d(ground, GROUND_SMOOTH_M / SAMPLE_STEP_M, mode="nearest")
        ground -= ground[i0]
        x0, y0, z0, bearing = placement
        c, sn = math.cos(math.radians(bearing)), math.sin(math.radians(bearing))
        turn = np.array([[c, sn], [-sn, c]])  # a clockwise turn by the bearing: north becomes the bearing
        rotation = np.array([[c, sn, 0.0], [-sn, c, 0.0], [0.0, 0.0, 1.0]]) @ rotation
        xy = world[:, :2] @ turn.T + np.array([x0, y0])
        ground = ground + z0
        step = np.gradient(xy, axis=0)
        tangent = step / np.maximum(np.linalg.norm(step, axis=1, keepdims=True), 1e-9)
        scenario = cls(name, route, intrinsics, start_m, rotation, origin, arc, xy, ground, tangent, start_camera_height_m=float(gh[i0] * route.scale),
                       offset=np.array([x0, y0, z0]))
        scenario._tree = cKDTree(xy)
        return scenario

    @classmethod
    def load(cls, name: str | None = None, start_m: float = DEFAULT_START_M) -> "TrailScenario | None":
        """The named trail from assets/footage/trail (the first one if no name), or None if there is none."""
        folder = trail_assets.TRAIL_DIR
        candidates = [folder / f"{name}.npz"] if name else sorted(folder.glob("*.npz"))
        for path in candidates:
            if path.exists():
                try:
                    route, k = trail_assets.TrailRoute.load(path)
                except (OSError, ValueError, KeyError):
                    continue
                return cls.build(path.stem, route, k, start_m)
        return None

    # --- the line in the world -------------------------------------------------------------------------------------------

    @property
    def length_m(self) -> float:
        return float(self.arc[-1])

    @property
    def start_index(self) -> int:
        return int(np.searchsorted(self.arc, self.start_m))

    def patrol_range(self) -> tuple[int, int]:
        """Indices of the samples the patrol uses: from the deploy point to `END_MARGIN_M` short of the end of the walk."""
        return self.start_index, max(int(np.searchsorted(self.arc, self.length_m - END_MARGIN_M)), self.start_index + 2)

    @property
    def bearing_deg(self) -> float:
        t = self.tangent[self.start_index]
        return math.degrees(math.atan2(t[0], t[1])) % 360.0

    def point(self, along_m: float, lateral_m: float = 0.0) -> tuple[float, float]:
        """A world point `along_m` past the deploy point and `lateral_m` to the right of the line."""
        s = float(np.clip(self.start_m + along_m, 0.0, self.length_m))
        x, y = (float(np.interp(s, self.arc, self.xy[:, k])) for k in range(2))
        tx, ty = (float(np.interp(s, self.arc, self.tangent[:, k])) for k in range(2))
        norm = math.hypot(tx, ty) or 1.0
        return x + (ty / norm) * lateral_m, y - (tx / norm) * lateral_m

    def sweep(self, spacing_m: float = 3.0) -> TrailSweep:
        """Waypoints along the line, from the deploy point to `END_MARGIN_M` short of the end of the walk."""
        stations = np.arange(spacing_m, self.length_m - self.start_m - END_MARGIN_M, spacing_m)
        points = [(*self.point(float(a)), "LANE 1") for a in stations]
        return TrailSweep(origin=(0.0, 0.0), bearing_deg=self.bearing_deg, clear_fraction=1.0, points=points, scenario=self)

    # --- the ground ---------------------------------------------------------------------------------------------------------

    def carve(self, world: TerrainWorld) -> None:
        """Cut the trail into the world: its elevation, a track and a verge, and nothing standing on them."""
        self.base_z = carve_world(world, self.xy, self.ground_z, self._tree)

    # --- the camera -----------------------------------------------------------------------------------------------------------

    def camera_state(self, x: float, y: float, heading_deg: float) -> TrailState:
        """Where a rover at (x, y) facing `heading_deg` is on the walk, and whether the walk can be drawn from there.

        The lateral offset and the turn are limited to what the walk supports (MAX_LATERAL_M, MAX_YAW_DEG): further than that the real place is
        still drawn, from the nearest supported camera, until the rover is more than FALL_BACK_M from the path."""
        dist, i = self._tree.query([x, y])
        i = int(i)
        tx, ty = self.tangent[i]
        right = np.array([ty, -tx])
        lateral = float((np.array([x, y]) - self.xy[i]) @ right)
        bearing = math.degrees(math.atan2(tx, ty))
        yaw = (heading_deg - bearing + 180.0) % 360.0 - 180.0
        s = float(self.arc[i])
        ok = float(dist) <= FALL_BACK_M and 0.5 < s < self.length_m - 0.5
        return TrailState(s, float(np.clip(lateral, -MAX_LATERAL_M, MAX_LATERAL_M)), float(np.clip(yaw, -MAX_YAW_DEG, MAX_YAW_DEG)), ok)

    def camera(self, state: TrailState) -> TrailCamera:
        """The splat camera for a state, as a place and direction in the simulated world."""
        pose = self.route.pose_at(state.s_m, state.lateral_m, -state.yaw_deg)  # the route's turn is anticlockwise, a heading clockwise
        eye_model = pose.centre
        local = ((eye_model - self.origin_model) * self.route.scale) @ self.rotation.T + self.offset
        forward = self.rotation @ pose.rotation[2]
        heading = math.degrees(math.atan2(forward[0], forward[1])) % 360.0
        pitch_down = math.degrees(-math.asin(float(np.clip(forward[2], -1.0, 1.0))))
        # in the walk's frame the camera at the deploy point is at height zero, and the ground there is `start_camera_height_m` below it
        return TrailCamera((float(local[0]), float(local[1]), float(self.base_z + local[2] + self.start_camera_height_m)), heading, pitch_down)


#: People and animals stand at least this far from the path: outside the carved track and verge (4.1 m), where the rover can pass and the
#: planner is not asked to squeeze by someone on a three-metre track.
KEEP_OFF_M = 4.6

#: Chained legs: a leg may turn this far from the way the one before it ended, and the path steers back toward the middle of the map
#: once it is this far from it, so that a long patrol stays inside the world. Far out, where the world's edge is near, it may turn
#: more at a join (the cut between two real places hides the corner).
TURN_MAX_DEG = 35.0
FAR_TURN_MAX_DEG = 80.0
WORLD_RADIUS_M = 45.0
FAR_RADIUS_M = 70.0


class TrailWorld:
    """Several trails chained into one patrol: leg n starts where leg n-1 ended, and the carved path is continuous.

    What the rover's camera shows changes from one real place to the next as it crosses into the next leg (a cut, as between shots),
    but what the LiDAR, the planner and the map see is one path. Only the patrol part of each leg is used (from its deploy point to
    `END_MARGIN_M` short of its end), so the poorly reconstructed head of a walk and the filmer's shadow at its tail are never driven.
    """

    def __init__(self, legs: list[TrailScenario]) -> None:
        self.legs = legs
        xy, ground, tangent, leg_of, local_arc = [], [], [], [], []
        for index, leg in enumerate(legs):
            i0, i1 = leg.patrol_range()
            xy.append(leg.xy[i0:i1])
            ground.append(leg.ground_z[i0:i1])
            tangent.append(leg.tangent[i0:i1])
            leg_of.append(np.full(i1 - i0, index))
            local_arc.append(leg.arc[i0:i1])
        self.xy, self.ground_z, self.tangent = np.concatenate(xy), np.concatenate(ground), np.concatenate(tangent)
        self.leg_of, self.local_arc = np.concatenate(leg_of), np.concatenate(local_arc)
        steps = np.linalg.norm(np.diff(self.xy, axis=0), axis=1)
        steps[np.diff(self.leg_of) != 0] = 0.0  # a join is a cut, not distance
        self.arc = np.concatenate([[0.0], np.cumsum(steps)])
        self._tree = cKDTree(self.xy)
        self.base_z = 0.0

    @classmethod
    def build(cls, trails: list[tuple[str, trail_assets.TrailRoute, Intrinsics]], start_m: float = DEFAULT_START_M) -> "TrailWorld":
        legs: list[TrailScenario] = []
        placement = (0.0, 0.0, 0.0, 0.0)
        for name, route, k in trails:
            leg = TrailScenario.build(name, route, k, start_m, placement)
            legs.append(leg)
            i0, i1 = leg.patrol_range()
            end = leg.xy[i1 - 1]
            heading = math.degrees(math.atan2(*leg.tangent[i1 - 1])) % 360.0
            turn = 0.0
            reach = math.hypot(*end)
            if reach > WORLD_RADIUS_M:
                toward_middle = math.degrees(math.atan2(-end[0], -end[1])) % 360.0
                limit = FAR_TURN_MAX_DEG if reach > FAR_RADIUS_M else TURN_MAX_DEG
                turn = float(np.clip((toward_middle - heading + 180.0) % 360.0 - 180.0, -limit, limit))
            placement = (float(end[0]), float(end[1]), float(leg.ground_z[i1 - 1]), heading + turn)
        return cls(legs)

    @classmethod
    def load(cls, names: list[str] | str | None = None, start_m: float = DEFAULT_START_M) -> "TrailWorld | None":
        """The named trails (or every packed trail) from assets/footage/trail, chained in the order given (sorted by name if all)."""
        folder = trail_assets.TRAIL_DIR
        if isinstance(names, str):
            names = [n for n in names.split(",") if n]
        paths = [folder / f"{n}.npz" for n in names] if names else sorted(folder.glob("*.npz"))
        trails = []
        for path in paths:
            if not path.exists():
                continue
            try:
                route, k = trail_assets.TrailRoute.load(path)
            except (OSError, ValueError, KeyError):
                continue
            trails.append((path.stem, route, k))
        return cls.build(trails, start_m) if trails else None

    # --- the same interface as one trail ------------------------------------------------------------------------------

    @property
    def name(self) -> str:
        return self.legs[0].name

    @property
    def names(self) -> list[str]:
        return [leg.name for leg in self.legs]

    @property
    def length_m(self) -> float:
        return float(self.arc[-1])

    @property
    def start_m(self) -> float:
        return 0.0

    @property
    def bearing_deg(self) -> float:
        return self.legs[0].bearing_deg

    def point(self, along_m: float, lateral_m: float = 0.0) -> tuple[float, float]:
        """A world point `along_m` along the patrol (a join counts for nothing) and `lateral_m` to the right of it."""
        s = float(np.clip(along_m, 0.0, self.length_m))
        x, y = (float(np.interp(s, self.arc, self.xy[:, k])) for k in range(2))
        tx, ty = (float(np.interp(s, self.arc, self.tangent[:, k])) for k in range(2))
        norm = math.hypot(tx, ty) or 1.0
        return x + (ty / norm) * lateral_m, y - (tx / norm) * lateral_m

    def sweep(self, spacing_m: float = 3.0) -> TrailSweep:
        points = []
        for a in np.arange(spacing_m, self.length_m - 2.0, spacing_m):
            leg = int(self.leg_of[min(int(np.searchsorted(self.arc, a)), len(self.arc) - 1)])
            points.append((*self.point(float(a)), f"LEG {leg + 1}"))
        return TrailSweep(origin=(0.0, 0.0), bearing_deg=self.bearing_deg, clear_fraction=1.0, points=points, scenario=self)

    def carve(self, world: TerrainWorld) -> None:
        self.base_z = carve_world(world, self.xy, self.ground_z, self._tree)
        for leg in self.legs:
            leg.base_z = self.base_z

    def camera_state(self, x: float, y: float, heading_deg: float) -> TrailState:
        dist, i = self._tree.query([x, y])
        i = int(i)
        tx, ty = self.tangent[i]
        right = np.array([ty, -tx])
        lateral = float((np.array([x, y]) - self.xy[i]) @ right)
        yaw = (heading_deg - math.degrees(math.atan2(tx, ty)) + 180.0) % 360.0 - 180.0
        ok = float(dist) <= FALL_BACK_M
        return TrailState(float(self.local_arc[i]), float(np.clip(lateral, -MAX_LATERAL_M, MAX_LATERAL_M)), float(np.clip(yaw, -MAX_YAW_DEG, MAX_YAW_DEG)), ok, int(self.leg_of[i]))

    def camera(self, state: TrailState) -> TrailCamera:
        return self.legs[state.leg].camera(state)

    def distance_to_line(self, x: float, y: float) -> float:
        """How far (x, y) is from the nearest part of the chained path, metres."""
        return float(self._tree.query([x, y])[0])

    def keep_off_line(self, x: float, y: float, side: float = 0.0, min_m: float = KEEP_OFF_M) -> tuple[float, float]:
        """The point nearest (x, y) that is at least `min_m` from the path, moved out along the path's normal. `side` (+1 right, -1 left) says
        which way if (x, y) is too close to tell; 0 keeps the side it is on."""
        dist, i = self._tree.query([x, y])
        if dist >= min_m:
            return x, y
        tx, ty = self.tangent[int(i)]
        right = np.array([ty, -tx])
        offset = float((np.array([x, y]) - self.xy[int(i)]) @ right)
        direction = side if side else (1.0 if offset >= 0.0 else -1.0)
        # Straight out to `min_m` is clear of the nearest part of the line but can land on another part of it where the chain runs back close beside itself
        # (two arms 5 m apart), so go further out, then try the other side, until the point is clear of all of it.
        for extra in np.arange(0.0, 12.0, 0.5):
            for d in (direction, -direction):
                out = self.xy[int(i)] + right * d * (min_m + extra)
                if self._tree.query(out)[0] >= min_m - 1e-6:
                    return float(out[0]), float(out[1])
        out = self.xy[int(i)] + right * direction * min_m
        return float(out[0]), float(out[1])

    def keep_path_off_line(self, path, side: float = 1.0, min_m: float = KEEP_OFF_M, step_m: float = 1.5) -> tuple[tuple[float, float], ...]:
        """A walking path with every point of it, not only its corners, at least `min_m` from the line. Where the line bends back on itself the
        straight run between two corners that are each clear of it can still cross it, so the run is cut into short steps first."""
        pts = [np.asarray(p[:2], dtype=float) for p in path]
        dense = [pts[0]]
        for a, b in zip(pts[:-1], pts[1:]):
            n = max(1, int(math.ceil(float(np.linalg.norm(b - a)) / step_m)))
            dense.extend(a + (b - a) * (k / n) for k in range(1, n + 1))
        return tuple(self.keep_off_line(float(p[0]), float(p[1]), side=side, min_m=min_m) for p in dense)

    def leg_sweep(self, index: int) -> TrailSweep:
        """The sweep of one leg alone, relative to that leg's own deploy point: what the scenario code places things by."""
        return self.legs[index].sweep()
