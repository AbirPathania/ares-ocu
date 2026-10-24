"""The thermal system as the console uses it: the day's field, the camera, the mosaic and the operator's settings.

Qt-free. The field takes seconds to solve (or a moment to load from a stored copy), so the window builds it on a worker
and the system says whether it is ready. Time of day is the operator's: the mission starts at the site's start hour and
the operator can move it, because what infrared shows depends on it more than on anything else.

No Qt import belongs in this file.
"""

from __future__ import annotations

import multiprocessing
import os
import pathlib
import threading

import numpy as np

from ocu.core import thermal_physics as tp
from ocu.core import thermal_scene as ts
from ocu.core.thermal_display import PALETTES, auto_range
from ocu.core.thermal_map import ThermalMap

MAP_BINNING = 2  # the mast camera is read 2 x 2 binned for mapping: cells are a quarter metre, not a pixel
RANGE_HYSTERESIS_K = 1.0  # the legend's range moves only when the ground it describes has moved this much
MIN_RANGE_SPAN_K = 4.0
DEFAULT_PALETTE = "iron"
CACHE_DIR = pathlib.Path.home() / ".cache" / "ares-ocu"


def _solve_and_store(world, targets, site, path: str, progress) -> None:
    """Runs in a separate process: solve the day for a copy of the world and store it. A thread in the console's own process
    starves while the interface and the renderers hold the interpreter; a process has a core and a lock of its own."""
    from ocu.core.camera import CameraRig

    field = tp.build_field(CameraRig(world), targets, site, lambda share: setattr(progress, "value", share))
    target = pathlib.Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.stem + ".part.npz")
    field.save(temporary)
    os.replace(temporary, target)
    progress.value = 1.0


class ThermalSystem:
    def __init__(self, rig, targets=(), site: tp.ThermalSite | None = None, cache_dir=CACHE_DIR) -> None:
        self.rig = rig
        self.targets = list(targets)
        self.site = site or tp.ThermalSite()
        self.cache_dir = pathlib.Path(cache_dir) if cache_dir else None
        self.field: tp.ThermalField | None = None
        self.imager: ts.ThermalImager | None = None
        self.map = ThermalMap(half_extent_m=rig._world.half_extent_m)
        self.palette = DEFAULT_PALETTE
        self.hold_range: tuple[float, float] | None = None  # None: automatic gain
        self.hour_offset_h = 0.0
        self.progress = 0.0
        self._range: tuple[float, float] | None = None
        self.rng = np.random.default_rng(20261023)
        self._process = None
        self._loader: threading.Thread | None = None
        self._shared_progress = None

    # --- the field -----------------------------------------------------------------------------------------

    @property
    def ready(self) -> bool:
        return self.field is not None

    def build(self, progress=None) -> None:
        """Load the day from the stored copy if there is one that matches this world, otherwise solve it and store it."""
        key = tp.cache_key(self.rig._world, self.site, self.targets)
        path = self.cache_dir / f"thermal_{key}.npz" if self.cache_dir else None
        field = None
        if path is not None and path.exists():
            try:
                field = tp.ThermalField.load(path, self.site)
            except (OSError, ValueError, KeyError):
                field = None
        if field is None:
            def report(share):
                self.progress = share
                if progress:
                    progress(share)

            field = tp.build_field(self.rig, self.targets, self.site, report)
            if path is not None:
                try:
                    self.cache_dir.mkdir(parents=True, exist_ok=True)
                    field.save(path)
                except OSError:
                    pass
        self.progress = 1.0
        self.install(field)

    @property
    def cache_path(self) -> pathlib.Path | None:
        if self.cache_dir is None:
            return None
        return self.cache_dir / f"thermal_{tp.cache_key(self.rig._world, self.site, self.targets)}.npz"

    def start_background(self) -> None:
        """Begin getting the day without holding anything up. A stored copy is loaded on a thread; otherwise it is solved in
        a process of its own (or, with no cache directory to hand it back through, on a thread). `poll()` says when it is
        there."""
        if self.ready or self._process is not None or self._loader is not None:
            return
        path = self.cache_path
        if path is not None and path.exists():
            self._loader = threading.Thread(target=self._load_stored, args=(path,), name="thermal-load", daemon=True)
            self._loader.start()
        elif path is not None:
            context = multiprocessing.get_context("spawn")
            self._shared_progress = context.Value("d", 0.0)
            self._process = context.Process(target=_solve_and_store, args=(self.rig._world, self.targets, self.site, str(path), self._shared_progress),
                                            name="thermal-solve", daemon=True)
            self._process.start()
        else:
            self._loader = threading.Thread(target=self.build, name="thermal-build", daemon=True)
            self._loader.start()

    def _load_stored(self, path: pathlib.Path) -> None:
        try:
            self.install(tp.ThermalField.load(path, self.site))
            self.progress = 1.0
        except (OSError, ValueError, KeyError):
            path.unlink(missing_ok=True)  # a damaged copy: let the next start solve it again
            self._loader = None

    def poll(self) -> bool:
        """Advance what is going on in the background; True once the day is ready to use."""
        if self.ready:
            return True
        if self._shared_progress is not None:
            self.progress = float(self._shared_progress.value)
        if self._process is not None and not self._process.is_alive():
            finished = self._process
            self._process = None
            if finished.exitcode == 0 and self.cache_path is not None and self.cache_path.exists():
                self._loader = threading.Thread(target=self._load_stored, args=(self.cache_path,), name="thermal-load", daemon=True)
                self._loader.start()
            else:
                self._shared_progress = None
        return self.ready

    def stop_background(self) -> None:
        """Let go of a solve that is still running (the console is closing)."""
        if self._process is not None and self._process.is_alive():
            self._process.terminate()
        self._process = None

    def install(self, field: tp.ThermalField) -> None:
        self.field = field
        self.imager = ts.ThermalImager(self.rig, field)

    # --- time and display ---------------------------------------------------------------------------------

    def local_hour(self, mission_s: float) -> float:
        return (self.site.start_hour + mission_s / 3600.0 + self.hour_offset_h) % 24.0

    def shift_hours(self, hours: float) -> None:
        self.hour_offset_h += hours

    def cycle_palette(self) -> str:
        self.palette = PALETTES[(PALETTES.index(self.palette) + 1) % len(PALETTES)]
        return self.palette

    def level_range(self, frame_temp: np.ndarray | None = None, valid: np.ndarray | None = None) -> tuple[float, float]:
        """Black and white levels: the operator's if held, otherwise the observed ground (for a feed, this frame)."""
        if self.hold_range is not None:
            return self.hold_range
        if frame_temp is not None:
            return auto_range(frame_temp, valid)
        return self._map_range()

    def _map_range(self) -> tuple[float, float]:
        seen = self.map.observed_range()
        if seen is None:
            return self._range or (0.0, 30.0)
        lo, hi = seen
        if hi - lo < MIN_RANGE_SPAN_K:
            mid = (hi + lo) / 2.0
            lo, hi = mid - MIN_RANGE_SPAN_K / 2.0, mid + MIN_RANGE_SPAN_K / 2.0
        if self._range is None or abs(lo - self._range[0]) > RANGE_HYSTERESIS_K or abs(hi - self._range[1]) > RANGE_HYSTERESIS_K:
            self._range = (lo, hi)
        return self._range

    def toggle_hold(self) -> bool:
        """Hold the current levels, or hand them back to the automatic gain."""
        if self.hold_range is None:
            self.hold_range = self._map_range()
            return True
        self.hold_range = None
        self._range = None
        return False

    # --- looking --------------------------------------------------------------------------------------------

    def _camera(self, unit_id: str, x: float, y: float, z: float, heading: float):
        """(sensor, camera height above sea datum, pitch down) for a unit."""
        ground = self.rig.elevation_at(x, y)
        if unit_id == "rover":
            return ts.ROVER_THERMAL, ground + ts.ROVER_THERMAL_HEIGHT_M, ts.ROVER_THERMAL_PITCH_DOWN_DEG
        from ocu.core.camera import SCOUT_PITCH_DOWN_DEG

        return ts.SCOUT_THERMAL, ground + z, SCOUT_PITCH_DOWN_DEG

    def look(self, unit_id: str, x: float, y: float, z: float, heading: float, mission_s: float, entities=(), poses=(), binned: bool = False) -> ts.ThermalFrame:
        """What a unit's thermal camera sees now. Not written to the mosaic."""
        sensor, camera_z, pitch = self._camera(unit_id, x, y, z, heading)
        if binned:
            sensor = sensor.binned(MAP_BINNING)
        return self.imager.render(sensor, x, y, camera_z, heading, pitch, self.local_hour(mission_s), entities, poses, rng=self.rng)

    def observe(self, unit_id: str, x: float, y: float, z: float, heading: float, mission_s: float, entities=(), poses=()) -> ts.ThermalFrame:
        """Image the ground and write what is seen into the mosaic. The mast camera is binned for this."""
        frame = self.look(unit_id, x, y, z, heading, mission_s, entities, poses, binned=unit_id == "rover")
        self.map.integrate(frame, mission_s, entities, poses)
        return frame
