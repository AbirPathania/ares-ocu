"""Draws the camera feeds on a thread of their own.

A frame is a few milliseconds of GPU work and a wait for the result, and four
of them every tick would take that long out of a UI thread that is trying to
redraw sixty times a second. The scene lives on this thread (a GL context
belongs to the thread that made it), requests arrive as plain data, and frames
come back as a signal.

Only the newest request for each unit is kept. A frame that was asked for and
has since been overtaken by a newer one is not worth drawing, and a worker that
fell behind would otherwise show the past.
"""

from __future__ import annotations

import threading

from PySide6.QtCore import QObject, Signal

from ocu.core.terrain import TerrainWorld


class GpuRenderWorker(QObject):
    frameReady = Signal(str, object)  # unit id, (height, width, 3) uint8
    ready = Signal(bool)  # True once the scene is built, False if there is no GPU

    def __init__(self, world: TerrainWorld, parent=None, trail_name: str | None = None) -> None:
        super().__init__(parent)
        self._world = world
        self._trail_name = trail_name
        self._prefetch: str | None = None
        self._pending: dict[str, dict] = {}
        self._wake = threading.Condition()
        self._stop = False
        self._scene = None
        self.available = False
        self.trail_length_m = 0.0  # metres of reconstructed trail loaded, 0 for none
        #: True once the scene has been built or has failed to be: until then there
        #: is nothing to draw with, and the interface waits instead of improvising.
        self.resolved = False
        self._thread = threading.Thread(target=self._run, name="gpu-camera", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def prefetch(self, trail_name: str) -> None:
        """Have a trail's splat read in when the worker next has a moment, so that the first frame drawn from it does not wait for the disk."""
        with self._wake:
            self._prefetch = trail_name
            self._wake.notify()

    def submit(self, unit_id: str, request: dict) -> None:
        with self._wake:
            self._pending[unit_id] = request
            self._wake.notify()

    def stop(self) -> None:
        with self._wake:
            self._stop = True
            self._wake.notify()
        self._thread.join(timeout=3.0)

    def _run(self) -> None:
        from ocu.render.gpu_scene import GpuScene

        self._scene = GpuScene.create(self._world)
        if self._scene is not None:
            self._scene.load_trail(self._trail_name)  # a reconstructed trail, if one has been built (assets/footage/trail)
            self.trail_length_m = self._scene.trail_length_m()
        self.available = self._scene is not None
        self.resolved = True
        self.ready.emit(self.available)
        if self._scene is None:
            return
        while True:
            with self._wake:
                while not self._pending and not self._stop:
                    self._wake.wait()
                if self._stop:
                    return
                work, self._pending = self._pending, {}
                warm, self._prefetch = self._prefetch, None
            if warm is not None:
                current = self._scene.trail["name"] if self._scene.trail else None
                self._scene.load_trail(warm)
                if current is not None:
                    self._scene.load_trail(current)  # the one being drawn stays the current one
            for unit_id, request in work.items():
                try:
                    frame = self._scene.render(**request)
                except Exception:
                    continue
                self.frameReady.emit(unit_id, frame)
