"""The mission clock.

Elapsed mission time, with play, hold and a speed multiplier. That is all it
is. It used to play back a scripted timeline (phase markers, link cuts,
keyframed positions), and every one of those was a thing happening on a
schedule whether or not it made sense: a phase called "alert" with nothing
alerting, a link cut nobody asked for, aircraft placed where a script said.
Where the vehicles are, what they find, what state the link is in and what
the mission is doing are all computed from the simulation now, so the clock
has nothing to narrate.

Elapsed time is the one thing sensors and the power model integrate against,
which is why it is a separate object that can be held and sped up without
changing how fast anything physical happens per unit of mission time.
"""

from __future__ import annotations

from PySide6.QtCore import QObject, QTimer, Signal

TICK_MS = 100


class MissionClock(QObject):
    clockTick = Signal(float)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._clock_s = 0.0
        self._speed = 1.0
        self._playing = False
        self._timer = QTimer(self)
        self._timer.setInterval(TICK_MS)
        self._timer.timeout.connect(self._tick)

    def play(self) -> None:
        self._playing = True
        self._timer.start()

    def pause(self) -> None:
        self._playing = False
        self._timer.stop()

    @property
    def playing(self) -> bool:
        return self._playing

    def set_speed(self, speed: float) -> None:
        self._speed = max(0.0, speed)

    @property
    def speed(self) -> float:
        return self._speed

    @property
    def clock_s(self) -> float:
        return self._clock_s

    def reset(self) -> None:
        self.pause()
        self._clock_s = 0.0
        self._speed = 1.0

    def _tick(self) -> None:
        self._clock_s += (TICK_MS / 1000.0) * self._speed
        self.clockTick.emit(self._clock_s)
