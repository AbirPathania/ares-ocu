"""How things arrive and leave.

Motion in a console is continuity, not decoration. A panel that appears
instantly asks the operator to re-find the layout and work out what changed;
one that eases in over a few frames keeps the eye where it already was, and
the direction it moves says where it came from. The same goes for leaving: a
banner that vanishes between two glances leaves a doubt about whether it was
answered or missed.

The durations are deliberately short. Long enough to be followed, short
enough that nobody is ever waiting for the interface: an operator who has
decided to press something should never watch an animation first. Anything
that would delay a decision is not animated at all.
"""

from __future__ import annotations

from PySide6.QtCore import QEasingCurve, QVariantAnimation

#: A change to something already on screen: a status, a selection.
FAST_MS = 110

#: Something arriving or leaving.
NORMAL_MS = 180


class Reveal(QVariantAnimation):
    """A 0 to 1 value for a widget that appears and disappears.

    The widget is shown for the whole of its fade in and stays visible for
    the whole of its fade out, hiding only once it has actually gone. What
    the widget does with the value is its own business: fade, slide, or both.

    Reversing mid-flight is scaled to the distance left to travel, so a
    banner dismissed while it is still arriving leaves at the speed it was
    arriving at rather than stalling for a full duration.
    """

    def __init__(self, widget, duration_ms: int = NORMAL_MS, on_value=None) -> None:
        super().__init__(widget)
        self._widget = widget
        self._duration_ms = duration_ms
        self._on_value_hook = on_value
        self._value = 0.0
        self.setEasingCurve(QEasingCurve.OutCubic)
        self.valueChanged.connect(self._on_value)
        self.finished.connect(self._on_finished)

    @property
    def value(self) -> float:
        return self._value

    def reveal(self) -> None:
        self._widget.show()
        self._to(1.0)

    def apply(self) -> None:
        """Push the current value through the hook without animating."""
        if self._on_value_hook is not None:
            self._on_value_hook(self._value)

    def dismiss(self) -> None:
        if not self._widget.isVisible():
            self._value = 0.0
            return
        self._to(0.0)

    def cancel(self) -> None:
        """Gone now, with no animation: a reset, or a mission starting over."""
        self.stop()
        self._value = 0.0
        self.apply()
        self._widget.hide()

    def _to(self, target: float) -> None:
        self.stop()
        distance = abs(target - self._value)
        if distance <= 0.001:
            self._value = target
            self._on_finished()
            return
        self.setStartValue(self._value)
        self.setEndValue(target)
        self.setDuration(max(int(self._duration_ms * distance), 1))
        self.start()

    def _on_value(self, value) -> None:
        self._value = float(value)
        if self._on_value_hook is not None:
            self._on_value_hook(self._value)
        self._widget.update()

    def _on_finished(self) -> None:
        if self._value <= 0.001:
            self._widget.hide()
