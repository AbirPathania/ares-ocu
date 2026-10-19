"""Things arriving and leaving: the interface's motion, not the vehicles'.

(The flight model's own motion is tested in test_motion.py.)

Motion here is continuity, not decoration, so what the tests hold is that it
never loses the thing it is animating: a widget is visible for the whole of
its exit and gone at the end of it, and an operator who acts twice in quick
succession does not end up with a banner stuck half way.
"""

import os
import time

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QWidget  # noqa: E402

from ocu.ui.motion import FAST_MS, NORMAL_MS, Reveal  # noqa: E402


@pytest.fixture(scope="module")
def app():
    yield QApplication.instance() or QApplication([])


def pump(app, ms):
    end = time.time() + ms / 1000.0
    while time.time() < end:
        app.processEvents()


def test_it_starts_hidden_and_at_zero(app):
    widget = QWidget()
    reveal = Reveal(widget)
    assert reveal.value == 0.0


def test_revealing_shows_the_widget_at_once_and_eases_the_value(app):
    """The widget has to be there for the whole of its arrival, or the first
    frames animate something nobody can see."""
    widget = QWidget()
    reveal = Reveal(widget)
    reveal.reveal()
    assert widget.isVisible()
    pump(app, NORMAL_MS // 3)
    assert 0.0 < reveal.value < 1.0
    pump(app, NORMAL_MS)
    assert reveal.value == 1.0


def test_dismissing_keeps_it_visible_until_it_has_actually_gone(app):
    widget = QWidget()
    reveal = Reveal(widget)
    reveal.reveal()
    pump(app, NORMAL_MS + 40)

    reveal.dismiss()
    pump(app, NORMAL_MS // 3)
    assert widget.isVisible(), "it must not vanish part way through leaving"
    assert 0.0 < reveal.value < 1.0
    pump(app, NORMAL_MS)
    assert reveal.value == 0.0
    assert not widget.isVisible()


def test_reversing_part_way_through_does_not_stall(app):
    """Dismissed while still arriving, it leaves at the speed it arrived at
    rather than taking a full duration to cover a fraction of the distance."""
    widget = QWidget()
    reveal = Reveal(widget)
    reveal.reveal()
    pump(app, NORMAL_MS // 4)
    part_way = reveal.value
    assert 0.0 < part_way < 1.0

    reveal.dismiss()
    pump(app, int(NORMAL_MS * part_way) + 60)
    assert reveal.value == 0.0
    assert not widget.isVisible()


def test_dismissing_something_already_hidden_is_harmless(app):
    widget = QWidget()
    reveal = Reveal(widget)
    reveal.dismiss()
    assert reveal.value == 0.0
    assert not widget.isVisible()


def test_cancelling_takes_it_away_without_animating(app):
    """A reset is not an event to watch: the mission is starting over."""
    widget = QWidget()
    reveal = Reveal(widget)
    reveal.reveal()
    pump(app, NORMAL_MS + 40)

    reveal.cancel()
    assert reveal.value == 0.0
    assert not widget.isVisible(), "gone immediately, with no frames in between"


def test_the_hook_sees_every_step(app):
    widget = QWidget()
    seen = []
    reveal = Reveal(widget, duration_ms=FAST_MS, on_value=seen.append)
    reveal.reveal()
    pump(app, FAST_MS + 60)
    assert len(seen) > 2, "it animated rather than jumping"
    assert seen[-1] == 1.0
    assert seen == sorted(seen), "and it only went one way"


def test_durations_are_short_enough_not_to_be_waited_on(app):
    assert FAST_MS <= 150 and NORMAL_MS <= 250
