"""The mission log.

A log nobody can scan is a log nobody reads. The two rules that make it
scannable are that one subject occupies one line however many times it is
reported, and that colour means severity and nothing else.
"""

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import Qt  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from ocu.ui.event_log import LEVEL_COLORS, MAX_LOG_ENTRIES, EventLog  # noqa: E402


@pytest.fixture(scope="module")
def app():
    yield QApplication.instance() or QApplication([])


def lines(log):
    return [log.log_list.item(i).text() for i in range(log.log_list.count())]


def test_a_plain_entry_is_timestamped_in_mission_time(app):
    log = EventLog()
    log.log(75.0, "Sweeping LANE 2")
    assert lines(log) == ["[01:15] Sweeping LANE 2"]


def test_repeated_reports_on_one_subject_rewrite_one_line(app):
    """A confidence climbing on one buried object is one event the operator
    is watching, not five lines of it."""
    log = EventLog()
    for confidence in (0.17, 0.25, 0.35, 0.54):
        log.log(8.0, f"MINE at (6, -18) · {confidence:.2f}", key="contact:6:-18")
    assert len(lines(log)) == 1
    assert lines(log)[0].endswith("0.54"), "and it shows the latest reading"


def test_different_subjects_keep_their_own_lines(app):
    log = EventLog()
    log.log(8.0, "contact A", key="contact:1:1")
    log.log(9.0, "contact B", key="contact:2:2")
    log.log(10.0, "contact A again", key="contact:1:1")
    assert len(lines(log)) == 3, "a subject only coalesces while it is the latest line"


def test_an_entry_with_no_subject_never_coalesces(app):
    log = EventLog()
    for _ in range(3):
        log.log(5.0, "Operator cut link")
    assert len(lines(log)) == 3


def test_severity_is_the_only_thing_that_colours_a_line(app):
    log = EventLog()
    log.log(1.0, "routine")
    log.log(2.0, "escalated", level="threat")
    log.log(3.0, "resolved", level="good")
    colours = [log.log_list.item(i).foreground().color().name() for i in range(3)]
    assert colours[0] == LEVEL_COLORS["info"].name()
    assert colours[1] == LEVEL_COLORS["threat"].name()
    assert colours[2] == LEVEL_COLORS["good"].name()


def test_a_rewritten_line_takes_the_new_severity(app):
    """A contact that escalates has to change colour where it already is."""
    log = EventLog()
    log.log(1.0, "contact 0.40", key="c", level="info")
    log.log(2.0, "contact 0.97", key="c", level="threat")
    assert len(lines(log)) == 1
    assert log.log_list.item(0).foreground().color().name() == LEVEL_COLORS["threat"].name()


def test_an_unknown_severity_falls_back_to_routine(app):
    log = EventLog()
    log.log(1.0, "something", level="not-a-level")
    assert log.log_list.item(0).foreground().color().name() == LEVEL_COLORS["info"].name()


def test_the_log_does_not_grow_without_bound(app):
    """It runs for a whole shift."""
    log = EventLog()
    for i in range(MAX_LOG_ENTRIES + 120):
        log.log(float(i), f"entry {i}")
    assert log.log_list.count() == MAX_LOG_ENTRIES
    assert lines(log)[-1].endswith(f"entry {MAX_LOG_ENTRIES + 119}"), "the newest is kept"


def test_clearing_empties_it(app):
    log = EventLog()
    log.log(1.0, "something", key="k")
    log.clear()
    assert lines(log) == []
    log.log(2.0, "after", key="k")
    assert len(lines(log)) == 1, "and the subject does not survive the clear"


# --- the link strip ---------------------------------------------------------------


def test_only_an_interruption_is_coloured(app):
    """A strip that paints ninety-five percent of its area solid to say
    nothing happened spends all its contrast on the usual case."""
    from ocu.ui.event_log import OUTAGE_COLORS

    assert "CONNECTED" not in OUTAGE_COLORS
    assert set(OUTAGE_COLORS) == {"DEGRADED", "LOST", "SILENT"}


def test_the_strip_records_what_it_was_told(app):
    from ocu.ui.event_log import HISTORY_SAMPLES, EventLog

    log = EventLog()
    strip = log.link_strip
    strip.set_link("tricopter-1", "LOST")
    for _ in range(4):
        strip._sample()
    assert strip._history["tricopter-1"][-4:] == ["LOST"] * 4
    assert strip._history["rover"][-1] == "CONNECTED", "the others are untouched"
    assert len(strip._history["tricopter-1"]) == HISTORY_SAMPLES, "the window is fixed"


def test_restoring_a_link_leaves_the_outage_in_the_history(app):
    """What an operator cannot get anywhere else is when it dropped and for
    how long. A strip that only showed the present would duplicate the dot on
    the roster card."""
    from ocu.ui.event_log import EventLog

    log = EventLog()
    strip = log.link_strip
    strip.set_link("tricopter-2", "LOST")
    for _ in range(3):
        strip._sample()
    strip.set_link("tricopter-2", "CONNECTED")
    for _ in range(3):
        strip._sample()

    assert strip._states["tricopter-2"] == "CONNECTED"
    assert "LOST" in strip._history["tricopter-2"], "the outage is still on the record"


def test_clearing_forgets_the_outage(app):
    from ocu.ui.event_log import EventLog

    log = EventLog()
    strip = log.link_strip
    strip.set_link("rover", "LOST")
    strip._sample()
    log.clear()
    assert set(strip._history["rover"]) == {"CONNECTED"}


def test_an_unknown_unit_is_ignored_rather_than_added(app):
    from ocu.ui.event_log import EventLog, ROW_ORDER

    log = EventLog()
    log.set_link("not-a-unit", "LOST")
    assert set(log.link_strip._states) == set(ROW_ORDER)
