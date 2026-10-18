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
