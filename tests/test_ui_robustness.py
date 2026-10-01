"""Operator actions fired in illegal orders must never raise.

A field operator will press Return To Base twice, hit Launch with nothing
selected, recall an aircraft that is already docked, and reset mid-launch.
None of that is a programming error on their part, so none of it may
produce a traceback. This test drives the real window with a pseudo-random
sequence of operator actions and asserts the app survives.
"""

import os
import random

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication  # noqa: E402

from ocu.ui.main_window import MainWindow  # noqa: E402

TRICOPTERS = ["tricopter-1", "tricopter-2", "tricopter-3"]


@pytest.fixture(scope="module")
def app():
    application = QApplication.instance() or QApplication([])
    yield application


def pump(app, iterations=40):
    for _ in range(iterations):
        app.processEvents()


def test_random_operator_actions_never_raise(app):
    window = MainWindow()
    console = window.context_panel.launch_console
    rng = random.Random(1337)

    actions = [
        lambda: window._on_bay_launch_requested(),
        lambda: console._on_stage_clicked(),
        lambda: console._on_launch_confirmed(),
        lambda: window._on_return_to_base(),
        lambda: window._on_recall_requested(rng.choice(TRICOPTERS)),
        lambda: window._on_altitude_commanded(rng.choice(TRICOPTERS), rng.uniform(1, 500)),
        lambda: window._on_cut_link(),
        lambda: window._on_restore_link(),
        lambda: window._on_play_clicked(),
        lambda: window._on_reset_clicked(),
        lambda: window._select_unit(rng.choice(["rover"] + TRICOPTERS)),
        lambda: window._on_take_manual(rng.choice(["rover", "tricopter-1"])),
        lambda: window._on_release_manual(),
        lambda: window._on_director_tick(),
    ]

    for i in range(400):
        rng.choice(actions)()
        if i % 20 == 0:
            pump(app, 5)

    pump(app, 20)
    assert set(window._docked_state) == set(TRICOPTERS)
    for unit_id, docked in window._docked_state.items():
        if docked:
            assert unit_id not in window.flight_director.airborne_ids


def test_launch_with_nothing_selected_is_a_noop(app):
    window = MainWindow()
    console = window.context_panel.launch_console
    window._on_bay_launch_requested()
    for box in console.checkboxes.values():
        box.setChecked(False)
    console._on_stage_clicked()
    console._on_launch_confirmed()
    pump(app)
    assert all(window._docked_state.values())


def test_recall_of_a_docked_aircraft_is_a_noop(app):
    window = MainWindow()
    window._on_recall_requested("tricopter-1")
    pump(app)
    assert window._docked_state["tricopter-1"] is True


def test_reset_midflight_restores_a_clean_bay(app):
    window = MainWindow()
    console = window.context_panel.launch_console
    window._on_bay_launch_requested()
    console._on_stage_clicked()
    console._on_launch_confirmed()
    pump(app, 30)
    window._on_reset_clicked()
    pump(app, 10)
    assert all(window._docked_state.values())
    assert window.flight_director.aircraft == {}
    assert all(owner == "scripted" for owner in window._position_owner.values())


def _select_mode(window, mode):
    index = window.mode_combo.findData(mode)
    assert index >= 0
    window.mode_combo.setCurrentIndex(index)


def test_ghost_mode_silences_the_feeds_and_the_sticks(app):
    window = MainWindow()
    _select_mode(window, "GHOST")
    pump(app)
    assert all(feed._emcon for feed in window.camera_feeds.values())
    assert all(unit.mission_mode == "GHOST" for unit in window.units.values())
    assert not window.context_panel.rover_panel.manual_btn.isEnabled()
    assert not window.context_panel.rover_panel.rtb_btn.isEnabled()


def test_autonomous_mode_blocks_intervention_but_keeps_the_feeds(app):
    window = MainWindow()
    _select_mode(window, "AUTONOMOUS")
    pump(app)
    assert not any(feed._emcon for feed in window.camera_feeds.values())
    assert not window.context_panel.rover_panel.manual_btn.isEnabled()


def test_switching_to_ghost_drops_an_existing_manual_hold(app):
    window = MainWindow()
    window._on_take_manual("rover")
    assert window.control_authority.manual_holder == "rover"
    _select_mode(window, "GHOST")
    pump(app)
    assert window.control_authority.manual_holder is None


def test_returning_to_standard_restores_control(app):
    window = MainWindow()
    _select_mode(window, "GHOST")
    _select_mode(window, "STANDARD")
    pump(app)
    assert not any(feed._emcon for feed in window.camera_feeds.values())
    assert window.context_panel.rover_panel.manual_btn.isEnabled()


def test_manual_lock_chip_only_appears_when_held(app):
    window = MainWindow()
    assert not window.manual_chip.isVisibleTo(window)
    window._on_take_manual("tricopter-1")
    window._refresh_all_rows()
    # tricopter-1 is docked, so manual is refused and the chip stays hidden
    window._on_take_manual("rover")
    window._refresh_all_rows()
    assert window.manual_chip.isVisibleTo(window)
    window._on_release_manual()
    window._refresh_all_rows()
    assert not window.manual_chip.isVisibleTo(window)


def test_cut_link_makes_the_divergence_visible_on_the_map(app):
    """The cut-link demonstration is the one the spec calls unmissable. It is
    only unmissable if the map actually shows the two replicas disagreeing."""
    window = MainWindow()
    window._on_detection(
        {"x": 10, "y": 4, "confidence": 0.5, "sensor": "gpr", "source": "rover",
         "source_id": "rover", "type": "unknown"}
    )
    pump(app)
    assert window.map_view._mesh_diverged == 0
    shared = window.map_view._holdings[(200, 80, 0)]
    assert shared == {"rover", "aircraft"}

    window._on_cut_link()
    window._on_detection(
        {"x": 20, "y": 8, "confidence": 0.7, "sensor": "thermal", "source": "aircraft",
         "source_id": "tricopter-1", "type": "unknown"}
    )
    pump(app)
    # Seen only by an aircraft while the link is down: one replica holds it.
    assert window.map_view._holdings[(400, 160, 0)] == {"aircraft"}
    assert window.map_view._mesh_diverged == 1
    assert window.map_view._mesh_rover != window.map_view._mesh_aircraft

    window._on_restore_link()
    pump(app)
    assert window.map_view._mesh_diverged == 0
    assert window.map_view._mesh_rover == window.map_view._mesh_aircraft
    assert window.map_view._holdings[(400, 160, 0)] == {"rover", "aircraft"}
    assert window.map_view._reconciled_flash > 0


def test_battery_burns_on_mission_time_not_wall_clock(app):
    """The director ticks in real seconds but the mission can be played back
    faster. Integrating battery against the wall clock left a fast-forwarded
    run showing a pack that had barely moved."""
    slow, fast = MainWindow(), MainWindow()
    slow.playback.set_speed(1.0)
    fast.playback.set_speed(10.0)
    slow.playback.play()
    fast.playback.play()

    for _ in range(50):
        slow._on_director_tick()
    for _ in range(5):
        fast._on_director_tick()

    # Same mission time elapsed either way, so the same charge should be gone.
    slow_used = 96.0 - slow.units["rover"].battery_pct
    fast_used = 96.0 - fast.units["rover"].battery_pct
    assert slow_used > 0
    assert abs(slow_used - fast_used) < slow_used * 0.05


def test_battery_endurance_matches_the_published_platform_figures(app):
    """The figures an investor can check against the spec sheet."""
    window = MainWindow()
    window._on_bay_launch_requested()
    console = window.context_panel.launch_console
    console._on_stage_clicked()
    console._on_launch_confirmed()
    pump(app, 60)

    # A 3 hour rover must barely move across a 90 second mission.
    for _ in range(900):
        window._on_director_tick()
    assert 96.0 - window.units["rover"].battery_pct < 1.0

    # An 11 minute aircraft must lose a believable slice, not most of the pack.
    used = 100.0 - window.units["tricopter-1"].battery_pct
    assert 5.0 < used < 30.0
