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

from PySide6.QtCore import QTimer  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from ocu.ui.main_window import MainWindow  # noqa: E402

TRICOPTERS = ["tricopter-1", "tricopter-2", "tricopter-3"]


@pytest.fixture(scope="module")
def app():
    application = QApplication.instance() or QApplication([])
    yield application


@pytest.fixture(autouse=True)
def stop_windows_after_each_test(monkeypatch):
    """Stop every window a test opened once the test is over.

    These tests build a real window each and never close it, so by the end
    of the module dozens are alive with their timers running, and every
    processEvents() call runs the director tick, the LiDAR scan and the
    camera render for all of them. The later a test sits in the file the
    slower it gets, for no reason to do with what it checks.
    """
    created = []
    original = MainWindow.__init__

    def tracking(self, *args, **kwargs):
        original(self, *args, **kwargs)
        created.append(self)

    monkeypatch.setattr(MainWindow, "__init__", tracking)
    yield
    for window in created:
        for timer in window.findChildren(QTimer):
            timer.stop()
        window.close()


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
        lambda: window._tick_cameras(),
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


def test_autonomous_drops_video_and_pings_rather_than_streams(app):
    """Section 4.1: minimal RF, periodic status pings only. Not a permission
    level on the operator's buttons, a different downlink."""
    window = MainWindow()
    _select_mode(window, "AUTONOMOUS")
    pump(app)
    assert all(feed._emcon for feed in window.camera_feeds.values()), "no video on minimal RF"
    assert window._comms.telemetry is True
    assert window._comms.continuous is False, "status arrives on a ping, not a stream"
    assert window._comms.uplink is False, "no real-time intervention"
    assert window._comms.mesh_traffic is True, "the vehicles still share a map"


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


def test_ghost_closes_the_downlink_entirely(app):
    """Section 4.1: zero intentional emissions. No video, no telemetry, no
    MANET. The operator sees the last picture before the fleet went quiet."""
    window = MainWindow()
    _select_mode(window, "GHOST")
    pump(app)
    assert window._comms.video is False
    assert window._comms.telemetry is False
    assert window._comms.mesh_traffic is False, "no MANET traffic at all"
    assert window._downlink_open() is False
    assert window.map_view._stale is True


def test_ghost_freezes_the_picture_while_the_fleet_keeps_flying(app):
    """The point of Ghost: the aircraft are still working, the operator just
    is not being told. The simulation must advance while the display does not."""
    window = MainWindow()
    # Put aircraft on station directly. The launch animation runs on a real
    # wall clock, which a test that spins the event loop cannot advance.
    for i, unit_id in enumerate(TRICOPTERS):
        window.flight_director.adopt(unit_id, 20.0 + i * 4, 6.0, 20.0, 0.0, 0.0, 0.0)
        window._docked_state[unit_id] = False
        window.flight_director.set_commanded_altitude(unit_id, 40.0)
    for _ in range(10):
        window._on_director_tick()

    _select_mode(window, "GHOST")
    pump(app)
    shown_before = {k: dict(v) for k, v in window.map_view._units.items()}
    truth_before = {u: window.units[u].pose.z for u in TRICOPTERS}

    for _ in range(60):
        window._on_director_tick()

    truth_after = {u: window.units[u].pose.z for u in TRICOPTERS}
    assert truth_after != truth_before, "the fleet must keep flying while silent"
    assert window.map_view._units == shown_before, "the operator's picture must not update"


def test_detections_found_while_silent_are_not_reported(app):
    window = MainWindow()
    _select_mode(window, "GHOST")
    pump(app)
    before = len(window.map_view._detections)
    window._on_detection(
        {"x": 31, "y": 9, "confidence": 0.77, "sensor": "gpr", "source": "rover",
         "source_id": "rover", "type": "unknown"}
    )
    pump(app)
    assert len(window.map_view._detections) == before, "nothing is radiated in Ghost"


def test_total_loss_of_contact_enters_lca_without_being_ordered(app):
    """Section 4.1: LCA triggers automatically, it is never selected."""
    window = MainWindow()
    assert "LCA" not in [window.mode_combo.itemData(i) for i in range(window.mode_combo.count())]

    for unit in window.units.values():
        unit.link_state = "LOST"
    window._check_lost_comms_autonomous()
    pump(app)
    assert window._comms.mode == "LCA"
    assert window.map_view._stale is True

    for unit in window.units.values():
        unit.link_state = "CONNECTED"
    window._check_lost_comms_autonomous()
    pump(app)
    assert window._comms.mode == "STANDARD"


def test_standard_streams_everything(app):
    window = MainWindow()
    _select_mode(window, "STANDARD")
    pump(app)
    assert window._comms.video and window._comms.telemetry and window._comms.continuous
    assert window._downlink_open() is True
    assert window.map_view._stale is False


def test_ghost_does_not_claim_a_connected_link(app):
    """A green CONNECTED chip beside a mode that radiates nothing is a
    contradiction an investor will spot before you finish the sentence."""
    window = MainWindow()
    _select_mode(window, "GHOST")
    window._refresh_all_rows()
    pump(app)
    assert "SILENT" in window.link_chip.text()
    assert "CONNECTED" not in window.link_chip.text()
    assert all(state == "SILENT" for state in window.event_log.link_strip._states.values())


def test_standard_reports_the_real_link_state(app):
    window = MainWindow()
    window._refresh_all_rows()
    pump(app)
    assert "CONNECTED" in window.link_chip.text()


# --- camera tiles ------------------------------------------------------------


def _undock_all(window):
    for unit_id in TRICOPTERS:
        window._set_docked(unit_id, False)


def test_the_rover_tile_shows_a_picture_from_the_world(app):
    window = MainWindow()
    assert window.camera_feeds["rover"]._frame is None
    for _ in range(4):
        window._tick_cameras()
    assert window.camera_feeds["rover"]._frame is not None


def test_aircraft_in_their_bay_have_no_picture_to_show(app):
    """The bay is dark. Showing the ground from an aircraft that has not
    left it would be showing something that is not happening."""
    window = MainWindow()
    for _ in range(20):
        window._tick_cameras()
    assert all(window.camera_feeds[uid]._frame is None for uid in TRICOPTERS)


def test_every_live_tile_gets_its_turn(app):
    window = MainWindow()
    _undock_all(window)
    for _ in range(8):
        window._tick_cameras()
    assert all(feed._frame is not None for feed in window.camera_feeds.values())


def test_a_recalled_aircraft_does_not_keep_showing_its_last_frame(app):
    window = MainWindow()
    _undock_all(window)
    for _ in range(8):
        window._tick_cameras()
    assert window.camera_feeds["tricopter-2"]._frame is not None

    window._set_docked("tricopter-2", True)
    assert window.camera_feeds["tricopter-2"]._frame is None


def test_no_video_means_no_new_frames_in_ghost_or_autonomous(app):
    """The link decides, not the camera. The aircraft's camera is running;
    nothing is being sent, so nothing new arrives."""
    for mode in ("GHOST", "AUTONOMOUS"):
        window = MainWindow()
        for _ in range(4):
            window._tick_cameras()
        before = window.camera_feeds["rover"]._frame

        _select_mode(window, mode)
        pump(app)
        for _ in range(8):
            window._tick_cameras()
        assert window.camera_feeds["rover"]._frame is before, f"{mode} must not deliver video"
        assert window.camera_feeds["rover"]._emcon


def test_video_resumes_when_the_link_carries_it_again(app):
    window = MainWindow()
    _select_mode(window, "GHOST")
    pump(app)
    for _ in range(4):
        window._tick_cameras()
    assert window.camera_feeds["rover"]._frame is None

    _select_mode(window, "STANDARD")
    pump(app)
    for _ in range(4):
        window._tick_cameras()
    assert window.camera_feeds["rover"]._frame is not None
    assert not window.camera_feeds["rover"]._emcon


def test_a_tile_is_rendered_at_a_sensible_size_for_its_shape(app):
    window = MainWindow()
    window.resize(1760, 1040)
    window.show()
    pump(app)
    width, height = window.camera_feeds["rover"].render_size()
    assert 96 <= width <= 240 and 40 <= height <= 96
    assert width / height > 1.8, "tiles are wide, and frames should match"
