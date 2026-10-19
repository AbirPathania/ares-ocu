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

from ocu.core.constants import ROVER_SURVEY_SPEED_MS  # noqa: E402
from ocu.ui.main_window import MainWindow  # noqa: E402

TRICOPTERS = ["tricopter-1", "tricopter-2", "tricopter-3"]

#: What the rover reports while it is heading for a destination: exploring until
#: it has seen the whole way, then going.
EN_ROUTE = ("EXPLORING TO DESTINATION", "GOING TO DESTINATION")


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
    slow.mission_clock.set_speed(1.0)
    fast.mission_clock.set_speed(10.0)
    slow.mission_clock.play()
    fast.mission_clock.play()

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


def test_a_tile_is_rendered_at_its_own_resolution_rather_than_upscaled(app):
    """A soft, upscaled picture is the one thing on this display that cannot
    be defended as what the sensor really saw."""
    window = MainWindow()
    window.resize(1760, 1040)
    window.show()
    pump(app)

    tile = window.camera_feeds["rover"]
    width, height = tile.render_size()
    ratio = tile.devicePixelRatioF() or 1.0
    assert width >= tile.width() * ratio - 1
    assert height >= tile.height() * ratio - 1
    assert abs(width / height - tile.width() / tile.height()) < 0.25, "same shape as the tile"


def test_a_tile_asks_for_a_sane_size_even_before_it_is_laid_out(app):
    """An unshown widget has no useful size yet, and a render of zero pixels
    would be a crash rather than a blank tile."""
    from ocu.ui.camera_feed import SimulatedCameraFeed

    width, height = SimulatedCameraFeed("rover", "ROVER").render_size()
    assert width >= 96 and height >= 40


# --- launches, in the real window, with the rover driving ---------------------------


def _drive(window, seconds):
    for _ in range(int(seconds / 0.1)):
        window.mission_clock._tick()
        window._on_director_tick()


def _deploy_and_drive(window, seconds):
    window._on_play_clicked()
    _drive(window, seconds)


def test_a_guided_launch_from_a_driving_rover_forms_on_the_pattern_the_operator_chose(app):
    """The regression that started this: the rover drives on during a launch
    and the formation must still end up where the dial and the pattern say,
    relative to where the rover is *now*."""
    import math

    from ocu.core.formation import compute_formation_targets

    window = MainWindow()
    _deploy_and_drive(window, 12.0)
    console = window.context_panel.launch_console
    window._on_bay_launch_requested()
    bearing, pattern = console.bearing_spin.value(), console.current_pattern()
    spacing, standoff, rally = (
        console.spacing_spin.value(), console.standoff_spin.value(), console.rally_altitude_spin.value()
    )
    console._on_stage_clicked()
    console._on_launch_confirmed()

    # Distance driven, not displacement: the lanes reverse, so a rover that
    # has swept thirty metres can finish a few metres from where it started.
    previous = (window.units["rover"].pose.x, window.units["rover"].pose.y)
    driven = 0.0
    for _ in range(450):
        window.mission_clock._tick()
        window._on_director_tick()
        here = (window.units["rover"].pose.x, window.units["rover"].pose.y)
        driven += math.hypot(here[0] - previous[0], here[1] - previous[1])
        previous = here
    rover = window.units["rover"].pose
    assert driven > 8.0, "the rover must have driven on during the launch"
    assert window.flight_director.launch_stage(TRICOPTERS) == "FORMED"

    # The dial reads against the rover's nose, so the pattern it describes is
    # wherever the nose is pointing now.
    want = compute_formation_targets(
        rover.x, rover.y, rover.heading + bearing, pattern, spacing, rally, standoff_m=standoff
    )
    for uid, target in zip(TRICOPTERS, want):
        pose = window.units[uid].pose
        assert math.hypot(pose.x - target.x, pose.y - target.y) < 0.6, uid
        assert abs(pose.z - target.z) < 0.7, uid


def test_a_formation_staged_before_the_rover_turns_is_still_off_the_nose_when_launched(app):
    """Stage facing one way, let the rover turn, launch: the aircraft belong
    where the dial says relative to the nose *now*. Measured before the fix:
    staged facing 127, launched facing 247, the aircraft finished 146 degrees
    off the nose, behind the vehicle."""
    import math

    from ocu.core.flight_director import to_body_frame

    window = MainWindow()
    _deploy_and_drive(window, 12.0)
    console = window.context_panel.launch_console
    window._on_bay_launch_requested()
    console.bearing_spin.setValue(0.0)
    console._on_stage_clicked()

    # The lane turns while the operator is deciding.
    window.flight_director.rover_body.heading = (window.units["rover"].pose.heading + 120.0) % 360.0
    console._on_launch_confirmed()
    _drive(window, 45.0)

    assert window.flight_director.launch_stage(TRICOPTERS) == "FORMED"
    rover = window.units["rover"].pose
    ahead = []
    for uid in TRICOPTERS:
        pose = window.units[uid].pose
        forward, _right = to_body_frame(pose.x - rover.x, pose.y - rover.y, rover.heading)
        ahead.append(forward)
    # A vee pushed forward: every aircraft is in front of the vehicle it flies on.
    assert min(ahead) > 5.0, ahead
    # Read off the slots rather than where the aircraft are this instant: the
    # rover turns as it sweeps and an aircraft trails its slot by a few metres
    # through a turn, which is true of aircraft and not what is under test.
    rights = [window.flight_director.aircraft[uid].slot_right for uid in TRICOPTERS]
    assert abs(sum(rights) / len(rights)) < 3.0, "a dead-ahead dial puts the group's centre on the nose line"
    assert all(math.isfinite(f) for f in ahead)


def test_the_speed_slider_sets_how_fast_the_rover_may_drive(app):
    from ocu.core.constants import ROVER_MIN_COMMAND_SPEED_MS, ROVER_TOP_SPEED_MS
    from ocu.ui.main_window import slider_to_pace

    window = MainWindow()
    assert window.flight_director.commanded_speed_ms == pytest.approx(ROVER_SURVEY_SPEED_MS, abs=0.05)

    def peak(window, seconds=40.0):
        fastest = 0.0
        for _ in range(int(seconds / 0.1)):
            window.mission_clock._tick()
            window._on_director_tick()
            fastest = max(fastest, window.flight_director.rover_body.speed_ms)
        return fastest

    window._on_play_clicked()
    crawl = peak(window)
    window.pace_slider.setValue(window.pace_slider.maximum())
    assert window.flight_director.commanded_speed_ms == pytest.approx(ROVER_TOP_SPEED_MS)
    fast = peak(window)
    assert crawl <= ROVER_SURVEY_SPEED_MS + 0.05
    assert fast > crawl * 3, "the slider is a real control, not a label"
    assert fast <= ROVER_TOP_SPEED_MS + 1e-6, "and never more than the platform can do"
    window.pace_slider.setValue(0)
    assert window.flight_director.commanded_speed_ms == pytest.approx(ROVER_MIN_COMMAND_SPEED_MS)
    assert slider_to_pace(0) < slider_to_pace(50) < slider_to_pace(100)


def test_the_speed_slider_is_finer_where_a_sweep_is_driven(app):
    """Quadratic: the survey crawl sits well inside the slider, not in its first pixel."""
    from ocu.ui.main_window import PACE_SLIDER_STEPS, pace_to_slider, slider_to_pace

    position = pace_to_slider(ROVER_SURVEY_SPEED_MS)
    assert 0.15 * PACE_SLIDER_STEPS < position < 0.45 * PACE_SLIDER_STEPS
    assert slider_to_pace(position) == pytest.approx(ROVER_SURVEY_SPEED_MS, abs=0.05)


def test_a_reset_leaves_the_speed_where_the_operator_put_it(app):
    window = MainWindow()
    window.pace_slider.setValue(60)
    chosen = window.flight_director.commanded_speed_ms
    window._on_reset_clicked()
    assert window.flight_director.commanded_speed_ms == chosen
    assert window.pace_slider.value() == 60


def test_the_slider_speed_caps_every_way_the_rover_drives(app):
    """Sweep, a route to a destination and the way home all go no faster than it."""
    window = MainWindow()
    window.pace_slider.setValue(20)
    ceiling = window.flight_director.commanded_speed_ms
    window._on_play_clicked()
    _drive(window, 15.0)
    assert window.flight_director.rover_body.speed_ms <= ceiling + 1e-6
    window._on_destination_placed(20.0, 20.0)
    for _ in range(150):
        window.mission_clock._tick()
        window._on_director_tick()
        assert window.flight_director.rover_body.speed_ms <= ceiling + 1e-6


def test_aircraft_ride_in_the_bay_with_the_rover_until_they_lift_off(app):
    import math

    window = MainWindow()
    _deploy_and_drive(window, 10.0)
    rover = window.units["rover"].pose
    for uid in TRICOPTERS:
        pose = window.units[uid].pose
        assert math.hypot(pose.x - rover.x, pose.y - rover.y) < 1e-6
        assert pose.z == 0.0


def _capture_log(window):
    messages = []
    original = window.event_log.log

    def recording(clock_s, message, *args, **kwargs):
        messages.append(message)
        original(clock_s, message, *args, **kwargs)

    window.event_log.log = recording
    return messages


def test_nothing_flies_unless_the_operator_launches_it(app):
    """Aircraft are launched by a person. The plan reaching aerial recon, the
    rover seeing hidden ground, a long run with no one touching anything:
    none of it puts an aircraft in the air."""
    window = MainWindow()
    _deploy_and_drive(window, 45.0)  # long enough to scan, sweep, find things and recommend
    assert window.flight_director.aircraft == {}
    assert all(window._docked_state.values())
    assert all(window.units[uid].pose.z == 0.0 for uid in TRICOPTERS)


def _drive_until_recommended(window, limit_s=100.0):
    for _ in range(int(limit_s / 0.1)):
        window.mission_clock._tick()
        window._on_director_tick()
        if window._recommended_bearing is not None:
            return True
    return False


def test_the_rover_recommends_scouts_from_its_own_map_but_does_not_launch_them(app):
    window = MainWindow()
    messages = _capture_log(window)
    window._on_play_clicked()
    assert _drive_until_recommended(window), "a third of the ground ahead is hidden at some point on this ground"

    advice = [m for m in messages if m.startswith("Rover recommends scouts:")]
    assert len(advice) == 1
    assert "hidden" in advice[0] and "best bearing" in advice[0]
    _drive(window, 20.0)
    assert len([m for m in messages if m.startswith("Rover recommends scouts:")]) == 1, "said once"
    assert window.flight_director.aircraft == {}


def test_the_rover_does_not_advise_before_it_is_going_anywhere(app):
    """At a standstill 'ahead' is whatever the vehicle happens to be parked
    facing. It says nothing until it is moving."""
    window = MainWindow()
    window._on_play_clicked()
    assert window.flight_director.rover_body.speed_ms < 0.5
    assert window._recommended_bearing is None


def test_the_recommendation_is_a_reading_of_the_rovers_own_map(app):
    """Everything known ahead: silence. Take a sector of it away: it speaks,
    and points at the sector."""
    from ocu.core.occlusion import FAR_M, NEAR_M
    from ocu.core.terrain import Terrain
    import math

    window = MainWindow()
    window._on_play_clicked()
    _drive(window, 8.0)  # moving now
    window._recommended_bearing = None
    window.flight_director.aircraft.clear()

    rover = window.units["rover"].pose
    reach = int(80 / window.perception.cell_size_m)
    for cx in range(-reach + int(rover.x / 0.5), reach + int(rover.x / 0.5) + 1):
        for cy in range(-reach + int(rover.y / 0.5), reach + int(rover.y / 0.5) + 1):
            window.perception.observe((cx, cy), Terrain.GROUND)
    window._check_scout_recommendation()
    assert window._recommended_bearing is None, "nothing is hidden, so nothing to say"

    # Hide the ground 40 degrees off the direction of travel.
    target = (rover.heading + 40.0) % 360.0
    for cx in range(-reach + int(rover.x / 0.5), reach + int(rover.x / 0.5) + 1):
        for cy in range(-reach + int(rover.y / 0.5), reach + int(rover.y / 0.5) + 1):
            x, y = cx * 0.5 - rover.x, cy * 0.5 - rover.y
            if NEAR_M <= math.hypot(x, y) <= FAR_M + 2:
                to_cell = math.degrees(math.atan2(x, y)) % 360.0
                if abs((to_cell - target + 180.0) % 360.0 - 180.0) <= 20.0:
                    window.perception._observed.pop((cx, cy), None)
    window._check_scout_recommendation()
    assert window._recommended_bearing is not None
    assert abs((window._recommended_bearing - target + 180.0) % 360.0 - 180.0) <= 20.0


def test_the_launch_dial_opens_on_the_rovers_recommendation(app):
    window = MainWindow()
    window._on_play_clicked()
    assert _drive_until_recommended(window)
    recommended = window._recommended_bearing

    window._on_bay_launch_requested()
    dial = window.context_panel.launch_console.bearing_spin.value()
    heading = window.units["rover"].pose.heading
    # The dial is relative to the nose, so the recommendation arrives as the
    # angle the operator would have to turn the formation to.
    assert abs((heading + dial - recommended + 180.0) % 360.0 - 180.0) < 1.0


def test_with_nothing_recommended_the_dial_opens_straight_ahead(app):
    """Zero is the rover's nose. Scouting along the line of travel is what
    nearly every launch is for, and it stays true when the rover turns."""
    window = MainWindow()
    _deploy_and_drive(window, 3.0)
    # Whether the rover has already said something by now depends on the ground
    # it happens to be crossing; the dial is what is under test.
    window._recommended_bearing = None
    window._on_bay_launch_requested()
    assert window.context_panel.launch_console.bearing_spin.value() == 0.0


def test_the_formation_preview_turns_with_the_rover(app):
    """A vee drawn ahead of a rover that has since turned is showing the
    operator somewhere the aircraft will not be."""
    import math

    window = MainWindow()
    _deploy_and_drive(window, 10.0)
    window._on_bay_launch_requested()
    console = window.context_panel.launch_console
    rover = window.units["rover"].pose

    def leader_bearing(heading):
        console.set_rover_pose(rover.x, rover.y, heading)
        lead = console._current_targets()[1]  # the slot on the formation's axis
        return math.degrees(math.atan2(lead.x - rover.x, lead.y - rover.y))

    before = leader_bearing(rover.heading)
    after = leader_bearing(rover.heading + 90.0)
    turned = (after - before + 180.0) % 360.0 - 180.0
    assert abs(turned - 90.0) < 2.0, f"the preview turned {turned:.0f} with a 90 degree turn"


def test_the_preview_is_redrawn_when_the_rover_turns(app):
    window = MainWindow()
    _deploy_and_drive(window, 10.0)
    window._on_bay_launch_requested()
    console = window.context_panel.launch_console
    rover = window.units["rover"].pose
    console.set_rover_pose(rover.x, rover.y, rover.heading)

    seen = []
    console.previewChanged.connect(lambda targets, origin: seen.append(targets))
    console.set_rover_pose(rover.x, rover.y, rover.heading)
    assert seen == [], "standing still is not a reason to redraw"

    console.set_rover_pose(rover.x, rover.y, rover.heading + 30.0)
    assert len(seen) == 1, "turning is"


def test_a_launch_the_operator_commands_is_flown_to_the_end(app):
    window = MainWindow()
    _deploy_and_drive(window, 20.0)
    console = window.context_panel.launch_console
    window._on_bay_launch_requested()
    console._on_stage_clicked()
    console._on_launch_confirmed()
    _drive(window, 40.0)
    assert window.flight_director.launch_stage(TRICOPTERS) == "FORMED"
    assert all(not docked for docked in window._docked_state.values())


# --- the speed readout -----------------------------------------------------------------


def test_the_speed_readout_is_the_vehicles_speed_even_when_the_timers_drift(app):
    """The position moves on the director's timer and the mission clock on
    another. The readout used to difference one against the other, so it
    swung between double and zero and a clamp hid the doubling. It must show
    what the vehicle is doing however the two timers happen to interleave."""
    import random

    window = MainWindow()
    window._on_play_clicked()
    rnd = random.Random(11)
    worst = 0.0
    for _ in range(400):
        for _ in range(rnd.choice([1, 1, 1, 2, 0, 1])):
            window._on_director_tick()
        window.mission_clock._tick()
        true_kmh = window.flight_director.rover_body.speed_ms * 3.6
        shown = float(window.context_panel.rover_panel.speed._value)
        worst = max(worst, abs(shown - true_kmh))
    assert worst < 0.06, "the readout is rounded to one decimal and nothing else"


def test_the_rover_never_exceeds_its_stated_top_speed(app):
    from ocu.core.constants import ROVER_TOP_SPEED_MS

    window = MainWindow()
    window._on_play_clicked()
    fastest = 0.0
    for _ in range(600):
        window._on_director_tick()
        fastest = max(fastest, window.flight_director.rover_body.speed_ms)
    assert fastest <= ROVER_TOP_SPEED_MS + 1e-6
    assert fastest < 3.0, "a clearance sweep is not driven flat out"


# --- the end of the mission -----------------------------------------------------------------


def test_the_mission_clock_runs_until_the_operator_holds_it(app):
    window = MainWindow()
    window._on_play_clicked()
    for _ in range(3000):  # five minutes of mission time, far past any old script
        window.mission_clock._tick()
    assert window.mission_clock.playing
    assert window.mission_clock.clock_s > 299.0


# --- nothing happens on a schedule -----------------------------------------------------------


def test_the_link_is_never_cut_unless_the_operator_cuts_it(app):
    window = MainWindow()
    messages = _capture_log(window)
    _deploy_and_drive(window, 80.0)
    assert not any("Link cut" in m or "link" in m.lower() and "cut" in m.lower() for m in messages)
    assert all(unit.link_state == "CONNECTED" for unit in window.units.values())


def test_the_log_never_announces_a_phase_it_is_not_in(app):
    window = MainWindow()
    messages = _capture_log(window)
    _deploy_and_drive(window, 80.0)
    assert not any(m.startswith("Phase:") for m in messages)


# --- the status is read from the vehicles ---------------------------------------------------


def test_the_top_bar_reads_what_the_rover_is_doing(app):
    window = MainWindow()
    assert window.phase_label.text() == "STANDBY"

    window._on_play_clicked()
    assert window.phase_label.text() in ("FIRST SCAN", "SWEEPING LANE 1")
    _drive(window, 5.0)
    assert window.phase_label.text().startswith("SWEEPING LANE")

    window._on_play_clicked()
    assert window.phase_label.text() == "HOLDING"
    assert window.play_btn.text().endswith("RESUME")

    window._on_play_clicked()
    window._on_return_to_base()
    _drive(window, 0.5)
    assert window.phase_label.text() == "RETURNING TO BASE"


def test_the_roster_reads_what_each_aircraft_is_doing(app):
    window = MainWindow()
    assert all(window.units[uid].current_task == "IN BAY" for uid in TRICOPTERS)

    _deploy_and_drive(window, 5.0)
    console = window.context_panel.launch_console
    window._on_bay_launch_requested()
    console._on_stage_clicked()
    console._on_launch_confirmed()
    _drive(window, 4.0)
    assert window.units["tricopter-1"].current_task == "CLIMBING"
    _drive(window, 40.0)
    assert all(window.units[uid].current_task == "ON STATION" for uid in TRICOPTERS)


def test_the_sweep_reports_when_it_is_complete(app):
    window = MainWindow()
    window._on_play_clicked()
    window.mission_clock._clock_s = 0.0
    window._waypoint_index = len(window._mission_waypoints)
    window.flight_director.goal = None
    window._refresh_status()
    assert window.phase_label.text() == "SWEEP COMPLETE"


# --- hold ---------------------------------------------------------------------------------------


def test_hold_stops_the_rover_where_it_is_and_resume_carries_on(app):
    window = MainWindow()
    _deploy_and_drive(window, 12.0)
    window._on_play_clicked()  # hold
    _drive(window, 4.0)  # let it brake
    held = (window.units["rover"].pose.x, window.units["rover"].pose.y)
    _drive(window, 6.0)
    after = (window.units["rover"].pose.x, window.units["rover"].pose.y)
    assert abs(after[0] - held[0]) < 1e-6 and abs(after[1] - held[1]) < 1e-6
    assert window.flight_director.rover_body.speed_ms == 0.0

    window._on_play_clicked()  # resume
    _drive(window, 6.0)
    moved = (window.units["rover"].pose.x, window.units["rover"].pose.y)
    assert abs(moved[0] - held[0]) + abs(moved[1] - held[1]) > 3.0


def test_the_aircraft_keep_station_on_the_rover_while_the_mission_is_held(app):
    import math

    window = MainWindow()
    _deploy_and_drive(window, 5.0)
    console = window.context_panel.launch_console
    window._on_bay_launch_requested()
    console._on_stage_clicked()
    console._on_launch_confirmed()
    _drive(window, 45.0)
    window._on_play_clicked()  # hold
    _drive(window, 10.0)
    rover = window.units["rover"].pose
    for uid in TRICOPTERS:
        state = window.flight_director.aircraft[uid]
        assert state.mode.value == "STATION"
        assert math.hypot(state.body.vx, state.body.vy) < 0.2, "hovering, not drifting"
        assert math.hypot(state.x - rover.x, state.y - rover.y) <= 50.0


# --- reporting, and what a cut link shows ----------------------------------------------------


def test_a_reading_is_reported_only_when_it_is_news(app):
    window = MainWindow()
    key = (1, "rover")
    assert window._worth_reporting(key, 0.30), "the first reading always is"
    assert not window._worth_reporting(key, 0.31), "a creep of a hundredth is not"
    assert not window._worth_reporting(key, 0.34)
    assert window._worth_reporting(key, 0.36), "a real step up is"
    assert not window._worth_reporting(key, 0.30), "and falling back is not"


def test_crossing_the_escalation_threshold_is_always_reported(app):
    from ocu.core.constants import ALERT_CONFIDENCE_THRESHOLD

    window = MainWindow()
    key = (2, "rover")
    just_below = ALERT_CONFIDENCE_THRESHOLD - 0.01
    assert window._worth_reporting(key, just_below)
    assert window._worth_reporting(key, ALERT_CONFIDENCE_THRESHOLD + 0.005), (
        "a step of half a hundredth is too small to report, except that it is the one that escalates"
    )


def _form_up(window, seconds_after_launch=45.0):
    _deploy_and_drive(window, 5.0)
    console = window.context_panel.launch_console
    window._on_bay_launch_requested()
    console._on_stage_clicked()
    console._on_launch_confirmed()
    _drive(window, seconds_after_launch)
    assert window.flight_director.launch_stage(TRICOPTERS) == "FORMED"


def _settle_heading(window, uid, limit_s=60.0):
    """Drive on until the aircraft is pointing steadily. "Ahead of it" only
    means something while it is not in the middle of a turn, and where the
    rover happens to be turning depends on the ground it is crossing."""
    scout = window.flight_director.aircraft[uid]
    history = []
    for _ in range(int(limit_s / 0.1)):
        window.mission_clock._tick()
        window._on_director_tick()
        history.append(scout.heading)
        recent = history[-20:]
        if len(recent) == 20 and max(recent) - min(recent) < 0.5:
            return


def _plant_surface_object_ahead_of(window, uid, distance_m=40.0):
    import math

    from ocu.core.targets import BuriedTarget

    from ocu.core.terrain import IMPASSABLE

    _settle_heading(window, uid)
    scout = window.flight_director.aircraft[uid]
    heading = math.radians(scout.heading)
    # An object under a tree or in a hole cannot be seen from above whatever
    # the scout does, so plant it on ground that is open to the sky.
    for distance in (distance_m, distance_m - 5, distance_m + 5, distance_m - 10, distance_m - 15):
        x, y = scout.x + math.sin(heading) * distance, scout.y + math.cos(heading) * distance
        if window.terrain.classify(x, y) not in IMPASSABLE:
            break
    target = BuriedTarget(x, y, 0.02, 90.0, True, 0.1, "planted")
    window.target_field.targets.append(target)
    return target


def test_a_scout_that_sees_something_reports_it_on_the_aircraft_map(app):
    from ocu.core.mesh_state import voxel_key

    window = MainWindow()
    _form_up(window)
    target = _plant_surface_object_ahead_of(window, "tricopter-2")
    _drive(window, 1.0)

    holdings = window.mesh_link.holdings()
    voxel = voxel_key(target.x, target.y, 0.0)
    assert voxel in holdings, "the scout saw it"
    assert any(
        threat["x"] == target.x and threat["y"] == target.y for threat in window.threat_panel._threats.values()
    )


def test_cutting_the_link_while_a_scout_sees_something_leaves_the_maps_diverged_until_restored(app):
    """The signature interaction. What the scout finds while the link is down
    stays on its side, visibly, and the restore reports what crossed."""
    from ocu.core.mesh_state import voxel_key

    window = MainWindow()
    _form_up(window)
    window._on_cut_link()
    assert window.mesh_link.divergence() == 0

    target = _plant_surface_object_ahead_of(window, "tricopter-2")
    _drive(window, 1.0)

    voxel = voxel_key(target.x, target.y, 0.0)
    assert window.mesh_link.holdings()[voxel] == {"aircraft"}, "seen by the scout, not yet by the rover"
    assert window.mesh_link.divergence() >= 1

    window._on_restore_link()
    assert window.mesh_link.holdings()[voxel] == {"aircraft", "rover"}
    assert window.mesh_link.divergence() == 0
    assert window.mesh_link.last_restore_delta >= 1


def test_the_rover_side_diverges_too_when_it_finds_something_during_a_cut(app):
    window = MainWindow()
    _deploy_and_drive(window, 2.0)
    window._on_cut_link()
    _drive(window, 25.0)  # the rover is still finding things on its first lane
    assert window.mesh_link.divergence() >= 1
    window._on_restore_link()
    assert window.mesh_link.divergence() == 0


def _drive_until_something_is_found(window, limit_s=90.0):
    """Drive until the threat list has an entry. How long that takes depends on
    the rover's pace and the ground, so the test waits for the event rather than
    for a number of seconds that would have to be re-measured at every pace."""
    for _ in range(int(limit_s / 0.1)):
        window.mission_clock._tick()
        window._on_director_tick()
        if window.threat_panel._threats:
            return True
    return False


def test_a_reset_forgets_what_was_found_so_it_can_be_found_again(app):
    window = MainWindow()
    window._on_play_clicked()
    assert _drive_until_something_is_found(window)
    window._on_reset_clicked()
    assert not window._target_evidence and not window._aerial_evidence and not window._escalated

    window._on_play_clicked()
    assert _drive_until_something_is_found(window), "the rover can find the same objects again"


# --- how fast things move, and how fast you watch them ------------------------------


def _peak_speed_over(window, seconds):
    fastest = 0.0
    for _ in range(int(seconds / 0.1)):
        window.mission_clock._tick()
        window._on_director_tick()
        fastest = max(fastest, window.flight_director.rover_body.speed_ms)
    return fastest


def test_the_rover_sweeps_at_survey_speed_not_transit_speed(app):
    """A clearance pass is limited by what the sensors can resolve, not by
    what the wheels can do. The platform's 15 km/h is a transit figure."""
    from ocu.core.constants import ROVER_SURVEY_SPEED_MS, ROVER_TOP_SPEED_MS

    window = MainWindow()
    window._on_play_clicked()
    assert _peak_speed_over(window, 60.0) <= ROVER_SURVEY_SPEED_MS + 1e-6
    assert ROVER_SURVEY_SPEED_MS < ROVER_TOP_SPEED_MS / 3, "a survey crawl, not a slightly slower drive"


def test_driving_home_goes_as_fast_as_the_operator_allows(app):
    """Nothing is being cleared on the way back, so nothing but the operator's
    speed setting holds it to a crawl."""
    from ocu.core.constants import ROVER_SURVEY_SPEED_MS

    window = MainWindow()
    window._on_play_clicked()
    _drive(window, 25.0)
    window.pace_slider.setValue(60)
    window._on_return_to_base()
    assert _peak_speed_over(window, 25.0) > ROVER_SURVEY_SPEED_MS * 1.5


def test_playback_speed_changes_when_you_see_the_mission_not_what_happens(app):
    """Faster playback is the same mission seen sooner.

    Exactly the same is not achievable and not claimed: the motion model is
    integrated in sub-steps so a body never travels further in one jump than
    its acceleration allows, but the control loop above it (replanning,
    taking the next lane point) runs once per tick like a real one, so fast
    playback revises the route less often per mission second. What that may
    not do is fly a different mission, so the vehicle has to end up on the
    same route, within a small fraction of how far it has driven.
    """
    import math

    def after(speed, mission_seconds):
        window = MainWindow()
        window.mission_clock.set_speed(speed)
        window._on_play_clicked()
        for _ in range(int(mission_seconds / (0.1 * speed))):
            window.mission_clock._tick()
            window._on_director_tick()
        pose = window.units["rover"].pose
        return (pose.x, pose.y), window.mission_clock.clock_s

    seconds = 30.0
    driven = ROVER_SURVEY_SPEED_MS * seconds  # upper bound on distance covered
    real_time, clock = after(1.0, seconds)
    for speed in (0.5, 2.0, 4.0):
        moved, other_clock = after(speed, seconds)
        assert abs(other_clock - clock) < 0.5, "the same mission time elapsed"
        drift = math.hypot(moved[0] - real_time[0], moved[1] - real_time[1])
        assert drift < driven * 0.05, f"playback {speed}x flew a different mission: {drift:.2f} m apart"


def test_the_batteries_burn_on_mission_time_at_any_playback_speed(app):
    slow, fast = MainWindow(), MainWindow()
    slow.mission_clock.set_speed(1.0)
    fast.mission_clock.set_speed(4.0)
    slow._on_play_clicked()
    fast._on_play_clicked()

    for _ in range(80):
        slow.mission_clock._tick(); slow._on_director_tick()
    for _ in range(20):
        fast.mission_clock._tick(); fast._on_director_tick()

    slow_used = 96.0 - slow.units["rover"].battery_pct
    fast_used = 96.0 - fast.units["rover"].battery_pct
    assert slow_used > 0
    assert abs(slow_used - fast_used) < slow_used * 0.05


# --- which feed gets the frames ---------------------------------------------------


def _count_frames(window, ticks=40):
    from collections import Counter

    seen = Counter()
    for unit_id, feed in window.camera_feeds.items():
        feed.set_frame = (lambda uid: (lambda frame: seen.update([uid])))(unit_id)
    for _ in range(ticks):
        window._tick_cameras()
    return seen


def test_the_feed_the_operator_is_watching_gets_the_frames(app):
    """A frame costs about twenty milliseconds, so they cannot all be live.
    An even share would make the one feed being watched as slow as the three
    that are not."""
    window = MainWindow()
    for uid in TRICOPTERS:
        window._set_docked(uid, False)
    window._select_unit("tricopter-2")

    seen = _count_frames(window)
    assert seen["tricopter-2"] >= sum(v for k, v in seen.items() if k != "tricopter-2")
    assert all(seen[uid] > 0 for uid in seen), "the others still refresh"


def test_every_live_feed_still_refreshes_when_the_rover_is_selected(app):
    window = MainWindow()
    for uid in TRICOPTERS:
        window._set_docked(uid, False)
    window._select_unit("rover")
    seen = _count_frames(window)
    assert set(seen) == {"rover", *TRICOPTERS}


def test_only_live_units_are_rendered(app):
    window = MainWindow()
    window._select_unit("rover")
    seen = _count_frames(window)
    assert set(seen) == {"rover"}, "aircraft in the bay have no view to render"


# --- small screens ---------------------------------------------------------------------------


@pytest.mark.parametrize("size", [(1000, 640), (1280, 720), (1440, 900)])
def test_the_window_works_at_laptop_sizes_with_no_panel_over_another(app, size):
    from PySide6.QtWidgets import QSplitter

    from ocu.ui import main_window as mw

    window = MainWindow()
    window.resize(*size)
    window.show()
    pump(app, 10)
    assert (window.width(), window.height()) == size, "the window honours a laptop size"
    splitter = window.findChild(QSplitter)
    columns = [splitter.widget(i) for i in range(3)]
    floors = (mw.LEFT_COLUMN_MIN_WIDTH, mw.MAP_MIN_WIDTH, mw.RIGHT_COLUMN_MIN_WIDTH)
    for column, floor in zip(columns, floors):
        assert column.width() >= floor - 1, (column.objectName(), column.width(), floor)
    assert sum(c.width() for c in columns) <= window.width()
    # The formation form scrolls rather than being cut off.
    assert window.context_scroll.widget() is window.context_panel
    window.close()


def test_the_alert_banner_keeps_both_decisions_whole_on_a_narrow_map(app):
    from ocu.ui.alert_banner import COMPACT_BELOW_WIDTH, AlertBanner

    banner = AlertBanner()
    banner.resize(COMPACT_BELOW_WIDTH - 100, 120)
    banner.show()
    banner.show_alert({"type": "mine", "x": 4.0, "y": -8.0, "confidence": 0.87, "source_id": "rover"})
    pump(app, 20)
    for button in (banner.confirm_btn, banner.reject_btn):
        assert button.width() >= button.sizeHint().width(), button.text()
        assert button.geometry().right() <= banner.width()
    banner.resize(COMPACT_BELOW_WIDTH + 200, 120)
    pump(app, 10)
    assert banner.confirm_btn.geometry().top() == banner.reject_btn.geometry().top()
    banner._timer.stop()


# --- the console is one window ---------------------------------------------------------------


def test_nothing_the_console_shows_ever_becomes_a_window_of_its_own(app):
    """A widget with no parent that is shown is a native window. Each new threat
    card used to be one for an instant: it took focus from the console, and on a
    Mac sent the operator back to the terminal that had launched it."""
    from PySide6.QtCore import QEvent, QObject
    from PySide6.QtWidgets import QWidget

    shown = []

    class Watch(QObject):
        def eventFilter(self, obj, event):
            if event.type() == QEvent.Show and isinstance(obj, QWidget) and obj.isWindow():
                shown.append(type(obj).__name__)
            return False

    watch = Watch()
    window = MainWindow()
    app.installEventFilter(watch)
    try:
        window.show()
        window._on_play_clicked()
        assert _drive_until_something_is_found(window)
        pump(app, 30)
    finally:
        app.removeEventFilter(watch)
        window.close()
    assert set(shown) <= {"MainWindow"}, shown


# --- a destination the operator sets --------------------------------------------------------


def _drive_until_settled(window, limit_s=240.0):
    for _ in range(int(limit_s / 0.1)):
        window.mission_clock._tick()
        window._on_director_tick()
        if window._destination_state != "enroute":
            return window._destination_state
    return window._destination_state


def test_the_rover_drives_to_a_destination_the_operator_sets_and_stops_there(app):
    import math

    window = MainWindow()
    window.pace_slider.setValue(60)  # a transit, not a survey crawl
    window._on_play_clicked()
    window._on_destination_placed(25.0, -18.0)
    assert window.phase_label.text() in EN_ROUTE
    assert _drive_until_settled(window) == "reached"
    rover = window.units["rover"].pose
    assert math.hypot(rover.x - 25.0, rover.y + 18.0) < 1.5
    assert window.phase_label.text() == "AT DESTINATION"
    _drive(window, 5.0)
    assert window.flight_director.rover_body.speed_ms < 0.05, "it stays put once it is there"


def test_the_rover_goes_round_what_is_in_the_way_to_a_destination(app):
    """The ground truth is checked tick by tick: the rover's own position is
    never on a rock, a tree or a pothole on the way."""
    from ocu.core.terrain import IMPASSABLE

    window = MainWindow()
    window.pace_slider.setValue(70)
    window._on_play_clicked()
    window._on_destination_placed(-45.0, -30.0)
    for _ in range(int(240.0 / 0.1)):
        window.mission_clock._tick()
        window._on_director_tick()
        rover = window.units["rover"].pose
        assert window.terrain.classify(rover.x, rover.y) not in IMPASSABLE, (rover.x, rover.y)
        if window._destination_state != "enroute":
            break
    assert window._destination_state == "reached"


def test_a_destination_set_before_deploying_is_the_job_the_rover_starts_on(app):
    window = MainWindow()
    messages = _capture_log(window)
    window._on_destination_placed(15.0, 15.0)
    assert window.phase_label.text() == "STANDBY"
    window._on_play_clicked()
    assert window.phase_label.text() in EN_ROUTE
    assert window.flight_director.goal is not None
    assert not any("Sweeping" in m for m in messages)


def test_clearing_a_destination_picks_the_lane_sweep_up_again(app):
    window = MainWindow()
    window._on_play_clicked()
    _drive(window, 10.0)
    lane = window._current_lane
    waypoint = window._waypoint_index
    window._on_destination_placed(30.0, 30.0)
    assert window._current_lane == "DESTINATION" and window._destination is not None
    _drive(window, 5.0)
    window._on_destination_cleared()
    assert window._destination is None
    assert window._waypoint_index == waypoint
    assert window._current_lane == lane
    assert window.phase_label.text().startswith("SWEEPING")
    assert window.map_view._destination is None


def test_a_destination_outside_the_operating_area_is_refused(app):
    window = MainWindow()
    window._on_play_clicked()
    window._on_destination_placed(500.0, 0.0)
    assert window._destination is None
    assert window.map_view._destination is None


def test_a_destination_is_refused_while_the_rover_is_under_manual_control(app):
    window = MainWindow()
    window._on_play_clicked()
    _drive(window, 2.0)
    window._on_take_manual("rover")
    window._on_destination_placed(20.0, 20.0)
    assert window._destination is None


def test_a_destination_can_be_set_while_the_mission_is_on_hold(app):
    window = MainWindow()
    window._on_play_clicked()
    _drive(window, 5.0)
    window._on_play_clicked()  # hold
    window._on_destination_placed(20.0, 20.0)
    before = (window.units["rover"].pose.x, window.units["rover"].pose.y)
    for _ in range(30):
        window._on_director_tick()
    assert (window.units["rover"].pose.x, window.units["rover"].pose.y) == pytest.approx(before, abs=0.05)
    assert window.phase_label.text() == "HOLDING"
    window._on_play_clicked()  # resume
    assert window.phase_label.text() in EN_ROUTE


def test_a_reset_forgets_the_destination_and_restores_the_sweep(app):
    window = MainWindow()
    window._on_play_clicked()
    window._on_destination_placed(20.0, 20.0)
    window._on_reset_clicked()
    assert window._destination is None and window.map_view._destination is None
    assert window._mission_waypoints is window.sweep.points


def test_the_goto_button_arms_places_and_clears(app):
    window = MainWindow()
    view = window.map_view
    assert "GO TO" in view.destination_btn.text()
    view.destination_btn.click()
    assert view._placing_destination and "CLICK" in view.destination_btn.text()
    view.destination_btn.click()
    assert not view._placing_destination, "a second press cancels"
    view.destination_btn.click()
    placed = []
    view.destinationPlaced.connect(lambda x, y: placed.append((x, y)))
    from PySide6.QtCore import QEvent, QPointF, Qt
    from PySide6.QtGui import QMouseEvent

    view.resize(900, 700)
    point = QPointF(300.0, 200.0)
    for kind in (QEvent.MouseButtonPress, QEvent.MouseButtonRelease):
        event = QMouseEvent(kind, point, point, Qt.LeftButton, Qt.LeftButton, Qt.NoModifier)
        (view.mousePressEvent if kind == QEvent.MouseButtonPress else view.mouseReleaseEvent)(event)
    assert len(placed) == 1
    window._on_destination_placed(*placed[0])
    assert "CLEAR" in view.destination_btn.text()
    view.destination_btn.click()
    assert window._destination is None


def test_dragging_the_map_does_not_set_a_destination(app):
    from PySide6.QtCore import QEvent, QPointF, Qt
    from PySide6.QtGui import QMouseEvent

    window = MainWindow()
    view = window.map_view
    view.resize(900, 700)
    view.set_placing_destination(True)
    placed = []
    view.destinationPlaced.connect(lambda x, y: placed.append((x, y)))
    down, up = QPointF(300.0, 200.0), QPointF(380.0, 260.0)
    view.mousePressEvent(QMouseEvent(QEvent.MouseButtonPress, down, down, Qt.LeftButton, Qt.LeftButton, Qt.NoModifier))
    view.mouseReleaseEvent(QMouseEvent(QEvent.MouseButtonRelease, up, up, Qt.LeftButton, Qt.NoButton, Qt.NoModifier))
    assert placed == []


def test_the_link_chip_text_sits_in_the_middle_of_its_box(app):
    from PySide6.QtCore import Qt

    window = MainWindow()
    assert window.link_chip.alignment() & Qt.AlignHCenter
    from ocu.ui.main_window import LINK_CHIP_MIN_WIDTH

    assert window.link_chip.minimumWidth() == LINK_CHIP_MIN_WIDTH


def test_there_are_few_contacts_to_deal_with(app):
    """A dozen contacts buried the one that mattered."""
    window = MainWindow()
    assert len(window.target_field) <= 6
    window._on_play_clicked()
    window.pace_slider.setValue(100)
    _drive(window, 200.0)
    assert len(window.threat_panel._threats) <= 5


# --- the map keeps the rover in view --------------------------------------------------------


def test_the_map_follows_a_rover_that_nears_its_edge(app):
    from ocu.ui.map_view import MapView

    view = MapView()
    view.resize(900, 700)
    view.set_unit_pose("rover", 0.0, 0.0, 0.0, 0.0)
    view._advance_shown_poses(1.0)
    view._follow_rover(0.016)
    assert view._pan.x() == 0.0 and view._pan.y() == 0.0, "comfortably inside: the view stays put"

    view.set_unit_pose("rover", 0.0, 60.0, 0.0, 0.0)  # far off the top of the view
    view._advance_shown_poses(1.0)
    for _ in range(300):
        view._follow_rover(0.016)
    point = view._to_screen(0.0, 60.0)
    assert 0.0 < point.y() < view.height(), "the rover is back on the map"


def test_the_map_leaves_the_view_alone_for_a_while_after_the_operator_drags_it(app):
    from PySide6.QtCore import QEvent, QPointF, Qt
    from PySide6.QtGui import QMouseEvent

    from ocu.ui.map_view import FOLLOW_QUIET_S, MapView

    view = MapView()
    view.resize(900, 700)
    down, up = QPointF(300.0, 300.0), QPointF(380.0, 340.0)
    view.mousePressEvent(QMouseEvent(QEvent.MouseButtonPress, down, down, Qt.LeftButton, Qt.LeftButton, Qt.NoModifier))
    view.mouseReleaseEvent(QMouseEvent(QEvent.MouseButtonRelease, up, up, Qt.LeftButton, Qt.NoButton, Qt.NoModifier))
    view.set_unit_pose("rover", 0.0, 90.0, 0.0, 0.0)
    view._advance_shown_poses(1.0)
    before = QPointF(view._pan)
    view._follow_rover(0.016)
    assert view._pan == before, "the operator said where to look"
    for _ in range(int(FOLLOW_QUIET_S / 0.016) + 5):
        view._follow_rover(0.016)
    assert view._pan != before, "and then it goes back to the rover"


def test_a_destination_off_the_edge_of_the_view_is_marked_on_the_edge(app):
    from PySide6.QtGui import QImage, QPainter

    from ocu.ui.map_view import MapView

    view = MapView()
    view.resize(900, 700)
    view.set_unit_pose("rover", 0.0, 0.0, 0.0, 0.0)
    view.set_destination((0.0, 400.0))
    image = QImage(900, 700, QImage.Format_ARGB32)
    image.fill(0)
    painter = QPainter(image)
    view._draw_destination(painter)
    painter.end()
    # Something amber on the top edge, nothing at the centre.
    top_rows = [image.pixelColor(x, y) for x in range(0, 900) for y in range(0, 60)]
    assert any(c.alpha() > 0 for c in top_rows)
    assert image.pixelColor(450, 350).alpha() == 0


# --- the rover only knows the way over ground it has seen ------------------------------------


def test_a_route_to_a_far_destination_is_drawn_only_over_ground_the_rover_has_seen(app):
    import math

    from ocu.core.navigation import PLANNING_HORIZON_M

    window = MainWindow()
    window.pace_slider.setValue(60)
    window._on_play_clicked()
    window._on_destination_placed(60.0, 70.0)  # ~90 m away, none of it seen
    for tick in range(400):
        window.mission_clock._tick()
        window._on_director_tick()
        rover = window.units["rover"].pose
        path = window.map_view._planned_path
        for x, y in path:
            assert math.hypot(x - rover.x, y - rover.y) <= PLANNING_HORIZON_M + 3.0, (tick, x, y)
            assert window.perception.is_known(window.perception_cell(x, y)), "a route over unseen ground"
        goal = window.map_view._planned_goal
        if goal is not None and not window.flight_director.route_known:
            assert goal != window._destination, "the frontier is marked, not the destination"
    assert window.phase_label.text() == "EXPLORING TO DESTINATION"


def test_the_rover_announces_the_route_once_it_has_seen_all_the_way(app):
    window = MainWindow()
    messages = _capture_log(window)
    window.pace_slider.setValue(60)
    window._on_play_clicked()
    window._on_destination_placed(-30.0, -25.0)
    assert _drive_until_settled(window) == "reached"
    assert sum("Route to the destination found" in m for m in messages) == 1
