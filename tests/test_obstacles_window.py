"""The real window, driving the real rover past things standing and walking on its lanes.

The first version of these entities were drawn and detected and nothing else: the rover drove straight
through a parked truck. These hold the fix to what matters, which is how close the rover's centre ever
comes to the edge of each thing, over a whole run, and that it still gets on with the mission."""

import math
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QTimer  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from ocu.core.dynamic_obstacles import DynamicObstacles  # noqa: E402
from ocu.core.entities import SurfaceEntity, stamp_static_entities  # noqa: E402
from ocu.ui.main_window import MainWindow  # noqa: E402


@pytest.fixture(scope="module")
def app():
    yield QApplication.instance() or QApplication([])


def edge_gap(e, x, y, t, hold):
    """Distance from a point to the edge of an entity: to the rectangle of a parked one, to the circle of a walking one."""
    ex, ey, h = e.pose_at(t - hold)
    if not e.static:
        return math.hypot(x - ex, y - ey) - e.obstacle_radius_m
    hr = math.radians(h)
    dx, dy = x - ex, y - ey
    a, b = dx * math.sin(hr) + dy * math.cos(hr), dx * math.cos(hr) - dy * math.sin(hr)
    return math.hypot(max(abs(a) - e.length_m / 2, 0.0), max(abs(b) - e.width_m / 2, 0.0))


def drive(app, make, seconds=240.0):
    w = MainWindow()
    for timer in w.findChildren(QTimer):
        timer.stop()
    ents = make(w)
    w.surface_entities = ents
    stamp_static_entities(w.terrain, ents)
    w.dynamic_obstacles = DynamicObstacles()
    w._on_play_clicked()
    w.pace_slider.setValue(100)
    gaps = {e.name: math.inf for e in ents}
    path = []
    for _ in range(int(seconds / 0.1)):
        w.mission_clock._tick()
        w._on_director_tick()
        t, r = w.mission_clock.clock_s, w.units["rover"].pose
        path.append((r.x, r.y))
        for e in ents:
            gaps[e.name] = min(gaps[e.name], edge_gap(e, r.x, r.y, t, w._entity_hold.get(e.name, 0.0)))
    travelled = sum(math.dist(a, b) for a, b in zip(path, path[1:], strict=False))
    log = [str(x) for x in getattr(w.event_log, "_entries", [])]
    w.close()
    return gaps, travelled, log


def truck_on_lane(heading):
    return lambda w: [SurfaceEntity("vehicle", "truck", 1.85, 1.9, 5.3, (w.sweep.at(0, 12.0, 0.0),), heading_deg=heading)]


@pytest.mark.parametrize("heading", [0.0, 90.0])
def test_the_rover_drives_round_a_truck_parked_on_its_lane_and_carries_on(app, heading):
    gaps, travelled, _ = drive(app, truck_on_lane(heading), 200.0)
    assert gaps["truck"] > 0.5, f"the rover came within {gaps['truck']:.2f} m of the truck"
    assert travelled > 60.0, "and it went on with the mission instead of stopping at the truck"


def test_the_rover_keeps_clear_of_a_person_crossing_its_lane(app):
    make = lambda w: [SurfaceEntity("person", "p", 1.75, 0.5, 0.3, (w.sweep.at(0, 9.0, 6.0), w.sweep.at(0, 9.0, -6.0)), speed_ms=1.4)]
    gaps, travelled, _ = drive(app, make, 200.0)
    assert gaps["p"] > 0.8, f"the rover came within {gaps['p']:.2f} m of the person"
    assert travelled > 60.0


def test_the_rover_keeps_clear_of_an_animal_on_its_lane_and_gives_up_the_point_it_will_not_clear(app):
    make = lambda w: [SurfaceEntity("animal", "a", 1.0, 0.4, 1.5, (w.sweep.at(0, 8.0, 3.0), w.sweep.at(0, 14.0, -3.0)), speed_ms=0.6)]
    gaps, travelled, _ = drive(app, make, 240.0)
    assert gaps["a"] > 0.8, f"the rover came within {gaps['a']:.2f} m of the animal"
    assert travelled > 100.0, "an animal standing beside a waypoint does not hold the mission up for good"


def test_the_default_scenario_never_puts_the_rover_inside_anything(app):
    gaps, travelled, _ = drive(app, lambda w: w.surface_entities, 240.0)
    assert min(gaps.values()) > 0.5, gaps
    assert travelled > 100.0


# ---- driving by hand

def _clear_heading(w):
    """A bearing from the deploy point with open ground (a metre either side) for 25 m."""
    from ocu.core.terrain import IMPASSABLE

    for deg in range(0, 360, 5):
        hh = math.radians(deg)
        if all(
            w.terrain.at(w.terrain.to_cell(d * math.sin(hh) + o * math.cos(hh), d * math.cos(hh) - o * math.sin(hh))) not in IMPASSABLE
            for d in (i * 0.5 for i in range(1, 50)) for o in (-1.0, 0.0, 1.0)
        ):
            return deg
    raise AssertionError("no clear bearing from the deploy point")


def _drive_by_hand(app, make, seconds=90.0, then_reverse=0.0):
    from PySide6.QtCore import Qt

    w = MainWindow()
    for timer in w.findChildren(QTimer):
        timer.stop()
    deg = _clear_heading(w)
    r = w.units["rover"].pose
    r.heading = w.flight_director.rover_body.heading = float(deg)
    ents = make(r.x, r.y, math.radians(deg))
    w.surface_entities = ents
    stamp_static_entities(w.terrain, ents)
    w.dynamic_obstacles = DynamicObstacles()
    w.control_authority.take_manual("rover")
    w.flight_director.take_manual("rover", r.x, r.y, float(deg))
    w._keys_down.add(Qt.Key_W)
    w.pace_slider.setValue(100)
    gap, path = math.inf, []
    for i in range(int(seconds / 0.1)):
        if then_reverse and i == int(seconds / 0.1) - int(then_reverse / 0.1):
            w._keys_down.discard(Qt.Key_W)
            w._keys_down.add(Qt.Key_S)
        w.mission_clock._tick()
        w._on_director_tick()
        t, p = w.mission_clock.clock_s, w.units["rover"].pose
        path.append((p.x, p.y))
        gap = min(gap, edge_gap(ents[0], p.x, p.y, t, w._entity_hold.get(ents[0].name, 0.0)))
    w.close()
    return gap, path


def _truck_ahead(x, y, h):
    return [SurfaceEntity("vehicle", "truck", 2.2, 2.12, 5.7, ((x + 12 * math.sin(h), y + 12 * math.cos(h)),), heading_deg=math.degrees(h) + 90.0)]


def test_driving_by_hand_straight_at_a_parked_truck_stops_at_it_and_never_goes_through(app):
    gap, path = _drive_by_hand(app, _truck_ahead, 80.0)
    assert gap > 0.15, f"the rover was driven to within {gap:.2f} m of the truck's side"
    assert math.dist(path[0], path[-1]) > 6.0, "and it did drive up to it first"


def test_driving_by_hand_at_a_person_walking_toward_the_rover_stops_a_body_length_short(app):
    make = lambda x, y, h: [SurfaceEntity("person", "p", 1.78, 0.54, 0.63, ((x + 14 * math.sin(h), y + 14 * math.cos(h)), (x + 3 * math.sin(h), y + 3 * math.cos(h))), speed_ms=1.0)]
    gap, _ = _drive_by_hand(app, make, 80.0)
    assert gap > 0.7, f"the rover was driven to within {gap:.2f} m of the person"


def test_having_stopped_at_something_the_operator_can_always_back_away(app):
    gap, path = _drive_by_hand(app, _truck_ahead, 80.0, then_reverse=10.0)
    assert math.dist(path[-1], path[-1 - 100]) > 0.5, "reversing is obeyed: only a step that makes things worse is refused"
