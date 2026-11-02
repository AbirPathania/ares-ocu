"""The accumulated point cloud on the map.

The cloud is written straight into the image's pixels rather than through a
painter call per point, because a scan is several thousand points. These
tests hold that fast path to what it replaced: points land where the world
says, in the colour the class and intensity say, and nothing lands outside.
"""

import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication  # noqa: E402

from ocu.core.lidar import ScanResult  # noqa: E402
from ocu.core.terrain import Terrain  # noqa: E402
from ocu.ui.map_view import MapView  # noqa: E402
from ocu.ui.theme import RETURN_COLORS, TERRAIN_COLORS  # noqa: E402


@pytest.fixture(scope="module")
def app():
    yield QApplication.instance() or QApplication([])


def scan_of(points, intensity=1.0):
    xs = np.array([p[0] for p in points], dtype=np.float64)
    ys = np.array([p[1] for p in points], dtype=np.float64)
    kinds = np.array([int(p[2]) for p in points], dtype=np.uint8)
    return ScanResult(
        xs=xs,
        ys=ys,
        kinds=kinds,
        intensities=np.full(xs.shape, intensity),
        ranges=np.hypot(xs, ys),
        new_cells=0,
        hits=0,
    )


def pixel_at(view, x, y):
    px = int(x * view._cloud_px_per_m) + view._cloud_half_px
    py = view._cloud_half_px - int(y * view._cloud_px_per_m)
    return view._perception_image.pixelColor(px, py)


def test_a_return_lands_at_its_own_world_position(app):
    view = MapView()
    view.add_returns(scan_of([(10.0, -4.0, Terrain.ROCK)]), 0.0, {})
    assert pixel_at(view, 10.0, -4.0).alpha() > 0
    assert pixel_at(view, -10.0, 4.0).alpha() == 0, "nothing anywhere else"


def test_a_full_intensity_return_is_drawn_in_the_return_colour(app):
    view = MapView()
    view.add_returns(scan_of([(6.0, 6.0, Terrain.TREE)], intensity=1.0), 0.0, {})
    colour = pixel_at(view, 6.0, 6.0)
    want = RETURN_COLORS["TREE"]
    for got, expected in ((colour.red(), want.red()), (colour.green(), want.green()), (colour.blue(), want.blue())):
        assert abs(got - expected) <= 3, (colour, want)


def test_a_weak_return_is_drawn_dimmer_than_a_strong_one(app):
    view = MapView()
    view.add_returns(
        scan_of([(4.0, 0.0, Terrain.GROUND)], intensity=1.0), 0.0, {}
    )
    view.add_returns(
        scan_of([(8.0, 0.0, Terrain.GROUND)], intensity=0.05), 0.0, {}
    )
    strong, weak = pixel_at(view, 4.0, 0.0), pixel_at(view, 8.0, 0.0)
    assert strong.alpha() > weak.alpha()
    assert strong.green() > weak.green()


def test_a_hard_return_paints_a_tighter_cluster_than_open_ground(app):
    view = MapView()
    view.add_returns(scan_of([(5.0, 5.0, Terrain.ROCK), (-5.0, -5.0, Terrain.GROUND)]), 0.0, {})

    def filled_around(x, y):
        px = int(x * view._cloud_px_per_m) + view._cloud_half_px
        py = view._cloud_half_px - int(y * view._cloud_px_per_m)
        return sum(
            view._perception_image.pixelColor(px + dx, py + dy).alpha() > 0
            for dx in (0, 1)
            for dy in (0, 1)
        )

    assert filled_around(5.0, 5.0) == 4
    assert filled_around(-5.0, -5.0) == 1


def test_returns_outside_the_world_are_dropped_rather_than_wrapped(app):
    """An index off the edge must not land on the far side of the image."""
    view = MapView()
    far = view._cloud_half_px / view._cloud_px_per_m + 50.0
    view.add_returns(scan_of([(far, far, Terrain.ROCK), (-far, -far, Terrain.TREE)]), 0.0, {})
    pixels = view._cloud_pixels()
    assert int((pixels != 0).sum()) == 0, "nothing was drawn at all"


def test_the_point_count_follows_the_scan(app):
    view = MapView()
    view.add_returns(scan_of([(1.0, 1.0, Terrain.GROUND)] * 7), 0.0, {})
    assert view._cloud_points == 7
    view.add_returns(scan_of([(2.0, 2.0, Terrain.GROUND)] * 3), 0.0, {})
    assert view._cloud_points == 10


def test_clearing_the_cloud_empties_the_image(app):
    view = MapView()
    view.add_returns(scan_of([(3.0, 3.0, Terrain.ROCK)]), 0.0, {})
    assert int((view._cloud_pixels() != 0).sum()) > 0
    view.clear_perception()
    assert int((view._cloud_pixels() != 0).sum()) == 0
    assert view._cloud_points == 0


def test_an_empty_scan_is_harmless(app):
    view = MapView()
    view.add_returns(scan_of([]), 0.0, {})
    assert view._cloud_points == 0


def test_the_map_carries_no_lidar_readout(app):
    """The classification key and scanner statistics were removed from the map:
    the map is for the ground, not for the instrument that drew it."""
    from ocu.ui.map_view import MapView

    view = MapView()
    assert not hasattr(view, "_draw_legend")


# --- the display runs faster than the simulation ----------------------------------


def test_the_drawn_icon_eases_toward_the_reported_position(app):
    """The simulation advances ten times a second. Drawing an aircraft only
    when it does makes it jump across the map; the display runs at sixty and
    eases, so a step in the simulation is a glide on the screen."""
    from ocu.ui.map_view import MapView

    view = MapView()
    view.set_unit_pose("rover", 0.0, 0.0, 0.0, 0.0)
    view.set_unit_pose("rover", 10.0, 0.0, 0.0, 0.0)

    view._advance_shown_poses(0.016)
    part_way = view._drawn_pose("rover")["x"]
    assert 0.0 < part_way < 10.0, "a frame covers part of the step, not all of it"

    for _ in range(60):
        view._advance_shown_poses(0.016)
    assert abs(view._drawn_pose("rover")["x"] - 10.0) < 0.01, "and it arrives"


def test_easing_never_changes_what_the_simulation_said(app):
    """The drawn position is a presentation detail. Anything that asks where
    a unit is gets the simulation's answer."""
    from ocu.ui.map_view import MapView

    view = MapView()
    view.set_unit_pose("rover", 0.0, 0.0, 0.0, 0.0)
    view.set_unit_pose("rover", 7.0, -3.0, 0.0, 90.0)
    view._advance_shown_poses(0.016)
    assert view._units["rover"] == {"x": 7.0, "y": -3.0, "z": 0.0, "heading": 90.0}


def test_the_easing_does_not_depend_on_how_often_it_runs(app):
    """Frame rate is not something the operator chose, so it must not change
    where anything appears to be."""
    from ocu.ui.map_view import MapView

    def after(step_s, total_s):
        view = MapView()
        view.set_unit_pose("rover", 0.0, 0.0, 0.0, 0.0)
        view.set_unit_pose("rover", 10.0, 0.0, 0.0, 0.0)
        for _ in range(int(round(total_s / step_s))):
            view._advance_shown_poses(step_s)
        return view._drawn_pose("rover")["x"]

    assert abs(after(0.016, 0.096) - after(0.004, 0.096)) < 0.05


def test_a_heading_change_takes_the_short_way_round(app):
    from ocu.ui.map_view import MapView

    view = MapView()
    view.set_unit_pose("rover", 0.0, 0.0, 0.0, 350.0)
    view.set_unit_pose("rover", 0.0, 0.0, 0.0, 10.0)
    for _ in range(3):
        view._advance_shown_poses(0.016)
    heading = view._drawn_pose("rover")["heading"]
    assert heading > 345.0 or heading < 25.0, f"swept the long way through {heading:.0f}"


def test_docking_jumps_rather_than_flying_the_icon_home(app):
    """A unit that docks is on the rover. Easing across that gap draws it
    crossing ground it never crossed."""
    from ocu.ui.map_view import MapView

    view = MapView()
    view.set_unit_pose("tricopter-1", 40.0, 40.0, 20.0, 0.0)
    view._advance_shown_poses(0.5)
    view.set_unit_docked("tricopter-1", True)
    view.set_unit_pose("tricopter-1", 0.0, 0.0, 0.0, 0.0)
    view._advance_shown_poses(0.016)
    assert view._drawn_pose("tricopter-1")["x"] == 0.0


def test_a_unit_appears_where_it_is_first_reported(app):
    """No glide in from the origin the first time something is seen."""
    from ocu.ui.map_view import MapView

    view = MapView()
    view.set_unit_pose("tricopter-2", 25.0, -14.0, 18.0, 200.0)
    assert view._drawn_pose("tricopter-2")["x"] == 25.0
    assert view._drawn_pose("tricopter-2")["y"] == -14.0


def test_detection_labels_that_would_stack_are_placed_clear_of_each_other():
    view = MapView()
    rects = [view._free_label_rect(100.0, 200.0, 92.0, 13.0) for _ in range(6)]
    for i, a in enumerate(rects):
        for b in rects[i + 1:]:
            assert not a.intersects(b)
