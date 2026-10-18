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


def test_the_legend_and_its_numbers_do_not_overlap(app):
    """The key is laid out from the bottom up, so adding a class to it pushes
    the block up rather than printing the numbers through it."""
    from ocu.ui.map_view import MapView

    view = MapView()
    view.resize(900, 700)
    keys, row_height, stats_gap = 6, 13.0, 16.0
    top = view.height() - 24.0 - 5 * 12.0 - stats_gap - keys * row_height
    last_key_row = top + (keys - 1) * row_height
    first_stat_row = top + keys * row_height + stats_gap
    assert first_stat_row > last_key_row + row_height
    assert first_stat_row + 4 * 12.0 + 13 <= view.height()
    assert top - 16 > 0
