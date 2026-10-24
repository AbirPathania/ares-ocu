"""The thermal mosaic: it holds only what the cameras have seen, and it ages."""

import numpy as np
import pytest

import ocu.core.thermal_map as tm
from ocu.core.entities import SurfaceEntity
from ocu.core.thermal_scene import SCOUT_THERMAL, ThermalFrame


def frame(points, temp=20.0, along=0.3, across=0.3, entity=None, ground=True):
    """A frame whose pixels are `points` (x, y), laid in a row."""
    n = len(points)
    hx = np.array([[p[0] for p in points]], dtype=np.float32)
    hy = np.array([[p[1] for p in points]], dtype=np.float32)
    full = lambda v: np.full((1, n), v, dtype=np.float32)  # noqa: E731
    return ThermalFrame(full(temp), np.full((1, n), ground), np.full((1, n), -1 if entity is None else entity, dtype=np.int16), hx, hy,
                        full(10.0), full(along), full(across), SCOUT_THERMAL, np.ones((1, n), dtype=bool))


def test_nothing_is_known_until_a_camera_has_seen_it():
    m = tm.ThermalMap(half_extent_m=20.0)
    assert m.seen_fraction == 0.0 and m.observed_range() is None
    assert m.rgba(0.0, 30.0, "iron", 0.0)[..., 3].max() == 0, "unseen ground is transparent"


def test_a_picture_writes_the_temperature_where_the_ray_met_the_ground_and_the_time():
    m = tm.ThermalMap(half_extent_m=20.0)
    m.integrate(frame([(3.0, 4.0)], temp=17.5), now_s=42.0)
    ix, iy = m._index(3.0, 4.0)
    assert m.temp_c[ix, iy] == pytest.approx(17.5) and m.seen_s[ix, iy] == 42.0
    assert m.seen_fraction > 0.0
    far = m._index(-10.0, -10.0)
    assert np.isnan(m.temp_c[far])


def test_a_pixel_that_covers_too_much_ground_is_not_written():
    m = tm.ThermalMap(half_extent_m=20.0)
    assert m.integrate(frame([(3.0, 4.0)], along=tm.MAX_FOOTPRINT_M * 3.0), 1.0) == 0
    assert m.seen_fraction == 0.0


def test_a_coarse_pixel_covers_several_cells_and_a_fine_one_covers_one():
    fine, coarse = tm.ThermalMap(half_extent_m=20.0), tm.ThermalMap(half_extent_m=20.0)
    fine.integrate(frame([(3.0, 4.0)], along=0.1, across=0.1), 1.0)
    coarse.integrate(frame([(3.0, 4.0)], along=1.5, across=1.5), 1.0)
    assert np.isfinite(fine.temp_c).sum() == 1
    assert np.isfinite(coarse.temp_c).sum() > 20


def test_a_newer_picture_replaces_an_older_one():
    m = tm.ThermalMap(half_extent_m=20.0)
    m.integrate(frame([(3.0, 4.0)], temp=10.0), 1.0)
    m.integrate(frame([(3.0, 4.0)], temp=22.0), 90.0)
    assert m.temp_c[m._index(3.0, 4.0)] == pytest.approx(22.0) and m.seen_s[m._index(3.0, 4.0)] == 90.0


def test_what_was_seen_long_ago_fades_but_never_to_nothing():
    m = tm.ThermalMap(half_extent_m=20.0)
    m.integrate(frame([(3.0, 4.0)]), 0.0)
    cell = m._index(3.0, 4.0)
    assert m.alpha(tm.FRESH_S - 1.0)[cell] == 1.0
    assert m.alpha(tm.FRESH_S + tm.FADE_S / 2.0)[cell] == pytest.approx((1.0 + tm.MIN_STALE_ALPHA) / 2.0, abs=0.01)
    assert m.alpha(1e6)[cell] == pytest.approx(tm.MIN_STALE_ALPHA)
    assert m.alpha(0.0)[m._index(0.0, 0.0)] == 0.0


def test_a_body_is_a_short_lived_contact_and_never_written_into_the_mosaic():
    m = tm.ThermalMap(half_extent_m=20.0)
    walker = SurfaceEntity("person", "w", 1.78, 0.54, 0.63, ((5.0, 5.0),))
    f = frame([(5.0, 5.0)] * 4 + [(1.0, 1.0)] * 20, temp=0.0)
    f.entity[0, :4] = 0
    f.temp_c[0, :4] = 22.0
    f.ground[0, :4] = False  # the pixels on the body are not ground
    m.integrate(f, 10.0, [walker], [(5.0, 5.0, 0.0)])
    assert np.isnan(m.temp_c[m._index(5.0, 5.0)]), "the walker is not in the mosaic"
    assert [c.kind for c in m.contacts] == ["person"]
    m.integrate(frame([(1.0, 1.0)]), 10.0 + tm.CONTACT_LIFE_S + 1.0)
    assert m.contacts == [], "and the contact is let go"


def test_the_picture_changes_the_version_so_a_view_knows_to_redraw():
    m = tm.ThermalMap(half_extent_m=20.0)
    v = m.version
    m.integrate(frame([(3.0, 4.0)]), 1.0)
    assert m.version > v
    m.clear()
    assert m.seen_fraction == 0.0 and m.contacts == []


def test_the_rgba_uses_the_palette_between_the_levels():
    m = tm.ThermalMap(half_extent_m=20.0)
    m.integrate(frame([(3.0, 4.0)], temp=30.0), 1.0)
    px = m.rgba(0.0, 30.0, "white_hot", 1.0)[m._index(3.0, 4.0)]
    assert tuple(px[:3]) == (255, 255, 255) and px[3] == 255
