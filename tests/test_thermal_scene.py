"""The thermal camera: bodies, buried objects and the sensor (V8, V10 of the registered checks)."""

import math

import numpy as np
import pytest

import ocu.core.thermal_physics as tp
import ocu.core.thermal_scene as ts
from ocu.core.camera import CameraRig
from ocu.core.entities import SurfaceEntity
from ocu.core.targets import BuriedTarget
from ocu.core.terrain import TerrainWorld

NIGHT, MIDDAY, DAWN = 2.0, 12.5, 6.8


@pytest.fixture(scope="module")
def scene():
    world = TerrainWorld.blank(half_extent_m=60.0)
    rig = CameraRig(world)
    targets = [BuriedTarget(0.0, 12.0, 0.03, 120.0, True, 0.3, "shallow")]
    field = tp.build_field(rig, targets)
    return world, rig, field, ts.ThermalImager(rig, field)


def person(x=0.0, y=10.0, heading=180.0):
    return SurfaceEntity("person", "p", 1.78, 0.54, 0.63, ((x, y),), heading_deg=heading)


def look(imager, hour, entities=(), sensor=ts.ROVER_THERMAL, seed=1, pitch=3.0, height=ts.ROVER_THERMAL_HEIGHT_M, at=(0.0, 0.0), heading=0.0):
    poses = [(e.path[0][0], e.path[0][1], e.heading_deg) for e in entities]
    return imager.render(sensor, at[0], at[1], height, heading, pitch, hour, entities, poses, rng=np.random.default_rng(seed))


# --- V8: a person -----------------------------------------------------------------------------------------------------

def air_6c_bands(facing):
    return ts.body_bands(person(), 6.0, math.radians(-10.0), 0.0, 0.0, 12.0, facing)  # night: no sun


def test_v8_exposed_skin_and_clothing_at_six_degrees_of_air():
    by_name = {b.name: b for b in air_6c_bands(True)}
    assert 24.0 <= by_name["face"].base_c <= 34.0
    for name in ("torso", "legs"):
        t = float(ts.band_temperature(by_name[name], 0.0, 6.0, tp.ThermalSite.sky_longwave(6.0), 12.0))
        assert 8.0 <= t <= 20.0, f"{name} {t:.1f} C"


def test_v8_a_person_is_far_warmer_than_the_ground_at_night(scene):
    _, _, _, imager = scene
    frame = look(imager, NIGHT, [person()])
    on_person = frame.entity == 0
    assert on_person.sum() >= 30
    assert frame.temp_c[on_person].mean() - np.median(frame.temp_c[frame.ground]) > 10.0


def test_v8_refuted_at_midday_at_this_cool_site_the_person_is_still_warmer_than_the_ground(scene):
    """DECISIONS.md predicted that at midday a person is no longer warmer than the ground. That is a hot-desert
    expectation, where the ground reaches 50 C. At 3 500 m on a cool October day the ground reaches 26 C and a person
    whose sunlit side is 30 C or more stays visible. The prediction was wrong for this site."""
    _, _, _, imager = scene
    frame = look(imager, MIDDAY, [person()])
    on_person = frame.entity == 0
    assert frame.temp_c[on_person].mean() > np.median(frame.temp_c[frame.ground]) + 1.0


def test_the_sunny_side_of_a_body_is_warmer_than_the_shaded_side(scene):
    """The same truck seen from two sides at dawn, when the sun is low in the east: the face to the sun is warmer."""
    _, _, _, imager = scene
    truck = SurfaceEntity("vehicle", "v", 2.2, 2.12, 5.7, ((0.0, 14.0),), heading_deg=0.0)
    east = look(imager, DAWN, [truck], at=(14.0, 14.0), heading=270.0, pitch=2.0)   # camera east of it, on the sunny side
    west = look(imager, DAWN, [truck], at=(-14.0, 14.0), heading=90.0, pitch=2.0)   # camera west of it, in its shade
    e, w = east.temp_c[east.entity == 0], west.temp_c[west.entity == 0]
    assert len(e) > 20 and len(w) > 20
    assert e.mean() > w.mean() + 1.0, f"sunny {e.mean():.1f} against shaded {w.mean():.1f}"


def test_a_truck_is_colder_than_the_ground_at_night_because_it_reflects_the_cold_sky(scene):
    _, _, _, imager = scene
    truck = SurfaceEntity("vehicle", "v", 2.2, 2.12, 5.7, ((0.0, 14.0),), heading_deg=0.0)
    frame = look(imager, NIGHT, [truck])
    assert frame.temp_c[frame.entity == 0].mean() < np.median(frame.temp_c[frame.ground]) + 0.5


def test_a_body_stands_in_front_of_what_is_behind_it_and_is_hidden_by_what_is_in_front_of_it(scene):
    _, _, _, imager = scene
    near, far = person(-1.0, 8.0), person(3.0, 20.0)
    both = look(imager, NIGHT, [near, far])
    assert (both.entity == 0).sum() > (both.entity == 1).sum() > 0, "the nearer person is bigger, and both are seen"
    # one directly behind the other along the line of sight: the nearer hides it
    inline = look(imager, NIGHT, [person(0.0, 8.0), person(0.0, 8.7)])
    assert (inline.entity == 0).sum() > 0 and (inline.entity == 1).sum() < (inline.entity == 0).sum()


# --- the buried patch -----------------------------------------------------------------------------------------------------

def test_a_shallow_buried_patch_appears_in_the_picture_at_the_temperature_its_column_says(scene):
    _, _, field, imager = scene
    hour = 9.0
    expected = float(field.target_delta_at(hour)[0])
    sensor = ts.ThermalSensor("test", 160, 120, 40.0, 0.0001, 0.0)  # an ideal sensor, to read the signal
    f = look(imager, hour, sensor=sensor, pitch=30.0, height=8.0, at=(0.0, 12.0 - 8.0 / math.tan(math.radians(30.0))))
    d = np.hypot(f.hx - 0.0, f.hy - 12.0)
    inside, ring = f.ground & (d < 0.15), f.ground & (d > 0.7) & (d < 1.6)
    assert inside.sum() >= 3 and ring.sum() >= 50
    measured = f.temp_c[inside].mean() - f.temp_c[ring].mean()
    assert measured == pytest.approx(expected, abs=0.25), f"measured {measured:.2f} K against the column's {expected:.2f} K"
    assert abs(expected) > 0.3, "and there is something to see at nine o'clock"


# --- V10: the sensor ----------------------------------------------------------------------------------------------------------

def test_v10_the_noise_is_the_sensors_noise_equivalent_temperature_difference(scene):
    _, _, _, imager = scene
    a, b = look(imager, MIDDAY, seed=11), look(imager, MIDDAY, seed=12)
    both = a.ground & b.ground
    sigma = float(np.std((a.temp_c - b.temp_c)[both]) / math.sqrt(2.0))
    assert sigma == pytest.approx(ts.ROVER_THERMAL.netd_k, rel=0.10)


def test_the_same_seed_gives_the_same_picture_and_a_different_one_a_different_noise(scene):
    _, _, _, imager = scene
    assert np.array_equal(look(imager, NIGHT, seed=5).temp_c, look(imager, NIGHT, seed=5).temp_c)
    assert not np.array_equal(look(imager, NIGHT, seed=5).temp_c, look(imager, NIGHT, seed=6).temp_c)


def test_the_sky_is_far_colder_than_the_ground_and_colder_looking_up_than_near_the_horizon(scene):
    _, _, _, imager = scene
    frame = look(imager, MIDDAY, pitch=-20.0)  # tipped up
    sky = ~frame.painted
    assert sky.sum() > 1000
    assert frame.temp_c[sky].mean() < np.median(frame.temp_c[frame.ground]) - 20.0 if frame.ground.any() else True
    top, bottom = frame.temp_c[:20][sky[:20]].mean(), frame.temp_c[-20:][sky[-20:]].mean()
    assert top < bottom, "the zenith is colder than the horizon"


def test_a_scout_sees_finer_ground_than_the_rover_does_at_the_same_range():
    assert ts.SCOUT_THERMAL.width < ts.ROVER_THERMAL.width
    assert ts.SCOUT_THERMAL.ifov_rad > ts.ROVER_THERMAL.ifov_rad, "a coarser pixel, but it is looking down from above"
    assert ts.ROVER_THERMAL.netd_k == ts.SCOUT_THERMAL.netd_k == 0.050
