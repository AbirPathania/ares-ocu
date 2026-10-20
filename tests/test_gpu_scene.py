"""The 3D renderer. Skipped where there is no GPU, which is not a failure: the
application falls back to the numpy renderer there."""

import numpy as np
import pytest

from ocu.core.terrain import Terrain, TerrainWorld

gpu_scene = pytest.importorskip("ocu.render.gpu_scene")


@pytest.fixture(scope="module")
def scene():
    world = TerrainWorld()
    made = gpu_scene.GpuScene.create(world)
    if made is None:
        pytest.skip("no GL context available")
    made.world = world
    return made


def view(scene, **overrides):
    args = dict(eye=(0.0, 0.0, 1.6), heading_deg=30.0, pitch_down_deg=3.0, hfov_deg=100.0, width=320, height=140)
    args.update(overrides)
    return scene.render(**args)


def test_a_frame_has_the_shape_asked_for(scene):
    frame = view(scene)
    assert frame.shape == (140, 320, 3) and frame.dtype == np.uint8


def test_a_frame_is_a_picture_not_a_flat_colour(scene):
    frame = view(scene)
    assert frame.std() > 20.0
    top, bottom = frame[:20].mean(axis=(0, 1)), frame[-20:].mean(axis=(0, 1))
    assert top[2] > top[0], "the top of a level view is sky, and sky is blue"
    assert not np.allclose(top, bottom, atol=10.0)


def test_the_same_view_twice_is_the_same_picture(scene):
    a, b = view(scene, grain=0.0), view(scene, grain=0.0)
    # Foliage sways with the clock, so allow for that and nothing more.
    assert np.mean(np.abs(a.astype(int) - b.astype(int))) < 2.0


def test_looking_the_other_way_shows_something_else(scene):
    a, b = view(scene, heading_deg=30.0), view(scene, heading_deg=210.0)
    assert np.mean(np.abs(a.astype(int) - b.astype(int))) > 8.0


def test_sensor_noise_is_grain_over_the_picture(scene):
    clean, noisy = view(scene, grain=0.0), view(scene, grain=0.04)
    assert np.mean(np.abs(clean.astype(int) - noisy.astype(int))) > 1.5


def test_a_scout_sees_the_rover_it_is_looking_at(scene):
    # Straight over a rover: the frame differs from the same frame without it.
    kwargs = dict(eye=(0.0, 0.0, 14.0), heading_deg=0.0, pitch_down_deg=89.0, hfov_deg=60.0, width=200, height=200)
    with_rover = scene.render(rover=(0.0, 0.0, 0.0), **kwargs)
    without = scene.render(**kwargs)
    middle = (slice(93, 107), slice(95, 105))
    assert np.mean(np.abs(with_rover[middle].astype(int) - without[middle].astype(int))) > 10.0


def test_a_rock_is_drawn_where_the_map_says_there_is_one(scene):
    """Look straight down at a boulder: its footprint is grey on the picture."""
    world = scene.world
    x, y, a, b, theta, top = world.boulders[int(np.argmax(world.boulders[:, 5]))]
    frame = scene.render(
        eye=(x, y, world._elevation[world._extent + int(x / world.cell_size_m), world._extent + int(y / world.cell_size_m)] + 9.0),
        heading_deg=0.0, pitch_down_deg=89.0, hfov_deg=50.0, width=200, height=200,
    )
    centre = frame[90:110, 90:110].astype(int)
    spread = centre.max(axis=2) - centre.min(axis=2)
    assert spread.mean() < 26.0, "stone is grey; the ground around it is not"


def test_frames_are_fast_enough_for_a_live_feed(scene):
    import time

    view(scene)
    start = time.time()
    for _ in range(10):
        view(scene, width=480, height=210)
    assert (time.time() - start) / 10 < 0.05
