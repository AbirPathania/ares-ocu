"""The 3D renderer. Skipped where there is no GPU, which is not a failure: the
application falls back to the numpy renderer there."""

import math

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
    centre = frame[90:110, 90:110].reshape(-1, 3).mean(axis=0)
    ring = np.concatenate([frame[:15].reshape(-1, 3), frame[-15:].reshape(-1, 3)]).mean(axis=0)
    assert np.abs(centre - ring).max() > 12.0, "a boulder is there, and it is not the colour of the ground round it"


def test_frames_are_fast_enough_for_a_live_feed(scene):
    import time

    view(scene)
    start = time.time()
    for _ in range(10):
        view(scene, width=480, height=210)
    assert (time.time() - start) / 10 < 0.05


# --- the downloaded models and textures ---------------------------------------


def test_the_scene_still_draws_without_the_downloaded_assets():
    made = gpu_scene.GpuScene.create(TerrainWorld(), use_assets=False)
    if made is None:
        pytest.skip("no GL context available")
    assert made._pbr_groups == []
    frame = made.render((0.0, 0.0, 1.6), 30.0, 3.0, 100.0, 200, 90)
    assert frame.shape == (90, 200, 3) and frame.std() > 15.0


def test_with_the_assets_the_rocks_are_the_photographed_ones(scene):
    from ocu.render import assets

    if not assets.available():
        pytest.skip("assets not downloaded (scripts/fetch_assets.py)")
    names = {g["name"] for g in scene._pbr_groups}
    assert any("boulder" in n for n in names)
    assert scene._pbr_groups, "the textured meshes are in the scene"
    # Every boulder the world has is drawn once, as a downloaded model.
    drawn = sum(len(g["rows"]) for g in scene._pbr_groups if g["mode"] == 0)
    assert drawn == len(scene.world.boulders)


# ---- people, vehicles and animals

def _changed_box(a, b, threshold=40, support=3):
    """The core of what changed between two frames: columns and rows holding at least `support` changed pixels,
    so the few stray pixels the shadow pass moves far away do not stretch the box."""
    diff = np.abs(a.astype(int) - b.astype(int)).sum(axis=2) > threshold
    cols, rows = np.where(diff.sum(axis=0) >= support)[0], np.where(diff.sum(axis=1) >= support)[0]
    if not len(cols) or not len(rows):
        return None
    return cols.min(), rows.min(), cols.max(), rows.max()


def _flat_site(scene):
    """The flattest open ground within 40 m of the deploy point, away from rock and trees: a fair place to stand a figure."""
    world, best = scene.world, None
    for cx in range(-40, 41, 2):
        for cy in range(-40, 41, 2):
            if math.hypot(cx, cy) < 14:
                continue
            ground = [world.elevation_at(world.to_cell(cx + dx, cy + dy)) for dx in (-14, -7, 0, 7) for dy in (-3, 0, 3)]
            clear = all(world.at(world.to_cell(cx + dx, cy + dy)).name in ("GROUND", "TRACK") for dx in range(-14, 9, 2) for dy in range(-6, 7, 2))
            if clear and (best is None or max(ground) - min(ground) < best[0]):
                best = (max(ground) - min(ground), cx, cy)
    return best[1], best[2]


@pytest.mark.parametrize("name,distance", [("walker", 7.0), ("armed walker", 7.0), ("parked truck", 12.0), ("buck", 6.0), ("ibex", 6.0)])
def test_an_entity_is_drawn_where_the_detector_model_says_its_box_is(scene, monkeypatch, name, distance):
    """The simulated detector's box (core/vision.py) and the picture have to agree, or the overlay lies."""
    from ocu.core import vision as pc
    from ocu.core.entities import SurfaceEntity, default_entities

    kind = next(e for e in default_entities() if e.name == name).kind
    monkeypatch.setattr(gpu_scene.time, "monotonic", lambda: 123.0)  # wind sways the grass; hold the clock still
    w, h = 640, 360
    sx, sy = _flat_site(scene)
    ground = lambda x, y: float(scene._ground(np.array([x]), np.array([y]))[0])  # noqa: E731
    # the camera stands `distance` south of the figure, looking north along the flat strip
    ex, ey = sx, sy
    cx, cy = ex, ey - distance
    eye_z = ground(cx, cy) + 1.5
    args = dict(eye=(cx, cy, eye_z), heading_deg=0.0, pitch_down_deg=3.0, hfov_deg=112.0, width=w, height=h)
    with scene.ctx:  # an earlier test in this file makes a second GL context; make this scene's the current one
        empty = scene.render(**args)
        from ocu.core.entities import default_entities as _defaults

        style = next(e for e in _defaults() if e.name == name).style
        drawn = scene.render(**args, entities=[(kind, ex, ey, 0.0, 0.0, style)])
    box = _changed_box(empty, drawn)
    assert box is not None, "the entity must change the picture"
    cam = pc.CameraSpec(cx, cy, eye_z, 0.0, 3.0, 112.0, w, h, aerial=False)

    class Rig:
        def elevation_at(self, x, y):
            return ground(x, y)

        def line_of_sight(self, *a):
            return True

    proto = next(e for e in default_entities() if e.name == name)
    ent = SurfaceEntity(kind, "x", proto.height_m, proto.width_m, proto.length_m, ((ex, ey),), box_offset_m=proto.box_offset_m,
                        carrying=proto.carrying, style=proto.style)
    ap = pc.appearance(cam, ent, (ex, ey, 0.0), Rig())
    assert ap is not None
    model_h = (ap.box[3] - ap.box[1]) * h
    drawn_h = box[3] - box[1] + 1
    assert drawn_h == pytest.approx(model_h, rel=0.35, abs=6), f"{kind}: drawn {drawn_h}px tall, model says {model_h:.0f}px"
    assert (box[0] + box[2]) / 2 / w == pytest.approx((ap.box[0] + ap.box[2]) / 2, abs=0.04)
    assert box[3] / h == pytest.approx(ap.box[3], abs=0.07), "its feet are where the model puts the bottom of the box"
