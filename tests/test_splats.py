"""Gaussian splat scenes: the file, the cameras, the drawing and the route (ocu/render/splats.py, trail.py).

Everything is built here from numbers; no reconstruction is needed to run these."""

import struct
from pathlib import Path

import numpy as np
import pytest

from ocu.render import splats, trail

PROPS = ["x", "y", "z", "f_dc_0", "f_dc_1", "f_dc_2"] + [f"f_rest_{i}" for i in range(24)] + ["opacity", "scale_0", "scale_1", "scale_2", "rot_0", "rot_1", "rot_2", "rot_3"]


def write_ply(path: Path, rows: list[dict]) -> None:
    header = ["ply", "format binary_little_endian 1.0", f"element vertex {len(rows)}"] + [f"property float {p}" for p in PROPS] + ["end_header"]
    with open(path, "wb") as f:
        f.write(("\n".join(header) + "\n").encode())
        for r in rows:
            f.write(struct.pack("<" + "f" * len(PROPS), *[r.get(p, 0.0) for p in PROPS]))


def one_splat(**kw):
    base = {"x": 0.0, "y": 0.0, "z": 3.0, "f_dc_0": 1.0, "f_dc_1": 0.0, "f_dc_2": -1.0, "opacity": 6.0,
            "scale_0": np.log(0.2), "scale_1": np.log(0.2), "scale_2": np.log(0.2), "rot_0": 1.0}
    base.update(kw)
    return base


def test_a_splat_file_is_read_with_its_opacity_and_scale_through_their_activations(tmp_path):
    write_ply(tmp_path / "s.ply", [one_splat(opacity=0.0)])
    cloud = splats.load_ply(tmp_path / "s.ply", min_opacity=0.0)
    assert len(cloud) == 1
    assert cloud.opacity[0] == pytest.approx(0.5)
    assert cloud.covariance[0, 0] == pytest.approx(0.04, rel=1e-4) and cloud.covariance[0, 1] == pytest.approx(0.0, abs=1e-7)


def test_faint_splats_are_dropped_on_load(tmp_path):
    write_ply(tmp_path / "s.ply", [one_splat(opacity=-9.0), one_splat(opacity=2.0)])
    assert len(splats.load_ply(tmp_path / "s.ply")) == 1


def test_a_file_that_is_not_what_its_header_says_is_refused(tmp_path):
    write_ply(tmp_path / "s.ply", [one_splat()])
    data = (tmp_path / "s.ply").read_bytes()
    (tmp_path / "short.ply").write_bytes(data[:-10])
    with pytest.raises(ValueError):
        splats.load_ply(tmp_path / "short.ply")
    (tmp_path / "text.ply").write_bytes(b"not a ply\n")
    with pytest.raises(ValueError):
        splats.load_ply(tmp_path / "text.ply")


def test_colmap_cameras_are_read_and_the_centre_is_where_the_camera_is(tmp_path):
    (tmp_path / "cameras.txt").write_text("# c\n1 SIMPLE_RADIAL 1280 720 643.4 640 360 0.0004\n")
    # camera at world (2, 0, 0) looking down +x of the world: rotation about y by +90 degrees maps world x to camera z
    (tmp_path / "images.txt").write_text("# i\n1 0.7071068 0 0.7071068 0 0 0 2 1 b.jpg\n\n2 1 0 0 0 0 0 0 1 a.jpg\n\n")
    k, poses = splats.read_colmap_text(tmp_path)
    assert (k.width, k.height, k.fx, k.cx) == (1280, 720, 643.4, 640.0)
    assert [p.name for p in poses] == ["a.jpg", "b.jpg"]
    assert np.allclose(poses[0].centre, 0.0)
    assert np.allclose(poses[1].rotation @ np.array([1.0, 0.0, 0.0]) , [0.0, 0.0, -1.0], atol=1e-5) or np.allclose(poses[1].rotation @ np.array([1.0, 0.0, 0.0]), [0.0, 0.0, 1.0], atol=1e-5)


def test_the_route_scale_comes_from_the_walking_pace_and_up_from_the_cameras():
    n = 61
    t = np.arange(n) * 0.1
    centres = np.stack([t * 0.5, np.zeros(n), np.zeros(n)], axis=1)  # a straight walk along x, model units
    # cameras level, looking along +x with y down: rotation world-to-camera
    rot = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])  # x_cam = R x_world: world x -> camera z
    poses = [splats.Pose(rot, -rot @ c) for c in centres]
    rng = np.random.default_rng(0)
    ground = np.stack([rng.uniform(0, 3.0, 4000), np.full(4000, 0.3), rng.uniform(-1.0, 1.0, 4000)], axis=1)  # a floor 0.3 below the cameras (y is down, so +y)
    sand = np.tile([200.0, 165.0, 125.0], (4000, 1))
    route, report = trail.build_route(poses, ground, sand, seconds_per_frame=0.1)
    assert report["path_length_m"] == pytest.approx(trail.WALK_PACE_MS * 6.0, rel=0.02), "scale makes the walk take as long as it did at a walking pace"
    assert report["up"][1] < -0.9, "up is the opposite of the cameras' downward axis"
    assert report["implied_camera_height_m"] == pytest.approx(0.3 * route.scale, rel=0.2)


def test_a_pose_on_the_route_can_be_moved_sideways_and_turned():
    n = 21
    rot = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])
    poses = [splats.Pose(rot, -rot @ np.array([i * 0.1, 0.0, 0.0])) for i in range(n)]
    pts = np.stack([np.linspace(0, 2, 500), np.full(500, 0.3), np.random.default_rng(1).uniform(-0.3, 0.3, 500)], axis=1)
    route, _ = trail.build_route(poses, pts, np.tile([200.0, 165.0, 125.0], (500, 1)), seconds_per_frame=0.1)
    centre = route.pose_at(1.0).centre
    right = route.pose_at(1.0, lateral_m=0.5).centre
    assert np.linalg.norm(right - centre) * route.scale == pytest.approx(0.5, rel=1e-3)
    turned = route.pose_at(1.0, yaw_deg=30.0)
    forward = turned.rotation[2]
    straight = route.pose_at(1.0).rotation[2]
    assert np.degrees(np.arccos(np.clip(forward @ straight, -1, 1))) == pytest.approx(30.0, abs=0.5)


def test_a_splat_in_front_of_the_camera_is_drawn_where_the_camera_says_and_behind_it_is_not(tmp_path):
    moderngl = pytest.importorskip("moderngl")
    try:
        ctx = moderngl.create_standalone_context()
    except Exception:  # no GL here: not a failure
        pytest.skip("no GL context")
    write_ply(tmp_path / "s.ply", [one_splat(x=0.5, z=3.0)])
    cloud = splats.load_ply(tmp_path / "s.ply")
    renderer = splats.SplatRenderer(ctx, cloud)
    k = splats.Intrinsics(320, 240, 300.0, 300.0, 160.0, 120.0)
    fbo = ctx.simple_framebuffer((320, 240), components=4)
    fbo.use()
    fbo.clear(0.0, 0.0, 0.0, 1.0)
    renderer.render(splats.Pose(np.eye(3), np.zeros(3)), k)
    image = np.frombuffer(fbo.read(components=3), dtype=np.uint8).reshape(240, 320, 3)[::-1]
    lit = np.argwhere(image.sum(axis=2) > 60)
    assert len(lit)
    # x = 0.5 at z = 3 is 300 * 0.5 / 3 = 50 pixels right of the centre; y = 0 is the middle row
    assert lit[:, 1].mean() == pytest.approx(160 + 50, abs=3) and lit[:, 0].mean() == pytest.approx(120, abs=3)
    assert image[lit[0][0], lit[0][1], 0] > image[lit[0][0], lit[0][1], 2], "the splat is red-ish (positive red, negative blue coefficient)"
    fbo.clear(0.0, 0.0, 0.0, 1.0)
    renderer.render(splats.Pose(np.diag([1.0, 1.0, -1.0]) @ np.diag([-1.0, 1.0, 1.0]), np.zeros(3)), k)  # turned right round: it is behind
    assert np.frombuffer(fbo.read(components=3), dtype=np.uint8).max() == 0
