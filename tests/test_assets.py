"""Models and textures come from outside, so they are read as untrusted data."""

import json
import shutil

import numpy as np
import pytest

from ocu.render import assets

needs_assets = pytest.mark.skipif(not assets.available(), reason="assets not downloaded")


def test_simplifying_a_mesh_keeps_its_shape_and_its_texture_coordinates():
    # A subdivided plane with a texture laid across it.
    n = 40
    xs, ys = np.meshgrid(np.linspace(0, 1, n), np.linspace(0, 1, n))
    positions = np.stack([xs.ravel(), ys.ravel(), np.zeros(n * n)], axis=1).astype(np.float32)
    normals = np.tile(np.array([0, 0, 1], dtype=np.float32), (n * n, 1))
    uvs = positions[:, :2].copy()
    idx = np.arange(n * n).reshape(n, n)
    tris = np.concatenate([np.stack([idx[:-1, :-1], idx[1:, :-1], idx[1:, 1:]], -1).reshape(-1, 3),
                           np.stack([idx[:-1, :-1], idx[1:, 1:], idx[:-1, 1:]], -1).reshape(-1, 3)])
    mesh = assets.Mesh(positions, normals, uvs, tris.reshape(-1).astype(np.uint32))
    out = assets.simplify(mesh, 200)
    assert out.triangles < mesh.triangles / 3
    assert np.allclose(out.positions[:, :2], out.uvs, atol=1e-4), "the texture still lies where it did"


def test_loose_triangles_are_welded_so_they_can_be_simplified():
    positions = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], dtype=np.float32)
    mesh = assets.Mesh(positions, np.tile(np.array([0, 0, 1], dtype=np.float32), (6, 1)), positions[:, :2].copy(), np.arange(6, dtype=np.uint32))
    welded = assets.weld(mesh)
    assert len(welded.positions) == 4 and welded.triangles == 2


@needs_assets
def test_every_model_in_the_plan_loads_with_a_near_and_a_far_mesh():
    for name in assets.PLAN:
        model = assets.load_model(name)
        assert model is not None, name
        for variant in model.variants:
            near, far = variant.lods
            assert far.triangles <= near.triangles
            assert near.triangles <= 7000, "cheap enough to instance"
            assert variant.height > 0.0 and min(variant.half_extent) > 0.0
            assert near.positions[:, 2].min() >= -variant.height  # rests on the ground


@needs_assets
def test_a_model_whose_buffer_is_cut_short_is_refused(tmp_path, monkeypatch):
    source = assets.ROOT / "models" / "namaqualand_boulder_03"
    copy = tmp_path / "models" / "namaqualand_boulder_03"
    shutil.copytree(source, copy)
    binary = copy / "namaqualand_boulder_03.bin"
    binary.write_bytes(binary.read_bytes()[:1000])
    monkeypatch.setattr(assets, "ROOT", tmp_path)
    assert assets.load_model("namaqualand_boulder_03") is None


@needs_assets
def test_a_model_that_points_outside_its_folder_is_refused(tmp_path, monkeypatch):
    source = assets.ROOT / "models" / "namaqualand_boulder_03"
    copy = tmp_path / "models" / "namaqualand_boulder_03"
    shutil.copytree(source, copy)
    gltf = copy / "namaqualand_boulder_03_1k.gltf"
    data = json.loads(gltf.read_text())
    data["buffers"][0]["uri"] = "../../../../etc/hosts"
    gltf.write_text(json.dumps(data))
    monkeypatch.setattr(assets, "ROOT", tmp_path)
    assert assets.load_model("namaqualand_boulder_03") is None


@needs_assets
def test_the_ground_textures_load_as_a_matching_set():
    ground = assets.load_ground_textures()
    assert ground is not None and set(ground) == {"ground", "scrub", "track", "dark", "slope"}
    sizes = {t.albedo.shape for t in ground.values()} | {t.normal.shape for t in ground.values()}
    assert len(sizes) == 1, "they go into one array, so they are one size"


def test_a_missing_model_is_not_an_error(tmp_path, monkeypatch):
    monkeypatch.setattr(assets, "ROOT", tmp_path)
    assert assets.load_model("namaqualand_boulder_03") is None
    assert assets.load_ground_textures() is None
    assert not assets.available()


@needs_assets
def test_the_second_load_comes_from_the_cache_and_is_the_same_model():
    import time

    name = "namaqualand_boulder_03"
    assets._cache_path(name).unlink(missing_ok=True)
    first = assets.load_model(name)
    assert assets._cache_path(name).exists()
    start = time.time()
    second = assets.load_model(name)
    assert time.time() - start < 0.5
    for a, b in zip(first.variants, second.variants):
        assert a.half_extent == b.half_extent and a.height == pytest.approx(b.height)
        for la, lb in zip(a.lods, b.lods):
            assert np.array_equal(la.indices, lb.indices) and np.allclose(la.positions, lb.positions)


@needs_assets
def test_a_changed_plan_does_not_reuse_a_stale_cache(monkeypatch):
    name = "namaqualand_boulder_03"
    assets.load_model(name)
    plan = dict(assets.PLAN)
    plan[name] = ("rock", 2000, 400, 0.10, 1)
    monkeypatch.setattr(assets, "PLAN", plan)
    model = assets.load_model(name)
    assert model.variants[0].lods[0].triangles <= 2100
