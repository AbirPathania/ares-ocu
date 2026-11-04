"""What is built from the reference footage: orientation, colour, tiling, and the scenery that follows from it.

None of this reads the raw video (it is not in the repository); the tests build what they need, or use what the footage
scripts have already written under assets/footage and ocu/data.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

import footage_ingest as fi  # noqa: E402
import footage_textures as ft  # noqa: E402

from ocu.render import assets, meshes, palette  # noqa: E402


# --- which way up -----------------------------------------------------------------------------------------------------

def test_a_phone_turned_over_stays_turned_over_through_frames_with_no_sky():
    scores = [0.20, 0.15, -0.40, -0.35, 0.00, 0.01, -0.02, 0.00, 0.25, 0.30]
    assert fi.uprightness(scores) == [True, True, False, False, False, False, False, False, True, True]


def test_one_blue_shadow_does_not_turn_the_picture_over():
    scores = [0.10, 0.12, -0.23, 0.01, 0.00, 0.11]
    assert all(fi.uprightness(scores)), "a single weak reading of sky in the wrong place is a sign or a shadow, not the phone"


def test_sky_score_is_positive_when_the_sky_is_on_top():
    frame = np.zeros((120, 160, 3), dtype=np.uint8)
    frame[:30] = (150, 190, 235)
    frame[30:] = (150, 120, 80)
    assert fi.sky_score(frame) > 0.5
    assert fi.sky_score(frame[::-1]) < -0.5


def test_hlg_mid_grey_and_white_come_out_in_order_and_in_range():
    signal = np.array([[0.0, 0.0, 0.0], [0.4, 0.4, 0.4], [0.75, 0.75, 0.75], [1.0, 1.0, 1.0]], dtype=np.float32)
    out = fi.linear_to_srgb8(fi.hlg_to_linear(signal))
    luminance = out.astype(int).sum(axis=1)
    assert (np.diff(luminance) > 0).all() and out.max() <= 255 and out[0].max() == 0


# --- textures ---------------------------------------------------------------------------------------------------------

def test_a_tile_has_no_seam_where_it_repeats():
    rng = np.random.default_rng(3)
    grain = np.stack([np.abs(rng.normal(0.4, 0.15, (256, 256)))] * 3, axis=2).astype(np.float32)
    grain += np.linspace(0.0, 0.3, 256)[None, :, None]  # a gradient the patch carries: the worst case for a seam
    tile = ft.make_tileable(grain)
    across = np.abs(tile[:, 0] - tile[:, -1]).mean()
    within = np.abs(tile[:, 100] - tile[:, 101]).mean()
    assert across < within * 2.5, "the edge meets the opposite edge no worse than neighbouring pixels meet each other"
    assert tile.std() > 0.8 * grain.std(), "the join keeps the grain of the rest"


def test_a_normal_map_points_up_on_flat_ground_and_tilts_on_a_slope():
    flat = ft.normal_map(np.full((64, 64), 0.5, dtype=np.float32), 2.0)
    assert abs(int(flat[..., 2].mean()) - 255) < 4 and abs(int(flat[..., 0].mean()) - 128) < 3
    bumpy = ft.normal_map(np.random.default_rng(1).random((64, 64)).astype(np.float32), 2.0)
    assert bumpy[..., 2].mean() < flat[..., 2].mean()


@pytest.mark.skipif(not assets.footage_ground_available(), reason="the footage textures have not been made")
def test_the_footage_ground_has_every_role_at_the_size_the_renderer_expects():
    ground = assets.load_footage_ground()
    assert set(ground) == set(assets.GROUND_ROLES)
    for textures in ground.values():
        assert textures.albedo.shape == (1024, 1024, 3) and textures.normal.shape == (1024, 1024, 3)
    track = ground["track"].albedo.reshape(-1, 3).mean(axis=0)
    assert track[0] > track[1] > track[2], "sand is red above green above blue"


# --- colour -----------------------------------------------------------------------------------------------------------

def finish(linear: np.ndarray, exposure: float = palette.EXPOSURE) -> np.ndarray:
    """The ending of shaders.finish(), in numpy."""
    c = palette._aces(linear * exposure)
    lum = float(c @ np.array([0.299, 0.587, 0.114]))
    c = lum + (c - lum) * palette.SATURATION
    return np.power(np.clip(c, 0.0, 1.0), 1 / 2.2) * 255.0


@pytest.mark.parametrize("display", [(178, 188, 195), (142, 185, 218), (82, 130, 204), (199, 167, 130)])
def test_a_measured_colour_run_backwards_and_forwards_comes_back(display):
    scene = palette.scene_from_display(display)
    assert np.abs(finish(scene) - np.array(display)).max() < 2.5


def test_the_sky_gets_bluer_and_deeper_with_height():
    points = palette.sky_points()
    blue_minus_red = [c[2] - c[0] for _, c in points]
    assert blue_minus_red[-1] > blue_minus_red[0]
    assert [d for d, _ in points] == sorted(d for d, _ in points)


def test_the_generated_sky_code_names_every_elevation_it_was_given():
    code = palette.sky_glsl()
    assert code.count("smoothstep") == len(palette.sky_points()) - 1 and "asin" in code


# --- scenery ----------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("make", [meshes.acacia_template, meshes.leafy_template, meshes.snag_template, meshes.bush_template, meshes.grass_template])
def test_a_footage_plant_is_a_sound_mesh(make):
    vertices, indices = make(5)
    assert vertices.shape[1] == meshes.VERTEX_FLOATS and np.isfinite(vertices).all()
    assert indices.max() < len(vertices) and len(indices) % 3 == 0 and len(indices) > 30


def test_trees_stand_on_the_origin_and_are_about_as_tall_as_they_say():
    for make in (meshes.acacia_template, meshes.leafy_template, meshes.snag_template):
        vertices, _ = make(11)
        z = vertices[:, 2]
        assert -0.01 <= z.min() < 0.05 and 0.75 < z.max() < 1.35, make.__name__


def test_wood_is_marked_so_that_the_crown_wobble_leaves_it_alone():
    vertices, _ = meshes.acacia_template(2)
    lengths = np.linalg.norm(vertices[:, 3:6], axis=1)
    assert (lengths > 1.5).any() and (lengths < 1.5).any(), "both wood and leaves are in the mesh"


def test_boulders_are_not_orange_once_graded():
    orange = np.zeros((8, 8, 3), dtype=np.uint8)
    orange[...] = (220, 110, 30)  # lichen
    graded = assets.grade_stone(orange).astype(int)
    assert graded[..., 0].mean() - graded[..., 2].mean() < 0.36 * graded[..., 0].mean()


def test_the_species_of_tree_the_world_plants_are_the_three_the_renderer_draws():
    from ocu.core.terrain import TerrainWorld

    species = set(np.unique(TerrainWorld().trees[:, 4]).astype(int))
    assert species <= {0, 1, 2} and len(species) >= 2
