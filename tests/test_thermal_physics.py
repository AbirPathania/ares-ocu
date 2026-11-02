"""The thermal model held to the checks registered in DECISIONS.md before it was built (V1 to V7, V9, V11)."""

import math

import numpy as np
import pytest

import ocu.core.thermal_physics as tp
from ocu.core.camera import CameraRig
from ocu.core.targets import BuriedTarget
from ocu.core.terrain import Terrain, TerrainWorld


# --- V1: heat is conserved -------------------------------------------------------------------------------------------

def test_v1_an_insulated_column_with_no_forcing_conserves_heat_to_a_billionth():
    rng = np.random.default_rng(3)
    k, c = rng.uniform(0.2, 3.0, 40), rng.uniform(0.8e6, 2.4e6, 40)
    solver = tp.ColumnSolver(k, c)
    solver.set_state(rng.uniform(250, 320, solver.temps.shape), rng.uniform(250, 320, solver.cells))
    before = solver.heat_content()
    zeros = np.zeros(40)
    for _ in range(288):  # a day
        solver.step(300.0, zeros, np.full(40, 0.95), 0.0, 280.0, np.full(40, 10.0), zeros, insulated=True)
    drift = np.abs(solver.heat_content() - before) / before
    assert drift.max() < 1e-9, f"heat drifted by {drift.max():.2e}"


# --- V2: conduction against the analytic answer ------------------------------------------------------------------------

def test_v2_a_daily_wave_decays_and_lags_with_depth_as_the_analytic_solution_says():
    k, c = 0.30, 1.3e6
    omega = 2.0 * math.pi / 86400.0
    delta = math.sqrt(2.0 * k / (c * omega))  # damping depth, about 8 cm
    layers = (0.005,) * 80  # 40 cm, five damping depths
    mean, amp, dt = 280.0, 10.0, 60.0
    solver = tp.ColumnSolver(np.full(1, k), np.full(1, c), layers_m=layers, deep_k=mean)
    solver.set_state(np.full_like(solver.temps, mean), np.full(1, mean))
    steps_per_day = int(86400 / dt)
    zero = np.zeros(1)
    series = []
    for step in range(6 * steps_per_day):
        t = (step + 0.5) * dt
        solver.step(dt, zero, zero, 0.0, mean + amp * math.cos(omega * t), np.full(1, 1e9), zero)
        if step >= 5 * steps_per_day:
            series.append(solver.temps[:, 0].copy())
    series = np.array(series)  # (steps, layers)
    t = (np.arange(len(series)) + 5 * steps_per_day + 1.0) * dt
    z = np.cumsum(layers) - np.array(layers) / 2.0
    j = int(np.argmin(np.abs(z - delta)))
    basis = np.stack([np.cos(omega * t), np.sin(omega * t)], axis=1)
    a, b = np.linalg.lstsq(basis, series[:, j] - series[:, j].mean(), rcond=None)[0]
    amplitude, lag = math.hypot(a, b), math.atan2(b, a)
    expected = amp * math.exp(-z[j] / delta)
    assert abs(amplitude - expected) / expected < 0.05, f"amplitude {amplitude:.3f} against {expected:.3f}"
    assert abs(lag - z[j] / delta) < 0.1, f"lag {lag:.3f} rad against {z[j] / delta:.3f}"


# --- V3: clear-sky sunlight ----------------------------------------------------------------------------------------------

def test_v3_clear_sky_sunlight_at_altitude():
    site = tp.ThermalSite()
    beam, diffuse = site.sunlight(math.radians(50.0))
    assert 800.0 <= beam * math.sin(math.radians(50.0)) + diffuse <= 1000.0
    assert site.sunlight(math.radians(-10.0)) == (0.0, 0.0)
    assert site.sunlight(math.radians(30.0))[0] < site.sunlight(math.radians(50.0))[0], "the beam falls as the sun drops"


def test_the_sun_the_visible_camera_draws_is_where_this_date_puts_it_at_the_start_hour():
    from ocu.core.camera import SUN_AZIMUTH_DEG, SUN_ELEVATION_DEG

    site = tp.ThermalSite()
    el, az = site.sun(site.start_hour)
    assert abs(math.degrees(el) - SUN_ELEVATION_DEG) < 3.0
    assert abs(math.degrees(az) - SUN_AZIMUTH_DEG) < 8.0


# --- the day's field -------------------------------------------------------------------------------------------------------

@pytest.fixture(scope="module")
def day():
    world = TerrainWorld.blank(half_extent_m=30.0)
    for cx in range(10, 14):
        for cy in range(10, 14):
            world.set_cell((cx, cy), Terrain.ROCK)  # a 2 m boulder block at about (6, 6)
    rig = CameraRig(world)
    targets = [BuriedTarget(-10.0, -10.0, 0.03, 100.0, True, 0.3, "shallow"), BuriedTarget(-10.0, -14.0, 0.30, 100.0, True, 0.3, "deep")]
    return world, rig, tp.build_field(rig, targets), targets


def cell_of(world, x, y):
    return int(x / world.cell_size_m) + world._extent, int(y / world.cell_size_m) + world._extent


def series(field, ij):
    return np.array([field.skin_c[f][ij] for f in range(tp.FRAMES_PER_DAY)], dtype=np.float64)


def hours():
    return np.arange(tp.FRAMES_PER_DAY) * tp.FRAME_S / 3600.0


# --- V4 -------------------------------------------------------------------------------------------------------------------

def test_v4_dry_ground_peaks_after_noon_and_cools_below_the_air_before_dawn(day):
    world, _, field, _ = day
    s = series(field, cell_of(world, -20.0, 20.0))
    h = hours()
    peak_hour = h[int(np.argmax(s))]
    assert 12.0 <= peak_hour <= 14.5 and 25.0 <= s.max() <= 45.0, f"peak {s.max():.1f} C at {peak_hour:.1f} h"
    air_min = min(field.site.air_c(x) for x in h)
    pre_dawn = s[(h >= 3.0) & (h <= 6.5)].min()
    assert 3.0 <= air_min - pre_dawn <= 9.0, f"{air_min - pre_dawn:.1f} K below the air minimum"


# --- V5 -------------------------------------------------------------------------------------------------------------------

def test_v5_rock_is_cooler_by_day_and_warmer_by_night_and_crosses_soil_twice(day):
    world, _, field, _ = day
    soil, rock = series(field, cell_of(world, -20.0, 20.0)), series(field, cell_of(world, 6.0, 6.0))
    h = hours()
    assert soil.max() - rock.max() >= 3.0, f"rock's peak is only {soil.max() - rock.max():.1f} K below soil's"
    night = (h >= 2.0) & (h <= 5.0)
    assert (rock[night] - soil[night]).mean() >= 2.0
    difference = rock - soil
    crossings = int(np.sum(np.sign(difference[1:]) != np.sign(difference[:-1])))
    assert crossings == 2, f"{crossings} crossings"


# --- V6 -------------------------------------------------------------------------------------------------------------------

def test_v6_a_cell_in_a_boulders_shadow_is_much_cooler_than_a_lit_one_in_the_late_morning(day):
    world, rig, field, _ = day
    relief = rig._relief
    standing = rig._elevation + np.maximum(np.maximum(relief[..., 1], relief[..., 2]), relief[..., 3])
    masks = [tp.shadow_mask(standing, world.cell_size_m, *field.site.sun(10.0 + m / 60.0)) for m in (0, 20, 40)]
    shaded_all = masks[0] & masks[1] & masks[2]
    never = ~(masks[0] | masks[1] | masks[2])
    cx, cy = cell_of(world, 6.0, 6.0)
    near = np.zeros_like(shaded_all)
    near[cx - 14 : cx + 15, cy - 14 : cy + 15] = True
    shadow_cells = np.argwhere(shaded_all & near)
    assert len(shadow_cells) >= 3, "the boulder casts a shadow"
    lit_cells = np.argwhere(never & near)
    frame = field.skin_at(10.0 + 40.0 / 60.0)
    shaded_t = np.mean([frame[tuple(c)] for c in shadow_cells])
    lit_t = np.mean([frame[tuple(c)] for c in lit_cells if not _is_rock(world, c)])
    assert lit_t - shaded_t >= 5.0, f"the shadow is only {lit_t - shaded_t:.1f} K cooler"


def _is_rock(world, c):
    return world._rock_height[tuple(c)] > 0.1


# --- V7 -------------------------------------------------------------------------------------------------------------------

def test_v7_a_shallow_disturbed_patch_is_faintly_visible_and_changes_sign_twice_a_day(day):
    _, _, field, _ = day
    delta = field.target_c[:, 0] - field.host_c[:, 0]
    assert 0.5 <= np.abs(delta).max() <= 4.0, f"largest difference {np.abs(delta).max():.2f} K"
    crossings = int(np.sum(np.sign(delta[1:]) != np.sign(delta[:-1])))
    assert crossings == 2, f"{crossings} crossings"


def test_v7_refuted_a_device_30_cm_down_is_not_invisible_while_its_backfill_is_fresh(day):
    """DECISIONS.md predicted under 0.15 K at 30 cm. It is refuted, and the prediction was the mistake: the hole dug to
    bury something at 30 cm is backfilled with loosened soil all the way to the surface, so the top of the column, which is
    what the surface feels, is the same as over a shallow device. What hides a deep device from a thermal camera is time
    (rain compacts the fill), not depth."""
    _, _, field, _ = day
    deep = np.abs(field.target_c[:, 1] - field.host_c[:, 1]).max()
    shallow = np.abs(field.target_c[:, 0] - field.host_c[:, 0]).max()
    assert deep > 0.15, "the registered claim (under 0.15 K) does not hold"
    assert deep == pytest.approx(shallow, rel=0.25), "it is about as visible as a shallow one while the soil is fresh"


def test_the_column_over_a_deep_device_has_loosened_fill_to_the_surface_and_the_body_low_down():
    base = tp.MATERIALS["soil"]
    k, c = tp.target_layers(0.30, base)
    assert k[0] == pytest.approx(base.conductivity * tp.DISTURBED_CONDUCTIVITY), "fill reaches the surface"
    assert c[-1] > base.heat_capacity, "and the body, heavier than soil, sits in the lower layers"
    k_shallow, _ = tp.target_layers(0.03, base)
    assert k_shallow[0] == pytest.approx(base.conductivity * tp.DISTURBED_CONDUCTIVITY) and k_shallow[-1] == pytest.approx(base.conductivity), "a shallow device leaves the deep ground alone"


# --- V9: Planck -------------------------------------------------------------------------------------------------------------

def test_v9_the_radiance_table_inverts_to_a_millikelvin():
    r = tp.RADIOMETER
    t = np.linspace(200.0, 400.0, 801)
    assert np.abs(r.temperature(r.radiance(t)) - t).max() < 1e-3
    assert np.all(np.diff(r.radiance(t)) > 0), "brighter when hotter"


def test_v9_integrated_over_every_wavelength_it_gives_stefan_boltzmann():
    for t in (260.0, 300.0, 340.0):
        full = tp._planck_band_radiance(np.array([t]), band_um=(0.5, 4000.0), samples=800_000)[0]
        assert abs(full - tp.STEFAN_BOLTZMANN * t**4 / math.pi) / (tp.STEFAN_BOLTZMANN * t**4 / math.pi) < 0.005


def test_a_less_emissive_surface_that_reflects_the_cold_sky_reads_colder_than_it_is():
    r = tp.RADIOMETER
    apparent = float(r.apparent_k(300.0, 0.85, 240.0))
    assert 280.0 < apparent < 297.0, "a bright metal roof under a clear sky"
    assert float(r.apparent_k(300.0, 1.0, 240.0)) == pytest.approx(300.0, abs=2e-3)


# --- V11 ------------------------------------------------------------------------------------------------------------------

def test_v11_the_atmosphere_is_nearly_transparent_over_a_hundred_metres():
    assert float(tp.transmission(100.0)) > 0.97
    assert float(tp.transmission(100.0)) > float(tp.transmission(1000.0))
