import numpy as np
import pytest

import ocu.core.thermal_physics as tp
import ocu.core.thermal_scene as ts
from ocu.core.camera import CameraRig
from ocu.core.entities import SurfaceEntity
from ocu.core.targets import BuriedTarget
from ocu.core.terrain import TerrainWorld
from ocu.core.thermal_system import ThermalSystem


@pytest.fixture(scope="module")
def system():
    rig = CameraRig(TerrainWorld.blank(half_extent_m=40.0))
    s = ThermalSystem(rig, [BuriedTarget(0.0, 10.0, 0.03, 100.0, True, 0.3, "x")], cache_dir=None)
    s.build()
    return s


def test_it_is_not_ready_until_the_day_is_solved():
    s = ThermalSystem(CameraRig(TerrainWorld.blank(half_extent_m=20.0)), cache_dir=None)
    assert not s.ready
    s.build()
    assert s.ready and s.progress == 1.0


def test_a_stored_day_is_loaded_instead_of_solved_again(tmp_path):
    rig = CameraRig(TerrainWorld.blank(half_extent_m=20.0))
    first = ThermalSystem(rig, cache_dir=tmp_path)
    first.build()
    assert len(list(tmp_path.glob("thermal_*.npz"))) == 1
    second = ThermalSystem(rig, cache_dir=tmp_path)
    second.build()
    assert np.array_equal(first.field.skin_c, second.field.skin_c)


def test_a_different_world_does_not_load_another_worlds_day(tmp_path):
    a = ThermalSystem(CameraRig(TerrainWorld.blank(half_extent_m=20.0)), cache_dir=tmp_path)
    a.build()
    b = ThermalSystem(CameraRig(TerrainWorld.blank(half_extent_m=24.0)), cache_dir=tmp_path)
    b.build()
    assert len(list(tmp_path.glob("thermal_*.npz"))) == 2


def test_the_mission_starts_at_the_sites_hour_and_the_operator_can_move_it(system):
    system.hour_offset_h = 0.0
    assert system.local_hour(0.0) == pytest.approx(system.site.start_hour)
    assert system.local_hour(1800.0) == pytest.approx(system.site.start_hour + 0.5)
    system.shift_hours(-8.0)
    assert system.local_hour(0.0) == pytest.approx((system.site.start_hour - 8.0) % 24.0)
    system.hour_offset_h = 0.0


def test_observing_writes_what_the_rover_sees_into_the_mosaic_and_not_what_it_does_not(system):
    system.map.clear()
    system.observe("rover", 0.0, 0.0, 0.0, 0.0, 12.0)
    assert 0.0 < system.map.seen_fraction < 0.05
    cell_behind = system.map._index(0.0, -20.0)
    assert np.isnan(system.map.temp_c[cell_behind]), "nothing was seen behind it"
    cell_ahead = np.argwhere(np.isfinite(system.map.temp_c))
    assert (cell_ahead[:, 1].mean() * system.map.cell_m - system.map.half_extent_m) > 0.0, "what was seen is ahead of it"


def test_a_body_in_view_is_a_heat_contact_and_not_in_the_mosaic(system):
    system.map.clear()
    walker = SurfaceEntity("person", "w", 1.78, 0.54, 0.63, ((1.0, 8.0),), heading_deg=180.0)
    system.hour_offset_h = 5.0  # a cool morning, so a person stands well out of the ground
    system.observe("rover", 0.0, 0.0, 0.0, 0.0, 12.0, [walker], [(1.0, 8.0, 180.0)])
    system.hour_offset_h = 0.0
    assert any(c.kind == "person" for c in system.map.contacts)


def test_the_legend_range_is_steady_and_held_levels_stay_held(system):
    system.map.clear()
    system.observe("rover", 0.0, 0.0, 0.0, 0.0, 12.0)
    first = system.level_range()
    system.observe("rover", 0.5, 0.0, 0.0, 0.0, 13.0)
    assert system.level_range() == first or max(abs(a - b) for a, b in zip(system.level_range(), first, strict=True)) > 1.0, "it does not jitter"
    assert system.toggle_hold() is True and system.hold_range == first
    system.map.clear()
    assert system.level_range() == first, "held levels do not follow the picture"
    assert system.toggle_hold() is False and system.hold_range is None


def test_the_palette_cycles_through_all_three(system):
    seen = {system.palette}
    for _ in range(3):
        seen.add(system.cycle_palette())
    assert seen == {"white_hot", "black_hot", "iron"}


def test_the_mast_camera_is_binned_for_mapping_and_the_noise_goes_down_with_it():
    binned = ts.ROVER_THERMAL.binned(2)
    assert (binned.width, binned.height) == (160, 128) and binned.hfov_deg == ts.ROVER_THERMAL.hfov_deg
    assert binned.netd_k == pytest.approx(ts.ROVER_THERMAL.netd_k / 2.0)
