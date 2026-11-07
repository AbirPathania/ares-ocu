"""What lives in the world beyond the track (core/population.py)."""

import math
from collections import Counter

import numpy as np
import pytest

from ocu.core import population, scenery
from ocu.core.entities import SurfaceEntity
from ocu.core.terrain import Terrain, TerrainWorld
from ocu.core.trail_scenario import CLEAR_ZONE_M
from tests.test_trail_scenario import chain


@pytest.fixture(scope="module")
def world_and_population():
    trail = chain(count=8, length_m=70.0, growth_m=6.0)
    world = TerrainWorld()
    trail.carve(world)
    scenery.dress(world, protect=trail.distances, protect_m=CLEAR_ZONE_M + 0.3)
    return trail, world, population.populate(world, trail)


def test_the_three_sites_are_well_off_the_track_and_apart_from_each_other(world_and_population):
    trail, _, pop = world_and_population
    assert set(pop.sites) == {"B", "C", "D"}
    centres = [s.centre for s in pop.sites.values()]
    for x, y in centres:
        assert trail.distance_to_line(x, y) >= population.MIN_FROM_TRACK_M
    for i, a in enumerate(centres):
        for b in centres[i + 1:]:
            assert math.dist(a, b) > 25.0


def test_the_old_sims_people_animal_and_trucks_are_there_and_a_truck_really_drives(world_and_population):
    _, _, pop = world_and_population
    kinds = Counter(e.kind for e in pop.entities)
    assert kinds["person"] >= 3 and kinds["animal"] >= 2 and kinds["vehicle"] == 2
    assert any(e.style == "armed" for e in pop.entities), "someone armed, as at the crossing"
    names = [e.name for e in pop.entities]
    assert len(names) == len(set(names)), "unique names: the window keys things by them"
    parked = [e for e in pop.entities if e.kind == "vehicle" and e.static]
    driving = [e for e in pop.entities if e.kind == "vehicle" and not e.static]
    assert len(parked) == 1 and len(driving) == 1
    x0, y0, _ = driving[0].pose_at(0.0)
    x1, y1, _ = driving[0].pose_at(10.0)
    assert math.hypot(x1 - x0, y1 - y0) > 10.0, "it moves"


def test_the_truck_drives_on_a_road_that_has_been_cleared_for_it(world_and_population):
    trail, world, pop = world_and_population
    truck = next(e for e in pop.entities if e.name == "patrol truck")
    from ocu.core.terrain import IMPASSABLE

    for x, y in truck.path:
        assert trail.distance_to_line(x, y) > 10.0, "off the footpath"
        assert world.classify(x, y) == Terrain.TRACK, "a road"
        cx, cy = world.to_cell(x, y)
        assert all(world.at((cx + i, cy + j)) not in IMPASSABLE for i in range(-3, 4) for j in range(-3, 4)), "nothing in its way"


def test_mines_and_other_buried_things_are_scattered_over_the_world_off_the_track_and_apart(world_and_population):
    trail, world, pop = world_and_population
    targets = pop.targets
    assert len(targets) >= 25
    truths = Counter(t.truth_class for t in targets)
    assert truths["high_metal_shallow"] >= 3 and truths["inert_scrap"] >= 2 and truths["low_metal_deep"] >= 2, "the cases the sensors are built for, not only one kind"
    d = trail.distances(np.array([[t.x, t.y] for t in targets]))
    assert d.min() >= population.MIN_FROM_TRACK_M - 2.0 - 1.0
    assert d.min() < 20.0 and d.max() > 30.0, "near the track and far from it"
    xs = np.array([[t.x, t.y] for t in targets])
    spread = xs.max(axis=0) - xs.min(axis=0)
    assert spread.min() > 60.0, "over a wide area, not in one block"
    scattered = targets[-population.SCATTER_COUNT:]
    for i, a in enumerate(scattered):
        for b in scattered[i + 1:]:
            assert math.hypot(a.x - b.x, a.y - b.y) >= population.SCATTER_SPACING_M - 1e-6


def test_the_window_builds_with_all_of_it_and_the_vehicle_and_walkers_are_in_the_simulation():
    import os

    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication

    from ocu.core.trail_scenario import TrailWorld

    if len(TrailWorld.load(None).legs) < 3:
        pytest.skip("needs reconstructed trails in assets/footage/trail")
    QApplication.instance() or QApplication([])
    from ocu.ui.main_window import MainWindow

    window = MainWindow(trail=True)
    try:
        kinds = Counter(e.kind for e in window.surface_entities)
        assert kinds["vehicle"] == 2 and kinds["person"] >= 8 and kinds["animal"] >= 8
        assert any(e.kind == "vehicle" and not e.static for e in window.surface_entities)
        assert len(window.target_field.targets) >= 50
        assert sum(not e.static for e in window.surface_entities) >= 20, "people and animals that move"
    finally:
        window.close()
