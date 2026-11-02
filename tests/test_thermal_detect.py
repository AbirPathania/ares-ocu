"""Finding people, animals and anomalies in a thermal picture (core/thermal_detect.py).

The detector reads temperatures and nothing else. These pictures come from the imager (core/thermal_scene.py), so what
is tested is the pair: a body the imager draws warm is a contact the detector reports, where it stands, and an empty
scene is not reported as occupied.
"""

import math

import numpy as np
import pytest

import ocu.core.thermal_detect as td
import ocu.core.thermal_physics as tp
import ocu.core.thermal_scene as ts
from ocu.core.camera import CameraRig
from ocu.core.entities import SurfaceEntity
from ocu.core.terrain import TerrainWorld

NIGHT, DAWN = 2.0, 6.5


@pytest.fixture(scope="module")
def imager():
    world = TerrainWorld.blank(half_extent_m=80.0)
    rig = CameraRig(world)
    return ts.ThermalImager(rig, tp.build_field(rig, []))


def person(x, y):
    return SurfaceEntity("person", "p", 1.78, 0.54, 0.63, ((x, y),), heading_deg=180.0, box_offset_m=-0.095)


def deer(x, y):
    return SurfaceEntity("animal", "d", 1.91, 0.72, 2.14, ((x, y),), heading_deg=90.0, box_offset_m=0.27, style="buck")


def look(imager, hour, entities=(), sensor=ts.ROVER_THERMAL, height=ts.ROVER_THERMAL_HEIGHT_M, pitch=3.0, seed=1):
    poses = [(e.path[0][0], e.path[0][1], e.heading_deg) for e in entities]
    return imager.render(sensor, 0.0, 0.0, height, 0.0, pitch, hour, entities, poses, rng=np.random.default_rng(seed))


def kinds(contacts):
    return [c.kind for c in contacts if c.reportable]


def test_a_person_at_night_is_a_person_and_is_put_where_they_stand(imager):
    contacts = td.detect(look(imager, NIGHT, [person(1.0, 14.0)]), "rover")
    found = [c for c in contacts if c.kind == "person"]
    assert len(found) == 1
    assert math.hypot(found[0].x - 1.0, found[0].y - 14.0) < 1.5, "the position is the body's own face, not the ground behind it"
    assert 1.3 <= found[0].height_m <= 2.3 and found[0].width_m <= 1.3
    assert found[0].contrast_k > 3.0


def test_an_animal_is_not_called_a_person(imager):
    contacts = td.detect(look(imager, DAWN, [deer(-2.0, 16.0)]), "rover")
    assert "animal" in kinds(contacts)
    assert "person" not in kinds(contacts)


def test_an_empty_scene_reports_no_people_animals_or_vehicles(imager):
    for hour in (NIGHT, DAWN):
        for seed in range(4):
            assert kinds(td.detect(look(imager, hour, seed=seed), "rover")) == []


def test_a_person_too_far_to_be_resolved_is_not_reported(imager):
    """About four pixels across the smaller side are needed to recognise a person (the recognition criterion). A scout
    camera sees that at a few tens of metres, not at a hundred."""
    far = look(imager, NIGHT, [person(0.0, 70.0)], sensor=ts.SCOUT_THERMAL, height=40.0, pitch=20.0)
    assert "person" not in kinds(td.detect(far, "scout"))


def test_a_warm_blob_that_is_not_the_shape_of_a_body_is_not_called_one():
    """A strip of warm ground is long and thin: it is an object, whatever its temperature."""
    h, w = 120, 160
    temp = np.full((h, w), 5.0, dtype=np.float32)
    temp[60:63, 20:140] += 6.0
    ones = np.ones((h, w), dtype=bool)
    ifov = ts.SCOUT_THERMAL.ifov_rad
    frame = ts.ThermalFrame(
        temp, ones, np.full((h, w), -1, dtype=np.int16), np.tile(np.linspace(-10, 10, w), (h, 1)), np.tile(np.linspace(10, 40, h)[:, None], (1, w)),
        np.full((h, w), 30.0, dtype=np.float32), np.full((h, w), 30.0 * ifov * 3.0, dtype=np.float32), np.full((h, w), 30.0 * ifov, dtype=np.float32),
        ts.SCOUT_THERMAL, ones,
    )
    assert kinds(td.detect(frame, "scout")) in ([], ["anomaly"])
