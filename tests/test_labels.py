"""The operator's vocabulary.

These names appear on the roster, the map, the camera tiles, the threat list
and the log. An abbreviation that means one thing in one place and another
somewhere else is a reader's problem, so there is one table.
"""

from ocu.ui.labels import SENSOR_ABBR, sensor_label, sensor_list, short_label


def test_units_are_called_by_their_call_signs():
    assert short_label("rover") == "ROVER"
    assert short_label("tricopter-1") == "TRI-1"
    assert short_label("tricopter-3") == "TRI-3"


def test_every_sensor_the_suite_reports_has_a_name():
    from ocu.core.detection import SENSORS

    for sensor in list(SENSORS) + ["saw_voc", "optical"]:
        assert sensor in SENSOR_ABBR, f"{sensor} would be shown raw"


def test_the_names_are_short_enough_to_sit_in_a_column():
    assert all(len(name) <= 4 for name in SENSOR_ABBR.values())


def test_pulse_induction_is_not_truncated_into_something_else():
    """Cutting the raw name to three letters gave PUL, which reads like a
    different instrument."""
    assert sensor_label("pulse_induction") == "PI"
    assert sensor_label("magnetic_gradiometer") == "MAG"


def test_an_unknown_sensor_is_shown_whole_rather_than_cut():
    assert sensor_label("neutron_backscatter") == "NEUTRON_BACKSCATTER"


def test_a_list_of_sensors_reads_in_the_order_given():
    assert sensor_list(["magnetic_gradiometer", "pulse_induction", "gpr"]) == "MAG PI GPR"
    assert sensor_list([]) == ""


def test_the_threat_list_and_the_log_use_the_same_table():
    import ocu.ui.threat_panel as panel
    import ocu.ui.main_window as window

    assert panel.sensor_list is sensor_list
    assert window.sensor_list is sensor_list


# --- the panels survive being shown -----------------------------------------------

def test_a_threat_card_can_be_built_and_shown(tmp_path):
    """Built and laid out for real. A card that only works while its panel is
    hidden passes every test and crashes the moment the window is on screen."""
    import os

    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication

    app = QApplication.instance() or QApplication([])
    from ocu.ui.threat_panel import ThreatPanel

    panel = ThreatPanel()
    panel.resize(360, 400)
    panel.show()
    app.processEvents()

    panel.add_detection(
        {"x": 4.0, "y": -8.0, "type": "mine", "confidence": 0.97, "sensors": ["gpr", "saw_voc"]}
    )
    app.processEvents()
    panel.repaint()
    assert len(panel._threats) == 1

    panel.set_status(4.0, -8.0, "CONFIRMED")
    app.processEvents()
    panel.repaint()


# --- the map and the cameras agree about what the ground looks like ---------------

def test_the_maps_classes_share_the_cameras_hues():
    """An operator looking from the map to a feed should not have to match a
    green patch to a different green. Open ground used to be blue on the map
    and tan in the camera."""
    from PySide6.QtGui import QColor

    from ocu.core.camera import CANOPY_RGB, GROUND_RGB, ROCK_RGB, SCRUB_RGB
    from ocu.core.terrain import Terrain
    from ocu.ui.theme import RETURN_COLORS, TERRAIN_COLORS

    pairs = {
        "GROUND": GROUND_RGB[Terrain.GROUND],
        "TRACK": GROUND_RGB[Terrain.TRACK],
        "SCRUB": SCRUB_RGB,
        "TREE": CANOPY_RGB,
        "ROCK": ROCK_RGB,
    }
    for name, rgb in pairs.items():
        camera = QColor(*rgb)
        for palette in (RETURN_COLORS, TERRAIN_COLORS):
            drawn = palette[name]
            spread = abs(drawn.hslHue() - camera.hslHue())
            assert min(spread, 360 - spread) <= 6, f"{name}: {drawn.name()} vs {camera.name()}"


def test_the_map_ranks_brightness_by_what_has_to_be_found():
    """Hue is the camera's; brightness is the map's own decision. Open ground
    is most of the view and stays quiet; a thing that stops the vehicle is
    the brightest mark on the display."""
    from ocu.ui.theme import RETURN_COLORS

    def lum(colour):
        return 0.2126 * colour.redF() + 0.7152 * colour.greenF() + 0.0722 * colour.blueF()

    ground = lum(RETURN_COLORS["GROUND"])
    assert ground < lum(RETURN_COLORS["SCRUB"]) < lum(RETURN_COLORS["TREE"])
    assert lum(RETURN_COLORS["TREE"]) < lum(RETURN_COLORS["ROCK"])
    assert ground < lum(RETURN_COLORS["TRACK"])


def test_a_hole_is_coloured_to_be_found_rather_than_recognised():
    """The one class that is deliberately not natural: it is a hazard, not
    terrain, and nothing else on the map is that colour."""
    from ocu.ui.theme import RETURN_COLORS

    hole = RETURN_COLORS["POTHOLE"]
    for name in ("GROUND", "TRACK", "SCRUB", "TREE", "ROCK"):
        spread = abs(hole.hslHue() - RETURN_COLORS[name].hslHue())
        assert min(spread, 360 - spread) > 60, name
