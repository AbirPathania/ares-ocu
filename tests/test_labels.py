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
