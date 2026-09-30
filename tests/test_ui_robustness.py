"""Operator actions fired in illegal orders must never raise.

A field operator will press Return To Base twice, hit Launch with nothing
selected, recall an aircraft that is already docked, and reset mid-launch.
None of that is a programming error on their part, so none of it may
produce a traceback. This test drives the real window with a pseudo-random
sequence of operator actions and asserts the app survives.
"""

import os
import random

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication  # noqa: E402

from ocu.ui.main_window import MainWindow  # noqa: E402

TRICOPTERS = ["tricopter-1", "tricopter-2", "tricopter-3"]


@pytest.fixture(scope="module")
def app():
    application = QApplication.instance() or QApplication([])
    yield application


def pump(app, iterations=40):
    for _ in range(iterations):
        app.processEvents()


def test_random_operator_actions_never_raise(app):
    window = MainWindow()
    console = window.context_panel.launch_console
    rng = random.Random(1337)

    actions = [
        lambda: window._on_bay_launch_requested(),
        lambda: console._on_stage_clicked(),
        lambda: console._on_launch_confirmed(),
        lambda: window._on_return_to_base(),
        lambda: window._on_recall_requested(rng.choice(TRICOPTERS)),
        lambda: window._on_altitude_commanded(rng.choice(TRICOPTERS), rng.uniform(1, 500)),
        lambda: window._on_cut_link(),
        lambda: window._on_restore_link(),
        lambda: window._on_play_clicked(),
        lambda: window._on_reset_clicked(),
        lambda: window._select_unit(rng.choice(["rover"] + TRICOPTERS)),
        lambda: window._on_take_manual(rng.choice(["rover", "tricopter-1"])),
        lambda: window._on_release_manual(),
        lambda: window._on_director_tick(),
    ]

    for i in range(400):
        rng.choice(actions)()
        if i % 20 == 0:
            pump(app, 5)

    pump(app, 20)
    assert set(window._docked_state) == set(TRICOPTERS)
    for unit_id, docked in window._docked_state.items():
        if docked:
            assert unit_id not in window.flight_director.airborne_ids


def test_launch_with_nothing_selected_is_a_noop(app):
    window = MainWindow()
    console = window.context_panel.launch_console
    window._on_bay_launch_requested()
    for box in console.checkboxes.values():
        box.setChecked(False)
    console._on_stage_clicked()
    console._on_launch_confirmed()
    pump(app)
    assert all(window._docked_state.values())


def test_recall_of_a_docked_aircraft_is_a_noop(app):
    window = MainWindow()
    window._on_recall_requested("tricopter-1")
    pump(app)
    assert window._docked_state["tricopter-1"] is True


def test_reset_midflight_restores_a_clean_bay(app):
    window = MainWindow()
    console = window.context_panel.launch_console
    window._on_bay_launch_requested()
    console._on_stage_clicked()
    console._on_launch_confirmed()
    pump(app, 30)
    window._on_reset_clicked()
    pump(app, 10)
    assert all(window._docked_state.values())
    assert window.flight_director.aircraft == {}
    assert all(owner == "scripted" for owner in window._position_owner.values())
