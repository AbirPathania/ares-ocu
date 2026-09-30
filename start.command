#!/bin/bash
# ARES Operator Control Unit — macOS launcher.
# Double-click this file in Finder to start the application.
# First run creates the virtual environment and installs dependencies,
# which takes a minute; every run after that starts immediately.

cd "$(dirname "$0")" || exit 1

if [ ! -x ".venv/bin/python" ]; then
    echo "First run: setting up the environment..."
    python3 -m venv .venv || {
        echo "Could not create the virtual environment. Is Python 3 installed?"
        read -r -p "Press Return to close."
        exit 1
    }
    .venv/bin/pip install --quiet --upgrade pip
    .venv/bin/pip install --quiet -r requirements.txt || {
        echo "Could not install dependencies."
        read -r -p "Press Return to close."
        exit 1
    }
    echo "Setup complete."
fi

echo "Starting ARES Operator Control Unit..."
exec .venv/bin/python -m ocu.main
