@echo off
REM ARES Operator Control Unit - Windows launcher.
REM Double-click this file to start the application.
REM First run creates the virtual environment and installs dependencies,
REM which takes a minute; every run after that starts immediately.

cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
    echo First run: setting up the environment...
    python -m venv .venv
    if errorlevel 1 (
        echo Could not create the virtual environment. Is Python 3 installed and on PATH?
        pause
        exit /b 1
    )
    .venv\Scripts\python -m pip install --quiet --upgrade pip
    .venv\Scripts\pip install --quiet -r requirements.txt
    if errorlevel 1 (
        echo Could not install dependencies.
        pause
        exit /b 1
    )
    echo Setup complete.
)

echo Starting ARES Operator Control Unit...
.venv\Scripts\python -m ocu.main
if errorlevel 1 pause
