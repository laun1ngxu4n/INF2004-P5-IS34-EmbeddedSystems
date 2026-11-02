@echo off
rem Double-click to open the robot control panel (works from any folder).
cd /d "%~dp0.."
python tools\is34_panel.py %*
if errorlevel 1 (
    echo.
    echo The panel stopped with an error ^(see above^).
    echo If it says "No module named paho": run  python -m pip install paho-mqtt
    pause
)
