@echo off
rem Builds tools\panel\exe\is34_panel.exe: the panel as one file, no Python
rem needed on the laptop that runs it (Mosquitto still is: see README.txt).
rem Needs, on the laptop that BUILDS it:  python -m pip install pyinstaller paho-mqtt
cd /d "%~dp0"
python -m PyInstaller --noconfirm --onefile --windowed --name is34_panel ^
    --add-data "%~dp0mosquitto_robot.conf;." ^
    --distpath exe --workpath "%TEMP%\is34_panel_build" --specpath "%TEMP%\is34_panel_build" ^
    "%~dp0is34_panel.py"
if errorlevel 1 pause
