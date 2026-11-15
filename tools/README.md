# Tools (run on the laptop)

| Folder | Tool | What it is for |
|---|---|---|
| [`panel/`](panel/) | `is34_panel.py` (double-click `is34_panel.bat`) | The robot control panel: START / STOP, barcode mapping, live status and sensors, the robot's own log over WiFi (**Save log...**). Needs `pip install paho-mqtt` and Mosquitto (`mosquitto_robot.conf`, port 1884). Its logs go to [`../logs/`](../logs/). |
| [`sim/`](sim/) | `linesim.py` | Line-following simulator, calibrated on the 7 Nov robot runs: tunes the line task's steering. |
| | `bypasssim.py` | Obstacle bypass simulator (uses `linesim.py`): straight / curved lines, boxes on junctions, box depths. `python tools/sim/bypasssim.py S` |
| [`websim/`](websim/) | `index.html` (served), `build.sh` | The real firmware in a browser: compiles `app_program/` to WebAssembly and runs it on a simulated Pico W, robot and tile track (obstacle, ramp, hump, barcodes). See its README. |
| [`logcheck/`](logcheck/) | `sonarcheck.py` | Re-runs the obstacle width logic on the `[SCAN] rays` lines of a robot log. |
| | `barcodecheck.py` | Re-runs the barcode decoder (and its repairs) on the `code ended` lines of a robot log. |
| | `verify_phase11_usb.py` | Checks a captured USB console transcript (bring-up phase 11). |
| [`elf2uf2/`](elf2uf2/) | | Source of the ELF-to-UF2 converter the build compiles (`build_make/pico_rp2040.mk` uses this path: do not move). |

Run them from the repo folder, e.g. `python tools/logcheck/barcodecheck.py logs/robot/<file>`.
