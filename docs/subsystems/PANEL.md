# The laptop panel: what it shows and how it works with the robot

For every team member: the whole demo is driven from this panel. The
panel belongs to subsystem 1 ([S1_TELEMETRY.md](S1_TELEMETRY.md)), but
every subsystem's results show up in it. The state on 21 Nov 2026.
The whole robot: [README.md](README.md).

## 1. In one minute (what to say)

"Our panel is a Python program on the laptop, also packaged as an .exe.
The panel and the robot both connect to an MQTT broker (Mosquitto) on the
laptop, over the phone's hotspot.

- **From the robot:** a 15-byte binary status every second, 8 bytes of
  live sensor readings twice a second, every result as a short sentence,
  and a copy of its console. The panel shows them and keeps a summary of
  each run.
- **From the panel:** START, STOP and the barcode mapping. The robot
  answers the mapping with what it stored.

STOP goes out the moment you press it, or the Space bar, and the robot
turns its motors off before it does anything else with the message."

## 2. How the pieces connect

```
  laptop                                          robot (Pico W)
  [panel] <--> [Mosquitto broker, port 1884] <== WiFi: phone hotspot ==> [kernel's WiFi task: MQTT client]
                                                                              |  app_mqtt_tx() / app_mqtt_rx()
                                                                              v
                                                              [S1 Telemetry] <--> S2, S3, S4, S5
```

| Direction | Topic | What | How often | Size | Robot side |
|---|---|---|---|---|---|
| robot to panel | `is34/telemetry` | the status | every 1 s | 15 bytes, binary | S1 packs it from every subsystem's values |
| robot to panel | `is34/sensors` | live sensor readings | every 0.5 s | 8 bytes, binary | S1, from S3, S4, S5 |
| robot to panel | `is34/event` | one sentence per result | when it happens | text | any subsystem, through `telemetry_event()` |
| robot to panel | `is34/ack/map` | the answer to a mapping | after each mapping | 5 bytes, binary | S1, after S3 stored it |
| robot to panel | `is34/log` | a copy of the robot's console | as it is printed | text | S1 |
| panel to robot | `is34/cmd/start` | START | when pressed | empty | S1 sets Mission's START bit (S5) |
| panel to robot | `is34/cmd/stop` | STOP | when pressed | empty | S1 sets the STOP reason first: motors off at once |
| panel to robot | `is34/cmd/map` | the barcode mapping | when sent | 4 bytes, binary | S1 checks it, S3 stores it |

The binary layout is in [MQTT_WIRE_FORMAT.md](../team/MQTT_WIRE_FORMAT.md).
The panel's decoder and the robot's `wire.c` are checked against each
other with shared test messages (`python tools/panel/test_wire.py`).

## 3. Start-up checklist

1. **Mosquitto** installed in `C:\Program Files\mosquitto` (once per
   laptop).
2. **Join the phone hotspot**, on **2.4 GHz** (the Pico W has no 5 GHz).
   The robot's WiFi image has the hotspot's name and password, and **this
   laptop's IP address as the broker**, built in.
3. **Start the panel:** double-click `tools/panel/exe/is34_panel.exe`, or
   run `python tools/panel/is34_panel.py` (needs `pip install paho-mqtt`).
   - It connects to the broker on this laptop, port 1884, and starts
     Mosquitto itself if none is running (or press **Start broker**).
   - When Windows asks, **allow `mosquitto.exe` through the firewall**
     (private network), or the robot cannot reach it.
4. **Power the robot from the power bank only.** Within a few seconds the
   top right says `robot: online` and the status box fills.
5. **Pick the mapping and press "Send mapping to robot".** Wait until it
   shows `OK ...` and `robot has: ... (same)`.
6. **START:** the green button or Ctrl+G. **STOP:** the red button or the
   **Space bar**.

## 4. The window, part by part

### 4.1 The top line

| Item | What it means |
|---|---|
| Broker address and port, **Connect**, **Start broker** | which broker the panel uses (default this laptop, port 1884) |
| `broker 127.0.0.1:1884` / `not connected` | the panel's own link to the broker |
| `robot: online` / `robot: OFFLINE (N s)` / `robot: no data` | the robot's status message arrived in the last 3 s / none for N seconds / none since the panel started. The status is the robot's heartbeat (FR10) |
| **Light** / **Dark** | switches the panel between a dark and a light look; the choice is kept in `is34_panel_theme.txt` next to the mapping file |

### 4.2 START and STOP

| Button | What happens in the robot |
|---|---|
| **START** (Ctrl+G) | Mission (S5) gets its START bit, clears the button and laptop stop reasons, and goes to CAL: 1 s standing still while the IMU (S4) learns level. Then RUN: it follows the line. In the stage test images, START also answers the "press START" prompts |
| **STOP** (Space) | S1's hook sets the STOP reason **before anything else**, so the motors go off at once. Mission goes back to IDLE. "Stop reason" then shows `laptop`, until START |

If the robot is not online when you press one, the Events tab says
`warning: the robot is not online, it will not see this`.

### 4.3 The barcode mapping box

**Picking a mapping:**
- Four drop-down boxes: which barcode (A-Z or 0-9) means **Left**,
  **Right**, **Straight**, **U-turn**.
- **Test set: A B C D** fills in the test mapping.
- Or paste one, e.g. `K=L,Q=R,X=F,M=U` or `K left, Q right, X straight,
  M u-turn`, and press **Use**.
- `will send: ...` shows what will go out. Two moves with the same letter
  are refused before sending.

**Send mapping to robot:**
1. The panel sends the mapping as 4 binary bytes.
2. On the robot, S1 checks it (exactly 4 pairs, each letter and each move
   once) and S3 stores it.
3. The robot answers: `OK <the mapping>`, or `REJECTED (the robot keeps
   ...; why: see Events)`. The Events tab then has the robot's reason,
   `mapping REJECTED: ... (old mapping kept)`.

**What the robot has:**
- `robot has: ... (same)` / `(different: press Send)` compares the
  robot's mapping in its 1 s status with the boxes.
- **The robot forgets its mapping when it is switched off or reset** (it
  is kept in RAM). After a restart the box says `robot: no mapping yet
  (press Send)`.
- The panel remembers the last mapping you picked
  (`is34_panel_mapping.txt`); the robot does not.

**Changing the mapping:**
- You can change it at any time, even during a run (FR2a).
- If a letter is waiting for its junction and its mapping disappears, the
  robot goes straight there.
- The Stage 2C and Stage 6 test images wait for a mapping before the
  course part, or START takes the test mapping.

### 4.4 The status box (every 1 s)

| Field | What it shows | From |
|---|---|---|
| State | IDLE, CAL, RUN or **MANOEUVRE**. MANOEUVRE shows in red but is normal: a junction turn or a bypass | S5 Mission |
| Speed L,R (mm/s) | both wheels' measured speeds | S2 Motion |
| Distance (mm) | the distance driven since power-on | S2 (odometer) |
| Line | ON, SEARCH (looking after an overshoot), LOST, FAULT (a sensor stuck) | S3 Line |
| Junctions | junctions since power-on (not since START) | S3 Line |
| Last barcode | the last letter read; `-` = none, or the last read failed | S3 Barcode |
| Mapping | the mapping the robot has now | S3 (sent by S1) |
| Ramp peak (mm) | the latest hump height reported | S4 IMU |
| Stop reason | `none`, `button` (also a lift), `laptop`, `fault` | the shared stop word |
| Faults / warnings | `motor stall`, `encoder (warning)`, `IMU (warning)`, `no sonar echo`, `line sensor stuck (one / both)`, `Wi-Fi lost (warning)`, `Motion task hung` | the shared fault word (S4, S5, S3, S1) |

### 4.5 Live sensors (every 0.5 s)

| Part | What it shows | From |
|---|---|---|
| Line + barcode | the three IR sensors L, R, B (a lit circle = black); the steering in mm/s; `in barcode`; `steering held (B on bars)` | S3, and S2 for "held" |
| Obstacle (front) | the sonar's distance straight ahead, `nothing in range`, or `NO ECHO (fault)`. Live even while IDLE | S5 Obstacle |
| IMU tilt | tilt in degrees, height climbed, `OK` / `NOT ANSWERING` | S4 IMU |

### 4.6 The tabs

**Run summary.** One block per START, built from the robot's events:
- **Totals:** barcodes read and failed, junctions, obstacles (bypassed,
  false stops), humps and their peak;
- every barcode (letter, move, bar widths) and every junction move;
- every obstacle: trip distance, width ± uncertainty, the distance at the
  scan, scan time, the bypass and whether it got back on the line, with
  `depth not measured` (the robot measures the width only);
- every hump height, and every stop with its reason;
- **Clear** empties it.

**Events.** Every line, with the time:
- `[robot]`: an event from the robot;
- `[sent]`: what the panel sent, with its bytes;
- `[panel]`: the panel's own notes (connected, saved, warnings).

**Robot log.** The robot's console, the same text as over USB, live:
- **follow** keeps it scrolled to the end;
- the view keeps 4,000 lines; the file keeps everything;
- gaps are marked: `[panel: N log messages lost on the WiFi]`, `[panel:
  the robot dropped N bytes, its buffer was full]`, `[panel: the robot
  restarted]`.

**Save log + summary** (on both tabs) writes two files to `logs/robot/`
(for the .exe, `logs\robot` next to it): `robot_log_<date>_<time>.txt`
and `robot_summary_<date>_<time>.txt`. Send both after a test.

The panel also saves everything as it goes, with no button:
`logs/panel/is34_<date>_<time>.log` (everything it showed) and
`logs/robot/is34_<date>_<time>_robot.txt` (the robot's log as it came).

### 4.7 Which subsystem sends which event

| Event sentence | Sent by |
|---|---|
| `start`, `STOP`, `stopped: <why> (press START to go again)` | S5 Mission |
| `barcode F ... -> LEFT at next junction`, `barcode READ_FAIL`, `... NO MAPPING, ignored`, `... -> NOT USED` | S5, reporting what S3 read |
| `junction 3 (crossing): LEFT (barcode F)` | S5 |
| `obstacle ahead at N mm: scanning`, `obstacle width W +/- U mm ...`, `bypass LEFT: ...`, `line found ...`, `bypass done ...` | S5 |
| `ramp peak height H mm ...`, `robot lifted or tipped over ...` | S4 IMU |
| `Safety: a wheel driven hard is not turning ...`, `Safety: the Motion task stopped running ...` | S4 Safety |
| `mapping REJECTED: ... (old mapping kept)` | S1 |

Each event also gives one short beep on the robot.

## 5. When something is wrong

| You see | Likely cause | What to do |
|---|---|---|
| `not connected` (top) | no broker on this laptop | press **Start broker**; check that Mosquitto is installed |
| `robot: no data` | the robot is not reaching the broker | the robot is on the same hotspot (2.4 GHz)? Mosquitto allowed through the firewall? Is this laptop's IP (`ipconfig`, the hotspot's adapter) the one built into the robot's image? Over USB the robot prints `mqtt: no answer from <ip>` or `mqtt: connected to <ip>`. A new IP means a rebuild of the WiFi image on the build laptop (`config/wifi_secrets.h`) |
| `robot: OFFLINE (N s)` | no status for 3 s: a WiFi drop, or the robot is off or restarting | if the robot is driving, it keeps going and reconnects by itself; STOP still works once it is back |
| `REJECTED (...)` after Send | a letter or a move twice, or a bad message | the reason is in Events; the robot kept its old mapping |
| `robot has: ... (different: press Send)` | the robot restarted (mapping lost), or the boxes changed | press **Send mapping to robot** |
| Stop reason stays `laptop` or `button` | a STOP (or a lift) is waiting to be cleared | press START |
| `Wi-Fi lost (warning)` flickers in Faults | short WiFi drops | only a warning: the robot keeps running |
| `(trip lost on the WiFi)` in the summary, or log gaps | an event or log message lost in a WiFi drop | the robot did it; the record of it is missing. Keep the laptop near the track |
| State in red while it turns | MANOEUVRE: a junction turn or a bypass | normal |

## 6. Limits

- **Events lost in a WiFi drop** are missing from the Events tab and the
  Run summary.
  - The robot keeps up to 8 messages while the link is down, and the
    status and sensor messages share that space.
  - The summary is built only from the events that arrived.
- **What the status box covers:** Last barcode and Ramp peak show the
  latest only. Junctions and Distance count since power-on. The per-run
  counts are in the Run summary.
- **The robot does not measure obstacle depth.** The bypass assumes up to
  120 mm and checks that the box is not still beside it before turning
  back.
- **The broker IP is built into the robot's image,** so another laptop
  works only with the same IP, or after a rebuild.
- **Older robot images** sent text messages; the panel still shows them.

## 7. Where the code is

| File | What is in it |
|---|---|
| [`tools/panel/is34_panel.py`](../../tools/panel/is34_panel.py) | the window, the MQTT client, the buttons, the mapping box, the logs |
| [`tools/panel/is34_wire.py`](../../tools/panel/is34_wire.py) | the binary messages: unpacking the status, sensors and acknowledgement, packing the mapping |
| [`tools/panel/is34_summary.py`](../../tools/panel/is34_summary.py) | the Run summary (also runs alone on a saved panel log) |
| [`tools/panel/test_wire.py`](../../tools/panel/test_wire.py), `wire_golden.c` | the golden-vector test against the robot's `wire.c` |
| [`tools/panel/mosquitto_robot.conf`](../../tools/panel/mosquitto_robot.conf) | the broker: port 1884, any address |
| `tools/panel/build_exe.bat`, [`tools/panel/exe/`](../../tools/panel/exe/) | the packaged .exe and how to rebuild it after any panel change |
| robot side: [`telemetry.c`](../../app_program/subsystem1/telemetry.c), [`wire.c`](../../app_program/subsystem1/wire.c), `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` | the hooks, the binary messages, the MQTT client in the kernel's WiFi task |
