# Subsystem 1: Telemetry, WiFi / MQTT and notification

**Owner:** B1. **Code:** [`app_program/subsystem1/`](../../app_program/subsystem1/)
(`telemetry.c`, `wire.c`). Two related parts outside that folder:
- the MQTT client, which runs inside the kernel's WiFi task
  (`lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`);
- the laptop panel ([`tools/panel/`](../../tools/panel/)).

**Tests:** Stage 5 (the demo image over WiFi). The state on 21 Nov 2026.
The whole robot: [README.md](README.md).

## 1. In one minute (what to say)

"Subsystem 1 is the robot's link to our laptop. Every second it sends a
15-byte binary status message over MQTT. Twice a second it sends the live
sensors in 8 bytes, and it sends every result from the other subsystems
as a text event, with one short beep on the robot's buzzer. From the
laptop it takes the barcode mapping, START and STOP; a STOP sets the
robot's stop bit before anything else happens. The MQTT client itself
runs in the kernel's own WiFi task, so our code only copies messages in
and out through two hooks that never wait. A WiFi drop is only a
warning: the robot keeps driving and reconnects by itself."

## 2. Why it exists

- **The course:**
  - the barcode mapping is sent from the PC just before the demo and must
    not be built in;
  - the demo needs the measurements (obstacle width, hump height) shown;
  - the robot must be stoppable from the laptop.
- **Requirements (Tribunal 2 list):**
  - FR7: a status message every 1 s, plus an event for each result,
    with a beep;
  - FR10: the status is the heartbeat; the panel shows the robot
    offline after 3 s without one;
  - FR2a: the mapping is taken over WiFi at any time without a restart,
    acknowledged, and a bad one refused;
  - FR12: START and STOP from the laptop;
  - FR15: compact binary messages (Change Requirement 1);
  - NFR5: at least 1 Hz; the robot keeps running when the WiFi drops and
    reconnects by itself;
  - NFR9: every MQTT payload is at most 160 bytes.

## 3. What it owns, and what it measures

- **Hardware:**
  - the buzzer on GP22 (PWM slice 3, 2 kHz);
  - the WiFi radio itself belongs to the kernel's WiFi task, and no
    application task may call the WiFi, lwIP or MQTT functions
    ([TEAM.md](../team/TEAM.md)).
- **What it measures:** nothing in the world; it reports what the others
  measure. It measures **the link**: when the WiFi task has not asked for
  a message for 1 s, the WiFi-loss warning bit (`FAULT_WIFI_LOSS`) is set,
  and it is cleared when the WiFi task asks again.

## 4. Task and timing

`telemetry_task`: priority 12 (the lowest application task), every 100 ms.
In each period it does what is due:

| What | How often |
|---|---|
| Wake-up check for Mission (IF-01 ping, used by the Stage 0 test) | 1 s |
| Status message, `is34/telemetry` (15 bytes) | 1 s |
| Live sensors, `is34/sensors` (8 bytes) | 0.5 s |
| Buzzer: one 100 ms beep if an event came (events in the same 100 ms give one beep) | 100 ms |
| Timing table on the console | 10 s in test images, 1 min in the mission and normal images |

The kernel's WiFi task (priority 4) calls `app_mqtt_tx()` every 100 ms to
take one message to publish, and calls `app_mqtt_rx()` for each command
that arrives.

## 5. How it works

### 5.1 Events

- **Any task** calls `telemetry_event("text")`. It prints `[EVT] text` on
  the console, copies the text into an **8-slot outbox** (when the outbox
  is full the oldest is dropped and counted), and asks for a beep.
- **Events stay text:** they are sentences for the operator, such as
  `junction 2 (crossing): LEFT (barcode F)` or `obstacle width 109 +/- 19
  mm`.

### 5.2 Status and sensors: binary (Change Requirement 1)

[`wire.c`](../../app_program/subsystem1/wire.c) packs each field into its
exact number of bits:
- byte 0 is the format version (1);
- the fields are packed with explicit shifts, little-endian;
- a value outside its field's range is clamped, never cut off;
- a C structure is never sent as it is, because the compiler chooses its
  bit order and padding.

| Message | Size | Fields |
|---|---|---|
| `is34/telemetry`, every 1 s | 15 bytes (about 93 as text) | mission state, line state, stop reasons, the 8 fault bits, both wheel speeds, distance driven, junction count, last letter read, the mapping, last hump height |
| `is34/sensors`, every 0.5 s | 8 bytes (about 59) | the three IR sensors, "steering held", "inside a barcode", IMU OK, steering, the distance ahead with its status (a distance / nothing in range / no echo), tilt, climbed height |
| `is34/ack/map`, after a mapping | 5 bytes (18) | accepted or not, a mapping in use, the mapping now in use |

The full specification is [MQTT_WIRE_FORMAT.md](../team/MQTT_WIRE_FORMAT.md).
The robot's `wire.c` and the panel's decoder are checked against each
other with shared "golden" test messages (`python tools/panel/test_wire.py`):
**PASS, 15 of 15**, and a deliberately wrong field width makes it fail.

### 5.3 Commands from the laptop

| Topic | What the robot does |
|---|---|
| `is34/cmd/stop` (empty) | **Sets the STOP reason first** (`app_stop_request(STOP_MQTT)`): the motors go off at once. Only then is Mission told. A STOP can never be lost behind other messages |
| `is34/cmd/start` (empty) | Sets Mission's START bit (IF-01) |
| `is34/cmd/map` (4 bytes binary, or the text `K=L,Q=R,X=F,M=U` typed by hand) | Checked: exactly 4 pairs, each letter A-Z or 0-9 used once, each move once. Accepted: stored in subsystem 3's mapping and acknowledged. Refused: the old mapping is kept and the reason goes out as an event |

### 5.4 The link

- **The MQTT client** (`mqtt_utk.c`, in the kernel's WiFi task)
  subscribes to `is34/cmd/#` and publishes one message every 100 ms. The
  outbox and the log copy take turns, so a long log never holds up an
  event.
- **Reconnecting:** one attempt at a time. An attempt with no answer in
  15 s is dropped and started again.
- **Our two hooks** only copy under the lock and never wait, because they
  run inside the kernel's task.
- **WiFi loss** is a warning bit, never a stop.

### 5.5 The robot's log over WiFi

- Every byte printed on the console is also copied into an 8 KB ring and
  sent on `is34/log` as `L<seq> <dropped>|text`. Tests can therefore run
  on the power bank with no USB cable; the panel shows the log and saves
  it.
- A gap in `seq` means messages lost on the WiFi; `dropped` counts bytes
  the ring could not keep.

### 5.6 The laptop panel

The full description is in [PANEL.md](PANEL.md): every part of the window,
where each value comes from, and what to do when something is wrong.
In short, [`tools/panel/is34_panel.py`](../../tools/panel/is34_panel.py)
(or `tools/panel/exe/is34_panel.exe`) has:
- START and STOP (the Space bar is STOP);
- the mapping box;
- live status and sensors;
- `robot: OFFLINE` after 3 s without a status message;
- the robot's log, and "Save log + summary";
- the **Run summary** (since 20 Nov), one block per START:
  - **totals** (since 21 Nov): barcodes read and failed, junctions,
    obstacles (bypassed, false stops), humps and their peak;
  - then the letters read, junction moves, obstacles with their widths,
    humps and stops;
  - each obstacle says **"depth not measured"**: the robot measures the
    width only, and the bypass assumes a box up to 120 mm deep.

### 5.7 Set-up

- The broker is Mosquitto on the laptop, on **port 1884**. The Windows
  Mosquitto service only listens on 1883 for the laptop itself, so a
  second broker runs on 1884 (`tools/panel/mosquitto_robot.conf`).
- Use a phone hotspot on **2.4 GHz**.
- The WiFi name, password and broker IP are built into the image from
  `config/wifi_secrets.h`, which is never committed. **If the laptop's IP
  on the hotspot changes, rebuild.**

## 6. Numbers to know

| Value | Setting |
|---|---|
| Status every | 1 s (`S1_STATUS_TICKS` 10 x 100 ms) |
| Sensors every | 0.5 s (`S1_SENSOR_TICKS`) |
| Outbox | 8 messages (`OUTBOX_N`), oldest dropped when full |
| WiFi-loss warning after | 1 s without a poll from the WiFi task (`LINK_LOST_US`) |
| Log ring | 8 KB (`LOG_RING`) |
| Beep | 100 ms at 2 kHz (`BUZZER_WRAP`) |
| Timing table | 10 s (test images), 1 min (mission) (`S1_REPORT_TICKS`) |
| Message sizes | status 15 B, sensors 8 B, acknowledgement 5 B, mapping 4 B, START / STOP 0 B |
| Panel offline after | 3 s without a status (`OFFLINE_S` in the panel) |

## 7. Shown on the robot

- **2 Nov, Stage 5:**
  - the robot joined the WiFi and connected to MQTT;
  - status every 1 s;
  - a mapping was acknowledged `OK`, and a bad one (`K=L`) was refused
    with its reason;
  - remote START / STOP: **3 of the 5 planned rounds, 3 of 3 passed**.
    Each STOP gave the STOP event in the same second, and IDLE with
    speed 0 by the next status.
- **7 Nov:**
  - after the broker was gone for about 100 s, the robot re-joined by
    itself;
  - the robot's log came over WiFi.
- **9 Nov:** the binary messages (Change Requirement 1). A mapping bug
  was found and fixed the same day (section 8).
- **15-20 Nov:** every mission run went over WiFi, and the mapping was
  changed between runs and acknowledged each time.
  - In one 20 Nov run the WiFi-loss warning flickered about a dozen times
    for under 1 s each, and the robot kept running (as designed).
  - Some events were lost in WiFi drops: a junction move, scan lines.
- **20 Nov:** one beep per event. The tester heard no beep at the hump
  because a hump is reported after 100 mm of level driving past it,
  about 1 s later.

## 8. Tried, changed or dropped, and why

| What | When | What happened, and why |
|---|---|---|
| Status and sensors as text | until 8 Nov | Replaced by binary messages (Change Requirement 1); events and the log stay text |
| The timing table every 1 s | 26 Oct | It delayed every task by about 12 ms now and then through the USB console, and output was lost: every 10 s since. In the mission image every 1 min (20 Nov), because its 10 lines every 10 s drowned the events in the log |
| Reconnect after a drop | 7 Nov | The robot printed `mqtt_client_connect() failed` hundreds of times and never came back: two retry paths fought. Fixed: one retry at a time, and an attempt with no answer in 15 s is restarted |
| The log over WiFi | 7 Nov | About 1 message in 20 was lost when lwIP's buffer was still full. Fixed: the message is kept and sent on the next poll |
| The binary mapping | 9 Nov | Every mapping from the panel was REJECTED: the MQTT glue turned unprintable bytes into `.` before our code saw them, which damaged the binary command. Fixed: our code gets the raw bytes |
| NeoPixel status LEDs on GP18 | planned | Not built. The panel shows the state, and the buzzer beeps for every event |
| A separate Notification task (Week 6 design) | 26 Oct | Merged into Telemetry: fewer tasks, the same job |
| The buzzer | 20 Nov | Built at the team's request: one short beep for every event, played by this task so no other task waits for it |
| Broker on port 1883 | 2 Nov | The Windows Mosquitto service holds 1883 for the laptop only: a second broker on 1884 |

## 9. Not done, and known limits

- **Remote START / STOP:** 3 of the 5 planned rounds were run (S1-T02).
- **Reconnect within 20 s** (NFR5) was seen on the robot but not measured
  in repeated trials under load; Tribunal 1 asked for 20.
- **Events can be lost in a WiFi drop.**
  - While the link is down, messages wait in the 8-slot outbox and go out
    once it reconnects. The status and sensor messages share the outbox
    (about 3 a second), so after a drop of more than about 3 s the events
    from its start have been pushed out.
  - A message already on its way when the link drops can be lost too
    (seen on 20 Nov).
  - The panel shows the gaps in the log.
- **The broker IP is built in:** a new hotspot address needs a rebuild.
- **The kernel's WiFi task (priority 4) runs above ours.** Its measured
  effect: starting the radio pauses every task for about 120 ms, once,
  0.6 s after reset.

## 10. Questions you may get

- **Why binary and not text?** Change Requirement 1: each value takes only
  the bits its range needs. The status went from about 93 bytes to 15.
- **Why are events still text?** They are sentences for the operator to
  read, not values for a program.
- **What happens if the WiFi drops mid-run?** The robot keeps driving.
  Only a warning bit is set and the link reconnects by itself. Messages wait
  in the 8-slot outbox meanwhile; in a drop longer than about 3 s, the
  newer status messages push the oldest events out.
- **How fast does a STOP from the laptop act?** Our hook sets the stop bit
  as soon as the command arrives, before anything else, and the motors go
  off in that same call. A 2 Nov STOP showed up in the same second.
- **Why does our code not call MQTT itself?** The template's rule: the
  network stack is not safe from any task but the kernel's WiFi task. We
  only copy messages in and out through two hooks.
- **How do you know the robot is alive?** The status message every
  second is the heartbeat. The panel shows the robot offline after 3 s
  without one.
- **How do you know the two ends agree on the binary format?** The robot's
  own `wire.c` is compiled on the laptop and checked against the panel's
  decoder with shared test messages (15 of 15 pass).

## 11. Where the code is

| File | What is in it |
|---|---|
| [`telemetry.c`](../../app_program/subsystem1/telemetry.c) | `telemetry_task`, `telemetry_event()`, the outbox, `app_mqtt_rx()` / `app_mqtt_tx()`, the log copy (`tm_console_mirror()`), the buzzer |
| [`wire.c`](../../app_program/subsystem1/wire.c) | packing the binary messages (`wire_status_pack()`, `wire_sensors_pack()`, `wire_ack_pack()`, `wire_map_unpack()`) |
| [`subsystem1.h`](../../app_program/subsystem1/subsystem1.h) | the periods |
| `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` | the MQTT client in the kernel's WiFi task (template code, changed for our topics) |
| [`tools/panel/`](../../tools/panel/) | the laptop panel, its decoder (`is34_wire.py`) and the golden-vector test |
