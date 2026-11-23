# The whole robot: how the five subsystems fit together

For every team member. This page explains what the robot does, how the
five subsystems work together, what they share, what was decided or
dropped along the way, and where each part stands. Each subsystem has its
own page with the details (section 3).

Everything here describes the code on `main` and the results in
[`testing/TEST_STATUS.md`](../../testing/TEST_STATUS.md) on **21 Nov 2026**.
Test results come from robot logs only. Where a number is a setting in
the code, its name is given so you can find it.

## 1. What the robot does

A two-wheeled robot car on a Raspberry Pi Pico W (Cytron Robo Pico
board). It runs micro T-Kernel 3.0, the course's real-time operating
system. In a demo run:

1. **Mapping.** The laptop panel sends the barcode mapping over WiFi: 4
   pairs such as `A=L, F=U, Q=F, Z=R` (letter to Left, U-turn, Forward,
   Right). The robot checks it and sends back what it stored.
2. **Start.** START is the GP20 button or the panel. The robot stands
   still for 1 s while the IMU learns "level", then follows the black tape
   at 120 mm/s.
3. **Barcodes.** It reads Code 39 barcodes beside the line in either
   direction, slowing to 50 mm/s over the bars, and keeps the letter.
4. **Junctions.** At the next junction it does that letter's move. No
   letter, or a letter with no mapping, means straight on.
5. **Obstacles.** It stops about 15 cm before an obstacle on the line and
   scans it with the ultrasonic sensor on a servo. It reports the width
   ± uncertainty, drives round the obstacle and gets back onto its own
   line.
6. **Hump.** It measures and reports the height of a hump under one
   wheel.
7. **Stops.** It stops at once on STOP (the GP21 button or the panel), on
   a fault, or when it is lifted, which is how a demo run ends.
8. **Reports.** The laptop gets:
   - a status message every second;
   - live sensor readings twice a second;
   - an event for every barcode, junction move, obstacle, hump and stop,
     each with one short beep on the robot.

## 2. The whole robot in one minute (what to say)

"Our robot runs eight tasks on a real-time kernel.

- The **Motion** task (subsystem 2) controls both wheel speeds every
  10 ms using the encoders.
- The **Line** task (subsystem 3) reads two IR sensors that straddle the
  tape and tells Motion how hard to steer.
- The **Barcode** task (also subsystem 3) measures bar widths as
  distance from the wheel encoders, so the speed does not matter. It
  decodes Code 39 in both directions.
- The **Mission** task (subsystem 5) is the course's state machine. It
  keeps the barcode's letter, does the mapped turn at the next junction,
  and runs the obstacle scan and bypass.
- The **IMU** task (subsystem 4) works out the hump height from the
  robot's tilt, and stops the motors when the robot is lifted.
- The **Safety** task (also subsystem 4) stops the motors if a wheel
  stalls or the Motion task stops running.
- The **Telemetry** task (subsystem 1) sends compact binary status over
  WiFi (MQTT) to our laptop panel. It takes the mapping, START and STOP
  from the panel.

Every stop goes through one shared stop word, which turns the motors off
at once from any task or interrupt."

## 3. The five subsystems

| # | Subsystem | Owner | What it does | Its tasks | Details |
|---|---|---|---|---|---|
| 1 | Telemetry, WiFi / MQTT, notification | B1 | status, sensors and events to the laptop; mapping, START and STOP from it; the buzzer | `telemetry_task` | [S1_TELEMETRY.md](S1_TELEMETRY.md) |
| 2 | Motion | B2 | motors, encoders, wheel speed control; drive, spin and line-follow commands | `motion_task` | [S2_MOTION.md](S2_MOTION.md) |
| 3 | Line following and barcodes | B3 | line sensors to steering; junctions; Code 39 decoder; keeps the mapping | `line_task`, `barcode_task` | [S3_LINE_AND_BARCODE.md](S3_LINE_AND_BARCODE.md) |
| 4 | IMU and Safety | B4 | hump height; stop when lifted or tipped over; stall and Motion-hung stop | `imu_task`, `safety_task` | [S4_IMU_AND_SAFETY.md](S4_IMU_AND_SAFETY.md) |
| 5 | Mission and obstacle | B5 | the course state machine; START / STOP buttons; junction moves; ultrasonic scan and bypass | `mission_task`, `obstacle_task` | [S5_MISSION_AND_OBSTACLE.md](S5_MISSION_AND_OBSTACLE.md) |

**The laptop panel**, where every subsystem's results show up and which
drives the demo: [PANEL.md](PANEL.md). It explains each part of the
window, which subsystem each value comes from, what each button does in
the robot, and what to do when something is wrong.

**Presenting:** [PRESENTER_NOTES.md](PRESENTER_NOTES.md) has one page per
buddy: what to say, what to show, which numbers to quote, what not to
claim, and the running order for the demo.

The code is in `app_program/subsystemN/`, the tests in `testing/subsystemN/`.
Only a subsystem's owner edits its folder; shared files change by team
agreement ([TEAM.md](../team/TEAM.md)).

## 4. Hardware

| Part | Pins | Subsystem |
|---|---|---|
| 2 IR line sensors (digital), one each side of the tape | GP1 left, GP0 right | 3 |
| IR barcode sensor B, analogue output, powered from 3V3 | GP28 (ADC input 2) | 3 |
| 2 DC motors (board outputs M1, M2), PWM 10 kHz | GP8-GP11 | 2 |
| 2 wheel encoders (quadrature, about 2,540 counts per turn) | GP4/GP5 left, GP16/GP17 right | 2 |
| IMU LSM303D (accelerometer, no gyroscope), I2C1 at 400 kHz | GP26 SDA, GP27 SCL | 4 |
| HC-SR04 ultrasonic sensor, run at 3.3 V | TRIG GP6, ECHO GP7 | 5 |
| Servo that turns the ultrasonic sensor | GP14 | 5 |
| START and STOP buttons (on the board, active low) | GP20, GP21 | 5 |
| Buzzer (the board's piezo; it has a mute switch) | GP22 | 1 |
| WiFi radio (Pico W) | GP23-GP25, GP29 | the kernel |

- **Power:** a USB power bank into VIN for runs. Never power the robot
  from USB and the power bank at the same time.
- **Barcode sensor:** its digital output (GP2) belongs to the old
  digital build only. [TEAM.md](../team/TEAM.md) still lists GP28 as
  unused (battery sense was removed on 26 Oct); the barcode sensor has
  used it since 7 Nov.

Geometry the code uses (measured 6 Nov unless noted):

| Value | Setting |
|---|---|
| Track, wheel centre to wheel centre | 123 mm (`S2_TRACK_MM`, from turn tests) |
| Encoder distance per count | 0.080 mm (`S2_UM_PER_COUNT`) |
| Line sensors ahead of the wheel axle | 115 mm (`S3_SENSOR_TO_AXLE_MM`) |
| Barcode sensor B | 85 mm left of the robot's centre line, 40 mm behind the line sensors |
| Gap between the two line sensors | about 30 mm, with the 20 mm tape centred between them (a build rule) |
| Robot size used for the bypass | 170 mm wide, 200 mm long (`S2_ROBOT_WIDTH_MM`, `S2_ROBOT_LENGTH_MM`; the front bar is not measured) |
| Ultrasonic sensor ahead of the axle | 120 mm (`S5_SONAR_TO_AXLE_MM`, **not measured**) |

## 5. The software frame

**The kernel.** micro T-Kernel 3.0 from the course template, run on
**one core** (`SMP=0`, team decision of 26 Oct). With one core, turning
interrupts off is enough to protect shared data, so the one lock
(`APP_LOCK`) is simple. The template's second core added no speed we
needed. The build refuses `SMP=1` until a hardware spinlock is added to
the lock.

**Timing.**
- The kernel ticks every 10 ms.
- The kernel's own USB console task (priority 3) and WiFi task
  (priority 4) run above ours.
- Periodic tasks are woken by cyclic handlers, so their periods are
  exact. Mission waits on an event flag instead.

**The 8 tasks** (priority 1 is the highest; set in [`app.h`](../../app_program/app.h)):

| Task | Subsystem | Priority | Runs | Does |
|---|---|---|---|---|
| `safety_task` | 4 | 5 | every 20 ms | stall, Motion-hung and encoder checks |
| `motion_task` | 2 | 6 | every 10 ms | encoders to wheel speeds to PWM; runs the drive, spin and follow commands |
| `mission_task` | 5 | 7 | on events (100 ms timeout) | the course state machine, junction moves, the bypass |
| `line_task` | 3 | 8 | every 20 ms | line sensors to steering; junctions; line lost |
| `imu_task` | 4 | 9 | every 10 ms | tilt, hump height, lift stop |
| `barcode_task` | 3 | 10 | every 10 ms | sensor edges to Code 39 letters |
| `obstacle_task` | 5 | 11 | every 60 ms | the forward ultrasonic watch |
| `telemetry_task` | 1 | 12 | every 100 ms | status, sensors, events, buzzer, timing table |

**Why this order.** What protects the robot runs first: Safety, then
Motion's 10 ms control loop. Mission reacts to events before the sensors'
next round. The laptop link is last, so a WiFi problem can never delay
driving. This is the Week 6 design order, shifted below the kernel's own
tasks.

**Timing evidence.**
- Stage 0 (26 Oct): every task woke on time, with no overruns.
- Change Requirement 1 build (9 Nov): 31 timing tables over 5 minutes.
  The intervals stayed within about ±1.7 ms of each period. The longest
  run times were telemetry 2.6 ms, IMU 1.0 ms and Motion 0.37 ms.
- The only overruns happen once, when the WiFi radio starts about 0.6 s
  after reset and pauses everything for about 120 ms.

## 6. How the subsystems talk to each other

```
               laptop panel  (MQTT broker on the laptop)
                  ^  status, sensors, events      |  mapping, START, STOP
                  |                               v
        [S1 Telemetry]  <-- telemetry_event() from every subsystem
             |  mapping -> S3     START -> S5 (event flag)    STOP -> stop word
             v
  [S3 Line] -- steering, speed % --> [S2 Motion] -- PWM --> motors
  [S3 Line] -- junction, line lost --+
  [S3 Barcode] -- letter ------------+--> [S5 Mission] -- drive / spin / follow --> [S2 Motion]
  [S5 Obstacle] -- something ahead --+
  [S4 IMU] -- hump height (event); lifted = STOP --+
  [S4 Safety] -- stall / Motion hung = STOP -------+--> stop word --> motors off at once
```

The agreed interfaces ([TEAM.md section 5](../team/TEAM.md#5-interfaces-between-subsystems)):

| ID | What | How |
|---|---|---|
| IF-01 | Wake Mission | the kernel event flag `gh_flg_mission`, owned by S5. Bits: STOP, START, JUNCTION, LINE_LOST, BARCODE, OBSTACLE, PING. Mission sleeps on it and clears what it handled |
| IF-02 | Stop the robot | `app_stop_request(reason)` from any task or interrupt: the motors go off at once and stay off while any reason is set |
| IF-03 | Steering | `line_steer_mm_s()` and `line_speed_pct()`: the latest values, read by Motion every 10 ms |
| IF-04 | Motion commands | `motion_drive()`, `motion_turn()`, `motion_follow()` return a command ID; `motion_get_status()` gives its result: BUSY, DONE, STOPPED or TIMEOUT |
| IF-05 | The laptop | `app_mqtt_rx()` / `app_mqtt_tx()`, two hooks the kernel's WiFi task calls |

Also used everywhere:
- `telemetry_event("...")`: every result goes to the laptop this way;
- the fault word (`app_stop_fault_set()` / `app_stop_fault_clear()`);
- `motor_odometer()`: the robot's travel in encoder counts, used by S3,
  S4 and S5.

## 7. Shared code (owned by the whole team)

| File | What it is |
|---|---|
| [`app_stop.c` / `.h`](../../app_program/app_stop.h) | **The stop word:** 3 reasons (button, which the lift stop also uses; laptop; fault). **The fault word:** 8 bits, one per fault (table below). Every change is made with interrupts off |
| [`app_lock.h`](../../app_program/app_lock.h) | `APP_LOCK` / `APP_UNLOCK`: interrupts off. The one lock for all shared data |
| [`app_timing.c` / `.h`](../../app_program/app_timing.h) | The per-task timing table (period, overruns, run time), printed every 10 s in test images and every 1 min in the mission image |
| [`app_gpio.c` / `.h`](../../app_program/app_gpio.h) | One GPIO interrupt dispatcher for every pin (each owner registers its pin). It also releases PWM and I2C from reset, which the kernel leaves them in |
| [`app_main.c`](../../app_program/app_main.c) | Creates the tasks and cyclic handlers at start-up |
| [`app_test.c`](../../app_program/app_test.c), [`test_report.h`](../../app_program/test_report.h) | The stage test runner and the PASS / FAIL macros |
| [`app_math.h`](../../app_program/app_math.h) | Square root, sine and clamp without the C maths library |
| [`app_types.h`](../../app_program/app_types.h) | `float32_t`, and `APP_FIELD(bits)` for the minimum-bit fields (Change Requirement 1) |

**The fault word** (one bit each; a warning never stops the robot):

| Bit | Fault | Set by | Stops the robot? |
|---|---|---|---|
| 0x01 | a wheel driven hard does not turn (stall) | S4 Safety | yes |
| 0x02 | encoder edges missed too often | S4 Safety | no, warning |
| 0x04 | the IMU does not answer | S4 IMU | no, warning |
| 0x08 | the ultrasonic sensor gives no echo, 5 pings in a row | S5 Obstacle | yes (Mission stops) |
| 0x10 | one line sensor black too long | S3 Line | yes (Mission stops) |
| 0x20 | both line sensors black too long | S3 Line | yes (Mission stops) |
| 0x40 | no MQTT link (WiFi lost) | S1 Telemetry | no, warning |
| 0x80 | the Motion task stopped running | S4 Safety | yes |

## 8. Course requirements and changes that shaped the robot

| When | Change | What it meant for us |
|---|---|---|
| 26 Oct | Instructor's barcode brief | Code 39, A-Z at least, in both directions. Store the letter and do the move, U-turn included, at the next cross-junction. The mapping comes over MQTT just before the demo, never built in |
| 26 Oct | Instructor's obstacle brief | Obstacles 10-20 cm wide: profile the obstacle, report its width ± uncertainty, take a safe path round it, find the line again |
| 26 Oct | Battery voltage sense (GP28) removed | No low-battery fault; GP28 was later used for the barcode sensor |
| 27 Oct | Ramp: only the lower practice ramp | 8 Nov: the course "ramp" is a 2-5 cm triangle under one wheel, which changed the IMU method (S4) |
| 27 Oct | The IMU board carries an LSM303D (not the LSM303DLHC the design names) | Own register code; ±4 g range |
| Week 8 | **Change Requirement 1** (Tribunal 1): minimum-bit data | Done 8 Nov ([CHANGE_REQ1.md](../team/CHANGE_REQ1.md)). Bit-fields for application data, one fault bitmask, and binary MQTT messages (status 93 to 15 bytes, sensors 59 to 8, mapping acknowledgement 18 to 5, mapping 15 to 4). Our modules' RAM went from 3,795 to 3,143 bytes in the normal image (-652 B, 17 %) |
| 13 Nov | **Lecturer:** IMU collision detection removed; lifting or tipping the robot must stop the motors | Lift stop built 13 Nov (S4). Collision detection had never been built |

All changes since the Week 6 design: [TEAM.md section 7](../team/TEAM.md#7-scope-changes-after-the-week-6-design)
and [`week6_report_changes.md`](../team/week6_report_changes.md).

## 9. Which subsystem covers which requirement (Tribunal 2 list)

| Requirement | Subsystems |
|---|---|
| FR1 follow the line at 120 mm/s; NFR1a no line lost over 3 m | S3 (steering), S2 (wheels) |
| FR2 read barcodes; FR2a mapping over WiFi; FR11 junctions and the latest letter; NFR4 a letter decoded before its junction | S3 (decoder, mapping), S1 (the link), S5 (stores the letter) |
| FR3 the move at the junction | S5 (decides), S2 (spins), S3 (finds the tape) |
| FR4 hump height | S4 |
| FR5 obstacle width; FR6 bypass; FR16 a blocked leg stops; NFR3 scan time; NFR6 / NFR6a / NFR12 stop distance and clearances | S5, with S2 (legs) and S3 (finding the line) |
| FR7 status and events; FR10 heartbeat; NFR5 link rules | S1 |
| FR8 find the line after a turn or a bypass | S5, S3 |
| FR9 / FR14 stops and the shared stop / fault word; NFR10 stop timing | shared `app_stop`, S4 (Safety), S5 (buttons) |
| FR12 START / STOP | S5 (buttons), S1 (laptop) |
| FR13 lift stop | S4 |
| FR15 minimum-bit data (Change Requirement 1) | all |
| NFR1 task periods; NFR7 Barr C; NFR9 RAM; NFR11 shared data | all |
| NFR8 accuracy: turns, barcodes, hump, width | S2, S3, S4, S5 |

## 10. Decisions taken for the whole robot, and why

- **One core.** Simpler, safe locking; nothing needed the second core.
- **Barcode widths by distance, not time.** The encoders stamp every bar
  edge, so a change of speed over a barcode does not change the reading.
- **STOP inside the interrupt.** The STOP button's interrupt turns the
  PWM off itself, and Motion checks the stop reasons under the lock
  before every write, so no task can undo a STOP.
- **WiFi loss is only a warning.** A dropped link never stops the robot
  (Week 6 rule, Tribunal 1).
- **No C maths library** (a freestanding build): the code has its own
  square root and sine.
- **The MQTT client stays in the kernel's WiFi task** (the template's
  rule). Our code only copies data in and out through two hooks that
  never wait.
- **Binary messages for values, text for events** (Change Requirement 1).
  An event is a sentence for the operator, not a value.
- **Coding standard:** Barr C 2018, with documented deviations
  ([BARR_C_DEVIATIONS.md](../team/BARR_C_DEVIATIONS.md)).
- **Every tuned number carries its evidence.** The comment beside each
  setting gives the dates, runs and measurements behind it.
- **Simulate on the laptop before the robot runs** a risky change:
  `tools/sim/linesim.py` (line following), `tools/sim/bypasssim.py`
  (the bypass), and the robot's own decoder replayed on logged reads.

## 11. Tried and dropped for the whole robot

| What | When | Why it was dropped |
|---|---|---|
| Message queues between tasks (Week 6 design: `motion_cmd_q`, `branch_q` and others) | before coding | Replaced by latest-value reads, one command slot and one event flag: simpler, and nothing can overflow |
| Safety above every task, with a 500 ms hardware watchdog (Tribunal 1 design) | 8 Nov | Safety is the highest application task (priority 5); the hardware watchdog was not built |
| Running on both cores | 26 Oct | One core: simpler locking, no speed needed |
| Battery voltage sense | 26 Oct | The jumper on the board is not fitted |
| NeoPixel status LEDs (GP18, in the plan) | not built | The panel shows the state, and the buzzer beeps for every event |
| A separate Notification task (Week 6) | 26 Oct | Merged into Telemetry |
| IMU collision detection | 13 Nov | Removed by the lecturer: it conflicted with avoiding obstacles. Never built |
| Faster driving (150, then 135 mm/s) | 7, 15, 18 Nov | 150 lost tight curves (7 Nov). 135 never ran on the course, so it went back to 120 before the demo (18 Nov) |
| The timing table every second | 26 Oct | It delayed every task by about 12 ms through the USB console: now every 10 s (1 min in the mission image) |

## 12. Where each part stands (21 Nov)

✅ shown on the robot · 🟡 works, but not to its target every time · ⬜ not run

| Part | Status |
|---|---|
| S1 link, status, mapping, START / STOP, log over WiFi, beeps | ✅. Remote START / STOP done 3 of the 5 planned rounds (all passed); reconnect timing not measured in trials |
| S2 directions, STOP, 500 mm drive, turns (1 Nov), stall stop | ✅. Real turn angles after the 9 Nov trim not written down; the Motion-hung test (S4-S01) ⬜ |
| S3 line following | ✅ 3.65 m and 4 m with no line lost (20 Nov), curves and S-bends; crossings and left branches detected |
| S3 barcodes | 🟡 A, F, Q, Z read and obeyed on track 4 (20 Nov), both directions; not every pass: a code crossed at an angle, near the hump, or with its start clipped breaks up |
| S4 hump | 🟡 humps reported in the course runs (8-19 mm); not checked against ruler-measured heights |
| S4 lift stop | ✅ motors off at the end of every run that ended with a lift |
| S5 obstacle and bypass | ✅ many bypasses back onto the line (heading 1-4 deg); widths mostly 85-125 mm for the 10 cm box (twice 142 and 151, when the first scan heard echoes across the whole sweep); most stops 150-165 mm from the box, one at 131 mm |
| S5 junction moves from barcodes | ✅ LEFT, RIGHT, FORWARD and U-TURN all done from barcodes |
| Whole course | ✅ full sequences on 15 Nov (run 7) and 20 Nov; the formal "3 full runs" (SYS-01) not recorded |

## 13. How each part is tested

| Image | Tests |
|---|---|
| any test image (`make TEST=1`) | Stage 0: every task on time (automatic) |
| `images/is34_test_stage1.uf2` | S2: directions, STOP, 500 mm, turns; S4-S01 Motion hung |
| `images/is34_test_stage2.uf2` | S3: line following, junctions, barcodes (2A, B00, 2B, 2C) |
| `images/is34_test_stage3.uf2` | S5: servo, sonar view, width (S5-O01), bypass (S5-O02) |
| `images/is34_test_stage4.uf2` | S4: tilt view, hump height (S4-R01), lift stop (S4-L01) |
| `images/is34_test_stage6.uf2` | the whole course with the TEST mapping |
| `images/wifi/is34_mission_wifi.uf2` | the demo image (S1 and the whole course over WiFi). Built on the laptop and never committed: it holds the WiFi password |

How to run each stage: [TESTING_GUIDE.md](../../testing/TESTING_GUIDE.md).
Results, run by run: [TEST_STATUS.md](../../testing/TEST_STATUS.md).

## 14. Laptop tools

| Tool | What it is for |
|---|---|
| `tools/panel/is34_panel.py` (or `tools/panel/exe/is34_panel.exe`) | The control panel: START / STOP, the mapping, live status and sensors, the robot's log over WiFi, Save log, the Run summary ([PANEL.md](PANEL.md)) |
| `tools/sim/linesim.py`, `tools/sim/bypasssim.py` | The line-following and bypass simulators |
| `tools/logcheck/barcodecheck.py`, `tools/logcheck/sonarcheck.py` | Re-run the decoder or the width rule on a saved robot log |
| `tools/websim/` | The real firmware compiled to run in a browser on a simulated robot |
| `tools/barr/check.py` | The Barr C checker |

## 15. Words used in these pages

| Word | Meaning |
|---|---|
| task | one thread of the program; the kernel switches between tasks by priority |
| cyclic handler | a kernel timer that wakes a task at an exact period |
| event flag | a kernel object with bits; a task sleeps until another sets a bit |
| PWM, duty | the motor power, switched on and off 10,000 times a second; duty is the "on" share, in per mille (1000 = full) |
| encoder count | one step of a wheel's encoder: 0.080 mm of travel |
| quadrature | two encoder signals a quarter-step apart, so the direction can be told |
| feed-forward, PI | feed-forward sets the power expected for a speed; P adds power for the error now, I for the error that has built up |
| MQTT, broker | a message service: the robot and the panel both connect to the broker (Mosquitto, on the laptop) |
| Code 39 | a barcode where each character is 9 bars and spaces, 3 of them wide; `*` starts and ends a code |
| element | one bar or one space of a barcode |
| median | the middle value of several readings: one bad reading cannot move it |
| sin(pitch) | the nose-up tilt, from how gravity falls across the accelerometer |
| bit-field | a variable that takes only the bits its range needs (Change Requirement 1) |
