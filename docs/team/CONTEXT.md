# P5-IS34 robot car: handover / context

Read this first if you are picking the project up: a teammate, a new
chat, or another AI. It says what exists, what has been proven on the
robot, what has not, and where everything is. Dates are 2026.

## 1. The project in one paragraph

INF2004 team P5-IS34 builds a line-following robot car on a Raspberry Pi
Pico W (Cytron Robo Pico board) running **micro T-Kernel 3.0** (course
template, single core). It follows a black line, reads Code 39 barcodes
beside the track and does the mapped move (left / right / forward /
U-turn) at the next cross-junction, profiles an obstacle (estimates and
reports its width +/- uncertainty) and drives round it, estimates the
height of a ramp from the IMU, and reports everything to a PC over
WiFi/MQTT. The 4-character barcode mapping is sent from the PC just before
the demo.

Team (subsystem owners, `TEAM.md` section 1):

| Subsystem | Owner | Code |
|---|---|---|
| S1 Telemetry / WiFi / MQTT (+ notification) | B1 Muhammad Ikhwan | `app_program/subsystem1/` |
| S2 Motion: motors, encoders, speed control, turns | B2 Ng Chern Loong Branson | `app_program/subsystem2/` |
| S3 Line following, junctions, barcode decoder | B3 Lau Ning Xuan | `app_program/subsystem3/` |
| S4 IMU ramp height (+ safety, not built) | B4 Muhammad Haziq | `app_program/subsystem4/` |
| S5 Obstacle scanner + Mission controller | B5 Muhammad Irfan | `app_program/subsystem5/` |

## 2. Where things are

- Repository: fork `github.com/mhdikhwan/kwanxkwan`, branches **`main`**
  and **`rtos-stage0`** (kept identical). The team's original repo is
  `laun1ngxu4n/INF2004-P5-IS34-EmbeddedSystems`; merge by a pull request
  from the fork's `main` (the RTOS layout moved the old bring-up code
  into `bringup/`, tell the team first).
- `CONTEXT.md` (this file), `TESTING_GUIDE.md` (how to run every stage),
  `RTOS_STAGE0.md`, `RTOS_STAGE1.md`, `TEAM.md` (pins, owners,
  priorities, interfaces, **section 7 = every requirement change since
  the Week 6 design**), `testing/PLAN.md` (which tests, which dropped),
  `testing/*/TEST_LOG.md` (test cases + results), `bringup/` (hardware
  bring-up tests and `bringup/docs/bringup_results.md`).
- The submitted design documents (Week 6 design review, v3, Tribunal) are
  Word files outside the repo. They are not edited; `TEAM.md` section 7
  overrides them.

## 3. Status (8 Nov, evening)

The test status, stage by stage, is in
[`testing/TEST_STATUS.md`](../../testing/TEST_STATUS.md) (the record; this
file does not repeat it). In short: Stages 0, 1, 3 and 5 passed on the
robot; 2 (line, barcodes, junction moves) and 4 (hump) work but not every
run; 6 (whole course) not run yet.

Since the last robot runs (8 Nov) the code was brought to the **Barr C
coding standard** (8 Nov evening, commits `f639186`..`79b9ca4`; deviations and
how it was checked: [`BARR_C_DEVIATIONS.md`](BARR_C_DEVIATIONS.md)).
Most steps were proven byte-identical, but `bool` instead of `BOOL` and
the if-nesting refactors changed the code of every stage, so **every
stage needs one confirmation run** (the next steps below). The robot
should behave exactly as on 8 Nov; one real fix: a drive or turn at
speed 0 is now refused (it was undefined behaviour; nothing passes 0).

### Next steps (in this order)

1. ~~Change Requirement 1, Minimum-Bit Data Representation~~: **done
   in the code, 8 Nov evening** ([`CHANGE_REQ1.md`](CHANGE_REQ1.md): what
   changed, the data table, RAM and message sizes, the laptop checks;
   binary MQTT format: [`MQTT_WIRE_FORMAT.md`](MQTT_WIRE_FORMAT.md)).
   The panel must be the updated one (it decodes the binary messages).
   New behaviour: Safety stops the motors for a stalled wheel or a hung
   Motion task.
2. **Full stage retest** on the result (one run per stage at least,
   `testing/TESTING_GUIDE.md`; WiFi images in `images/wifi/`, power bank
   only): full course with the mission image, then Stage 2 (S3-B00 x2,
   one 2B run, 2C), Stage 1, Stage 3 (one width, covered sonar, one
   bypass), Stage 4 (hump runs), plus the Change Requirement 1 tests
   (`CHANGE_REQ1.md` section 7.2: timing tables packed / unpacked, Wi-Fi
   drop, STOP in a burst, a held wheel, mapping). This covers the
   Barr C clean-up as well. A failure: save the log; the images from
   before the clean-up are in git at `61cc060` (USB images only).
3. **Turn tuning** (turns end a little past the angle), during the
   retest: run S2-M02 on the course floor, write down the 9 real angles
   next to the printed encoder angles. Encoder angle over the target =
   coasting after the motors switch off (end the turn early or brake);
   real over encoder = `S2_TRACK_MM` (new = 123 x encoder / real).

## 4. Hardware (measured in bring-up)

- Pico W on Cytron Robo Pico. Pins: `TEAM.md` section 2. Summary: IR
  left GP0, IR centre GP1, IR barcode GP2 (1 = black); encoders GP4/5
  (left) and GP16/17 (right); TRIG GP6, ECHO GP7; motors GP8-11
  (forward = A PWM, B low, after the 26 Oct M1 wire swap); servo GP14;
  START GP20, STOP GP21 (active low); IMU I2C1 GP26/27; GP28 not used.
- Encoders about 2,540 counts/rev, 65 mm wheel, 0.080 mm/count; the left
  one counts down for forward (the code negates it).
- Speeds, wheels lifted: 240 / 325 / 407 / 573 / 798 mm/s at 30 / 40 /
  50 / 70 / 100 % duty.
- IMU board "GY-511" carries an **LSM303D** (0x1D, WHO_AM_I 0x49):
  accelerometer + magnetometer, no gyro. +/-4 g. Its Z reads 0.12-0.15 g
  high: calibrate level at standstill (done at every START).
- HC-SR04 (5 V module) works at 3.3 V, about 50 mm to 1.5 m.
- Barcode IR sensor replaced on 26 Oct (reads slightly better, not
  re-measured).
- Power: USB power bank into VIN for runs. **Never USB and power bank
  together** (not checked). Tests that need the USB log run on laptop USB
  only (motors work on USB 5 V).

## 5. Software

### RTOS frame (Stage 0)
- micro T-Kernel 3.0, **single core** (`SMP=0`, team decision; the lock
  `APP_LOCK` = interrupts off and refuses to build with SMP=1).
- Kernel tick 10 ms. Kernel's own USB console task priority 3 and WiFi
  task priority 4 sit above ours.
- 8 tasks (`app_program/app.h`):

| Task | Pri | Period | Does |
|---|---|---|---|
| safety_task (S4) | 5 | 20 ms | stall and Motion-hung stop (Change Requirement 1); IMU collision removed 13 Nov |
| motion_task (S2) | 6 | 10 ms | encoders -> speed -> PI per wheel -> PWM; drive / turn / follow commands |
| mission_task (S5) | 7 | event, 100 ms | the course state machine |
| line_task (S3) | 8 | 20 ms | IR -> steering (IF-03), junction / line-lost events |
| imu_task (S4) | 9 | 10 ms | LSM303D over I2C1, pitch, ramp height |
| barcode_task (S3) | 10 | 10 ms timeout | edge ring -> Code 39 decode, mapping |
| obstacle_task (S5) | 11 | 60 ms | forward ping watch; the profiling scan runs in Mission's call |
| telemetry_task (S1) | 12 | 100 ms | status line 1 s, IF-01 ping, timing table every 10 s |

- Shared helpers (whole team): `app_lock.h` (the one lock),
  `app_timing.[ch]` (per-task timing table), `app_gpio.[ch]` (the one GPIO
  interrupt dispatcher, IO_IRQ_BANK0; also `app_gpio_hw_release()` because the
  kernel leaves PWM and I2C1 **in reset**), `app_stop.[ch]` (the one STOP
  helper: `app_stop_request(reason)` turns the motors off at once from any
  context; Motion refuses to drive while any reason is set).
- Interfaces (`TEAM.md` section 5): IF-01 `flg_mission` event flag
  (bits in `subsystem5.h`: STOP, START, JUNCTION, LINE_LOST, BARCODE,
  OBSTACLE, PING); IF-02 STOP helper; IF-03 `line_steer_mm_s()` latest
  steering; IF-04 `motion_get_status()` (command id + result);
  IF-05 MQTT through `app_mqtt_rx()` / `app_mqtt_tx()`.

### What each stage added
- **Stage 1 (S2)**: PWM 10 kHz; quadrature decoding in the interrupt;
  speed = counts over 40 ms; feed-forward + PI per wheel; ramp up and a
  slow-down curve; a term that keeps both wheels at equal distance;
  `motion_drive(speed, mm)`, `motion_turn(deg)` (+ = left, spin on the
  spot), `motion_follow(speed)`, `motion_halt()`. Every command times out
  at 2x expected + 1 s. GP21 STOP interrupt -> `app_stop_request`.
- **Stage 2 (S3)**: line logic in `line.c`. Since 7 Nov the two
  line sensors straddle the tape (L GP1 left, R GP0 right, about 30 mm
  apart): both white = straight, one black = turn toward it (gentle,
  then hard and slow after 25 mm), overshoot search after a long
  correction, lost after 400 mm of white. Junctions: crossing = both
  black 12-45 mm; left branch = the barcode sensor B (40 mm behind,
  70 mm left) sees a patch about one tape wide where the robot drove
  straight. Mission turns 115 mm after the junction centre. Barcode: each GP2 edge stamped with the odometer (bar
  widths in distance, so speed does not matter), chatter < 0.5 mm merged,
  end of code after 40 mm quiet, Code 39 table (A-Z, 0-9, symbols),
  3 widest of 9 = wide, `*` start check, reversed if needed. Mapping
  parser: exactly 4 pairs `K=L,Q=R,X=F,M=U`, each char and move once.
- **Stage 3 (S5)**: forward watch trips at 175 mm, stops about 160 mm away (190 from 15 to 20 Nov; Mission speed 120 mm/s; 135 from 15 to 18 Nov, never run) (2 readings; 250 until 15 Nov).
  Scan 40-140 deg in 5 deg steps, servo settle 3 ms/deg (>= 60 ms),
  5 pings per ray, median; open space = no echo or farther than front +
  250 mm; on the obstacle while within 150 mm of the front distance; edge
  after 2 rays off it; edge angle pulled in by `S5_BEAM_HALF_DEG` for the
  sonar cone; +/- from the angle step and a 3 deg beam allowance. Two
  scans (second after reversing 80 mm) are averaged, +/- at least half
  their difference. Bypass plans for max(width + unc, 200 mm) and goes
  round the clear side with the nearer edge: turn 90, out, turn back,
  past, turn toward the line, drive until the centre sensor sees it,
  wheels onto the line, turn back, follow. Ahead is pinged before the out
  and past legs.
- **Stage 4 (S4)**: own polled I2C1 driver (`imu_i2c.c`, 400 kHz). Pitch
  without libm: sin(pitch - level pitch) from the accelerometer, low-pass
  about 100 ms. Height = sum of (distance driven x sin(pitch)), so it works
  for a long ramp and a short hump. When level again for 100 mm, a peak of
  8 mm or more is reported (`[EVT] ramp peak height ...`, plus L x
  sin(peak pitch)), then no second report for 600 mm (the way down).
- **Stage 5 (S1)**: `telemetry_event()` prints `[EVT] ...` and queues it for
  MQTT `is34/event`; status line on `is34/telemetry` every 1 s;
  commands `is34/cmd/map|start|stop`; mapping ack on `is34/ack/map`.
  `mqtt_utk.c` (kernel WiFi task) now subscribes to `is34/cmd/#`, calls
  `app_mqtt_rx()` for each message and publishes one outbox message every
  100 ms via `app_mqtt_tx()`; `lwipopts.h` got 2 more timeout slots.
- **Stage 6 (S5)**: `mission.c` state machine (see
  `TESTING_GUIDE.md` section 5). Follows at 120 mm/s everywhere (slow for
  barcodes; 150 until 7 Nov). Test images arm Mission only for the course part, so their
  START presses don't start a run.

### Tests
`make TEST=1 TEST_STAGE=n` (n = 1, 2, 3, 4, 6): Stage 0 auto checks, then
the hands-on test of stage n (`app_program/app_test.c` and each
`subsystemN_test.c`). Steps and pass criteria: `TESTING_GUIDE.md`.

## 6. Requirement changes after the Week 6 design (`TEAM.md` section 7)

For the next report, `docs/week6_report_changes.md` goes through the
Week 6 document section by section: what to change, add or drop.

- GP28 battery sense removed (no low-battery fault, no VBAT, no A12
  thresholds).
- Barcode (instructor): Code 39 A-Z, both directions, store the
  character, do the move (incl. U-turn) **at the cross-junction**, the
  4-character mapping comes over MQTT and is not hard-coded (no mapping =
  stop and report). Slow speed over barcodes; widths by encoder distance.
- Obstacle (instructor): 10-20 cm wide; profiling not metrology; settle
  wait, median of several pings, background, hysteresis, 2 positions,
  plan for the largest width, report width +/- uncertainty, safe path,
  rejoin the line.
- Ramp (instructor): only the lower practice ramp height matters.
- IMU part is the LSM303D, not the LSM303DLHC the design names.
- **IMU (lecturer, 13 Nov): collision detection removed; lifting or
  tipping the robot over must stop the motors** (lifting ends the demo
  run). Built 13 Nov: `docs/team/course/IMU_REQUIREMENT_CHANGE.md`.
- Test plan "essentials only" (`testing/PLAN.md`): dropped tests are
  listed there and must be reported as "not tested".

## 7. Decisions and why

- Single core: simpler locking; the template's dual core adds no needed
  speed. Switching later needs a spinlock in `APP_LOCK`.
- Bar widths by encoder distance, not time: speed changes over a barcode
  don't matter.
- STOP turns the PWM off inside the button interrupt, and Motion's
  writes check the STOP reasons under the lock, so a task can never undo
  a STOP.
- No libc/libm calls in the app (freestanding build): own sqrt, sin and
  string compare.
- The MQTT code stays in the kernel's WiFi task (template rule); the app
  only copies data in and out through two hooks.

## 8. Guesses to tune after the first runs

| Value | Where | Now | Tuned by |
|---|---|---|---|
| Track width | `S2_TRACK_MM` | 123 mm (from S2-M02 angles, 1 Nov) | ruler, or S2-M02 angles |
| Speed controller | `FF_*`, `KP`, `KI` in `motion.c` | FF 1.25/mm/s + 80, KP 1, KI 6 (per mille) | wobble or sluggish speed |
| Line steering | `STEER_SMALL/HARD` in `line.c` | 25 / 40 mm/s (2 Nov) | S3-L01 |
| Junction length | `S3_JUNC_MIN_MM`, `S3_CROSS_MAX_MM` | 12-45 mm crossing (tape 20 mm) | S3-L02, L05, L10 |
| Sensor to axle | `S3_SENSOR_TO_AXLE_MM` | 115 mm (measured 6 Nov) | junction turns land off the line |
| Barcode sensor position | `S3_BAR_AHEAD_MM`, `S3_BAR_LEFT_MM` | -40 (behind) / 70 mm (measured 6 Nov) | S3-L03, S3-L05 |
| Barcode speed | `S2_SPEED_BARCODE` | 150 mm/s | S3-B03 |
| Servo direction / scale | `S5_SERVO_LEFT_HIGH`, `S5_SERVO_US_PER_DEG` | 1 / 11 | Stage 3 servo check |
| Sonar cone pull-in | `S5_BEAM_HALF_DEG` | 6 deg | S5-O01 |
| Robot size | `S2_ROBOT_WIDTH_MM/LENGTH_MM` | 170 / 200 (6 Nov; front bar not measured) | ruler |
| IMU L | `S4_L_MM` | 80 | ruler (secondary estimate only) |

## 9. Not built / open

- Safety task: collision (2 g) is **removed** from the requirements
  (lecturer, 13 Nov; the IMU stops the motors when the robot is lifted
  instead). Stall and fault latching (FR9 minus battery) were not in the
  essentials plan; `safety_task` is the Stage 0 stub.
  Change Requirement 1 and the tribunal (Safety cuts the motors if
  Motion hangs) need it: `CHANGE_REQ1.md` section 8 (stall and Motion-hung are now built).
- Notification (NeoPixel / buzzer) not built.
- NFR measurements (CPU load, stack use, reconnect trials) not done.
- Questions for the instructor: may we change `mqtt_utk.c` and
  `lwipopts.h` (the template's WiFi code)? WiFi task priority 4 above our
  tasks? The mapping message format (our draft `K=L,Q=R,X=F,M=U` on
  `is34/cmd/map`)? Are all four moves (U-turn too) done at the junction?
  Week 10 check format?
- Measured Stage 0 findings: the WiFi radio start pauses every task about
  120 ms at 0.6 s after boot; printing a lot on USB delays tasks ~12 ms
  (keep USB output small); +/-2.5 ms jitter on all tasks (cause unknown);
  a 10 ms kernel timeout lasts up to 20 ms.

## 10. Working conventions

- Only the owner edits a subsystem folder; shared files (`app_*.c/h`,
  `TEAM.md`) by team agreement.
- Style: the **Barr C** coding standard (`BARR_C_DEVIATIONS.md`):
  `int32_t` / `uint32_t` / `bool` / `float32_t`, Barr names (`g_` globals,
  `p_` pointers, `b_` booleans, `h_` kernel IDs), public functions prefixed
  with their module (`line_...`, `barcode_...`), every cast commented,
  braces always, `if` nesting at most 2. Formatting:
  `app_program/.clang-format`. Check before committing:
  `python tools/barr/check.py` (0 for every rule but the deviations).
- A change meant not to alter the program (formatting, renames): prove it
  with `bash tools/barr/compare.sh save <dir>` before and `check <dir>`
  after (all 12 images byte-identical).
- Interrupt callbacks: a few lines, never wait, no printing.
- Every test row goes in `testing/*/TEST_LOG.md` with the commit, tester
  and log file; never invent results; bench results are bench results.
- Commit to `main`, fast-forward `rtos-stage0`, push both. Rebuild the
  images after a code change (`images/*.uf2` committed with `git add -f`;
  `images/wifi/` never: it holds the WiFi password).

