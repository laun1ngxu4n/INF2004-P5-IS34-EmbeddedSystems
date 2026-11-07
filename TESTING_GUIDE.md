# P5-IS34 testing guide: every stage, step by step

Everything below is built and compiles with 0 warnings (1 Nov). **Stage 1 passed on
the robot (1 Nov, S2_TRACK_MM set to 123); Stages 2-6 have not run yet.** Expect to tune the numbers marked
"NOT MEASURED" in section 2 after the first runs.

Test plan and test IDs: `testing/PLAN.md`. Record every run as a row in
the owner's `testing/subsystemN/TEST_LOG.md` and save the log under
`testing/subsystemN/logs/`.

## 1. Power: never USB and the power bank at the same time

Do not plug the laptop USB and the power bank (VIN) in together; we never
checked that the board allows it.

| Test | Power | Why |
|---|---|---|
| Stages 1-4 tests (they print the results) | **Laptop USB only**, long cable, laptop follows the robot | the log is on USB. The motors run on USB 5 V (bring-up motor and STOP tests ran on laptop USB); slower, but the speed controller makes up for it |
| Stage 5-6 / demo (results over WiFi) | **Power bank on VIN only**, no USB | results arrive on the PC over MQTT |
| **Any stage over WiFi** (`images/wifi/is34_test_stageN_wifi.uf2`, since 7 Nov) | **Power bank on VIN only**, no USB | the whole log arrives in the panel's **Robot log** tab; "Save log..." writes it to a file. START / STOP on the panel answer the test prompts too |

If the robot is too weak on USB for a test, run that test on the power
bank and read the result from the robot's behaviour, or move on to the
MQTT image, which sends every `[EVT]` line to the PC as well.

## 2. Numbers to measure once (ruler, 5 minutes)

They are guesses in the code. Measure them, change the define, rebuild.

| What | Define, file | Now |
|---|---|---|
| Track width, wheel centre to wheel centre | `S2_TRACK_MM`, `subsystem2/subsystem2.h` | 123 (from S2-M02, 1 Nov) |
| Robot width (outside of wheels) / length | `S2_ROBOT_WIDTH_MM`, `S2_ROBOT_LENGTH_MM`, same file | 150 / 200 |
| Line sensors in front of the wheel axle | `S3_SENSOR_TO_AXLE_MM`, `subsystem3/subsystem3.h` | 60 |
| Axle to front support (for L x sin, reported beside the main height) | `S4_L_MM`, `subsystem4/subsystem4.h` | 80 |
| Lower practice ramp: height, length, flat top? | write in `TEAM.md` section 7 | - |

Also tuned from tests: `S5_SERVO_LEFT_HIGH` (Stage 3 servo check),
`S5_BEAM_HALF_DEG` (S5-O01 widths), `S2_SPEED_BARCODE` (S3-B03).

## 3. Images

Prebuilt (no WiFi details needed):

| Image | Runs |
|---|---|
| `..._test_s1.uf2` | Stage 0 auto tests, then Stage 1: turns, slow speed, STOP |
| `..._test_s2.uf2` | then Stage 2: IR view, line + junctions, barcodes, then the course with the TEST mapping |
| `..._test_s3.uf2` | then Stage 3: servo check, sonar view, 15 width scans, then the course (bypass) |
| `..._test_s4.uf2` | then Stage 4: IMU check, tilt view, 5 ramp runs |
| `..._test_s6.uf2` | then the whole course with the TEST mapping A=L, B=R, C=F, D=U |

Built by you (they need your WiFi and broker IP, section 6):

| Image | Command (in `build_make`) |
|---|---|
| Demo image: MQTT mapping, START/STOP, telemetry | `make NET=lwip MQTT=1` |
| Course test that waits for the PC's mapping | `make NET=lwip MQTT=1 TEST=1 TEST_STAGE=6` |
| **Any stage test with its log over WiFi** (no USB cable) | `make NET=lwip MQTT=1 TEST=1 TEST_STAGE=n`, then copy `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt_test_sn.uf2` to `images/wifi/is34_test_stagen_wifi.uf2` (git-ignored: it holds the WiFi password) |

Build any image yourself: `make TEST=1 TEST_STAGE=n` (n = 1, 2, 3, 4, 6),
Pico SDK 2.2.0 (see `RTOS_STAGE0.md` section 1). Changing `TEST_STAGE`
rebuilds everything automatically.

Every test image first runs the 17 Stage 0 checks (about 25 s, all must
PASS), then prints `[DRIVE] ...` and **waits for START (GP20) before every
move**. Each move starts 1 s after you let go. **STOP (GP21) stops the
motors at once, at any time.**

## 4. Stage by stage

### Gates: a console PASS is not enough

Move to the next step only with the evidence in this table. Every
`[TEST] PASS` must be backed by the physical result it stands for, and
every test image re-runs the Stage 0 timing checks on the exact firmware
you are testing.

| Gate | Evidence before going on |
|---|---|
| Stage 0 | the 17 auto checks PASS at the start of the image under test |
| Stage 1 | right directions and signs (lifted), STOP works (lifted, then 5/5 on the floor), real 0.5 m distance measured, real turn angles measured |
| Stage 2A | at least 3 m of line following without loss; junction count = real count |
| Stage 2B | the codes placed on the track, in order, decoded in both directions |
| Stage 2C | the right move at each junction, U-turn, and the stop for an unmapped code (obstacle watch and IMU are OFF here) |
| Stage 3 | servo direction, widths of known obstacles inside W +/- U, a clean bypass; a covered sonar must stop the run |
| Stage 4 | measured ramp height, 5 runs within +/- 5 mm |
| Stage 5 | mapping and telemetry with the robot STILL first, then START/STOP while it moves |
| Stage 6 | repeated full runs, with logs and what you saw |

Stages 3 and 4 only need Stage 1 (and the line for the bypass); they can
be done in either order. All subsystems still start in every image:
`TEST_STAGE` only picks the hands-on part.

### Stage 1: motion + STOP (`_test_s1`), owner B2

Order (review, 1 Nov): wheels lifted first, then the floor.
1. **S2-M05 directions, wheels LIFTED**: both wheels forward 300 mm, then
   back. Watch that both turn the robot's forward way first. Pass: PASS
   line, and what you saw matches.
2. **S2-M06 STOP, wheels lifted**: press STOP after a second. Pass: wheels
   stop at once.
3. Robot on the floor from here. **S2-M03**: mark the front, it drives
   500 mm at 150 mm/s; **measure the real distance**. Pass: PASS line
   (speed) and the real distance 475-525 mm. Otherwise the mm/count scale
   is off: send the number.
4. **S2-M02 turns**: 3 rounds of left 90, right 90, 180 from a tape mark.
   The PASS line only means the commands finished; **pass/fail is your
   measured angles**: 90 +/- 10, 180 +/- 15. All off by the same factor =
   track width wrong: new `S2_TRACK_MM` = old x (commanded / real angle).
5. **S2-M04 STOP**: 5 tries on the floor, press STOP while it drives.
   Pass: 5/5 within 500 ms.

If a wheel turns the wrong way, or it spins when it should drive: STOP,
send the log (a motor or encoder wire differs from bring-up).

### Stage 2: line, junctions, barcodes (`_test_s2`), owner B3 (+B5)

0. **Before flashing: sensor layout** (see `subsystem3/subsystem3.h`).
   The two line sensors **straddle the tape**: L (GP1) just left of it,
   R (GP0) just right of it, about 30 mm apart centre to centre, the
   tape centred between them (both white). B is 40 mm behind them and
   70 mm left of L; if you move it, change `S3_BAR_AHEAD_MM` /
   `S3_BAR_LEFT_MM` (behind = negative) and rebuild.
1. **IR view (30 s)**: slide black tape under each sensor; L, R and B
   must each change. Nothing changes = sensor or pot. Robot centred on
   the tape: L 0 and R 0.
   **B is on its analogue output since 7 Nov** (AO to GP28, module on
   **3V3, not 5 V**); every image is built that way (`S3_BARCODE_ANALOG 1`)
   with fixed thresholds (black from 1300, white below 800: printed bars
   read much greyer than tape, S3-B00 on 7 Nov). B's pot does nothing.
   The live view prints the lowest and highest reading each second:
   white (paper, foam) must stay under 800, printed bars over 1300. If
   not, send the lines to
   B3. B shows 1 over white = `S3_BAR_BLACK_HIGH 0`. Back to the digital
   D0 on GP2: `S3_BARCODE_ANALOG 0` and rebuild every image.
2. **2A, S3-L01 / S3-L02**: robot on the line, START, let it run (up to
   60 s). Pass L01: `[TEST] S3-L01 PASS` = at least 3 m followed with no
   line lost, and you saw it stay on the line. L02: printed junction count = real count (write both down).
   Each junction prints `[S3] junction N at X mm: crossing` or `: branch
   left (seen by B)`; at the end `junctions this run: crossings C, left
   branches L`. Start with the tape centred between L and R.
   If it wobbles off the line on curves: `STEER_SMALL` / `STEER_HARD` in
   `subsystem3/task_line.c`.
   **One place at a time**: then 7 short runs, each prompts where to put
   the robot: 20 cm before the X crossing (S3-L10), a left-only branch
   (L03), a T end (L05); the start of the tightest right curve (L06) and
   left curve (L07); 20 cm before a gap in the line on track 2 (L08); and
   on a straight with a black card held under R once it moves (L09, it
   must stop with `LINE SENSOR FAULT`). Each prints its own PASS/FAIL. No
   such place on the track: START, then STOP at once, and write "not on
   the track" in the log.
   Left branches: every first bar under B prints `B patch 19.8 mm (...)`;
   a left branch must show about 20 mm. Nothing printed = B does not pass
   over the branch (move B, or its pot).
   Each part of Stage 2 (2A, S3-B00, 2B, 2C) first asks: **START = run
   it, STOP = skip it**, so you can go straight to the barcodes.
3. **S3-B00, barcode read driving straight** (do this before 2B): 4 runs,
   each drives 300 mm straight with no line following. Put the robot so
   **B** (70 mm left of the line sensors) is about 5 cm before the first
   bar, square to the bars; where the line is does not matter. Runs: a
   code; the same code from the other end; "A small" (all at 50 mm/s, the
   speed inside a code); any code at the follow speed 120 mm/s (the robot
   only slows once B is on the first bar, so this shows whether the start
   still reads). Each prints `code ended, N elements
   (mm): ...` (groups of 10 = one character), the decode, and a `B
   profile`: every bar `B` and space `s` with its width and its darkest
   or lightest reading. `!` = a bar that never reached black (1300) or a
   space that never got back to white (800): the sensor's spot is wider
   than that bar or space. Then B must be lower (closer to the floor), or
   the thresholds moved: send the log to B3. Pass: `S3-B00 PASS` on runs
   1-3.
4. **2B, S3-B01..B03**: before each run **write down the codes on the
   track in the order the robot meets them**; 3 runs, forwards and
   backwards. Each prints `[S3] barcode X` or `READ_FAIL`. Pass: the printed
   sequence equals your written one ("it decoded something" is not a pass). "A small" must read too
   (S3-B03): if it fails, lower `S2_SPEED_BARCODE` and rebuild.
   Each code is read in **one pass** (no backing up since 7 Nov): it
   slows to 50 mm/s on the first bar and holds the steering straight
   while B is over the bars. A missed code is not read again.
5. **2C, S5-B01 / S5-B02**: Mission takes over with the TEST mapping (A=L,
   B=R, C=F, D=U), with obstacle watch and IMU **off** (they are tested in
   Stages 3 and 4). START: it follows the line, reads barcodes and does
   the move at the next junction (`[EVT] junction ...`). Check a U-turn
   barcode too (D). For S5-B02 use a code not in the mapping (e.g. E):
   it must stop at the junction with "no mapping for barcode E".

### Stage 3: obstacle (`_test_s3`), owner B5

1. **Servo check**: at the first START prompt the sensor must point
   LEFT, at the second RIGHT. Wrong way: `S5_SERVO_LEFT_HIGH 0`, rebuild.
2. **Sonar view (10 s)**: hand in front, distances must follow. `-1` =
   nothing in range, `-2` = no ECHO at all (sensor or wiring fault).
   Failure policy: during a run, 5 pings in a row with no ECHO stop the
   robot ("ultrasonic not answering"); a scan ray with no ECHO counts as
   blocked, never as open space. Check it once: cover/unplug ECHO during
   S5-O02 and the robot must stop.
3. **S5-O01 width**: 15 tries, obstacle about 250 mm straight ahead.
   Use 100, 150 and 200 mm wide obstacles, 5 each. Each try scans,
   reverses 80 mm, scans again and prints
   `width W +/- U mm (scan 1 ..., scan 2 ...)` plus the raw rays
   (`[SCAN] rays ...`). Write down the real width. Pass: the real width
   is inside W +/- U. If every width is too big (or too small) by about
   the same amount, raise (or lower) `S5_BEAM_HALF_DEG`.
4. **S5-O02 bypass**: obstacle on the line, START. It stops, scans
   twice, reports the width, drives round on the clear side and finds the
   line again (`[EVT] bypass done`). Pass: no contact, stays within 300 mm
   of the line, back on the line within 500 mm after the obstacle.

### Stage 4: ramp height (`_test_s4`), owner B4

0. First measure the lower practice ramp (height, length, flat top?).
1. `[TEST] S4-I01 PASS` = IMU found. Tilt view: lifting the front must
   change `sin(pitch)`.
2. **S4-R01**: 5 runs. Robot level, about 30 cm before the ramp, aimed
   straight at it. START: it calibrates 1 s, drives 1.5 m and prints
   `reported ramp height H mm`. Pass: within +/- 5 mm of the measured
   height in 5/5 runs. Systematic error: send me the 5 numbers.
   The `[EVT]` line also gives `L*sin(peak pitch)`, a second estimate.

### Stage 5: telemetry + MQTT (demo image), owner B1

Needs section 6 first. Power bank only. **Robot on a stand (wheels
lifted) for steps 1-2**, on the floor only for step 3. On the laptop:
```sh
mosquitto_sub -h <broker IP> -t 'is34/#' -v
```
1. **S1-T01**: every second an `is34/telemetry` line appears (state,
   speed, distance, barcode, mapping, ramp peak). Every `[EVT]` result
   appears on `is34/event`.
2. Mapping: `mosquitto_pub -h <broker IP> -t is34/cmd/map -m "K=L,Q=R,X=F,M=U"`
   gives `is34/ack/map OK K=L,...`. A bad one (e.g. `K=L`) gives
   `REJECTED ...` and keeps the old mapping.
3. **S1-T02**: `mosquitto_pub ... -t is34/cmd/start -m go` starts the run,
   `... -t is34/cmd/stop -m now` stops it. 5 of each.

### Stage 6: whole course (demo image or `_test_s6`), whole team

**SYS-01**, 3 full runs: send the 4-character mapping, START, course with
barcodes, junctions, an obstacle and the ramp. Pass: every move at its
junction, obstacle passed with its width reported, ramp height reported,
line found after every manoeuvre.

## 5. What the robot does (demo image)

`IDLE` -> START (GP20 or `is34/cmd/start`) -> `CAL` (IMU level, 1 s) ->
`RUN`: follows the line at 150 mm/s.
- Barcode -> stores the character, reports it and the planned move.
- Junction -> stored character's move (L / R / F / U-turn); none stored
  = straight; stored but not in the mapping = stop "no mapping".
- Obstacle closer than 250 mm -> stop, scan, reverse 80 mm, scan,
  report width +/-, bypass on the clear side, find the line, carry on.
- Ramp -> when level again, reports the peak height.
- Line lost / blocked bypass -> stops and reports. STOP -> `IDLE`.

## 6. WiFi + MQTT setup (for the demo image)

1. Phone hotspot on 2.4 GHz; laptop on the same hotspot.
2. On the laptop run Mosquitto with `bringup/tools/mosquitto.conf`.
3. Copy `config/wifi_secrets.example.h` to `config/wifi_secrets.h` and
   fill in SSID, password and the laptop's IP as `MQTT_BROKER_IP`.
   (This file is git-ignored: never commit it.)
4. `cd build_make && make NET=lwip MQTT=1` and flash
   `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`.
5. On the laptop: `pip install paho-mqtt`, then `python tools/is34_panel.py`.
   The panel shows the status line and events, and has START, STOP
   (Space) and mapping buttons; it logs everything to `tools/is34_*.log`.
   "Start broker" runs Mosquitto on port 1884
   (`tools/mosquitto_robot.conf`): use that port in `wifi_secrets.h` if
   the Mosquitto Windows service already holds 1883 for localhost only.
6. **Tests over WiFi** (any stage, no USB cable): flash
   `images/wifi/is34_test_stageN_wifi.uf2`, power bank on VIN only. The
   panel's **Robot log** tab shows the same text as the USB console, from
   boot on (the robot keeps 8 KB until the WiFi is up). Prompts that say
   "press START" also take START on the panel; the Stage 2 "run it / skip
   it" questions take START / STOP on the panel. **Save log...** writes
   everything since the last **Clear** to a text file: send that file. It
   is also saved as it arrives to `tools/is34_<date>_<time>_robot.txt`.
   Lines `[panel: N log messages lost on the WiFi]` or `[panel: the
   robot dropped N bytes]` mark gaps (WiFi drop-outs, or a burst bigger
   than the robot's buffer). Stage 2C and 6 over WiFi wait for a mapping
   from the panel: for the TEST mapping send `A=L,B=R,C=F,D=U`.

## 7. Send back

For each test: the whole log, plus what you wrote down (real angles, real
widths, real junction count, which barcodes, measured ramp height).
