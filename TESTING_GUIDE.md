# P5-IS34 testing guide: every stage, step by step

Everything below is built and compiles with 0 warnings (1 Nov). **None of
Stages 1-6 has run on the robot yet.** Expect to tune the numbers marked
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

If the robot is too weak on USB for a test, run that test on the power
bank and read the result from the robot's behaviour, or move on to the
MQTT image, which sends every `[EVT]` line to the PC as well.

## 2. Numbers to measure once (ruler, 5 minutes)

They are guesses in the code. Measure them, change the define, rebuild.

| What | Define, file | Now |
|---|---|---|
| Track width, wheel centre to wheel centre | `S2_TRACK_MM`, `subsystem2/subsystem2.h` | 130 |
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
| `..._test_s4.uf2` | then Stage 4: IMU check, tilt view, 10 ramp runs |
| `..._test_s6.uf2` | then the whole course with the TEST mapping A=L, B=R, C=F, D=U |

Built by you (they need your WiFi and broker IP, section 6):

| Image | Command (in `build_make`) |
|---|---|
| Demo image: MQTT mapping, START/STOP, telemetry | `make NET=lwip MQTT=1` |
| Course test that waits for the PC's mapping | `make NET=lwip MQTT=1 TEST=1 TEST_STAGE=6` |

Build any image yourself: `make TEST=1 TEST_STAGE=n` (n = 1, 2, 3, 4, 6),
Pico SDK 2.2.0 (see `RTOS_STAGE0.md` section 1). Changing `TEST_STAGE`
rebuilds everything automatically.

Every test image first runs the 17 Stage 0 checks (about 25 s, all must
PASS), then prints `[DRIVE] ...` and **waits for START (GP20) before every
move**. Each move starts 1 s after you let go. **STOP (GP21) stops the
motors at once, at any time.**

## 4. Stage by stage

### Stage 1: motion + STOP (`_test_s1`), owner B2

1. Robot on the floor, tape mark along one side for its heading.
2. **S2-M02 turns**: 3 rounds of left 90, right 90, 180. After each round
   write down the real angles (protractor app). Pass: 90 +/- 10, 180 +/- 15.
   All turns off by the same factor = track width wrong: new
   `S2_TRACK_MM` = old x (commanded angle / real angle).
3. **S2-M03 slow speed**: 0.5 m at 150 mm/s. Pass: `[TEST] S2-M03 PASS`
   (mean within 10 %).
4. **S2-M04 STOP**: 5 tries, press STOP while it drives. Pass: 5/5 within
   500 ms.

If a wheel turns the wrong way, or it spins when it should drive: STOP,
send the log (a motor or encoder wire differs from bring-up).

### Stage 2: line, junctions, barcodes (`_test_s2`), owner B3 (+B5)

1. **IR view (15 s)**: slide black tape under each sensor; L, C and B
   must each change. Nothing changes = sensor or pot.
2. **S3-L01 / S3-L02**: robot on the line, START. It follows up to 60 s
   and prints each junction. Pass L01: `[TEST] S3-L01 PASS` (no line
   lost). L02: printed junction count = real count (write both down).
   If it wobbles off the line on curves: `STEER_SMALL` / `STEER_HARD` in
   `subsystem3/task_line.c`.
3. **S3-B01..B03**: 3 runs over stretches with barcodes, forwards and
   backwards; each barcode prints `[S3] barcode X` or `READ_FAIL`. Write
   down which codes were on the track. "A small" must read too
   (S3-B03): if it fails, lower `S2_SPEED_BARCODE` and rebuild.
4. **S5-B01 / S5-B02**: Mission takes over with the TEST mapping (A=L,
   B=R, C=F, D=U). START: it follows the line, reads barcodes and does
   the move at the next junction (`[EVT] junction ...`). Check a U-turn
   barcode too (D). For S5-B02 use a code not in the mapping (e.g. E):
   it must stop at the junction with "no mapping for barcode E".

### Stage 3: obstacle (`_test_s3`), owner B5

1. **Servo check**: at the first START prompt the sensor must point
   LEFT, at the second RIGHT. Wrong way: `S5_SERVO_LEFT_HIGH 0`, rebuild.
2. **Sonar view (10 s)**: hand in front, distances must follow.
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
2. **S4-R01**: 10 runs. Robot level, about 30 cm before the ramp, aimed
   straight at it. START: it calibrates 1 s, drives 1.5 m and prints
   `reported ramp height H mm`. Pass: within +/- 5 mm of the measured
   height in 10/10 runs. Systematic error: send me the 10 numbers.
   The `[EVT]` line also gives `L*sin(peak pitch)`, a second estimate.

### Stage 5: telemetry + MQTT (demo image), owner B1

Needs section 6 first. Power bank only. On the laptop:
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

## 7. Send back

For each test: the whole log, plus what you wrote down (real angles, real
widths, real junction count, which barcodes, measured ramp height).
