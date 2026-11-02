# P5-IS34 test status checklist

Last updated: 2 Nov 2026. How to run each test:
[TESTING_GUIDE.md](TESTING_GUIDE.md). Formal results:
[testing/](testing/) `TEST_LOG.md` files.

Legend: ✅ passed on the robot · 🟡 works, but not to the pass mark yet ·
⬜ not tested yet · ❌ tested and failing

## Summary

| Stage | What | Owner | Status |
|---|---|---|---|
| 0 | RTOS timing, all 8 tasks | B1 | ✅ passed 26 Oct |
| 1 | Motion + STOP | B2 | ✅ passed 1 Nov |
| 2A | Line following + junctions | B3 | 🟡 follows well, 3 m not shown yet |
| 2B | Barcodes | B3 | 🟡 partial reads, no full decode yet |
| 2C | Junction moves from barcodes | B3 + B5 | ⬜ needs 2B |
| 3 | Obstacle: sonar, servo, width, bypass | B5 | 🟡 servo + sonar work; width and bypass not shown |
| 4 | IMU + ramp height | B4 | 🟡 IMU works; height reads too high on the hump |
| 5 | Telemetry + MQTT | B1 | ⬜ |
| 6 | Whole course | all | ⬜ |

## Stage 0: RTOS timing (`make TEST=1`)

- [x] ✅ S1/S2/S3/S4/S5-U01..U04, INT-01, INT-02: every task wakes on time,
  no overruns (26 Oct, `testing/integration/logs/2026-10-26_stage0_test.txt`)
- [x] ✅ Kernel tick fix (`0786f1a`): after a power-on or RUN reset the system
  timer ran about 43x slow. Fixed 2 Nov. **Everyone: pull before testing.**

## Stage 1: motion + STOP (`TEST_STAGE=1`)

- [x] ✅ S2-M05 wheel directions and encoder signs (wheels lifted)
- [x] ✅ S2-M06 STOP with the wheels lifted
- [x] ✅ S2-M03 500 mm at 150 mm/s: measured about 500 mm
- [x] ✅ S2-M02 turns: with `S2_TRACK_MM` 123, 90° about right, 180° about
  5° over (limit ±15°)
- [x] ✅ S2-M04 STOP on the floor

Still to do: write the Stage 1 rows into `testing/subsystem2/TEST_LOG.md`.

## Stage 2: line, junctions, barcodes (`TEST_STAGE=2`)

### What works
- [x] ✅ IR live view: S, C and B all respond; B prints edges per second
  so you can tune its pot
- [x] ✅ Steering direction: the side sensor is on the RIGHT of C
  (`S3_SIDE -1` in `subsystem3/subsystem3.h`)
- [x] ✅ Line following: about 2 m straight without losing the line. A
  little side to side, but it corrects on the spot and creeps forward when
  off the line
- [x] ✅ Barcode noise filter: ignores sensor flicker and single grout lines
- [x] ✅ Rescan: on bar-space-bar the robot backs up 60 mm and reads forward
  slowly (seen working on the robot)

### Not done yet
- [ ] 🟡 **S3-L01**: at least 3 m with no line lost. Runs stopped at
  1.2-2.1 m only because the track was too short. **Needs a 3 m line.**
- [ ] 🟡 **S3-L02** junction count = real count. Side-to-side movement can
  give false junctions; count the real ones and compare.
- [ ] 🟡 **S3-B01..B03** barcodes: best run read 7 of the 29 bars of `*F*`
  correctly ("3.3 3.2 10.0 3.5 9.9 3.8 2.6" mm), then steering moved the
  sensor off the bars. Latest build drives straight during the slow read:
  **not tested on the robot yet.** Pass = the codes on the track decoded in
  order, both directions, including "A small".
- [ ] ⬜ **S5-B01 / S5-B02** (2C): right move at each junction, U-turn, stop on
  an unmapped code

### Unknowns to measure
- Tape width, and the gap between C and the side sensor
- Where the barcode sensor sits relative to C (in front, behind, beside),
  and whether it passes over the bars while C follows the line
- Real number of junctions on the test track

## Stage 3: obstacle (`TEST_STAGE=3`)

### What works
- [x] ✅ Servo on **GP14** (servo header 3). It was plugged into GP15
  (header 4) at first and did not move.
- [x] ✅ Servo direction: centre, LEFT, RIGHT correct (`S5_SERVO_LEFT_HIGH 1`)
- [x] ✅ Servo turns slowly now (2 deg per 20 ms): full speed broke the horn
- [x] ✅ Sonar live view: steady readings (about +/- 10 mm), follows a hand

### Not done yet
- [ ] 🟡 **S5-O01** width, now 6 tries (any 3 flat boxes, 2 each, real width
  by ruler, box about 250 mm ahead, nothing else within 50 cm). With the
  scan at +/- 25 deg the sonar never saw past a box's edge (beam is about
  30 deg wide), so no width came out. Scan widened to **+/- 40 deg: not
  tested on the robot yet.** Route the cables at the back first so they
  do not pull at 40 deg.
- [ ] ⬜ **S5-O02** bypass: box on the line, robot stops, scans, drives round
  the clear side, back on the line. Pass = no contact, within 300 mm of the
  line, back on it within 500 mm. Needs S5-O01 working (with no clear side
  found it stops with "no clear side").
- [ ] ⬜ Sonar fault: cover or unplug ECHO during S5-O02; the robot must stop
  with "ultrasonic not answering"

## Stage 4: ramp (`TEST_STAGE=4`)

The practice "ramp" is a **hump**: a block 10 mm high, about 5 cm long in
the driving direction (20 cm across), 0.5 cm flat on top. Much shorter
than the robot, so the robot tips over it instead of driving up a slope.

### What works
- [x] ✅ S4-I01 IMU found (no read dropouts in the 2 Nov runs after reseating)
- [x] ✅ Tilt view: lifting the front gives sin(pitch) up to about 500, back
  to 0 when level; sign correct
- [x] ✅ Test stops the robot itself once a ramp is reported, and prints
  distance / tilt / height every 0.5 s plus the highest climb and
  L x peak tilt for every run

### Not done yet
- [ ] 🟡 **S4-R01** height within ±5 mm. 3 runs over the 10 mm hump read 18.3,
  66.0 and no report (L x peak tilt: 15.6, 52.0, 18.6 mm). Cause: hitting the
  hump jolts the robot and the accelerometer (no gyro) reads the jolt as
  tilt (up to 40 deg; the hump can only tilt it about 7 deg).
  **Fix built, not tested on the robot:** readings where |a| is more than
  5% off the calibrated 1 g are skipped as jolts (`JOLT_FRAC`,
  `task_imu.c`); each run prints how many were skipped. If tilt then
  sticks near 0 with hundreds skipped, loosen `JOLT_FRAC`.
- [ ] ⬜ **Measure L** (`S4_L_MM`, now a guess of 80): wheel axle centre to where
  the front support touches the floor. For a short hump the height is
  L x peak sin(pitch); if that is steady near 10 mm it should become the
  main report.
- [ ] ⬜ Tape the hump down for the runs (a loose block can be pushed along)
- Note for B4: a hard STOP can give a false ramp report (braking tilts
  the robot); the jolt filter should help with this too

## Stage 5: telemetry + MQTT (demo image)

- [ ] ⬜ S1-T01 1 Hz telemetry on `is34/telemetry`, events on `is34/event`
- [ ] ⬜ Mapping accepted / bad mapping rejected
- [ ] ⬜ S1-T02 remote START/STOP, 5 of each

## Stage 6: whole course

- [ ] ⬜ SYS-01: 3 full runs

## Rules that stay in force

- Never power from laptop USB and the power bank at the same time.
- `config/wifi_secrets.h` is git-ignored: never commit it.
- Don't build with `CRASH_REPORT=1` while the IR sensors are on GP0/GP1.
- The `.uf2` files in `images/` are from 1 Nov. For Stage 2, build from the
  latest source (`make TEST=1 TEST_STAGE=2`).
