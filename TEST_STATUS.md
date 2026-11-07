# P5-IS34 test status checklist

Last updated: 6 Nov 2026. How to run each test:
[TESTING_GUIDE.md](TESTING_GUIDE.md). Formal results:
[testing/](testing/) `TEST_LOG.md` files. What the robot does, its limits
and everything still open: [ROBOT_BEHAVIOUR.md](ROBOT_BEHAVIOUR.md).

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
| 4 | IMU + ramp height | B4 | 🟡 2 of 7 runs within ±5 mm on the 10 mm hump; hump not held down |
| 5 | Telemetry + MQTT | B1 | ✅ passed 2 Nov (START/STOP 3 of 5 rounds) |
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
- [ ] ⬜ **S3-L03..L09** (new, 6-7 Nov logic): left branch, right branch, T end,
  right curve, left curve, gap (track 2), stuck S (black card under S);
  one place at a time, each prints PASS/FAIL. The track photos are in
  `docs/track/`: track 3 (foam tiles) has tight curves (about 15 cm)
  and an X crossing, but no single branch or T end unless tiles are
  turned to make one.
- [ ] 🟡 **S3-B01..B03** barcodes: best run read 7 of the 29 bars of `*F*`
  correctly ("3.3 3.2 10.0 3.5 9.9 3.8 2.6" mm), then steering moved the
  sensor off the bars. Latest build drives straight during the slow read:
  **not tested on the robot yet.** Pass = the codes on the track decoded in
  order, both directions, including "A small".
- [ ] ⬜ **S5-B01 / S5-B02** (2C): right move at each junction, U-turn, stop on
  an unmapped code

- [ ] ⬜ Junction turns with the **measured** sensor-to-axle distance of 115 mm
  (the code had a guess of 60): it spins when the wheels are 115 mm past
  where C crossed the junction centre. Re-check on the robot (paused).

### Changed 6 Nov, not tried on the robot yet
- Junction detection uses the filtered sensor readings (one flicker split
  a crossing in two before).
- **Left junctions** from the barcode sensor B (40 mm behind C, 70 mm
  left): a patch 12-28 mm long (tape 20 mm), with C on the line at that
  place before and after. The Stage 2 run prints `B patch 19.8 mm (...)`
  for every first bar B sees (also grout and barcode start bars), and
  `junction N at X mm, branch left / right / both sides`.
- S junctions: steering held straight for up to 28 mm when S goes black
  from normal following; a branch = S white again by itself within
  12-28 mm (was 8-45 with no hold). Drift and right curves keep S black
  longer, so they no longer count.
- Crossroads: S and B within 40 mm = one junction. T end (line ends at a
  cross bar): S sees the right arm, the robot drives straight on up to
  80 mm until B has seen the left arm = a junction; one arm = a corner.
- **Before the run: move S to 15 mm from C** (centre to centre); the photo
  shows about 30 mm.
- To check on curves: no `junction` lines on curves without branches;
  following on the sharpest right curve not worse than before.
- After a barcode meaning L/R/U: 100 mm/s until the junction.
- Barcode sensor on analogue (optional, `S3_BARCODE_ANALOG 1`): power it
  from **3V3**, AO to GP28, flash `..._test_s2_analog.uf2`, send the
  `B analogue` lines over white, tape and grout.
- To check: does the barcode sensor see the left branches, and is there
  at least 40 mm of white between a barcode and the left branch after it?

### Unknowns to measure
- Measured 6 Nov: line sensors 115 mm in front of the axle, body 130 x
  150 mm, barcode arm sticks out 17 mm (see ROBOT_BEHAVIOUR.md)
- Still to measure: how far the IR/barcode bar sticks out in front
- Tape width: 20 mm. Barcode sensor B: 40 mm **behind** C, 70 mm left
  (measured 6 Nov)
- C to S gap: about 30 mm (photo), to be set to 15 mm
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
- [ ] 🟡 **S4-R01** height within ±5 mm, now **5 runs** (was 10).

  **Runs 2 Nov** (old code): 18.3, 66.0, none. Jolts were read as tilt.

  **Runs 6 Nov, batch 1** (jolt filter 5 %): 49.6, none, none. Problems
  found in the log and fixed:
  - the IMU's anti-alias filter was 773 Hz, so motor vibration showed as
    tilt (-2..-4 deg on the flat floor) -> now 50 Hz (`CTRL2 0xC8`)
  - the 5 % jolt filter skipped 30-48 % of all readings, and the height kept
    adding up with the frozen tilt -> now 10 %
  - driving tilts the chassis a little, so the height drifted -10 mm before
    the hump -> tilt is now measured from a slowly following "driving level"

  **Runs 6 Nov, batch 2** (fixes above): 27.4, none, **9.0 ✅**
  (L x tilt 10.6). Jolts skipped dropped to 14-215 per run.

  **Runs 6 Nov, batch 3**: 56.2 (a -56 mm **dip** reported as a climb), none,
  **8.1 ✅** (L x tilt 12.1), none; run 5 not done. **Fixed, not run yet:**
  only climbs are reported now (nose-down dips ignored), and L x tilt uses
  the nose-up peak only.

  Still wrong on the robot side (not code):
  - **The hump is not held down.** A 10 mm hump with L = 80 mm can tilt the
    robot at most sin 0.125 (7 deg); failed runs peaked at 0.23-0.48, so the
    block slid, flipped or got under the robot. Hold it (fingertips on both
    ends, Blu-tack or a rubber mat under it).
  - **"IMU read failed" twice** in batch 3, once mid-run: reseat the Grove
    cable at both ends.
  - Check the IMU board is fixed firmly to the chassis (a loose board
    wobbles on every bump).
  - The hump showed up at 450-1340 mm into the runs, not about 300 mm:
    check the start distance and that the robot drives straight at it.
  - Ask: is this block the lab's lower practice ramp? An accelerometer
    cannot measure a hump much shorter than the robot reliably; a real
    slope is much easier.
- [x] ✅ **L measured: 80 mm** (`S4_L_MM`), same as the guess
- Note for B4: a hard STOP can give a false ramp report (braking tilt)

## Stage 5: telemetry + MQTT (demo image)

Run 2 Nov on a phone hotspot (2.4 GHz), robot on the power bank for
S1-T02. Robot got IP 10.14.83.102; broker on the laptop.

- [x] ✅ WiFi join + MQTT connect (`mqtt: connected`), heartbeat both ways
- [x] ✅ S1-T01 status line on `is34/telemetry` every 1 s (state, speed,
  distance, line, junctions, barcode, mapping, ramp peak, stop reason);
  events on `is34/event`
- [x] ✅ Mapping `K=L,Q=R,X=F,M=U` gives `is34/ack/map OK ...` and shows in
  the status line; bad `K=L` gives `REJECTED 1 pairs, need exactly 4 (old
  mapping kept)`
- [x] ✅ S1-T02 remote START / STOP: **3 of 5 rounds done, 3/3 passed**. Each
  START gave CAL then RUN at 150-170 mm/s; each STOP gave the `STOP` event
  in the same second and IDLE, speed 0, `stop=2` by the next status line.
  Two more rounds would make it the full 5.

Setup notes:
- The laptop's Mosquitto Windows service only listens on localhost:1883,
  so a second broker was run on **port 1884** (`listener 1884 0.0.0.0`,
  `allow_anonymous true`) and `MQTT_BROKER_PORT` set to 1884 in
  `config/wifi_secrets.h` (git-ignored).
- The broker IP is built into the image: if the laptop's IP on the
  hotspot changes, edit `MQTT_BROKER_IP` and rebuild
  (`make NET=lwip MQTT=1`).

## Stage 6: whole course

- [ ] ⬜ SYS-01: 3 full runs

## Rules that stay in force

- Never power from laptop USB and the power bank at the same time.
- `config/wifi_secrets.h` is git-ignored: never commit it.
- Don't build with `CRASH_REPORT=1` while the IR sensors are on GP0/GP1.
- The `.uf2` files in `images/` are from 1 Nov. For Stage 2, build from the
  latest source (`make TEST=1 TEST_STAGE=2`).
