# P5-IS34 test status checklist

Last updated: 2 Nov 2026 (commit `bcbcce9`). How to run each test:
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
| 3 | Obstacle: sonar, servo, width, bypass | B5 | ⬜ |
| 4 | IMU + ramp height | B4 | ⬜ |
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

- [ ] ⬜ Servo direction (LEFT, then RIGHT)
- [ ] ⬜ Sonar live view; covered/unplugged ECHO must stop the run
- [ ] ⬜ S5-O01 width: 15 tries, 100/150/200 mm obstacles
- [ ] ⬜ S5-O02 bypass: no contact, back on the line within 500 mm

## Stage 4: ramp (`TEST_STAGE=4`)

- [ ] ⬜ S4-I01 IMU found. IMU I2C "read failed" dropouts seen in Stage 2
  logs: **reseat the Grove cable (port 6) first**
- [ ] ⬜ S4-R01 ramp height, 10 runs within ±5 mm
- Note for B4: a hard STOP can give a false ramp report (braking tilts
  the robot)

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
