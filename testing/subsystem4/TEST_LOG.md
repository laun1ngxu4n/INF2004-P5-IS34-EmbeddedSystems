# Subsystem 4 — test log

**Subsystem:** Safety / Fault Monitor + IMU (B4)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | safety_task wakes every 20 ms (Stage 0, `make TEST=1`) | NFR1/NFR7 timing hooks | auto | `subsystem4_test.c`, 2 s window | wake count within 10 % of 100; `[TEST] S4-U01 PASS` |
| S4-U02 | safety_task: no overrun, longest interval <= 35 ms | build plan timing hooks | auto | same | `[TEST] S4-U02 PASS` |
| S4-U03 | imu_task wakes every 10 ms (Stage 0, `make TEST=1`) | NFR1/NFR7 timing hooks | auto | `subsystem4_test.c`, 2 s window | wake count within 10 % of 200; `[TEST] S4-U03 PASS` |
| S4-U04 | imu_task: no overrun, longest interval <= 25 ms | build plan timing hooks | auto | same | `[TEST] S4-U04 PASS` |
| S4-I01 | *planned (Stage 4)* IMU found and configured under the RTOS | FR4 | auto | `make TEST=1 TEST_STAGE=4` | `[TEST] S4-I01 PASS` |
| S4-R01 | *planned (Stage 4)* Ramp height | FR4, A4, TEAM.md 7 (ramp) | manual | first measure the lower practice ramp (height, length, flat top?) and the robot's wheelbase L; then 10 runs over the ramp | 10/10 detected; every reported height within +/- 5 mm of the measured height |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-26 | S4-U01 | `836d3fe` | B1 Ikhwan | PASS | `../integration/logs/2026-10-26_stage0_test.txt` | test image `stage0_test_v4.uf2`, laptop USB, 3 s start-up wait |
| 2026-10-26 | S4-U02 | `836d3fe` | B1 Ikhwan | PASS | `../integration/logs/2026-10-26_stage0_test.txt` | test image `stage0_test_v4.uf2`, laptop USB, 3 s start-up wait |
| 2026-10-26 | S4-U03 | `836d3fe` | B1 Ikhwan | PASS | `../integration/logs/2026-10-26_stage0_test.txt` | test image `stage0_test_v4.uf2`, laptop USB, 3 s start-up wait |
| 2026-10-26 | S4-U04 | `836d3fe` | B1 Ikhwan | PASS | `../integration/logs/2026-10-26_stage0_test.txt` | test image `stage0_test_v4.uf2`, laptop USB, 3 s start-up wait |
| 2026-11-09 | S4-U01..U04 | `490d07c` | B1 Ikhwan | PASS | `logs/2026-11-09_stage4_imu_robot.txt` | Stage 4 WiFi test image, CR1 packed build |
| 2026-11-09 | S4-I01 | `490d07c` | B1 Ikhwan | PASS | `logs/2026-11-09_stage4_imu_robot.txt` | LSM303D found at 0x1D |
| 2026-11-09 | S4-R01 | `490d07c` | B1 Ikhwan | INCOMPLETE | `logs/2026-11-09_stage4_imu_robot.txt` | run 1 of 5: the test saw the hump, side tilt peak 143/1000 = 17.6 mm (real height not recorded); the IMU task did not report it (level check too strict, fixed after); STOP ended the remaining runs (fixed after: STOP ends one run) |
| 2026-11-09 | S4-R01 | `e6e7fed` | B1 Ikhwan | FAIL (+-5 mm) | `logs/2026-11-09_stage4_imu_robot_2.txt` | 5 runs, humps about 10 / 10 / 25 / 25 / 10 mm (estimated): read 16.6 / 17.2 / 40.8 / 37.0 / 17.6 mm, about 1.6x high; every hump seen; IMU task reported runs 1, 3, 5 only (quiet distance carried over). Fixed after: x 0.63 scale (fitted on these runs), reset at calibration |
