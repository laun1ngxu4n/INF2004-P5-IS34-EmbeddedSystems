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
