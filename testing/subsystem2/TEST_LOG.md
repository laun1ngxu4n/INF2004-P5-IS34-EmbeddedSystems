# Subsystem 2 — test log

**Subsystem:** Motion Control (B2)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | motion_task wakes every 10 ms (Stage 0, `make TEST=1`) | NFR1/NFR7 timing hooks | auto | `subsystem2_test.c`, 2 s window | wake count within 10 % of 200; `[TEST] S2-U01 PASS` |
| S2-U02 | motion_task: no overrun, longest interval <= 25 ms | build plan timing hooks | auto | same | `[TEST] S2-U02 PASS` |
| S2-M01 | *planned (Stage 1)* Drive straight 1 m | NFR8 | manual | 3 runs at line speed, tape measure | each within 1 m +/- 5 %, ends within the line width |
| S2-M02 | *planned (Stage 1)* Turn 90 deg left, 90 deg right, 180 deg | FR3, NFR8 | manual | 3 of each, mark the floor | 90 deg +/- 10 deg; 180 deg +/- 15 deg |
| S2-M03 | *planned (Stage 1)* Hold the slow barcode speed | TEAM.md 7.1 B4 | auto | drive 1 m at the barcode speed, encoder speed printed | speed steady over the 1 m; record the speed (the value S3-B03 starts from) |
| S2-M04 | *planned (Stage 1)* STOP button stops the motors | FR12 | manual | press GP21 while driving, 5 times | wheels stop every time |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-26 | S2-U01 | `836d3fe` | B1 Ikhwan | PASS | `../integration/logs/2026-10-26_stage0_test.txt` | test image `stage0_test_v4.uf2`, laptop USB, 3 s start-up wait |
| 2026-10-26 | S2-U02 | `836d3fe` | B1 Ikhwan | PASS | `../integration/logs/2026-10-26_stage0_test.txt` | test image `stage0_test_v4.uf2`, laptop USB, 3 s start-up wait |
