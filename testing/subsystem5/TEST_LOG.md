# Subsystem 5 — test log

**Subsystem:** Mission Controller + Obstacle Scanner (B5)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | mission_task keeps waking on its 100 ms timeout (Stage 0, `make TEST=1`) | build plan timing hooks | auto | `subsystem5_test.c`, 2 s window | at least 18 wakes; `[TEST] S5-U01 PASS` |
| S5-U02 | mission_task: longest interval <= 125 ms | | auto | same | `[TEST] S5-U02 PASS` |
| S5-U03 | obstacle_task wakes every 60 ms (Stage 0, `make TEST=1`) | NFR1/NFR7 timing hooks | auto | `subsystem5_test.c`, 2 s window | wake count within 10 % of 33; `[TEST] S5-U03 PASS` |
| S5-U04 | obstacle_task: no overrun, longest interval <= 75 ms | build plan timing hooks | auto | same | `[TEST] S5-U04 PASS` |
| S5-B01 | *planned (Stage 2)* Move at the cross-junction, U-turn included | TEAM.md 7.1 B5, B6 | manual | one barcode per mapped move, then the junction | each move happens at the junction, not at the barcode |
| S5-B02 | *planned* No mapping received | B7 | manual | decode a barcode before any map is sent | robot reports the letter "NO MAPPING, ignored" and carries on straight (since 20 Nov; was a stop) |
| S5-O01 | *planned (Stage 3)* Width estimate | TEAM.md 7.2 O1-O6 | manual | 3 tries (Stage 3 image): 3 flat boxes 10-20 cm wide, one each, real width by ruler, box about 250 mm ahead (350 mm 7-8 Nov), nothing else within 60 cm; record the scan time in the notes | reported width +/- uncertainty contains the true width; record the error |
| S5-O02 | *planned* Bypass and line reacquire | O7, O8 | manual | same obstacles | no contact, within 300 mm of the line, rejoin within 500 mm |
| S5-O03 | **dropped (1 Nov, essentials only)**: scan time is recorded in S5-O01's notes instead | O9 | - | - | - |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-26 | S5-U01 | `836d3fe` | B1 Ikhwan | PASS | `../integration/logs/2026-10-26_stage0_test.txt` | test image `stage0_test_v4.uf2`, laptop USB, 3 s start-up wait |
| 2026-10-26 | S5-U02 | `836d3fe` | B1 Ikhwan | PASS | `../integration/logs/2026-10-26_stage0_test.txt` | test image `stage0_test_v4.uf2`, laptop USB, 3 s start-up wait |
| 2026-10-26 | S5-U03 | `836d3fe` | B1 Ikhwan | PASS | `../integration/logs/2026-10-26_stage0_test.txt` | test image `stage0_test_v4.uf2`, laptop USB, 3 s start-up wait |
| 2026-10-26 | S5-U04 | `836d3fe` | B1 Ikhwan | PASS | `../integration/logs/2026-10-26_stage0_test.txt` | test image `stage0_test_v4.uf2`, laptop USB, 3 s start-up wait |
| 2026-11-09 | S5-U01..U04 | `cc87b08` | B1 Ikhwan | PASS | `logs/2026-11-09_stage3_obstacle_robot.txt` | Stage 3 WiFi test image, CR1 packed build |
| 2026-11-09 | S5-O01 | `cc87b08` | B1 Ikhwan | NOT JUDGED | `logs/2026-11-09_stage3_obstacle_robot.txt` | widths 175 +/- 30, 136 +/- 33, 210 +/- 36 mm, box fronts 271 / 304 / 304 mm, both edges seen every try; real widths not recorded |
| 2026-11-09 | S5-O02 | `cc87b08` | B1 Ikhwan | PASS | `logs/2026-11-09_stage3_obstacle_robot.txt` | stopped at 241 mm, width 114, bypass LEFT out 254 past 785, line met behind the box 703 mm along, bypass done; no contact reported. Team: the 90 deg spins turn about 5 deg too much (fixed after: S2_TURN_TRIM_DEG) |
