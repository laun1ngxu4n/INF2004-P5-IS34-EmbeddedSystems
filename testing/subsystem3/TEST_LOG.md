# Subsystem 3 — test log

**Subsystem:** Line Following + Barcode Decoder (B3)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | line_task wakes every 20 ms (Stage 0, `make TEST=1`) | NFR1/NFR7 timing hooks | auto | `subsystem3_test.c`, 2 s window | wake count within 10 % of 100; `[TEST] S3-U01 PASS` |
| S3-U02 | line_task: no overrun, longest interval <= 35 ms | build plan timing hooks | auto | same | `[TEST] S3-U02 PASS` |
| S3-U03 | barcode_task keeps waking on its 10 ms timeout (Stage 0, `make TEST=1`) | build plan timing hooks | auto | `subsystem3_test.c`, 2 s window | at least 100 wakes; `[TEST] S3-U03 PASS` |
| S3-U04 | barcode_task: longest interval <= 35 ms | | auto | same | `[TEST] S3-U04 PASS` |
| S3-L01 | *planned (Stage 2)* Follow the track | FR1 | manual | 3 laps of the practice track (test image: up to 60 s) | at least 3 m followed with no line loss, seen on the track |
| S3-L02 | *planned (Stage 2)* Detect the cross-junction | FR11 | manual | 10 passes | 10/10 detected, no false junctions |
| S3-L03 | *planned (Stage 2)* Left-only branch seen by the barcode sensor B | FR11 | auto + manual | Stage 2 image, "one place at a time": robot 20 cm before a left-only branch, drives at most 450 mm | `[TEST] S3-L03 PASS`: exactly 1 junction, branch left |
| S3-L04 | **dropped (7 Nov)**: right-only branch. None on the track, and with the sensors straddling the tape it looks like a correction | FR11 | - | - | - |
| S3-L05 | *planned (Stage 2)* T end (line stops at a cross bar) | FR11 | auto + manual | same, 20 cm before a T end | `[TEST] S3-L05 PASS`: exactly 1 junction, a crossing |
| S3-L06 | *planned (Stage 2)* No false junction on a right curve | FR11, A3 | auto + manual | same, start of the tightest right curve, drives at most 1.2 m | `[TEST] S3-L06 PASS`: no junction, no line lost |
| S3-L07 | *planned (Stage 2)* No false junction on a left curve | FR11, A3 | auto + manual | same, tightest left curve | `[TEST] S3-L07 PASS`: no junction, no line lost |
| S3-L08 | *planned (Stage 2)* Gap in the line | A3 | auto + manual | same, 20 cm before the gap on track 2, drives at most 450 mm | `[TEST] S3-L08 PASS`: across, no line lost, no junction |
| S3-L09 | *planned (Stage 2)* Stuck line sensor stop | safety | auto + manual | same, on a straight; once it moves hold a black card under R | `[TEST] S3-L09 PASS`: stops with LINE SENSOR FAULT (one sensor black) within about 300 mm |
| S3-L10 | *planned (Stage 2)* X crossing (straddle logic, 7 Nov) | FR11 | auto + manual | same, 20 cm before the X crossing on track 3, square to it | `[TEST] S3-L10 PASS`: exactly 1 junction, a crossing |
| S3-B00 | *planned (Stage 2)* Barcode read driving straight, no line following (7 Nov) | B1, B2, B3 | auto + manual | robot with B 5 cm before a code, square to it, drives 300 mm straight: a code, the same from the other end, "A small", one at the follow speed | `[TEST] S3-B00 PASS` on runs 1-3; no `!` in the B profile |
| S3-B01 | *planned (Stage 2)* Decode every A-Z barcode | TEAM.md 7.1 B1 | manual | drive over each test card at barcode speed, 3 passes each | correct character on every pass |
| S3-B02 | *planned* Decode reversed | B2 | manual | same cards, robot driving the other way | same character as forward |
| S3-B03 | *planned* Thin barcode at barcode speed (new sensor) | B3, B4 | manual | "A small" card, 10 passes, starting at the S2-M03 speed; slow down until 10/10 | 10/10 correct; record the speed used |
| S3-B04 | *planned* Mapping over MQTT | B7, B8 | manual | send a valid and an invalid map from the PC | valid: ack shows stored map; invalid: rejected, old map kept |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-26 | S3-U01 | `836d3fe` | B1 Ikhwan | PASS | `../integration/logs/2026-10-26_stage0_test.txt` | test image `stage0_test_v4.uf2`, laptop USB, 3 s start-up wait |
| 2026-10-26 | S3-U02 | `836d3fe` | B1 Ikhwan | PASS | `../integration/logs/2026-10-26_stage0_test.txt` | test image `stage0_test_v4.uf2`, laptop USB, 3 s start-up wait |
| 2026-10-26 | S3-U03 | `836d3fe` | B1 Ikhwan | PASS | `../integration/logs/2026-10-26_stage0_test.txt` | test image `stage0_test_v4.uf2`, laptop USB, 3 s start-up wait |
| 2026-10-26 | S3-U04 | `836d3fe` | B1 Ikhwan | PASS | `../integration/logs/2026-10-26_stage0_test.txt` | test image `stage0_test_v4.uf2`, laptop USB, 3 s start-up wait |
| 2026-11-09 | S3-U01..U04 | `cc87b08` | B1 Ikhwan | PASS | `logs/2026-11-09_stage2_line_robot.txt` | Stage 2 WiFi test image, CR1 packed build |
| 2026-11-09 | S3-L01 | `cc87b08` | B1 Ikhwan | FAIL (length only) | `logs/2026-11-09_stage2_line_robot.txt` | 1505 mm, line lost 0 times, left and right curves; stopped by hand (small track, needs 3 m). 13 corrections/m, 1 sweep. Team: follows well |
| 2026-11-09 | S3-L02 | `cc87b08` | B1 Ikhwan | PASS (seen) | `logs/2026-11-09_stage2_line_robot.txt` | 1 crossing counted at 1194 mm, no false junctions; tester saw the crossing handled (real count not written down) |
