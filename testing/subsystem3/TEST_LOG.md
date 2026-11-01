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
