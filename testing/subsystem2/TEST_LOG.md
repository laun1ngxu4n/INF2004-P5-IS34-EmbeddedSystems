# Subsystem 2 — test log

**Subsystem:** Motion Control (B2)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | motion_task wakes every 10 ms (Stage 0, `make TEST=1`) | NFR1/NFR7 timing hooks | auto | `subsystem2_test.c`, 2 s window | wake count within 10 % of 200; `[TEST] S2-U01 PASS` |
| S2-U02 | motion_task: no overrun, longest interval <= 25 ms | build plan timing hooks | auto | same | `[TEST] S2-U02 PASS` |
| S2-M01 | **dropped (1 Nov)**: no room for a 1 m straight run and line following steers anyway | NFR8 | - | - | - |
| S2-M05 | *planned (Stage 1)* Directions and encoder signs, wheels lifted (review 1 Nov) | FR3 | auto + watch | forward 300 mm then back 300 mm | both encoders count the commanded way, within 25 % of each other, and you see both wheels turn forward first |
| S2-M06 | *planned (Stage 1)* STOP with the wheels lifted, before any floor test | FR12 | auto | press STOP after a second | wheels stop within 500 ms |
| S2-M02 | *planned (Stage 1)* Turn 90 deg left, 90 deg right, 180 deg | FR3, NFR8 | manual + auto | `make TEST=1` drive tests (RTOS_STAGE1.md): 3 rounds of left 90, right 90, 180 from a tape mark; console prints the encoder angle, you write down the real one | 90 deg +/- 10 deg; 180 deg +/- 15 deg |
| S2-M03 | *planned (Stage 1)* Hold the slow barcode speed | TEAM.md 7.1 B4 | auto | drive 0.5 m at the barcode speed (150 mm/s); speed measured over the middle 300 mm | mean within 10 % of 150 mm/s AND the measured real distance 475-525 mm (checks the mm/count scale) |
| S2-M04 | *planned (Stage 1)* STOP button stops the motors | FR12 | auto | press GP21 while it drives at 200 mm/s, 5 tries; console prints press-to-stopped time and roll-on distance | 5/5 stopped within 500 ms |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-26 | S2-U01 | `836d3fe` | B1 Ikhwan | PASS | `../integration/logs/2026-10-26_stage0_test.txt` | test image `stage0_test_v4.uf2`, laptop USB, 3 s start-up wait |
| 2026-10-26 | S2-U02 | `836d3fe` | B1 Ikhwan | PASS | `../integration/logs/2026-10-26_stage0_test.txt` | test image `stage0_test_v4.uf2`, laptop USB, 3 s start-up wait |
| 2026-11-09 | S2-U01 | `f5ccc28` | B1 Ikhwan | PASS | `logs/2026-11-09_stage1_part1_robot.txt` | Stage 0 re-run on the Barr C + Change Requirement 1 code: 201 wakes in 2000 ms, power bank, WiFi image |
| 2026-11-09 | S2-U02 | `f5ccc28` | B1 Ikhwan | PASS | `logs/2026-11-09_stage1_part1_robot.txt` | longest interval 10.8 ms, no overrun |
| 2026-11-09 | S2-M05 | `f5ccc28` | B1 Ikhwan | PASS | `logs/2026-11-09_stage1_part1_robot.txt` | lifted: forward L 3762 / R 3761 counts, back -3763 / -3762 |
| 2026-11-09 | S2-M06 | `f5ccc28` | B1 Ikhwan | PASS | `logs/2026-11-09_stage1_part1_robot.txt` | lifted: stopped 109 ms after the press, rolled 7 mm |
| 2026-11-09 | S2-M03 | `f5ccc28` | B1 Ikhwan | PASS (encoders); real distance not measured | `logs/2026-11-09_stage1_part1_robot.txt` | mean 120 mm/s (116-127), encoders say 500 mm in 4320 ms, left - right 0 mm. Measure the real distance next time |
| 2026-11-09 | S2-M02 | `f5ccc28` | B1 Ikhwan | INCOMPLETE | `logs/2026-11-09_stage1_part1_robot.txt, photos logs/2026-11-09_S2-M02_180_before.jpg / _after.jpg` | round 1 only: encoder angles left 90.6, right -90.2, 180 -> 180.6; the real 180 looked about 5 deg over (from the photos, about +-5 deg). Rounds 2-3 not run: STOP was pressed at the next-round prompt. Follow-up: S2_TURN_TRACK_MM = 120 for spins (about 4 deg less on a 180); re-run all 3 rounds and write down the real angles |
| 2026-11-09 | S2-M04 | `f5ccc28` | B1 Ikhwan | 2 of 5 run | `logs/2026-11-09_stage1_part2_held_wheel_robot.txt` | try 1: STOP (panel) stopped the wheels 149 ms after the press, rolled 6 mm; try 2: the wheel was held on purpose, Safety stopped the motors (see CR1 held wheel); tries 3-5 not run |
| 2026-11-09 | CR1 held wheel | `f5ccc28` | B1 Ikhwan | PASS | `logs/2026-11-09_stage1_part2_held_wheel_robot.txt` | Change Requirement 1 / Safety: "[EVT] Safety: a wheel driven hard is not turning, motors off", motors off. START working again after 2 s: not shown in the log, to confirm |
