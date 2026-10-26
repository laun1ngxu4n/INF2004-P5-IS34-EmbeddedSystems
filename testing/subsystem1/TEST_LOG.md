# Subsystem 1 — test log

**Subsystem:** Telemetry / WiFi + Notification (B1)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | telemetry_task wakes every 100 ms (Stage 0, `make TEST=1`) | NFR1/NFR7 timing hooks | auto | `subsystem1_test.c`, 2 s window | wake count within 10 % of 20; `[TEST] S1-U01 PASS` |
| S1-U02 | telemetry_task: no overrun, longest interval <= 115 ms | build plan timing hooks | auto | same | `[TEST] S1-U02 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-26 | S1-U01 | `836d3fe` | B1 Ikhwan | PASS | `../integration/logs/2026-10-26_stage0_test.txt` | test image `stage0_test_v4.uf2`, laptop USB, 3 s start-up wait |
| 2026-10-26 | S1-U02 | `836d3fe` | B1 Ikhwan | PASS | `../integration/logs/2026-10-26_stage0_test.txt` | test image `stage0_test_v4.uf2`, laptop USB, 3 s start-up wait |
