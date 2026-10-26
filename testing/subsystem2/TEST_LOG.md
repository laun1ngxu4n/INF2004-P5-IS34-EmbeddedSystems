# Subsystem 2 — test log

**Subsystem:** Motion Control (B2)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | motion_task wakes every 10 ms (Stage 0, `make TEST=1`) | NFR1/NFR7 timing hooks | auto | `subsystem2_test.c`, 2 s window | wake count within 10 % of 200; `[TEST] S2-U01 PASS` |
| S2-U02 | motion_task: no overrun, longest interval <= 25 ms | build plan timing hooks | auto | same | `[TEST] S2-U02 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S2-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
