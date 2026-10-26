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

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
