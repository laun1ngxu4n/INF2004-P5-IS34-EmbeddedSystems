# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | Telemetry's 1 Hz ping wakes Mission through `flg_mission` (2-4 pings in 3 s) | IF-01 | auto | `subsystem5_test.c` | `[TEST] INT-01 PASS` |
| INT-02 | all 8 Stage 0 tasks run together for 60 s with 0 overruns and no task missing from the timing table | build plan step 1, NFR9 prerequisite | manual | normal image, read the `[tmg]` table after 60 s | every task present, overruns column 0, intervals near the period |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
