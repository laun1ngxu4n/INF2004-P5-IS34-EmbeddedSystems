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
| 2026-10-26 | INT-01 | `836d3fe` | B1 Ikhwan | PASS | `logs/2026-10-26_stage0_test.txt` | 3 pings in 3000 ms |
| 2026-10-26 | INT-02 | `836d3fe` | B1 Ikhwan | PASS | `logs/2026-10-26_stage0_run.txt` (Stage 0 run starts at line 28; lines 1-27 are the end of the previous session) | run image `stage0_run_v4.uf2`, 60 s: all 8 tasks in every table, overruns stayed at the start-up values 11/5/1 (from the ~120 ms WiFi radio start-up pause at 0.6 s), 10 ms tasks 7.4-12.6 ms, 20 ms tasks 19.98-20.01 ms, no lost output. Earlier runs with the table printed every 1 s had a ~12 ms all-task delay with lost output (fixed in `836d3fe`, see TEAM.md section 4) |
