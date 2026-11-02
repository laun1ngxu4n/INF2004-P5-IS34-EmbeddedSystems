# Test plan: essentials only (team decision, 1 Nov)

We test only what the demo needs. Measurements that were "nice to have"
are dropped, and the graded ones (barcode, obstacle width, ramp height) are
folded into the stage tests below instead of being run on their own.

Every test ID below has its row in the owner's `TEST_LOG.md`. How to run
each one: `TESTING_GUIDE.md` (image `make TEST=1 TEST_STAGE=n`).

| Stage | What gets built | Tests (owner folder) | Graded item folded in |
|---|---|---|---|
| 0 | RTOS frame, 8 tasks | S1-S5 U01-U04, INT-01, INT-02 (**done**, 26 Oct) | - |
| 1 | Motion + STOP | S2-M02 turns, S2-M03 slow speed, S2-M04 STOP (S2-M01 straight 1 m dropped) | S2-M03 sets the barcode speed |
| 2 | Line + barcode + junction moves | S3-L01 track, S3-L02 junction, S3-B01..B04 barcode, S5-B01..B02 junction moves | new IR sensor checked at speed (S3-B03); A-Z both ways; MQTT map |
| 3 | Obstacle | S5-O01 width, S5-O02 bypass | width +/- uncertainty; sonar accuracy is checked through the known obstacle widths; scan time noted in S5-O01 |
| 4 | IMU ramp height | S4-R01 | ramp and L measured at the start of S4-R01 |
| 5 | Telemetry + MQTT | S1-T01 results on the PC, S1-T02 START/STOP | - |
| 6 | Mission, whole run | SYS-01 demo rehearsal | all of the above together |

## Dropped (not tested)

Recorded so nobody wonders later. If one of these causes a problem, test it then.

- Straight 1 m distance accuracy (S2-M01).
- IR sensor mounted height and pot margin.
- Ultrasonic stand-alone accuracy batches, angled target, separate scan-time test (S5-O03).
- Stall detection timing, heading drift with motors running, lowest working duty.
- IMU hand-bump / false-shock test.
- GP28 battery sense (removed from scope, TEAM.md section 7).
- WiFi reconnect trials (NFR5 20 worst-phase trials), snapshot timing (NFR5a).

These are requirements in the Week 6 design, so the final report should list
them as "not tested" rather than claim them.
