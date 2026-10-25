# Requirement coverage of the bring-up tests (against the submitted Week 6 design)

Coverage levels:
- Bench: the test measures the requirement's own quantity on the real part, but standalone (no RTOS, no other tasks running). It can show a problem, or support the number; it is not acceptance.
- Prerequisite: the test measures something the requirement depends on (calibration, wiring, sensor limits), not the requirement itself.
- None: needs integrated firmware, the RTOS or the course. No bring-up test can show it.

## Functional requirements

| ID | Short form | Coverage | From which test | Still needed |
|---|---|---|---|---|
| FR1 | Follow the line with IR | Prerequisite | ir_test (polarity, separation, chatter) | Line controller on the course |
| FR2 | Decode Code 39, A-D mapping | Prerequisite | ir_test (bar widths, 1.8x ratio), encoder_test (mm/count) | Decoder, 20 passes per code |
| FR2a | New mapping over MQTT at runtime | Prerequisite | wifi_mqtt_test (command path, parser, verdicts) | robot/cmd/map handler and apply-without-restart |
| FR3 | Execute barcode commands with encoders | Prerequisite | encoder_test (counts/rev, direction), parts_check (mapping) | PID turns and junction logic |
| FR4 | Detect humps, peak height, report | Prerequisite | imu_test (static tilt h_est, pitch noise) | Moving hump trials, telemetry field |
| FR5 | Detect and profile obstacles | Prerequisite | ultrasonic_test (accuracy, outcomes, scans) | Profiling logic |
| FR6 | Navigate around obstacles | None | - | Bypass trials (A9) |
| FR7 | Telemetry at a regular interval + snapshot | Bench (rate only) | wifi_mqtt_test (1 Hz publish, payload capacity) | Real fields, snapshot with ages (NFR5a) |
| FR8 | Reacquire line after manoeuvres | None | - | Course trials |
| FR9 | Stop and FAULT on listed causes | Prerequisite | encoder_test (stall rule timing), imu_test (shock rule), parts_check (GP28) | Safety task, latching, reset rule |
| FR10 | Heartbeat while connected | Bench | wifi_mqtt_test + mqtt_watch.py | Repeat inside the RTOS |
| FR11 | Junctions, apply latest command | None | - | Line + Mission integration |
| FR12 | START/STOP by button or MQTT, mission done | Prerequisite | parts_check (buttons), wifi_mqtt_test (command path) | Mission controller |
| FR13 | IMU motion classes, collision, turn rate | Prerequisite | imu_test (shock rule inputs, heading noise) | Classifier |
| FR14 | Safety events pending until acknowledged | None | - | A17/A18 fault injection |
| FR15 | Mission event order, epoch/ID rejection | None | - | A18 race tests |
| FR16 | Blocked bypass stops; retrace rule | None | - | A9/A19 |

## Non-functional requirements

| ID | Target | Coverage | From which test | Still needed |
|---|---|---|---|---|
| NFR1 | Line loop period <= 20 ms | None | - | Integrated task timing |
| NFR1a | No line loss > 300 ms, 10-25 cm/s | None | - | Course runs |
| NFR2 | IMU sampled >= 100 Hz | Bench | imu_test (fresh samples/s, overruns, loop work) | Same under RTOS load |
| NFR3 | Coarse scan <= 1.2 s, fine <= 2.0 s | Bench | ultrasonic_test long press (timed, 2 pings/ray, through recentre) | Settle-time validation; repeat in the Obstacle task |
| NFR4 | Last bar to nav_cmd_q <= 200 ms at 150 mm/s | None | - | Decoder timing at controlled speed |
| NFR5 | >= 1 Hz; reconnect <= 20 s in 20 worst-phase trials; retry gap <= 2 s | Bench | wifi_mqtt_test (recovery, backoff), mqtt_watch.py | 20 trials under max load, worst phase, independent availability marker |
| NFR5a | Snapshot <= 500 ms with per-field age | None | - | Snapshot handler with real producers |
| NFR5b | Heartbeat 1 s +/- 100 ms; laptop offline after 3 missed | Bench | wifi_mqtt_test (send jitter), mqtt_watch.py (receive interval, OFFLINE) | Repeat with all tasks running |
| NFR6 | Stop with >= 150 mm clearance at <= 250 mm/s | Prerequisite | ultrasonic_test (accuracy, latency), encoder_test (speed per duty) | Moving stop trials |
| NFR6a | Bypass within 300 mm, rejoin within 500 mm | None | - | Bypass trials |
| NFR7 | C, Pico SDK, micro T-Kernel, Barr C, repo structure | Prerequisite | build (C, Pico SDK) | RTOS port, Barr C review |
| NFR8 | Turn, distance, barcode, hump accuracy | Prerequisite | encoder_test (scale), ir_test, imu_test | Closed-loop and course trials |
| NFR9 | Static RAM <= 200 KB; stacks <= 75 %; CPU <= 60 %; payload <= 512 B | Bench (RAM per image, payload capacity) | build report, wifi_mqtt_test probe | Integrated image RAM, stack high-water marks, CPU over 10 s, 512 B enforcement |
| NFR10 | Safe output <= 20 ms with Motion hung | None | - | motor_safe_off + fault injection (A20) |
| NFR11 | Overflow/concurrent faults inhibit, no stale advance | None | wifi command queue shows counted drops only | A17/A18 |
| NFR12 | No turn into unknown space; decision <= 120 ms | Prerequisite | ultrasonic_test (unconfirmed rays count as unknown) | Clearance logic, timestamps |

## Assumptions (A1-A20 from Week 6, plus A21 from the v3 proposals)

| ID | Short form | Coverage | From which test |
|---|---|---|---|
| A1 | WiFi/MQTT stable, reconnect <= 20 s | Bench | wifi_mqtt_test, mqtt_watch.py |
| A2 | Encoders accurate, little slip | Prerequisite | encoder_test (counts/rev, mismatch); 1 m and turn runs need PID |
| A3 | IR separates line, junctions, curves | Bench (static) | ir_test; moving logs at 10/15/25 cm/s still needed |
| A4 | IMU hump height within +/- 5 mm | Bench (static tilt) | imu_test; moving humps still needed |
| A5 | HC-SR04 works at 3.3 V, beam fits margin | Bench | ultrasonic_test |
| A6 | Track is clean black on light | None | course |
| A7 | Barcode format and position | Prerequisite | ir_test (bar widths) |
| A8 | Humps 10-30 mm, no grounding | None | course |
| A9 | Obstacle depth and corridors | None | bypass trials |
| A10 | WiFi coverage over the course | Bench | wifi_mqtt_test RSSI + mqtt_watch.py --csv walk |
| A11 | Element-width resolution <= 1 mm | Prerequisite | encoder_test (mm/count) + ir_test (edge timing) |
| A12 | Approved power, GP28 scaling | Bench | parts_check (GP28 idle and sag; divider ratio by meter) |
| A13 | Sensor mounting | Prerequisite | ir_test, ultrasonic_test at mounted height |
| A14 | Stall <= 200 ms; 2 g separates collision from hump | Bench | encoder_test (stall), imu_test (shock / false shock) |
| A15 | LED/buzzer add no jitter to 10 ms loop | Bench | notify_test (timer-IRQ model, not the RTOS) |
| A16 | Every state completes or times out | None | integration |
| A17 | Static capacities fit bursts | None | integration (the WiFi queue size is one small input) |
| A18 | Race-free pending/epoch records | None | integration |
| A19 | Swept corridor proof | None | integration |
| A20 | Independent safe-off and watchdog | None | integration |
| A21 (v3 proposal, not in Week 6) | Per-task time allocations and encoder edge rate at top speed | Bench (per part) | encoder_test (peak edges/s, callback load), wifi_mqtt_test (poll and activation time), imu_test (loop work) |

## Known limits of the bench numbers

- No RTOS: task priorities, dispatch and kernel critical sections are absent, so CPU, jitter and poll numbers are best case.
- One part at a time: the shared GPIO interrupt load (encoders + barcode + echo together) is only exercised for liveness in parts_check, not timed.
- Reconnect timing: the 18 Oct trials used button marks, so they include reaction time and trial 1's recovery was not measured. From 25 Oct use tools/outage_test.py (laptop-clock timing in broker mode; marks pressed before each switch in hotspot mode, giving upper bounds).
- Servo angles are nominal pulses (about 11.1 us per degree), not calibrated angles; scan timing depends on the assumed settle time.
- Bench reconnect trials are preliminary; NFR5 needs 20 worst-phase trials under integrated load, and the 18 Oct trials do not count towards them.
