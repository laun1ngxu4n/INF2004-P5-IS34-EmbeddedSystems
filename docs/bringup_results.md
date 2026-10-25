# Bring-up results (fill in, commit, link in the Tribunal template)

Date / place / board used: 22 Oct 2026 (IMU + parts_check), 25 Oct 2026 (IR, encoders by hand): Pico W on Cytron Robo Pico; IMU board marked GY-511 (chip is an LSM303D)
Power arrangement used: laptop USB only
Firmware: the 22 Oct IMU and parts_check results above were taken with bring-up v3.1 (images reproduced byte-for-byte, docs/logs/repro_check_2026-10-25.txt). From 25 Oct use bring-up v3.2 (review fixes); write the version next to every new result. 25 Oct IR and encoder results: parts_check, ir_test and encoder_test from the v3.2 image bundle (ir_test byte-identical to the 18 Oct image). Commit link: bring-up v3.2 is commit 20f97c1 on branch bringup-v3.2-ir

For each value, write the measured number, how many trials, and the log
file or video name. "Not measured" is an honest answer; leave the reason.

These are standalone bring-up measurements (plain Pico SDK, no RTOS). They
test hardware limits and assumptions; they are not NFR acceptance tests.
Keep four labels apart: design target, test capability, measured result,
accepted requirement.

## Already recorded (no hardware needed)

| Item | Result | Evidence |
|---|---|---|
| All 7 images build for pico_w, SDK 2.1.1, gcc 13.2.1 | 0 warnings | `docs/logs/build_report_2026-10-18.txt` |
| Static RAM per image (NFR9 <= 200 KB) | WiFi test 59,288 B; others 8.4-10.5 KB | same file (`__bss_end__` - 0x20000000) |
| Command parser, queue and verdicts | all host cases pass | `docs/logs/host_tests_2026-10-18.txt` |
| IR overflow: no width across a dropped edge | pass (v2 failed this case) | same file |

## All buddies - parts_check

| Part | Expected | Result (OK / CHECK / FAIL / what you saw) | Notes |
|---|---|---|---|
| IMU (0x19, 0x1E "H43", ~1 g) | OK | OK (22 Oct): LSM303D at 0x1D, \|a\| 0.882 g | v3 failed (looked only at 0x19/0x1E); v3.1 detects LSM303D. \|a\| is before Z-offset correction (parts_check does not correct) |
| Ultrasonic (5 pings at a wall) | OK | FAIL (22 Oct): not wired yet | - |
| Servo left / right / centre | moves both ways | | |
| NeoPixels red / green / blue | both LEDs, right colours | | |
| Buzzer two beeps | audible | | |
| GP28 pin voltage idle | - | 0.08-0.11 V, no battery connected (22 Oct) | not a battery reading; divider ratio measured: |
| GP28 lowest under one motor (sag) | - | | |
| Left motor fwd(A=PWM): which encoder, sign, wheel direction | own encoder only | | |
| Right motor fwd(A=PWM): which encoder, sign, wheel direction | own encoder only | | |
| Hand turn forward: L and R count direction | - | L counts DOWN, R counts UP for forward (25 Oct, v3.2); each wheel moves only its own count, invalid 0 | confirms Grove 3 (white GP4 = A, yellow GP5 = B) and Grove 4 (white GP16 = A, yellow GP17 = B). parts_check_2026-10-25_encoders.txt |
| IR L / C / B flip on black/white | yes | yes (25 Oct, v3.2): each sensor flips only its own pin, L = GP0, C = GP1, B = GP2 | confirms the Grove rule on Grove 1 (white GP0, yellow GP1) and Grove 2 (white GP2). Output pasted in chat, not saved as a log |
| Buttons GP20 / GP21 read DOWN when pressed | yes | | |

## Buddy 1 - wifi_mqtt_test (A1, A10, NFR5, NFR5b, NFR9)

| Item | Design value | Measured | Trials | Log / video |
|---|---|---|---|---|
| CYW43 init time | - | | | |
| WiFi join + DHCP | <= 5 s | | | |
| MQTT connect | <= 3 s | | | |
| cyw43_arch_poll call time avg / max | v3 allocation <= 3 ms per 10 ms (not in Week 6) | | | |
| Whole activation time max (poll + state machine + publish) | same | | | |
| Poll gap max (start to start) | v3 proposal <= 50 ms | | | |
| Heartbeat send jitter, robot side (10 ms poll) | 1 s +/- 100 ms, 100 % | | | |
| Heartbeat receive interval, laptop side | 1 s +/- 100 ms | | | |
| Poll period where jitter first exceeds 100 ms | - | | | |
| Poll period where MQTT drops | - | | | |
| Capacity probe: largest payload accepted | >= 512 B (capacity only) | | | |
| Command round-trip, accepted ping | - (not in Week 6) | | | |
| d / o / u / m / b give the expected rejection | all correct | | | |
| Burst of 8: accepted / dropped (queue 4) | drops counted, none silent | | | |
| Loss detection time (AP off) | not specified | | | |
| Loss detection time (broker off) | not specified | | | |
| Recovery after AP back on (outage_test.py hotspot mode, timed from the Enter mark: conservative) | <= 20 s in every one of 20 trials (5 = preliminary) | | | |
| Recovery after broker back on (outage_test.py broker mode, laptop clock) | same | | | |
| RSSI range at the test spot / around the course (A10) | no heartbeat gap > 5 s | | | |

## Buddy 1 - notify_test (A15)

| Item | Design value | Measured | Trials | Log / video |
|---|---|---|---|---|
| LEDs show R/G/B/white, both LEDs | yes | | | |
| Fault and mission-complete beeps audible | yes | | | |
| 10 ms loop period deviation max: QUIET | <= 1 ms | | | |
| same: DESIGN rate (LED 100 ms, buzzer 50 ms) | <= 1 ms, not worse than QUIET | | | |
| same: STRESS (back-to-back writes) | <= 1 ms | | | |
| One LED update (2 pixels) max time | - | | | |
| One buzzer write max time | - | | | |

## Buddy 2 - encoder_test (A2, A11, A14 stall; A21 is a v3 proposal)

| Item | Design value | Measured | Trials | Log / video |
|---|---|---|---|---|
| Wheel diameter | - | 65 mm (ruler; about +/-1-2 %), circumference pi x 65 = 204 mm (calculated, not rolled) | 1 | - |
| Counts per rev (L / R) | - | L 2,538 / R 2,540 (4x decoded; total over 4 forward turns 10,150 / 10,160). Per turn L 2,524 / 2,551 / 2,495 / 2,580, R 2,526 / 2,574 / 2,522 / 2,538 (hand alignment about +/-2 %). 1 turn back: L 2,490, R 2,497 | 4 forward + 1 back per wheel | encoder_test_2026-10-25_hand.txt |
| mm per count | <= ~1 mm (A11) | 0.080 mm (calculated: 204 mm / 2,540); about 12x finer than A11 needs. Narrow 3.5 mm bar about 44 counts, wide 10.5 mm about 130 | - | same |
| Both channels working (L / R) | yes | yes / yes: A and B edges rise together (L 6,336 / 6,335, R 6,334 / 6,340) | same session | same |
| Steady counts/s at 30/40/50/70/100 % | - | | | |
| Lowest duty that turns the wheel | - | | | |
| L/R mismatch at 50 % | - | | | |
| Peak edges/s (highest duty) | - | | | |
| Callback-body time worst / load (excludes IRQ entry and dispatch) | ISRs only count | | | |
| Longest natural gap between count changes at 40 % / 100 % | well below 200 ms | | | |
| Stall detection: no-change time at detection, wheel held | 200 ms rule; A14 wants <= 200 ms in 10/10 | | | |
| Any START-UP false stall at 40 % | none | | | |
| Do L and R count the same sign for forward? | - | no: L negative, R positive for forward (mirrored motors); flip the left sign in software | 4 turns each | same |
| Invalid transitions | 0 | 0 / 0 (hand turning only; motor speeds not tested yet) | whole session | same |

## Buddy 3 - ir_test (A3, A7, A11)

| Item | Design value | Measured | Trials | Log / video |
|---|---|---|---|---|
| Level on black / white (each sensor) | - | all three: 0 at rest over white, 1 over black (from the logged levels; module LED state not recorded) | 3 sessions | ir_test_2026-10-25_*.txt |
| Mounted height | - | not measured (sensors hand-held, about 3 mm intended) | - | - |
| Pot margin (reliable range) | - | not measured. All three pots were re-tuned; that removed the chatter below | - | - |
| Chatter pulses at a line edge | - | after re-tune: 0 on L, C and B (L 32, C 54, B 46 printed edges; shortest pulse in the session 7.9 / 9.6 / 21.6 ms, far above the 500 us chatter limit). Before re-tune: L about 750 chatter pulses in one session, min pulse 2 us; C 0 | 4 runs after re-tune | ir_test_2026-10-25_3_after_retune.txt; before: _1_ and _2_before_retune.txt |
| Ambiguous edges / edges lost during a capture | 0 (else discard that capture) | after re-tune: 0 / 0. Before re-tune the barcode output buzzed at every edge: bursts of about 170-240 flips lasting 1-2 ms (once 9.5 ms), 18,085 of 18,105 barcode events AMBIGUOUS, 601,742 edges dropped (counter since boot), so no usable widths | 4 runs before, 4 after | same files |
| Narrow / wide bar time at steady drag | ratio >= 1.8 | not measured: the clean hand drags gave 8 and 14 barcode edges instead of the 30 a full *A* pass gives. Skipped; redo at controlled speed with the motors. Course barcodes: small 3.5 / 10.5 mm, medium 5.17 / 15.5 mm (P5-IS34_Course_Barcodes_A4.pdf) | - | ir_test_2026-10-25_3_after_retune.txt |

## Buddy 4 - imu_test (A4, A14 collision, NFR2)

| Item | Design value | Measured | Trials | Log / video |
|---|---|---|---|---|
| I2C addresses / chip | 0x19, 0x1E, "H43" (LSM303DLHC) | 0x1D only, WHO_AM_I 0x49 = **LSM303D** (accel + mag, one address, no gyro). Nothing at 0x19/0x1E | repeated runs, i2c_diag v3 + imu_test | imu_test_2026-10-22.txt |
| Wiring | Grove 6, SDA GP26, SCL GP27 | white = SDA (GP26), yellow = SCL (GP27), red = GY-511 VIN, black = GND. Grove rule: white = lower GPIO on every port | - | same |
| Real accel sample rate | >= 100 Hz (NFR2) | 100.0-100.5 Hz (bench) | every 2 s window | same |
| Read time at 100 kHz / 400 kHz | IMU <= 1 ms per 10 ms | 400 kHz: avg 355, max 386 us; 100 kHz: avg 1,311, max 1,322 us | many windows each | same |
| Loop work max (accel + mag + maths) | same | 400 kHz: 515-578 us still (up to 837 us in windows with handling); 100 kHz: 1,791-1,799 us = over budget, so use 400 kHz | many windows | same |
| Pitch sd, still | - (compare with the 5 deg hump threshold) | 0.082-0.105 deg at 400 kHz (0.21 deg at 100 kHz) | many windows | same |
| Failures: status / data / mag per window | 0 | 0 / 0 / 0 in every window; overruns: 1 in the first window, 0 after | all windows | same |
| Z zero-g offset (added row) | - | +0.118 g (\|a\| 0.882 g raw -> 1.000 g after GP21 calibration). Flip test: 0.881 g one way up, 1.143 g the other, so scale about 1.01 | 1 flip test, several calibrations | same |
| Peak pitch / h_est on 10, 20, 30 mm blocks | +/- 5 mm | NOT SHOWN. Static, best effort, run without calibration; Z offset removed by hand afterwards: tilt 4.16 / 6.23 / 13.18 deg. L not measured. Implied L = 138 / 184 / 132 mm, so the 20 mm trial is inconsistent | 1 each | same |
| Pitch error when pushed on flat table | - | not measured | - | - |
| Hand bumps detected as `[shock]` | >= 9/10 | NOT SHOWN: 2 `[shock]` lines (2.98 g, 2.13 g) over a bump + hump session; most events peaked 0.11-1.59 g, below 2 g. Bump/hump sequence not recorded | 1 session | same |
| False shocks over a hump / 20 mm drop | 0 | not separable (sequence not recorded) | - | same |
| Peak change and saturated samples in a bump | - | max change 2.98 g, max \|a\| 2.76 g, 0 saturated samples (+/-4 g is enough range) | 1 session | same |
| Heading shift with motors running | - | not measured (needs the power bank) | - | - |

## Buddy 5 - ultrasonic_test (A5, NFR3; prerequisite for NFR6, not NFR6)

| Item | Design value | Measured | Trials | Log / video |
|---|---|---|---|---|
| Module variant / supply | 3.3 V first | | | |
| Mean / sd at 50, 100, 150, 300, 500 mm (servo held, 20 readings each) | - | | | |
| Outcome counts: too short / long pulse / no fall / no rise / stuck high | - | | | |
| Valid readings out of 20 at each distance | - | | | |
| Trigger -> echo rise latency | - | | | |
| No-target behaviour | - | | | |
| Missing echoes with angled target | - | | | |
| Coarse scan time (start -> centred), 2 pings/ray | <= 1200 ms (NFR3) | | | |
| Fine scan time (start -> centred), 2 pings/ray | <= 2000 ms (NFR3) | | | |
| Unconfirmed rays per scan; settle ms/deg used | 0 | | | |

## What changes because of these results

(One line per finding that contradicts the design, and who owns the fix.)

- IR (25 Oct): a badly tuned LM393 module oscillates at every black/white edge (hundreds of edges per transition, 1-2 ms bursts) and floods the GPIO interrupt. Re-tuning the pot fixed it on the bench. The line and barcode code must survive a burst (merge edges inside a short window, bound the interrupt rate) and the pots need a tuning check before every run. Owner: B3 (B5 for the shared GPIO dispatcher).
- Encoders (25 Oct): about 2,540 counts/rev and 0.080 mm/count. A Barcode field of the low 16 bits (Version C) wraps every 65,536 counts = about 5.3 m, so bar widths are safe; an 8-bit delta would not fit (large-barcode wide bar 21 mm = about 261 counts, 20 mm quiet zone = about 249). 1 m is about 12,400 counts. At 250 mm/s about 3,100 counts/s per wheel (about 6,200/s both), below the v3 PIO threshold of about 10,000 edges/s; recheck at top speed in the motor runs. Left counts down for forward. Owner: B2 (field ranges for the field table), B3 (barcode widths).
