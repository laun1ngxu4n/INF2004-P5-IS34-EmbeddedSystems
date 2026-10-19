# Bring-up results (fill in, commit, link in the Tribunal template)

Date / place / board used:
Power arrangement used:
Firmware commit:

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
| IMU (0x19, 0x1E "H43", ~1 g) | OK | | |
| Ultrasonic (5 pings at a wall) | OK | | |
| Servo left / right / centre | moves both ways | | |
| NeoPixels red / green / blue | both LEDs, right colours | | |
| Buzzer two beeps | audible | | |
| GP28 pin voltage idle | - | | divider ratio measured: |
| GP28 lowest under one motor (sag) | - | | |
| Left motor fwd(A=PWM): which encoder, sign, wheel direction | own encoder only | | |
| Right motor fwd(A=PWM): which encoder, sign, wheel direction | own encoder only | | |
| Hand turn forward: L and R count direction | - | | |
| IR L / C / B flip on black/white | yes | | |
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
| Recovery after AP back on (button-marked; includes reaction time) | <= 20 s in every one of 20 trials (5 = preliminary) | | | |
| Recovery after broker back on | same | | | |
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
| Wheel diameter | - | | | |
| Counts per rev (L / R) | - | | | |
| mm per count | <= ~1 mm (A11) | | | |
| Both channels working (L / R) | yes | | | |
| Steady counts/s at 30/40/50/70/100 % | - | | | |
| Lowest duty that turns the wheel | - | | | |
| L/R mismatch at 50 % | - | | | |
| Peak edges/s (highest duty) | - | | | |
| Callback-body time worst / load (excludes IRQ entry and dispatch) | ISRs only count | | | |
| Longest natural gap between count changes at 40 % / 100 % | well below 200 ms | | | |
| Stall detection: no-change time at detection, wheel held | 200 ms rule; A14 wants <= 200 ms in 10/10 | | | |
| Any START-UP false stall at 40 % | none | | | |
| Do L and R count the same sign for forward? | - | | | |
| Invalid transitions | 0 | | | |

## Buddy 3 - ir_test (A3, A7, A11)

| Item | Design value | Measured | Trials | Log / video |
|---|---|---|---|---|
| Level on black / white (each sensor) | - | | | |
| Mounted height | - | | | |
| Pot margin (reliable range) | - | | | |
| Chatter pulses at a line edge | - | | | |
| Ambiguous edges / edges lost during a capture | 0 (else discard that capture) | | | |
| Narrow / wide bar time at steady drag | ratio >= 1.8 | | | |

## Buddy 4 - imu_test (A4, A14 collision, NFR2)

| Item | Design value | Measured | Trials | Log / video |
|---|---|---|---|---|
| I2C addresses / mag ID | 0x19, 0x1E, "H43" | | | |
| Real accel sample rate | >= 100 Hz (NFR2) | | | |
| Read time at 100 kHz / 400 kHz | IMU <= 1 ms per 10 ms | | | |
| Loop work max (accel + mag + maths) | same | | | |
| Pitch sd, still | - (compare with the 5 deg hump threshold) | | | |
| Failures: status / data / mag per window | 0 | | | |
| Peak pitch / h_est on 10, 20, 30 mm blocks | +/- 5 mm | | | |
| Pitch error when pushed on flat table | - | | | |
| Hand bumps detected as `[shock]` | >= 9/10 | | | |
| False shocks over a hump / 20 mm drop | 0 | | | |
| Peak change and saturated samples in a bump | - | | | |
| Heading shift with motors running | - | | | |

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
