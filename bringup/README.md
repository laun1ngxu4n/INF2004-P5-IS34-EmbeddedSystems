# P5-IS34 hardware bring-up tests (v3.2)

Eight small standalone programs. Each runs on the robot with the plain
Pico SDK (no RTOS yet) and prints real numbers over USB serial. They back
up, or contradict, the numbers in the submitted Week 6 design. A result
that contradicts the design is still good evidence: write it down and say
what changes.

| Program | Owner | Checks | Key numbers to record |
|---|---|---|---|
| `parts_check` | all | every part alive, wiring and direction | per-part OK/CHECK/FAIL, motor/encoder mapping, GP28 sag |
| `wifi_mqtt_test` | Buddy 1 | A1, A10, NFR5, NFR5b, NFR9 payload | join/DHCP, MQTT connect, poll and activation time, heartbeat jitter, RSSI, payload capacity, command verdicts, recovery time |
| `notify_test` | Buddy 1 | A15 | 10 ms loop jitter with LED/buzzer quiet, at design rate and stressed; cost per LED/buzzer write |
| `encoder_test` | Buddy 2 (+3) | A2, A11, A14 stall, A21 (v3 proposal) | counts/rev (4x decoded), mm/count, counts/s per duty, L/R mismatch, longest completed interval between counts, stall detection time |
| `ir_test` | Buddy 3 | A3, A7, A11 | polarity, line/floor separation, bar widths in us, chatter |
| `imu_test` | Buddy 4 | A4, A14 collision, NFR2 | real sample rate, read and loop time, pitch sd, tilt-block height, shock peaks |
| `i2c_diag` | any | wiring | Grove line levels, slow I2C scan of every Grove port in both wire orders, chip ID |
| `ultrasonic_test` | Buddy 5 | A5, NFR3 | 3.3 V works?, mean/sd at 50-500 mm, outcome counts, coarse/fine scan time |

What these are not: not the robot firmware and not NFR acceptance tests.
Each is a separate image with its own main(). They cannot show line
tracking, stopping distance, bypass safety, RTOS timing or concurrent fault
handling. Report results as measurements of hardware and assumptions
(A1-A20, and A21 from the v3 proposals), and say "measured on the bench" when a number is compared with
an NFR. `docs/requirements_coverage.md` lists, for every FR/NFR/A, what
these tests can and cannot show.

Debug output is USB only. GP0/GP1 are the IR sensors, so UART0 is off in
every test (Week 6 reviewer point).

## What changed in v3.2 (25 Oct 2026, fixes from the 25 Oct code review)

All confirmed in the code, fixed, host-tested, and built with the original
toolchain (SDK 2.1.1, GCC 13.2.rel1-2), 0 warnings. Not yet run on the robot.

- `parts_check` motor check (finding 03): any GP21 press is latched in the
  GPIO interrupt and checked before every drive and in every wait (start
  delay, run, 400 ms spin-down), so the motors never start while STOP is
  held and a short press between steps is not forgotten. Logic in
  `tests/common/motor_seq.h`.
- `encoder_test` stall rule (finding 04): both last-edge times and then the
  timer are read in one interrupts-off section, and a negative age is
  clamped to 0, so an edge between two reads can no longer look like a
  4,295 s stall. Longest gap (finding 14): the ISR now records the longest
  completed interval between count changes; the result line shows it next to
  the 1 ms sampled open gap, and the margin uses the worse of the two.
  Logic in `tests/common/enc_timing.h`.
- `ultrasonic_test` timed scans (finding 12): the servo is first moved to
  centre from where it really is (not timed), so the first ray gets its full
  settle time after manual stepping.
- `parts_check` and `ultrasonic_test` share one echo-width classifier
  (finding 13, `tests/common/sonar_class.h`): parts_check now reports pulses
  over 25,000 us as "long pulse" instead of a valid range.
- `imu_test` calibration (finding 16): the whole result is checked first
  (moved, not flat over 10 deg, offset out of range, no samples) and only
  then committed; a rejected calibration leaves offset, pitch zero, filter
  and peak untouched (`tests/common/imu_cal.h`).
- `i2c_diag` (finding 20): the verdict names the chip actually found (our
  LSM303D at 0x1D, and whether the wires are the design way round), and a
  clock line that never rises marks that port's results unreliable
  (`tests/common/chip_id.h`).
- `host_tests/run_host_tests.sh` (finding 15): every test runs once into a
  log and its exit status is checked before any filtering; the runner exits
  1 on any failure. New host tests: `test_motor_seq.c`,
  `test_enc_timing.c`, `test_sonar_imu_chip.c` (each was checked to fail
  when the old v3.1 behaviour is put back).
- Unchanged: `ir_test`, `notify_test`, `wifi_mqtt_test_prompt` (they build
  byte-identical to the 18 Oct images). Not changed on purpose: finding 11
  (command seq rule) belongs in the new binary wire spec, not the bench test.
- Reproducibility: all eight v3.1 images rebuild byte-for-byte from the v3.1
  source with this toolchain (`docs/logs/repro_check_2026-10-25.txt`).
  Build with `SOURCE_DATE_EPOCH` set to repeat a build exactly.

## Added 25 Oct 2026 (before v3.2)

- `tools/outage_test.py`: reconnect trials timed on the laptop clock (see
  wifi_mqtt_test step 6). No firmware change; all .uf2 images unchanged.
- `docs/P5-IS34_Buddy1_Test_Results.md`: evidence review added; hotspot
  name and IP addresses removed.

## What changed in v3.1 (22 Oct 2026, IMU bring-up)

- Our "GY-511" board carries an **LSM303D** (one address, 0x1D, WHO_AM_I
  0x49), not the LSM303DLHC the design assumed (0x19 + 0x1E "H43").
  `imu_test` and `parts_check` now detect either chip and configure both
  the same way (accel 100 Hz, +/-4 g).
- `imu_test` GP21 calibration now also measures and removes the Z zero-g
  offset (our board: about +0.12 g), waits 1 s after the press (hands
  off) and rejects the calibration if the board moved (pitch sd > 0.3 deg).
- DLHC path no longer right-shifts signed data (Barr C 5.3b): divide by 16.
- New `i2c_diag`: finds an I2C device on any Grove port in either wire
  order and identifies common accel/mag chips. Use it when a sensor says
  NO ANSWER.
- Results so far: `docs/bringup_results.md` (Buddy 4 table), logs in
  `docs/logs/imu_test_2026-10-22.txt`.

## What changed in v3

- New `parts_check`: all parts in one image through one shared GPIO
  dispatcher; motor/encoder mapping and battery sag.
- New `notify_test`: NeoPixel (PIO) and buzzer (PWM) against a 10 ms
  timer loop, for A15.
- WiFi: 4-slot command queue (overflow counted and published, not
  silent); strict flat-JSON parser (rejects nesting, leading zeros,
  trailing commas, duplicates); unknown command rejected; every poll goes
  through one measured helper so the capacity probe can no longer show up
  as a false poll gap; whole activation time; RSSI in every heartbeat.
- `mqtt_watch.py`: matches duplicate/oversize/malformed acks correctly,
  burst test, unknown and malformed commands, RSSI, `--csv` heartbeat log,
  offline also when no heartbeat ever arrives, unmatched counts in summary.
- IR: a dropped edge now marks the next stored edge, so widths across the
  gap are discarded at the right place (v2 still printed one).
- Encoder: design stall rule (PWM above 30 %, no count change for
  200 ms) with detection time and natural-gap margin; 40 % duty step
  added; late-sample counter reset per run.
- IMU: per-sample acceleration change and |a| deviation, `[shock]` lines
  above 2 g with the design's 200 ms hold-off, saturation count, whole
  loop work time.
- Ultrasonic: timed coarse and fine scans (NFR3) on a long GP21 press;
  final-reading overrun now counted in its batch.
- `host_tests/`: parser, command queue and IR gap checks run on a PC.
- Added during Buddy 1 testing (18 Oct): WiFi settings typed over serial
  (`wifi_mqtt_test_prompt`); missed heartbeats skipped and counted instead
  of a catch-up burst; buttons read every 1 ms at any poll period; GP21
  marks alternate OFF/ON. Results: `docs/P5-IS34_Buddy1_Test_Results.md`.

## Build

Pico SDK 2.x, board `pico_w`. Built and checked with SDK 2.1.1 and
arm-none-eabi-gcc 13.2.1, zero warnings (`docs/logs/build_report_2026-10-18.txt`).

VS Code Pico extension: *Import Project*, pick this folder, board Pico W.

Command line:

```
export PICO_SDK_PATH=/path/to/pico-sdk
mkdir build && cd build
cmake ..
make -j4
```

The `.uf2` files end up in `build/tests/<test>/`.

Two WiFi images come from the same source. `wifi_mqtt_test_prompt` always
builds and asks for SSID, password and broker IP over USB serial at
start-up (nothing stored), so it can be shared prebuilt. `wifi_mqtt_test`
has them compiled in and is only built once you create its config:

```
cp tests/wifi_mqtt/wifi_config.example.h tests/wifi_mqtt/wifi_config.h
```

then edit SSID, password and broker IP. `wifi_config.h` is git-ignored.

Host logic tests (no Pico needed, gcc on Linux/macOS/WSL):

```
sh host_tests/run_host_tests.sh
```

## Run

1. Hold BOOTSEL, plug in USB, drag the `.uf2` onto the RPI-RP2 drive.
2. Open the USB serial port (VS Code Serial Monitor, PuTTY, `screen`).
   Output starts about 3 s after reset.
3. Copy the whole serial output into `docs/logs/<test>_<date>.txt` and
   commit it. Take a short phone video for anything physical.
4. Fill in `docs/bringup_results.md` and use the commit link as evidence
   in the Tribunal template.

Record with every result: date, board, power arrangement, firmware
commit, trial count.

## Suggested order for one session (about 2.5 h)

1. `parts_check` (10 min). Fix anything that says FAIL before going on.
2. `wifi_mqtt_test` + `mqtt_watch.py` (40 min, 5 reconnect trials; 20 if time).
3. `notify_test` (5 min).
4. `encoder_test` (20 min).
5. `ir_test` (20 min).
6. `imu_test` (20 min).
7. `ultrasonic_test` (25 min).

## Per-test procedure

### parts_check (all)

Robot on blocks, wheels off the table, ultrasonic pointed at a wall
20-100 cm away.

1. Boot: read the summary. IMU and ultrasonic say OK/CHECK/FAIL; servo,
   LEDs and buzzer say LOOK/LISTEN, so write down what you saw and heard.
   No buzzer sound: check the mute switch on the board.
2. Battery (**skip: GP28 battery sense removed from scope, TEAM.md section 7**): the GP28 pin voltage is printed. Measure the battery with a
   meter and set `VBAT_DIVIDER = V_batt / V_pin` in `parts_check.c`. Near
   0 V or noisy means GP28 is not wired to the battery on this setup (A12).
3. Live line: turn each wheel forward by hand and check the count goes up
   on the correct side; move each IR sensor over black and white and
   check its level flips.
4. Press GP20 for the motor check. It reports which encoder moved and its
   sign for each motor and direction, and the lowest GP28 reading under
   load. Watch the wheels: "fwd(A=PWM)" should drive forward on both sides.
   Hold GP21 to stop at once.

### wifi_mqtt_test (Buddy 1)

Setup: phone hotspot on 2.4 GHz (school WiFi is WPA2-Enterprise and will
not work), laptop on the same hotspot running mosquitto with
`tools/mosquitto.conf`, then
`python tools/mqtt_watch.py --host 127.0.0.1 --csv docs/logs/hb_<date>.csv`.

1. Boot: record init time, `joined + DHCP in`, `mqtt connected in`.
   Design budget: join 5 s, MQTT 3 s.
2. Payload sizes are tested automatically once (capacity probe, not the
   512 B limit rule). lwIP's default output buffer is 256 bytes; this
   test raises it to 1024 in `lwipopts.h`.
3. Leave it running 2-3 minutes. Record poll call avg/max, activation
   max and gap max from the summary line, and the heartbeat deviation on
   both sides (robot send jitter, laptop receive interval).
4. Starvation: press GP20 to step the poll period 10 -> 45 -> 95 ->
   245 -> 495 -> 1995 ms, about 30 s each. Note when heartbeat jitter
   goes past 100 ms and whether MQTT drops.
5. Commands: in `mqtt_watch.py` press `p`, `d`, `o`, `u`, `m`, `b`, one
   at a time. Each must give the expected ack; record the round-trip of
   `p`. Then `r` (burst of 8): record accepted count and any
   `cmd_dropped` event. Summary at `q` shows unmatched counts.
6. Reconnect (A1): use `tools/outage_test.py` (added 25 Oct), not the
   GP21 marks. Broker mode runs Mosquitto itself, stops it for a random
   5-25 s and times the robot's return on the laptop clock:
   `python tools/outage_test.py --trials 5 --serial COM5` (stop any
   Mosquitto service and close Tera Term first; `pip install paho-mqtt
   pyserial`). Hotspot mode: run Mosquitto yourself, then
   `python tools/outage_test.py --mode hotspot --trials 5 --serial COM5`
   and press Enter BEFORE you switch the hotspot off, and again BEFORE you
   switch it on (marks first, so times are upper bounds). Add `--phase sweep`
   in broker mode to restart at planned points of the retry cycle. Pass: <= 20 000 ms
   every time. Outputs go to `docs/logs/outage_*`. Do not time outages with
   `mqtt_watch.py`: it drops and reconnects with the broker. The GP21 marks
   still work but include reaction time.
7. A10 (optional): carry the robot around the course area; the CSV has
   RSSI per heartbeat. Pass in the design: no heartbeat gap over 5 s.

### notify_test (Buddy 1, A15)

1. Press GP21: confirm both LEDs show red, green, blue, white, off, then
   the fault and mission-complete beeps.
2. Leave it for 1 minute (2 rounds of QUIET/DESIGN/STRESS). Record
   "period dev max" per phase and the LED/buzzer write times.
   A15 pass: within 1 ms in every phase, and DESIGN/STRESS not worse than
   QUIET by a measurable amount. GP20 mutes the buzzer.

### encoder_test (Buddy 2, with Buddy 3)

1. Measure the wheel diameter with a ruler.
2. Mark the tyre, press GP21 to zero, turn one wheel exactly 1 rev by
   hand. Record counts. Repeat 3 times per wheel, and backwards once.
3. If counts stay near 0 while A edges increase, channel B is not
   working (single-channel encoder or wiring).
4. Put the counts/rev and diameter into the two `#define`s, rebuild:
   it prints mm per count. A11 needs about 1 mm or finer.
5. Lift the wheels. Press GP20 five times (30/40/50/70/100 % duty).
   Record steady counts/s, L/R mismatch, peak edges/s, callback load and
   the longest natural gap between count changes (the stall-rule margin).
6. Stall (A14): run at 50 % and grab one wheel firmly. The `[stall]` line
   gives the no-change time at detection (rule: 200 ms). Repeat 10 times.
   A `START-UP` stall means the wheel never turned within 200 ms of the
   start: a false-trip risk for the design rule at that duty.

### ir_test (Buddy 3)

1. Hold each sensor at mounted height over white floor, then the black
   line. Record the level for each (polarity) and the LED behaviour.
2. Adjust the potentiometer until line vs floor is reliable; note how
   much margin there is (turn until it fails both ways).
3. Slide a sensor slowly across the 18 mm line edge: count `CHATTER`
   lines (comparator sitting on its threshold).
4. Drag IR_BARCODE over a printed Code 39 barcode at a steady speed.
   The `previous level lasted` values are bar widths in time; check
   wide/narrow ratio against the 1.8x decoder threshold. Discard any
   capture with an `AMBIGUOUS` or `EDGES LOST` line.

### imu_test (Buddy 4)

Wiring: Grove 6, red to the GY-511 **VIN** (not its 3V3 pin: the SDA/SCL
pull-ups on the board run from VIN), black GND, white = SDA (GP26),
yellow = SCL (GP27). On every Grove port white is the lower-numbered GPIO.

1. Record the I2C scan and chip lines (our board: 0x1D, LSM303D). If it
   says no chip found, run `i2c_diag`.
2. Flat and still, press GP21 and take your hands off; check the `[cal]`
   line does not say REJECTED and |a| reads about 1.00 g afterwards. GP21
   means "this is flat": never press it with the robot tilted. Calibration
   is lost on reset. Record the `[stats]` line: sample rate
   (~100 Hz expected), pitch sd, read time, loop work max. Press GP20 and
   repeat at the other I2C speed.
3. Tilt the robot onto blocks of known height (10, 20, 30 mm) under one
   end. Read `rel` (tilt from the calibrated flat); return to flat between
   blocks and check `rel` goes back to about 0. With `WHEELBASE_MM` set,
   compare `h_est` to the real height (target +/- 5 mm).
4. Push the robot back and forth on a flat table: how far does pitch
   move with no tilt at all? That is the acceleration error.
5. Collision (A14): bump the robot into a box at low speed by hand 10
   times: count `[shock]` lines (design wants >= 9/10). Then roll it over
   a hump or drop one end from 20 mm: count false shocks (want 0).
   Record peak change and saturated samples.
6. Watch `heading` with motors off and on.

### ultrasonic_test (Buddy 5)

Jumper wires only (not Grove). Start at 3.3 V.

1. Flat target (box or book) at 50, 100, 150, 300, 500 mm measured from
   the sensor face. Press GP20 at each. Record mean, sd, valid count and
   the other outcome counts.
2. Point it at open space: record what the no-target case looks like.
3. Angle the target 30-45 degrees: how often does the echo go missing?
4. GP21 short press steps the servo so you can check it moves.
5. NFR3: robot still, a few objects around it. Hold GP21 for about 1 s
   and release. Record COARSE and FINE `TIME` against 1200/2000 ms and the
   unconfirmed-ray count. If pings on a ray disagree, the settle time is
   too short: raise `SERVO_SETTLE_MS_PER_DEG`, rebuild, repeat, and record
   the lowest value that gives agreeing pings, with its scan time.
If it fails at 3.3 V, stop and decide on 5 V + divider as a team.

## Safety

- Lift the wheels for every motor test. GP21 stops the motors.
- Never connect a 5 V ECHO straight to GP7.
- Test on the approved power arrangement only.
