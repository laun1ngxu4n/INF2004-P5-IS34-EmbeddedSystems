# Change Requirement 1: Minimum-Bit Data Representation

**Status (8 Nov, evening):** implemented in the code and checked on the laptop. The
robot retest (section 7.2) is still to be done.

**Requirement:** [`course/CHANGE_REQUIREMENT_1.md`](course/CHANGE_REQUIREMENT_1.md),
word for word. **Team decisions it follows:**
[`course/TRIBUNAL1_SUMMARY.md`](course/TRIBUNAL1_SUMMARY.md).
**MQTT format:** [`MQTT_WIRE_FORMAT.md`](MQTT_WIRE_FORMAT.md).

This page holds the Week 13 evidence the requirement asks for:

1. the data representation table (section 3);
2. the RAM comparison (section 4);
3. the message size comparison (section 5);
4. correctness and performance (section 7).

## 1. What changed, in short

| Area | Change |
|---|---|
| **Fault word** | Every fault condition is one bit of one shared fault word, kept with the STOP reasons in one packed word in `app_stop.c`. The bits are motor stall, encoder, IMU, no echo, line sensor stuck (one / both), Wi-Fi loss and Motion hung. Changes go through `app_stop_fault_set()` / `app_stop_fault_clear()` under `APP_LOCK`. This replaced `gb_chip_ok`, `gb_sensor_fault` and `g_fault_why`. |
| **Safety** | `safety_task` was an empty stub. It now keeps three of the bits (section 6). |
| **MQTT** | The status, sensors and mapping-acknowledgement messages are binary (`wire.c`, version byte, explicit shifts), and so is the mapping command. START and STOP have empty payloads. Event sentences and the console log stay text. The laptop panel decodes the binary and still reads an older robot image's text. |
| **Shared data and messages** | Each field is in the bits its range needs, as a C bit-field (`APP_FIELD(n)`). One word per writer, so a field is never read-modify-written by two contexts. Values going into a field are clamped (`app_clamp`), never cut off. |
| **Barcode edge ring** | The ring keeps the low 16 bits of the odometer (Tribunal 1); the decoder widens them back exactly. 1,024 B → 512 B. |
| **The build switch** | `APP_PACKED` 1 (the default) packs. `APP_PACKED` 0 gives every field its own full storage unit again, for the before/after comparison: `make EXTRA_CFLAGS=-DAPP_PACKED=0`. The build profile now includes `EXTRA_CFLAGS`, so builds with different flags never share objects. |

Not covered:
- `bringup/` (October bench code, not in the firmware);
- the hands-on stage tests' scratch data (bench samples, debug width arrays): test images only, never in the mission image.

## 2. What stayed wider, and why

The requirement asks for the bits each field's *valid range* needs. These
keep a wider type, each for a stated reason.

| Data | Width | Why |
|---|---|---|
| Encoder counts `g_enc_count[2]`, odometer positions (`g_junc_odo`, the barcode mark's centre, `count_l/r`) | 32 bits | **Range:** 0.080 mm per count, so a 20 m run is 250,000 counts (19 bits + sign) and there is no fixed upper bound for a run.<br>**Concurrency:** written in the encoder interrupt and read as one aligned word, so the read is atomic without a lock. Tribunal 1 decided to pack at the boundary, not in the encoder interrupt. |
| Command ID `h_cmd_id` | 32 bits (kernel `ID`) | It only increases. A narrow ID would wrap and could let an old result count as the new command's (Tribunal 1, R5). |
| Counters since boot (junctions, corrections, sweeps, IMU reports and jolts, quadrature errors, decoded results, `g_bar_seen`) | 32 bits | They only go up and are used as differences ("new since"). A narrow counter would have to wrap in step with every reader. Where a counter is sent or stored in a record, it is narrowed there (junctions mod 256 on MQTT, the result's `seq` in 8 bits). |
| Times in µs (`g_stop_time_us`, `g_echo_rise_us`, `g_link_us`, timing monitor) | 32 bits | Times from the 32-bit hardware timer (wraps after 71 min); only differences are used. |
| Flags with different writers (the sonar's `gb_echo_rose` / `gb_echo_done` (ISR and task), `gb_watch`, `gb_idle_ping`, `gb_scanning`, the IMU's `gb_cal_request` / `gb_calibrated`, `gb_enabled`, `gb_paused`, `gb_busy`) | 1 byte each | Tribunal 1 fallback: "if packing a shared field into one word cannot be made race-free, that field keeps its minimum width but gets its own storage unit". Packing them with another writer's bits would need a lock in an interrupt. |
| Filter and controller state inside one task or interrupt (PID integrators, float filters, the ADC level trackers `g_env_w/b`, `g_rel_lo/hi`, loop variables) | computing width | Computational state, not data handed over. Whether it counts as application data is the question Tribunal 1 left for B5 to ask the lecturer. |

**Not application data** (kept, and said so):

| Data | Why |
|---|---|
| Console log ring (8 KB, `telemetry.c`) | a text copy of the console for the panel's Robot log |
| Event sentences on `is34/event` | sentences for the operator, not values |
| Task timing monitor (`app_timing.c`) | diagnostics |
| Telemetry outbox slots (162 B each) | they carry the event text; the binary messages use 4–15 bytes of a slot |

## 3. Data representation table

"Before" is the representation in the code before the change.
Positions in the encoder-count rows are encoder counts (0.080 mm).

### Shared fault and STOP word (`app_stop.c`, `g_word`)

| Field | Valid range | Bits | Representation | Before |
|---|---|---|---|---|
| STOP reasons | button, laptop, fault | 3 | bit-field in `stop_word_t` (4 B) | `uint32_t g_stop_reasons` |
| faults | 8 causes, 1 bit each | 8 | bit-field in the same word; `FAULT_...` masks | `bool gb_chip_ok`, `bool gb_sensor_fault`, `int32_t g_fault_why`; stall, encoder, Wi-Fi, hung not represented |

### Motion and encoders (`motion.c`, `motor.c`)

| Field | Valid range | Bits | Representation | Before |
|---|---|---|---|---|
| command type | NONE, DRIVE, TURN, HALT, FOLLOW | 3 | `motion_cmd_t` bit-field | `int32_t` |
| command speed | ±1023 mm/s (in use ±200) | 11 signed | bit-field, clamped | `int32_t` |
| command amount | ±8191 (drive mm / turn deg) | 14 signed | bit-field, clamped | `int32_t` |
| command result | NONE, BUSY, DONE, STOPPED, TIMEOUT | 3 | `motion_status_t` bit-field | `int32_t` |
| measured speeds L, R | ±2047 mm/s | 12 signed each | bit-fields, clamped | `int32_t` × 2 |
| duties last written L, R (new, for Safety) | ±1000 ‰ | 11 signed each | `motor_duty_t` bit-fields | not stored |
| encoder A/B levels L, R | 2 bits each | 2 × 2 | one byte, masks and shifts | `uint8_t[2]` |

### Line following (`line.c`, `g_out`, written only by the line task)

| Field | Valid range | Bits | Representation | Before |
|---|---|---|---|---|
| line state | ON, SEARCH, LOST, FAULT | 2 | `line_out_t` bit-field (4 B in all) | `int32_t` |
| steering | ±40 mm/s | 8 signed | bit-field | `int32_t` |
| speed share | 35..100 % | 7 | bit-field | `int32_t` |
| junction branches | crossing, left arm | 2 | bit-field | `int32_t` |
| line fault reason | none, one, both | 2 fault bits | in the fault word | `int32_t g_fault_why` |

### Barcode (`barcode.c`, `barcode.h`)

| Field | Valid range | Bits | Representation | Before |
|---|---|---|---|---|
| edge: odometer | the low 16 bits (an edge is never 32,768 counts behind) | 16 | `bar_edge_t` bit-field, 4 B per edge | `int32_t` |
| edge: level | black / white | 1 | bit-field | `uint8_t` |
| result: sequence | new-or-not | 8 (wraps) | `barcode_result_t` bit-field (8 B in all) | `uint32_t` |
| result: ok, reversed | yes / no | 1 + 1 | `bool` bit-fields | `bool` × 2 |
| result: letter | 7-bit ASCII Code 39 character | 7 | bit-field | `uint8_t` |
| result: elements | ≤ 99 | 7 | bit-field | `int32_t` |
| result: narrow, wide width | 0..51.1 mm in 0.1 mm | 9 each | bit-fields, clamped | `int32_t` × 2 |
| result: fix kind | 0..3 | 2 | bit-field | `uint8_t` |
| mapping pair: letter, move | 7-bit ASCII; L R F U | 7 + 2 | `bar_pair_t` × 4 | `uint8_t[4]` × 2 |
| mark width | 0..51.1 mm in 0.1 mm | 9 | `bar_mark_t` bit-field | `int32_t` |
| raw ADC sample | 12-bit ADC | 16 (own halfword: ISR writer) | `uint16_t` | `int32_t` |

### Obstacle (`obstacle.c`, `obstacle.h`)

| Field | Valid range | Bits | Representation | Before |
|---|---|---|---|---|
| profile flags: found, clear ×2, seen ×2 | yes / no | 5 × 1 | `obstacle_profile_t` bit-fields (16 B in all) | `bool` × 5 |
| profile rays, fault rays | ≤ 17 | 6 each | bit-fields | `int32_t` × 2 |
| profile front | −2..4350 mm (OPEN / FAULT / mm) | 14 signed | bit-field, clamped | `int32_t` |
| profile left, right edge | ±4095 mm | 13 signed each | bit-fields, clamped | `int32_t` × 2 |
| profile width | ±8191 mm | 14 signed | bit-field | `int32_t` |
| profile uncertainties (total, left, right) | 0..4095 mm | 13 signed each (signed: used in `sgn × unc`) | bit-fields, clamped | `int32_t` × 3 |
| forward distance `g_front_mm` | −2..4350 mm | 16 (own halfword: several writers) | `int16_t` | `int32_t` |
| echo width | 0..25,000 µs valid | 16 (own halfword: ISR writer) | `uint16_t`, saturates (= open) | `uint32_t` |
| servo angle | 50..130° | 8 | `uint8_t` | `int32_t` |
| sensor fault | yes / no | 1 fault bit | `FAULT_NO_ECHO` | `bool` |

### IMU and hump (`imu.c`, `g_out`, written only by the IMU task)

| Field | Valid range | Bits | Representation | Before |
|---|---|---|---|---|
| sin(pitch), sin(roll) × 1000 | ±1023 | 11 signed each | `imu_out_t` bit-fields, clamped | `int32_t` × 2 |
| climbed height | ±409.5 mm in 0.1 mm | 13 signed | bit-field, clamped | `int32_t` |
| one-wheel height | 0..204.7 mm in 0.1 mm | 11 | bit-field, clamped | `int32_t` |
| ramp peak reported | 0..409.5 mm in 0.1 mm | 12 | bit-field, clamped | `int32_t` |
| IMU answering | yes / no | 1 fault bit | `FAULT_IMU` | `bool` |

### Mission (`mission.c`)

| Field | Valid range | Bits | Representation | Before |
|---|---|---|---|---|
| mission state | IDLE, CAL, RUN, MANOEUVRE | 2 | `mission_word_t` bit-field (mission task only) | `int32_t` |
| pending barcode | 7-bit ASCII, 0 = none | 7 | the same word | `uint8_t` |
| armed, use obstacles, use IMU | yes / no | 3 × 1 | `mission_flags_t` bit-fields, under `APP_LOCK` | `bool` × 3 |
| event flag bits `MSN_FLG_...` | 7 events | 7 | kernel event flag | unchanged (already bits) |

### Safety (`safety.c`, task-local)

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| stall counters L, R | 0..50 periods | 6 each | `safety_state_t` bit-fields |
| fault hold | 0..100 periods | 7 | bit-field |
| encoder window | 0..50 periods | 6 | bit-field |

### Telemetry (`telemetry.c`)

| Field | Valid range | Bits | Representation | Before |
|---|---|---|---|---|
| outbox topic | 4 topics | 2 | `tele_msg_t` bit-field | `uint8_t` |
| outbox length (new: binary payloads contain 0 bytes) | 0..159 | 8 | `uint8_t` | NUL-terminated text |
| MQTT payloads | section 5 | | binary, `wire.c` | text |

## 4. RAM comparison

Measured with [`tools/barr/cr1_measure.sh`](../../tools/barr/cr1_measure.sh)
(`arm-none-eabi-size` on the image and on our objects; record sizes from
`sizeof` in the robot's own debug info). There are two comparisons:

- **original:** the last commit before the change;
- **`APP_PACKED=0`:** the new code with every field in its own storage unit.

The second is the fair same-features comparison Tribunal 1 asked for.

| | Original | `APP_PACKED=0` | `APP_PACKED=1` | Saved (vs original) |
|---|---|---|---|---|
| Our modules' RAM, normal image | 3,795 B | 3,818 B | **3,228 B** | 567 B (14.9 %) |
| Our modules' RAM, WiFi/MQTT image | 12,004 B | 12,027 B | **11,437 B** | 567 B (4.7 %; 8 KB of it is the log ring) |
| Image RAM (.data + .bss), normal | 24,196 B | 24,216 B | 23,624 B | 572 B |
| Image code (.text), normal | 301,480 B | 304,288 B | 305,672 B | **+4,192 B** (new Safety, wire.c, bit-field access code) |

The packed build holds more data than the original (the fault word, Safety's state, the duties, the link time) and still uses 567 B less.

Record sizes:

| Record | Original | `APP_PACKED=0` | `APP_PACKED=1` |
|---|---|---|---|
| barcode edge (`bar_edge_t`) × 128 | 8 B (1,024) | 8 B (1,024) | **4 B (512)** |
| barcode result (`barcode_result_t`) | 24 B | 32 B | **8 B** |
| obstacle profile (`obstacle_profile_t`) | 44 B | 44 B | **16 B** |
| motion status (`motion_status_t`) | 28 B | 28 B | **20 B** |
| motion command (`motion_cmd_t`) | 16 B | 16 B | **12 B** |
| line results | 5 × 4 B | 16 B | **4 B** |
| IMU results | 5 × 4 B | 20 B | **12 B** |
| STOP + fault word | 4 B + 3 variables | 8 B | **4 B** |
| mapping (4 pairs, `bar_pair_t`) | 8 B | 16 B | **8 B** (9 bits used of each 2-byte pair) |
| telemetry outbox slot (`tele_msg_t`) | 161 B | 162 B | 162 B (it carries event text) |

## 5. Message size comparison

"Before" is measured from 30 panel logs of 7–8 Nov: 12,487 status lines,
23,894 sensor lines, 319 commands.

| Message | Before (text) | After (binary) | Bandwidth |
|---|---|---|---|
| `is34/telemetry`, 1 Hz | 86–111 B, mean 93 | **15 B** | 93 → 15 B/s |
| `is34/sensors`, 2 Hz | 57–66 B, mean 59 | **8 B** | 118 → 16 B/s |
| `is34/ack/map` | 18 B | **5 B** | |
| `is34/cmd/map` | 15 B | **4 B** | |
| `is34/cmd/start`, `/stop` | 2–3 B | **0 B** (the topic is the command) | |
| `is34/event`, `is34/log` | text | text (unchanged, section 2) | |

## 6. Concurrency: who writes each packed object

On the RP2040 (Cortex-M0+, single core: `app_lock.h` refuses SMP), an
aligned 32-bit read is atomic. A read-modify-write of a word that two
contexts write needs interrupts off (`APP_LOCK`), because there are no
exclusive-access instructions. Every packed word below has one writer, or
is changed only under `APP_LOCK`.

| Object | Writers | Readers | Protection |
|---|---|---|---|
| `g_word` (STOP + faults) | any task, interrupts (STOP button) | everyone | every change under `APP_LOCK`; reads are one word |
| `g_duty` | Motion (`motor_write`), any context (`motor_safe_off`) | Safety | `APP_LOCK` |
| `g_enc_prev` | the encoder interrupt only (and once before it is on) | the same interrupt | none needed |
| `g_cmd_slot`, `g_status` | callers / Motion task | Motion task / everyone | `APP_LOCK` (as before) |
| line `g_out` | the line task only | Motion, Mission, Telemetry | one writer; a field read is one word read |
| `g_ring[]` (barcode edges) | the barcode interrupt only (whole-entry store) | the barcode task | one writer; `g_ring_w` published after the entry |
| `g_last`, `g_map[]`, `g_mark` | the barcode task / any caller | Mission, Telemetry, Line | `APP_LOCK` (as before) |
| IMU `g_out` | the IMU task only | Mission, Telemetry, tests | one writer |
| `g_mw` (mission state, pending) | the mission task only | Telemetry | one writer |
| `g_flags` (armed, features) | test task (`mission_arm`, `mission_features`) | the mission task | `APP_LOCK` |
| obstacle flags, `g_front_mm`, echo width | ISR / tasks | tasks | not packed together: each has its own byte or halfword (section 2) |
| `safety_state_t` | Safety's own local | Safety | task-local |

The longest interrupts-off section is still to be measured on the robot (section 7.2).

## 7. Correctness and performance

### 7.1 Done on the laptop (8 Nov)

- **Builds.** The normal, WiFi/MQTT and stage-test images were built with `-Wall -Wextra`, packed and unpacked (8 builds): no warnings or errors in our code. The build had no `-Wall` before; the old warnings it showed (an unused interrupt handler and an unused variable outside their `#if`, missing initialisers in a test table) were fixed.
- **Golden vectors.** `python tools/panel/test_wire.py` builds the robot's `wire.c` on the laptop and checks it against the panel's decoder. It covers 10 messages with boundary and out-of-range values (clamping), 2 map commands round trip, and 3 broken map commands refused: **PASS, 15/15.** A deliberately wrong field width makes it fail (26 problems), so the test does catch layout errors.
- **Barr C:** `python tools/barr/check.py` shows only the documented deviations. `APP_FIELD` is added to deviation D4 (a declaration macro cannot be a function).
- **Barcode widths from 16-bit edges.** `odo_from16()` gives back exactly the 32-bit count while an edge is less than 32,768 counts (2.6 m) behind the odometer. The ring is emptied every 10 ms, so the bar widths are the same numbers as before by construction. The S3-B00 runs in 7.2 confirm it on the robot.

### 7.2 To do on the robot (the full retest)

Flash the new images, use the **updated panel** (the old one cannot read the binary messages), and run:

1. **Every stage, as in `testing/TESTING_GUIDE.md`.**
   - Expect the same results as on 8 Nov.
   - The panel's status, sensors and mapping work as before, with a new "Faults / warnings" field.
2. **Timing (performance).**
   - Save the `[tmg]` table (printed every 10 s) from a full run with the normal image.
   - Save the same from a run with an `APP_PACKED=0` image: `make NET=lwip MQTT=1 EXTRA_CFLAGS=-DAPP_PACKED=0`, saved as `images/wifi/is34_mission_wifi_unpacked.uf2`.
   - Pass: no overruns, and the intervals and run times about the same.
3. **Wi-Fi drop during line following.**
   - Switch the hotspot off while it follows the line.
   - Pass: it keeps going; after reconnecting, the panel's warning shows "Wi-Fi lost" for at most a second, then clears.
4. **MQTT STOP inside a command burst.**
   - Press START and STOP quickly several times on the panel, ending with STOP.
   - Pass: the robot stops and nothing later restarts it.
5. **Motor stall (new Safety check).**
   - While it drives, hold one wheel still for about 2 s.
   - Pass: about 1 s after the wheel stops, the motors go off. The event says "Safety: a wheel driven hard is not turning". After 2 s the fault clears and START works again.
6. **Mapping.**
   - Send a mapping from the panel: the acknowledgement shows it.
   - Send one with the same barcode twice (paste `A=L,A=R,...`): it is refused, the event says why, and the old mapping is kept.

## 8. What changes for the robot's behaviour

| Change | Effect |
|---|---|
| Packing (everything in section 3) | none intended: the same values in fewer bits, clamped to ranges no real value reaches |
| Safety: motor stall | **new:** a wheel driven at ≥ 80 % duty that turns slower than 15 mm/s for 1 s stops the motors (normal driving uses about 25 %). Cleared 2 s later; Mission waits for START. |
| Safety: Motion task hung | **new:** motors on but Motion not run for 200 ms stops the motors |
| Encoder, IMU, Wi-Fi-loss bits | warnings only: reported on the panel, never stop the robot |
| MQTT | binary messages: **the panel must be the updated one** (`tools/panel/is34_panel.py`, with `is34_wire.py`) |

## 9. Tribunal 1 decisions: how each was done

| Decision | Done |
|---|---|
| One latched bit per fault cause in a shared word | yes (section 3). The "new since last check" reset mask was not needed: each stop fault has one owner that sets and clears it (Safety), and Line, Obstacle and IMU clear only their own bits when their cause is gone. |
| Wi-Fi loss is a warning, never a stop | yes: `FAULT_WARNINGS`. Nothing stops on that bit. |
| MQTT STOP sets the STOP bit before anything is queued | yes (already true; there is no command queue that could drop it) |
| Pack at the boundary; the encoder interrupt and PID stay 32-bit | yes (section 2) |
| Barcode ring keeps the low 16 bits of the count; widths are differences | yes |
| Binary wire format: version byte, explicit shifts, little-endian, golden vectors | yes (`MQTT_WIRE_FORMAT.md`, `test_wire.py`) |
| Explicit sensor status (no echo never reads as clear) | yes: `front_status` on the wire, `SONAR_FAULT` / `FAULT_NO_ECHO` kept |
| Command IDs not narrowed without a stale-result test | kept 32-bit (section 2) |
| A packed on/off build switch for a fair comparison | yes: `APP_PACKED` |
| A per-object protection table | yes (section 6) |
| Safety cuts the motors if Motion hangs (claimed "no change") | now built (section 8). There is still no hardware watchdog. |
| The structures named in the record (`motion_cmd_q`, `branch_q`, `ir_edge_t`, a 6-state line machine) | the built code has an event flag and locked slots instead of queues, the barcode edge ring instead of `ir_edge_t`, and 4 line states. The table in section 3 is about the built code. |
