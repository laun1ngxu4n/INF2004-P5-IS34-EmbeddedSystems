# Change Requirement 1 (Minimum-Bit Data Representation): what has to change

Status on 9 Nov: **not implemented.** This page lists what the change
requirement and our Tribunal 1 decisions (PostTri, Versions B and C) ask
for, what the code does now, and what to change, subsystem by subsystem.
Nothing here has been changed in the code yet.

Sources: [`course/CHANGE_REQUIREMENT_1.md`](course/CHANGE_REQUIREMENT_1.md)
and [`course/TRIBUNAL1_POSTTRI.md`](course/TRIBUNAL1_POSTTRI.md) (Version B
decisions, Version C fixes): text copies of the course `.docx` files.

## 1. In short: how much will it change?

| | Changes? |
|---|---|
| **What the robot does** (line following, barcodes, turns, bypass, hump) | **Should not change.** Packing changes how values are stored and sent, not the decisions made with them. Done right, every run looks the same. |
| **The code** | **A lot of places, small edits each.** Every subsystem's shared data, the barcode edge ring, Telemetry's messages and the laptop panel's decoder. |
| **The fault word and Safety** | **Yes, on purpose.** The requirement wants faults as bits in one shared word. Our Tribunal "no change" row also says Safety cuts the motors if Motion hangs, and that part is still a stub today. Once Safety acts on fault bits, the robot can stop in situations where it does not stop now (e.g. a stalled motor). That is new behaviour and needs its own tests. |
| **The laptop panel** | **Yes.** It parses text today; it has to decode the binary messages. |
| **RAM** | **Small saving in bytes, but real.** Our own data is about 12 KB in the WiFi build. Of that, 8 KB is the console log ring (text, see 4.9) and 1.3 KB is Telemetry's text outbox. The packing itself saves about 2 KB at most (outbox and edge ring). Everything else is tens of bytes per subsystem. Report it honestly; do not claim more. |
| **MQTT message size** | **Big saving in percent.** A status line is about 100 characters of text today; packed, it is about 12 bytes. |

Where it can go wrong (the risks to test):

- **Overflow or truncation:** a field made too narrow for a value that does occur.
- **Races:** two writers in one packed word.
- **Firmware and panel disagree:** the bit layout differs between the two ends.
- **Stale results:** a command ID that wraps too soon accepts an old result.

## 2. Baseline to record before any change ("before")

Measure on the commit before the first change, and keep the numbers:

| What | How | Value now |
|---|---|---|
| RAM, whole image | `arm-none-eabi-size` on the `.elf` | normal: 24,088 B bss + 108 B data; WiFi/MQTT: 90,568 B bss + 116 B data |
| RAM, our modules | `arm-none-eabi-size` on each `app_program` object | about 12,004 B (WiFi build): `telemetry.o` 9,513 B (log ring 8,192 + outbox 8 × 161); `barcode.o` 1,268 B (edge ring 128 × 8 B); `app_timing.o` 676 B; `line.o` 175 B; the rest under 60 B each |
| MQTT payload sizes | count bytes of each topic's messages in a panel log | to measure: `is34/telemetry`, `is34/sensors`, `is34/cmd/map`, `is34/ack/map` |
| Task timing | the `[tmg]` table every 10 s (`app_timing`) | to save from a full run |

PostTri decision: compare with **one build and a compile-time switch**
(`APP_PACKED` 0 / 1). It keeps the same features, buffers and compiler
settings, so the comparison is fair.

## 3. The shared fault word (do first: a lost fault is a safety failure)

**Now:**

- STOP causes are one bitmask (`app_stop.h`: `STOP_BUTTON`, `STOP_MQTT`, `STOP_FAULT`), set from any context, interrupts too.
- `STOP_FAULT` is never set.
- The real faults are separate variables: IMU (`gb_chip_ok`, `imu.c`), no echo (`gb_sensor_fault`, `obstacle.c`), stuck line sensor (`g_fault_why`, `line.c`), quadrature errors (`g_enc_invalid`, a counter, `motor.c`).
- Motor stall, encoder fault and Wi-Fi loss are not detected at all.
- `safety_task` is an empty stub, and there is no watchdog.

**Change:**

| Bit | Cause | Set by | Stops the motors? |
|---|---|---|---|
| 0 | STOP button | GP21 ISR | yes |
| 1 | STOP over MQTT | Telemetry | yes (already set before anything is queued, as PostTri requires) |
| 2 | motor stall (PWM on, no encoder counts) | Safety (new check) | yes |
| 3 | encoder fault (too many quadrature errors) | Safety | yes |
| 4 | IMU fault | IMU task | no for the line; yes only if the hump is required |
| 5 | sonar no ECHO | Obstacle task | yes while the forward watch is on (as now) |
| 6 | line sensor stuck | Line task | yes (as now, Mission halts) |
| 7 | Wi-Fi lost | Telemetry | **no: warning only** (PostTri decision) |

- Two masks: the **stop mask** (bits that inhibit motion) and the **warning mask** (Wi-Fi loss). Only stop-mask bits may stop the robot or raise a fault state.
- Set and clear only through helpers that use `APP_LOCK`, because several writers share one word (the Cortex-M0+ has no atomic read-modify-write).
- Reset rule (PostTri Version C row 1): a "new since last check" tracking mask, so a fault that comes back during a reset is not erased. Only Safety clears stop-mask fault bits. Telemetry clears Wi-Fi loss on reconnect.
- Safety task: check the word every 20 ms, `motor_safe_off()` on any stop bit, and the motor-stall and encoder checks. Decide whether to add the hardware watchdog (the PostTri "no change" row claims a 500 ms watchdog).

Tests (PostTri Version C):

- Inject the same fault, and a different one, during a reset: the robot never restarts.
- Wi-Fi drop during line following: the robot keeps running, the warning is set, and it clears on reconnect.

## 4. Field by field

Widths are from each value's valid range. Where the range depends on a
measurement, measure first (PostTri R6, R9). "Keep 32 bits" is allowed only
with a written range (PostTri Version C row 4).

### 4.1 Motion and encoders (B2: `motion.c`, `motor.c`)

| Field | Now | Range | Bits | Note |
|---|---|---|---|---|
| encoder counts `g_enc_count[2]` | `int32_t` | ± one run: 0.08 mm/count; a 20 m run is 250,000 counts | 19 + sign | **keep 32**: written in the ISR, read as one aligned word (atomic). Write down the range as the justification (PostTri: pack at the boundary, not at the source) |
| command type | `int32_t` | NONE, DRIVE, TURN, HALT, FOLLOW (5) | 3 | |
| command speed | `int32_t` | ±1000 mm/s (really ±200) | 11 signed | |
| command amount | `int32_t` | drive ±5000 mm, turn ±360 deg | 14 signed | |
| result | `int32_t` | NONE, BUSY, DONE, STOPPED, TIMEOUT (5) | 3 | |
| command ID `h_cmd_id` | `ID` (32) | increments per command | keep, or 8 with a wrap rule | PostTri R5: do not narrow without a stale-result test |
| measured speeds | `int32_t` | ±2000 mm/s | 12 signed | |
| quadrature errors | `uint32_t` | count | → fault bit 3 (plus a small count for diagnosis) | |
| PID integrators, float maths | `float32_t` | | keep | ask the lecturer whether computational state is in scope (PostTri action for B5) |

### 4.2 Line following (B3: `line.c`)

| Field | Now | Range | Bits |
|---|---|---|---|
| line state `g_state` | `int32_t` | ON, SEARCH, LOST, FAULT (**4**, not the 6 in the PostTri) | 2 |
| fault reason `g_fault_why` | `int32_t` | none, ONE, BOTH | 2 → into the fault word |
| steering `g_steer` | `int32_t` | ±30 mm/s (STEER_CORR / SEARCH) | 7 signed |
| speed share `g_speed_pct` | `int32_t` | 20..100 % | 7 |
| junction sides | `int32_t` | 2 flags | 2 |
| junction count | `uint32_t` | junctions per run (under 64) | 8 (wraps; only differences are used) |
| corrections, sweeps | `uint32_t` | per run | 16 each |
| last junction position | `int32_t` | odometer counts | same width as the encoder counts |

### 4.3 Barcode (B3: `barcode.c`)

| Field | Now | Range | Bits | Note |
|---|---|---|---|---|
| edge ring `bar_edge_t` | `int32_t odo` + `uint8_t level` = 8 B × 128 | | 16 + 1 | PostTri decision: store the **low 16 bits** of the count; bar widths are differences mod 2^16. One 32-bit word per edge = 4 B × 128 (saves 512 B). Test: same widths as with 32-bit counts on the same capture |
| result `barcode_result_t` (about 24 B) | | | | |
| — letter | `uint8_t` | A–Z, 0–9 (36) + none | 6 | |
| — ok, reversed | `bool` × 2 | | 1 + 1 | |
| — fix kind | `uint8_t` | 0..3 | 2 | |
| — elements | `int32_t` | ≤ 99 | 7 | |
| — narrow / wide width | `int32_t` (0.1 mm) | narrow 2–6 mm, wide 6–15 mm | 8 each (0–25.5 mm) | |
| — sequence | `uint32_t` | increments | 8 with a wrap rule | compared only for "new?" |
| mapping `g_map_ch[4]`, `g_map_mv[4]` | 8 B | 4 × (letter 6 + move 2) | 4 × 8 bits | the same packing as the MQTT command (5.2) |
| debug widths `g_dbg_w[40]` | `int32_t` | 0.1 mm, under 30 mm | 9 each | test code only: decide in or out |

### 4.4 Obstacle (B5: `obstacle.c`)

| Field | Now | Range | Bits | Note |
|---|---|---|---|---|
| front distance | `int32_t` | 0..4000 mm, or OPEN, or FAULT | 12 + 2-bit status | PostTri: keep an explicit status; "no echo" must never read as clear |
| sensor fault | `bool` | | → fault bit 5 | |
| profile: left, right, width, front | `int32_t` | ±600 mm | 11 signed each | |
| profile: uncertainties | `int32_t` | 0..300 mm | 9 | |
| profile: found, clear ×2, seen ×2 | `bool` × 5 | | 5 | |
| rays, fault rays | `int32_t` | ≤ 17 | 5 each | |
| servo angle | `int32_t` | 50..130 deg | 8 | |

### 4.5 IMU and hump (B4: `imu.c`)

| Field | Now | Range | Bits | Note |
|---|---|---|---|---|
| hump height (`g_ramp_peak_mm10`, `g_height_mm10`, `g_one_h_mm10`) | `int32_t` (0.1 mm) | 0–50 mm at 0.1 mm | 9 | PostTri said "5 bits for 0–30 mm at 1 mm"; ours is 0.1 mm resolution, so state the resolution |
| sin(pitch), sin(roll) ×1000 | `int32_t` | ±1000 | 11 signed each | |
| ramp reports, jolts | `uint32_t` | per run | 8 each (wrap) | |
| chip OK, calibrated | `bool` | | → fault bit 4 / 1 flag | |

### 4.6 Mission and vehicle state (B5: `mission.c`)

| Field | Now | Range | Bits |
|---|---|---|---|
| mission state `g_mstate` | `int32_t` | IDLE, CAL, RUN, MANOEUVRE (4) | 2 |
| pending letter | `uint8_t` | 37 values | 6 |
| features (obstacles, IMU), armed | `bool` × 3 | | 3 |
| bypass leg / side | locals | legs 1–5, side ±1 | 3 + 1 |
| event flag bits `MSN_FLG_*` | already a 7-bit mask | | ✅ already compliant |

### 4.7 Telemetry (B1: `telemetry.c`)

| Field | Now | Change |
|---|---|---|
| outbox `tele_msg_t` | topic `uint8_t` + 160 B text, × 8 | topic 2 bits; binary payloads (section 5); slot about 16 B → outbox about 130 B instead of 1,288 |
| state text pointer | `const char *` | the 2-bit mission state from 4.6 |

### 4.8 Data shared between tasks or with an ISR

Write the per-object table PostTri Version C asks for:

- each shared object;
- its writers and readers;
- whether it has one writer or several;
- its protection: none (aligned 32-bit, one writer) or `APP_LOCK`.

Rules:

- Never put fields written by an ISR and fields written by a task into the same word.
- Measure the longest interrupts-off section.

### 4.9 Decide and write down: what is not "application data"

| Item | Proposal |
|---|---|
| console log ring (8 KB of text, `telemetry.c`) | a copy of the console text for the panel's log tab: not application data. Keep it, and say so |
| event sentences (`is34/event`), e.g. "junction 2: LEFT" | human-readable text for the operator: keep as text, and say why |
| task timing monitor (`app_timing`) | diagnostics, not application data |
| PID and float intermediates | ask the lecturer (PostTri) |

## 5. MQTT binary format (B1 with the panel)

PostTri Version C decisions:

- byte arrays built with explicit shifts and masks; never send a C bit-field struct;
- little-endian;
- byte 0 = format version;
- reserved bits sent as 0 and ignored on receive;
- a spec document and golden test vectors shared by the firmware and `tools/panel/is34_panel.py`.

### 5.1 Robot → laptop

| Topic | Now (text) | Binary (draft) |
|---|---|---|
| `is34/telemetry` (1 s) | `state=RUN speed=120,118 dist=1234 line=0 junctions=3 barcode=A map=K=L,... ramp_peak=12.3 stop=0` (about 100 B) | version 8; mission state 2; line state 2; stop/fault word 8; warnings 8; speed L, R 12+12; distance 16; junctions 8; last letter 6; ramp peak 9; mapping 32 → about 15 B |
| `is34/sensors` (0.5 s) | `ir=010 steer=.. hold=.. inbar=.. front=.. tilt=.. h10=.. imu=..` (about 60 B) | version 8; IR 3; hold, inbar, IMU OK 3; steer 7; front 12 + status 2; sin pitch 11; height 9 → about 8 B |
| `is34/ack/map` | `OK K=L,...` / `REJECTED ...` | version; result code 3 bits; mapping 4 B; reason code → about 6 B |
| `is34/event` | sentences | keep text (4.9) |
| `is34/log` | console copy | keep text (4.9) |

### 5.2 Laptop → robot

| Topic | Now | Binary |
|---|---|---|
| `is34/cmd/map` | `K=L,Q=R,X=F,M=U` (15 B) | version + 4 × (letter 6 bits + move 2 bits) = 5 B |
| `is34/cmd/start`, `/stop` | text | 1 byte command code, or keep the topic as the command with an empty payload (decide) |

Robot-side checks:

- reject a wrong version or wrong length;
- reject an out-of-range letter or move;
- STOP stays a latched bit set before anything is queued (already true).

## 6. PostTri entries that do not match the code (fix or explain)

| PostTri says | Code | Action |
|---|---|---|
| queues `motion_cmd_q`, `branch_q`, `nav_cmd_q`, `mission_evt_q`, `line_cmd_q` | none: one event flag (`gh_flg_mission`) plus shared slots under `APP_LOCK` | explain in Week 13 (design changed since Week 6, TEAM.md section 7); the "inter-task messages" to measure are the command slot, the barcode result, the edge ring and the outbox |
| `ir_edge_t` with a sensor ID | `bar_edge_t` (barcode sensor only) | no sensor ID needed (PostTri Version C row 6 already asks this) |
| line state machine has 6 states | 4 | use 4 (2 bits) |
| Safety at the highest priority, 20 ms safe-off, 500 ms watchdog ("no change") | stub, priority 5 (the kernel's USB and WiFi tasks sit above), no watchdog | implement (section 3) or correct the claim |
| repository `laun1ngxu4n/INF2004-P5-IS34-EmbeddedSystems` | this work is in `mhdikhwan/kwanxkwan` | the Week 13 evidence has to be in the team repo |

## 7. Week 13 evidence checklist

- [ ] Data representation table: every field, range, bits, how it is stored (sections 4 and 5 are the draft)
- [ ] RAM before / after: `arm-none-eabi-size` and per-module sizes, `APP_PACKED` 0 and 1
- [ ] Message sizes before / after: each MQTT topic, the outbox slot, the edge record, the barcode result
- [ ] Correctness: the on-target stage tests pass with `APP_PACKED` 1, plus one full course run
- [ ] Performance: the `[tmg]` timing table under a full run, before and after (no overruns, intervals within limits); the longest interrupts-off time
- [ ] The new PostTri tests: fault injected during a reset; Wi-Fi drop during line following; STOP during a command burst; 16-bit vs 32-bit bar widths on the same capture; panel decoder golden vectors

## 8. Suggested order

1. Record the baseline (section 2).
2. Fault word and Safety (section 3).
3. MQTT binary formats and the panel decoder (section 5): the biggest visible saving.
4. Barcode edge ring and result (4.3), then the other fields (4.1, 4.2, 4.4–4.7) behind `APP_PACKED`.
5. Per-object protection table (4.8) and the "not application data" decisions (4.9).
6. Measure and write the Week 13 evidence (section 7).

After steps 2 to 4: the full stage retest (`testing/TESTING_GUIDE.md`).
