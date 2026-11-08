# Tribunal 1 (PostTri): what was decided

The decisions, risks and planned tests from our Week 8 Tribunal 1 record
(`IS34_PostTri1.doc.docx`) that matter for the code. They come from
Version B (live discussion) and Version C (the review after it). Names
and student IDs are left out; buddies are shown by role:

| | Role |
|---|---|
| B1 | WiFi, MQTT, telemetry (+ notification) |
| B2 | motors, encoders, PID |
| B3 | IR sensors, line following, barcode |
| B4 | IMU, hump, Safety / fault monitor |
| B5 | ultrasonic, servo, obstacle; Mission controller and integration |

The curveball was **Change Requirement 1, Minimum-Bit Data Representation**
([`CHANGE_REQUIREMENT_1.md`](CHANGE_REQUIREMENT_1.md)). How the current
code compares, and the work list: [`../CHANGE_REQ1_PLAN.md`](../CHANGE_REQ1_PLAN.md).

The record describes the **Week 6 design** (message queues `motion_cmd_q`,
`branch_q`, `nav_cmd_q`, `mission_evt_q`, `line_cmd_q`; `ir_edge_t`; a
6-state line machine; Safety at the highest priority with a watchdog).
The built code differs. Where it does: `CHANGE_REQ1_PLAN.md` section 6.

## 1. Decisions

### Fault and stop word (B4, with B1, B2, B5)

- **One bitmask, one latched bit per cause:** stop causes and faults (motor stall, encoder fault, IMU fault, no echo, Wi-Fi loss).
- **Set before anything is posted:** a cause's bit is set before any message about it.
- **Interrupts off for every change:** several tasks and interrupts write the same word, so every read-modify-write runs with interrupts off (a short lock).
- **Wi-Fi loss is a warning, never a stop.** A Wi-Fi drop must not stop the robot (Week 6 rule). It goes in a warning mask, separate from the motion-inhibit mask. Only inhibit bits may raise a fault state. Everything that reads the fault word must respect this, not only Motion.
- **Who clears what:** Telemetry sets Wi-Fi loss on disconnect and clears it on reconnect through the shared lock helper. Only Safety clears the inhibit bits.
- **Reset rule (proposal, from the review):** keep a second "new since last check" mask. A producer sets the fault bit and its tracking bit together. A reset clears the tracking bits before it starts checking. At the end, in one locked step, it clears a fault bit only if its tracking bit is still clear and the cause is gone. That way a fault that recurs during the reset is never lost, and a fault that persists is never cleared. This must pass the injection test (section 3) before it counts as agreed.
- **MQTT STOP** sets the STOP bit as soon as Telemetry accepts it, through the same helper as the STOP button, before anything is queued. A STOP must never be lost to a full command queue. (On the bench, an 8-command burst into a 4-slot queue dropped 3.)

### Where to pack (B2, B3)

- **Pack at the boundary, not at the source.** The encoder interrupt and PID stay 32-bit. Packing happens where data is handed over: messages, the result slot, telemetry.
- **Every width needs a written range, even the kept ones.** "Keep 32 bits" still needs a written range (counts per revolution, longest run, top speed): signedness and width for each stored field. PID accumulators and temporary values are listed separately.
- **Question for the lecturer (B5 to ask):** does internal computational state (PID accumulators, temporaries) count as application data?

### Encoder interface for the barcode (B2, B3)

- **One interface:**
  - Motion keeps the 32-bit count.
  - At each edge, the barcode interrupt reads it and stores the **low 16 bits**.
  - A bar width is the difference between consecutive edges, mod 2^16.
  - Motion sends no separate 16-bit value.
- **The branch position is a separate field** with its own unit (encoder counts, or mm converted once by Barcode).
- **Valid only while** fewer than 65,536 counts pass between two edges. That's easily true for 3–9 mm bars, but confirm it once counts per mm are measured. A capture with a longer gap is thrown away. Barcode is off during turns.

### Field widths (B3, B4, B5)

- **Edge record:** packing 1 byte of flags plus a 32-bit timestamp saves **nothing**. The compiler pads it back to 8 bytes; the "~60 %" claim was wrong and withdrawn. A real saving needs:
  - the flag bits plus a shorter timestamp in one 32-bit word (a 27-bit µs time wraps after about 134 s, which is fine within one capture); or
  - separate arrays.
  Report `sizeof` and the ring size before and after. Check whether a sensor ID is needed at all (only the barcode input interrupts).
- **Line state:** 3 bits (Week 6 counted 6 states). *The built code has 4 states, so 2 bits.*
- **Branch position:** 14 bits (up to 16,383) assumes a course of at most about 10 m. State that assumption and what happens beyond it.
- **Hump height:** 0–30 mm at 1 mm resolution is 5 bits, not 6. Keep 6 only with a stated 0.5 mm resolution. Define the out-of-range value.
- **IMU / Safety states:** 3 bits for 6 motion classes, 3 bits for 7 IMU states, 2 bits for 3 Safety states (as stated in the discussion).
- **Mission / obstacle (B5):**
  - Keep an explicit sensor status: a missing echo must never read as a clear path.
  - Don't narrow command IDs or timestamps without justifying it: an old result must never advance the bypass.
  - One common header with the agreed bit layouts and pack / unpack functions.

### MQTT wire format (B1)

- **Compact binary instead of text,** with a format version at the start.
- **Packed into a byte array with explicit shifts and masks.** Never send a C bit-field struct: the bit order and padding are the compiler's choice.
- **The spec:** little-endian, byte offsets, widths, scales, the version byte, reserved bits sent as 0 and ignored on receipt.
- **Golden test vectors** shared by the firmware and the laptop decoder.
- Record the current text sizes first. *Superseded: the before/after comparison comes from the packed on/off build switch, below.*
- Connecting and reconnecting do not change.

### Shared data and concurrency (B5 with all)

- **A table of every shared object:** its writers and readers, one writer or several, and its protection (none, the interrupts-off helper, or a spinlock on dual core).
- **A single aligned 32-bit read is atomic, but snapshots of several fields still need the lock.**
- **The real hazard is several writers doing read-modify-write** on one word. The Cortex-M0+ has no exclusive-access instructions, so those need interrupts off.
- **Single core is a team decision.** Measure the longest interrupts-off section.
- **Fallback:** if a packed shared word cannot be made race-free, that field keeps its minimum width but gets its own storage unit.

### Measuring (all, B5 coordinating)

- **One build with a compile-time switch** (packed on / off), with the same features, buffer sizes and compiler settings. This gives a fair before/after comparison of RAM and message sizes. Comparing with the Week 6 estimates would not measure real savings.
- **Measure record sizes and allocated bytes separately.** Logical bit savings are not proof of RAM savings.

### Priority order (from the review)

1. Shared fault and stop words (B4, B5): a lost fault is a safety failure.
2. Cross-buddy formats: the MQTT wire format (B1) and the encoder interface (B2, B3).
3. Each buddy's own fields.

### Kept as designed ("no change")

- **WiFi:** low priority, never stops the robot, reconnects within 20 s.
- **Pins and hardware:** a software-only change.
- **Safety:** cuts the motors within 20 ms even if Motion hangs, backed by a 500 ms hardware watchdog. *Not built: `safety_task` is a stub, and there is no watchdog.*

## 2. Risk register (R1–R9, all open)

| Risk | Area | Cause | Impact | Mitigation | Lead |
|---|---|---|---|---|---|
| R1 | Timing | packing / interrupt / lock overhead | missed control or sensing deadlines | bound lock and interrupt work; measure the combined load | B5 |
| R2 | Wiring | sonar supply / ECHO voltage not verified | damage or unsafe range | scope ECHO; check GP6/GP7 and the supply | B5 |
| R3 | Interface | scale / layout / version mismatch | wrong command or measurement | shared codec spec; reject malformed input; golden vectors | B1 |
| R4 | Shared state | lost update or stale reset | a fresh stop or fault erased | guarded set / clear and acknowledgement; race tests | B4 |
| R5 | Identity | IDs wrap while old records survive | an old completion accepted | bound lifetime and reuse; test cancel, wrap and restart | B5 |
| R6 | Numerical | narrowing / quantisation / overflow | control or sensing error | range and error analysis; physical regression tests | B2 |
| R7 | Clearance | invalid or stale sonar, incomplete sweep | unsafe bypass or recovery | keep status, age and per-move guards; safe fault | B5 |
| R8 | Tasks overwriting | several tasks updating the fault mask unprotected | a new fault erased, robot fails to stop | interrupts off while editing; ignore Wi-Fi loss in motor-stop checks | B4 |
| R9 | Hardware dependency | packed encoder widths depend on unmeasured counts per revolution | wrong widths | measure first | B2 |

The review asked for each risk to be re-rated with a one-line reason. All
of them were Medium likelihood and High severity.

## 3. Tests the team committed to (all planned)

| Area | Test | Pass when | Evidence |
|---|---|---|---|
| Safe-off | STOP / fault during PWM; Motion unresponsive | safe output within 20 ms; no re-enable while inhibited; watchdog checked separately | scope trace on GP8–GP11, build ID |
| Safety / shared state | two causes; a new fault during reset; full detail queue | no lost inhibit; STOP / OBSTACLE kept; reset never restarts motion | injected interleaving, output logs |
| Fault reset rule | a different cause and the same cause injected during a reset | no fault lost, motion never restarts | log |
| Wi-Fi drop | Wi-Fi off during line following | robot keeps running; warning set, then cleared on reconnect | log |
| MQTT STOP | STOP inside an 8-command burst | robot stops; no later command restarts it | log |
| Encoder widths | the same capture decoded from 16-bit differences and from 32-bit counts | same bar classification | log |
| Mission / Motion | cancelled completion; forced ID wrap | no stale leg advance; STOPPED is not an ordinary completion | ID / state trace |
| Mapping | update during a capture; READ_FAIL + junction; bad packet | one map per capture; old turn cleared; a rejected map leaves the valid one | packet and navigation logs |
| Binary interfaces | boundary, signed, reserved, short and wrong-version inputs | valid values round-trip; invalid input rejected before any of it is applied | firmware / laptop golden vectors |
| Integrated load | all tasks, reconnects, telemetry saturation | control deadlines kept; ≥1 Hz telemetry; heartbeat 1 s ±100 ms; reconnect ≤20 s | task / interrupt / poll / broker logs |
| Obstacle / recovery | threshold, invalid sonar, blocked leg, leg-five handover | unknown never grants clearance; stop ≥150 mm at ≤250 mm/s; recovery ≤4 s or safe fault | waveform, distance, state / video |
| Motion / hump regression | 1 m and turn trials; known 10 / 20 / 30 mm humps | distance ±5 %; turns 90 ±10°, 180 ±15°; hump ±5 mm | measured trials, raw data |
| RAM / messages | packed on / off builds | real allocations and message sizes reported; no unsupported savings | map / size files, payloads, stack high-water |

## 4. Other points from the review

- **Hardware readiness** was lowered to Yellow: pins are assigned, but the sonar supply / ECHO and motor safe-off have not been verified on hardware. To do: scope the ECHO voltage, take a wiring photo, check the motor outputs across a reset.
- **WiFi bench evidence (assumption A1)** is only partly evidenced:
  - join + DHCP 2.8–4.4 s; MQTT connect 61–81 ms;
  - 2 broker-off recoveries in ≤2.8 s;
  - the laptop saw heartbeat gaps up to 2.8 s, which nearly trips the 3-missed-heartbeats offline rule. Test with WiFi power-save on and off.
  - Still to do: the full 20 trials under integrated load.
- **Change-log evidence:** later submissions give a reason instead of "NIL", and cite real evidence (commit, log, timestamp), not just the requirement document.
- **Final summary:**
  - Strongest part: the safety path as designed.
  - Most likely to fail: integrating shared state under the minimum-bit change (fault words with several writers, the encoder interface, the MQTT binary format), plus hardware quantities not yet measured.
