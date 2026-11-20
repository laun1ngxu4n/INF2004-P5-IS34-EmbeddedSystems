# Week 6 design report: what to change in the next report

A checklist for whoever writes the next report. It goes through the
submitted Week 6 design review (`IS34_W6_design_review.docx`) section by
section: what it says now, what to write instead, and why. Leave the
submitted file as it is and make these changes in the new version.

Sources: `TEAM.md` section 7 (scope changes), `CONTEXT.md`,
`testing/PLAN.md`, the code in `app_program/`. Status words:
**built** = in the code, not yet proven on the robot; **tested** = has a
logged result in `testing/` or `bringup/docs/`. Update the status of each
item when its test is done.

Legend: **Change** = the design is wrong now; **Add** = something new to
describe; **Drop** = remove, or report as "not done / not tested".

## Front matter
- **Add** a short "Changes since Week 6" paragraph: GP28 removed,
  instructor rules for barcodes, obstacles and the ramp, IMU part, single
  core, essentials-only test plan.

## 1.1 Functional requirements

| ID | Week 6 says | Change to | Why |
|---|---|---|---|
| FR2 | Code 39 with a configurable table; **default A=Left, B=Right, C=Straight, D=U-Turn**; Z reserved | Code 39 **A-Z (and 0-9)**, read **in both directions**. **No default mapping**: the 4 pairs come over MQTT just before the run; a character with no mapping stops the robot at the junction and is reported | Instructor brief (TEAM.md 7.1 B1, B2, B7) |
| FR2a | updated mapping over WiFi/MQTT at runtime | Keep. Add the format: topic `is34/cmd/map`, payload `K=L,Q=R,X=F,M=U` (exactly 4 pairs, each character and move once), ack on `is34/ack/map` (OK or REJECTED + reason; old mapping kept) | B8; format still to confirm with the instructor |
| FR3 | L/R/Straight at the next junction; **U-Turn as soon as the barcode is passed** | **All four moves, U-turn included, at the next cross-junction** | Instructor brief (B6) |
| FR4 | humps, highest peak; humps 10-30 mm | Ramp about the height of the **lower practice ramp** (higher ramp ignored); report the height reached, estimated from the IMU | Instructor ramp update (27 Oct) |
| FR5 | detect and profile obstacles | Add: **estimate and report the width with its uncertainty** (width +/- U mm); obstacles 10-20 cm wide | Instructor brief (TEAM.md 7.2) |
| FR7 | telemetry ... plus **on-demand snapshot** with per-field age/validity | Telemetry 1 s on `is34/telemetry`, results on `is34/event`. Snapshot: **not built** (drop or mark "not implemented") | Essentials only |
| FR9 | FAULT on collision, stall, **critically low battery**, line loss, no bypass, timeout, missed heartbeat | Remove **low battery** (GP28 removed). Built today: STOP (button/MQTT) stops at once; line lost, no clear side, blocked leg, sonar fault, manoeuvre timeout stop and report. **Collision, stall, task heartbeats and fault latching: not built** | TEAM.md 7; safety task not in essentials |
| FR10 | MQTT heartbeat while connected | The template's heartbeat runs every **5 s** on its own topic; NFR5b's 1 s is **not met**. Either change it to 1 s or say telemetry (1 s) serves as the heartbeat | Not changed yet |
| FR12 | ... report **mission complete** | START/STOP (GP20/GP21 or MQTT) built. **Mission complete: not built** (no end marker; Z decoding was disabled anyway) | Course end convention unknown |
| FR13 | motion classes, collisions, turn rate | **Not built** (drop or "not implemented"). **13 Nov (lecturer): IMU collision detection removed; replaced by "lifted or overturned: motors stop at once"** (built, `docs/team/course/IMU_REQUIREMENT_CHANGE.md`) | Essentials only; lecturer 13 Nov |
| FR14, FR15 | pending safety events, epoch/command-ID rejection, ordered processing | Simplified: one event flag for Mission + command ids from Motion (`motion_get_status`); STOP is a shared "reason" word that always wins. No epochs, no safety queue | Simpler design for single core |
| FR16 | blocked bypass stops; **retrace** to the stop pose | Blocked bypass stops and reports. **Retrace: not built** | Essentials only |

## 1.2 Non-functional requirements

| ID | Change to / status |
|---|---|
| NFR1 / NFR1a | Line task 20 ms (built). Speed now **150 mm/s everywhere** (slow for barcodes, TEAM.md 7.1 B4) |
| NFR3 | Coarse 1.2 s / fine 2.0 s no longer applies: one scan of 21 rays (40-140 deg, 5 deg steps), settle wait, **5 pings per ray**, done **twice** from two positions. Re-set the limit from the measured scan time (S5-O01 notes) |
| NFR4 | Not measured (drop or "not tested") |
| NFR5 / NFR5a / NFR5b | Reconnect trials, snapshot and 1 s heartbeat: **not tested / not built** (`testing/PLAN.md` dropped list) |
| NFR6 | Week 6 says trip distance **at least 300 mm**. **Decided 15 Nov: change NFR6 to 190 mm, 20 Nov to 175 mm** (`S5_TRIP_MM`; was 250 until 15 Nov): the team found the robot stopped too far away; Tribunal 1 set "stop at least 150 mm away at up to 250 mm/s". At the Mission speed of 120 mm/s it stopped at 168 mm with 185 and 175 mm with 190, so about 160 mm with 175 (do not raise the speed without re-measuring), from where the scan still sees both edges of the course box | Team, 15 Nov; Tribunal 1 |
| NFR8 | Turns: **tested** (Stage 1, 1 Nov: 90 deg about right, 180 about 5 deg over, track width tuned to 123 mm). Straight 1 m: **dropped**. Barcode at 150 mm/s and ramp +/- 5 mm: to be tested (S3-B, S4-R01) |
| NFR9 | RAM: images about 80 KB static (bss) with MQTT; stack/CPU measurements **not done** |
| NFR10 | STOP calls `motor_safe_off()` inside the button interrupt (no task involved). A separate Safety path for a hung Motion: **not built** |
| NFR11, NFR12 | Not tested |

## 2. Hardware integration / pin layout
- **Drop** the GP28 "Battery voltage sense / VBAT_SENSE" row: not used,
  no solder jumper (TEAM.md 7).
- **Change** the IMU from "GY-511 (LSM303DLHC)" to **GY-511 board with an
  LSM303D** (one I2C address 0x1D, WHO_AM_I 0x49), +/-4 g range.
- HC-SR04: the 5 V module **works at 3.3 V** on the bench (about 50 mm to
  1.5 m), so no 5 V supply or ECHO divider (A5 bench result).
- Barcode IR sensor was **replaced** (26 Oct).
- Power: USB power bank into VIN for runs; never USB and power bank
  together.

## 3. RTOS task table
- **9 tasks become 8**: Notification is merged into Telemetry (and the
  LED/buzzer part is not built).
- **Priorities 2-10 become 5-12**: the kernel's own USB console (3) and
  WiFi (4) tasks sit above ours. Same order. Single core (`SMP=0`).
- Telemetry: 100 ms task. The 10 ms cyw43/lwIP poll is the kernel's WiFi
  task, not ours; MQTT stays inside it (two hooks, `app_mqtt_rx/tx`).
- Barcode decoder: wakes on a 10 ms timeout (not a semaphore per edge);
  edges come from an interrupt ring stamped with the **encoder odometer**.
- Obstacle scanner: the 60 ms task only does the forward watch; the
  profiling scan runs when Mission calls it.
- Safety task: exists for timing only; its fault work is **not built**.
- Inputs/outputs: replace the queue names (next section).

## 4. Task interaction diagram
Week 6 queues and what replaced them (TEAM.md section 5):

| Week 6 | Now |
|---|---|
| `line_cmd_q` | `line_steer_mm_s()`, latest value read by Motion every 10 ms (IF-03) |
| `motion_cmd_q` + `completion_slot` | `motion_drive/turn/follow/halt()` return an id; `motion_get_status()` gives id + DONE/STOPPED/TIMEOUT (IF-04) |
| `mission_flg` | `flg_mission` with bits STOP, START, JUNCTION, LINE_LOST, BARCODE, OBSTACLE, PING (IF-01) |
| `nav_cmd_q` | `barcode_last()` + MSN_FLG_BARCODE |
| `obs_evt_q`, `obs_ctl_q` | MSN_FLG_OBSTACLE + `obstacle_scan()` / `obstacle_combine()` |
| `barcode_cfg_q` | `barcode_set_mapping()` called from the MQTT hook |
| `telemetry_q` | `tele_event()` outbox -> `is34/event`; status line -> `is34/telemetry` |
| `stop_reasons` / `fault_flg` | `app_stop_request(reason)` / `app_stop_reasons()` (IF-02) |
| `safety_evt_q`, `notify_q`, `branch_q`, `line_ctl_q` | not built |

Redraw the diagram with these names. Add the shared GPIO interrupt
dispatcher (`app_gpio.c`): encoders, IR, ECHO and STOP all arrive there.

## 5. System-level statechart
Week 6: READY / FOLLOW_LINE / BARCODE_MANOEUVRE / AVOID_OBSTACLE /
REACQUIRE_LINE / FAULT / COMPLETE.
Now (`task_mission.c`): **IDLE -> CAL (IMU level, 1 s) -> RUN, with
MANOEUVRE** for junction moves and bypasses; any stop (STOP, line lost,
no clear side, timeout, sonar fault, no mapping) returns to IDLE with the
reason reported. **No latched FAULT state and no COMPLETE** (not built).
INIT self-test: only "IMU found"; no power check (GP28 removed).

## 6. Subsystem statecharts
- **6.1 WiFi**: topics `is34/cmd/map|start|stop`, `is34/event`,
  `is34/telemetry`, `is34/ack/map`. Snapshot not built. Reconnect is the
  template's.
- **6.2 Motion**: "PID" becomes **feed-forward + PI per wheel** on speed
  (40 ms average), a ramp/slow-down profile, equal-distance correction,
  command timeout 2x expected + 1 s. Turns are spins on the spot, track
  width 123 mm (tuned). **Stall detection not built.** STOP wins over
  every PWM write.
- **6.3 Line + barcode**: junction = both sensors black for 8-45 mm, then
  left white again with centre black. **U-turn at the junction.** Decoder:
  elements in encoder distance, chatter < 0.5 mm merged, quiet zone 40 mm,
  3 widest of 9 = wide, start `*` checked, reversed if needed; full table.
  Branch candidates and arm/disarm: not used.
- **6.4 IMU**: no gyro. Height = sum of (distance driven x sin(pitch)),
  pitch relative to the level reading taken at START, low-pass ~100 ms;
  reported when level again (plus L x sin(peak pitch) as a second value).
  Motion classes and collision: **not built**.
- **6.5 Obstacle**: COARSE_SCAN + FINE_SCAN become **one scan** (40-140
  deg, 5 deg), settle 3 ms/deg (>= 60 ms), **median of 5 pings**,
  background (> front + 250 mm = open), **2-ray hysteresis** for edges,
  sonar-cone pull-in (`S5_BEAM_HALF_DEG`), **second scan after reversing
  80 mm**, width +/- uncertainty reported. Bypass planned for
  **max(width + U, 200 mm)** on the clear side with the nearer edge; ahead
  pinged before the out and past legs. **Sonar fault policy**: no ECHO = a
  fault, never open space; 5 in a row stops the run. Retrace not built.

## 7. Assumptions
- **A4**: LSM303D, not LSM303DLHC; calibration at standstill required
  (zero-g offset about +/-60 mg ~ 3.4 deg); method changed (6.4 above).
- **A5**: bench result: works at 3.3 V; width error target "<= 20 mm"
  becomes "report width +/- U; the real width must be inside it" (S5-O01).
- **A8**: humps 10-30 mm -> lower practice ramp, measured height.
- **A9**: D_max 150 mm kept (bypass "past" leg).
- **A12**: drop the GP28 scaling and V_warn / V_crit; keep "approved
  power": power bank on VIN, tested (bringup_results.md).
- **A13**: sensor-to-axle distance and robot size are still guesses in the
  code (`TESTING_GUIDE.md` section 2): measure and quote them.

## 8. AI-supported design evidence
- **Change** the sentence that calls the IMU "GY-511 (LSM303DLHC)".
- **Add** a section on AI use during implementation: the code was written
  with an AI assistant; an independent review (1 Nov,
  `docs/reviews/2026-11-01_stage_test_review.txt`) found a numerical bug
  (sqrt gave 17,700 instead of 8,185 for IMU values) and a missing sonar
  failure policy, both fixed and documented in `CONTEXT.md`. Earlier AI
  errors caught by the team: the IR module's LED meaning during bring-up,
  and an assumption about what the Week 10 check needs.

## 9. Design consistency table
Update the state names in the right-hand column to section 5/6 above,
and mark each requirement built / tested / not built / not tested.

## Testing (new section or appendix)
- Plan: `testing/PLAN.md` (essentials only, graded items folded into the
  stage tests). Dropped tests are listed there; report them as "not
  tested", never as passed.
- Results: `testing/*/TEST_LOG.md` and the logs under `testing/*/logs`
  and `bringup/docs/logs`.

## Still to decide (team / instructor)
- NFR6 trip distance: 175 mm in the code since 20 Nov (250 until 15 Nov, then 190) vs at least 300 mm in Week 6: change NFR6 to 175 mm (Tribunal 1 minimum: 150 mm).
- FR10 / NFR5b heartbeat: change to 1 s, or count the 1 s telemetry.
- Whether changing the template's `mqtt_utk.c` / `lwipopts.h` is allowed.
- The mapping message format and "all moves at the junction" (instructor).
