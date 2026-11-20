# Team and subsystems

**P5-IS34 (26 Oct): DRAFT for the team to confirm.** Filled in from the
Week 6 design, the Tribunal 1 Version B/C decisions and the bring-up
results (`bringup/docs/bringup_results.md`). Rows marked *draft* still need
the team's agreement; student IDs and GitHub usernames are left for each
member to fill in.

Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | B1 Muhammad Ikhwan: WiFi/MQTT telemetry, remote commands, NeoPixel + buzzer | | mhdikhwan | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | B2 Ng Chern Loong Branson: motors, encoders, PID, distance moves, spin turns | | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | B3 Lau Ning Xuan: IR line following, Code 39 barcode, junctions | | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | B4 Muhammad Haziq: IMU (humps, lift stop; collisions removed 13 Nov), Safety / Fault Monitor | | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | B5 Muhammad Irfan: ultrasonic + servo, obstacle bypass, Mission Controller, integration | | | `app_program/subsystem5/` | `testing/subsystem5/` |

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | S3 | IR left D0 (Grove 1 white). Never build `CRASH_REPORT=1` with it plugged in |
| GP1 | S3 | IR centre D0 (Grove 1 yellow). Same `CRASH_REPORT=1` warning |
| GP2 | S3 | IR barcode D0 (Grove 2 white) |
| GP3 | - | unused (Grove 2 yellow, taped) |
| GP4 | S2 | left encoder A (Grove 3 white) |
| GP5 | S2 | left encoder B (Grove 3 yellow). Left counts DOWN for forward |
| GP6 | S5 | ultrasonic TRIG (Grove 5 white, jumper wire) |
| GP7 | S5 | ultrasonic ECHO (Grove 7 white, jumper wire), HC-SR04 at 3.3 V |
| GP8 | S2 | left motor M1A, PWM slice 4 (A = PWM drives forward after the 26 Oct M1 wire swap) |
| GP9 | S2 | left motor M1B, PWM slice 4 |
| GP10 | S2 | right motor M2A, PWM slice 5 |
| GP11 | S2 | right motor M2B, PWM slice 5 |
| GP12 | - | unused (servo header 1) |
| GP13 | - | unused (servo header 2) |
| GP14 | S5 | servo PWM, slice 7 (servo header 3) |
| GP15 | - | unused (servo header 4) |
| GP16 | S2 | right encoder A (Grove 4 white) |
| GP17 | S2 | right encoder B (Grove 4 yellow) |
| GP18 | S1 | NeoPixel status LEDs (PIO) |
| GP19 | - | unused (the template's example LED is removed) |
| GP20 | S5 | START button (on board, active low) |
| GP21 | S5 | STOP button, held = RESET (on board, active low); one shared STOP helper (section 5) |
| GP22 | S1 | buzzer, PWM slice 3 |
| GP26 | S4 | IMU SDA, I2C1 (Grove 6 white; Grove 5 yellow is the same pin, taped) |
| GP27 | S4 | IMU SCL, I2C1 (Grove 6 yellow) |
| GP28 | - | **not used, removed from scope (26 Oct)**: no battery sense, see section 7; Grove 7 yellow taped |

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | - | | |
| I2C0 | - | | |
| I2C1 | S4 | imu_task (`imu_i2c.c`, polled) | GP26, GP27 (LSM303D at 0x1D, 400 kHz) |
| SPI0 | - | | |
| SPI1 | - | | |
| ADC | - | | not used (GP28 battery sense removed from scope, section 7) |
| PWM slices 4, 5 | S2 | motion_task (normal writes) | GP8-GP11 motors. *Draft exception:* `safety_task` may call S2's `motor_safe_off()` directly, so the motors stop even if Motion hangs (NFR10) |
| PWM slice 3 | S1 | telemetry_task | GP22 buzzer |
| PWM slice 7 | S5 | obstacle_task | GP14 servo |
| PIO0 (one SM) | S1 | telemetry_task | GP18 NeoPixel |
| GPIO interrupts (one callback) | shared | `app_program/app_gpio.c` (Stage 1) | GP0-2 IR, GP4/5/16/17 encoders, GP7 echo, GP21 STOP: each owner calls `app_gpio_irq_add()` in its `_init`; the dispatcher hands each pin to its owner |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students *can* do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
- **Send or receive data over the network** by adding code that *runs
  inside the service task*. The MQTT client is in
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. It publishes a heartbeat
  today. A "cloud / reporting" subsystem would add a kernel object
  (e.g. a message buffer) that application tasks fill. The MQTT code then
  reads it with `TMO_POL` (never blocking) and publishes it.
  Because this changes shared library code, **agree it with the team and
  the instructor first**, and record it as an interface in section 5.
  It needs the `NET=lwip MQTT=1` build (README section 3).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). Each subsystem gets its own band by default;
change it only by team agreement. Write down every task you create.

**P5-IS34, draft:** single core (`SMP=0`, team decision 26 Oct), so the core
column is not used. Instead of one band per subsystem, the Week 6 task order
is kept and shifted below the kernel (Week 6 used 2-10). The values live in
`app_program/app.h`. Notification is merged into Telemetry (8 tasks).

| Subsystem | Tasks (name – priority – period) | Week 6 priority |
|---|---|---|
| 4 | safety_task – 5 – 20 ms | 2 |
| 2 | motion_task – 6 – 10 ms | 3 |
| 5 | mission_task – 7 – event, 100 ms timeout | 4 |
| 3 | line_task – 8 – 20 ms | 5 |
| 4 | imu_task – 9 – 10 ms | 6 |
| 3 | barcode_task – 10 – event, 10 ms timeout | 7 |
| 5 | obstacle_task – 11 – 60 ms | 8 |
| 1 | telemetry_task – 12 – 100 ms, timing table every 10 s | 9 (Notification was 10) |

Open with the instructor: the kernel's WiFi task (priority 4) runs above
Safety and Motion, while the design keeps WiFi lowest. Stage 0 measures
its effect in the timing table.

Measured on the robot (Stage 0, 26 Oct):
- Bringing the WiFi radio up stops every task for about 120 ms at about
  0.6 s after reset (once). Never re-initialise the radio while driving.
- Printing the timing table over USB every second delayed every task by
  about 12 ms now and then (the kernel's USB console task is priority 3),
  each time with lost output. Printing every 10 s removed it. **Team rule:
  keep USB console output small while the robot drives** (a few short
  lines per second at most); real telemetry goes over MQTT.
- All tasks still wobble about +/-2.5 ms around their period with little
  printing; not caused by printing, cause not yet known (WiFi or USB
  background work). No overruns.
- A kernel timeout ends on the next 10 ms tick: a "10 ms" timeout lasts
  about 20 ms. Use a cyclic handler where exact timing matters.

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | S1 → S5 (later also buttons, Safety, MQTT) | event flag (owned by S5) | `flg_mission` | bit `MSN_FLG_PING`, Stage 0 only; later START/STOP/finish bits | *draft* |
| IF-02 | all → all | STOP helper `app_stop.[ch]` (Stage 1) + `APP_LOCK` | `app_stop_request(reason)` | motors off at once from any context; Motion cancels its command while any reason is set; owner of a reason clears it | *draft* |
| IF-03 | S3 → S2 | latest value, read by Motion every 10 ms (Stage 2) | `line_steer_mm_s()` | steering mm/s, + = left; newest only (finding 07) | *draft* |
| IF-04 | S2 → S5 | command id + result slot (Stage 1) | `motion_drive/turn/follow()` return an id; `motion_get_status()` | `{id, result DONE/STOPPED/TIMEOUT/BUSY, counts, speeds}` | *draft* |
| IF-05 | S1 (MQTT) ↔ all | hooks `app_mqtt_rx()` / `app_mqtt_tx()` called by `mqtt_utk.c` (Stage 5) | `is34/cmd/map|start|stop`, `is34/event`, `is34/telemetry`, `is34/ack/map` | MQTT STOP calls `app_stop_request(STOP_MQTT)` first; events from every subsystem go through `telemetry_event()` | *draft, needs instructor OK for the `mqtt_utk.c` / `lwipopts.h` change* |
| IF-06 | | | | | |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The numbers below are the example's usage.

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | 1 | 1 | 2 | 2 | 2 |
| Semaphore | 16 | | | | | |
| Event flag | 16 | | | | | 1 |
| Mailbox | 12 | | | | | |
| Mutex | 12 | | | | | |
| Message buffer | 12 | | | | | |
| Fixed memory pool | 12 | | | | | |
| Variable memory pool | 12 | | | | | |
| Cyclic handler | 12 | 1 | 1 | 1 | 2 | 1 |
| Alarm handler | 12 | | | | | |

(Stage 0 usage. The kernel's USB and WiFi service tasks also use a few of these.)

## 7. Scope changes after the Week 6 design

The submitted design documents are left as submitted; these changes
override them. Section-by-section edits for the next report:
`docs/week6_report_changes.md`.

| Date | Change | Design items affected |
|---|---|---|
| 26 Oct | **GP28 battery sense removed.** The solder jumper is not fitted and no battery voltage is measured. The robot runs from a USB power bank on VIN. | FR9: "critically low battery" is no longer a fault cause (the other FR9 causes stay). Safety task: no VBAT_SENSE input. INIT self-test: no power-supply check. A12: V_warn / V_crit thresholds dropped. Telemetry: no battery field. |
| 26 Oct | **Barcode IR sensor replaced.** Reads slightly better than the old one (team observation, not yet re-measured). | Re-run the bring-up IR barcode pass, including the thin "A small" barcode, before writing the decoder. |
| 27 Oct | **Ramp: only the lower practice ramp matters.** The demo ramp is about the lower practice ramp's height, slightly lower or higher; ignore the higher ramp. Goal: estimate the height reached from the IMU, by sensing and processing. | FR4 and the hump assumption (10-30 mm, tests at 10/20/30 mm): re-centre the range and the tests on the measured height of the lower ramp. Measure that ramp's height and length first. |
| 27 Oct | **IMU is the LSM303D** (found at 0x1D in bring-up), not the GY-511 / LSM303DLHC the design names. Still accelerometer + magnetometer, no gyroscope. | IMU register code follows the LSM303D datasheet (one I2C address for both sensors). Accel range **+/-4 g**, not +/-2 g: +/-2 g would clip the 2 g collision threshold (A-safety). Stationary offset calibration stays mandatory: the datasheet zero-g offset (typ. +/-60 mg, about 3.4 deg of tilt) is as big as the +/-5 mm height error allows (about 3.6 deg at L = 80 mm). Do not use the built-in high-pass filter for pitch: it removes gravity. |
| 26 Oct | **Barcode requirements from the instructor**, see 7.1. | FR2, FR2a, FR3, FR11 |
| 26 Oct | **Obstacle requirements from the instructor**, see 7.2. | FR5, FR6, NFR3, A5 |

### 7.1 Barcode (S3 decoder, S1 MQTT, S5 Mission, S2 Motion)

Instructor brief: decode Code 39 A-Z, readable in both directions, store
the character, carry on to the cross-junction, then do the mapped move
(forward, U-turn, left, right). The 4-character mapping is sent from the
PC over MQTT just before the demo and must not be hard-coded.

| # | Requirement | Changes from Week 6 | Owner |
|---|---|---|---|
| B1 | Decode every Code 39 character, at least A-Z, using the full pattern table (9 elements, 3 wide). | Week 6 only defined A-D (+ Z reserved). | S3 |
| B2 | Decode in both directions: if the start/stop `*` reads reversed, reverse the elements and decode again. | New. | S3 |
| B3 | Measure bar widths in **encoder counts** (0.080 mm/count), not time, so a speed change over the barcode does not change the reading. | New. Time-based widths only work at constant speed. | S3 (reads S2 counts) |
| B4 | Drive slower while line following, at a speed chosen by test so the thinnest bar is still read reliably. | New. Week 6 had no barcode speed limit. | S2, S5 |
| B5 | Store the decoded character. Keep the latest unconsumed one (FR11). | Same as Week 6. | S5 |
| B6 | **All four moves, U-turn included, happen at the next cross-junction.** | Week 6 did the U-turn straight after the barcode: changed. | S5, S2 |
| B7 | No mapping is hard-coded. Until a mapping arrives over MQTT, a decoded character is only reported. A character with no mapping is reported and ignored: the robot carries on (team, 20 Nov; until then it stopped at the junction with "no mapping"). | Week 6 had default A-D: removed. | S3, S5 |
| B8 | The mapping arrives over MQTT at run time (no restart) and is acknowledged with what was stored. | FR2a kept, message format below is new. | S1, S3 |

Mapping message, **draft (confirm the format with the instructor)**:
- topic `is34/cmd/map`, payload four `char=move` pairs, e.g. `K=L,Q=R,X=F,M=U`
  (L left, R right, F forward, U U-turn).
- Rejected unless it has exactly 4 pairs, each character A-Z used once and
  each move used once. A rejected message keeps the old mapping.
- Ack on `is34/ack/map`: the stored mapping, or the reason it was rejected.

### 7.2 Obstacle (S5 Obstacle Scanner, S2 Motion)

Instructor brief: obstacle 10-20 cm wide. Profile it (no precise
measuring needed), estimate the width and **report it with its
uncertainty**, choose the best move and do it at a reasonable speed:
a consistently safe path, clear the obstacle, find the line again.

| # | Requirement | Changes from Week 6 | Owner |
|---|---|---|---|
| O1 | After every servo move, wait for the servo to settle (time set by test) before pinging. | Week 6 only estimated settle time for NFR3. | S5 |
| O2 | Take 3-5 pings per angle and use the median. | Week 6: 2 pings to confirm "clear". | S5 |
| O3 | Background subtraction: a ray belongs to the obstacle only if it is clearly shorter than the background (open space) at that angle. | New. | S5 |
| O4 | An edge needs 2 consecutive angles past the threshold, with hysteresis (separate on/off thresholds). | Week 6: one >100 mm range jump. | S5 |
| O5 | Scan from 2 robot positions (e.g. after reversing a short distance) and combine the two widths. | Week 6 measured from one position. | S5, S2 |
| O6 | Report `width_mm`, `uncertainty_mm` and the two edge angles in telemetry. Uncertainty from the angle step and beam width at that range, plus the spread between the two scans. | Week 6 reported the profile but no uncertainty. | S5, S1 |
| O7 | Plan the bypass with `max(measured width + uncertainty, 200 mm)` plus the 50 mm margins each side. | New: never plan for less than the largest expected obstacle. | S5 |
| O8 | Bypass, clear the obstacle and reacquire the line as Week 6 (5 legs, rejoin within 500 mm, no contact). | Same. | S5, S2, S3 |
| O9 | NFR3 scan times (1.2 s coarse, 2.0 s fine) to be re-set after O1, O2 and O5 are measured. | O1, O2 and O5 make the scan longer. | S5 |
