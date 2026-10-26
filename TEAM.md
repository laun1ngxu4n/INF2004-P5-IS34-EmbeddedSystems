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
| 4 | B4 Muhammad Haziq: IMU (humps, collisions), Safety / Fault Monitor | | | `app_program/subsystem4/` | `testing/subsystem4/` |
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
| I2C1 | S4 | imu_task | GP26, GP27 (LSM303D at 0x1D, 400 kHz) |
| SPI0 | - | | |
| SPI1 | - | | |
| ADC | - | | not used (GP28 battery sense removed from scope, section 7) |
| PWM slices 4, 5 | S2 | motion_task (normal writes) | GP8-GP11 motors. *Draft exception:* `safety_task` may call S2's `motor_safe_off()` directly, so the motors stop even if Motion hangs (NFR10) |
| PWM slice 3 | S1 | telemetry_task | GP22 buzzer |
| PWM slice 7 | S5 | obstacle_task | GP14 servo |
| PIO0 (one SM) | S1 | telemetry_task | GP18 NeoPixel |
| GPIO interrupts (one callback) | S5 | shared dispatcher, *draft* | GP0-2 IR, GP4/5/16/17 encoders, GP7 echo, GP21 STOP: one dispatcher hands each pin to its owner |
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
| IF-02 | all → all | shared words under `APP_LOCK` (`app_lock.h`) | `stop_reasons`, fault bits | every multi-writer read-modify-write uses the one lock helper | *draft* |
| IF-03 | S3 → S2 | latest-value slot + event flag | `line_cmd` | newest steering only; old steering is dropped (finding 07) | *draft* |
| IF-04 | S2 → S5 | Week 6 `completion_slot` | | `{epoch, id, status, stopped}` (kept instead of the v3 message) | *draft* |
| IF-05 | S1 (MQTT) → S4/S5 | STOP helper, then message buffer | | MQTT STOP sets the STOP bit through the same helper as GP21 before any queue (Version C) | *draft, needs instructor OK for `mqtt_utk.c`* |
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
override them.

| Date | Change | Design items affected |
|---|---|---|
| 26 Oct | **GP28 battery sense removed.** The solder jumper is not fitted and no battery voltage is measured. The robot runs from a USB power bank on VIN. | FR9: "critically low battery" is no longer a fault cause (the other FR9 causes stay). Safety task: no VBAT_SENSE input. INIT self-test: no power-supply check. A12: V_warn / V_crit thresholds dropped. Telemetry: no battery field. |

