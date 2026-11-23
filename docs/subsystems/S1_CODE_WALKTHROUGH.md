# Subsystem 1: a walk through the code

**For B1**, to answer questions about the code itself. This page goes through
the code function by function, with the reason behind each piece.
[S1_TELEMETRY.md](S1_TELEMETRY.md) covers the rest: what the subsystem is for,
the numbers, what was shown on the robot and what was tried. Read that page
first. The code is as on `main`, 23 Nov 2026.

## 1. The picture to keep in mind

```
OUT:  any task ──telemetry_event("text")──┐
      telemetry_task, every 1 s / 0.5 s ──┼──> outbox (8 slots) ──> app_mqtt_tx() ──> broker ──> panel
                                          │       pulled by the kernel's WiFi task every 100 ms
IN:   panel ──> broker ──> WiFi task ──> app_mqtt_rx() ──> STOP / START / mapping
```

**The one rule behind the design:** our code never calls a WiFi, lwIP or
MQTT function. lwIP is not safe to call from more than one task, so only
the kernel's WiFi task (priority 4) touches it (the template's rule,
[TEAM.md](../team/TEAM.md)). It calls two **hooks** in our code:
- `app_mqtt_tx()`, to take one message to send;
- `app_mqtt_rx()`, to hand over each command that arrives.

Both hooks only copy data and never wait, because they run inside the
kernel's task.

## 2. The files

| File | What is in it |
|---|---|
| [`telemetry.c`](../../app_program/subsystem1/telemetry.c) | the task, the outbox, the two hooks, the commands, the buzzer, the log over WiFi |
| [`wire.c`](../../app_program/subsystem1/wire.c) / [`wire.h`](../../app_program/subsystem1/wire.h) | packing the binary messages (Change Requirement 1) |
| [`subsystem1.c`](../../app_program/subsystem1/subsystem1.c) | at start-up: creates the task and its 100 ms timer, adds its row to the timing table |
| [`subsystem1.h`](../../app_program/subsystem1/subsystem1.h) | the periods (`S1_STATUS_TICKS`, `S1_SENSOR_TICKS`, `S1_REPORT_TICKS`) |
| [`telemetry.h`](../../app_program/subsystem1/telemetry.h) | the topic names, `TELE_TEXT_MAX` (160 bytes) |
| `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` | the MQTT client in the kernel's WiFi task (template code; our changes are marked `P5-IS34`) |

## 3. `telemetry.c`, function by function

### `telemetry_task()`: the task

- It sleeps (`tk_slp_tsk`) until a cyclic handler (a kernel timer)
  wakes it every 100 ms.
- `tk_can_wup()` returns any wake-ups that piled up while it was still
  busy. They are counted as overruns in the timing table
  (`app_timing_wake()`).
- Each wake it counts ticks and does each job when its count is reached:

| Job | Every |
|---|---|
| `buzzer_tick()` | tick (100 ms) |
| Mission ping (IF-01, for the Stage 0 test) | 10 ticks (1 s) |
| `check_link()` then `status_line()` | 10 ticks (1 s) |
| `sensor_line()` | 5 ticks (0.5 s) |
| `app_timing_report()` | 600 ticks (1 min); 100 (10 s) in test images |

- **Why one task with counters:** one timer and one task do every job,
  instead of a task for each.
- **Why the lowest priority (12):** reporting must never delay driving.
  When the WiFi is slow, only this task waits.

### `outbox_put()`: the 8-slot queue

- It is a ring buffer with two counters that only ever count up:
  `g_ob_w` (written) and `g_ob_r` (read).
  - The slot is `counter % OUTBOX_N`.
  - The queue is full when `g_ob_w - g_ob_r >= 8`. Unsigned subtraction
    stays right even when the counters wrap round.
- When it is full, it **drops the oldest** (`g_ob_r++`) and counts it
  in `g_out_dropped`.
- **Why drop instead of wait:** the caller may be Mission or another
  task that drives the robot. It must never wait because the WiFi is
  slow. The newest status is the most useful one anyway.
- **Why under `APP_LOCK`:**
  - any task writes into the queue, and the WiFi task reads from it;
  - `APP_LOCK` turns interrupts off for the few microseconds of the copy;
  - on one core that is enough, and unlike a mutex it can never make
    anyone wait.

### `telemetry_event(text)`: how every subsystem reports

- Every subsystem reports its results through this one call.
- It prints `[EVT] text` on the console, puts the text in the outbox
  (`outbox_text()`, at most 159 bytes) and sets the beep flag.
- **Why events stay text:** they are sentences for the operator to read,
  not values for a program.

### The buzzer: `buzzer_init()` and `buzzer_tick()`

- GP22 is PWM slice 3.
  - The divider takes the 125 MHz clock to 1 MHz.
  - A wrap of 500 gives 2 kHz.
  - Half duty (`cc` = 250) makes the sound; `cc` = 0 is silent.
- A beep is on for one 100 ms tick, then off.
- **Why the beep is played here:** the task that reported only sets a
  flag, so it never waits for a beep. Events in the same 100 ms give one
  beep.

### `app_mqtt_tx()`: the WiFi task takes a message (every 100 ms)

1. It stores the time in `g_link_us`. The WiFi task only calls this hook
   while it is connected, so this time proves the link is alive.
2. It **takes turns** between a log chunk and an outbox message
   (`gb_log_turn`), so a long log never holds up an event.
3. It copies one message out under the lock: the topic name from the
   message's 2-bit topic index, then the bytes.
4. It returns the length; 0 means nothing to send.

### `app_mqtt_rx()`: a command arrives

- The topic must start with `is34/cmd/`. Then:

| Rest of the topic | What happens |
|---|---|
| `map` | `on_map_command()` |
| `start` | sets Mission's START flag. Mission decides, and only starts if it is IDLE |
| `stop` | **`app_stop_request(STOP_MQTT)` first.** It turns the motors off inside that call. Only then is Mission's STOP flag set |
| anything else | ignored |

- **Why STOP comes first:** nothing can delay it. The motors are off
  before Mission even hears about it.

### `on_map_command()`: the mapping

1. **Which kind is it?**
   - If byte 0 is 1 (the format version), it is the panel's 4-byte binary
     mapping. `wire_map_unpack()` turns it into 4 letters, which are
     written as text such as `A=L,F=R,Q=F,Z=U`.
   - Otherwise it is text typed by hand. A typed character is never the
     byte value 1, which is how the two are told apart.
2. **The check:** `barcode_set_mapping()` in Subsystem 3. Exactly 4
   pairs; each letter A-Z or 0-9; each letter and each move used once.
   - **Why S3 checks it:** S3 owns the mapping and its decoder uses it.
     S1 only carries it.
3. **The answer:** an acknowledgement is **always** sent on
   `is34/ack/map`, with the mapping now in use. If the mapping was
   refused, the reason goes out as an event and the old mapping is kept.

### `status_line()` and `sensor_line()`

- They read every subsystem through its public getters
  (`motion_get_status()`, `line_state()`, `imu_ramp_peak_mm10()`,
  `obstacle_front_mm()` and others), pack them with `wire.c` and put them
  in the outbox.
- S1 measures nothing itself. It reports what the others measure.

### `check_link()`: the WiFi-loss warning

- If the WiFi task has not asked for a message for 1 s
  (`LINK_LOST_US`; it normally asks every 100 ms), it sets
  `FAULT_WIFI_LOSS`. Otherwise it clears it.
- It is only a **warning**: the robot keeps driving, because nothing it
  needs to drive comes from the laptop.

### The log over WiFi: `tm_console_mirror()` and `log_take()`

- The console driver calls `tm_console_mirror()` for every byte it
  prints. The bytes go into an 8 KB ring (`\r` is left out).
- `log_take()` sends a chunk as `L<seq> <dropped>|text`:
  - a gap in `seq` means messages lost over WiFi;
  - `dropped` counts bytes lost because the ring was full.
- **Why 8192:** a power of 2, so `& (LOG_RING - 1)` replaces `%`.
- **Why the log goes over WiFi:** tests can run on the power bank with
  no USB cable.

## 4. `wire.c`: the binary messages

| Function | What it does |
|---|---|
| `put_start()` | zeroes the buffer, puts the version (1) in byte 0, starts the fields at bit 8 |
| `put_bits()` | writes a value lowest bit first; the message after byte 0 is one little-endian number |
| `put_unsigned()`, `put_signed()` | clamp the value to its field's range, then write it (signed fields as two's complement) |
| `wire_letter_code()`, `wire_code_letter()` | a letter as 6 bits: 0 none, 1-26 A-Z, 27-36 0-9, 63 any other character |
| `wire_status_pack()`, `wire_sensors_pack()`, `wire_ack_pack()` | build the three messages the robot sends |
| `wire_map_unpack()` | reads the 4-byte mapping; false for a wrong length, version or letter |

- **Why clamp, not cut off:**
  - cutting 300 down to 8 bits gives 44, a wrong value that looks real;
  - clamping gives 255, the field's maximum, which is plainly "this much
    or more".
- **Why not send a C structure as it is:** the compiler picks the bit
  order and padding of a structure, and the panel is Python. Explicit
  shifts give the same layout on both ends.
- **Why `wire.c` makes no kernel calls:** it is compiled on the laptop
  too, by `tools/panel/wire_golden.c`, and checked against the panel's
  decoder (`python tools/panel/test_wire.py`, 15 of 15 pass).

### The status message, field by field (15 bytes)

| Field | Bits | Why that many |
|---|---|---|
| mission state | 2 | 4 states (IDLE, CAL, RUN, MANOEUVRE) |
| line state | 2 | 4 states (ON, SEARCH, LOST, FAULT) |
| stop reasons | 3 | 3 reasons (button, laptop, fault) |
| faults | 8 | the 8 fault bits |
| speed left, speed right | 12 + 12, signed | ±2047 mm/s |
| distance | 20, signed | ±524 m |
| junctions | 8 | counts round after 255 |
| last letter | 6 | the 6-bit letter code |
| mapping L, R, F, U | 4 × 6 | one letter code each |
| hump peak | 12 | 0.1 mm steps, up to 409.5 mm |

Total: 109 bits, so 14 bytes, plus the version byte = **15**. In the
same way:
- the sensors message is 53 bits + the version = **8** bytes;
- the acknowledgement is 26 bits + the version = **5**;
- the mapping is 24 bits + the version = **4**.

All fields are listed in [MQTT_WIRE_FORMAT.md](../team/MQTT_WIRE_FORMAT.md).

## 5. `mqtt_utk.c`: the kernel's WiFi task

| Part | What it does |
|---|---|
| `mqtt_utk_start()` | connects to the broker (IP and port from `config/wifi_secrets.h`, never committed); keep-alive 60 s |
| `connection_cb()` | once connected: subscribes to `is34/cmd/#` at QoS 0 and starts the pump |
| `is34_pump_timeout()` | every 100 ms: one message from `app_mqtt_tx()`, published. A message the library refuses because its buffer is still full is kept and tried again, for up to 3 s (fixed 7 Nov) |
| `incoming_data_cb()` | collects a command's bytes **unchanged** (the 9 Nov fix for the binary mapping), then calls `app_mqtt_rx()` |
| `connect_watchdog()`, `retry_timeout()` | one connect attempt at a time; no answer in 15 s = start again; a failed attempt is retried after 3 s (fixed 7 Nov) |
| `heartbeat_timeout()` | a heartbeat every 5 s on the template's test topic (only the first is printed) |

- **What the pump rate means:** one message per 100 ms is up to 10 a
  second.
  - Status and sensors use 3 of them.
  - The log and the outbox take turns, so events always get at least
    5 a second.
- **Why `app_mqtt_rx` and `app_mqtt_tx` are declared `weak` here:** the
  template still builds without our code. Ours replace them when linked.

## 6. Who shares what (concurrency)

| Data | Written by | Read by | Protection |
|---|---|---|---|
| the outbox and its counters | any task (`telemetry_event()`), the telemetry task | the WiFi task (`app_mqtt_tx()`) | `APP_LOCK` |
| the log ring and its counters | the console driver, any context | the WiFi task (`log_take()`) | `APP_LOCK` |
| `gb_beep_req` | any task | the telemetry task | `APP_LOCK` |
| `g_link_us` | the WiFi task | the telemetry task | none needed: one aligned 32-bit store or load, which the Cortex-M0+ does in one step (`volatile`) |
| `g_start_count` | the WiFi task (each START command) | the test images, waiting for START from the panel (`app_test.c`) | the same, `volatile` |

## 7. Code questions you may get

The general questions (why binary, WiFi drops, how fast STOP is) are in
[S1_TELEMETRY.md section 10](S1_TELEMETRY.md#10-questions-you-may-get).

- **Why `APP_LOCK` and not a semaphore or mutex?** The hooks run in the
  kernel's task and must never block. A mutex can make a task wait;
  turning interrupts off for a few-microsecond copy cannot. It works from
  any context, and on one core it is a complete lock.
- **What happens when the outbox is full?** The oldest message is
  dropped and counted. Nobody waits.
- **Why does your code not call `mqtt_publish()` itself?** lwIP is only
  safe in the kernel's WiFi task. We hand it messages through
  `app_mqtt_tx()`.
- **How is WiFi loss detected?** No call from the WiFi task for 1 s
  (`check_link()`). Separately, the panel shows OFFLINE after 3 s with no
  status.
- **Why QoS 0?**
  - The status repeats every second anyway.
  - The STOP button and the lift stop do not need the WiFi.
  - The broker does not keep QoS 0 messages for an offline client, so a
    START sent while the robot was offline is never delivered later.
- **What is byte 0 for?** The format version.
  - The panel drops a message with the wrong version or length.
  - The robot uses it to tell a binary mapping from typed text.
- **Can the mapping change during a run?** Yes, at any time (FR2a).
  - Mission looks up the move at the junction, with the mapping in use
    then (`mission.c`).
  - If a letter was already read and the new mapping no longer has it,
    the robot goes straight on.
- **How does the packing work?** `put_bits()` sets one bit at a time:
  - byte `pos >> 3`;
  - bit `pos & 7`;
  - then `pos++`.
- **How do you know the two ends agree?** The robot's own `wire.c` is
  compiled on the laptop and checked against the panel's decoder with
  shared test messages: 15 of 15 pass. A deliberately wrong field width
  makes it fail.
- **What are the overruns in the timing table?** Wake-ups that came
  while the task was still busy (`tk_can_wup()`). Telemetry had 0 in the
  9 Nov log (about 7 minutes,
  `testing/subsystem2/logs/2026-11-09_stage1_part2_held_wheel_robot.txt`).
  The other tasks' few overruns all came once, when the WiFi radio
  starts 0.6 s after reset.
