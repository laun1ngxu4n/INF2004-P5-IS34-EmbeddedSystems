# P5-IS34 RTOS Stage 0: build, flash and test

Stage 0 is the first step of the build plan: micro T-Kernel boots on the
Pico W **on one core** and runs **the 8 design tasks at their real priorities
and periods**. The tasks do no robot work yet. Each one is timed, and
Telemetry prints a timing table every 10 seconds. Later stages fill in the task
bodies (Motion, Line, STOP/safe-off, telemetry) without changing this frame.

Nothing is driven: no motors, servo, LEDs or buzzer. Laptop USB power is
enough and nothing needs to be wired or unplugged. **Do not build with
`CRASH_REPORT=1` while the IR sensors are plugged in**: it uses GP0/GP1.

## What is in it

| Task | Owner (folder) | Priority | Period |
|---|---|---|---|
| safety_task | B4 (`app_program/subsystem4/`) | 5 | 20 ms |
| motion_task | B2 (`subsystem2/`) | 6 | 10 ms |
| mission_task | B5 (`subsystem5/`) | 7 | event, 100 ms timeout |
| line_task | B3 (`subsystem3/`) | 8 | 20 ms |
| imu_task | B4 (`subsystem4/`) | 9 | 10 ms |
| barcode_task | B3 (`subsystem3/`) | 10 | event, 10 ms timeout |
| obstacle_task | B5 (`subsystem5/`) | 11 | 60 ms |
| telemetry_task | B1 (`subsystem1/`) | 12 | 100 ms, timing table every 10 s |

Shared pieces, used by everyone from now on:
- `app_program/app.h`: the priority and period table.
- `app_program/app_lock.h`: `APP_LOCK` / `APP_UNLOCK`, the one lock for
  shared data (interrupts off on single core; refuses to build with `SMP=1`).
- `app_program/app_timing.[ch]`: per-task timing (period, overruns, run time).
- `app_program/app_main.c`: `app_main_create_wake_cyclic()` wakes a task every N ms.
- `TEAM.md`: pins, peripheral owners, priorities, interfaces (draft for the
  team to confirm).
- The bring-up tests and results are unchanged in `bringup/`.

## 1. Build (or use the prebuilt images)

Prebuilt images, built from the commit that added this file (SDK 2.2.0, arm-none-eabi-gcc
13.2.1, 0 warnings):
- `mtk3pico_smp0_usb_cdc_wifi.uf2`: normal image (timing table every 10 s)
- `mtk3pico_smp0_usb_cdc_wifi_test.uf2`: test image (runs the Stage 0 tests)

To build yourself (WSL Ubuntu on Windows), follow `README.md` sections 1-2
with **Pico SDK 2.2.0** (2.1.1 does not link: its WiFi driver needs a
`sleep_ms` the template only provides for 2.2.0), then:

```sh
cd build_make
make -j8            # normal image  -> mtk3pico_smp0_usb_cdc_wifi.uf2
make TEST=1 -j8     # test image    -> mtk3pico_smp0_usb_cdc_wifi_test.uf2
```

`smp0` in the name means single core. The makefile default is now `SMP=0`.

## 2. Test A: the test image (automatic, about 20 s)

1. Start logging in the terminal (Tera Term: File > Log).
2. Hold BOOTSEL, plug in USB, drag `mtk3pico_smp0_usb_cdc_wifi_test.uf2`
   onto RPI-RP2. The drive disappears and the Pico restarts.
3. Open the Pico's COM port at 115200 baud. If you missed the start,
   unplug and replug USB (no BOOTSEL) with the terminal already open.
4. Wait for `[TEST] === test run end ===` (about 25 s: 3 s for start-up, then each task is
   watched for 2 s, then 3 s for INT-01).

Expected:
```
P5-IS34 test image, hands-on TEST_STAGE=1. Kernel tick 10 ms
all subsystems started
[TEST] === test run start ===
[TEST] telemetry: 20 wakes in 2000 ms (expected 20), longest interval ... overruns 0
[TEST] S1-U01 PASS  wake rate
[TEST] S1-U02 PASS  no overrun, longest interval within limit
...
[TEST] INT-01 PASS  IF-01 ping from Telemetry wakes Mission once a second
[TEST] subsystem5 SUMMARY 5 passed, 0 failed
[TEST] === test run end ===
```
**Pass:** 17 PASS lines and every SUMMARY says `0 failed`.
The timing table prints every 10 s in between; that is normal.

## 3. Test B: the normal image (manual, 60 s, INT-02)

1. Flash `mtk3pico_smp0_usb_cdc_wifi.uf2` the same way, logging on.
2. Leave it running for **at least 60 s** (6 tables).
3. Read the last `[tmg]` table:
```
[tmg] t=60123 ms | task pri period | wakes overruns | interval min-max ms (boot max @ s) | run max us (boot max)
[tmg] safety     5  20 |   3000   5 | 19.99-20.00 (126.91 @ 1.2) | 2 (19)
[tmg] motion     6  10 |   6000  12 | 7.73-12.26 (126.96 @ 1.2) | 2 (13)
...
```
**Pass:** all 8 tasks listed; the **overruns** column stops growing after
start-up (it counts since boot); the window intervals stay close to the
period (event tasks: barcode about 20 ms, mission up to about 110 ms,
shorter when a ping arrives).

First run on the robot (26 Oct, earlier image) showed three things to
expect, each explained in section 4:
- one system-wide pause of about 117 ms near start-up (every task's boot
  max is about 127 ms, and the 10/20/60 ms tasks show 12/5/1 overruns
  from it). The `@ s` value now shows when it happened.
- the 10 ms tasks wobble by about +/-2.3 ms (7.7-12.3 ms), the 20 ms and
  60 ms tasks do not.
- the barcode task's "10 ms" timeout gives about 20 ms (the 10 ms kernel
  tick rounds a timeout up by one tick).

## 4. What to send back / record

- Both logs. Save them as `testing/integration/logs/<date>_stage0_test.txt`
  and `<date>_stage0_60s.txt`.
- Add a row to each `testing/*/TEST_LOG.md` *Results* table: date, test
  ID, commit (`git rev-parse --short HEAD`), tester, PASS/FAIL, log file.

Useful numbers in the table for the team:
- **boot max @ s**: the ~120 ms pause at about 0.6 s is the WiFi radio
  starting up, harmless while the robot is still. If it is later, or the
  overruns column grows after start-up, something stalls the whole system.
- **boot max interval** of safety and motion: shows how far the kernel's
  USB (priority 3) and WiFi (priority 4) tasks delay the highest-priority
  robot tasks. This is the open question for the instructor.
- **telemetry run max**: the cost of printing the table (lowest priority).

## 5. If something goes wrong

| What you see | Likely cause | What to do |
|---|---|---|
| No COM port | the image did not start, or a charge-only cable | re-flash; try another cable |
| Only the banner, then silence | a crash (USB output stops with it) | send me the log; a `CRASH_REPORT=1` build shows the crash on GP0/GP1, but **unplug the IR sensors first** |
| `ERROR ... : -<number>` lines | a kernel call failed | send me the log |
| A `FAIL` line | that task missed its timing | send me the log; the line before it prints the measured numbers |
| `overruns` above 0 | a task was still running when its next period came | send me the log |
