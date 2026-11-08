# P5-IS34 RTOS Stage 1: motion + STOP

Stage 1 makes the robot move under the RTOS. On top of Stage 0 it adds:

- **Motors and encoders** (`app_program/subsystem2/motor.c`): PWM 10 kHz
  on GP8-GP11, encoders on GP4/5 and GP16/17 (+ = forward on both wheels).
- **Motion task** (`subsystem2/motion.c`, every 10 ms): a speed
  controller per wheel, a ramp up and slow-down, and two commands:
  `motion_drive(speed, mm)` and `motion_turn(deg)` (spin on the spot,
  + = left).
- **STOP** (`app_program/app_stop.[ch]`, GP21 in `subsystem5`): pressing
  STOP turns the motors off inside the button interrupt, before any task
  runs. Motion cancels its command while STOP is set.
- **One GPIO interrupt dispatcher** (`app_program/app_gpio.[ch]`), shared by
  everyone who needs pin interrupts (encoders and STOP now; IR and echo
  later).

The normal image does not drive: there is no mission yet. Only the test
image drives, and only after you press START.

## Images

| File | What it does |
|---|---|
| `mtk3pico_smp0_usb_cdc_wifi_test_s1.uf2` (`make TEST=1 TEST_STAGE=1`) | Stage 0 timing tests (about 25 s), then the **drive tests** below |
| `mtk3pico_smp0_usb_cdc_wifi.uf2` | Normal image: timing table every 10 s, nothing moves |

Build them yourself as in `RTOS_STAGE0.md` section 1 (`make` and `make TEST=1`).

## Before you start

- **Laptop USB only, no power bank** (never both together: not checked).
  The motors run on USB 5 V (the bring-up motor tests did); a long cable,
  the laptop follows the robot. See `TESTING_GUIDE.md` section 1.
- Turns need the **track width**: wheel centre to wheel centre, in mm.
  The code uses **130 mm, a guess**. Measure it with a ruler and send it
  to me. If you can't, S2-M02 below tells us the right value anyway.
- Clear floor: about 1.5 m in front of the robot.
- Tape on the floor to mark the robot's starting heading (along one side
  of the robot) for the turn test.

## Run the test image

1. Start logging in Tera Term, flash `..._test.uf2`, open the COM port.
2. Wait about 25 s for `[TEST] === test run end ===`. The 17 Stage 0
   lines should still all PASS.
3. `[DRIVE] === Stage 1 drive tests ===` appears. From here every move
   waits for **START (GP20)**, then starts 1 s later so you can take your
   hand away. **STOP (GP21) stops it at any time.**

### S2-M02 turns (3 rounds)

Each round: left 90, right 90, then 180. After each turn the console
prints e.g. `[S2-M02] round 1 left 90: DONE in 1450 ms, encoder angle 90.2 deg`.

- After each round, **write down the real angles** you see against the
  tape (a phone protractor app or a printed protractor is enough), then
  put the robot back on the mark and press START.
- Pass: 90 deg turns within +/- 10 deg, 180 within +/- 15 deg.
- If every turn is off by the same factor (e.g. all about 80 instead of
  90), the track width is off: send me the real angles and I fix it.

### S2-M03 slow speed

Drives 0.5 m at 150 mm/s (the starting barcode speed) and prints the
mean, min and max speed over the middle 300 mm. Pass: mean within 10 %.

### S2-M04 STOP (5 tries)

Each try drives at 200 mm/s for up to 1.5 m. **Press STOP while it
drives.** The console prints how long the wheels took to stop after the
press and how far they rolled. Release STOP, then press START for the next
try. Pass: 5 of 5 stop within 500 ms.

At the end: `[TEST] subsystem2 drive SUMMARY ...` and the number of
encoder errors (should be 0 or close to it).

## What to send back

- The whole log (save it as `testing/subsystem2/logs/<date>_stage1_test.txt`).
- The real angles you wrote down for the 9 turns.
- The track width if you measured it.

## If something goes wrong

| What you see | What to do |
|---|---|
| A wheel spins the wrong way, or the robot spins instead of driving straight | Press STOP. Send me the log: a motor or encoder wire is the other way round from bring-up |
| It drives, then stops early with `TIMEOUT` | A wheel was blocked or too weak (power bank off?); check VIN power |
| It does not move at all, result `TIMEOUT` | Too weak on USB (try the test on the power bank alone and watch it), or the motor switch on the board is off |
| Turns way over or under, all by the same amount | Track width: send the real angles |
| STOP does nothing | Unplug the power bank, send me the log |
