# Subsystem 2: Motion (motors, encoders, speed control)

**Owner:** B2. **Code:** [`app_program/subsystem2/`](../../app_program/subsystem2/)
(`motor.c`, `motion.c`). **Tests:** Stage 1 (`images/is34_test_stage1.uf2`).
The state on 21 Nov 2026. The whole robot: [README.md](README.md).

## 1. In one minute (what to say)

"Subsystem 2 makes the wheels do what the other subsystems ask. Each
wheel's encoder gives about 2,540 counts per turn, 0.08 mm per count.
Every 10 ms the Motion task works out each wheel's speed and sets its
motor power with a feed-forward plus PI controller.

It runs three kinds of command: drive a distance, spin on the spot by an
angle, and follow the line using the steering from subsystem 3. Every
command gets an ID and a result, so Mission knows when it has finished,
been stopped, or timed out.

A STOP turns the motors off in the interrupt itself, and Motion refuses
to move while any stop reason is set."

## 2. Why it exists

- **The course:** motors, encoders and closed-loop speed control (PID) are
  buddy 2's part. Every other subsystem moves the robot through it.
- **Requirements (Tribunal 2 list):**
  - FR1: follow the line at 120 mm/s, with subsystem 3;
  - FR3, FR6, FR8: the junction turns and the bypass legs;
  - FR9, FR14: stop, and stay stopped until START;
  - NFR1: Motion every 10 ms;
  - NFR8: turns within ±10° for a 90 and ±15° for a 180;
  - NFR10: the motor drive goes off inside the STOP interrupt.

## 3. What it owns, and what it measures

**Motors** (board outputs M1, M2; GP8-GP11):
- forward is pin A driven with PWM and pin B low; reverse is A low and B
  driven with PWM;
- PWM at 10 kHz (125 MHz / 12,500); power ("duty") in per mille, where
  1000 is full.

**Encoders** (quadrature, GP4/GP5 left, GP16/GP17 right):
- every edge of both signals is decoded (4 per step) in the GPIO
  interrupt, with a lookup table;
- about 2,540 counts per wheel turn on the 65 mm wheel: **0.080 mm per
  count**;
- the left encoder counts down for forward, and the code turns that
  round;
- impossible jumps are counted, and Safety warns when there are more than
  50 a second.

**It measures** each wheel's distance (counts) and speed (counts in the
last 40 ms, in mm/s), and the robot's travel: the odometer,
`motor_odometer()`, used by subsystems 3, 4 and 5.

Measured in bring-up, wheels lifted: 240, 325, 407, 573 and 798 mm/s at
30, 40, 50, 70 and 100 % power.

## 4. Task

`motion_task`: priority 6, just below Safety, every 10 ms (woken by a
cyclic handler).

## 5. How it works

### 5.1 Every 10 ms

1. Read both encoders. Each wheel's speed is the counts of the last 4
   periods (40 ms), turned into mm/s.
2. Take a new command if one was posted. It is a one-slot mailbox under
   the lock, and the newest command replaces the old one.
3. **If any stop reason is set:** motors off, and the command is
   cancelled with the result STOPPED.
4. Otherwise run the command: work out a target speed for each wheel,
   then the controller below sets the PWM.
5. Publish the status (command ID, result, speeds, counts) under the
   lock.

### 5.2 The wheel speed controller (one per wheel)

`power = feed-forward + P + I`, limited to ±1000 per mille.

- **Feed-forward:** 1.25 per mille per mm/s of target speed, plus 80 per
  mille in the direction of travel to overcome friction. These come from
  the bring-up speed-against-power measurements. They get the wheel close
  to the right speed at once.
- **P:** 1.0 per mille per mm/s of error now.
- **I:** 6.0 per mille per mm/s of error per second, built up and limited
  to ±400. It removes what is left: floor, load, battery.
- **A target of exactly 0** gives power 0 and resets the I part.

### 5.3 Drive a distance, spin by an angle

- **Speed profile:**
  - it speeds up at 400 mm/s²;
  - it never goes faster than it can stop in the distance left
    (√(2 × 400 × distance left));
  - it never goes under 50 mm/s near the end.
- **Done** when the mean of the two wheels has covered the distance. For
  a spin, that distance is the wheel arc: π × 123 mm × angle / 360.
- **Keeping the wheels equal:** 3 mm/s of correction per mm the two
  wheels differ, so straight drives stay straight and spins stay on the
  spot.
- **Spins:**
  - each wheel at 120 mm/s by default; + = left (anticlockwise);
  - `motion_turn_at()` spins slower, and Mission uses 40 mm/s to creep
    onto the tape;
  - **spins of 45° or more stop 6° early** (`S2_TURN_TRIM_DEG`): on the
    mat the robot turns a few degrees more than its wheels say (9 Nov).
- **Time limit:** twice the expected time plus 1 s, then the result is
  TIMEOUT (blocked).
- **Refused:**
  - a speed of 0 (it would never end), returning `E_PAR`;
  - any command while a stop reason is set, returning `E_OBJ`.

### 5.4 Following the line (the follow command)

Every period it reads the **steering** (mm/s, + = left) and the **speed
share** from the line task (interface IF-03). The left wheel's target is
the speed minus the steering, the right wheel's the speed plus it.

**Speed:**
- the command's speed: Mission uses 120 mm/s;
- cut to the line task's share while correcting: 60 %, then 40 % after
  50 mm of correction, and 20 % while searching;
- capped by a follow limit: Mission sets 100 mm/s after a barcode whose
  move is a turn;
- **50 mm/s while the barcode sensor is inside a barcode**
  (`S2_SPEED_IN_BARCODE`).

**Over barcode bars** (the barcode sensor B is 85 mm left of the robot's
centre):
- **Straight hold** (`HOLD_OVER_BARS` 1, the default since 20 Nov): while
  B is over bars, the steering is held at 0, so B crosses the code in one
  straight line.
- **The hold ends** when the robot is leaving its line, that is a long
  correction or a search (line speed under 60 %, `HOLD_MIN_PCT`). Staying
  on the line comes first.
- **A left steer is capped** at 0.36 x the speed over bars, or 0.65 x in a
  long correction (`BAR_LEFT_STEER_MAX` / `_LONG`). A hard left turn
  would make B, on the inside of the turn, stand still or run backwards
  over the bars, and that breaks the reading.
- **Over bars a long correction keeps 50 mm/s** instead of slowing to
  40 % or 20 %, so B keeps moving forward.

On the line, a wheel is never run backwards.

### 5.5 STOP

- `app_stop_request()` turns the PWM off at once, from any task or
  interrupt. It is called by the STOP button, the laptop, Safety and the
  lift stop.
- Motion's own writes check the stop reasons under the lock, so no task
  can undo a STOP.
- Motion then sees the reason and cancels its command (STOPPED). A new
  command is refused until START clears the reason.

### 5.6 What it gives the others

| Call | For |
|---|---|
| `motion_drive(speed, mm)`, `motion_turn(deg)`, `motion_turn_at(deg, speed)`, `motion_follow(speed)`, `motion_halt()` | Mission's moves (each returns a command ID) |
| `motion_get_status()` | the result of that ID (BUSY / DONE / STOPPED / TIMEOUT), wheel speeds, counts |
| `motion_follow_limit(speed)` | a lower speed without restarting the follow |
| `motor_odometer()`, `motor_encoders()` | travel in counts (line, barcode, IMU, Mission) |
| `motor_duties()`, `motor_safe_off()` | Safety's checks, and motors off |
| `motion_steer_held()` | the panel's "steering held" |

## 6. Numbers to know

| Value | Setting |
|---|---|
| Distance per count | 0.080 mm (`S2_UM_PER_COUNT`) |
| Track (wheel to wheel) | 123 mm (`S2_TRACK_MM`, `S2_TURN_TRACK_MM`) |
| Spins of 45° or more stop | 6° early (`S2_TURN_TRIM_DEG`, `S2_TURN_TRIM_MIN_DEG`) |
| Spin speed | 120 mm/s each wheel (`S2_SPEED_TURN`) |
| Over a barcode | 50 mm/s (`S2_SPEED_IN_BARCODE`) |
| Acceleration | 400 mm/s² (`ACCEL_MM_S2`), slowest 50 mm/s near the end (`V_MIN_MM_S`) |
| Controller | feed-forward 1.25 per mille per mm/s + 80; P 1.0; I 6.0, limited to ±400 (`FF_PER_MM_S`, `FF_OFFSET`, `KP`, `KI`, `I_LIMIT`) |
| Keep the wheels equal | 3 mm/s per mm (`KEEP_EQUAL`) |
| Time limit | 2 x expected + 1 s |
| Straight hold over bars | on (`HOLD_OVER_BARS` 1), ends under 60 % line speed (`HOLD_MIN_PCT`) |
| Left steer over bars | at most 0.36 x speed, 0.65 x in a long correction |

## 7. Shown on the robot

- **1 Nov, Stage 1:**
  - wheel directions and encoder signs correct;
  - STOP with the wheels lifted;
  - 500 mm at 150 mm/s measured about 500 mm;
  - turns with the 123 mm track: 90° about right, 180° about 5° over
    (inside ±15°);
  - STOP on the floor.
- **9 Nov re-run** (after the Barr C clean-up and Change Requirement 1):
  - directions and STOP;
  - 500 mm at 120 mm/s by the encoders, mean speed 120;
  - turns, round 1 only (encoders 90.6 / -90.2 / 180.6°; the real 180
    about 5° over);
  - **a held wheel:** Safety turned the motors off and sent its event.
- **In the mission runs:**
  - every junction turn and bypass spin ran;
  - the first U-turn from a barcode (8 Nov) came out about 190°, which
    led to the spins that end on the tape;
  - after a bypass the robot came back within 1-4° of its old heading by
    the wheels (20 Nov).

## 8. Tried, changed or dropped, and why

| What | When | What happened, and why |
|---|---|---|
| Track width 130 mm | 1 Nov | The turns came out about 95° and 190°. 130 x 90 / 95 = 123 mm |
| Spins worked out with a 120 mm track | 9 Nov | About 4° less on a 180, but the bypass's 90s still turned about 5° too much: the extra turn is a fixed few degrees, not a percentage. Back to 123 mm, and every spin of 45° or more stops 6° early |
| A plain 180° U-turn | 8 Nov | It came out about 190°. Junction spins now end on the tape: fast to 30° short, then slowly until a line sensor finds it (subsystem 5) |
| 150 mm/s | 7 Nov | It ran past tight curves: 120 mm/s since |
| 100 mm/s | 7 Nov | Tried together with other line changes; all of them were undone |
| 135 mm/s | 15-18 Nov | It never ran on the course: back to 120 before the demo, the only speed with course runs behind it |
| Straight hold over bars | 15-20 Nov | It first held only the last 15 mm, then the whole code (15 Nov: the hold let go inside a code and the robot swung B across the bars). On 20 Nov it was switched off, because the widths are now measured with B's own travel, and the robot ran its line better. It was switched back on that night, because the evening's best barcode runs were on the hold image. It now ends when the robot leaves its line (the hold had driven it off its line twice) |
| No limit on a left steer over bars | 20 Nov | After START the robot steered hard left over a code (wheels -6 and 44 mm/s at 50 mm/s), and B ran backwards over the bars. The caps keep B moving forward |
| A drive or spin at speed 0 | 8 Nov | It worked out an infinite time limit (undefined behaviour). It is now refused; nothing in the code asks for it |

## 9. Not done, and known limits

- **Real turn angles after the 6° trim** (S2-M02 rounds 2 and 3) are not
  written down. The encoders now read about 6° under the real angle by
  design, so judge a turn by the real angle.
- **STOP on the floor after 9 Nov** (S2-M04): try 1 stopped in 149 ms;
  tries 3-5 not run.
- **The Motion-hung test** (S4-S01, Stage 1, added 17 Nov) has not been
  run: the test freezes Motion for 1 s, and Safety should cut the motors.
- The real distance of a 500 mm drive was not re-measured after 9 Nov
  (the encoders said 500).
- **On the mat a spin turns a few degrees more than its wheels measure.**
  The 6° trim covers it there, and another floor may need a different
  trim.
- On a curve, the 115 mm drive to the junction centre is less accurate.

## 10. Questions you may get

- **How do you measure speed?** Encoder counts over the last 40 ms, x
  0.080 mm, / 0.04 s, for each wheel, every 10 ms.
- **Why feed-forward plus PI and not PID?** The feed-forward, from the
  bring-up measurements, puts the wheel close to the right speed at once,
  and PI removes the rest. No D term was added, because this controller
  passed the Stage 1 tests (the 500 mm drive and turns within their
  limits).
- **How does Mission know a move is finished?** Each command returns an
  ID; `motion_get_status()` gives that ID's result: DONE, STOPPED (a STOP
  came) or TIMEOUT (it took more than twice the expected time plus 1 s,
  e.g. blocked).
- **Why do spins stop 6° early?** On the mat the robot turns a few degrees
  more than its wheels measure, about the same for a 90 as for a 180
  (9 Nov photos). Junction turns also correct themselves by finding the
  tape.
- **Why 120 mm/s?** 150 ran past tight curves (7 Nov), and 120 is the
  speed of every course run without a lost line.
- **What happens on STOP?** The interrupt turns the PWM off at once.
  Motion cancels its command and refuses new ones until START.
- **Why hold the steering straight over a barcode?** The barcode sensor
  sits 85 mm left of the centre. Weaving over a code changes its path
  across the bars, and a hard left turn can stop it or run it backwards,
  which breaks the code into pieces. The hold lets go if the robot starts
  leaving its line.

## 11. Where the code is

| File | What is in it |
|---|---|
| [`motor.c`](../../app_program/subsystem2/motor.c) | PWM, the encoder interrupt and its lookup table, `motor_odometer()`, `motor_safe_off()` |
| [`motion.c`](../../app_program/subsystem2/motion.c) | `motion_task`, the controller (`wheel_duty()`), the commands, the follow logic and the hold over bars |
| [`motion.h`](../../app_program/subsystem2/motion.h) | the commands, the results, the status |
| [`subsystem2.h`](../../app_program/subsystem2/subsystem2.h) | the geometry and speeds |
| [`subsystem2_test.c`](../../app_program/subsystem2/subsystem2_test.c) | the Stage 1 tests (S2-M02 to M07, S4-S01) |
