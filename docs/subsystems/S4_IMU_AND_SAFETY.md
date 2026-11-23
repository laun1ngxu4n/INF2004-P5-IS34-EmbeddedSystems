# Subsystem 4: IMU (hump height, lift stop) and Safety

**Owner:** B4. **Code:** [`app_program/subsystem4/`](../../app_program/subsystem4/)
(`imu.c`, `imu_i2c.c`, `safety.c`). **Tests:** Stage 4
(`images/is34_test_stage4.uf2`: tilt view, S4-R01 hump height, S4-L01 lift
stop) and Stage 1 (S4-S01 Motion hung). The state on 21 Nov 2026.
The whole robot: [README.md](README.md).

## 1. In one minute (what to say)

"Subsystem 4 has two tasks.

**The IMU task** reads the accelerometer 100 times a second.
- **Hump:** the course's hump is a small triangle under one wheel, so the
  robot tips sideways. The height is the track width times the sine of
  that side tilt, at its peak. It is reported once, when the robot is
  level again.
- **Lift stop:** when gravity points more than 35° away from where it
  pointed at START, for 100 ms, the motors go off. That is how lifting the
  robot ends a demo run.

**The Safety task** checks every 20 ms that no wheel is stalled and that
the Motion task is still running. If not, it turns the motors off itself,
without waiting for Motion."

## 2. Why it exists

- **The course:**
  - estimate the hump or ramp height from the IMU (only the lower
    practice ramp matters, 27 Oct);
  - **lecturer, 13 Nov:** IMU collision detection is removed; the IMU must
    detect the robot being lifted or overturned and stop the motors;
  - the Safety / fault monitor is buddy 4's part.
- **Requirements (Tribunal 2 list):**
  - FR4: detect a hump under one wheel and report its peak height;
  - FR13: motors off within 100 ms when lifted or tipped more than 35°;
  - FR9, FR14: the stall and Motion-hung stops, and the shared fault word;
  - NFR8: hump height within ±5 mm for humps of 10-30 mm;
  - NFR10: motors off within 220 ms of Motion stopping, and within 1 s
    of a wheel stalling.

## 3. What it owns, and what it measures

**The IMU:**
- an **LSM303D** on the GY-511 board: an accelerometer and a
  magnetometer, **no gyroscope**;
- I2C1 at 400 kHz (GP26 SDA, GP27 SCL), address 0x1D, using our own
  polled driver (`imu_i2c.c`);
- range ±4 g, 100 readings a second, with the chip's own 50 Hz filter
  against motor vibration;
- its Z axis reads 0.12-0.15 g high, which is why the robot calibrates
  "level" at every START.

**What it measures:**
- the robot's nose-up tilt (pitch) and side tilt, from how gravity falls
  across the accelerometer;
- the height climbed;
- the direction of gravity, for the lift stop.

**Safety reads:** the motor power, the wheel speeds, when the Motion task
last ran, and the encoders' error count.

## 4. Tasks

| Task | Priority | Runs | Does |
|---|---|---|---|
| `safety_task` | 5, the highest application task | every 20 ms | stall, Motion hung, encoder warning |
| `imu_task` | 9 | every 10 ms | tilt, hump height, lift stop |

## 5. How the IMU part works (`imu.c`)

### 5.1 Calibration at START

Mission's CAL state: the robot must stand still and level for 1 s (100
readings). The IMU learns the level pitch and the level direction of
gravity.

### 5.2 Tilt

- **Pitch:** sin(pitch) relative to the level one, filtered over about
  100 ms.
- **Jolts are skipped.** Tilt alone keeps the total acceleration at 1 g. A
  reading more than 10 % away from 1 g is a bump or a speed change, so
  the last tilt is kept.
- **The driving level:** driving tilts the chassis a little (motor torque
  on the caster). While the tilt is within about 3.4° of it, the level
  follows the tilt slowly (over about 5 s), so that lean does not count as
  a climb. On a hump it stays put.

### 5.3 Hump height: two ways, the larger is reported

- **One wheel** (the course's hump: a 2-5 cm triangle under one wheel,
  found 8 Nov):
  - the robot tips sideways about the wheel left on the floor;
  - **height = 123 mm (the track) x sin(side tilt) at its peak**, then
    x 0.63 (`S4_ONE_WHEEL_GAIN_PCT`; see section 7);
  - the side tilt is filtered lightly (about 30 ms), because the
    triangle's sharp top lasts only a moment;
  - a cross-check from the nose-down tilt, 2 x 80 mm x sin(pitch), is
    printed beside it.
- **Both wheels** (a ramp): **climbed height = the sum of (distance
  driven x sin(pitch))**, all the time while driving.

**Reporting:**
- **one wheel (the course's hump), since 23 Nov:** 50 mm after the
  highest point (`PEAK_PAST_MM`), once the slower side-tilt copy is level
  again (under about 1.7°), whatever the nose tilt. That is about 0.5 s
  after the top at 120 mm/s;
  - it reports `ramp peak height H mm (one wheel: side tilt ...; from
    the nose-down tilt ...)` once, if H is 4 mm or more (8 mm until
    23 Nov);
  - why: driving lean and speed changes kept the nose tilt over 1.7°, so
    waiting for the nose to be level too reported seconds late, or not
    at all when the run was stopped first (23 Nov runs);
  - for 200 ms after the line steering jumps by 15 mm/s or more, the side
    tilt is not taken for the peak (`STEER_HOLD_TICKS`,
    `STEER_STEP_MM_S`): the jerk of a sharp correction read as 4-6 mm
    humps in sharp turns (23 Nov).
- **both wheels (a ramp):** once the robot is level again (nose and side
  under about 1.7°) for 100 mm, a climb of 4 mm or more is reported;
- then no second report for the next 600 mm (the way down);
- a STOP or a lift within about 0.5 s of the hump loses the report: let
  the robot drive on about 30 cm after the hump.

**During manoeuvres** (20 Nov):
- when a junction turn or an obstacle stop starts, a hump being measured
  is reported at once;
- nothing is measured while Mission manoeuvres, because spins and bypass
  legs tilt the robot.

### 5.4 Lift or tip-over stop (built 13 Nov)

- Every 10 ms the IMU compares the direction of gravity now (filtered
  about 30 ms) with the one at the last calibration.
- **More than 35° away for 100 ms:**
  - the motors go off as if STOP were pressed (the button's stop reason);
  - Mission is told;
  - the event `robot lifted or tipped over (tilt over 35 deg): motors off,
    START to go again` is sent.
- It re-arms once the robot is back within 20°. START clears it, and
  while the robot is still tilted a START is stopped again.
- **Why 35°:** the largest tilt seen while driving was about 19° (a 25 mm
  hump, 9 Nov), and a robot picked up by hand tilts more. Upside down is
  180°, so overturning is caught too.

### 5.5 IMU fault

If the chip does not answer, the IMU fault bit is set. It is a warning
and does not stop the robot; Mission then skips the calibration.

## 6. How Safety works (`safety.c`)

| Check | Condition | Then |
|---|---|---|
| **Stall** | a wheel driven at 800 per mille or more turns slower than 15 mm/s for 1 s (normal driving needs about 250) | stall fault bit, motors off (fault stop reason), event `Safety: a wheel driven hard is not turning, motors off` |
| **Motion hung** | the motors are on but the Motion task has not run for 200 ms | Motion-hung bit, motors off **without waiting for Motion** (Tribunal 1), event |
| **Encoders** | more than 50 missed encoder edges in a second | encoder warning only: reported, never a stop |
| **Release** | 2 s with the motors off and the cause gone | Safety clears its own bits and the stop reason. Mission is IDLE by then and waits for START |

Only Safety clears these bits. Every change to the fault word is made with
interrupts off (`app_stop.c`), because several tasks and interrupts write
the same word.

## 7. Numbers to know

| Value | Setting |
|---|---|
| Accelerometer | ±4 g, 100 Hz, 50 Hz anti-alias filter |
| Calibration | 1 s, still (`CAL_SAMPLES` 100) |
| Pitch filter | about 100 ms (`LPF_K`); side tilt about 30 ms (`ROLL_LPF_K`) |
| Jolt | 10 % away from 1 g (`JOLT_FRAC`) |
| Level | under about 1.7° (`FLAT_SIN`) for 100 mm (`FLAT_MM`) |
| Smallest report | 4 mm (`REPORT_MIN_MM10`; 8 mm until 23 Nov, the course's hump is about 5 mm); then quiet for 600 mm (`QUIET_AFTER_MM`) |
| One-wheel height | 123 mm x sin(side tilt) x 0.63 (`S4_ONE_WHEEL_GAIN_PCT`) |
| Axle to front support | 80 mm (`S4_L_MM`, measured) |
| Lift stop | over 35° for 100 ms; re-armed under 20° (`LIFT_COS`, `LIFT_TICKS`, `LIFT_CLEAR_COS`) |
| Stall | power 800 per mille or more, under 15 mm/s, for 1 s (`STALL_DUTY`, `STALL_MM_S`, `STALL_TICKS`) |
| Motion hung | 200 ms without a run, motors on (`HUNG_US`) |
| Encoder warning | over 50 bad edges in 1 s (`ENC_BAD_PER_S`) |
| Fault release | 2 s with the motors off (`FAULT_HOLD_TICKS`) |

## 8. Shown on the robot

- **2 Nov:** IMU found (S4-I01), no read dropouts after the cable was
  reseated. The tilt view shows the right sign.
- **6 Nov** (old, pitch-only code, the 10 mm block under both wheels): 2
  of 7 runs within ±5 mm (9.0 and 8.1 mm). The block was not held down,
  and the IMU cable dropped out twice.
- **9 Nov, the one-wheel method:**
  - every hump was seen, but the heights read about 1.6 x too high:
    16.6 / 17.2 / 40.8 / 37.0 / 17.6 mm for humps the tester estimated at
    10 / 10 / 25 / 25 / 10 mm;
  - the 0.63 gain, fitted on those 5 runs, would make them 10.4 / 10.8 /
    25.7 / 23.3 / 11.0 mm (within ±2 mm). It is fitted on those runs, so
    it needs new runs to confirm it.
- **15 Nov, course run 7:** hump `ramp peak height 18.5 mm`.
- **20 Nov, track 4:** humps reported at 8.0-14.0 mm across the runs.
  The real height was not measured with a ruler; the team judged them
  about right.
- **Lift stop:** the motors went off in every run that ended with a lift
  (15 and 20 Nov).
- **Safety, 9 Nov:** a held wheel turned the motors off and sent the
  event.

## 9. Tried, changed or dropped, and why

| What | When | What happened, and why |
|---|---|---|
| Height from the pitch only (a climb, both wheels) | 2-6 Nov | The readings were jolts: 18, 66, -, 49, -, -, 27, -, 9 mm. The course hump turned out to be under one wheel (8 Nov), which tips the robot sideways; the pitch-only code could not see that. The climb method stays for a ramp under both wheels |
| The chip's anti-alias filter at 773 Hz | 6 Nov | Motor vibration showed as -2 to -4° of tilt on the flat: 50 Hz since |
| Jolt filter at 5 % | 6 Nov | It skipped 30-48 % of all readings, and the height kept adding up with a frozen tilt: 10 % since |
| Tilt measured from the level at START only | 6 Nov | Driving tilts the chassis, and the height drifted -10 mm before the hump: the slowly following driving level |
| Reporting dips as well as climbs | 6 Nov | A -56 mm dip was reported as a climb: climbs only |
| One-wheel height with tan | 8 Nov | tan assumes the wheels stay 123 mm apart on the floor: 5 mm too high at 5 cm. The axle tilts as one rigid piece: sin |
| "Level again" from the fast side tilt | 9 Nov | It jitters ±0.03 while driving, so the level stretch kept restarting and nothing was reported: a slower copy since |
| STOP ended all 5 test runs | 9 Nov | The next START did nothing: STOP now ends only the current run |
| The 600 mm quiet distance carried into the next run | 9 Nov | Calibration now resets it |
| Measuring during spins and the bypass | 9-20 Nov | A false 41.2 mm during a bypass (9 Nov), and two reports for one hump (20 Nov). Now nothing is measured while Mission manoeuvres, and a hump in progress is reported when a manoeuvre starts |
| IMU collision detection (2 g; ±4 g was chosen so 2 g would not clip) | 13 Nov | Removed by the lecturer: it conflicted with avoiding obstacles. It had never been built; the lift stop replaces it |
| Safety as a stub (Stage 0) | until 8 Nov | Change Requirement 1 and Tribunal 1 needed a real fault monitor: stall, Motion hung and the encoder warning were built |
| A 500 ms hardware watchdog (Tribunal 1 design) | - | Not built |
| Battery voltage sense (GP28) | 26 Oct | Removed from scope: the board's jumper is not fitted, so there is no low-battery fault |

## 10. Not done, and known limits

- **Hump accuracy (±5 mm) has not been shown against ruler-measured
  humps.** The 0.63 gain was fitted on 5 runs whose humps were estimated,
  not measured. Measure the hump, run S4-R01 5 times, and compare.
- **A hump is reported 50 mm past its top** (one wheel; since 23 Nov,
  100 mm of level driving before), so its event and beep come about
  0.5 s after the hump. A STOP or a lift before that loses it.
- **No gyroscope:** any push (a bump, braking) looks like tilt. The jolt
  filter helps but cannot remove it all, and a hard STOP while moving can
  give a false hump report.
- **A short hump under both wheels** tips the robot instead of lifting it
  along a slope, so the climb sum does not equal its height. That is why
  the one-wheel method exists.
- **A lift that keeps the robot within 35° of level** is not seen: an
  accelerometer feels gravity, not height. Pick the robot up the usual,
  tilted way.
- **Not run:**
  - **the Motion-hung test, S4-S01** (Stage 1, wheels lifted; it freezes
    Motion for 1 s);
  - the formal S4-L01 lift test (3 tries). The lift stop worked at the
    end of the course runs instead.
- **Not measured:** the stop times of NFR10 (220 ms and 1 s) with a scope.
- **The IMU's cable dropped out twice** (6 Nov): reseat it at both ends,
  and keep the board fixed firmly.

## 11. Questions you may get

- **How do you get a height from an accelerometer without a gyro?** At
  standstill or steady driving it feels only gravity, so the direction of
  gravity gives the tilt. With one wheel on the hump, the rigid axle tilts
  about the other wheel: height = track width x sin(side tilt).
- **Why the side tilt and not the pitch?** The course's hump goes under
  one wheel only. The robot tips sideways, and its nose even dips a
  little. The pitch-only version saw nothing reliable.
- **Why scale by 0.63?** On 9 Nov the one-wheel readings were about 1.6 x
  the real height for both hump sizes, because the wheel climbing jolts
  the robot sideways and the accelerometer adds that to the tilt. 0.63 is
  the least-squares fit over those runs, and it needs confirming.
- **Why 35° for the lift stop?** Driving never tilted the robot more than
  about 19°, and a hand lift tilts more than 35°. It must last 100 ms,
  so a bump cannot trigger it.
- **What if the Motion task hangs?** Safety sees that Motion has not run
  for 200 ms while the motors are on, and turns them off itself, without
  Motion. So the motors are off within 220 ms (200 ms plus one 20 ms
  Safety period).
- **Why is an encoder fault only a warning?** Missed edges make the
  distance a little wrong, but stopping would end the run for a small
  error. It is reported so the tester knows.

## 12. Where the code is

| File | What is in it |
|---|---|
| [`imu.c`](../../app_program/subsystem4/imu.c) | `imu_task`: calibration, tilt, jolts, the driving level, both height methods, reporting, the lift stop |
| [`imu_i2c.c`](../../app_program/subsystem4/imu_i2c.c) | the polled I2C1 driver |
| [`safety.c`](../../app_program/subsystem4/safety.c) | `safety_task`: stall, Motion hung, encoder warning, release |
| [`subsystem4.h`](../../app_program/subsystem4/subsystem4.h) | `S4_L_MM`, the one-wheel gain |
| [`subsystem4_test.c`](../../app_program/subsystem4/subsystem4_test.c) | the Stage 4 tests (tilt view, S4-R01, S4-L01) |
| [IMU_REQUIREMENT_CHANGE.md](../team/course/IMU_REQUIREMENT_CHANGE.md) | the lecturer's 13 Nov change, word for word |
