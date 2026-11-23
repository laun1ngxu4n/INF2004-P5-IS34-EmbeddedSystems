# Presenting our parts: notes for each buddy

For each buddy, before the demo and Tribunal 2. One page each:
- what to say;
- what to point at while the robot runs;
- which numbers to quote;
- what **not** to claim;
- short answers to likely questions;
- what to say if something goes wrong live.

The full detail is on each subsystem page ([README.md](README.md),
section 3). Every number here comes from those pages and
[TEST_STATUS.md](../../testing/TEST_STATUS.md) on 21 Nov 2026, and so
from robot logs. Check TEST_STATUS for anything newer before you
present.

## Rules for everyone

- **Quote only what a log shows.** If asked about something not tested,
  say "not tested yet", then what the test would be. Never round a result
  up.
- **Say "on 20 Nov the log shows ...",** not "it always ...". A date and
  a number beat an adjective.
- **Say your part in your own words.** Read the "Say" box twice, then put
  it away.
- **End with the hand-over line,** so the next buddy starts where you
  stopped.
- **Questions about the whole robot:**

  | Topic | Who answers |
  |---|---|
  | task priorities, one core, integration | B5 |
  | the stop word and the fault word | B4 (Safety), with B5 (the buttons) |
  | binary messages (Change Requirement 1) | B1 |

## Running order: follow one demo run

| # | Buddy | The moment in the run | Hand-over line |
|---|---|---|---|
| 0 | B5 | the whole robot in 30 s | "Let's follow one run, starting with the laptop." |
| 1 | B1 | mapping sent, START from the panel | "Now the robot starts following the line." |
| 2 | B3 | following the line, reading a barcode | "The steering goes to the wheels." |
| 3 | B2 | wheels, speed control, the junction spin | "Who decides when to turn, and what about obstacles?" |
| 4 | B5 | junction move, obstacle scan and bypass | "Then the hump, and how the run ends." |
| 5 | B4 | hump height, lift stop, Safety | (close) |

---

## 0. B5: the whole robot (30 s)

**Say:**
- "Eight tasks on micro T-Kernel, on one core. What protects the robot
  runs first: Safety, then Motion every 10 ms. The laptop link runs last,
  so WiFi can never delay driving."
- "Every stop goes through one shared stop word: any task or interrupt
  can turn the motors off at once, and nothing can turn them back on
  until START."
- "Change Requirement 1: every value takes only the bits it needs. Our
  modules' RAM went from 3,795 to 3,143 bytes (-652 B, 17 %)."

**Numbers:**
- Timing tables over 5 minutes (9 Nov): every task within about ±1.7 ms
  of its period.
- Longest run times: telemetry 2.6 ms, IMU 1.0 ms, Motion 0.37 ms.

**If asked why one core:** with one core, interrupts off is enough to
protect shared data, and nothing needed the second core's speed. The
build refuses `SMP=1` until the lock has a hardware spinlock.

---

## 1. B1: Telemetry, WiFi / MQTT, buzzer

**Say:**
- "My part is the robot's link to our laptop.
- **To the laptop:** a 15-byte binary status every second, which is
  also its heartbeat; 8 bytes of live sensors twice a second; every
  result as a text event, with one beep on the robot.
- **From the laptop:** the barcode mapping, START and STOP. A STOP sets
  the stop bit before anything else, so the motors go off in that same
  call.
- **Where the network code runs:** the MQTT client runs in the kernel's
  own WiFi task; our code only copies in and out through two hooks that
  never wait.
- **If the WiFi drops:** that is only a warning. The robot keeps driving
  and reconnects by itself."

**Show:**
- top right of the panel: `robot: online`;
- press **Send mapping to robot**: the answer `OK ...`, then
  `robot has: ... (same)`;
- the status box changing every second: "all of this is one 15-byte
  message";
- the Events tab (`[robot]` lines), and the Run summary after the run.

**Numbers:**
- Change Requirement 1:

  | Message | Text | Binary |
  |---|---|---|
  | status | 93 B | 15 B |
  | sensors | 59 B | 8 B |
  | mapping answer | 18 B | 5 B |
  | mapping | 15 B | 4 B |

- The robot's `wire.c` against the panel's decoder: 15 of 15 shared test
  messages pass.
- Remote START / STOP (2 Nov): 3 of 3 passed. Each STOP's event came in
  the same second.
- After the broker was gone for about 100 s, the robot re-joined by
  itself (7 Nov).

**Don't claim:**
- that reconnecting within 20 s was measured: it was seen, but not timed
  in repeated trials;
- all 5 START / STOP rounds: 3 of 5 were run;
- that no event is ever lost: after a drop longer than about 3 s, the
  oldest events are pushed out of the 8-slot outbox.

**Questions:**
- *Why binary?* Each value takes only the bits its range needs (Change
  Requirement 1).
- *Why are events still text?* They are sentences for the operator, not
  values for a program.
- *Why not call MQTT from your own task?* The network stack is only safe
  inside the kernel's WiFi task (the template's rule), so we use two
  hooks.
- *How do you know it's alive?* The status every second is the
  heartbeat. The panel shows `OFFLINE` after 3 s without one.

**If it goes wrong live:**
- **`robot: no data`:** is the robot on the same 2.4 GHz hotspot? Has the
  laptop's IP changed (`ipconfig`)? The IP is built into the image, and a
  new one needs a rebuild.
- **Say:** "The STOP button on the robot works without the WiFi."

---

## 2. B3: Line following, junctions, barcodes

**Say:**
- "My part is the robot's eyes on the floor.
- **The line:** two IR sensors straddle the 20 mm tape, so both white
  means centred, one black says which way to steer, and both black is a
  cross line. A steering memory keeps the robot turning through curves
  between corrections.
- **Junctions:** both sensors black at once is a crossing. A left arm is
  seen by the barcode sensor, which sits further left.
- **Barcodes:**
  - every bar edge is stamped with the distance the barcode sensor
    itself has travelled, so speed and steering don't change the widths;
  - we decode Code 39 in both directions;
  - we repair a few typical faults, but only when exactly one letter
    fits, so we never guess.
- **The mapping:** my part also keeps the mapping from the laptop."

**Show:**
- the L, R, B circles on the panel lighting up as it corrects;
- `Speed L,R` dropping to about 50 over a barcode;
- the event `barcode F ... -> LEFT at next junction`, then the turn at
  the junction.

**Numbers:**
- **3.65 m and 4 m** of following with no line lost (20 Nov). The target
  (NFR1a) is 3 m.
- **The line simulator, calibrated on 3 robot runs:** the old settings
  passed 297 of 480 scenarios, the new ones 478 of 480, at the same
  speed.
- **A, F, Q and Z** were read and obeyed on track 4, in both directions
  (20 Nov).
- **Best session:** 7 of 12 passes read.
- **No wrong letter has ever been acted on.**

**Don't claim:**
- that every barcode is read on every pass: the best session was 7 of
  12;
- that a right-only branch is detected: it can't be told from a curve;
- the formal junction count (S3-L02), or every Stage 2 test on the final
  code: neither was done.

**Questions:**
- *Why straddle the tape?* "Both white = centred" gives a clear left /
  right signal; riding on the tape could not.
- *Why measure bars by distance?* The width is then the same at any
  speed, and using the barcode sensor's own travel means turning doesn't
  change it either.
- *A missed or damaged bar?* It is repaired only when exactly one letter
  fits; otherwise `READ_FAIL`, and the robot goes straight. A doubtful
  read gives no move.

**Before the demo:**
- check the IR live view; the sensor pots shift when the robot is
  handled;
- place barcodes flat and square on straights, away from the hump;
- set the robot down centred, straight, and at least 10 cm before a code.

**If it goes wrong live:** after a `READ_FAIL`, say "a doubtful read
gives no move: it goes straight on purpose, never a wrong turn."

---

## 3. B2: Motion (motors, encoders, speed control)

**Say:**
- "My part makes the wheels do what the others ask.
- **Speed:** each encoder gives about 2,540 counts per wheel turn, 0.08
  mm each. Every 10 ms the Motion task works out each wheel's speed and
  sets its power with feed-forward plus PI.
- **Three commands:** drive a distance, spin by an angle, and follow the
  line with the steering from subsystem 3.
- **Results:** every command has an ID and a result, so Mission knows if
  it finished, was stopped, or timed out.
- **STOP:** turns the motors off in the interrupt itself, and Motion
  refuses to move while any stop reason is set."

**Show:**
- the smooth start (it speeds up at 400 mm/s²);
- `Speed L,R` near 120 while following;
- a junction spin that ends on the tape;
- STOP: the wheels stop at once.

**Numbers:**
- 0.080 mm per count, a 10 ms control loop.
- **500 mm drive** (1 Nov): measured about 500 mm.
- **Turns** (1 Nov): the 90° about right, the 180° about 5° over. The
  limit is ±15°.
- **A wheel held** (9 Nov): Safety cut the motors.
- **After a bypass** (20 Nov): back within 1-4° of the old heading, by
  the wheels.

**Don't claim:**
- real turn angles after the 6° trim: they were not written down;
- STOP timing on the floor over 5 tries: only try 1 was run (149 ms);
- the Motion-hung test (S4-S01): not run;
- "PID": it is feed-forward plus PI, with no D term.

**Questions:**
- *Why no D term?* The feed-forward, measured in bring-up, gets close to
  the right speed at once, PI removes the rest, and that passed the
  Stage 1 tests.
- *How do you measure speed?* The counts over the last 40 ms, x 0.080 mm,
  / 0.04 s, every 10 ms.
- *Why do spins stop 6° early?* On the mat the robot turns a few degrees
  more than its wheels measure. Junction turns also finish by finding the
  tape.
- *Why 120 mm/s?* 150 ran past tight curves (7 Nov); 120 is the speed of
  every course run without a lost line.

**If it goes wrong live:** after a turn that overshoots, say "junction
turns correct themselves: the robot spins slowly until a sensor finds
the tape."

---

## 4. B5: Mission and obstacles

**Say:**
- "My part runs the course. Mission is a state machine: IDLE, START, a
  1 s calibration, RUN.
- **Junctions:** it keeps the last mapped letter. At the next junction it
  drives the wheels onto the junction's centre and spins onto the new
  line.
- **Obstacles:**
  - the Obstacle task pings ahead every 60 ms, and two readings under
    18 cm stop the robot;
  - the servo sweeps the sonar ±40° in 5° steps, 3 pings each, and the
    robot reports the width ± uncertainty;
  - it drives round on a clear side with a 10 cm margin, watching ahead
    on every leg, finds its own line, and turns back to its old
    direction.
- **Stops:** anything unsafe, or a step that doesn't finish, stops the
  robot with a reason."

**Show:**
- the stop about 15 cm before the box, and the servo sweep;
- the event `obstacle width W +/- U mm`;
- the five legs round the box;
- `bypass done, following the line (heading N deg ...)`;
- in the Run summary, the obstacle block, including "depth not
  measured".

**Numbers:**
- **Stop distance** (20 Nov): 9 of 10 stops at 150-165 mm.
- **The 10 cm box:** widths mostly 95-125 mm.
- **Heading after a bypass:** 1-4° of the old one.
- **Junction moves:** LEFT, RIGHT, FORWARD and U-TURN all done from
  barcodes (20 Nov).
- **Edge rule:** both edges found in 21 of 21 logged scans; the old rule
  found 6.
- **The bypass simulator:** the line found again in 471 of 486 cases,
  against 36 with the old fixed rectangle.
- **Time from stop to width:** 6-8 s with one scan, about 17 s with two.

**Don't claim:**
- every stop ≥ 150 mm: one was 131 mm (20 Nov, cause not known);
- the assumed box depth of 120 mm (21 Nov): not run on the robot yet;
- that it measures depth: it measures width only and assumes up to
  120 mm;
- a second obstacle: it is not bypassed, the robot stops safely;
- the formal 3 full runs (SYS-01): not recorded as such, though full
  sequences ran on 15 and 20 Nov.

**Questions:**
- *How does it choose a side?* The side that is clear beyond its edge;
  if both are, the nearer edge. Neither: it stops.
- *Where does the ± come from?* The 5° step and the sonar's beam,
  combined for both edges.
- *Why plan for at least 150 mm?* The brief says 10-20 cm, and a scan can
  read narrow. The plan is capped at 300 mm, after one bad scan read
  500 mm.
- *Why not measure the depth?* The sonar faces forward. A 40° side look
  was tried (8 Nov), but it heard only the box's front corner, not its
  side.
- *How does it find its line?* It drives back until any sensor sees
  black, then turns back like a junction turn until a sensor finds the
  tape.

**Before the demo:**
- obstacles at least 30 cm after a barcode;
- nothing else beside the box;
- the box on a straight.

**If it goes wrong live:** a stop such as `no clear side` or
`bypass leg 4 blocked` is a designed safe stop. Say so, then press START.

---

## 5. B4: IMU (hump, lift stop) and Safety

**Say:**
- "My part has two tasks.
- **The IMU task** reads the accelerometer 100 times a second.
  - **Hump:** the course's hump is under one wheel, so the robot tips
    sideways. The height is the track width times the sine of that side
    tilt at its peak. It is reported once, when the robot is level again.
  - **Lift stop:** if gravity points more than 35° away from where it
    pointed at START, for 100 ms, the motors go off. That is how lifting
    the robot ends a run.
- **The Safety task** checks every 20 ms that no wheel is stalled and
  that Motion is still running. If not, it turns the motors off itself."

**Show:**
- after the hump, about 0.5 s later: the event `ramp peak height H mm`
  and a beep (let the robot drive on about 30 cm before lifting it);
- `IMU tilt` on the panel while it climbs;
- lift the robot to end the run: the motors stop, with the event
  `robot lifted or tipped over`.

**Numbers:**
- **Lift stop:** more than 35° for 100 ms. Driving never tilted the robot
  more than about 19°.
- **Motors off on every run that ended with a lift** (15 and 20 Nov).
- **Humps reported** at 8-14 mm on track 4 (20 Nov), and 18.5 mm (15 Nov).
- **Safety:** a held wheel cut the motors (9 Nov).
- **By design:** the motors go off within 220 ms if Motion hangs.

**Don't claim:**
- ±5 mm accuracy: it has not been checked against ruler-measured humps.
  The 0.63 gain was fitted on 5 runs whose heights were estimated;
- the stop times (NFR10) measured with a scope: not done;
- the Motion-hung test (S4-S01): not run;
- collision detection: removed by the lecturer on 13 Nov, never built.

**Questions:**
- *A height without a gyro?* Standing still or driving steadily, the
  accelerometer feels only gravity, so it gives the tilt. With one wheel
  up, the rigid axle tilts about the other wheel: height = track x
  sin(side tilt).
- *Why 0.63?* The wheel climbing jolts the robot sideways, so the raw
  readings came out about 1.6 x the height. 0.63 is the fit, and it still
  needs confirming.
- *Why 35°?* Driving tilted the robot at most about 19°, and a hand lift
  tilts more. It must last 100 ms, so a bump can't trigger it.
- *Why is an encoder fault only a warning?* Missed edges make the
  distance a little wrong, and stopping would end the run for a small
  error.

**Before the demo:**
- the robot still and level for the 1 s calibration after START;
- the IMU cable seated firmly at both ends.

**If it goes wrong live:** no hump report yet? It comes 50 mm past the
hump's top, once the side tilt is level (about 0.5 s); a STOP or a lift
before that loses it, and nothing is measured during a turn or a bypass.

---

## Questions anyone may get

| Question | Short answer | Who |
|---|---|---|
| What happens when you press STOP? | The stop word: motors off at once, from any task or interrupt; nothing drives until START | B5 / B2 |
| What if the WiFi drops mid-run? | Only a warning; the robot keeps going and reconnects | B1 |
| What did Change Requirement 1 change? | Minimum-bit fields, one fault bitmask, binary messages; RAM -652 B (17 %) | B1 / B5 |
| What was tried and dropped? | Each subsystem page, section "Tried, changed or dropped" | everyone |
| What would you do with more time? | Each page's "Not done, and known limits" | everyone |
