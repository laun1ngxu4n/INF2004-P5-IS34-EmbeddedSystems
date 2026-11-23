# Subsystem 5: Mission controller and obstacle scanner

**Owner:** B5 (also integration). **Code:** [`app_program/subsystem5/`](../../app_program/subsystem5/)
(`mission.c`, `obstacle.c`, `subsystem5.c`). **Tests:** Stage 3
(`images/is34_test_stage3.uf2`: servo, sonar view, S5-O01 width, S5-O02
bypass), Stage 6 and the mission image (the whole course). The state on
21 Nov 2026. The whole robot: [README.md](README.md).

## 1. In one minute (what to say)

"Subsystem 5 runs the course. The Mission task is a state machine: IDLE,
then START, a 1 s calibration, then RUN.

- **Barcodes and junctions:** it keeps the last mapped letter from a
  barcode. At the next junction it drives the wheels onto the junction's
  centre and spins onto the new line.
- **Obstacles:** the Obstacle task pings straight ahead every 60 ms. Two
  readings under 18 cm and Mission stops the robot and scans. The servo
  turns the ultrasonic sensor from 40° left to 40° right in 5° steps,
  with 3 pings per step. From that it works out where the obstacle's
  edges are, its width and an uncertainty. It drives round on a clear
  side with a 10 cm margin, watching ahead on every leg. It finds its own
  line again and turns back to its old direction.
- **Stops:** if anything is unsafe, or a step does not finish, it stops
  and says why."

## 2. Why it exists

**The course:**
- the obstacle brief (TEAM.md 7.2, O1-O9): obstacles 10-20 cm wide;
  profile, don't measure precisely; settle the servo; take the median of
  several pings; edges with hysteresis; two positions; report width ±
  uncertainty; plan for the largest width; a safe path; find the line
  again;
- the barcode moves, U-turn included, at the cross-junction (TEAM.md 7.1,
  B5-B7);
- buddy 5 also integrates the subsystems.

**Requirements (Tribunal 2 list):**
- FR3: the mapped move at the junction; an unmapped letter is ignored;
- FR5: detect, stop, scan ±40°, report W ± U;
- FR6: go round on a clear side, watch every leg, rejoin the own line in
  the old heading;
- FR8: find the line or stop;
- FR11: the latest unused letter;
- FR12: the START and STOP buttons;
- FR16: a blocked leg stops the robot;
- NFR3: a scan within 10 s;
- NFR4: a barcode at least 200 mm before its junction;
- NFR6: stop at least 150 mm away;
- NFR6a: within 300 mm of the line, no contact;
- NFR12: the watch on every leg, and turning beside the box kept 185 mm
  clear.

## 3. What it owns, and what it measures

| Part | Pins | How it is used |
|---|---|---|
| HC-SR04 ultrasonic, run at 3.3 V | TRIG GP6, ECHO GP7 | The echo is timed in the GPIO interrupt: distance = echo time x 0.174 mm per µs (348 m/s at 28 °C, there and back). Valid from 0.1 to 25 ms, about 17 mm to 4.35 m; otherwise "nothing in range" |
| Servo turning the sensor | GP14 (PWM slice 7) | 1,500 µs = straight ahead, 11 µs per degree; ±40° (the cables at the back limit it); at most 2° per 20 ms (full speed broke the horn, 2 Nov) |
| START button | GP20 | read by polling; it acts when the button is **released** |
| STOP button | GP21 | an interrupt: the motors go off in the interrupt itself, then Mission is told |

**What it measures:**
- the distance straight ahead;
- an obstacle's left and right edges (mm from the robot's centre line),
  its width and uncertainty;
- the robot's heading change over a bypass, by the wheels (a guide only).

## 4. Tasks

| Task | Priority | Runs | Does |
|---|---|---|---|
| `mission_task` | 7 | on the event flag, at least every 100 ms | the state machine; runs junction moves and the bypass step by step |
| `obstacle_task` | 11 | every 60 ms | the forward watch; pings for the panel while IDLE |

The scan itself runs inside Mission's call (`obstacle_scan()`), while the
robot stands still.

## 5. The mission (`mission.c`)

### 5.1 States

```
IDLE --START--> CAL (1 s, IMU learns level) --> RUN --junction turn / obstacle--> MANOEUVRE --> RUN
  ^                                                |
  +---- STOP, a fault, or "stopped: <why>" --------+
```

- **START** is the GP20 button (on release) or the panel. It clears the
  button and laptop STOP reasons (Safety clears its own), calibrates the
  IMU, and resets the barcode reader, any old letter and any ignore
  window. Then it follows the line at 120 mm/s.
- **STOP** (button, panel, Safety, lift): the motors are already off.
  Mission turns the watches off and goes back to IDLE.
- **A stop with a reason:** `stopped: <why> (press START to go again)`.

### 5.2 Barcodes

- **A letter in the mapping** becomes the pending letter; a newer mapped
  letter replaces it.
- **A letter not in the mapping** is reported (`-> NO MAPPING, ignored`)
  and the robot carries on. It does not replace a mapped letter that is
  still waiting.
- **A half code that is not used** (subsystem 3) is only reported.
- **After a letter whose move is a turn**, the robot follows at 100 mm/s
  until the junction, so it weaves less and does not overrun the junction.
- **A failed read** (`READ_FAIL`) is reported and its widths printed; no
  letter is stored.

### 5.3 Junctions

1. **A barcode still being read:** the letter comes about 100 mm after
   the line sensors pass the code's last bar, so the junction waits while
   B is still in a code. The robot keeps following, at most 150 mm past
   the centre.
2. **The move:**
   - the pending letter's move; no letter means **straight on**;
   - if the mapping changed meanwhile and the letter is no longer in it,
     also straight.
3. **A turn:**
   - drive until the wheels are over the junction centre: 115 mm from
     where the line sensors crossed it, backing up if the robot went past;
   - then **spin onto the line**: fast to 30° short of the turn (+90 left,
     -90 right, 180 U-turn), then slowly (40 mm/s per wheel) until a line
     sensor touches the tape (at most 30° past), then 12° more to centre
     the tape between the sensors;
   - no tape found: back to the planned angle, `turn: no tape found`.
4. **Follow again.** Junction and line-lost detection come back after
   60 mm.

### 5.4 An obstacle, step by step

1. **The forward watch** (Obstacle task): one ping every 60 ms. Two
   readings in a row under **180 mm** tell Mission.
2. **Stop and get ready:**
   - halt and wait until still;
   - a barcode in the middle of being read is dropped and reported
     (backing up would run B back over the bars);
   - no echo at all: stop, `ultrasonic not answering`.
3. **Scan 1, from where it stopped:** it backs up only if nearer than
   160 mm (section 6).
   - Nothing found: `scan found nothing ahead, going on` (a false stop
     costs a few seconds, nothing else).
4. **Scan 2:** only if scan 1 missed an edge. The robot backs up 100 mm,
   scans again, and the two are combined.
5. **Report:** `obstacle width W +/- U mm (scan 1 ..., scan 2 ...); edges
   L ... R ... mm; front ... mm`.
6. **Choose a side:** one that is clear beyond its edge; if both are, the
   nearer edge (less to drive). Neither: stop, `no clear side`.
7. **Plan:**
   - the width planned for is the measured width + its uncertainty, but
     **at least 150 mm and at most 300 mm** (`S5_MAX_WIDTH_MM`,
     `S5_PLAN_MAX_MM`);
   - the planned edge is never more than 200 mm out from the line;
   - **out** = that edge + half the robot's width (85) + the margin
     (80; 100 until 23 Nov): about 23-24 cm for the 10 cm box;
   - **past** = the distance to the box + the assumed depth (120) + the
     robot's length (200) + the margin (80): about 555 mm from where it
     stopped (575 until 23 Nov).
8. **The legs**, each with the forward watch on (2 pings under 150 mm
   stop the robot, `something in the way during the bypass`):

   | Leg | What happens |
   |---|---|
   | 1 | turn 90° to the clear side |
   | 2 | check the way out is clear (else `bypass leg 2 blocked`), then drive out |
   | 3 | turn back to the old direction |
   | 4 | check ahead (else `bypass leg 4 blocked`), then drive past the box |
   | 5 | turn toward the line. If the sonar still hears the box beside, it turns back, drives 100 mm further and looks again, up to 4 times; otherwise `obstacle longer than expected` |
   | 6 | drive until **any** line sensor sees black, giving up 400 mm past where the line should be (`line not found after the bypass`) |
   | 7 | drive 115 mm so the wheels are on the line, then **turn back to the old direction like a junction turn**: fast, then slowly until a sensor finds the tape within 30° either side, then centred |

9. **Finish:**
   - the barcode sensor is ignored for the next 200 mm, because the robot
     is still settling and B could read the line's own tape as bars;
   - `bypass done, following the line (heading N deg from before the box,
     by the wheels)`.

### 5.5 Every way Mission stops, with its message

| Message | When |
|---|---|
| `STOP` | the button, the panel, Safety or a lift |
| `line lost` | the line task found no line |
| `a line sensor black for 30 cm ...` / `both line sensors black for 15 cm ...` | a stuck sensor |
| `junction move did not finish`, `bypass step did not finish` | a Motion command was stopped or timed out |
| `no clear side` | neither side clear |
| `bypass leg 2 blocked`, `bypass leg 4 blocked`, `something in the way during the bypass` | something else in the way |
| `obstacle longer than expected (...)` | still beside the box after 4 extra steps |
| `line not found after the bypass` | no black on the way back |
| `ultrasonic not answering (no ECHO), not driving blind`, `... during the scan` | a sensor fault |
| `obstacle lost between scans` | scan 2 did not find what scan 1 saw |

## 6. The obstacle scanner (`obstacle.c`)

- **The sweep:** 17 rays from 40° left to 40° right in 5° steps.
  - The servo settles 3 ms per degree moved, at least 60 ms.
  - **3 pings per ray, the median.** Two or more missing pings mean open
    space.
  - From the stop to the width report: 6-8 s with one scan and about
    17 s with two (20 Nov run summaries). The Tribunal 2 target is 10 s
    for one scan.
- **Which rays are on the obstacle** (the geometry rule, 8 Nov):
  - a flat face square to the robot is at front / cos(angle) along a ray
    at that angle, so a ray is on the obstacle when its distance fits that
    (within 15 mm + 4 %);
  - within 10° of straight ahead, any echo near the front distance counts;
  - a ray much nearer than that is the face's echo heard through the side
    of the beam, so it counts as neither the box nor something else;
  - a ray farther than front + 250 mm is open space.
- **An edge** lies between the last ray on the obstacle and the first of
  two rays off it. The sweep's last ray alone will do when it is open
  space (20 Nov).
  - The edge angle is pulled in by **12°** for the width of the sonar's
    beam (`S5_BEAM_HALF_DEG`): the beam is about 30° wide and "sees" the
    box well past its edge.
- **Edge position** = distance x sin(angle). **Width** = left edge -
  right edge.
- **Uncertainty:** each edge has a ± from the 5° step and the beam. The
  total is √(left² + right² + beam²), where the beam part is
  2 x front x sin(3°).
- **Two scans:** each edge comes from the scan that saw past it. Where
  both did, the nearer scan's is used, because every pair of real scans
  read narrower from nearer.

## 7. Numbers to know

| Value | Setting |
|---|---|
| Mission speed | 120 mm/s (`S5_FOLLOW_SPEED`); 100 after a turn letter (`S5_APPROACH_SPEED`) |
| Junction | wheels 115 mm past where the sensors crossed; waits up to 150 mm for a barcode (`JUNC_WAIT_MM`); spin 30° short, then 40 mm/s, +12° (`TURN_SEARCH_DEG`, `SPIN_SPEED`, `TURN_CENTRE_DEG`) |
| Line events back after | 60 mm (`REJOIN_ENABLE_MM`) |
| Stop for an obstacle | 2 readings under 180 mm (`S5_TRIP_MM`); it stops about 150-165 mm away |
| No echo | 5 pings in a row (0.3 s) |
| Scan | ±40°, 5° steps, 17 rays, 3 pings each, settle at least 60 ms |
| Scan from | where it stopped; backs up if nearer than 160 mm (`SCAN_NEAR_MM`); scan 2 after 100 mm back (`REVERSE_MM`) |
| Beam pull-in | 12° (`S5_BEAM_HALF_DEG`) |
| Plan | width + uncertainty, between 150 and 300 mm; margin 80 mm (`MARGIN_MM`; 100 until 23 Nov: bypasssim, no contact at 80, closest pass 1 mm); assumed depth 120 mm (`D_MAX_MM`, 150 until 21 Nov) |
| Watch on the legs | 2 pings under 150 mm (`WATCH_STOP_MM`) |
| Box still beside | 100 mm further, up to 4 times (`EXTEND_MM`, `EXTEND_TRIES`) |
| Back to the line | gives up 400 mm past (`REJOIN_FAR_MM`); barcode ignored 200 mm after (`REJOIN_BAR_MM`) |
| Clearances | side pass about 12-13 cm from the box; turning beside it, the robot's centre at least 185 mm from its side (corners swing 155 mm) |

## 8. Shown on the robot

**The obstacle and the bypass:**
- **7 Nov:** the first bypass worked first time and found the line again.
  The same day a wrong 445 mm detour met something in the room and
  stopped with `bypass leg 4 blocked`, as designed.
- **8 Nov:** a bypass with the box standing **on a junction**: no
  contact, back on the line. The team called Stage 3 done.
- **9 Nov** (Change Requirement 1 build): the bypass worked as before.
- **15 Nov, course run 7: the whole sequence.** Barcode F to LEFT,
  obstacle stop at 168 mm, width 116 mm, bypass, hump 18.5 mm, lifted:
  motors off.
- **20 Nov, track 4** (10 cm box):
  - widths 85-138 mm, most of them 95-125 (85, 95, 97, 108, 109, 115,
    116, 118, 119, 120, 123, 125, 138);
  - twice too wide (142 ± 29 and 151 ± 28), when scan 1 heard echoes
    across the whole sweep and the second scan was used;
  - rejoins at 1-4° of the old heading after the turn-back change;
  - 3.65 m and 4 m of following after a bypass.
- **Stop distances, 20 Nov,** since the trip was set to 180 mm (runs 7-13,
  the scan's straight-ahead reading): 9 of 10 stops were **150-165 mm**
  from the box. **One was 131 mm** (the evening run, log
  `2026-11-20_track4_mission_11_robot.txt`). Its watch had reported
  161 mm, and the cause is not known. That stop is below the 150 mm
  target (NFR6).
- **False stops:** two (75 and 164 mm) found nothing in the scan, and the
  robot went on.
- **`no clear side`:** seen once in a run (20 Nov, something 47 mm
  ahead).

**Mission and junctions:**
- **8 Nov:** the first U-turn from a barcode.
- **15 Nov:** every junction LEFT as mapped.
- **20 Nov:** LEFT, RIGHT, FORWARD and U-TURN all done from barcodes.
  Letters with no mapping were ignored, and the mapping was changed
  between runs.

## 9. Tried, changed or dropped, and why

**The scan and the width:**

| What | When | What happened, and why |
|---|---|---|
| Servo ±25° | early Nov | It never saw past a box's edge (the beam is about 30° wide): ±40°, with the cables routed for it |
| Full servo speed | 2 Nov | It broke the horn: 2° per 20 ms |
| A ray "on the box" when within 150 mm of the front distance | until 8 Nov | Whole sweeps read as box (400-500 mm for 15-23 cm boxes), and one detour ran into the room. The geometry rule found both edges in 21 of 21 logged scans (the old rule 6) |
| Backing up to 350 mm before scanning | 7-8 Nov | With the geometry rule the near scans are better (every pair read narrower from nearer): scan from where it stops, and 100 mm back only for scan 2 |
| Always two scans | until 15 Nov | The robot turned about 12° while backing up and lost the box: `obstacle lost between scans`, though scan 1 had both edges. Scan 2 now only when scan 1 missed an edge |
| 5 pings per ray | until 20 Nov | A scan took about 6.5 s, 9 s from the stop to the width: 3 pings, about 4.5 s for the scan |
| Beam pull-in 6° | until 20 Nov | The 10 cm box read 116 and 156: with 12° the same scans give 83 and 118. It was fitted on two scans; the readings since are mostly 95-125 |
| Stop distance 250 mm | until 15 Nov | It stopped too far away: 185, 190, then 175 (stops at 150-162 mm, no margin), then 180 |

**The bypass:**

| What | When | What happened, and why |
|---|---|---|
| Plan for at least 200 mm | until 20 Nov | Team: go round tighter. At least 150 mm, which passes the 10 cm box about 25 mm closer |
| A fixed rectangle, looking for black only near where the line should be | until 8 Nov | In the bypass simulator it found the line in 36 of 486 cases: a few degrees of spin error moved the line out of its window. The new way found it in 471, with no contact |
| Side margin 50, then 70 mm | 7-8 Nov | The barcode arm sticks out on the left, and with spins 5 % off the robot passed a 200 mm box 21 mm away at 70 mm. Now 100 mm |
| Looking for the line on the leg past the box | 8-17 Nov | 8 Nov: a junction's side line beside the box was followed away. Later it watched only past the box's back, but on 15 Nov that was a different piece of track beside the box. Since 17 Nov the leg past never takes a line |
| "Side look": the sonar turned 40° toward the box on the leg past, to hear when it is really past | 8 Nov | Built, then **switched off** (`S5_SERVO_SIDE_DEG` 0): on the robot it did not hear the box's side at 40°, only its front corner. The box is about 13 cm deep, inside the assumed depth |
| After the bypass, skip the turn if a sensor already saw black, else stop at the first tape of a 170° spin | until 20 Nov | On track 4 it followed other tape beside its line. Now it always turns back to the old direction, like a junction turn |
| The barcode sensor live straight after a bypass | until 20 Nov | B crossed the line's own tape at an angle, read it as bars, and held the steering straight, so the robot drove diagonally off the line. Now ignored for 200 mm |
| Assumed box depth 150 mm | until 21 Nov | It went further behind the box than needed: 120 mm. **Not run on the robot yet.** In the simulator, boxes up to 120 mm deep: 36 of 36 with no contact |
| Going round a second obstacle met during the bypass | - | **Not built.** The watch stops the robot safely instead. Planning a second detour, and keeping track of the line after two detours, was too much to build and test before the demo |

**Mission:**

| What | When | What happened, and why |
|---|---|---|
| A letter with no mapping stops the robot at the junction | until 20 Nov | Team: carry on instead. It is now reported and ignored |
| A junction just after a barcode | 8 Nov | The letter came after the junction and was used at the next one. The junction now waits for a read in progress |
| An obstacle in the middle of a barcode | 8 Nov | Backing up garbled the read: the read is now dropped and reported |
| A newer barcode cancels an older waiting move | 20 Nov | Rejected: a decoy barcode could cancel the real one |
| 135 mm/s | 15-18 Nov | Never ran on the course: back to 120 |

## 10. Not done, and known limits

- **One stop at 131 mm** (20 Nov), below the 150 mm target. Every other
  stop was 150-165 mm.
- **Not run as formal tests:**
  - `no clear side` (S5, boxes or a wall on both sides);
  - a leg blocked on purpose;
  - the sonar covered during a bypass (it must stop with "ultrasonic not
    answering").
- **The "box still beside" extension** has never been triggered on the
  robot: the course box is shallow.
- **The 21 Nov change** (assumed depth 120 mm) has not run on the robot.
- **The whole-course test** SYS-01 (3 full runs) is not recorded as such;
  full sequences ran on 15 and 20 Nov.
- **Limits:**
  - the bypass assumes nothing else is beside the obstacle, and a second
    obstacle stops the robot;
  - a line that turns 90° away right behind the box is not found;
  - other tape between the robot's path and its own line could be taken
    for it;
  - the sonar's position (120 mm ahead of the axle) is not measured;
  - round or soft objects reflect badly.
- **Keep obstacles at least 30 cm after a barcode.** A stop mid-code drops
  that read.

## 11. Questions you may get

- **How does it choose a side?** The side that is clear beyond its edge;
  if both are, the nearer edge, because there is less to drive. If
  neither is, it stops.
- **How is the width worked out, and why the ±?** Each ray gives a point
  (distance x sin(angle)). The edges are where the rays leave the face,
  pulled in for the beam's width. The ± comes from the 5° step and the
  beam, combined for both edges.
- **Why does it plan for at least 150 mm, whatever it measured?** The
  instructor's brief says 10-20 cm, and a scan can read narrow. The plan
  is also capped at 300 mm, because one wrong scan read 500 mm and the
  detour went into the room.
- **What if there is another obstacle in the way?** The watch is on for
  every leg: two pings under 150 mm stop the robot (`something in the
  way during the bypass`). It does not plan a second detour.
- **How does it find its own line again?** It drives back toward the line
  until any line sensor sees black. Then it turns back to its old
  direction like a junction turn, slowly near the end until a sensor finds
  the tape. The heading after a bypass was within 1-4° on 20 Nov.
- **Why stop at about 15-16 cm?** Tribunal 1 requires at least 150 mm.
  From there the scan still sees both edges of the course box. The robot
  stops about 15-25 mm inside the 180 mm trip.
- **What happens with a barcode letter that has no mapping?** It is
  reported and ignored, and the robot carries on. A mapped letter that is
  already waiting is kept.
- **Why do the turns end on the tape instead of at an exact angle?** A
  plain 180° spin came out about 190° on the mat. Finding the tape makes
  each turn land on the line whatever the spin error.

## 12. Where the code is

| File | What is in it |
|---|---|
| [`mission.c`](../../app_program/subsystem5/mission.c) | `mission_task`; `start_run()`, `on_barcode()`, `on_junction()`, `turn_onto_line()`, `on_obstacle()`, `pass_box()`, `drive_until_line()`, `turn_back_clear()` |
| [`obstacle.c`](../../app_program/subsystem5/obstacle.c) | `obstacle_task` (the forward watch), the echo interrupt, the servo, `obstacle_scan()`, the edge and width rules, `obstacle_combine()` |
| [`subsystem5.c`](../../app_program/subsystem5/subsystem5.c), [`subsystem5.h`](../../app_program/subsystem5/subsystem5.h) | the event flag, the buttons, the STOP interrupt; trip distance, servo and beam settings |
| [`subsystem5_test.c`](../../app_program/subsystem5/subsystem5_test.c) | the Stage 3 tests |
| `tools/sim/bypasssim.py`, `tools/logcheck/sonarcheck.py` | the bypass simulator; the width rule replayed on a robot log |
