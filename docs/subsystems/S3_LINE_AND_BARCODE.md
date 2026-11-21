# Subsystem 3: Line following, junctions and barcodes

**Owner:** B3. **Code:** [`app_program/subsystem3/`](../../app_program/subsystem3/)
(`line.c`, `barcode.c`). **Tests:** Stage 2 (`images/is34_test_stage2.uf2`:
2A line, B00 barcode driving straight, 2B barcodes while following, 2C
junction moves). The state on 21 Nov 2026. The whole robot: [README.md](README.md).

## 1. In one minute (what to say)

"Subsystem 3 is the robot's eyes on the floor.

- **The line:** two IR sensors straddle the 20 mm tape, one each side, so
  both read white when the robot is centred. When one sees black, the
  Line task steers toward it. A steering memory keeps the robot turning
  through curves.
- **Junctions:** both sensors black at once is a crossing. A left arm is
  seen by the barcode sensor, which sits further left.
- **Barcodes:** the barcode sensor's edges are stamped with the distance
  the sensor itself has travelled, from the wheel encoders. Bar widths are
  therefore distances, and the speed or steering over the code does not
  change them. The decoder reads Code 39 in both directions. It can repair
  a few typical faults, but only when exactly one letter fits, so it
  never guesses.
- **The mapping:** this subsystem keeps the 4-letter mapping that comes
  from the laptop."

## 2. Why it exists

**The course briefs (TEAM.md 7.1, B1-B8):**
- decode Code 39, A-Z at least, in both directions;
- measure bar widths in encoder counts, not time;
- slow down over barcodes;
- keep the letter for the next cross-junction;
- no mapping built in: it arrives over MQTT.

**Requirements (Tribunal 2 list):**
- FR1: follow the line at 120 mm/s;
- FR2: read barcodes in either direction, repair only when exactly one
  letter fits;
- FR11: detect cross- and T-junctions; the latest unused letter is used
  at the next one, and none means straight;
- FR8, with S5: find the line after a turn or a bypass;
- NFR1a: no line lost over 3 m;
- NFR4: a barcode at least 200 mm before its junction is decoded in
  time;
- NFR8: never a wrong letter (a doubtful read gives no move).

## 3. What it owns, and what it measures

| Sensor | Where | What it gives |
|---|---|---|
| Left line sensor L (digital IR) | GP1, just left of the tape | 1 = black |
| Right line sensor R (digital IR) | GP0, just right of the tape | 1 = black |
| Barcode sensor B (IR, **analogue** since 7 Nov) | GP28 (ADC input 2), powered from 3V3; 85 mm left of the robot's centre, 40 mm behind L and R, 70 mm left of L | a reading from 0 (white) to 4095 (black), sampled 2,000 times a second |

- The line sensors are **115 mm in front of the wheel axle**, which
  matters for where the robot spins at a junction.
- The L-R gap is about 30 mm for the 20 mm tape: both read white when
  centred, with a few mm to spare each side.
- **What it measures:**
  - where the line is: left, right, centred, or a cross line;
  - junctions and where their centre was (odometer reading);
  - the barcode elements' widths in mm, and the letter.

## 4. Tasks and interrupts

| Task / interrupt | Priority | Runs | Does |
|---|---|---|---|
| `line_task` | 8 | every 20 ms | sensors to steering and speed share; junctions; line lost; stuck sensor |
| `barcode_task` | 10 | every 10 ms | takes the edges from the ring, finds the end of a code, decodes it, publishes the letter |
| ADC FIFO interrupt | - | 2,000 samples / s | turns each barcode reading into black / white edges, stamped with B's travel |

## 5. Line following (`line.c`)

### 5.1 The straddle rule (since 7 Nov)

| L | R | Means | Does |
|---|---|---|---|
| white | white | centred, or a gap | straight (plus the steering memory) |
| black | white | the line moves left | turn left |
| white | black | the line moves right | turn right |
| black | black | a cross line | straight (a crossing) |

### 5.2 Steering

- **A correction:** 30 mm/s between the wheels, at once. The robot drops
  to 60 % of its speed, and to 40 % after 50 mm of correcting, so the same
  steering turns tighter.
- **The steering builds up** by at most 10 mm/s per 20 ms and eases off at
  once. The sensors are acted on as read (1 reading).
- **Steering memory:** the steering averaged over the last 60 mm.
  - While both sensors are white, the robot keeps 40 % of it (at most
    40 mm/s).
  - On a straight the corrections alternate left and right, so the memory
    stays near 0.
  - On a curve it settles at the turn the curve needs, so the robot keeps
    turning between corrections instead of driving straight off.
  - It also slows a little in curves: 0.5 % per mm/s of memory.
- **Overshoot:** on a tight curve the tape can pass right under a sensor
  and out the other side, which also reads "both white". After a
  correction and then 60 mm of white, it turns back toward the side that
  saw the line last, at 30 mm/s and 20 % speed (LINE_SEARCH).
- **A long straight, all white** looks the same as no line at all. After
  200 mm without touching the tape it steers very gently (12 mm/s)
  toward the side it saw last for 70 mm, then the other way for 140 mm.
  A sensor touching the tape ends this.
- **Line lost:** the probe found nothing either way, or 250 mm / 4 s of
  searching after an overshoot. Mission stops, "line lost". A line that
  ends is driven past by about 400 mm before the stop.
- **Stuck sensor:** one sensor black for 300 mm of driving, or both for
  150 mm, gives LINE_FAULT, and Mission stops and says which. A robot
  standing still never trips this.

**How it was tuned (7 Nov).** A simulator of the robot
(`tools/sim/linesim.py`: track, sensor position, motor lag) was
calibrated until it reproduced that day's three robot runs. Then 600
settings were tried on 480 scenarios: curves of 100-130 mm, the S-bend,
straights, narrow sensor windows, motor lag, wheel mismatch. The old
settings passed 297 of 480, these pass 478 of 480, at the same speed.

### 5.3 Junctions

- **Crossing** (an X, a crossroads, a T end):
  - L and R both black for 12-45 mm: one tape width, longer when crossed
    at an angle;
  - and the robot was **driving straight** for the 20-40 mm before it.
    On a tight curve the robot can sweep across its own line at an angle,
    which gave 2 false crossings on 7 Nov.
- **A crossing while correcting (8 Nov):** both black for 10-45 mm **and**
  B then sees a patch about a tape wide (9 mm or more) within 25 mm of the
  same place: B is seeing the crossing's left arm. This counts even while
  the robot corrects. On 8 Nov the robot zigzagged, the straight-drive
  rule threw every junction away, and it drove on through the junction
  after reading a U-turn barcode.
- **Left branch** (an arm to the left only):
  - B sees a black patch 12-28 mm long, about one tape width;
  - the robot was driving straight at that place (the corrections are
    kept by distance and looked up when B's patch arrives);
  - **the patch is alone** (20 Nov): the decision waits until B is 30 mm
    past the patch, and it is dropped if the barcode reader has seen 2 or
    more elements there. A barcode's wide bar looked like an arm before.
- **A branch to the right only** cannot be told from a correction (R
  touches it like a curve). The track has none.
- Two junctions within 40 mm are one. Mission is told through the event
  flag, with **where the line sensors crossed the junction centre**, so it
  can drive the wheels onto that centre before it turns.

## 6. Barcodes (`barcode.c`)

### 6.1 From the sensor to edges

- **Sampling:** the ADC samples B 2,000 times a second in its interrupt.
  The white and black levels are learned as the robot drives.
- **Relative edges:**
  - it turns black once the reading has risen 700 above the lowest
    reading since the last bar, and it is at least 1500 (`S3_BAR_DELTA`,
    `S3_BAR_BLACK_MIN`);
  - it turns white again once the reading has fallen 700 below the
    highest reading since the bar began;
  - why relative edges: a fixed threshold cannot see both a narrow bar
    (it peaks low through the blurred spot) and the narrow space between
    two wide bars (it never gets back to white).
- **Each edge is stamped with B's own travel** (`b_odometer`, 20 Nov):
  - the robot's travel, less the heading change x 85 / 123, worked out
    from the two wheels;
  - B sits 85 mm left of the centre, so while the robot turns, B moves
    slower or faster than the centre;
  - with B's own travel, steering over a code does not distort the widths.
- **Flicker under 1.2 mm is merged:** the sensor chatters at its
  threshold.
- Each edge keeps only the low 16 bits of the count (Change Requirement 1,
  Tribunal 1's agreed encoder interface). The task widens them back.

### 6.2 From edges to a code

- A code **ends after 60 mm with no bar** (`S3_QUIET_MM`, the Code 39
  quiet zone for a 6 mm print).
- A run of 20 or more elements is decoded; shorter runs are noise
  (printed from 5 elements up for the log).
- **Code 39 rules:**
  - a code has 10 x characters - 1 elements, at least 3 characters (`*`,
    the letter, `*`);
  - in each character the 3 widest of the 9 elements are wide, and wide
    must be at least 1.3 x the narrow;
  - the first character must be `*`; if it is not, the elements are
    reversed and decoded again, so codes read backwards are fine;
  - `*X*` is looked for starting at any bar of the run, so a stray mark
    just before or after a code (a tile join) does not spoil it.

### 6.3 Repairs (only when the clean decode fails, and only if exactly one letter fits)

Every repair was checked on the PC with the robot's own C decoder on all
the logged reads and on thousands of random bar patterns before it went
on the robot.

| Repair (added) | What it fixes | Evidence |
|---|---|---|
| One `*` damaged (8 Nov) | B started on the first bars: the other `*` and the letter must decode with wide at least 2 x narrow | The logs' good reads with a `*` replaced by noise: 988 right, 0 wrong |
| A lost bar put back (8 Nov) | blur merged two elements (27 elements): each element is tried as three, and the result must decode cleanly | Every narrow element of the good reads removed in turn: 117 recovered, 0 wrong |
| A missing end bar put back (8 Nov) | a code cut short by 1-2 narrow elements at an end | The logs' good reads cut short: 33 right, 0 wrong |
| A cut start (15 Nov) | the first 3-4 elements of the opening `*` never seen | 3,870 cut codes read, 0 wrong; 2 false in 50,000 random patterns |
| Thin bars widened (20 Nov) | through B's blurred spot every bar reads narrower and every space wider by about the same amount: the bars are widened and the spaces narrowed by half the difference of their medians | The 21 distinct failed reads in the logs: 2 letters, both right; 20,000 random patterns: none |
| Bars and spaces judged apart (20 Nov) | each kind gets its own narrow / wide cut | 1 more letter (an F) over the logged pieces; random patterns: 1 in 100,000 |
| Half a code (20 Nov) | a letter with one `*`, the other `*` never seen (19+ elements); also with its `*` cut short (2 narrow elements put back at each end) | **Used only when the letter is mapped and both characters are clean** (every wide at least 1.4 x the widest narrow, `HALF_RATIO_X10`); otherwise reported `-> NOT USED`. Random junk is used 0.037 % of the time |

Each repaired read says so in its event, e.g. `barcode F (bars read thin,
widened) (narrow 2.9 mm, wide 8.0 mm) -> U-TURN at next junction`. A read
that cannot be decoded is `READ_FAIL`, and its widths are printed to the
log so it can be looked at afterwards.

### 6.4 What else the reader does

- **`barcode_busy()`:** B is inside a code, so Motion slows to 50 mm/s.
- **`barcode_in_bars()`:** B saw a bar within the last 60 mm, so Motion
  holds the steering straight (subsystem 2).
- **After a bypass, B is ignored for 200 mm** (20 Nov, set by Mission):
  the robot is still turning onto its line, and B crossed the line's own
  tape and read it as bars.
- **The mapping:** `barcode_set_mapping()` checks it (4 pairs, each
  letter and move once) and keeps it under the lock;
  `barcode_move_for(letter)` gives the move, or none.
- **Resets:** the reader is reset at START, after every spin, and when an
  obstacle stop happens in the middle of a code (that read is dropped).

## 7. Numbers to know

| Value | Setting |
|---|---|
| Correction | 30 mm/s at once; 60 %, then 40 % after 50 mm; 20 % searching (`STEER_CORR`, `S3_CORR_PCT`, `S3_LOW_PCT`, `S3_SEARCH_PCT`) |
| Steering memory | over 60 mm, 40 % kept, at most 40 mm/s (`S3_MEM_MM`, `S3_MEM_K`, `S3_MEM_MAX`) |
| Overshoot search after | 60 mm of white (`S3_OVERSHOOT_MM`) |
| Probe after | 200 mm of white; 70 mm, then 140 mm (`S3_PROBE_AFTER_MM`, `S3_PROBE_MM`) |
| Lost after searching | 250 mm or 4 s (`S3_LOST_MM`, `S3_LOST_MS`) |
| Stuck sensor | one black 300 mm, both 150 mm (`S3_STUCK_ONE_MM`, `S3_STUCK_BOTH_MM`) |
| Crossing | both black 12-45 mm, straight 20-40 mm before; while correcting, 10-45 mm plus B's patch within 25 mm |
| Left arm | B patch 12-28 mm, alone (decided 30 mm after it, `S3_ARM_CLEAR_MM`) |
| Barcode edges | rise / fall 700, black from 1500 (`S3_BAR_DELTA`, `S3_BAR_BLACK_MIN`) |
| End of a code | 60 mm without a bar (`S3_QUIET_MM`) |
| Wide vs narrow | at least 1.3 x (`S3_WIDE_RATIO_X10`); half codes at least 1.4 x (`HALF_RATIO_X10`) |
| Elements decoded | 20 or more (19 for a half code) |
| B ignored after a bypass | 200 mm (`REJOIN_BAR_MM`, in mission.c) |

## 8. Shown on the robot

**Line following:**
- **7 Nov**, the simulator-tuned version: "works extremely well" (team),
  the S-bend included.
- **9 Nov** (Change Requirement 1 build): 1,505 mm through left and right
  curves with no line lost, the X crossing detected, no false junctions.
- **15 Nov:** 7 course runs at 120 mm/s with no line lost.
- **20 Nov, track 4:** **3.65 m** (run 2) and **4 m** (run 6) of following
  with no line lost (NFR1a).

**Junctions:**
- **9 Nov:** the X crossing detected.
- **15 Nov:** every junction turned LEFT as mapped (run 6).
- **20 Nov:** turns from barcodes at crossings, and after the lone-patch
  rule no false "branch left" (run 10).

**Barcodes:**
- **7 Nov:** "A small" decoded driving straight (S3-B00), read backwards
  at 120 mm/s, and read while following.
- **8 Nov:** F read in both directions; the first U-turn from a barcode.
- **15 Nov, course:** F read cleanly, and LEFT at the junction (runs 5-7);
  A after the mapping changed (run 6).
- **20 Nov, track 4:**
  - Q, Z and F read with no mapping (run 7);
  - F to LEFT done (run 7);
  - Q and F read (run 8); Z read (run 9);
  - F to LEFT (run 10);
  - **A to LEFT, Q (thin-bar repair) to FORWARD, small F to U-TURN**
    (run 11);
  - Z to RIGHT (run 13).
- Best session: 7 of 12 passes read (20 Nov, the Tribunal 2 NFR8 figure).
- **No wrong letter has ever been acted on.** The one wrong letter
  anywhere: replaying all the logged pieces on the PC gave a half code
  "B" where the track had no B. B was not mapped, so it was only
  reported, which is why an unmapped or unclean half code is never used.

## 9. Tried, changed or dropped, and why

**Line following:**

| What | When | What happened, and why |
|---|---|---|
| Edge following (one sensor on the tape, one beside it) | 1-2 Nov | It followed about 2 m, but corrected unevenly and turned the wrong way when the tape sat between the sensors. Replaced by the straddle rule (7 Nov) |
| Hard steering at once | 7 Nov | A continuous zigzag (both sensors black on every swing) |
| Gentle steering, then hard | 7 Nov | Good (the "third run"), but the overshoot search fired on straights |
| Two readings must agree | until 7 Nov | Every correction came 20-40 ms late: 1 reading since |
| The IR mount raised | 7 Nov | The sensors saw only the middle of the tape and lost curves. Line following was reset to the third run's settings, then re-tuned in the simulator |
| A heading rule for left branches | 7 Nov | Undone with that reset |
| A crossing needs a straight approach, only | 8 Nov | With the robot zigzagging every junction was thrown away, so the B pairing rule was added |
| A left arm from any tape-wide B patch | 20 Nov | A barcode's wide bar (12-17 mm) looked like an arm: two false junctions over a barcode. Now the patch must be alone |
| The right-only branch test (S3-L04) | 7 Nov | Dropped: there is none on the track, and it cannot be told from a correction |

**Barcodes:**

| What | When | What happened, and why |
|---|---|---|
| Back up and re-read slowly | 2-7 Nov | It worked on 2 Nov, but on 7 Nov a false bar-space-bar made the robot back up and the read never ended. Removed at the team's choice: one pass |
| The digital output with the module's own threshold | until 7 Nov | Printed bars read grey to the IR: the analogue output with thresholds set in software |
| Fixed thresholds 2500 / 1700, then 1300 / 800 | 7 Nov | They lost thin bars or merged wide ones. Relative edges: "A small" in simulation 54 of 54, against 24 and 6 of 54 |
| Wide vs narrow at least 1.5 x | 7 Nov | Blur lost 4 reads in 18 in simulation: 1.3 x (still 0 of 20,000 random patterns) |
| Thresholds 700 / 300 (more sensitive) | 15 Nov | Worse while driving: thin bars and spaces ran together into 6-7 mm blobs. Back to 1500 / 700 the same day |
| Thresholds 1000 / 450 (a test image) | 20 Nov | It failed often in the team's runs: not used |
| The end of a code after 40 mm without a bar | until 20 Nov | Two lost thin bars side by side ended a code mid-way. Now 60 mm |
| Widths from the robot's centre travel | until 20 Nov | Turning over a code distorted them: now B's own travel |
| Half codes reported only | 20 Nov | A's start was clipped pass after pass (placement), so A was never used. Now used when mapped and clean, from 1.6 x, then 1.4 x (an A at 1.4-1.6 was refused and an older barcode's move used) |
| Putting a lost bar back anywhere | 20 Nov, PC | Without a width check, a damaged A could come out as B, A, 1 or 8: refused, because it would be a guess |
| Matching whole codes to templates | 20 Nov, PC | Too ambiguous (A against U, 1, K): not used |
| A newer barcode cancels an older waiting move | 20 Nov | Rejected by the team: a decoy barcode could cancel the real one |
| "A's ink is faint to the IR" | 20 Nov | First suspected; the logs then showed A's far end reading cleanly and its start clipped: placement, not ink |

## 10. Not done, and known limits

- **A barcode is not read on every pass.** It breaks into pieces when:
  - it is crossed at an angle or near its ends;
  - it is right after the hump, with the robot still rocking;
  - its start is clipped because the robot was set down too close to it.

  What worked on 20 Nov: barcodes flat and square on straights, away
  from the hump, and the robot set down centred and straight at least
  10 cm before a code.
- A code that is missed is not read again: Mission goes on as if there
  was no barcode.
- **The IR sensors' pots** shift when the robot is handled. Check the live
  view before every session; tile grout can read black.
- **Not detected:**
  - a line sensor stuck white or unplugged: that side never corrects,
    the robot drifts off and stops with "line lost";
  - right-only branches;
  - a branch on a curve is ignored.
- **Keep apart:**
  - at least 40 mm of white between a left branch and a barcode,
    otherwise they read as one;
  - other tape at least 70 mm from the robot's path (B could take it for
    an arm);
  - an obstacle at least 30 cm after a barcode.
- **Tests short of their pass mark:**
  - S3-L02, the junction count against the real count, was not counted;
  - the Stage 2 tests S3-L03 and L05-L10 were not all run on the final
    code (the mission runs cover most of them).

## 11. Questions you may get

- **Why straddle the tape instead of riding on it?** With both sensors
  beside the tape, "both white" means centred, one black says which way
  the line went, and both black can only be a cross line. Riding on the
  tape could not tell left from right well.
- **How do you follow curves with on/off sensors?** The steering memory
  keeps the turn a curve needs between corrections, the robot slows while
  correcting, and an overshoot search turns back if the tape slips past
  a sensor.
- **Why can't you see a right-only branch?** R touches it exactly like a
  curve to the right. A left arm is seen by B, which sits further left.
- **Why measure bars by distance?** The encoders stamp each edge, so the
  width is the same at any speed (TEAM.md B3). B's own travel is used, so
  turning does not change it either.
- **How do you read a code backwards?** If the first character is not
  `*`, the elements are reversed and decoded again.
- **What if a bar is missed?** A few typical faults are repaired, but only
  when exactly one letter fits. Each repair was tested on all the logged
  reads and on random patterns with no wrong letter. Otherwise the read is
  `READ_FAIL`, and the robot goes straight at the junction.
- **Why does it slow down over a barcode?** The sensor is not very
  sensitive, and bars read reliably only slowly and straight across:
  50 mm/s.
- **What does "half a code" mean?** One `*` was never seen, usually
  because the robot started too close. The letter is used only when it is
  in the mapping and cleanly printed; otherwise it is only reported.

## 12. Where the code is

| File | What is in it |
|---|---|
| [`line.c`](../../app_program/subsystem3/line.c) | `line_task`: steering, memory, overshoot, probe, lost, stuck; junctions (`junction_at()`, the B pairing, the lone-patch rule) |
| [`barcode.c`](../../app_program/subsystem3/barcode.c) | ADC sampling and relative edges (`adc_sample()`), B's odometer, `barcode_task`, the decoder and its repairs (`decode_chain()`, `decode_thin()`, `decode_kinds()`, `decode_half()`), the mapping |
| [`subsystem3.h`](../../app_program/subsystem3/subsystem3.h) | pins, thresholds, geometry, junction sizes |
| [`subsystem3_test.c`](../../app_program/subsystem3/subsystem3_test.c) | the Stage 2 tests (IR live view, S3-B00 bench profile, 2A, 2B, 2C) |
| `tools/sim/linesim.py`, `tools/logcheck/barcodecheck.py` | the line simulator; the decoder replayed on a robot log |
