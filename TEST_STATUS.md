# P5-IS34 test status checklist

Last updated: 6 Nov 2026. How to run each test:
[TESTING_GUIDE.md](TESTING_GUIDE.md). Formal results:
[testing/](testing/) `TEST_LOG.md` files. What the robot does, its limits
and everything still open: [ROBOT_BEHAVIOUR.md](ROBOT_BEHAVIOUR.md).

Legend: ✅ passed on the robot · 🟡 works, but not to the pass mark yet ·
⬜ not tested yet · ❌ tested and failing

## Summary

| Stage | What | Owner | Status |
|---|---|---|---|
| 0 | RTOS timing, all 8 tasks | B1 | ✅ passed 26 Oct |
| 1 | Motion + STOP | B2 | ✅ passed 1 Nov |
| 2A | Line following + junctions | B3 | 🟡 follows well (7 Nov, third run); 3 m, X crossing and left branch not shown yet |
| 2B | Barcodes | B3 | 🟡 partial reads, no full decode yet; S3-B00 straight read to run next |
| 2C | Junction moves from barcodes | B3 + B5 | ⬜ needs 2B |
| 3 | Obstacle: sonar, servo, width, bypass | B5 | 🟡 servo + sonar work; width and bypass not shown |
| 4 | IMU + ramp height | B4 | 🟡 2 of 7 runs within ±5 mm on the 10 mm hump; hump not held down |
| 5 | Telemetry + MQTT | B1 | ✅ passed 2 Nov (START/STOP 3 of 5 rounds) |
| 6 | Whole course | all | ⬜ |

## Stage 0: RTOS timing (`make TEST=1`)

- [x] ✅ S1/S2/S3/S4/S5-U01..U04, INT-01, INT-02: every task wakes on time,
  no overruns (26 Oct, `testing/integration/logs/2026-10-26_stage0_test.txt`)
- [x] ✅ Kernel tick fix (`0786f1a`): after a power-on or RUN reset the system
  timer ran about 43x slow. Fixed 2 Nov. **Everyone: pull before testing.**

## Stage 1: motion + STOP (`TEST_STAGE=1`)

- [x] ✅ S2-M05 wheel directions and encoder signs (wheels lifted)
- [x] ✅ S2-M06 STOP with the wheels lifted
- [x] ✅ S2-M03 500 mm at 150 mm/s: measured about 500 mm
- [x] ✅ S2-M02 turns: with `S2_TRACK_MM` 123, 90° about right, 180° about
  5° over (limit ±15°)
- [x] ✅ S2-M04 STOP on the floor

Still to do: write the Stage 1 rows into `testing/subsystem2/TEST_LOG.md`.

## Stage 2: line, junctions, barcodes (`TEST_STAGE=2`)

### What works
- [x] ✅ IR live view: S, C and B all respond; B prints edges per second
  so you can tune its pot
- [x] ✅ Steering direction (1 Nov): GP0 is the RIGHT sensor, GP1 the left
  (now `PIN_IR_LINE_R` / `PIN_IR_LINE_L`)
- [x] ✅ Line following (old edge following, 1-2 Nov): about 2 m straight
  without losing the line, with side to side. **Replaced 7 Nov by the
  straddle logic: re-run S3-L01 with it.**
- [x] ✅ Barcode noise filter: ignores sensor flicker and single grout lines
- ~~Rescan (back up 60 mm, read again slowly)~~: worked on the robot 2
  Nov, **removed 7 Nov** at the team's choice; codes are read in one pass
  with the steering held straight over the bars.

### Not done yet
- [ ] 🟡 **S3-L01**: at least 3 m with no line lost. Runs stopped at
  1.2-2.1 m only because the track was too short. **Needs a 3 m line.**
- [ ] 🟡 **S3-L02** junction count = real count. Side-to-side movement can
  give false junctions; count the real ones and compare.
- **7 Nov run (straddle, first version), `linefolo` log:** B analogue
  works (white 250-570, black ~4040). L01 started with a rescan (tape
  left under B after the live view) and was stopped at 375 mm. Right
  curves held; **left curves lost**: L saw the tape only 21-27 mm, the
  gentle first steering at 150 mm/s let it slip out past L, and the
  overshoot search needed 30 mm. L06 (right curve) gave 2 **false
  crossings** (the robot swept across the line diagonally). L05 PASS (1
  crossing). L10 and L03 no junction. Also: one `IMU read failed`, and a
  `ramp peak 8.4 mm` while following on foam (tile edges look like humps:
  check in Stage 4). **Fixed the same day, not yet re-run:** hard steering
  at once with 50 % / 25 % speed, overshoot search after 15 mm, crossings
  need a straight approach, barcode reader reset at every start, follow
  speed 120 mm/s, fixed B thresholds 2500 / 1700.
- **7 Nov, second run (hard steering at once):** zigzag. The trace
  repeats R, L+R, L, 1 mm white, R ... about every 40 mm: never centred,
  both sensors black on every swing (heading swinging about ±40 deg). S3-L01
  1761 mm, no line lost; 1 left branch counted (false). **Retuned the
  same day, not yet re-run:** gentle for 15 mm at full speed, then hard
  at 60 % / 35 %, steering ramp 10 per 20 ms; runs now print a wobble
  count (corrections per metre, sweeps across the tape).
- **7 Nov, third run (gentle 15 mm, then hard and slower), `linefolo`
  log: good, the team is happy with the following.** About 10
  corrections per metre, 2-4 sweeps across the tape per run (was a
  continuous zigzag). L01 890 mm, no line lost (stopped by hand, FAIL
  only on the 3 m). **L05 PASS** (1 crossing), **L06 PASS** and **L07
  PASS** (tightest right and left curve, no junction, no line lost),
  **L08 PASS** (gap). L10 (X crossing) FAIL: 450 mm, no junction seen;
  L03 skipped (no left-only branch); L09 not finished. B on analogue:
  white 165-200 on this surface, black 2900-3945. On the L07 left curve B
  saw 8-10 mm patches (tile joins?) and one false `bar-space-bar`: the
  robot backed up and then drove straight for the rest of the run (the
  read never ended). **Changed the same day:** the back-up-and-reread is
  removed; codes are read in one pass with the steering held straight
  only while B is over bars.
- **7 Nov, after the IR mount was raised (`scuffed abit` log): the line
  sensors no longer see the tape properly.** L05 (T end, passed twice
  before) FAIL: L saw the T bar for 4 mm, R never; L01 lost the line at
  557 mm after the tape went from L to R in 5 mm (the sensors only see
  the middle of the tape now); 2B runs ended LINE LOST / LINE FAULT. **Re-
  tune the L and R pots at the new height** (each must read 1 over the
  whole 20 mm tape and 0 over white, live view), or lower the mount.
  B: whitest 164, darkest 4083, fine. B thresholds raised to 700 / 1500
  (team). The robot's own IMU reported ramps of 22 and 248 mm during 2B
  (robot lifted / handled): ignore. S3-L10 (X crossing, the track has
  them) failed again: both black for only 1 mm; retest once L/R are
  re-tuned.
- **7 Nov, after re-tuning the pots (`linefolo` log, last part):** L01
  followed 3303 mm (a first) but shaky: 9 corrections per metre, **10
  sweeps across the tape**, then both black (FAULT) at the end. L05 PASS,
  L06 PASS (17 corrections per metre). L07 FAIL: 2 false left branches
  on the curve (B over the curve's own tape). L03 FAIL: the real branch
  (B patch 17 mm) rejected because the robot was correcting. L10 FAIL:
  no crossing. Found in the trace: **the overshoot search fired on
  straights** (after an ordinary 15-20 mm correction and 40 mm of white
  it turned hard back and swept across the tape), and the 2-reading
  filter made every correction 20-40 ms late. Changed: search only after
  a correction that looks like a curve (35 mm, or 15 mm twice on the
  same side); filter 1 reading; left branch = heading turned under 30 deg
  in the 200 mm before it (encoders), instead of "no correction just
  before".
- **7 Nov, next run: worse on a left curve.** L corrected 3 times
  (17-31 mm), R touched once mid-curve (so "twice on the same side"
  failed), the tape slipped out past L and the robot drove 300 mm
  straight off the curve; a false left branch (B crossed the curve tape
  while the robot drove straight) and a false crossing (the probe found
  the line at a slant). **Changed the same day, not run yet:** the search
  rule is back to "a correction of 15 mm" (it handled the curves in the
  third run); the probe counts as turning for the crossing check; follow
  speed 120 -> 100 mm/s. Filter 1 reading and the heading rule for left
  branches stay. The line sensors still see the tape late since the
  mount was raised: line following was best before that (third run).
- **7 Nov, last line run (sensors at their lowest after the mount was
  raised): drove off the left curve again.** L saw the tape for only
  6-30 mm per correction (38-61 mm in the third run on the same curve),
  so the robot never turned enough. The mount cannot go lower. **Line
  following reset to `9d10fa6`** (the third run: L05-L08 PASS), at the
  team's request: task_line.c as it was and 120 mm/s. Undone with it:
  the straight-line probe (false LINE LOST on long straights comes back),
  the 1-reading filter, the heading rule for left branches. Kept: the
  barcode reader (relative edges, one pass, steering held straight over
  the bars), WiFi log, Stage 4 test.
- **7 Nov, later: line following re-tuned in a simulation**
  (`tools/linesim.py`), calibrated until it reproduced the three real runs
  of the day (lost left curves at 150 mm/s, the zigzag, the good third
  run); the S-bend of track 3 (photo `docs/track/overshoot.jpg`) failed in it the
  same way as on the robot with a narrow sensor window. New: correction
  ±30 at once at 60 % / 40 %, steering memory (keeps 40 % of the last
  60 mm of steering while both white), overshoot search after 60 mm of
  white (±30 at 20 %), 1-reading filter, the straight-line probe back,
  120 mm/s. Simulated: 478/480 (9d10fa6: 297), unseen scenarios 177/216
  (89), no sweeps. **First robot run (`linefolo` log, last): "works
  extremely well" (team)**, the S-bend included; most guided runs were
  skipped (START then STOP). S3-L10 (X crossing) still not detected: both
  sensors black together for only 1-6 mm. **S3-B00 run 1 PASS: "A small"
  decoded A** driving straight (29 elements, narrow 3.0 mm, wide 8.2 mm;
  profile: narrow bars 2089-2552, white about 200, none nearly missed).
  The WiFi watchdog re-joined the broker after about 100 s without it
  (`no answer ... trying again`).
- [ ] ⬜ **S3-L10, L03, L05..L09** (straddle logic, 7 Nov): X crossing, left
  branch, T end, right curve, left curve, gap (track 2), stuck R (black
  card under R); one place at a time, each prints PASS/FAIL. S3-L04
  (right-only branch) dropped: none on the track. Track photos in
  `docs/track/`: track 3 (foam tiles) has tight curves (about 15 cm) and
  an X crossing, but no left-only branch or T end unless tiles are
  turned to make one.
- [ ] ⬜ **S3-B00** (new 7 Nov): barcode read driving **straight**, no line
  following: 4 runs at 50 mm/s (a code, the same code from the other end,
  "A small") and one at the follow speed 120 mm/s: with no reread, the
  first bars are met at that speed, so run 4 matters too. Prints the elements, the decode
  and (analogue) each bar's darkest and each space's lightest reading;
  `!` marks a bar that never reached black (`S3_BAR_ADC_ON`) or a space
  that never got back to white (`S3_BAR_ADC_OFF`). Run this before
  S3-B01..B03: it separates sensor and decoder problems from steering
  problems. Each Stage 2 section (2A, B00, 2B, 2C) now asks first:
  START = run it, STOP = skip it.
  - **7 Nov, first run over WiFi (`robot_log_20261107_1509`), run 1
    FAIL:** 17 bars and spaces instead of 29, with spaces of 24 and 30
    mm inside the code (a code has none over ~11 mm). Paper reads 170-220,
    but the printed wide bars peaked at only 2400-3300 (tape: ~4000), two
    of them under the 2500 threshold, and the narrow bars evidently never
    reached the profile's 2100 split. Bars came out narrow (2.2 mm narrow,
    6-8 mm wide; 3.3 / 10 expected). So the print is grey to the IR sensor,
    and/or B sits too high and blurs the bars. **Thresholds lowered to 1300
    / 800 the same day**; check B's height above the floor (TCRT5000 best
    about 2-4 mm) and whether the cards are inkjet (dye inks can be nearly
    invisible to IR; laser prints read black).
  - **7 Nov, second WiFi run, all with "A small":** run 4 (120 mm/s)
    decoded **A (backwards)**, and 2B run 2 decoded **A** while following.
    Runs 1-3 failed: run 1 had two 35 mm bars (a black floor line under
    B); run 2 read 27 elements because the narrow space between the two
    wide bars of the stop `*` never dropped below the threshold (the two
    bars merged into one 20.5 mm bar); run 3 missed narrow bars that
    peaked at only 1700-2200. A fixed threshold cannot see both (narrow
    bars peak lower than that space dips). **Changed the same day:**
    relative edges (500 from the last extreme, never black under 900) and
    the decoder's wide/narrow check 1.5 -> 1.3. Simulated "A small" with a
    blurred spot (sigma 1.2-1.6 mm, ink 65-100 %): fixed 2500/1700 24/54,
    1300/800 6/54, relative + 1.3 54/54; random widths still 0/20000.
    Beyond sigma 1.8 nothing decodes: B lower is the real fix.
  - **2B runs 1 and 2 stopped with LINE LOST on a good straight** (0 and
    1 corrections): with the tape centred between L and R, 400 mm of white
    looked like no line. **Fixed the same day:** after 200 mm of white it
    probes gently each way for the tape (see ROBOT_BEHAVIOUR.md). 2B run
    3: a crossing at 934 mm, then both black for 150 mm, LINE SENSOR
    FAULT (what is there on the track?).
  - **WiFi reconnect:** after a drop the robot printed `mqtt_client_connect()
    failed, err=-10` hundreds of times and did not come back (two retry
    paths fought; the first attempt never got an answer). Fixed the same
    day: one retry at a time, and an attempt with no answer in 15 s is
    dropped and started again.
  - The robot log over WiFi lost about 1 message in 20 (`[panel: N log
    messages lost]`): the robot dropped a message whenever lwIP's MQTT
    buffer was still full. Fixed the same day: it keeps it and sends it
    on the next poll. The heartbeat lines (every 5 s) are no longer
    printed.
- [ ] 🟡 **S3-B01..B03** barcodes: best run read 7 of the 29 bars of `*F*`
  correctly ("3.3 3.2 10.0 3.5 9.9 3.8 2.6" mm), then steering moved the
  sensor off the bars. Latest build drives straight during the slow read:
  **not tested on the robot yet.** Pass = the codes on the track decoded in
  order, both directions, including "A small".
- [ ] ⬜ **S5-B01 / S5-B02** (2C): right move at each junction, U-turn, stop on
  an unmapped code

- [ ] ⬜ Junction turns with the **measured** sensor-to-axle distance of 115 mm
  (the code had a guess of 60): it spins when the wheels are 115 mm past
  where the line sensors crossed the junction centre. Re-check on the
  robot (paused).

### Changed 6-7 Nov, not tried on the robot yet
- **Line sensors straddle the tape (7 Nov)**: L left of it, R right of
  it, about 30 mm apart, both white when centred = straight. Replaces
  the edge following (C on the tape, S beside it), which corrected
  unevenly and turned the wrong way when the tape sat between them.
- Crossing: L and R both black for 12-45 mm. The Stage 2 run prints
  `junction N at X mm: crossing` or `: branch left (seen by B)`.
- **Left junctions** from the barcode sensor B (40 mm behind, 70 mm left
  of L): a patch 12-28 mm long (tape 20 mm) where the robot drove
  straight before and after. It prints `B patch 19.8 mm (...)` for every
  first bar B sees (also grout and barcode start bars).
- Tight curves: gentle steering, then hard and slow after 25 mm black;
  overshoot (60 mm white after a 30 mm correction) turns back toward the
  last side seen.
- To check on curves: no `junction` lines on curves without branches;
  smoother than the edge following.
- After a barcode meaning L/R/U: 100 mm/s until the junction.
- Barcode sensor on analogue (optional, `S3_BARCODE_ANALOG 1`): power it
  from **3V3**, AO to GP28, flash `..._test_s2_analog.uf2`, send the
  `B analogue` lines over white, tape and grout.
- To check: does the barcode sensor see the left branches, and is there
  at least 40 mm of white between a barcode and the left branch after it?

### Unknowns to measure
- Measured 6 Nov: line sensors 115 mm in front of the axle, body 130 x
  150 mm, barcode arm sticks out 17 mm (see ROBOT_BEHAVIOUR.md)
- Still to measure: how far the IR/barcode bar sticks out in front
- Tape width: 20 mm. Barcode sensor B: 40 mm **behind** C, 70 mm left
  (measured 6 Nov)
- L to R gap: about 30 mm, tape centred between them (7 Nov)
- **IR mount raised (7 Nov, after the WiFi barcode runs):** redo the IR
  live view (L/R pots), S3-B00 and 2B, a short 2A check (S3-L01, L07),
  and re-measure B's position (`S3_BAR_AHEAD_MM`, `S3_BAR_LEFT_MM`) if
  it moved. B higher = blurrier bars: "A small" may stop reading.
- Real number of junctions on the test track

## Stage 3: obstacle (`TEST_STAGE=3`)

### What works
- [x] ✅ Servo on **GP14** (servo header 3). It was plugged into GP15
  (header 4) at first and did not move.
- [x] ✅ Servo direction: centre, LEFT, RIGHT correct (`S5_SERVO_LEFT_HIGH 1`)
- [x] ✅ Servo turns slowly now (2 deg per 20 ms): full speed broke the horn
- [x] ✅ Sonar live view: steady readings (about +/- 10 mm), follows a hand

### Not done yet
- **Changed 7 Nov, not run yet:** the bypass now backs up until the
  obstacle is about 350 mm away before scanning, and scans again 100 mm
  further back. From where the forward watch stops (200-250 mm) a 200 mm
  box, with the beam's ~30 deg width, filled the whole +-40 deg scan, and
  the two scans were combined as "a side is clear only if both scans say
  so": "no clear side" for the larger boxes. Now each edge comes from the
  scan(s) that saw past it. Side margin 50 -> 70 mm (barcode arm on the
  left). S5-O01 now places the box at 350 mm and reverses 100 mm.
- **7 Nov, first Stage 3 run over WiFi (`ultrason` log): the bypass
  (S5-O02) worked first time.** Stopped at 245 mm, scanned (scan 1 saw
  the box fill the sweep, 411 mm; scan 2 found the edges, 140 mm), went
  round on the RIGHT (out 214 mm, past 930 mm), found the line 123 mm
  after the last turn, "bypass done, following the line"; then line lost
  at a floor X junction (not the track). Servo directions and the sonar
  view fine. S5-O01 widths varied a lot (522, 218, 240, 302, 370, 435
  mm; real widths still to be sent): several scans heard the box across
  almost the whole +-40 deg (a flat face echoes straight back, and with
  open space behind nothing else answers first). Width test cut to 3
  tries the same day (team).
- **Changed 8 Nov, not run yet (bypass rejoin):** chosen in
  `tools/bypasssim.py` (486 simulated bypasses). The line sensors watch
  on the leg past the box, any black counts on the way back, and the
  robot spins toward the old direction until a line sensor finds the
  tape instead of a fixed 90 deg turn. Old way 36/486 (straights 24/54:
  spin errors moved the line out of its 100 mm window), new 471/486, no
  contact with the box. Side margin 70 -> 100 mm.
- **7 Nov, second Stage 3 run (`ultrason` log 2):** the bypass part
  waited for a mapping that was not sent (START pressed many times; looked
  like a crash: fixed, it reminds every 10 s and START takes the TEST
  mapping). Then: a floor X junction made it lose the line once; the
  next bypass measured the box as **502 mm** (both scans heard it across
  the whole sweep), planned 445 mm out and stopped with **"bypass leg 4
  blocked"** (something in the room within 1 m). A spin came out about
  96 deg. Restarted there, it followed the floor's lines (START = follow
  from here). Box widths used: 20, 14.5, 15 and 23 cm.
  **Fixed 8 Nov, not run yet:** a ray only counts as on the box when its
  distance fits a flat face (front / cos angle); side-of-beam echoes are
  ignored (`tools/sonarcheck.py` on the 21 real scans: both edges in 21,
  old rule 6; 100-340 mm instead of up to 500+). The plan is capped at
  300 mm wide / 200 mm out from the line.
- [ ] 🟡 **S5-O01** width, 3 tries (3 flat boxes, one each, real width
  by ruler, box about 350 mm ahead, nothing else within 60 cm). With the
  scan at +/- 25 deg the sonar never saw past a box's edge (beam is about
  30 deg wide), so no width came out. Scan widened to **+/- 40 deg: not
  tested on the robot yet.** Route the cables at the back first so they
  do not pull at 40 deg.
- [ ] ⬜ **S5-O02** bypass: box on the line, robot stops, scans, drives round
  the clear side, back on the line. Pass = no contact, within 300 mm of the
  line, back on it within 500 mm. Needs S5-O01 working (with no clear side
  found it stops with "no clear side").
- [ ] ⬜ Before the ±40 deg scan: **route the servo cables at the back** so
  they do not pull at 40 deg (the horn already broke once at full speed)
- [ ] ⬜ Forward watch while following: a box on the line, the robot must
  stop **before** about 250 mm (2 readings in a row), part of S5-O02
- [ ] ⬜ "No clear side": boxes or a wall on both sides of the obstacle;
  must stop with "no clear side", not drive into one
- [ ] ⬜ "Bypass leg blocked": something in the way of a bypass leg; it
  pings before each long leg and must stop with "bypass leg N blocked"
- [ ] ⬜ Back on the line after the bypass: the robot drives back toward
  the line until C sees it (floor grout only counts near where the line
  should be); then the normal line checks (junctions, stuck sensor) start
  again after 60 mm
- [ ] ⬜ Sonar fault: cover or unplug ECHO during S5-O02; the robot must stop
  with "ultrasonic not answering"

## Stage 4: ramp (`TEST_STAGE=4`)

The practice "ramp" is a **hump**: a block 10 mm high, about 5 cm long in
the driving direction (20 cm across), 0.5 cm flat on top. Much shorter
than the robot, so the robot tips over it instead of driving up a slope.

### What works
- [x] ✅ S4-I01 IMU found (no read dropouts in the 2 Nov runs after reseating)
- [x] ✅ Tilt view: lifting the front gives sin(pitch) up to about 500, back
  to 0 when level; sign correct
- [x] ✅ Test stops the robot itself once a ramp is reported, and prints
  distance / tilt / height every 0.5 s plus the highest climb and
  L x peak tilt for every run

### Not done yet
- [ ] 🟡 **S4-R01** height within ±5 mm, now **5 runs** (was 10).

  **Runs 2 Nov** (old code): 18.3, 66.0, none. Jolts were read as tilt.

  **Runs 6 Nov, batch 1** (jolt filter 5 %): 49.6, none, none. Problems
  found in the log and fixed:
  - the IMU's anti-alias filter was 773 Hz, so motor vibration showed as
    tilt (-2..-4 deg on the flat floor) -> now 50 Hz (`CTRL2 0xC8`)
  - the 5 % jolt filter skipped 30-48 % of all readings, and the height kept
    adding up with the frozen tilt -> now 10 %
  - driving tilts the chassis a little, so the height drifted -10 mm before
    the hump -> tilt is now measured from a slowly following "driving level"

  **Runs 6 Nov, batch 2** (fixes above): 27.4, none, **9.0 ✅**
  (L x tilt 10.6). Jolts skipped dropped to 14-215 per run.

  **Runs 6 Nov, batch 3**: 56.2 (a -56 mm **dip** reported as a climb), none,
  **8.1 ✅** (L x tilt 12.1), none; run 5 not done. **Fixed, not run yet:**
  only climbs are reported now (nose-down dips ignored), and L x tilt uses
  the nose-up peak only.

  Still wrong on the robot side (not code):
  - **The hump is not held down.** A 10 mm hump with L = 80 mm can tilt the
    robot at most sin 0.125 (7 deg); failed runs peaked at 0.23-0.48, so the
    block slid, flipped or got under the robot. Hold it (fingertips on both
    ends, Blu-tack or a rubber mat under it).
  - **"IMU read failed" twice** in batch 3, once mid-run: reseat the Grove
    cable at both ends.
  - Check the IMU board is fixed firmly to the chassis (a loose board
    wobbles on every bump).
  - The hump showed up at 450-1340 mm into the runs, not about 300 mm:
    check the start distance and that the robot drives straight at it.
  - Ask: is this block the lab's lower practice ramp? An accelerometer
    cannot measure a hump much shorter than the robot reliably; a real
    slope is much easier.
  **Run 7 Nov (over WiFi), two humps in one run:** crossed at about
  230-480 mm and 830-1400 mm, **neither reported**: the tilt went mostly
  nose-down (-120 and -196: the drive wheels climbing after the front
  support, plus braking on impact, which the accelerometer reads as
  tilt), 157 readings skipped as jolts (1.6 s with the tilt held), and
  the 0.5 s trace missed the short nose-up (peak 302, L x sin 24 mm: a
  jolt, not the 10 mm hump). **Changed the same day:** start 20 cm before
  the ramp, the run ends 30 cm after the last one, each ramp printed as a
  bump (nose-up / nose-down peaks, L x sin, climb, jolts) with a 10 ms
  trace while on it. Not run yet.
- **The course ramp is a hump too** (team, 7 Nov): slightly higher than
  the 10 mm practice block. So the robot tips over it and L x sin(nose-up
  peak) is the height to use; the IMU task's own report (integrated
  climb, used by Mission and the demo) should switch to it once S4-R01
  shows L x sin within +/- 5 mm.
- [x] ✅ **L measured: 80 mm** (`S4_L_MM`), same as the guess
- Note for B4: a hard STOP can give a false ramp report (braking tilt)

## Stage 5: telemetry + MQTT (demo image)

Run 2 Nov on a phone hotspot (2.4 GHz), robot on the power bank for
S1-T02. Robot got IP 10.14.83.102; broker on the laptop.

- [x] ✅ WiFi join + MQTT connect (`mqtt: connected`), heartbeat both ways
- [x] ✅ S1-T01 status line on `is34/telemetry` every 1 s (state, speed,
  distance, line, junctions, barcode, mapping, ramp peak, stop reason);
  events on `is34/event`
- [x] ✅ Mapping `K=L,Q=R,X=F,M=U` gives `is34/ack/map OK ...` and shows in
  the status line; bad `K=L` gives `REJECTED 1 pairs, need exactly 4 (old
  mapping kept)`
- [x] ✅ S1-T02 remote START / STOP: **3 of 5 rounds done, 3/3 passed**. Each
  START gave CAL then RUN at 150-170 mm/s; each STOP gave the `STOP` event
  in the same second and IDLE, speed 0, `stop=2` by the next status line.
  Two more rounds would make it the full 5.

Setup notes:
- The laptop's Mosquitto Windows service only listens on localhost:1883,
  so a second broker was run on **port 1884** (`listener 1884 0.0.0.0`,
  `allow_anonymous true`) and `MQTT_BROKER_PORT` set to 1884 in
  `config/wifi_secrets.h` (git-ignored).
- The broker IP is built into the image: if the laptop's IP on the
  hotspot changes, edit `MQTT_BROKER_IP` and rebuild
  (`make NET=lwip MQTT=1`).

## Stage 6: whole course

- [ ] ⬜ SYS-01: 3 full runs

## Rules that stay in force

- Never power from laptop USB and the power bank at the same time.
- `config/wifi_secrets.h` is git-ignored: never commit it.
- Don't build with `CRASH_REPORT=1` while the IR sensors are on GP0/GP1.
- The `.uf2` files in `images/` are rebuilt with every code change (last
  7 Nov). Pull before flashing.
