# P5-IS34 test status checklist

Last updated: 8 Nov 2026 (evening). **Since the robot runs earlier on 8 Nov the code was
brought to the Barr C standard and Change Requirement 1 (minimum-bit data,
binary MQTT) was implemented: every stage needs a confirmation run with the
new images and the updated panel. The extra tests are in
[CHANGE_REQ1.md](../docs/team/CHANGE_REQ1.md) section 7.2.** How to run each test:
[TESTING_GUIDE.md](TESTING_GUIDE.md). Formal results:
[testing/](./) `TEST_LOG.md` files. What the robot does, its limits
and everything still open: [ROBOT_BEHAVIOUR.md](../docs/robot/ROBOT_BEHAVIOUR.md).

Legend: ✅ passed on the robot · 🟡 works, but not to the pass mark yet ·
⬜ not tested yet · ❌ tested and failing

## Summary

| Stage | What | Owner | Status |
|---|---|---|---|
| 0 | RTOS timing, all 8 tasks | B1 | ✅ passed 26 Oct |
| 1 | Motion + STOP | B2 | ✅ passed 1 Nov |
| 2A | Line following + junctions | B3 | 🟡 follows well; crossings now detected while correcting (8 Nov); 3 m run, X crossing / left branch tests not shown |
| 2B | Barcodes | B3 | 🟡 F and A decode, driving straight and while following (8 Nov); not every pass (paper not taped down); forgiving decode added, not run yet |
| 2C | Junction moves from barcodes | B3 + B5 | 🟡 first U-turn from a barcode 8 Nov (spun ~190 deg, fixed, not run); left/right moves and the unmapped stop not shown |
| 3 | Obstacle: sonar, servo, width, bypass | B5 | ✅ bypass passed 8 Nov (box on a junction, no contact, back on the line); width reads somewhat wide (planned for anyway) |
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

**9 Nov re-run** (new code: Barr C + Change Requirement 1; rows in
`testing/subsystem2/TEST_LOG.md`):
- [x] ✅ Stage 0 automatic checks, S2-M05 directions, S2-M06 STOP lifted
- [x] ✅ S2-M03 encoders 500 mm at 120 mm/s (mean 120); 🟡 the real distance not measured
- [ ] 🟡 S2-M02: round 1 only. Encoders 90.6 / -90.2 / 180.6 deg; the real 180 about 5° over (photos).
  **Changed after it:** spins use `S2_TURN_TRACK_MM` = 120 (was 123), about 4° less on a 180.
  Then the Stage 3 bypass's 90s still turned about 5° too much (about +7 on the
  wheels' 88), so the over-turn is a fixed few degrees, not a percentage.
  **Changed again 9 Nov:** track back to 123, and every spin of 45° or more stops
  6° early (`S2_TURN_TRIM_DEG`); expected about 91 for a 90 and 178 for a 180.
  The S2-M02 encoder angle now reads about 6° under (84 / 174): judge by the
  real angle. Re-run all 3 rounds and write down the real angles.
- [ ] 🟡 S2-M04: try 1 stopped in 149 ms; tries 3-5 not run
- [x] ✅ Held wheel (Change Requirement 1 Safety): motors off, event sent; START again after 2 s: to confirm
- [ ] ⬜ **S4-S01 Motion hung (NFR10), added 17 Nov, not run yet:** Stage 1, wheels lifted: the test freezes the Motion task for 1 s (test images only); Safety must turn the motors off within about 220 ms of Motion's last run and clear the fault 2 s later

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
  team's request: line.c as it was and 120 mm/s. Undone with it:
  the straight-line probe (false LINE LOST on long straights comes back),
  the 1-reading filter, the heading rule for left branches. Kept: the
  barcode reader (relative edges, one pass, steering held straight over
  the bars), WiFi log, Stage 4 test.
- **7 Nov, later: line following re-tuned in a simulation**
  (`tools/sim/linesim.py`), calibrated until it reproduced the three real runs
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
- **8 Nov Stage 2 run (`linefolo` log, 8 Nov):** mapping from the new
  panel box arrived (`A=L,B=R,C=F,F=U`). 2C: **barcode F read, "U-TURN at
  next junction", but the junction was not detected**, it drove on. Same
  in S3-L01 (junction at about 800 mm missed) and 2B run 2 (about 620
  mm). Cause: the robot zigzagged (S3-L01: 24 corrections a metre, 19
  sweeps; 7 Nov good runs 0-6), and a crossing only counted after 20 mm
  of driving straight. Both missed junctions had L and R black together
  for 16-18 mm and B a tape-wide patch at the same place (7-12 mm apart).
  **Fixed 8 Nov, not run yet:** both black 10-45 mm + B's tape patch
  within 25 mm = a crossing, correcting or not (the saved logs: 11 such
  places, 3 detected then). Barcode: S3-B00 run 2 PASS (F, from the
  other end); run 1 lost its first 4 elements; 2B read F twice.
  Zigzag: check the line sensor mount (did it move?).
- **8 Nov, second Stage 2 run:** junction now detected in 2C (`junction 1
  (crossing)`), but the barcode before it read `READ_FAIL (27 elements)`,
  so the robot went FORWARD (no letter stored). S3-B00: F and A decoded
  (runs 1, 3); run 2 failed with the start `*` damaged (team: barcode
  positions were off). 2B: F read once, run 1 LINE LOST at 350 mm.
  **Added 8 Nov, not run yet: forgiving decode** (one `*` damaged, or one
  narrow element lost), 0 wrong on every test (`tools/logcheck/barcodecheck.py`);
  the run-2 read now decodes (A), one older 27-element read too (A).
- **8 Nov, third Stage 2 run: first U-turn from a barcode.** `barcode F ->
  U-TURN at next junction`, `junction 4 (crossing): U-TURN (barcode F)`;
  the spin came out about 190 deg. Before it: START at the mapping prompt
  took the TEST mapping (no F): "no mapping for barcode F", stop (as
  designed); then a READ_FAIL of 27 elements = F with its weak last bar
  missed. Reads inconsistent overall (team: the paper is not taped down).
  **Fixed 8 Nov, not run yet:** cut-short repair (that read now decodes
  F; 0 wrong on the logs); junction spins end on the line (fast to 30 deg
  short, slow until a sensor touches the tape, 12 deg more).
- **Edge cases fixed 8 Nov, not run yet:** a junction less than about 8
  cm after a barcode used the letter at the NEXT junction (now it waits
  for the read); an obstacle met mid-barcode garbled the read (now
  dropped, reported).
- **9 Nov, Change Requirement 1 packed build (`2026-11-09_stage2_line_robot.txt`
  in `subsystem3/logs`): line following good, team happy.** S3-L01 1505
  mm, **line lost 0 times**, through left and right curves like the ones
  it used to lose (FAIL only on the 3 m: stopped by hand, small track).
  13 corrections per metre, 1 sweep across the tape. The X crossing was
  detected (`junction 1 at 1194 mm: crossing`), no false junctions. No
  barcode on this track: barcodes (S3-B00, 2B, 2C) still to run. Stage 0
  checks in the image all PASS; no unexpected faults.
- **15 Nov course runs, Mission image (`subsystem5/logs/2026-11-15_course_mission_1/2_robot.txt`):
  barcode F (mapped LEFT) before a junction every run; the robot never
  turned left.** Two reads reached the decoder, both `READ_FAIL`: 23
  elements (a narrow bar lost in the letter, a 19.2 mm merged space) and
  25 elements, where the F and the closing `*` read cleanly but the first
  4 elements of the opening `*` were never seen. The other runs broke the
  code into short pieces (`inbar` on and off, under 20 elements: dropped
  as noise). Narrow bars read only 1.5-2.3 mm against 4.3-5.8 mm narrow
  spaces: the sensor counts only the darkest middle of each bar, so thin
  bars are at its limit. **Added 15 Nov, not run yet:** `decode_cut_star`,
  a code whose first 3-4 elements were lost (the 25-element read now
  decodes as F; the real C on the PC: 3870 synthetic cut codes read, 0
  wrong; 2 false in 50 000 random patterns). **Still to do: the B
  thresholds** (`S3_BAR_DELTA` 700, `S3_BAR_BLACK_MIN` 1500) from a
  Stage 2 live view of the course barcode (white and thin-bar readings).
- **15 Nov, third course run (`..._course_mission_3_robot.txt`, with the cut-start
  decode): F again not read, no READ_FAIL at all.** The panel trace shows
  the robot steering +-30 over the bars and the over-the-bars hold
  switching on and off (it let go after 15 mm without an edge: one missed
  thin bar plus an 11 mm wide space), so B swung across the bars and the
  code broke into pieces. **Changed 15 Nov, not run yet:** the steering is
  held straight for the whole code (until 40 mm without an edge).
- **15 Nov, Stage 2 live view of the course barcode (`subsystem3/logs/2026-11-15_stage2_liveview_course_barcode.txt`):
  the cause.** White 160-220, wide bars 2000-3600, thin bars only
  929-1535: most thin bars never passed `S3_BAR_BLACK_MIN` 1500.
  **Changed 15 Nov, not run yet:** `S3_BAR_BLACK_MIN` 1500 -> 700,
  `S3_BAR_DELTA` 700 -> 300 (noise +-20).
- **15 Nov, course run 4 with 700 / 300 (`..._course_mission_4_robot.txt`): worse.**
  Only 7-element pieces of even 6-7 mm blobs (neighbouring thin bars and
  spaces run together while driving), against 23-25 of 29 elements with
  1500 / 700. **Put back to 1500 / 700 the same day**; the cut-start
  decode and the straight hold stay. The hand live view does not show
  what driving over the code gives.
- **15 Nov, course run 5 (`..._course_mission_5_robot.txt`): ✅ barcode F read cleanly**
  (`narrow 3.4 mm, wide 9.3 mm -> LEFT at next junction`), then `junction 3
  (crossing): LEFT (barcode F)`. STOP was pressed during the planned
  115 mm drive past the junction centre (it looked like a miss), so the
  left spin did not run: let it go next time. Runs 13:52-14:04 saw only
  four 7 mm marks (`7 7 7 20 7 7 7`) with both threshold settings:
  something other than the bars was under the sensor. **The fix was
  only the barcode's position and tilt** (tester, 15 Nov; same print, not
  reprinted): square to the line, B over the middle of the bars. So the
  thin-bar readings of the day (929-1535, 1.5-2.3 mm) came from B crossing
  the bars at an angle or near their ends, not from the print.
- **15 Nov, course run 6 (`..._course_mission_6_robot.txt`): barcodes and junctions
  ✅** (F read twice, A after the mapping changed, every junction LEFT as
  mapped; one READ_FAIL). **Obstacle: `obstacle lost between scans`.** Scan
  1 at 216 mm saw the box with both edges (211 mm); scan 2, 100 mm back,
  read 650 straight ahead and the box only at +5..+30 deg: the robot
  turned about 12 deg while backing up. **Changed 15 Nov, not run yet:**
  when scan 1 sees both edges it is the profile; the back-up and scan 2
  only when scan 1 missed an edge.
- **Added 15 Nov, not run yet (bypass safety, team request):** (1) the
  sonar watches ahead on the sideways leg, the leg past the box and the
  leg back to the line: 2 pings nearer than 150 mm (or no ECHO) stop the
  robot, `something in the way during the bypass`; (2) after turning back
  toward the line it pings first: an echo nearer than the line means the
  box is longer than the assumed 150 mm, so it turns back, drives 100 mm
  further along it and looks again, up to 4 times (`box still beside:
  going 100 mm further along it`, else `obstacle longer than expected`).
  Turning on the spot beside the box is safe: centre at least 185 mm from
  its side, corners swing about 155 mm. Tests: TESTING_GUIDE Stage 3 step 4.
- **Changed 17 Nov, not run yet:** the leg past the box takes no line. In
  course run 7 the `line met behind the box, 503 mm along` was **a
  different piece of track** beside the box's far side (tester), so the
  robot followed the wrong line. Now it always drives the full leg, turns
  back (checking the box is not still beside), finds its own line and
  spins onto it in the old direction. A different track between its path
  and its own line would still be taken: keep other track away from the
  box's sides.
- **15 Nov, course run 7 (`..._course_mission_7_robot.txt`, stop at 185, 120 mm/s): ✅
  whole sequence.** Barcode F -> LEFT at junction 2; obstacle stopped at
  168 mm, width 116 mm (the course box, about 13 cm), bypass LEFT, `line
  met behind the box, 503 mm along` (later, 17 Nov: a different piece of
  track, see above), `bypass done`; hump `ramp peak height 18.5 mm`;
  lifted at the end: motors off.
- **Changed 15 Nov, not run yet:** obstacle stop distance 250 -> 190 mm
  (`S5_TRIP_MM`; team: it stopped too far away), scan back-up only if
  nearer than 160 mm (`SCAN_NEAR_MM`, was 220). **Mission speed 120 -> 135 mm/s** (`S5_FOLLOW_SPEED`; stage tests stay at 120): re-check the tightest left and right curves and one bypass. Expect "obstacle ahead at
  about 165-175 mm". Tribunal 1 minimum: 150 mm (little margin: re-check if
  the speed is raised).
- **Changed 18 Nov, not run yet:** Mission speed **back to 120 mm/s**
  (team: 135 never ran on the course; 120 is the speed of the 7 course
  runs without a line lost and of run 7's whole sequence). With the 190
  mm trip expect "obstacle ahead at about 170-175 mm".
- **20 Nov, track 4, Mission run 1 (`subsystem5/logs/2026-11-20_track4_mission_1_robot.txt`, 120 mm/s, 190 mm):**
  ✅ WiFi + MQTT on the new hotspot address; ✅ `obstacle ahead at 180
  mm` (scan front 175 mm: NFR6 >= 150), width **156 +/- 20 mm** (real box
  width not yet written down), one scan only (both edges seen), bypass
  LEFT out 272 mm, past 625 mm. `ramp peak height 10.0 mm` reported during
  the bypass (real height and the hump's place not yet written down).
  On the way back it met a line 133 mm after its last turn: its own line
  was expected at about 157 mm (272 out, less the 115 mm the line sensors
  sit ahead of the wheels), so most likely its own line, about 24 mm early
  (a few degrees of heading over the bypass). Then `bypass done, following
  the line` and ❌ soon **`line lost`**: cause not known yet (where the
  box was, and what the robot did after rejoining, still to be told).
  Lifted at the end: motors off ✅. No barcode or junction in this run
  (one panel message lost on the WiFi). Team: real box width **10 cm**
  (read 156 +/- 20: too wide); hump height looked right (not measured).
- **20 Nov, track 4, Mission run 2, two laps (`subsystem5/logs/2026-11-20_track4_mission_2_robot.txt`,
  trip 175, beam 12):** 8.0 m driven. ✅ **3.65 m of following without a
  line lost** (from the first bypass to the second box, through junction 1:
  first run past 3 m, NFR1a). **Box 1:** tripped at 171, scan front **152
  mm** (NFR6 >= 150: only 2 mm to spare), width **97 +/- 17 mm (real 100)
  ✅**, one scan, about 9 s from the stop to the result (NFR3 says 7 s);
  bypass RIGHT out 280 past 602, line met 238 mm after the last turn
  (expected about 165), ✅ rejoined and carried on (16 s). **Box 2 (same
  box):** tripped at 171, scan 1 at 157 mm heard echoes across the whole
  sweep (143 +/- 52), backed up, scan 2 at 260 mm (196 +/- 30): reported
  **142 +/- 29 ❌ (real 100)**, about 17 s for both scans; bypass LEFT out
  255 past 710, line met 152 mm after the last turn (expected about 140),
  `bypass done`, then ❌ **`line lost` about 300 mm (5 s) later**, as in run
  1 (also a LEFT bypass). Ramp 12.6 and 11.2 mm, both reported during a
  bypass (where the hump is: to be told). ❌ **Barcode: bars seen** (panel
  `inbar=1` for about 7 s before junction 1, steering held) **but no letter
  and no READ_FAIL**: the pieces were under 5 elements; junction 1 went
  FORWARD with no barcode. WiFi-loss warning flickered (faults=64) about a
  dozen times for under 1 s; the robot kept running.
- **20 Nov, track 4, Mission run 7, six runs (`subsystem5/logs/2026-11-20_track4_mission_7_robot.txt`):**
  ✅ letters read with no mapping and carried on: **Q, Z, F** (read
  backwards, narrow about 6 mm, wide about 16 mm); ✅ **run 5: `F -> LEFT`,
  then `junction 2 (crossing): LEFT (barcode F)`**: the barcode's move done
  on track 4. Mapping changed between runs, acknowledged each time. ❌
  Runs 4 and 6 ended `line lost` a few seconds after a crossing: B saw tape
  for seconds (`inbar=1`), so the steering was held straight (`hold=1`)
  while the line task asked for full steering (`steer=30`), and the robot
  ran straight off its line. **Changed 20 Nov, not run yet:** the straight
  hold over a barcode ends when the line task is in a long correction (one
  sensor on the tape over 50 mm) or searching (`HOLD_MIN_PCT`, motion.c);
  short corrections over a barcode stay held (the 15 Nov reads). The
  panel's "steering held" now shows the real hold (`motion_steer_held`).
- **Changed 20 Nov (after run 6), not run yet (team):** (1) the hump: a
  junction turn or an obstacle stop reports the hump being measured at
  once (the tester's hump is just before the box: it was reported halfway
  round), and the IMU measures nothing while Mission manoeuvres (no false
  humps from the bypass or spins; `mission_manoeuvring()`); humps while
  following are unchanged. (2) Stop trip 175 -> 180 mm (stops were 150-162:
  no margin): expect about 155-167 mm.
- **20 Nov, track 4, Mission run 6 (`subsystem5/logs/2026-11-20_track4_mission_6_robot.txt`, beeps):**
  ✅ 4 m of following from junction 1 to the box, box front 162 mm, width
  **125 +/- 19** (real 100), bypass LEFT, back on the line (heading 3 deg),
  2.8 m more to junction 2 (FORWARD). Hump: **two reports for one hump**
  (tester): 14.0 mm during the bypass (side tilt sin 0.181, 7 s into it,
  500 mm of driving after the stop) and 12.7 mm 2.8 m later while following,
  3 s before junction 2: one of them is not the hump (where it is: to be
  told). Tester: no beep heard at the hump; the hump's beep comes when it
  is reported, after 100 mm of level driving past it (about 1 s), not
  while on it. Barcode not decoded (pieces; placement).
- **Added 20 Nov, not run yet (team):** one short beep (100 ms, 2 kHz) on
  the board's buzzer (GP22) for every event, played by the Telemetry task
  so nothing waits for it. Check: a beep at each barcode, junction,
  obstacle, hump and stop; driving and timing unchanged.
- **20 Nov, track 4, Mission run 5 (`subsystem5/logs/2026-11-20_track4_mission_5_robot.txt`, no mapping):**
  ✅ **barcode F decoded on track 4, read backwards** (narrow 6.0 mm, wide
  15.8 mm: the track 4 print, larger than the 3.5 / 10 mm one), then the
  stop at the junction `no mapping for barcode F` (as designed then).
  **Changed 20 Nov, not run yet (team):** a letter with no mapping is
  reported (`-> NO MAPPING, ignored`) and ignored: the robot carries on,
  straight at the junction, and a mapped letter already waiting is kept.
  The panel's Run summary (new) gives the run in a few lines.
- **20 Nov, track 4, Mission run 4, two runs (`subsystem5/logs/2026-11-20_track4_mission_4_robot.txt`, rejoin turns back):**
  ✅ **rejoin: `bypass done ... heading 3 deg from before the box`**, then
  followed on (run B: 30 s until STOP, no line lost). ✅ Run B: 3.7 m of
  following from junction 1 to the box. Boxes: front **160** and **150 mm**
  (150 = the minimum: no margin), width **118 +/- 18** and **119 +/- 17**
  (real 100), one scan each, bypass LEFT out 260 / 255. Hump: **12.6 mm**
  (pitch form; real height not measured), reported during the bypass as
  every time so far: the stop's braking tilt restarts the 100 mm level
  stretch, and standing still adds none, so the report waits for the
  bypass legs (run A started after the hump: no report). Two false
  obstacle stops in run A (75 and 164 mm; the scans found nothing, `going
  on` ✅). ❌ **Barcode: nothing decoded**, pieces of 5-7 elements
  dropped (one element 18.4-18.8 mm in most), as on 15 Nov when the code
  was crossed at an angle or near its ends. WiFi: one association
  failure, then joined. Lift stop ✅.
- **Changed 20 Nov (team, after more track 4 runs: "it does not
  reorient, it follows the wrong line"), not run yet:** after the bypass
  the robot always turns back to its old direction, like a junction turn
  (fast to 30 deg short of 90, then slowly until a sensor finds the tape
  within 30 deg either side, then centred); before, it skipped the turn
  when a sensor already saw black after the 115 mm onto the line, and
  otherwise stopped at the first tape of a 170 deg spin. `bypass done`
  now prints the heading from before the box, by the wheels (spins read
  a few degrees under on the wheels: a guide, not a measurement).
- **20 Nov, track 4, Mission run 3 (`subsystem5/logs/2026-11-20_track4_mission_3_robot.txt`, 3 pings):**
  tripped at 161 mm; the scan, width and bypass lines were lost on the
  WiFi; line found 120 mm after the last turn, `bypass done`, then ❌ the
  robot drove diagonally off the line (tester). Panel trace: right after
  the rejoin `inbar=1 hold=1 steer=0` at 50 mm/s: the barcode sensor
  (85 mm left of the centre) crossed the line's own tape while the robot
  was still at an angle, read it as bars, and Motion held the steering
  straight. **Changed 20 Nov, not run yet:** after a bypass the barcode
  sensor is ignored for 200 mm (`barcode_ignore_mm`, `REJOIN_BAR_MM`):
  no read, no slowing, no straight hold while the robot settles onto
  its line. Junction turns unchanged.
- **Changed 20 Nov (after run 2), not run yet (team: scan faster, go
  round a little tighter):** 3 pings per ray instead of 5 (a scan about
  4.5 s instead of about 6.5, so about 5.5 s from the stop to the width
  instead of 9); the sweep's last ray alone counts as past an edge when it
  is open (run 2's second box: 117 mm from the first scan, no back-up and
  second scan; 4 of the 52 saved scans change, `tools/logcheck/sonarcheck.py`,
  and the robot's own edge code compiled on the PC gives the same);
  the bypass plans for at least 150 mm instead of 200 (`S5_MAX_WIDTH_MM`):
  out about 255 instead of 280 for the 10 cm box.
- **Changed 20 Nov, run in run 2:** stop distance `S5_TRIP_MM` 190 -> 175
  (team: a little closer; the robot stops about 15 mm inside the trip, so
  about 160 mm, minimum 150); sonar beam correction `S5_BEAM_HALF_DEG` 6 ->
  12 (the two course-box scans read 116 and 156 for 10 cm; the same scans
  give 83 and 118 with 12, checked by recomputing them). The bypass path
  does not change (it plans for at least 200 mm).
- [ ] 🟡 **S5-B01 / S5-B02** (2C): right move at each junction, U-turn, stop on
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
  `tools/sim/bypasssim.py` (486 simulated bypasses). The line sensors watch
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
  ignored (`tools/logcheck/sonarcheck.py` on the 21 real scans: both edges in 21,
  old rule 6; 100-340 mm instead of up to 500+). The plan is capped at
  300 mm wide / 200 mm out from the line.
- **8 Nov, third Stage 3 run (`ultrason` log 3), the geometry rule on the
  robot:** S5-O01 widths 221 / 273 / 267 mm (box at about 26 cm instead
  of 35; scan 1 alone 205 / 354 / 225); START gave the TEST mapping as
  planned. **Bypass:** width 287 mm, planned 300, went round RIGHT (out
  383, past 906), no contact, but `line met beside the box, 599 mm
  along`: the box stood on a junction, the leg alongside the box met the
  junction's side line (sensors 14 cm past the box's front) and the robot
  followed it away. **Fixed 8 Nov, not run yet:** that leg watches only
  past the back of a 150 mm deep box (`tools/sim/bypasssim.py`, boxes on T/X
  junctions: 5-45 of 54 -> 54/54 each; ordinary straight and curved
  lines 471/486 instead of 476: a few 300 mm curves that start right
  behind the box are now met later and missed). Also:
  scan 1 from where it stops, then 100 mm back (was: back to 350 mm, then
  100 more), and an edge both scans saw is taken from the nearer one
  (every pair read narrower from nearer). Still not handled: a junction
  side line on the bypass side within about 35 cm behind the box (looks
  the same as the line itself turning 90 deg there).
- **Added 8 Nov, not run yet: side look.** On the leg past the box the
  sonar turns 40 deg toward it (90 has no cable clearance) and listens
  for its side face; every echo pushes the end of the leg further, never
  shorter than the fixed 150 mm depth (another group does something
  similar). `tools/sim/bypasssim.py S`: depths 60-350 mm all 36/36 (fixed
  only: 250 mm deep 16 of 36 hit, 350 mm all hit), ordinary cases and
  junctions as before; 53/54 deep runs even with half the side echoes
  missed. Relies on the side of the beam hearing a face beside the
  robot: **new Stage 3 bench check** (`[S5] side N mm`, box beside the
  robot about 20 cm out should read about 200).
- **8 Nov, fourth Stage 3 run (`ultrason` log 4): bypass worked.** Box
  218 mm ahead: width 186 mm (scan 1; scan 2 from 320 mm: 334), went
  round RIGHT (out 312, past 770), `line found 188 mm after the last
  turn`, `bypass done`. S5-O01: 276 / 114 / 91 mm (boxes at 278 / 155 /
  98 mm; real widths not sent). `tools/logcheck/sonarcheck.py` gives the same
  widths from the logged rays. **Side look did not hear the box's side:**
  bench check (box beside the robot, sonar 40 deg right) read mostly
  `-1`, once 222 and 275, otherwise 400-900 (the room). In the bypass it
  heard the box from 184 to 297 mm along the leg, i.e. while approaching
  the box's front corner (front at about 320), not beside it, so the leg
  stayed the fixed 770 mm. At 40 deg the side face is not heard; depth
  is still the 150 mm assumption. **Side look switched off** (the
  obstacle is 9 x 13 cm x 19-20 cm high, so at most 130 mm deep; bypasssim
  with it: no contact either way round, closest 35 mm, junctions 27/27).
- **Stage 3 called done by the team, 8 Nov.**
- **9 Nov, Change Requirement 1 packed build (`subsystem5/logs/2026-11-09_stage3_obstacle_robot.txt`):
  bypass works as before.** Servo check run; sonar view steady 240-246 mm.
  S5-O01: 175 / 136 / 210 mm, both edges found every time (real widths
  not recorded). S5-O02: stopped at 241 mm, width 114 mm, went round
  LEFT (out 254, past 785), `line met behind the box, 703 mm along`,
  `bypass done`, then `line lost` further on. Team: works, but **the
  90° turns turn about 5° too much**: fixed the same day (Stage 1
  section, `S2_TURN_TRIM_DEG`), not run yet.
  - **Mapping from the panel always REJECTED: a bug, fixed 9 Nov, not
    run yet.** The MQTT glue (`lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`)
    turned every non-printable byte into '.' before the app saw it, so the
    new binary map command (Change Requirement 1) arrived damaged and was
    tried as text. Now the app gets the raw bytes; only the printed copy is
    cleaned. START with the TEST mapping worked meanwhile.
  - A false `ramp peak height 41.2 mm` came during the bypass (side tilt
    0.335 g: the spin's start or stop, not a slope). Check in Stage 4
    whether turning gives false ramps.
  - Mission's `[tmg]` run time shows 35 s during the bypass: Mission runs
    the whole bypass in one wake (it waits for each move), so this is the
    bypass's length, not an overrun (overruns 0). Safety's Motion-hung
    check watches Motion, which kept running every 10 ms.
- [ ] 🟡 **S5-O01** width: works (both edges found every time since the
  geometry rule), but reads wide from further away: the 9 cm obstacle
  read 186 mm from 218 mm, 91 mm from 98 mm. Harmless for the bypass
  (it plans for at least 200 mm). Real widths of the tries were not
  recorded, so the pass mark (true width inside W +/- U) is not shown.
- [x] ✅ **S5-O02** bypass, 8 Nov (log 4), with the box **on a junction**:
  stopped at 194 mm, went round RIGHT, `line found 188 mm after the last
  turn`, `bypass done`, followed on. No contact. (7 Nov the first bypass
  also rejoined; the junction run before the 8 Nov fix followed the
  side line away.)
- [x] ✅ Servo at +/- 40 deg: cables hold (8 Nov runs). 90 deg has no
  clearance (side look built, off).
- [x] ✅ Forward watch while following: stopped at 194-245 mm in every
  bypass run (limit 250)
- [x] ✅ "Bypass leg blocked": 7 Nov a wrong 445 mm detour met something
  in the room and stopped with "bypass leg 4 blocked"
- [ ] ⬜ "No clear side": boxes or a wall on both sides of the obstacle;
  must stop with "no clear side", not drive into one
- [ ] ⬜ "Bypass leg blocked": something in the way of a bypass leg; it
  pings before each long leg and must stop with "bypass leg N blocked"
- [x] ✅ Back on the line after the bypass (8 Nov, also from a junction)
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
- **8 Nov: the course's hump is a 2-5 cm triangle under the RIGHT wheel
  only** (not a block under both wheels, as assumed above). That tilts
  the robot sideways, which the code never measured: the readings above
  were jolts. **Changed 8 Nov, not run yet:** height = 123 mm x sin(side
  tilt) at its peak, cross-checked with 160 mm x sin(nose-down tilt);
  the test prints both (`[S4] bump N one wheel: ...`), the tilt view
  shows the side tilt (put the right wheel on a block), and the run
  prompt places the triangle under the right wheel.
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
- **9 Nov, first one-wheel run (`subsystem4/logs/2026-11-09_stage4_imu_robot.txt`,
  Change Requirement 1 build): the hump was seen**, side tilt peak
  0.143 at about 214 mm = **17.6 mm** (the test's own figure; the real
  height was not written down, so not judged yet). Tilt view: front
  lifted gives sin(pitch) up to 251; a block under the right wheel read
  16.3 mm once. **Two bugs, fixed 9 Nov, not run yet:**
  - **The IMU task never reported it** ("no ramp reported by the IMU
    task"): it reports once tilt and side tilt are both under 0.030 for
    100 mm, but the fast side-tilt reading jitters +-0.03 while driving
    (-34, -31 on the flat), so the level stretch kept restarting (longest
    43 mm in the trace). Now the level test uses a slower copy (about
    100 ms, as the pitch): at most 0.006 on the same flat stretches; the
    peak still comes from the fast one.
  - **STOP ended all 5 runs**, so START afterwards did nothing (the test
    was over). Now STOP ends only the current run; the next START runs
    the next one.
  - Also seen: a "bump 2" at 596 mm on the flat (nose-down -42, noise), so
    the run did not end by itself; STOP ends it.
- **9 Nov, second Stage 4 run, 5 runs (`subsystem4/logs/2026-11-09_stage4_imu_robot_2.txt`).**
  Humps (tester's estimates): about 10 mm for runs 1, 2 and 5, about 25 mm
  for runs 3 and 4. Every hump was seen. Heights read **about 1.6 times
  too high**: 16.6 / 17.2 / 40.8 / 37.0 / 17.6 mm (+5 to +16 mm, FAIL on
  +-5 mm), the same factor for both sizes, while standing still on a
  block the tilt view read a steady 23-24 mm. The IMU task reported runs
  1, 3 and 5 only. **Fixed 9 Nov, not run yet:**
  - The reported height is scaled by 0.63 (`S4_ONE_WHEEL_GAIN_PCT`, least
    squares over these 5 runs): they would read 10.4 / 10.8 / 25.7 / 23.3
    / 11.0 mm, all within +-2 mm. Fitted on these runs, so it must be
    confirmed on new runs (measure the humps with a ruler).
  - Runs 2 and 4 came right after a reported run: the "no second report
    for 600 mm" distance carried over into the next run. Calibrating at
    the start of a run now resets it.
- [ ] ⬜ **S4-L01 lift stop (lecturer, 13 Nov; replaces IMU collision
  detection): built 13 Nov, not run yet.** Gravity more than 35 deg from
  the calibrated level for 100 ms = motors off as STOP, Mission told,
  event `robot lifted or tipped over ...`. Stage 4 image, 3 tries; in the
  Mission, lift at the end of every run. See
  `docs/team/course/IMU_REQUIREMENT_CHANGE.md`.

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
