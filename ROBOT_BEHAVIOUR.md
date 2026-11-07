# P5-IS34 robot: what it does, its limits, what is not done

Plain-language overview of the mission image (`make NET=lwip MQTT=1`), with
the numbers the code uses on 2 Nov 2026. Test results: [TEST_STATUS.md](TEST_STATUS.md).
How to run the tests: [TESTING_GUIDE.md](TESTING_GUIDE.md).

## 1. What the robot does

### Start
1. Send the mapping from the laptop panel (`tools/is34_panel.bat`), e.g.
   `K=L,Q=R,X=F,M=U`: exactly 4 pairs, each barcode letter to Left,
   Right, Forward or U-turn. A bad mapping is rejected and the old one kept.
2. START: button GP20 or the panel.
3. **CAL**, 1 s: the robot must stand still and level; the IMU learns "flat".
4. **RUN**: line following starts.

### Line following
- 120 mm/s everywhere (the whole course runs at the slow barcode speed;
  150 ran past tight curves on 7 Nov).
- **The two line sensors straddle the tape** (since 7 Nov): L just left
  of it, R just right of it, about 30 mm apart for the 20 mm tape, so
  both read white when centred.
  | L | R | Meaning | Does |
  |---|---|---|---|
  | white | white | centred, or a gap | straight |
  | black | white | line moving left | turn left |
  | white | black | line moving right | turn right |
  | black | black | a cross line | straight (a crossing) |
- A correction is hard at once (±40 mm/s between the wheels) and the
  robot slows to 50 %, then to 25 % after 10 mm of it (turn radius about
  9 cm, then 4 cm). Steering builds up over 60 ms and eases off at once.
  (Until 7 Nov it started gentle at full speed: on tight left curves the
  tape slipped out past L before the hard turn began.)
- **Overshoot**: on a tight curve the tape can pass right under a sensor
  and out the other side, which also reads "both white". After a
  correction of 15 mm or more and then 40 mm of white, it turns back
  toward the side that saw the line last (searching at 20 %).
- **Gaps** need nothing special: both white = straight on.
- **Line lost**: 400 mm of white (no search), or 250 mm / 4 s of
  searching. It stops and reports "line lost". So a line that ends is
  driven past by up to 400 mm before it stops.
- **Stuck line sensor**: one sensor black for 300 mm of driving, or both
  black for 150 mm: it stops and says which. A robot standing still
  never trips this.

### Barcodes (Code 39, barcode sensor B)
- Inside a barcode: 50 mm/s.
- After bar, space, bar: backs up 60 mm, then reads forward at 40 mm/s
  driving straight (no steering). Gives up 340 mm later; no new rescan for
  the next 200 mm.
- Bar widths are measured with the wheel encoders (distance, not time).
  Read in either direction.
- Reports `barcode F (narrow 3.5 mm, wide 10.5 mm) -> LEFT` or `READ_FAIL`
  and **keeps the letter until the next junction**.

### Junctions
- **Crossing** (X, crossroads, T end): L and R both black for 12-45 mm
  (one tape width, more when crossed at an angle), **after driving
  straight** for the 20-40 mm before it. On a tight curve the robot can
  sweep across its own line at an angle (L, then both, then R): that
  came out as 2 false crossings on 7 Nov, and is now ignored because it
  was correcting just before.
- **Left branch** (an arm to the left only): the barcode sensor B
  (**40 mm behind** the line sensors, 70 mm left of L) sees a black patch
  12-28 mm long (reported as soon as B leaves it), and the robot was
  driving straight before that place (after it, L touches the arm and
  the robot turns a little toward it, so that is not checked). The line task remembers where it was correcting,
  by distance, to check this. On a curve it corrects all the time, so a
  line B sees there does not count. Barcode bars (narrow 3.5, wide
  10.5 mm) and grout lines are thinner. Reported about 50 mm after the
  line sensors crossed it; Mission then drives the rest of the 115 mm.
- **A branch to the right only** cannot be told from a correction (R
  touches it like a curve): the track has none (7 Nov). It gives a small
  jerk toward it.
- A crossing and B's left arm within 40 mm: one junction.
- After a barcode that means L, R or U: follows at 100 mm/s (not 150)
  until the junction.
- No letter kept: straight on. Letter in the mapping: drives until the
  wheels are over the junction centre (115 mm after the line sensors
  crossed it, measured
  with the wheel encoders), spins on the spot (left +90, right -90, U-turn
  180, each wheel 120 mm/s), then follows again (junction and line-lost
  detection off for the first 60 mm).
- Letter not in the mapping: stops, "no mapping for barcode E".

### Obstacles (ultrasonic on the servo)
- While running: one ping straight ahead every 60 ms.
- **Two readings in a row closer than 250 mm**: stop, wait until still.
- No echo 5 times in a row (0.3 s): stops, "ultrasonic not answering".
- Scan 1: servo **40 deg left to 40 deg right in 5 deg steps** (17 rays),
  turning slowly (2 deg per 20 ms), 5 pings per ray, median. About 6 s.
- **Reverses 80 mm**, scan 2.
- Reports `obstacle width W +/- U mm` (two scans averaged; ± from the 5 deg
  step, the sonar beam and the difference between the scans).
- Picks the side that is clear beyond the edge (if both, the nearer edge);
  neither clear: stops, "no clear side".
- Drives round on a rectangle, pinging before each long leg:
  1. turn 90 to the clear side
  2. out: half of max(width + U, 200 mm) + half the robot width + 50 mm
     (about 235 mm for a 150 mm box)
  3. turn back; past: distance to the box + 150 mm (deepest expected box)
     + robot length + 50 mm (about 730 mm)
  4. turn 90 toward the line; drive until C sees it (gives up 200 mm past
     where it should be)
  5. wheels onto the line, turn 90 to face the old direction, follow.

### Ramp / hump (IMU)
- Climbed height = distance driven x sin(tilt), all the time. Tilt is
  measured from the "driving level", which follows the tilt slowly (about
  5 s) while it is within about 3.4 deg, so the chassis leaning a little
  while driving does not count as a climb.
- The IMU's own 50 Hz filter cuts motor vibration; readings during jolts
  are skipped (|acceleration| more than 10 % off 1 g).
- Climbed 8 mm or more (climbs only: nose-down dips are ignored) and then
  level (under about 1.7 deg) for 100 mm: reports `ramp peak height H mm`
  plus L x peak nose-up tilt; no second report for the next 600 mm.

### Always
- STOP (button GP21, or the panel / Space bar): motors off at once, back to IDLE.
- To the laptop: status line every 1 s, live sensors every 0.5 s, every
  result above as an event. Sonar distance is live in the panel even when
  idle.

## 2. Limitations (by design or hardware)

**Sensors**
- The IR modules are digital (on/off) with no hysteresis: they flicker at
  every edge. The code filters this (line: 2 readings agree; barcode: a
  level must hold 1.2 mm of travel), at the cost of a little delay.
- On the tiled floor, the dark **grout lines can read as black**. Pot
  setting matters, and it shifts when the robot is handled.
- The barcode sensor is not very sensitive; bars only read reliably at
  low speed and straight across.
- **IR: what can go wrong** (and what covers it):

  | Risk | Covered by |
  |---|---|
  | Pot set wrong, or shifted by handling | nothing in code: IR live view before every session |
  | Paper and foam reflect differently | nothing in code (analogue B learns its levels): live view on each surface |
  | Room light, sunlight, glossy tape | nothing: try once near a window |
  | Sensor stuck black | stop after 300 mm (one) or 150 mm (both) of driving |
  | Sensor stuck white / cable out | not detected: that side never corrects, the robot drifts off and stops with "line lost" |
  | L-R gap wrong for the tape | build: about 30 mm for 20 mm tape, tape centred between them |
  | Tight curves (about 15 cm on the foam track) | hard steering and slow after 25 mm; overshoot search; S3-L06/L07 |
  | Gap in the line | straight on (up to 400 mm); S3-L08 |
  | Left branch met while correcting, or on a curve | ignored (not trusted); S3-L03 |
  | A branch to the right only | looks like a correction: not detected (none on the track) |
  | Foam tile joins, grout, marks about 20 mm under B | could look like a left arm; watch `B patch` prints |
  | Another track line within 70 mm of the robot's path | B could see it as a left arm; watch `B patch` prints |
  | Barcode under 40 mm from a left branch | both lost |
- Sonar beam is about 30 deg wide: it "sees" a box well past its edge, so
  widths are only as good as the beam correction (`S5_BEAM_HALF_DEG`).
  Round or soft objects reflect badly.
- Servo travel is limited by the cables at the back: ±40 deg needs the
  cables routed. ±25 deg was too narrow to see past a box.
- The LSM303D has **no gyro**, only an accelerometer: any push (bumps,
  braking) looks like tilt. The jolt filter helps but cannot remove it all.

**Behaviour**
- Line sensors straddle the tape. Crossings are seen by both; left
  branches by B, and only if there is **at least 40 mm of white** between
  the branch and any barcode or other mark next to the line (otherwise
  they read as one code). Right-only branches are not detected.
- Curves: corrections both ways the same. A curve tighter than the hard
  steering at full speed (about 23 cm radius) relies on slowing to 20 %;
  a very tight one can overshoot and needs the search. A branch on a
  curve is ignored, and on a curve the turn point (drives straight
  115 mm) is less accurate. **Not tried on the robot yet.**
- Line lost = stop, after 400 mm of white (it cannot tell centred from
  lost sooner) or a failed search.
- The bypass is a fixed rectangle planned for boxes up to 200 mm wide and
  150 mm deep; it assumes nothing else is next to the obstacle.
- A ramp is only reported once the robot is level again for 100 mm. On a
  short hump (shorter than the robot) the robot tips instead of climbing,
  and the sum of distance x tilt does not equal the hump height.
- A hard STOP while moving can give a false ramp report (braking tilt).
- Single speed (150 mm/s) for the whole course.
- Safety (collision / stall detection) is not built: the safety task only
  runs the timing checks.

**WiFi / laptop**
- The broker IP is built into the image (`config/wifi_secrets.h`): if the
  laptop's IP on the hotspot changes, rebuild.
- Phone hotspot must be 2.4 GHz. Broker on port 1884 (the laptop's
  Mosquitto service holds 1883 for itself only).

## 3. Not done yet

**Not working / not proven on the robot**
- [ ] Barcode decode: never decoded a full code yet (best: 7 of 29 bars of
  `*F*`). Straight-read fix not tested.
- [ ] Barcode in both directions, and "A small" (S3-B03).
- [ ] Junction moves from barcodes (Stage 2C): needs barcodes first.
- [ ] Obstacle width with the ±40 deg scan (S5-O01): not run.
- [ ] Bypass (S5-O02) and the "cover the sonar" stop: never run.
- [ ] Ramp height within ±5 mm (S4-R01): 2 of 7 runs on the 10 mm hump
  passed (9.0 and 8.1 mm), 6 Nov. The hump was not held down and the IMU
  cable dropped out twice; see TEST_STATUS.md Stage 4.
- [ ] Whole course, 3 runs (SYS-01).

**Robot measurements** (measured 6 Nov unless marked)

| Value | In the code | Affects |
|---|---|---|
| Robot width (`S2_ROBOT_WIDTH_MM`) | 170 mm (130 body + 17 mm barcode arm, counted both sides) | bypass: how far out |
| Robot length (`S2_ROBOT_LENGTH_MM`) | 200 mm (150 body + IR/barcode bar in front, **bar not measured**) | bypass: how far past |
| Line sensors to wheel axle (`S3_SENSOR_TO_AXLE_MM`) | 115 mm (was a guess of 60) | where it spins at a junction |
| Axle to front support, L (`S4_L_MM`) | 80 mm | hump height = L x sin(tilt) |
| Tape width (`S3_TAPE_MM`) | 20 mm | junction length window 12-28 mm |
| Barcode sensor B from C (`S3_BAR_AHEAD_MM`, `S3_BAR_LEFT_MM`) | **40 mm behind**, 70 mm left (corrected 6 Nov, was "40 ahead, 50 left") | left junctions |
| L to R gap, centre to centre (`S3_LR_GAP_MM`, build guide only) | about 30 mm, the tape centred between them (7 Nov) | steering; both white when centred |
| Real junction count on the track | **not measured** | S3-L02 |

**Tests short of their target**
- [ ] S3-L01: 3 m of line following (track was only 1.2-2.1 m; about 2 m
  followed without losing the line).
- [ ] S3-L02: junction count vs the real count.
- [ ] Junction turns with the measured 115 mm sensor-to-axle distance
  (was 60), measured from the junction centre: not tried on the robot.
- [ ] Straddle line following (7 Nov), crossings, left junctions seen by
  B, overshoot search, stuck-sensor stop, slower approach after a
  barcode: not tried on the robot (S3-L03, L05..L10).
- [ ] S1-T02: remote START/STOP 3 of 5 rounds (all passed).
- [ ] Stage 1 and later results not yet written into the `testing/*/TEST_LOG.md` tables.
