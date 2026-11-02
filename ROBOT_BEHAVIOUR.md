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
- 150 mm/s everywhere (the whole course runs at the slow barcode speed).
- Centre sensor C rides on the line, side sensor S sits just right of it:
  the robot follows the line's right edge.
- Steering: ±25 mm/s between the wheels for a small correction, ±40 mm/s
  for a big one. It builds up gradually and eases off at once.
- C off the line: slows to 20 % (about 30 mm/s) and turns back on the spot.
- **Line lost**: C off the line for more than 250 mm or 4 s. It stops and
  reports "line lost".

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
- Seen when S and C are both black briefly, then S goes white while C
  stays black.
- No letter kept: straight on. Letter in the mapping: drives 50 mm (wheels
  over the junction), spins on the spot (left +90, right -90, U-turn 180,
  each wheel 120 mm/s), then follows again (junction and line-lost
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
     (about 225 mm for a 150 mm box)
  3. turn back; past: distance to the box + 150 mm (deepest expected box)
     + robot length + 50 mm (about 730 mm)
  4. turn 90 toward the line; drive until C sees it (gives up 200 mm past
     where it should be)
  5. wheels onto the line, turn 90 to face the old direction, follow.

### Ramp / hump (IMU)
- Climbed height = distance driven x sin(tilt), all the time.
- Readings during jolts are skipped (|acceleration| more than 5 % off 1 g).
- Climbed 8 mm or more and then level (under about 1.7 deg) for 100 mm:
  reports `ramp peak height H mm`; no second report for the next 600 mm.

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
- Sonar beam is about 30 deg wide: it "sees" a box well past its edge, so
  widths are only as good as the beam correction (`S5_BEAM_HALF_DEG`).
  Round or soft objects reflect badly.
- Servo travel is limited by the cables at the back: ±40 deg needs the
  cables routed. ±25 deg was too narrow to see past a box.
- The LSM303D has **no gyro**, only an accelerometer: any push (bumps,
  braking) looks like tilt. The jolt filter helps but cannot remove it all.

**Behaviour**
- Follows only the line's right edge; a line that branches to the right
  can look like a junction. Weaving can give **false junctions**.
- Line lost = stop. It does not search for the line again.
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
- [ ] Ramp height within ±5 mm (S4-R01): 10 mm hump read 18 and 66 mm;
  jolt filter not tested.
- [ ] Whole course, 3 runs (SYS-01).

**Not measured (the code uses guesses)**

| Value | Guess | Affects |
|---|---|---|
| Robot width (`S2_ROBOT_WIDTH_MM`) | 150 mm | bypass: how far out |
| Robot length (`S2_ROBOT_LENGTH_MM`) | 200 mm | bypass: how far past |
| Line sensors to wheel axle (`S3_SENSOR_TO_AXLE_MM`) | 60 mm | where it spins at a junction |
| Axle to front support, L (`S4_L_MM`) | 80 mm | hump height = L x sin(tilt) |
| Tape width, C to S gap | unknown | false junctions |
| Real junction count on the track | unknown | S3-L02 |

**Tests short of their target**
- [ ] S3-L01: 3 m of line following (track was only 1.2-2.1 m; about 2 m
  followed without losing the line).
- [ ] S3-L02: junction count vs the real count.
- [ ] S1-T02: remote START/STOP 3 of 5 rounds (all passed).
- [ ] Stage 1 and later results not yet written into the `testing/*/TEST_LOG.md` tables.
