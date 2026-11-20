# P5-IS34 testing guide: every stage, step by step

Everything below is built and compiles with 0 warnings (1 Nov). **Stage 1 passed on
the robot (1 Nov, S2_TRACK_MM set to 123); Stages 2-6 have not run yet.** Expect to tune the numbers marked
"NOT MEASURED" in section 2 after the first runs.

Test plan and test IDs: `testing/PLAN.md`. Record every run as a row in
the owner's `testing/subsystemN/TEST_LOG.md` and save the log under
`testing/subsystemN/logs/`.

## 1. Power: never USB and the power bank at the same time

Do not plug the laptop USB and the power bank (VIN) in together; we never
checked that the board allows it.

| Test | Power | Why |
|---|---|---|
| Stages 1-4 tests (they print the results) | **Laptop USB only**, long cable, laptop follows the robot | the log is on USB. The motors run on USB 5 V (bring-up motor and STOP tests ran on laptop USB); slower, but the speed controller makes up for it |
| Stage 5-6 / demo (results over WiFi) | **Power bank on VIN only**, no USB | results arrive on the PC over MQTT |
| **Any stage over WiFi** (`images/wifi/is34_test_stageN_wifi.uf2`, since 7 Nov) | **Power bank on VIN only**, no USB | the whole log arrives in the panel's **Robot log** tab; "Save log + summary" writes it and the **Run summary** to files in `logs/robot/`. START / STOP on the panel answer the test prompts too |

If the robot is too weak on USB for a test, run that test on the power
bank and read the result from the robot's behaviour, or move on to the
MQTT image, which sends every `[EVT]` line to the PC as well.

## 2. Numbers to measure once (ruler, 5 minutes)

They are guesses in the code. Measure them, change the define, rebuild.

| What | Define, file | Now |
|---|---|---|
| Track width, wheel centre to wheel centre | `S2_TRACK_MM`, `subsystem2/subsystem2.h` | 123 (from S2-M02, 1 Nov) |
| Robot width (outside of wheels) / length | `S2_ROBOT_WIDTH_MM`, `S2_ROBOT_LENGTH_MM`, same file | 150 / 200 |
| Line sensors in front of the wheel axle | `S3_SENSOR_TO_AXLE_MM`, `subsystem3/subsystem3.h` | 60 |
| Axle to front support (for L x sin, reported beside the main height) | `S4_L_MM`, `subsystem4/subsystem4.h` | 80 |
| Lower practice ramp: height, length, flat top? | write in `TEAM.md` section 7 | - |

Also tuned from tests: `S5_SERVO_LEFT_HIGH` (Stage 3 servo check),
`S5_BEAM_HALF_DEG` (S5-O01 widths), `S2_SPEED_BARCODE` (S3-B03).

## 3. Images

Prebuilt (no WiFi details needed):

| Image | Runs |
|---|---|
| `..._test_s1.uf2` | Stage 0 auto tests, then Stage 1: turns, slow speed, STOP |
| `..._test_s2.uf2` | then Stage 2: IR view, line + junctions, barcodes, then the course with the TEST mapping |
| `..._test_s3.uf2` | then Stage 3: servo check, sonar view, 15 width scans, then the course (bypass) |
| `..._test_s4.uf2` | then Stage 4: IMU check, tilt view, 5 ramp runs |
| `..._test_s6.uf2` | then the whole course with the TEST mapping A=L, B=R, C=F, D=U |

Built by you (they need your WiFi and broker IP, section 6):

| Image | Command (in `build_make`) |
|---|---|
| Demo image: MQTT mapping, START/STOP, telemetry | `make NET=lwip MQTT=1` |
| Course test that waits for the PC's mapping | `make NET=lwip MQTT=1 TEST=1 TEST_STAGE=6` |
| **Any stage test with its log over WiFi** (no USB cable) | `make NET=lwip MQTT=1 TEST=1 TEST_STAGE=n`, then copy `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt_test_sn.uf2` to `images/wifi/is34_test_stagen_wifi.uf2` (git-ignored: it holds the WiFi password) |

Build any image yourself: `make TEST=1 TEST_STAGE=n` (n = 1, 2, 3, 4, 6),
Pico SDK 2.2.0 (see `RTOS_STAGE0.md` section 1). Changing `TEST_STAGE`
rebuilds everything automatically.

Every test image first runs the 17 Stage 0 checks (about 25 s, all must
PASS), then prints `[DRIVE] ...` and **waits for START (GP20) before every
move**. Each move starts 1 s after you let go. **STOP (GP21) stops the
motors at once, at any time.**

## 4. Stage by stage

### Gates: a console PASS is not enough

Move to the next step only with the evidence in this table. Every
`[TEST] PASS` must be backed by the physical result it stands for, and
every test image re-runs the Stage 0 timing checks on the exact firmware
you are testing.

| Gate | Evidence before going on |
|---|---|
| Stage 0 | the 17 auto checks PASS at the start of the image under test |
| Stage 1 | right directions and signs (lifted), STOP works (lifted, then 5/5 on the floor), real 0.5 m distance measured, real turn angles measured |
| Stage 2A | at least 3 m of line following without loss; junction count = real count |
| Stage 2B | the codes placed on the track, in order, decoded in both directions |
| Stage 2C | the right move at each junction, U-turn, and an unmapped code reported and ignored (obstacle watch and IMU are OFF here) |
| Stage 3 | servo direction, widths of known obstacles inside W +/- U, a clean bypass; a covered sonar must stop the run |
| Stage 4 | measured ramp height, 5 runs within +/- 5 mm |
| Stage 5 | mapping and telemetry with the robot STILL first, then START/STOP while it moves |
| Stage 6 | repeated full runs, with logs and what you saw |

Stages 3 and 4 only need Stage 1 (and the line for the bypass); they can
be done in either order. All subsystems still start in every image:
`TEST_STAGE` only picks the hands-on part.

### Stage 1: motion + STOP (`_test_s1`), owner B2

Order (review, 1 Nov): wheels lifted first, then the floor.
1. **S2-M05 directions, wheels LIFTED**: both wheels forward 300 mm, then
   back. Watch that both turn the robot's forward way first. Pass: PASS
   line, and what you saw matches.
2. **S2-M06 STOP, wheels lifted**: press STOP after a second. Pass: wheels
   stop at once.
2a. **S4-S01 Motion hung, wheels still lifted** (added 17 Nov; keep the
   robot level, on a box, so the lift stop does not trip): press START,
   the wheels turn, then the test freezes the Motion task for 1 s on
   purpose. Pass: PASS line: `motors OFF, Motion-hung fault set`, off
   within about 220 ms of Motion's last run, and `fault cleared ... START
   works again` about 2 s later.
3. Robot on the floor from here. **S2-M03**: mark the front, it drives
   500 mm at 150 mm/s; **measure the real distance**. Pass: PASS line
   (speed) and the real distance 475-525 mm. Otherwise the mm/count scale
   is off: send the number.
4. **S2-M02 turns**: 3 rounds of left 90, right 90, 180 from a tape mark.
   The PASS line only means the commands finished; **pass/fail is your
   measured angles**: 90 +/- 10, 180 +/- 15. All off by the same factor =
   track width wrong: new `S2_TRACK_MM` = old x (commanded / real angle).
5. **S2-M04 STOP**: 5 tries on the floor, press STOP while it drives.
   Pass: 5/5 within 500 ms.

If a wheel turns the wrong way, or it spins when it should drive: STOP,
send the log (a motor or encoder wire differs from bring-up).

### Stage 2: line, junctions, barcodes (`_test_s2`), owner B3 (+B5)

0. **Before flashing: sensor layout** (see `subsystem3/subsystem3.h`).
   The two line sensors **straddle the tape**: L (GP1) just left of it,
   R (GP0) just right of it, about 30 mm apart centre to centre, the
   tape centred between them (both white). B is 40 mm behind them and
   70 mm left of L; if you move it, change `S3_BAR_AHEAD_MM` /
   `S3_BAR_LEFT_MM` (behind = negative) and rebuild.
1. **IR view (30 s)**: slide black tape under each sensor; L, R and B
   must each change. Nothing changes = sensor or pot. Robot centred on
   the tape: L 0 and R 0.
   **B is on its analogue output since 7 Nov** (AO to GP28, module on
   **3V3, not 5 V**); every image is built that way (`S3_BARCODE_ANALOG 1`)
   with **relative edges** (since 7 Nov): black once the reading rises 700
   above the lowest since the last bar (and is at least 1500), white once
   it falls 700 below the highest. Printed bars read much greyer than tape
   and the narrow ones blur, so fixed thresholds lost bars. B's pot does
   nothing. The live view prints the lowest and highest reading each
   second: white (paper, foam) must stay under 1500. If not, send the
   lines to B3. B shows 1 over white = `S3_BAR_BLACK_HIGH 0`. Back to the digital
   D0 on GP2: `S3_BARCODE_ANALOG 0` and rebuild every image.
2. **2A, S3-L01 / S3-L02**: robot on the line, START, let it run (up to
   60 s). Pass L01: `[TEST] S3-L01 PASS` = at least 3 m followed with no
   line lost, and you saw it stay on the line. L02: printed junction count = real count (write both down).
   Each junction prints `[S3] junction N at X mm: crossing` or `: branch
   left (seen by B)`; at the end `junctions this run: crossings C, left
   branches L`. Start with the tape centred between L and R.
   If it wobbles or runs off curves: the steering settings are at the top
   of `subsystem3/line.c`; try a change in `tools/sim/linesim.py` first
   (`python tools/sim/linesim.py`).
   **One place at a time**: then 7 short runs, each prompts where to put
   the robot: 20 cm before the X crossing (S3-L10), a left-only branch
   (L03), a T end (L05); the start of the tightest right curve (L06) and
   left curve (L07); 20 cm before a gap in the line on track 2 (L08); and
   on a straight with a black card held under R once it moves (L09, it
   must stop with `LINE SENSOR FAULT`). Each prints its own PASS/FAIL. No
   such place on the track: START, then STOP at once, and write "not on
   the track" in the log.
   Left branches: every first bar under B prints `B patch 19.8 mm (...)`;
   a left branch must show about 20 mm. Nothing printed = B does not pass
   over the branch (move B, or its pot).
   Each part of Stage 2 (2A, S3-B00, 2B, 2C) first asks: **START = run
   it, STOP = skip it**, so you can go straight to the barcodes.
3. **S3-B00, barcode read driving straight** (do this before 2B): 4 runs,
   each drives 300 mm straight with no line following. Put the robot so
   **B** (70 mm left of the line sensors) is about 5 cm before the first
   bar, square to the bars; where the line is does not matter. Runs: a
   code; the same code from the other end; "A small" (all at 50 mm/s, the
   speed inside a code); any code at the follow speed (120 mm/s) (the robot
   only slows once B is on the first bar, so this shows whether the start
   still reads). Each prints `code ended, N elements
   (mm): ...` (groups of 10 = one character), the decode, and a `B
   profile`: every bar `B` and space `s` with its width and its darkest
   or lightest reading. `!` = little contrast to the element before
   (under 1400): nearly missed, the sensor's spot is almost as wide as
   that bar or space. Many `!` = B must be lower (closer to the floor).
   B must be over the barcode only: a black floor line under B shows as a
   bar 20-35 mm wide. Pass: `S3-B00 PASS` on runs 1-3.
4. **2B, S3-B01..B03**: before each run **write down the codes on the
   track in the order the robot meets them**; 3 runs, forwards and
   backwards. Each prints `[S3] barcode X` or `READ_FAIL`. Pass: the printed
   sequence equals your written one ("it decoded something" is not a pass). "A small" must read too
   (S3-B03): if it fails, lower `S2_SPEED_BARCODE` and rebuild.
   Each code is read in **one pass** (no backing up since 7 Nov): it
   slows to 50 mm/s on the first bar and holds the steering straight
   while B is over the bars. A missed code is not read again.
5. **2C, S5-B01 / S5-B02**: Mission takes over, with obstacle watch and
   IMU **off** (they are tested in Stages 3 and 4). It first needs a
   mapping: pick and send one in the panel (below), or just press START
   for the TEST mapping (A=L, B=R, C=F, D=U); USB-only images always use
   the TEST mapping. START again: it follows the line, reads barcodes and
   does the move at the next junction (`[EVT] junction ...`). Check a
   U-turn barcode too. For S5-B02 use a code not in the mapping (e.g. E):
   it must report `barcode E ... -> NO MAPPING, ignored` and go straight
   at the junction (since 20 Nov; it used to stop there).

#### Barcode mapping (panel)

What a barcode means is not built into the robot: before a run you tell
it which **4 barcodes** mean **turn left, turn right, go straight and
U-turn**. It stores the last barcode it read and does that move **at the
next junction** (the U-turn too: instructor brief, TEAM.md 7.1 B6).

In the panel (`python tools/panel/is34_panel.py`), box "Barcode mapping":
1. Under each move pick its barcode letter (A-Z or 0-9). Each move needs
   a different letter. If the instructor gives you a line like
   `K=L,Q=R,X=F,M=U` or "K left, Q right, X straight, M U-turn", paste it
   in "or paste one" and press Use: the four boxes fill in.
   "Test set: A B C D" = A left, B right, C straight, D U-turn.
2. Press **Send mapping to robot**. The robot answers (`OK ...` in green,
   or why it refused) and the line next to it shows what the robot has:
   green "(same)" = ready. "(different: press Send)" or "no mapping yet":
   send again (it forgets the mapping when it restarts).
3. While it runs, every barcode it reads shows as an event, e.g. `barcode
   K -> LEFT at next junction`, then `junction 3 (crossing): LEFT
   (barcode K)`.

The panel remembers the last mapping you picked
(`tools/panel/is34_panel_mapping.txt`). On the wire it is the message
`K=L,Q=R,X=F,M=U` on `is34/cmd/map` (barcode=move, L R F U).

### Stage 3: obstacle (`_test_s3`), owner B5

1. **Servo check**: at the first START prompt the sensor must point
   LEFT, at the second RIGHT. Wrong way: `S5_SERVO_LEFT_HIGH 0`, rebuild.
2. **Sonar view (10 s)**: hand in front, distances must follow. `-1` =
   nothing in range, `-2` = no ECHO at all (sensor or wiring fault).
   Failure policy: during a run, 5 pings in a row with no ECHO stop the
   robot ("ultrasonic not answering"); a scan ray with no ECHO counts as
   blocked, never as open space. Check it once: cover/unplug ECHO during
   S5-O02 and the robot must stop.
   **Side look check** (only when `S5_SERVO_SIDE_DEG` is not 0; off
   since 8 Nov, so it is skipped), 12 s of `[S5] side N mm`: the sonar
   points 40 deg right. Hold a box **beside** the robot on the right, its
   face parallel to the robot's side (as when driving past it), about 20
   cm out, level with the sonar: it should read about 200. Then slide it
   backwards past the sonar: the reading should go away once the box is
   behind it. Tell me what it reads: this is what the side look relies
   on (if it never reads about 200, the side look hears nothing and the
   bypass simply uses the fixed 150 mm depth).
3. **S5-O01 width**: 3 tries, obstacle about **160 mm** straight ahead
   (where the forward watch stops and the bypass scans first, since 8
   Nov). Use 3 flat boxes of
   different widths (about 100-200 mm), one try each. Each try also
   prints which scan saw each edge (`edge seen by scan 1 L 1 R 1 ...`). Each try scans,
   reverses 100 mm, scans again and prints
   `width W +/- U mm (scan 1 ..., scan 2 ...)` plus the raw rays
   (`[SCAN] rays ...`). Write down the real width. Pass: the real width
   is inside W +/- U. If every width is too big (or too small) by about
   the same amount, raise (or lower) `S5_BEAM_HALF_DEG`.
4. **S5-O02 bypass**: obstacle on the line, START. It stops, scans
   (a second time, 100 mm further back, only if the first scan missed an
   edge: 15 Nov), reports the width, drives round on the clear side and finds the
   line again (`[EVT] bypass done`). Pass: no contact, stays within 300 mm
   of the line, back on the line within 500 mm after the obstacle.
   **Added 15 Nov (two checks):**
   - **Long box** (30-40 cm front to back, or two boxes end to end along
     the line): before driving back to the line it looks toward it; with
     the box still beside it says `box still beside: going 100 mm further
     along it` and tries again (up to 4 times, then `obstacle longer than
     expected`). Pass: no contact, back on the line behind the box.
   - **Something in a leg's way** (a second box where the robot will drive
     sideways or past the first): the sonar watches ahead on every leg and
     stops with `something in the way during the bypass` when it is
     closer than 150 mm. Pass: it stops before touching it.
   Over WiFi (`images/wifi/is34_test_stage3_wifi.uf2`) this part first
   waits for a mapping: send one from the panel, or press START once to
   use the TEST mapping `A=L,B=R,C=F,D=U` (it reminds you every 10 s);
   then START again to run (**mapping: see "Barcode mapping" below**). Start on a straight stretch; since 8
   Nov the line behind the box may also curve (the robot spins until a
   line sensor finds it). It always goes all the way round (15 Nov: it no
   longer takes a line on the leg past the box, where it once followed a
   different piece of track), then `line found N mm after the last turn`.
   Try a box standing on a T or X junction too (the 8 Nov fix): it must
   come back to the line it came along, not the side line. On the leg
   past the box the sonar looks 40 deg toward the box: expect `side look:
   box heard from A to B mm along, leg L mm (fixed F)` (B - A about the
   box's depth) and, for a deep box (25-30 cm front to back: try one),
   `side look: box still beside, going on`. `side look heard nothing`
   means the fixed 150 mm was used: say so.

### Stage 4: ramp height (`_test_s4`), owner B4

0. **Measure the hump**: the course's hump is a small triangle (2-5 cm
   high) that only the RIGHT wheel goes over (8 Nov). Write down its
   height with a ruler. Tape it down so it does not slide.
1. `[TEST] S4-I01 PASS` = IMU found. Tilt view (15 s): lift the front a
   little (`sin(pitch)` changes), then put the RIGHT wheel on a 2-3 cm
   block: `sin(side)` changes and `one-wheel` should read about the
   block's height.
2. **S4-R01**: 5 runs. Robot level, about 20 cm before the hump, pointing
   straight ahead, **the triangle in the path of the right wheel only**.
   START: it calibrates 1 s, then drives until 30 cm after the last bump
   (at most 1.5 m). For every bump it prints `[S4] bump N one wheel: side
   tilt peak sin S/1000, height T x sin = H mm` (and the older nose-up
   numbers), a `[S4T]` line every 10 ms while on it, and the IMU task's
   report `ramp peak height H mm (one wheel: ...; from the nose-down tilt
   P mm)`. Pass: H within +/- 5 mm of the measured height, 5 runs. Send
   the log and the measured height. The reported height is the side tilt
   x T x 0.63 (`S4_ONE_WHEEL_GAIN_PCT`, fitted 9 Nov); the test also
   prints it per bump (`x 0.63 for driving = ... mm`).
3. **S4-L01 lift stop** (lecturer, 13 Nov: lifting the robot ends the demo
   run). 3 tries: robot on the floor, 1 m clear; START: it drives slowly;
   pick it up as you will at the end of the demo (tilted, as usual). Pass:
   the wheels stop at once and the panel shows `robot lifted or tipped
   over ...: motors off`, all 3 tries (`[TEST] S4-L01 PASS`). A lift kept
   within 35 deg of level all the way is not seen (by design). In the
   Mission, lifting at the end of every run tests it again.

### Stage 5: telemetry + MQTT (demo image), owner B1

Needs section 6 first. Power bank only. **Robot on a stand (wheels
lifted) for steps 1-2**, on the floor only for step 3. On the laptop:
```sh
mosquitto_sub -h <broker IP> -t 'is34/#' -v
```
1. **S1-T01**: every second an `is34/telemetry` line appears (state,
   speed, distance, barcode, mapping, ramp peak). Every `[EVT]` result
   appears on `is34/event`.
2. Mapping: `mosquitto_pub -h <broker IP> -t is34/cmd/map -m "K=L,Q=R,X=F,M=U"`
   gives `is34/ack/map OK K=L,...`. A bad one (e.g. `K=L`) gives
   `REJECTED ...` and keeps the old mapping.
3. **S1-T02**: `mosquitto_pub ... -t is34/cmd/start -m go` starts the run,
   `... -t is34/cmd/stop -m now` stops it. 5 of each.

### Stage 6: whole course (demo image or `_test_s6`), whole team

**Barcode placement (15 Nov: the whole day's failed reads were this).**
Lay each barcode square to the line (bars at 90 deg to the tape), flat,
taped only on its white margins, and placed so the barcode sensor (about
7 cm left of the line sensors) passes over the **middle** of the bars. A
barcode at a slight angle or off to the side gave thin bars that read
faint and narrow, codes in pieces, or no read at all; the same print laid
square read cleanly (`narrow 3.4 mm, wide 9.3 mm`). Check with the robot
on the line just before it: where will the sensor cross?

**SYS-01**, 3 full runs: send the 4-character mapping, START, course with
barcodes, junctions, an obstacle and the ramp. Pass: every move at its
junction, obstacle passed with its width reported, ramp height reported,
line found after every manoeuvre.

## 5. What the robot does (demo image)

`IDLE` -> START (GP20 or `is34/cmd/start`) -> `CAL` (IMU level, 1 s) ->
`RUN`: follows the line at 120 mm/s (`S5_FOLLOW_SPEED`; 135 from 15 to 18 Nov, never run; 50 mm/s over bars).
- Every event (barcode read, junction move, obstacle, width, bypass,
  hump, any stop) also gives **one short beep** (the board's buzzer on
  GP22, since 20 Nov; silent = check the buzzer mute switch on the board).
- Barcode -> stores the character, reports it and the planned move.
- Junction -> stored character's move (L / R / F / U-turn); none stored
  = straight; a letter not in the mapping is reported and ignored (carries on, since 20 Nov).
- Obstacle closer than 180 mm -> stop (about 155-167 mm away), scan (again 100 mm further back
  only if an edge was not seen),
  report width +/-, bypass on the clear side, find the line, carry on.
- Ramp -> when level again, reports the peak height.
- Line lost / blocked bypass -> stops and reports. STOP -> `IDLE`.

## 6. WiFi + MQTT setup (for the demo image)

1. Phone hotspot on 2.4 GHz; laptop on the same hotspot.
2. On the laptop run Mosquitto with `bringup/tools/mosquitto.conf`.
3. Copy `config/wifi_secrets.example.h` to `config/wifi_secrets.h` and
   fill in SSID, password and the laptop's IP as `MQTT_BROKER_IP`.
   (This file is git-ignored: never commit it.)
4. `cd build_make && make NET=lwip MQTT=1` and flash
   `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`.
5. On the laptop: `pip install paho-mqtt`, then `python tools/panel/is34_panel.py`.
   Or, with no Python (any laptop): `tools/panel/exe/is34_panel.exe`, see
   `tools/panel/exe/README.txt` (Mosquitto still needed; the robot's image
   has the broker laptop's IP built in).
   The panel shows the status line and events, and has START, STOP
   (Space) and mapping buttons; it logs everything to `logs/panel/` (the robot's own console to `logs/robot/`).
   "Start broker" runs Mosquitto on port 1884
   (`tools/panel/mosquitto_robot.conf`): use that port in `wifi_secrets.h` if
   the Mosquitto Windows service already holds 1883 for localhost only.
6. **Tests over WiFi** (any stage, no USB cable): flash
   `images/wifi/is34_test_stageN_wifi.uf2`, power bank on VIN only. The
   panel's **Robot log** tab shows the same text as the USB console, from
   boot on (the robot keeps 8 KB until the WiFi is up). Prompts that say
   "press START" also take START on the panel; the Stage 2 "run it / skip
   it" questions take START / STOP on the panel. **Save log + summary**
   writes everything since the last **Clear** to
   `logs/robot/robot_log_<date>_<time>.txt`, and the **Run summary** tab
   (since 20 Nov: per START, the barcodes read, junction moves, each
   obstacle's width, scan time and bypass, hump heights, stops) to
   `robot_summary_<date>_<time>.txt` next to it: send both. The log is
   also saved as it arrives to `logs/robot/is34_<date>_<time>_robot.txt`.
   Lines `[panel: N log messages lost on the WiFi]` or `[panel: the
   robot dropped N bytes]` mark gaps (WiFi drop-outs, or a burst bigger
   than the robot's buffer). Stage 2C and 6 over WiFi wait for a mapping
   from the panel: for the TEST mapping send `A=L,B=R,C=F,D=U`.

## 6a. Change Requirement 1 tests (binary MQTT, fault word, Safety)

Since the evening of 8 Nov the robot's MQTT messages are binary
([MQTT_WIRE_FORMAT.md](../docs/team/MQTT_WIRE_FORMAT.md)): use the
**updated panel** (`tools/panel/is34_panel.py` with `is34_wire.py`); an
older panel shows nothing. The panel has a new "Faults / warnings" field.
On the laptop first: `python tools/panel/test_wire.py` (needs gcc) must
print PASS. Then on the robot, the extra tests in
[CHANGE_REQ1.md](../docs/team/CHANGE_REQ1.md) section 7.2:

- timing tables of a full run (at least 1 min: the normal and Mission
  images print the table once a minute since 20 Nov) with the normal image and with an
  `APP_PACKED=0` image;
- the WiFi switched off during line following (keeps going);
- STOP at the end of a quick START/STOP burst (stays stopped);
- one wheel held while driving (Safety stops the motors after about 1 s,
  START works again 2 s later);
- a good and a refused mapping.

## 6b. Tribunal 2 round: everything still owed (17 Nov)

Every requirement in the Tribunal 2 A1 table that has no robot log yet,
in four sessions. Bring a ruler and a phone stopwatch. Save the panel
log after **every** run, and write down what you measured.

**Session 1: Mission image on the course (about 1 hour).** 3 full runs
(SYS-01) at 120 mm/s, trip at 180 mm (stops about 155-167 mm away). In them:
- barcodes for Left, Right, Straight and U-turn (or change the mapping
  between runs) so every move is seen once; one run with a barcode
  that has no mapping: the robot reports it (`NO MAPPING, ignored`) and carries on straight (FR3; a stop until 20 Nov);
- measure each barcode-to-junction distance (NFR4);
- obstacle: ruler the stop distance (NFR6), the real box width (FR5),
  stopwatch the scan from stop to sensor centred again (NFR3);
- bypass: the furthest it gets from the line (a phone video is fine),
  no contact, back on its own line (FR6, NFR6a);
- at least one run with 3 m of following and no `line lost` (NFR1a);
- lift the robot at the end of every run: motors off (FR13);
- robot idle on the hotspot: hotspot off 10 s, on again, stopwatch
  until the panel shows it online (NFR5);
- quick START/STOP clicks on the panel, ending on STOP: it stays
  stopped (NFR11).

**Session 2: Stage 3 image, bypass extras (15 min).**
- a hand in front of the robot on a bypass leg: `something in the way
  during the bypass` (FR6, NFR12);
- a long box: `box still beside: going 100 mm further along it` (FR6);
- sonar unplugged **with the power off**, then run:
  `ultrasonic not answering` stop (FR9).

**Session 3: Stage 4 image, IMU (15 min).**
- 3 humps, each height measured with a ruler first (FR4, NFR8);
- S4-L01: 3 lifts (FR13).

**Session 4: Stage 1 and Stage 2 images (30 min).**
- S2-M02: 3 rounds, write down the real angles (NFR8);
- S2-M04: tries 3-5;
- a wheel held while driving, then START after 2 s (FR9, FR14);
- one barcode, 20 passes, count the reads, a few in the reverse
  direction (FR2, NFR8).

**Checked without the robot driving:** message sizes and format
(FR15) by `python tools/panel/test_wire.py` (PASS 17 Nov, 15 golden
vectors). The Motion-hung stop (NFR10) is Stage 1 step 2a, S4-S01 (wheels
lifted): run it in session 4.

## 7. Send back

For each test: the whole log, plus what you wrote down (real angles, real
widths, real junction count, which barcodes, measured ramp height).
