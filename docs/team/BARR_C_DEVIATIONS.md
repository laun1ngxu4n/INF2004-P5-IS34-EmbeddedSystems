# Barr C coding standard: scope and deviations

Our code follows the Barr Group *Embedded C Coding Standard* (2018)
(requirement NFR7). This page says what it covers, how it is checked, and
where we deliberately deviate, and why.

## Scope

| Code | Barr C? |
|---|---|
| `app_program/` (our robot code and its tests) | **yes** |
| `kernel/`, `include/`, `lib/`, `device/`, `etc/`, `config/` (micro T-Kernel 3.0, the Pico SDK glue, lwIP, the CYW43 driver: given to us with the course template) | no: third-party code, kept as delivered so updates and the course's own fixes still apply |
| `bringup/` (October bring-up tests) | written to Barr C at the time; frozen |
| `tools/` (Python, laptop) | not C |

## How it is checked

- `tools/barr/check.py`: counts what is left, rule by rule
  (`python tools/barr/check.py`, `--list <rule>` for the places). Every
  rule it checks is at 0 except the deviations below.
- The rules `check.py` does not count were reviewed on 21 Nov (scripts
  over every file of `app_program/`, and reading the hold, bypass,
  barcode and test code). What does not comply is D7-D10 below and the
  short list under *Known small departures*. The rest complies: no
  `auto`, `register`, `abort()`, `exit()`, `setjmp()`, `#pragma` or inline
  assembly; no `short` / `long`; every float constant ends in `f`; the
  Barr prefixes (`g_`, `p_` / `pp_`, `b_`, `h_`, in that order) on every
  variable; interrupt handlers `static` and named `..._isr`; every task
  named `..._task`; every public function prototyped in a header (except
  the D2 hooks and the kernel's `usermain()`); every `switch` with a
  `default` and its `break`s at the `case`; constants left of `==` and
  `&&` / `||` operands in brackets (checked again by a second script).
- `app_program/.clang-format`: the layout rules (indent, braces, spacing,
  80 columns, no tabs).
- `tools/barr/transform.py`, `fix_layout.py`, `names.py`: the scripts
  that made the clean-up (constants on the left of `==`, brackets, one
  variable per declaration, `for (;;)`, blank lines around blocks, Barr
  names); `check.py` uses `transform.py` and `names.py`. The one-off
  scripts, run once and kept for the record, are in `tools/barr/oneoff/`
  (`doxygen.py` and `add_briefs.py`: Doxygen comments; `final_else.py`;
  `make_renames.py` with `renames_6.1i.txt`).
- `tools/barr/compare.sh`: builds all 12 images and compares them byte
  for byte with the images from before a clean-up. A clean-up that changes
  no byte cannot change what the robot does. Where the bytes had to change
  (module renames, refactors), the old and new objects were compared
  function by function instead; the commits say which functions changed.

## Deviations

| # | Rule | Deviation | Why |
|---|---|---|---|
| D1 | 4.1b module names unique in 8 characters | `subsystem1` ... `subsystem5` and `subsystem1_test` ... `subsystem5_test` share "subsyste". | The course template's names (folders, files, and the `subsystemN_init/start/test()` entry points `usermain()` calls). Kept as given. Our own modules have unique names (`line`, `barcode`, `motion`, `motor`, `imu`, `imu_i2c`, `safety`, `mission`, `obstacle`, `telemetry`, `app_*`). |
| D2 | 6.1i public names start with the module name; 5.2a fixed-width types | `app_mqtt_rx()`, `app_mqtt_tx()` (`telemetry.c`) keep their names and their `int` / `unsigned` / `char` parameters; `tm_console_mirror()` keeps its name and `UB` / `INT` meaning. | The RTOS-side glue calls them by these names with these types (weak symbols in `lib/liblwip/.../mqtt_utk.c` and `lib/libtm/.../tm_com.c`). |
| D3 | 5.2a fixed-width integer types | Values passed to or received from micro T-Kernel calls keep the kernel's types (`ID`, `ER`, `PRI`, `FP`, `TMO`, `RELTIM`, `T_CTSK` ...), e.g. `UINT ptn` for `tk_wai_flg()` in `mission.c`. | The kernel API's own types; converting at every call would add casts (each needing a comment, 1.6a) without making anything safer. Everywhere else: `int32_t`, `uint32_t`, `uint8_t`, `float32_t`, `bool`. |
| D4 | 6.3a no function-like macros | `APP_LOCK()` / `APP_UNLOCK()` (`app_lock.h`); `TEST_INIT()`, `TEST_CHECK()`, `TEST_SUMMARY()`, `TEST_PERIODIC()`, `TEST_TIMEOUT()` and their shared body `TEST_TIMED_()` (`test_report.h`); `APP_FIELD(bits)` (`app_types.h`, Change Requirement 1). | `APP_LOCK` wraps the kernel's `DI()` / `EI()`, which save the interrupt state into the caller's own variable. The `TEST_` macros count into the calling test's own counters and report the caller's file and line. `APP_FIELD` is part of a declaration (a bit-field width, or nothing in the `APP_PACKED=0` build), which no function can be. Every other macro became a function or a constant. |
| D5 | 6.2a functions at most 100 lines ("all reasonable effort") | 14 functions are longer (lines on 21 Nov; on 8 Nov 472, 349, 275, 267, 129, 117, 109, 109: the fixes since then added to them): the task loops `line_task` (485), `motion_task` (381), `on_obstacle` (286), `imu_task` (347 on 23 Nov), `pass_box` (133), `barcode_task` (120); the decoders / scan `decode_one_star` (109), `obstacle_scan` (121); and six hands-on test scripts (`follow_and_watch` 208, `ir_live` 166, `subsystem3_test_stage` 159, `ramp_run` 151, `bench_profile` 118, `subsystem5_test_stage` 108). | Each task loop is one state machine whose state lives across the loop; the deep `if` nesting (8.2b) was already moved out into functions, and splitting further would scatter that state through tuned, hardware-tested control code close to the deadline. The test scripts are linear sequences of prompts and checks. |
| D6 | 1.7c / 1.7d avoid `goto` and `continue` (preferred practice, not mandatory) | 15 `goto`s, all forward to one exit label in the same function (`publish` in `motion_task`, `done` in `imu_task`, `out` / `fail` in the bypass); 14 `continue`s, in loops that skip the rest of one pass (e.g. barcode decoding). | The rule allows a forward `goto` to a later label; one exit path for a failed manoeuvre is clearer than repeating it at every step. |
| D7 | 5.3a no bit-fields in signed integer types | 19 signed bit-fields (Change Requirement 1): the command's speed and distance (`motion.c`), the measured wheel speeds (`motion.h`), the motor duties (`motor.c`), the line steer (`line.c`), the IMU's tilt and climbed height (`imu.c`), and the obstacle profile (`obstacle.h`). | Change Requirement 1 asks for the fewest bits each value's range needs, and these ranges have a sign: speeds, duties, steer, tilt and edges go both ways, `front_mm` also holds the codes `SONAR_OPEN` (-1) and `SONAR_FAULT` (-2), and the profile's counts and uncertainties are used in signed arithmetic (e.g. `sgn x unc`). Each value is kept inside its field's range where it is written: clamped (`app_clamp` or a limit test, FR15), or, for the line steer (+-40 mm/s), limited by the rule that makes it. GCC, the only compiler for this target, keeps them as two's complement. No bit-field structure is sent as it is (`wire.c` packs every message bit by bit), so padding and bit order (5.5) never reach a message. An unsigned field with an offset would add a conversion at every read and write. |
| D8 | 5.4b floats: no `==` / `!=` tests; check results with `isfinite()` | `wheel_duty()` (`motion.c`) tests `0.0f == target`; no `isfinite()` call anywhere. | The test asks whether the wheel is told to stand still, and where that is meant the caller sets exactly `0.0f` (the clamps in `motion_task`); a computed target that is only near 0 is driven as a small speed, as intended. No calculation can give an infinity or a NaN: every input is a small integer from a sensor or a command (encoder counts, speeds clamped to +-1023 mm/s, 16-bit IMU readings, sonar distances); every divisor is a constant or is checked first (a command speed of 0 is refused; the IMU's vector lengths are checked against 1, 0.1 or 0 before dividing; sin(90 - angle) >= 0.76 over the +-40 deg scan); `app_sqrtf()` returns 0 for a value <= 0 and divides only by its own estimate, which stays above 0. |
| D9 | 1.8c `volatile` on data shared with an interrupt or between tasks | Data used by an interrupt or by two tasks without `volatile`, in six cases: (1) used only under `APP_LOCK`: the telemetry outbox and log copy, Motion's command mailbox and status, the barcode ignore window, last result, mapping, debug and mark copies, the timing table; (2) read by another task as one aligned word: `g_mission_pings` (one writer), `g_log_dropped` (written under the lock); (3) filled by an interrupt behind a `volatile` index: the barcode edge ring `g_ring` (`g_ring_w`); (4) set before the interrupt or handler that reads it starts, or under the lock: the GPIO callback table `g_gpio_cb`, the flag ID `gh_flg_mission`, the task and handler IDs, the timing slot numbers; (5) handed between two tasks by a `volatile` flag: the servo angle `g_servo_now` (`gb_scanning`); (6) used only inside one interrupt: `g_enc_prev` (the encoders' last state), `g_rel_lo` / `g_rel_hi` (the barcode reader's levels). Also the analogue levels `g_env_w` / `g_env_b`, written in the ADC interrupt and read once per call by `barcode_adc()` (the Stage 2 sensor display). | What keeps shared data right is NFR11's rule: one writer, or only under the one lock. `APP_LOCK` is `DI()`, which calls `disint()`, an external assembly function, so the compiler cannot keep a shared value in a register across the lock. On this single core an interrupt runs to its end before the task goes on, and the task reads the `volatile` index before the entries. Aligned 32-bit and 8-bit reads and writes are single instructions on the Cortex-M0+, so no reader sees half a value. Data only one interrupt uses is shared with nothing. |
| D10 | 4.3b the order of a source file's sections; 7.2c file-scope data at the top | In `barcode.c`, `line.c`, `mission.c`, `motion.c`, `motor.c`, `telemetry.c`, `obstacle.c`, `app_gpio.c` and the Stage 2-4 test files, a tuned constant or a piece of file-scope data sits just above the functions that use it (e.g. `HALF_RATIO_X10` and `g_half_rev` with the half-code decoder, `EXTEND_MM` with the bypass, `gb_held` with `motion_steer_held()`), not in one block at the top; and private helpers come before the public functions that use them. | Each tuned constant carries its evidence (the dates, measurements and runs behind the number) and is read with the code it tunes; one block at the top would part the number from what it does. Helpers first means no forward prototypes to keep in step. |

### Known small departures (found 21 Nov, left until after the demo)

Layout and naming points, and two signed / unsigned details of values
that are never negative. None changes what the robot does. They are left
as they are so the images tested on the robot stay unchanged until the
demo; most can be tidied byte-identically (`compare.sh`).

| Rule | Where | What |
|---|---|---|
| 3.3b blank line before a block | `motor.c:216`; `barcode.c:459`, `529`, `742`; `subsystem3_test.c:454` | `check.py` does not count these: it takes a line starting with `*` (`*p_x = ...;`) for a comment |
| 3.2b members aligned | `subsystem3_test.c:841` | `bool b_want_fault;` is not aligned with the members above it |
| 5.1b types named via `typedef` | `subsystem3_test.c:758`, `833` | two test tables are anonymous `static const struct` |
| 7.2d pointers start at `NULL` | `app_timing.c:28`, `45`, `98`; `telemetry.c:184`, `424` | declared without a value; each is set before it is used |
| 4.3c a `.c` includes its own `.h` | `app_timing.c` | gets `app_timing.h` through `app.h` only |
| 5.3b no bitwise operators on signed values | junction sides: `line.c:207`, `225`, `236`; `mission.c:479`; `subsystem3_test.c:293`, `307`. Barcode levels: `barcode.c:290-346`, `457-458` | the junction mask (1, 2 or 3) is an `int32_t`; each shifted barcode level is >= 0 (a 12-bit reading, or a difference taken the way its `if` tests) |
| 5.3c `u` on unsigned constants | `motion.c:495`; `obstacle.c:158`, `661`, `737`; `mission.c:147` | `10` and `90` without `u`; an `int32_t` (0..3) stored in an unsigned field |
| 6.1h descriptive names | `mission.c:158` | `mv()` waits for a Motion command to end |
| 1.6a cast comment on the range | `motion.c:399` | gives the sign convention only; the value is \|steer\| <= 40 mm/s (an 8-bit field), exact as a float |

### Notes on how rules were read

- **1.6a casts**: every cast has a comment on its value range (one gives
  only the sign: see the list above). A `(void)`
  cast is not counted: it converts no value, it only discards one (an
  unused parameter, a result not needed). The one `char *` -> `uint8_t *`
  cast for T-Monitor's text functions is in `app_str()` (`app.h`).
- **2.2h Doxygen comments**: a function counts as documented when its
  `/** ... */` comment is at its definition or at its prototype in the
  header (`check.py` reads it this way).
- **6.2c one exit point** (a preferred practice, not a rule): 74
  functions return in more than one place, mostly early returns from a
  guard (`if (...) { return ...; }`). Not counted as deviations.
- **5.5 structures on the wire**: no structure is sent as it is;
  `wire.c` packs each MQTT message bit by bit (little-endian, versioned).
- **5.6a `bool`**: our code uses `bool`, `true`, `false` everywhere. The
  kernel / BSP calls that take a `BOOL` (an `unsigned int`) get our `bool`
  values, which convert to 1 / 0.
- **6.4a tasks**: task functions end in `_task`, as the rule allows.
- **6.1i kernel macros**: `LOCAL` (`static`), `EXPORT` (empty) and
  `IMPORT` (`extern`) from `include/sys/machine.h` are used as they are.

## What the clean-up changed in the robot's behaviour

Almost nothing: most steps were proven byte-identical. The exceptions:

- `motion_drive()` / `motion_turn_at()` refuse a speed of 0 (`E_PAR`);
  before, it made the time limit infinity cast to `uint32_t`, undefined
  behaviour. No call in the code passes 0.
- `bool` instead of `BOOL`: booleans are bytes; every value stored in one
  was checked to be 0 or 1 already.
- The 8.2b refactors (statements moved into functions, same order):
  `barcode_task`, `line_task`, `imu_task`, `obstacle_task`, `mission_task`
  (START), `app_mqtt_rx`, and the Stage 2 bench test. `line_task` now
  writes `LINE_SEARCH` or `LINE_LOST` once instead of `LINE_SEARCH` and
  then `LINE_LOST`.
