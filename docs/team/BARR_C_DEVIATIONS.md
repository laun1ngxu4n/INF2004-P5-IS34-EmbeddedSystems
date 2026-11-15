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
- `app_program/.clang-format`: the layout rules (indent, braces, spacing,
  80 columns, no tabs).
- `tools/barr/transform.py`, `fix_layout.py`, `names.py`, `doxygen.py`,
  `add_briefs.py`: the scripts that made the clean-up (constants on the
  left of `==`, brackets, one variable per declaration, `for (;;)`,
  blank lines around blocks, Barr names, Doxygen comments).
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
| D4 | 6.3a no function-like macros | `APP_LOCK()` / `APP_UNLOCK()` (`app_lock.h`); `TEST_INIT()`, `TEST_CHECK()`, `TEST_SUMMARY()`, `TEST_PERIODIC()`, `TEST_TIMEOUT()` (`test_report.h`); `APP_FIELD(bits)` (`app_types.h`, Change Requirement 1). | `APP_LOCK` wraps the kernel's `DI()` / `EI()`, which save the interrupt state into the caller's own variable. The `TEST_` macros count into the calling test's own counters and report the caller's file and line. `APP_FIELD` is part of a declaration (a bit-field width, or nothing in the `APP_PACKED=0` build), which no function can be. Every other macro became a function or a constant. |
| D5 | 6.2a functions at most 100 lines ("all reasonable effort") | 14 functions are longer: the task loops `line_task` (472), `motion_task` (349), `on_obstacle` (275), `imu_task` (267), `pass_box` (129), `barcode_task` (117); the decoders / scan `decode_one_star` (109), `obstacle_scan` (109); and six hands-on test scripts (`follow_and_watch`, `ir_live`, `subsystem3_test_stage`, `ramp_run`, `bench_profile`, `subsystem5_test_stage`). | Each task loop is one state machine whose state lives across the loop; the deep `if` nesting (8.2b) was already moved out into functions, and splitting further would scatter that state through tuned, hardware-tested control code close to the deadline. The test scripts are linear sequences of prompts and checks. |
| D6 | 1.7c / 1.7d avoid `goto` and `continue` (preferred practice, not mandatory) | 16 `goto`s, all forward to one exit label in the same function (`publish` in `motion_task`, `done` in `imu_task`, `out` / `fail` in the bypass); 14 `continue`s, in loops that skip the rest of one pass (e.g. barcode decoding). | The rule allows a forward `goto` to a later label; one exit path for a failed manoeuvre is clearer than repeating it at every step. |

### Notes on how rules were read

- **1.6a casts**: every cast has a comment on its value range. A `(void)`
  cast is not counted: it converts no value, it only discards one (an
  unused parameter, a result not needed). The one `char *` -> `uint8_t *`
  cast for T-Monitor's text functions is in `app_str()` (`app.h`).
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
