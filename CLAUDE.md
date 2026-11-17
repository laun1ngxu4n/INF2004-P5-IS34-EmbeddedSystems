# P5-IS34 robot car: instructions for Claude

Line-following robot on a Pico W (Cytron Robo Pico), micro T-Kernel 3.0,
code in `app_program/`. Read before doing anything:

1. [`docs/team/CONTEXT.md`](docs/team/CONTEXT.md): the handover. What exists,
   what is proven, **section 3 = current status and the next steps in order**.
2. [`testing/TEST_STATUS.md`](testing/TEST_STATUS.md): what passed on the robot.
3. Change Requirement 1 (implemented 8 Nov, robot retest pending):
   [`docs/team/CHANGE_REQ1.md`](docs/team/CHANGE_REQ1.md) (what was done, the
   Week 13 evidence, the retest list in section 7.2), the binary MQTT format
   [`docs/team/MQTT_WIRE_FORMAT.md`](docs/team/MQTT_WIRE_FORMAT.md), and the course
   documents in [`docs/team/course/`](docs/team/course/).
   IMU requirement change (lecturer, 13 Nov: collision detection removed,
   lifting the robot must stop the motors; built 13 Nov):
   [`docs/team/course/IMU_REQUIREMENT_CHANGE.md`](docs/team/course/IMU_REQUIREMENT_CHANGE.md).
4. How to run the tests: [`testing/TESTING_GUIDE.md`](testing/TESTING_GUIDE.md).
   Coding standard: [`docs/team/BARR_C_DEVIATIONS.md`](docs/team/BARR_C_DEVIATIONS.md).

## Rules that must not be broken

- **Never commit** `config/wifi_secrets.h` or anything in `images/wifi/`
  (the WiFi images contain the WiFi password in plain text). Both are
  git-ignored; do not `git add -f` them.
- Do not commit `.vscode/`, `desktop.ini`, the Barr standard PDF, or the
  course `.docx` files (they hold names and student IDs). They live in
  `private/` (git-ignored, with the Tribunal 2 template and the filled
  submission); `.vscode/` and `desktop.ini` are git-ignored too. What matters
  in them is in `docs/team/course/`: the change requirement word for
  word, and a summary of the tribunal decisions with buddies by role
  (B1-B5) only.
- Never tell the user to power the robot from USB and the power bank at
  the same time.
- Never build with `CRASH_REPORT=1` while the IR sensors are on GP0/GP1.
- The analogue barcode module is powered from 3V3.
- Never delete `build_make/mtkernel_3/app_program/`: its `subdir.mk` is
  tracked, and without it the build "succeeds" with none of our code in it.
- Never invent test results. A result comes from a robot log.

## Building (Windows)

- The build needs MSYS2 bash (Git Bash has no `make`):
  `C:\msys64\usr\bin\bash.exe .vscode/build.sh -j8 TEST=1 TEST_STAGE=2`
  (`.vscode/build.sh` is local, not committed: it sets the Pico toolchain
  path; see README for the toolchain).
- Images: `images/is34_test_stageN.uf2`, `images/is34_normal_no_wifi.uf2`
  (committed, `git add -f`); WiFi images in `images/wifi/` (never committed).
  Rebuild all after a code change.
- After any C change: `python tools/barr/check.py` must stay at 0 except
  the documented deviations; format with `app_program/.clang-format` and
  `python tools/barr/fix_layout.py`. A change meant not to alter the
  program is proven with `tools/barr/compare.sh` (byte-identical images).

## How the owner wants to work

- A question gets an answer and a suggestion, not a code change. Change
  code only when asked (a failure log counts as a request to fix).
- Commit and push to `main` after each finished change, and fast-forward
  `rtos-stage0` to it.
- Explain in plain words: what changed, why, what needs retesting on the robot.
