# websim: your robot firmware in a browser

A simulator that runs the real code in `app_program/` (compiled to WebAssembly,
not rewritten) against a simulated Pico W, robot and Cytron-style tile track.
Build a course from tiles, press START, and watch your line, barcode, junction,
obstacle and IMU code drive it. The firmware's own console output appears in the
page.

## Run it

    sh tools/websim/build.sh                  # compiles app_program/ -> web/firmware.wasm
    python3 -m http.server -d tools/websim/web 8000
    # open http://localhost:8000

Needs `clang` (with the wasm32 target), `wasm-ld` (package `lld`), Node and npm
(the build installs binaryen's `wasm-opt` locally). Linux, macOS or WSL.

To publish it: GitHub > Settings > Pages > Source = "GitHub Actions". The workflow
`.github/workflows/websim.yml` then rebuilds and redeploys the page on every push
that touches `app_program/` or `tools/websim/`. Edit your C, push, and the page
runs the new code.

Check the firmware without a browser (also run by the workflow):

    node tools/websim/test/check.mjs

## Tiles

`S` straight, `C` curve, `T` T-junction, `X` crossing, `B` barcode (a Code 39 strip
on the left of the line, 85 mm from it), `O` obstacle box on the line, `R` ramp,
`H` hump under the right wheel only, `.` empty. A digit after the letter is the
rotation in quarter turns clockwise (`S1` is horizontal). Optional settings follow
a colon: `B0:K` letter, `O0:150,100` width and depth, `R0:20,110` height and slope
length, `H0:30` height. Tiles are 300 mm (`TILE` in `web/world.js`).

## What is real and what is modelled

Real: all of `app_program/*.c` except the tests, unmodified. The one build-time
change is a single line in `app_timing.c`, where the direct read of the RP2040 timer
register becomes a call into the simulator (`build.sh` does it with `sed` on a copy).

Replaced by `shim/`:
- micro T-Kernel: only the calls the firmware uses (tasks, sleep/wake, delay, event
  flags, cyclic handlers, interrupt handlers). Tasks are priority scheduled and
  preempt on wake-up, but they run in zero simulated time, so CPU load, missed
  deadlines and interrupt latency are not simulated.
- RP2040 registers the firmware touches: GPIO (incl. edge interrupts), PWM, the
  timer, I2C1 with an LSM303D at 0x1D, and the ADC FIFO with its interrupt.
- WiFi/MQTT: the page injects START, STOP and the mapping through the firmware's
  `app_mqtt_rx()`, like the panel does.

Modelled (approximations in `web/sim.js` `ROBOT` and `web/world.js`; adjust to match
your robot): wheel speed vs duty from your 26 Oct measurements with a 45 ms motor lag
(a guess), IR line sensors reacting to a 14 mm window of the tape, barcode ink at 80 %
read through a 1.4 mm spot, sonar with a 15 degree cone and 1 mm noise that ignores
very slanted faces, accelerometer with noise and the Z offset, one wheel lifting the
chassis on a hump, 300 mm tiles. Sensor positions, track width and the other
geometry come from your headers (`S3_*`, `S2_*`, `S4_L_MM`, `S5_SONAR_TO_AXLE_MM`).

Nothing here is a robot result: it shows how your code behaves in a model, and where
the model is wrong it will be wrong with it. Compare against real logs before trusting
a number.
