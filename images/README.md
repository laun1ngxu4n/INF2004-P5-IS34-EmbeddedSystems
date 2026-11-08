# Prebuilt images (rebuilt 7 Nov: straddle line following)

Built from this repo with Pico SDK 2.3.1 (GCC 15.2), 0 warnings, S2_TRACK_MM 123.
Status of each stage: `../testing/TEST_STATUS.md`. The 7 Nov line following
(sensors straddling the tape, crossings, left branches from the barcode
sensor) is **not run on the robot yet**; it expects the two line sensors
about 30 mm apart with the tape centred between them, and B 40 mm behind
them and 70 mm left (`../app_program/subsystem3/subsystem3.h`).
All images read the barcode sensor on its **analogue output (AO to GP28,
module on 3V3)** since 7 Nov; for the digital D0 on GP2 build with
`S3_BARCODE_ANALOG 0`.
Flash: hold BOOTSEL, plug in USB, drag the file onto RPI-RP2.
Steps for each: `../testing/TESTING_GUIDE.md`.

| File | Same as `make ...` in `build_make` | What it does |
|---|---|---|
| `is34_test_stage1.uf2` | `TEST=1 TEST_STAGE=1` | Stage 0 auto tests, then motion: turns, slow speed, STOP |
| `is34_test_stage2.uf2` | `TEST=1 TEST_STAGE=2` | then IR view, line + junctions, one place at a time (S3-L03, L05..L10), straight barcode read (S3-B00), barcodes while following, course with the TEST mapping; each part can be skipped with STOP |
| `is34_test_stage3.uf2` | `TEST=1 TEST_STAGE=3` | then servo check, sonar view, 3 width scans, course (bypass) |
| `is34_test_stage4.uf2` | `TEST=1 TEST_STAGE=4` | then IMU check, tilt view, 5 ramp runs |
| `is34_test_stage6.uf2` | `TEST=1 TEST_STAGE=6` | then the whole course with the TEST mapping A=L, B=R, C=F, D=U |
| `is34_normal_no_wifi.uf2` | (plain `make`) | the mission image without MQTT: START button runs it, but with no mapping it stops at the first junction after a barcode |

Not here: the **demo image with MQTT** (`make NET=lwip MQTT=1`) and the
**test images over WiFi** (`make NET=lwip MQTT=1 TEST=1 TEST_STAGE=n`,
which send the whole log to the panel's Robot log tab). They need your
WiFi name, password and broker IP compiled in, so each person builds
them with their own `config/wifi_secrets.h` into `images/wifi/`, which
git ignores: never commit them (TESTING_GUIDE.md section 6).

## Bring-up part checks (`bringup/`, no RTOS)

`images/bringup/` holds the stand-alone part tests, rebuilt on 1 Nov from
the current `bringup/` source (v3.2) with Pico SDK 2.2.0, 0 warnings.
They do not use the RTOS. How to run each one and what it prints:
`../bringup/README.md`; past results: `../bringup/docs/bringup_results.md`.

| File | Checks |
|---|---|
| `parts_check_v3.2.uf2` | every part alive: IR, encoders, motors (wheels lifted), sonar, servo, IMU, buttons |
| `encoder_test_v3.2.uf2` | encoder counts, direction, speed per duty |
| `ir_test_v3.2.uf2` | IR line and barcode sensor edges |
| `ultrasonic_test_v3.2.uf2` | sonar accuracy, servo scans |
| `imu_test_v3.2.uf2` | IMU pitch, tilt height, shocks |
| `i2c_diag_v3.2.uf2` | finds the IMU on the I2C bus |
| `notify_test_v3.2.uf2` | NeoPixel and buzzer |
| `wifi_mqtt_test_prompt_v3.2.uf2` | WiFi + MQTT; asks for SSID, password and broker IP over USB |

Quick health check before a test session: flash `parts_check_v3.2.uf2`
(wheels lifted), then flash the stage image you want.

After changing any code, rebuild and replace these files, or they no
longer match the code.
