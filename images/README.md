# Prebuilt images (1 Nov, commit 3ddc0a8 code)

Built from this repo with Pico SDK 2.2.0, 0 warnings. **Not run on the
robot yet.** Flash: hold BOOTSEL, plug in USB, drag the file onto RPI-RP2.
Steps for each: `../TESTING_GUIDE.md`.

| File | Same as `make ...` in `build_make` | What it does |
|---|---|---|
| `is34_test_stage1.uf2` | `TEST=1 TEST_STAGE=1` | Stage 0 auto tests, then motion: turns, slow speed, STOP |
| `is34_test_stage2.uf2` | `TEST=1 TEST_STAGE=2` | then IR view, line + junctions, barcodes, course with the TEST mapping |
| `is34_test_stage3.uf2` | `TEST=1 TEST_STAGE=3` | then servo check, sonar view, 15 width scans, course (bypass) |
| `is34_test_stage4.uf2` | `TEST=1 TEST_STAGE=4` | then IMU check, tilt view, 10 ramp runs |
| `is34_test_stage6.uf2` | `TEST=1 TEST_STAGE=6` | then the whole course with the TEST mapping A=L, B=R, C=F, D=U |
| `is34_normal_no_wifi.uf2` | (plain `make`) | the mission image without MQTT: START button runs it, but with no mapping it stops at the first junction after a barcode |

Not here: the **demo image with MQTT** (`make NET=lwip MQTT=1`). It needs
your WiFi name, password and broker IP compiled in, so each person builds
it with their own `config/wifi_secrets.h` (TESTING_GUIDE.md section 6).

After changing any code, rebuild and replace these files, or they no
longer match the code.
