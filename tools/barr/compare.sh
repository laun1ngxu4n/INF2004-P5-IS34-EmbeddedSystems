#!/usr/bin/bash
# Build every firmware image in "compare" mode and check them byte for byte
# against a saved set, to prove a change (formatting, renaming, Barr C
# clean-up) did not change the program.
#
#   bash tools/barr/compare.sh save  <dir>   # before the change
#   bash tools/barr/compare.sh check <dir>   # after it: same bytes?
#
# Compare mode (EXTRA_CFLAGS=-DAPP_COMPARE_BUILD=1) only leaves the file
# name and line number out of failed-test messages (app_program/
# test_report.h), which move when code is reformatted.  Everything else is
# the normal build.  BUILD is the command that runs make in build_make with
# the toolchain on the PATH (default: the local .vscode/build.sh).
set -e
mode="$1"
dir="$2"
if [ -z "$mode" ] || [ -z "$dir" ]; then
    echo "usage: $0 save|check <dir>"
    exit 2
fi
cd "$(dirname "$0")/../.."
BUILD="${BUILD:-bash .vscode/build.sh}"
mkdir -p "$dir"

variants=(
    "TEST=1 TEST_STAGE=1|mtk3pico_smp0_usb_cdc_wifi_test_s1"
    "TEST=1 TEST_STAGE=2|mtk3pico_smp0_usb_cdc_wifi_test_s2"
    "TEST=1 TEST_STAGE=3|mtk3pico_smp0_usb_cdc_wifi_test_s3"
    "TEST=1 TEST_STAGE=4|mtk3pico_smp0_usb_cdc_wifi_test_s4"
    "TEST=1 TEST_STAGE=6|mtk3pico_smp0_usb_cdc_wifi_test_s6"
    "TEST=1 TEST_STAGE=1 NET=lwip MQTT=1|mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt_test_s1"
    "TEST=1 TEST_STAGE=2 NET=lwip MQTT=1|mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt_test_s2"
    "TEST=1 TEST_STAGE=3 NET=lwip MQTT=1|mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt_test_s3"
    "TEST=1 TEST_STAGE=4 NET=lwip MQTT=1|mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt_test_s4"
    "TEST=1 TEST_STAGE=6 NET=lwip MQTT=1|mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt_test_s6"
    "|mtk3pico_smp0_usb_cdc_wifi"
    "NET=lwip MQTT=1|mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt"
)

bad=0
for v in "${variants[@]}"; do
    args="${v%%|*}"
    name="${v##*|}"
    out=$($BUILD -j8 $args EXTRA_CFLAGS=-DAPP_COMPARE_BUILD=1 2>&1) || {
        echo "$out" | tail -20
        echo "BUILD FAILED: $args"
        exit 1
    }
    if echo "$out" | grep -q "warning:"; then
        echo "$out" | grep "warning:" | head -5
        echo "WARNINGS: $args"
        bad=1
    fi
    if [ "$mode" = save ]; then
        cp "build_make/$name.uf2" "$dir/$name.uf2"
        echo "saved  $name"
    elif cmp -s "build_make/$name.uf2" "$dir/$name.uf2"; then
        echo "same   $name"
    else
        echo "DIFFER $name"
        cp "build_make/$name.uf2" "$dir/$name.new.uf2"
        bad=1
    fi
done
exit $bad
