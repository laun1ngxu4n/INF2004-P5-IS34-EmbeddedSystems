#!/bin/sh
# Host-side logic tests (no Pico needed): gcc on Linux/macOS/WSL.
# They compile the real source files (or the shared logic headers in
# tests/common) against stub SDK headers in stubs/.
#
# v3.2 (25 Oct, review finding 15): every test runs once with its output
# saved to a log, and its exit status is checked BEFORE anything is filtered
# for display, so a failing test can no longer be hidden by a pipe. The
# runner exits non-zero if any build or test fails.
cd "$(dirname "$0")" || exit 2
CF="-std=c11 -Wall -Wextra -Wno-unused-parameter -Wno-unused-function -Istubs"
OUT=${TMPDIR:-/tmp}/p5is34_host_tests
mkdir -p "$OUT" || exit 2
FAILS=0

build() {
    name=$1
    shift
    if ! gcc $CF -o "$OUT/$name" "$@" -lm; then
        echo "BUILD FAILED: $name"
        FAILS=$((FAILS + 1))
        return 1
    fi
    return 0
}

# run NAME: runs the test once into $OUT/NAME.log; returns its exit status.
run() {
    "$OUT/$1" > "$OUT/$1.log" 2>&1
    st=$?
    if [ "$st" -ne 0 ]; then
        echo "TEST FAILED: $1 (exit $st) - full output in $OUT/$1.log"
        FAILS=$((FAILS + 1))
    fi
    return "$st"
}

echo "== cmd_json =="
build t_json test_cmd_json.c ../tests/wifi_mqtt/cmd_json.c && run t_json; cat "$OUT/t_json.log" 2>/dev/null

echo "== wifi command queue =="
build t_cmdq test_wifi_cmdq.c ../tests/wifi_mqtt/cmd_json.c && run t_cmdq; grep -v '^\[cmd\] ' "$OUT/t_cmdq.log" 2>/dev/null

echo "== IR overflow marker (no width may span 25,600 -> 25,800 us) =="
if build t_irgap test_ir_gap.c && run t_irgap; then
    tail -4 "$OUT/t_irgap.log"
    # Every printed width must be the 100 us edge spacing, except the single
    # 0 us restart marker after the loss; a 200 us width would span the gap.
    if grep -q "EDGES LOST before this point" "$OUT/t_irgap.log" \
        && grep -q "25800 us  BARCODE 1->0  previous level lasted        0 us" "$OUT/t_irgap.log" \
        && ! grep "previous level lasted" "$OUT/t_irgap.log" | grep -v -E "lasted +(100|0) us" | grep -q .; then
        echo "IR gap: PASS"
    else
        echo "IR gap: FAIL (marker missing or a width spans the gap)"
        FAILS=$((FAILS + 1))
    fi
fi

echo "== motor STOP sequence (parts_check, finding 03) =="
build t_motor test_motor_seq.c && run t_motor; cat "$OUT/t_motor.log" 2>/dev/null

echo "== encoder timing (encoder_test, findings 04 and 14) =="
build t_enc test_enc_timing.c && run t_enc; cat "$OUT/t_enc.log" 2>/dev/null

echo "== sonar classifier, IMU calibration, chip ID (findings 13, 16, 20) =="
build t_misc test_sonar_imu_chip.c && run t_misc; cat "$OUT/t_misc.log" 2>/dev/null

if [ "$FAILS" -ne 0 ]; then
    echo "HOST TESTS: $FAILS FAILURE(S)"
    exit 1
fi
echo "HOST TESTS: ALL PASSED"
exit 0
