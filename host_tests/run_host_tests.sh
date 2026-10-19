#!/bin/sh
# Host-side logic tests (no Pico needed): gcc on Linux/macOS/WSL.
# They compile the real source files against stub SDK headers in stubs/.
set -e
cd "$(dirname "$0")"
CF="-std=c11 -Wall -Wextra -Wno-unused-parameter -Wno-unused-function -Istubs"
OUT=${TMPDIR:-/tmp}
gcc $CF -o "$OUT/t_json" test_cmd_json.c ../tests/wifi_mqtt/cmd_json.c
gcc $CF -o "$OUT/t_cmdq" test_wifi_cmdq.c ../tests/wifi_mqtt/cmd_json.c
gcc $CF -o "$OUT/t_irgap" test_ir_gap.c
echo "== cmd_json ==";  "$OUT/t_json"
echo "== wifi command queue =="; "$OUT/t_cmdq" | grep -v '^\[cmd\] '
echo "== IR overflow marker (last lines; no width may span 25,600 -> 25,800 us) =="
"$OUT/t_irgap" | tail -4
"$OUT/t_irgap" | grep -q "25800 us  BARCODE 1->0  previous level lasted        0 us" \
    && echo "IR gap: PASS" || { echo "IR gap: FAIL"; exit 1; }
