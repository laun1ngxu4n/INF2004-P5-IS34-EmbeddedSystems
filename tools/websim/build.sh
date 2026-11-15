#!/bin/sh
# Build the robot firmware (app_program/, unchanged) into web/firmware.wasm.
#
#   sh tools/websim/build.sh
#
# Needs: clang with the wasm32 target, wasm-ld, node + npm (for binaryen's
# wasm-opt, installed locally into tools/websim/.tools on first run).
# The firmware sources are NOT modified.  Two things are done at build time:
#   1. app_timing.c is copied to the build folder with ONE line substituted
#      (the direct read of the RP2040 timer address becomes a call into the
#      simulator).  Nothing else is changed.
#   2. the kernel and the chip registers are supplied by shim/ (kernel.c,
#      libc.c, include/) instead of micro T-Kernel and the Pico SDK.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
APP="$REPO/app_program"
OUT="$HERE/build"
WEB="$HERE/web"
mkdir -p "$OUT/src" "$OUT/obj" "$WEB"

CLANG=${CLANG:-clang}
CFLAGS="--target=wasm32 -O2 -g0 -std=gnu11 -ffreestanding -fno-builtin -fno-stack-protector \
 -Wno-everything -DAPP_TEST=0 -DAPP_TEST_STAGE=0 -DTM_CONSOLE_UART=0 -DBOARD_UART0_FREE=1 \
 -I$HERE/shim/include -I$REPO/include -I$APP -I$REPO/config"

# 1. the one substituted line (timer register -> simulator)
sed 's|^#define RP2040_TIMERAWL .*|#define RP2040_TIMERAWL hw_read(0x40054028u)|' \
    "$APP/app_timing.c" > "$OUT/src/app_timing.c"
grep -q 'hw_read(0x40054028u)' "$OUT/src/app_timing.c" || { echo "timer substitution failed"; exit 1; }
# include path so "app.h" etc. resolve from the copied file
SRC="$APP/app_main.c $APP/app_gpio.c $APP/app_stop.c $OUT/src/app_timing.c"
for d in subsystem1 subsystem2 subsystem3 subsystem4 subsystem5; do
  for f in "$APP/$d"/*.c; do
    case "$f" in *_test.c) ;; *) SRC="$SRC $f" ;; esac
  done
done
SRC="$SRC $HERE/shim/kernel.c $HERE/shim/libc.c"

rm -f "$OUT"/obj/*.o
OBJS=""
for f in $SRC; do
  o="$OUT/obj/$(basename "$f" .c).o"
  $CLANG $CFLAGS -I"$(dirname "$f")" -c "$f" -o "$o"
  OBJS="$OBJS $o"
done

# 2. link, one export per hook the simulator calls
EXPORTS="--export=usermain --export=app_mqtt_rx --export=app_mqtt_tx"
wasm-ld $OBJS -o "$OUT/fw_raw.wasm" --no-entry --allow-undefined \
  -z stack-size=32768 --initial-memory=2097152 --max-memory=8388608 \
  --export=__stack_pointer --export=__heap_base $EXPORTS --export-memory --gc-sections

# 3. Asyncify: lets blocking kernel calls (tk_dly_tsk, tk_slp_tsk, ...) suspend a task
TOOLS="$HERE/.tools"
if [ ! -f "$TOOLS/node_modules/binaryen/bin/wasm-opt" ]; then
  mkdir -p "$TOOLS"
  npm install --prefix "$TOOLS" --no-save --no-audit --no-fund binaryen
fi
node "$TOOLS/node_modules/binaryen/bin/wasm-opt" "$OUT/fw_raw.wasm" -O2 \
  --asyncify --pass-arg=asyncify-imports@env.sim_yield \
  -o "$WEB/firmware.wasm"
ls -l "$WEB/firmware.wasm"
