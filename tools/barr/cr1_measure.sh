#!/usr/bin/bash
# Change Requirement 1 evidence: RAM and record sizes, APP_PACKED=1 (packed)
# against APP_PACKED=0 (every field in its own storage unit, as before).
# Same sources, features, buffers and compiler settings; only the switch
# differs.  Needs MSYS2 bash (make) and the local .vscode/build.sh.
#
#   bash tools/barr/cr1_measure.sh > cr1_measure.txt
cd "$(dirname "$0")/../.."
BIN=/c/Users/tyran/.pico-sdk/toolchain/15_2_Rel1/bin
TYPES="stop_word_t motor_duty_t motion_cmd_t motion_status_t line_out_t \
bar_edge_t barcode_result_t bar_pair_t bar_mark_t imu_out_t \
obstacle_profile_t mission_word_t mission_flags_t safety_state_t tele_msg_t"
VARS="g_word g_duty g_enc_prev g_cmd_slot g_status g_out g_ring g_last g_map \
g_mark g_adc_raw g_mw g_flags g_front_mm g_echo_width_us g_servo_now g_outbox"

for build in "mission (no WiFi)|" "mission WiFi/MQTT|NET=lwip MQTT=1"; do
    name=${build%%|*}
    args=${build#*|}
    for packed in 0 1; do
        bash .vscode/build.sh -j8 $args "EXTRA_CFLAGS=-DAPP_PACKED=$packed" \
            > /dev/null 2>&1
        elf=$(ls -t build_make/*.elf | head -1)
        echo "## $name, APP_PACKED=$packed ($elf)"
        "$BIN/arm-none-eabi-size" "$elf" | tail -1 |
            awk '{print "image: text " $1 ", data " $2 ", bss " $3}'
        "$BIN/arm-none-eabi-size" build_make/mtkernel_3/app_program/*.o \
            build_make/mtkernel_3/app_program/*/*.o |
            awk 'NR > 1 {d += $2; b += $3} END {print "our modules: data " d \
                 ", bss " b ", RAM " d + b}'
        for v in $VARS; do
            "$BIN/arm-none-eabi-nm" -S "$elf" | awk -v n="$v" \
                '$4 == n {printf "var %s %d\n", n, strtonum("0x" $2)}'
        done
        for t in $TYPES; do
            s=$("$BIN/arm-none-eabi-gdb" -batch -ex "print sizeof($t)" \
                "$elf" 2>/dev/null | sed -n 's/^\$1 = //p')
            [ -n "$s" ] && echo "type $t $s"
        done
    done
done
