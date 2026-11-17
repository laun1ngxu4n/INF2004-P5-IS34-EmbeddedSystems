"""Build the Barr C 7.1 rename list for app_program/ from the inventory.

    python tools/barr/names.py list > vars.tsv
    python tools/barr/oneoff/make_renames.py vars.tsv > renames.txt
    python tools/barr/names.py apply renames.txt

Prefixes (7.1j, k, m, n, o): g_ file-scope, p_ / pp_ pointer, b_ Boolean,
h_ kernel object handle (ID), in the order [g][p|pp][b|h].  Names shorter
than 3 characters (7.1e) get a name from BASE (by name, type) or SPECIAL
(by file, function, name).
"""
import os
import sys

# (name, type word or None) -> new base name
BASE = {
    ('i', None): 'idx', ('j', None): 'jdx', ('k', None): 'kdx',
    ('s', 'UINT'): 'irq_state',
    ('er', None): 'err',
    ('st', None): 'status',
    ('id', None): 'cmd',
    ('t', 'INT'): 'tick', ('t', 'APP_TIMING'): 'slot', ('t', 'BARCODE_RESULT'): 'trial',
    ('t', 'W'): 'tmp',
    ('t0', None): 't_start',
    ('r', 'BARCODE_RESULT'): 'result', ('r', 'OBSTACLE_PROFILE'): 'profile',
    ('l0', None): 'left_start', ('r0', None): 'right_start',
    ('ch', None): 'letter', ('mv', None): 'move',
    ('mm', None): 'dist_mm',
    ('dn', None): 'dbg_count', ('dw', None): 'dbg_widths',
    ('ns', 'W'): 'narrow_sum', ('ws', 'W'): 'wide_sum',
    ('ok', 'BOOL'): 'ok',
    ('on', 'BOOL'): 'on',
    ('v', None): 'value',
    ('n', None): 'count',
    ('e', None): 'elem',
    ('p', 'OBSTACLE_PROFILE'): 'profile',
    ('a', 'OBSTACLE_PROFILE'): 'scan_near', ('b', 'OBSTACLE_PROFILE'): 'scan_far',
    ('d', 'INT'): 'dist',
    ('q', None): 'seq',
    ('m', 'INT'): 'move',
    ('lo', None): 'low', ('hi', None): 'high',
    ('bl', None): 'black_lvl', ('wl', None): 'white_lvl',
    ('h', 'INT'): 'height',
    ('x', 'p:float'): 'value',
}

# (file, function, name) -> new base name
SPECIAL = {
    ('app_math.h', 'app_sqrtf', 'r'): 'root',
    ('app_math.h', 'app_sqrtf', 's'): 'scaled',
    ('app_math.h', 'app_sqrtf', 'i'): 'iter',
    ('app_math.h', 'app_sin_deg', 'x'): 'rad',
    ('app_math.h', 'app_sin_deg', 'x2'): 'rad_sq',
    ('app_gpio.c', 'gpio_inthdr', 'sh'): 'slot',
    ('app_gpio.c', 'gpio_inthdr', 'ev'): 'events',
    ('app_gpio.c', 'app_gpio_irq_add', 'cb'): 'callback',
    ('app_stage_test.c', 'test_print_x10', 'a'): 'magnitude',
    ('app_timing.c', 'app_timing_report', 'c'): 'snap',
    ('app_timing.c', 'app_timing_wake', 'iv'): 'interval',
    ('app_timing.c', 'print_ms', 'us'): 'micros',
    ('task_telemetry.c', 'log_take', 'h'): 'head_len',
    ('task_telemetry.c', 'log_take', 'n'): 'count',
    ('task_telemetry.c', 'app_mqtt_tx', 'm'): 'msg',
    ('task_telemetry.c', 'app_mqtt_tx', 'n'): 'len',
    ('task_telemetry.c', 'same', 'a'): 'first',
    ('task_telemetry.c', 'same', 'b'): 'second',
    ('task_telemetry.c', 'starts_with', 's'): 'text',
    ('task_telemetry.c', 'status_line', 'bc'): 'last_code',
    ('motion_hw.c', 'enc_edge', 'ab'): 'state_ab',
    ('motion_hw.c', 'enc_ab', 'in'): 'pins',
    ('motion_hw.c', 'encoder_read', 'l'): 'left',
    ('motion_hw.c', 'encoder_read', 'r'): 'right',
    ('task_motion.c', 'motion_task', 'h'): 'hist_pos',
    ('task_motion.c', 'motion_task', 'cl'): 'count_left',
    ('task_motion.c', 'motion_task', 'cr'): 'count_right',
    ('task_motion.c', 'motion_task', 'sl'): 'done_left',
    ('task_motion.c', 'motion_task', 'sr'): 'done_right',
    ('task_motion.c', 'motion_task', 'vl'): 'vel_left',
    ('task_motion.c', 'motion_task', 'vr'): 'vel_right',
    ('task_motion.c', 'wheel_duty', 'ff'): 'feedfwd',
    ('task_motion.c', 'wheel_duty', 'w'): 'wheel',
    ('task_motion.c', 'cmd_post', 'id'): 'cmd',
    ('subsystem2_test.c', 'test_directions', 'dl'): 'delta_left',
    ('subsystem2_test.c', 'test_directions', 'dr'): 'delta_right',
    ('subsystem2_test.c', 'res_name', 'r'): 'res',
    ('subsystem2_test.c', 'test_stop_once', 'k'): 'run',
    ('task_barcode.c', 'adc_sample', 'b'): 'black',
    ('task_barcode.c', 'adc_sample', 'w'): 'ring_pos',
    ('task_barcode.c', 'adc_sample', 'x'): 'blackness',
    ('task_barcode.c', 'adc_sample', 'v'): 'sample',
    ('task_barcode.c', 'bar_edge', 'w'): 'ring_pos',
    ('task_barcode.c', 'barcode_adc', 'w'): 'white',
    ('task_barcode.c', 'barcode_adc', 'b'): 'black',
    ('task_barcode.c', 'barcode_in_bars', 'd'): 'since',
    ('task_barcode.c', 'barcode_task', 'd'): 'delta',
    ('task_barcode.c', 'barcode_task', 'ed'): 'edge',
    ('task_barcode.c', 'barcode_task', 'n'): 'count',
    ('task_barcode.c', 'barcode_task', 'r'): 'result',
    ('task_barcode.c', 'barcode_set_mapping', 'a'): 'first',
    ('task_barcode.c', 'barcode_set_mapping', 'b'): 'second',
    ('task_barcode.c', 'barcode_set_mapping', 'c'): 'letter',
    ('task_barcode.c', 'barcode_set_mapping', 'm'): 'move_ch',
    ('task_barcode.c', 'barcode_set_mapping', 'ch'): 'letters',
    ('task_barcode.c', 'barcode_set_mapping', 'mv'): 'moves',
    ('task_barcode.c', 'barcode_mapping_text', 'ch'): 'letters',
    ('task_barcode.c', 'barcode_mapping_text', 'mv'): 'moves',
    ('task_barcode.c', 'barcode_mark', 'q'): 'seq',
    ('task_barcode.c', 'barcode_move_for', 'mv'): 'move',
    ('task_barcode.c', 'is_code_char', 'c'): 'letter',
    ('task_barcode.c', 'clear_split', 's'): 'sorted',
    ('task_barcode.c', 'nar_wid', 's'): 'sorted',
    ('task_barcode.c', 'sum9', 's'): 'sum',
    ('task_barcode.c', 'sort_w', 's'): 'values',
    ('task_barcode.c', 'decode_one_star', 'a'): 'first_ch',
    ('task_barcode.c', 'decode_one_star', 'b'): 'last_ch',
    ('task_barcode.c', 'decode_one_star', 'm'): 'sorted',
    ('task_barcode.c', 'decode_one_star', 'sm'): 'sum_mid',
    ('task_barcode.c', 'decode_one_star', 'so'): 'sum_other',
    ('task_barcode.c', 'decode_merged', 'a'): 'first_opt',
    ('task_barcode.c', 'decode_merged', 'c'): 'last_opt',
    ('task_barcode.c', 'decode_merged', 'x'): 'trial_elems',
    ('task_barcode.c', 'decode_truncated', 'x'): 'trial_elems',
    ('task_barcode.c', 'mark_pub', 'w'): 'width',
    ('task_line.c', 'line_task', 'l'): 'raw_left',
    ('task_line.c', 'line_task', 'r'): 'raw_right',
    ('task_line.c', 'line_task', 'fl'): 'left',
    ('task_line.c', 'line_task', 'fr'): 'right',
    ('task_line.c', 'line_task', 's'): 'demand',
    ('task_line.c', 'line_task', 'm'): 'mem_part',
    ('task_line.c', 'line_task', 'a'): 'share',
    ('task_line.c', 'line_task', 'cc'): 'centre',
    ('task_line.c', 'line_task', 'dd'): 'gap',
    ('task_line.c', 'line_task', 'ds'): 'step',
    ('task_line.c', 'line_task', 'mc'): 'mark_centre',
    ('task_line.c', 'line_task', 'mw'): 'mark_width',
    ('task_line.c', 'junction_at', 'c'): 'centre',
    ('task_line.c', 'junction_at', 'd'): 'delta',
    ('task_line.c', 'quiet_over', 'a'): 'from_odo',
    ('task_line.c', 'quiet_over', 'b'): 'to_odo',
    ('task_line.c', 'quiet_over', 'x'): 'corr_from',
    ('task_line.c', 'quiet_over', 'y'): 'corr_to',
    ('subsystem3_test.c', 'follow_and_watch', 'e0'): 'edges_start',
    ('subsystem3_test.c', 'follow_and_watch', 'j0'): 'junc_start',
    ('subsystem3_test.c', 'follow_and_watch', 'jn'): 'junc_now',
    ('subsystem3_test.c', 'follow_and_watch', 'lc'): 'pattern',
    ('subsystem3_test.c', 'follow_and_watch', 'mo'): 'mark_odo',
    ('subsystem3_test.c', 'follow_and_watch', 'mw'): 'mark_width',
    ('subsystem3_test.c', 'follow_and_watch', 'nc'): 'corrections',
    ('subsystem3_test.c', 'follow_and_watch', 'ns'): 'sweeps',
    ('subsystem3_test.c', 'follow_and_watch', 'sd'): 'sides',
    ('subsystem3_test.c', 'ir_live', 'e1'): 'edges_prev',
    ('subsystem3_test.c', 'ir_live', 'e2'): 'edges_now',
    ('subsystem3_test.c', 'bar_bench_run', 'w'): 'width',
    ('subsystem3_test.c', 'bar_bench_run', 'x'): 'blackness',
    ('subsystem3_test.c', 'subsystem3_stage_test', 'j'): 'part_junc',
    ('subsystem3_test.c', 'subsystem3_stage_test', 'l'): 'part_lost',
    ('subsystem3_test.c', 'subsystem3_stage_test', 'o'): 'part_ok',
    ('subsystem3_test.c', 'subsystem3_stage_test', 'f'): 'part_fail',
    ('subsystem3_test.c', 'subsystem3_stage_test', 'ok'): 'codes_ok',
    ('task_imu.c', 'imu_task', 'ax'): 'acc_x',
    ('task_imu.c', 'imu_task', 'ay'): 'acc_y',
    ('task_imu.c', 'imu_task', 'az'): 'acc_z',
    ('task_imu.c', 'imu_task', 'b'): 'raw',
    ('task_imu.c', 'imu_task', 'c0'): 'cos_level',
    ('task_imu.c', 'imu_task', 'cp'): 'cos_pitch',
    ('task_imu.c', 'imu_task', 'ds'): 'step_mm',
    ('task_imu.c', 'imu_task', 'h'): 'height',
    ('task_imu.c', 'imu_task', 'hp'): 'height_pitch',
    ('task_imu.c', 'imu_task', 'hr'): 'height_roll',
    ('task_imu.c', 'imu_task', 'm0'): 'mag_level',
    ('task_imu.c', 'imu_task', 'r'): 'roll_abs',
    ('task_imu.c', 'imu_task', 's0'): 'sin_level',
    ('task_imu.c', 'imu_task', 'sp'): 'sin_pitch',
    ('task_imu.c', 'imu_task', 'y0'): 'side_level',
    ('task_imu.c', '<file>', 'ok'): 'chip_ok',
    ('subsystem4_test.c', 'ramp_run', 'jb'): 'jolts_bump',
    ('subsystem4_test.c', 'ramp_run', 'j0'): 'jolts_start',
    ('subsystem4_test.c', 'ramp_run', 'n0'): 'reports_start',
    ('subsystem4_test.c', 'ramp_run', 'nb'): 'bumps',
    ('subsystem4_test.c', 'ramp_run', 'ra'): 'roll_abs',
    ('subsystem4_test.c', 'ramp_run', 'rl'): 'roll',
    ('subsystem4_test.c', 'ramp_run', 'sn'): 'sin_pitch',
    ('subsystem4_test.c', 'subsystem4_stage_test', 'n0'): 'reports_start',
    ('subsystem5_test.c', 'subsystem5_test', 'p0'): 'pings_start',
    ('task_mission.c', 'mission_state_name', 'n'): 'names',
    ('task_mission.c', 'mission_task', 'go'): 'go',
    ('task_mission.c', 'set_state', 's'): 'new_state',
    ('task_mission.c', 'on_barcode', 'm'): 'move',
    ('task_mission.c', 'on_junction', 'm'): 'move',
    ('task_mission.c', 'on_junction', 'j0'): 'junc_odo',
    ('task_mission.c', 'on_obstacle', 'yc'): 'centre',
    ('task_mission.c', 'pass_box', 'r'): 'outcome',
    ('task_mission.c', 'pass_box', 'd'): 'sonar_mm',
    ('task_mission.c', 'turn_onto_line', 'a'): 'angle',
    ('task_mission.c', 'spin_line', 'dd'): 'wheel_diff',
    ('task_mission.c', 'mv', 'id'): 'cmd',
    ('task_obstacle.c', 'servo_write', 'us'): 'pulse_us',
    ('task_obstacle.c', 'side_edge', 'a'): 'angle',
    ('task_obstacle.c', 'side_edge', 'd'): 'dist',
    ('task_obstacle.c', 'side_edge', 'in'): 'on_box',
    ('task_obstacle.c', 'side_edge', 'r'): 'range',
    ('task_obstacle.c', 'obstacle_scan', 'nv'): 'valid',
    ('task_obstacle.c', 'obstacle_scan', 'ul'): 'unc_left',
    ('task_obstacle.c', 'obstacle_scan', 'ur'): 'unc_right',
    ('task_obstacle.c', 'median5', 'v'): 'values',
    ('task_obstacle.c', 'pick_edge', 'aa'): 'abs_a',
    ('task_obstacle.c', 'pick_edge', 'ab'): 'abs_b',
    ('task_obstacle.c', 'pick_edge', 'ea'): 'edge_a',
    ('task_obstacle.c', 'pick_edge', 'eb'): 'edge_b',
    ('task_obstacle.c', 'pick_edge', 'ua'): 'unc_a',
    ('task_obstacle.c', 'pick_edge', 'ub'): 'unc_b',
    ('task_obstacle.c', 'pick_edge', 'sa'): 'seen_a',
    ('task_obstacle.c', 'pick_edge', 'sb'): 'seen_b',
    ('task_obstacle.c', 'pick_edge', 'ca'): 'clear_a',
    ('task_obstacle.c', 'pick_edge', 'cb'): 'clear_b',
    ('task_obstacle.c', 'pick_edge', 'e'): 'edge',
    ('task_obstacle.c', 'pick_edge', 'u'): 'unc',
    ('task_obstacle.c', 'obstacle_combine', 'a'): 'near',
    ('task_obstacle.c', 'obstacle_combine', 'b'): 'far',
    ('task_obstacle.c', 'obstacle_scan', 'p'): 'profile',
}

FUNC_POINTER_TYPES = {'APP_GPIO_CB', 'FP'}


def main():
    rows = [l.rstrip('\n').split('\t') for l in open(sys.argv[1], encoding='utf-8')]
    seen_global = set()
    out = []
    for f, scope, name, spec, stars, arr in rows:
        base_f = os.path.basename(f.replace('\\', '/'))
        words = spec.replace('param ', '').replace('*', ' ').split()
        tword = next((w for w in reversed(words) if w not in ('const', 'volatile', 'static', 'LOCAL', 'EXPORT', 'IMPORT', 'unsigned', 'signed')), None)
        if 'unsigned' in words and tword in (None, 'int'):
            tword = 'unsigned'
        nstars = len(stars)
        is_arr = arr == '[]'
        is_ptr = (nstars > 0 and not is_arr) or (tword in FUNC_POINTER_TYPES and not is_arr)
        is_bool = tword in ('BOOL', 'bool') and nstars == 0 or (tword in ('BOOL', 'bool') and nstars > 0)
        is_handle = tword == 'ID' and nstars == 0
        is_global = scope == '<file>'
        exported = any(w in ('EXPORT', 'IMPORT') for w in words)

        base = name
        key3 = (base_f, scope, name)
        if key3 in SPECIAL:
            base = SPECIAL[key3]
        elif len(name) < 3:
            pkey = 'p:' + tword if spec.startswith('param ') else tword
            base = BASE.get((name, pkey)) or BASE.get((name, tword)) or BASE.get((name, None))
            if base is None:
                print(f'# NO NAME: {f} {scope} {name} ({spec}{stars}{arr})', file=sys.stderr)
                continue
        prefix = ('g' if is_global else '') + \
                 ('pp' if (nstars >= 2 and not is_arr) else 'p' if is_ptr else '') + \
                 ('b' if is_bool else 'h' if is_handle else '')
        new = (prefix + '_' + base) if prefix else base
        if new == name:
            continue
        if is_global and exported:
            if name not in seen_global:
                out.append(f'global {name} {new}')
                seen_global.add(name)
        elif is_global:
            out.append(f'file {f} {name} {new}')
        else:
            out.append(f'local {f} {scope} {name} {new}')
    # two names of one function (or file) must not end up the same, nor
    # take the name of another variable already there
    from collections import defaultdict
    targets = defaultdict(dict)
    existing = defaultdict(set)
    for f, scope, name, spec, stars, arr in rows:
        existing[(f, scope)].add(name)
    for line in out:            # a name being renamed away is free
        w = line.split()
        if w[0] == 'local':
            existing[(w[1], w[2])].discard(w[3])
        elif w[0] == 'file':
            existing[(w[1], '<file>')].discard(w[2])
    bad = 0
    for line in out:
        w = line.split()
        if w[0] == 'local':
            key, old, new = (w[1], w[2]), w[3], w[4]
        elif w[0] == 'file':
            key, old, new = (w[1], '<file>'), w[2], w[3]
        else:
            continue
        if new in targets[key] and targets[key][new] != old:
            print(f'# COLLISION {key}: {targets[key][new]} and {old} -> {new}', file=sys.stderr)
            bad += 1
        targets[key][new] = old
        if new in existing[key]:
            print(f'# TAKEN {key}: {old} -> {new} (already a variable)', file=sys.stderr)
            bad += 1
    seen = set()
    for line in out:
        if line not in seen:
            print(line)
            seen.add(line)


if __name__ == '__main__':
    main()
