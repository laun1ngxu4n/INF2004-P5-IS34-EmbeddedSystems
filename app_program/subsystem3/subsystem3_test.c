/**
 * @file    subsystem3_test.c
 *  On-target tests for subsystem 3.  Built only with: make TEST=1
 *
 *  Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *  console output to testing/subsystem3/logs/ and record the result
 *  in testing/subsystem3/TEST_LOG.md.  Test IDs: S3-U01, S3-U02, ...
 */

#include "subsystem3.h"
#include "../test_report.h"
#include "../app_test.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem5/subsystem5.h"
#include "subsystem3_test.h"

#if APP_TEST

#define TICKS_PER_S   100  /* 10 ms each */
#define IR_VIEW_TICKS 3000 /* 30 s at 10 ms */
#define BAR_RUNS      3    /* the prompts say 3 */

/** Subsystem 3's start-up tests (task timing). */
EXPORT void subsystem3_test (void)
{
    TEST_INIT();

    /* S3-U01 / S3-U02: Line Following wakes every 20 ms without overruns */
    TEST_PERIODIC("S3-U01", "S3-U02", g_tmg_line);

    /* S3-U03 / S3-U04: Barcode Decoder's 10 ms timeout keeps it running */
    TEST_TIMEOUT("S3-U03", "S3-U04", g_tmg_barcode);

    TEST_SUMMARY("subsystem3");
}

/* ---- Stage 2 hands-on tests ---------------------------------------- */

/** Live view of the IR sensors for secs seconds, with the barcode sensor
 * levels. */
LOCAL void ir_live (void)
{
    int32_t  tick;
    int32_t  raw;
    int32_t  white_lvl;
    int32_t  black_lvl;
    int32_t  low        = 4095;
    int32_t  high       = 0;
    uint32_t prev       = 0xFFu;
    uint32_t edges_prev = barcode_edges();
    bool     b_ana      = barcode_adc(&raw, &white_lvl, &black_lvl);

    tm_printf(
        app_str("\n[S3] IR live view for 30 s: slide black tape under each "
                "sensor. L R B = line sensor left of the tape, right of the "
                "tape, "
                "barcode (1 = black). Centred on the tape: L 0 and R 0\n"));

    if (!b_ana)
    {
        tm_printf((
            uint8_t *)"[S3] B on its DIGITAL output (GP2). Tune: hold it STILL "
                      "over white, then STILL over black; 'B edges/s' must "
                      "read 0 "
                      "in both (turn its pot)\n");
    }
    else
    {
        tm_printf((
            uint8_t *)"[S3] B on its ANALOGUE output (GP28, module on 3V3): "
                      "its "
                      "pot does nothing here. Pass B over white, over black "
                      "tape "
                      "(it "
                      "reads 0 until it has seen black once), and over a grout "
                      "line or "
                      "tile join. Each second: the lowest and highest reading. "
                      "At "
                      "the "
                      "end: thresholds to copy into subsystem3.h. B reads 1 "
                      "over "
                      "WHITE = this module's output is lower over black: set "
                      "S3_BAR_BLACK_HIGH 0\n");

        if (S3_BAR_RELATIVE)
        {
            tm_printf(
                app_str("[S3] B uses RELATIVE edges: black once the reading "
                        "rises %d above the lowest since the last bar (and "
                        "is at "
                        "least %d), white once it falls %d below the "
                        "highest. "
                        "White "
                        "must stay under %d everywhere\n"),
                S3_BAR_DELTA, S3_BAR_BLACK_MIN, S3_BAR_DELTA, S3_BAR_BLACK_MIN);
        }
        else if (S3_BAR_ADC_ON > 0)
        {
            tm_printf(
                app_str("[S3] B uses FIXED thresholds: black from %d, white "
                        "again below %d (S3_BAR_ADC_ON/OFF; 0 = learn)\n"),
                S3_BAR_ADC_ON, S3_BAR_ADC_OFF);
        }
        else
        {
            /* nothing to do for any other case */
        }
    }

    for (tick = 0; tick < IR_VIEW_TICKS; tick++)
    {
        /* comparisons are 0 or 1 */
        uint32_t now =
            ((uint32_t)(IR_BLACK == app_gpio_get(PIN_IR_LINE_L)) << 2) |
            ((uint32_t)(IR_BLACK == app_gpio_get(PIN_IR_LINE_R)) << 1) |
            (barcode_black() ? 1u : 0u);

        if (now != prev)
        {
            /* single bits: fit %d */
            tm_printf(app_str("[S3] L %d  R %d  B %d\n"),
                      (int32_t)((now >> 2) & 1u), (int32_t)((now >> 1) & 1u),
                      (int32_t)(now & 1u));
            prev = now;
        }

        if (b_ana)
        {
            (void)barcode_adc(&raw, &white_lvl, &black_lvl);

            if (raw < low)
            {
                low = raw;
            }

            if (raw > high)
            {
                high = raw;
            }
        }

        if (99 == (tick % 100))
        {
            /* the interrupt sees every flicker; the 10 ms view above does not
             */
            uint32_t edges_now = barcode_edges();

            /* edges in one second (a few hundred) and one bit: fit %d */
            tm_printf(app_str("[S3] B edges/s %d (B now %d)\n"),
                      (int32_t)(edges_now - edges_prev), (int32_t)(now & 1u));

            if (b_ana)
            {
                tm_printf(
                    app_str("[S3] B analogue now %d, this second %d..%d "
                            "(learned white %d, black %d; they must differ "
                            "by "
                            "%d or more)\n"),
                    raw, low, high, white_lvl, black_lvl, S3_BAR_MIN_SPAN);
                low  = 4095;
                high = 0;
            }

            edges_prev = edges_now;
        }

        tk_dly_tsk(10);
    }

    if (b_ana)
    {
        (void)barcode_adc(&raw, &white_lvl, &black_lvl);

        if (S3_BAR_RELATIVE)
        {
            tm_printf(app_str("[S3] B: whitest %d, darkest %d seen%s\n"),
                      white_lvl, black_lvl,
                      (((black_lvl - white_lvl) < S3_BAR_MIN_SPAN) &&
                       ((white_lvl - black_lvl) < S3_BAR_MIN_SPAN))
                          ? " (it never saw black: pass it over tape)"
                          : "");
        }
        else if (((black_lvl - white_lvl) >= S3_BAR_MIN_SPAN) ||
                 ((white_lvl - black_lvl) >= S3_BAR_MIN_SPAN))
        {
            /* black from 60 % of the way from white to black, white
               again below 40 %: grout should stay under 40 % */
            tm_printf(
                app_str("[S3] B thresholds from what it saw: "
                        "S3_BAR_ADC_ON %d, S3_BAR_ADC_OFF %d. Grout or a "
                        "tile "
                        "join must stay on the white side of OFF (see the "
                        "readings above); if not, tell B3 before using "
                        "them\n"),
                white_lvl + ((black_lvl - white_lvl) * 6 / 10),
                white_lvl + ((black_lvl - white_lvl) * 4 / 10));
        }
        else
        {
            tm_printf(
                app_str("[S3] B has not seen enough contrast (white %d, "
                        "black %d): check AO is on GP28, the module is on "
                        "3V3, "
                        "and pass it over black tape\n"),
                white_lvl, black_lvl);
        }
    }
}

/* Junctions seen in the last follow_and_watch(), by kind */
LOCAL int32_t g_j_cross;
LOCAL int32_t g_j_left;

/** Follow the line for up to secs (and up to max_mm, 0 = no limit); print
   junctions, barcodes, line lost */
LOCAL int32_t follow_and_watch (int32_t secs, int32_t max_mm, int32_t * p_junc,
                                int32_t * p_lost, int32_t * p_codes_ok,
                                int32_t * p_codes_fail)
{
    int32_t          odo0        = motor_odometer();
    uint32_t         edges_start = barcode_edges();
    uint32_t         junc_start  = line_junctions();
    uint32_t         junc_now;
    barcode_result_t result;
    uint32_t         seq;
    int32_t          tick;
    int32_t          last_state = LINE_ON;
    int32_t          traced     = 0;
    uint32_t         lc_prev    = 0xFFu;
    int32_t          dbg_count;
    int32_t          dbg_widths[BARCODE_DBG_MAX];
    int32_t          kdx;
    uint32_t dseq = barcode_debug(&dbg_count, dbg_widths, BARCODE_DBG_MAX);
    int32_t  mark_odo;
    int32_t  mark_width;
    uint32_t mseq = barcode_mark(&mark_odo, &mark_width);

    barcode_last(&result);
    seq       = result.seq;
    g_j_cross = g_j_left = 0;
    barcode_reset(); /* nothing left over from the live view */
    line_set_enabled(true);
    tm_printf(
        app_str("[S3] at start: L %d  R %d (want L 0, R 0: the tape centred "
                "between them)\n"),
        IR_BLACK == app_gpio_get(PIN_IR_LINE_L),
        IR_BLACK == app_gpio_get(PIN_IR_LINE_R));

    if (motion_follow(S2_SPEED_BARCODE) < E_OK)
    {
        return 0;
    }

    for (tick = 0; (tick < (secs * TICKS_PER_S)) && (!app_test_stop_pressed());
         tick++)
    {
        uint32_t pattern;

        tk_dly_tsk(10);

        if ((max_mm > 0) &&
            (motion_counts_to_mm(motor_odometer() - odo0) >= max_mm))
        {
            break;
        }

        /* Trace what the line sensors see: every change, first 150 */
        /* comparisons are 0 or 1 */
        pattern = ((uint32_t)(IR_BLACK == app_gpio_get(PIN_IR_LINE_L)) << 1) |
                  (uint32_t)(IR_BLACK == app_gpio_get(PIN_IR_LINE_R));

        if ((pattern != lc_prev) && (traced < 150))
        {
            /* single bits: fit %d */
            tm_printf(app_str("[S3T] %d mm  L %d  R %d  steer %d\n"),
                      motion_counts_to_mm(motor_odometer() - odo0),
                      (int32_t)(pattern >> 1), (int32_t)(pattern & 1u),
                      line_steer_mm_s());
            lc_prev = pattern;
            traced++;
        }

        junc_now = line_junctions();

        /* *p_junc counts up from 0: never negative */
        if (junc_now != (junc_start + (uint32_t)*p_junc))
        {
            int32_t sides = line_junction_sides();

            (*p_junc)++;

            if ((sides & JUNC_SIDE_S) != 0)
            {
                g_j_cross++;
            }
            else
            {
                g_j_left++;
            }

            tm_printf(
                app_str("[S3] junction %d at %d mm: %s\n"), *p_junc,
                motion_counts_to_mm(motor_odometer() - odo0),
                (sides == (JUNC_SIDE_S | JUNC_SIDE_B))
                    ? "crossing (both line sensors black, B saw the left arm)"
                : ((sides & JUNC_SIDE_S) != 0)
                    ? "crossing (both line sensors black)"
                    : "branch left (seen by B)");
        }

        if (line_state() != last_state)
        {
            last_state = line_state();

            if (LINE_LOST == last_state)
            {
                (*p_lost)++;
                tm_printf(app_str("[S3] LINE LOST, stopping\n"));
                break;
            }

            if (LINE_FAULT == last_state)
            {
                (*p_lost)++;
                tm_printf(
                    app_str("[S3] LINE SENSOR FAULT: %s, stopping\n"),
                    (LINE_FAULT_ONE == line_fault())
                        ? "one sensor black for 30 cm (stuck black?)"
                        : "both black for 15 cm (stuck, or a wide black area)");
                break;
            }
        }

        if (barcode_debug(&dbg_count, dbg_widths, BARCODE_DBG_MAX) != dseq)
        {
            dseq = barcode_debug(&dbg_count, dbg_widths, BARCODE_DBG_MAX);

            if (dbg_count >= 2)
            {
                /* 1 = a grout line or junction */
                tm_printf(app_str("[S3] code ended, %d elements (mm):"),
                          dbg_count);

                for (kdx = 0; (kdx < dbg_count) && (kdx < BARCODE_DBG_MAX);
                     kdx++)
                {
                    tm_printf(app_str("%s%d.%d"),
                              ((0 == (kdx % 10)) && (kdx > 0)) ? " | " : " ",
                              dbg_widths[kdx] / 10, dbg_widths[kdx] % 10);
                }

                tm_printf(app_str("\n"));
            }
        }

        /* first bars under B: a left arm if about one tape wide */
        if (barcode_mark(&mark_odo, &mark_width) != mseq)
        {
            mseq = barcode_mark(&mark_odo, &mark_width);
            tm_printf(
                app_str("[S3] %d mm: B patch %d.%d mm (%s)\n"),
                motion_counts_to_mm(mark_odo - odo0), mark_width / 10,
                mark_width % 10,
                ((mark_width >= (S3_JUNC_MIN_MM * 10)) &&
                 (mark_width <= (S3_JUNC_MAX_MM * 10)))
                    ? "tape width: a left arm, if the robot drove straight "
                      "there"
                    : "not tape width: grout or a barcode bar, not a junction");
        }

        barcode_last(&result);

        if (result.seq != seq)
        {
            seq = result.seq;

            if (result.b_ok)
            {
                (*p_codes_ok)++;
                tm_printf(app_str("[S3] barcode %c%s%s, narrow %d.%d mm, "
                                  "wide %d.%d "
                                  "mm\n"),
                          result.letter,
                          result.b_reversed ? " (backwards)" : "",
                          (BAR_FIX_STAR == result.fixed) ? " (one * damaged)"
                          : (BAR_FIX_MERGED == result.fixed)
                              ? " (a lost bar put back)"
                          : (BAR_FIX_CUT == result.fixed)
                              ? " (a missing end bar put back)"
                              : "",
                          result.narrow_mm10 / 10, result.narrow_mm10 % 10,
                          result.wide_mm10 / 10, result.wide_mm10 % 10);
            }
            else
            {
                (*p_codes_fail)++;
                tm_printf(app_str("[S3] barcode READ_FAIL, %d elements\n"),
                          result.elements);
            }
        }
    }

    motion_halt();
    line_set_enabled(false); /* the next run starts clean */
    /* edges in one run, thousands at most: fit %d */
    tm_printf(app_str("[S3] barcode sensor B: %d edges this run "
                      "(0 = it never saw black)\n"),
              (int32_t)(barcode_edges() - edges_start));
    tm_printf(
        app_str("[S3] junctions this run: crossings %d, left branches %d\n"),
        g_j_cross, g_j_left);
    {
        uint32_t corrections;
        uint32_t sweeps;
        int32_t  dist_mm = motion_counts_to_mm(motor_odometer() - odo0);

        line_wobble(&corrections, &sweeps);
        /* counts in one run, hundreds at most: fit %d */
        tm_printf(
            app_str("[S3] wobble this run: %d corrections (%d per metre), "
                    "%d sweeps right across the tape (a zigzag; want 0)\n"),
            (int32_t)corrections,
            (dist_mm > 0) ? (int32_t)corrections * 1000 / dist_mm : 0,
            (int32_t)sweeps);
    }
    return motion_counts_to_mm(motor_odometer() - odo0); /* mm followed */
}

/** START = run this part, STOP = skip it.  true = run it. */
LOCAL bool ask_run (const char * p_what)
{
    uint32_t remote = telemetry_start_count();

    app_stop_clear(STOP_BUTTON | STOP_MQTT); /* a STOP from the last run */
    tm_printf(
        app_str("\n[DRIVE] %s\n[DRIVE] START (GP20) = run it, STOP (GP21) = "
                "skip it (or START / STOP on the panel)\n"),
        p_what);

    for (;;)
    {
        if ((button_is_down(PIN_STOP)) ||
            ((app_stop_reasons() & STOP_MQTT) != 0u))
        {
            while (button_is_down(PIN_STOP))
            {
                tk_dly_tsk(10);
            }

            tk_dly_tsk(50);
            app_stop_clear(STOP_BUTTON | STOP_MQTT);
            tm_printf(app_str("[DRIVE] skipped\n"));
            return false;
        }

        if (telemetry_start_count() != remote)
        {
            return true;
        }

        if (button_is_down(PIN_START))
        {
            tk_dly_tsk(30); /* debounce */

            if ((button_is_down(PIN_START)) && (!button_is_down(PIN_STOP)))
            {
                while (button_is_down(PIN_START))
                {
                    tk_dly_tsk(10);
                }

                return true;
            }
        }

        tk_dly_tsk(10);
    }
}

/** Print the result of the last code, if a new one came: true = decoded */
LOCAL bool print_new_code (uint32_t * p_seq, uint32_t * p_dseq)
{
    barcode_result_t result;
    int32_t          dbg_count;
    int32_t          dbg_widths[BARCODE_DBG_MAX];
    int32_t          kdx;
    bool             b_ok = false;

    if (barcode_debug(&dbg_count, dbg_widths, BARCODE_DBG_MAX) != *p_dseq)
    {
        *p_dseq = barcode_debug(&dbg_count, dbg_widths, BARCODE_DBG_MAX);
        tm_printf(app_str("[S3] code ended, %d elements (mm):"), dbg_count);

        for (kdx = 0; (kdx < dbg_count) && (kdx < BARCODE_DBG_MAX); kdx++)
        {
            tm_printf(app_str("%s%d.%d"),
                      ((0 == (kdx % 10)) && (kdx > 0)) ? " | " : " ",
                      dbg_widths[kdx] / 10, dbg_widths[kdx] % 10);
        }

        tm_printf(app_str("\n"));
    }

    barcode_last(&result);

    if (result.seq != *p_seq)
    {
        *p_seq = result.seq;
        if (result.b_ok)
        {
            b_ok = true;
            tm_printf(app_str("[S3] barcode %c%s%s, narrow %d.%d mm, wide "
                              "%d.%d mm\n"),
                      result.letter, result.b_reversed ? " (backwards)" : "",
                      (BAR_FIX_STAR == result.fixed) ? " (one * damaged)"
                      : (BAR_FIX_MERGED == result.fixed)
                          ? " (a lost bar put back)"
                      : (BAR_FIX_CUT == result.fixed)
                          ? " (a missing end bar put back)"
                          : "",
                      result.narrow_mm10 / 10, result.narrow_mm10 % 10,
                      result.wide_mm10 / 10, result.wide_mm10 % 10);
        }
        else
        {
            tm_printf(app_str("[S3] barcode READ_FAIL, %d elements (a code "
                              "is 29)\n"),
                      result.elements);
        }
    }

    return b_ok;
}

/* S3-B00: B straight over one barcode, no line following, so only the
   sensor and the decoder are tested, not the steering.  With the analogue
   input it also keeps the raw reading every 10 ms (0.5 mm at 50 mm/s)
   and prints each bar and space with its darkest / lightest reading; one
   with little contrast to its neighbour is marked '!' (the sensor's spot
   is nearly as wide as the bar: lower the module). */
#define BENCH_MM        300
#define BENCH_N         1000
#define BENCH_TICKS     2000 /* 20 s at 10 ms */
#define BENCH_PRINT_MAX 80   /* edges printed */
LOCAL int32_t  g_bn_odo[BENCH_N];
LOCAL uint16_t g_bn_v[BENCH_N];

/** Bench view: bigger = blacker, whichever way the module is wired,
   and back to the raw reading */
Inline int32_t bench_blackness (int32_t raw)
{
    return S3_BAR_BLACK_HIGH ? raw : (4095 - raw);
}
/** Bench view: blackness back to the raw analogue reading. */
Inline int32_t bench_raw (int32_t blackness)
{
    return S3_BAR_BLACK_HIGH ? blackness : (4095 - blackness);
}

/** The S3-B00 profile: the reader's own rule (S3_BAR_RELATIVE) on the
   count 10 ms samples of the run, each bar and space with its darkest /
   lightest reading.  '!' = its contrast to the element before was under
   2 x S3_BAR_DELTA, close to being missed (a narrow bar or space the spot
   blurs). */
LOCAL void bench_profile (int32_t count)
{
    int32_t idx;
    int32_t cnt = 0;
    int32_t low;
    int32_t high;
    int32_t ref       = 0;
    int32_t weak      = 0;
    bool    b_blk     = false;
    bool    b_started = false;
    int32_t st_odo    = 0;

    low = high = bench_blackness(g_bn_v[0]);

    for (idx = 1; (idx < count) && (cnt < BENCH_PRINT_MAX); idx++)
    {
        int32_t blackness = bench_blackness(g_bn_v[idx]);
        bool    b_change;

        if (!b_blk)
        {
            if (blackness < low)
            {
                low = blackness;
            }

            b_change = ((blackness > (low + S3_BAR_DELTA)) &&
                        (blackness >= bench_blackness(S3_BAR_BLACK_MIN)));
        }
        else
        {
            if (blackness > high)
            {
                high = blackness;
            }

            b_change = (blackness < (high - S3_BAR_DELTA));
        }

        if (!b_change)
        {
            continue;
        }

        if (!b_started)
        {
            b_started = true; /* the white before the code */
            tm_printf(app_str("[S3] B profile, bar (B) and space (s): width "
                              "mm / darkest or lightest reading; '!' = under "
                              "%d "
                              "from the one before, nearly missed:\n[S3]  "),
                      2 * S3_BAR_DELTA);
        }
        else
        {
            int32_t width = g_bn_odo[idx] - st_odo;
            int32_t mm10 =
                ((width < 0) ? -width : width) * S2_UM_PER_COUNT / 100;
            int32_t ext = b_blk ? high : low;
            bool b_bad  = (b_blk ? ext - ref : ref - ext) < (2 * S3_BAR_DELTA);

            if (b_bad)
            {
                weak++;
            }

            tm_printf(app_str("%c%d.%d/%d%s "), b_blk ? 'B' : 's', mm10 / 10,
                      mm10 % 10, bench_raw(ext), b_bad ? "!" : "");

            cnt++;

            if (0 == (cnt % 10))
            {
                tm_printf(app_str("\n[S3]  "));
            }
        }

        ref    = b_blk ? high : low;
        st_odo = g_bn_odo[idx];

        if (!b_blk)
        {
            ref   = low;
            b_blk = true;
            high  = blackness;
        }
        else
        {
            b_blk = false;
            low   = blackness;
        }
    }

    if (b_blk && b_started)
    {
        /* the last bar */
        int32_t width = g_bn_odo[count - 1] - st_odo;
        int32_t mm10  = ((width < 0) ? -width : width) * S2_UM_PER_COUNT / 100;

        tm_printf(app_str("B%d.%d/%d (end of samples) "), mm10 / 10, mm10 % 10,
                  bench_raw(high));
        cnt++;
    }

    if (!b_started)
    {
        tm_printf(app_str("[S3] B profile: no bar in %d samples (lightest %d, "
                          "darkest %d): B did not pass over the bars\n"),
                  count, bench_raw(low), bench_raw(high));
    }
    else
    {
        tm_printf(app_str("\n[S3] %d bars and spaces (a code is 29), %d "
                          "nearly "
                          "missed. At 120 mm/s the samples are 1.2 mm apart: "
                          "narrow "
                          "ones blur together here, not in the reader\n"),
                  cnt, weak);
    }
}

/** One S3-B00 run: drives straight over a barcode and prints the profile. */
LOCAL bool bar_bench_run (int32_t speed)
{
    motion_status_t  status;
    barcode_result_t result;
    uint32_t         seq;
    uint32_t         dseq;
    int32_t          count = 0;
    int32_t          tick;
    int32_t          dbg_count;
    int32_t          dbg_widths[BARCODE_DBG_MAX];
    int32_t          raw;
    int32_t          white_lvl;
    int32_t          black_lvl;
    bool             b_ana = false;
    bool             b_ok  = false;

    barcode_last(&result);
    seq  = result.seq;
    dseq = barcode_debug(&dbg_count, dbg_widths, BARCODE_DBG_MAX);
    barcode_reset();

    if (motion_drive(speed, BENCH_MM) < E_OK)
    {
        return false;
    }

    for (tick = 0; (tick < BENCH_TICKS) && (!app_test_stop_pressed()); tick++)
    {
        tk_dly_tsk(10);
        b_ana = barcode_adc(&raw, &white_lvl, &black_lvl);

        if (b_ana && (count < BENCH_N))
        {
            g_bn_odo[count] = motor_odometer();
            g_bn_v[count]   = (uint16_t)raw; /* 12-bit ADC: 0..4095 */
            count++;
        }

        if (print_new_code(&seq, &dseq))
        {
            b_ok = true;
        }

        motion_get_status(&status);

        if ((tick > 10) && (status.result != MOT_RES_BUSY))
        {
            break;
        }
    }

    motion_halt();
    tk_dly_tsk(50);

    if (print_new_code(&seq, &dseq))
    {
        b_ok = true;
    }

    if (b_ana && (count > 0))
    {
        bench_profile(count);
    }

    return b_ok;
}

/** Stage 2 S3-B00: barcode reads driving straight, four runs. */
LOCAL void bar_bench (void)
{
    static const struct
    {
        const char * p_what;
        int32_t      speed;
    } run[] = {
        {"a barcode, B about 5 cm before its first bar", S2_SPEED_IN_BARCODE},
        {"the SAME barcode from the other end (robot turned round)",
         S2_SPEED_IN_BARCODE},
        {"the thinnest barcode (\"A small\"), any direction",
         S2_SPEED_IN_BARCODE},
        {"any barcode, at the FOLLOW speed (the robot slows only once B "
         "is on the first bar, so the start is read at this speed)",
         S2_SPEED_BARCODE},
    };
    int32_t idx;
    TEST_INIT();

    /* a handful of entries: the count fits int32_t (here and below) */
    for (idx = 0; idx < (int32_t)(sizeof(run) / sizeof(run[0])); idx++)
    {
        bool b_ok;

        /* a handful of entries: the count fits %d */
        tm_printf(
            app_str("\n[DRIVE] S3-B00 run %d of %d: %s. Square to the bars, "
                    "B over them, the line does not matter. It drives "
                    "straight %d mm at %d mm/s. Write down the code.\n"),
            idx + 1, (int32_t)(sizeof(run) / sizeof(run[0])), run[idx].p_what,
            BENCH_MM, run[idx].speed);
        app_test_wait_start(
            "Robot placed (no such barcode: START, then STOP at once).");
        b_ok = bar_bench_run(run[idx].speed);
        TEST_CHECK("S3-B00", b_ok, "decoded driving straight");
    }

    TEST_SUMMARY("subsystem3 S3-B00");
}

/** Stage 2 hands-on tests: line following, junctions, barcodes, junction
 * moves. */
EXPORT void subsystem3_test_stage (void)
{
    int32_t junc     = 0;
    int32_t lost     = 0;
    int32_t codes_ok = 0;
    int32_t fail     = 0;
    int32_t kdx;
    int32_t dist;
    TEST_INIT();

    tm_printf(
        app_str("\n[DRIVE] === Stage 2: line, junctions, barcodes ===\n"
                "[DRIVE] Speed %d mm/s. STOP (GP21) stops at any time.\n"),
        S2_SPEED_BARCODE);
    ir_live();

    if (ask_run("--- Stage 2A: line + junctions (S3-L01..L10) ---"))
    {
        app_test_wait_start(
            "S3-L01 / S3-L02: robot on the line. It follows for up to "
            "60 s (let it run; STOP only if it goes wrong); count the "
            "junctions you see.");
        dist = follow_and_watch(60, 0, &junc, &lost, &codes_ok, &fail);
        tm_printf(
            app_str("[S3-L01] followed %d mm, line lost %d times; [S3-L02] "
                    "junctions counted %d (write down the real number)\n"),
            dist, lost, junc);
        TEST_CHECK("S3-L01", (0 == lost) && (dist >= 3000),
                   "followed at least 3 m without LINE_LOST");

        /* One kind of place at a time: the robot drives at most max_mm, so
           only that place is in the run.  No such place on the track: press
           STOP straight after START (the check then fails: write "not on
           the track" in the log). */
        {
            static const struct
            {
                const char * p_test_id;
                const char * p_where;
                const char * p_pass;
                int32_t      max_mm;
                int32_t      want_cross;
                int32_t      want_left;
                bool b_want_fault; /* true: must stop with LINE_FAULT_ONE */
            } pl[] = {
                /* S3-L04 (right-only branch) dropped 7 Nov: the track has
                   none, and with the sensors straddling the tape it looks
                   like an ordinary correction */
                {"S3-L10",
                 "about 20 cm before the X CROSSING (track 3), square "
                 "to it",
                 "exactly 1 junction, a crossing", 450, 1, 0},
                {"S3-L03",
                 "about 20 cm before a branch to the LEFT only (the "
                 "line goes on), no barcode next to it",
                 "exactly 1 junction, branch left", 450, 0, 1},
                {"S3-L05",
                 "about 20 cm before a T end (the line stops at a cross "
                 "bar with arms both ways)",
                 "exactly 1 junction, a crossing", 450, 1, 0},
                {"S3-L06",
                 "at the start of the tightest RIGHT curve with no "
                 "branches",
                 "no junction and no line lost", 1200, 0, 0},
                {"S3-L07",
                 "at the start of the tightest LEFT curve with no "
                 "branches",
                 "no junction and no line lost", 1200, 0, 0},
                {"S3-L08",
                 "about 20 cm before a GAP in the line (track 2), "
                 "after a straight",
                 "drives across, no line lost, no junction", 450, 0, 0},
                {"S3-L09",
                 "on a straight. Once it moves, hold a black card "
                 "under R (the RIGHT line sensor) and keep it there (walk "
                 "along)",
                 "stops with LINE SENSOR FAULT (one sensor black) within "
                 "about 30 cm",
                 1000, 0, 0, true},
            };
            int32_t idx;

            tm_printf(
                app_str("\n[DRIVE] --- Stage 2A: one place at a time ---\n"));

            /* a handful of entries: the count fits int32_t */
            for (idx = 0; idx < (int32_t)(sizeof(pl) / sizeof(pl[0])); idx++)
            {
                int32_t part_junc = 0;
                int32_t part_lost = 0;
                int32_t part_ok   = 0;
                int32_t part_fail = 0;

                tm_printf(app_str("\n[DRIVE] %s: robot on the line %s. It "
                                  "drives at "
                                  "most %d mm. Pass: %s.\n"),
                          pl[idx].p_test_id, pl[idx].p_where, pl[idx].max_mm,
                          pl[idx].p_pass);
                app_test_wait_start(
                    "Robot placed as above (no such place: START, "
                    "then STOP at once).");
                follow_and_watch(30, pl[idx].max_mm, &part_junc, &part_lost,
                                 &part_ok, &part_fail);

                if (pl[idx].b_want_fault)
                {
                    TEST_CHECK(pl[idx].p_test_id,
                               (LINE_FAULT == line_state()) &&
                                   (LINE_FAULT_ONE == line_fault()),
                               pl[idx].p_pass);
                }
                else
                {
                    TEST_CHECK(pl[idx].p_test_id,
                               (0 == part_lost) &&
                                   (g_j_cross == pl[idx].want_cross) &&
                                   (g_j_left == pl[idx].want_left),
                               pl[idx].p_pass);
                }
            }
        }
    } /* 2A */

    if (ask_run("--- S3-B00: barcode read driving STRAIGHT, no line "
                "following (4 runs). Do this before 2B ---"))
    {
        bar_bench();
    }

    if (ask_run(
            "--- Stage 2B: barcodes while following the line (3 runs). "
            "Before each run write down the codes on the track in the "
            "order the robot meets them; compare with the printed ones ---"))
    {
        for (kdx = 1; kdx <= BAR_RUNS; kdx++)
        {
            int32_t part_junc = 0;
            int32_t part_lost = 0;
            tm_printf(app_str("\n[DRIVE] S3-B01..B03 barcodes, run %d of 3\n"),
                      kdx);
            app_test_wait_start(
                "Put the robot on the line before a stretch with barcodes "
                "(both directions over the runs). Up to 40 s.");
            follow_and_watch(40, 0, &part_junc, &part_lost, &codes_ok, &fail);
        }

        tm_printf(app_str("[S3-B01..B03] decoded %d, READ_FAIL %d (write "
                          "down which "
                          "codes were on the track)\n"),
                  codes_ok, fail);
    } /* 2B */

    TEST_SUMMARY("subsystem3 stage 2");

    if (ask_run("--- Stage 2C: S5-B01 / S5-B02 junction moves. Mission takes "
                "over, obstacle watch and IMU OFF (untested until Stages 3 "
                "and 4) ---"))
    {
        app_test_course(false, false);
    }
}

#endif /* APP_TEST */

/*** end of file ***/

