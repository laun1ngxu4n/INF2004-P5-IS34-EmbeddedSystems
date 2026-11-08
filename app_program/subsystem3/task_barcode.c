/*
 *  task_barcode.c
 *  Subsystem 3 -- the Barcode Decoder task (wakes every 10 ms).
 *
 *  The GP2 interrupt stores every edge with the odometer reading (encoder
 *  counts), so element widths are distances (TEAM.md 7.1 B3).  When the
 *  robot has moved S3_QUIET_MM past the last edge, the edges since the
 *  first black bar are one barcode:
 *    - n elements must be 10k - 1 (k characters of 9 elements, a narrow
 *      gap between characters), k >= 3 ("*X*");
 *    - in each character the 3 widest of 9 are wide (Code 39 rule);
 *    - the first character must be '*'.  If it is not, the elements are
 *      reversed and decoded again: read backwards (B2);
 *    - a stray mark within S3_QUIET_MM before or after the code (a tile
 *      join, grout) adds a bar and a space: the "*X*" is looked for at
 *      every bar in the run, not only at the first.
 *  Result: barcode_last(), and MSN_FLG_BARCODE to Mission.
 */

#include "subsystem3.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem5/subsystem5.h"

#define RING        128 /* edges kept, a power of 2 */
#define MAX_ELEM    99  /* up to 10 characters */
#define S3_QUIET_MM 40  /* no edge for this far = end of barcode */
/* < 1.2 mm apart = sensor chatter. The sensor flickers at its threshold
 * (thousands of edges, Stage 2 2 Nov); must stay under half the narrowest
 * bar: 3.5 mm small codes = 44 counts */
#define MERGE_COUNTS    15
#define MIN_ELEM_REPORT 20 /* fewer elements = noise, not reported */
/* the 3 widest of 9 at least 1.3 x the rest. Was 1.5: blur widens a narrow
 * element beside a wide one (4.6 mm against 7.0 wide, simulated "A small"),
 * and 1.5 lost 4 reads in 18 at spot sigma 1.6 mm; 1.3 none. Random widths:
 * 0 of 20000 decode either way. */
#define S3_WIDE_RATIO_X10 13

typedef struct
{
    W  odo;
    UB level;
} bar_edge_t;

LOCAL bar_edge_t  g_ring[RING];
LOCAL volatile UW g_ring_w; /* written by the interrupt */
LOCAL UW          g_ring_r; /* read by the task */

LOCAL volatile BOOL gb_busy;    /* inside a barcode (in_code) */
LOCAL volatile BOOL gb_paused;  /* barcode_reset(): ignore edges */
LOCAL volatile UW   g_seq_now;  /* results published so far */
LOCAL volatile W    g_edge_pub; /* odometer at the last clean edge */

#define S3_IN_BARS_MM 15 /* edge this recent = B over bars */

LOCAL barcode_result_t g_last;
LOCAL UB               g_map_ch[4];
LOCAL UB               g_map_mv[4];
LOCAL BOOL             gb_map_set;

/* Code 39: 9 elements bar/space/bar..., 1 = wide.  Bit 8 = first bar. */
typedef struct
{
    UB letter;
    UH pat;
} c39_char_t;
LOCAL const c39_char_t g_c39[] = {
    {'0', 0x034}, {'1', 0x121}, {'2', 0x061}, {'3', 0x160}, {'4', 0x031},
    {'5', 0x130}, {'6', 0x070}, {'7', 0x025}, {'8', 0x124}, {'9', 0x064},
    {'A', 0x109}, {'B', 0x049}, {'C', 0x148}, {'D', 0x019}, {'E', 0x118},
    {'F', 0x058}, {'G', 0x00D}, {'H', 0x10C}, {'I', 0x04C}, {'J', 0x01C},
    {'K', 0x103}, {'L', 0x043}, {'M', 0x142}, {'N', 0x013}, {'O', 0x112},
    {'P', 0x052}, {'Q', 0x007}, {'R', 0x106}, {'S', 0x046}, {'T', 0x016},
    {'U', 0x181}, {'V', 0x0C1}, {'W', 0x1C0}, {'X', 0x091}, {'Y', 0x190},
    {'Z', 0x0D0}, {'-', 0x085}, {'.', 0x184}, {' ', 0x0C4}, {'*', 0x094},
    {'$', 0x0A8}, {'/', 0x0A2}, {'+', 0x08A}, {'%', 0x02A}};

/* GP2 edge: interrupt context */
LOCAL void barcode_edge_isr (UINT pin, UW events)
{
    UW ring_pos = g_ring_w;

    (void)events;
    g_ring[ring_pos & (RING - 1)].odo   = motion_odometer();
    g_ring[ring_pos & (RING - 1)].level = (UB)app_gpio_get(pin);
    g_ring_w                            = ring_pos + 1;
}

#if !S3_BARCODE_ANALOG

EXPORT ER barcode_hw_init (void)
{
    ER err = app_gpio_input(PIN_IR_BARCODE, FALSE);

    return (err < E_OK) ? err
                        : app_gpio_irq_add(PIN_IR_BARCODE, APP_GPIO_BOTH,
                                           barcode_edge_isr);
}
EXPORT BOOL barcode_black (void)
{
    return IR_BLACK == app_gpio_get(PIN_IR_BARCODE);
}
EXPORT BOOL barcode_adc (INT * p_raw, INT * p_white, INT * p_black)
{
    *p_raw = *p_white = *p_black = 0;
    return FALSE;
}

#else /* S3_BARCODE_ANALOG */

/* AO on GP28, sampled by the ADC at 2 kHz (0.075 mm apart at 150 mm/s).
   The interrupt turns the samples into black/white with hysteresis and
   stores each change exactly like a GP2 edge, so the decoder below is the
   same for both inputs.  The kernel's sample ADC driver ("adca") is not
   used by the application; its interrupt handler is replaced here. */
#define ADC_REG(o)     (0x4004C000u + (o))
#define ADC_CS         ADC_REG(0x00)
#define ADC_FCS        ADC_REG(0x08)
#define ADC_FIFO       ADC_REG(0x0C)
#define ADC_DIV        ADC_REG(0x10)
#define ADC_INTE       ADC_REG(0x18)
#define CS_EN          (1u << 0)
#define CS_START_MANY  (1u << 3)
#define CS_READY       (1u << 8)
#define FCS_EN         (1u << 0)
#define FCS_EMPTY      (1u << 8)
#define FCS_THRESH(n)  ((UW)(n) << 24)
#define INTNO_ADC_FIFO 22
#define INTPRI_ADC     2 /* below the GPIO edges (1) */
#define ADC_RATE_HZ    2000
#define ENV_SH         4 /* levels kept x16 */
#define ENV_UP_SH      3 /* follow a new extreme in ~8 samples */
/* forget it over ~9 min. B is beside the line and sees black only at
 * barcodes and branches: forgetting in 16 s (shift 15) lost the black level
 * within a minute of driving, then a darker tile join could be learned as
 * "black". */
#define ENV_DECAY_SH   20

LOCAL volatile INT g_adc_raw;
LOCAL INT          g_env_w = -1;
LOCAL INT          g_env_b = -1; /* white / black level x16 */
LOCAL INT          g_rel_lo;
LOCAL INT          g_rel_hi; /* S3_BAR_RELATIVE: extremes so far */
LOCAL volatile UB  g_adc_lvl = !IR_BLACK;

/* bigger = blacker, whichever way the module is wired */
#define BLACKNESS(v)   (S3_BAR_BLACK_HIGH ? (v) : 4095 - (v))

LOCAL void adc_sample (INT sample)
{
    INT black = BLACKNESS(sample) << ENV_SH;
    UB  lvl   = g_adc_lvl;

    g_adc_raw = sample;
    if (g_env_w < 0)
    {
        g_env_w = g_env_b = black;
        g_rel_lo = g_rel_hi = BLACKNESS(sample);
    }
    /* black level: rises fast to a new darkest, sinks very slowly */
    if (black > g_env_b)
    {
        g_env_b += (black - g_env_b) >> ENV_UP_SH;
    }
    else
    {
        g_env_b -= (g_env_b - black) >> ENV_DECAY_SH;
    }
    /* white level: the same the other way */
    if (black < g_env_w)
    {
        g_env_w -= (g_env_w - black) >> ENV_UP_SH;
    }
    else
    {
        g_env_w += (black - g_env_w) >> ENV_DECAY_SH;
    }

    if (S3_BAR_RELATIVE)
    {
        INT blackness = BLACKNESS(sample);

        if (lvl != IR_BLACK)
        {
            if (blackness < g_rel_lo)
            {
                g_rel_lo = blackness;
            }
            if ((blackness > (g_rel_lo + S3_BAR_DELTA)) &&
                (blackness >= BLACKNESS(S3_BAR_BLACK_MIN)))
            {
                lvl      = IR_BLACK;
                g_rel_hi = blackness;
            }
        }
        else
        {
            if (blackness > g_rel_hi)
            {
                g_rel_hi = blackness;
            }
            if (blackness < (g_rel_hi - S3_BAR_DELTA))
            {
                lvl      = !IR_BLACK;
                g_rel_lo = blackness;
            }
        }
    }
    else if (S3_BAR_ADC_ON > 0)
    {
        INT blackness = BLACKNESS(sample);
        if ((lvl != IR_BLACK) && (blackness >= BLACKNESS(S3_BAR_ADC_ON)))
        {
            lvl = IR_BLACK;
        }
        else if ((IR_BLACK == lvl) && (blackness <= BLACKNESS(S3_BAR_ADC_OFF)))
        {
            lvl = !IR_BLACK;
        }
    }
    else
    {
        INT span      = (g_env_b - g_env_w) >> ENV_SH;
        INT mid       = (g_env_b + g_env_w) >> (ENV_SH + 1);
        INT hyst      = span / 6;
        INT blackness = BLACKNESS(sample);

        if (span < S3_BAR_MIN_SPAN)
        {
            lvl = !IR_BLACK; /* never seen black */
        }
        else if ((lvl != IR_BLACK) && (blackness > (mid + hyst)))
        {
            lvl = IR_BLACK;
        }
        else if ((IR_BLACK == lvl) && (blackness < (mid - hyst)))
        {
            lvl = !IR_BLACK;
        }
    }
    if (lvl != g_adc_lvl)
    {
        UW ring_pos = g_ring_w;

        g_adc_lvl = lvl;
        if (!gb_paused)
        {
            g_ring[ring_pos & (RING - 1)].odo   = motion_odometer();
            g_ring[ring_pos & (RING - 1)].level = lvl;
            g_ring_w                            = ring_pos + 1;
        }
    }
}

LOCAL void adc_fifo_isr (UINT intno)
{
    while (0u == (in_w(ADC_FCS) & FCS_EMPTY))
    {
        adc_sample((INT)(in_w(ADC_FIFO) & 0x0FFFu));
    }
    ClearInt(intno);
}

EXPORT ER barcode_hw_init (void)
{
    T_DINT dint;
    ER     err;

    app_hw_release(RESETS_RESET_ADC);
    /* GP28 as an analogue pin: no digital input, no pulls */
    out_w(GPIO_CTRL(PIN_IR_BARCODE_AO), 31); /* FUNCSEL null */
    clr_w(GPIO(PIN_IR_BARCODE_AO), GPIO_IE | GPIO_PUE | GPIO_PDE);
    set_w(GPIO(PIN_IR_BARCODE_AO), GPIO_OD);

    out_w(ADC_CS, ((UW)S3_BAR_ADC_CH << 12) | CS_EN);
    while (0u == (in_w(ADC_CS) & CS_READY))
    {
        ;
    }
    out_w(ADC_DIV, (UW)((48000000 / ADC_RATE_HZ) - 1)
                       << 8); /* clk_adc 48 MHz */
    out_w(ADC_FCS, FCS_EN | FCS_THRESH(1));
    while (0u == (in_w(ADC_FCS) & FCS_EMPTY))
    {
        (void)in_w(ADC_FIFO);
    }

    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)adc_fifo_isr;
    err         = tk_def_int(INTNO_ADC_FIFO, &dint);
    if (err < E_OK)
    {
        return err;
    }
    ClearInt(INTNO_ADC_FIFO);
    out_w(ADC_INTE, 1u);
    EnableInt(INTNO_ADC_FIFO, INTPRI_ADC);
    set_w(ADC_CS, CS_START_MANY);
    return E_OK;
}
EXPORT BOOL barcode_black (void)
{
    return IR_BLACK == g_adc_lvl;
}
EXPORT BOOL barcode_adc (INT * p_raw, INT * p_white, INT * p_black)
{
    INT white = g_env_w;
    INT black = g_env_b;

    *p_raw   = g_adc_raw;
    *p_white = (white < 0) ? 0 : (white >> ENV_SH);
    *p_black = (black < 0) ? 0 : (black >> ENV_SH);
    if (!S3_BAR_BLACK_HIGH)
    {
        *p_white = 4095 - *p_white;
        *p_black = 4095 - *p_black;
    }
    return TRUE;
}

#endif /* S3_BARCODE_ANALOG */

EXPORT BOOL barcode_busy (void)
{
    return gb_busy;
}
EXPORT UW barcode_edges (void)
{
    return g_ring_w;
}

/* B is over bars now: an edge in the last S3_IN_BARS_MM (widest element
   10.5 mm).  Motion holds the steering straight then: B sits 85 mm left
   of the robot's centre, so any turn changes B's own speed over the bars
   by +-50 % or more against the odometer, and the widths come out wrong. */
EXPORT BOOL barcode_in_bars (void)
{
    W since;

    if (!gb_busy)
    {
        return FALSE;
    }
    since = motion_odometer() - g_edge_pub;
    if (since < 0)
    {
        since = -since;
    }
    return since < (W)(S3_IN_BARS_MM * 1000 / S2_UM_PER_COUNT);
}

/* Debug (Stage 2 tests): the element widths of the last code that ended,
   even one too short to report, so a test can show what the sensor saw */
#define DBG_MAX 40
LOCAL INT         g_dbg_w[DBG_MAX]; /* 0.1 mm */
LOCAL INT         g_dbg_n;
LOCAL volatile UW g_dbg_seq;

EXPORT UW barcode_debug (INT * p_count, INT * p_w_mm10, INT max)
{
    INT  idx;
    UINT irq_state;

    APP_LOCK(irq_state);
    *p_count = g_dbg_n;
    for (idx = 0; (idx < g_dbg_n) && (idx < DBG_MAX) && (idx < max); idx++)
    {
        p_w_mm10[idx] = g_dbg_w[idx];
    }
    APP_UNLOCK(irq_state);
    return g_dbg_seq;
}
/* Single black patches (left branches, grout lines) */
LOCAL W           g_mark_odo;
LOCAL INT         g_mark_w; /* 0.1 mm */
LOCAL volatile UW g_mark_seq;

EXPORT UW barcode_mark (W * p_centre_odo, INT * p_width_mm10)
{
    UINT irq_state;
    UW   seq;

    APP_LOCK(irq_state);
    *p_centre_odo = g_mark_odo;
    *p_width_mm10 = g_mark_w;
    seq           = g_mark_seq;
    APP_UNLOCK(irq_state);
    return seq;
}

/* The first bar of a run has just ended at end_odo, w counts wide.
   Published at once, not after the quiet gap, so a left branch is known
   while there is still room to turn (B sits behind C).  Every first bar
   is published (also a barcode's narrow start bar and grout lines); the
   line task keeps only the ones about one tape wide.  No Code 39 bar is
   that wide (wide bars measured 10.5 mm). */
LOCAL void mark_pub (W end_odo, W width)
{
    UINT irq_state;

    APP_LOCK(irq_state);
    g_mark_odo = end_odo - (width / 2); /* driving forward: centre is behind */
    g_mark_w   = (INT)(width * S2_UM_PER_COUNT / 100);
    g_mark_seq++;
    APP_UNLOCK(irq_state);
}

EXPORT UW barcode_seq (void)
{
    return g_seq_now;
}

/* Forget any half-read code (start of a run, after a manoeuvre).  On 7 Nov
   tape held under B at the end of the live view left a code open, and
   the run started with a rescan.  Task context; blocks 30 ms. */
EXPORT void barcode_reset (void)
{
    gb_paused = TRUE;
    tk_dly_tsk(30); /* the task drops it within 10 ms */
    gb_paused = FALSE;
}

EXPORT void barcode_last (barcode_result_t * p_result)
{
    UINT irq_state;

    APP_LOCK(irq_state);
    *p_result = g_last;
    APP_UNLOCK(irq_state);
}

/* One character from 9 widths; 0 = no match */
LOCAL UB decode_char (const W * p_elem, W * p_narrow_sum, W * p_wide_sum)
{
    INT idx;
    INT jdx;
    INT wide[3] = {-1, -1, -1};
    UH  pat     = 0;
    W   min_wide;
    W   max_narrow = 0;

    for (jdx = 0; jdx < 3; jdx++)
    {
        INT best = -1;
        for (idx = 0; idx < 9; idx++)
        {
            if ((idx == wide[0]) || (idx == wide[1]))
            {
                continue;
            }
            if ((best < 0) || (p_elem[idx] > p_elem[best]))
            {
                best = idx;
            }
        }
        wide[jdx] = best;
        pat |= (UH)(1u << (8 - best));
    }
    min_wide = p_elem[wide[2]];
    for (idx = 0; idx < 9; idx++)
    {
        if ((pat & (1u << (8 - idx))) != 0u)
        {
            *p_wide_sum += p_elem[idx];
        }
        else
        {
            *p_narrow_sum += p_elem[idx];
            if (p_elem[idx] > max_narrow)
            {
                max_narrow = p_elem[idx];
            }
        }
    }
    if ((min_wide * 10) < (max_narrow * S3_WIDE_RATIO_X10))
    {
        return 0; /* wide and narrow not separable */
    }
    for (idx = 0; idx < (INT)(sizeof(g_c39) / sizeof(g_c39[0])); idx++)
    {
        if (g_c39[idx].pat == pat)
        {
            return g_c39[idx].letter;
        }
    }
    return 0;
}

/* Decode n elements; fills r.  TRUE = valid "*...*".  The code may start
   at any bar (off even) and end before the last element: a stray mark
   within S3_QUIET_MM of the code is part of the same run. */
LOCAL BOOL decode (W * p_elem, INT count, barcode_result_t * p_result)
{
    INT kdx;
    INT pass;
    INT idx;
    INT off;
    W   narrow_sum = 0;
    W   wide_sum   = 0;

    p_result->b_ok     = FALSE;
    p_result->elements = count;
    if (count < 29)
    {
        return FALSE;
    }
    for (pass = 0; pass < 2; pass++)
    {
        for (off = 0; (off + 29) <= count; off += 2)
        {
            narrow_sum = wide_sum = 0;
            if (decode_char(&p_elem[off], &narrow_sum, &wide_sum) != '*')
            {
                continue;
            }
            for (kdx = (count - off + 1) / 10; kdx >= 3; kdx--)
            {
                UB letter;

                narrow_sum = wide_sum = 0;
                (void)decode_char(&p_elem[off], &narrow_sum, &wide_sum);
                if (decode_char(&p_elem[off + ((kdx - 1) * 10)], &narrow_sum,
                                &wide_sum) != '*')
                {
                    continue;
                }
                letter = decode_char(&p_elem[off + 10], &narrow_sum, &wide_sum);
                if ((letter != 0) && (letter != '*'))
                {
                    p_result->b_ok       = TRUE;
                    p_result->letter     = letter;
                    p_result->b_reversed = (1 == pass);
                    /* 3 chars of 6 narrow + 3 wide counted above */
                    p_result->narrow_mm10 =
                        (INT)(narrow_sum * S2_UM_PER_COUNT / 100 / 18);
                    p_result->wide_mm10 =
                        (INT)(wide_sum * S2_UM_PER_COUNT / 100 / 9);
                    return TRUE;
                }
            }
        }
        /* try backwards */
        for (idx = 0; idx < (count / 2); idx++)
        {
            W tmp                   = p_elem[idx];
            p_elem[idx]             = p_elem[count - 1 - idx];
            p_elem[count - 1 - idx] = tmp;
        }
    }
    return FALSE;
}

/* ---- Forgiving decode (8 Nov) -------------------------------------------
   Two ways a real read failed (logs 7-8 Nov):
   - one '*' damaged (the robot started on the first bars: 7.2 5.6 8.0
     mm where 3.4 10 3.4 were): accepted if the other '*' and the data
     character decode with a clear wide/narrow split (wide >= 2 x narrow),
     the two gaps between characters are narrow, and the damaged block is
     as long as a character (+-15 %);
   - a narrow element lost (blur: two bars or two spaces merged, 27
     elements): each element is tried as wide-or-narrow + narrow +
     wide-or-narrow (its width +-20 %), and the result must decode
     cleanly, with both '*'.
   Either is used only if exactly one character comes out.  Tested on the
   logs' reads (Python copy of this decoder): every narrow element of 9
   good reads lost in turn: 120 recovered, 0 wrong, 39 not read; start or
   stop replaced by noise: 1059 right, 0 wrong; 200 000 random bar
   patterns: 3 decoded as something (the clean decoder 0); of 16 failed
   real reads, 2 now decode (both 'A', the "A small" code used then). */
LOCAL void sort_w (W * p_values, INT count)
{
    INT idx;
    INT jdx;
    W   tmp;

    for (idx = 1; idx < count; idx++)
    { /* insertion sort, n <= 32 */
        tmp = p_values[idx];
        for (jdx = idx; (jdx > 0) && (p_values[jdx - 1] > tmp); jdx--)
        {
            p_values[jdx] = p_values[jdx - 1];
        }
        p_values[jdx] = tmp;
    }
}

/* the 3 widest of 9 at least twice the 6 narrow */
LOCAL BOOL clear_split (const W * p_elem)
{
    W   sorted[9];
    INT idx;

    for (idx = 0; idx < 9; idx++)
    {
        sorted[idx] = p_elem[idx];
    }
    sort_w(sorted, 9);
    return sorted[6] >= (2 * sorted[5]);
}

LOCAL W sum9 (const W * p_elem)
{
    W   sum = 0;
    INT idx;

    for (idx = 0; idx < 9; idx++)
    {
        sum += p_elem[idx];
    }
    return sum;
}

/* typical narrow and wide width of n elements (n <= 32): the median of
   the narrowest two thirds and of the widest quarter */
LOCAL void nar_wid (const W * p_elem, INT count, W * p_nar, W * p_wid)
{
    W   sorted[32];
    INT idx;

    if (count > 32)
    {
        count = 32;
    }
    for (idx = 0; idx < count; idx++)
    {
        sorted[idx] = p_elem[idx];
    }
    sort_w(sorted, count);
    *p_nar = sorted[(count * 2 / 3) / 2];
    *p_wid = sorted[count - 1 - ((count / 4) / 2)];
}

LOCAL BOOL decode_one_star (W * p_elem, INT count, barcode_result_t * p_result)
{
    INT  pass;
    INT  off;
    INT  idx;
    UB   found       = 0;
    BOOL b_rev_found = FALSE;
    W    f_nar       = 0;
    W    f_wid       = 0;

    if (count < 29)
    {
        return FALSE;
    }
    for (pass = 0; pass < 2; pass++)
    {
        for (off = 0; (off + 29) <= count; off += 2)
        {
            W    narrow_sum = 0;
            W    wide_sum   = 0;
            W    nar;
            W    wid;
            W    sum_mid;
            W    sum_other;
            W    sorted[29];
            UB   first_ch;
            UB   letter;
            UB   last_ch;
            BOOL b_a_ok;
            INT  kdx;

            first_ch = decode_char(&p_elem[off], &narrow_sum, &wide_sum);
            letter   = decode_char(&p_elem[off + 10], &narrow_sum, &wide_sum);
            last_ch  = decode_char(&p_elem[off + 20], &narrow_sum, &wide_sum);
            if ((0 == letter) || ('*' == letter) ||
                (('*' == first_ch) == ('*' == last_ch)))
            {
                continue;
            }
            b_a_ok = ('*' == first_ch);
            for (kdx = 0; kdx < 29; kdx++)
            {
                sorted[kdx] = p_elem[off + kdx];
            }
            sort_w(sorted, 29); /* m[14]: the median, a narrow one */
            if (((p_elem[off + 9] * 10) >= (sorted[14] * 16)) ||
                ((p_elem[off + 19] * 10) >= (sorted[14] * 16)))
            {
                continue; /* a gap between characters is not narrow */
            }
            if ((!clear_split(&p_elem[off + 10])) ||
                (!clear_split(&p_elem[b_a_ok ? off : off + 20])))
            {
                continue;
            }
            sum_mid   = sum9(&p_elem[off + 10]);
            sum_other = sum9(&p_elem[b_a_ok ? off + 20 : off]);
            if ((((sum_other > sum_mid) ? sum_other - sum_mid
                                        : sum_mid - sum_other) *
                 100) > (sum_mid * 15))
            {
                continue;
            }
            if ((found != 0) && (found != letter))
            {
                found = 0xFF; /* two answers */
            }
            else
            {
                found = letter;
            }
            b_rev_found = (1 == pass);
            nar_wid(&p_elem[off], 29, &nar, &wid);
            f_nar = nar;
            f_wid = wid;
        }
        for (idx = 0; idx < (count / 2); idx++)
        { /* backwards */
            W tmp = p_elem[idx];

            p_elem[idx]             = p_elem[count - 1 - idx];
            p_elem[count - 1 - idx] = tmp;
        }
    }
    if ((0 == found) || (0xFF == found))
    {
        return FALSE;
    }
    p_result->b_ok        = TRUE;
    p_result->letter      = found;
    p_result->b_reversed  = b_rev_found;
    p_result->fixed       = BAR_FIX_STAR;
    p_result->narrow_mm10 = (INT)(f_nar * S2_UM_PER_COUNT / 100);
    p_result->wide_mm10   = (INT)(f_wid * S2_UM_PER_COUNT / 100);
    return TRUE;
}

LOCAL BOOL decode_merged (const W * p_elem, INT count,
                          barcode_result_t * p_result)
{
    W                trial_elems[31];
    W                nar;
    W                wid;
    W                tot;
    W                opt[2];
    INT              idx;
    INT              first_opt;
    INT              last_opt;
    INT              jdx;
    UB               found = 0;
    barcode_result_t trial;
    barcode_result_t keep;

    if ((count != 27) && (count != 28))
    {
        return FALSE;
    }
    keep = *p_result;
    nar_wid(p_elem, count, &nar, &wid);
    opt[0] = nar;
    opt[1] = wid;
    for (idx = 0; idx < count; idx++)
    {
        for (first_opt = 0; first_opt < 2; first_opt++)
        {
            for (last_opt = 0; last_opt < 2; last_opt++)
            {
                tot = opt[first_opt] + nar + opt[last_opt];
                if ((((p_elem[idx] > tot) ? p_elem[idx] - tot
                                          : tot - p_elem[idx]) *
                     5) > tot)
                {
                    continue;
                }
                for (jdx = 0; jdx < idx; jdx++)
                {
                    trial_elems[jdx] = p_elem[jdx];
                }
                trial_elems[idx]     = opt[first_opt];
                trial_elems[idx + 1] = nar;
                trial_elems[idx + 2] = opt[last_opt];
                for (jdx = idx + 1; jdx < count; jdx++)
                {
                    trial_elems[jdx + 2] = p_elem[jdx];
                }
                trial = *p_result;
                if (decode(trial_elems, count + 2, &trial))
                {
                    if ((found != 0) && (found != trial.letter))
                    {
                        return FALSE; /* two answers */
                    }
                    found = trial.letter;
                    keep  = trial;
                }
            }
        }
    }
    if (0 == found)
    {
        return FALSE;
    }
    keep.elements = count;
    keep.fixed    = BAR_FIX_MERGED;
    *p_result     = keep;
    return TRUE;
}

/* Cut short (8 Nov): F's last narrow bar is printed weakly (darkest 1839
   against ~2900 for the others) and was not seen, with the space before
   it: 27 elements, the first 27 of a good F.  27-28 elements: add one or
   two narrow elements at the end, or at the start, and decode cleanly.
   Logs' good reads with 1-2 elements cut off either end: 33 right, 0
   wrong, 11 not read; 50 000 random 27-28 element patterns: 1 decoded. */
LOCAL BOOL decode_truncated (const W * p_elem, INT count,
                             barcode_result_t * p_result)
{
    W                trial_elems[31];
    W                nar;
    W                wid;
    INT              kdx;
    INT              side;
    INT              jdx;
    UB               found = 0;
    barcode_result_t trial;
    barcode_result_t keep;

    if ((count != 27) && (count != 28))
    {
        return FALSE;
    }
    keep = *p_result;
    nar_wid(p_elem, count, &nar, &wid);
    for (kdx = 1; kdx <= 2; kdx++)
    {
        if ((count + kdx) < 29)
        {
            continue;
        }
        for (side = 0; side < 2; side++)
        {
            for (jdx = 0; jdx < (count + kdx); jdx++)
            {
                trial_elems[jdx] = nar;
            }
            for (jdx = 0; jdx < count; jdx++)
            {
                trial_elems[(0 == side) ? jdx : jdx + kdx] = p_elem[jdx];
            }
            trial = *p_result;
            if (decode(trial_elems, count + kdx, &trial))
            {
                if ((found != 0) && (found != trial.letter))
                {
                    return FALSE; /* two answers */
                }
                found = trial.letter;
                keep  = trial;
            }
        }
    }
    if (0 == found)
    {
        return FALSE;
    }
    keep.elements = count;
    keep.fixed    = BAR_FIX_CUT;
    *p_result     = keep;
    return TRUE;
}

EXPORT void barcode_task (INT stacd, void * p_exinf)
{
    W    elem[MAX_ELEM];
    INT  count      = 0;
    W    prev_odo   = 0;
    BOOL b_in_code  = FALSE;
    BOOL b_too_long = FALSE;
    UW   seq        = 0;
    UW   end;
    UB   stable   = !IR_BLACK; /* debounced level */
    UB   raw_lvl  = !IR_BLACK; /* level after the last raw edge */
    W    raw_odo  = 0;         /* where that raw edge was */
    BOOL b_raw_ok = FALSE;

    /* A level counts only once it has held for MERGE_COUNTS of travel:
       then it starts where its last raw edge was.  Flicker never holds,
       so it neither makes elements nor keeps a code open (Stage 2, 2 Nov:
       pair-cancelling let every other flicker edge through). */
#define COMMIT(lvl, pos)                 \
    do                                   \
    {                                    \
        if ((lvl) != stable)             \
        {                                \
            stable = (lvl);              \
            if (!b_in_code)              \
            {                            \
                if ((lvl) == IR_BLACK)   \
                {                        \
                    b_in_code  = TRUE;   \
                    b_too_long = FALSE;  \
                    count      = 0;      \
                    prev_odo   = (pos);  \
                    g_edge_pub = (pos);  \
                    gb_busy    = TRUE;   \
                }                        \
            }                            \
            else                         \
            {                            \
                W w_ = (pos) - prev_odo; \
                if (w_ < 0)              \
                    w_ = -w_;            \
                if (count < MAX_ELEM)    \
                    elem[count++] = w_;  \
                else                     \
                    b_too_long = TRUE;   \
                if (count == 1)          \
                    mark_pub((pos), w_); \
                prev_odo   = (pos);      \
                g_edge_pub = (pos);      \
            }                            \
        }                                \
    } while (0)

    (void)stacd;
    (void)p_exinf;
    g_ring_r = g_ring_w;

    for (;;)
    {
        (void)tk_slp_tsk(TIMEOUT_BARCODE_MS);
        app_timing_wake(g_tmg_barcode, 0);

        /* barcode_reset(): drop this read and every edge until it
           ends, then start fresh */
        if (gb_paused)
        {
            g_ring_r  = g_ring_w;
            b_in_code = FALSE;
            count     = 0;
            gb_busy   = FALSE;
            stable    = !IR_BLACK;
            b_raw_ok  = FALSE;
            app_timing_done(g_tmg_barcode);
            continue;
        }

        /* Drain the edges that are here now, never more than the ring
           holds: a chattering sensor keeps adding edges, and an
           unbounded loop starved every task below this one (530 ms,
           Stage 2 run, 1 Nov).  If the ring overflowed, edges were
           lost: drop the barcode being read. */
        {
            UW avail = g_ring_w - g_ring_r;

            if (avail > RING)
            {
                g_ring_r  = g_ring_w - RING;
                b_in_code = FALSE;
                count     = 0;
                stable    = !IR_BLACK;
                b_raw_ok  = FALSE;
                avail     = RING;
            }
            end = g_ring_r + avail;
        }
        while (g_ring_r != end)
        {
            bar_edge_t edge = g_ring[g_ring_r & (RING - 1)];
            W          delta;

            g_ring_r++;
            if (b_raw_ok)
            {
                delta = edge.odo - raw_odo;
                if (delta < 0)
                {
                    delta = -delta;
                }
                if (delta >= MERGE_COUNTS)
                {
                    COMMIT(raw_lvl, raw_odo); /* that level held */
                }
            }
            raw_lvl  = edge.level;
            raw_odo  = edge.odo;
            b_raw_ok = TRUE;
        }
        /* The latest level may have held long enough by now */
        if (b_raw_ok)
        {
            W delta = motion_odometer() - raw_odo;

            if (delta < 0)
            {
                delta = -delta;
            }
            if (delta >= MERGE_COUNTS)
            {
                COMMIT(raw_lvl, raw_odo);
            }
        }

        /* End of a barcode: quiet for S3_QUIET_MM */
        if (b_in_code)
        {
            W gap = motion_odometer() - prev_odo;

            if (gap < 0)
            {
                gap = -gap;
            }
            if (S2_COUNTS_TO_MM(gap) > S3_QUIET_MM)
            {
                b_in_code = FALSE;
                {
                    INT  idx;
                    UINT irq_state;

                    APP_LOCK(irq_state);
                    g_dbg_n = count;
                    for (idx = 0; (idx < count) && (idx < DBG_MAX); idx++)
                    {
                        g_dbg_w[idx] = (INT)(elem[idx] * S2_UM_PER_COUNT / 100);
                    }
                    g_dbg_seq++;
                    APP_UNLOCK(irq_state);
                }
                if (count >= MIN_ELEM_REPORT)
                {
                    barcode_result_t result = {0};
                    UINT             irq_state;

                    if ((!b_too_long) && (!decode(elem, count, &result)) &&
                        (!decode_one_star(elem, count, &result)) &&
                        (!decode_merged(elem, count, &result)))
                    {
                        (void)decode_truncated(elem, count, &result);
                    }
                    result.seq      = ++seq;
                    result.elements = count;
                    APP_LOCK(irq_state);
                    g_last = result;
                    APP_UNLOCK(irq_state);
                    g_seq_now = seq;
                    tk_set_flg(gh_flg_mission, MSN_FLG_BARCODE);
                }
                count = 0;
            }
        }
        gb_busy = b_in_code;

        app_timing_done(g_tmg_barcode);
    }
}

/* ---- Mapping (B7, B8) ---------------------------------------------- */

LOCAL BOOL is_code_char (UB letter)
{
    return ((letter >= 'A') && (letter <= 'Z')) ||
           ((letter >= '0') && (letter <= '9'));
}

EXPORT BOOL barcode_set_mapping (const UB * p_text, INT len, UB * p_why,
                                 INT why_max)
{
    UB   letters[4];
    UB   moves[4];
    INT  pairs = 0;
    INT  idx   = 0;
    INT  first;
    INT  second;
    UINT irq_state;

    p_why[0] = '\0';
    while (idx < len)
    {
        UB letter;
        UB move_ch;

        while ((idx < len) && ((' ' == p_text[idx]) || (',' == p_text[idx])))
        {
            idx++;
        }
        if (idx >= len)
        {
            break;
        }
        if ((4 == pairs) || ((idx + 2) >= len) || (p_text[idx + 1] != '='))
        {
            tm_sprintf(p_why,
                       (const UB *)"expected 4 pairs like K=L,Q=R,X=F,M=U");
            return FALSE;
        }
        letter  = p_text[idx];
        move_ch = p_text[idx + 2];
        if ((letter >= 'a') && (letter <= 'z'))
        {
            letter = (UB)(letter - 'a' + 'A');
        }
        if ((move_ch >= 'a') && (move_ch <= 'z'))
        {
            move_ch = (UB)(move_ch - 'a' + 'A');
        }
        if ((!is_code_char(letter)) ||
            ((move_ch != MOVE_LEFT) && (move_ch != MOVE_RIGHT) &&
             (move_ch != MOVE_FORWARD) && (move_ch != MOVE_UTURN)))
        {
            tm_sprintf(p_why,
                       (const UB *)"pair %d: '%c=%c' not A-Z/0-9 = L/R/F/U",
                       pairs + 1, letter, move_ch);
            return FALSE;
        }
        letters[pairs] = letter;
        moves[pairs]   = move_ch;
        pairs++;
        idx += 3;
    }
    if (pairs != 4)
    {
        tm_sprintf(p_why, (const UB *)"%d pairs, need exactly 4", pairs);
        return FALSE;
    }
    for (first = 0; first < 4; first++)
    {
        for (second = first + 1; second < 4; second++)
        {
            if ((letters[first] == letters[second]) ||
                (moves[first] == moves[second]))
            {
                tm_sprintf(
                    p_why,
                    (const UB *)"each character and each move only once");
                return FALSE;
            }
        }
    }
    (void)why_max;
    APP_LOCK(irq_state);
    for (first = 0; first < 4; first++)
    {
        g_map_ch[first] = letters[first];
        g_map_mv[first] = moves[first];
    }
    gb_map_set = TRUE;
    APP_UNLOCK(irq_state);
    return TRUE;
}

EXPORT INT barcode_move_for (UB letter)
{
    INT  idx;
    INT  move = MOVE_NONE;
    UINT irq_state;

    APP_LOCK(irq_state);
    if (gb_map_set)
    {
        for (idx = 0; idx < 4; idx++)
        {
            if (g_map_ch[idx] == letter)
            {
                move = g_map_mv[idx];
            }
        }
    }
    APP_UNLOCK(irq_state);
    return move;
}

EXPORT INT barcode_mapping_text (UB * p_out, INT max)
{
    UB   letters[4];
    UB   moves[4];
    BOOL b_set;
    UINT irq_state;
    INT  idx;

    APP_LOCK(irq_state);
    b_set = gb_map_set;
    for (idx = 0; idx < 4; idx++)
    {
        letters[idx] = g_map_ch[idx];
        moves[idx]   = g_map_mv[idx];
    }
    APP_UNLOCK(irq_state);
    if ((!b_set) || (max < 16))
    {
        p_out[0] = '\0';
        return 0;
    }
    return tm_sprintf(p_out, (const UB *)"%c=%c,%c=%c,%c=%c,%c=%c", letters[0],
                      moves[0], letters[1], moves[1], letters[2], moves[2],
                      letters[3], moves[3]);
}

/*** end of file ***/

