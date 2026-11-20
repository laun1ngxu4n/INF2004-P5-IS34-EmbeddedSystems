/**
 * @file    barcode.c
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
#include "../app_math.h" /* app_clamp() */
#include "barcode.h"

#define RING        128 /* edges kept, a power of 2 */
#define MAX_ELEM    99  /* up to 10 characters */
#define S3_QUIET_MM 40  /* no edge for this far = end of barcode */
/* < 1.2 mm apart = sensor chatter. The sensor flickers at its threshold
 * (thousands of edges, Stage 2 2 Nov); must stay under half the narrowest
 * bar: 3.5 mm small codes = 44 counts */
#define MERGE_COUNTS    15
#define MIN_ELEM_REPORT 20 /* fewer elements = noise, not reported */
#define SHORT_RUN_PRINT 5  /* ... but printed from this many (15 Nov) */

/* Code 39 */
#define C39_CHAR_ELEMS 9  /* bars and spaces in a character */
#define C39_WIDE_ELEMS 3  /* the wide ones among them */
#define C39_CHAR_PITCH 10 /* a character and the gap after it */
#define C39_MIN_CHARS  3  /* start, one letter, stop */
#define C39_MIN_ELEMS  29 /* their elements: 3 x 10 - 1 */
#define DECODE_PASSES  2  /* as read, then reversed */
#define WIDTH_OPTS     2  /* narrow or wide */
#define CUT_MAX        2  /* elements cut off at most */
#define CUT_SIDES      2  /* at the start or at the end */
/* decode_cut_star (15 Nov): the first elements of a read lost, more than
   decode_truncated puts back.  At most 4 lost, so at least 5 of the cut
   '*' are left to check (5 lost: 1 false read in 10 000 random patterns,
   4: 1 in 100 000). */
#define CUT_STAR_MIN 3
#define CUT_STAR_MAX 4
#define STAR_PAT     0x094u /* '*' in g_c39 */
#define ELEM_KINDS   2      /* bar (even index), space (odd) */
#define BAR_MOVES    4      /* L, R, F, U: one letter each */
/* the 3 widest of 9 at least 1.3 x the rest. Was 1.5: blur widens a narrow
 * element beside a wide one (4.6 mm against 7.0 wide, simulated "A small"),
 * and 1.5 lost 4 reads in 18 at spot sigma 1.6 mm; 1.3 none. Random widths:
 * 0 of 20000 decode either way. */
#define S3_WIDE_RATIO_X10 13

/* One edge in the ring (Change Requirement 1, Tribunal 1): the interrupt
   keeps only the low 16 bits of the odometer; the task widens it back
   (odo_from16), exact while the edge is less than 32768 counts (2.6 m)
   behind the odometer: the ring is emptied every 10 ms. */
typedef struct
{
    uint32_t odo16 APP_FIELD(16); /* odometer counts, low 16 bits */
    uint32_t level APP_FIELD(1);  /* IR_BLACK or not */
} bar_edge_t;

LOCAL bar_edge_t        g_ring[RING];
LOCAL volatile uint32_t g_ring_w; /* written by the interrupt */
LOCAL uint32_t          g_ring_r; /* read by the task */

LOCAL volatile bool     gb_busy;    /* inside a barcode (in_code) */
LOCAL volatile bool     gb_paused;  /* barcode_reset(): ignore edges */
LOCAL volatile uint32_t g_seq_now;  /* results published so far */
LOCAL volatile int32_t  g_edge_pub; /* odometer at the last clean edge */
/* barcode_ignore_mm(): from this odometer reading, for this many counts
   (0 = not ignoring).  Mission writes both under APP_LOCK, the task reads
   them under it. */
LOCAL int32_t g_ignore_odo;
LOCAL int32_t g_ignore_counts;

/* Edge this recent = B over bars: Motion holds the steering straight.
   Was 15 mm; 15 Nov course runs: one missed thin bar (they read 1.5-2.3
   mm) plus an 11 mm wide space passed 15 mm, the hold let go inside the
   code, the robot steered +-30 over the bars and swung B across them, and
   the code broke into pieces.  Now the whole code: until it ends. */
#define S3_IN_BARS_MM S3_QUIET_MM

/* One pair of the mapping: a letter (7-bit ASCII) and its move as an
   index into MOVE_CODES (2 bits) */
/* (uint16_t fields: the 9 bits fit a 2-byte unit; uint32_t would take 4) */
typedef struct
{
    uint16_t letter APP_FIELD(7);
    uint16_t move   APP_FIELD(2);
} bar_pair_t;

#define MOVE_CODES "LRFU" /* move index -> MOVE_... */

LOCAL barcode_result_t g_last;
LOCAL bar_pair_t       g_map[BAR_MOVES]; /* under APP_LOCK */
LOCAL bool             gb_map_set;

/** An edge's 16-bit odometer back to the full count (see bar_edge_t). */
LOCAL int32_t odo_from16 (uint32_t odo16)
{
    uint32_t now  = (uint32_t)motor_odometer(); /* the same bits, mod 2^32 */
    uint32_t back = (now - odo16) & 0xFFFFu;    /* how far back, mod 2^16 */

    /* back < 32768 counts (see bar_edge_t): now - back is the edge's count,
       mod 2^32; as int32_t it is the odometer value again */
    return (int32_t)(now - back);
}

/** A width in 0.1 mm into its 9-bit result field (saturates at 51.1 mm). */
Inline uint32_t width_field (int32_t mm10)
{
    /* 0..511 when it is cast: fits uint32_t */
    return (mm10 < 0) ? 0u : ((mm10 > 511) ? 511u : (uint32_t)mm10);
}

/* Code 39: 9 elements bar/space/bar..., 1 = wide.  Bit 8 = first bar. */
typedef struct
{
    uint8_t  letter;
    uint16_t pat;
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

#if !S3_BARCODE_ANALOG

/** GP2 edge: interrupt context */
LOCAL void barcode_edge_isr (uint32_t pin, uint32_t events)
{
    uint32_t   ring_pos = g_ring_w;
    bar_edge_t edge;

    (void)events;
    /* the low 16 bits of the count (see bar_edge_t) */
    edge.odo16                    = (uint32_t)motor_odometer() & 0xFFFFu;
    edge.level                    = app_gpio_get(pin); /* 0 or 1 */
    g_ring[ring_pos & (RING - 1)] = edge;
    g_ring_w                      = ring_pos + 1;
}

/** Sets up the barcode sensor input and its interrupt. */
EXPORT ER barcode_hw_init (void)
{
    ER err = app_gpio_input(PIN_IR_BARCODE, false);

    return (err < E_OK) ? err
                        : app_gpio_irq_add(PIN_IR_BARCODE, APP_GPIO_BOTH,
                                           barcode_edge_isr);
}
/** true while the barcode sensor sees black. */
EXPORT bool barcode_black (void)
{
    return IR_BLACK == app_gpio_get(PIN_IR_BARCODE);
}
EXPORT bool barcode_adc (int32_t * p_raw, int32_t * p_white, int32_t * p_black)
{
    *p_raw = *p_white = *p_black = 0;
    return false;
}

#else /* S3_BARCODE_ANALOG */

/* AO on GP28, sampled by the ADC at 2 kHz (0.075 mm apart at 150 mm/s).
   The interrupt turns the samples into black/white with hysteresis and
   stores each change exactly like a GP2 edge, so the decoder below is the
   same for both inputs.  The kernel's sample ADC driver ("adca") is not
   used by the application; its interrupt handler is replaced here. */
#define ADC_BASE       0x4004C000u
#define ADC_CS         (ADC_BASE + 0x00u)
#define ADC_FCS        (ADC_BASE + 0x08u)
#define ADC_FIFO       (ADC_BASE + 0x0Cu)
#define ADC_DIV        (ADC_BASE + 0x10u)
#define ADC_INTE       (ADC_BASE + 0x18u)
#define CS_EN          (1u << 0)
#define CS_START_MANY  (1u << 3)
#define CS_READY       (1u << 8)
#define FCS_EN         (1u << 0)
#define FCS_EMPTY      (1u << 8)
#define FCS_THRESH_1   (1u << 24) /* FIFO interrupt at 1 sample */
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

LOCAL volatile uint16_t g_adc_raw; /* 12-bit ADC, its own halfword:
                                       written by the interrupt alone */
LOCAL int32_t          g_env_w = -1;
LOCAL int32_t          g_env_b = -1; /* white / black level x16 */
LOCAL int32_t          g_rel_lo;
LOCAL int32_t          g_rel_hi; /* S3_BAR_RELATIVE: extremes so far */
LOCAL volatile uint8_t g_adc_lvl = !IR_BLACK;

/** bigger = blacker, whichever way the module is wired */
Inline int32_t adc_blackness (int32_t sample)
{
    return S3_BAR_BLACK_HIGH ? sample : (4095 - sample);
}

/** S3_BAR_RELATIVE: black once the reading rises S3_BAR_DELTA above the
   lowest since the last bar (and is at least S3_BAR_BLACK_MIN), white
   once it falls S3_BAR_DELTA below the highest since.  The level after
   this sample. */
LOCAL uint8_t relative_level (uint8_t lvl, int32_t blackness)
{
    uint8_t next = lvl;

    if (lvl != IR_BLACK)
    {
        if (blackness < g_rel_lo)
        {
            g_rel_lo = blackness;
        }

        if ((blackness > (g_rel_lo + S3_BAR_DELTA)) &&
            (blackness >= adc_blackness(S3_BAR_BLACK_MIN)))
        {
            next     = IR_BLACK;
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
            next     = !IR_BLACK;
            g_rel_lo = blackness;
        }
    }

    return next;
}

/** One barcode sensor sample: learned levels, relative edges, edge to the
 * ring. */
LOCAL void adc_sample (int32_t sample)
{
    int32_t black = adc_blackness(sample) << ENV_SH;
    uint8_t lvl   = g_adc_lvl;

    g_adc_raw = (uint16_t)sample; /* 12 bits (adc_fifo_isr masks it) */

    if (g_env_w < 0)
    {
        g_env_w = g_env_b = black;
        g_rel_lo = g_rel_hi = adc_blackness(sample);
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
        lvl = relative_level(lvl, adc_blackness(sample));
    }
    else if (S3_BAR_ADC_ON > 0)
    {
        int32_t blackness = adc_blackness(sample);

        if ((lvl != IR_BLACK) && (blackness >= adc_blackness(S3_BAR_ADC_ON)))
        {
            lvl = IR_BLACK;
        }
        else if ((IR_BLACK == lvl) &&
                 (blackness <= adc_blackness(S3_BAR_ADC_OFF)))
        {
            lvl = !IR_BLACK;
        }
        else
        {
            /* nothing to do for any other case */
        }
    }
    else
    {
        int32_t span      = (g_env_b - g_env_w) >> ENV_SH;
        int32_t mid       = (g_env_b + g_env_w) >> (ENV_SH + 1);
        int32_t hyst      = span / 6;
        int32_t blackness = adc_blackness(sample);

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
        else
        {
            /* nothing to do for any other case */
        }
    }

    if (lvl != g_adc_lvl)
    {
        uint32_t ring_pos = g_ring_w;

        g_adc_lvl = lvl;

        if (!gb_paused)
        {
            bar_edge_t edge;

            /* the low 16 bits of the count (see bar_edge_t) */
            edge.odo16 = (uint32_t)motor_odometer() & 0xFFFFu;
            edge.level = lvl;
            g_ring[ring_pos & (RING - 1)] = edge;
            g_ring_w                      = ring_pos + 1;
        }
    }
}

/** ADC FIFO interrupt: turns each barcode sensor sample into black / white
 * edges. */
LOCAL void adc_fifo_isr (uint32_t intno)
{
    while (0u == (in_w(ADC_FCS) & FCS_EMPTY))
    {
        adc_sample((int32_t)(in_w(ADC_FIFO) & 0x0FFFu)); /* 12 bits */
    }

    ClearInt(intno);
}

/** Sets up the barcode sensor input and its interrupt. */
EXPORT ER barcode_hw_init (void)
{
    T_DINT dint;
    ER     err;

    app_gpio_hw_release(RESETS_RESET_ADC);
    /* GP28 as an analogue pin: no digital input, no pulls */
    out_w(GPIO_CTRL(PIN_IR_BARCODE_AO), 31); /* FUNCSEL null */
    clr_w(GPIO(PIN_IR_BARCODE_AO), GPIO_IE | GPIO_PUE | GPIO_PDE);
    set_w(GPIO(PIN_IR_BARCODE_AO), GPIO_OD);

    /* channel 0..4 */
    out_w(ADC_CS, ((uint32_t)S3_BAR_ADC_CH << 12) | CS_EN);

    while (0u == (in_w(ADC_CS) & CS_READY))
    {
        /* nothing: wait for the ADC to be ready */
    }

    /* a positive constant, 23999 */
    out_w(ADC_DIV, (uint32_t)((48000000 / ADC_RATE_HZ) - 1)
                       << 8); /* clk_adc 48 MHz */
    out_w(ADC_FCS, FCS_EN | FCS_THRESH_1);

    while (0u == (in_w(ADC_FCS) & FCS_EMPTY))
    {
        (void)in_w(ADC_FIFO);
    }

    dint.intatr = TA_HLNG;
    /* T-Kernel keeps every handler as FP; with TA_HLNG it calls it with the
       interrupt number, adc_fifo_isr's one parameter */
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
/** true while the barcode sensor sees black. */
EXPORT bool barcode_black (void)
{
    return IR_BLACK == g_adc_lvl;
}
EXPORT bool barcode_adc (int32_t * p_raw, int32_t * p_white, int32_t * p_black)
{
    int32_t white = g_env_w;
    int32_t black = g_env_b;

    *p_raw   = g_adc_raw;
    *p_white = (white < 0) ? 0 : (white >> ENV_SH);
    *p_black = (black < 0) ? 0 : (black >> ENV_SH);
    if (!S3_BAR_BLACK_HIGH)
    {
        *p_white = 4095 - *p_white;
        *p_black = 4095 - *p_black;
    }

    return true;
}

#endif /* S3_BARCODE_ANALOG */

/** true while the barcode sensor is inside a barcode (Motion slows down). */
EXPORT bool barcode_busy (void)
{
    return gb_busy;
}
/** Barcode sensor edges since boot. */
EXPORT uint32_t barcode_edges (void)
{
    return g_ring_w;
}

/** B is over bars now: an edge in the last S3_IN_BARS_MM (the code's
   end gap, S3_QUIET_MM).  Motion holds the steering straight then: B sits 85 mm
   left of the robot's centre, so any turn changes B's own speed over the bars
   by +-50 % or more against the odometer, and the widths come out wrong. */
EXPORT bool barcode_in_bars (void)
{
    int32_t since;

    if (!gb_busy)
    {
        return false;
    }

    since = motor_odometer() - g_edge_pub;

    if (since < 0)
    {
        since = -since;
    }

    return since < (S3_IN_BARS_MM * 1000 / S2_UM_PER_COUNT);
}

/* Debug (Stage 2 tests): the element widths of the last code that ended,
   even one too short to report, so a test can show what the sensor saw */
/* (Change Requirement 1: an element's width in 0.1 mm, 16 bits each,
   0..6553.5 mm, clamped; the count is <= MAX_ELEM, 8 bits.  In every image:
   Mission prints them after a READ_FAIL.) */
LOCAL uint16_t          g_dbg_w[BARCODE_DBG_MAX]; /* 0.1 mm */
LOCAL uint8_t           g_dbg_n;
LOCAL volatile uint32_t g_dbg_seq;

EXPORT uint32_t barcode_debug (int32_t * p_count, int32_t * p_w_mm10,
                               int32_t max)
{
    int32_t  idx;
    uint32_t irq_state;

    APP_LOCK(irq_state);

    *p_count = g_dbg_n;
    for (idx = 0; (idx < g_dbg_n) && (idx < BARCODE_DBG_MAX) && (idx < max);
         idx++)
    {
        p_w_mm10[idx] = g_dbg_w[idx];
    }

    APP_UNLOCK(irq_state);
    return g_dbg_seq;
}
/* Single black patches (left branches, grout lines) */
/* the last first-bar: its centre (an odometer count: 32 bits like the
   encoder counts) and its width, 0.1 mm, 9 bits (saturates at 51.1 mm) */
typedef struct
{
    int32_t         odo;
    uint32_t w_mm10 APP_FIELD(9);
} bar_mark_t;

LOCAL bar_mark_t        g_mark; /* under APP_LOCK */
LOCAL volatile uint32_t g_mark_seq;

EXPORT uint32_t barcode_mark (int32_t * p_centre_odo, int32_t * p_width_mm10)
{
    uint32_t irq_state;
    uint32_t seq;

    APP_LOCK(irq_state);
    *p_centre_odo = g_mark.odo;
    *p_width_mm10 = g_mark.w_mm10;
    seq           = g_mark_seq;
    APP_UNLOCK(irq_state);
    return seq;
}

/** The first bar of a run has just ended at end_odo, w counts wide.
   Published at once, not after the quiet gap, so a left branch is known
   while there is still room to turn (B sits behind C).  Every first bar
   is published (also a barcode's narrow start bar and grout lines); the
   line task keeps only the ones about one tape wide.  No Code 39 bar is
   that wide (wide bars measured 10.5 mm). */
LOCAL void mark_pub (int32_t end_odo, int32_t width)
{
    uint32_t irq_state;

    APP_LOCK(irq_state);
    g_mark.odo = end_odo - (width / 2); /* driving forward: centre is behind */
    g_mark.w_mm10 = width_field(width * S2_UM_PER_COUNT / 100);
    g_mark_seq++;
    APP_UNLOCK(irq_state);
}

EXPORT uint32_t barcode_seq (void)
{
    return g_seq_now;
}

/** Ignore B for the next mm of driving.  20 Nov, track 4: just after a
   bypass, with the robot still at an angle to its line, B (85 mm left of
   the centre) crossed the line's own tape; that read as bars, so Motion
   slowed to 50 mm/s and held the steering straight, and the robot drove
   diagonally off the line ("line lost"). */
EXPORT void barcode_ignore_mm (int32_t mm)
{
    uint32_t irq_state;

    APP_LOCK(irq_state);
    g_ignore_odo    = motor_odometer();
    g_ignore_counts = mm * 1000 / S2_UM_PER_COUNT; /* a few thousand */
    APP_UNLOCK(irq_state);
}

/** true while barcode_ignore_mm()'s distance has not been driven yet. */
LOCAL bool ignoring (void)
{
    uint32_t irq_state;
    int32_t  odo0;
    int32_t  counts;
    int32_t  driven;

    APP_LOCK(irq_state);
    odo0   = g_ignore_odo;
    counts = g_ignore_counts;
    APP_UNLOCK(irq_state);
    driven = motor_odometer() - odo0;

    if (driven < 0)
    {
        driven = -driven;
    }

    return driven < counts;
}

/** Forget any half-read code (start of a run, after a manoeuvre).  On 7 Nov
   tape held under B at the end of the live view left a code open, and
   the run started with a rescan.  Task context; blocks 30 ms. */
EXPORT void barcode_reset (void)
{
    gb_paused = true;
    tk_dly_tsk(30); /* the task drops it within 10 ms */
    gb_paused = false;
}

/** Copies the latest barcode result. */
EXPORT void barcode_last (barcode_result_t * p_result)
{
    uint32_t irq_state;

    APP_LOCK(irq_state);
    *p_result = g_last;
    APP_UNLOCK(irq_state);
}

/** One character from 9 widths; 0 = no match */
LOCAL uint8_t decode_char (const int32_t * p_elem, int32_t * p_narrow_sum,
                           int32_t * p_wide_sum)
{
    int32_t  idx;
    int32_t  jdx;
    int32_t  wide[C39_WIDE_ELEMS] = {-1, -1, -1};
    uint16_t pat                  = 0;
    int32_t  min_wide;
    int32_t  max_narrow = 0;

    for (jdx = 0; jdx < C39_WIDE_ELEMS; jdx++)
    {
        int32_t best = -1;

        for (idx = 0; idx < C39_CHAR_ELEMS; idx++)
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
        pat |= (uint16_t)(1u << (8 - best)); /* best 0..8: bits 0..8 */
    }

    min_wide = p_elem[wide[2]];

    for (idx = 0; idx < C39_CHAR_ELEMS; idx++)
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

    /* 44 characters: the count fits int32_t */
    for (idx = 0; idx < (int32_t)(sizeof(g_c39) / sizeof(g_c39[0])); idx++)
    {
        if (g_c39[idx].pat == pat)
        {
            return g_c39[idx].letter;
        }
    }

    return 0;
}

/** Decode n elements; fills r.  true = valid "*...*".  The code may start
   at any bar (off even) and end before the last element: a stray mark
   within S3_QUIET_MM of the code is part of the same run. */
LOCAL bool decode (int32_t * p_elem, int32_t count, barcode_result_t * p_result)
{
    int32_t kdx;
    int32_t pass;
    int32_t idx;
    int32_t off;
    int32_t narrow_sum = 0;
    int32_t wide_sum   = 0;

    p_result->b_ok     = false;
    p_result->elements = count;

    if (count < C39_MIN_ELEMS)
    {
        return false;
    }

    for (pass = 0; pass < DECODE_PASSES; pass++)
    {
        for (off = 0; (off + C39_MIN_ELEMS) <= count; off += 2)
        {
            narrow_sum = wide_sum = 0;

            if (decode_char(&p_elem[off], &narrow_sum, &wide_sum) != '*')
            {
                continue;
            }

            for (kdx = (count - off + 1) / C39_CHAR_PITCH; kdx >= C39_MIN_CHARS;
                 kdx--)
            {
                uint8_t letter;

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
                    p_result->b_ok       = true;
                    p_result->letter     = letter;
                    p_result->b_reversed = (1 == pass);
                    /* 3 chars of 6 narrow + 3 wide counted above */
                    p_result->narrow_mm10 =
                        width_field(narrow_sum * S2_UM_PER_COUNT / 100 / 18);
                    p_result->wide_mm10 =
                        width_field(wide_sum * S2_UM_PER_COUNT / 100 / 9);
                    return true;
                }
            }
        }

        /* try backwards */
        for (idx = 0; idx < (count / 2); idx++)
        {
            int32_t tmp             = p_elem[idx];
            p_elem[idx]             = p_elem[count - 1 - idx];
            p_elem[count - 1 - idx] = tmp;
        }
    }

    return false;
}

/** ---- Forgiving decode (8 Nov) -------------------------------------------
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
LOCAL void sort_w (int32_t * p_values, int32_t count)
{
    int32_t idx;
    int32_t jdx;
    int32_t tmp;

    for (idx = 1; idx < count; idx++)
    {
        /* insertion sort, n <= 32 */
        tmp = p_values[idx];

        for (jdx = idx; (jdx > 0) && (p_values[jdx - 1] > tmp); jdx--)
        {
            p_values[jdx] = p_values[jdx - 1];
        }

        p_values[jdx] = tmp;
    }
}

/** the 3 widest of 9 at least twice the 6 narrow */
LOCAL bool clear_split (const int32_t * p_elem)
{
    int32_t sorted[C39_CHAR_ELEMS];
    int32_t idx;

    for (idx = 0; idx < C39_CHAR_ELEMS; idx++)
    {
        sorted[idx] = p_elem[idx];
    }

    sort_w(sorted, 9);
    return sorted[6] >= (2 * sorted[5]);
}

/** Sum of 9 element widths (one character). */
LOCAL int32_t sum9 (const int32_t * p_elem)
{
    int32_t sum = 0;
    int32_t idx;

    for (idx = 0; idx < C39_CHAR_ELEMS; idx++)
    {
        sum += p_elem[idx];
    }

    return sum;
}

/** typical narrow and wide width of n elements (n <= 32): the median of
   the narrowest two thirds and of the widest quarter */
LOCAL void nar_wid (const int32_t * p_elem, int32_t count, int32_t * p_nar,
                    int32_t * p_wid)
{
    int32_t sorted[32];
    int32_t idx;

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

/** Forgiving decode: one start / stop character damaged. */
LOCAL bool decode_one_star (int32_t * p_elem, int32_t count,
                            barcode_result_t * p_result)
{
    int32_t pass;
    int32_t off;
    int32_t idx;
    uint8_t found       = 0;
    bool    b_rev_found = false;
    int32_t f_nar       = 0;
    int32_t f_wid       = 0;

    if (count < C39_MIN_ELEMS)
    {
        return false;
    }

    for (pass = 0; pass < DECODE_PASSES; pass++)
    {
        for (off = 0; (off + C39_MIN_ELEMS) <= count; off += 2)
        {
            int32_t narrow_sum = 0;
            int32_t wide_sum   = 0;
            int32_t nar;
            int32_t wid;
            int32_t sum_mid;
            int32_t sum_other;
            int32_t sorted[C39_MIN_ELEMS];
            uint8_t first_ch;
            uint8_t letter;
            uint8_t last_ch;
            bool    b_a_ok;
            int32_t kdx;

            first_ch = decode_char(&p_elem[off], &narrow_sum, &wide_sum);
            letter   = decode_char(&p_elem[off + 10], &narrow_sum, &wide_sum);
            last_ch  = decode_char(&p_elem[off + 20], &narrow_sum, &wide_sum);

            if ((0 == letter) || ('*' == letter) ||
                (('*' == first_ch) == ('*' == last_ch)))
            {
                continue;
            }

            b_a_ok = ('*' == first_ch);

            for (kdx = 0; kdx < C39_MIN_ELEMS; kdx++)
            {
                sorted[kdx] = p_elem[off + kdx];
            }

            sort_w(sorted, C39_MIN_ELEMS); /* m[14]: the median, a narrow one */

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
        {
            /* backwards */
            int32_t tmp = p_elem[idx];

            p_elem[idx]             = p_elem[count - 1 - idx];
            p_elem[count - 1 - idx] = tmp;
        }
    }

    if ((0 == found) || (0xFF == found))
    {
        return false;
    }

    p_result->b_ok        = true;
    p_result->letter      = found;
    p_result->b_reversed  = b_rev_found;
    p_result->fixed       = BAR_FIX_STAR;
    p_result->narrow_mm10 = width_field(f_nar * S2_UM_PER_COUNT / 100);
    p_result->wide_mm10   = width_field(f_wid * S2_UM_PER_COUNT / 100);
    return true;
}

/** Forgiving decode: a code with one narrow element lost (27-28 elements). */
LOCAL bool decode_merged (const int32_t * p_elem, int32_t count,
                          barcode_result_t * p_result)
{
    int32_t          trial_elems[31];
    int32_t          nar;
    int32_t          wid;
    int32_t          tot;
    int32_t          opt[WIDTH_OPTS];
    int32_t          idx;
    int32_t          first_opt;
    int32_t          last_opt;
    int32_t          jdx;
    uint8_t          found = 0;
    barcode_result_t trial;
    barcode_result_t keep;

    if ((count != 27) && (count != 28))
    {
        return false;
    }

    keep = *p_result;
    nar_wid(p_elem, count, &nar, &wid);
    opt[0] = nar;
    opt[1] = wid;

    for (idx = 0; idx < count; idx++)
    {
        for (first_opt = 0; first_opt < WIDTH_OPTS; first_opt++)
        {
            for (last_opt = 0; last_opt < WIDTH_OPTS; last_opt++)
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
                        return false; /* two answers */
                    }

                    found = trial.letter;
                    keep  = trial;
                }
            }
        }
    }

    if (0 == found)
    {
        return false;
    }

    keep.elements = count;
    keep.fixed    = BAR_FIX_MERGED;
    *p_result     = keep;
    return true;
}

/** Cut short (8 Nov): F's last narrow bar is printed weakly (darkest 1839
   against ~2900 for the others) and was not seen, with the space before
   it: 27 elements, the first 27 of a good F.  27-28 elements: add one or
   two narrow elements at the end, or at the start, and decode cleanly.
   Logs' good reads with 1-2 elements cut off either end: 33 right, 0
   wrong, 11 not read; 50 000 random 27-28 element patterns: 1 decoded. */
LOCAL bool decode_truncated (const int32_t * p_elem, int32_t count,
                             barcode_result_t * p_result)
{
    int32_t          trial_elems[31];
    int32_t          nar;
    int32_t          wid;
    int32_t          kdx;
    int32_t          side;
    int32_t          jdx;
    uint8_t          found = 0;
    barcode_result_t trial;
    barcode_result_t keep;

    if ((count != 27) && (count != 28))
    {
        return false;
    }

    keep = *p_result;
    nar_wid(p_elem, count, &nar, &wid);

    for (kdx = 1; kdx <= CUT_MAX; kdx++)
    {
        if ((count + kdx) < C39_MIN_ELEMS)
        {
            continue;
        }

        for (side = 0; side < CUT_SIDES; side++)
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
                    return false; /* two answers */
                }

                found = trial.letter;
                keep  = trial;
            }
        }
    }

    if (0 == found)
    {
        return false;
    }

    keep.elements = count;
    keep.fixed    = BAR_FIX_CUT;
    *p_result     = keep;
    return true;
}

/* ---- Cut-short start (15 Nov) -------------------------------------------
   15 Nov course run: "READ_FAIL (25 elements)" of an F where the F and the
   closing '*' read cleanly but the first 4 elements of the opening '*'
   were never seen (its narrow bars read 1.6-2.3 mm, at the sensor's
   limit).  Accepted only if the letter and the full '*' decode, both gaps
   are narrow, and every element left of the cut '*' is narrow or wide as
   in '*', judged against the two clean characters' bars and spaces
   separately (bars read about 3 mm thinner than spaces that day).
   Prototype: 15 Nov read -> F; 4000 synthetic cut codes 97 % read, 0
   wrong; random patterns 1 in 100 000. */

/** Sums of one decoded character's widths: [bar/space][narrow/wide]. */
LOCAL void kind_sums (const int32_t * p_elem, uint16_t pat,
                      int32_t p_sum[ELEM_KINDS][WIDTH_OPTS],
                      int32_t p_n[ELEM_KINDS][WIDTH_OPTS])
{
    int32_t idx;

    for (idx = 0; idx < C39_CHAR_ELEMS; idx++)
    {
        int32_t kind = idx % ELEM_KINDS;
        /* idx 0..8: shift 8..0 */
        int32_t wide = (int32_t)((pat >> (uint32_t)(8 - idx)) & 1u);

        p_sum[kind][wide] += p_elem[idx];
        p_n[kind][wide]++;
    }
}

/** The 9-bit pattern of a letter (0 = not in the table). */
LOCAL uint16_t c39_pat (uint8_t letter)
{
    int32_t idx;

    /* 44 characters: the count fits int32_t */
    for (idx = 0; idx < (int32_t)(sizeof(g_c39) / sizeof(g_c39[0])); idx++)
    {
        if (g_c39[idx].letter == letter)
        {
            return g_c39[idx].pat;
        }
    }

    return 0u;
}

/** true = width is wide for its kind: above the middle of that kind's
   narrow and wide averages (widths and counts small: no overflow). */
LOCAL bool cut_is_wide (int32_t width, int32_t kind,
                        int32_t p_sum[ELEM_KINDS][WIDTH_OPTS],
                        int32_t p_n[ELEM_KINDS][WIDTH_OPTS])
{
    int32_t n_nar = p_n[kind][0];
    int32_t n_wid = p_n[kind][1];

    return (width * 2 * n_nar * n_wid) >
           ((p_sum[kind][0] * n_wid) + (p_sum[kind][1] * n_nar));
}

/** One trial: 29 slots, -1 where cut.  b_front: the partial '*' is the
   first character.  Returns the letter, 0 = no. */
LOCAL uint8_t cut_try (const int32_t * p_full, bool b_front)
{
    const int32_t * p_part = b_front ? &p_full[0] : &p_full[2 * C39_CHAR_PITCH];
    const int32_t * p_star = b_front ? &p_full[2 * C39_CHAR_PITCH] : &p_full[0];
    int32_t         sum[ELEM_KINDS][WIDTH_OPTS] = {{0, 0}, {0, 0}};
    int32_t         num[ELEM_KINDS][WIDTH_OPTS] = {{0, 0}, {0, 0}};
    int32_t         nar_sum                     = 0;
    int32_t         wid_sum                     = 0;
    int32_t         idx;
    uint8_t         letter;

    if (decode_char(p_star, &nar_sum, &wid_sum) != '*')
    {
        return 0;
    }

    letter = decode_char(&p_full[C39_CHAR_PITCH], &nar_sum, &wid_sum);

    if ((0 == letter) || ('*' == letter))
    {
        return 0;
    }

    kind_sums(p_star, STAR_PAT, sum, num);
    kind_sums(&p_full[C39_CHAR_PITCH], c39_pat(letter), sum, num);

    for (idx = 0; idx < ELEM_KINDS; idx++)
    {
        if ((0 == num[idx][0]) || (0 == num[idx][1]))
        {
            return 0;
        }
    }

    /* the two gaps between characters are narrow spaces */
    if ((cut_is_wide(p_full[C39_CHAR_PITCH - 1], 1, sum, num)) ||
        (cut_is_wide(p_full[(2 * C39_CHAR_PITCH) - 1], 1, sum, num)))
    {
        return 0;
    }

    for (idx = 0; idx < C39_CHAR_ELEMS; idx++)
    {
        /* idx 0..8: shift 8..0 */
        bool b_star_wide = ((STAR_PAT >> (uint32_t)(8 - idx)) & 1u) != 0u;

        if ((p_part[idx] >= 0) && (cut_is_wide(p_part[idx], idx % ELEM_KINDS,
                                               sum, num) != b_star_wide))
        {
            return 0;
        }
    }

    return letter;
}

/** Forgiving decode: the first CUT_STAR_MIN..CUT_STAR_MAX elements of the
   read lost (either reading direction); one answer only. */
LOCAL bool decode_cut_star (const int32_t * p_elem, int32_t count,
                            barcode_result_t * p_result)
{
    int32_t full[C39_MIN_ELEMS];
    int32_t cut = C39_MIN_ELEMS - count;
    int32_t pass;
    int32_t side;
    int32_t idx;
    int32_t nar;
    int32_t wid;
    uint8_t found = 0;
    bool    b_rev = false;

    if ((cut < CUT_STAR_MIN) || (cut > CUT_STAR_MAX))
    {
        return false;
    }

    for (pass = 0; pass < DECODE_PASSES; pass++)
    {
        for (side = 0; side < CUT_SIDES; side++)
        {
            uint8_t letter;

            for (idx = 0; idx < C39_MIN_ELEMS; idx++)
            {
                full[idx] = -1;
            }

            for (idx = 0; idx < count; idx++)
            {
                full[(0 == side) ? (idx + cut) : idx] =
                    p_elem[(0 == pass) ? idx : (count - 1 - idx)];
            }

            letter = cut_try(full, 0 == side);

            if ((letter != 0) && (found != 0) && (letter != found))
            {
                return false; /* two answers */
            }

            if (letter != 0)
            {
                found = letter;
                b_rev = (1 == pass);
            }
        }
    }

    if (0 == found)
    {
        return false;
    }

    nar_wid(p_elem, count, &nar, &wid);
    p_result->b_ok        = true;
    p_result->letter      = found;
    p_result->b_reversed  = b_rev;
    p_result->fixed       = BAR_FIX_CUT;
    p_result->narrow_mm10 = width_field(nar * S2_UM_PER_COUNT / 100);
    p_result->wide_mm10   = width_field(wid * S2_UM_PER_COUNT / 100);
    return true;
}

/* the code being read: the decoder's state from one edge to the next */
typedef struct
{
    int32_t elem[MAX_ELEM]; /* element widths so far, odometer counts */
    int32_t count;          /* elements so far */
    int32_t prev_odo;       /* where the last element ended */
    bool    b_in_code;      /* inside a code */
    bool    b_too_long;     /* more than MAX_ELEM elements: no decode */
    uint8_t stable;         /* the debounced level */
} bar_read_t;

/** The end of an element of the code being read, at pos: its width. */
LOCAL void add_element (bar_read_t * p_read, int32_t pos)
{
    int32_t width = pos - p_read->prev_odo;

    if (width < 0)
    {
        width = -width;
    }

    if (p_read->count < MAX_ELEM)
    {
        p_read->elem[p_read->count] = width;
        p_read->count++;
    }
    else
    {
        p_read->b_too_long = true;
    }

    if (1 == p_read->count)
    {
        mark_pub(pos, width);
    }

    p_read->prev_odo = pos;
    g_edge_pub       = pos;
}

/** A level counts only once it has held for MERGE_COUNTS of travel: then
   it starts at pos, where its last raw edge was.  Flicker never holds, so
   it neither makes elements nor keeps a code open (Stage 2, 2 Nov:
   pair-cancelling let every other flicker edge through). */
LOCAL void commit_level (bar_read_t * p_read, uint8_t lvl, int32_t pos)
{
    if ((lvl != p_read->stable) && (!p_read->b_in_code))
    {
        p_read->stable = lvl;

        if (IR_BLACK == lvl)
        {
            /* the first bar: a code starts */
            p_read->b_in_code  = true;
            p_read->b_too_long = false;
            p_read->count      = 0;
            p_read->prev_odo   = pos;
            g_edge_pub         = pos;
            gb_busy            = true;
        }
    }
    else if (lvl != p_read->stable)
    {
        p_read->stable = lvl;
        add_element(p_read, pos);
    }
    else
    {
        /* the same level: nothing changes */
    }
}

/** The end of a code (quiet for S3_QUIET_MM): its element widths are kept
   for the tests (barcode_debug); a code long enough is decoded (the
   forgiving decodes if the clean one fails) and published, and Mission is
   told.  Returns the number of results published so far. */
LOCAL uint32_t code_ended (bar_read_t * p_read, uint32_t seq)
{
    int32_t  idx;
    uint32_t irq_state;
    uint32_t seq_now = seq;

    APP_LOCK(irq_state);
    g_dbg_n = (uint8_t)p_read->count; /* <= MAX_ELEM (99): fits uint8_t */

    for (idx = 0; (idx < p_read->count) && (idx < BARCODE_DBG_MAX); idx++)
    {
        /* clamped to 0..65535 first: fits uint16_t */
        g_dbg_w[idx] = (uint16_t)app_clamp(
            p_read->elem[idx] * S2_UM_PER_COUNT / 100, 0, 65535);
    }

    g_dbg_seq++;
    APP_UNLOCK(irq_state);

    if (p_read->count >= MIN_ELEM_REPORT)
    {
        barcode_result_t result = {0};

        if ((!p_read->b_too_long) &&
            (!decode(p_read->elem, p_read->count, &result)) &&
            (!decode_one_star(p_read->elem, p_read->count, &result)) &&
            (!decode_merged(p_read->elem, p_read->count, &result)) &&
            (!decode_truncated(p_read->elem, p_read->count, &result)))
        {
            (void)decode_cut_star(p_read->elem, p_read->count, &result);
        }

        seq_now++;
        result.seq      = seq_now;
        result.elements = p_read->count;
        APP_LOCK(irq_state);
        g_last = result;
        APP_UNLOCK(irq_state);
        g_seq_now = seq_now;
        tk_set_flg(gh_flg_mission, MSN_FLG_BARCODE);
    }
    else if (p_read->count >= SHORT_RUN_PRINT)
    {
        /* 15 Nov: a barcode broken into pieces left no trace; a piece of
           several elements is printed (a grout line is 1-2) */
        tm_printf(app_str("[S3] short run dropped (%d elements), widths "
                          "(mm):"),
                  p_read->count);

        for (idx = 0; idx < p_read->count; idx++)
        {
            /* counts x 0.08 mm, a few hundred at most: 0.1 mm fits */
            int32_t mm10 = p_read->elem[idx] * S2_UM_PER_COUNT / 100;

            tm_printf(app_str(" %d.%d"), mm10 / 10, mm10 % 10);
        }

        tm_printf(app_str("\n"));
    }
    else
    {
        /* 1-4 elements: a grout line or a tile join, nothing to report */
    }

    return seq_now;
}

/** The barcode decoder task: builds bar widths from the edges and decodes
 * each code. */
EXPORT void barcode_task (int32_t stacd, void * p_exinf)
{
    bar_read_t read = {{0}, 0, 0, false, false, !IR_BLACK};
    uint32_t   seq  = 0;
    uint32_t   end;
    uint8_t    raw_lvl  = !IR_BLACK; /* level after the last raw edge */
    int32_t    raw_odo  = 0;         /* where that raw edge was */
    bool       b_raw_ok = false;

    (void)stacd;
    (void)p_exinf;
    g_ring_r = g_ring_w;

    for (;;)
    {
        (void)tk_slp_tsk(TIMEOUT_BARCODE_MS);
        app_timing_wake(g_tmg_barcode, 0);

        /* barcode_reset() or barcode_ignore_mm(): drop this read and
           every edge until it ends, then start fresh */

        if ((gb_paused) || (ignoring()))
        {
            g_ring_r       = g_ring_w;
            read.b_in_code = false;
            read.count     = 0;
            gb_busy        = false;
            read.stable    = !IR_BLACK;
            b_raw_ok       = false;
            app_timing_done(g_tmg_barcode);
            continue;
        }

        /* Drain the edges that are here now, never more than the ring
           holds: a chattering sensor keeps adding edges, and an
           unbounded loop starved every task below this one (530 ms,
           Stage 2 run, 1 Nov).  If the ring overflowed, edges were
           lost: drop the barcode being read. */
        {
            uint32_t avail = g_ring_w - g_ring_r;

            if (avail > RING)
            {
                g_ring_r       = g_ring_w - RING;
                read.b_in_code = false;
                read.count     = 0;
                read.stable    = !IR_BLACK;
                b_raw_ok       = false;
                avail          = RING;
            }

            end = g_ring_r + avail;
        }
        while (g_ring_r != end)
        {
            bar_edge_t edge     = g_ring[g_ring_r & (RING - 1)];
            int32_t    edge_odo = odo_from16(edge.odo16);
            int32_t    delta;

            g_ring_r++;

            if (b_raw_ok)
            {
                delta = edge_odo - raw_odo;

                if (delta < 0)
                {
                    delta = -delta;
                }

                if (delta >= MERGE_COUNTS)
                {
                    commit_level(&read, raw_lvl, raw_odo); /* that level held */
                }
            }

            raw_lvl  = edge.level; /* 0 or 1 */
            raw_odo  = edge_odo;
            b_raw_ok = true;
        }

        /* The latest level may have held long enough by now */
        if (b_raw_ok)
        {
            int32_t delta = motor_odometer() - raw_odo;

            if (delta < 0)
            {
                delta = -delta;
            }

            if (delta >= MERGE_COUNTS)
            {
                commit_level(&read, raw_lvl, raw_odo);
            }
        }

        /* End of a barcode: quiet for S3_QUIET_MM */
        if (read.b_in_code)
        {
            int32_t gap = motor_odometer() - read.prev_odo;

            if (gap < 0)
            {
                gap = -gap;
            }

            if (motion_counts_to_mm(gap) > S3_QUIET_MM)
            {
                read.b_in_code = false;
                seq            = code_ended(&read, seq);
                read.count     = 0;
            }
        }

        gb_busy = read.b_in_code;

        app_timing_done(g_tmg_barcode);
    }
}

/* ---- Mapping (B7, B8) ---------------------------------------------- */

/** true for a letter or digit a barcode can carry. */
LOCAL bool is_code_char (uint8_t letter)
{
    return ((letter >= 'A') && (letter <= 'Z')) ||
           ((letter >= '0') && (letter <= '9'));
}

/** A move letter ('L' 'R' 'F' 'U') as its index in MOVE_CODES. */
LOCAL uint32_t move_code (uint8_t move_ch)
{
    uint32_t code = 0u;
    uint32_t idx;

    for (idx = 0u; idx < BAR_MOVES; idx++)
    {
        code = (MOVE_CODES[idx] == move_ch) ? idx : code;
    }

    return code;
}

/** Takes a new mapping text; false and the reason if it is refused. */
EXPORT bool barcode_set_mapping (const uint8_t * p_text, int32_t len,
                                 uint8_t * p_why, int32_t why_max)
{
    uint8_t  letters[BAR_MOVES];
    uint8_t  moves[BAR_MOVES];
    int32_t  pairs = 0;
    int32_t  idx   = 0;
    int32_t  first;
    int32_t  second;
    uint32_t irq_state;

    p_why[0] = '\0';

    while (idx < len)
    {
        uint8_t letter;
        uint8_t move_ch;

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
            tm_sprintf(p_why, app_str("expected 4 pairs like K=L,Q=R,X=F,M=U"));
            return false;
        }

        letter  = p_text[idx];
        move_ch = p_text[idx + 2];

        if ((letter >= 'a') && (letter <= 'z'))
        {
            letter = (uint8_t)(letter - 'a' + 'A'); /* a..z -> A..Z */
        }

        if ((move_ch >= 'a') && (move_ch <= 'z'))
        {
            move_ch = (uint8_t)(move_ch - 'a' + 'A'); /* a..z -> A..Z */
        }

        if ((!is_code_char(letter)) ||
            ((move_ch != MOVE_LEFT) && (move_ch != MOVE_RIGHT) &&
             (move_ch != MOVE_FORWARD) && (move_ch != MOVE_UTURN)))
        {
            tm_sprintf(p_why, app_str("pair %d: '%c=%c' not A-Z/0-9 = L/R/F/U"),
                       pairs + 1, letter, move_ch);
            return false;
        }

        letters[pairs] = letter;
        moves[pairs]   = move_ch;
        pairs++;
        idx += 3;
    }

    if (pairs != 4)
    {
        tm_sprintf(p_why, app_str("%d pairs, need exactly 4"), pairs);
        return false;
    }

    for (first = 0; first < BAR_MOVES; first++)
    {
        for (second = first + 1; second < BAR_MOVES; second++)
        {
            if ((letters[first] == letters[second]) ||
                (moves[first] == moves[second]))
            {
                tm_sprintf(p_why,
                           app_str("each character and each move only once"));
                return false;
            }
        }
    }

    (void)why_max;
    APP_LOCK(irq_state);

    for (first = 0; first < BAR_MOVES; first++)
    {
        g_map[first].letter = letters[first];
        g_map[first].move   = move_code(moves[first]);
    }

    gb_map_set = true;
    APP_UNLOCK(irq_state);
    return true;
}

/** The letter mapped to each move, in MOVE_CODES order (L, R, F, U); 0 for
   every move while there is no mapping. */
EXPORT void barcode_mapping_get (uint8_t * p_letters)
{
    uint32_t irq_state;
    int32_t  idx;

    APP_LOCK(irq_state);

    for (idx = 0; idx < BAR_MOVES; idx++)
    {
        p_letters[idx] = 0u;
    }

    for (idx = 0; gb_map_set && (idx < BAR_MOVES); idx++)
    {
        p_letters[g_map[idx].move] = g_map[idx].letter; /* 7-bit ASCII */
    }

    APP_UNLOCK(irq_state);
}

/** The mapped move (MOVE_LEFT ...) for a barcode letter, or MOVE_NONE. */
EXPORT int32_t barcode_move_for (uint8_t letter)
{
    int32_t  idx;
    int32_t  move = MOVE_NONE;
    uint32_t irq_state;

    APP_LOCK(irq_state);

    if (gb_map_set)
    {
        for (idx = 0; idx < BAR_MOVES; idx++)
        {
            if (g_map[idx].letter == letter)
            {
                move = (int32_t)MOVE_CODES[g_map[idx].move]; /* a letter */
            }
        }
    }

    APP_UNLOCK(irq_state);
    return move;
}

/** The mapping now in use as "K=L,Q=R,X=F,M=U" (empty if none); its length. */
EXPORT int32_t barcode_mapping_text (uint8_t * p_out, int32_t max)
{
    uint8_t  letters[BAR_MOVES];
    uint8_t  moves[BAR_MOVES];
    bool     b_set;
    uint32_t irq_state;
    int32_t  idx;

    APP_LOCK(irq_state);
    b_set = gb_map_set;

    for (idx = 0; idx < BAR_MOVES; idx++)
    {
        letters[idx] = g_map[idx].letter;           /* 7-bit ASCII */
        moves[idx]   = MOVE_CODES[g_map[idx].move]; /* 'L' 'R' 'F' 'U' */
    }

    APP_UNLOCK(irq_state);

    if ((!b_set) || (max < 16))
    {
        p_out[0] = '\0';
        return 0;
    }

    return tm_sprintf(p_out, app_str("%c=%c,%c=%c,%c=%c,%c=%c"), letters[0],
                      moves[0], letters[1], moves[1], letters[2], moves[2],
                      letters[3], moves[3]);
}

/*** end of file ***/
