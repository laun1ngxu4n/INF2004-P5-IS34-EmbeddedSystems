/**
 * @file    task_line.c
 *  Subsystem 3 -- the Line Following task (every 20 ms).
 *
 *  Two sensors STRADDLE the tape (7 Nov): L left of it, R right of it,
 *  a few mm clear of each edge when centred.
 *    L white, R white   centred (or a gap)      straight
 *    L black, R white   the line moves left     turn left
 *    L white, R black   the line moves right    turn right
 *    L black, R black   a cross line            straight (a crossing)
 *  Steering (tuned 7 Nov in a simulation calibrated on that day's three
 *  robot runs: it reproduced the lost left curves at 150 mm/s, the zigzag
 *  of the hard-at-once version and the good third run; scenarios: 100-130
 *  mm curves, S-bends, straights, sensor windows 6-20 mm, motor lag and
 *  delay, +-2-4 % wheel mismatch.  Old settings 297/480, these 478/480
 *  with no sweeps, same speed):
 *    - a correction is STEER_CORR at once, Motion at S3_CORR_PCT, then
 *      S3_LOW_PCT after S3_LOW_AFTER_MM of it;
 *    - steering memory: the steering averaged over the last S3_MEM_MM;
 *      while both sensors are white the robot keeps S3_MEM_K of it.  On a
 *      straight the corrections alternate L, R and it stays near 0; on a
 *      curve it settles at the turn the curve needs, so the robot keeps
 *      turning between corrections instead of driving straight off (the
 *      S-bend of track 3, 7 Nov).  It also slows a little in curves.
 *  Overshoot: on a tight curve the tape can pass right under a sensor
 *  and out the other side, which also reads "both white".  After a
 *  correction (S3_LONG_CORR_MM) followed by S3_OVERSHOOT_MM of white it
 *  turns STEER_SEARCH toward the side that saw the line last (LINE_SEARCH)
 *  at S3_SEARCH_PCT; LINE_LOST after S3_LOST_MM or S3_LOST_MS of that.
 *  Both white is also "no line at all": on a long straight, well
 *  centred, the robot touched nothing for 400 mm and stopped with
 *  LINE_LOST on a good line (Stage 2B, 7 Nov).  So after
 *  S3_PROBE_AFTER_MM of white it steers very gently (STEER_PROBE) toward
 *  the side it saw last, for S3_PROBE_MM, then the other way for twice
 *  that: a sensor touching the tape ends the probe.  Nothing either way:
 *  LINE_LOST (a line end is driven past by about 400 mm).
 *  Stuck sensor: one black for S3_STUCK_ONE_MM, or both black for
 *  S3_STUCK_BOTH_MM, of driving: LINE_FAULT, Mission stops.
 *
 *  Junctions (see subsystem3.h):
 *    Crossing (X, crossroads, T end): both black for S3_JUNC_MIN_MM ..
 *    S3_CROSS_MAX_MM (longer than one tape width when crossed at an
 *    angle), after driving straight (no correction) before it.  The tape
 *    is narrower than the L-R gap, so the line itself only makes both
 *    black when the robot sweeps across it at an angle, on a tight curve
 *    while correcting: 2 false crossings on a curve on 7 Nov.
 *    Left branch: B saw a patch about one tape wide, and the robot was
 *    driving straight (no correction, no search) before that place.  B
 *    is not level with the line sensors, so the corrections are kept by
 *    distance (g_off_a/g_off_b) and looked up when B's patch arrives.  On
 *    curves the robot corrects all the time, so a line seen by B there
 *    does not count.  (After the arm L touches it and the robot turns a
 *    little toward it, so after it is not checked.)
 *    Crossing with a left arm, two sensors agreeing (8 Nov): both black
 *    for S3_PAIR_BOTH_MIN_MM .. S3_CROSS_MAX_MM, and B (S3_BAR_AHEAD_MM
 *    behind) then sees a patch about a tape wide within S3_PAIR_MM of
 *    the same place.  Counts even while the robot corrects: with the
 *    robot zigzagging (24 corrections a metre, 8 Nov) the "drove
 *    straight before it" rule threw away every junction, and the robot
 *    read a barcode and drove on through its junction.  In the saved
 *    logs of 7-8 Nov this pairing finds 11 places, 3 of them detected at
 *    the time; the zigzag's own both-black stretches are 2-6 mm and B
 *    sees nothing there.
 *    Two within S3_JUNC_MERGE_MM are one junction.
 *  Mission is told through gh_flg_mission (MSN_FLG_JUNCTION /
 *  MSN_FLG_LINE_LOST); line_junction_odo() says where the centre was.
 */

#include "subsystem3.h"
#include "../subsystem2/subsystem2.h" /* motion_odometer() */
#include "../subsystem5/subsystem5.h" /* gh_flg_mission */

/* Steering (mm/s each wheel) and Motion's speed while correcting */
#define STEER_CORR      30 /* every correction, at once */
#define STEER_SEARCH    30 /* the overshoot search */
#define STEER_STEP      10 /* most the steering may grow per 20 ms */
#define S3_CORR_PCT     60 /* speed while correcting ... */
#define S3_LOW_PCT      40 /* ... and after S3_LOW_AFTER_MM of it */
#define S3_LOW_AFTER_MM 50
#define S3_SEARCH_PCT   20 /* searching (0 rocked in place, 2 Nov) */
/* Steering memory */
#define S3_MEM_MM   60   /* averaged over about this distance */
#define S3_MEM_K    0.4f /* share kept while both sensors are white */
#define S3_MEM_MAX  40
#define S3_MEM_SLOW 0.5f /* speed % less per mm/s of memory (curves) */
/* a sensor pattern counts only after this many readings in a row (2 made
 * every correction 20-40 ms late) */
#define S3_FILTER_N 1
/* a correction this long may have let the tape slip out past the sensor ... */
#define S3_LONG_CORR_MM 5
/* ... white this long after it: overshot (the memory keeps turning
 * meanwhile) */
#define S3_OVERSHOOT_MM 60
/* Crossing: no correction from both_from - S3_CROSS_BEFORE_MM to
   both_from - S3_CROSS_LEAD_MM (one sensor may reach an angled cross
   line first) */
#define S3_CROSS_BEFORE_MM 40
#define S3_CROSS_LEAD_MM   20
#define S3_PROBE_AFTER_MM  200 /* white this long: look for the tape */
/* one way this far, the other way twice that; nothing = lost. Longest gap it
 * can cross: about 200 + 3 x 70 mm */
#define S3_PROBE_MM 70
/* mm/s: the sensors move ~5 mm sideways (the clearance to the tape) in ~30
 * mm */
#define STEER_PROBE         12
#define S3_LOST_MM          250  /* searching this far ... */
#define S3_LOST_MS          4000 /* ... or this long (it turns on the spot) */
#define S3_CROSS_MAX_MM     45   /* both black, crossed at up to ~60 deg */
#define S3_PAIR_BOTH_MIN_MM 10   /* both black this long ... */
/* ... and B a patch from 9.0 mm (B's tape patches read 11.5-15.9 mm) ... */
#define S3_PAIR_B_MIN_MM10 90
/* ... within this of each other (logs: 1-16 mm) = a crossing */
#define S3_PAIR_MM       25
#define S3_STUCK_ONE_MM  300
#define S3_STUCK_BOTH_MM 150
/** mm to odometer counts */
Inline int32_t mm_to_counts (int32_t dist_mm)
{
    return (dist_mm * 1000) / S2_UM_PER_COUNT;
}

/* Left arm under B, centre at cc (where the line sensors crossed it):
   no correction from cc - S3_ARM_BEFORE_MM to cc - S3_ARM_SELF_MM (L
   touching the arm itself is allowed).  Decided once the line sensors
   are S3_ARM_AFTER_MM past it (at once when B is behind). */
#define S3_ARM_BEFORE_MM 40
#define S3_ARM_SELF_MM   22 /* half a tape + the ~6 mm filter delay + margin */
#define S3_ARM_AFTER_MM  20

#define PAT_L 2u
#define PAT_R 1u

LOCAL volatile int32_t  g_steer;
LOCAL volatile int32_t  g_state = LINE_ON;
LOCAL volatile uint32_t g_junctions;
LOCAL volatile BOOL gb_enabled = FALSE; /* until a start (Mission or a test) */
LOCAL volatile int32_t  g_speed_pct = 100;
LOCAL volatile int32_t  g_junc_odo; /* last junction centre */
LOCAL volatile int32_t  g_junc_sides;
LOCAL volatile int32_t  g_fault_why;
LOCAL volatile uint32_t g_n_corr;  /* corrections since the start */
LOCAL volatile uint32_t g_n_sweep; /* sweeps right across the tape */

EXPORT int32_t line_steer_mm_s (void)
{
    return g_steer;
}
/** The line task's state (LINE_ON, LINE_SEARCH, LINE_LOST, LINE_FAULT). */
EXPORT int32_t line_state (void)
{
    return g_state;
}
/** Why the line task stopped with LINE_FAULT (LINE_FAULT_...), or 0. */
EXPORT int32_t line_fault (void)
{
    return g_fault_why;
}
/** Junctions seen since boot. */
EXPORT uint32_t line_junctions (void)
{
    return g_junctions;
}
/** Corrections begun and sweeps right across the tape since the start. */
EXPORT void line_wobble (uint32_t * p_corrections, uint32_t * p_sweeps)
{
    *p_corrections = g_n_corr;
    *p_sweeps      = g_n_sweep;
}
/** Turns junction and line-lost events on or off (off during turns and
 * bypasses). */
EXPORT void line_set_enabled (BOOL b_on)
{
    gb_enabled = b_on;
}
/** Share of the follow speed Motion may use now: 100 on the line, less
 * while correcting. */
EXPORT int32_t line_speed_pct (void)
{
    return g_speed_pct;
}
/** Odometer count where the line sensors crossed the last junction's
 * centre. */
EXPORT int32_t line_junction_odo (void)
{
    return g_junc_odo;
}
/** The last junction's branches (JUNC_SIDE_S, JUNC_SIDE_B). */
EXPORT int32_t line_junction_sides (void)
{
    return g_junc_sides;
}
/** TRUE while either line sensor sees black (raw). */
EXPORT BOOL line_any_black (void)
{
    return (IR_BLACK == app_gpio_get(PIN_IR_LINE_L)) ||
           (IR_BLACK == app_gpio_get(PIN_IR_LINE_R));
}

/* A junction centred at c (odometer counts).  One close to the last one
   is another view of the same junction. */
LOCAL int32_t g_last_c;
LOCAL BOOL    gb_have_last;

/** Reports a junction centred at odometer count centre (one close to the
 * last is the same junction). */
LOCAL void junction_at (int32_t centre, int32_t side)
{
    int32_t delta = centre - g_last_c;

    if (delta < 0)
    {
        delta = -delta;
    }

    if (gb_have_last && (delta < mm_to_counts(S3_JUNC_MERGE_MM)))
    {
        g_junc_sides |= side; /* same one: no second event */
        return;
    }

    g_last_c     = centre;
    gb_have_last = TRUE;
    g_junc_odo   = centre;
    g_junc_sides = side;
    g_junctions++;
    tk_set_flg(gh_flg_mission, MSN_FLG_JUNCTION);
}

/* Where the robot was correcting or searching: the last OFFS stretches,
   by odometer.  The newest is still open while it is. */
#define OFFS 16
LOCAL int32_t  g_off_a[OFFS];
LOCAL int32_t  g_off_b[OFFS];
LOCAL uint32_t g_off_n;
LOCAL BOOL     gb_off_open;
LOCAL int32_t  g_hist_from; /* nothing known before this */

/** Forgets the record of where the robot was correcting. */
LOCAL void hist_reset (int32_t odo)
{
    g_off_n     = 0;
    gb_off_open = FALSE;
    g_hist_from = odo;
}

/** Records where a correction (or search) starts and ends, by odometer. */
LOCAL void hist_step (int32_t odo, BOOL b_busy)
{
    if (b_busy && (!gb_off_open))
    {
        g_off_a[g_off_n % OFFS] = odo;
        g_off_n++;
        gb_off_open = TRUE;
    }
    else if ((!b_busy) && gb_off_open)
    {
        g_off_b[(g_off_n - 1) % OFFS] = odo;
        gb_off_open                   = FALSE;
    }
    else
    {
        /* nothing to do for any other case */
    }
}

/** TRUE if the robot drove straight (no correction) all of [a, b] */
LOCAL BOOL quiet_over (int32_t from_odo, int32_t to_odo, int32_t now)
{
    uint32_t idx;

    if (to_odo < g_hist_from)
    {
        return TRUE; /* before the start: the robot
    was put down straight */
    }

    if (from_odo < g_hist_from)
    {
        from_odo = g_hist_from;
    }

    if ((g_off_n > OFFS) && (from_odo < g_off_a[(g_off_n - OFFS) % OFFS]))
    {
        return FALSE; /* not on record */
    }

    for (idx = (g_off_n > OFFS) ? g_off_n - OFFS : 0; idx < g_off_n; idx++)
    {
        int32_t corr_from = g_off_a[idx % OFFS];
        int32_t corr_to =
            (gb_off_open && (idx == (g_off_n - 1))) ? now : g_off_b[idx % OFFS];

        if ((corr_from <= to_odo) && (corr_to >= from_odo))
        {
            return FALSE;
        }
    }

    return TRUE;
}

/** Sets up the two line sensor inputs. */
EXPORT ER line_hw_init (void)
{
    ER err = app_gpio_input(PIN_IR_LINE_L, FALSE); /* modules drive D0 */

    return (err < E_OK) ? err : app_gpio_input(PIN_IR_LINE_R, FALSE);
}

/** The line following task: steering from the two line sensors, junctions,
 * line lost. */
EXPORT void line_task (int32_t stacd, void * p_exinf)
{
    uint32_t  stable        = 0u;
    uint32_t  cand          = 0u; /* filtered pattern: PAT_L | PAT_R */
    int32_t   cand_n        = 0;
    int32_t   out           = 0;
    BOOL      b_was_enabled = FALSE;
    int32_t   last_side     = 0; /* +1 line last seen on the left, -1 right */
    int32_t   single_from   = 0; /* start of the current one-sensor stretch */
    BOOL      b_long_corr   = FALSE; /* the last correction was long */
    int32_t   white_from    = 0; /* start of the current all-white stretch */
    BOOL      b_searching   = FALSE;
    int32_t   search_from   = 0;
    int32_t   search_n      = 0;
    int32_t   both_from     = 0;
    uint32_t  before_both = 0u; /* the one-sensor pattern before a both-black */
    BOOL      b_bpend   = FALSE; /* B patch waiting until the sensors pass it */
    int32_t   probe     = 0;     /* probing for the tape: +1 left, -1 right */
    int32_t   probe_leg = 0;     /* 1 first way, 2 back the other way */
    int32_t   probe_from   = 0;
    BOOL      b_probe_none = FALSE; /* probed both ways: no tape */
    float32_t mem          = 0.0f;  /* steering memory, mm/s */
    int32_t   mem_odo      = 0;
    int32_t   bpend_c      = 0;
    int32_t   pair_c       = 0; /* centre of the last both-black, for B */
    BOOL      b_pair_ok    = FALSE;
    int32_t   mark_centre;
    int32_t   mark_width;
    uint32_t  mark_seen = barcode_mark(&mark_centre, &mark_width);

    (void)stacd;
    (void)p_exinf;
    hist_reset(motion_odometer());

    for (;;)
    {
        BOOL     b_raw_left;
        BOOL     b_raw_right;
        BOOL     b_left;
        BOOL     b_right;
        BOOL     b_single;
        int32_t  odo;
        int32_t  demand;
        int32_t  target;
        uint32_t pat;
        uint32_t prev;

        tk_slp_tsk(TMO_FEVR); /* woken every 20 ms */
        app_timing_wake(g_tmg_line, tk_can_wup(TSK_SELF));

        b_raw_left  = (IR_BLACK == app_gpio_get(PIN_IR_LINE_L));
        b_raw_right = (IR_BLACK == app_gpio_get(PIN_IR_LINE_R));
        odo         = motion_odometer();
        pat         = (b_raw_left ? PAT_L : 0u) | (b_raw_right ? PAT_R : 0u);

        if (gb_enabled && (!b_was_enabled))
        {
            /* a start, or after a manoeuvre: begin clean */
            stable = cand = pat;
            cand_n        = S3_FILTER_N;
            out           = 0;
            last_side     = 0;
            single_from = white_from = both_from = odo;
            b_long_corr = b_searching = b_bpend = b_pair_ok = FALSE;
            probe = probe_leg = 0;
            b_probe_none      = FALSE;
            mem               = 0.0f;
            mem_odo           = odo;
            before_both       = 0u;
            g_n_corr = g_n_sweep = 0u;
            hist_reset(odo);
            mark_seen =
                barcode_mark(&mark_centre, &mark_width); /* drop old patches */
            g_state     = LINE_ON;
            g_fault_why = 0;
        }

        b_was_enabled = gb_enabled;

        /* Filter: ignore a pattern that does not last S3_FILTER_N readings */
        prev = stable;

        if (pat == cand)
        {
            if (cand_n < S3_FILTER_N)
            {
                cand_n++;
            }
        }
        else
        {
            cand   = pat;
            cand_n = 1;
        }

        if (cand_n >= S3_FILTER_N)
        {
            stable = cand;
        }

        b_left   = (stable & PAT_L) != 0u;
        b_right  = (stable & PAT_R) != 0u;
        b_single = (b_left != b_right);

        /* Stretches start where the filtered pattern changed */
        if (stable != prev)
        {
            if (((PAT_L == prev) || (PAT_R == prev)) &&
                ((odo - single_from) >= mm_to_counts(S3_LONG_CORR_MM)))
            {
                b_long_corr = TRUE; /* a long correction ended */
            }
            else if ((PAT_L == prev) || (PAT_R == prev))
            {
                b_long_corr = FALSE;
            }
            else
            {
                /* nothing to do for any other case */
            }

            if (b_single)
            {
                single_from = odo;

                if (0u == prev)
                {
                    g_n_corr++;
                }

                /* L, both, R (or R, both, L): swept right across */
                if ((prev == (PAT_L | PAT_R)) && (before_both != 0u) &&
                    (before_both != stable))
                {
                    g_n_sweep++;
                }
            }

            if (0u == stable)
            {
                white_from = odo;
            }

            if (stable == (PAT_L | PAT_R))
            {
                both_from   = odo;
                before_both = ((PAT_L == prev) || (PAT_R == prev)) ? prev : 0u;
            }
        }

        if ((stable != 0u) && b_searching)
        {
            b_searching = FALSE; /* found it again */
            b_long_corr = FALSE;
        }

        if (b_single)
        {
            last_side = b_left ? +1 : -1;
        }

        /* Overshoot: white for a while right after a long correction */
        if ((!b_searching) && (0u == stable) && b_long_corr &&
            ((odo - white_from) >= mm_to_counts(S3_OVERSHOOT_MM)))
        {
            b_searching = TRUE;
            search_from = odo;
            search_n    = 0;
        }

        /* Long straight, all white: is the tape still there? */
        if ((stable != 0u) || b_searching)
        {
            probe = probe_leg = 0;
            b_probe_none      = FALSE;
        }
        else if ((0 == probe) && (!b_probe_none) &&
                 ((odo - white_from) >= mm_to_counts(S3_PROBE_AFTER_MM)))
        {
            probe      = (last_side >= 0) ? +1 : -1;
            probe_leg  = 1;
            probe_from = odo;
        }
        else if ((probe != 0) &&
                 ((odo - probe_from) >= mm_to_counts(S3_PROBE_MM * probe_leg)))
        {
            if (1 == probe_leg)
            {
                probe      = -probe; /* the other way, twice as far */
                probe_leg  = 2;
                probe_from = odo;
            }
            else
            {
                probe = probe_leg = 0;
                b_probe_none      = TRUE; /* no tape either way */
            }
        }
        else
        {
            /* nothing to do for any other case */
        }

        /* Steering (+ = turn left) */
        if (b_single)
        {
            demand = b_right ? -STEER_CORR : STEER_CORR;
        }
        else if (b_searching)
        {
            demand = (last_side >= 0) ? STEER_SEARCH : -STEER_SEARCH;
        }
        else if (probe != 0)
        {
            demand = probe * STEER_PROBE; /* looking for the tape */
        }
        else
        {
            /* centred, gap or crossing: keep part of the recent
               steering (a curve keeps turning) */
            float32_t mem_part = S3_MEM_K * mem;

            if (mem_part > (float32_t)S3_MEM_MAX)
            {
                mem_part = (float32_t)S3_MEM_MAX;
            }

            if (mem_part < -(float32_t)S3_MEM_MAX)
            {
                mem_part = -(float32_t)S3_MEM_MAX;
            }

            demand = (int32_t)mem_part;
        }

        /* Build a turn up gently (STEER_STEP per period), but ease off
           at once: ramping down too kept the robot turning after it
           had found the line again, and it swung across (2 Nov). */
        target = demand;

        if (((out > 0) && (target < 0)) || ((out < 0) && (target > 0)))
        {
            out = 0; /* other way: drop the old turn */
        }

        if (((out >= 0) && (target >= 0) && (target <= out)) ||
            ((out <= 0) && (target <= 0) && (target >= out)))
        {
            out = target; /* easing off */
        }
        else if (out < (target - STEER_STEP))
        {
            out += STEER_STEP;
        }
        else if (out > (target + STEER_STEP))
        {
            out -= STEER_STEP;
        }
        else
        {
            out = target;
        }

        g_steer = out;
        /* steering memory: average of the steering by distance */
        {
            int32_t step = odo - mem_odo;

            if (step > 0)
            {
                float32_t share =
                    (float32_t)motion_counts_to_mm(step) / (float32_t)S3_MEM_MM;

                if (share > 1.0f)
                {
                    share = 1.0f;
                }

                mem += ((float32_t)out - mem) * share;
            }

            mem_odo = odo;
        }
        /* Motion slows down while correcting or searching, so the same
           steering turns much tighter; a little in curves too */

        if (b_searching)
        {
            g_speed_pct = S3_SEARCH_PCT;
        }
        else
        {
            int32_t cap;

            if (b_single &&
                ((odo - single_from) >= mm_to_counts(S3_LOW_AFTER_MM)))
            {
                g_speed_pct = S3_LOW_PCT;
            }
            else if (b_single)
            {
                g_speed_pct = S3_CORR_PCT;
            }
            else
            {
                g_speed_pct = 100;
            }

            cap = 100 - (int32_t)(S3_MEM_SLOW * ((mem < 0.0f) ? -mem : mem));

            if (cap < 35)
            {
                cap = 35;
            }

            if (cap < g_speed_pct)
            {
                g_speed_pct = cap;
            }
        }

        /* the probe turns too: a crossing met while probing came at an
           angle (7 Nov: taken as a crossing) */
        hist_step(odo, b_single || b_searching || (probe != 0));

        if (!gb_enabled)
        {
            /* turning, bypassing: no events */
            b_bpend = FALSE;
            app_timing_done(g_tmg_line);
            continue;
        }

        /* ---- Lost, and stuck sensors ---- */
        if ((g_state != LINE_FAULT) && (g_state != LINE_LOST))
        {
            int32_t why = 0;

            if (b_searching)
            {
                g_state = LINE_SEARCH;

                if (((odo - search_from) > mm_to_counts(S3_LOST_MM)) ||
                    (++search_n > (S3_LOST_MS / PERIOD_LINE_MS)))
                {
                    g_state = LINE_LOST;
                }
            }
            else if (b_probe_none)
            {
                g_state = LINE_LOST;
            }
            else
            {
                g_state = LINE_ON;
            }

            if (b_single &&
                ((odo - single_from) > mm_to_counts(S3_STUCK_ONE_MM)))
            {
                why = LINE_FAULT_ONE;
            }
            else if ((stable == (PAT_L | PAT_R)) &&
                     ((odo - both_from) > mm_to_counts(S3_STUCK_BOTH_MM)))
            {
                why = LINE_FAULT_BOTH;
            }
            else
            {
                /* nothing to do for any other case */
            }

            if (why != 0)
            {
                g_fault_why = why;
                g_state     = LINE_FAULT;
            }

            if ((LINE_LOST == g_state) || (LINE_FAULT == g_state))
            {
                tk_set_flg(gh_flg_mission, MSN_FLG_LINE_LOST);
            }
        }

        /* ---- Crossing: the end of a "both black" ---- */
        if ((prev == (PAT_L | PAT_R)) && (stable != prev))
        {
            int32_t len = odo - both_from;

            if ((len >= mm_to_counts(S3_JUNC_MIN_MM)) &&
                (len <= mm_to_counts(S3_CROSS_MAX_MM)) &&
                (quiet_over(both_from - mm_to_counts(S3_CROSS_BEFORE_MM),
                            both_from - mm_to_counts(S3_CROSS_LEAD_MM), odo)))
            {
                junction_at(both_from + (len / 2), JUNC_SIDE_S);
            }
            else if ((len >= mm_to_counts(S3_PAIR_BOTH_MIN_MM)) &&
                     (len <= mm_to_counts(S3_CROSS_MAX_MM)))
            {
                /* while correcting: a crossing only if B agrees */
                pair_c    = both_from + (len / 2);
                b_pair_ok = TRUE;
            }
            else
            {
                /* nothing to do for any other case */
            }

            /* else swept across the line on a curve: not a crossing */
        }

        if (b_pair_ok &&
            ((odo - pair_c) > mm_to_counts(S3_PAIR_MM - S3_BAR_AHEAD_MM + 20)))
        {
            b_pair_ok = FALSE; /* B is past it and saw nothing */
        }

        /* ---- Left arm, from B: a patch about one tape wide.  cc is
           where the line sensors crossed the same place (B is
           S3_BAR_AHEAD_MM in front of them; negative = behind) ---- */
        {
            uint32_t seq = barcode_mark(&mark_centre, &mark_width);

            if (seq != mark_seen)
            {
                int32_t centre = mark_centre + mm_to_counts(S3_BAR_AHEAD_MM);
                int32_t gap    = centre - pair_c;

                mark_seen = seq;

                if (gap < 0)
                {
                    gap = -gap;
                }

                if (b_pair_ok && (gap <= mm_to_counts(S3_PAIR_MM)) &&
                    (mark_width >= S3_PAIR_B_MIN_MM10) &&
                    (mark_width <= (S3_JUNC_MAX_MM * 10)))
                {
                    junction_at(pair_c, JUNC_SIDE_S | JUNC_SIDE_B);
                    b_pair_ok = FALSE;
                }

                if ((mark_width >= (S3_JUNC_MIN_MM * 10)) &&
                    (mark_width <= (S3_JUNC_MAX_MM * 10)))
                {
                    b_bpend = TRUE;
                    bpend_c = mark_centre + mm_to_counts(S3_BAR_AHEAD_MM);
                }
            }
        }

        if (b_bpend && ((odo - bpend_c) >= mm_to_counts(S3_ARM_AFTER_MM)))
        {
            int32_t centre = bpend_c;

            b_bpend = FALSE;

            if (quiet_over(centre - mm_to_counts(S3_ARM_BEFORE_MM),
                           centre - mm_to_counts(S3_ARM_SELF_MM), odo))
            {
                junction_at(centre, JUNC_SIDE_B);
            }

            /* else on a curve or correcting there: not trusted */
        }

        app_timing_done(g_tmg_line);
    }
}

/*** end of file ***/

