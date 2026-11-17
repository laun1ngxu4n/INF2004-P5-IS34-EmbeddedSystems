/**
 * @file    motion.c
 *  Subsystem 2 -- the Motion Control task.
 *
 *  Every 10 ms:
 *    1. read both encoders, measure each wheel's speed (40 ms average);
 *    2. if any STOP reason is set: motors off, cancel the command;
 *    3. otherwise run the command: a speed profile (ramp up, cruise,
 *       slow down before the end), a correction that keeps both wheels
 *       at the same distance (straight lines, clean spins), and a speed
 *       controller per wheel (feed-forward + PI) that writes the PWM.
 *
 *  Commands come through a one-slot mailbox under APP_LOCK: the newest
 *  command replaces the old one.  Results are read with
 *  motion_get_status().
 */

#include "subsystem2.h"
#include "../app_stop.h"
#include "../app_math.h"
#include "../subsystem3/subsystem3.h" /* IF-03: line_steer_mm_s() */
#include "motion.h"

#define CMD_NONE   0
#define CMD_DRIVE  1
#define CMD_TURN   2
#define CMD_HALT   3
#define CMD_FOLLOW 4

#define DT_S        0.010f /* task period */
#define SPEED_AVG_N 4      /* speed over the last 4 periods */
#define ACCEL_MM_S2 400.0f /* ramp up and down */
#define V_MIN_MM_S  50.0f  /* slowest speed near the end */
#define KEEP_EQUAL  3.0f   /* mm/s per mm the wheels differ */

/* Per wheel speed controller, duty in per mille.  Feed-forward from the
   bring-up runs (wheels lifted: about 1.25 per mille per mm/s, through 0)
   plus an offset for the load on the floor; PI takes up the rest. */
#define FF_PER_MM_S 1.25f
#define FF_OFFSET   80.0f
#define KP          1.0f /* per mille per mm/s error */
#define KI          6.0f /* per mille per (mm/s * s) */
#define I_LIMIT     400.0f

/* A command (Change Requirement 1: each field in the bits its range
   needs; cmd_post() clamps into range).  The command ID stays an ID: it
   only ever goes up, and a narrow ID that wrapped could let an old
   result count as the new command's (Tribunal 1, R5). */
typedef struct
{
    uint32_t type  APP_FIELD(3); /* CMD_NONE .. CMD_FOLLOW (5) */
    ID             h_cmd_id;
    int32_t speed  APP_FIELD(11); /* mm/s, +-1023, sign = direction */
    int32_t amount APP_FIELD(14); /* drive: mm, turn: deg (+ = left),
                                     +-8191 */
} motion_cmd_t;

LOCAL motion_cmd_t    g_cmd_slot; /* written by callers, under APP_LOCK */
LOCAL bool            gb_cmd_new;
LOCAL ID              gh_cmd_last_id;
LOCAL motion_status_t g_status; /* written by the task, under APP_LOCK */

/** Hands a new command to the motion task; its handle. */
LOCAL ID cmd_post (int32_t type, int32_t speed, int32_t amount)
{
    uint32_t irq_state;
    ID       h_cmd;

    if ((type != CMD_HALT) && (app_stop_reasons() != 0u))
    {
        return E_OBJ;
    }

    /* a drive or turn at speed 0 would never end: its time limit (distance
       / speed) could not be worked out */

    if (((CMD_DRIVE == type) || (CMD_TURN == type)) && (0 == speed))
    {
        return E_PAR;
    }

    APP_LOCK(irq_state);
    h_cmd               = ++gh_cmd_last_id;
    g_cmd_slot.type     = type;
    g_cmd_slot.h_cmd_id = h_cmd;
    g_cmd_slot.speed    = app_clamp(speed, -1023, 1023);
    g_cmd_slot.amount   = app_clamp(amount, -8191, 8191);
    gb_cmd_new          = true;
    g_status.h_cmd_id   = h_cmd;
    g_status.result     = MOT_RES_BUSY;
    APP_UNLOCK(irq_state);
    return h_cmd;
}

EXPORT ID motion_drive (int32_t speed_mm_s, int32_t dist_mm)
{
    return cmd_post(CMD_DRIVE, speed_mm_s, dist_mm);
}

/** Spins on the spot by deg degrees (+ = left); its handle. */
EXPORT ID motion_turn (int32_t deg)
{
    return cmd_post(CMD_TURN, S2_SPEED_TURN, deg);
}

/** Spins on the spot by deg degrees at a given wheel speed; its handle. */
EXPORT ID motion_turn_at (int32_t deg, int32_t wheel_mm_s)
{
    return cmd_post(CMD_TURN, wheel_mm_s, deg);
}

/** Follows the line at speed mm/s until another command. */
EXPORT ID motion_follow (int32_t speed_mm_s)
{
    return cmd_post(CMD_FOLLOW, speed_mm_s, 0);
}

/* mm/s, 0 = none; +-1023 (11 bits), its own halfword: Mission writes it,
   the motion task reads it (Change Requirement 1) */
LOCAL volatile int16_t g_follow_limit;

EXPORT void motion_follow_limit (int32_t speed_mm_s)
{
    /* clamped to +-1023 first: fits int16_t */
    g_follow_limit = (int16_t)app_clamp(speed_mm_s, -1023, 1023);
}

/** Stops the wheels (no STOP reason). */
EXPORT void motion_halt (void)
{
    (void)cmd_post(CMD_HALT, 0, 0);
}

/** Copies the result of the motion task's last command. */
EXPORT void motion_get_status (motion_status_t * p_status)
{
    uint32_t irq_state;

    APP_LOCK(irq_state);
    *p_status = g_status;
    APP_UNLOCK(irq_state);
}

#if APP_TEST
/* S4-S01: ms the motion task stays away at its next wake, 0 = none; one
   word: the test task writes it, the motion task reads and clears it */
LOCAL volatile uint32_t g_test_hang_ms;

/** Test images only: the motion task stops running for ms (S4-S01). */
EXPORT void motion_test_hang (uint32_t ms)
{
    g_test_hang_ms = ms;
}
#endif

/** value limited to low..high. */
LOCAL float32_t clampf (float32_t value, float32_t low, float32_t high)
{
    return (value < low) ? low : ((value > high) ? high : value);
}

/** Absolute value of a float. */
LOCAL float32_t absf (float32_t value)
{
    return (value < 0.0f) ? -value : value;
}

/** The angle a spin of deg is driven to: a big spin stops S2_TURN_TRIM_DEG
 * early (it turns that much more on the mat, 9 Nov), a small one does not. */
LOCAL float32_t spin_target_deg (int32_t deg)
{
    float32_t angle = absf((float32_t)deg); /* |deg| < 2^13: exact */

    /* small whole degrees: exact as floats */
    if (angle >= (float32_t)S2_TURN_TRIM_MIN_DEG)
    {
        angle -= (float32_t)S2_TURN_TRIM_DEG; /* exact */
    }

    return angle;
}

typedef struct
{
    float32_t integ;
} wheel_pi_t;

/** Motor duty for one wheel: feed-forward plus PI on its speed. */
LOCAL int32_t wheel_duty (wheel_pi_t * p_wheel, float32_t target,
                          float32_t measured)
{
    float32_t err = target - measured;
    float32_t feedfwd;

    if (0.0f == target)
    {
        p_wheel->integ = 0.0f;
        return 0;
    }

    feedfwd =
        (FF_PER_MM_S * target) + ((target > 0.0f) ? FF_OFFSET : -FF_OFFSET);
    p_wheel->integ =
        clampf(p_wheel->integ + (KI * err * DT_S), -I_LIMIT, I_LIMIT);
    /* clamped to -1000..1000 first: fits int32_t */
    return (int32_t)clampf(feedfwd + (KP * err) + p_wheel->integ, -1000.0f,
                           1000.0f);
}

/** The motion task: runs drive, turn and follow commands with wheel speed
 * control. */
EXPORT void motion_task (int32_t stacd, void * p_exinf)
{
    int32_t      hist_l[SPEED_AVG_N] = {0};
    int32_t      hist_r[SPEED_AVG_N] = {0};
    int32_t      hist_pos            = 0;
    int32_t      count_left;
    int32_t      count_right;
    int32_t      prev_l;
    int32_t      prev_r;
    int32_t      start_l = 0;
    int32_t      start_r = 0;
    uint32_t     invalid;
    motion_cmd_t cur        = {0};
    wheel_pi_t   pi_l       = {0};
    wheel_pi_t   pi_r       = {0};
    float32_t    v_prof     = 0.0f; /* profile speed now, mm/s */
    uint32_t     elapsed_ms = 0;
    uint32_t     limit_ms   = 0;
    int32_t      result     = MOT_RES_NONE;
    uint32_t     irq_state;

    (void)stacd;
    (void)p_exinf;
    motor_encoders(&prev_l, &prev_r, NULL);

    for (;;)
    {
        float32_t vel_left;
        float32_t vel_right;
        float32_t dl_mm;
        float32_t dr_mm;
        int32_t   duty_l = 0;
        int32_t   duty_r = 0;
        int32_t   sum_l  = 0;
        int32_t   sum_r  = 0;
        int32_t   idx;
        bool      b_fresh;

        tk_slp_tsk(TMO_FEVR); /* woken every 10 ms */
        app_timing_wake(g_tmg_motion, tk_can_wup(TSK_SELF));

#if APP_TEST
        if (g_test_hang_ms != 0u)
        {
            /* a few hundred ms: fits TMO */
            const TMO hang_ms = (TMO)g_test_hang_ms;

            g_test_hang_ms = 0u;
            tk_dly_tsk(hang_ms); /* "hung": the motors keep their duty */
        }
#endif

        /* 1. Encoders and speed */
        motor_encoders(&count_left, &count_right, &invalid);
        hist_l[hist_pos] = count_left - prev_l;
        hist_r[hist_pos] = count_right - prev_r;
        hist_pos         = (hist_pos + 1) % SPEED_AVG_N;
        prev_l           = count_left;
        prev_r           = count_right;

        for (idx = 0; idx < SPEED_AVG_N; idx++)
        {
            sum_l += hist_l[idx];
            sum_r += hist_r[idx];
        }

        /* counts per 40 ms -> mm/s (a few hundred counts: exact as
           floats) */
        vel_left =
            (float32_t)sum_l * S2_UM_PER_COUNT / (1000.0f * SPEED_AVG_N * DT_S);
        /* the same for the right wheel */
        vel_right =
            (float32_t)sum_r * S2_UM_PER_COUNT / (1000.0f * SPEED_AVG_N * DT_S);

        /* Take a new command (it replaces a running one) */
        b_fresh = false;
        APP_LOCK(irq_state);

        if (gb_cmd_new)
        {
            cur        = g_cmd_slot;
            gb_cmd_new = false;
            b_fresh    = true;
        }

        APP_UNLOCK(irq_state);

        /* 2. STOP wins over everything */
        if (app_stop_reasons() != 0u)
        {
            motor_safe_off();

            if (cur.type != CMD_NONE)
            {
                result   = MOT_RES_STOPPED;
                cur.type = CMD_NONE;
            }

            pi_l.integ = pi_r.integ = 0.0f;
            v_prof                  = 0.0f;
            goto publish;
        }

        /* Start the new command from where the wheels are now */
        if (b_fresh)
        {
            start_l    = count_left;
            start_r    = count_right;
            elapsed_ms = 0;
            v_prof     = 0.0f;
            pi_l.integ = pi_r.integ = 0.0f;
            result                  = MOT_RES_BUSY;

            if (CMD_DRIVE == cur.type)
            {
                /* mm and mm/s: small, exact as floats; 2 x the expected ms
                   is positive and far below 2^32 (cmd_post refuses a speed
                   of 0) */
                limit_ms = (uint32_t)(2000 * absf((float32_t)cur.amount) /
                                      absf((float32_t)cur.speed)) +
                           1000;
            }
            else if (CMD_TURN == cur.type)
            {
                /* degrees and mm/s: small, exact as floats; 2 x the expected
                   ms is positive and far below 2^32 (cmd_post refuses a
                   speed of 0) */
                limit_ms = (uint32_t)(2000.0f * 3.14159f * S2_TURN_TRACK_MM *
                                      absf((float32_t)cur.amount) / 360.0f /
                                      absf((float32_t)cur.speed)) +
                           1000;
            }
            else if (CMD_FOLLOW == cur.type)
            {
                limit_ms = 0; /* runs until replaced */
            }
            else
            {
                /* HALT */
                result   = MOT_RES_DONE;
                cur.type = CMD_NONE;
            }
        }

        /* 3. Run the command */
        if (CMD_FOLLOW == cur.type)
        {
            float32_t steer = (float32_t)line_steer_mm_s(); /* + = steer left */
            float32_t t_l;
            float32_t t_r;
            /* speeds are mm/s, |v| <= 1000: exact as floats (here and below) */
            float32_t cap   = absf((float32_t)cur.speed);
            int32_t   pct   = line_speed_pct(); /* 100 = on track */
            bool      b_off = (pct < 100);

            if ((g_follow_limit > 0) &&
                (cap > (float32_t)g_follow_limit)) /* mm/s */
            {
                cap = (float32_t)g_follow_limit; /* exact */
            }

            /* Straight while B is over the bars (until the code ends,
               15 Nov; was the last 15 mm): B is 85 mm off centre, so a turn
               changes its speed over the bars against the odometer by 50 % or
               more and the widths come out wrong (2 Nov the steering
               broke a read into pieces).  The code is read in one
               pass; the back-up-and-reread (2-7 Nov) was removed on
               7 Nov. */

            if (barcode_in_bars())
            {
                steer = 0.0f;
                b_off = false;
                pct   = 100;
            }

            /* Correcting: forward speed drops to what the line task
               asks (a long correction 60 %, then 35 %, a search 20 %),
               so the same steering turns much tighter and the robot
               does not run past the line (Stage 2 run, 7 Nov: the tape
               slipped out past the sensor on tight left curves).  Over a
               barcode: slow. */

            if (b_off)
            {
                cap = cap * (float32_t)pct / 100.0f; /* 0..100: exact */
            }

            /* S2_SPEED_IN_BARCODE, 50 mm/s: exact as a float */
            if ((barcode_busy()) && (cap > (float32_t)S2_SPEED_IN_BARCODE))
            {
                cap = (float32_t)S2_SPEED_IN_BARCODE; /* exact */
            }

            v_prof += ACCEL_MM_S2 * DT_S;

            if (v_prof > cap)
            {
                v_prof = cap;
            }

            t_l = v_prof - steer;
            t_r = v_prof + steer;

            if (!b_off)
            {
                /* on the line: never run a wheel backwards */
                t_l = (t_l < 0.0f) ? 0.0f : t_l;
                t_r = (t_r < 0.0f) ? 0.0f : t_r;
            }

            duty_l = wheel_duty(&pi_l, t_l, vel_left);
            duty_r = wheel_duty(&pi_r, t_r, vel_right);
            motor_write(duty_l, duty_r);
        }
        else if ((CMD_DRIVE == cur.type) || (CMD_TURN == cur.type))
        {
            float32_t target;
            float32_t done;
            float32_t remain;
            float32_t v_cap;
            float32_t done_left;
            float32_t done_right;
            float32_t corr;

            elapsed_ms += 10;
            /* counts since the command started: exact as a float up to 2^24
               (1.3 km) */
            dl_mm =
                (float32_t)(count_left - start_l) * S2_UM_PER_COUNT / 1000.0f;
            /* the same for the right wheel */
            dr_mm =
                (float32_t)(count_right - start_r) * S2_UM_PER_COUNT / 1000.0f;

            if (CMD_DRIVE == cur.type)
            {
                target     = absf((float32_t)cur.amount); /* mm: exact */
                done_left  = (cur.speed >= 0) ? dl_mm : -dl_mm;
                done_right = (cur.speed >= 0) ? dr_mm : -dr_mm;
            }
            else
            {
                /* spin: + = left = left wheel back, right forward */
                target     = 3.14159f * S2_TURN_TRACK_MM *
                             spin_target_deg(cur.amount) / 360.0f;
                done_left  = (cur.amount >= 0) ? -dl_mm : dl_mm;
                done_right = (cur.amount >= 0) ? dr_mm : -dr_mm;
            }

            done   = 0.5f * (done_left + done_right);
            remain = target - done;

            if ((remain <= 0.0f) || (elapsed_ms > limit_ms))
            {
                motor_safe_off();
                result     = (remain <= 0.0f) ? MOT_RES_DONE : MOT_RES_TIMEOUT;
                cur.type   = CMD_NONE;
                pi_l.integ = pi_r.integ = 0.0f;
                v_prof                  = 0.0f;
                goto publish;
            }

            /* Profile: ramp up, never faster than can stop in 'remain' */
            v_cap = app_sqrtf(2.0f * ACCEL_MM_S2 * remain);

            if (v_cap < V_MIN_MM_S)
            {
                v_cap = V_MIN_MM_S;
            }

            v_prof += ACCEL_MM_S2 * DT_S;

            if (v_prof > absf((float32_t)cur.speed)) /* exact */
            {
                v_prof = absf((float32_t)cur.speed); /* exact */
            }

            if (v_prof > v_cap)
            {
                v_prof = v_cap;
            }

            /* Keep both wheels at the same distance */
            corr = KEEP_EQUAL * (done_left - done_right);
            {
                float32_t t_l = v_prof - corr;
                float32_t t_r = v_prof + corr;
                float32_t m_l;
                float32_t m_r;

                if (t_l < 0.0f)
                {
                    t_l = 0.0f;
                }

                if (t_r < 0.0f)
                {
                    t_r = 0.0f;
                }

                /* measured speeds in the same "along the move" sense */
                if (CMD_DRIVE == cur.type)
                {
                    m_l = (cur.speed >= 0) ? vel_left : -vel_left;
                    m_r = (cur.speed >= 0) ? vel_right : -vel_right;
                }
                else
                {
                    m_l = (cur.amount >= 0) ? -vel_left : vel_left;
                    m_r = (cur.amount >= 0) ? vel_right : -vel_right;
                }

                duty_l = wheel_duty(&pi_l, t_l, m_l);
                duty_r = wheel_duty(&pi_r, t_r, m_r);
            }

            /* back to wheel directions */
            if ((CMD_DRIVE == cur.type) && (cur.speed < 0))
            {
                duty_l = -duty_l; /* reversing */
                duty_r = -duty_r;
            }
            else if (CMD_DRIVE == cur.type)
            {
                /* driving forward: already the wheels' directions */
            }
            else if (cur.amount >= 0)
            {
                duty_l = -duty_l; /* spin left: left wheel back */
            }
            else
            {
                duty_r = -duty_r; /* spin right: right wheel back */
            }

            motor_write(duty_l, duty_r);
        }
        else
        {
            motor_write(0, 0);
        }

    publish:
        APP_LOCK(irq_state);

        if (!gb_cmd_new)
        {
            /* a newer command keeps its BUSY */
            g_status.result = result;
        }

        g_status.count_l = count_left;
        g_status.count_r = count_right;
        /* measured speeds, |v| < 2000 mm/s: fit int32_t */
        g_status.speed_l = app_clamp((int32_t)vel_left, -2047, 2047);
        g_status.speed_r =
            app_clamp((int32_t)vel_right, -2047, 2047); /* the same */
        g_status.invalid = invalid;
        APP_UNLOCK(irq_state);

        app_timing_done(g_tmg_motion);
    }
}

/*** end of file ***/
