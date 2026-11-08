/*
 *  task_motion.c
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

typedef struct
{
    INT type;
    ID  h_cmd_id;
    INT speed;  /* mm/s, sign = direction (drive) */
    INT amount; /* drive: mm; turn: deg (+ = left) */
} motion_cmd_t;

LOCAL motion_cmd_t    g_cmd_slot; /* written by callers, under APP_LOCK */
LOCAL BOOL            gb_cmd_new;
LOCAL ID              gh_cmd_last_id;
LOCAL motion_status_t g_status; /* written by the task, under APP_LOCK */

LOCAL ID cmd_post (INT type, INT speed, INT amount)
{
    UINT irq_state;
    ID   h_cmd;

    if ((type != CMD_HALT) && (app_stop_reasons() != 0u))
    {
        return E_OBJ;
    }
    APP_LOCK(irq_state);
    h_cmd               = ++gh_cmd_last_id;
    g_cmd_slot.type     = type;
    g_cmd_slot.h_cmd_id = h_cmd;
    g_cmd_slot.speed    = speed;
    g_cmd_slot.amount   = amount;
    gb_cmd_new          = TRUE;
    g_status.cmd_id     = (UW)h_cmd;
    g_status.result     = MOT_RES_BUSY;
    APP_UNLOCK(irq_state);
    return h_cmd;
}

EXPORT ID motion_drive (INT speed_mm_s, INT dist_mm)
{
    return cmd_post(CMD_DRIVE, speed_mm_s, dist_mm);
}

EXPORT ID motion_turn (INT deg)
{
    return cmd_post(CMD_TURN, S2_SPEED_TURN, deg);
}

EXPORT ID motion_turn_at (INT deg, INT wheel_mm_s)
{
    return cmd_post(CMD_TURN, wheel_mm_s, deg);
}

EXPORT ID motion_follow (INT speed_mm_s)
{
    return cmd_post(CMD_FOLLOW, speed_mm_s, 0);
}

LOCAL volatile INT g_follow_limit; /* 0 = none */

EXPORT void motion_follow_limit (INT speed_mm_s)
{
    g_follow_limit = speed_mm_s;
}

EXPORT void motion_halt (void)
{
    (void)cmd_post(CMD_HALT, 0, 0);
}

EXPORT void motion_get_status (motion_status_t * p_status)
{
    UINT irq_state;

    APP_LOCK(irq_state);
    *p_status = g_status;
    APP_UNLOCK(irq_state);
}

LOCAL float clampf (float value, float low, float high)
{
    return (value < low) ? low : ((value > high) ? high : value);
}

LOCAL float absf (float value)
{
    return (value < 0.0f) ? -value : value;
}

typedef struct
{
    float integ;
} wheel_pi_t;

LOCAL INT wheel_duty (wheel_pi_t * p_wheel, float target, float measured)
{
    float err = target - measured;
    float feedfwd;

    if (0.0f == target)
    {
        p_wheel->integ = 0.0f;
        return 0;
    }
    feedfwd =
        (FF_PER_MM_S * target) + ((target > 0.0f) ? FF_OFFSET : -FF_OFFSET);
    p_wheel->integ =
        clampf(p_wheel->integ + (KI * err * DT_S), -I_LIMIT, I_LIMIT);
    return (INT)clampf(feedfwd + (KP * err) + p_wheel->integ, -1000.0f,
                       1000.0f);
}

EXPORT void motion_task (INT stacd, void * p_exinf)
{
    W            hist_l[SPEED_AVG_N] = {0};
    W            hist_r[SPEED_AVG_N] = {0};
    INT          hist_pos            = 0;
    W            count_left;
    W            count_right;
    W            prev_l;
    W            prev_r;
    W            start_l = 0;
    W            start_r = 0;
    UW           invalid;
    motion_cmd_t cur        = {0};
    wheel_pi_t   pi_l       = {0};
    wheel_pi_t   pi_r       = {0};
    float        v_prof     = 0.0f; /* profile speed now, mm/s */
    UW           elapsed_ms = 0;
    UW           limit_ms   = 0;
    INT          result     = MOT_RES_NONE;
    UINT         irq_state;

    (void)stacd;
    (void)p_exinf;
    encoder_read(&prev_l, &prev_r, NULL);

    for (;;)
    {
        float vel_left;
        float vel_right;
        float dl_mm;
        float dr_mm;
        INT   duty_l = 0;
        INT   duty_r = 0;
        W     sum_l  = 0;
        W     sum_r  = 0;
        INT   idx;
        BOOL  b_fresh;

        tk_slp_tsk(TMO_FEVR); /* woken every 10 ms */
        app_timing_wake(g_tmg_motion, tk_can_wup(TSK_SELF));

        /* 1. Encoders and speed */
        encoder_read(&count_left, &count_right, &invalid);
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
        /* counts per 40 ms -> mm/s */
        vel_left =
            (float)sum_l * S2_UM_PER_COUNT / (1000.0f * SPEED_AVG_N * DT_S);
        vel_right =
            (float)sum_r * S2_UM_PER_COUNT / (1000.0f * SPEED_AVG_N * DT_S);

        /* Take a new command (it replaces a running one) */
        b_fresh = FALSE;
        APP_LOCK(irq_state);
        if (gb_cmd_new)
        {
            cur        = g_cmd_slot;
            gb_cmd_new = FALSE;
            b_fresh    = TRUE;
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
                limit_ms = (UW)(2000 * absf((float)cur.amount) /
                                absf((float)cur.speed)) +
                           1000;
            }
            else if (CMD_TURN == cur.type)
            {
                limit_ms =
                    (UW)(2000.0f * 3.14159f * S2_TRACK_MM *
                         absf((float)cur.amount) / 360.0f / (float)cur.speed) +
                    1000;
            }
            else if (CMD_FOLLOW == cur.type)
            {
                limit_ms = 0; /* runs until replaced */
            }
            else
            { /* HALT */
                result   = MOT_RES_DONE;
                cur.type = CMD_NONE;
            }
        }

        /* 3. Run the command */
        if (CMD_FOLLOW == cur.type)
        {
            float steer = (float)line_steer_mm_s(); /* + = steer left */
            float t_l;
            float t_r;
            float cap   = absf((float)cur.speed);
            INT   pct   = line_speed_pct(); /* 100 = on track */
            BOOL  b_off = (pct < 100);

            if ((g_follow_limit > 0) && (cap > (float)g_follow_limit))
            {
                cap = (float)g_follow_limit;
            }

            /* Straight while B is over the bars (an edge in the last
               15 mm): B is 85 mm off centre, so a turn changes its
               speed over the bars against the odometer by 50 % or
               more and the widths come out wrong (2 Nov the steering
               broke a read into pieces).  The code is read in one
               pass; the back-up-and-reread (2-7 Nov) was removed on
               7 Nov. */
            if (barcode_in_bars())
            {
                steer = 0.0f;
                b_off = FALSE;
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
                cap = cap * (float)pct / 100.0f;
            }
            if ((barcode_busy()) && (cap > (float)S2_SPEED_IN_BARCODE))
            {
                cap = (float)S2_SPEED_IN_BARCODE;
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
                if (t_l < 0.0f)
                {
                    t_l = 0.0f;
                }
                if (t_r < 0.0f)
                {
                    t_r = 0.0f;
                }
            }
            duty_l = wheel_duty(&pi_l, t_l, vel_left);
            duty_r = wheel_duty(&pi_r, t_r, vel_right);
            motor_write(duty_l, duty_r);
        }
        else if ((CMD_DRIVE == cur.type) || (CMD_TURN == cur.type))
        {
            float target;
            float done;
            float remain;
            float v_cap;
            float done_left;
            float done_right;
            float corr;

            elapsed_ms += 10;
            dl_mm = (float)(count_left - start_l) * S2_UM_PER_COUNT / 1000.0f;
            dr_mm = (float)(count_right - start_r) * S2_UM_PER_COUNT / 1000.0f;

            if (CMD_DRIVE == cur.type)
            {
                target     = absf((float)cur.amount);
                done_left  = (cur.speed >= 0) ? dl_mm : -dl_mm;
                done_right = (cur.speed >= 0) ? dr_mm : -dr_mm;
            }
            else
            {
                /* spin: + = left = left wheel back, right forward */
                target =
                    3.14159f * S2_TRACK_MM * absf((float)cur.amount) / 360.0f;
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
            if (v_prof > absf((float)cur.speed))
            {
                v_prof = absf((float)cur.speed);
            }
            if (v_prof > v_cap)
            {
                v_prof = v_cap;
            }

            /* Keep both wheels at the same distance */
            corr = KEEP_EQUAL * (done_left - done_right);
            {
                float t_l = v_prof - corr;
                float t_r = v_prof + corr;
                float m_l;
                float m_r;

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
            if (CMD_DRIVE == cur.type)
            {
                if (cur.speed < 0)
                {
                    duty_l = -duty_l;
                    duty_r = -duty_r;
                }
            }
            else if (cur.amount >= 0)
            {
                duty_l = -duty_l;
            }
            else
            {
                duty_r = -duty_r;
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
        { /* a newer command keeps its BUSY */
            g_status.result = result;
        }
        g_status.count_l = count_left;
        g_status.count_r = count_right;
        g_status.speed_l = (INT)vel_left;
        g_status.speed_r = (INT)vel_right;
        g_status.invalid = invalid;
        APP_UNLOCK(irq_state);

        app_timing_done(g_tmg_motion);
    }
}

/*** end of file ***/

