/*
 *  task_obstacle.c
 *  Subsystem 5 -- the Obstacle Scanner task (every 60 ms) and the
 *  profiling scan.
 *
 *  Forward watch: one ping per 60 ms with the servo straight ahead; two
 *  readings in a row under S5_TRIP_MM tell Mission (MSN_FLG_OBSTACLE).
 *
 *  Profiling scan (TEAM.md 7.2), robot standing still:
 *    O1  servo settle wait after every move (3 ms per degree, >= 60 ms)
 *    O2  5 pings per ray (60 ms apart), median; >= 3 missing = open space
 *    O3  background: a ray farther than front + BG_MM is open space
 *    O4  an edge needs 2 rays in a row off the obstacle (hysteresis:
 *        on the obstacle while within IN_MM of its front distance)
 *    O6  width = left edge - right edge, with a +/- from the angle step,
 *        the beam and (O5, obstacle_combine) the spread of two scans.
 */

#include "subsystem5.h"
#include "../app_math.h"

#define PIN_TRIG    6
#define PIN_ECHO    7
#define PIN_SERVO   14 /* PWM slice 7 A */
#define SERVO_SLICE 7

#define SCAN_FROM     (90 - S5_SERVO_HALF_DEG) /* servo degrees */
#define SCAN_TO       (90 + S5_SERVO_HALF_DEG)
#define SCAN_STEP     5
#define NRAYS         ((SCAN_TO - SCAN_FROM) / SCAN_STEP + 1)
#define PINGS         5
#define SETTLE_MS_DEG 3
#define SETTLE_MIN_MS 60
#define IN_MM         150 /* on the obstacle while |d - front| <= this */
/* Geometry rule (8 Nov).  A flat face square to the robot is at
   front / cos(angle) along a ray at that angle.  The HC-SR04 also hears
   that face's echo from straight in front through the side of its beam,
   so off to the side it often reports about the straight-ahead distance
   while pointing past the box: whole +-40 deg sweeps read as "box" (7 Nov,
   400-500 mm widths for 15-23 cm boxes).  A ray counts as on the box only
   when its distance fits front / cos(angle); one much nearer is that
   side-of-beam echo: neither the box nor something else.  Checked on the
   21 scans of 7 Nov (tools/logcheck/sonarcheck.py): both edges found in 21 (the
   old rule 6), the clean scans unchanged (218 / 138 / 168 mm for 20 / 14.5 / 15
   cm boxes). */
#define GEO_TOL_MM   15
#define GEO_TOL_PCT  4
#define CENTRE_DEG   10  /* within this, any echo near front is the box */
#define BG_MM        250 /* open space beyond front + this */
#define FAR          SONAR_OPEN
#define MAX_FRONT_MM 600 /* nothing nearer = no obstacle */
#define BEAM_UNC_DEG 3   /* +/- on the beam correction */

LOCAL volatile uint32_t g_echo_rise_us;
LOCAL volatile uint32_t g_echo_width_us;
LOCAL volatile BOOL     gb_echo_rose;
LOCAL volatile BOOL     gb_echo_done;
LOCAL volatile BOOL     gb_watch;
LOCAL volatile BOOL gb_idle_ping = TRUE; /* Mission IDLE: ping for the panel */
LOCAL volatile BOOL gb_scanning;
LOCAL volatile int32_t g_front_mm = FAR;
LOCAL volatile BOOL    gb_sensor_fault;
LOCAL int32_t          g_servo_now = 90;

/* ECHO edges: interrupt context */
LOCAL void sonar_echo_isr (uint32_t pin, uint32_t events)
{
    uint32_t now = app_time_us();

    if (app_gpio_get(pin) != 0u)
    {
        g_echo_rise_us = now;
        gb_echo_rose   = TRUE;
    }
    else if (gb_echo_rose)
    {
        g_echo_width_us = now - g_echo_rise_us;
        gb_echo_done    = TRUE;
    }
    else
    {
        /* nothing to do for any other case */
    }

    (void)events;
}

LOCAL void trigger (void)
{
    uint32_t t_start;

    gb_echo_rose = FALSE;
    gb_echo_done = FALSE;
    gpio_set_val(PIN_TRIG, 1);
    t_start = app_time_us();

    while ((app_time_us() - t_start) < 12u)
    {
        ;
    }

    gpio_set_val(PIN_TRIG, 0);
}

/* Result of the last trigger: mm, FAR (nothing in range) or SONAR_FAULT */
LOCAL int32_t echo_mm (void)
{
    if (!gb_echo_rose)
    {
        return SONAR_FAULT; /* ECHO never rose */
    }

    if ((!gb_echo_done) || (g_echo_width_us < 100u) ||
        (g_echo_width_us > 25000u))
    {
        return FAR;
    }

    return (int32_t)(g_echo_width_us * 174u /
                     1000u); /* 0.348 mm/us at 28 C, / 2 */
}

EXPORT void sonar_ping_start (void)
{
    trigger();
}
EXPORT int32_t sonar_ping_result (void)
{
    return echo_mm();
}

EXPORT int32_t sonar_ping_mm (void)
{
    int32_t dist_mm;

    trigger();
    tk_dly_tsk(60);
    dist_mm = echo_mm();
    /* straight ahead: also what the panel shows.  Test images do not
       idle-ping (their tests ping themselves), so the panel's distance
       stayed frozen through Stage 3 (7 Nov) */

    if (90 == g_servo_now)
    {
        g_front_mm = dist_mm;
    }

    return dist_mm;
}

LOCAL void servo_write (int32_t deg)
{
    int32_t pulse_us = 1500 + ((deg - 90) * S5_SERVO_US_PER_DEG *
                               (S5_SERVO_LEFT_HIGH ? 1 : -1));

    pwm_set_cc(PIN_SERVO, (uint32_t)pulse_us); /* 1 count = 1 us */
    g_servo_now = deg;
}

/* Walk there S5_SERVO_DEG_STEP per 20 ms frame: a full-speed jump broke
   the servo horn */
/* turning limit: the scan's, or the side look's if that is larger */
#define SERVO_LIMIT                                              \
    ((S5_SERVO_SIDE_DEG > S5_SERVO_HALF_DEG) ? S5_SERVO_SIDE_DEG \
                                             : S5_SERVO_HALF_DEG)

EXPORT void servo_set_angle (int32_t deg)
{
    if (deg < (90 - SERVO_LIMIT))
    {
        deg = 90 - SERVO_LIMIT;
    }

    if (deg > (90 + SERVO_LIMIT))
    {
        deg = 90 + SERVO_LIMIT;
    }

    while (g_servo_now != deg)
    {
        int32_t next = g_servo_now + ((deg > g_servo_now) ? S5_SERVO_DEG_STEP
                                                          : -S5_SERVO_DEG_STEP);

        if ((deg > g_servo_now) ? (next > deg) : (next < deg))
        {
            next = deg;
        }

        servo_write(next);
        tk_dly_tsk(20);
    }
}

EXPORT void servo_look_side (int32_t side)
{
    int32_t left = S5_SERVO_LEFT_HIGH ? 1 : -1;

    servo_set_angle(90 + (side * left * S5_SERVO_SIDE_DEG));
}

EXPORT void obstacle_watch (BOOL b_on)
{
    gb_watch = b_on;
}
EXPORT void obstacle_idle_ping (BOOL b_on)
{
    gb_idle_ping = b_on;
}
EXPORT int32_t obstacle_front_mm (void)
{
    return g_front_mm;
}
EXPORT BOOL obstacle_sensor_fault (void)
{
    return gb_sensor_fault;
}

EXPORT ER obstacle_hw_init (void)
{
    ER err;

    app_hw_release(RESETS_RESET_PWM);
    /* servo: 125 MHz / 125 = 1 MHz, wrap 20000 = 50 Hz */
    out_w(PWM_BASE + (SERVO_SLICE * 0x14) + PWM_CHx_DIV, 125u << 4);
    pwm_set_wrap(PIN_SERVO, 19999);
    pwm_set_cc(PIN_SERVO, 1500);
    pwm_set_pin(PIN_SERVO);
    pwm_set_enabled(PIN_SERVO, TRUE);

    err = app_gpio_output(PIN_TRIG);

    if (err < E_OK)
    {
        return err;
    }

    err = app_gpio_input(PIN_ECHO, FALSE);

    if (err < E_OK)
    {
        return err;
    }

    clr_w(GPIO(PIN_ECHO), GPIO_PUE);
    set_w(GPIO(PIN_ECHO), GPIO_PDE); /* as in bring-up */
    return app_gpio_irq_add(PIN_ECHO, APP_GPIO_BOTH, sonar_echo_isr);
}

/* ---- maths without libm -------------------------------------------- */

LOCAL int32_t median5 (int32_t * p_values, int32_t count)
{
    int32_t idx;
    int32_t jdx;
    int32_t tmp;

    for (idx = 1; idx < count; idx++)
    {
        for (jdx = idx; (jdx > 0) && (p_values[jdx] < p_values[jdx - 1]); jdx--)
        {
            tmp               = p_values[jdx];
            p_values[jdx]     = p_values[jdx - 1];
            p_values[jdx - 1] = tmp;
        }
    }

    return p_values[count / 2];
}

/* ---- the profiling scan --------------------------------------------- */

/* One side: walk outward from straight ahead.  dir = +1 / -1 is the
   direction in the ray array (which side that is depends on the servo).
   Returns the lateral edge (mm, signed) and its half-step uncertainty. */
LOCAL void side_edge (const int32_t * p_dist, const int32_t * p_phi,
                      int32_t centre, int32_t dir, int32_t front,
                      int32_t * p_edge_mm, int32_t * p_unc_mm, BOOL * pb_clear,
                      BOOL * pb_seen)
{
    int32_t idx       = centre;
    int32_t last_in   = centre;
    int32_t out_run   = 0;
    int32_t first_out = -1;
    BOOL    b_other   = FALSE;

    for (idx = centre + dir; (idx >= 0) && (idx < NRAYS); idx += dir)
    {
        int32_t angle = (p_phi[idx] < 0) ? -p_phi[idx] : p_phi[idx];
        int32_t expd =
            (int32_t)((float32_t)front / app_sin_deg((float32_t)(90 - angle)));
        int32_t tol          = GEO_TOL_MM + (expd * GEO_TOL_PCT / 100);
        BOOL    b_near_front = (p_dist[idx] >= 0) &&
                               ((p_dist[idx] - front) <= IN_MM) &&
                               ((front - p_dist[idx]) <= IN_MM);
        BOOL    b_on_box =
            (p_dist[idx] >= 0) &&
            (((angle <= CENTRE_DEG) && b_near_front) ||
             (((p_dist[idx] - expd) <= tol) && ((expd - p_dist[idx]) <= tol)));
        BOOL b_lobe = (!b_on_box) && (p_dist[idx] >= 0) &&
                      (p_dist[idx] < (expd - tol)) &&
                      (p_dist[idx] >= (front - IN_MM)); /* side-of-beam echo */

        if (b_on_box && (out_run < 2))
        {
            last_in   = idx;
            out_run   = 0;
            first_out = -1;
        }
        else if (!b_on_box)
        {
            if (0 == out_run)
            {
                first_out = idx;
            }

            out_run++;

            /* past the edge: open space, or something else nearer? */
            if ((out_run >= 2) &&
                ((SONAR_FAULT == p_dist[idx]) ||
                 ((p_dist[idx] >= 0) && (p_dist[idx] < (front + BG_MM)) &&
                  (!b_lobe))))
            {
                b_other = TRUE; /* something nearer, or unknown */
            }
        }
        else
        {
            /* nothing to do for any other case */
        }
    }

    if ((out_run < 2) || (first_out < 0))
    {
        /* obstacle wider than the scan on this side */
        *p_edge_mm = (int32_t)((float32_t)p_dist[last_in] *
                               app_sin_deg((float32_t)p_phi[last_in]));
        *p_unc_mm  = 50;
        *pb_clear  = FALSE;
        *pb_seen   = FALSE;
        return;
    }

    *pb_seen = TRUE;
    {
        float32_t a_in  = (float32_t)p_phi[last_in];
        float32_t a_out = (float32_t)p_phi[first_out];
        float32_t side  = (a_out > 0.0f) ? 1.0f : -1.0f; /* + = left */
        float32_t a_edge =
            (0.5f * (a_in + a_out)) - (side * (float32_t)S5_BEAM_HALF_DEG);
        float32_t range = (float32_t)p_dist[last_in];

        if ((a_edge * side) < 0.0f)
        {
            a_edge = 0.0f; /* never past the centre */
        }

        *p_edge_mm = (int32_t)(range * app_sin_deg(a_edge));
        *p_unc_mm =
            (int32_t)(range * (app_sin_deg(a_out) - app_sin_deg(a_in)) / 2.0f);

        if (*p_unc_mm < 0)
        {
            *p_unc_mm = -*p_unc_mm;
        }

        *pb_clear = !b_other;
    }
}

EXPORT void obstacle_scan (obstacle_profile_t * p_profile)
{
    int32_t dist[NRAYS];
    int32_t phi[NRAYS];
    int32_t kdx;
    int32_t centre = (90 - SCAN_FROM) / SCAN_STEP;
    int32_t unc_left;
    int32_t unc_right;

    gb_scanning = TRUE;

    for (kdx = 0; kdx < NRAYS; kdx++)
    {
        int32_t ang = SCAN_FROM + (kdx * SCAN_STEP);
        int32_t move =
            (ang > g_servo_now) ? ang - g_servo_now : g_servo_now - ang;
        int32_t settle = move * SETTLE_MS_DEG;
        int32_t value[PINGS];
        int32_t valid  = 0;
        int32_t nfault = 0;
        int32_t jdx;

        servo_set_angle(ang);
        tk_dly_tsk((settle > SETTLE_MIN_MS) ? settle : SETTLE_MIN_MS); /* O1 */

        for (jdx = 0; jdx < PINGS; jdx++)
        {
            /* O2 */
            int32_t dist_mm = sonar_ping_mm();

            if (dist_mm >= 0)
            {
                value[valid++] = dist_mm;
            }
            else if (SONAR_FAULT == dist_mm)
            {
                nfault++;
            }
            else
            {
                /* nothing to do for any other case */
            }
        }

        /* >= 3 good = median; >= 3 no ECHO = fault (unknown, never open) */
        dist[kdx] = (valid >= 3)    ? median5(value, valid)
                    : (nfault >= 3) ? SONAR_FAULT
                                    : FAR;
        /* + = left of the robot */
        phi[kdx] = (ang - 90) * (S5_SERVO_LEFT_HIGH ? 1 : -1);
    }

    servo_set_angle(90);
    tk_dly_tsk(60 * SETTLE_MS_DEG);
    gb_scanning = FALSE;

    tm_printf((uint8_t *)"[SCAN] rays (deg:mm, -1 = open, -2 = no ECHO):");

    for (kdx = 0; kdx < NRAYS; kdx++)
    {
        tm_printf((uint8_t *)" %d:%d", phi[kdx], dist[kdx]);
    }

    tm_printf((uint8_t *)"\n");

    p_profile->rays       = NRAYS;
    p_profile->fault_rays = 0;

    for (kdx = 0; kdx < NRAYS; kdx++)
    {
        if (SONAR_FAULT == dist[kdx])
        {
            p_profile->fault_rays++;
        }
    }

    p_profile->front_mm = dist[centre];
    p_profile->b_found =
        ((dist[centre] >= 0) && (dist[centre] <= MAX_FRONT_MM));

    if (!p_profile->b_found)
    {
        p_profile->width_mm = p_profile->unc_mm = p_profile->left_mm =
            p_profile->right_mm                 = 0;
        p_profile->left_unc = p_profile->right_unc = 0;
        p_profile->b_left_clear = p_profile->b_right_clear = TRUE;
        p_profile->b_left_seen = p_profile->b_right_seen = FALSE;
        return;
    }

    /* phi rises with k only if LEFT_HIGH: walk "left" in the right order */
    side_edge(dist, phi, centre, S5_SERVO_LEFT_HIGH ? +1 : -1, dist[centre],
              &p_profile->left_mm, &unc_left, &p_profile->b_left_clear,
              &p_profile->b_left_seen);
    side_edge(dist, phi, centre, S5_SERVO_LEFT_HIGH ? -1 : +1, dist[centre],
              &p_profile->right_mm, &unc_right, &p_profile->b_right_clear,
              &p_profile->b_right_seen);
    p_profile->left_unc  = unc_left;
    p_profile->right_unc = unc_right;
    p_profile->width_mm  = p_profile->left_mm - p_profile->right_mm;
    {
        float32_t beam    = 2.0f * (float32_t)dist[centre] *
                            app_sin_deg((float32_t)BEAM_UNC_DEG);
        p_profile->unc_mm = (int32_t)app_sqrtf(
            (float32_t)((unc_left * unc_left) + (unc_right * unc_right)) +
            (beam * beam));
    }
}

/* One side from two scans.  An edge counts from a scan that saw past it;
   a scan the obstacle filled to the end of its travel says nothing about
   that side (it used to make the side "not clear": from 20-25 cm a 20 cm
   box fills +-40 deg once the beam's width is added). */
LOCAL void pick_edge (int32_t edge_a, int32_t unc_a, BOOL b_seen_a,
                      BOOL b_clear_a, int32_t edge_b, int32_t unc_b,
                      BOOL b_seen_b, BOOL b_clear_b, int32_t * p_edge,
                      int32_t * p_unc, BOOL * pb_seen, BOOL * pb_clear)
{
    if (b_seen_a && b_seen_b)
    {
        /* both saw it: the nearer scan's (a).  On the 7 Nov scans the
           nearer of each pair always read narrower, nearer the ruler
           (until 8 Nov: the mean, uncertainty half the difference) */
        (void)edge_b;
        (void)unc_b;
        *p_edge   = edge_a;
        *p_unc    = unc_a;
        *pb_seen  = TRUE;
        *pb_clear = b_clear_a && b_clear_b;
    }
    else if (b_seen_a || b_seen_b)
    {
        *p_edge   = b_seen_a ? edge_a : edge_b;
        *p_unc    = b_seen_a ? unc_a : unc_b;
        *pb_seen  = TRUE;
        *pb_clear = b_seen_a ? b_clear_a : b_clear_b;
    }
    else
    {
        /* neither saw it: the wider of the two, not clear */
        int32_t abs_a = (edge_a < 0) ? -edge_a : edge_a;
        int32_t abs_b = (edge_b < 0) ? -edge_b : edge_b;

        *p_edge   = (abs_a > abs_b) ? edge_a : edge_b;
        *p_unc    = 50;
        *pb_seen  = FALSE;
        *pb_clear = FALSE;
    }
}

EXPORT void obstacle_combine (const obstacle_profile_t * p_near,
                              const obstacle_profile_t * p_far,
                              obstacle_profile_t *       p_out)
{
    float32_t beam;

    *p_out         = *p_far;
    p_out->b_found = (p_near->b_found) && (p_far->b_found);
    pick_edge(p_near->left_mm, p_near->left_unc, p_near->b_left_seen,
              p_near->b_left_clear, p_far->left_mm, p_far->left_unc,
              p_far->b_left_seen, p_far->b_left_clear, &p_out->left_mm,
              &p_out->left_unc, &p_out->b_left_seen, &p_out->b_left_clear);
    pick_edge(p_near->right_mm, p_near->right_unc, p_near->b_right_seen,
              p_near->b_right_clear, p_far->right_mm, p_far->right_unc,
              p_far->b_right_seen, p_far->b_right_clear, &p_out->right_mm,
              &p_out->right_unc, &p_out->b_right_seen, &p_out->b_right_clear);
    p_out->width_mm = p_out->left_mm - p_out->right_mm;
    beam            = 2.0f * (float32_t)p_out->front_mm *
                      app_sin_deg((float32_t)BEAM_UNC_DEG);
    p_out->unc_mm =
        (int32_t)app_sqrtf((float32_t)((p_out->left_unc * p_out->left_unc) +
                                       (p_out->right_unc * p_out->right_unc)) +
                           (beam * beam));
}

/* ---- the task: forward watch ---------------------------------------- */

EXPORT void obstacle_task (int32_t stacd, void * p_exinf)
{
    int32_t close_count = 0;
    int32_t fault_count = 0;
    BOOL    b_pinged    = FALSE;
    BOOL    b_told      = FALSE;

    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        tk_slp_tsk(TMO_FEVR); /* woken every 60 ms */
        app_timing_wake(g_tmg_obstacle, tk_can_wup(TSK_SELF));

        if (gb_scanning || (!gb_watch))
        {
            close_count = 0;
            fault_count = 0;
            b_told      = FALSE;

            if (!gb_scanning)
            {
                gb_sensor_fault = FALSE; /* fresh start next watch */
            }
#if !APP_TEST
            /* Idle: keep pinging ahead so the laptop panel shows a live
               distance; Mission is told nothing until the watch is on.
               Only while Mission is IDLE: in a manoeuvre Mission pings
               itself (ahead_clear) and two pings at once corrupt each
               other's echo.  Not in test images: their tests turn the
               servo and ping themselves. */

            if ((!gb_scanning) && gb_idle_ping)
            {
                if (b_pinged)
                {
                    g_front_mm = echo_mm();
                }

                if (g_servo_now != 90)
                {
                    servo_set_angle(90);
                }

                trigger();
                b_pinged = TRUE;
            }
            else
            {
                b_pinged = FALSE;
            }
#else
            b_pinged = FALSE;
#endif
        }
        else
        {
            if (b_pinged)
            {
                int32_t dist_mm = echo_mm();

                g_front_mm = dist_mm;
                /* No ECHO 5 times in a row (300 ms): the sensor is
                   not working, so Mission must stop, not drive blind */
                fault_count = (SONAR_FAULT == dist_mm) ? fault_count + 1 : 0;

                if ((fault_count >= 5) && (!b_told))
                {
                    gb_sensor_fault = TRUE;
                    b_told          = TRUE;
                    tk_set_flg(gh_flg_mission, MSN_FLG_OBSTACLE);
                }

                if ((dist_mm >= 0) && (dist_mm < S5_TRIP_MM))
                {
                    if ((++close_count >= 2) && (!b_told))
                    {
                        b_told = TRUE;
                        tk_set_flg(gh_flg_mission, MSN_FLG_OBSTACLE);
                    }
                }
                else
                {
                    close_count = 0;
                }
            }

            if (g_servo_now != 90)
            {
                servo_set_angle(90);
            }

            trigger();
            b_pinged = TRUE;
        }

        app_timing_done(g_tmg_obstacle);
    }
}

/*** end of file ***/

