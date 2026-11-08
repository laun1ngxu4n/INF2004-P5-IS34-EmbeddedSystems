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

LOCAL volatile UW   echo_rise_us;
LOCAL volatile UW   echo_width_us;
LOCAL volatile BOOL echo_rose;
LOCAL volatile BOOL echo_done;
LOCAL volatile BOOL watch;
LOCAL volatile BOOL idle_ping = TRUE; /* Mission IDLE: ping for the panel */
LOCAL volatile BOOL scanning;
LOCAL volatile INT  front_mm = FAR;
LOCAL volatile BOOL sensor_fault;
LOCAL INT           servo_now = 90;

/* ECHO edges: interrupt context */
LOCAL void echo_edge (UINT pin, UW events)
{
    UW now = app_time_us();

    if (app_gpio_get(pin) != 0u)
    {
        echo_rise_us = now;
        echo_rose    = TRUE;
    }
    else if (echo_rose)
    {
        echo_width_us = now - echo_rise_us;
        echo_done     = TRUE;
    }
    (void)events;
}

LOCAL void trigger (void)
{
    UW t0;

    echo_rose = FALSE;
    echo_done = FALSE;
    gpio_set_val(PIN_TRIG, 1);
    t0 = app_time_us();
    while ((app_time_us() - t0) < 12u)
    {
        ;
    }
    gpio_set_val(PIN_TRIG, 0);
}

/* Result of the last trigger: mm, FAR (nothing in range) or SONAR_FAULT */
LOCAL INT echo_mm (void)
{
    if (!echo_rose)
    {
        return SONAR_FAULT; /* ECHO never rose */
    }
    if ((!echo_done) || (echo_width_us < 100u) || (echo_width_us > 25000u))
    {
        return FAR;
    }
    return (INT)(echo_width_us * 174u / 1000u); /* 0.348 mm/us at 28 C, / 2 */
}

EXPORT void sonar_ping_start (void)
{
    trigger();
}
EXPORT INT sonar_ping_result (void)
{
    return echo_mm();
}

EXPORT INT sonar_ping_mm (void)
{
    INT mm;

    trigger();
    tk_dly_tsk(60);
    mm = echo_mm();
    /* straight ahead: also what the panel shows.  Test images do not
       idle-ping (their tests ping themselves), so the panel's distance
       stayed frozen through Stage 3 (7 Nov) */
    if (90 == servo_now)
    {
        front_mm = mm;
    }
    return mm;
}

LOCAL void servo_write (INT deg)
{
    INT us = 1500 +
             ((deg - 90) * S5_SERVO_US_PER_DEG * (S5_SERVO_LEFT_HIGH ? 1 : -1));

    pwm_set_cc(PIN_SERVO, (UW)us); /* 1 count = 1 us */
    servo_now = deg;
}

/* Walk there S5_SERVO_DEG_STEP per 20 ms frame: a full-speed jump broke
   the servo horn */
/* turning limit: the scan's, or the side look's if that is larger */
#define SERVO_LIMIT                                              \
    ((S5_SERVO_SIDE_DEG > S5_SERVO_HALF_DEG) ? S5_SERVO_SIDE_DEG \
                                             : S5_SERVO_HALF_DEG)

EXPORT void servo_set_angle (INT deg)
{
    if (deg < (90 - SERVO_LIMIT))
    {
        deg = 90 - SERVO_LIMIT;
    }
    if (deg > (90 + SERVO_LIMIT))
    {
        deg = 90 + SERVO_LIMIT;
    }
    while (servo_now != deg)
    {
        INT next = servo_now +
                   ((deg > servo_now) ? S5_SERVO_DEG_STEP : -S5_SERVO_DEG_STEP);

        if ((deg > servo_now) ? (next > deg) : (next < deg))
        {
            next = deg;
        }
        servo_write(next);
        tk_dly_tsk(20);
    }
}

EXPORT void servo_look_side (INT side)
{
    INT left = S5_SERVO_LEFT_HIGH ? 1 : -1;

    servo_set_angle(90 + (side * left * S5_SERVO_SIDE_DEG));
}

EXPORT void obstacle_watch (BOOL on)
{
    watch = on;
}
EXPORT void obstacle_idle_ping (BOOL on)
{
    idle_ping = on;
}
EXPORT INT obstacle_front_mm (void)
{
    return front_mm;
}
EXPORT BOOL obstacle_sensor_fault (void)
{
    return sensor_fault;
}

EXPORT ER obstacle_hw_init (void)
{
    ER er;

    app_hw_release(RESETS_RESET_PWM);
    /* servo: 125 MHz / 125 = 1 MHz, wrap 20000 = 50 Hz */
    out_w(PWM_BASE + (SERVO_SLICE * 0x14) + PWM_CHx_DIV, 125u << 4);
    pwm_set_wrap(PIN_SERVO, 19999);
    pwm_set_cc(PIN_SERVO, 1500);
    pwm_set_pin(PIN_SERVO);
    pwm_set_enabled(PIN_SERVO, TRUE);

    er = app_gpio_output(PIN_TRIG);
    if (er < E_OK)
    {
        return er;
    }
    er = app_gpio_input(PIN_ECHO, FALSE);
    if (er < E_OK)
    {
        return er;
    }
    clr_w(GPIO(PIN_ECHO), GPIO_PUE);
    set_w(GPIO(PIN_ECHO), GPIO_PDE); /* as in bring-up */
    return app_gpio_irq_add(PIN_ECHO, APP_GPIO_BOTH, echo_edge);
}

/* ---- maths without libm -------------------------------------------- */

LOCAL INT median5 (INT * v, INT n)
{
    INT i;
    INT j;
    INT t;

    for (i = 1; i < n; i++)
    {
        for (j = i; (j > 0) && (v[j] < v[j - 1]); j--)
        {
            t        = v[j];
            v[j]     = v[j - 1];
            v[j - 1] = t;
        }
    }
    return v[n / 2];
}

/* ---- the profiling scan --------------------------------------------- */

/* One side: walk outward from straight ahead.  dir = +1 / -1 is the
   direction in the ray array (which side that is depends on the servo).
   Returns the lateral edge (mm, signed) and its half-step uncertainty. */
LOCAL void side_edge (const INT * d, const INT * phi, INT centre, INT dir,
                      INT front, INT * edge_mm, INT * unc_mm, BOOL * clear,
                      BOOL * seen)
{
    INT  i         = centre;
    INT  last_in   = centre;
    INT  out_run   = 0;
    INT  first_out = -1;
    BOOL other     = FALSE;

    for (i = centre + dir; (i >= 0) && (i < NRAYS); i += dir)
    {
        INT  a          = (phi[i] < 0) ? -phi[i] : phi[i];
        INT  expd       = (INT)((float)front / app_sin_deg((float)(90 - a)));
        INT  tol        = GEO_TOL_MM + (expd * GEO_TOL_PCT / 100);
        BOOL near_front = (d[i] >= 0) && ((d[i] - front) <= IN_MM) &&
                          ((front - d[i]) <= IN_MM);
        BOOL in =
            (d[i] >= 0) && (((a <= CENTRE_DEG) && near_front) ||
                            (((d[i] - expd) <= tol) && ((expd - d[i]) <= tol)));
        BOOL lobe = (!in) && (d[i] >= 0) && (d[i] < (expd - tol)) &&
                    (d[i] >= (front - IN_MM)); /* side-of-beam echo */

        if (in && (out_run < 2))
        {
            last_in   = i;
            out_run   = 0;
            first_out = -1;
        }
        else if (!in)
        {
            if (0 == out_run)
            {
                first_out = i;
            }
            out_run++;
            /* past the edge: open space, or something else nearer? */
            if ((out_run >= 2) &&
                ((SONAR_FAULT == d[i]) ||
                 ((d[i] >= 0) && (d[i] < (front + BG_MM)) && (!lobe))))
            {
                other = TRUE; /* something nearer, or unknown */
            }
        }
    }
    if ((out_run < 2) || (first_out < 0))
    {
        /* obstacle wider than the scan on this side */
        *edge_mm = (INT)((float)d[last_in] * app_sin_deg((float)phi[last_in]));
        *unc_mm  = 50;
        *clear   = FALSE;
        *seen    = FALSE;
        return;
    }
    *seen = TRUE;
    {
        float a_in  = (float)phi[last_in];
        float a_out = (float)phi[first_out];
        float side  = (a_out > 0.0f) ? 1.0f : -1.0f; /* + = left */
        float a_edge =
            (0.5f * (a_in + a_out)) - (side * (float)S5_BEAM_HALF_DEG);
        float r = (float)d[last_in];

        if ((a_edge * side) < 0.0f)
        {
            a_edge = 0.0f; /* never past the centre */
        }
        *edge_mm = (INT)(r * app_sin_deg(a_edge));
        *unc_mm  = (INT)(r * (app_sin_deg(a_out) - app_sin_deg(a_in)) / 2.0f);
        if (*unc_mm < 0)
        {
            *unc_mm = -*unc_mm;
        }
        *clear = !other;
    }
}

EXPORT void obstacle_scan (OBSTACLE_PROFILE * p)
{
    INT d[NRAYS];
    INT phi[NRAYS];
    INT k;
    INT centre = (90 - SCAN_FROM) / SCAN_STEP;
    INT ul;
    INT ur;

    scanning = TRUE;
    for (k = 0; k < NRAYS; k++)
    {
        INT ang    = SCAN_FROM + (k * SCAN_STEP);
        INT move   = (ang > servo_now) ? ang - servo_now : servo_now - ang;
        INT settle = move * SETTLE_MS_DEG;
        INT v[PINGS];
        INT nv     = 0;
        INT nfault = 0;
        INT j;

        servo_set_angle(ang);
        tk_dly_tsk((settle > SETTLE_MIN_MS) ? settle : SETTLE_MIN_MS); /* O1 */
        for (j = 0; j < PINGS; j++)
        { /* O2 */
            INT mm = sonar_ping_mm();
            if (mm >= 0)
            {
                v[nv++] = mm;
            }
            else if (SONAR_FAULT == mm)
            {
                nfault++;
            }
        }
        /* >= 3 good = median; >= 3 no ECHO = fault (unknown, never open) */
        d[k] = (nv >= 3) ? median5(v, nv) : (nfault >= 3) ? SONAR_FAULT : FAR;
        /* + = left of the robot */
        phi[k] = (ang - 90) * (S5_SERVO_LEFT_HIGH ? 1 : -1);
    }
    servo_set_angle(90);
    tk_dly_tsk(60 * SETTLE_MS_DEG);
    scanning = FALSE;

    tm_printf((UB *)"[SCAN] rays (deg:mm, -1 = open, -2 = no ECHO):");
    for (k = 0; k < NRAYS; k++)
    {
        tm_printf((UB *)" %d:%d", phi[k], d[k]);
    }
    tm_printf((UB *)"\n");

    p->rays       = NRAYS;
    p->fault_rays = 0;
    for (k = 0; k < NRAYS; k++)
    {
        if (SONAR_FAULT == d[k])
        {
            p->fault_rays++;
        }
    }
    p->front_mm = d[centre];
    p->found    = ((d[centre] >= 0) && (d[centre] <= MAX_FRONT_MM));
    if (!p->found)
    {
        p->width_mm = p->unc_mm = p->left_mm = p->right_mm = 0;
        p->left_unc = p->right_unc = 0;
        p->left_clear = p->right_clear = TRUE;
        p->left_seen = p->right_seen = FALSE;
        return;
    }
    /* phi rises with k only if LEFT_HIGH: walk "left" in the right order */
    side_edge(d, phi, centre, S5_SERVO_LEFT_HIGH ? +1 : -1, d[centre],
              &p->left_mm, &ul, &p->left_clear, &p->left_seen);
    side_edge(d, phi, centre, S5_SERVO_LEFT_HIGH ? -1 : +1, d[centre],
              &p->right_mm, &ur, &p->right_clear, &p->right_seen);
    p->left_unc  = ul;
    p->right_unc = ur;
    p->width_mm  = p->left_mm - p->right_mm;
    {
        float beam = 2.0f * (float)d[centre] * app_sin_deg((float)BEAM_UNC_DEG);
        p->unc_mm =
            (INT)app_sqrtf((float)((ul * ul) + (ur * ur)) + (beam * beam));
    }
}

/* One side from two scans.  An edge counts from a scan that saw past it;
   a scan the obstacle filled to the end of its travel says nothing about
   that side (it used to make the side "not clear": from 20-25 cm a 20 cm
   box fills +-40 deg once the beam's width is added). */
LOCAL void pick_edge (INT ea, INT ua, BOOL sa, BOOL ca, INT eb, INT ub, BOOL sb,
                      BOOL cb, INT * e, INT * u, BOOL * seen, BOOL * clear)
{
    if (sa && sb)
    {
        /* both saw it: the nearer scan's (a).  On the 7 Nov scans the
           nearer of each pair always read narrower, nearer the ruler
           (until 8 Nov: the mean, uncertainty half the difference) */
        (void)eb;
        (void)ub;
        *e     = ea;
        *u     = ua;
        *seen  = TRUE;
        *clear = ca && cb;
    }
    else if (sa || sb)
    {
        *e     = sa ? ea : eb;
        *u     = sa ? ua : ub;
        *seen  = TRUE;
        *clear = sa ? ca : cb;
    }
    else
    {
        /* neither saw it: the wider of the two, not clear */
        INT aa = (ea < 0) ? -ea : ea;
        INT ab = (eb < 0) ? -eb : eb;

        *e     = (aa > ab) ? ea : eb;
        *u     = 50;
        *seen  = FALSE;
        *clear = FALSE;
    }
}

EXPORT void obstacle_combine (const OBSTACLE_PROFILE * a,
                              const OBSTACLE_PROFILE * b,
                              OBSTACLE_PROFILE *       out)
{
    float beam;

    *out       = *b;
    out->found = (a->found) && (b->found);
    pick_edge(a->left_mm, a->left_unc, a->left_seen, a->left_clear, b->left_mm,
              b->left_unc, b->left_seen, b->left_clear, &out->left_mm,
              &out->left_unc, &out->left_seen, &out->left_clear);
    pick_edge(a->right_mm, a->right_unc, a->right_seen, a->right_clear,
              b->right_mm, b->right_unc, b->right_seen, b->right_clear,
              &out->right_mm, &out->right_unc, &out->right_seen,
              &out->right_clear);
    out->width_mm = out->left_mm - out->right_mm;
    beam = 2.0f * (float)out->front_mm * app_sin_deg((float)BEAM_UNC_DEG);
    out->unc_mm = (INT)app_sqrtf((float)((out->left_unc * out->left_unc) +
                                         (out->right_unc * out->right_unc)) +
                                 (beam * beam));
}

/* ---- the task: forward watch ---------------------------------------- */

EXPORT void obstacle_task (INT stacd, void * exinf)
{
    INT  close_count = 0;
    INT  fault_count = 0;
    BOOL pinged      = FALSE;
    BOOL told        = FALSE;

    (void)stacd;
    (void)exinf;

    for (;;)
    {
        tk_slp_tsk(TMO_FEVR); /* woken every 60 ms */
        app_timing_wake(tmg_obstacle, tk_can_wup(TSK_SELF));

        if (scanning || (!watch))
        {
            close_count = 0;
            fault_count = 0;
            told        = FALSE;
            if (!scanning)
            {
                sensor_fault = FALSE; /* fresh start next watch */
            }
#if !APP_TEST
            /* Idle: keep pinging ahead so the laptop panel shows a live
               distance; Mission is told nothing until the watch is on.
               Only while Mission is IDLE: in a manoeuvre Mission pings
               itself (ahead_clear) and two pings at once corrupt each
               other's echo.  Not in test images: their tests turn the
               servo and ping themselves. */
            if ((!scanning) && idle_ping)
            {
                if (pinged)
                {
                    front_mm = echo_mm();
                }
                if (servo_now != 90)
                {
                    servo_set_angle(90);
                }
                trigger();
                pinged = TRUE;
            }
            else
            {
                pinged = FALSE;
            }
#else
            pinged = FALSE;
#endif
        }
        else
        {
            if (pinged)
            {
                INT mm = echo_mm();

                front_mm = mm;
                /* No ECHO 5 times in a row (300 ms): the sensor is
                   not working, so Mission must stop, not drive blind */
                fault_count = (SONAR_FAULT == mm) ? fault_count + 1 : 0;
                if ((fault_count >= 5) && (!told))
                {
                    sensor_fault = TRUE;
                    told         = TRUE;
                    tk_set_flg(flg_mission, MSN_FLG_OBSTACLE);
                }
                if ((mm >= 0) && (mm < S5_TRIP_MM))
                {
                    if ((++close_count >= 2) && (!told))
                    {
                        told = TRUE;
                        tk_set_flg(flg_mission, MSN_FLG_OBSTACLE);
                    }
                }
                else
                {
                    close_count = 0;
                }
            }
            if (servo_now != 90)
            {
                servo_set_angle(90);
            }
            trigger();
            pinged = TRUE;
        }

        app_timing_done(tmg_obstacle);
    }
}

/*** end of file ***/

