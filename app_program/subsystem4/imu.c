/**
 * @file    imu.c
 *  Subsystem 4 -- the IMU / Terrain Monitor task (every 10 ms).
 *
 *  Reads the LSM303D accelerometer (+/-4 g, 100 Hz) and tracks the ramp
 *  height: see subsystem4.h.  Register values are the ones bring-up
 *  verified (bringup/tests/imu/imu_test.c).
 *
 *  Lifted or tipped over (lecturer, 13 Nov: replaces IMU collision
 *  detection): when gravity points more than LIFT_DEG away from where it
 *  pointed at the last level calibration, for LIFT_TICKS, the motors go
 *  off as if STOP were pressed.  Lifting the robot ends the demo run.
 */

#include "subsystem4.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem5/subsystem5.h" /* gh_flg_mission, MSN_FLG_STOP */
#include "../app_math.h"
#include "imu_i2c.h"
#include "imu.h"

#define IMU_ADDR      0x1D
#define REG_WHO_AM_I  0x0F
#define WHO_AM_I_VAL  0x49
#define REG_OUT_X_L_A 0x28

#define CAL_SAMPLES     100    /* 1 s */
#define LPF_K           0.1f   /* about 100 ms */
#define FLAT_SIN        0.030f /* |pitch| < about 1.7 deg = level */
#define FLAT_MM         100    /* level for this far = back on the floor */
#define REPORT_MIN_MM10 40     /* peaks under 4 mm are not ramps */
#define QUIET_AFTER_MM  600    /* no 2nd report for the way down */
/* REPORT_MIN_MM10: 8 mm until 23 Nov.  The course's hump is only about
   5 mm (team, 23 Nov), which should read about 5 mm (x 0.63,
   S4_ONE_WHEEL_GAIN_PCT, fitted on 10 and 25 mm humps, not checked at
   5 mm): under 8 mm it would not be reported.  The largest false reading
   on the flat seen so far is about 3.0 mm, at the start of a run (9 Nov
   S4-R01 run 2: side tilt 38/1000); driving on the flat, about 2.5 mm
   (side tilt 31-34/1000).  4 mm sits between them. */
/* One wheel up (the course's hump): reported this far after its highest
   point, once the side tilt is level again, whatever the nose tilt (23
   Nov: driving lean and speed changes kept the nose tilt over FLAT_SIN,
   so waiting for both to be level reported seconds late, or never when
   the run was stopped first) */
#define PEAK_PAST_MM 50
/* Tilt alone keeps |a| at 1 g; a bump or a speed change adds to it.  A
   sample more than this far from the calibrated |a| is a jolt, not tilt:
   keep the last pitch (Stage 4 run, 2 Nov: hitting a 10 mm hump read as
   up to 40 deg) */
/* 0.05 skipped 30-48% of all samples while driving on the flat floor (Stage
 * 4 run, 6 Nov) */
#define JOLT_FRAC 0.10f
/* Driving level: driving tilts the chassis a little (motor torque on the
   caster), which read as -2..-4 deg on the flat floor and drifted the
   height -10 mm before the hump (6 Nov).  While the tilt is within
   BASE_WIN of it, the level follows the tilt slowly (about 5 s); on a
   ramp or hump (10 mm hump: about 0.12) it stays put. */
#define BASE_K   0.002f
#define BASE_WIN 0.060f /* about 3.4 deg */
/* One-wheel hump (8 Nov): the course's hump is a small triangle, 2-5 cm
   high, under the RIGHT wheel only (the barcode sensor's other side).
   The robot stands on both wheels and the front support, so lifting one
   wheel by h tilts it sideways by h over the track width and tips the
   nose DOWN a little (the axle centre rises h / 2, the front support
   stays).  The chassis is rigid: the axle between the wheels stays T
   long and tilts, so the lifted wheel is h = T x sin(side tilt) above
   the other (T = S2_TRACK_MM, 123 mm; 5 cm = 24 deg, 2 cm = 9 deg; the
   formula another group uses too).  From the nose-down tilt: the axle
   centre rises h / 2 at S4_L_MM from the front support, h = 2 x S4_L_MM
   x sin(pitch), as a cross-check.  (A first version used tan, which
   assumes the wheels stay T apart on the floor: 5 mm too high at 5 cm.)  The
   pitch-only code saw nothing of it (it ignores nose-down): 2-6 Nov
   S4-R01 read 18, 66, -, 49, -, -, 27, -, 9 mm.  The triangle has a
   sharp top, so the side tilt peaks briefly: a lighter filter than the
   pitch's (ROLL_LPF_K ~ 30 ms; 100 ms took about 15 % off the peak). */
#define ROLL_LPF_K 0.3f
/* "Back on the floor" uses a slower copy of the side tilt (as the pitch's
   filter, about 100 ms): the fast one jitters +-30/1000 while driving (9 Nov
   S4-R01: -34, -31 on the flat), which kept resetting the level distance,
   so no hump was ever reported.  The hump itself reads about 140/1000. */
#define ROLL_FLAT_K 0.1f
/* Lifted or tipped over: the angle between gravity now and gravity at the
   last calibration.  The largest tilt while driving was about 19 deg (the
   25 mm hump, 9 Nov: side tilt sin 0.33), so 35 deg leaves a margin; a
   robot picked up by hand tilts more.  Upside down is 180 deg (pitch and
   side tilt alone read about 0 there).  The direction is filtered like
   the side tilt (about 30 ms), and must stay past 35 deg for 100 ms. */
#define LIFT_COS       0.819f /* cos(35 deg) */
#define LIFT_CLEAR_COS 0.940f /* cos(20 deg): level again, re-armed */
#define LIFT_TICKS     10     /* 100 ms at 10 ms */
#define LIFT_LPF_K     0.3f
#define LIFT_AXES      3 /* x, y, z */

/* The lift check's state: written only by the IMU task */
typedef struct
{
    float32_t level[LIFT_AXES]; /* gravity direction at calibration (unit) */
    float32_t now[LIFT_AXES];   /* gravity direction now, filtered */
    int32_t   ticks;            /* 10 ms periods past LIFT_COS, 0..LIFT_TICKS */
    bool      b_known;          /* level[] set by a calibration */
    bool      b_lifted;         /* reported; cleared when level again */
} lift_state_t;

LOCAL volatile bool gb_cal_request = true; /* calibrate once at boot too */
LOCAL volatile bool gb_calibrated;
/* The IMU's results (Change Requirement 1): written only by the IMU task,
   each in the bits its range needs, clamped (app_clamp) on the way in.
   Whether the chip answers is FAULT_IMU in the fault word.  The
   calibration flags above stay separate: Mission writes them too. */
typedef struct
{
    int32_t sin_x1000   APP_FIELD(11);     /* sin(pitch) x 1000, +-1023 */
    int32_t roll_x1000  APP_FIELD(11);     /* sin(side tilt) x 1000 */
    int32_t height_mm10 APP_FIELD(13);     /* climbed height, 0.1 mm,
                                              +-409.5 mm */
    uint32_t one_h_mm10 APP_FIELD(11);     /* one wheel up, 0.1 mm,
                                              0..204.7 (track x 1 = 123) */
    uint32_t ramp_peak_mm10 APP_FIELD(12); /* last ramp reported, 0.1 mm,
                                              0..409.5 */
} imu_out_t;

LOCAL volatile imu_out_t g_out = {0, 0, 0, 0u, 0u};
LOCAL volatile uint32_t  g_ramp_reports;
LOCAL volatile uint32_t  g_jolts;
LOCAL volatile uint32_t  g_lifts; /* lifts seen since boot */

/** true once the IMU answered and was configured. */
EXPORT bool imu_ok (void)
{
    return 0u == (app_stop_faults() & FAULT_IMU);
}
/** Lifts (or tip-overs) that stopped the motors since boot. */
EXPORT uint32_t imu_lifts (void)
{
    return g_lifts;
}

/** Starts a 1 s level calibration (robot level and still). */
EXPORT void imu_calibrate (void)
{
    gb_calibrated  = false;
    gb_cal_request = true;
}
/** true once the level calibration is done. */
EXPORT bool imu_calibrated (void)
{
    return gb_calibrated;
}
/** Current climbed height, 0.1 mm. */
EXPORT int32_t imu_height_mm10 (void)
{
    return g_out.height_mm10;
}
/** sin(pitch) x 1000, relative to the level. */
EXPORT int32_t imu_pitch_mdeg_sin (void)
{
    return g_out.sin_x1000;
}
/** sin(side tilt) x 1000, relative to the level. */
EXPORT int32_t imu_roll_sin_x1000 (void)
{
    return g_out.roll_x1000;
}
/** Height of one wheel above the other now (track x sin(side tilt)), 0.1
 * mm. */
EXPORT int32_t imu_one_wheel_mm10 (void)
{
    return g_out.one_h_mm10;
}
/** The last reported ramp height, 0.1 mm. */
EXPORT int32_t imu_ramp_peak_mm10 (void)
{
    return g_out.ramp_peak_mm10;
}
/** Ramps reported since boot. */
EXPORT uint32_t imu_ramp_reports (void)
{
    return g_ramp_reports;
}
/** Samples skipped as jolts since boot. */
EXPORT uint32_t imu_jolts (void)
{
    return g_jolts;
}

/** Checks the LSM303D answers and configures its accelerometer. */
LOCAL bool chip_setup (void)
{
    uint8_t who = 0;

    if ((imu_i2c_read(IMU_ADDR, REG_WHO_AM_I, &who, 1) != E_OK) ||
        (who != WHO_AM_I_VAL))
    {
        return false;
    }

    return (E_OK == imu_i2c_write(IMU_ADDR, 0x20,
                                  0x6F)) && /* CTRL1: 100 Hz, BDU, XYZ */
           (E_OK == imu_i2c_write(IMU_ADDR, 0x21, 0xC8)) && /* CTRL2: +/-4 g, 50
                                   Hz anti-alias filter (was 773 Hz: motor
                                   vibration folded into the tilt) */
           (E_OK == imu_i2c_write(IMU_ADDR, 0x24, 0x70)) && /* CTRL5 */
           (E_OK == imu_i2c_write(IMU_ADDR, 0x25, 0x20)) && /* CTRL6 */
           (E_OK == imu_i2c_write(IMU_ADDR, 0x26, 0x00));   /* CTRL7: no HPF */
}

/** The IMU is not answering: sets it up again every 1 s (*p_retry counts
   the 10 ms periods down). */
LOCAL void chip_retry (int32_t * p_retry)
{
    (*p_retry)--;

    if (*p_retry <= 0)
    {
        *p_retry = 100; /* try again in 1 s */

        if (chip_setup())
        {
            app_stop_fault_clear(FAULT_IMU);
            tm_printf(app_str("[IMU] LSM303D found at 0x1D\n"));
        }
    }
}

/** The level gravity direction from the calibration sums (unit vector). */
LOCAL void lift_level (lift_state_t * p_lift, float32_t sum_x, float32_t sum_y,
                       float32_t sum_z)
{
    float32_t len =
        app_sqrtf((sum_x * sum_x) + (sum_y * sum_y) + (sum_z * sum_z));
    int32_t idx;

    if (len > 0.0f)
    {
        p_lift->level[0] = sum_x / len;
        p_lift->level[1] = sum_y / len;
        p_lift->level[2] = sum_z / len;

        for (idx = 0; idx < LIFT_AXES; idx++)
        {
            p_lift->now[idx] = p_lift->level[idx];
        }

        p_lift->ticks   = 0;
        p_lift->b_known = true;
    }
}

/** Motors off as STOP does, and tell Mission. */
LOCAL void lift_stop (void)
{
    app_stop_request(STOP_BUTTON); /* START clears it, as a STOP */
    tk_set_flg(gh_flg_mission, MSN_FLG_STOP);
}

/** One 10 ms lift check: unit gravity (x, y, z) from this sample. */
LOCAL void lift_check (lift_state_t * p_lift, float32_t unit_x,
                       float32_t unit_y, float32_t unit_z)
{
    float32_t len;
    float32_t cos_now;
    int32_t   duty_l;
    int32_t   duty_r;

    p_lift->now[0] += LIFT_LPF_K * (unit_x - p_lift->now[0]);
    p_lift->now[1] += LIFT_LPF_K * (unit_y - p_lift->now[1]);
    p_lift->now[2] += LIFT_LPF_K * (unit_z - p_lift->now[2]);

    len = app_sqrtf((p_lift->now[0] * p_lift->now[0]) +
                    (p_lift->now[1] * p_lift->now[1]) +
                    (p_lift->now[2] * p_lift->now[2]));

    if ((!p_lift->b_known) || (len < 0.1f))
    {
        return;
    }

    cos_now = ((p_lift->now[0] * p_lift->level[0]) +
               (p_lift->now[1] * p_lift->level[1]) +
               (p_lift->now[2] * p_lift->level[2])) /
              len;

    if (cos_now < LIFT_COS)
    {
        p_lift->ticks =
            (p_lift->ticks < LIFT_TICKS) ? p_lift->ticks + 1 : LIFT_TICKS;
    }
    else
    {
        p_lift->ticks = 0;

        if (cos_now > LIFT_CLEAR_COS)
        {
            p_lift->b_lifted = false; /* level again: re-armed */
        }
    }

    if (p_lift->ticks < LIFT_TICKS)
    {
        return;
    }

    if (!p_lift->b_lifted)
    {
        p_lift->b_lifted = true;
        g_lifts++;
        lift_stop();
        telemetry_event(app_str("robot lifted or tipped over (tilt over 35 "
                                "deg): motors off, START to go again"));
    }
    else
    {
        /* still lifted: keep the motors off if START was pressed */
        motor_duties(&duty_l, &duty_r);

        if ((duty_l != 0) || (duty_r != 0))
        {
            lift_stop();
        }
    }
}

/** Reports a one-wheel hump: o10 its height (0.1 mm, scaled for driving),
   the side tilt at its peak and the nose-down cross-check in mm. */
LOCAL void report_one_wheel (int32_t o10, float32_t roll_peak,
                             float32_t pitch_peak_mm)
{
    uint8_t msg[TELE_TEXT_MAX];
    /* a height in mm, small: fits int32_t in 0.1 mm */
    int32_t c10 = (int32_t)(pitch_peak_mm * 10.0f);

    g_out.ramp_peak_mm10 = app_clamp(o10, 0, 4095);
    g_ramp_reports++;
    /* sin x 1000, |sin| <= 1: fits %d */
    tm_sprintf(msg,
               app_str("ramp peak height %d.%d mm (one wheel: side tilt "
                       "sin %d/1000 x T x 0.%d; from the nose-down tilt "
                       "%d.%d mm)"),
               o10 / 10, o10 % 10, (int32_t)(roll_peak * 1000.0f),
               S4_ONE_WHEEL_GAIN_PCT, c10 / 10, c10 % 10);
    telemetry_event(msg);
}

/** The IMU task: reads the accelerometer, tracks tilt and ramp height,
 * reports ramps. */
EXPORT void imu_task (int32_t stacd, void * p_exinf)
{
    int32_t      retry          = 0;
    int32_t      ncal           = 0;
    float32_t    cal_s          = 0.0f;
    float32_t    cal_c          = 0.0f;
    float32_t    sin_level      = 0.0f;
    float32_t    cos_level      = 1.0f;
    float32_t    cal_m          = 0.0f;
    float32_t    mag_level      = 0.0f; /* |a| when level and still */
    float32_t    cal_y          = 0.0f;
    float32_t    cal_z          = 0.0f; /* gravity's z share, summed */
    float32_t    side_level     = 0.0f; /* sideways share when level */
    float32_t    roll_f         = 0.0f;
    float32_t    base_r         = 0.0f;
    float32_t    roll           = 0.0f;
    float32_t    roll_slow      = 0.0f; /* roll, filtered more: level test */
    float32_t    one_peak       = 0.0f;
    float32_t    one_pitch_peak = 0.0f;
    float32_t    roll_peak      = 0.0f;
    float32_t    sin_f          = 0.0f;
    float32_t    height         = 0.0f;
    float32_t    peak           = 0.0f;
    float32_t    peak_sin       = 0.0f;
    float32_t    base           = 0.0f;
    float32_t    tilt           = 0.0f; /* driving level; sin_f - base */
    int32_t      odo_prev;
    int32_t      flat_mm10  = 0;
    int32_t      quiet_mm10 = 0;     /* distance left before another report */
    int32_t      past_mm10  = 0;     /* driven since one_peak last rose */
    bool         b_manoeuv  = false; /* Mission manoeuvring last period */
    lift_state_t lift       = {
        {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 0, false, false};

    (void)stacd;
    (void)p_exinf;
    imu_i2c_init();
    odo_prev = motor_odometer();

    for (;;)
    {
        uint8_t raw[6];
        int32_t odo;

        tk_slp_tsk(TMO_FEVR); /* woken every 10 ms */
        app_timing_wake(g_tmg_imu, tk_can_wup(TSK_SELF));

        if (!imu_ok())
        {
            chip_retry(&retry);
            goto done;
        }

        if (imu_i2c_read(IMU_ADDR, REG_OUT_X_L_A, raw, 6) != E_OK)
        {
            app_stop_fault_set(FAULT_IMU);
            telemetry_event(app_str("IMU read failed, retrying"));
            goto done;
        }

        {
            /* two bytes, low first, make the 16-bit signed reading; as a
               float it is exact */
            float32_t acc_x = (float32_t)(int16_t)(raw[0] | (raw[1] << 8));
            /* the same for y */
            float32_t acc_y = (float32_t)(int16_t)(raw[2] | (raw[3] << 8));
            /* the same for z */
            float32_t acc_z = (float32_t)(int16_t)(raw[4] | (raw[5] << 8));
            float32_t mag =
                app_sqrtf((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));
            float32_t sin_pitch;
            float32_t cos_pitch;
            float32_t rel;

            if (mag < 1.0f)
            {
                goto done;
            }

            sin_pitch = acc_x / mag; /* sin(pitch) */
            cos_pitch = app_sqrtf((acc_y * acc_y) + (acc_z * acc_z)) /
                        mag; /* cos(pitch) */

            if (gb_cal_request)
            {
                cal_s += sin_pitch;
                cal_c += cos_pitch;
                cal_m += mag;
                cal_y += acc_y / mag;
                cal_z += acc_z / mag;

                ncal++;

                if (ncal >= CAL_SAMPLES)
                {
                    sin_level  = cal_s / CAL_SAMPLES;
                    cos_level  = cal_c / CAL_SAMPLES;
                    mag_level  = cal_m / CAL_SAMPLES;
                    side_level = cal_y / CAL_SAMPLES;
                    lift_level(&lift, cal_s, cal_y, cal_z);
                    cal_s = cal_c = cal_m = cal_y = cal_z = 0.0f;
                    ncal                                  = 0;
                    gb_cal_request                        = false;
                    gb_calibrated                         = true;
                    /* a new run: the last run's level stretch and its
                       quiet distance after a report do not carry over
                       (9 Nov: runs 2 and 4 right after a reported run
                       never reported) */
                    flat_mm10  = 0;
                    quiet_mm10 = 0;
                    past_mm10  = 0;
                    height = peak = peak_sin = 0.0f;
                    sin_f = base = tilt = 0.0f;
                    roll_f = base_r = roll = roll_slow = 0.0f;
                    one_peak = one_pitch_peak = roll_peak = 0.0f;
                }

                goto done;
            }

            lift_check(&lift, sin_pitch, acc_y / mag, acc_z / mag);

            /* sin(pitch - level pitch), low-pass filtered; jolts
               skipped (the pitch from before them is kept) */

            if (app_absf(mag - mag_level) <= (JOLT_FRAC * mag_level))
            {
                rel = (sin_pitch * cos_level) - (cos_pitch * sin_level);
                sin_f += LPF_K * (rel - sin_f);

                if (app_absf(sin_f - base) < BASE_WIN)
                {
                    base += BASE_K * (sin_f - base);
                }

                tilt = sin_f - base;
                /* |sin| <= 1: x 1000 fits int32_t */
                g_out.sin_x1000 =
                    app_clamp((int32_t)(tilt * 1000.0f), -1023, 1023);
                /* side tilt, the same way (level offset, filter,
                   slowly following driving level) */
                roll_f += ROLL_LPF_K * (((acc_y / mag) - side_level) - roll_f);

                if (app_absf(roll_f - base_r) < BASE_WIN)
                {
                    base_r += BASE_K * (roll_f - base_r);
                }

                roll = roll_f - base_r;
                roll_slow += ROLL_FLAT_K * (roll - roll_slow);
                /* |sin| <= 1: x 1000 fits int32_t */
                g_out.roll_x1000 =
                    app_clamp((int32_t)(roll * 1000.0f), -1023, 1023);
            }
            else
            {
                g_jolts++;
            }
        }

        /* Height: distance driven x sin(pitch) */
        odo = motor_odometer();
        {
            /* counts in one 10 ms period, a few: exact */
            float32_t step_mm =
                (float32_t)(odo - odo_prev) * S2_UM_PER_COUNT / 1000.0f;
            /* 20 Nov, track 4: a hump just before the box was reported
               halfway round it (the stop leaves no level distance), and the
               bypass (off the track, spins) can look like a hump.  So a
               junction turn or an obstacle reports the hump being measured
               at once, and nothing is measured while it lasts. */
            bool b_now   = mission_manoeuvring();
            bool b_flush = b_now && (!b_manoeuv);

            b_manoeuv = b_now;
            odo_prev  = odo;
            height += step_mm * tilt;
            /* metres at most: fits int32_t in 0.1 mm */
            g_out.height_mm10 =
                app_clamp((int32_t)(height * 10.0f), -4095, 4095);
            /* climbs only: a nose-down dip (rear wheels on the
               hump, or coming off it) is not a ramp (6 Nov run 1
               reported a -56 mm dip as 56 mm) */

            if (height > peak)
            {
                peak = height;
            }

            if (tilt > peak_sin)
            {
                peak_sin = tilt;
            }

            /* one wheel up: h = T x sin(side tilt); cross-check
               from the nose-down tilt, 2 L x sin(pitch) */
            {
                float32_t roll_abs = app_absf(roll);
                /* mm, small: exact as a float, back to 0.1 mm in int32_t */
                float32_t height_roll = (float32_t)S2_TRACK_MM * roll_abs;

                /* 0.1 mm: fits int32_t */
                g_out.one_h_mm10 =
                    app_clamp((int32_t)(height_roll * 10.0f), 0, 2047);

                if (height_roll > one_peak)
                {
                    one_peak  = height_roll;
                    roll_peak = roll_abs;
                    past_mm10 = 0;
                }
                else
                {
                    /* one period's mm, small: fits int32_t in 0.1 mm */
                    past_mm10 += (int32_t)(app_absf(step_mm) * 10.0f);
                }

                if (tilt < 0.0f)
                {
                    /* mm, small: exact as a float */
                    float32_t height_pitch = 2.0f * (float32_t)S4_L_MM * -tilt;

                    if (height_pitch > one_pitch_peak)
                    {
                        one_pitch_peak = height_pitch;
                    }
                }
            }

            if (quiet_mm10 > 0)
            {
                /* one period's mm, small: fits int32_t in 0.1 mm */
                quiet_mm10 -= (int32_t)(app_absf(step_mm) * 10.0f);
            }

            if ((app_absf(tilt) < FLAT_SIN) && (app_absf(roll_slow) < FLAT_SIN))
            {
                /* one period's mm, small: fits int32_t in 0.1 mm */
                flat_mm10 += (int32_t)(app_absf(step_mm) * 10.0f);
            }
            else
            {
                flat_mm10 = 0;
            }

            /* One wheel: PEAK_PAST_MM past its highest point and the side
               tilt level again (or a manoeuvre starts) */

            if (((past_mm10 > (PEAK_PAST_MM * 10)) &&
                 (app_absf(roll_slow) < FLAT_SIN)) ||
                b_flush)
            {
                /* scaled for driving (S4_ONE_WHEEL_GAIN_PCT), 0.1 mm */
                int32_t o10 =
                    (int32_t)(one_peak * 10.0f *
                              (float32_t)S4_ONE_WHEEL_GAIN_PCT / 100.0f);

                if ((o10 >= REPORT_MIN_MM10) && (quiet_mm10 <= 0))
                {
                    report_one_wheel(o10, roll_peak, one_pitch_peak);
                    quiet_mm10 = QUIET_AFTER_MM * 10;
                    height = peak = peak_sin = 0.0f;
                    flat_mm10                = 0;
                }

                one_peak = one_pitch_peak = roll_peak = 0.0f;
                past_mm10                             = 0;
            }

            if ((flat_mm10 > (FLAT_MM * 10)) || b_flush)
            {
                /* heights in mm, small: fit int32_t in 0.1 mm */
                int32_t p10 = (int32_t)(peak * 10.0f);
                /* the same, scaled for driving (S4_ONE_WHEEL_GAIN_PCT) */
                int32_t o10 =
                    (int32_t)(one_peak * 10.0f *
                              (float32_t)S4_ONE_WHEEL_GAIN_PCT / 100.0f);

                if ((o10 >= REPORT_MIN_MM10) && (o10 >= p10) &&
                    (quiet_mm10 <= 0))
                {
                    /* one wheel went over it (the course's hump) */
                    report_one_wheel(o10, roll_peak, one_pitch_peak);
                    quiet_mm10 = QUIET_AFTER_MM * 10;
                }
                else if ((p10 >= REPORT_MIN_MM10) && (quiet_mm10 <= 0))
                {
                    uint8_t msg[TELE_TEXT_MAX];
                    /* |sin| <= 1: L x sin in 0.1 mm and sin x 1000 fit
                       int32_t */
                    int32_t l10 = (int32_t)(peak_sin * S4_L_MM * 10.0f);

                    g_out.ramp_peak_mm10 = app_clamp(p10, 0, 4095);
                    g_ramp_reports++;
                    /* sin x 1000, |sin| <= 1: fits %d */
                    tm_sprintf(msg,
                               app_str("ramp peak height %d.%d mm "
                                       "(L*sin(peak pitch) "
                                       "%d.%d mm, peak sin %d/1000)"),
                               p10 / 10, p10 % 10, l10 / 10, l10 % 10,
                               (int32_t)(peak_sin * 1000.0f));
                    telemetry_event(msg);
                    quiet_mm10 = QUIET_AFTER_MM * 10;
                }
                else
                {
                    /* nothing to do for any other case */
                }

                height = peak = peak_sin = 0.0f; /* back on the floor */
                one_peak = one_pitch_peak = roll_peak = 0.0f;
                flat_mm10                             = 0;
                past_mm10                             = 0;
            }

            if (b_now)
            {
                /* manoeuvring: nothing measured, a fresh start after */
                height = peak = peak_sin = 0.0f;
                one_peak = one_pitch_peak = roll_peak = 0.0f;
                flat_mm10                             = 0;
                past_mm10                             = 0;
            }
        }
    done:
        app_timing_done(g_tmg_imu);
    }
}

/*** end of file ***/
