/*
 *  task_imu.c
 *  Subsystem 4 -- the IMU / Terrain Monitor task (every 10 ms).
 *
 *  Reads the LSM303D accelerometer (+/-4 g, 100 Hz) and tracks the ramp
 *  height: see subsystem4.h.  Register values are the ones bring-up
 *  verified (bringup/tests/imu/imu_test.c).
 */

#include "subsystem4.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../app_math.h"

#define IMU_ADDR      0x1D
#define REG_WHO_AM_I  0x0F
#define WHO_AM_I_VAL  0x49
#define REG_OUT_X_L_A 0x28

#define CAL_SAMPLES     100    /* 1 s */
#define LPF_K           0.1f   /* about 100 ms */
#define FLAT_SIN        0.030f /* |pitch| < about 1.7 deg = level */
#define FLAT_MM         100    /* level for this far = back on the floor */
#define REPORT_MIN_MM10 80     /* peaks under 8 mm are not ramps */
#define QUIET_AFTER_MM  600    /* no 2nd report for the way down */
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

IMPORT void imu_i2c_init (void);
IMPORT ER   imu_i2c_read (UB addr, UB reg, UB * buf, INT n);
IMPORT ER   imu_i2c_write (UB addr, UB reg, UB val);

LOCAL volatile BOOL gb_chip_ok;
LOCAL volatile BOOL gb_cal_request = TRUE; /* calibrate once at boot too */
LOCAL volatile BOOL gb_calibrated;
LOCAL volatile INT  g_height_mm10;
LOCAL volatile INT  g_sin_x1000;
LOCAL volatile INT  g_roll_x1000;
LOCAL volatile INT  g_one_h_mm10; /* one-wheel height now, 0.1 mm */
LOCAL volatile INT  g_ramp_peak_mm10;
LOCAL volatile UW   g_ramp_reports;
LOCAL volatile UW   g_jolts;

EXPORT BOOL imu_ok (void)
{
    return gb_chip_ok;
}
EXPORT void imu_calibrate (void)
{
    gb_calibrated  = FALSE;
    gb_cal_request = TRUE;
}
EXPORT BOOL imu_calibrated (void)
{
    return gb_calibrated;
}
EXPORT INT imu_height_mm10 (void)
{
    return g_height_mm10;
}
EXPORT INT imu_pitch_mdeg_sin (void)
{
    return g_sin_x1000;
}
EXPORT INT imu_roll_sin_x1000 (void)
{
    return g_roll_x1000;
}
EXPORT INT imu_one_wheel_mm10 (void)
{
    return g_one_h_mm10;
}
EXPORT INT imu_ramp_peak_mm10 (void)
{
    return g_ramp_peak_mm10;
}
EXPORT UW imu_ramp_reports (void)
{
    return g_ramp_reports;
}
EXPORT UW imu_jolts (void)
{
    return g_jolts;
}

LOCAL BOOL chip_setup (void)
{
    UB who = 0;

    if ((imu_i2c_read(IMU_ADDR, REG_WHO_AM_I, &who, 1) != E_OK) ||
        (who != WHO_AM_I_VAL))
    {
        return FALSE;
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

EXPORT void imu_task (INT stacd, void * p_exinf)
{
    INT   retry          = 0;
    INT   ncal           = 0;
    float cal_s          = 0.0f;
    float cal_c          = 0.0f;
    float sin_level      = 0.0f;
    float cos_level      = 1.0f;
    float cal_m          = 0.0f;
    float mag_level      = 0.0f; /* |a| when level and still */
    float cal_y          = 0.0f;
    float side_level     = 0.0f; /* sideways share when level */
    float roll_f         = 0.0f;
    float base_r         = 0.0f;
    float roll           = 0.0f;
    float one_peak       = 0.0f;
    float one_pitch_peak = 0.0f;
    float roll_peak      = 0.0f;
    float sin_f          = 0.0f;
    float height         = 0.0f;
    float peak           = 0.0f;
    float peak_sin       = 0.0f;
    float base           = 0.0f;
    float tilt           = 0.0f; /* driving level; sin_f - base */
    W     odo_prev;
    INT   flat_mm10  = 0;
    INT   quiet_mm10 = 0; /* distance left before another report */

    (void)stacd;
    (void)p_exinf;
    imu_i2c_init();
    odo_prev = motion_odometer();

    for (;;)
    {
        UB raw[6];
        W  odo;

        tk_slp_tsk(TMO_FEVR); /* woken every 10 ms */
        app_timing_wake(g_tmg_imu, tk_can_wup(TSK_SELF));

        if (!gb_chip_ok)
        {
            if (--retry <= 0)
            {
                retry      = 100; /* try again in 1 s */
                gb_chip_ok = chip_setup();
                if (gb_chip_ok)
                {
                    tm_printf((UB *)"[IMU] LSM303D found at 0x1D\n");
                }
            }
            goto done;
        }
        if (imu_i2c_read(IMU_ADDR, REG_OUT_X_L_A, raw, 6) != E_OK)
        {
            gb_chip_ok = FALSE;
            tele_event((const UB *)"IMU read failed, retrying");
            goto done;
        }
        {
            float acc_x = (float)(H)(raw[0] | (raw[1] << 8));
            float acc_y = (float)(H)(raw[2] | (raw[3] << 8));
            float acc_z = (float)(H)(raw[4] | (raw[5] << 8));
            float mag =
                app_sqrtf((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));
            float sin_pitch;
            float cos_pitch;
            float rel;

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
                if (++ncal >= CAL_SAMPLES)
                {
                    sin_level  = cal_s / CAL_SAMPLES;
                    cos_level  = cal_c / CAL_SAMPLES;
                    mag_level  = cal_m / CAL_SAMPLES;
                    side_level = cal_y / CAL_SAMPLES;
                    cal_s = cal_c = cal_m = cal_y = 0.0f;
                    ncal                          = 0;
                    gb_cal_request                = FALSE;
                    gb_calibrated                 = TRUE;
                    height = peak = peak_sin = 0.0f;
                    sin_f = base = tilt = 0.0f;
                    roll_f = base_r = roll = 0.0f;
                    one_peak = one_pitch_peak = roll_peak = 0.0f;
                }
                goto done;
            }
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
                tilt        = sin_f - base;
                g_sin_x1000 = (INT)(tilt * 1000.0f);
                /* side tilt, the same way (level offset, filter,
                   slowly following driving level) */
                roll_f += ROLL_LPF_K * (((acc_y / mag) - side_level) - roll_f);
                if (app_absf(roll_f - base_r) < BASE_WIN)
                {
                    base_r += BASE_K * (roll_f - base_r);
                }
                roll         = roll_f - base_r;
                g_roll_x1000 = (INT)(roll * 1000.0f);
            }
            else
            {
                g_jolts++;
            }
        }

        /* Height: distance driven x sin(pitch) */
        odo = motion_odometer();
        {
            float step_mm = (float)(odo - odo_prev) * S2_UM_PER_COUNT / 1000.0f;

            odo_prev = odo;
            height += step_mm * tilt;
            g_height_mm10 = (INT)(height * 10.0f);
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
                float roll_abs    = app_absf(roll);
                float height_roll = (float)S2_TRACK_MM * roll_abs;

                g_one_h_mm10 = (INT)(height_roll * 10.0f);
                if (height_roll > one_peak)
                {
                    one_peak  = height_roll;
                    roll_peak = roll_abs;
                }
                if (tilt < 0.0f)
                {
                    float height_pitch = 2.0f * (float)S4_L_MM * -tilt;

                    if (height_pitch > one_pitch_peak)
                    {
                        one_pitch_peak = height_pitch;
                    }
                }
            }

            if (quiet_mm10 > 0)
            {
                quiet_mm10 -= (INT)(app_absf(step_mm) * 10.0f);
            }
            if ((app_absf(tilt) < FLAT_SIN) && (app_absf(roll) < FLAT_SIN))
            {
                flat_mm10 += (INT)(app_absf(step_mm) * 10.0f);
            }
            else
            {
                flat_mm10 = 0;
            }
            if (flat_mm10 > (FLAT_MM * 10))
            {
                INT p10 = (INT)(peak * 10.0f);
                INT o10 = (INT)(one_peak * 10.0f);

                if ((o10 >= REPORT_MIN_MM10) && (o10 >= p10) &&
                    (quiet_mm10 <= 0))
                {
                    /* one wheel went over it (the course's hump) */
                    UB  msg[TELE_TEXT_MAX];
                    INT c10 = (INT)(one_pitch_peak * 10.0f);

                    g_ramp_peak_mm10 = o10;
                    g_ramp_reports++;
                    tm_sprintf(msg,
                               (const UB *)"ramp peak height %d.%d mm (one "
                                           "wheel: side tilt "
                                           "sin %d/1000; from the nose-down "
                                           "tilt %d.%d mm)",
                               o10 / 10, o10 % 10, (INT)(roll_peak * 1000.0f),
                               c10 / 10, c10 % 10);
                    tele_event(msg);
                    quiet_mm10 = QUIET_AFTER_MM * 10;
                }
                else if ((p10 >= REPORT_MIN_MM10) && (quiet_mm10 <= 0))
                {
                    UB  msg[TELE_TEXT_MAX];
                    INT l10 = (INT)(peak_sin * S4_L_MM * 10.0f);

                    g_ramp_peak_mm10 = p10;
                    g_ramp_reports++;
                    tm_sprintf(msg,
                               (const UB *)"ramp peak height %d.%d mm "
                                           "(L*sin(peak pitch) "
                                           "%d.%d mm, peak sin %d/1000)",
                               p10 / 10, p10 % 10, l10 / 10, l10 % 10,
                               (INT)(peak_sin * 1000.0f));
                    tele_event(msg);
                    quiet_mm10 = QUIET_AFTER_MM * 10;
                }
                height = peak = peak_sin = 0.0f; /* back on the floor */
                one_peak = one_pitch_peak = roll_peak = 0.0f;
                flat_mm10                             = 0;
            }
        }
    done:
        app_timing_done(g_tmg_imu);
    }
}

/*** end of file ***/

