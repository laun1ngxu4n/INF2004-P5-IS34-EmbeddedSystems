/*
 *	task_imu.c
 *	Subsystem 4 -- the IMU / Terrain Monitor task (every 10 ms).
 *
 *	Reads the LSM303D accelerometer (+/-4 g, 100 Hz) and tracks the ramp
 *	height: see subsystem4.h.  Register values are the ones bring-up
 *	verified (bringup/tests/imu/imu_test.c).
 */

#include "subsystem4.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../app_math.h"

#define IMU_ADDR	0x1D
#define REG_WHO_AM_I	0x0F
#define WHO_AM_I_VAL	0x49
#define REG_OUT_X_L_A	0x28

#define CAL_SAMPLES	100		/* 1 s */
#define LPF_K		0.1f		/* about 100 ms */
#define FLAT_SIN	0.030f		/* |pitch| < about 1.7 deg = level */
#define FLAT_MM		100		/* level for this far = back on the floor */
#define REPORT_MIN_MM10	80		/* peaks under 8 mm are not ramps */
#define QUIET_AFTER_MM	600		/* no 2nd report for the way down */
/* Tilt alone keeps |a| at 1 g; a bump or a speed change adds to it.  A
   sample more than this far from the calibrated |a| is a jolt, not tilt:
   keep the last pitch (Stage 4 run, 2 Nov: hitting a 10 mm hump read as
   up to 40 deg) */
#define JOLT_FRAC	0.10f	/* 0.05 skipped 30-48% of all samples while
				   driving on the flat floor (Stage 4 run, 6 Nov) */
/* Driving level: driving tilts the chassis a little (motor torque on the
   caster), which read as -2..-4 deg on the flat floor and drifted the
   height -10 mm before the hump (6 Nov).  While the tilt is within
   BASE_WIN of it, the level follows the tilt slowly (about 5 s); on a
   ramp or hump (10 mm hump: about 0.12) it stays put. */
#define BASE_K		0.002f
#define BASE_WIN	0.060f	/* about 3.4 deg */

IMPORT void	imu_i2c_init(void);
IMPORT ER	imu_i2c_read(UB addr, UB reg, UB *buf, INT n);
IMPORT ER	imu_i2c_write(UB addr, UB reg, UB val);

LOCAL volatile BOOL	ok;
LOCAL volatile BOOL	cal_request = TRUE;	/* calibrate once at boot too */
LOCAL volatile BOOL	calibrated;
LOCAL volatile INT	height_mm10;
LOCAL volatile INT	sin_x1000;
LOCAL volatile INT	ramp_peak_mm10;
LOCAL volatile UW	ramp_reports;
LOCAL volatile UW	jolts;

EXPORT BOOL imu_ok(void)		{ return ok; }
EXPORT void imu_calibrate(void)		{ calibrated = FALSE; cal_request = TRUE; }
EXPORT BOOL imu_calibrated(void)	{ return calibrated; }
EXPORT INT imu_height_mm10(void)	{ return height_mm10; }
EXPORT INT imu_pitch_mdeg_sin(void)	{ return sin_x1000; }
EXPORT INT imu_ramp_peak_mm10(void)	{ return ramp_peak_mm10; }
EXPORT UW imu_ramp_reports(void)		{ return ramp_reports; }
EXPORT UW imu_jolts(void)		{ return jolts; }

LOCAL BOOL chip_setup(void)
{
	UB who = 0;

	if (imu_i2c_read(IMU_ADDR, REG_WHO_AM_I, &who, 1) != E_OK || who != WHO_AM_I_VAL) {
		return FALSE;
	}
	return imu_i2c_write(IMU_ADDR, 0x20, 0x6F) == E_OK &&	/* CTRL1: 100 Hz, BDU, XYZ */
	       imu_i2c_write(IMU_ADDR, 0x21, 0xC8) == E_OK &&	/* CTRL2: +/-4 g, 50 Hz
								   anti-alias filter (was
								   773 Hz: motor vibration
								   folded into the tilt) */
	       imu_i2c_write(IMU_ADDR, 0x24, 0x70) == E_OK &&	/* CTRL5 */
	       imu_i2c_write(IMU_ADDR, 0x25, 0x20) == E_OK &&	/* CTRL6 */
	       imu_i2c_write(IMU_ADDR, 0x26, 0x00) == E_OK;	/* CTRL7: no HPF */
}

EXPORT void imu_task(INT stacd, void *exinf)
{
	INT	retry = 0, ncal = 0;
	float	cal_s = 0.0f, cal_c = 0.0f, s0 = 0.0f, c0 = 1.0f;
	float	cal_m = 0.0f, m0 = 0.0f;		/* |a| when level and still */
	float	sin_f = 0.0f, h = 0.0f, peak = 0.0f, peak_sin = 0.0f;
	float	base = 0.0f, tilt = 0.0f;	/* driving level; sin_f - base */
	W	odo_prev;
	INT	flat_mm10 = 0;
	INT	quiet_mm10 = 0;		/* distance left before another report */

	(void)stacd; (void)exinf;
	imu_i2c_init();
	odo_prev = motion_odometer();

	while (1) {
		UB	b[6];
		W	odo;

		tk_slp_tsk(TMO_FEVR);			/* woken every 10 ms */
		app_timing_wake(tmg_imu, tk_can_wup(TSK_SELF));

		if (!ok) {
			if (--retry <= 0) {
				retry = 100;		/* try again in 1 s */
				ok = chip_setup();
				if (ok) {
					tm_printf((UB*)"[IMU] LSM303D found at 0x1D\n");
				}
			}
			goto done;
		}
		if (imu_i2c_read(IMU_ADDR, REG_OUT_X_L_A, b, 6) != E_OK) {
			ok = FALSE;
			tele_event((const UB*)"IMU read failed, retrying");
			goto done;
		}
		{
			float	ax = (float)(H)(b[0] | (b[1] << 8));
			float	ay = (float)(H)(b[2] | (b[3] << 8));
			float	az = (float)(H)(b[4] | (b[5] << 8));
			float	mag = app_sqrtf(ax * ax + ay * ay + az * az);
			float	sp, cp, rel;

			if (mag < 1.0f) goto done;
			sp = ax / mag;				/* sin(pitch) */
			cp = app_sqrtf(ay * ay + az * az) / mag;	/* cos(pitch) */

			if (cal_request) {
				cal_s += sp;
				cal_c += cp;
				cal_m += mag;
				if (++ncal >= CAL_SAMPLES) {
					s0 = cal_s / CAL_SAMPLES;
					c0 = cal_c / CAL_SAMPLES;
					m0 = cal_m / CAL_SAMPLES;
					cal_s = cal_c = cal_m = 0.0f;
					ncal = 0;
					cal_request = FALSE;
					calibrated = TRUE;
					h = peak = peak_sin = 0.0f;
					sin_f = base = tilt = 0.0f;
				}
				goto done;
			}
			/* sin(pitch - level pitch), low-pass filtered; jolts
			   skipped (the pitch from before them is kept) */
			if (app_absf(mag - m0) <= JOLT_FRAC * m0) {
				rel = sp * c0 - cp * s0;
				sin_f += LPF_K * (rel - sin_f);
				if (app_absf(sin_f - base) < BASE_WIN) {
					base += BASE_K * (sin_f - base);
				}
				tilt = sin_f - base;
				sin_x1000 = (INT)(tilt * 1000.0f);
			} else {
				jolts++;
			}
		}

		/* Height: distance driven x sin(pitch) */
		odo = motion_odometer();
		{
			float	ds = (float)(odo - odo_prev) * S2_UM_PER_COUNT / 1000.0f;

			odo_prev = odo;
			h += ds * tilt;
			height_mm10 = (INT)(h * 10.0f);
			/* climbs only: a nose-down dip (rear wheels on the
			   hump, or coming off it) is not a ramp (6 Nov run 1
			   reported a -56 mm dip as 56 mm) */
			if (h > peak) peak = h;
			if (tilt > peak_sin) peak_sin = tilt;

			if (quiet_mm10 > 0) {
				quiet_mm10 -= (INT)(app_absf(ds) * 10.0f);
			}
			if (app_absf(tilt) < FLAT_SIN) {
				flat_mm10 += (INT)(app_absf(ds) * 10.0f);
			} else {
				flat_mm10 = 0;
			}
			if (flat_mm10 > FLAT_MM * 10) {
				INT p10 = (INT)(peak * 10.0f);

				if (p10 >= REPORT_MIN_MM10 && quiet_mm10 <= 0) {
					UB	msg[TELE_TEXT_MAX];
					INT	l10 = (INT)(peak_sin * S4_L_MM * 10.0f);

					ramp_peak_mm10 = p10;
					ramp_reports++;
					tm_sprintf(msg, (const UB*)
						"ramp peak height %d.%d mm (L*sin(peak pitch) "
						"%d.%d mm, peak sin %d/1000)",
						p10 / 10, p10 % 10, l10 / 10, l10 % 10,
						(INT)(peak_sin * 1000.0f));
					tele_event(msg);
					quiet_mm10 = QUIET_AFTER_MM * 10;
				}
				h = peak = peak_sin = 0.0f;	/* back on the floor */
				flat_mm10 = 0;
			}
		}
done:
		app_timing_done(tmg_imu);
	}
}
