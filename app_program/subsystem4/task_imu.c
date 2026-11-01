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

EXPORT BOOL imu_ok(void)		{ return ok; }
EXPORT void imu_calibrate(void)		{ calibrated = FALSE; cal_request = TRUE; }
EXPORT BOOL imu_calibrated(void)	{ return calibrated; }
EXPORT INT imu_height_mm10(void)	{ return height_mm10; }
EXPORT INT imu_pitch_mdeg_sin(void)	{ return sin_x1000; }
EXPORT INT imu_ramp_peak_mm10(void)	{ return ramp_peak_mm10; }
EXPORT UW imu_ramp_reports(void)		{ return ramp_reports; }

LOCAL float sqrt_f(float x)
{
	float	r = (x > 1.0f) ? x : 1.0f;
	INT	i;

	if (x <= 0.0f) return 0.0f;
	for (i = 0; i < 12; i++) r = 0.5f * (r + x / r);
	return r;
}

LOCAL float abs_f(float x)	{ return (x < 0.0f) ? -x : x; }

LOCAL BOOL chip_setup(void)
{
	UB who = 0;

	if (imu_i2c_read(IMU_ADDR, REG_WHO_AM_I, &who, 1) != E_OK || who != WHO_AM_I_VAL) {
		return FALSE;
	}
	return imu_i2c_write(IMU_ADDR, 0x20, 0x6F) == E_OK &&	/* CTRL1: 100 Hz, BDU, XYZ */
	       imu_i2c_write(IMU_ADDR, 0x21, 0x08) == E_OK &&	/* CTRL2: +/-4 g */
	       imu_i2c_write(IMU_ADDR, 0x24, 0x70) == E_OK &&	/* CTRL5 */
	       imu_i2c_write(IMU_ADDR, 0x25, 0x20) == E_OK &&	/* CTRL6 */
	       imu_i2c_write(IMU_ADDR, 0x26, 0x00) == E_OK;	/* CTRL7: no HPF */
}

EXPORT void imu_task(INT stacd, void *exinf)
{
	INT	retry = 0, ncal = 0;
	float	cal_s = 0.0f, cal_c = 0.0f, s0 = 0.0f, c0 = 1.0f;
	float	sin_f = 0.0f, h = 0.0f, peak = 0.0f, peak_sin = 0.0f;
	W	odo_prev;
	INT	flat_mm10 = 0;
	W	quiet_until = 0;

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
			float	mag = sqrt_f(ax * ax + ay * ay + az * az);
			float	sp, cp, rel;

			if (mag < 1.0f) goto done;
			sp = ax / mag;				/* sin(pitch) */
			cp = sqrt_f(ay * ay + az * az) / mag;	/* cos(pitch) */

			if (cal_request) {
				cal_s += sp;
				cal_c += cp;
				if (++ncal >= CAL_SAMPLES) {
					s0 = cal_s / CAL_SAMPLES;
					c0 = cal_c / CAL_SAMPLES;
					cal_s = cal_c = 0.0f;
					ncal = 0;
					cal_request = FALSE;
					calibrated = TRUE;
					h = peak = peak_sin = 0.0f;
					sin_f = 0.0f;
				}
				goto done;
			}
			/* sin(pitch - level pitch), low-pass filtered */
			rel = sp * c0 - cp * s0;
			sin_f += LPF_K * (rel - sin_f);
			sin_x1000 = (INT)(sin_f * 1000.0f);
		}

		/* Height: distance driven x sin(pitch) */
		odo = motion_odometer();
		{
			float	ds = (float)(odo - odo_prev) * S2_UM_PER_COUNT / 1000.0f;

			odo_prev = odo;
			h += ds * sin_f;
			height_mm10 = (INT)(h * 10.0f);
			if (abs_f(h) > abs_f(peak)) peak = h;
			if (abs_f(sin_f) > abs_f(peak_sin)) peak_sin = sin_f;

			if (abs_f(sin_f) < FLAT_SIN) {
				flat_mm10 += (INT)(abs_f(ds) * 10.0f);
			} else {
				flat_mm10 = 0;
			}
			if (flat_mm10 > FLAT_MM * 10) {
				INT p10 = (INT)(abs_f(peak) * 10.0f);

				if (p10 >= REPORT_MIN_MM10 && odo - quiet_until >= 0) {
					UB	msg[TELE_TEXT_MAX];
					INT	l10 = (INT)(abs_f(peak_sin) * S4_L_MM * 10.0f);

					ramp_peak_mm10 = p10;
					ramp_reports++;
					tm_sprintf(msg, (const UB*)
						"ramp peak height %d.%d mm (L*sin(peak pitch) "
						"%d.%d mm, peak sin %d/1000)",
						p10 / 10, p10 % 10, l10 / 10, l10 % 10,
						(INT)(abs_f(peak_sin) * 1000.0f));
					tele_event(msg);
					quiet_until = odo + QUIET_AFTER_MM * 1000 / S2_UM_PER_COUNT;
				}
				h = peak = peak_sin = 0.0f;	/* back on the floor */
				flat_mm10 = 0;
			}
		}
done:
		app_timing_done(tmg_imu);
	}
}
