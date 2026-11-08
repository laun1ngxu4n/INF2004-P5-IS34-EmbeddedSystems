/*
 *	subsystem4.h
 *	Subsystem 4 -- Safety / Fault Monitor + IMU / Terrain Monitor
 *	(Buddy 4, Muhammad Haziq).
 *
 *	This header is subsystem 4's PUBLIC interface: only what other
 *	subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *	files.  Other subsystems include it as "../subsystem4/subsystem4.h".
 *
 *	Stage 0: Safety runs every 20 ms at the highest application priority;
 *	the IMU task runs every 10 ms.  Neither touches hardware yet.  The IMU
 *	is an LSM303D at 0x1D on I2C1 (GP26 SDA, GP27 SCL), use 400 kHz.
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

/*
 * Stage 4: IMU (LSM303D at 0x1D on I2C1, GP26/GP27, 400 kHz, +/-4 g).
 * Ramp height (TEAM.md 7, ramp): height = sum of (distance driven x
 * sin(pitch)), pitch relative to the level pitch measured at START
 * (imu_calibrate).  Integration needs the encoders, so acceleration
 * spikes (the bench saw up to 6 deg of false pitch) are smoothed by a
 * low-pass filter and only count while the robot moves.  When the robot is
 * level again after a climb, the peak is reported (tele_event) and kept
 * for telemetry.  L x sin(peak pitch) (L = S4_L_MM) is reported beside it.
 * Safety (collision, stall) is not in the essentials plan: safety_task
 * stays the Stage 0 timing stub.
 */
#define S4_L_MM			80	/* axle to front support, measured 6 Nov */

IMPORT BOOL	imu_ok(void);			/* chip found and configured */
IMPORT void	imu_calibrate(void);		/* robot level and still: 1 s */
IMPORT BOOL	imu_calibrated(void);
IMPORT INT	imu_height_mm10(void);		/* current height, 0.1 mm */
IMPORT INT	imu_pitch_mdeg_sin(void);	/* sin(pitch) x 1000, relative */
IMPORT INT	imu_roll_sin_x1000(void);	/* sin(side tilt) x 1000, relative */
IMPORT INT	imu_one_wheel_mm10(void);	/* one wheel up: T x sin(side tilt) */
IMPORT INT	imu_ramp_peak_mm10(void);	/* last reported ramp peak, 0.1 mm */
IMPORT UW	imu_ramp_reports(void);		/* ramps reported since boot */
IMPORT UW	imu_jolts(void);		/* samples skipped as jolts since boot */

/* Private to subsystem 4, shared between its own .c files */
IMPORT ID	tid_safety;
IMPORT ID	cyc_safety;
IMPORT INT	tmg_safety;
IMPORT ID	tid_imu;
IMPORT ID	cyc_imu;
IMPORT INT	tmg_imu;
IMPORT void	safety_task(INT stacd, void *exinf);
IMPORT void	imu_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM4_H */
