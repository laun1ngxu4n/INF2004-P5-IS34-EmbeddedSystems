/*
 *	task_imu.c
 *	Subsystem 4 -- the IMU / Terrain Monitor task.
 *
 *	Stage 0: wakes every 10 ms, is timed, does not read the IMU yet.
 */

#include "subsystem4.h"

EXPORT void imu_task(INT stacd, void *exinf)
{
	(void)stacd; (void)exinf;

	while (1) {
		tk_slp_tsk(TMO_FEVR);			/* woken every 10 ms */
		app_timing_wake(tmg_imu, tk_can_wup(TSK_SELF));

		/* Stage 2: read the LSM303D, pitch filter, hump and shock rules. */

		app_timing_done(tmg_imu);
	}
}
