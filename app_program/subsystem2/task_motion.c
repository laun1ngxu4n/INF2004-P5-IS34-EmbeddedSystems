/*
 *	task_motion.c
 *	Subsystem 2 -- the Motion Control task.
 *
 *	Stage 0: wakes every 10 ms, is timed, does no motor work.
 */

#include "subsystem2.h"

EXPORT void motion_task(INT stacd, void *exinf)
{
	(void)stacd; (void)exinf;

	while (1) {
		tk_slp_tsk(TMO_FEVR);			/* woken every 10 ms */
		app_timing_wake(tmg_motion, tk_can_wup(TSK_SELF));

		/* Stage 1: read encoders, PID, write PWM here. */

		app_timing_done(tmg_motion);
	}
}
