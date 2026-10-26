/*
 *	task_obstacle.c
 *	Subsystem 5 -- the Obstacle Scanner task.
 *
 *	Stage 0: wakes every 60 ms, is timed, does not ping yet.
 */

#include "subsystem5.h"

EXPORT void obstacle_task(INT stacd, void *exinf)
{
	(void)stacd; (void)exinf;

	while (1) {
		tk_slp_tsk(TMO_FEVR);			/* woken every 60 ms */
		app_timing_wake(tmg_obstacle, tk_can_wup(TSK_SELF));

		/* Stage 3: trigger a ping, classify the echo, obstacle events. */

		app_timing_done(tmg_obstacle);
	}
}
