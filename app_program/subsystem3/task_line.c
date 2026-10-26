/*
 *	task_line.c
 *	Subsystem 3 -- the Line Following task.
 *
 *	Stage 0: wakes every 20 ms, is timed, does no sensing yet.
 */

#include "subsystem3.h"

EXPORT void line_task(INT stacd, void *exinf)
{
	(void)stacd; (void)exinf;

	while (1) {
		tk_slp_tsk(TMO_FEVR);			/* woken every 20 ms */
		app_timing_wake(tmg_line, tk_can_wup(TSK_SELF));

		/* Stage 1: read IR_LEFT / IR_CENTER, send steering to Motion. */

		app_timing_done(tmg_line);
	}
}
