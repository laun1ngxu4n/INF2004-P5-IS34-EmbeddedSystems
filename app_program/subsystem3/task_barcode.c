/*
 *	task_barcode.c
 *	Subsystem 3 -- the Barcode Decoder task.
 *
 *	Event-driven in the design: woken by each IR_BARCODE edge, with a
 *	10 ms timeout to check the end-of-barcode quiet zone.  Stage 0 only
 *	has the timeout, so it wakes every 10-20 ms (a kernel timeout ends on
 *	the next 10 ms tick).
 */

#include "subsystem3.h"

EXPORT void barcode_task(INT stacd, void *exinf)
{
	(void)stacd; (void)exinf;

	while (1) {
		/* E_TMOUT is the normal Stage 0 result; an edge wake-up later */
		(void)tk_slp_tsk(TIMEOUT_BARCODE_MS);
		app_timing_wake(tmg_barcode, 0);

		/* Stage 2: drain the edge ring, measure bars, decode Code 39. */

		app_timing_done(tmg_barcode);
	}
}
