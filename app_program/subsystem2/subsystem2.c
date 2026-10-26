/*
 *	subsystem2.c
 *	Subsystem 2 -- Motion Control: creates its objects and starts its task.
 */

#include "subsystem2.h"

EXPORT ID	tid_motion;
EXPORT ID	cyc_motion;
EXPORT INT	tmg_motion;

EXPORT ER subsystem2_init(void)
{
	tmg_motion = app_timing_add("motion", PRI_MOTION, PERIOD_MOTION_MS);
	cyc_motion = app_create_wake_cyclic(&tid_motion, PERIOD_MOTION_MS);
	if (tmg_motion < E_OK) {
		return tmg_motion;
	}
	return (cyc_motion < E_OK) ? cyc_motion : E_OK;
}

EXPORT ER subsystem2_start(void)
{
	tid_motion = app_start_task(motion_task, PRI_MOTION, 0, "motion_task");
	if (tid_motion < E_OK) {
		return tid_motion;
	}
	return tk_sta_cyc(cyc_motion);
}
