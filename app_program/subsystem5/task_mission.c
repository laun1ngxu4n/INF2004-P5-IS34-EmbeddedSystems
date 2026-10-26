/*
 *	task_mission.c
 *	Subsystem 5 -- the Mission Controller task.
 *
 *	The design's Mission waits on mission_flg with a <= 100 ms timeout and
 *	then drains every input queue.  Stage 0 waits the same way and only
 *	counts the IF-01 pings from Telemetry.
 */

#include "subsystem5.h"

EXPORT void mission_task(INT stacd, void *exinf)
{
	UINT	ptn;
	ER	er;

	(void)stacd; (void)exinf;

	while (1) {
		er = tk_wai_flg(flg_mission, MSN_FLG_ALL, TWF_ORW | TWF_BITCLR,
				&ptn, TIMEOUT_MISSION_MS);
		app_timing_wake(tmg_mission, 0);

		if (er == E_OK && (ptn & MSN_FLG_PING) != 0u) {
			mission_pings++;
		}
		/* Stage 1: drain inputs with TMO_POL, apply the precedence rules. */

		app_timing_done(tmg_mission);
	}
}
