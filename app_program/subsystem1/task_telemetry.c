/*
 *	task_telemetry.c
 *	Subsystem 1 -- the Telemetry task (Notification merged in).
 *
 *	Stage 0: every 100 ms it wakes and is timed; once a second it sends
 *	IF-01 (MSN_FLG_PING to Mission) and prints the timing table.
 *	Later stages add: LED/buzzer patterns on this 100 ms tick, the 1 Hz
 *	telemetry/heartbeat handed to the MQTT code in the WiFi service task,
 *	and the WIFI_LOSS warning from cyw43_utk_get_status().
 */

#include "subsystem1.h"
#include "../subsystem5/subsystem5.h"	/* IF-01: flg_mission */

EXPORT void telemetry_task(INT stacd, void *exinf)
{
	UINT	report_tick = 0;
	UINT	ping_tick = 0;

	(void)stacd; (void)exinf;

	while (1) {
		tk_slp_tsk(TMO_FEVR);			/* woken every 100 ms */
		app_timing_wake(tmg_telemetry, tk_can_wup(TSK_SELF));

		if (++ping_tick >= S1_PING_TICKS) {
			ping_tick = 0;
			CHECK(tk_set_flg(flg_mission, MSN_FLG_PING), "IF-01 ping");
		}
		if (++report_tick >= S1_REPORT_TICKS) {
			report_tick = 0;
			/* The report's printing counts in this task's run time on
			   purpose: it is real Telemetry load at the lowest priority. */
			app_timing_report();
		}

		app_timing_done(tmg_telemetry);
	}
}
