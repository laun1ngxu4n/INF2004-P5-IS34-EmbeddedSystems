/*
 *	subsystem5.c
 *	Subsystem 5 -- Mission + Obstacle: creates its objects and starts its
 *	tasks.
 */

#include "subsystem5.h"

EXPORT ID	flg_mission;		/* IF-01 */
EXPORT ID	tid_mission;
EXPORT INT	tmg_mission;
EXPORT UW	mission_pings;
EXPORT ID	tid_obstacle;
EXPORT ID	cyc_obstacle;
EXPORT INT	tmg_obstacle;

EXPORT ER subsystem5_init(void)
{
	T_CFLG	cflg = {0};

	/* Several setters (Telemetry now; buttons, Safety, MQTT later), one
	   waiter (Mission), which clears the bits it has handled. */
	cflg.flgatr = TA_TFIFO | TA_WSGL;
	cflg.iflgptn = 0;
	flg_mission = tk_cre_flg(&cflg);
	if (flg_mission < E_OK) {
		return flg_mission;
	}

	tmg_mission = app_timing_add("mission", PRI_MISSION, TIMEOUT_MISSION_MS);
	tmg_obstacle = app_timing_add("obstacle", PRI_OBSTACLE, PERIOD_OBSTACLE_MS);
	cyc_obstacle = app_create_wake_cyclic(&tid_obstacle, PERIOD_OBSTACLE_MS);
	if (tmg_mission < E_OK) {
		return tmg_mission;
	}
	if (tmg_obstacle < E_OK) {
		return tmg_obstacle;
	}
	return (cyc_obstacle < E_OK) ? cyc_obstacle : E_OK;
}

EXPORT ER subsystem5_start(void)
{
	tid_mission = app_start_task(mission_task, PRI_MISSION, 0, "mission_task");
	if (tid_mission < E_OK) {
		return tid_mission;
	}
	tid_obstacle = app_start_task(obstacle_task, PRI_OBSTACLE, 0, "obstacle_task");
	if (tid_obstacle < E_OK) {
		return tid_obstacle;
	}
	return tk_sta_cyc(cyc_obstacle);
}
