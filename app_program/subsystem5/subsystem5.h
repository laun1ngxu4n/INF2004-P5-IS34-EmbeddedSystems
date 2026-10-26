/*
 *	subsystem5.h
 *	Subsystem 5 -- Mission Controller + Obstacle Scanner
 *	(Buddy 5, Muhammad Irfan).
 *
 *	This header is subsystem 5's PUBLIC interface: only what other
 *	subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *	files.  Other subsystems include it as "../subsystem5/subsystem5.h".
 *
 *	Stage 0: Mission waits on flg_mission with a 100 ms timeout; the
 *	Obstacle Scanner runs every 60 ms (the design's ping period) without
 *	pinging yet (TRIG GP6, ECHO GP7; works at 3.3 V on the bench).
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"

/* IF-01: Mission's wake-up event flag, created and owned by subsystem 5.
   Other subsystems only set bits; Mission waits and clears them. */
IMPORT ID	flg_mission;
#define MSN_FLG_PING	0x0001u		/* Stage 0 check from Telemetry, 1 Hz */
#define MSN_FLG_ALL	0x0001u		/* every bit Mission waits for */

/* Private to subsystem 5, shared between its own .c files */
IMPORT ID	tid_mission;
IMPORT INT	tmg_mission;
IMPORT UW	mission_pings;		/* written only by the Mission task */
IMPORT ID	tid_obstacle;
IMPORT ID	cyc_obstacle;
IMPORT INT	tmg_obstacle;
IMPORT void	mission_task(INT stacd, void *exinf);
IMPORT void	obstacle_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM5_H */
