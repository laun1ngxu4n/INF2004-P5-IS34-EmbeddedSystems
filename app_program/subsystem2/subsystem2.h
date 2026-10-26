/*
 *	subsystem2.h
 *	Subsystem 2 -- Motion Control (Buddy 2, Ng Chern Loong Branson).
 *
 *	This header is subsystem 2's PUBLIC interface: only what other
 *	subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *	files.  Other subsystems include it as "../subsystem2/subsystem2.h".
 *
 *	Stage 0: the Motion task runs every 10 ms and does no motor work.
 *	Stage 1 adds PWM on GP8-GP11 and the encoders on GP4/5 and GP16/17
 *	(bring-up: about 2,540 counts/rev, left counts down for forward), then
 *	PID speed control and line steering from Line Following.
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"

/* Private to subsystem 2, shared between its own .c files */
IMPORT ID	tid_motion;
IMPORT ID	cyc_motion;
IMPORT INT	tmg_motion;
IMPORT void	motion_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM2_H */
