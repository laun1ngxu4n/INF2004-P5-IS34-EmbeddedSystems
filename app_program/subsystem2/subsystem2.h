/*
 *	subsystem2.h
 *	Subsystem 2 -- Motion Control (Buddy 2, Ng Chern Loong Branson).
 *
 *	This header is subsystem 2's PUBLIC interface: only what other
 *	subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *	files.  Other subsystems include it as "../subsystem2/subsystem2.h".
 *
 *	Stage 1: motors on GP8-GP11 (PWM 10 kHz), encoders on GP4/5 and
 *	GP16/17, a speed controller per wheel every 10 ms, and two commands:
 *	drive a distance and spin on the spot.  Line steering (IF-03) comes
 *	in Stage 2.
 *
 *	Measured in bring-up (bringup/docs/bringup_results.md): 2,540
 *	counts/rev, 65 mm wheel, 0.080 mm/count; the left encoder counts DOWN
 *	for forward (the counts below are already corrected: + = forward).
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"

/* Robot geometry */
#define S2_UM_PER_COUNT		80	/* 0.080 mm per encoder count */
#define S2_TRACK_MM		130	/* wheel centre to wheel centre.
					   NOT MEASURED YET: turns scale with
					   it (S2-M02 shows how to correct it) */

/* Speeds used by the team (mm/s) */
#define S2_SPEED_LINE		200	/* line following, design 10-25 cm/s */
#define S2_SPEED_BARCODE	150	/* over barcodes (TEAM.md 7.1 B4) */
#define S2_SPEED_TURN		120	/* each wheel during a spin */

/* Result of the last command */
#define MOT_RES_NONE		0	/* nothing run yet */
#define MOT_RES_BUSY		1	/* still running */
#define MOT_RES_DONE		2	/* reached its distance / angle */
#define MOT_RES_STOPPED		3	/* cancelled by app_stop_request() */
#define MOT_RES_TIMEOUT		4	/* took > 2x expected + 1 s (blocked?) */

typedef struct {
	UW	id;		/* id of the last command accepted */
	INT	result;		/* MOT_RES_... of that command */
	W	count_l;	/* encoder counts since boot, + = forward */
	W	count_r;
	INT	speed_l;	/* measured mm/s, + = forward (40 ms average) */
	INT	speed_r;
	UW	invalid;	/* quadrature errors since boot (missed edges) */
} MOTION_STATUS;

/* Commands.  Each returns the command id (> 0), or E_OBJ while a STOP
   reason is set.  A new command replaces a running one. */
IMPORT ID	motion_drive(INT speed_mm_s, INT dist_mm);	/* speed < 0 = reverse */
IMPORT ID	motion_turn(INT deg);				/* + = left (CCW) */
IMPORT ID	motion_follow(INT speed_mm_s);	/* follow the line (IF-03 steering from
						   line_steer_mm_s()) until another command */
IMPORT void	motion_halt(void);				/* stop, no STOP reason */
IMPORT void	motion_get_status(MOTION_STATUS *st);

/* Motors off now.  Any context, also interrupt handlers; used by the
   STOP helper (app_stop.c) and, later, Safety (TEAM.md section 3). */
IMPORT void	motor_safe_off(void);

/* Distance driven since boot, mean of both wheels, in encoder counts
   (0.080 mm each).  Any context, also interrupt handlers (barcode edges). */
IMPORT W	motion_odometer(void);
#define S2_COUNTS_TO_MM(c)	((c) * S2_UM_PER_COUNT / 1000)

/* Robot body (NOT MEASURED YET, used by obstacle bypass and junctions) */
#define S2_ROBOT_WIDTH_MM	150	/* outside of the wheels */
#define S2_ROBOT_LENGTH_MM	200	/* front bumper to back */

/* Private to subsystem 2, shared between its own .c files */
IMPORT ID	tid_motion;
IMPORT ID	cyc_motion;
IMPORT INT	tmg_motion;
IMPORT void	motion_task(INT stacd, void *exinf);
IMPORT ER	motion_hw_init(void);
IMPORT void	motor_write(INT duty_l, INT duty_r);	/* -1000..1000 per mille */
IMPORT void	encoder_read(W *l, W *r, UW *invalid);

#endif /* SUBSYSTEM2_H */
