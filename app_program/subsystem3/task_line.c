/*
 *	task_line.c
 *	Subsystem 3 -- the Line Following task (every 20 ms).
 *
 *	IR_LEFT sits just left of the line, IR_CENTER over it:
 *	  centre black, left white   on the line           steer 0
 *	  centre black, left black   drifting right, or a junction: steer left
 *	  centre white, left black   far right of the line  steer left hard
 *	  centre white, left white   left of the line (or lost): steer right,
 *	                             LINE_LOST after S3_LOST_MM
 *	Junction: both black for S3_JUNC_MIN_MM..S3_JUNC_MAX_MM of travel,
 *	then left white again with centre still black.  Mission is told
 *	through flg_mission (MSN_FLG_JUNCTION / MSN_FLG_LINE_LOST).
 */

#include "subsystem3.h"
#include "../subsystem2/subsystem2.h"	/* motion_odometer() */
#include "../subsystem5/subsystem5.h"	/* flg_mission */

#define STEER_SMALL	40	/* mm/s */
#define STEER_HARD	110
#define S3_LOST_MM	250
#define S3_JUNC_MIN_MM	8
#define S3_JUNC_MAX_MM	45

LOCAL volatile INT	steer;
LOCAL volatile INT	state = LINE_ON;
LOCAL volatile UW	junctions;
LOCAL volatile BOOL	enabled = TRUE;

EXPORT INT line_steer_mm_s(void)	{ return steer; }
EXPORT INT line_state(void)		{ return state; }
EXPORT UW line_junctions(void)		{ return junctions; }
EXPORT BOOL line_center_black(void)	{ return app_gpio_get(PIN_IR_CENTER) == IR_BLACK; }
EXPORT void line_set_enabled(BOOL on)	{ enabled = on; }

EXPORT ER line_hw_init(void)
{
	ER er = app_gpio_input(PIN_IR_LEFT, FALSE);	/* modules drive D0 */

	return (er < E_OK) ? er : app_gpio_input(PIN_IR_CENTER, FALSE);
}

EXPORT void line_task(INT stacd, void *exinf)
{
	W	lost_from = 0, both_from = 0;
	BOOL	both = FALSE, was_enabled = TRUE;
	INT	last_side = 0;		/* +1 line last seen to the left, -1 right */

	(void)stacd; (void)exinf;

	while (1) {
		BOOL	l, c;
		W	odo;
		INT	s;

		tk_slp_tsk(TMO_FEVR);			/* woken every 20 ms */
		app_timing_wake(tmg_line, tk_can_wup(TSK_SELF));

		l = (app_gpio_get(PIN_IR_LEFT) == IR_BLACK);
		c = (app_gpio_get(PIN_IR_CENTER) == IR_BLACK);
		odo = motion_odometer();
		if (enabled && !was_enabled) {
			both = FALSE;			/* start clean after a manoeuvre */
			lost_from = odo;
		}
		was_enabled = enabled;

		/* Steering */
		if (c && !l) {
			s = 0;
			last_side = 0;
		} else if (c && l) {
			s = STEER_SMALL;
			last_side = +1;
		} else if (!c && l) {
			s = STEER_HARD;
			last_side = +1;
		} else {
			/* both white: the line is to the right, unless we last
			   saw it to the left */
			s = (last_side > 0) ? STEER_HARD : -STEER_HARD;
		}
		steer = s;

		/* Lost / found */
		if (c || l) {
			lost_from = odo;
			if (state != LINE_ON) {
				state = LINE_ON;
			}
		} else if (state == LINE_ON) {
			state = LINE_SEARCH;
		} else if (state == LINE_SEARCH && enabled &&
			   S2_COUNTS_TO_MM(odo - lost_from) > S3_LOST_MM) {
			state = LINE_LOST;
			tk_set_flg(flg_mission, MSN_FLG_LINE_LOST);
		}

		/* Junction */
		if (c && l) {
			if (!both) {
				both = TRUE;
				both_from = odo;
			}
		} else if (both) {
			INT len = (INT)S2_COUNTS_TO_MM(odo - both_from);

			both = FALSE;
			if (enabled && c && !l &&
			    len >= S3_JUNC_MIN_MM && len <= S3_JUNC_MAX_MM) {
				junctions++;
				tk_set_flg(flg_mission, MSN_FLG_JUNCTION);
			}
		}

		app_timing_done(tmg_line);
	}
}
