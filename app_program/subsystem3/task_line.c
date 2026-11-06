/*
 *	task_line.c
 *	Subsystem 3 -- the Line Following task (every 20 ms).
 *
 *	The side sensor (GP0) sits just beside the line, IR_CENTER over it.
 *	Written for the side sensor on the LEFT; S3_SIDE = -1 (side sensor on
 *	the right, our robot) mirrors every steering value:
 *	  centre black, left white   on the line           steer 0
 *	  centre black, left black   drifting right, or a junction: steer left
 *	  centre white, left black   far right of the line  steer left hard
 *	  centre white, left white   left of the line (or lost): steer right,
 *	                             LINE_LOST after S3_LOST_MM
 *	This needs S just beside the tape edge (S3_S_GAP_MM in subsystem3.h).
 *
 *	Junctions (see subsystem3.h), the line goes on after them:
 *	  S side branch: coming from "C only", both black for about one tape
 *	  width (S3_JUNC_MIN_MM..S3_JUNC_MAX_MM) with the steering HELD, then
 *	  S white again with C still black.  Drifting toward the line or a
 *	  curve toward S keeps S black longer (nothing turns it white while
 *	  the steering is held), so neither counts.
 *	  Left branch: B saw a patch about one tape wide, and at that place
 *	  C was on the line before and after it.  B is not level with C, so
 *	  where C was on or off the line is kept by distance (off_a/off_b)
 *	  and looked up when B's patch arrives.
 *	  Two within S3_JUNC_MERGE_MM are one crossroads.
 *	Line ends at a cross bar (T end): S saw the right arm and C went
 *	white, AND B saw the left arm at the same place with C off the line
 *	after it.  Only one arm = a corner: not a junction, the line
 *	following goes round it.  With B behind C the robot drives straight
 *	on (steering held) until B has passed the arm.
 *	Mission is told through flg_mission (MSN_FLG_JUNCTION /
 *	MSN_FLG_LINE_LOST); line_junction_odo() says where the centre was.
 */

#include "subsystem3.h"
#include "../subsystem2/subsystem2.h"	/* motion_odometer() */
#include "../subsystem5/subsystem5.h"	/* flg_mission */

/* Steering, tuned on the Stage 2 run (1 Nov): 40 / 110 swung the sensors
   right across the line and off the other side. */
#define STEER_SMALL	25	/* mm/s */
#define STEER_HARD	40	/* also the on-the-spot turn speed (70 overshot, 2 Nov) */
#define STEER_STEP	15	/* most the steering may change per 20 ms */
#define S3_FILTER_N	2	/* a sensor pattern steers only after this many
				   readings in a row (2 = 40 ms, ~6 mm) */
#define S3_LOST_MM	250
#define S3_LOST_MS	4000	/* searching on the spot this long = lost */
#define MM(mm)		((W)(mm) * 1000 / S2_UM_PER_COUNT)	/* mm -> counts */

/* Left arm under B, centre at cc (where C crossed it), checked on C:
     on  from cc - S3_ARM_BEFORE_MM to cc + S3_ARM_AFTER_MM: a branch
     on  before, off from cc + S3_ARM_OFF_MM to cc + S3_ARM_AFTER_MM:
         the line ended at the arm (T end, or a left corner)
   Decided once C is S3_ARM_AFTER_MM past the arm (at once if B is behind). */
#define S3_ARM_BEFORE_MM	20
#define S3_ARM_OFF_MM		(S3_TAPE_MM)		/* C leaves at half a tape
							   + the filter delay */
#define S3_ARM_AFTER_MM		(S3_TAPE_MM + 10)
/* T end: the right arm waits this long for B (behind: until B is past) */
#define S3_TEND_KEEP_MM		(((S3_BAR_AHEAD_MM < 0) ? -S3_BAR_AHEAD_MM : 0) + 40)

LOCAL volatile INT	steer;
LOCAL volatile INT	state = LINE_ON;
LOCAL volatile UW	junctions;
LOCAL volatile BOOL	enabled = TRUE;
LOCAL volatile BOOL	centre_on = TRUE;	/* filtered centre sensor */
LOCAL volatile W	junc_odo;		/* last junction centre (C) */
LOCAL volatile INT	junc_sides;

EXPORT INT line_steer_mm_s(void)	{ return steer; }
EXPORT INT line_state(void)		{ return state; }
EXPORT UW line_junctions(void)		{ return junctions; }
EXPORT BOOL line_center_black(void)	{ return app_gpio_get(PIN_IR_CENTER) == IR_BLACK; }
EXPORT void line_set_enabled(BOOL on)	{ enabled = on; }
EXPORT BOOL line_centre_on(void)	{ return centre_on; }
EXPORT W line_junction_odo(void)	{ return junc_odo; }
EXPORT INT line_junction_sides(void)	{ return junc_sides; }

/* A junction centred at c (odometer counts, where C crossed it).  One
   close to the last one is the other arm of the same crossroads. */
LOCAL W		last_c;
LOCAL BOOL	have_last;

LOCAL void junction_at(W c, INT side)
{
	W d = c - last_c;

	if (d < 0) d = -d;
	if (have_last && d < MM(S3_JUNC_MERGE_MM)) {
		junc_sides |= side;		/* same one: no second event */
		return;
	}
	last_c = c;
	have_last = TRUE;
	junc_odo = c;
	junc_sides = side;
	junctions++;
	tk_set_flg(flg_mission, MSN_FLG_JUNCTION);
}

/* Where C (filtered) was OFF the line: the last OFFS stretches, by
   odometer.  The newest is still open while C is off. */
#define OFFS	8
LOCAL W		off_a[OFFS], off_b[OFFS];
LOCAL UW	off_n;			/* stretches recorded since hist_from */
LOCAL BOOL	off_open;
LOCAL W		hist_from;		/* nothing known before this */

LOCAL void hist_reset(W odo, BOOL fc)
{
	off_n = 0;
	off_open = FALSE;
	hist_from = odo;
	if (!fc) {
		off_a[0] = odo;
		off_n = 1;
		off_open = TRUE;
	}
}

LOCAL void hist_step(W odo, BOOL fc)
{
	if (!fc && !off_open) {
		off_a[off_n % OFFS] = odo;
		off_n++;
		off_open = TRUE;
	} else if (fc && off_open) {
		off_b[(off_n - 1) % OFFS] = odo;
		off_open = FALSE;
	}
}

/* TRUE if C was on the line all of [a, b] (and it is all on record) */
LOCAL BOOL c_on_over(W a, W b, W now)
{
	UW i;

	if (a < hist_from || (off_n > OFFS && a < off_a[(off_n - OFFS) % OFFS])) {
		return FALSE;			/* too long ago */
	}
	for (i = (off_n > OFFS) ? off_n - OFFS : 0; i < off_n; i++) {
		W x = off_a[i % OFFS];
		W y = (off_open && i == off_n - 1) ? now : off_b[i % OFFS];

		if (x <= b && y >= a) return FALSE;
	}
	return TRUE;
}

/* TRUE if C was off the line all of [a, b] */
LOCAL BOOL c_off_over(W a, W b, W now)
{
	UW i;

	for (i = (off_n > OFFS) ? off_n - OFFS : 0; i < off_n; i++) {
		W x = off_a[i % OFFS];
		W y = (off_open && i == off_n - 1) ? now : off_b[i % OFFS];

		if (x <= a && y >= b) return TRUE;
	}
	return FALSE;
}

EXPORT ER line_hw_init(void)
{
	ER er = app_gpio_input(PIN_IR_LEFT, FALSE);	/* modules drive D0 */

	return (er < E_OK) ? er : app_gpio_input(PIN_IR_CENTER, FALSE);
}

EXPORT void line_task(INT stacd, void *exinf)
{
	W	lost_from = 0, both_from = 0;
	INT	search_n = 0;		/* periods spent searching */
	BOOL	both = FALSE, was_enabled = TRUE, both_clean = FALSE;
	INT	last_side = 0;		/* +1 line last seen to the left, -1 right */
	UW	stable = 1u, cand = 1u;	/* filtered sensor pattern: bit 1 side, bit 0 centre */
	INT	cand_n = 0, out = 0;
	BOOL	bpend = FALSE;		/* B patch waiting until C is past it */
	W	bpend_c = 0;
	BOOL	tend_s = FALSE, tend_b = FALSE;	/* T end: arm seen by S / by B */
	W	tend_s_c = 0, tend_b_c = 0;
	W	mc;
	INT	mw;
	UW	mark_seen = barcode_mark(&mc, &mw);

	(void)stacd; (void)exinf;
	hist_reset(motion_odometer(), TRUE);

	while (1) {
		BOOL	l, c, fl, fc, hold_t;
		W	odo;
		INT	s, target;
		UW	pat, prev_stable;

		tk_slp_tsk(TMO_FEVR);			/* woken every 20 ms */
		app_timing_wake(tmg_line, tk_can_wup(TSK_SELF));

		l = (app_gpio_get(PIN_IR_LEFT) == IR_BLACK);
		c = (app_gpio_get(PIN_IR_CENTER) == IR_BLACK);
		odo = motion_odometer();
		pat = ((UW)l << 1) | (UW)c;
		if (enabled && !was_enabled) {
			both = FALSE;			/* start clean after a manoeuvre */
			lost_from = odo;
			stable = cand = pat;
			cand_n = S3_FILTER_N;
			out = 0;
			tend_s = tend_b = bpend = FALSE;
			hist_reset(odo, (pat & 1u) != 0u);
			mark_seen = barcode_mark(&mc, &mw);	/* drop old patches */
		}
		was_enabled = enabled;

		prev_stable = stable;
		/* Filter: ignore a pattern that does not last S3_FILTER_N readings */
		if (pat == cand) {
			if (cand_n < S3_FILTER_N) cand_n++;
		} else {
			cand = pat;
			cand_n = 1;
		}
		if (cand_n >= S3_FILTER_N) stable = cand;
		fl = (stable & 2u) != 0u;
		fc = (stable & 1u) != 0u;
		hist_step(odo, fc);

		/* S black too: a branch, or the robot drifting away from S (S
		   is just beside the tape), or a curve toward S.  Coming
		   straight from "C only", hold the steering for up to one tape
		   width +40%: S crosses a branch and goes white again by
		   itself; drift or a curve keeps it black, and then the
		   steering corrects as usual.  Without the hold the steering
		   itself turned S white after a short burst, which looked just
		   like a branch.  28 mm at a 5 deg drift is 2.5 mm sideways. */
		if (enabled && fc && fl && !both) {
			both = TRUE;
			both_from = odo;
			both_clean = (prev_stable == 1u);	/* from "C only" */
		}
		/* T end: drive straight on (no creeping and turning toward
		   the right arm) until B's view of the left arm is in: B
		   behind C, until B has passed it; B ahead, while its patch
		   waits for C to get past */
		hold_t = enabled && tend_s && (S3_BAR_AHEAD_MM < 0 || bpend);

		/* Steering, from the filtered pattern */
		if (fc && !fl) {
			s = 0;
			last_side = 0;
		} else if (fc && fl) {
			s = (both && both_clean &&
			     odo - both_from < MM(S3_JUNC_MAX_MM)) ? 0 : STEER_SMALL;
			last_side = +1;
		} else if (!fc && fl) {
			s = STEER_HARD;
			last_side = +1;
		} else {
			/* both white: the line is to the right, unless we last
			   saw it to the left */
			s = (last_side > 0) ? STEER_HARD : -STEER_HARD;
		}
		if (hold_t) s = 0;
		/* Build a turn up gently (STEER_STEP per period), but ease off
		   at once: ramping down too kept the robot turning after the
		   centre had found the line again, and it swung across
		   (Stage 2 run, 2 Nov). */
		target = s * S3_SIDE;
		if ((out > 0 && target < 0) || (out < 0 && target > 0)) {
			out = 0;			/* other way: drop the old turn */
		}
		if ((out >= 0 && target >= 0 && target <= out) ||
		    (out <= 0 && target <= 0 && target >= out)) {
			out = target;			/* easing off */
		} else if (out < target - STEER_STEP) {
			out += STEER_STEP;
		} else if (out > target + STEER_STEP) {
			out -= STEER_STEP;
		} else {
			out = target;
		}
		steer = out;
		/* Motion slows to a creep while C is off the line; not while
		   driving straight on at a T end */
		centre_on = fc || hold_t;

		/* Lost / found */
		if (c || l) {
			lost_from = odo;
			search_n = 0;
			if (state != LINE_ON) {
				state = LINE_ON;
			}
		} else if (state == LINE_ON) {
			state = LINE_SEARCH;
		} else if (state == LINE_SEARCH && enabled &&
			   (S2_COUNTS_TO_MM(odo - lost_from) > S3_LOST_MM ||
			    ++search_n > S3_LOST_MS / PERIOD_LINE_MS)) {
			/* distance, or time: Motion turns on the spot while
			   searching, which adds no distance */
			state = LINE_LOST;
			tk_set_flg(flg_mission, MSN_FLG_LINE_LOST);
		}

		if (!enabled) {			/* turning, bypassing: no junctions */
			tend_s = tend_b = bpend = FALSE;
			both = FALSE;
			app_timing_done(tmg_line);
			continue;
		}

		/* ---- S side: the end of a held "both black" (started above),
		   from the filtered pattern like the steering (one flicker in
		   the middle of a crossing split it in two) ---- */
		if (both && !(fc && fl)) {
			INT len = (INT)S2_COUNTS_TO_MM(odo - both_from);
			W   jc = both_from + (odo - both_from) / 2;

			both = FALSE;
			if (both_clean && len >= S3_JUNC_MIN_MM && len <= S3_JUNC_MAX_MM) {
				if (fc && !fl) {	/* line goes on: a branch */
					junction_at(jc, JUNC_SIDE_S);
				} else if (!fc) {	/* line ends: T end or corner */
					tend_s = TRUE;
					tend_s_c = jc;
				}
			}
		}

		/* ---- Left, from B: a patch about one tape wide.  cc is where
		   C crossed the same place (B is S3_BAR_AHEAD_MM in front of
		   C; negative = behind) ---- */
		{
			UW q = barcode_mark(&mc, &mw);

			if (q != mark_seen) {
				mark_seen = q;
				if (mw >= S3_JUNC_MIN_MM * 10 && mw <= S3_JUNC_MAX_MM * 10) {
					bpend = TRUE;
					bpend_c = mc + MM(S3_BAR_AHEAD_MM);
				}
			}
		}
		if (bpend && odo - bpend_c >= MM(S3_ARM_AFTER_MM)) {
			W cc = bpend_c;

			bpend = FALSE;
			if (c_on_over(cc - MM(S3_ARM_BEFORE_MM), cc + MM(S3_ARM_AFTER_MM), odo)) {
				/* the line goes on past the arm: a left branch */
				junction_at(cc, JUNC_SIDE_B);
			} else if (c_on_over(cc - MM(S3_ARM_BEFORE_MM), cc, odo) &&
				   c_off_over(cc + MM(S3_ARM_OFF_MM), cc + MM(S3_ARM_AFTER_MM), odo)) {
				/* the line ended at the arm: a T end, or a left
				   corner (a T end only if S saw the other arm) */
				tend_b = TRUE;
				tend_b_c = cc;
			}
			/* else C was wandering there: not trusted, ignored */
		}

		/* ---- T end: both arms at the same place.  One alone is a
		   corner, followed like any other bend ---- */
		if (tend_s && tend_b) {
			W d = tend_s_c - tend_b_c;

			if (d < 0) d = -d;
			if (d < MM(S3_JUNC_MERGE_MM)) {
				junction_at((tend_s_c + tend_b_c) / 2, JUNC_SIDE_S | JUNC_SIDE_B);
				tend_s = tend_b = FALSE;
			}
		}
		if (tend_s && odo - tend_s_c > MM(S3_TEND_KEEP_MM)) tend_s = FALSE;
		if (tend_b && odo - tend_b_c > MM(S3_TEND_KEEP_MM)) tend_b = FALSE;

		app_timing_done(tmg_line);
	}
}
