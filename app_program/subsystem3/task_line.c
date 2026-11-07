/*
 *	task_line.c
 *	Subsystem 3 -- the Line Following task (every 20 ms).
 *
 *	Two sensors STRADDLE the tape (7 Nov): L left of it, R right of it,
 *	a few mm clear of each edge when centred.
 *	  L white, R white   centred (or a gap)      straight
 *	  L black, R white   the line moves left     turn left
 *	  L white, R black   the line moves right    turn right
 *	  L black, R black   a cross line            straight (a crossing)
 *	A correction is gentle (STEER_GENTLE) at full speed for its first
 *	S3_GENTLE_MM; if the sensor is still black (a curve) it goes hard
 *	(STEER_HARD) at S3_MID_PCT, and S3_LOW_PCT after S3_LOW_AFTER_MM.
 *	Tuned on 7 Nov between two failures: gentle for 25 mm at 150 mm/s let
 *	the tape slip out past L on tight left curves; hard at once with the
 *	speed at 50/25 % zigzagged (the sensors are 115 mm ahead of the
 *	wheels, so turning swings them sideways fast: the bar swept across
 *	the 20 mm tape and the 5 mm clearance before the loop could react,
 *	both sensors black on every swing, never centred).
 *	Overshoot: on a tight curve the tape can pass right under a sensor
 *	and out the other side, which also reads "both white".  After a long
 *	correction (S3_LONG_CORR_MM) followed by S3_OVERSHOOT_MM of white, it
 *	turns back toward the side that saw the line last (LINE_SEARCH);
 *	LINE_LOST after S3_LOST_MM or S3_LOST_MS of that.  Otherwise both
 *	white is straight on (gaps need nothing special); LINE_LOST after
 *	S3_LOST_STRAIGHT_MM of white.
 *	Stuck sensor: one black for S3_STUCK_ONE_MM, or both black for
 *	S3_STUCK_BOTH_MM, of driving: LINE_FAULT, Mission stops.
 *
 *	Junctions (see subsystem3.h):
 *	  Crossing (X, crossroads, T end): both black for S3_JUNC_MIN_MM ..
 *	  S3_CROSS_MAX_MM (longer than one tape width when crossed at an
 *	  angle), after driving straight (no correction) before it.  The tape
 *	  is narrower than the L-R gap, so the line itself only makes both
 *	  black when the robot sweeps across it at an angle, on a tight curve
 *	  while correcting: 2 false crossings on a curve on 7 Nov.
 *	  Left branch: B saw a patch about one tape wide, and the robot was
 *	  driving straight (no correction, no search) before that place.  B
 *	  is not level with the line sensors, so the corrections are kept by
 *	  distance (off_a/off_b) and looked up when B's patch arrives.  On
 *	  curves the robot corrects all the time, so a line seen by B there
 *	  does not count.  (After the arm L touches it and the robot turns a
 *	  little toward it, so after it is not checked.)
 *	  Two within S3_JUNC_MERGE_MM are one junction.
 *	Mission is told through flg_mission (MSN_FLG_JUNCTION /
 *	MSN_FLG_LINE_LOST); line_junction_odo() says where the centre was.
 */

#include "subsystem3.h"
#include "../subsystem2/subsystem2.h"	/* motion_odometer() */
#include "../subsystem5/subsystem5.h"	/* flg_mission */

/* Steering (mm/s each wheel).  Turn radius at 120 mm/s: gentle 30 cm;
   hard at 60 % 11 cm, at 35 % 6 cm (tight curves ~15 cm). */
#define STEER_GENTLE	25	/* first part of a correction, full speed */
#define STEER_HARD	40	/* a long correction (a curve) and the search */
#define STEER_STEP	10	/* most the steering may grow per 20 ms (15 with
				   the hard-at-once version zigzagged) */
#define S3_GENTLE_MM		15	/* gentle this long, then hard */
#define S3_MID_PCT		60	/* speed while correcting hard ... */
#define S3_LOW_PCT		35	/* ... and after S3_LOW_AFTER_MM of it */
#define S3_LOW_AFTER_MM		35
#define S3_SEARCH_PCT		20	/* searching (0 rocked in place, 2 Nov) */
#define S3_FILTER_N	2	/* a sensor pattern counts only after this many
				   readings in a row (2 = 40 ms, ~6 mm) */
#define S3_LONG_CORR_MM		15	/* a correction this long may have let the
					   tape slip out past the sensor (left
					   curves, 7 Nov: 21-27 mm, then lost) */
#define S3_OVERSHOOT_MM		40	/* white this long after one: overshot */
/* Crossing: no correction from both_from - S3_CROSS_BEFORE_MM to
   both_from - S3_CROSS_LEAD_MM (one sensor may reach an angled cross
   line first) */
#define S3_CROSS_BEFORE_MM	40
#define S3_CROSS_LEAD_MM	20
#define S3_LOST_STRAIGHT_MM	400	/* white this long (no search): lost.  Also
					   the longest gap it can cross */
#define S3_LOST_MM		250	/* searching this far ... */
#define S3_LOST_MS		4000	/* ... or this long (it turns on the spot) */
#define S3_CROSS_MAX_MM		45	/* both black, crossed at up to ~60 deg */
#define S3_STUCK_ONE_MM		300
#define S3_STUCK_BOTH_MM	150
#define MM(mm)		((W)(mm) * 1000 / S2_UM_PER_COUNT)	/* mm -> counts */

/* Left arm under B, centre at cc (where the line sensors crossed it):
   no correction from cc - S3_ARM_BEFORE_MM to cc - S3_ARM_SELF_MM (L
   touching the arm itself is allowed).  Decided once the line sensors
   are S3_ARM_AFTER_MM past it (at once when B is behind). */
#define S3_ARM_BEFORE_MM	40
#define S3_ARM_SELF_MM		22	/* half a tape + the ~6 mm filter delay + margin */
#define S3_ARM_AFTER_MM		20

#define PAT_L		2u
#define PAT_R		1u

LOCAL volatile INT	steer;
LOCAL volatile INT	state = LINE_ON;
LOCAL volatile UW	junctions;
LOCAL volatile BOOL	enabled = FALSE;	/* until a start (Mission or a test) */
LOCAL volatile INT	speed_pct = 100;
LOCAL volatile W	junc_odo;		/* last junction centre */
LOCAL volatile INT	junc_sides;
LOCAL volatile INT	fault_why;
LOCAL volatile UW	n_corr;			/* corrections since the start */
LOCAL volatile UW	n_sweep;		/* sweeps right across the tape */

EXPORT INT line_steer_mm_s(void)	{ return steer; }
EXPORT INT line_state(void)		{ return state; }
EXPORT INT line_fault(void)		{ return fault_why; }
EXPORT UW line_junctions(void)		{ return junctions; }
EXPORT void line_wobble(UW *corrections, UW *sweeps)
{
	*corrections = n_corr;
	*sweeps = n_sweep;
}
EXPORT void line_set_enabled(BOOL on)	{ enabled = on; }
EXPORT INT line_speed_pct(void)		{ return speed_pct; }
EXPORT W line_junction_odo(void)	{ return junc_odo; }
EXPORT INT line_junction_sides(void)	{ return junc_sides; }
EXPORT BOOL line_any_black(void)
{
	return app_gpio_get(PIN_IR_LINE_L) == IR_BLACK ||
	       app_gpio_get(PIN_IR_LINE_R) == IR_BLACK;
}

/* A junction centred at c (odometer counts).  One close to the last one
   is another view of the same junction. */
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

/* Where the robot was correcting or searching: the last OFFS stretches,
   by odometer.  The newest is still open while it is. */
#define OFFS	16
LOCAL W		off_a[OFFS], off_b[OFFS];
LOCAL UW	off_n;
LOCAL BOOL	off_open;
LOCAL W		hist_from;		/* nothing known before this */

LOCAL void hist_reset(W odo)
{
	off_n = 0;
	off_open = FALSE;
	hist_from = odo;
}

LOCAL void hist_step(W odo, BOOL busy)
{
	if (busy && !off_open) {
		off_a[off_n % OFFS] = odo;
		off_n++;
		off_open = TRUE;
	} else if (!busy && off_open) {
		off_b[(off_n - 1) % OFFS] = odo;
		off_open = FALSE;
	}
}

/* TRUE if the robot drove straight (no correction) all of [a, b] */
LOCAL BOOL quiet_over(W a, W b, W now)
{
	UW i;

	if (b < hist_from) return TRUE;		/* before the start: the robot
						   was put down straight */
	if (a < hist_from) a = hist_from;
	if (off_n > OFFS && a < off_a[(off_n - OFFS) % OFFS]) {
		return FALSE;			/* not on record */
	}
	for (i = (off_n > OFFS) ? off_n - OFFS : 0; i < off_n; i++) {
		W x = off_a[i % OFFS];
		W y = (off_open && i == off_n - 1) ? now : off_b[i % OFFS];

		if (x <= b && y >= a) return FALSE;
	}
	return TRUE;
}

EXPORT ER line_hw_init(void)
{
	ER er = app_gpio_input(PIN_IR_LINE_L, FALSE);	/* modules drive D0 */

	return (er < E_OK) ? er : app_gpio_input(PIN_IR_LINE_R, FALSE);
}

EXPORT void line_task(INT stacd, void *exinf)
{
	UW	stable = 0u, cand = 0u;	/* filtered pattern: PAT_L | PAT_R */
	INT	cand_n = 0, out = 0;
	BOOL	was_enabled = FALSE;
	INT	last_side = 0;		/* +1 line last seen on the left, -1 right */
	W	single_from = 0;	/* start of the current one-sensor stretch */
	BOOL	long_corr = FALSE;	/* the last correction was long */
	W	white_from = 0;		/* start of the current all-white stretch */
	BOOL	searching = FALSE;
	W	search_from = 0;
	INT	search_n = 0;
	W	both_from = 0;
	UW	before_both = 0u;	/* the one-sensor pattern before a both-black */
	BOOL	bpend = FALSE;		/* B patch waiting until the sensors pass it */
	W	bpend_c = 0;
	W	mc;
	INT	mw;
	UW	mark_seen = barcode_mark(&mc, &mw);

	(void)stacd; (void)exinf;
	hist_reset(motion_odometer());

	while (1) {
		BOOL	l, r, fl, fr, single;
		W	odo;
		INT	s, target;
		UW	pat, prev;

		tk_slp_tsk(TMO_FEVR);			/* woken every 20 ms */
		app_timing_wake(tmg_line, tk_can_wup(TSK_SELF));

		l = (app_gpio_get(PIN_IR_LINE_L) == IR_BLACK);
		r = (app_gpio_get(PIN_IR_LINE_R) == IR_BLACK);
		odo = motion_odometer();
		pat = (l ? PAT_L : 0u) | (r ? PAT_R : 0u);
		if (enabled && !was_enabled) {
			/* a start, or after a manoeuvre: begin clean */
			stable = cand = pat;
			cand_n = S3_FILTER_N;
			out = 0;
			last_side = 0;
			single_from = white_from = both_from = odo;
			long_corr = searching = bpend = FALSE;
			before_both = 0u;
			n_corr = n_sweep = 0u;
			hist_reset(odo);
			mark_seen = barcode_mark(&mc, &mw);	/* drop old patches */
			state = LINE_ON;
			fault_why = 0;
		}
		was_enabled = enabled;

		/* Filter: ignore a pattern that does not last S3_FILTER_N readings */
		prev = stable;
		if (pat == cand) {
			if (cand_n < S3_FILTER_N) cand_n++;
		} else {
			cand = pat;
			cand_n = 1;
		}
		if (cand_n >= S3_FILTER_N) stable = cand;
		fl = (stable & PAT_L) != 0u;
		fr = (stable & PAT_R) != 0u;
		single = (fl != fr);

		/* Stretches start where the filtered pattern changed */
		if (stable != prev) {
			if ((prev == PAT_L || prev == PAT_R) &&
			    odo - single_from >= MM(S3_LONG_CORR_MM)) {
				long_corr = TRUE;	/* a long correction ended */
			} else if (prev == PAT_L || prev == PAT_R) {
				long_corr = FALSE;
			}
			if (single) {
				single_from = odo;
				if (prev == 0u) n_corr++;
				/* L, both, R (or R, both, L): swept right across */
				if (prev == (PAT_L | PAT_R) && before_both != 0u &&
				    before_both != stable) {
					n_sweep++;
				}
			}
			if (stable == 0u) white_from = odo;
			if (stable == (PAT_L | PAT_R)) {
				both_from = odo;
				before_both = (prev == PAT_L || prev == PAT_R) ? prev : 0u;
			}
		}
		if (stable != 0u && searching) {
			searching = FALSE;		/* found it again */
			long_corr = FALSE;
		}
		if (single) last_side = fl ? +1 : -1;

		/* Overshoot: white for a while right after a long correction */
		if (!searching && stable == 0u && long_corr &&
		    odo - white_from >= MM(S3_OVERSHOOT_MM)) {
			searching = TRUE;
			search_from = odo;
			search_n = 0;
		}

		/* Steering (+ = turn left) */
		if (single) {
			s = (odo - single_from < MM(S3_GENTLE_MM)) ? STEER_GENTLE
								   : STEER_HARD;
			if (fr) s = -s;
		} else if (searching) {
			s = (last_side >= 0) ? STEER_HARD : -STEER_HARD;
		} else {
			s = 0;				/* centred, gap, or crossing */
		}
		/* Build a turn up gently (STEER_STEP per period), but ease off
		   at once: ramping down too kept the robot turning after it
		   had found the line again, and it swung across (2 Nov). */
		target = s;
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
		/* Motion slows down while correcting or searching, so the same
		   steering turns much tighter */
		if (searching) {
			speed_pct = S3_SEARCH_PCT;
		} else if (single && odo - single_from >= MM(S3_LOW_AFTER_MM)) {
			speed_pct = S3_LOW_PCT;
		} else if (single && odo - single_from >= MM(S3_GENTLE_MM)) {
			speed_pct = S3_MID_PCT;
		} else {
			speed_pct = 100;
		}
		hist_step(odo, single || searching);

		if (!enabled) {			/* turning, bypassing: no events */
			bpend = FALSE;
			app_timing_done(tmg_line);
			continue;
		}

		/* ---- Lost, and stuck sensors ---- */
		if (state != LINE_FAULT && state != LINE_LOST) {
			INT why = 0;

			if (searching) {
				state = LINE_SEARCH;
				if (odo - search_from > MM(S3_LOST_MM) ||
				    ++search_n > S3_LOST_MS / PERIOD_LINE_MS) {
					state = LINE_LOST;
				}
			} else if (stable == 0u &&
				   odo - white_from > MM(S3_LOST_STRAIGHT_MM)) {
				state = LINE_LOST;
			} else {
				state = LINE_ON;
			}
			if (single && odo - single_from > MM(S3_STUCK_ONE_MM)) {
				why = LINE_FAULT_ONE;
			} else if (stable == (PAT_L | PAT_R) &&
				   odo - both_from > MM(S3_STUCK_BOTH_MM)) {
				why = LINE_FAULT_BOTH;
			}
			if (why != 0) {
				fault_why = why;
				state = LINE_FAULT;
			}
			if (state == LINE_LOST || state == LINE_FAULT) {
				tk_set_flg(flg_mission, MSN_FLG_LINE_LOST);
			}
		}

		/* ---- Crossing: the end of a "both black" ---- */
		if (prev == (PAT_L | PAT_R) && stable != prev) {
			W len = odo - both_from;

			if (len >= MM(S3_JUNC_MIN_MM) && len <= MM(S3_CROSS_MAX_MM) &&
			    quiet_over(both_from - MM(S3_CROSS_BEFORE_MM),
				       both_from - MM(S3_CROSS_LEAD_MM), odo)) {
				junction_at(both_from + len / 2, JUNC_SIDE_S);
			}
			/* else swept across the line on a curve: not a crossing */
		}

		/* ---- Left arm, from B: a patch about one tape wide.  cc is
		   where the line sensors crossed the same place (B is
		   S3_BAR_AHEAD_MM in front of them; negative = behind) ---- */
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
			if (quiet_over(cc - MM(S3_ARM_BEFORE_MM), cc - MM(S3_ARM_SELF_MM), odo)) {
				junction_at(cc, JUNC_SIDE_B);
			}
			/* else on a curve or correcting there: not trusted */
		}

		app_timing_done(tmg_line);
	}
}
