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
 *	Steering (tuned 7 Nov in a simulation calibrated on that day's three
 *	robot runs: it reproduced the lost left curves at 150 mm/s, the zigzag
 *	of the hard-at-once version and the good third run; scenarios: 100-130
 *	mm curves, S-bends, straights, sensor windows 6-20 mm, motor lag and
 *	delay, +-2-4 % wheel mismatch.  Old settings 297/480, these 478/480
 *	with no sweeps, same speed):
 *	  - a correction is STEER_CORR at once, Motion at S3_CORR_PCT, then
 *	    S3_LOW_PCT after S3_LOW_AFTER_MM of it;
 *	  - steering memory: the steering averaged over the last S3_MEM_MM;
 *	    while both sensors are white the robot keeps S3_MEM_K of it.  On a
 *	    straight the corrections alternate L, R and it stays near 0; on a
 *	    curve it settles at the turn the curve needs, so the robot keeps
 *	    turning between corrections instead of driving straight off (the
 *	    S-bend of track 3, 7 Nov).  It also slows a little in curves.
 *	Overshoot: on a tight curve the tape can pass right under a sensor
 *	and out the other side, which also reads "both white".  After a
 *	correction (S3_LONG_CORR_MM) followed by S3_OVERSHOOT_MM of white it
 *	turns STEER_SEARCH toward the side that saw the line last (LINE_SEARCH)
 *	at S3_SEARCH_PCT; LINE_LOST after S3_LOST_MM or S3_LOST_MS of that.
 *	Both white is also "no line at all": on a long straight, well
 *	centred, the robot touched nothing for 400 mm and stopped with
 *	LINE_LOST on a good line (Stage 2B, 7 Nov).  So after
 *	S3_PROBE_AFTER_MM of white it steers very gently (STEER_PROBE) toward
 *	the side it saw last, for S3_PROBE_MM, then the other way for twice
 *	that: a sensor touching the tape ends the probe.  Nothing either way:
 *	LINE_LOST (a line end is driven past by about 400 mm).
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
 *	  Crossing with a left arm, two sensors agreeing (8 Nov): both black
 *	  for S3_PAIR_BOTH_MIN_MM .. S3_CROSS_MAX_MM, and B (S3_BAR_AHEAD_MM
 *	  behind) then sees a patch about a tape wide within S3_PAIR_MM of
 *	  the same place.  Counts even while the robot corrects: with the
 *	  robot zigzagging (24 corrections a metre, 8 Nov) the "drove
 *	  straight before it" rule threw away every junction, and the robot
 *	  read a barcode and drove on through its junction.  In the saved
 *	  logs of 7-8 Nov this pairing finds 11 places, 3 of them detected at
 *	  the time; the zigzag's own both-black stretches are 2-6 mm and B
 *	  sees nothing there.
 *	  Two within S3_JUNC_MERGE_MM are one junction.
 *	Mission is told through flg_mission (MSN_FLG_JUNCTION /
 *	MSN_FLG_LINE_LOST); line_junction_odo() says where the centre was.
 */

#include "subsystem3.h"
#include "../subsystem2/subsystem2.h"	/* motion_odometer() */
#include "../subsystem5/subsystem5.h"	/* flg_mission */

/* Steering (mm/s each wheel) and Motion's speed while correcting */
#define STEER_CORR	30	/* every correction, at once */
#define STEER_SEARCH	30	/* the overshoot search */
#define STEER_STEP	10	/* most the steering may grow per 20 ms */
#define S3_CORR_PCT		60	/* speed while correcting ... */
#define S3_LOW_PCT		40	/* ... and after S3_LOW_AFTER_MM of it */
#define S3_LOW_AFTER_MM		50
#define S3_SEARCH_PCT		20	/* searching (0 rocked in place, 2 Nov) */
/* Steering memory */
#define S3_MEM_MM		60	/* averaged over about this distance */
#define S3_MEM_K		0.4f	/* share kept while both sensors are white */
#define S3_MEM_MAX		40
#define S3_MEM_SLOW		0.5f	/* speed % less per mm/s of memory (curves) */
#define S3_FILTER_N	1	/* a sensor pattern counts only after this many
				   readings in a row (2 made every correction
				   20-40 ms late) */
#define S3_LONG_CORR_MM		5	/* a correction this long may have let the
					   tape slip out past the sensor ... */
#define S3_OVERSHOOT_MM		60	/* ... white this long after it: overshot
					   (the memory keeps turning meanwhile) */
/* Crossing: no correction from both_from - S3_CROSS_BEFORE_MM to
   both_from - S3_CROSS_LEAD_MM (one sensor may reach an angled cross
   line first) */
#define S3_CROSS_BEFORE_MM	40
#define S3_CROSS_LEAD_MM	20
#define S3_PROBE_AFTER_MM	200	/* white this long: look for the tape */
#define S3_PROBE_MM		70	/* one way this far, the other way twice
					   that; nothing = lost.  Longest gap it
					   can cross: about 200 + 3 x 70 mm */
#define STEER_PROBE		12	/* mm/s: the sensors move ~5 mm sideways
					   (the clearance to the tape) in ~30 mm */
#define S3_LOST_MM		250	/* searching this far ... */
#define S3_LOST_MS		4000	/* ... or this long (it turns on the spot) */
#define S3_CROSS_MAX_MM		45	/* both black, crossed at up to ~60 deg */
#define S3_PAIR_BOTH_MIN_MM	10	/* both black this long ... */
#define S3_PAIR_B_MIN_MM10	90	/* ... and B a patch from 9.0 mm (B's
					   tape patches read 11.5-15.9 mm) ... */
#define S3_PAIR_MM		25	/* ... within this of each other (logs:
					   1-16 mm) = a crossing */
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
	return (IR_BLACK == app_gpio_get(PIN_IR_LINE_L)) ||
	       (IR_BLACK == app_gpio_get(PIN_IR_LINE_R));
}

/* A junction centred at c (odometer counts).  One close to the last one
   is another view of the same junction. */
LOCAL W		last_c;
LOCAL BOOL	have_last;

LOCAL void junction_at(W c, INT side)
{
	W d = c - last_c;

	if (d < 0) d = -d;
	if (have_last && (d < MM(S3_JUNC_MERGE_MM))) {
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
LOCAL W		off_a[OFFS];
LOCAL W off_b[OFFS];
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
	if (busy && (!off_open)) {
		off_a[off_n % OFFS] = odo;
		off_n++;
		off_open = TRUE;
	} else if ((!busy) && off_open) {
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
	if ((off_n > OFFS) && (a < off_a[(off_n - OFFS) % OFFS])) {
		return FALSE;			/* not on record */
	}
	for (i = (off_n > OFFS) ? off_n - OFFS : 0; i < off_n; i++) {
		W x = off_a[i % OFFS];
		W y = (off_open && (i == (off_n - 1))) ? now : off_b[i % OFFS];

		if ((x <= b) && (y >= a)) return FALSE;
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
	UW	stable = 0u;
	UW cand = 0u;	/* filtered pattern: PAT_L | PAT_R */
	INT	cand_n = 0;
	INT out = 0;
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
	INT	probe = 0;		/* probing for the tape: +1 left, -1 right */
	INT	probe_leg = 0;		/* 1 first way, 2 back the other way */
	W	probe_from = 0;
	BOOL	probe_none = FALSE;	/* probed both ways: no tape */
	float	mem = 0.0f;		/* steering memory, mm/s */
	W	mem_odo = 0;
	W	bpend_c = 0;
	W	pair_c = 0;		/* centre of the last both-black, for B */
	BOOL	pair_ok = FALSE;
	W	mc;
	INT	mw;
	UW	mark_seen = barcode_mark(&mc, &mw);

	(void)stacd; (void)exinf;
	hist_reset(motion_odometer());

	for (;;) {
		BOOL	l;
		BOOL r;
		BOOL fl;
		BOOL fr;
		BOOL single;
		W	odo;
		INT	s;
		INT target;
		UW	pat;
		UW prev;

		tk_slp_tsk(TMO_FEVR);			/* woken every 20 ms */
		app_timing_wake(tmg_line, tk_can_wup(TSK_SELF));

		l = (IR_BLACK == app_gpio_get(PIN_IR_LINE_L));
		r = (IR_BLACK == app_gpio_get(PIN_IR_LINE_R));
		odo = motion_odometer();
		pat = (l ? PAT_L : 0u) | (r ? PAT_R : 0u);
		if (enabled && (!was_enabled)) {
			/* a start, or after a manoeuvre: begin clean */
			stable = cand = pat;
			cand_n = S3_FILTER_N;
			out = 0;
			last_side = 0;
			single_from = white_from = both_from = odo;
			long_corr = searching = bpend = pair_ok = FALSE;
			probe = probe_leg = 0;
			probe_none = FALSE;
			mem = 0.0f;
			mem_odo = odo;
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
			if (((PAT_L == prev) || (PAT_R == prev)) &&
			    ((odo - single_from) >= MM(S3_LONG_CORR_MM))) {
				long_corr = TRUE;	/* a long correction ended */
			} else if ((PAT_L == prev) || (PAT_R == prev)) {
				long_corr = FALSE;
			}
			if (single) {
				single_from = odo;
				if (0u == prev) n_corr++;
				/* L, both, R (or R, both, L): swept right across */
				if ((prev == (PAT_L | PAT_R)) && (before_both != 0u) &&
				    (before_both != stable)) {
					n_sweep++;
				}
			}
			if (0u == stable) white_from = odo;
			if (stable == (PAT_L | PAT_R)) {
				both_from = odo;
				before_both = ((PAT_L == prev) || (PAT_R == prev)) ? prev : 0u;
			}
		}
		if ((stable != 0u) && searching) {
			searching = FALSE;		/* found it again */
			long_corr = FALSE;
		}
		if (single) last_side = fl ? +1 : -1;

		/* Overshoot: white for a while right after a long correction */
		if ((!searching) && (0u == stable) && long_corr &&
		    ((odo - white_from) >= MM(S3_OVERSHOOT_MM))) {
			searching = TRUE;
			search_from = odo;
			search_n = 0;
		}

		/* Long straight, all white: is the tape still there? */
		if ((stable != 0u) || searching) {
			probe = probe_leg = 0;
			probe_none = FALSE;
		} else if ((0 == probe) && (!probe_none) &&
			   ((odo - white_from) >= MM(S3_PROBE_AFTER_MM))) {
			probe = (last_side >= 0) ? +1 : -1;
			probe_leg = 1;
			probe_from = odo;
		} else if ((probe != 0) &&
			   ((odo - probe_from) >= MM(S3_PROBE_MM * probe_leg))) {
			if (1 == probe_leg) {
				probe = -probe;		/* the other way, twice as far */
				probe_leg = 2;
				probe_from = odo;
			} else {
				probe = probe_leg = 0;
				probe_none = TRUE;	/* no tape either way */
			}
		}

		/* Steering (+ = turn left) */
		if (single) {
			s = fr ? -STEER_CORR : STEER_CORR;
		} else if (searching) {
			s = (last_side >= 0) ? STEER_SEARCH : -STEER_SEARCH;
		} else if (probe != 0) {
			s = probe * STEER_PROBE;	/* looking for the tape */
		} else {
			/* centred, gap or crossing: keep part of the recent
			   steering (a curve keeps turning) */
			float m = S3_MEM_K * mem;

			if (m > (float)S3_MEM_MAX) m = (float)S3_MEM_MAX;
			if (m < -(float)S3_MEM_MAX) m = -(float)S3_MEM_MAX;
			s = (INT)m;
		}
		/* Build a turn up gently (STEER_STEP per period), but ease off
		   at once: ramping down too kept the robot turning after it
		   had found the line again, and it swung across (2 Nov). */
		target = s;
		if (((out > 0) && (target < 0)) || ((out < 0) && (target > 0))) {
			out = 0;			/* other way: drop the old turn */
		}
		if (((out >= 0) && (target >= 0) && (target <= out)) ||
		    ((out <= 0) && (target <= 0) && (target >= out))) {
			out = target;			/* easing off */
		} else if (out < (target - STEER_STEP)) {
			out += STEER_STEP;
		} else if (out > (target + STEER_STEP)) {
			out -= STEER_STEP;
		} else {
			out = target;
		}
		steer = out;
		/* steering memory: average of the steering by distance */
		{
			W ds = odo - mem_odo;

			if (ds > 0) {
				float a = (float)S2_COUNTS_TO_MM(ds) / (float)S3_MEM_MM;

				if (a > 1.0f) a = 1.0f;
				mem += ((float)out - mem) * a;
			}
			mem_odo = odo;
		}
		/* Motion slows down while correcting or searching, so the same
		   steering turns much tighter; a little in curves too */
		if (searching) {
			speed_pct = S3_SEARCH_PCT;
		} else {
			INT cap;

			if (single && ((odo - single_from) >= MM(S3_LOW_AFTER_MM))) {
				speed_pct = S3_LOW_PCT;
			} else if (single) {
				speed_pct = S3_CORR_PCT;
			} else {
				speed_pct = 100;
			}
			cap = 100 - (INT)(S3_MEM_SLOW * ((mem < 0.0f) ? -mem : mem));
			if (cap < 35) cap = 35;
			if (cap < speed_pct) speed_pct = cap;
		}
		/* the probe turns too: a crossing met while probing came at an
		   angle (7 Nov: taken as a crossing) */
		hist_step(odo, single || searching || (probe != 0));

		if (!enabled) {			/* turning, bypassing: no events */
			bpend = FALSE;
			app_timing_done(tmg_line);
			continue;
		}

		/* ---- Lost, and stuck sensors ---- */
		if ((state != LINE_FAULT) && (state != LINE_LOST)) {
			INT why = 0;

			if (searching) {
				state = LINE_SEARCH;
				if (((odo - search_from) > MM(S3_LOST_MM)) ||
				    (++search_n > (S3_LOST_MS / PERIOD_LINE_MS))) {
					state = LINE_LOST;
				}
			} else if (probe_none) {
				state = LINE_LOST;
			} else {
				state = LINE_ON;
			}
			if (single && ((odo - single_from) > MM(S3_STUCK_ONE_MM))) {
				why = LINE_FAULT_ONE;
			} else if ((stable == (PAT_L | PAT_R)) &&
				   ((odo - both_from) > MM(S3_STUCK_BOTH_MM))) {
				why = LINE_FAULT_BOTH;
			}
			if (why != 0) {
				fault_why = why;
				state = LINE_FAULT;
			}
			if ((LINE_LOST == state) || (LINE_FAULT == state)) {
				tk_set_flg(flg_mission, MSN_FLG_LINE_LOST);
			}
		}

		/* ---- Crossing: the end of a "both black" ---- */
		if ((prev == (PAT_L | PAT_R)) && (stable != prev)) {
			W len = odo - both_from;

			if ((len >= MM(S3_JUNC_MIN_MM)) && (len <= MM(S3_CROSS_MAX_MM)) &&
			    (quiet_over(both_from - MM(S3_CROSS_BEFORE_MM),
				       both_from - MM(S3_CROSS_LEAD_MM), odo))) {
				junction_at(both_from + (len / 2), JUNC_SIDE_S);
			} else if ((len >= MM(S3_PAIR_BOTH_MIN_MM)) && (len <= MM(S3_CROSS_MAX_MM))) {
				/* while correcting: a crossing only if B agrees */
				pair_c = both_from + (len / 2);
				pair_ok = TRUE;
			}
			/* else swept across the line on a curve: not a crossing */
		}
		if (pair_ok && ((odo - pair_c) > MM(S3_PAIR_MM - S3_BAR_AHEAD_MM + 20))) {
			pair_ok = FALSE;		/* B is past it and saw nothing */
		}

		/* ---- Left arm, from B: a patch about one tape wide.  cc is
		   where the line sensors crossed the same place (B is
		   S3_BAR_AHEAD_MM in front of them; negative = behind) ---- */
		{
			UW q = barcode_mark(&mc, &mw);

			if (q != mark_seen) {
				W cc = mc + MM(S3_BAR_AHEAD_MM);
				W dd = cc - pair_c;

				mark_seen = q;
				if (dd < 0) dd = -dd;
				if (pair_ok && (dd <= MM(S3_PAIR_MM)) &&
				    (mw >= S3_PAIR_B_MIN_MM10) && (mw <= (S3_JUNC_MAX_MM * 10))) {
					junction_at(pair_c, JUNC_SIDE_S | JUNC_SIDE_B);
					pair_ok = FALSE;
				}
				if ((mw >= (S3_JUNC_MIN_MM * 10)) && (mw <= (S3_JUNC_MAX_MM * 10))) {
					bpend = TRUE;
					bpend_c = mc + MM(S3_BAR_AHEAD_MM);
				}
			}
		}
		if (bpend && ((odo - bpend_c) >= MM(S3_ARM_AFTER_MM))) {
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
