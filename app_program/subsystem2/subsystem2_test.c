/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem2/logs/ and record the result
 *	in testing/subsystem2/TEST_LOG.md.  Test IDs: S2-U01, S2-U02, ...
 *
 *	subsystem2_drive_test() (Stage 1) DRIVES THE ROBOT.  It runs after the
 *	automatic tests and waits for the START button (GP20) before every
 *	move.  STOP (GP21) stops the motors at any time.
 */

#include "subsystem2.h"
#include "../test_report.h"
#include "../subsystem5/subsystem5.h"	/* START / STOP buttons */
#include "../app_stage_test.h"

#if APP_TEST

EXPORT void subsystem2_test(void)
{
	TEST_INIT();

	/* S2-U01 / S2-U02: Motion wakes every 10 ms without overruns */
	TEST_PERIODIC("S2-U01", "S2-U02", tmg_motion);

	TEST_SUMMARY("subsystem2");
}

/* ---- Stage 1 drive tests ------------------------------------------- */

#define PI_F		3.14159f

LOCAL const char *res_name(INT r)
{
	switch (r) {
	case MOT_RES_DONE:	return "DONE";
	case MOT_RES_STOPPED:	return "STOPPED";
	case MOT_RES_TIMEOUT:	return "TIMEOUT";
	case MOT_RES_BUSY:	return "BUSY";
	default:		return "NONE";
	}
}

/* Wait until command id is no longer BUSY (at most 15 s) */
LOCAL INT wait_done(ID id, MOTION_STATUS *st)
{
	INT	t;

	for (t = 0; t < 1500; t++) {
		motion_get_status(st);
		if (st->id == (UW)id && st->result != MOT_RES_BUSY) {
			return st->result;
		}
		tk_dly_tsk(10);
	}
	motion_halt();
	return MOT_RES_TIMEOUT;
}

/* S2-M02: three rounds of left 90, right 90, 180 */
LOCAL BOOL test_turns(void)
{
	static const INT	turns[3] = {90, -90, 180};
	static const char	*names[3] = {"left 90", "right 90", "180 (left)"};
	MOTION_STATUS		st;
	BOOL			all_done = TRUE;
	INT			round, k;

	test_wait_start("S2-M02 turns: robot on the floor, mark its heading "
		   "(tape along one side). 3 rounds of: left 90, right 90, 180.");
	for (round = 1; round <= 3; round++) {
		for (k = 0; k < 3; k++) {
			W	l0, r0;
			UW	t0 = app_time_us();
			ID	id;
			INT	res, deg10;

			motion_get_status(&st);
			l0 = st.count_l;
			r0 = st.count_r;
			id = motion_turn(turns[k]);
			if (id < E_OK) {
				tm_printf((UB*)"[DRIVE] STOP is set, turn not started\n");
				return FALSE;
			}
			res = wait_done(id, &st);
			tk_dly_tsk(300);		/* let it settle, then count */
			motion_get_status(&st);
			/* spin angle from the wheels: (right - left) / track */
			deg10 = (INT)((float)((st.count_r - r0) - (st.count_l - l0)) *
				S2_UM_PER_COUNT / 1000.0f / S2_TRACK_MM * 1800.0f / PI_F);
			tm_printf((UB*)"[S2-M02] round %d %s: %s in %d ms, ",
				  round, names[k], res_name(res),
				  (INT)((app_time_us() - t0) / 1000u) - 300);
			test_print_x10("encoder angle ", deg10, " deg\n");
			if (res != MOT_RES_DONE) {
				all_done = FALSE;
				if (res == MOT_RES_STOPPED) {
					return FALSE;
				}
			}
			tk_dly_tsk(700);
		}
		tm_printf((UB*)"[S2-M02] round %d done: write down the real "
			  "angles (left 90, right 90, 180) from the tape\n", round);
		if (round < 3) {
			test_wait_start("S2-M02 next round: put the robot back on its mark");
		}
	}
	return all_done;
}

/* S2-M03: 500 mm at the barcode speed; speed over the middle 300 mm */
LOCAL BOOL test_slow_speed(void)
{
	MOTION_STATUS	st;
	W		l0, r0;
	INT		n = 0, vmin = 100000, vmax = -100000;
	W		vsum = 0;
	ID		id;
	INT		res, mean = 0, diff_mm;

	test_wait_start("S2-M03 slow speed: 0.5 m clear in front of the robot. "
		   "It drives 500 mm at the barcode speed.");
	motion_get_status(&st);
	l0 = st.count_l;
	r0 = st.count_r;
	id = motion_drive(S2_SPEED_BARCODE, 500);
	if (id < E_OK) {
		return FALSE;
	}
	do {
		INT	mm, v;

		tk_dly_tsk(20);
		motion_get_status(&st);
		mm = (INT)(((st.count_l - l0) + (st.count_r - r0)) / 2 *
			   S2_UM_PER_COUNT / 1000);
		if (mm >= 100 && mm <= 400) {
			v = (st.speed_l + st.speed_r) / 2;
			vsum += v;
			n++;
			if (v < vmin) vmin = v;
			if (v > vmax) vmax = v;
		}
	} while (!(st.id == (UW)id && st.result != MOT_RES_BUSY));
	res = st.result;
	if (n > 0) {
		mean = (INT)(vsum / n);
	}
	diff_mm = (INT)(((st.count_l - l0) - (st.count_r - r0)) * S2_UM_PER_COUNT / 1000);
	tm_printf((UB*)"[S2-M03] %s; target %d mm/s; middle 300 mm: mean %d, "
		  "min %d, max %d mm/s (%d samples); left - right %d mm\n",
		  res_name(res), S2_SPEED_BARCODE, mean, vmin, vmax, n, diff_mm);
	return res == MOT_RES_DONE && n > 0 &&
	       mean >= S2_SPEED_BARCODE * 9 / 10 && mean <= S2_SPEED_BARCODE * 11 / 10;
}

/* S2-M04: drive, press STOP; time from press to wheels stopped */
LOCAL BOOL test_stop_once(INT k)
{
	MOTION_STATUS	st;
	ID		id;
	W		last_l, last_r, c0_l = 0, c0_r = 0;
	UW		t_press = 0, t_last_move = 0;
	BOOL		pressed = FALSE;
	INT		res;

	tm_printf((UB*)"\n[DRIVE] S2-M04 try %d of 5: it drives up to 1.5 m at "
		  "%d mm/s. PRESS STOP (GP21) while it drives.\n", k, S2_SPEED_LINE);
	test_wait_start("S2-M04 STOP");
	id = motion_drive(S2_SPEED_LINE, 1500);
	if (id < E_OK) {
		return FALSE;
	}
	motion_get_status(&st);
	last_l = st.count_l;
	last_r = st.count_r;
	while (1) {
		tk_dly_tsk(10);
		motion_get_status(&st);
		if (!pressed && app_stop_reasons() != 0u) {
			pressed = TRUE;
			t_press = app_stop_time_us();
			c0_l = st.count_l;
			c0_r = st.count_r;
			t_last_move = app_time_us();
		}
		if (st.count_l != last_l || st.count_r != last_r) {
			last_l = st.count_l;
			last_r = st.count_r;
			t_last_move = app_time_us();
		}
		if (pressed && app_time_us() - t_last_move > 100000u) {
			break;			/* no count for 100 ms: stopped */
		}
		if (!pressed && st.id == (UW)id && st.result != MOT_RES_BUSY) {
			break;			/* finished without a STOP */
		}
	}
	res = st.result;
	if (!pressed) {
		tm_printf((UB*)"[S2-M04] try %d: no STOP pressed (%s), not counted\n",
			  k, res_name(res));
		return FALSE;
	}
	tm_printf((UB*)"[S2-M04] try %d: %s; wheels stopped %d ms after the "
		  "press; rolled %d mm after the press\n", k, res_name(res),
		  (INT)((t_last_move - t_press) / 1000u),
		  (INT)(((last_l - c0_l) + (last_r - c0_r)) / 2 *
			S2_UM_PER_COUNT / 1000));
	return res == MOT_RES_STOPPED && (t_last_move - t_press) < 500000u;
}

EXPORT void subsystem2_drive_test(void)
{
	MOTION_STATUS	st;
	INT		k, stops = 0;
	TEST_INIT();

	tm_printf((UB*)"\n[DRIVE] === Stage 1 drive tests ===\n"
		  "[DRIVE] Power bank on VIN, USB to the laptop for the log.\n"
		  "[DRIVE] Robot ON THE FLOOR with about 1.5 m clear in front.\n"
		  "[DRIVE] STOP (GP21) stops the motors at any time.\n"
		  "[DRIVE] Track width used for turns: %d mm (S2_TRACK_MM)\n",
		  S2_TRACK_MM);

	TEST_CHECK("S2-M02", test_turns(),
		   "all 9 turns finished (judge the angles from the tape: "
		   "90 +/- 10, 180 +/- 15)");
	TEST_CHECK("S2-M03", test_slow_speed(),
		   "barcode speed: mean within 10 % over the middle 300 mm");
	for (k = 1; k <= 5; k++) {
		if (test_stop_once(k)) {
			stops++;
		}
	}
	tm_printf((UB*)"[S2-M04] %d of 5 stopped within 500 ms\n", stops);
	TEST_CHECK("S2-M04", stops == 5, "STOP stops the wheels, 5 of 5");

	motion_get_status(&st);
	tm_printf((UB*)"[DRIVE] encoder quadrature errors since boot: %d\n",
		  (INT)st.invalid);
	TEST_SUMMARY("subsystem2 drive");
	tm_printf((UB*)"[DRIVE] === Stage 1 drive tests end ===\n");
}

#endif /* APP_TEST */
