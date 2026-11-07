/*
 *	task_mission.c
 *	Subsystem 5 -- the Mission Controller task.
 *
 *	Waits on flg_mission (100 ms timeout) and runs the course:
 *	  IDLE      START button (pressed and released) or MQTT start
 *	  CAL       IMU level calibration, 1 s, robot still
 *	  RUN       follow the line at the barcode speed; react to:
 *	    barcode   store the latest character (FR11, TEAM.md 7.1 B5)
 *	    junction  do the mapped move for the stored character, U-turn
 *	              included (B6); no character = straight; a character
 *	              with no mapping = stop and report (B7)
 *	    obstacle  stop, scan twice from two positions, report width +/-,
 *	              bypass on the clear side, find the line again (7.2)
 *	    line lost stop and report
 *	  STOP (button or MQTT) at any time: motors off at once (in the
 *	  interrupt), Mission goes back to IDLE.
 *	Manoeuvres are run here step by step; each step waits for Motion's
 *	result and gives up as soon as a STOP reason is set.
 */

#include "subsystem5.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"

#define S5_FOLLOW_SPEED		S2_SPEED_BARCODE	/* B4: slow everywhere */
#define S5_APPROACH_SPEED	100	/* barcode read, its junction not yet:
					   slower, so weaving is less likely
					   and the junction is not overrun */
/* Scan positions (O5).  First scan where the forward watch stopped (20-25
   cm from the obstacle; backs up only if nearer than SCAN_NEAR_MM), then
   back REVERSE_MM and scan again.  Until 8 Nov it first backed up to 350
   mm, because from 20-25 cm the old width rule saw the box across the
   whole sweep.  With the geometry rule (task_obstacle.c) the near scans
   are the better ones: on the 29 real scans of 7 Nov, 200-280 mm away
   read 138-240 mm for 14.5-23 cm boxes, 340-380 mm away 206-293, 450+
   away 225-346; every pair of scans read narrower from nearer.  And it
   backs up 100 mm instead of about 210. */
#define SCAN_NEAR_MM		220
#define REVERSE_MM		100	/* second scan position */
#define D_MAX_MM		150	/* deepest expected obstacle (A9) */
#define MARGIN_MM		100	/* 50 until 7 Nov, then 70: tools/bypasssim.py
					   with +-5 % spin error passed a 200 mm box
					   21 mm away at 70, 42 mm at 100 */
#define REJOIN_ENABLE_MM	60	/* line events back on after this */
#define REJOIN_FAR_MM		400	/* back toward the line: give up this far
					   past where it should be */
#define SPIN_SEARCH_DEG		170	/* rejoin spin: at most this far ... */
#define SPIN_IGNORE_DEG		20	/* ... and black only after this */
#define SPIN_SPEED		40	/* each wheel, mm/s: slow, so it stops on
					   the 20 mm tape */

#define M_IDLE		0
#define M_CAL		1
#define M_RUN		2
#define M_MANOEUVRE	3

LOCAL INT		mstate = M_IDLE;
LOCAL volatile BOOL	armed = !APP_TEST;
LOCAL UB		pending;		/* latest unconsumed barcode, 0 = none */
LOCAL BOOL		use_obstacles = TRUE;
LOCAL BOOL		use_imu = TRUE;

EXPORT void mission_features(BOOL obstacles, BOOL imu)
{
	use_obstacles = obstacles;
	use_imu = imu;
}

LOCAL void watch_on(void)
{
	obstacle_watch(use_obstacles);
}

EXPORT void mission_arm(BOOL on)	{ armed = on; }

EXPORT const char *mission_state_name(void)
{
	static const char *n[] = {"IDLE", "CAL", "RUN", "MANOEUVRE"};
	return n[mstate];
}

LOCAL void set_state(INT s)
{
	mstate = s;
	obstacle_idle_ping(s == M_IDLE);
	tele_set_state(mission_state_name());
}

LOCAL void say(const UB *text)
{
	tele_event(text);
}

/* Wait for a Motion command; FALSE if it did not finish (STOP, timeout) */
LOCAL BOOL mv(ID id)
{
	MOTION_STATUS st;

	if (id < E_OK) return FALSE;
	while (1) {
		motion_get_status(&st);
		if (st.id == (UW)id && st.result != MOT_RES_BUSY) {
			return st.result == MOT_RES_DONE;
		}
		if (app_stop_reasons() != 0u) return FALSE;
		tk_dly_tsk(10);
	}
}

LOCAL void wait_still(void)
{
	MOTION_STATUS	st;
	INT		t;

	for (t = 0; t < 100; t++) {
		motion_get_status(&st);
		if (st.speed_l == 0 && st.speed_r == 0) break;
		tk_dly_tsk(10);
	}
}

LOCAL void halt(const char *why)
{
	UB msg[TELE_TEXT_MAX];

	motion_halt();
	motion_follow_limit(0);
	obstacle_watch(FALSE);
	line_set_enabled(FALSE);
	set_state(M_IDLE);
	tm_sprintf(msg, (const UB*)"stopped: %s (press START to go again)", why);
	say(msg);
}

/* Follow again; line events back on after a short distance */
LOCAL BOOL resume_follow(void)
{
	W odo0 = motion_odometer();

	barcode_reset();		/* a spin can leave a half-read code */
	if (motion_follow(S5_FOLLOW_SPEED) < E_OK) return FALSE;
	while (S2_COUNTS_TO_MM(motion_odometer() - odo0) < REJOIN_ENABLE_MM) {
		if (app_stop_reasons() != 0u) return FALSE;
		tk_dly_tsk(10);
	}
	line_set_enabled(TRUE);
	watch_on();
	tk_clr_flg(flg_mission, ~(MSN_FLG_JUNCTION | MSN_FLG_LINE_LOST |
				  MSN_FLG_OBSTACLE | MSN_FLG_BARCODE));
	return TRUE;
}

LOCAL void start_run(void)
{
	INT t;

	say((const UB*)"start");
	set_state(M_CAL);
	motion_halt();
	if (use_imu && imu_ok()) {
		imu_calibrate();
		for (t = 0; t < 200 && !imu_calibrated(); t++) tk_dly_tsk(10);
	}
	pending = 0;
	motion_follow_limit(0);
	barcode_reset();		/* nothing left over from before START */
	tk_clr_flg(flg_mission, 0);
	if (motion_follow(S5_FOLLOW_SPEED) < E_OK) {
		halt("STOP still set");
		return;
	}
	line_set_enabled(TRUE);
	watch_on();
	set_state(M_RUN);
}

LOCAL void on_barcode(void)
{
	BARCODE_RESULT	r;
	UB		msg[TELE_TEXT_MAX];
	INT		m;

	barcode_last(&r);
	if (!r.ok) {
		tm_sprintf(msg, (const UB*)"barcode READ_FAIL (%d elements)", r.elements);
		say(msg);
		return;
	}
	pending = r.ch;
	m = barcode_move_for(r.ch);
	/* a turn is coming: approach its junction slowly */
	motion_follow_limit((m == MOVE_LEFT || m == MOVE_RIGHT || m == MOVE_UTURN) ?
			    S5_APPROACH_SPEED : 0);
	tm_sprintf(msg, (const UB*)"barcode %c%s (narrow %d.%d mm, wide %d.%d mm) -> %s",
		   r.ch, r.reversed ? " read backwards" : "",
		   r.narrow_mm10 / 10, r.narrow_mm10 % 10, r.wide_mm10 / 10, r.wide_mm10 % 10,
		   (m == MOVE_LEFT) ? "LEFT at next junction" :
		   (m == MOVE_RIGHT) ? "RIGHT at next junction" :
		   (m == MOVE_FORWARD) ? "FORWARD at next junction" :
		   (m == MOVE_UTURN) ? "U-TURN at next junction" : "NO MAPPING");
	say(msg);
}

LOCAL void on_junction(void)
{
	UB	msg[TELE_TEXT_MAX];
	UB	ch = pending;
	INT	m = (ch != 0) ? barcode_move_for(ch) : MOVE_FORWARD;
	INT	deg, sides = line_junction_sides(), to_go;

	motion_follow_limit(0);
	if (ch != 0 && m == MOVE_NONE) {
		tm_sprintf(msg, (const UB*)"no mapping for barcode %c", ch);
		halt((const char*)msg);
		return;
	}
	pending = 0;
	tm_sprintf(msg, (const UB*)"junction %d (%s): %s (barcode %c)",
		   (INT)line_junctions(),
		   ((sides & JUNC_SIDE_S) != 0) ? "crossing" : "branch left",
		   (m == MOVE_LEFT) ? "LEFT" : (m == MOVE_RIGHT) ? "RIGHT" :
		   (m == MOVE_UTURN) ? "U-TURN" : "FORWARD", (ch != 0) ? ch : '-');
	say(msg);
	if (m == MOVE_FORWARD) return;

	set_state(M_MANOEUVRE);
	line_set_enabled(FALSE);
	obstacle_watch(FALSE);
	deg = (m == MOVE_LEFT) ? 90 : (m == MOVE_RIGHT) ? -90 : 180;
	/* wheels over the junction centre, then spin.  The line task says
	   where the line sensors crossed the centre: a left branch is
	   reported later than a crossing (B is behind them), so measure
	   from there. */
	to_go = S3_SENSOR_TO_AXLE_MM -
		(INT)S2_COUNTS_TO_MM(motion_odometer() - line_junction_odo());
	if ((to_go > 0 && !mv(motion_drive(S5_FOLLOW_SPEED, to_go))) ||
	    (to_go < 0 && !mv(motion_drive(-S5_FOLLOW_SPEED, -to_go))) ||
	    !mv(motion_turn(deg)) || !resume_follow()) {
		halt("junction move did not finish");
		return;
	}
	set_state(M_RUN);
}

/* Check the way ahead is clear for dist_mm before driving it */
LOCAL BOOL ahead_clear(INT dist_mm)
{
	INT d = sonar_ping_mm();

	return d == SONAR_OPEN || (d >= 0 && d > dist_mm + MARGIN_MM);	/* fault = not clear */
}

/* Drive dist_mm straight; stop as soon as a line sensor sees black, but
   only after from_mm.  1 = line found, 0 = drove it all, -1 = failed */
LOCAL INT drive_until_line(INT dist_mm, INT from_mm)
{
	W	odo0 = motion_odometer();

	if (motion_drive(S5_FOLLOW_SPEED, dist_mm) < E_OK) return -1;
	while (1) {
		MOTION_STATUS st;

		if (app_stop_reasons() != 0u) return -1;
		if (line_any_black() &&
		    S2_COUNTS_TO_MM(motion_odometer() - odo0) >= from_mm) {
			motion_halt();
			wait_still();
			return 1;
		}
		motion_get_status(&st);
		if (st.result != MOT_RES_BUSY) {
			return (st.result == MOT_RES_DONE) ? 0 : -1;
		}
		tk_dly_tsk(5);
	}
}

/* Leg 4, past the box (8 Nov).  With the side look the sonar turns 90 deg
   toward the box and pings every 60 ms while the robot drives: the box's
   side face echoes while the robot is beside it, and SIDE_MISS_PINGS in
   a row without it = past its back, whatever its depth.  The robot goes
   on until its axle is half its width + MARGIN_MM past the last echo
   (room to turn and drive back to the line), and only from there do the
   line sensors count (a junction's side line beside the box is not the
   line).  Until 8 Nov: a fixed distance for a box up to 150 mm deep
   (D_MAX_MM, team assumption A9); tools/bypasssim.py: a 250 mm deep box
   was hit that way.  The side look never saw the box: that fixed
   distance.  A sonar fault counts as "box still there" (never past by
   mistake).
   1 = line met, 0 = past the box (turn back to the line), -1 = halted */
#define SIDE_SEEN_PAST_LINE_MM	50	/* an echo nearer than the line + this
					   is the box */
#define SIDE_MISS_PINGS		3
#define SIDE_EXTRA_MM		300	/* on at most this past x_pass while
					   still beside the box */
#define SIDE_BEAM_TAN_X1000	176	/* tan 10 deg: the beam (about +-15 deg)
					   still hears the box's back corner
					   after the sonar has passed it; take
					   back 10 deg of that (bypasssim: tight
					   curves behind the box missed without) */
LOCAL INT pass_box(INT sgn, INT front, INT y_off, INT x_pass)
{
	UB	msg[TELE_TEXT_MAX];
	BOOL	side = (S5_SERVO_SIDE_DEG > 0), seen = FALSE, past = FALSE;
	BOOL	never = FALSE, line_on;
	INT	thr = y_off + SIDE_SEEN_PAST_LINE_MM;
	INT	line_from = front + D_MAX_MM + 30;	/* without the side look */
	INT	first = 0, last = 0, dlast = 0, miss = 0, stop_at = 0, dist, done, d, r = -1;
	UW	t_ping;
	W	odo0;
	MOTION_STATUS st;

	/* how far it may go: the way ahead must be clear */
	dist = x_pass + (side ? SIDE_EXTRA_MM : 0);
	d = sonar_ping_mm();
	if (d == SONAR_FAULT || (d >= 0 && d - MARGIN_MM < x_pass)) {
		halt("bypass leg 4 blocked");
		return -1;
	}
	if (d >= 0 && d - MARGIN_MM < dist) dist = d - MARGIN_MM;

	if (side) servo_look_side(-sgn);		/* the box is on the other side */
	odo0 = motion_odometer();
	if (motion_drive(S5_FOLLOW_SPEED, dist) < E_OK) goto out;
	sonar_ping_start();
	t_ping = app_time_us();
	while (1) {
		if (app_stop_reasons() != 0u) goto out;
		done = (INT)S2_COUNTS_TO_MM(motion_odometer() - odo0);

		if (side && app_time_us() - t_ping >= 60000u) {
			d = sonar_ping_result();
			sonar_ping_start();
			t_ping = app_time_us();
			if (d == SONAR_FAULT || (d >= 0 && d < thr)) {
				if (!seen) first = done;
				seen = TRUE;
				last = done;
				if (d >= 0) dlast = d;
				miss = 0;
			} else if (seen && !past && ++miss >= SIDE_MISS_PINGS) {
				past = TRUE;
				/* the box's back: where the sonar was at the
				   last echo, less a little of the beam */
				stop_at = last + S5_SONAR_TO_AXLE_MM -
					  dlast * SIDE_BEAM_TAN_X1000 / 1000 +
					  S2_ROBOT_WIDTH_MM / 2 + MARGIN_MM;
				tm_sprintf(msg, (const UB*)"side look: box beside for %d mm, "
					   "past it %d mm along", last - first, done);
				say(msg);
			}
			if (!seen && !never && done > front + D_MAX_MM + 60) {
				never = TRUE;
				say((const UB*)"side look never saw the box: assuming "
				    "it is at most 150 mm deep");
			}
		}
		line_on = past || ((!side || never) && done >= line_from);
		if (line_on && line_any_black()) { r = 1; break; }
		if (past && done >= stop_at) { r = 0; break; }
		if (never && done >= x_pass) { r = 0; break; }
		motion_get_status(&st);
		if (st.result != MOT_RES_BUSY) {
			if (st.result != MOT_RES_DONE) goto out;
			if (!side) { r = 0; break; }	/* drove x_pass */
			motion_halt();
			servo_set_angle(90);
			halt(past ? "no room past the box" :
			     "still beside the box at the end of the leg (box too long?)");
			return -1;
		}
		tk_dly_tsk(5);
	}
	motion_halt();
	wait_still();
	if (side) servo_set_angle(90);
	return r;
out:
	motion_halt();
	if (side) servo_set_angle(90);
	halt("bypass step did not finish");
	return -1;
}

/* Spin slowly by up to deg; stop when a line sensor sees black after
   SPIN_IGNORE_DEG.  1 = found, 0 = turned it all, -1 = failed */
LOCAL INT spin_until_line(INT deg)
{
	MOTION_STATUS	st;
	W		l0, r0;

	motion_get_status(&st);
	l0 = st.count_l;
	r0 = st.count_r;
	if (motion_turn_at(deg, SPIN_SPEED) < E_OK) return -1;
	while (1) {
		W   dd;
		INT turned;

		if (app_stop_reasons() != 0u) return -1;
		motion_get_status(&st);
		dd = (st.count_r - r0) - (st.count_l - l0);
		if (dd < 0) dd = -dd;
		/* counts -> mm of wheel difference -> degrees over the track */
		turned = (INT)((dd * S2_UM_PER_COUNT / 1000) * 573 / (S2_TRACK_MM * 10));
		if (turned >= SPIN_IGNORE_DEG && line_any_black()) {
			motion_halt();
			wait_still();
			return 1;
		}
		if (st.result != MOT_RES_BUSY) {
			return (st.result == MOT_RES_DONE) ? 0 : -1;
		}
		tk_dly_tsk(5);
	}
}

LOCAL void on_obstacle(void)
{
	OBSTACLE_PROFILE	a, b, p;
	UB			msg[TELE_TEXT_MAX];
	INT			sgn, w_plan, yc, edge_meas, edge_plan, y_off, x_pass;
	INT			found;
	W			odo0;

	set_state(M_MANOEUVRE);
	motion_halt();
	obstacle_watch(FALSE);
	line_set_enabled(FALSE);
	wait_still();
	if (obstacle_sensor_fault()) {
		halt("ultrasonic not answering (no ECHO), not driving blind");
		return;
	}
	tm_sprintf(msg, (const UB*)"obstacle ahead at %d mm: scanning", obstacle_front_mm());
	say(msg);

	/* first scan from here, unless it stopped nearer than SCAN_NEAR_MM */
	{
		INT d = sonar_ping_mm();
		INT back = SCAN_NEAR_MM - ((d >= 0) ? d : S5_TRIP_MM);

		if (back > 10) {
			if (!mv(motion_drive(-S5_FOLLOW_SPEED, back))) {
				halt("reverse for the 1st scan did not finish");
				return;
			}
			wait_still();
		}
	}
	obstacle_scan(&a);
	if (a.fault_rays > a.rays / 2) {
		halt("ultrasonic not answering during the scan");
		return;
	}
	if (!a.found) {
		say((const UB*)"scan found nothing ahead, going on");
		if (!resume_follow()) halt("could not restart");
		else set_state(M_RUN);
		return;
	}
	if (!mv(motion_drive(-S5_FOLLOW_SPEED, REVERSE_MM))) {	/* O5 */
		halt("reverse for 2nd scan did not finish");
		return;
	}
	wait_still();
	obstacle_scan(&b);
	obstacle_combine(&a, &b, &p);
	tm_sprintf(msg, (const UB*)"obstacle width %d +/- %d mm (scan 1: %d +/- %d, "
		   "scan 2: %d +/- %d); edges L %d R %d mm; front %d mm",
		   p.width_mm, p.unc_mm, a.width_mm, a.unc_mm, b.width_mm, b.unc_mm,
		   p.left_mm, p.right_mm, p.front_mm);
	say(msg);
	if (!p.found) {
		halt("obstacle lost between scans");
		return;
	}

	/* Side: clear, and the nearer edge (less to drive) */
	if (p.left_clear && (!p.right_clear || p.left_mm <= -p.right_mm)) {
		sgn = +1;
	} else if (p.right_clear) {
		sgn = -1;
	} else {
		halt("no clear side");
		return;
	}
	/* O7: plan for at least the largest expected obstacle */
	w_plan = p.width_mm + p.unc_mm;
	if (w_plan < S5_MAX_WIDTH_MM) w_plan = S5_MAX_WIDTH_MM;
	if (w_plan > S5_PLAN_MAX_MM) w_plan = S5_PLAN_MAX_MM;
	yc = (p.left_mm + p.right_mm) / 2;
	edge_plan = yc + sgn * w_plan / 2;
	edge_meas = ((sgn > 0) ? p.left_mm : p.right_mm) + sgn * p.unc_mm;
	if (sgn * edge_meas > sgn * edge_plan) edge_plan = edge_meas;
	/* never further out than the largest planned box, sitting on the
	   line give or take 50 mm */
	if (sgn * edge_plan > S5_PLAN_MAX_MM / 2 + 50) {
		edge_plan = sgn * (S5_PLAN_MAX_MM / 2 + 50);
	}
	y_off = sgn * edge_plan + S2_ROBOT_WIDTH_MM / 2 + MARGIN_MM;
	x_pass = p.front_mm + D_MAX_MM + S2_ROBOT_LENGTH_MM + MARGIN_MM;
	tm_sprintf(msg, (const UB*)"bypass %s: planned width %d mm, out %d mm, past %d mm",
		   (sgn > 0) ? "LEFT" : "RIGHT", w_plan, y_off, x_pass);
	say(msg);

	/* legs 1-4.  The leg past the box (pass_box) watches the line
	   sensors once the robot is past the box's back: a line that curves
	   toward this side behind the box is met there.  Not before: 8 Nov
	   the box stood on a junction and this leg met the junction's side
	   line beside the box ("line met beside the box, 599 mm along" = 14
	   cm past its front) and followed it away (tools/bypasssim.py, boxes
	   on a junction).  Until 8 Nov it watched from the box's front. */
	if (!mv(motion_turn(sgn * 90))) goto fail;
	if (!ahead_clear(y_off)) { halt("bypass leg 2 blocked"); return; }
	if (!mv(motion_drive(S5_FOLLOW_SPEED, y_off))) goto fail;
	if (!mv(motion_turn(-sgn * 90))) goto fail;
	odo0 = motion_odometer();
	found = pass_box(sgn, p.front_mm, y_off, x_pass);
	if (found < 0) return;			/* halted, reason given */
	if (found) {
		tm_sprintf(msg, (const UB*)"line met behind the box, %d mm along",
			   (INT)S2_COUNTS_TO_MM(motion_odometer() - odo0));
		say(msg);
	} else {
		/* leg 5-6: back toward the line, any black counts (it may
		   have curved either way behind the box) */
		if (!mv(motion_turn(-sgn * 90))) goto fail;
		odo0 = motion_odometer();
		found = drive_until_line(y_off + REJOIN_FAR_MM, 20);
		if (found < 0) goto fail;
		if (!found) {
			halt("line not found after the bypass");
			return;
		}
		tm_sprintf(msg, (const UB*)"line found %d mm after the last turn",
			   (INT)S2_COUNTS_TO_MM(motion_odometer() - odo0));
		say(msg);
	}

	/* leg 7: wheels onto the line, then spin toward the old direction
	   until a line sensor finds the tape (whatever way the line runs
	   now: straight, curved, or the turns were a few degrees off) */
	if (!mv(motion_drive(S5_FOLLOW_SPEED, S3_SENSOR_TO_AXLE_MM))) goto fail;
	if (!line_any_black()) {
		/* (already on the tape: a curve met at a shallow angle, so the
		   robot runs along it; spinning would turn it away) */
		found = spin_until_line(sgn * SPIN_SEARCH_DEG);
		if (found < 0) goto fail;
		if (!found) {
			/* not the other way: that found the line pointing back at
			   the box in tools/bypasssim.py and drove into it */
			halt("line not found after the bypass (spin)");
			return;
		}
	}
	if (!resume_follow()) goto fail;
	say((const UB*)"bypass done, following the line");
	set_state(M_RUN);
	return;
fail:
	halt("bypass step did not finish");
}

EXPORT void mission_task(INT stacd, void *exinf)
{
	UINT	ptn;
	ER	er;
	BOOL	start_was_down = FALSE;

	(void)stacd; (void)exinf;
	tele_set_state(mission_state_name());

	while (1) {
		er = tk_wai_flg(flg_mission, MSN_FLG_ALL, TWF_ORW | TWF_BITCLR,
				&ptn, TIMEOUT_MISSION_MS);
		app_timing_wake(tmg_mission, 0);
		if (er != E_OK) ptn = 0;

		if ((ptn & MSN_FLG_PING) != 0u) {
			mission_pings++;
		}

		/* STOP: the motors are already off (app_stop_request) */
		if (mstate != M_IDLE && app_stop_reasons() != 0u) {
			obstacle_watch(FALSE);
			line_set_enabled(FALSE);
			set_state(M_IDLE);
			say((const UB*)"STOP");
		}

		if (mstate == M_IDLE && armed) {
			BOOL down = button_down(PIN_START) && !button_down(PIN_STOP);
			BOOL go = (ptn & MSN_FLG_START) != 0u;

			if (start_was_down && !down) go = TRUE;	/* released */
			start_was_down = down;
			if (go) {
				if (!button_down(PIN_STOP)) {
					app_stop_clear(STOP_BUTTON | STOP_MQTT);
				}
				start_run();
			}
		} else if (mstate == M_RUN) {
			/* barcode first: a code just before a junction counts */
			if ((ptn & MSN_FLG_BARCODE) != 0u) on_barcode();
			if (line_state() == LINE_FAULT) {
				/* a stuck line sensor: nothing it says can be trusted */
				halt((line_fault() == LINE_FAULT_ONE) ?
				     "a line sensor black for 30 cm (stuck black? check its pot and cable)" :
				     "both line sensors black for 15 cm (stuck, or a wide black area)");
			} else if ((ptn & MSN_FLG_OBSTACLE) != 0u) {
				on_obstacle();
			} else if ((ptn & MSN_FLG_JUNCTION) != 0u) {
				on_junction();
			} else if ((ptn & MSN_FLG_LINE_LOST) != 0u) {
				halt("line lost");
			}
		}

		app_timing_done(tmg_mission);
	}
}
