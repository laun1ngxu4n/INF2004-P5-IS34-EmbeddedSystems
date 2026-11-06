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
#define REVERSE_MM		80	/* second scan position (O5) */
#define D_MAX_MM		150	/* deepest expected obstacle (A9) */
#define MARGIN_MM		50
#define REJOIN_ENABLE_MM	60	/* line events back on after this */
#define LINE_NEAR_MM		100	/* bypass: line accepted from y_off - this */

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
	tm_sprintf(msg, (const UB*)"junction %d (branch %s): %s (barcode %c)",
		   (INT)line_junctions(),
		   (sides == (JUNC_SIDE_S | JUNC_SIDE_B)) ? "both sides" :
		   (sides == JUNC_SIDE_B) ? "left" : (S3_SIDE < 0) ? "right" : "left",
		   (m == MOVE_LEFT) ? "LEFT" : (m == MOVE_RIGHT) ? "RIGHT" :
		   (m == MOVE_UTURN) ? "U-TURN" : "FORWARD", (ch != 0) ? ch : '-');
	say(msg);
	if (m == MOVE_FORWARD) return;

	set_state(M_MANOEUVRE);
	line_set_enabled(FALSE);
	obstacle_watch(FALSE);
	deg = (m == MOVE_LEFT) ? 90 : (m == MOVE_RIGHT) ? -90 : 180;
	/* wheels over the junction centre, then spin.  The line task says
	   where C crossed the centre: a left branch is reported later than
	   a right one (B must see the white after it), so measure from there. */
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

LOCAL void on_obstacle(void)
{
	OBSTACLE_PROFILE	a, b, p;
	UB			msg[TELE_TEXT_MAX];
	INT			sgn, w_plan, yc, edge_meas, edge_plan, y_off, x_pass;
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
	yc = (p.left_mm + p.right_mm) / 2;
	edge_plan = yc + sgn * w_plan / 2;
	edge_meas = ((sgn > 0) ? p.left_mm : p.right_mm) + sgn * p.unc_mm;
	if (sgn * edge_meas > sgn * edge_plan) edge_plan = edge_meas;
	y_off = sgn * edge_plan + S2_ROBOT_WIDTH_MM / 2 + MARGIN_MM;
	x_pass = p.front_mm + D_MAX_MM + S2_ROBOT_LENGTH_MM + MARGIN_MM;
	tm_sprintf(msg, (const UB*)"bypass %s: planned width %d mm, out %d mm, past %d mm",
		   (sgn > 0) ? "LEFT" : "RIGHT", w_plan, y_off, x_pass);
	say(msg);

	/* legs 1-5 */
	if (!mv(motion_turn(sgn * 90))) goto fail;
	if (!ahead_clear(y_off)) { halt("bypass leg 2 blocked"); return; }
	if (!mv(motion_drive(S5_FOLLOW_SPEED, y_off))) goto fail;
	if (!mv(motion_turn(-sgn * 90))) goto fail;
	if (!ahead_clear(x_pass)) { halt("bypass leg 4 blocked"); return; }
	if (!mv(motion_drive(S5_FOLLOW_SPEED, x_pass))) goto fail;
	if (!mv(motion_turn(-sgn * 90))) goto fail;

	/* leg 6: drive back toward the line until the centre sensor sees it */
	odo0 = motion_odometer();
	if (motion_drive(S5_FOLLOW_SPEED, y_off + 200) < E_OK) goto fail;
	/* Black only counts near where the line should be: on the tiled
	   floor the grout reads black too */
	while (!line_center_black() ||
	       S2_COUNTS_TO_MM(motion_odometer() - odo0) < y_off - LINE_NEAR_MM) {
		MOTION_STATUS st;

		motion_get_status(&st);
		if (app_stop_reasons() != 0u) goto fail;
		if (st.result != MOT_RES_BUSY) {
			halt("line not found after the bypass");
			return;
		}
		tk_dly_tsk(5);
	}
	motion_halt();
	wait_still();
	tm_sprintf(msg, (const UB*)"line found %d mm after the last turn",
		   (INT)S2_COUNTS_TO_MM(motion_odometer() - odo0));
	say(msg);

	/* leg 7: wheels onto the line, face the original direction, follow */
	if (!mv(motion_drive(S5_FOLLOW_SPEED, S3_SENSOR_TO_AXLE_MM))) goto fail;
	if (!mv(motion_turn(sgn * 90))) goto fail;
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
			if ((ptn & MSN_FLG_OBSTACLE) != 0u) {
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
