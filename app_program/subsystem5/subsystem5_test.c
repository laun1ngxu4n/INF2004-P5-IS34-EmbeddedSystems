/*
 *	subsystem5_test.c
 *	On-target tests for subsystem 5.  Built only with: make TEST=1
 *
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem5/logs/ and record the result
 *	in testing/subsystem5/TEST_LOG.md.  Test IDs: S5-U01, S5-U02, ...
 *	INT-01 is recorded in testing/integration/TEST_LOG.md.
 */

#include "subsystem5.h"
#include "../test_report.h"
#include "../app_stage_test.h"
#include "../subsystem2/subsystem2.h"

#if APP_TEST

EXPORT void subsystem5_test(void)
{
	UW	p0;

	TEST_INIT();

	/* S5-U01 / S5-U02: Mission keeps waking with its 100 ms timeout */
	TEST_TIMEOUT("S5-U01", "S5-U02", tmg_mission);

	/* S5-U03 / S5-U04: Obstacle Scanner wakes every 60 ms without overruns */
	TEST_PERIODIC("S5-U03", "S5-U04", tmg_obstacle);

	/* INT-01: Telemetry's 1 Hz ping (IF-01) reaches Mission */
	p0 = mission_pings;
	tk_dly_tsk(3000);
	tm_printf((UB*)"[TEST] mission: %d pings in 3000 ms (expected 3)\n",
		  (INT)(mission_pings - p0));
	TEST_CHECK("INT-01", (mission_pings - p0) >= 2 && (mission_pings - p0) <= 4,
		   "IF-01 ping from Telemetry wakes Mission once a second");

	TEST_SUMMARY("subsystem5");
}

/* ---- Stage 3 hands-on tests ---------------------------------------- */

EXPORT void subsystem5_stage_test(void)
{
	OBSTACLE_PROFILE	a, b, p;
	MOTION_STATUS		st;
	INT			k, t;
	TEST_INIT();

	tm_printf((UB*)"\n[DRIVE] === Stage 3: obstacle ===\n");
	servo_set_angle(90);
	test_wait_start("Servo check: the ultrasonic should point STRAIGHT AHEAD "
			"(if not, refit the horn while it is powered).");
	servo_set_angle(90 + S5_SERVO_HALF_DEG);
	test_wait_start("Servo check: it should now point LEFT. If it points "
			"RIGHT, set S5_SERVO_LEFT_HIGH to 0 and rebuild. Do the "
			"wires at the back still have slack?");
	servo_set_angle(90 - S5_SERVO_HALF_DEG);
	test_wait_start("Servo check: now it should point RIGHT. Wires still slack?");
	if (S5_SERVO_SIDE_DEG > 0) {
		/* the side look on the leg past the box (8 Nov) */
		servo_look_side(+1);
		test_wait_start("Side look: it should now point STRAIGHT LEFT (90 deg, "
				"square to the robot's side). Wires slack, nothing "
				"pulling? If it cannot go that far, lower "
				"S5_SERVO_SIDE_DEG (0 = no side look) and rebuild.");
		servo_look_side(-1);
		test_wait_start("Side look: now STRAIGHT RIGHT (90 deg). Wires still "
				"slack?");
		tm_printf((UB*)"[S5] side distance for 8 s: hold a box side-on about "
			  "20 cm to the right, then take it away\n");
		for (t = 0; t < 25; t++) {
			tm_printf((UB*)"[S5] right %d mm\n", sonar_ping_mm());
			tk_dly_tsk(270);
		}
	}
	servo_set_angle(90);

	tm_printf((UB*)"[S5] forward distance for 10 s: move your hand in front\n");
	for (t = 0; t < 30; t++) {
		tm_printf((UB*)"[S5] front %d mm\n", sonar_ping_mm());
		tk_dly_tsk(270);
	}

	for (k = 1; k <= 3; k++) {
		tm_printf((UB*)"\n[DRIVE] S5-O01 width, try %d of 3 (a different box "
			  "each time, 10-20 cm wide; STOP ends early)\n", k);
		test_wait_start("Box about 250 mm straight ahead of the sensor (where "
				"the forward watch stops and the bypass scans first), "
				"flat face to it, nothing else within 60 cm. Write down "
				"its real width (ruler).");
		obstacle_scan(&a);
		motion_drive(-S2_SPEED_BARCODE, 100);	/* as the bypass does */
		do {
			tk_dly_tsk(10);
			motion_get_status(&st);
		} while (st.result == MOT_RES_BUSY);
		tk_dly_tsk(300);
		obstacle_scan(&b);
		obstacle_combine(&a, &b, &p);
		tm_printf((UB*)"[S5-O01] try %d: width %d +/- %d mm (scan 1 %d +/- %d, "
			  "scan 2 %d +/- %d), edges L %d R %d, front %d, clear L %d R %d, "
			  "edge seen by scan 1 L %d R %d, scan 2 L %d R %d\n",
			  k, p.width_mm, p.unc_mm, a.width_mm, a.unc_mm, b.width_mm,
			  b.unc_mm, p.left_mm, p.right_mm, p.front_mm,
			  p.left_clear, p.right_clear, a.left_seen, a.right_seen,
			  b.left_seen, b.right_seen);
		if (test_stop_pressed()) break;
	}
	TEST_SUMMARY("subsystem5 stage 3");

	tm_printf((UB*)"\n[DRIVE] S5-O02 bypass: Mission takes over. Put an obstacle "
		  "on the line ahead of the robot.\n");
	app_course_test(TRUE, FALSE);		/* IMU is Stage 4 */
}

#endif /* APP_TEST */
