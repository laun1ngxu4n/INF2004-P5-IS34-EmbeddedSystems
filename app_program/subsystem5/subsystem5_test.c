/*
 *  subsystem5_test.c
 *  On-target tests for subsystem 5.  Built only with: make TEST=1
 *
 *  Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *  console output to testing/subsystem5/logs/ and record the result
 *  in testing/subsystem5/TEST_LOG.md.  Test IDs: S5-U01, S5-U02, ...
 *  INT-01 is recorded in testing/integration/TEST_LOG.md.
 */

#include "subsystem5.h"
#include "../test_report.h"
#include "../app_stage_test.h"
#include "../subsystem2/subsystem2.h"

#if APP_TEST

EXPORT void subsystem5_test (void)
{
    UW pings_start;

    TEST_INIT();

    /* S5-U01 / S5-U02: Mission keeps waking with its 100 ms timeout */
    TEST_TIMEOUT("S5-U01", "S5-U02", g_tmg_mission);

    /* S5-U03 / S5-U04: Obstacle Scanner wakes every 60 ms without overruns */
    TEST_PERIODIC("S5-U03", "S5-U04", g_tmg_obstacle);

    /* INT-01: Telemetry's 1 Hz ping (IF-01) reaches Mission */
    pings_start = g_mission_pings;
    tk_dly_tsk(3000);
    tm_printf((UB *)"[TEST] mission: %d pings in 3000 ms (expected 3)\n",
              (INT)(g_mission_pings - pings_start));
    TEST_CHECK("INT-01",
               ((g_mission_pings - pings_start) >= 2) &&
                   ((g_mission_pings - pings_start) <= 4),
               "IF-01 ping from Telemetry wakes Mission once a second");

    TEST_SUMMARY("subsystem5");
}

/* ---- Stage 3 hands-on tests ---------------------------------------- */

EXPORT void subsystem5_stage_test (void)
{
    obstacle_profile_t scan_near;
    obstacle_profile_t scan_far;
    obstacle_profile_t profile;
    motion_status_t    status;
    INT                kdx;
    INT                tick;
    TEST_INIT();

    tm_printf((UB *)"\n[DRIVE] === Stage 3: obstacle ===\n");
    servo_set_angle(90);
    test_wait_start("Servo check: the ultrasonic should point STRAIGHT AHEAD "
                    "(if not, refit the horn while it is powered).");
    servo_set_angle(90 + S5_SERVO_HALF_DEG);
    test_wait_start("Servo check: it should now point LEFT. If it points "
                    "RIGHT, set S5_SERVO_LEFT_HIGH to 0 and rebuild. Do the "
                    "wires at the back still have slack?");
    servo_set_angle(90 - S5_SERVO_HALF_DEG);
    test_wait_start(
        "Servo check: now it should point RIGHT. Wires still slack?");
    if (S5_SERVO_SIDE_DEG > 0)
    {
        /* the side look on the leg past the box (8 Nov): does the side
           of the beam hear a box face beside the robot? */
        servo_look_side(-1);
        test_wait_start("Side look check next (12 s): get a box ready to hold "
                        "beside the robot on the right, then press START.");
        tm_printf(
            (UB *)"[S5] side look check, 12 s: the sonar points %d deg "
                  "right. Hold a box BESIDE the robot on the right, its face "
                  "parallel to the robot's side (as when driving past it), "
                  "about 20 cm out, level with the sonar: it should read "
                  "about 200. Then slide it backwards past the sonar: the "
                  "reading should go away once the box is behind it.\n",
            S5_SERVO_SIDE_DEG);
        for (tick = 0; tick < 40; tick++)
        {
            tm_printf((UB *)"[S5] side %d mm\n", sonar_ping_mm());
            tk_dly_tsk(240);
        }
    }
    servo_set_angle(90);

    tm_printf(
        (UB *)"[S5] forward distance for 10 s: move your hand in front\n");
    for (tick = 0; tick < 30; tick++)
    {
        tm_printf((UB *)"[S5] front %d mm\n", sonar_ping_mm());
        tk_dly_tsk(270);
    }

    for (kdx = 1; kdx <= 3; kdx++)
    {
        tm_printf((UB *)"\n[DRIVE] S5-O01 width, try %d of 3 (a different box "
                        "each time, 10-20 cm wide; STOP ends early)\n",
                  kdx);
        test_wait_start(
            "Box about 250 mm straight ahead of the sensor (where "
            "the forward watch stops and the bypass scans first), "
            "flat face to it, nothing else within 60 cm. Write down "
            "its real width (ruler).");
        obstacle_scan(&scan_near);
        motion_drive(-S2_SPEED_BARCODE, 100); /* as the bypass does */
        do
        {
            tk_dly_tsk(10);
            motion_get_status(&status);
        } while (MOT_RES_BUSY == status.result);
        tk_dly_tsk(300);
        obstacle_scan(&scan_far);
        obstacle_combine(&scan_near, &scan_far, &profile);
        tm_printf(
            (UB *)"[S5-O01] try %d: width %d +/- %d mm (scan 1 %d +/- %d, "
                  "scan 2 %d +/- %d), edges L %d R %d, front %d, clear L %d R "
                  "%d, "
                  "edge seen by scan 1 L %d R %d, scan 2 L %d R %d\n",
            kdx, profile.width_mm, profile.unc_mm, scan_near.width_mm,
            scan_near.unc_mm, scan_far.width_mm, scan_far.unc_mm,
            profile.left_mm, profile.right_mm, profile.front_mm,
            profile.b_left_clear, profile.b_right_clear, scan_near.b_left_seen,
            scan_near.b_right_seen, scan_far.b_left_seen,
            scan_far.b_right_seen);
        if (test_stop_pressed())
        {
            break;
        }
    }
    TEST_SUMMARY("subsystem5 stage 3");

    tm_printf(
        (UB *)"\n[DRIVE] S5-O02 bypass: Mission takes over. Put an obstacle "
              "on the line ahead of the robot.\n");
    app_course_test(TRUE, FALSE); /* IMU is Stage 4 */
}

#endif /* APP_TEST */

/*** end of file ***/

