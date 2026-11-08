/*
 *  subsystem4_test.c
 *  On-target tests for subsystem 4.  Built only with: make TEST=1
 *
 *  Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *  console output to testing/subsystem4/logs/ and record the result
 *  in testing/subsystem4/TEST_LOG.md.  Test IDs: S4-U01, S4-U02, ...
 */

#include "subsystem4.h"
#include "../test_report.h"
#include "../app_stage_test.h"
#include "../subsystem2/subsystem2.h"

#if APP_TEST

EXPORT void subsystem4_test (void)
{
    TEST_INIT();

    /* S4-U01 / S4-U02: Safety wakes every 20 ms without overruns */
    TEST_PERIODIC("S4-U01", "S4-U02", g_tmg_safety);

    /* S4-U03 / S4-U04: IMU wakes every 10 ms without overruns */
    TEST_PERIODIC("S4-U03", "S4-U04", g_tmg_imu);

    TEST_SUMMARY("subsystem4");
}

/* ---- Stage 4 hands-on tests ---------------------------------------- */

/* One S4-R01 run: drive at the follow speed, cut the run into "bumps"
   (from |tilt| over BUMP_SIN until BUMP_FLAT_MM level again) and print
   each one: where, nose-up and nose-down peaks, L x sin(nose-up peak)
   (the hump height when the robot tips over a hump shorter than itself),
   the height climbed, and the readings skipped as jolts.  While on a bump
   it prints the tilt every 10 ms (about 1.2 mm), so the order of front
   support up / wheels up shows.  The run ends BUMP_END_MM after the last
   bump (two ramps in one run are fine), or at RUN_MAX_MM.  7 Nov: two
   humps in one 1.5 m run, neither reported, and the 0.5 s trace missed
   the short nose-up on each. */
#define RUN_MAX_MM   1500
#define BUMP_SIN     40  /* x1000: about 2.3 deg */
#define BUMP_FLAT_MM 60  /* level this far = the bump is over */
#define BUMP_END_MM  300 /* run ends this far after the last bump */
#define TRACE_MAX    400 /* 10 ms trace lines per run */

LOCAL void ramp_run (UW reports_start)
{
    W    odo0        = motion_odometer();
    UW   jolts_start = imu_jolts();
    UW   jolts_bump  = 0;
    INT  tick;
    INT  bumps     = 0;
    INT  traced    = 0;
    BOOL b_in_bump = FALSE;
    INT  b_from    = 0;
    INT  b_flat    = 0;
    INT  b_up      = 0;
    INT  b_down    = 0;
    INT  b_h0      = 0;
    INT  b_hmax    = 0;
    INT  b_roll    = 0;
    INT  b_one     = 0;
    INT  last_end  = -1;
    UW   seen      = reports_start;

    (void)motion_drive(S2_SPEED_BARCODE, RUN_MAX_MM);
    for (tick = 0; tick < 2000; tick++)
    {
        motion_status_t status;
        INT dist_mm   = (INT)S2_COUNTS_TO_MM(motion_odometer() - odo0);
        INT sin_pitch = imu_pitch_mdeg_sin();
        INT height    = imu_height_mm10();
        INT roll      = imu_roll_sin_x1000();
        INT one       = imu_one_wheel_mm10();
        INT roll_abs  = (roll < 0) ? -roll : roll;

        motion_get_status(&status);
        if ((tick > 10) && (status.result != MOT_RES_BUSY))
        {
            break;
        }
        if (test_stop_pressed())
        {
            break;
        }

        if ((!b_in_bump) && ((sin_pitch > BUMP_SIN) ||
                             (sin_pitch < -BUMP_SIN) || (roll_abs > BUMP_SIN)))
        {
            b_in_bump = TRUE;
            b_from    = dist_mm;
            b_flat    = dist_mm;
            b_up = b_down = 0;
            b_roll = b_one = 0;
            b_h0 = b_hmax = height;
            jolts_bump    = imu_jolts();
            tm_printf((UB *)"[S4] bump %d starts at %d mm\n", bumps + 1,
                      dist_mm);
        }
        if (b_in_bump)
        {
            if (sin_pitch > b_up)
            {
                b_up = sin_pitch;
            }
            if (sin_pitch < b_down)
            {
                b_down = sin_pitch;
            }
            if (height > b_hmax)
            {
                b_hmax = height;
            }
            if (roll_abs > b_roll)
            {
                b_roll = roll_abs;
            }
            if (one > b_one)
            {
                b_one = one;
            }
            if ((sin_pitch > BUMP_SIN) || (sin_pitch < -BUMP_SIN) ||
                (roll_abs > BUMP_SIN))
            {
                b_flat = dist_mm;
            }
            if (traced < TRACE_MAX)
            {
                tm_printf((UB *)"[S4T] %d mm  sin %d  side %d  one-wheel %d.%d "
                                " h %s%d.%d\n",
                          dist_mm, sin_pitch, roll, one / 10, one % 10,
                          (height < 0) ? "-" : "",
                          ((height < 0) ? -height : height) / 10,
                          ((height < 0) ? -height : height) % 10);
                traced++;
            }
            if ((dist_mm - b_flat) >= BUMP_FLAT_MM)
            {
                INT l10 = b_up * S4_L_MM / 100; /* 0.1 mm */
                INT c10 = b_hmax - b_h0;

                b_in_bump = FALSE;
                bumps++;
                last_end = dist_mm;
                tm_printf(
                    (UB *)"[S4] bump %d: %d-%d mm, nose-up peak %d, "
                          "nose-down peak %d, L x sin(up) %d.%d mm, climbed "
                          "%d.%d mm, %d readings skipped as jolts\n",
                    bumps, b_from, b_flat, b_up, b_down, l10 / 10, l10 % 10,
                    c10 / 10, c10 % 10, (INT)(imu_jolts() - jolts_bump));
                tm_printf((UB *)"[S4] bump %d one wheel: side tilt peak "
                                "sin %d/1000, height T x sin = %d.%d mm\n",
                          bumps, b_roll, b_one / 10, b_one % 10);
            }
        }
        if (imu_ramp_reports() != seen)
        {
            seen = imu_ramp_reports();
            test_print_x10("[S4] IMU task reported a ramp: ",
                           imu_ramp_peak_mm10(), " mm\n");
        }
        if ((!b_in_bump) && (last_end >= 0) &&
            ((dist_mm - last_end) >= BUMP_END_MM))
        {
            break; /* well past the last ramp */
        }
        tk_dly_tsk(10);
    }
    motion_halt();
    tm_printf(
        (UB *)"[S4-R01] %d bump(s) in %d mm, %d readings skipped as jolts "
              "in all\n",
        bumps, (INT)S2_COUNTS_TO_MM(motion_odometer() - odo0),
        (INT)(imu_jolts() - jolts_start));
}

EXPORT void subsystem4_stage_test (void)
{
    INT kdx;
    INT tick;
    TEST_INIT();

    tm_printf((UB *)"\n[DRIVE] === Stage 4: IMU ramp height ===\n");
    for (tick = 0; (tick < 30) && (!imu_ok()); tick++)
    {
        tk_dly_tsk(100);
    }
    TEST_CHECK("S4-I01", imu_ok(), "LSM303D found at 0x1D");
    if (!imu_ok())
    {
        TEST_SUMMARY("subsystem4 stage 4");
        return;
    }
    test_wait_start("Robot level and still: calibrating for 1 s.");
    imu_calibrate();
    while (!imu_calibrated())
    {
        tk_dly_tsk(10);
    }
    tm_printf(
        (UB *)"[S4] tilt view for 15 s: lift the front a little, then put "
              "the RIGHT wheel on a 2-3 cm block (x1000; one-wheel height in "
              "mm)\n");
    for (tick = 0; tick < 30; tick++)
    {
        INT one = imu_one_wheel_mm10();

        tm_printf(
            (UB *)"[S4] sin(pitch) %d  sin(side) %d  one-wheel %d.%d mm\n",
            imu_pitch_mdeg_sin(), imu_roll_sin_x1000(), one / 10, one % 10);
        tk_dly_tsk(500);
    }

    for (kdx = 1; kdx <= 5; kdx++)
    {
        UW reports_start = imu_ramp_reports();

        tm_printf((UB *)"\n[DRIVE] S4-R01 run %d of 5\n", kdx);
        test_wait_start(
            "Robot level about 20 cm before the hump, pointing "
            "straight ahead, the hump (triangle) in the path of the "
            "RIGHT wheel only, as on the course. It calibrates 1 s, "
            "then drives until 30 cm after the last bump (at most "
            "1.5 m). Write down the hump's real height.");
        imu_calibrate();
        while (!imu_calibrated())
        {
            tk_dly_tsk(10);
        }
        ramp_run(reports_start);
        tk_dly_tsk(500);
        if (imu_ramp_reports() != reports_start)
        {
            tm_printf((UB *)"[S4-R01] %d ramp(s) reported by the IMU task, the "
                            "last ",
                      (INT)(imu_ramp_reports() - reports_start));
            test_print_x10("", imu_ramp_peak_mm10(),
                           " mm (write down the measured ramp heights)\n");
        }
        else
        {
            tm_printf(
                (UB *)"[S4-R01] no ramp reported by the IMU task this run\n");
        }
        if (test_stop_pressed())
        {
            tm_printf((UB *)"[S4-R01] STOP pressed: runs ended\n");
            break;
        }
    }
    TEST_SUMMARY("subsystem4 stage 4");
}

#endif /* APP_TEST */

/*** end of file ***/

