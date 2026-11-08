/*
 *  subsystem2_test.c
 *  On-target tests for subsystem 2.  Built only with: make TEST=1
 *
 *  Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *  console output to testing/subsystem2/logs/ and record the result
 *  in testing/subsystem2/TEST_LOG.md.  Test IDs: S2-U01, S2-U02, ...
 *
 *  subsystem2_drive_test() (Stage 1) DRIVES THE ROBOT.  It runs after the
 *  automatic tests and waits for the START button (GP20) before every
 *  move.  STOP (GP21) stops the motors at any time.
 */

#include "subsystem2.h"
#include "../test_report.h"
#include "../subsystem5/subsystem5.h" /* START / STOP buttons */
#include "../app_stage_test.h"

#if APP_TEST

EXPORT void subsystem2_test (void)
{
    TEST_INIT();

    /* S2-U01 / S2-U02: Motion wakes every 10 ms without overruns */
    TEST_PERIODIC("S2-U01", "S2-U02", g_tmg_motion);

    TEST_SUMMARY("subsystem2");
}

/* ---- Stage 1 drive tests ------------------------------------------- */

#define PI_F 3.14159f

LOCAL const char * res_name (INT res)
{
    switch (res)
    {
        case MOT_RES_DONE:
            return "DONE";
        case MOT_RES_STOPPED:
            return "STOPPED";
        case MOT_RES_TIMEOUT:
            return "TIMEOUT";
        case MOT_RES_BUSY:
            return "BUSY";
        default:
            return "NONE";
    }
}

/* Wait until command id is no longer BUSY (at most 15 s) */
LOCAL INT wait_done (ID h_cmd, motion_status_t * p_status)
{
    INT tick;

    for (tick = 0; tick < 1500; tick++)
    {
        motion_get_status(p_status);
        if ((p_status->cmd_id == (UW)h_cmd) &&
            (p_status->result != MOT_RES_BUSY))
        {
            return p_status->result;
        }
        tk_dly_tsk(10);
    }
    motion_halt();
    return MOT_RES_TIMEOUT;
}

/* S2-M02: three rounds of left 90, right 90, 180 */
LOCAL BOOL test_turns (void)
{
    static const INT    turns[3] = {90, -90, 180};
    static const char * names[3] = {"left 90", "right 90", "180 (left)"};
    motion_status_t     status;
    BOOL                b_all_done = TRUE;
    INT                 round;
    INT                 kdx;

    test_wait_start(
        "S2-M02 turns: robot on the floor, mark its heading "
        "(tape along one side). 3 rounds of: left 90, right 90, 180.");
    for (round = 1; round <= 3; round++)
    {
        for (kdx = 0; kdx < 3; kdx++)
        {
            W   left_start;
            W   right_start;
            UW  t_start = app_time_us();
            ID  h_cmd;
            INT res;
            INT deg10;

            motion_get_status(&status);
            left_start  = status.count_l;
            right_start = status.count_r;
            h_cmd       = motion_turn(turns[kdx]);
            if (h_cmd < E_OK)
            {
                tm_printf((UB *)"[DRIVE] STOP is set, turn not started\n");
                return FALSE;
            }
            res = wait_done(h_cmd, &status);
            tk_dly_tsk(300); /* let it settle, then count */
            motion_get_status(&status);
            /* spin angle from the wheels: (right - left) / track */
            deg10 =
                (INT)((float)((status.count_r - right_start) -
                              (status.count_l - left_start)) *
                      S2_UM_PER_COUNT / 1000.0f / S2_TRACK_MM * 1800.0f / PI_F);
            tm_printf((UB *)"[S2-M02] round %d %s: %s in %d ms, ", round,
                      names[kdx], res_name(res),
                      (INT)((app_time_us() - t_start) / 1000u) - 300);
            test_print_x10("encoder angle ", deg10, " deg\n");
            if (res != MOT_RES_DONE)
            {
                b_all_done = FALSE;
                if (MOT_RES_STOPPED == res)
                {
                    return FALSE;
                }
            }
            tk_dly_tsk(700);
        }
        tm_printf((UB *)"[S2-M02] round %d done: write down the real "
                        "angles (left 90, right 90, 180) from the tape\n",
                  round);
        if (round < 3)
        {
            test_wait_start(
                "S2-M02 next round: put the robot back on its mark");
        }
    }
    return b_all_done;
}

/* S2-M03: 500 mm at the barcode speed; speed over the middle 300 mm */
LOCAL BOOL test_slow_speed (void)
{
    motion_status_t status;
    W               left_start;
    W               right_start;
    INT             count = 0;
    INT             vmin  = 100000;
    INT             vmax  = -100000;
    W               vsum  = 0;
    ID              h_cmd;
    INT             res;
    INT             mean = 0;
    INT             diff_mm;

    UW t_start;

    test_wait_start(
        "S2-M03 slow speed: 0.5 m clear in front of the robot. "
        "Mark where the robot's front is now: it drives 500 mm at the "
        "barcode speed; then MEASURE how far it really went.");
    motion_get_status(&status);
    left_start  = status.count_l;
    right_start = status.count_r;
    t_start     = app_time_us();
    h_cmd       = motion_drive(S2_SPEED_BARCODE, 500);
    if (h_cmd < E_OK)
    {
        return FALSE;
    }
    do
    {
        INT dist_mm;
        INT value;

        tk_dly_tsk(20);
        motion_get_status(&status);
        dist_mm = (INT)(((status.count_l - left_start) +
                         (status.count_r - right_start)) /
                        2 * S2_UM_PER_COUNT / 1000);
        if ((dist_mm >= 100) && (dist_mm <= 400))
        {
            value = (status.speed_l + status.speed_r) / 2;
            vsum += value;
            count++;
            if (value < vmin)
            {
                vmin = value;
            }
            if (value > vmax)
            {
                vmax = value;
            }
        }
    } while (
        !((status.cmd_id == (UW)h_cmd) && (status.result != MOT_RES_BUSY)));
    res = status.result;
    if (count > 0)
    {
        mean = (INT)(vsum / count);
    }
    diff_mm =
        (INT)(((status.count_l - left_start) - (status.count_r - right_start)) *
              S2_UM_PER_COUNT / 1000);
    tm_printf((UB *)"[S2-M03] %s; target %d mm/s; middle 300 mm: mean %d, "
                    "min %d, max %d mm/s (%d samples); left - right %d mm\n",
              res_name(res), S2_SPEED_BARCODE, mean, vmin, vmax, count,
              diff_mm);
    tm_printf(
        (UB *)"[S2-M03] encoders say %d mm in %d ms. WRITE DOWN the real "
              "distance: more than 25 mm off 500 = wrong mm/count scale\n",
        (INT)(((status.count_l - left_start) + (status.count_r - right_start)) /
              2 * S2_UM_PER_COUNT / 1000),
        (INT)((app_time_us() - t_start) / 1000u));
    return (MOT_RES_DONE == res) && (count > 0) &&
           (mean >= (S2_SPEED_BARCODE * 9 / 10)) &&
           (mean <= (S2_SPEED_BARCODE * 11 / 10));
}

/* S2-M04: drive, press STOP; time from press to wheels stopped */
LOCAL BOOL test_stop_once (INT run, BOOL b_lifted)
{
    motion_status_t status;
    ID              h_cmd;
    W               last_l;
    W               last_r;
    W               c0_l        = 0;
    W               c0_r        = 0;
    UW              t_press     = 0;
    UW              t_last_move = 0;
    BOOL            b_pressed   = FALSE;
    INT             res;

    if (b_lifted)
    {
        tm_printf(
            (UB *)"\n[DRIVE] S2-M06 STOP check, wheels LIFTED: the wheels "
                  "turn forward; PRESS STOP (GP21) after a second.\n");
    }
    else
    {
        tm_printf(
            (UB *)"\n[DRIVE] S2-M04 try %d of 5: it drives up to 1.5 m at "
                  "%d mm/s. PRESS STOP (GP21) while it drives.\n",
            run, S2_SPEED_LINE);
    }
    test_wait_start(b_lifted ? "S2-M06 STOP (wheels lifted)" : "S2-M04 STOP");
    h_cmd = motion_drive(S2_SPEED_LINE, 1500);
    if (h_cmd < E_OK)
    {
        return FALSE;
    }
    motion_get_status(&status);
    last_l = status.count_l;
    last_r = status.count_r;
    for (;;)
    {
        tk_dly_tsk(10);
        motion_get_status(&status);
        if ((!b_pressed) && (app_stop_reasons() != 0u))
        {
            b_pressed   = TRUE;
            t_press     = app_stop_time_us();
            c0_l        = status.count_l;
            c0_r        = status.count_r;
            t_last_move = app_time_us();
        }
        if ((status.count_l != last_l) || (status.count_r != last_r))
        {
            last_l      = status.count_l;
            last_r      = status.count_r;
            t_last_move = app_time_us();
        }
        if (b_pressed && ((app_time_us() - t_last_move) > 100000u))
        {
            break; /* no count for 100 ms: stopped */
        }
        if ((!b_pressed) && (status.cmd_id == (UW)h_cmd) &&
            (status.result != MOT_RES_BUSY))
        {
            break; /* finished without a STOP */
        }
    }
    res = status.result;
    if (!b_pressed)
    {
        tm_printf((UB *)"[%s] try %d: no STOP pressed (%s), not counted\n",
                  b_lifted ? "S2-M06" : "S2-M04", run, res_name(res));
        return FALSE;
    }
    tm_printf((UB *)"[%s] try %d: %s; wheels stopped %d ms after the "
                    "press; rolled %d mm after the press\n",
              b_lifted ? "S2-M06" : "S2-M04", run, res_name(res),
              (INT)((t_last_move - t_press) / 1000u),
              (INT)(((last_l - c0_l) + (last_r - c0_r)) / 2 * S2_UM_PER_COUNT /
                    1000));
    return (MOT_RES_STOPPED == res) && ((t_last_move - t_press) < 500000u);
}

/* S2-M05: wheels lifted, both wheels forward then back; check both signs */
LOCAL BOOL test_directions (void)
{
    static const INT dir[2] = {+1, -1};
    motion_status_t  status;
    BOOL             b_ok = TRUE;
    INT              kdx;

    test_wait_start("S2-M05 directions: LIFT THE WHEELS off the table. Both "
                    "wheels turn forward 300 mm, then backward 300 mm. Watch "
                    "that both turn the robot's forward way first.");
    for (kdx = 0; kdx < 2; kdx++)
    {
        W   left_start;
        W   right_start;
        W   delta_left;
        W   delta_right;
        INT res;

        motion_get_status(&status);
        left_start  = status.count_l;
        right_start = status.count_r;
        res =
            wait_done(motion_drive(dir[kdx] * S2_SPEED_BARCODE, 300), &status);
        tk_dly_tsk(300);
        motion_get_status(&status);
        delta_left  = status.count_l - left_start;
        delta_right = status.count_r - right_start;
        tm_printf((UB *)"[S2-M05] %s: %s, left %d counts, right %d counts "
                        "(expected about %s3750 each)\n",
                  (0 == kdx) ? "forward" : "backward", res_name(res),
                  (INT)delta_left, (INT)delta_right, (0 == kdx) ? "+" : "-");
        /* both the commanded sign, and within 25 % of each other */
        if (((delta_left * dir[kdx]) <= 0) || ((delta_right * dir[kdx]) <= 0) ||
            ((delta_left * 4 * dir[kdx]) < (delta_right * 3 * dir[kdx])) ||
            ((delta_right * 4 * dir[kdx]) < (delta_left * 3 * dir[kdx])))
        {
            b_ok = FALSE;
        }
        if (MOT_RES_STOPPED == res)
        {
            return FALSE;
        }
        tk_dly_tsk(700);
    }
    return b_ok;
}

EXPORT void subsystem2_drive_test (void)
{
    motion_status_t status;
    INT             kdx;
    INT             stops = 0;
    TEST_INIT();

    tm_printf(
        (UB *)"\n[DRIVE] === Stage 1 drive tests ===\n"
              "[DRIVE] Laptop USB only (never USB and the power bank "
              "together).\n"
              "[DRIVE] Wheels LIFTED for S2-M05 and S2-M06, then on the floor\n"
              "[DRIVE] with about 1.5 m clear in front.\n"
              "[DRIVE] STOP (GP21) stops the motors at any time.\n"
              "[DRIVE] Track width used for turns: %d mm (S2_TRACK_MM)\n",
        S2_TRACK_MM);

    /* Order (review 1 Nov): directions and STOP with the wheels lifted,
       then speed/distance on the floor, then turns, then the STOP tries. */
    TEST_CHECK("S2-M05", test_directions(),
               "wheels lifted: both encoders count the commanded way, "
               "within 25 % of each other");
    TEST_CHECK("S2-M06", test_stop_once(0, TRUE),
               "wheels lifted: STOP stopped the wheels within 500 ms");
    tm_printf((UB *)"\n[DRIVE] From here: robot ON THE FLOOR.\n");
    TEST_CHECK("S2-M03", test_slow_speed(),
               "barcode speed: mean within 10 % (encoders); "
               "ALSO record the measured real distance");
    TEST_CHECK("S2-M02", test_turns(),
               "all 9 turn COMMANDS finished. This is not the angle check: "
               "pass/fail is your measured angles (90 +/- 10, 180 +/- 15)");
    for (kdx = 1; kdx <= 5; kdx++)
    {
        if (test_stop_once(kdx, FALSE))
        {
            stops++;
        }
    }
    tm_printf((UB *)"[S2-M04] %d of 5 stopped within 500 ms\n", stops);
    TEST_CHECK("S2-M04", 5 == stops, "STOP stops the wheels, 5 of 5");

    motion_get_status(&status);
    tm_printf((UB *)"[DRIVE] encoder quadrature errors since boot: %d\n",
              (INT)status.invalid);
    TEST_SUMMARY("subsystem2 drive");
    tm_printf((UB *)"[DRIVE] === Stage 1 drive tests end ===\n");
}

#endif /* APP_TEST */

/*** end of file ***/

