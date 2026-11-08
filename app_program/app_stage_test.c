/**
 * @file    app_stage_test.c
 *  Picks the hands-on stage test (TEST_STAGE) and holds the helpers the
 *  stage tests share.  Built only with: make TEST=1 TEST_STAGE=n
 */

#include "app.h"
#include "app_stage_test.h"
#include "subsystem1/subsystem1.h"
#include "subsystem3/subsystem3.h"
#include "subsystem5/subsystem5.h"

#if APP_TEST

#ifndef APP_TEST_STAGE
#define APP_TEST_STAGE 1
#endif

/** Prints what to do, waits for START (button or panel), then waits 1 s. */
EXPORT void test_wait_start (const char * p_what)
{
    uint32_t remote = tele_start_count();

    tm_printf(
        app_str("\n[DRIVE] %s\n[DRIVE] press START (GP20, or START on the "
                "panel) to go\n"),
        p_what);

    while (tele_start_count() == remote)
    {
        if (button_is_down(PIN_START))
        {
            tk_dly_tsk(30); /* debounce */

            if ((button_is_down(PIN_START)) && (!button_is_down(PIN_STOP)))
            {
                while (button_is_down(PIN_START))
                {
                    tk_dly_tsk(10);
                }
                break;
            }
        }

        tk_dly_tsk(10);
    }

    app_stop_clear(STOP_BUTTON | STOP_MQTT);
    tm_printf(app_str("[DRIVE] starting in 1 s, hands off\n"));
    tk_dly_tsk(1000);
}

/** true while a STOP reason is set. */
EXPORT bool test_stop_pressed (void)
{
    return app_stop_reasons() != 0u;
}

/** tm_printf has no %f: print x10 as "12.3" */
EXPORT void test_print_x10 (const char * p_before, int32_t x10,
                            const char * p_after)
{
    int32_t magnitude = (x10 < 0) ? -x10 : x10;

    tm_printf(app_str("%s%s%d.%d%s"), p_before, (x10 < 0) ? "-" : "",
              magnitude / 10, magnitude % 10, p_after);
}

/** Loads the test-only barcode mapping A=L, B=R, C=F, D=U. */
EXPORT void test_use_test_mapping (void)
{
    static const uint8_t map[] = "A=L,B=R,C=F,D=U";
    uint8_t              why[64];

    (void)barcode_set_mapping(map, sizeof(map) - 1, why, sizeof(why));
    tm_printf(
        app_str("[DRIVE] TEST mapping A=L, B=R, C=F, D=U (test image only; "
                "the demo mapping comes over MQTT)\n"));
}

/** Mission in charge.  Stage 2 runs it without obstacle watch and IMU
   (not tested yet), Stage 3 adds obstacles, Stage 6 uses everything. */
EXPORT void app_course_test (bool b_obstacles, bool b_imu)
{
    uint8_t map[24];

    mission_features(b_obstacles, b_imu);
    tm_printf(
        app_str("\n[DRIVE] === Mission run: obstacle watch %s, IMU %s ===\n"),
        b_obstacles ? "ON" : "OFF", b_imu ? "ON" : "OFF");
#if defined(TM_MQTT) && TM_MQTT
    {
        /* 7 Nov: START was pressed again and again while this waited
           for a mapping that was never sent; it looked like a crash.
           Remind every 10 s; START with no mapping = the TEST mapping. */
        uint32_t remote = tele_start_count();
        int32_t  tick   = 0;

        tm_printf(
            app_str("[DRIVE] send the mapping from the PC (panel: Barcode "
                    "mapping box), or press START to use the TEST mapping "
                    "A=L,B=R,C=F,D=U\n"));

        while (0 == barcode_mapping_text(map, sizeof(map)))
        {
            if ((tele_start_count() != remote) || (button_is_down(PIN_START)))
            {
                while (button_is_down(PIN_START))
                {
                    tk_dly_tsk(10);
                }

                test_use_test_mapping();
                break;
            }

            if (0 == (++tick % 100))
            {
                tm_printf(
                    app_str("[DRIVE] still waiting for a mapping (panel: "
                            "Barcode "
                            "mapping box) or START for the TEST mapping\n"));
            }

            tk_dly_tsk(100);
        }

        (void)barcode_mapping_text(map, sizeof(map));
        tm_printf(app_str("[DRIVE] mapping %s in use\n"), map);
    }
#else
    test_use_test_mapping();
    (void)map;
#endif
    tm_printf(
        app_str("[DRIVE] Mission armed: press START to run, STOP to stop.\n"
                "[DRIVE] Every result is printed as an [EVT] line.\n"));
    mission_arm(true);
}

/** Runs the hands-on test chosen at build time (TEST_STAGE). */
EXPORT void app_stage_test (void)
{
    switch (APP_TEST_STAGE)
    {
        case 1:
            subsystem2_drive_test();
        break;
        case 2:
            subsystem3_stage_test();
        break;
        case 3:
            subsystem5_stage_test();
        break;
        case 4:
            subsystem4_stage_test();
        break;
        case 6:
            app_course_test(true, true);
        break;
        default:
            tm_printf(app_str("[DRIVE] no hands-on test for TEST_STAGE=%d\n"),
                      APP_TEST_STAGE);
        break;
    }
}

#endif /* APP_TEST */

/*** end of file ***/

