/*
 *	app_stage_test.c
 *	Picks the hands-on stage test (TEST_STAGE) and holds the helpers the
 *	stage tests share.  Built only with: make TEST=1 TEST_STAGE=n
 */

#include "app.h"
#include "app_stage_test.h"
#include "subsystem1/subsystem1.h"
#include "subsystem3/subsystem3.h"
#include "subsystem5/subsystem5.h"

#if APP_TEST

#ifndef APP_TEST_STAGE
#define APP_TEST_STAGE	1
#endif

EXPORT void test_wait_start(const char *what)
{
	UW	remote = tele_start_count();

	tm_printf((UB*)"\n[DRIVE] %s\n[DRIVE] press START (GP20, or START on the "
		  "panel) to go\n", what);
	while (tele_start_count() == remote) {
		if (button_down(PIN_START)) {
			tk_dly_tsk(30);			/* debounce */
			if (button_down(PIN_START) && !button_down(PIN_STOP)) {
				while (button_down(PIN_START)) {
					tk_dly_tsk(10);
				}
				break;
			}
		}
		tk_dly_tsk(10);
	}
	app_stop_clear(STOP_BUTTON | STOP_MQTT);
	tm_printf((UB*)"[DRIVE] starting in 1 s, hands off\n");
	tk_dly_tsk(1000);
}

EXPORT BOOL test_stop_pressed(void)
{
	return app_stop_reasons() != 0u;
}

/* tm_printf has no %f: print x10 as "12.3" */
EXPORT void test_print_x10(const char *before, INT x10, const char *after)
{
	INT a = (x10 < 0) ? -x10 : x10;

	tm_printf((UB*)"%s%s%d.%d%s", before, (x10 < 0) ? "-" : "",
		  a / 10, a % 10, after);
}

EXPORT void test_use_test_mapping(void)
{
	static const UB	map[] = "A=L,B=R,C=F,D=U";
	UB		why[64];

	(void)barcode_set_mapping(map, sizeof(map) - 1, why, sizeof(why));
	tm_printf((UB*)"[DRIVE] TEST mapping A=L, B=R, C=F, D=U (test image only; "
		  "the demo mapping comes over MQTT)\n");
}

/* Mission in charge.  Stage 2 runs it without obstacle watch and IMU
   (not tested yet), Stage 3 adds obstacles, Stage 6 uses everything. */
EXPORT void app_course_test(BOOL obstacles, BOOL imu)
{
	UB map[24];

	mission_features(obstacles, imu);
	tm_printf((UB*)"\n[DRIVE] === Mission run: obstacle watch %s, IMU %s ===\n",
		  obstacles ? "ON" : "OFF", imu ? "ON" : "OFF");
#if defined(TM_MQTT) && TM_MQTT
	{
		/* 7 Nov: START was pressed again and again while this waited
		   for a mapping that was never sent; it looked like a crash.
		   Remind every 10 s; START with no mapping = the TEST mapping. */
		UW	remote = tele_start_count();
		INT	t = 0;

		tm_printf((UB*)"[DRIVE] send the mapping from the PC (panel: Barcode "
			  "mapping box), or press START to use the TEST mapping "
			  "A=L,B=R,C=F,D=U\n");
		while (barcode_mapping_text(map, sizeof(map)) == 0) {
			if (tele_start_count() != remote || button_down(PIN_START)) {
				while (button_down(PIN_START)) tk_dly_tsk(10);
				test_use_test_mapping();
				break;
			}
			if (++t % 100 == 0) {
				tm_printf((UB*)"[DRIVE] still waiting for a mapping (panel: Barcode "
					  "mapping box) or START for the TEST mapping\n");
			}
			tk_dly_tsk(100);
		}
		(void)barcode_mapping_text(map, sizeof(map));
		tm_printf((UB*)"[DRIVE] mapping %s in use\n", map);
	}
#else
	test_use_test_mapping();
	(void)map;
#endif
	tm_printf((UB*)"[DRIVE] Mission armed: press START to run, STOP to stop.\n"
		  "[DRIVE] Every result is printed as an [EVT] line.\n");
	mission_arm(TRUE);
}

EXPORT void app_stage_test(void)
{
	switch (APP_TEST_STAGE) {
	case 1:	subsystem2_drive_test();	break;
	case 2:	subsystem3_stage_test();	break;
	case 3:	subsystem5_stage_test();	break;
	case 4:	subsystem4_stage_test();	break;
	case 6:	app_course_test(TRUE, TRUE);	break;
	default:
		tm_printf((UB*)"[DRIVE] no hands-on test for TEST_STAGE=%d\n",
			  APP_TEST_STAGE);
		break;
	}
}

#endif /* APP_TEST */
