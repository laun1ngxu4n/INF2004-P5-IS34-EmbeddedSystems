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
	tm_printf((UB*)"\n[DRIVE] %s\n[DRIVE] press START (GP20) to go\n", what);
	while (1) {
		while (!button_down(PIN_START)) {
			tk_dly_tsk(10);
		}
		tk_dly_tsk(30);				/* debounce */
		if (button_down(PIN_START) && !button_down(PIN_STOP)) {
			break;
		}
	}
	while (button_down(PIN_START)) {
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

/* Stage 6: the whole course, Mission in charge */
EXPORT void app_course_test(void)
{
	UB map[24];

	tm_printf((UB*)"\n[DRIVE] === Stage 6: whole course (SYS-01) ===\n");
#if defined(TM_MQTT) && TM_MQTT
	tm_printf((UB*)"[DRIVE] send the mapping from the PC: topic is34/cmd/map\n");
	while (barcode_mapping_text(map, sizeof(map)) == 0) {
		tk_dly_tsk(100);
	}
	tm_printf((UB*)"[DRIVE] mapping %s received\n", map);
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
	case 6:	app_course_test();		break;
	default:
		tm_printf((UB*)"[DRIVE] no hands-on test for TEST_STAGE=%d\n",
			  APP_TEST_STAGE);
		break;
	}
}

#endif /* APP_TEST */
