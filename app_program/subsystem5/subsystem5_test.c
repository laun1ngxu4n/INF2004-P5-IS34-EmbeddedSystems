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

#endif /* APP_TEST */
