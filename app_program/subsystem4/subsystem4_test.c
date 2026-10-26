/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4.  Built only with: make TEST=1
 *
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem4/logs/ and record the result
 *	in testing/subsystem4/TEST_LOG.md.  Test IDs: S4-U01, S4-U02, ...
 */

#include "subsystem4.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem4_test(void)
{
	TEST_INIT();

	/* S4-U01 / S4-U02: Safety wakes every 20 ms without overruns */
	TEST_PERIODIC("S4-U01", "S4-U02", tmg_safety);

	/* S4-U03 / S4-U04: IMU wakes every 10 ms without overruns */
	TEST_PERIODIC("S4-U03", "S4-U04", tmg_imu);

	TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
