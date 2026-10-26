/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem2/logs/ and record the result
 *	in testing/subsystem2/TEST_LOG.md.  Test IDs: S2-U01, S2-U02, ...
 */

#include "subsystem2.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem2_test(void)
{
	TEST_INIT();

	/* S2-U01 / S2-U02: Motion wakes every 10 ms without overruns */
	TEST_PERIODIC("S2-U01", "S2-U02", tmg_motion);

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
