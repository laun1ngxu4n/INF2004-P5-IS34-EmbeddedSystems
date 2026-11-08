/*
 *  subsystem1_test.c
 *  On-target tests for subsystem 1.  Built only with: make TEST=1
 *
 *  Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *  console output to testing/subsystem1/logs/ and record the result
 *  in testing/subsystem1/TEST_LOG.md.  Test IDs: S1-U01, S1-U02, ...
 */

#include "subsystem1.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem1_test (void)
{
    TEST_INIT();

    /* S1-U01 / S1-U02: Telemetry wakes every 100 ms without overruns */
    TEST_PERIODIC("S1-U01", "S1-U02", tmg_telemetry);

    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */

/*** end of file ***/

