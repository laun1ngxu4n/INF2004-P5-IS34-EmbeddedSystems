/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem3/logs/ and record the result
 *	in testing/subsystem3/TEST_LOG.md.  Test IDs: S3-U01, S3-U02, ...
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem3_test(void)
{
	TEST_INIT();

	/* S3-U01 / S3-U02: Line Following wakes every 20 ms without overruns */
	TEST_PERIODIC("S3-U01", "S3-U02", tmg_line);

	/* S3-U03 / S3-U04: Barcode Decoder's 10 ms timeout keeps it running */
	TEST_TIMEOUT("S3-U03", "S3-U04", tmg_barcode);

	TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
