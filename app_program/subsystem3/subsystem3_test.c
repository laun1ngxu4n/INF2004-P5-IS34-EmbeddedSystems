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
#include "../app_stage_test.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem5/subsystem5.h"

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

/* ---- Stage 2 hands-on tests ---------------------------------------- */

LOCAL void ir_live(void)
{
	INT	t;
	UW	prev = 0xFFu;

	tm_printf((UB*)"\n[S3] IR live view for 15 s: slide black tape under each "
		  "sensor. L C B = left, centre, barcode (1 = black)\n");
	for (t = 0; t < 1500; t++) {
		UW now = (app_gpio_get(PIN_IR_LEFT) << 2) | (app_gpio_get(PIN_IR_CENTER) << 1) |
			 app_gpio_get(PIN_IR_BARCODE);
		if (now != prev) {
			tm_printf((UB*)"[S3] L %d  C %d  B %d\n", (INT)((now >> 2) & 1u),
				  (INT)((now >> 1) & 1u), (INT)(now & 1u));
			prev = now;
		}
		tk_dly_tsk(10);
	}
}

/* Follow the line for up to secs; print junctions, barcodes, line lost */
LOCAL void follow_and_watch(INT secs, INT *junc, INT *lost, INT *codes_ok, INT *codes_fail)
{
	UW		j0 = line_junctions(), jn;
	BARCODE_RESULT	r;
	UW		seq;
	INT		t, last_state = LINE_ON;

	barcode_last(&r);
	seq = r.seq;
	line_set_enabled(TRUE);
	if (motion_follow(S2_SPEED_BARCODE) < E_OK) return;
	for (t = 0; t < secs * 100 && !test_stop_pressed(); t++) {
		tk_dly_tsk(10);
		jn = line_junctions();
		if (jn != j0 + (UW)*junc) {
			(*junc)++;
			tm_printf((UB*)"[S3] junction %d at %d mm\n", *junc,
				  (INT)S2_COUNTS_TO_MM(motion_odometer()));
		}
		if (line_state() != last_state) {
			last_state = line_state();
			if (last_state == LINE_LOST) {
				(*lost)++;
				tm_printf((UB*)"[S3] LINE LOST, stopping\n");
				break;
			}
		}
		barcode_last(&r);
		if (r.seq != seq) {
			seq = r.seq;
			if (r.ok) {
				(*codes_ok)++;
				tm_printf((UB*)"[S3] barcode %c%s, narrow %d.%d mm, wide %d.%d mm\n",
					  r.ch, r.reversed ? " (backwards)" : "",
					  r.narrow_mm10 / 10, r.narrow_mm10 % 10,
					  r.wide_mm10 / 10, r.wide_mm10 % 10);
			} else {
				(*codes_fail)++;
				tm_printf((UB*)"[S3] barcode READ_FAIL, %d elements\n", r.elements);
			}
		}
	}
	motion_halt();
}

EXPORT void subsystem3_stage_test(void)
{
	INT	junc = 0, lost = 0, ok = 0, fail = 0, k;
	TEST_INIT();

	tm_printf((UB*)"\n[DRIVE] === Stage 2: line, junctions, barcodes ===\n"
		  "[DRIVE] Speed %d mm/s. STOP (GP21) stops at any time.\n",
		  S2_SPEED_BARCODE);
	ir_live();

	test_wait_start("S3-L01 / S3-L02: robot on the line. It follows for up to "
			"60 s; count the junctions you see.");
	follow_and_watch(60, &junc, &lost, &ok, &fail);
	tm_printf((UB*)"[S3-L01] line lost %d times; [S3-L02] junctions counted %d "
		  "(write down the real number)\n", lost, junc);
	TEST_CHECK("S3-L01", lost == 0, "followed the line without LINE_LOST");

	for (k = 1; k <= 3; k++) {
		INT j = 0, l = 0;
		tm_printf((UB*)"\n[DRIVE] S3-B01..B03 barcodes, run %d of 3\n", k);
		test_wait_start("Put the robot on the line before a stretch with barcodes "
				"(both directions over the runs). Up to 40 s.");
		follow_and_watch(40, &j, &l, &ok, &fail);
	}
	tm_printf((UB*)"[S3-B01..B03] decoded %d, READ_FAIL %d (write down which "
		  "codes were on the track)\n", ok, fail);

	TEST_SUMMARY("subsystem3 stage 2");

	tm_printf((UB*)"\n[DRIVE] S5-B01 / S5-B02 junction moves: Mission takes over.\n");
	app_course_test();
}

#endif /* APP_TEST */
