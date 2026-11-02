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
	UW	prev = 0xFFu, e1 = barcode_edges();

	tm_printf((UB*)"\n[S3] IR live view for 30 s: slide black tape under each "
		  "sensor. S C B = side (right of C), centre, barcode (1 = black)\n"
		  "[S3] Barcode sensor tune: hold it STILL over white, then STILL "
		  "over black; 'B edges/s' must read 0 in both (turn its pot)\n");
	for (t = 0; t < 3000; t++) {
		UW now = (app_gpio_get(PIN_IR_LEFT) << 2) | (app_gpio_get(PIN_IR_CENTER) << 1) |
			 app_gpio_get(PIN_IR_BARCODE);
		if (now != prev) {
			tm_printf((UB*)"[S3] S %d  C %d  B %d\n", (INT)((now >> 2) & 1u),
				  (INT)((now >> 1) & 1u), (INT)(now & 1u));
			prev = now;
		}
		if (t % 100 == 99) {
			/* the interrupt sees every flicker; the 10 ms view above does not */
			UW e2 = barcode_edges();
			tm_printf((UB*)"[S3] B edges/s %d (B now %d)\n", (INT)(e2 - e1),
				  (INT)(now & 1u));
			e1 = e2;
		}
		tk_dly_tsk(10);
	}
}

/* Follow the line for up to secs; print junctions, barcodes, line lost */
LOCAL INT follow_and_watch(INT secs, INT *junc, INT *lost, INT *codes_ok, INT *codes_fail)
{
	W		odo0 = motion_odometer();
	UW		e0 = barcode_edges();
	UW		j0 = line_junctions(), jn;
	BARCODE_RESULT	r;
	UW		seq;
	INT		t, last_state = LINE_ON, traced = 0;
	UW		lc_prev = 0xFFu;
	INT		dn, dw[40], ph = 0, k;
	UW		dseq = barcode_debug(&dn, dw, 40);

	barcode_last(&r);
	seq = r.seq;
	line_set_enabled(TRUE);
	tm_printf((UB*)"[S3] at start: S %d  C %d (want S 0, C 1: centre on the "
		  "line, side sensor just right of it on white)\n",
		  app_gpio_get(PIN_IR_LEFT) == IR_BLACK,
		  app_gpio_get(PIN_IR_CENTER) == IR_BLACK);
	if (motion_follow(S2_SPEED_BARCODE) < E_OK) return 0;
	for (t = 0; t < secs * 100 && !test_stop_pressed(); t++) {
		UW lc;

		tk_dly_tsk(10);
		/* Trace what the line sensors see: every change, first 150 */
		lc = ((app_gpio_get(PIN_IR_LEFT) == IR_BLACK) << 1) |
		     (app_gpio_get(PIN_IR_CENTER) == IR_BLACK);
		if (lc != lc_prev && traced < 150) {
			tm_printf((UB*)"[S3T] %d mm  S %d  C %d  steer %d\n",
				  (INT)S2_COUNTS_TO_MM(motion_odometer() - odo0),
				  (INT)(lc >> 1), (INT)(lc & 1u), line_steer_mm_s());
			lc_prev = lc;
			traced++;
		}
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
		/* What the barcode reader is doing */
		if (motion_scan_phase() != ph) {
			ph = motion_scan_phase();
			tm_printf((UB*)"[S3] %d mm: %s\n",
				  (INT)S2_COUNTS_TO_MM(motion_odometer() - odo0),
				  (ph == 1) ? "rescan: bar-space-bar seen, backing up" :
				  (ph == 2) ? "rescan: reading slowly" : "rescan: done, normal speed");
		}
		if (barcode_debug(&dn, dw, 40) != dseq) {
			dseq = barcode_debug(&dn, dw, 40);
			if (dn >= 2) {		/* 1 = a grout line or junction */
				tm_printf((UB*)"[S3] code ended, %d elements (mm):", dn);
				for (k = 0; k < dn && k < 40; k++) {
					tm_printf((UB*)" %d.%d", dw[k] / 10, dw[k] % 10);
				}
				tm_printf((UB*)"\n");
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
	tm_printf((UB*)"[S3] barcode sensor (GP2): %d edges this run "
		  "(0 = it never saw black)\n", (INT)(barcode_edges() - e0));
	return (INT)S2_COUNTS_TO_MM(motion_odometer() - odo0);	/* mm followed */
}

EXPORT void subsystem3_stage_test(void)
{
	INT	junc = 0, lost = 0, ok = 0, fail = 0, k, dist;
	TEST_INIT();

	tm_printf((UB*)"\n[DRIVE] === Stage 2: line, junctions, barcodes ===\n"
		  "[DRIVE] Speed %d mm/s. STOP (GP21) stops at any time.\n",
		  S2_SPEED_BARCODE);
	ir_live();

	tm_printf((UB*)"\n[DRIVE] --- Stage 2A: line + junctions ---\n");
	test_wait_start("S3-L01 / S3-L02: robot on the line. It follows for up to "
			"60 s (let it run; STOP only if it goes wrong); count the "
			"junctions you see.");
	dist = follow_and_watch(60, &junc, &lost, &ok, &fail);
	tm_printf((UB*)"[S3-L01] followed %d mm, line lost %d times; [S3-L02] "
		  "junctions counted %d (write down the real number)\n", dist, lost, junc);
	TEST_CHECK("S3-L01", lost == 0 && dist >= 3000,
		   "followed at least 3 m without LINE_LOST");

	tm_printf((UB*)"\n[DRIVE] --- Stage 2B: barcodes. Before each run write "
		  "down the codes on the track in the order the robot meets them; "
		  "compare with the printed ones ---\n");
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

	tm_printf((UB*)"\n[DRIVE] --- Stage 2C: S5-B01 / S5-B02 junction moves. "
		  "Mission takes over, obstacle watch and IMU OFF (untested "
		  "until Stages 3 and 4) ---\n");
	app_course_test(FALSE, FALSE);
}

#endif /* APP_TEST */
