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
	INT	t, raw, wl, bl, lo = 4095, hi = 0;
	UW	prev = 0xFFu, e1 = barcode_edges();
	BOOL	ana = barcode_adc(&raw, &wl, &bl);

	tm_printf((UB*)"\n[S3] IR live view for 30 s: slide black tape under each "
		  "sensor. S C B = side (right of C), centre, barcode (1 = black)\n");
	if (!ana) {
		tm_printf((UB*)"[S3] B on its DIGITAL output (GP2). Tune: hold it STILL "
			  "over white, then STILL over black; 'B edges/s' must read 0 "
			  "in both (turn its pot)\n");
	} else {
		tm_printf((UB*)"[S3] B on its ANALOGUE output (GP28, module on 3V3): its "
			  "pot does nothing here. Pass B over white, over black tape (it "
			  "reads 0 until it has seen black once), and over a grout line or "
			  "tile join. Each second: the lowest and highest reading. At the "
			  "end: thresholds to copy into subsystem3.h. B reads 1 over "
			  "WHITE = this module's output is lower over black: set "
			  "S3_BAR_BLACK_HIGH 0\n");
	}
	for (t = 0; t < 3000; t++) {
		UW now = (app_gpio_get(PIN_IR_LEFT) << 2) | (app_gpio_get(PIN_IR_CENTER) << 1) |
			 (barcode_black() ? (UW)IR_BLACK : (UW)!IR_BLACK);
		if (now != prev) {
			tm_printf((UB*)"[S3] S %d  C %d  B %d\n", (INT)((now >> 2) & 1u),
				  (INT)((now >> 1) & 1u), (INT)(now & 1u));
			prev = now;
		}
		if (ana) {
			(void)barcode_adc(&raw, &wl, &bl);
			if (raw < lo) lo = raw;
			if (raw > hi) hi = raw;
		}
		if (t % 100 == 99) {
			/* the interrupt sees every flicker; the 10 ms view above does not */
			UW e2 = barcode_edges();

			tm_printf((UB*)"[S3] B edges/s %d (B now %d)\n", (INT)(e2 - e1),
				  (INT)(now & 1u));
			if (ana) {
				tm_printf((UB*)"[S3] B analogue now %d, this second %d..%d "
					  "(learned white %d, black %d; they must differ by "
					  "%d or more)\n", raw, lo, hi, wl, bl, S3_BAR_MIN_SPAN);
				lo = 4095;
				hi = 0;
			}
			e1 = e2;
		}
		tk_dly_tsk(10);
	}
	if (ana) {
		(void)barcode_adc(&raw, &wl, &bl);
		if (bl - wl >= S3_BAR_MIN_SPAN || wl - bl >= S3_BAR_MIN_SPAN) {
			/* black from 60 % of the way from white to black, white
			   again below 40 %: grout should stay under 40 % */
			tm_printf((UB*)"[S3] B thresholds from what it saw: "
				  "S3_BAR_ADC_ON %d, S3_BAR_ADC_OFF %d. Grout or a tile "
				  "join must stay on the white side of OFF (see the "
				  "readings above); if not, tell B3 before using them\n",
				  wl + (bl - wl) * 6 / 10, wl + (bl - wl) * 4 / 10);
		} else {
			tm_printf((UB*)"[S3] B has not seen enough contrast (white %d, "
				  "black %d): check AO is on GP28, the module is on 3V3, "
				  "and pass it over black tape\n", wl, bl);
		}
	}
}

/* Junctions seen in the last follow_and_watch(), by branch side */
LOCAL INT	j_right, j_left, j_both;

/* Follow the line for up to secs (and up to max_mm, 0 = no limit); print
   junctions, barcodes, line lost */
LOCAL INT follow_and_watch(INT secs, INT max_mm, INT *junc, INT *lost,
			   INT *codes_ok, INT *codes_fail)
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
	W		mo;
	INT		mw;
	UW		mseq = barcode_mark(&mo, &mw);

	barcode_last(&r);
	seq = r.seq;
	j_right = j_left = j_both = 0;
	line_set_enabled(TRUE);
	tm_printf((UB*)"[S3] at start: S %d  C %d (want S 0, C 1: centre on the "
		  "line, side sensor just right of it on white)\n",
		  app_gpio_get(PIN_IR_LEFT) == IR_BLACK,
		  app_gpio_get(PIN_IR_CENTER) == IR_BLACK);
	if (motion_follow(S2_SPEED_BARCODE) < E_OK) return 0;
	for (t = 0; t < secs * 100 && !test_stop_pressed(); t++) {
		UW lc;

		tk_dly_tsk(10);
		if (max_mm > 0 && S2_COUNTS_TO_MM(motion_odometer() - odo0) >= max_mm) {
			break;
		}
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
			INT sd = line_junction_sides();

			(*junc)++;
			if (sd == (JUNC_SIDE_S | JUNC_SIDE_B)) j_both++;
			else if (sd == JUNC_SIDE_B) j_left++;
			else j_right++;
			tm_printf((UB*)"[S3] junction %d at %d mm, branch %s\n", *junc,
				  (INT)S2_COUNTS_TO_MM(motion_odometer() - odo0),
				  (sd == (JUNC_SIDE_S | JUNC_SIDE_B)) ? "both sides (crossroads or T end)" :
				  (sd == JUNC_SIDE_B) ? "left (seen by B)" : "right (seen by S)");
		}
		if (line_state() != last_state) {
			last_state = line_state();
			if (last_state == LINE_LOST) {
				(*lost)++;
				tm_printf((UB*)"[S3] LINE LOST, stopping\n");
				break;
			}
			if (last_state == LINE_FAULT) {
				(*lost)++;
				tm_printf((UB*)"[S3] LINE SENSOR FAULT: %s, stopping\n",
					  (line_fault() == LINE_FAULT_S) ?
					  "S black too long (stuck black?)" :
					  "C did not change for 3 m (stuck?)");
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
		/* first bars under B: a left arm if about one tape wide */
		if (barcode_mark(&mo, &mw) != mseq) {
			mseq = barcode_mark(&mo, &mw);
			tm_printf((UB*)"[S3] %d mm: B patch %d.%d mm (%s)\n",
				  (INT)S2_COUNTS_TO_MM(mo - odo0), mw / 10, mw % 10,
				  (mw >= S3_JUNC_MIN_MM * 10 && mw <= S3_JUNC_MAX_MM * 10) ?
				  "tape width: a left arm, if C was on the line there" :
				  "not tape width: grout or a barcode bar, not a junction");
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
	line_set_enabled(FALSE);	/* the next run starts clean */
	tm_printf((UB*)"[S3] barcode sensor B: %d edges this run "
		  "(0 = it never saw black)\n", (INT)(barcode_edges() - e0));
	tm_printf((UB*)"[S3] junctions this run: right %d, left %d, both sides %d\n",
		  j_right, j_left, j_both);
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
	dist = follow_and_watch(60, 0, &junc, &lost, &ok, &fail);
	tm_printf((UB*)"[S3-L01] followed %d mm, line lost %d times; [S3-L02] "
		  "junctions counted %d (write down the real number)\n", dist, lost, junc);
	TEST_CHECK("S3-L01", lost == 0 && dist >= 3000,
		   "followed at least 3 m without LINE_LOST");

	/* One kind of place at a time: the robot drives at most max_mm, so
	   only that place is in the run.  No such place on the track: press
	   STOP straight after START (the check then fails: write "not on
	   the track" in the log). */
	{
		static const struct {
			const char *id, *where, *pass;
			INT max_mm, want_r, want_l, want_b;
			BOOL want_fault;	/* TRUE: must stop with LINE_FAULT_S */
		} pl[] = {
			{"S3-L03", "about 20 cm before a branch to the LEFT (the line "
			 "goes on), no barcode next to it", "exactly 1 junction, branch left",
			 450, 0, 1, 0},
			{"S3-L04", "about 20 cm before a branch to the RIGHT (the line "
			 "goes on)", "exactly 1 junction, branch right",
			 450, 1, 0, 0},
			{"S3-L05", "about 20 cm before a T end (the line stops at a cross "
			 "bar with arms both ways)", "exactly 1 junction, both sides",
			 450, 0, 0, 1},
			{"S3-L06", "at the start of the tightest RIGHT curve with no "
			 "branches", "no junction and no line lost", 1200, 0, 0, 0},
			{"S3-L07", "at the start of the tightest LEFT curve with no "
			 "branches", "no junction and no line lost", 1200, 0, 0, 0},
			{"S3-L08", "about 20 cm before a GAP in the line (track 2), "
			 "after a straight", "drives across, no line lost, no junction",
			 450, 0, 0, 0},
			{"S3-L09", "on a straight. Once it moves, hold a black card "
			 "under S (the RIGHT line sensor) and keep it there (walk along)",
			 "stops with LINE SENSOR FAULT (S black too long) within "
			 "about 20 cm", 1000, 0, 0, 0, TRUE},
		};
		INT i;

		tm_printf((UB*)"\n[DRIVE] --- Stage 2A: one place at a time ---\n");
		for (i = 0; i < (INT)(sizeof(pl) / sizeof(pl[0])); i++) {
			INT j = 0, l = 0, o = 0, f = 0;

			tm_printf((UB*)"\n[DRIVE] %s: robot on the line %s. It drives at "
				  "most %d mm. Pass: %s.\n", pl[i].id, pl[i].where,
				  pl[i].max_mm, pl[i].pass);
			test_wait_start("Robot placed as above (no such place: START, "
					"then STOP at once).");
			follow_and_watch(30, pl[i].max_mm, &j, &l, &o, &f);
			if (pl[i].want_fault) {
				TEST_CHECK(pl[i].id, line_state() == LINE_FAULT &&
					   line_fault() == LINE_FAULT_S, pl[i].pass);
			} else {
				TEST_CHECK(pl[i].id, l == 0 && j_right == pl[i].want_r &&
					   j_left == pl[i].want_l && j_both == pl[i].want_b,
					   pl[i].pass);
			}
		}
	}

	tm_printf((UB*)"\n[DRIVE] --- Stage 2B: barcodes. Before each run write "
		  "down the codes on the track in the order the robot meets them; "
		  "compare with the printed ones ---\n");
	for (k = 1; k <= 3; k++) {
		INT j = 0, l = 0;
		tm_printf((UB*)"\n[DRIVE] S3-B01..B03 barcodes, run %d of 3\n", k);
		test_wait_start("Put the robot on the line before a stretch with barcodes "
				"(both directions over the runs). Up to 40 s.");
		follow_and_watch(40, 0, &j, &l, &ok, &fail);
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
