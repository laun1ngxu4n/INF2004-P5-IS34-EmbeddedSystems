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
		  "sensor. L R B = line sensor left of the tape, right of the tape, "
		  "barcode (1 = black). Centred on the tape: L 0 and R 0\n");
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
		if (S3_BAR_ADC_ON > 0) {
			tm_printf((UB*)"[S3] B uses FIXED thresholds: black from %d, white "
				  "again below %d (S3_BAR_ADC_ON/OFF; 0 = learn)\n",
				  S3_BAR_ADC_ON, S3_BAR_ADC_OFF);
		}
	}
	for (t = 0; t < 3000; t++) {
		UW now = ((UW)(app_gpio_get(PIN_IR_LINE_L) == IR_BLACK) << 2) |
			 ((UW)(app_gpio_get(PIN_IR_LINE_R) == IR_BLACK) << 1) |
			 (barcode_black() ? 1u : 0u);
		if (now != prev) {
			tm_printf((UB*)"[S3] L %d  R %d  B %d\n", (INT)((now >> 2) & 1u),
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

/* Junctions seen in the last follow_and_watch(), by kind */
LOCAL INT	j_cross, j_left;

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
	INT		dn, dw[40], k;
	UW		dseq = barcode_debug(&dn, dw, 40);
	W		mo;
	INT		mw;
	UW		mseq = barcode_mark(&mo, &mw);

	barcode_last(&r);
	seq = r.seq;
	j_cross = j_left = 0;
	barcode_reset();		/* nothing left over from the live view */
	line_set_enabled(TRUE);
	tm_printf((UB*)"[S3] at start: L %d  R %d (want L 0, R 0: the tape centred "
		  "between them)\n",
		  app_gpio_get(PIN_IR_LINE_L) == IR_BLACK,
		  app_gpio_get(PIN_IR_LINE_R) == IR_BLACK);
	if (motion_follow(S2_SPEED_BARCODE) < E_OK) return 0;
	for (t = 0; t < secs * 100 && !test_stop_pressed(); t++) {
		UW lc;

		tk_dly_tsk(10);
		if (max_mm > 0 && S2_COUNTS_TO_MM(motion_odometer() - odo0) >= max_mm) {
			break;
		}
		/* Trace what the line sensors see: every change, first 150 */
		lc = ((UW)(app_gpio_get(PIN_IR_LINE_L) == IR_BLACK) << 1) |
		     (UW)(app_gpio_get(PIN_IR_LINE_R) == IR_BLACK);
		if (lc != lc_prev && traced < 150) {
			tm_printf((UB*)"[S3T] %d mm  L %d  R %d  steer %d\n",
				  (INT)S2_COUNTS_TO_MM(motion_odometer() - odo0),
				  (INT)(lc >> 1), (INT)(lc & 1u), line_steer_mm_s());
			lc_prev = lc;
			traced++;
		}
		jn = line_junctions();
		if (jn != j0 + (UW)*junc) {
			INT sd = line_junction_sides();

			(*junc)++;
			if ((sd & JUNC_SIDE_S) != 0) j_cross++;
			else j_left++;
			tm_printf((UB*)"[S3] junction %d at %d mm: %s\n", *junc,
				  (INT)S2_COUNTS_TO_MM(motion_odometer() - odo0),
				  ((sd & JUNC_SIDE_S) != 0) ?
				  "crossing (both line sensors black)" :
				  "branch left (seen by B)");
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
					  (line_fault() == LINE_FAULT_ONE) ?
					  "one sensor black for 30 cm (stuck black?)" :
					  "both black for 15 cm (stuck, or a wide black area)");
				break;
			}
		}
		if (barcode_debug(&dn, dw, 40) != dseq) {
			dseq = barcode_debug(&dn, dw, 40);
			if (dn >= 2) {		/* 1 = a grout line or junction */
				tm_printf((UB*)"[S3] code ended, %d elements (mm):", dn);
				for (k = 0; k < dn && k < 40; k++) {
					tm_printf((UB*)"%s%d.%d",
						  (k % 10 == 0 && k > 0) ? " | " : " ",
						  dw[k] / 10, dw[k] % 10);
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
				  "tape width: a left arm, if the robot drove straight there" :
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
	tm_printf((UB*)"[S3] junctions this run: crossings %d, left branches %d\n",
		  j_cross, j_left);
	{
		UW  nc, ns;
		INT mm = (INT)S2_COUNTS_TO_MM(motion_odometer() - odo0);

		line_wobble(&nc, &ns);
		tm_printf((UB*)"[S3] wobble this run: %d corrections (%d per metre), "
			  "%d sweeps right across the tape (a zigzag; want 0)\n",
			  (INT)nc, (mm > 0) ? (INT)nc * 1000 / mm : 0, (INT)ns);
	}
	return (INT)S2_COUNTS_TO_MM(motion_odometer() - odo0);	/* mm followed */
}

/* START = run this part, STOP = skip it.  TRUE = run it. */
LOCAL BOOL ask_run(const char *what)
{
	tm_printf((UB*)"\n[DRIVE] %s\n[DRIVE] START (GP20) = run it, STOP (GP21) = "
		  "skip it\n", what);
	while (1) {
		if (button_down(PIN_STOP)) {
			while (button_down(PIN_STOP)) tk_dly_tsk(10);
			tk_dly_tsk(50);
			app_stop_clear(STOP_BUTTON | STOP_MQTT);
			tm_printf((UB*)"[DRIVE] skipped\n");
			return FALSE;
		}
		if (button_down(PIN_START)) {
			tk_dly_tsk(30);			/* debounce */
			if (button_down(PIN_START) && !button_down(PIN_STOP)) {
				while (button_down(PIN_START)) tk_dly_tsk(10);
				return TRUE;
			}
		}
		tk_dly_tsk(10);
	}
}

/* Print the result of the last code, if a new one came: TRUE = decoded */
LOCAL BOOL print_new_code(UW *seq, UW *dseq)
{
	BARCODE_RESULT	r;
	INT		dn, dw[40], k;
	BOOL		ok = FALSE;

	if (barcode_debug(&dn, dw, 40) != *dseq) {
		*dseq = barcode_debug(&dn, dw, 40);
		tm_printf((UB*)"[S3] code ended, %d elements (mm):", dn);
		for (k = 0; k < dn && k < 40; k++) {
			tm_printf((UB*)"%s%d.%d", (k % 10 == 0 && k > 0) ? " | " : " ",
				  dw[k] / 10, dw[k] % 10);
		}
		tm_printf((UB*)"\n");
	}
	barcode_last(&r);
	if (r.seq != *seq) {
		*seq = r.seq;
		if (r.ok) {
			ok = TRUE;
			tm_printf((UB*)"[S3] barcode %c%s, narrow %d.%d mm, wide %d.%d mm\n",
				  r.ch, r.reversed ? " (backwards)" : "",
				  r.narrow_mm10 / 10, r.narrow_mm10 % 10,
				  r.wide_mm10 / 10, r.wide_mm10 % 10);
		} else {
			tm_printf((UB*)"[S3] barcode READ_FAIL, %d elements (a code "
				  "is 29)\n", r.elements);
		}
	}
	return ok;
}

/* S3-B00: B straight over one barcode, no line following, so only the
   sensor and the decoder are tested, not the steering.  With the analogue
   input it also keeps the raw reading every 10 ms (0.4 mm at 40 mm/s)
   and prints each bar and space with its darkest / lightest reading: a
   narrow bar that never reaches S3_BAR_ADC_ON, or a narrow space that
   never drops below S3_BAR_ADC_OFF, is marked '!' (the sensor's spot is
   wider than the bar: lower the module, or move the thresholds). */
#define BENCH_MM	300
#define BENCH_N		1000
LOCAL W		bn_odo[BENCH_N];
LOCAL UH	bn_v[BENCH_N];

LOCAL BOOL bar_bench_run(INT speed)
{
	MOTION_STATUS	st;
	BARCODE_RESULT	r;
	UW		seq, dseq;
	INT		n = 0, t, dn, dw[40], raw, wl, bl, i;
	BOOL		ana = FALSE, ok = FALSE;

	barcode_last(&r);
	seq = r.seq;
	dseq = barcode_debug(&dn, dw, 40);
	barcode_reset();
	if (motion_drive(speed, BENCH_MM) < E_OK) return FALSE;
	for (t = 0; t < 2000 && !test_stop_pressed(); t++) {
		tk_dly_tsk(10);
		ana = barcode_adc(&raw, &wl, &bl);
		if (ana && n < BENCH_N) {
			bn_odo[n] = motion_odometer();
			bn_v[n] = (UH)raw;
			n++;
		}
		if (print_new_code(&seq, &dseq)) ok = TRUE;
		motion_get_status(&st);
		if (t > 10 && st.result != MOT_RES_BUSY) break;
	}
	motion_halt();
	tk_dly_tsk(50);
	if (print_new_code(&seq, &dseq)) ok = TRUE;

	if (ana && n > 0) {
		/* split at the middle of the two thresholds (or of the learned
		   levels); skip the white before the first bar and after the
		   last */
		INT on = S3_BAR_ADC_ON, offv = S3_BAR_ADC_OFF, mid, cnt = 0;
		INT first = -1, last = -1, start, ext;
		BOOL blk;

		if (on <= 0) {
			on = (wl + bl) / 2 + (bl - wl) / 6;
			offv = (wl + bl) / 2 - (bl - wl) / 6;
		}
		mid = (on + offv) / 2;
#define IS_BLK(v)	(S3_BAR_BLACK_HIGH ? ((INT)(v) >= mid) : ((INT)(v) <= mid))
		for (i = 0; i < n; i++) {
			if (IS_BLK(bn_v[i])) {
				if (first < 0) first = i;
				last = i;
			}
		}
		if (first < 0) {
			tm_printf((UB*)"[S3] B profile: never darker than %d (max seen "
				  "over %d samples): B did not pass over a bar\n", mid, n);
		} else {
			tm_printf((UB*)"[S3] B profile, bar (B) and space (s): width mm / "
				  "darkest or lightest reading. A bar must reach %d, a "
				  "space drop below %d, '!' = it did not:\n[S3]  ", on, offv);
			start = first;
			blk = TRUE;
			ext = bn_v[first];
			for (i = first + 1; i <= last + 1; i++) {
				BOOL b = (i <= last) ? IS_BLK(bn_v[i]) : !blk;

				if (i <= last && b == blk) {
					INT v = bn_v[i];
					if (S3_BAR_BLACK_HIGH ? (blk ? v > ext : v < ext)
							      : (blk ? v < ext : v > ext)) {
						ext = v;
					}
					continue;
				}
				{
					W   w = bn_odo[(i <= last) ? i : last] - bn_odo[start];
					INT mm10 = (INT)((w < 0 ? -w : w) * S2_UM_PER_COUNT / 100);
					BOOL bad = blk ?
						(S3_BAR_BLACK_HIGH ? ext < on : ext > on) :
						(S3_BAR_BLACK_HIGH ? ext > offv : ext < offv);

					tm_printf((UB*)"%c%d.%d/%d%s ", blk ? 'B' : 's',
						  mm10 / 10, mm10 % 10, ext, bad ? "!" : "");
					if (++cnt % 10 == 0) tm_printf((UB*)"\n[S3]  ");
					if (cnt >= 80) break;
				}
				start = i;
				blk = b;
				ext = (i < n) ? bn_v[i] : 0;
			}
#undef IS_BLK
			tm_printf((UB*)"\n[S3] %d bars and spaces (a code is 29)\n", cnt);
		}
	}
	return ok;
}

LOCAL void bar_bench(void)
{
	static const struct { const char *what; INT speed; } run[] = {
		{"a barcode, B about 5 cm before its first bar", S2_SPEED_IN_BARCODE},
		{"the SAME barcode from the other end (robot turned round)",
		 S2_SPEED_IN_BARCODE},
		{"the thinnest barcode (\"A small\"), any direction",
		 S2_SPEED_IN_BARCODE},
		{"any barcode, at the FOLLOW speed (the robot slows only once B "
		 "is on the first bar, so the start is read at this speed)",
		 S2_SPEED_BARCODE},
	};
	INT i;
	TEST_INIT();

	for (i = 0; i < (INT)(sizeof(run) / sizeof(run[0])); i++) {
		BOOL ok;

		tm_printf((UB*)"\n[DRIVE] S3-B00 run %d of %d: %s. Square to the bars, "
			  "B over them, the line does not matter. It drives "
			  "straight %d mm at %d mm/s. Write down the code.\n",
			  i + 1, (INT)(sizeof(run) / sizeof(run[0])), run[i].what,
			  BENCH_MM, run[i].speed);
		test_wait_start("Robot placed (no such barcode: START, then STOP at once).");
		ok = bar_bench_run(run[i].speed);
		TEST_CHECK("S3-B00", ok, "decoded driving straight");
	}
	TEST_SUMMARY("subsystem3 S3-B00");
}

EXPORT void subsystem3_stage_test(void)
{
	INT	junc = 0, lost = 0, ok = 0, fail = 0, k, dist;
	TEST_INIT();

	tm_printf((UB*)"\n[DRIVE] === Stage 2: line, junctions, barcodes ===\n"
		  "[DRIVE] Speed %d mm/s. STOP (GP21) stops at any time.\n",
		  S2_SPEED_BARCODE);
	ir_live();

	if (ask_run("--- Stage 2A: line + junctions (S3-L01..L10) ---")) {
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
				INT max_mm, want_cross, want_left;
				BOOL want_fault;	/* TRUE: must stop with LINE_FAULT_ONE */
			} pl[] = {
				/* S3-L04 (right-only branch) dropped 7 Nov: the track has
				   none, and with the sensors straddling the tape it looks
				   like an ordinary correction */
				{"S3-L10", "about 20 cm before the X CROSSING (track 3), square "
				 "to it", "exactly 1 junction, a crossing", 450, 1, 0},
				{"S3-L03", "about 20 cm before a branch to the LEFT only (the "
				 "line goes on), no barcode next to it",
				 "exactly 1 junction, branch left", 450, 0, 1},
				{"S3-L05", "about 20 cm before a T end (the line stops at a cross "
				 "bar with arms both ways)", "exactly 1 junction, a crossing",
				 450, 1, 0},
				{"S3-L06", "at the start of the tightest RIGHT curve with no "
				 "branches", "no junction and no line lost", 1200, 0, 0},
				{"S3-L07", "at the start of the tightest LEFT curve with no "
				 "branches", "no junction and no line lost", 1200, 0, 0},
				{"S3-L08", "about 20 cm before a GAP in the line (track 2), "
				 "after a straight", "drives across, no line lost, no junction",
				 450, 0, 0},
				{"S3-L09", "on a straight. Once it moves, hold a black card "
				 "under R (the RIGHT line sensor) and keep it there (walk along)",
				 "stops with LINE SENSOR FAULT (one sensor black) within "
				 "about 30 cm", 1000, 0, 0, TRUE},
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
						   line_fault() == LINE_FAULT_ONE, pl[i].pass);
				} else {
					TEST_CHECK(pl[i].id, l == 0 && j_cross == pl[i].want_cross &&
						   j_left == pl[i].want_left, pl[i].pass);
				}
			}
		}
	}	/* 2A */

	if (ask_run("--- S3-B00: barcode read driving STRAIGHT, no line "
		    "following (4 runs). Do this before 2B ---")) {
		bar_bench();
	}

	if (ask_run("--- Stage 2B: barcodes while following the line (3 runs). "
		    "Before each run write down the codes on the track in the "
		    "order the robot meets them; compare with the printed ones ---")) {
		for (k = 1; k <= 3; k++) {
			INT j = 0, l = 0;
			tm_printf((UB*)"\n[DRIVE] S3-B01..B03 barcodes, run %d of 3\n", k);
			test_wait_start("Put the robot on the line before a stretch with barcodes "
					"(both directions over the runs). Up to 40 s.");
			follow_and_watch(40, 0, &j, &l, &ok, &fail);
		}
		tm_printf((UB*)"[S3-B01..B03] decoded %d, READ_FAIL %d (write down which "
			  "codes were on the track)\n", ok, fail);
	}	/* 2B */

	TEST_SUMMARY("subsystem3 stage 2");

	if (ask_run("--- Stage 2C: S5-B01 / S5-B02 junction moves. Mission takes "
		    "over, obstacle watch and IMU OFF (untested until Stages 3 "
		    "and 4) ---")) {
		app_course_test(FALSE, FALSE);
	}
}

#endif /* APP_TEST */
