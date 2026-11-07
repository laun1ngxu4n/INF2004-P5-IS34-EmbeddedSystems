/*
 *	task_barcode.c
 *	Subsystem 3 -- the Barcode Decoder task (wakes every 10 ms).
 *
 *	The GP2 interrupt stores every edge with the odometer reading (encoder
 *	counts), so element widths are distances (TEAM.md 7.1 B3).  When the
 *	robot has moved S3_QUIET_MM past the last edge, the edges since the
 *	first black bar are one barcode:
 *	  - n elements must be 10k - 1 (k characters of 9 elements, a narrow
 *	    gap between characters), k >= 3 ("*X*");
 *	  - in each character the 3 widest of 9 are wide (Code 39 rule);
 *	  - the first character must be '*'.  If it is not, the elements are
 *	    reversed and decoded again: read backwards (B2);
 *	  - a stray mark within S3_QUIET_MM before or after the code (a tile
 *	    join, grout) adds a bar and a space: the "*X*" is looked for at
 *	    every bar in the run, not only at the first.
 *	Result: barcode_last(), and MSN_FLG_BARCODE to Mission.
 */

#include "subsystem3.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem5/subsystem5.h"

#define RING		128		/* edges kept, a power of 2 */
#define MAX_ELEM	99		/* up to 10 characters */
#define S3_QUIET_MM	40		/* no edge for this far = end of barcode */
#define MERGE_COUNTS	15		/* < 1.2 mm apart = sensor chatter.  The
					   sensor flickers at its threshold
					   (thousands of edges, Stage 2 2 Nov);
					   must stay under half the narrowest
					   bar: 3.5 mm small codes = 44 counts */
#define MIN_ELEM_REPORT	20		/* fewer elements = noise, not reported */

typedef struct { W odo; UB level; } EDGE;

LOCAL EDGE		ring[RING];
LOCAL volatile UW	ring_w;		/* written by the interrupt */
LOCAL UW		ring_r;		/* read by the task */

LOCAL volatile BOOL	busy;		/* inside a barcode (in_code) */
LOCAL volatile INT	n_now;		/* clean elements so far in this one */
LOCAL volatile BOOL	paused;		/* Motion is backing up: ignore edges */
LOCAL volatile UW	seq_now;	/* results published so far */
LOCAL volatile W	edge_pub;	/* odometer at the last clean edge */
LOCAL volatile UW	cand_ends;	/* runs of >= S3_SCAN_MIN_ELEM that ended */

#define S3_SCAN_MIN_ELEM	3	/* bar, space, bar: looks like a barcode
					   start (a grout line or junction is 1) */
#define S3_IN_BARS_MM		15	/* edge this recent = B over bars */

LOCAL BARCODE_RESULT	last;
LOCAL UB		map_ch[4];
LOCAL UB		map_mv[4];
LOCAL BOOL		map_set;

/* Code 39: 9 elements bar/space/bar..., 1 = wide.  Bit 8 = first bar. */
typedef struct { UB ch; UH pat; } C39;
LOCAL const C39 c39[] = {
	{'0',0x034},{'1',0x121},{'2',0x061},{'3',0x160},{'4',0x031},
	{'5',0x130},{'6',0x070},{'7',0x025},{'8',0x124},{'9',0x064},
	{'A',0x109},{'B',0x049},{'C',0x148},{'D',0x019},{'E',0x118},
	{'F',0x058},{'G',0x00D},{'H',0x10C},{'I',0x04C},{'J',0x01C},
	{'K',0x103},{'L',0x043},{'M',0x142},{'N',0x013},{'O',0x112},
	{'P',0x052},{'Q',0x007},{'R',0x106},{'S',0x046},{'T',0x016},
	{'U',0x181},{'V',0x0C1},{'W',0x1C0},{'X',0x091},{'Y',0x190},
	{'Z',0x0D0},{'-',0x085},{'.',0x184},{' ',0x0C4},{'*',0x094},
	{'$',0x0A8},{'/',0x0A2},{'+',0x08A},{'%',0x02A}
};

/* GP2 edge: interrupt context */
LOCAL void bar_edge(UINT pin, UW events)
{
	UW	w = ring_w;

	(void)events;
	ring[w & (RING - 1)].odo = motion_odometer();
	ring[w & (RING - 1)].level = (UB)app_gpio_get(pin);
	ring_w = w + 1;
}

#if !S3_BARCODE_ANALOG

EXPORT ER barcode_hw_init(void)
{
	ER er = app_gpio_input(PIN_IR_BARCODE, FALSE);

	return (er < E_OK) ? er : app_gpio_irq_add(PIN_IR_BARCODE, APP_GPIO_BOTH, bar_edge);
}
EXPORT BOOL barcode_black(void)	{ return app_gpio_get(PIN_IR_BARCODE) == IR_BLACK; }
EXPORT BOOL barcode_adc(INT *raw, INT *white, INT *black)
{
	*raw = *white = *black = 0;
	return FALSE;
}

#else	/* S3_BARCODE_ANALOG */

/* AO on GP28, sampled by the ADC at 2 kHz (0.075 mm apart at 150 mm/s).
   The interrupt turns the samples into black/white with hysteresis and
   stores each change exactly like a GP2 edge, so the decoder below is the
   same for both inputs.  The kernel's sample ADC driver ("adca") is not
   used by the application; its interrupt handler is replaced here. */
#define ADC_REG(o)	(0x4004C000u + (o))
#define ADC_CS		ADC_REG(0x00)
#define ADC_FCS		ADC_REG(0x08)
#define ADC_FIFO	ADC_REG(0x0C)
#define ADC_DIV		ADC_REG(0x10)
#define ADC_INTE	ADC_REG(0x18)
#define CS_EN		(1u << 0)
#define CS_START_MANY	(1u << 3)
#define CS_READY	(1u << 8)
#define FCS_EN		(1u << 0)
#define FCS_EMPTY	(1u << 8)
#define FCS_THRESH(n)	((UW)(n) << 24)
#define INTNO_ADC_FIFO	22
#define INTPRI_ADC	2		/* below the GPIO edges (1) */
#define ADC_RATE_HZ	2000
#define ENV_SH		4		/* levels kept x16 */
#define ENV_UP_SH	3		/* follow a new extreme in ~8 samples */
#define ENV_DECAY_SH	20		/* forget it over ~9 min.  B is beside the
					   line and sees black only at barcodes and
					   branches: forgetting in 16 s (shift 15)
					   lost the black level within a minute of
					   driving, then a darker tile join could be
					   learned as "black". */

LOCAL volatile INT	adc_raw;
LOCAL INT		env_w = -1, env_b = -1;	/* white / black level x16 */
LOCAL volatile UB	adc_lvl = !IR_BLACK;

/* bigger = blacker, whichever way the module is wired */
#define BLACKNESS(v)	(S3_BAR_BLACK_HIGH ? (v) : 4095 - (v))

LOCAL void adc_sample(INT v)
{
	INT	b = BLACKNESS(v) << ENV_SH;
	UB	lvl = adc_lvl;

	adc_raw = v;
	if (env_w < 0) env_w = env_b = b;
	/* black level: rises fast to a new darkest, sinks very slowly */
	if (b > env_b) env_b += (b - env_b) >> ENV_UP_SH;
	else env_b -= (env_b - b) >> ENV_DECAY_SH;
	/* white level: the same the other way */
	if (b < env_w) env_w -= (env_w - b) >> ENV_UP_SH;
	else env_w += (b - env_w) >> ENV_DECAY_SH;

	if (S3_BAR_ADC_ON > 0) {
		INT x = BLACKNESS(v);
		if (lvl != IR_BLACK && x >= BLACKNESS(S3_BAR_ADC_ON)) lvl = IR_BLACK;
		else if (lvl == IR_BLACK && x <= BLACKNESS(S3_BAR_ADC_OFF)) lvl = !IR_BLACK;
	} else {
		INT span = (env_b - env_w) >> ENV_SH;
		INT mid = (env_b + env_w) >> (ENV_SH + 1);
		INT hyst = span / 6;
		INT x = BLACKNESS(v);

		if (span < S3_BAR_MIN_SPAN) lvl = !IR_BLACK;	/* never seen black */
		else if (lvl != IR_BLACK && x > mid + hyst) lvl = IR_BLACK;
		else if (lvl == IR_BLACK && x < mid - hyst) lvl = !IR_BLACK;
	}
	if (lvl != adc_lvl) {
		UW w = ring_w;

		adc_lvl = lvl;
		if (!paused) {
			ring[w & (RING - 1)].odo = motion_odometer();
			ring[w & (RING - 1)].level = lvl;
			ring_w = w + 1;
		}
	}
}

LOCAL void adc_inthdr(UINT intno)
{
	while ((in_w(ADC_FCS) & FCS_EMPTY) == 0u) {
		adc_sample((INT)(in_w(ADC_FIFO) & 0x0FFFu));
	}
	ClearInt(intno);
}

EXPORT ER barcode_hw_init(void)
{
	T_DINT	dint;
	ER	er;

	app_hw_release(RESETS_RESET_ADC);
	/* GP28 as an analogue pin: no digital input, no pulls */
	out_w(GPIO_CTRL(PIN_IR_BARCODE_AO), 31);		/* FUNCSEL null */
	clr_w(GPIO(PIN_IR_BARCODE_AO), GPIO_IE | GPIO_PUE | GPIO_PDE);
	set_w(GPIO(PIN_IR_BARCODE_AO), GPIO_OD);

	out_w(ADC_CS, ((UW)S3_BAR_ADC_CH << 12) | CS_EN);
	while ((in_w(ADC_CS) & CS_READY) == 0u) {
		;
	}
	out_w(ADC_DIV, (UW)(48000000 / ADC_RATE_HZ - 1) << 8);	/* clk_adc 48 MHz */
	out_w(ADC_FCS, FCS_EN | FCS_THRESH(1));
	while ((in_w(ADC_FCS) & FCS_EMPTY) == 0u) (void)in_w(ADC_FIFO);

	dint.intatr = TA_HLNG;
	dint.inthdr = (FP)adc_inthdr;
	er = tk_def_int(INTNO_ADC_FIFO, &dint);
	if (er < E_OK) return er;
	ClearInt(INTNO_ADC_FIFO);
	out_w(ADC_INTE, 1u);
	EnableInt(INTNO_ADC_FIFO, INTPRI_ADC);
	set_w(ADC_CS, CS_START_MANY);
	return E_OK;
}
EXPORT BOOL barcode_black(void)	{ return adc_lvl == IR_BLACK; }
EXPORT BOOL barcode_adc(INT *raw, INT *white, INT *black)
{
	INT w = env_w, b = env_b;

	*raw = adc_raw;
	*white = (w < 0) ? 0 : (w >> ENV_SH);
	*black = (b < 0) ? 0 : (b >> ENV_SH);
	if (!S3_BAR_BLACK_HIGH) {
		*white = 4095 - *white;
		*black = 4095 - *black;
	}
	return TRUE;
}

#endif	/* S3_BARCODE_ANALOG */

EXPORT BOOL barcode_busy(void)	{ return busy; }
EXPORT UW barcode_edges(void)	{ return ring_w; }
EXPORT BOOL barcode_candidate(void)	{ return busy && n_now >= S3_SCAN_MIN_ELEM; }
EXPORT UW barcode_cand_ends(void)	{ return cand_ends; }

/* B is over bars now: an edge in the last S3_IN_BARS_MM (widest element
   10.5 mm).  Motion holds the steering straight then: B sits 85 mm left
   of the robot's centre, so any turn changes B's own speed over the bars
   by +-50 % or more against the odometer, and the widths come out wrong. */
EXPORT BOOL barcode_in_bars(void)
{
	W d;

	if (!busy) return FALSE;
	d = motion_odometer() - edge_pub;
	if (d < 0) d = -d;
	return d < (W)(S3_IN_BARS_MM * 1000 / S2_UM_PER_COUNT);
}

/* Debug (Stage 2 tests): the element widths of the last code that ended,
   even one too short to report, so a test can show what the sensor saw */
#define DBG_MAX		40
LOCAL INT		dbg_w[DBG_MAX];	/* 0.1 mm */
LOCAL INT		dbg_n;
LOCAL volatile UW	dbg_seq;

EXPORT UW barcode_debug(INT *n, INT *w_mm10, INT max)
{
	INT	i;
	UINT	s;

	APP_LOCK(s);
	*n = dbg_n;
	for (i = 0; i < dbg_n && i < DBG_MAX && i < max; i++) w_mm10[i] = dbg_w[i];
	APP_UNLOCK(s);
	return dbg_seq;
}
/* Single black patches (left branches, grout lines) */
LOCAL W			mark_odo;
LOCAL INT		mark_w;		/* 0.1 mm */
LOCAL volatile UW	mark_seq;

EXPORT UW barcode_mark(W *centre_odo, INT *width_mm10)
{
	UINT	s;
	UW	q;

	APP_LOCK(s);
	*centre_odo = mark_odo;
	*width_mm10 = mark_w;
	q = mark_seq;
	APP_UNLOCK(s);
	return q;
}

/* The first bar of a run has just ended at end_odo, w counts wide.
   Published at once, not after the quiet gap, so a left branch is known
   while there is still room to turn (B sits behind C).  Every first bar
   is published (also a barcode's narrow start bar and grout lines); the
   line task keeps only the ones about one tape wide.  No Code 39 bar is
   that wide (wide bars measured 10.5 mm). */
LOCAL void mark_pub(W end_odo, W w)
{
	UINT	s;

	APP_LOCK(s);
	mark_odo = end_odo - w / 2;	/* driving forward: centre is behind */
	mark_w = (INT)(w * S2_UM_PER_COUNT / 100);
	mark_seq++;
	APP_UNLOCK(s);
}

EXPORT UW barcode_seq(void)	{ return seq_now; }
EXPORT void barcode_pause(BOOL on)	{ paused = on; }

/* Forget any half-read code (start of a run, after a manoeuvre).  On 7 Nov
   tape held under B at the end of the live view left a code open, and
   the run started with a rescan.  Task context; blocks 30 ms. */
EXPORT void barcode_reset(void)
{
	paused = TRUE;
	tk_dly_tsk(30);			/* the task drops it within 10 ms */
	paused = FALSE;
}

EXPORT void barcode_last(BARCODE_RESULT *r)
{
	UINT s;

	APP_LOCK(s);
	*r = last;
	APP_UNLOCK(s);
}

/* One character from 9 widths; 0 = no match */
LOCAL UB decode_char(const W *e, W *narrow_sum, W *wide_sum)
{
	INT	i, j, wide[3] = {-1, -1, -1};
	UH	pat = 0;
	W	min_wide, max_narrow = 0;

	for (j = 0; j < 3; j++) {
		INT best = -1;
		for (i = 0; i < 9; i++) {
			if (i == wide[0] || i == wide[1]) continue;
			if (best < 0 || e[i] > e[best]) best = i;
		}
		wide[j] = best;
		pat |= (UH)(1u << (8 - best));
	}
	min_wide = e[wide[2]];
	for (i = 0; i < 9; i++) {
		if ((pat & (1u << (8 - i))) != 0u) {
			*wide_sum += e[i];
		} else {
			*narrow_sum += e[i];
			if (e[i] > max_narrow) max_narrow = e[i];
		}
	}
	if (min_wide * 10 < max_narrow * 15) {
		return 0;			/* wide and narrow not separable */
	}
	for (i = 0; i < (INT)(sizeof(c39) / sizeof(c39[0])); i++) {
		if (c39[i].pat == pat) return c39[i].ch;
	}
	return 0;
}

/* Decode n elements; fills r.  TRUE = valid "*...*".  The code may start
   at any bar (off even) and end before the last element: a stray mark
   within S3_QUIET_MM of the code is part of the same run. */
LOCAL BOOL decode(W *e, INT n, BARCODE_RESULT *r)
{
	INT	k, pass, i, off;
	W	ns = 0, ws = 0;

	r->ok = FALSE;
	r->elements = n;
	if (n < 29) {
		return FALSE;
	}
	for (pass = 0; pass < 2; pass++) {
		for (off = 0; off + 29 <= n; off += 2) {
			ns = ws = 0;
			if (decode_char(&e[off], &ns, &ws) != '*') continue;
			for (k = (n - off + 1) / 10; k >= 3; k--) {
				UB ch;

				ns = ws = 0;
				(void)decode_char(&e[off], &ns, &ws);
				if (decode_char(&e[off + (k - 1) * 10], &ns, &ws) != '*') {
					continue;
				}
				ch = decode_char(&e[off + 10], &ns, &ws);
				if (ch != 0 && ch != '*') {
					r->ok = TRUE;
					r->ch = ch;
					r->reversed = (pass == 1);
					/* 3 chars of 6 narrow + 3 wide counted above */
					r->narrow_mm10 = (INT)(ns * S2_UM_PER_COUNT / 100 / 18);
					r->wide_mm10 = (INT)(ws * S2_UM_PER_COUNT / 100 / 9);
					return TRUE;
				}
			}
		}
		/* try backwards */
		for (i = 0; i < n / 2; i++) {
			W t = e[i];
			e[i] = e[n - 1 - i];
			e[n - 1 - i] = t;
		}
	}
	return FALSE;
}

EXPORT void barcode_task(INT stacd, void *exinf)
{
	W	elem[MAX_ELEM];
	INT	n = 0;
	W	prev_odo = 0;
	BOOL	in_code = FALSE, too_long = FALSE;
	UW	seq = 0, end;
	UB	stable = !IR_BLACK;	/* debounced level */
	UB	raw_lvl = !IR_BLACK;	/* level after the last raw edge */
	W	raw_odo = 0;		/* where that raw edge was */
	BOOL	raw_ok = FALSE;

	/* A level counts only once it has held for MERGE_COUNTS of travel:
	   then it starts where its last raw edge was.  Flicker never holds,
	   so it neither makes elements nor keeps a code open (Stage 2, 2 Nov:
	   pair-cancelling let every other flicker edge through). */
#define COMMIT(lvl, pos)						\
	do {								\
		if ((lvl) != stable) {					\
			stable = (lvl);					\
			if (!in_code) {					\
				if ((lvl) == IR_BLACK) {		\
					in_code = TRUE;			\
					too_long = FALSE;		\
					n = 0;				\
					prev_odo = (pos);		\
					edge_pub = (pos);		\
					busy = TRUE;			\
				}					\
			} else {					\
				W w_ = (pos) - prev_odo;		\
				if (w_ < 0) w_ = -w_;			\
				if (n < MAX_ELEM) elem[n++] = w_;	\
				else too_long = TRUE;			\
				if (n == 1) mark_pub((pos), w_);	\
				prev_odo = (pos);			\
				edge_pub = (pos);			\
			}						\
		}							\
	} while (0)

	(void)stacd; (void)exinf;
	ring_r = ring_w;

	while (1) {
		(void)tk_slp_tsk(TIMEOUT_BARCODE_MS);
		app_timing_wake(tmg_barcode, 0);

		/* Motion is backing up to rescan: drop this read and every edge
		   until it drives forward again, then start fresh */
		if (paused) {
			ring_r = ring_w;
			in_code = FALSE;
			n = 0;
			busy = FALSE;
			n_now = 0;
			stable = !IR_BLACK;
			raw_ok = FALSE;
			app_timing_done(tmg_barcode);
			continue;
		}

		/* Drain the edges that are here now, never more than the ring
		   holds: a chattering sensor keeps adding edges, and an
		   unbounded loop starved every task below this one (530 ms,
		   Stage 2 run, 1 Nov).  If the ring overflowed, edges were
		   lost: drop the barcode being read. */
		{
			UW	avail = ring_w - ring_r;

			if (avail > RING) {
				ring_r = ring_w - RING;
				in_code = FALSE;
				n = 0;
				stable = !IR_BLACK;
				raw_ok = FALSE;
				avail = RING;
			}
			end = ring_r + avail;
		}
		while (ring_r != end) {
			EDGE ed = ring[ring_r & (RING - 1)];
			W    d;

			ring_r++;
			if (raw_ok) {
				d = ed.odo - raw_odo;
				if (d < 0) d = -d;
				if (d >= MERGE_COUNTS) {
					COMMIT(raw_lvl, raw_odo);	/* that level held */
				}
			}
			raw_lvl = ed.level;
			raw_odo = ed.odo;
			raw_ok = TRUE;
		}
		/* The latest level may have held long enough by now */
		if (raw_ok) {
			W d = motion_odometer() - raw_odo;

			if (d < 0) d = -d;
			if (d >= MERGE_COUNTS) {
				COMMIT(raw_lvl, raw_odo);
			}
		}

		/* End of a barcode: quiet for S3_QUIET_MM */
		if (in_code) {
			W gap = motion_odometer() - prev_odo;

			if (gap < 0) gap = -gap;
			if (S2_COUNTS_TO_MM(gap) > S3_QUIET_MM) {
				in_code = FALSE;
				if (n >= S3_SCAN_MIN_ELEM) cand_ends++;
				{
					INT	i;
					UINT	s;

					APP_LOCK(s);
					dbg_n = n;
					for (i = 0; i < n && i < DBG_MAX; i++) {
						dbg_w[i] = (INT)(elem[i] * S2_UM_PER_COUNT / 100);
					}
					dbg_seq++;
					APP_UNLOCK(s);
				}
				if (n >= MIN_ELEM_REPORT) {
					BARCODE_RESULT	r = {0};
					UINT		s;

					if (!too_long) {
						(void)decode(elem, n, &r);
					}
					r.seq = ++seq;
					r.elements = n;
					APP_LOCK(s);
					last = r;
					APP_UNLOCK(s);
					seq_now = seq;
					tk_set_flg(flg_mission, MSN_FLG_BARCODE);
				}
				n = 0;
			}
		}
		busy = in_code;
		n_now = in_code ? n : 0;

		app_timing_done(tmg_barcode);
	}
}

/* ---- Mapping (B7, B8) ---------------------------------------------- */

LOCAL BOOL is_code_char(UB c)
{
	return (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9');
}

EXPORT BOOL barcode_set_mapping(const UB *text, INT len, UB *why, INT why_max)
{
	UB	ch[4], mv[4];
	INT	pairs = 0, i = 0, a, b;
	UINT	s;

	why[0] = '\0';
	while (i < len) {
		UB c, m;

		while (i < len && (text[i] == ' ' || text[i] == ',')) i++;
		if (i >= len) break;
		if (pairs == 4 || i + 2 >= len || text[i + 1] != '=') {
			tm_sprintf(why, (const UB*)"expected 4 pairs like K=L,Q=R,X=F,M=U");
			return FALSE;
		}
		c = text[i];
		m = text[i + 2];
		if (c >= 'a' && c <= 'z') c = (UB)(c - 'a' + 'A');
		if (m >= 'a' && m <= 'z') m = (UB)(m - 'a' + 'A');
		if (!is_code_char(c) ||
		    (m != MOVE_LEFT && m != MOVE_RIGHT && m != MOVE_FORWARD && m != MOVE_UTURN)) {
			tm_sprintf(why, (const UB*)"pair %d: '%c=%c' not A-Z/0-9 = L/R/F/U",
				   pairs + 1, c, m);
			return FALSE;
		}
		ch[pairs] = c;
		mv[pairs] = m;
		pairs++;
		i += 3;
	}
	if (pairs != 4) {
		tm_sprintf(why, (const UB*)"%d pairs, need exactly 4", pairs);
		return FALSE;
	}
	for (a = 0; a < 4; a++) {
		for (b = a + 1; b < 4; b++) {
			if (ch[a] == ch[b] || mv[a] == mv[b]) {
				tm_sprintf(why, (const UB*)"each character and each move only once");
				return FALSE;
			}
		}
	}
	(void)why_max;
	APP_LOCK(s);
	for (a = 0; a < 4; a++) {
		map_ch[a] = ch[a];
		map_mv[a] = mv[a];
	}
	map_set = TRUE;
	APP_UNLOCK(s);
	return TRUE;
}

EXPORT INT barcode_move_for(UB ch)
{
	INT	i, mv = MOVE_NONE;
	UINT	s;

	APP_LOCK(s);
	if (map_set) {
		for (i = 0; i < 4; i++) {
			if (map_ch[i] == ch) mv = map_mv[i];
		}
	}
	APP_UNLOCK(s);
	return mv;
}

EXPORT INT barcode_mapping_text(UB *out, INT max)
{
	UB	ch[4], mv[4];
	BOOL	set;
	UINT	s;
	INT	i;

	APP_LOCK(s);
	set = map_set;
	for (i = 0; i < 4; i++) {
		ch[i] = map_ch[i];
		mv[i] = map_mv[i];
	}
	APP_UNLOCK(s);
	if (!set || max < 16) {
		out[0] = '\0';
		return 0;
	}
	return tm_sprintf(out, (const UB*)"%c=%c,%c=%c,%c=%c,%c=%c",
			  ch[0], mv[0], ch[1], mv[1], ch[2], mv[2], ch[3], mv[3]);
}
