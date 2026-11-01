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
 *	    reversed and decoded again: read backwards (B2).
 *	Result: barcode_last(), and MSN_FLG_BARCODE to Mission.
 */

#include "subsystem3.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem5/subsystem5.h"

#define RING		128		/* edges kept, a power of 2 */
#define MAX_ELEM	99		/* up to 10 characters */
#define S3_QUIET_MM	40		/* no edge for this far = end of barcode */
#define MERGE_COUNTS	6		/* < 0.5 mm apart = sensor chatter */
#define MIN_ELEM_REPORT	20		/* fewer elements = noise, not reported */

typedef struct { W odo; UB level; } EDGE;

LOCAL EDGE		ring[RING];
LOCAL volatile UW	ring_w;		/* written by the interrupt */
LOCAL UW		ring_r;		/* read by the task */

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

EXPORT ER barcode_hw_init(void)
{
	ER er = app_gpio_input(PIN_IR_BARCODE, FALSE);

	return (er < E_OK) ? er : app_gpio_irq_add(PIN_IR_BARCODE, APP_GPIO_BOTH, bar_edge);
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

/* Decode n elements; fills r.  TRUE = valid "*...*" */
LOCAL BOOL decode(W *e, INT n, BARCODE_RESULT *r)
{
	INT	k, pass, i;
	W	ns = 0, ws = 0;

	r->ok = FALSE;
	r->elements = n;
	if ((n + 1) % 10 != 0 || n < 29) {
		return FALSE;
	}
	k = (n + 1) / 10;
	for (pass = 0; pass < 2; pass++) {
		UB first, lastc;

		ns = ws = 0;
		first = decode_char(&e[0], &ns, &ws);
		if (first == '*') {
			lastc = decode_char(&e[(k - 1) * 10], &ns, &ws);
			if (lastc == '*') {
				UB ch = decode_char(&e[10], &ns, &ws);
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
	UW	seq = 0;

	(void)stacd; (void)exinf;
	ring_r = ring_w;

	while (1) {
		(void)tk_slp_tsk(TIMEOUT_BARCODE_MS);
		app_timing_wake(tmg_barcode, 0);

		/* Drain new edges */
		while (ring_r != ring_w) {
			EDGE ed = ring[ring_r & (RING - 1)];
			W    w;

			ring_r++;
			if (!in_code) {
				if (ed.level == IR_BLACK) {	/* first bar starts */
					in_code = TRUE;
					too_long = FALSE;
					n = 0;
					prev_odo = ed.odo;
				}
				continue;
			}
			w = ed.odo - prev_odo;
			if (w < 0) w = -w;
			if (w < MERGE_COUNTS) {
				/* chatter: this edge and the previous one cancel */
				if (n == 0) {
					in_code = FALSE;	/* the "first bar" was a glitch */
				} else {
					n--;
					prev_odo -= elem[n];	/* back to the start of that element */
				}
				continue;
			}
			if (n < MAX_ELEM) {
				elem[n++] = w;
			} else {
				too_long = TRUE;
			}
			prev_odo = ed.odo;
		}

		/* End of a barcode: quiet for S3_QUIET_MM */
		if (in_code) {
			W gap = motion_odometer() - prev_odo;

			if (gap < 0) gap = -gap;
			if (S2_COUNTS_TO_MM(gap) > S3_QUIET_MM) {
				in_code = FALSE;
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
					tk_set_flg(flg_mission, MSN_FLG_BARCODE);
				}
				n = 0;
			}
		}

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
