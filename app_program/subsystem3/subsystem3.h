/*
 *	subsystem3.h
 *	Subsystem 3 -- Line Following + Barcode Decoder (Buddy 3, Lau Ning Xuan).
 *
 *	This header is subsystem 3's PUBLIC interface: only what other
 *	subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *	files.  Other subsystems include it as "../subsystem3/subsystem3.h".
 *
 *	Stage 2:
 *	  Line (20 ms): two sensors STRADDLE the tape, L (GP1) left of it and
 *	  R (GP0) right of it, both white when centred (1 = black).  Steering
 *	  goes to Motion through IF-03 (line_steer_mm_s, latest value only).
 *	  Crossing = both black for about one tape width.
 *	  Barcode: GP2 edges are stamped with the encoder odometer in the
 *	  interrupt, so bar widths are distances, not times (TEAM.md 7.1 B3).
 *	  The decoder reads the full Code 39 table in both directions (B1, B2)
 *	  and keeps the 4-entry mapping that arrives over MQTT (B7, B8).
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"

/* Line sensors, straddling the tape (7 Nov; before that GP1 rode on the
   tape and GP0 sat beside its right edge) */
#define PIN_IR_LINE_L		1	/* left of the tape */
#define PIN_IR_LINE_R		0	/* right of the tape */
#define PIN_IR_BARCODE		2
#define IR_BLACK		1	/* D0 level over black (bring-up) */

/* Barcode sensor input.  0: its digital D0 on GP2 (the module's pot sets
   the threshold).  1: its analogue AO on GP28 (ADC 2); the threshold and
   hysteresis are set in software, from the white and black levels it
   has seen.  For 1: power the module from 3V3, NOT 5 V (AO would go
   above the ADC's 3.3 V limit), and wire AO to GP28.  Analogue since
   7 Nov (B wired to GP28 then). */
#define S3_BARCODE_ANALOG	1
#define PIN_IR_BARCODE_AO	28
#define S3_BAR_ADC_CH		2	/* GP28 = ADC input 2 */
#define S3_BAR_BLACK_HIGH	1	/* AO is higher over black (TCRT5000
					   modules); the live view shows it */
/* Fixed thresholds (raw 0..4095): 0 = learn them from the white and black
   levels seen (the live view prints them to copy here).  From the 7 Nov
   live view: white 170-570, tape about 4040.  But printed bars read much
   greyer (S3-B00, 7 Nov: wide bars peaked at only 2400-3300, narrow ones
   did not reach 2100 at all), so 2500 / 1700 lost bars: lowered to sit
   just above the whitest white. */
#define S3_BAR_ADC_ON		1300	/* black from here (above, if BLACK_HIGH) */
#define S3_BAR_ADC_OFF		800	/* white again from here (white up to 570) */
/* Relative edges (used instead of ON/OFF when 1): black once the reading
   rises S3_BAR_DELTA above the lowest since the last bar (and is at least
   S3_BAR_BLACK_MIN), white again once it falls S3_BAR_DELTA below the
   highest since the bar began.  "A small" through B's blurred spot (7 Nov,
   S3-B00): narrow bars peaked at 1700-3100, yet the narrow space between
   two wide bars stayed above 1050, so no fixed pair of thresholds sees
   both.  Simulated (spot sigma 1.2-1.6 mm, ink 65-100 %): fixed 2500/1700
   decoded 24/54, 1300/800 6/54, relative 54/54. */
#define S3_BAR_RELATIVE		1
#define S3_BAR_DELTA		700	/* white noise is +-20 */
#define S3_BAR_BLACK_MIN	1500	/* never black below this (white up to 570).
					   Raised from 500 / 900 on 7 Nov at the
					   team's request (fewer faint marks seen
					   as bars); simulated "A small" still
					   54/54, 800 / 1800 began to lose reads */
#define S3_BAR_MIN_SPAN		400	/* learned: black-white gap needed before
					   anything counts as black */

/* Geometry, measured 6 Nov */
#define S3_SENSOR_TO_AXLE_MM	115	/* line sensors in front of the wheel axle
					   (was a guess of 60) */
#define S3_TAPE_MM		20	/* line tape width */
/* L to R, sensor head centre to centre, sideways.  Not used by the code:
   it is how the robot must be built.  A little wider than the tape, so
   both read white when centred, with a few mm to spare each side: about
   30 mm for 20 mm tape (7 Nov).  Much wider = it wanders before it
   corrects; narrower than the tape = both black all the time. */
#define S3_LR_GAP_MM		30
#define S3_BAR_AHEAD_MM		(-40)	/* barcode sensor B in front of the line
					   sensors; it is BEHIND (6 Nov), so
					   negative */
#define S3_BAR_LEFT_MM		70	/* B left of the left line sensor */

/* Junctions (details in task_line.c).
   Crossing (X, crossroads, T end): L and R both black for about one tape
   width (the tape is narrower than the L-R gap, so only a cross line
   does that).  A branch to the LEFT only: B (which also reads the
   barcodes) sees a black patch about one tape width long while the robot
   was driving straight there.  A branch to the RIGHT only cannot be told
   from a correction: the track has none (7 Nov).  Two within
   S3_JUNC_MERGE_MM = one junction. */
#define S3_JUNC_MIN_MM		(S3_TAPE_MM * 6 / 10)	/* tape width -40 % .. +40 % */
#define S3_JUNC_MAX_MM		(S3_TAPE_MM * 14 / 10)
#define S3_JUNC_MERGE_MM	40
#define JUNC_SIDE_S		1	/* a crossing: both line sensors black */
#define JUNC_SIDE_B		2	/* a left arm, seen by B */

/* IF-03: steering for Motion, mm/s added to the right wheel and taken
   from the left (+ = turn left).  Latest value only. */
IMPORT INT	line_steer_mm_s(void);

/* Line state, for Mission and telemetry */
#define LINE_ON		0	/* following */
#define LINE_SEARCH	1	/* overshot a tight curve: turning back */
#define LINE_LOST	2	/* no line for too long */
#define LINE_FAULT	3	/* a line sensor looks stuck (line_fault() says
				   which); Mission stops as for LINE_LOST */
IMPORT INT	line_state(void);
#define LINE_FAULT_BOTH	1	/* both black for S3_STUCK_BOTH_MM */
#define LINE_FAULT_ONE	2	/* one black for S3_STUCK_ONE_MM */
IMPORT INT	line_fault(void);		/* LINE_FAULT_... or 0 */
IMPORT UW	line_junctions(void);		/* junctions seen since boot */
IMPORT void	line_wobble(UW *corrections, UW *sweeps);	/* since the start:
						   corrections begun, and sweeps right
						   across the tape (zigzag) */
IMPORT W	line_junction_odo(void);	/* last junction's centre, where the
						   line sensors crossed it (odometer
						   counts) */
IMPORT INT	line_junction_sides(void);	/* its branches: JUNC_SIDE_S | _B */
IMPORT void	line_set_enabled(BOOL on);	/* FALSE: no junction / lost events
						   (during turns and bypasses) */
IMPORT BOOL	line_any_black(void);		/* raw: either line sensor on black */
IMPORT INT	line_speed_pct(void);		/* % of the follow speed Motion may
						   use: 100 on track, less while
						   correcting or searching */

/* Barcode results */
typedef struct {
	UW	seq;		/* +1 for every decode attempt */
	BOOL	ok;		/* FALSE = READ_FAIL */
	UB	ch;		/* decoded data character ('A'..'Z', '0'..'9', ...) */
	BOOL	reversed;	/* read backwards */
	INT	elements;	/* bars + spaces seen */
	INT	narrow_mm10;	/* mean narrow element, 0.1 mm */
	INT	wide_mm10;	/* mean wide element, 0.1 mm */
	UB	fixed;		/* 0 clean read; BAR_FIX_STAR one '*' damaged;
				   BAR_FIX_MERGED a lost narrow element put back;
				   BAR_FIX_CUT a missing end bar put back */
} BARCODE_RESULT;
#define BAR_FIX_STAR	1
#define BAR_FIX_MERGED	2
#define BAR_FIX_CUT	3	/* a weak narrow bar at an end not seen */
IMPORT void	barcode_last(BARCODE_RESULT *r);
IMPORT BOOL	barcode_busy(void);		/* inside a barcode now (Motion slows) */
IMPORT UW	barcode_edges(void);		/* barcode sensor edges since boot */
IMPORT BOOL	barcode_black(void);		/* sensor sees black now (either input) */
/* Analogue input only (S3_BARCODE_ANALOG 1): last raw value and the learned
   white / black levels, 0..4095.  FALSE in the digital build. */
IMPORT BOOL	barcode_adc(INT *raw, INT *white, INT *black);
IMPORT BOOL	barcode_in_bars(void);		/* B is over bars now: hold the
						   steering straight */
IMPORT UW	barcode_seq(void);		/* +1 for every result published */
IMPORT void	barcode_reset(void);		/* drop a half-read code (start of
						   a run); blocks 30 ms */
IMPORT UW	barcode_debug(INT *n, INT *w_mm10, INT max);	/* last code that
						   ended, any length; returns a seq */
/* Last first-bar under B (published as soon as it ends): a left branch
   if about one tape wide; else a grout line or a barcode's start bar.
   Its centre (odometer counts) and width.  Returns a seq, +1 for each. */
IMPORT UW	barcode_mark(W *centre_odo, INT *width_mm10);

/* Mapping (TEAM.md 7.1 B7/B8): exactly 4 pairs "K=L,Q=R,X=F,M=U".
   L left, R right, F forward, U U-turn.  Returns TRUE if accepted.  On
   FALSE the old mapping stays and why[] says why. */
#define MOVE_NONE	0
#define MOVE_LEFT	'L'
#define MOVE_RIGHT	'R'
#define MOVE_FORWARD	'F'
#define MOVE_UTURN	'U'
IMPORT BOOL	barcode_set_mapping(const UB *text, INT len, UB *why, INT why_max);
IMPORT INT	barcode_move_for(UB ch);	/* MOVE_... or MOVE_NONE */
IMPORT INT	barcode_mapping_text(UB *out, INT max);	/* "K=L,Q=R,..." or "" */

/* Private to subsystem 3, shared between its own .c files */
IMPORT ID	tid_line;
IMPORT ID	cyc_line;
IMPORT INT	tmg_line;
IMPORT ID	tid_barcode;
IMPORT INT	tmg_barcode;
IMPORT void	line_task(INT stacd, void *exinf);
IMPORT void	barcode_task(INT stacd, void *exinf);
IMPORT ER	barcode_hw_init(void);
IMPORT ER	line_hw_init(void);

#endif /* SUBSYSTEM3_H */
