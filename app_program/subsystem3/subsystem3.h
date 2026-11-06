/*
 *	subsystem3.h
 *	Subsystem 3 -- Line Following + Barcode Decoder (Buddy 3, Lau Ning Xuan).
 *
 *	This header is subsystem 3's PUBLIC interface: only what other
 *	subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *	files.  Other subsystems include it as "../subsystem3/subsystem3.h".
 *
 *	Stage 2:
 *	  Line (20 ms): IR_LEFT (GP0) sits left of the line, IR_CENTER (GP1)
 *	  over it (1 = black).  Steering goes to Motion through IF-03
 *	  (line_steer_mm_s, latest value only).  Junction = both black for a
 *	  short distance, then left white again while centre stays black.
 *	  Barcode: GP2 edges are stamped with the encoder odometer in the
 *	  interrupt, so bar widths are distances, not times (TEAM.md 7.1 B3).
 *	  The decoder reads the full Code 39 table in both directions (B1, B2)
 *	  and keeps the 4-entry mapping that arrives over MQTT (B7, B8).
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"

#define PIN_IR_LEFT		0	/* the side (edge) sensor; see S3_SIDE */
#define PIN_IR_CENTER		1
#define PIN_IR_BARCODE		2
#define IR_BLACK		1	/* D0 level over black (bring-up) */

/* Barcode sensor input.  0: its digital D0 on GP2 (the module's pot sets
   the threshold).  1: its analogue AO on GP28 (ADC 2); the threshold and
   hysteresis are set in software, from the white and black levels it
   has seen.  For 1: power the module from 3V3, NOT 5 V (AO would go
   above the ADC's 3.3 V limit), and wire AO to GP28. */
#define S3_BARCODE_ANALOG	0
#define PIN_IR_BARCODE_AO	28
#define S3_BAR_ADC_CH		2	/* GP28 = ADC input 2 */
#define S3_BAR_BLACK_HIGH	1	/* AO is higher over black (TCRT5000
					   modules); the live view shows it */
/* Fixed thresholds (raw 0..4095): 0 = learn them from the white and black
   levels seen (the live view prints them to copy here) */
#define S3_BAR_ADC_ON		0	/* black from here (above, if BLACK_HIGH) */
#define S3_BAR_ADC_OFF		0	/* white again from here */
#define S3_BAR_MIN_SPAN		400	/* learned: black-white gap needed before
					   anything counts as black */

/* Which side of the centre sensor the GP0 side sensor is mounted on,
   seen from behind the robot: +1 left, -1 right.  On our robot it is on
   the RIGHT (Stage 2 run, 1 Nov), so the line task follows the line's
   right edge and every steering value is mirrored. */
#define S3_SIDE			(-1)

/* Geometry, measured 6 Nov */
#define S3_SENSOR_TO_AXLE_MM	115	/* line sensors in front of the wheel axle
					   (was a guess of 60) */
#define S3_TAPE_MM		20	/* line tape width */
/* C to S, sensor head centre to centre, sideways.  Not used by the code:
   it is how the robot must be built.  With C on the tape, S must sit
   just past the tape edge, so drifting one way puts S on the tape and
   the other way takes C off it.  Wider than the tape, the tape can sit
   BETWEEN C and S (both white), and the robot turns the wrong way first
   (the side to side of the Stage 2 runs; about 30 mm in the 6 Nov photo). */
#define S3_S_GAP_MM		15
#define S3_BAR_AHEAD_MM		(-40)	/* barcode sensor B in front of C; it is
					   BEHIND (6 Nov), so negative */
#define S3_BAR_LEFT_MM		70	/* B left of C (clears the main line while
					   the tape is narrower than 2 x this) */

/* Junctions (details in task_line.c).  A branch on the S side: S black
   for about one tape width while C stays on the line, with the steering
   held (so drift and curves, which keep S black longer, do not count).
   A branch on the LEFT: B (which also reads the barcodes) sees a black
   patch about one tape width long, and at that place C was on the line
   before and after it.  Both within S3_JUNC_MERGE_MM = one crossroads.
   Line ending at a cross bar with both arms (T end) = a junction; one
   arm only = a corner, just followed. */
#define S3_JUNC_MIN_MM		(S3_TAPE_MM * 6 / 10)	/* tape width -40 % .. +40 % */
#define S3_JUNC_MAX_MM		(S3_TAPE_MM * 14 / 10)
#define S3_JUNC_MERGE_MM	40
#define JUNC_SIDE_S		1	/* branch on the side sensor's side */
#define JUNC_SIDE_B		2	/* branch on the left, seen by B */

/* IF-03: steering for Motion, mm/s added to the right wheel and taken
   from the left (+ = turn left).  Latest value only. */
IMPORT INT	line_steer_mm_s(void);

/* Line state, for Mission and telemetry */
#define LINE_ON		0	/* centre on the line */
#define LINE_SEARCH	1	/* centre lost, steering back */
#define LINE_LOST	2	/* lost for more than S3_LOST_MM */
IMPORT INT	line_state(void);
IMPORT UW	line_junctions(void);		/* junctions seen since boot */
IMPORT W	line_junction_odo(void);	/* last junction's centre, where C
						   crossed it (odometer counts) */
IMPORT INT	line_junction_sides(void);	/* its branches: JUNC_SIDE_S | _B */
IMPORT void	line_set_enabled(BOOL on);	/* FALSE: no junction / lost events
						   (during turns and bypasses) */
IMPORT BOOL	line_center_black(void);	/* raw centre sensor */
IMPORT BOOL	line_centre_on(void);		/* filtered centre sensor on the line */

/* Barcode results */
typedef struct {
	UW	seq;		/* +1 for every decode attempt */
	BOOL	ok;		/* FALSE = READ_FAIL */
	UB	ch;		/* decoded data character ('A'..'Z', '0'..'9', ...) */
	BOOL	reversed;	/* read backwards */
	INT	elements;	/* bars + spaces seen */
	INT	narrow_mm10;	/* mean narrow element, 0.1 mm */
	INT	wide_mm10;	/* mean wide element, 0.1 mm */
} BARCODE_RESULT;
IMPORT void	barcode_last(BARCODE_RESULT *r);
IMPORT BOOL	barcode_busy(void);		/* inside a barcode now (Motion slows) */
IMPORT UW	barcode_edges(void);		/* barcode sensor edges since boot */
IMPORT BOOL	barcode_black(void);		/* sensor sees black now (either input) */
/* Analogue input only (S3_BARCODE_ANALOG 1): last raw value and the learned
   white / black levels, 0..4095.  FALSE in the digital build. */
IMPORT BOOL	barcode_adc(INT *raw, INT *white, INT *black);
IMPORT BOOL	barcode_candidate(void);	/* bar, space, bar seen: worth a rescan */
IMPORT UW	barcode_seq(void);		/* +1 for every result published */
IMPORT void	barcode_pause(BOOL on);		/* TRUE: ignore edges (backing up);
						   FALSE: start a fresh read */
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
