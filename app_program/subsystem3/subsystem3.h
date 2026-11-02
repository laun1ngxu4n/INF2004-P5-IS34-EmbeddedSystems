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

/* Which side of the centre sensor the GP0 side sensor is mounted on,
   seen from behind the robot: +1 left, -1 right.  On our robot it is on
   the RIGHT (Stage 2 run, 1 Nov), so the line task follows the line's
   right edge and every steering value is mirrored. */
#define S3_SIDE			(-1)

/* Geometry, NOT MEASURED YET: set from the robot */
#define S3_SENSOR_TO_AXLE_MM	60	/* line sensors in front of the wheel axle */

/* IF-03: steering for Motion, mm/s added to the right wheel and taken
   from the left (+ = turn left).  Latest value only. */
IMPORT INT	line_steer_mm_s(void);

/* Line state, for Mission and telemetry */
#define LINE_ON		0	/* centre on the line */
#define LINE_SEARCH	1	/* centre lost, steering back */
#define LINE_LOST	2	/* lost for more than S3_LOST_MM */
IMPORT INT	line_state(void);
IMPORT UW	line_junctions(void);		/* junctions seen since boot */
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
IMPORT BOOL	barcode_candidate(void);	/* bar, space, bar seen: worth a rescan */
IMPORT UW	barcode_seq(void);		/* +1 for every result published */
IMPORT void	barcode_pause(BOOL on);		/* TRUE: ignore edges (backing up);
						   FALSE: start a fresh read */
IMPORT UW	barcode_debug(INT *n, INT *w_mm10, INT max);	/* last code that
						   ended, any length; returns a seq */

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
