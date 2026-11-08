/**
 * @file    subsystem2.h
 *  Subsystem 2 -- Motion Control (Buddy 2, Ng Chern Loong Branson).
 *
 *  This header is subsystem 2's PUBLIC interface: only what other
 *  subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *  files.  Other subsystems include it as "../subsystem2/subsystem2.h".
 *
 *  Stage 1: motors on GP8-GP11 (PWM 10 kHz), encoders on GP4/5 and
 *  GP16/17, a speed controller per wheel every 10 ms, and two commands:
 *  drive a distance and spin on the spot.  Line steering (IF-03) comes
 *  in Stage 2.
 *
 *  Measured in bring-up (bringup/docs/bringup_results.md): 2,540
 *  counts/rev, 65 mm wheel, 0.080 mm/count; the left encoder counts DOWN
 *  for forward (the counts below are already corrected: + = forward).
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"

/* Robot geometry */
#define S2_UM_PER_COUNT 80 /* 0.080 mm per encoder count */
/* wheel centre to wheel centre. Effective value from S2-M02 (1 Nov): 130
 * turned about 95 / 190 deg for 90 / 180, so 130 x 90/95 = 123. Re-run
 * S2-M02 to confirm. */
#define S2_TRACK_MM 123

/* Line following: the slow-down while correcting is set by the line task
   (line_speed_pct(); 0 rocked in place on 2 Nov, so it always creeps).
   Speed while the barcode sensor is inside a barcode: */
#define S2_SPEED_IN_BARCODE 50

/* Speeds used by the team (mm/s) */
#define S2_SPEED_LINE 200 /* line following, design 10-25 cm/s */
/* line following and over barcodes (TEAM.md 7.1 B4). 150 ran past tight
 * curves (Stage 2 run, 7 Nov). 100 was tried later on 7 Nov with other line
 * changes; all of them were undone, back to the 9d10fa6 line following (the
 * third 7 Nov run) */
#define S2_SPEED_BARCODE 120
#define S2_SPEED_TURN    120 /* each wheel during a spin */

/** Odometer counts (a difference of motor_odometer()) to mm */
Inline int32_t motion_counts_to_mm (int32_t counts)
{
    return (counts * S2_UM_PER_COUNT) / 1000;
}

/* Robot body (NOT MEASURED YET, used by obstacle bypass and junctions) */
/* 130 measured (6 Nov) + the barcode arm sticking out 17 mm, counted both
 * sides */
#define S2_ROBOT_WIDTH_MM 170
/* body 150 measured (6 Nov) + the IR / barcode bar in front (not measured) */
#define S2_ROBOT_LENGTH_MM 200

/* Private to subsystem 2, shared between its own .c files */
IMPORT ID      gh_tid_motion;
IMPORT ID      gh_cyc_motion;
IMPORT int32_t g_tmg_motion;

#include "motion.h"
#include "motor.h"

#endif /* SUBSYSTEM2_H */

/*** end of file ***/

