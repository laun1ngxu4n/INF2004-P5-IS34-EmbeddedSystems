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

/* Result of the last command */
#define MOT_RES_NONE    0 /* nothing run yet */
#define MOT_RES_BUSY    1 /* still running */
#define MOT_RES_DONE    2 /* reached its distance / angle */
#define MOT_RES_STOPPED 3 /* cancelled by app_stop_request() */
#define MOT_RES_TIMEOUT 4 /* took > 2x expected + 1 s (blocked?) */

typedef struct
{
    uint32_t cmd_id;  /* id of the last command accepted */
    int32_t  result;  /* MOT_RES_... of that command */
    int32_t  count_l; /* encoder counts since boot, + = forward */
    int32_t  count_r;
    int32_t  speed_l; /* measured mm/s, + = forward (40 ms average) */
    int32_t  speed_r;
    uint32_t invalid; /* quadrature errors since boot (missed edges) */
} motion_status_t;

/** Commands.  Each returns the command id (> 0), or E_OBJ while a STOP
   reason is set.  A new command replaces a running one. */
IMPORT ID motion_drive (int32_t speed_mm_s,
                        int32_t dist_mm); /* speed < 0 = reverse */
IMPORT ID motion_turn (int32_t deg);      /* + = left (CCW) */
IMPORT ID motion_turn_at (int32_t deg, int32_t wheel_mm_s); /* a slower spin */
IMPORT ID motion_follow (int32_t speed_mm_s); /* follow the line (IF-03 steering
                         from line_steer_mm_s()) until another command */
IMPORT void motion_halt (void);               /* stop, no STOP reason */
/** Line following: never faster than this (0 = no limit).  Changes the
   speed of a running follow without restarting it (a new motion_follow
   would start again from standstill). */
IMPORT void motion_follow_limit (int32_t speed_mm_s);
IMPORT void motion_get_status (motion_status_t * st);

/** Motors off now.  Any context, also interrupt handlers; used by the
   STOP helper (app_stop.c) and, later, Safety (TEAM.md section 3). */
IMPORT void motor_safe_off (void);

/** Distance driven since boot, mean of both wheels, in encoder counts
   (0.080 mm each).  Any context, also interrupt handlers (barcode edges). */
IMPORT int32_t motion_odometer (void);
/** Odometer counts (a difference of motion_odometer()) to mm */
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
IMPORT void    motion_task (int32_t stacd, void * exinf);
IMPORT ER      motion_hw_init (void);
IMPORT void    motor_write (int32_t duty_l,
                            int32_t duty_r); /* -1000..1000 per mille */
IMPORT void    encoder_read (int32_t * l, int32_t * r, uint32_t * invalid);

#endif /* SUBSYSTEM2_H */

/*** end of file ***/

