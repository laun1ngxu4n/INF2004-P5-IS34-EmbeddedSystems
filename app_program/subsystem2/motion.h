/**
 * @file    motion.h
 *  Motion Control (motion.c): drive, turn and follow commands and
 *  their status.  Part of subsystem 2's interface (subsystem2.h
 *  includes it).
 */

#ifndef MOTION_H
#define MOTION_H

#include "../app.h"

/* Result of the last command */
#define MOT_RES_NONE    0 /* nothing run yet */
#define MOT_RES_BUSY    1 /* still running */
#define MOT_RES_DONE    2 /* reached its distance / angle */
#define MOT_RES_STOPPED 3 /* cancelled by app_stop_request() */
#define MOT_RES_TIMEOUT 4 /* took > 2x expected + 1 s (blocked?) */

typedef struct
{
    ID       h_cmd_id; /* id of the last command accepted */
    int32_t  result;   /* MOT_RES_... of that command */
    int32_t  count_l;  /* encoder counts since boot, + = forward */
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

IMPORT void motion_task (int32_t stacd, void * exinf);

#endif /* MOTION_H */

/*** end of file ***/

