/**
 * @file    obstacle.h
 *  Obstacle Scanner (obstacle.c): the sonar on its servo, the forward
 *  watch, the profiling scan.  Part of subsystem 5's interface
 *  (subsystem5.h includes it).
 */

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "../app.h"

/* Sonar results: mm, or SONAR_OPEN (echo came back too late or not at
   all after rising: nothing in range), or SONAR_FAULT (ECHO never rose:
   sensor or wiring fault, never treated as open space). */
#define SONAR_OPEN  (-1)
#define SONAR_FAULT (-2)

IMPORT void obstacle_watch (bool on); /* forward watch during line following */
IMPORT void obstacle_idle_ping (bool on); /* mission image: ping for the panel
                         while Mission is IDLE (never during manoeuvres) */
IMPORT bool    obstacle_sensor_fault (void); /* 5 pings in a row without ECHO */
IMPORT int32_t obstacle_front_mm (void); /* last forward reading, -1 = none */

typedef struct
{
    bool    b_found;      /* an obstacle straight ahead */
    int32_t front_mm;     /* distance straight ahead */
    int32_t left_mm;      /* left edge, mm left of the robot's centre line */
    int32_t right_mm;     /* right edge, mm (negative = right of centre) */
    int32_t width_mm;     /* left - right */
    int32_t unc_mm;       /* +/- uncertainty */
    bool    b_left_clear; /* open space beyond the left edge */
    bool    b_right_clear;
    bool    b_left_seen; /* the scan saw past the left edge (false:
                  the obstacle filled the scan that side,
                  edge and clear unknown) */
    bool    b_right_seen;
    int32_t left_unc; /* +/- of each edge */
    int32_t right_unc;
    int32_t rays;       /* rays scanned */
    int32_t fault_rays; /* rays where the sensor did not answer */
} obstacle_profile_t;

/** Blocking: called by Mission (or a test) while the robot stands still.
   Turns the servo, settles, 5 pings per ray, median.  About 7 s. */
IMPORT void obstacle_scan (obstacle_profile_t * p);
/** Combine two scans (O5): mean width, uncertainty includes their spread */
IMPORT void obstacle_combine (const obstacle_profile_t * a,
                              const obstacle_profile_t * b,
                              obstacle_profile_t *       out);
IMPORT void obstacle_servo_angle (int32_t deg); /* 90 = ahead, clamped to 90 +/-
                          the larger of S5_SERVO_HALF_DEG and
                          S5_SERVO_SIDE_DEG; blocks while it
                          turns (slowly) */
IMPORT void obstacle_look_side (int32_t side);  /* +1 left, -1 right, 0 ahead:
                            S5_SERVO_SIDE_DEG that way */
IMPORT int32_t
obstacle_ping_mm (void); /* blocking 60 ms; mm, SONAR_OPEN or SONAR_FAULT */
/** The same in two halves, so the caller can do other things in the 60 ms:
   start, then read at least 60 ms later */
IMPORT void    obstacle_ping_start (void);
IMPORT int32_t obstacle_ping_result (void);

IMPORT void obstacle_task (int32_t stacd, void * exinf);
IMPORT ER   obstacle_hw_init (void);

#endif /* OBSTACLE_H */

/*** end of file ***/

