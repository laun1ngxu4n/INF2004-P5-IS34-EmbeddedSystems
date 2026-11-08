/*
 *  subsystem5.h
 *  Subsystem 5 -- Mission Controller + Obstacle Scanner
 *  (Buddy 5, Muhammad Irfan).
 *
 *  This header is subsystem 5's PUBLIC interface: only what other
 *  subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *  files.  Other subsystems include it as "../subsystem5/subsystem5.h".
 *
 *  Stage 1: GP21 STOP stops the motors at once from its interrupt
 *  (app_stop_request) and tells Mission (MSN_FLG_STOP); GP20 START is
 *  read by polling (button_down).
 *  Stage 3: Obstacle Scanner (servo GP14, HC-SR04 TRIG GP6 / ECHO GP7):
 *  forward watch every 60 ms while enabled; on request a profiling scan
 *  (TEAM.md 7.2 O1-O6).
 *  Stage 6: Mission runs the whole course (task_mission.c).
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"
#include "../app_gpio.h"
#include "../app_stop.h"

/* IF-01: Mission's wake-up event flag, created and owned by subsystem 5.
   Other subsystems only set bits; Mission waits and clears them. */
IMPORT ID gh_flg_mission;
#define MSN_FLG_PING      0x0001u /* Stage 0 check from Telemetry, 1 Hz */
#define MSN_FLG_STOP      0x0002u /* STOP button or MQTT stop */
#define MSN_FLG_START     0x0004u /* MQTT start (the button is polled) */
#define MSN_FLG_JUNCTION  0x0008u /* Line: junction confirmed */
#define MSN_FLG_LINE_LOST 0x0010u /* Line: lost */
#define MSN_FLG_BARCODE   0x0020u /* Barcode: new result */
#define MSN_FLG_OBSTACLE  0x0040u /* Obstacle: something ahead */
#define MSN_FLG_ALL       0x007Fu /* every bit Mission waits for */

/* On-board buttons (active low), owned by subsystem 5 */
#define PIN_START        20
#define PIN_STOP         21
#define button_down(pin) (app_gpio_get(pin) == 0u)

/* ---- Obstacle Scanner ---------------------------------------------- */

#define S5_TRIP_MM 250 /* stop for an obstacle closer than this */
/* largest expected obstacle (O7): the bypass always plans for at least this */
#define S5_MAX_WIDTH_MM 200
/* ... and never for more: a scan that says 500 mm is wrong (instructor:
 * 10-20 cm), and the detour it asked for ran into the room (7 Nov, "leg 4
 * blocked") */
#define S5_PLAN_MAX_MM 300

/* Servo: pulse = 1500 us + (angle - 90) x S5_SERVO_US_PER_DEG.  Angle 90 =
   straight ahead.  If 150 points RIGHT instead of left, set
   S5_SERVO_LEFT_HIGH to 0 (the Stage 3 test shows it). */
#define S5_SERVO_US_PER_DEG 11 /* bring-up nominal 11.1 */
#define S5_SERVO_LEFT_HIGH  1
/* Servo travel: never more than this either side of straight ahead (the
   wires at the back limit it), and at most S5_SERVO_DEG_STEP degrees per
   20 ms servo frame (full speed broke the horn, Stage 3 run, 2 Nov).
   The scan covers the whole travel in 5 degree rays. */
/* 25 never saw past a box's edge (the sonar beam is about 30 deg wide);
 * route the cables for 40 */
#define S5_SERVO_HALF_DEG 40
#define S5_SERVO_DEG_STEP 2 /* = 100 deg/s */
/* Side look (8 Nov): on the leg past the box the sonar turns this far
   toward the box and listens for its side face, so the robot knows when
   it is really past (any depth) instead of assuming a 150 mm deep box.
   40 = the cables' limit (90, square to the face, has no clearance); the
   side of the beam still hears the face (task_mission.c pass_box).
   0 = no side look (the fixed 150 mm).
   OFF since 8 Nov: on the robot the side face was not heard at 40 deg
   (bench check mostly -1; in a bypass only the box's front corner while
   approaching), and the obstacle is 9 x 13 cm (at most 130 mm deep,
   inside the fixed 150 mm).  40 turns it back on. */
#define S5_SERVO_SIDE_DEG 0
/* sonar in front of the wheel axle (NOT MEASURED: measure and fix) */
#define S5_SONAR_TO_AXLE_MM 120
/* Sonar beam: each edge is seen this many degrees early (the cone),
   so edges are pulled in by it.  Tune with S5-O01 known widths. */
#define S5_BEAM_HALF_DEG 6

/* Sonar results: mm, or SONAR_OPEN (echo came back too late or not at
   all after rising: nothing in range), or SONAR_FAULT (ECHO never rose:
   sensor or wiring fault, never treated as open space). */
#define SONAR_OPEN  (-1)
#define SONAR_FAULT (-2)

IMPORT void obstacle_watch (BOOL on); /* forward watch during line following */
IMPORT void obstacle_idle_ping (BOOL on); /* mission image: ping for the panel
                         while Mission is IDLE (never during manoeuvres) */
IMPORT BOOL obstacle_sensor_fault (void); /* 5 pings in a row without ECHO */
IMPORT INT  obstacle_front_mm (void);     /* last forward reading, -1 = none */

typedef struct
{
    BOOL b_found;      /* an obstacle straight ahead */
    INT  front_mm;     /* distance straight ahead */
    INT  left_mm;      /* left edge, mm left of the robot's centre line */
    INT  right_mm;     /* right edge, mm (negative = right of centre) */
    INT  width_mm;     /* left - right */
    INT  unc_mm;       /* +/- uncertainty */
    BOOL b_left_clear; /* open space beyond the left edge */
    BOOL b_right_clear;
    BOOL b_left_seen; /* the scan saw past the left edge (FALSE:
               the obstacle filled the scan that side,
               edge and clear unknown) */
    BOOL b_right_seen;
    INT  left_unc; /* +/- of each edge */
    INT  right_unc;
    INT  rays;       /* rays scanned */
    INT  fault_rays; /* rays where the sensor did not answer */
} obstacle_profile_t;

/* Blocking: called by Mission (or a test) while the robot stands still.
   Turns the servo, settles, 5 pings per ray, median.  About 7 s. */
IMPORT void obstacle_scan (obstacle_profile_t * p);
/* Combine two scans (O5): mean width, uncertainty includes their spread */
IMPORT void obstacle_combine (const obstacle_profile_t * a,
                              const obstacle_profile_t * b,
                              obstacle_profile_t *       out);
IMPORT void servo_set_angle (INT deg);  /* 90 = ahead, clamped to 90 +/-
                           the larger of S5_SERVO_HALF_DEG and
                           S5_SERVO_SIDE_DEG; blocks while it
                           turns (slowly) */
IMPORT void servo_look_side (INT side); /* +1 left, -1 right, 0 ahead:
                           S5_SERVO_SIDE_DEG that way */
IMPORT INT
sonar_ping_mm (void); /* blocking 60 ms; mm, SONAR_OPEN or SONAR_FAULT */
/* The same in two halves, so the caller can do other things in the 60 ms:
   start, then read at least 60 ms later */
IMPORT void sonar_ping_start (void);
IMPORT INT  sonar_ping_result (void);

/* ---- Mission ------------------------------------------------------- */
IMPORT const char * mission_state_name (void);
/* Mission reacts to START only when armed: always in the normal image;
   in a test image only after the test arms it (so tests can use START). */
IMPORT void mission_arm (BOOL on);
/* Which parts Mission uses (all TRUE in the normal image).  Stage tests
   switch off the parts not tested yet, so a Stage 2 run does not depend
   on the untested obstacle (Stage 3) or IMU (Stage 4) code. */
IMPORT void mission_features (BOOL obstacles, BOOL imu);

/* Private to subsystem 5, shared between its own .c files */
IMPORT ID   gh_tid_mission;
IMPORT INT  g_tmg_mission;
IMPORT UW   g_mission_pings; /* written only by the Mission task */
IMPORT ID   gh_tid_obstacle;
IMPORT ID   gh_cyc_obstacle;
IMPORT INT  g_tmg_obstacle;
IMPORT void mission_task (INT stacd, void * exinf);
IMPORT void obstacle_task (INT stacd, void * exinf);
IMPORT ER   obstacle_hw_init (void);

#endif /* SUBSYSTEM5_H */

/*** end of file ***/

