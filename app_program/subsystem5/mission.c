/**
 * @file    mission.c
 *  Subsystem 5 -- the Mission Controller task.
 *
 *  Waits on gh_flg_mission (100 ms timeout) and runs the course:
 *    IDLE      START button (pressed and released) or MQTT start
 *    CAL       IMU level calibration, 1 s, robot still
 *    RUN       follow the line at the barcode speed; react to:
 *      barcode   store the latest mapped character (FR11, TEAM.md 7.1
 *                B5); one with no mapping is reported and ignored (20
 *                Nov; was a stop at the junction, B7)
 *      junction  do the mapped move for the stored character, U-turn
 *                included (B6); no character = straight
 *      obstacle  stop, scan twice from two positions, report width +/-,
 *                bypass on the clear side, find the line again (7.2)
 *      line lost stop and report
 *    STOP (button or MQTT) at any time: motors off at once (in the
 *    interrupt), Mission goes back to IDLE.
 *  Manoeuvres are run here step by step; each step waits for Motion's
 *  result and gives up as soon as a STOP reason is set.
 */

#include "subsystem5.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"
#include "mission.h"

/* Mission's speed: line following and the bypass legs (mm/s).  Was
   S2_SPEED_BARCODE (120, "slow everywhere").  15 Nov raised to 135
   (team: a bit faster), 18 Nov back to 120 before the demo: 120 is the
   only speed with course runs behind it (7 runs, no line lost; run 7 the
   whole sequence), 135 never ran, and 150 lost tight left curves on 7 Nov
   (before the steering was retuned).  Over bars it still slows to
   S2_SPEED_IN_BARCODE, after a barcode to S5_APPROACH_SPEED. */
#define S5_FOLLOW_SPEED 120
/* barcode read, its junction not yet: slower, so weaving is less likely and
 * the junction is not overrun */
#define S5_APPROACH_SPEED 100
/* Scan positions (O5).  First scan where the forward watch stopped (about
   16 cm from the obstacle since 20 Nov, S5_TRIP_MM 180; backs up only
   if nearer than SCAN_NEAR_MM), then,
   only if that scan missed an edge (15 Nov), back REVERSE_MM and scan
   again.  Until 8 Nov it first backed up to 350
   mm, because from 20-25 cm the old width rule saw the box across the
   whole sweep.  With the geometry rule (obstacle.c) the near scans
   are the better ones: on the 29 real scans of 7 Nov, 200-280 mm away
   read 138-240 mm for 14.5-23 cm boxes, 340-380 mm away 206-293, 450+
   away 225-346; every pair of scans read narrower from nearer.  And it
   backs up 100 mm instead of about 210. */
#define SCAN_NEAR_MM 160 /* 15 Nov: 220 -> 160 (S5_TRIP_MM then 190) */
#define REVERSE_MM   100 /* second scan position */
#define D_MAX_MM     150 /* deepest expected obstacle (A9) */
/* 50 until 7 Nov, then 70: tools/sim/bypasssim.py with +-5 % spin error
 * passed a 200 mm box 21 mm away at 70, 42 mm at 100 */
#define MARGIN_MM        100
#define REJOIN_ENABLE_MM 60 /* line events back on after this */
/* after a bypass the barcode sensor is ignored this far: the robot is
   still turning onto its line and B can cross the line's own tape (20 Nov,
   track 4: read as bars, the steering was held straight, line lost) */
#define REJOIN_BAR_MM 200
/* back toward the line: give up this far past where it should be */
#define REJOIN_FAR_MM 400
#define STILL_TICKS   100 /* 1 s at 10 ms */
#define IMU_CAL_TICKS 200 /* 2 s at 10 ms */
/* each wheel, mm/s: slow, so it stops on the 20 mm tape */
#define SPIN_SPEED 40

#define M_IDLE      0
#define M_CAL       1
#define M_RUN       2
#define M_MANOEUVRE 3

/* Mission's state (Change Requirement 1): one word, written only by the
   mission task; other tasks read the state through mission_state(). */
typedef struct
{
    uint32_t state   APP_FIELD(2); /* M_IDLE .. M_MANOEUVRE */
    uint32_t pending APP_FIELD(7); /* latest unconsumed barcode letter,
                                      7-bit ASCII, 0 = none */
} mission_word_t;

/* What other tasks set (a test arms Mission and picks its parts): one word,
   changed only under APP_LOCK */
typedef struct
{
    bool b_armed         APP_FIELD(1);
    bool b_use_obstacles APP_FIELD(1);
    bool b_use_imu       APP_FIELD(1);
} mission_flags_t;

LOCAL volatile mission_word_t  g_mw    = {M_IDLE, 0u};
LOCAL volatile mission_flags_t g_flags = {!APP_TEST, true, true};
LOCAL uint32_t g_bar_seen; /* barcode_seq() already handled: the same
                              32-bit counter */
/* A junction waits at most this far past its centre for a barcode that
   is still being read (see on_junction) */
#define JUNC_WAIT_MM 150

EXPORT void mission_features (bool b_obstacles, bool b_imu)
{
    uint32_t irq_state;

    APP_LOCK(irq_state);
    g_flags.b_use_obstacles = b_obstacles;
    g_flags.b_use_imu       = b_imu;
    APP_UNLOCK(irq_state);
}

/** Turns the forward obstacle watch on, if this build uses it. */
LOCAL void watch_on (void)
{
    obstacle_watch(g_flags.b_use_obstacles);
}

EXPORT void mission_arm (bool b_on)
{
    uint32_t irq_state;

    APP_LOCK(irq_state);
    g_flags.b_armed = b_on;
    APP_UNLOCK(irq_state);
}

/** The mission state: M_IDLE, M_CAL, M_RUN or M_MANOEUVRE (0..3). */
EXPORT uint32_t mission_state (void)
{
    return g_mw.state;
}

/** true in M_MANOEUVRE: a junction turn or an obstacle bypass. */
EXPORT bool mission_manoeuvring (void)
{
    return M_MANOEUVRE == g_mw.state;
}

EXPORT const char * mission_state_name (void)
{
    static const char * names[] = {"IDLE", "CAL", "RUN", "MANOEUVRE"};
    return names[g_mw.state];
}

/** Sets the mission state and tells the panel. */
LOCAL void set_state (int32_t new_state)
{
    g_mw.state = new_state;
    obstacle_idle_ping(M_IDLE == new_state);
}

/** Sends a text event to the panel. */
LOCAL void say (const uint8_t * p_text)
{
    telemetry_event(p_text);
}

/** Wait for a Motion command; false if it did not finish (STOP, timeout) */
LOCAL bool mv (ID h_cmd)
{
    motion_status_t status;

    if (h_cmd < E_OK)
    {
        return false;
    }

    for (;;)
    {
        motion_get_status(&status);

        if ((status.h_cmd_id == h_cmd) && (status.result != MOT_RES_BUSY))
        {
            return MOT_RES_DONE == status.result;
        }

        if (app_stop_reasons() != 0u)
        {
            return false;
        }

        tk_dly_tsk(10);
    }
}

/** Waits until both wheels have stopped (at most 1 s). */
LOCAL void wait_still (void)
{
    motion_status_t status;
    int32_t         tick;

    for (tick = 0; tick < STILL_TICKS; tick++)
    {
        motion_get_status(&status);

        if ((0 == status.speed_l) && (0 == status.speed_r))
        {
            break;
        }

        tk_dly_tsk(10);
    }
}

/** Stops the robot, turns the watches off and reports why. */
LOCAL void halt (const char * p_why)
{
    uint8_t msg[TELE_TEXT_MAX];

    motion_halt();
    motion_follow_limit(0);
    obstacle_watch(false);
    line_set_enabled(false);
    set_state(M_IDLE);
    tm_sprintf(msg, app_str("stopped: %s (press START to go again)"), p_why);
    say(msg);
}

/** Follow again; line events back on after a short distance */
LOCAL bool resume_follow (void)
{
    int32_t odo0 = motor_odometer();

    barcode_reset(); /* a spin can leave a half-read code */

    if (motion_follow(S5_FOLLOW_SPEED) < E_OK)
    {
        return false;
    }

    while (motion_counts_to_mm(motor_odometer() - odo0) < REJOIN_ENABLE_MM)
    {
        if (app_stop_reasons() != 0u)
        {
            return false;
        }

        tk_dly_tsk(10);
    }

    line_set_enabled(true);
    watch_on();
    tk_clr_flg(gh_flg_mission, ~(MSN_FLG_JUNCTION | MSN_FLG_LINE_LOST |
                                 MSN_FLG_OBSTACLE | MSN_FLG_BARCODE));
    return true;
}

/** Starts a run: calibrates the IMU, clears old results, follows the line. */
LOCAL void start_run (void)
{
    int32_t tick;

    say(app_str("start"));
    /* which barcode thresholds this image has (15 Nov: changed in the day) */
    tm_printf(app_str("[S3] barcode: black from %d, step %d\n"),
              S3_BAR_BLACK_MIN, S3_BAR_DELTA);
    set_state(M_CAL);
    motion_halt();

    if ((g_flags.b_use_imu) && (imu_ok()))
    {
        imu_calibrate();

        for (tick = 0; (tick < IMU_CAL_TICKS) && (!imu_calibrated()); tick++)
        {
            tk_dly_tsk(10);
        }
    }

    g_mw.pending = 0;
    motion_follow_limit(0);
    barcode_reset();            /* nothing left over from before START */
    g_bar_seen = barcode_seq(); /* ... nor its last result */
    tk_clr_flg(gh_flg_mission, 0);

    if (motion_follow(S5_FOLLOW_SPEED) < E_OK)
    {
        halt("STOP still set");
        return;
    }

    line_set_enabled(true);
    watch_on();
    set_state(M_RUN);
}

/** Handles a new barcode result: stores its letter, reports the move. */
LOCAL void on_barcode (void)
{
    barcode_result_t result;
    uint8_t          msg[TELE_TEXT_MAX];
    int32_t          move;

    barcode_last(&result);

    if (result.seq == g_bar_seen)
    {
        return; /* handled already */
    }

    g_bar_seen = result.seq;

    if (!result.b_ok)
    {
        int32_t dbg_count;
        int32_t dbg_widths[BARCODE_DBG_MAX];
        int32_t kdx;

        tm_sprintf(msg, app_str("barcode READ_FAIL (%d elements)"),
                   result.elements);
        say(msg);
        /* the widths to the log, so a failed read can be looked at
           (8 Nov: a 27-element failure had nothing to go on) */
        (void)barcode_debug(&dbg_count, dbg_widths, BARCODE_DBG_MAX);
        tm_printf(app_str("[S3] READ_FAIL widths (mm):"));

        for (kdx = 0; (kdx < dbg_count) && (kdx < BARCODE_DBG_MAX); kdx++)
        {
            tm_printf(app_str(" %d.%d"), dbg_widths[kdx] / 10,
                      dbg_widths[kdx] % 10);
        }

        tm_printf(app_str("\n"));
        return;
    }

    /* A letter with no mapping is reported and ignored (20 Nov, team:
       carry on instead of stopping at the next junction; it does not
       replace a mapped letter still waiting for its junction either) */
    move = barcode_move_for(result.letter);

    if (move != MOVE_NONE)
    {
        g_mw.pending = result.letter;
    }

    /* a turn is coming: approach its junction slowly */
    motion_follow_limit(
        ((MOVE_LEFT == move) || (MOVE_RIGHT == move) || (MOVE_UTURN == move))
            ? S5_APPROACH_SPEED
            : 0);
    tm_sprintf(msg,
               app_str("barcode %c%s%s (narrow %d.%d mm, wide %d.%d "
                       "mm) -> %s"),
               result.letter, result.b_reversed ? " read backwards" : "",
               (BAR_FIX_STAR == result.fixed)     ? " (one * damaged)"
               : (BAR_FIX_MERGED == result.fixed) ? " (a lost bar put back)"
               : (BAR_FIX_CUT == result.fixed) ? " (a missing end bar put back)"
               : (BAR_FIX_THIN == result.fixed) ? " (bars read thin, widened)"
                                                : "",
               result.narrow_mm10 / 10, result.narrow_mm10 % 10,
               result.wide_mm10 / 10, result.wide_mm10 % 10,
               (MOVE_LEFT == move)      ? "LEFT at next junction"
               : (MOVE_RIGHT == move)   ? "RIGHT at next junction"
               : (MOVE_FORWARD == move) ? "FORWARD at next junction"
               : (MOVE_UTURN == move)   ? "U-TURN at next junction"
                                        : "NO MAPPING, ignored");
    say(msg);
}

/* Junction turns end on the new line (8 Nov): a U-turn came out about
   190 deg (spins run a few % over on this floor; Stage 1: 180 about 5
   over).  Spin at the normal speed to TURN_SEARCH_DEG short of the move,
   then slowly (SPIN_SPEED) until a line sensor touches the tape (at most
   TURN_SEARCH_DEG past the move), then TURN_CENTRE_DEG more: the sensors
   straddle the tape, so the first touch is at the leading sensor, about
   12 deg (15 mm sensor offset + half the tape, at 115 mm) before the
   tape is centred.  No tape found: back to the planned angle and follow
   (the line task looks for it).  At an X the side arms are 90 deg away,
   so the search (from 30 short) meets the arm it is turning to first. */
#define TURN_SEARCH_DEG 30
#define TURN_CENTRE_DEG 12
LOCAL int32_t spin_line (int32_t deg, int32_t ignore);

/** A junction turn that ends on the new line (fast, then slowly until a
 * sensor finds it). */
LOCAL bool turn_onto_line (int32_t deg)
{
    int32_t sgn   = (deg > 0) ? 1 : -1;
    int32_t angle = deg * sgn;
    int32_t found;

    if (!mv(motion_turn(sgn * (angle - TURN_SEARCH_DEG))))
    {
        return false;
    }

    found = spin_line(sgn * 2 * TURN_SEARCH_DEG, 0);

    if (found < 0)
    {
        return false;
    }

    if (found)
    {
        return mv(motion_turn(sgn * TURN_CENTRE_DEG));
    }

    say(app_str("turn: no tape found, back to the planned angle"));
    return mv(motion_turn(-sgn * TURN_SEARCH_DEG));
}

/** Handles a junction: waits for a barcode still being read, then does the
 * move. */
LOCAL void on_junction (void)
{
    uint8_t msg[TELE_TEXT_MAX];
    uint8_t letter;
    int32_t move;
    int32_t deg;
    int32_t sides = line_junction_sides();
    int32_t to_go;
    int32_t junc_odo = line_junction_odo();

    /* A barcode just before the junction may still be being read: B is
       40 mm behind the line sensors, and a code only ends after 60 mm
       with no bar (40 until 20 Nov), so its letter comes about 100 mm
       after the line sensors passed its last bar.  A junction closer than
       that came first: no letter yet, straight on, and the letter was
       used at the NEXT junction.  Wait for the read (the robot keeps following), at most
       JUNC_WAIT_MM past the centre; the move below measures from the
       centre, so it backs up if it went past. */

    while ((barcode_busy()) &&
           (motion_counts_to_mm(motor_odometer() - junc_odo) < JUNC_WAIT_MM))
    {
        if (app_stop_reasons() != 0u)
        {
            return;
        }

        tk_dly_tsk(10);
    }

    on_barcode(); /* a letter decoded meanwhile (once) */
    letter = g_mw.pending;
    move   = (letter != 0) ? barcode_move_for(letter) : MOVE_FORWARD;

    motion_follow_limit(0);

    if ((letter != 0) && (MOVE_NONE == move))
    {
        /* mapped when it was read, but a new mapping came meanwhile
           without it: straight on, as for no barcode (20 Nov: no longer a
           stop) */
        move = MOVE_FORWARD;
    }

    g_mw.pending = 0;
    /* junctions since boot, a handful: fit %d */
    tm_sprintf(msg, app_str("junction %d (%s): %s (barcode %c)"),
               (int32_t)line_junctions(),
               ((sides & JUNC_SIDE_S) != 0) ? "crossing" : "branch left",
               (MOVE_LEFT == move)    ? "LEFT"
               : (MOVE_RIGHT == move) ? "RIGHT"
               : (MOVE_UTURN == move) ? "U-TURN"
                                      : "FORWARD",
               (letter != 0) ? letter : '-');
    say(msg);

    if (MOVE_FORWARD == move)
    {
        return;
    }

    set_state(M_MANOEUVRE);
    line_set_enabled(false);
    obstacle_watch(false);
    deg = (MOVE_LEFT == move) ? 90 : (MOVE_RIGHT == move) ? -90 : 180;
    /* wheels over the junction centre, then spin.  The line task says
       where the line sensors crossed the centre: a left branch is
       reported later than a crossing (B is behind them), so measure
       from there. */
    to_go = S3_SENSOR_TO_AXLE_MM -
            motion_counts_to_mm(motor_odometer() - line_junction_odo());

    if (((to_go > 0) && (!mv(motion_drive(S5_FOLLOW_SPEED, to_go)))) ||
        ((to_go < 0) && (!mv(motion_drive(-S5_FOLLOW_SPEED, -to_go)))) ||
        (!turn_onto_line(deg)) || (!resume_follow()))
    {
        halt("junction move did not finish");
        return;
    }

    set_state(M_RUN);
}

/** Check the way ahead is clear for dist_mm before driving it */
LOCAL bool ahead_clear (int32_t dist_mm)
{
    int32_t dist = obstacle_ping_mm();

    return (SONAR_OPEN == dist) ||
           ((dist >= 0) &&
            (dist > (dist_mm + MARGIN_MM))); /* fault = not clear */
}

/* The bypass legs' forward watch (15 Nov): the scan sees only +-40 deg,
   so something beside the box, or that the scan could not hear, may be in
   a leg's way.  While a leg drives, the sonar pings ahead every 60 ms;
   WATCH_CLOSE_PINGS in a row nearer than WATCH_STOP_MM (or no ECHO: the
   sensor is not working) stop the robot.  The box itself is beside the
   robot on these legs, outside the beam. */
#define WATCH_STOP_MM     150
#define WATCH_CLOSE_PINGS 2
#define WATCH_PERIOD_US   60000u /* one ping's echo time */
#define DRIVE_BLOCKED     (-2)   /* drive_until_line: the watch stopped it */

/* one leg's forward watch: written only by the Mission task */
typedef struct
{
    uint32_t t_ping; /* when the ping now out was started */
    int32_t  close;  /* pings in a row nearer than WATCH_STOP_MM */
} leg_watch_t;

/** Starts a leg's forward watch (the servo points ahead). */
LOCAL void leg_watch_start (leg_watch_t * p_watch)
{
    obstacle_ping_start();
    p_watch->t_ping = app_timing_now_us();
    p_watch->close  = 0;
}

/** Called often while a leg drives: true = something is in the way. */
LOCAL bool leg_watch_blocked (leg_watch_t * p_watch)
{
    int32_t dist;

    if ((app_timing_now_us() - p_watch->t_ping) < WATCH_PERIOD_US)
    {
        return false;
    }

    dist = obstacle_ping_result();
    obstacle_ping_start();
    p_watch->t_ping = app_timing_now_us();

    if ((SONAR_FAULT == dist) || ((dist >= 0) && (dist < WATCH_STOP_MM)))
    {
        p_watch->close++;
    }
    else
    {
        p_watch->close = 0;
    }

    return p_watch->close >= WATCH_CLOSE_PINGS;
}

/** Drive dist_mm straight with the forward watch on; stop as soon as a
   line sensor sees black, but only after from_mm (more than dist_mm: never).
   1 = line found, 0 = drove it all, -1 = failed, DRIVE_BLOCKED = the watch
   stopped it (halted, reason given) */
LOCAL int32_t drive_until_line (int32_t dist_mm, int32_t from_mm)
{
    int32_t     odo0 = motor_odometer();
    leg_watch_t watch;

    if (motion_drive(S5_FOLLOW_SPEED, dist_mm) < E_OK)
    {
        return -1;
    }

    leg_watch_start(&watch);

    for (;;)
    {
        motion_status_t status;

        if (app_stop_reasons() != 0u)
        {
            return -1;
        }

        if (leg_watch_blocked(&watch))
        {
            halt("something in the way during the bypass");
            return DRIVE_BLOCKED;
        }

        if ((line_any_black()) &&
            (motion_counts_to_mm(motor_odometer() - odo0) >= from_mm))
        {
            motion_halt();
            wait_still();
            return 1;
        }

        motion_get_status(&status);

        if (status.result != MOT_RES_BUSY)
        {
            return (MOT_RES_DONE == status.result) ? 0 : -1;
        }

        tk_dly_tsk(5);
    }
}

#define SIDE_TOL_IN_MM  120 /* echo from lat_exp - this ... */
#define SIDE_TOL_OUT_MM 80  /* ... to lat_exp + this = the box */
#define SIDE_MISS_PINGS 5   /* no echo in this many = not beside */
#define SIDE_EXTRA_MM   300 /* at most this past x_pass */

/* the side look's record of the box on the leg past it (pass_box) */
typedef struct
{
    bool    b_seen;  /* the box heard at all */
    bool    b_told;  /* the panel told what the side look found */
    int32_t first;   /* mm along the leg where the box was first heard */
    int32_t last;    /* ... and where it was last heard */
    int32_t miss;    /* side pings in a row without the box */
    int32_t stop_at; /* the leg ends here, mm along */
} side_look_t;

/** One side-look ping, done mm along the leg: an echo near lat_exp (or a
   sensor fault) is the box, and the leg goes on at least until the robot
   is past where it was heard. */
LOCAL void side_ping (side_look_t * p_look, int32_t sonar_mm, int32_t lat_exp,
                      int32_t done)
{
    if ((SONAR_FAULT == sonar_mm) ||
        ((sonar_mm >= (lat_exp - SIDE_TOL_IN_MM)) &&
         (sonar_mm <= (lat_exp + SIDE_TOL_OUT_MM))))
    {
        int32_t past =
            done + S5_SONAR_TO_AXLE_MM + (S2_ROBOT_WIDTH_MM / 2) + MARGIN_MM;

        if (!p_look->b_seen)
        {
            p_look->first = done;
        }

        p_look->b_seen = true;
        p_look->last   = done;
        p_look->miss   = 0;

        if (past > p_look->stop_at)
        {
            p_look->stop_at = past;
        }
    }
    else
    {
        p_look->miss++;
    }
}

/** At x_pass along the leg, once: tells the panel what the side look
   found. */
LOCAL void side_report (side_look_t * p_look, int32_t done, int32_t x_pass)
{
    if ((!p_look->b_told) && (done >= x_pass))
    {
        p_look->b_told = true;

        if (!p_look->b_seen)
        {
            say(app_str("side look heard nothing: assuming "
                        "the box is at most 150 mm deep"));
        }
        else if (p_look->stop_at > x_pass)
        {
            say(app_str("side look: box still beside, going on"));
        }
        else
        {
            /* past the box already: nothing to say */
        }
    }
}

/** Leg 4, past the box (8 Nov).  With the side look the sonar turns 40 deg
   (the cables' limit) toward the box and pings every 60 ms while the
   robot drives.  At 40 deg the beam's axis meets the box's side face at a
   glancing angle (a flat face mostly reflects that away), but the side of
   the beam hears the face at its square-on distance while the sonar is
   beside it (the 7 Nov scans: 40 deg off a face still read its square-on
   distance).  So an echo near lat_exp, the sideways distance to the box's
   side face from the scan (SIDE_TOL_IN/OUT_MM), counts as the box.

   Each such echo pushes the end of the leg to at least where the axle is
   half the robot's width + MARGIN_MM past the point where it was heard;
   never earlier than the fixed distance for a box up to 150 mm deep
   (x_pass, D_MAX_MM, team assumption A9).  So the side look can only make
   the leg longer: a stray echo makes it longer, a missed one changes
   nothing, a side look that hears nothing is the fixed distance.  (Asking
   "still beside it at x_pass?" was wrong: the sonar is 120 mm ahead of
   the axle and had just passed a 350 mm deep box's back there.)  The line
   sensors count from past a 150 mm deep box and not while the box is
   beside (an echo in the last SIDE_MISS_PINGS): a junction's side line
   beside the box is not the line (8 Nov run).  A sonar fault counts as an
   echo.  tools/sim/bypasssim.py S: box depths 60-350 mm all 36/36, closest 56
   mm (fixed distance: 250 mm deep 16 of 36 hit, 350 mm all hit); with
   half the side echoes missed still 53/54.
   No line is taken on this leg (15 Nov): on the course a different piece
   of track ran beside the box's far side and the robot followed it
   ("line met behind the box, 503 mm along") instead of going round and
   back to its own line.  The robot always drives the full leg, turns
   back and finds its own line (leg 5-6).
   0 = past the box (turn back to the line), -1 = halted */
LOCAL int32_t pass_box (int32_t sgn, int32_t lat_exp, int32_t x_pass)
{
    uint8_t         msg[TELE_TEXT_MAX];
    bool            b_side = (S5_SERVO_SIDE_DEG > 0);
    side_look_t     look   = {false, false, 0, 0, SIDE_MISS_PINGS, x_pass};
    int32_t         dist;
    int32_t         done;
    int32_t         sonar_mm;
    int32_t         outcome = -1;
    uint32_t        t_ping;
    int32_t         odo0;
    motion_status_t status;
    leg_watch_t     watch;

    /* how far it may go: the way ahead must be clear */
    dist     = x_pass + (b_side ? SIDE_EXTRA_MM : 0);
    sonar_mm = obstacle_ping_mm();

    if ((SONAR_FAULT == sonar_mm) ||
        ((sonar_mm >= 0) && ((sonar_mm - MARGIN_MM) < x_pass)))
    {
        halt("bypass leg 4 blocked");
        return -1;
    }

    if ((sonar_mm >= 0) && ((sonar_mm - MARGIN_MM) < dist))
    {
        dist = sonar_mm - MARGIN_MM;
    }

    if (b_side)
    {
        obstacle_look_side(-sgn); /* the box is on the other side */
    }

    odo0 = motor_odometer();

    if (motion_drive(S5_FOLLOW_SPEED, dist) < E_OK)
    {
        goto out;
    }

    leg_watch_start(&watch); /* the side look uses the same pings */
    t_ping = watch.t_ping;

    for (;;)
    {
        if (app_stop_reasons() != 0u)
        {
            goto out;
        }

        done = motion_counts_to_mm(motor_odometer() - odo0);

        if (b_side && ((app_timing_now_us() - t_ping) >= WATCH_PERIOD_US))
        {
            sonar_mm = obstacle_ping_result();
            obstacle_ping_start();
            t_ping = app_timing_now_us();
            side_ping(&look, sonar_mm, lat_exp, done);
            side_report(&look, done, x_pass);
        }
        else if ((!b_side) && (leg_watch_blocked(&watch)))
        {
            /* the sonar points ahead: the forward watch (15 Nov) */
            motion_halt();
            halt("something in the way during the bypass");
            return -1;
        }
        else
        {
            /* nothing to do for any other case */
        }

        if (done >= look.stop_at)
        {
            outcome = 0;
            break;
        }

        motion_get_status(&status);

        if (status.result != MOT_RES_BUSY)
        {
            if (status.result != MOT_RES_DONE)
            {
                goto out;
            }

            if (!b_side)
            {
                outcome = 0;
                break;
            } /* drove x_pass */

            motion_halt();
            obstacle_servo_angle(90);
            halt((dist < (x_pass + SIDE_EXTRA_MM))
                     ? "box still beside, no room ahead to get past it"
                     : "still beside the box 300 mm on (box too long?)");
            return -1;
        }

        tk_dly_tsk(5);
    }

    motion_halt();
    wait_still();

    if (b_side)
    {
        obstacle_servo_angle(90);

        if (look.b_seen)
        {
            tm_sprintf(msg,
                       app_str("side look: box heard from %d to %d "
                               "mm along, leg %d mm (fixed %d)"),
                       look.first, look.last, done, x_pass);
            say(msg);
        }
    }

    return outcome;
out:
    motion_halt();

    if (b_side)
    {
        obstacle_servo_angle(90);
    }

    halt("bypass step did not finish");
    return -1;
}

/** Spin slowly by up to deg; stop when a line sensor sees black after
   ignore degrees.  1 = found, 0 = turned it all, -1 = failed */
LOCAL int32_t spin_line (int32_t deg, int32_t ignore)
{
    motion_status_t status;
    int32_t         left_start;
    int32_t         right_start;

    motion_get_status(&status);
    left_start  = status.count_l;
    right_start = status.count_r;

    if (motion_turn_at(deg, SPIN_SPEED) < E_OK)
    {
        return -1;
    }

    for (;;)
    {
        int32_t wheel_diff;
        int32_t turned;

        if (app_stop_reasons() != 0u)
        {
            return -1;
        }

        motion_get_status(&status);
        wheel_diff =
            (status.count_r - right_start) - (status.count_l - left_start);

        if (wheel_diff < 0)
        {
            wheel_diff = -wheel_diff;
        }

        /* counts -> mm of wheel difference -> degrees over the track */
        turned = (int32_t)((wheel_diff * S2_UM_PER_COUNT / 1000) * 573 /
                           (S2_TURN_TRACK_MM * 10));

        if ((turned >= ignore) && (line_any_black()))
        {
            motion_halt();
            wait_still();
            return 1;
        }

        if (status.result != MOT_RES_BUSY)
        {
            return (MOT_RES_DONE == status.result) ? 0 : -1;
        }

        tk_dly_tsk(5);
    }
}

/* Before the leg back to the line (15 Nov): the box is assumed at most
   D_MAX_MM deep.  Once turned toward the line, the sonar faces the box's
   side square-on if the box is longer (a strong echo); then it turns
   back, goes EXTEND_MM further along the box and looks again, at most
   EXTEND_TRIES times.  Turning on the spot beside the box is safe: the
   robot's centre is y_off - edge >= half its width + MARGIN_MM (185 mm)
   from the box's side, its corners swing at most about 155 mm. */
#define EXTEND_MM    100
#define EXTEND_TRIES 4

/** Facing the line, y_off away: true = nothing nearer than the line
   (a sensor fault counts as something). */
LOCAL bool line_side_clear (int32_t y_off)
{
    int32_t dist = obstacle_ping_mm();

    return (SONAR_OPEN == dist) ||
           ((dist >= 0) && (dist >= (y_off - S5_SONAR_TO_AXLE_MM)));
}

/** Turns toward the line (-sgn) and makes sure the box is not still
   beside; goes further along it if it is.  true = facing the line, the way
   clear; false = halted, reason given. */
LOCAL bool turn_back_clear (int32_t sgn, int32_t y_off)
{
    int32_t tries;
    int32_t result;

    for (tries = 0;; tries++)
    {
        if (!mv(motion_turn(-sgn * 90)))
        {
            halt("bypass step did not finish");
            return false;
        }

        wait_still();

        if (line_side_clear(y_off))
        {
            return true;
        }

        if (tries >= EXTEND_TRIES)
        {
            halt("obstacle longer than expected (still beside it 400 mm "
                 "on)");
            return false;
        }

        say(app_str("box still beside: going 100 mm further along it"));

        if (!mv(motion_turn(sgn * 90)))
        {
            halt("bypass step did not finish");
            return false;
        }

        result = drive_until_line(EXTEND_MM, EXTEND_MM + 1);

        if (result != 0)
        {
            if (result != DRIVE_BLOCKED)
            {
                halt("bypass step did not finish");
            }

            return false; /* DRIVE_BLOCKED: halted, reason given */
        }
    }
}

/** Degrees turned since p_before (+ = left), from the wheel counts.
   Spins read a few degrees under on the wheels (S2-M02), so this is a
   guide for the log, not a measurement. */
LOCAL int32_t heading_deg (const motion_status_t * p_before)
{
    motion_status_t now;

    motion_get_status(&now);
    /* a few thousand counts: exact as floats; the degrees fit int32_t */
    return (int32_t)((float32_t)((now.count_r - p_before->count_r) -
                                 (now.count_l - p_before->count_l)) *
                     S2_UM_PER_COUNT / 1000.0f / S2_TURN_TRACK_MM * 57.2958f);
}

/** Handles an obstacle ahead: scans its width, drives round it, finds the
 * line. */
LOCAL void on_obstacle (void)
{
    obstacle_profile_t scan_near;
    obstacle_profile_t scan_far;
    obstacle_profile_t profile;
    uint8_t            msg[TELE_TEXT_MAX];
    int32_t            sgn;
    int32_t            w_plan;
    int32_t            centre;
    int32_t            edge_meas;
    int32_t            edge_plan;
    int32_t            y_off;
    int32_t            x_pass;
    int32_t            found;
    int32_t            odo0;
    motion_status_t    before; /* the wheel counts before the box */

    set_state(M_MANOEUVRE);
    motion_halt();
    motion_get_status(&before);
    obstacle_watch(false);
    line_set_enabled(false);
    /* stopped in the middle of a barcode: the backing up would run B
       back over the bars and garble it; drop that read, say so */

    if (barcode_busy())
    {
        barcode_reset();
        say(app_str("obstacle while reading a barcode: that read is "
                    "dropped"));
    }

    wait_still();

    if (obstacle_sensor_fault())
    {
        halt("ultrasonic not answering (no ECHO), not driving blind");
        return;
    }

    tm_sprintf(msg, app_str("obstacle ahead at %d mm: scanning"),
               obstacle_front_mm());
    say(msg);

    /* first scan from here, unless it stopped nearer than SCAN_NEAR_MM */
    {
        int32_t dist = obstacle_ping_mm();
        int32_t back = SCAN_NEAR_MM - ((dist >= 0) ? dist : S5_TRIP_MM);

        if (back > 10)
        {
            if (!mv(motion_drive(-S5_FOLLOW_SPEED, back)))
            {
                halt("reverse for the 1st scan did not finish");
                return;
            }

            wait_still();
        }
    }
    obstacle_scan(&scan_near);

    if (scan_near.fault_rays > (scan_near.rays / 2))
    {
        halt("ultrasonic not answering during the scan");
        return;
    }

    if (!scan_near.b_found)
    {
        say(app_str("scan found nothing ahead, going on"));

        if (!resume_follow())
        {
            halt("could not restart");
        }
        else
        {
            set_state(M_RUN);
        }

        return;
    }

    /* Both edges seen from here: this scan is the profile.  The 2nd scan
       is for a box that fills this one (an edge not seen); where both see
       an edge the nearer is taken anyway.  15 Nov course run: the robot
       turned about 12 deg while backing up for the 2nd scan, the box left
       the centre ray and the run stopped "obstacle lost between scans",
       though scan 1 had both edges (211 mm). */

    if ((scan_near.b_left_seen) && (scan_near.b_right_seen))
    {
        scan_far = scan_near;
        profile  = scan_near;
    }
    else
    {
        if (!mv(motion_drive(-S5_FOLLOW_SPEED, REVERSE_MM)))
        {
            /* O5 */
            halt("reverse for 2nd scan did not finish");
            return;
        }

        wait_still();
        obstacle_scan(&scan_far);
        obstacle_combine(&scan_near, &scan_far, &profile);
    }

    tm_sprintf(msg,
               app_str("obstacle width %d +/- %d mm (scan 1: %d +/- %d, "
                       "scan 2: %d +/- %d); edges L %d R %d mm; front %d mm"),
               profile.width_mm, profile.unc_mm, scan_near.width_mm,
               scan_near.unc_mm, scan_far.width_mm, scan_far.unc_mm,
               profile.left_mm, profile.right_mm, profile.front_mm);
    say(msg);

    if (!profile.b_found)
    {
        halt("obstacle lost between scans");
        return;
    }

    /* Side: clear, and the nearer edge (less to drive) */
    if ((profile.b_left_clear) &&
        ((!profile.b_right_clear) || (profile.left_mm <= -profile.right_mm)))
    {
        sgn = +1;
    }
    else if (profile.b_right_clear)
    {
        sgn = -1;
    }
    else
    {
        halt("no clear side");
        return;
    }

    /* O7: plan for at least the largest expected obstacle */
    w_plan = profile.width_mm + profile.unc_mm;

    if (w_plan < S5_MAX_WIDTH_MM)
    {
        w_plan = S5_MAX_WIDTH_MM;
    }

    if (w_plan > S5_PLAN_MAX_MM)
    {
        w_plan = S5_PLAN_MAX_MM;
    }

    centre    = (profile.left_mm + profile.right_mm) / 2;
    edge_plan = centre + (sgn * w_plan / 2);
    edge_meas = ((sgn > 0) ? profile.left_mm : profile.right_mm) +
                (sgn * profile.unc_mm);

    if ((sgn * edge_meas) > (sgn * edge_plan))
    {
        edge_plan = edge_meas;
    }

    /* never further out than the largest planned box, sitting on the
       line give or take 50 mm */

    if ((sgn * edge_plan) > ((S5_PLAN_MAX_MM / 2) + 50))
    {
        edge_plan = sgn * ((S5_PLAN_MAX_MM / 2) + 50);
    }

    y_off  = (sgn * edge_plan) + (S2_ROBOT_WIDTH_MM / 2) + MARGIN_MM;
    x_pass = profile.front_mm + D_MAX_MM + S2_ROBOT_LENGTH_MM + MARGIN_MM;
    tm_sprintf(msg,
               app_str("bypass %s: planned width %d mm, out %d mm, "
                       "past %d mm"),
               (sgn > 0) ? "LEFT" : "RIGHT", w_plan, y_off, x_pass);
    say(msg);

    /* legs 1-4.  The leg past the box (pass_box) takes no line: the robot
       always goes round and back to its own line (15 Nov: a different
       piece of track beside the box's far side was followed instead).
       Before, it took a line once past the box's back (8 Nov: from the
       box's front, a junction's side line beside the box was followed). */

    if (!mv(motion_turn(sgn * 90)))
    {
        goto fail;
    }

    if (!ahead_clear(y_off))
    {
        halt("bypass leg 2 blocked");
        return;
    }

    /* leg 2 with the forward watch (15 Nov) */
    found = drive_until_line(y_off, y_off + 1);

    if (DRIVE_BLOCKED == found)
    {
        return; /* halted, reason given */
    }

    if (found != 0)
    {
        goto fail;
    }

    if (!mv(motion_turn(-sgn * 90)))
    {
        goto fail;
    }

    /* sideways from the robot to the box's side face, as the scan saw it */
    found = pass_box(
        sgn, y_off - (sgn * ((sgn > 0) ? profile.left_mm : profile.right_mm)),
        x_pass);

    if (found < 0)
    {
        return; /* halted, reason given */
    }

    /* leg 5-6: back toward its own line, any black counts (it may have
       curved either way behind the box); first make sure the box is not
       still beside (15 Nov: longer than D_MAX_MM) */

    if (!turn_back_clear(sgn, y_off))
    {
        return; /* halted, reason given */
    }

    odo0  = motor_odometer();
    found = drive_until_line(y_off + REJOIN_FAR_MM, 20);

    if (DRIVE_BLOCKED == found)
    {
        return; /* halted, reason given */
    }

    if (found < 0)
    {
        goto fail;
    }

    if (!found)
    {
        halt("line not found after the bypass");
        return;
    }

    tm_sprintf(msg, app_str("line found %d mm after the last turn"),
               motion_counts_to_mm(motor_odometer() - odo0));
    say(msg);

    /* leg 7: wheels onto the line, then always back to the old direction
       (the bypass turned sgn 90, -sgn 90, -sgn 90; the "box still beside"
       steps cancel), like a junction turn: fast to TURN_SEARCH_DEG short,
       then slowly until a sensor finds the tape within TURN_SEARCH_DEG of
       the old direction, then centred on it.  20 Nov, track 4: it used to
       skip the turn when a sensor already saw black, and otherwise stop at
       the first tape of a 170 deg spin, so it followed other tape beside
       its line, across it. */

    if ((!mv(motion_drive(S5_FOLLOW_SPEED, S3_SENSOR_TO_AXLE_MM))) ||
        (!turn_onto_line(sgn * 90)))
    {
        goto fail;
    }

    barcode_ignore_mm(REJOIN_BAR_MM);

    if (!resume_follow())
    {
        goto fail;
    }

    tm_sprintf(msg,
               app_str("bypass done, following the line (heading %d deg "
                       "from before the box, by the wheels)"),
               heading_deg(&before));
    say(msg);
    set_state(M_RUN);
    return;
fail:
    halt("bypass step did not finish");
}

/** The mission task: START / STOP, barcodes, junction moves, obstacles,
 * line lost. */
EXPORT void mission_task (int32_t stacd, void * p_exinf)
{
    /* NOTE: kernel type, tk_wai_flg() writes a UINT (Barr C deviation) */
    UINT ptn;
    ER   err;
    bool b_start_was_down = false;

    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        err = tk_wai_flg(gh_flg_mission, MSN_FLG_ALL, TWF_ORW | TWF_BITCLR,
                         &ptn, TIMEOUT_MISSION_MS);
        app_timing_wake(g_tmg_mission, 0);

        if (err != E_OK)
        {
            ptn = 0;
        }

        if ((ptn & MSN_FLG_PING) != 0u)
        {
            g_mission_pings++;
        }

        /* STOP: the motors are already off (app_stop_request) */
        if ((g_mw.state != M_IDLE) && (app_stop_reasons() != 0u))
        {
            obstacle_watch(false);
            line_set_enabled(false);
            set_state(M_IDLE);
            say(app_str("STOP"));
        }

        if ((M_IDLE == g_mw.state) && (g_flags.b_armed))
        {
            bool b_down =
                (button_is_down(PIN_START)) && (!button_is_down(PIN_STOP));
            bool b_go = (ptn & MSN_FLG_START) != 0u;

            if (b_start_was_down && (!b_down))
            {
                b_go = true; /* released */
            }

            b_start_was_down = b_down;

            if (b_go && (!button_is_down(PIN_STOP)))
            {
                app_stop_clear(STOP_BUTTON | STOP_MQTT);
            }

            if (b_go)
            {
                start_run();
            }
        }
        else if (M_RUN == g_mw.state)
        {
            /* barcode first: a code just before a junction counts */
            if ((ptn & MSN_FLG_BARCODE) != 0u)
            {
                on_barcode();
            }

            if (LINE_FAULT == line_state())
            {
                /* a stuck line sensor: nothing it says can be trusted */
                halt((LINE_FAULT_ONE == line_fault())
                         ? "a line sensor black for 30 cm (stuck black? check "
                           "its pot and cable)"
                         : "both line sensors black for 15 cm (stuck, or a "
                           "wide black area)");
            }
            else if ((ptn & MSN_FLG_OBSTACLE) != 0u)
            {
                on_obstacle();
            }
            else if ((ptn & MSN_FLG_JUNCTION) != 0u)
            {
                on_junction();
            }
            else if ((ptn & MSN_FLG_LINE_LOST) != 0u)
            {
                halt("line lost");
            }
            else
            {
                /* nothing to do for any other case */
            }
        }
        else
        {
            /* nothing to do for any other case */
        }

        app_timing_done(g_tmg_mission);
    }
}

/*** end of file ***/
