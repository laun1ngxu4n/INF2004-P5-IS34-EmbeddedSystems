/**
 * @file    safety.c
 *  Subsystem 4 -- the Safety / Fault Monitor task.
 *
 *  Every 20 ms it checks the motors and the Motion task and keeps their
 *  bits of the shared fault word (app_stop.h, Change Requirement 1):
 *
 *  - FAULT_MOTOR_STALL: a wheel driven at STALL_DUTY or more turns slower
 *    than STALL_MM_S for STALL_TICKS (1 s).  The motors go off
 *    (STOP_FAULT).  Normal driving needs about 250 per mille, so only a
 *    blocked wheel gets there.
 *  - FAULT_MOTION_HUNG: the motors are on but the Motion task has not run
 *    for HUNG_US.  The motors go off (STOP_FAULT) without waiting for
 *    Motion (Tribunal 1).
 *  - FAULT_ENCODER: more than ENC_BAD_PER_S missed encoder edges in a
 *    second.  A warning only (FAULT_WARNINGS): reported, never stops.
 *
 *  A stop fault is cleared after FAULT_HOLD_TICKS with the motors off and
 *  the cause gone; Mission is IDLE by then and waits for START.  Only
 *  Safety clears these bits.
 */

#include "subsystem4.h"
#include "safety.h"
#include "../subsystem1/subsystem1.h" /* telemetry_event() */
#include "../subsystem2/subsystem2.h" /* motion status, motor duties */

#define STALL_DUTY       800 /* per mille */
#define STALL_MM_S       15  /* slower than this = not turning */
#define STALL_TICKS      50  /* 1 s at 20 ms */
#define HUNG_US          200000u
#define ENC_BAD_PER_S    50  /* missed edges in one second */
#define ENC_WINDOW_TICKS 50  /* 1 s at 20 ms */
#define FAULT_HOLD_TICKS 100 /* 2 s at 20 ms */

/* Safety's own state between periods (counts of 20 ms periods) */
typedef struct
{
    uint32_t stall_l APP_FIELD(6); /* 0..STALL_TICKS */
    uint32_t stall_r APP_FIELD(6);
    uint32_t hold    APP_FIELD(7); /* 0..FAULT_HOLD_TICKS */
    uint32_t window  APP_FIELD(6); /* 0..ENC_WINDOW_TICKS */
} safety_state_t;

/** |value| */
Inline int32_t abs32 (int32_t value)
{
    return (value < 0) ? -value : value;
}

/** One stall counter step: driven hard and not turning counts up. */
Inline uint32_t stall_step (uint32_t count, int32_t duty, int32_t speed)
{
    uint32_t next = 0u;

    if ((abs32(duty) >= STALL_DUTY) && (abs32(speed) < STALL_MM_S))
    {
        next = (count < STALL_TICKS) ? count + 1u : count;
    }

    return next;
}

/** A stop fault: its bit, motors off, the panel told. */
LOCAL void stop_fault (uint32_t fault, const char * p_why)
{
    app_stop_fault_set(fault);
    app_stop_request(STOP_FAULT);
    telemetry_event(app_str(p_why));
}

/** Stall and Motion-hung checks, one 20 ms period. */
LOCAL void check_motors (safety_state_t * p_state)
{
    motion_status_t status;
    app_timing_t    motion_tmg;
    int32_t         duty_l;
    int32_t         duty_r;
    bool            b_on;

    motor_duties(&duty_l, &duty_r);
    motion_get_status(&status);
    b_on = (duty_l != 0) || (duty_r != 0);

    p_state->stall_l = stall_step(p_state->stall_l, duty_l, status.speed_l);
    p_state->stall_r = stall_step(p_state->stall_r, duty_r, status.speed_r);

    if (((STALL_TICKS == p_state->stall_l) ||
         (STALL_TICKS == p_state->stall_r)) &&
        (0u == (app_stop_faults() & FAULT_MOTOR_STALL)))
    {
        stop_fault(FAULT_MOTOR_STALL,
                   "Safety: a wheel driven hard is not turning, motors off");
    }

    if (b_on && (E_OK == app_timing_get(g_tmg_motion, &motion_tmg)) &&
        ((app_timing_now_us() - motion_tmg.last_us) > HUNG_US) &&
        (0u == (app_stop_faults() & FAULT_MOTION_HUNG)))
    {
        stop_fault(FAULT_MOTION_HUNG,
                   "Safety: the Motion task stopped running, motors off");
    }
}

/** Clears the stop faults once the motors have been off FAULT_HOLD_TICKS. */
LOCAL void release_faults (safety_state_t * p_state)
{
    const uint32_t stop_faults = FAULT_MOTOR_STALL | FAULT_MOTION_HUNG;
    int32_t        duty_l;
    int32_t        duty_r;

    motor_duties(&duty_l, &duty_r);

    if ((0u == (app_stop_faults() & stop_faults)) || (duty_l != 0) ||
        (duty_r != 0))
    {
        p_state->hold = 0u;
    }
    else if (p_state->hold < FAULT_HOLD_TICKS)
    {
        p_state->hold++;
    }
    else
    {
        p_state->hold    = 0u;
        p_state->stall_l = 0u;
        p_state->stall_r = 0u;
        app_stop_fault_clear(stop_faults);
        app_stop_clear(STOP_FAULT);
    }
}

/** The safety task: the motors, the Motion task and the encoders. */
EXPORT void safety_task (int32_t stacd, void * p_exinf)
{
    safety_state_t state       = {0u, 0u, 0u, 0u};
    uint32_t       invalid_old = 0u; /* a counter: all 32 bits */

    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        tk_slp_tsk(TMO_FEVR); /* woken every 20 ms */
        app_timing_wake(g_tmg_safety, tk_can_wup(TSK_SELF));

        check_motors(&state);
        release_faults(&state);

        state.window++;

        if (state.window >= ENC_WINDOW_TICKS)
        {
            int32_t  left;
            int32_t  right;
            uint32_t invalid;

            state.window = 0u;
            motor_encoders(&left, &right, &invalid);

            if ((invalid - invalid_old) > ENC_BAD_PER_S)
            {
                app_stop_fault_set(FAULT_ENCODER); /* a warning */
            }
            else
            {
                app_stop_fault_clear(FAULT_ENCODER);
            }

            invalid_old = invalid;
        }

        app_timing_done(g_tmg_safety);
    }
}

/*** end of file ***/

