/**
 * @file    subsystem2.c
 *  Subsystem 2 -- Motion Control: sets up the motors and encoders, creates
 *  its objects and starts its task.
 */

#include "subsystem2.h"

EXPORT ID      gh_tid_motion;
EXPORT ID      gh_cyc_motion;
EXPORT int32_t g_tmg_motion;

/** Creates subsystem 2's objects (motion): hardware, timing, cyclic
 * handler. */
EXPORT ER subsystem2_init (void)
{
    ER err;

    err = motor_init(); /* motors off, encoders counting */

    if (err < E_OK)
    {
        return err;
    }

    g_tmg_motion = app_timing_add("motion", PRI_MOTION, PERIOD_MOTION_MS);
    gh_cyc_motion =
        app_main_create_wake_cyclic(&gh_tid_motion, PERIOD_MOTION_MS);

    if (g_tmg_motion < E_OK)
    {
        return g_tmg_motion;
    }

    return (gh_cyc_motion < E_OK) ? gh_cyc_motion : E_OK;
}

/** Starts subsystem 2's task (motion) and its cyclic handler. */
EXPORT ER subsystem2_start (void)
{
    gh_tid_motion =
        app_main_start_task(motion_task, PRI_MOTION, 0, "motion_task");

    if (gh_tid_motion < E_OK)
    {
        return gh_tid_motion;
    }

    return tk_sta_cyc(gh_cyc_motion);
}

/*** end of file ***/

