/*
 *  task_safety.c
 *  Subsystem 4 -- the Safety / Fault Monitor task.
 *
 *  Stage 0: wakes every 20 ms, is timed, raises no faults yet.
 */

#include "subsystem4.h"

EXPORT void safety_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;

    for (;;)
    {
        tk_slp_tsk(TMO_FEVR); /* woken every 20 ms */
        app_timing_wake(tmg_safety, tk_can_wup(TSK_SELF));

        /* Stage 1: check stop_reasons / fault bits, motor_safe_off(),
           task heartbeats; feed the watchdog. */

        app_timing_done(tmg_safety);
    }
}

/*** end of file ***/

