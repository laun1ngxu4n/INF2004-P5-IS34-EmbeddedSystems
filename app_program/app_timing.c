/*
 *  app_timing.c
 *  Per-task timing monitor (see app_timing.h).
 *
 *  Each slot is written by its own task (wake/done) and read and reset
 *  by Telemetry (report), so every access goes through APP_LOCK.  The
 *  locked sections are a few loads and stores.
 */

#include "app.h"

/* RP2040 TIMER block, TIMERAWL: free-running 1 MHz counter, read-only.
   Reading it has no side effects (unlike TIMELR/TIMEHR). */
#define RP2040_TIMERAWL (*(volatile UW *)0x40054028)

LOCAL app_timing_t g_tmg[APP_TIMING_MAX];
LOCAL INT          g_tmg_used;

EXPORT UW app_time_us (void)
{
    return RP2040_TIMERAWL;
}

/* Called only from the subsystems' _init(), which usermain() runs one
   after another before any task starts, so no lock is needed here. */
EXPORT INT app_timing_add (const char * p_name, PRI pri, UINT period_ms)
{
    app_timing_t * p_slot;

    if (g_tmg_used >= APP_TIMING_MAX)
    {
        return E_LIMIT;
    }
    p_slot                = &g_tmg[g_tmg_used];
    p_slot->p_name        = p_name;
    p_slot->pri           = pri;
    p_slot->period_ms     = period_ms;
    p_slot->win_min_iv_us = 0xFFFFFFFFu;
    return g_tmg_used++;
}

EXPORT void app_timing_wake (INT slot, INT overruns)
{
    app_timing_t * p_slot;
    UW             now = app_time_us();
    UW             interval;
    UINT           irq_state;

    if ((slot < 0) || (slot >= g_tmg_used))
    {
        return;
    }
    p_slot = &g_tmg[slot];

    APP_LOCK(irq_state);
    if (p_slot->count > 0)
    {
        interval = now - p_slot->last_us;
        if (interval < p_slot->win_min_iv_us)
        {
            p_slot->win_min_iv_us = interval;
        }
        if (interval > p_slot->win_max_iv_us)
        {
            p_slot->win_max_iv_us = interval;
        }
        if (interval > p_slot->max_iv_us)
        {
            p_slot->max_iv_us    = interval;
            p_slot->max_iv_at_us = now;
        }
        if (interval > p_slot->test_max_iv_us)
        {
            p_slot->test_max_iv_us = interval;
        }
    }
    if (overruns > 0)
    {
        p_slot->overruns += (UW)overruns;
    }
    p_slot->last_us      = now;
    p_slot->run_start_us = now;
    p_slot->count++;
    APP_UNLOCK(irq_state);
}

EXPORT void app_timing_done (INT slot)
{
    app_timing_t * p_slot;
    UW             run;
    UINT           irq_state;

    if ((slot < 0) || (slot >= g_tmg_used))
    {
        return;
    }
    p_slot = &g_tmg[slot];

    APP_LOCK(irq_state);
    run = app_time_us() - p_slot->run_start_us;
    if (run > p_slot->win_max_run_us)
    {
        p_slot->win_max_run_us = run;
    }
    if (run > p_slot->max_run_us)
    {
        p_slot->max_run_us = run;
    }
    APP_UNLOCK(irq_state);
}

EXPORT void app_timing_test_start (INT slot)
{
    UINT irq_state;

    if ((slot < 0) || (slot >= g_tmg_used))
    {
        return;
    }
    APP_LOCK(irq_state);
    g_tmg[slot].test_max_iv_us = 0;
    APP_UNLOCK(irq_state);
}

EXPORT ER app_timing_get (INT slot, app_timing_t * p_p_copy)
{
    UINT irq_state;

    if ((slot < 0) || (slot >= g_tmg_used))
    {
        return E_PAR;
    }
    APP_LOCK(irq_state);
    *p_p_copy = g_tmg[slot];
    APP_UNLOCK(irq_state);
    return E_OK;
}

/* Print microseconds as milliseconds with two decimals, e.g. "10.02" */
LOCAL void print_ms (UW micros)
{
    tm_printf((UB *)"%d.%02d", (INT)(micros / 1000),
              (INT)((micros % 1000) / 10));
}

EXPORT void app_timing_report (void)
{
    app_timing_t snap;
    INT          idx;
    UINT         irq_state;

    tm_printf(
        (UB *)"[tmg] t=%d ms | task pri period | wakes overruns | "
              "interval min-max ms (boot max @ s) | run max us (boot max)\n",
        (INT)(app_time_us() / 1000));
    for (idx = 0; idx < g_tmg_used; idx++)
    {
        /* Copy and start a new window in one locked step, print after */
        APP_LOCK(irq_state);
        snap                      = g_tmg[idx];
        g_tmg[idx].win_min_iv_us  = 0xFFFFFFFFu;
        g_tmg[idx].win_max_iv_us  = 0;
        g_tmg[idx].win_max_run_us = 0;
        APP_UNLOCK(irq_state);

        tm_printf((UB *)"[tmg] %-9s %2d %3d | %6d %3d | ", snap.p_name,
                  (INT)snap.pri, (INT)snap.period_ms, (INT)snap.count,
                  (INT)snap.overruns);
        if (0 == snap.win_max_iv_us)
        {
            tm_printf((UB *)"no wake this window");
        }
        else
        {
            print_ms(snap.win_min_iv_us);
            tm_printf((UB *)"-");
            print_ms(snap.win_max_iv_us);
            tm_printf((UB *)" (");
            print_ms(snap.max_iv_us);
            tm_printf((UB *)" @ %d.%d", (INT)(snap.max_iv_at_us / 1000000),
                      (INT)((snap.max_iv_at_us % 1000000) / 100000));
            tm_printf((UB *)")");
        }
        tm_printf((UB *)" | %d (%d)\n", (INT)snap.win_max_run_us,
                  (INT)snap.max_run_us);
    }
}

/*** end of file ***/

