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
#define RP2040_TIMERAWL (*(volatile uint32_t *)0x40054028)

LOCAL app_timing_t g_tmg[APP_TIMING_MAX];
LOCAL int32_t      g_tmg_used;

EXPORT uint32_t app_time_us (void)
{
    return RP2040_TIMERAWL;
}

/* Called only from the subsystems' _init(), which usermain() runs one
   after another before any task starts, so no lock is needed here. */
EXPORT int32_t app_timing_add (const char * p_name, PRI pri, uint32_t period_ms)
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

EXPORT void app_timing_wake (int32_t slot, int32_t overruns)
{
    app_timing_t * p_slot;
    uint32_t       now = app_time_us();
    uint32_t       interval;
    uint32_t       irq_state;

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
        p_slot->overruns += (uint32_t)overruns;
    }
    p_slot->last_us      = now;
    p_slot->run_start_us = now;
    p_slot->count++;
    APP_UNLOCK(irq_state);
}

EXPORT void app_timing_done (int32_t slot)
{
    app_timing_t * p_slot;
    uint32_t       run;
    uint32_t       irq_state;

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

EXPORT void app_timing_test_start (int32_t slot)
{
    uint32_t irq_state;

    if ((slot < 0) || (slot >= g_tmg_used))
    {
        return;
    }
    APP_LOCK(irq_state);
    g_tmg[slot].test_max_iv_us = 0;
    APP_UNLOCK(irq_state);
}

EXPORT ER app_timing_get (int32_t slot, app_timing_t * p_p_copy)
{
    uint32_t irq_state;

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
LOCAL void print_ms (uint32_t micros)
{
    tm_printf((uint8_t *)"%d.%02d", (int32_t)(micros / 1000),
              (int32_t)((micros % 1000) / 10));
}

EXPORT void app_timing_report (void)
{
    app_timing_t snap;
    int32_t      idx;
    uint32_t     irq_state;

    tm_printf((uint8_t *)"[tmg] t=%d ms | task pri period | wakes overruns | "
                         "interval min-max ms (boot max @ s) | run max us "
                         "(boot max)\n",
              (int32_t)(app_time_us() / 1000));
    for (idx = 0; idx < g_tmg_used; idx++)
    {
        /* Copy and start a new window in one locked step, print after */
        APP_LOCK(irq_state);
        snap                      = g_tmg[idx];
        g_tmg[idx].win_min_iv_us  = 0xFFFFFFFFu;
        g_tmg[idx].win_max_iv_us  = 0;
        g_tmg[idx].win_max_run_us = 0;
        APP_UNLOCK(irq_state);

        tm_printf((uint8_t *)"[tmg] %-9s %2d %3d | %6d %3d | ", snap.p_name,
                  (int32_t)snap.pri, (int32_t)snap.period_ms,
                  (int32_t)snap.count, (int32_t)snap.overruns);
        if (0 == snap.win_max_iv_us)
        {
            tm_printf((uint8_t *)"no wake this window");
        }
        else
        {
            print_ms(snap.win_min_iv_us);
            tm_printf((uint8_t *)"-");
            print_ms(snap.win_max_iv_us);
            tm_printf((uint8_t *)" (");
            print_ms(snap.max_iv_us);
            tm_printf((uint8_t *)" @ %d.%d",
                      (int32_t)(snap.max_iv_at_us / 1000000),
                      (int32_t)((snap.max_iv_at_us % 1000000) / 100000));
            tm_printf((uint8_t *)")");
        }
        tm_printf((uint8_t *)" | %d (%d)\n", (int32_t)snap.win_max_run_us,
                  (int32_t)snap.max_run_us);
    }
}

/*** end of file ***/

