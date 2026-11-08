/**
 * @file    subsystem1.c
 *  Subsystem 1 -- Telemetry: creates its objects and starts its task.
 */

#include "subsystem1.h"

EXPORT ID      gh_tid_telemetry;
EXPORT ID      gh_cyc_telemetry;
EXPORT int32_t g_tmg_telemetry;

EXPORT ER subsystem1_init (void)
{
    g_tmg_telemetry =
        app_timing_add("telemetry", PRI_TELEMETRY, PERIOD_TELEMETRY_MS);
    gh_cyc_telemetry =
        app_main_create_wake_cyclic(&gh_tid_telemetry, PERIOD_TELEMETRY_MS);

    if (g_tmg_telemetry < E_OK)
    {
        return g_tmg_telemetry;
    }

    return (gh_cyc_telemetry < E_OK) ? gh_cyc_telemetry : E_OK;
}

/** Starts subsystem 1's task (telemetry). */
EXPORT ER subsystem1_start (void)
{
    gh_tid_telemetry =
        app_main_start_task(telemetry_task, PRI_TELEMETRY, 0, "telemetry_task");

    if (gh_tid_telemetry < E_OK)
    {
        return gh_tid_telemetry;
    }

    return tk_sta_cyc(gh_cyc_telemetry);
}

/*** end of file ***/

