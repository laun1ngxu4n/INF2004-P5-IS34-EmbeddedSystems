/*
 *  subsystem1.c
 *  Subsystem 1 -- Telemetry: creates its objects and starts its task.
 */

#include "subsystem1.h"

EXPORT ID  tid_telemetry;
EXPORT ID  cyc_telemetry;
EXPORT INT tmg_telemetry;

EXPORT ER subsystem1_init (void)
{
    tmg_telemetry =
        app_timing_add("telemetry", PRI_TELEMETRY, PERIOD_TELEMETRY_MS);
    cyc_telemetry = app_create_wake_cyclic(&tid_telemetry, PERIOD_TELEMETRY_MS);
    if (tmg_telemetry < E_OK)
    {
        return tmg_telemetry;
    }
    return (cyc_telemetry < E_OK) ? cyc_telemetry : E_OK;
}

EXPORT ER subsystem1_start (void)
{
    tid_telemetry =
        app_start_task(telemetry_task, PRI_TELEMETRY, 0, "telemetry_task");
    if (tid_telemetry < E_OK)
    {
        return tid_telemetry;
    }
    return tk_sta_cyc(cyc_telemetry);
}

/*** end of file ***/

