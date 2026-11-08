/**
 * @file    subsystem4.c
 *  Subsystem 4 -- Safety + IMU: creates its objects and starts its tasks.
 */

#include "subsystem4.h"

EXPORT ID      gh_tid_safety;
EXPORT ID      gh_cyc_safety;
EXPORT int32_t g_tmg_safety;
EXPORT ID      gh_tid_imu;
EXPORT ID      gh_cyc_imu;
EXPORT int32_t g_tmg_imu;

/** Creates subsystem 4's objects (safety and IMU tasks). */
EXPORT ER subsystem4_init (void)
{
    g_tmg_safety = app_timing_add("safety", PRI_SAFETY, PERIOD_SAFETY_MS);
    g_tmg_imu    = app_timing_add("imu", PRI_IMU, PERIOD_IMU_MS);
    gh_cyc_safety =
        app_main_create_wake_cyclic(&gh_tid_safety, PERIOD_SAFETY_MS);
    gh_cyc_imu = app_main_create_wake_cyclic(&gh_tid_imu, PERIOD_IMU_MS);

    if (g_tmg_safety < E_OK)
    {
        return g_tmg_safety;
    }

    if (g_tmg_imu < E_OK)
    {
        return g_tmg_imu;
    }

    if (gh_cyc_safety < E_OK)
    {
        return gh_cyc_safety;
    }

    return (gh_cyc_imu < E_OK) ? gh_cyc_imu : E_OK;
}

/** Starts subsystem 4's tasks (safety, IMU). */
EXPORT ER subsystem4_start (void)
{
    gh_tid_safety =
        app_main_start_task(safety_task, PRI_SAFETY, 0, "safety_task");

    if (gh_tid_safety < E_OK)
    {
        return gh_tid_safety;
    }

    gh_tid_imu = app_main_start_task(imu_task, PRI_IMU, 0, "imu_task");

    if (gh_tid_imu < E_OK)
    {
        return gh_tid_imu;
    }

    app_check(tk_sta_cyc(gh_cyc_safety), "cyc_safety");
    return tk_sta_cyc(gh_cyc_imu);
}

/*** end of file ***/

