/*
 *  subsystem4.c
 *  Subsystem 4 -- Safety + IMU: creates its objects and starts its tasks.
 */

#include "subsystem4.h"

EXPORT ID  tid_safety;
EXPORT ID  cyc_safety;
EXPORT INT tmg_safety;
EXPORT ID  tid_imu;
EXPORT ID  cyc_imu;
EXPORT INT tmg_imu;

EXPORT ER subsystem4_init (void)
{
    tmg_safety = app_timing_add("safety", PRI_SAFETY, PERIOD_SAFETY_MS);
    tmg_imu    = app_timing_add("imu", PRI_IMU, PERIOD_IMU_MS);
    cyc_safety = app_create_wake_cyclic(&tid_safety, PERIOD_SAFETY_MS);
    cyc_imu    = app_create_wake_cyclic(&tid_imu, PERIOD_IMU_MS);
    if (tmg_safety < E_OK)
    {
        return tmg_safety;
    }
    if (tmg_imu < E_OK)
    {
        return tmg_imu;
    }
    if (cyc_safety < E_OK)
    {
        return cyc_safety;
    }
    return (cyc_imu < E_OK) ? cyc_imu : E_OK;
}

EXPORT ER subsystem4_start (void)
{
    tid_safety = app_start_task(safety_task, PRI_SAFETY, 0, "safety_task");
    if (tid_safety < E_OK)
    {
        return tid_safety;
    }
    tid_imu = app_start_task(imu_task, PRI_IMU, 0, "imu_task");
    if (tid_imu < E_OK)
    {
        return tid_imu;
    }
    CHECK(tk_sta_cyc(cyc_safety), "cyc_safety");
    return tk_sta_cyc(cyc_imu);
}

/*** end of file ***/

