/**
 * @file    subsystem5.c
 *  Subsystem 5 -- Mission + Obstacle: creates its objects and starts its
 *  tasks.
 */

#include "subsystem5.h"

EXPORT ID       gh_flg_mission; /* IF-01 */
EXPORT ID       gh_tid_mission;
EXPORT int32_t  g_tmg_mission;
EXPORT uint32_t g_mission_pings;
EXPORT ID       gh_tid_obstacle;
EXPORT ID       gh_cyc_obstacle;
EXPORT int32_t  g_tmg_obstacle;

/** GP21 pressed: interrupt context.  Stop first, then tell Mission. */
LOCAL void stop_button_isr (uint32_t pin, uint32_t events)
{
    (void)pin;
    (void)events;
    app_stop_request(STOP_BUTTON);
    tk_set_flg(gh_flg_mission, MSN_FLG_STOP);
}

/** Creates subsystem 5's objects (mission flag, mission and obstacle tasks,
 * STOP button). */
EXPORT ER subsystem5_init (void)
{
    T_CFLG cflg = {0};
    ER     err;

    /* Several setters (Telemetry now; buttons, Safety, MQTT later), one
       waiter (Mission), which clears the bits it has handled. */
    cflg.flgatr    = TA_TFIFO | TA_WSGL;
    cflg.iflgptn   = 0;
    gh_flg_mission = tk_cre_flg(&cflg);

    if (gh_flg_mission < E_OK)
    {
        return gh_flg_mission;
    }

    err = obstacle_hw_init();

    if (err < E_OK)
    {
        return err;
    }

    err = app_gpio_input(PIN_START, TRUE);

    if (err < E_OK)
    {
        return err;
    }

    err = app_gpio_input(PIN_STOP, TRUE);

    if (err < E_OK)
    {
        return err;
    }

    err = app_gpio_irq_add(PIN_STOP, APP_GPIO_FALL, stop_button_isr);

    if (err < E_OK)
    {
        return err;
    }

    g_tmg_mission = app_timing_add("mission", PRI_MISSION, TIMEOUT_MISSION_MS);
    g_tmg_obstacle =
        app_timing_add("obstacle", PRI_OBSTACLE, PERIOD_OBSTACLE_MS);
    gh_cyc_obstacle =
        app_create_wake_cyclic(&gh_tid_obstacle, PERIOD_OBSTACLE_MS);

    if (g_tmg_mission < E_OK)
    {
        return g_tmg_mission;
    }

    if (g_tmg_obstacle < E_OK)
    {
        return g_tmg_obstacle;
    }

    return (gh_cyc_obstacle < E_OK) ? gh_cyc_obstacle : E_OK;
}

/** Starts subsystem 5's tasks (mission, obstacle). */
EXPORT ER subsystem5_start (void)
{
    gh_tid_mission =
        app_start_task(mission_task, PRI_MISSION, 0, "mission_task");

    if (gh_tid_mission < E_OK)
    {
        return gh_tid_mission;
    }

    gh_tid_obstacle =
        app_start_task(obstacle_task, PRI_OBSTACLE, 0, "obstacle_task");

    if (gh_tid_obstacle < E_OK)
    {
        return gh_tid_obstacle;
    }

    return tk_sta_cyc(gh_cyc_obstacle);
}

/*** end of file ***/

