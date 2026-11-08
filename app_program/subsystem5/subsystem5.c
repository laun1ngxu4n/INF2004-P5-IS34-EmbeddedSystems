/*
 *  subsystem5.c
 *  Subsystem 5 -- Mission + Obstacle: creates its objects and starts its
 *  tasks.
 */

#include "subsystem5.h"

EXPORT ID  flg_mission; /* IF-01 */
EXPORT ID  tid_mission;
EXPORT INT tmg_mission;
EXPORT UW  mission_pings;
EXPORT ID  tid_obstacle;
EXPORT ID  cyc_obstacle;
EXPORT INT tmg_obstacle;

/* GP21 pressed: interrupt context.  Stop first, then tell Mission. */
LOCAL void stop_button_isr (UINT pin, UW events)
{
    (void)pin;
    (void)events;
    app_stop_request(STOP_BUTTON);
    tk_set_flg(flg_mission, MSN_FLG_STOP);
}

EXPORT ER subsystem5_init (void)
{
    T_CFLG cflg = {0};
    ER     er;

    /* Several setters (Telemetry now; buttons, Safety, MQTT later), one
       waiter (Mission), which clears the bits it has handled. */
    cflg.flgatr  = TA_TFIFO | TA_WSGL;
    cflg.iflgptn = 0;
    flg_mission  = tk_cre_flg(&cflg);
    if (flg_mission < E_OK)
    {
        return flg_mission;
    }

    er = obstacle_hw_init();
    if (er < E_OK)
    {
        return er;
    }
    er = app_gpio_input(PIN_START, TRUE);
    if (er < E_OK)
    {
        return er;
    }
    er = app_gpio_input(PIN_STOP, TRUE);
    if (er < E_OK)
    {
        return er;
    }
    er = app_gpio_irq_add(PIN_STOP, APP_GPIO_FALL, stop_button_isr);
    if (er < E_OK)
    {
        return er;
    }

    tmg_mission  = app_timing_add("mission", PRI_MISSION, TIMEOUT_MISSION_MS);
    tmg_obstacle = app_timing_add("obstacle", PRI_OBSTACLE, PERIOD_OBSTACLE_MS);
    cyc_obstacle = app_create_wake_cyclic(&tid_obstacle, PERIOD_OBSTACLE_MS);
    if (tmg_mission < E_OK)
    {
        return tmg_mission;
    }
    if (tmg_obstacle < E_OK)
    {
        return tmg_obstacle;
    }
    return (cyc_obstacle < E_OK) ? cyc_obstacle : E_OK;
}

EXPORT ER subsystem5_start (void)
{
    tid_mission = app_start_task(mission_task, PRI_MISSION, 0, "mission_task");
    if (tid_mission < E_OK)
    {
        return tid_mission;
    }
    tid_obstacle =
        app_start_task(obstacle_task, PRI_OBSTACLE, 0, "obstacle_task");
    if (tid_obstacle < E_OK)
    {
        return tid_obstacle;
    }
    return tk_sta_cyc(cyc_obstacle);
}

/*** end of file ***/

