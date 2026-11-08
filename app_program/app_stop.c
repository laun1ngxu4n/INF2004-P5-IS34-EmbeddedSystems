/*
 *  app_stop.c
 *  The one STOP helper (see app_stop.h).
 */

#include "app.h"
#include "app_stop.h"
#include "subsystem2/subsystem2.h" /* motor_safe_off() */

LOCAL volatile UW g_stop_reasons;
LOCAL volatile UW g_stop_time_us;

EXPORT void app_stop_request (UW reason)
{
    UINT irq_state;

    motor_safe_off(); /* first, before anything else */
    APP_LOCK(irq_state);
    if (0u == g_stop_reasons)
    {
        g_stop_time_us = app_time_us();
    }
    g_stop_reasons |= reason;
    APP_UNLOCK(irq_state);
}

EXPORT void app_stop_clear (UW reason)
{
    UINT irq_state;

    APP_LOCK(irq_state);
    g_stop_reasons &= ~reason;
    APP_UNLOCK(irq_state);
}

EXPORT UW app_stop_reasons (void)
{
    return g_stop_reasons;
}

EXPORT UW app_stop_time_us (void)
{
    return g_stop_time_us;
}

/*** end of file ***/

