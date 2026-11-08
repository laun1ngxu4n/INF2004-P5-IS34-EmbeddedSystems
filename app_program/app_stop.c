/**
 * @file    app_stop.c
 *  The one STOP helper (see app_stop.h).
 */

#include "app.h"
#include "app_stop.h"
#include "subsystem2/subsystem2.h" /* motor_safe_off() */

LOCAL volatile uint32_t g_stop_reasons;
LOCAL volatile uint32_t g_stop_time_us;

/** Stops the motors at once and records why; any context, interrupts too. */
EXPORT void app_stop_request (uint32_t reason)
{
    uint32_t irq_state;

    motor_safe_off(); /* first, before anything else */
    APP_LOCK(irq_state);

    if (0u == g_stop_reasons)
    {
        g_stop_time_us = app_timing_now_us();
    }

    g_stop_reasons |= reason;
    APP_UNLOCK(irq_state);
}

/** Clears the given STOP reasons (bit mask). */
EXPORT void app_stop_clear (uint32_t reason)
{
    uint32_t irq_state;

    APP_LOCK(irq_state);
    g_stop_reasons &= ~reason;
    APP_UNLOCK(irq_state);
}

/** The STOP reasons now set; 0 = free to move. */
EXPORT uint32_t app_stop_reasons (void)
{
    return g_stop_reasons;
}

/** app_timing_now_us() at the last STOP request. */
EXPORT uint32_t app_stop_time_us (void)
{
    return g_stop_time_us;
}

/*** end of file ***/

