/**
 * @file    app_stop.c
 *  The one STOP helper (see app_stop.h).
 */

#include "app.h"
#include "app_stop.h"
#include "subsystem2/subsystem2.h" /* motor_safe_off() */

/* The STOP reasons and the fault bits, one word (Change Requirement 1),
   changed only under APP_LOCK.  The IMU counts as not answering until it
   has answered once. */
typedef struct
{
    uint32_t stop   APP_FIELD(STOP_BITS);  /* STOP_... */
    uint32_t faults APP_FIELD(FAULT_BITS); /* FAULT_... */
} stop_word_t;

LOCAL volatile stop_word_t g_word = {0u, FAULT_IMU};
LOCAL volatile uint32_t    g_stop_time_us; /* a time in us: all 32 bits */

/** Stops the motors at once and records why; any context, interrupts too. */
EXPORT void app_stop_request (uint32_t reason)
{
    uint32_t irq_state;

    motor_safe_off(); /* first, before anything else */
    APP_LOCK(irq_state);

    if (0u == g_word.stop)
    {
        g_stop_time_us = app_timing_now_us();
    }

    g_word.stop |= reason;
    APP_UNLOCK(irq_state);
}

/** Clears the given STOP reasons (bit mask). */
EXPORT void app_stop_clear (uint32_t reason)
{
    uint32_t irq_state;

    APP_LOCK(irq_state);
    g_word.stop &= ~reason;
    APP_UNLOCK(irq_state);
}

/** The STOP reasons now set; 0 = free to move. */
EXPORT uint32_t app_stop_reasons (void)
{
    return g_word.stop;
}

/** app_timing_now_us() at the last STOP request. */
EXPORT uint32_t app_stop_time_us (void)
{
    return g_stop_time_us;
}

/** Sets fault bits; any context, interrupts too. */
EXPORT void app_stop_fault_set (uint32_t mask)
{
    uint32_t irq_state;

    APP_LOCK(irq_state);
    g_word.faults |= mask;
    APP_UNLOCK(irq_state);
}

/** Clears fault bits; any context, interrupts too. */
EXPORT void app_stop_fault_clear (uint32_t mask)
{
    uint32_t irq_state;

    APP_LOCK(irq_state);
    g_word.faults &= ~mask;
    APP_UNLOCK(irq_state);
}

/** The FAULT_... bits set now. */
EXPORT uint32_t app_stop_faults (void)
{
    return g_word.faults;
}

/*** end of file ***/

