/*
 *	app_stop.c
 *	The one STOP helper (see app_stop.h).
 */

#include "app.h"
#include "app_stop.h"
#include "subsystem2/subsystem2.h"	/* motor_safe_off() */

LOCAL volatile UW	stop_reasons;
LOCAL volatile UW	stop_time_us;

EXPORT void app_stop_request(UW reason)
{
	UINT	s;

	motor_safe_off();			/* first, before anything else */
	APP_LOCK(s);
	if (0u == stop_reasons) {
		stop_time_us = app_time_us();
	}
	stop_reasons |= reason;
	APP_UNLOCK(s);
}

EXPORT void app_stop_clear(UW reason)
{
	UINT	s;

	APP_LOCK(s);
	stop_reasons &= ~reason;
	APP_UNLOCK(s);
}

EXPORT UW app_stop_reasons(void)
{
	return stop_reasons;
}

EXPORT UW app_stop_time_us(void)
{
	return stop_time_us;
}
