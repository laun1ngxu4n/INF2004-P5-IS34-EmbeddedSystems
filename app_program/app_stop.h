/*
 *	app_stop.h
 *	The one STOP helper (TEAM.md section 5, IF-02 / IF-05).  Shared by the
 *	whole team.
 *
 *	Anything that must stop the robot calls app_stop_request() with its
 *	reason bit: the motors go off at once, from any context (task, cyclic
 *	or interrupt handler), and stay off while any reason bit is set.
 *	Motion cancels its command when it sees a reason; whoever owns a
 *	reason clears it (STOP button: released and START pressed).
 */

#ifndef APP_STOP_H
#define APP_STOP_H

#include <tk/tkernel.h>

#define STOP_BUTTON	0x0001u		/* GP21 pressed (S5) */
#define STOP_MQTT	0x0002u		/* STOP from the PC (S1, later) */
#define STOP_FAULT	0x0004u		/* Safety fault (S4, later) */

IMPORT void	app_stop_request(UW reason);	/* any context */
IMPORT void	app_stop_clear(UW reason);
IMPORT UW	app_stop_reasons(void);		/* 0 = free to move */
IMPORT UW	app_stop_time_us(void);		/* app_time_us() of the last request */

#endif /* APP_STOP_H */
