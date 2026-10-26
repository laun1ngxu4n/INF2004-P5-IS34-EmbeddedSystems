/*
 *	subsystem1.h
 *	Subsystem 1 -- Telemetry / WiFi + Notification (Buddy 1, Muhammad Ikhwan).
 *
 *	This header is subsystem 1's PUBLIC interface: only what other
 *	subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *	files.  Other subsystems include it as "../subsystem1/subsystem1.h".
 *
 *	Stage 0: the Telemetry task runs every 100 ms (the LED/buzzer tick of
 *	the merged Notification part) and prints the task timing table once a
 *	second.  No WiFi/MQTT work yet: in this template MQTT lives in the
 *	kernel's WiFi service task (lib/liblwip/sysdepend/pico_rp2040/
 *	mqtt_utk.c), see TEAM.md section 3.
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"

#define S1_REPORT_TICKS		10	/* timing table every 10 x 100 ms = 1 s */
#define S1_PING_TICKS		10	/* IF-01 ping every 10 x 100 ms = 1 s */

/* Private to subsystem 1, shared between its own .c files */
IMPORT ID	tid_telemetry;
IMPORT ID	cyc_telemetry;
IMPORT INT	tmg_telemetry;
IMPORT void	telemetry_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM1_H */
