/**
 * @file    subsystem1.h
 *  Subsystem 1 -- Telemetry / WiFi + Notification (Buddy 1, Muhammad Ikhwan).
 *
 *  This header is subsystem 1's PUBLIC interface: only what other
 *  subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *  files.  Other subsystems include it as "../subsystem1/subsystem1.h".
 *
 *  Stage 5:
 *    telemetry_event()  every subsystem reports results (barcode, junction
 * move, obstacle width, ramp height, faults) through this one call: printed on
 * USB as "[EVT] ..." and published on MQTT topic is34/event. Telemetry task
 * publishes a status line on is34/telemetry every 1 s. MQTT commands (only with
 * "make NET=lwip MQTT=1"): is34/cmd/map    "K=L,Q=R,X=F,M=U"  -> barcode
 * mapping, ack on is34/ack/map is34/cmd/start  (any payload)      -> Mission
 * START is34/cmd/stop   (any payload)      -> STOP at once The MQTT code itself
 * stays in the kernel's WiFi task
 *    (lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c); it calls the two
 *    hooks app_mqtt_rx() / app_mqtt_tx() defined in telemetry.c.
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"

/* timing table every 100 x 100 ms = 10 s. Printing it every 1 s delayed
 * every task by ~12 ms now and then (26 Oct): keep USB output small, see
 * TEAM.md section 4 */
#define S1_REPORT_TICKS 100
#define S1_PING_TICKS   10 /* IF-01 ping every 10 x 100 ms = 1 s */
#define S1_STATUS_TICKS 10 /* is34/telemetry every 1 s */
#define S1_SENSOR_TICKS 5  /* is34/sensors every 0.5 s (laptop panel) */

/* Private to subsystem 1, shared between its own .c files */
IMPORT ID      gh_tid_telemetry;
IMPORT ID      gh_cyc_telemetry;
IMPORT int32_t g_tmg_telemetry;

#include "telemetry.h"

#endif /* SUBSYSTEM1_H */

/*** end of file ***/

