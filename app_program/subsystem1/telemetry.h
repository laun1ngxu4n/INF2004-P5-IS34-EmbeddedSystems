/**
 * @file    telemetry.h
 *  Telemetry + Notification (telemetry.c): MQTT topics, event lines,
 *  the status line.  Part of subsystem 1's interface (subsystem1.h
 *  includes it).
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "../app.h"

#define TOPIC_EVENT      "is34/event"
#define TOPIC_TELEMETRY  "is34/telemetry"
#define TOPIC_ACK_MAP    "is34/ack/map"
#define TOPIC_SENSORS    "is34/sensors"
#define TOPIC_LOG        "is34/log" /* copy of the console (MQTT images) */
#define TOPIC_CMD_PREFIX "is34/cmd/"

#define TELE_TEXT_MAX 160

/** Report one event line (any task; not from interrupt handlers) */
IMPORT void telemetry_event (const uint8_t * text);

/** START commands received over MQTT since boot (the stage tests accept
   the panel's START as well as the button) */
IMPORT uint32_t telemetry_start_count (void);

IMPORT void telemetry_task (int32_t stacd, void * exinf);

#endif /* TELEMETRY_H */

/*** end of file ***/

