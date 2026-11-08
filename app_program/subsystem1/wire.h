/**
 * @file    wire.h
 *  The binary MQTT messages (Change Requirement 1, Minimum-Bit Data
 *  Representation).  Format: docs/team/MQTT_WIRE_FORMAT.md.
 *
 *  Byte 0 is the format version (WIRE_VERSION).  Then the fields, in a
 *  fixed order, each in the bits its range needs, packed with explicit
 *  shifts (never a C bit-field: the compiler chooses their order): field
 *  bits are written lowest bit first, starting at bit 0 of byte 1, so the
 *  whole message after byte 0 is one little-endian number.  Signed fields
 *  are two's complement in their width.  A value outside its field's
 *  range is clamped to it; spare bits at the end are 0.
 *
 *  Pure functions, no kernel calls: tools/panel/wire_golden.c builds this
 *  file on the laptop and checks it against the panel's decoder.
 */

#ifndef WIRE_H
#define WIRE_H

#ifndef EXPORT
#include <tk/tkernel.h> /* LOCAL / EXPORT / IMPORT (the laptop build
                           defines them itself) */
#endif

#include "../app_types.h"

#define WIRE_VERSION     1u
#define WIRE_STATUS_LEN  15 /* is34/telemetry, every 1 s */
#define WIRE_SENSORS_LEN 8  /* is34/sensors, every 0.5 s */
#define WIRE_ACK_LEN     5  /* is34/ack/map */
#define WIRE_MAP_LEN     4  /* is34/cmd/map (laptop -> robot) */
#define WIRE_MAX_LEN     15 /* the longest of them */

#define WIRE_LETTER_NONE  0u  /* no letter */
#define WIRE_LETTER_OTHER 63u /* a Code 39 character not A-Z / 0-9 */

#define WIRE_FRONT_MM    0u /* sensors: front_status */
#define WIRE_FRONT_OPEN  1u
#define WIRE_FRONT_FAULT 2u

/* The values that go into the status message (plain C values) */
typedef struct
{
    uint32_t mission_state; /* 0 IDLE, 1 CAL, 2 RUN, 3 MANOEUVRE */
    uint32_t line_state;    /* LINE_ON .. LINE_FAULT */
    uint32_t stop_reasons;  /* STOP_... bits */
    uint32_t faults;        /* FAULT_... bits */
    int32_t  speed_l;       /* mm/s */
    int32_t  speed_r;       /* mm/s */
    int32_t  dist_mm;       /* odometer since boot */
    uint32_t junctions;     /* since boot (sent mod 256) */
    uint8_t  last_letter;   /* last barcode read (ASCII), 0 = none */
    uint8_t  map_letter[4]; /* the letter for L, R, F, U (ASCII), 0 =
                               no mapping */
    int32_t ramp_peak_mm10; /* last ramp reported, 0.1 mm */
} wire_status_t;

/* The values that go into the sensors message */
typedef struct
{
    bool    b_ir_left; /* line sensors and barcode sensor: black */
    bool    b_ir_right;
    bool    b_ir_bar;
    bool    b_hold;  /* steering held straight (B over bars) */
    bool    b_inbar; /* inside a barcode */
    bool    b_imu_ok;
    int32_t steer;      /* mm/s */
    int32_t front_mm;   /* mm, SONAR_OPEN (-1) or SONAR_FAULT (-2) */
    int32_t tilt_x1000; /* sin(pitch) x 1000 */
    int32_t height_mm10;
} wire_sensors_t;

IMPORT uint32_t wire_letter_code (uint8_t letter);
IMPORT uint8_t  wire_code_letter (uint32_t code);
IMPORT int32_t  wire_status_pack (const wire_status_t * p_in, uint8_t * p_out);
IMPORT int32_t wire_sensors_pack (const wire_sensors_t * p_in, uint8_t * p_out);
IMPORT int32_t wire_ack_pack (bool b_ok, const uint8_t * p_map_letter,
                              uint8_t * p_out);
IMPORT bool    wire_map_unpack (const uint8_t * p_in, int32_t len,
                                uint8_t * p_map_letter);

#endif /* WIRE_H */

/*** end of file ***/

