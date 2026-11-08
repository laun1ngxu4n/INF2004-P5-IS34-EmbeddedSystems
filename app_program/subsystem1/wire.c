/**
 * @file    wire.c
 *  The binary MQTT messages (see wire.h and docs/team/MQTT_WIRE_FORMAT.md).
 */

#include "wire.h"

/* the field widths, in message order (MQTT_WIRE_FORMAT.md) */
#define W_MISSION   2u
#define W_LINE      2u
#define W_STOP      3u
#define W_FAULTS    8u
#define W_SPEED     12u
#define W_DIST      20u
#define W_JUNCTIONS 8u
#define W_LETTER    6u
#define W_RAMP      12u
#define W_FLAG      1u
#define W_STEER     8u
#define W_FRONT_ST  2u
#define W_FRONT     13u
#define W_TILT      11u
#define W_HEIGHT    13u
#define MAP_MOVES   4 /* L, R, F, U */

/* a message being written: its bytes and the next bit */
typedef struct
{
    uint8_t * p_buf;
    uint32_t  pos;
} wire_put_t;

/** Starts a message: all bytes 0, the version in byte 0. */
LOCAL void put_start (wire_put_t * p_put, uint8_t * p_out, int32_t len)
{
    int32_t idx;

    for (idx = 0; idx < len; idx++)
    {
        p_out[idx] = 0u;
    }

    p_out[0]     = WIRE_VERSION;
    p_put->p_buf = p_out;
    p_put->pos   = 8u; /* fields start at bit 0 of byte 1 */
}

/** The low bits of value, lowest first. */
LOCAL void put_bits (wire_put_t * p_put, uint32_t value, uint32_t bits)
{
    uint32_t idx;

    for (idx = 0u; idx < bits; idx++)
    {
        if (0u != ((value >> idx) & 1u))
        {
            /* a bit position 0..7: fits uint8_t */
            p_put->p_buf[p_put->pos >> 3] |= (uint8_t)(1u << (p_put->pos & 7u));
        }

        p_put->pos++;
    }
}

/** An unsigned field, clamped to 0 .. 2^bits - 1. */
LOCAL void put_unsigned (wire_put_t * p_put, uint32_t value, uint32_t bits)
{
    uint32_t top = (1u << bits) - 1u;

    put_bits(p_put, (value > top) ? top : value, bits);
}

/** A signed field (two's complement), clamped to its range. */
LOCAL void put_signed (wire_put_t * p_put, int32_t value, uint32_t bits)
{
    /* bits <= 20: 2^(bits - 1) fits int32_t */
    int32_t high    = (int32_t)(1u << (bits - 1u)) - 1;
    int32_t low     = -high - 1;
    int32_t clamped = (value < low) ? low : ((value > high) ? high : value);

    /* the two's complement bits of a value in range; only the low bits
       are kept */
    put_bits(p_put, (uint32_t)clamped & ((1u << bits) - 1u), bits);
}

/** The 6-bit code of a barcode letter: 0 none, 1-26 A-Z, 27-36 0-9,
   WIRE_LETTER_OTHER any other Code 39 character. */
EXPORT uint32_t wire_letter_code (uint8_t letter)
{
    uint32_t code = WIRE_LETTER_OTHER;

    if (0u == letter)
    {
        code = WIRE_LETTER_NONE;
    }
    else if ((letter >= 'A') && (letter <= 'Z'))
    {
        code = 1u + (uint32_t)(letter - 'A'); /* 0..25: not negative */
    }
    else if ((letter >= '0') && (letter <= '9'))
    {
        code = 27u + (uint32_t)(letter - '0'); /* 0..9: not negative */
    }
    else
    {
        /* any other character: WIRE_LETTER_OTHER */
    }

    return code;
}

/** The letter of a 6-bit code (wire_letter_code backwards); 0 for none
   or a code that is no letter. */
EXPORT uint8_t wire_code_letter (uint32_t code)
{
    uint8_t letter = 0u;

    if ((code >= 1u) && (code <= 26u))
    {
        letter = (uint8_t)('A' + (code - 1u)); /* 'A'..'Z' */
    }
    else if ((code >= 27u) && (code <= 36u))
    {
        letter = (uint8_t)('0' + (code - 27u)); /* '0'..'9' */
    }
    else
    {
        /* none, or not a letter: 0 */
    }

    return letter;
}

/** The status message (WIRE_STATUS_LEN bytes); its length. */
EXPORT int32_t wire_status_pack (const wire_status_t * p_in, uint8_t * p_out)
{
    wire_put_t put;
    int32_t    idx;

    put_start(&put, p_out, WIRE_STATUS_LEN);
    put_unsigned(&put, p_in->mission_state, W_MISSION);
    put_unsigned(&put, p_in->line_state, W_LINE);
    put_unsigned(&put, p_in->stop_reasons, W_STOP);
    put_unsigned(&put, p_in->faults, W_FAULTS);
    put_signed(&put, p_in->speed_l, W_SPEED);
    put_signed(&put, p_in->speed_r, W_SPEED);
    put_signed(&put, p_in->dist_mm, W_DIST);
    put_bits(&put, p_in->junctions, W_JUNCTIONS); /* mod 256 */
    put_unsigned(&put, wire_letter_code(p_in->last_letter), W_LETTER);

    for (idx = 0; idx < MAP_MOVES; idx++)
    {
        put_unsigned(&put, wire_letter_code(p_in->map_letter[idx]), W_LETTER);
    }

    /* clamped at 0 first: a negative value is no height */
    put_unsigned(
        &put, (p_in->ramp_peak_mm10 < 0) ? 0u : (uint32_t)p_in->ramp_peak_mm10,
        W_RAMP);
    return WIRE_STATUS_LEN;
}

/** The sensors message (WIRE_SENSORS_LEN bytes); its length. */
EXPORT int32_t wire_sensors_pack (const wire_sensors_t * p_in, uint8_t * p_out)
{
    wire_put_t put;
    uint32_t   front_status = WIRE_FRONT_MM;
    uint32_t   front_mm     = 0u;

    if (-1 == p_in->front_mm)
    {
        front_status = WIRE_FRONT_OPEN; /* SONAR_OPEN */
    }
    else if (p_in->front_mm < 0)
    {
        front_status = WIRE_FRONT_FAULT; /* SONAR_FAULT */
    }
    else
    {
        front_mm = (uint32_t)p_in->front_mm; /* >= 0 here */
    }

    put_start(&put, p_out, WIRE_SENSORS_LEN);
    put_unsigned(&put, p_in->b_ir_left ? 1u : 0u, W_FLAG);
    put_unsigned(&put, p_in->b_ir_right ? 1u : 0u, W_FLAG);
    put_unsigned(&put, p_in->b_ir_bar ? 1u : 0u, W_FLAG);
    put_unsigned(&put, p_in->b_hold ? 1u : 0u, W_FLAG);
    put_unsigned(&put, p_in->b_inbar ? 1u : 0u, W_FLAG);
    put_unsigned(&put, p_in->b_imu_ok ? 1u : 0u, W_FLAG);
    put_signed(&put, p_in->steer, W_STEER);
    put_unsigned(&put, front_status, W_FRONT_ST);
    put_unsigned(&put, front_mm, W_FRONT);
    put_signed(&put, p_in->tilt_x1000, W_TILT);
    put_signed(&put, p_in->height_mm10, W_HEIGHT);
    return WIRE_SENSORS_LEN;
}

/** The mapping acknowledgement (WIRE_ACK_LEN bytes): accepted or not, and
   the mapping now in use (the letter for L, R, F, U; 0 = none). */
EXPORT int32_t wire_ack_pack (bool b_ok, const uint8_t * p_map_letter,
                              uint8_t * p_out)
{
    wire_put_t put;
    int32_t    idx;

    put_start(&put, p_out, WIRE_ACK_LEN);
    put_unsigned(&put, b_ok ? 1u : 0u, W_FLAG);
    put_unsigned(&put, (0u != p_map_letter[0]) ? 1u : 0u, W_FLAG);

    for (idx = 0; idx < MAP_MOVES; idx++)
    {
        put_unsigned(&put, wire_letter_code(p_map_letter[idx]), W_LETTER);
    }

    return WIRE_ACK_LEN;
}

/** A mapping command (WIRE_MAP_LEN bytes) into the letters for L, R, F, U.
   false (and nothing written) for a wrong length, a wrong version or a
   code that is no letter. */
EXPORT bool wire_map_unpack (const uint8_t * p_in, int32_t len,
                             uint8_t * p_map_letter)
{
    uint8_t  letters[MAP_MOVES];
    uint32_t bits = 0u;
    int32_t  idx;
    bool     b_ok = (WIRE_MAP_LEN == len) && (WIRE_VERSION == p_in[0]);

    if (b_ok)
    {
        /* bytes 1-3 (the length was checked), little-endian: 24 bits, 6
           per move; bytes widened, nothing lost */
        bits = (uint32_t)p_in[1] | ((uint32_t)p_in[2] << 8) |
               ((uint32_t)p_in[3] << 16);
    }

    for (idx = 0; idx < MAP_MOVES; idx++)
    {
        /* idx 0..3: shift 0..18 */
        letters[idx] = wire_code_letter((bits >> ((uint32_t)idx * W_LETTER)) &
                                        ((1u << W_LETTER) - 1u));
        b_ok         = b_ok && (letters[idx] != 0u);
    }

    for (idx = 0; b_ok && (idx < MAP_MOVES); idx++)
    {
        p_map_letter[idx] = letters[idx];
    }

    return b_ok;
}

/*** end of file ***/

