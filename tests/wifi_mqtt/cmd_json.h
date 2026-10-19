/*
 * Strict parser for the flat JSON command objects used by the bring-up
 * test, e.g. {"boot_id":123,"seq":4,"cmd":"ping"}.
 *
 * Accepted: one top-level object; keys are plain strings; values are
 * unsigned integers (0..4294967295, no sign, no leading zeros, no
 * fraction/exponent) or strings without escapes. JSON whitespace is
 * allowed anywhere between tokens.
 * Rejected: nesting, arrays, true/false/null, duplicate keys, trailing
 * commas, anything after the closing brace, more than CMD_JSON_MAX_KEYS.
 *
 * Pure C with no Pico SDK dependency, so it can be tested on the host.
 */
#ifndef CMD_JSON_H
#define CMD_JSON_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CMD_JSON_MAX_KEYS   8u

typedef enum
{
    JV_UINT = 0,
    JV_STRING
} json_vtype_t;

typedef struct
{
    const char  *p_key;     /* points into the source text (not terminated) */
    size_t       key_len;
    json_vtype_t type;
    uint32_t     u32;       /* JV_UINT */
    const char  *p_str;     /* JV_STRING, points into the source text */
    size_t       str_len;
} json_field_t;

typedef struct
{
    json_field_t field[CMD_JSON_MAX_KEYS];
    uint32_t     count;
} json_obj_t;

/* Parses p_text[0..len). Returns false on any rule above. */
bool cmd_json_parse(const char *p_text, size_t len, json_obj_t *p_obj);

/* Lookups on a parsed object. Return false if missing or wrong type. */
bool cmd_json_get_u32(const json_obj_t *p_obj, const char *p_key, uint32_t *p_out);
bool cmd_json_str_equals(const json_obj_t *p_obj, const char *p_key, const char *p_value);
bool cmd_json_has(const json_obj_t *p_obj, const char *p_key);

#endif /* CMD_JSON_H */
