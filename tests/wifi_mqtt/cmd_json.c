/*
 * Strict flat-object JSON parser for bring-up commands. See cmd_json.h.
 */
#include <string.h>

#include "cmd_json.h"

typedef struct
{
    const char *p;
    const char *p_end;
} cursor_t;

static void skip_ws(cursor_t *p_c)
{
    while ((p_c->p < p_c->p_end) &&
           ((*p_c->p == ' ') || (*p_c->p == '\t') || (*p_c->p == '\n') || (*p_c->p == '\r')))
    {
        p_c->p++;
    }
}

/* "..." with no escapes and no control characters. */
static bool parse_string(cursor_t *p_c, const char **pp_out, size_t *p_len)
{
    const char *p_start;

    if ((p_c->p >= p_c->p_end) || (*p_c->p != '"'))
    {
        return false;
    }
    p_c->p++;
    p_start = p_c->p;
    while ((p_c->p < p_c->p_end) && (*p_c->p != '"'))
    {
        if ((*p_c->p == '\\') || ((unsigned char)*p_c->p < 0x20u))
        {
            return false;
        }
        p_c->p++;
    }
    if (p_c->p >= p_c->p_end)
    {
        return false;   /* unterminated */
    }
    *pp_out = p_start;
    *p_len = (size_t)(p_c->p - p_start);
    p_c->p++;           /* closing quote */
    return true;
}

static bool parse_uint(cursor_t *p_c, uint32_t *p_out)
{
    uint64_t value = 0u;
    uint32_t digits = 0u;
    const char *p_start = p_c->p;

    while ((p_c->p < p_c->p_end) && (*p_c->p >= '0') && (*p_c->p <= '9'))
    {
        value = (value * 10u) + (uint64_t)(*p_c->p - '0');
        digits++;
        if ((digits > 10u) || (value > 0xFFFFFFFFull))
        {
            return false;
        }
        p_c->p++;
    }
    if (digits == 0u)
    {
        return false;   /* sign, quote-less word, object, array, literal */
    }
    if ((digits > 1u) && (*p_start == '0'))
    {
        return false;   /* leading zero */
    }
    if ((p_c->p < p_c->p_end) &&
        ((*p_c->p == '.') || (*p_c->p == 'e') || (*p_c->p == 'E')))
    {
        return false;   /* fraction or exponent */
    }
    *p_out = (uint32_t)value;
    return true;
}

static bool key_matches(const json_field_t *p_f, const char *p_key)
{
    size_t n = strlen(p_key);
    return (p_f->key_len == n) && (memcmp(p_f->p_key, p_key, n) == 0);
}

bool cmd_json_parse(const char *p_text, size_t len, json_obj_t *p_obj)
{
    cursor_t c;
    uint32_t i;

    c.p = p_text;
    c.p_end = p_text + len;
    p_obj->count = 0u;

    skip_ws(&c);
    if ((c.p >= c.p_end) || (*c.p != '{'))
    {
        return false;
    }
    c.p++;
    skip_ws(&c);

    if ((c.p < c.p_end) && (*c.p == '}'))
    {
        c.p++;          /* empty object */
    }
    else
    {
        for (;;)
        {
            json_field_t f;

            if (p_obj->count >= CMD_JSON_MAX_KEYS)
            {
                return false;
            }
            if (!parse_string(&c, &f.p_key, &f.key_len))
            {
                return false;
            }
            skip_ws(&c);
            if ((c.p >= c.p_end) || (*c.p != ':'))
            {
                return false;
            }
            c.p++;
            skip_ws(&c);
            if ((c.p < c.p_end) && (*c.p == '"'))
            {
                f.type = JV_STRING;
                f.u32 = 0u;
                if (!parse_string(&c, &f.p_str, &f.str_len))
                {
                    return false;
                }
            }
            else
            {
                f.type = JV_UINT;
                f.p_str = NULL;
                f.str_len = 0u;
                if (!parse_uint(&c, &f.u32))
                {
                    return false;
                }
            }
            for (i = 0u; i < p_obj->count; i++)
            {
                if ((p_obj->field[i].key_len == f.key_len) &&
                    (memcmp(p_obj->field[i].p_key, f.p_key, f.key_len) == 0))
                {
                    return false;   /* duplicate key */
                }
            }
            p_obj->field[p_obj->count] = f;
            p_obj->count++;

            skip_ws(&c);
            if ((c.p < c.p_end) && (*c.p == ','))
            {
                c.p++;
                skip_ws(&c);
                continue;   /* a '}' here is a trailing comma: parse_string rejects it */
            }
            if ((c.p < c.p_end) && (*c.p == '}'))
            {
                c.p++;
                break;
            }
            return false;
        }
    }

    skip_ws(&c);
    return (c.p == c.p_end);    /* nothing after the object */
}

static const json_field_t *find(const json_obj_t *p_obj, const char *p_key)
{
    uint32_t i;

    for (i = 0u; i < p_obj->count; i++)
    {
        if (key_matches(&p_obj->field[i], p_key))
        {
            return &p_obj->field[i];
        }
    }
    return NULL;
}

bool cmd_json_get_u32(const json_obj_t *p_obj, const char *p_key, uint32_t *p_out)
{
    const json_field_t *p_f = find(p_obj, p_key);

    if ((p_f == NULL) || (p_f->type != JV_UINT))
    {
        return false;
    }
    *p_out = p_f->u32;
    return true;
}

bool cmd_json_str_equals(const json_obj_t *p_obj, const char *p_key, const char *p_value)
{
    const json_field_t *p_f = find(p_obj, p_key);
    size_t n = strlen(p_value);

    return (p_f != NULL) && (p_f->type == JV_STRING) &&
           (p_f->str_len == n) && (memcmp(p_f->p_str, p_value, n) == 0);
}

bool cmd_json_has(const json_obj_t *p_obj, const char *p_key)
{
    return find(p_obj, p_key) != NULL;
}
