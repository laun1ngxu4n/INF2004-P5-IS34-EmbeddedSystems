/*
 * websim: the few C library pieces the firmware needs, and tm_printf.
 * No libc is linked (freestanding wasm32), so everything is here.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <stdarg.h>

SIM_IMPORT("sim_log") void sim_log(const char *p, int len);
SIM_IMPORT("gpio_hw_pin_mode") void gpio_hw_pin_mode(uint32_t pin, uint32_t mode);
SIM_IMPORT("gpio_hw_pin_val") void gpio_hw_pin_val(uint32_t pin, uint32_t val);
SIM_IMPORT("pwm_hw_pin") void pwm_hw_pin(uint32_t pin);
SIM_IMPORT("pwm_hw_wrap") void pwm_hw_wrap(uint32_t pin, uint32_t wrap);
SIM_IMPORT("pwm_hw_cc") void pwm_hw_cc(uint32_t pin, uint32_t cc);
SIM_IMPORT("pwm_hw_enable") void pwm_hw_enable(uint32_t pin, uint32_t on);

/* ---- memory ----------------------------------------------------------- */
void *memcpy(void *d, const void *s, size_t n)
{
    uint8_t *dp = d;
    const uint8_t *sp = s;
    while (n--) { *dp++ = *sp++; }
    return d;
}

void *memmove(void *d, const void *s, size_t n)
{
    uint8_t *dp = d;
    const uint8_t *sp = s;
    if (dp < sp) { while (n--) { *dp++ = *sp++; } }
    else { dp += n; sp += n; while (n--) { *--dp = *--sp; } }
    return d;
}

void *memset(void *d, int c, size_t n)
{
    uint8_t *dp = d;
    while (n--) { *dp++ = (uint8_t)c; }
    return d;
}

int memcmp(const void *a, const void *b, size_t n)
{
    const uint8_t *x = a, *y = b;
    while (n--) { if (*x != *y) { return *x - *y; } x++; y++; }
    return 0;
}

size_t strlen(const char *s)
{
    size_t n = 0;
    while (s[n]) { n++; }
    return n;
}

/* ---- printf: %d %i %u %x %X %c %s %%, width, 0 and - flags, l ---------- */
static int put_num(char *out, int pos, int max, uint32_t v, int base,
                   int upper, int neg, int width, int zero, int left)
{
    char tmp[12];
    int  n = 0, i, pad;
    const char *dig = upper ? "0123456789ABCDEF" : "0123456789abcdef";

    do { tmp[n++] = dig[v % (uint32_t)base]; v /= (uint32_t)base; } while (v);
    pad = width - n - (neg ? 1 : 0);
    if (!left && !zero) { for (i = 0; i < pad; i++) { if (pos < max) out[pos] = ' '; pos++; } }
    if (neg) { if (pos < max) out[pos] = '-'; pos++; }
    if (!left && zero) { for (i = 0; i < pad; i++) { if (pos < max) out[pos] = '0'; pos++; } }
    while (n) { if (pos < max) out[pos] = tmp[--n]; else n--; pos++; }
    if (left) { for (i = 0; i < pad; i++) { if (pos < max) out[pos] = ' '; pos++; } }
    return pos;
}

static int vfmt(char *out, int max, const char *f, va_list ap)
{
    int pos = 0;

    for (; *f; f++)
    {
        int width = 0, zero = 0, left = 0;

        if (*f != '%') { if (pos < max) out[pos] = *f; pos++; continue; }
        f++;
        for (;; f++)
        {
            if (*f == '0') zero = 1; else if (*f == '-') left = 1; else break;
        }
        while (*f >= '0' && *f <= '9') { width = width * 10 + (*f - '0'); f++; }
        while (*f == 'l' || *f == 'h' || *f == 'z') f++;
        switch (*f)
        {
        case 'd': case 'i':
        {
            int32_t v = va_arg(ap, int32_t);
            pos = put_num(out, pos, max, v < 0 ? (uint32_t)(-v) : (uint32_t)v, 10, 0, v < 0, width, zero, left);
            break;
        }
        case 'u': pos = put_num(out, pos, max, va_arg(ap, uint32_t), 10, 0, 0, width, zero, left); break;
        case 'x': pos = put_num(out, pos, max, va_arg(ap, uint32_t), 16, 0, 0, width, zero, left); break;
        case 'X': pos = put_num(out, pos, max, va_arg(ap, uint32_t), 16, 1, 0, width, zero, left); break;
        case 'c': { int c = va_arg(ap, int); if (pos < max) out[pos] = (char)c; pos++; break; }
        case 's':
        {
            const char *s = va_arg(ap, const char *);
            int n, i;
            if (!s) s = "(null)";
            n = (int)strlen(s);
            if (!left) for (i = n; i < width; i++) { if (pos < max) out[pos] = ' '; pos++; }
            for (i = 0; i < n; i++) { if (pos < max) out[pos] = s[i]; pos++; }
            if (left) for (i = n; i < width; i++) { if (pos < max) out[pos] = ' '; pos++; }
            break;
        }
        case '%': if (pos < max) out[pos] = '%'; pos++; break;
        default: break;
        }
    }
    return pos;
}

INT tm_sprintf(UB *str, const UB *format, ...)
{
    va_list ap;
    int n;

    va_start(ap, format);
    n = vfmt((char *)str, 0x7fffffff, (const char *)format, ap);
    va_end(ap);
    str[n] = 0;
    return n;
}

INT tm_printf(const UB *format, ...)
{
    char buf[256];
    va_list ap;
    int n;

    va_start(ap, format);
    n = vfmt(buf, (int)sizeof buf - 1, (const char *)format, ap);
    va_end(ap);
    if (n > (int)sizeof buf - 1) n = (int)sizeof buf - 1;
    sim_log(buf, n);
    return n;
}

INT tm_putstring(const UB *str)
{
    sim_log((const char *)str, (int)strlen((const char *)str));
    return 0;
}

/* ---- board helpers (libbsp) ------------------------------------------- */
ER gpio_set_pin(UINT no, UINT mode) { gpio_hw_pin_mode(no, mode); return E_OK; }
ER gpio_set_val(UINT no, UINT val)  { gpio_hw_pin_val(no, val);  return E_OK; }
UINT gpio_get_val(UINT no)          { return (hw_read(GPIO_IN) >> no) & 1u; }
ER pwm_set_pin(UINT no)             { pwm_hw_pin(no); return E_OK; }
ER pwm_set_wrap(UINT no, UW wrap)   { pwm_hw_wrap(no, wrap); return E_OK; }
ER pwm_set_cc(UINT no, UW cc)       { pwm_hw_cc(no, cc); return E_OK; }
ER pwm_set_enabled(UINT no, BOOL e) { pwm_hw_enable(no, e); return E_OK; }
