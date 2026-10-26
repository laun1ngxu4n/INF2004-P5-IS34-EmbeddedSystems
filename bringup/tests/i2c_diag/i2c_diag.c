/*
 * P5-IS34 I2C / Grove diagnostic (IMU not answering)
 *
 * Answers three questions without a multimeter:
 *   1. Line levels on every Grove signal pin: is something powered pulling
 *      it up, is it floating (nothing connected / unpowered), or is it held
 *      low (shorted, or an unpowered chip clamping it)?
 *   2. Slow software I2C scan (about 10 kHz, works even with weak pull-ups)
 *      on EVERY Grove port, in BOTH wire orders. Finds the IMU even if it is
 *      on the wrong port or SDA/SCL are swapped.
 *   3. If a device answers: chip ID bytes (LSM303DLHC mag "H43", or the
 *      LSM303AGR / other variants' WHO_AM_I registers).
 *
 * Pins are only ever released (input with pull-up) or pulled LOW; nothing
 * is driven high. Motor, servo, LED and buzzer pins are not touched. Still,
 * unplug every Grove part except the IMU: pulling a push-pull sensor
 * output (e.g. HC-SR04 ECHO) low fights it.
 *
 * Runs once about 3 s after reset, then again every 10 s or on a GP21 press.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "../common/chip_id.h"

#define BTN_RUN_PIN      21u
#define HALF_BIT_US      50u      /* about 10 kHz */
#define REPEAT_MS        10000u
#define NUM_PORTS        7u

typedef struct
{
    const char *p_name;
    uint8_t     pin_a;   /* lower-numbered signal on the port, per datasheet */
    uint8_t     pin_b;
} grove_port_t;

static const grove_port_t g_ports[NUM_PORTS] =
{
    {"Grove 1", 0u,  1u},
    {"Grove 2", 2u,  3u},
    {"Grove 3", 4u,  5u},
    {"Grove 4", 16u, 17u},
    {"Grove 5", 6u,  26u},
    {"Grove 6", 26u, 27u},
    {"Grove 7", 7u,  28u},
};

/* ---- Pin helpers ---------------------------------------------------- */

static void pin_release_all(void)
{
    uint32_t i;
    for (i = 0u; i < NUM_PORTS; i++)
    {
        gpio_init(g_ports[i].pin_a);
        gpio_init(g_ports[i].pin_b);
        gpio_set_dir(g_ports[i].pin_a, GPIO_IN);
        gpio_set_dir(g_ports[i].pin_b, GPIO_IN);
        gpio_disable_pulls(g_ports[i].pin_a);
        gpio_disable_pulls(g_ports[i].pin_b);
    }
}

typedef enum
{
    LVL_PULLED_UP,   /* reads high even with internal pull-down: external pull-up / powered device */
    LVL_FLOATING,    /* follows the internal pull: nothing powered on it */
    LVL_HELD_LOW     /* reads low even with internal pull-up: shorted or clamped */
} level_t;

static level_t pin_level(uint8_t pin)
{
    bool hi_with_down;
    bool hi_with_up;

    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_down(pin);
    sleep_ms(2u);
    hi_with_down = gpio_get(pin);
    gpio_pull_up(pin);
    sleep_ms(2u);
    hi_with_up = gpio_get(pin);
    gpio_disable_pulls(pin);

    if (hi_with_down)
    {
        return LVL_PULLED_UP;
    }
    if (!hi_with_up)
    {
        return LVL_HELD_LOW;
    }
    return LVL_FLOATING;
}

static const char *level_name(level_t lvl)
{
    if (lvl == LVL_PULLED_UP)
    {
        return "PULLED UP";
    }
    if (lvl == LVL_HELD_LOW)
    {
        return "HELD LOW";
    }
    return "floating";
}

/* ---- Software I2C (open drain: release = high, drive = low) ---------- */

static uint8_t g_sda;
static uint8_t g_scl;
static bool    g_scl_fail;   /* SCL did not rise at least once (held low / clock stretch) */

static void line_release(uint8_t pin)
{
    gpio_set_dir(pin, GPIO_IN);
}

static void line_low(uint8_t pin)
{
    gpio_put(pin, 0);
    gpio_set_dir(pin, GPIO_OUT);
}

static bool scl_high(void)
{
    uint32_t t = 0u;
    line_release(g_scl);
    while (!gpio_get(g_scl))
    {
        sleep_us(10u);
        t += 10u;
        if (t > 2000u)
        {
            g_scl_fail = true;   /* reported per port: results there are unreliable */
            return false;        /* SCL never rose */
        }
    }
    sleep_us(HALF_BIT_US);
    return true;
}

static void sw_setup(uint8_t sda, uint8_t scl)
{
    g_sda = sda;
    g_scl = scl;
    gpio_init(sda);
    gpio_init(scl);
    gpio_pull_up(sda);
    gpio_pull_up(scl);
    line_release(sda);
    line_release(scl);
    sleep_us(200u);
}

static void sw_start(void)
{
    line_release(g_sda);
    (void)scl_high();
    line_low(g_sda);
    sleep_us(HALF_BIT_US);
    line_low(g_scl);
    sleep_us(HALF_BIT_US);
}

static void sw_stop(void)
{
    line_low(g_sda);
    sleep_us(HALF_BIT_US);
    (void)scl_high();
    line_release(g_sda);
    sleep_us(HALF_BIT_US);
}

/* Returns true if the byte was ACKed. */
static bool sw_write(uint8_t byte)
{
    int32_t bit;
    bool ack;

    for (bit = 7; bit >= 0; bit--)
    {
        if (((byte >> bit) & 1u) != 0u)
        {
            line_release(g_sda);
        }
        else
        {
            line_low(g_sda);
        }
        sleep_us(HALF_BIT_US);
        (void)scl_high();
        line_low(g_scl);
    }
    line_release(g_sda);
    sleep_us(HALF_BIT_US);
    (void)scl_high();
    ack = !gpio_get(g_sda);
    line_low(g_scl);
    sleep_us(HALF_BIT_US);
    return ack;
}

static uint8_t sw_read(bool ack)
{
    uint8_t value = 0u;
    uint32_t bit;

    line_release(g_sda);
    for (bit = 0u; bit < 8u; bit++)
    {
        sleep_us(HALF_BIT_US);
        (void)scl_high();
        value = (uint8_t)((value << 1) | (gpio_get(g_sda) ? 1u : 0u));
        line_low(g_scl);
    }
    if (ack)
    {
        line_low(g_sda);
    }
    sleep_us(HALF_BIT_US);
    (void)scl_high();
    line_low(g_scl);
    line_release(g_sda);
    return value;
}

static void sw_bus_recover(void)
{
    uint32_t i;
    for (i = 0u; i < 9u; i++)
    {
        line_low(g_scl);
        sleep_us(HALF_BIT_US);
        (void)scl_high();
    }
    sw_stop();
}

static bool sw_probe(uint8_t addr)
{
    bool ack;
    sw_start();
    ack = sw_write((uint8_t)(addr << 1));
    sw_stop();
    return ack;
}

static bool sw_read_reg(uint8_t addr, uint8_t reg, uint8_t *p_val)
{
    sw_start();
    if (!sw_write((uint8_t)(addr << 1)) || !sw_write(reg))
    {
        sw_stop();
        return false;
    }
    sw_start();
    if (!sw_write((uint8_t)((addr << 1) | 1u)))
    {
        sw_stop();
        return false;
    }
    *p_val = sw_read(false);
    sw_stop();
    return true;
}

static void sw_finish(void)
{
    line_release(g_sda);
    line_release(g_scl);
    gpio_disable_pulls(g_sda);
    gpio_disable_pulls(g_scl);
}

/* Read the ID registers used by the common accelerometer / magnetometer
 * chips that can sit at 0x1C-0x1F, name the match and return its kind. */
static chip_kind_t identify(uint8_t addr)
{
    static const uint8_t regs[5] = {0x00u, 0x0Du, 0x0Fu, 0x0Au, 0x4Fu};
    uint8_t v[5] = {0u, 0u, 0u, 0u, 0u};
    bool ok[5];
    uint32_t i;
    chip_match_t m;

    for (i = 0u; i < 5u; i++)
    {
        ok[i] = sw_read_reg(addr, regs[i], &v[i]);
    }
    printf("    0x%02X ID regs: [0x00]=%s%02X [0x0D]=%s%02X [0x0F]=%s%02X [0x0A]=%s%02X [0x4F]=%s%02X\n",
           addr, ok[0] ? "" : "?", v[0], ok[1] ? "" : "?", v[1], ok[2] ? "" : "?", v[2],
           ok[3] ? "" : "?", v[3], ok[4] ? "" : "?", v[4]);
    m = chip_from_ids(ok, v);
    printf("    0x%02X looks like: %s\n", addr, m.p_name);
    return m.kind;
}

/* What the scan found, for the verdict. */
typedef struct
{
    bool        dlhc;          /* something at 0x19 or 0x1E (LSM303DLHC addresses) */
    bool        lsm303d;       /* WHO_AM_I 0x49 found */
    bool        other_known;   /* a recognised chip we do not support */
    const char *p_port;        /* where the supported chip answered */
    uint8_t     sda;
    uint8_t     scl;
    uint8_t     addr;
} scan_result_t;

/* Scan one pin order. Returns number of devices found. */
static uint32_t scan_order(const char *p_port, uint8_t sda, uint8_t scl, scan_result_t *p_res)
{
    uint32_t found = 0u;
    uint8_t addr;

    sw_setup(sda, scl);
    g_scl_fail = false;
    if (!gpio_get(sda) || !gpio_get(scl))
    {
        sw_bus_recover();
    }
    if (!gpio_get(sda) || !gpio_get(scl))
    {
        printf("  %s SDA=GP%u SCL=GP%u: bus stuck low (SDA %s, SCL %s), not scanned\n",
               p_port, sda, scl, gpio_get(sda) ? "high" : "LOW", gpio_get(scl) ? "high" : "LOW");
        sw_finish();
        return 0u;
    }

    for (addr = 0x08u; addr < 0x78u; addr++)
    {
        if (sw_probe(addr))
        {
            if (found == 0u)
            {
                printf("  %s SDA=GP%u SCL=GP%u: FOUND", p_port, sda, scl);
            }
            printf(" 0x%02X", addr);
            found++;
        }
    }
    if (found > 0u)
    {
        uint8_t v0 = 0u;
        uint8_t v1 = 0u;
        uint8_t v2 = 0u;
        printf("\n");
        for (addr = 0x08u; addr < 0x78u; addr++)
        {
            if (sw_probe(addr))
            {
                chip_kind_t kind = identify(addr);
                if ((kind == CHIP_LSM303D) && !p_res->lsm303d)
                {
                    p_res->lsm303d = true;
                    p_res->p_port = p_port;
                    p_res->sda = sda;
                    p_res->scl = scl;
                    p_res->addr = addr;
                }
                else if (kind == CHIP_OTHER_KNOWN)
                {
                    p_res->other_known = true;
                }
                else
                {
                    /* unknown: keep scanning */
                }
            }
        }
        if (sw_probe(0x1Eu))
        {
            bool ok = sw_read_reg(0x1Eu, 0x0Au, &v0) && sw_read_reg(0x1Eu, 0x0Bu, &v1) &&
                      sw_read_reg(0x1Eu, 0x0Cu, &v2);
            printf("    mag 0x1E ID regs 0x0A-0x0C: %s %02X %02X %02X ('%c%c%c'; LSM303DLHC = 'H43')\n",
                   ok ? "" : "(read failed)", v0, v1, v2,
                   (v0 >= 0x20u && v0 < 0x7Fu) ? v0 : '.', (v1 >= 0x20u && v1 < 0x7Fu) ? v1 : '.',
                   (v2 >= 0x20u && v2 < 0x7Fu) ? v2 : '.');
            if (sw_read_reg(0x1Eu, 0x4Fu, &v0))
            {
                printf("    mag 0x1E WHO_AM_I 0x4F: %02X (LSM303AGR = 40)\n", v0);
            }
            p_res->dlhc = true;
        }
        if (sw_probe(0x19u))
        {
            if (sw_read_reg(0x19u, 0x0Fu, &v0))
            {
                printf("    accel 0x19 reg 0x0F: %02X (LSM303AGR = 33; DLHC has no ID here)\n", v0);
            }
            p_res->dlhc = true;
        }
    }
    else
    {
        printf("  %s SDA=GP%u SCL=GP%u: nothing answered\n", p_port, sda, scl);
    }
    if (g_scl_fail)
    {
        printf("  %s SDA=GP%u SCL=GP%u: WARNING SCL did not rise at least once during the scan "
               "(held low or a slow pull-up) - results for this port are unreliable\n", p_port, sda, scl);
    }
    sw_finish();
    return found;
}

/* Hardware I2C1 on GP26/GP27, the pins the real firmware uses. */
static bool hw_reg(uint8_t addr, uint8_t reg, uint8_t *p_val)
{
    if (i2c_write_timeout_us(i2c1, addr, &reg, 1u, true, 2000u) != 1)
    {
        return false;
    }
    return i2c_read_timeout_us(i2c1, addr, p_val, 1u, false, 2000u) == 1;
}

static void hw_check(uint32_t hz)
{
    static const uint8_t regs[7] = {0x00u, 0x0Au, 0x0Bu, 0x0Cu, 0x0Du, 0x0Fu, 0x4Fu};
    uint8_t addr;
    uint8_t dummy;
    uint32_t i;

    i2c_init(i2c1, hz);
    gpio_set_function(26u, GPIO_FUNC_I2C);
    gpio_set_function(27u, GPIO_FUNC_I2C);
    gpio_pull_up(26u);
    gpio_pull_up(27u);
    sleep_ms(2u);

    printf("  hardware I2C1 GP26/GP27 @ %lu kHz scan:", (unsigned long)(hz / 1000u));
    for (addr = 0x08u; addr < 0x78u; addr++)
    {
        if (i2c_read_timeout_us(i2c1, addr, &dummy, 1u, false, 2000u) >= 0)
        {
            printf(" 0x%02X", addr);
        }
    }
    printf("\n");

    for (addr = 0x19u; addr <= 0x1Eu; addr++)
    {
        if ((addr != 0x19u) && (addr != 0x1Du) && (addr != 0x1Eu))
        {
            continue;
        }
        printf("    hw 0x%02X regs:", addr);
        for (i = 0u; i < 7u; i++)
        {
            uint8_t v = 0u;
            if (hw_reg(addr, regs[i], &v))
            {
                printf(" [0x%02X]=%02X", regs[i], v);
            }
            else
            {
                printf(" [0x%02X]=--", regs[i]);
            }
        }
        printf("\n");
    }

    i2c_deinit(i2c1);
    gpio_set_function(26u, GPIO_FUNC_SIO);
    gpio_set_function(27u, GPIO_FUNC_SIO);
    gpio_set_dir(26u, GPIO_IN);
    gpio_set_dir(27u, GPIO_IN);
    gpio_disable_pulls(26u);
    gpio_disable_pulls(27u);
}

static void run_diag(uint32_t run)
{
    uint32_t i;
    scan_result_t res = {false, false, false, NULL, 0u, 0u, 0u};
    uint32_t total = 0u;
    level_t g6a;
    level_t g6b;

    printf("\n=== I2C / Grove diagnostic v3.2, run %lu ===\n", (unsigned long)run);
    pin_release_all();

    printf("[1] Line levels (PULLED UP = a powered board's pull-up is on it; floating = nothing powered;"
           " HELD LOW = shorted or unpowered chip)\n");
    for (i = 0u; i < NUM_PORTS; i++)
    {
        level_t a = pin_level(g_ports[i].pin_a);
        level_t b = pin_level(g_ports[i].pin_b);
        printf("  %s: GP%u %-9s GP%u %s\n", g_ports[i].p_name, g_ports[i].pin_a, level_name(a),
               g_ports[i].pin_b, level_name(b));
    }
    g6a = pin_level(26u);
    g6b = pin_level(27u);

    printf("[2] Slow software I2C scan, every Grove port, both wire orders\n");
    for (i = 0u; i < NUM_PORTS; i++)
    {
        total += scan_order(g_ports[i].p_name, g_ports[i].pin_a, g_ports[i].pin_b, &res);
        total += scan_order(g_ports[i].p_name, g_ports[i].pin_b, g_ports[i].pin_a, &res);
    }

    printf("[3] Hardware I2C on the design pins\n");
    hw_check(100000u);
    hw_check(400000u);

    printf("[verdict] ");
    if (res.lsm303d)
    {
        bool design = (res.sda == 26u) && (res.scl == 27u);
        printf("LSM303D found at 0x%02X on %s with SDA=GP%u SCL=GP%u%s (supported by imu_test and "
               "parts_check v3.1+).\n", res.addr, res.p_port, res.sda, res.scl,
               design ? " - the design pins, wires the right way round"
                      : " - NOT the design order: move or swap the wires to SDA=GP26 SCL=GP27");
    }
    else if (res.dlhc)
    {
        printf("something answers at 0x19/0x1E (LSM303DLHC addresses) - see the FOUND line(s) for "
               "the port and wire order.\n");
    }
    else if (res.other_known)
    {
        printf("a recognised chip answers but not one our firmware supports - see the 'looks like' "
               "line(s).\n");
    }
    else if (total > 0u)
    {
        printf("something answers but its ID is not in the table - see the FOUND and ID lines.\n");
    }
    else if ((g6a == LVL_HELD_LOW) || (g6b == LVL_HELD_LOW))
    {
        printf("nothing answers; Grove 6 line(s) HELD LOW - IMU likely unpowered (no 3.3 V at its VIN)"
               " or SDA/SCL touching GND.\n");
    }
    else if ((g6a == LVL_FLOATING) && (g6b == LVL_FLOATING))
    {
        printf("nothing answers; Grove 6 lines float - no powered board is connected to them"
               " (wrong port, cable/contact fault, or VIN not powered).\n");
    }
    else
    {
        printf("nothing answers although Grove 6 has pull-ups - board powered but chip not responding"
               " (check SDA/SCL contact, or a faulty module).\n");
    }
    printf("Next run in 10 s, or press GP21.\n");
}

int main(void)
{
    uint32_t run = 1u;
    uint32_t next;

    stdio_init_all();
    gpio_init(BTN_RUN_PIN);
    gpio_set_dir(BTN_RUN_PIN, GPIO_IN);
    gpio_pull_up(BTN_RUN_PIN);

    sleep_ms(3000u);
    for (;;)
    {
        run_diag(run);
        run++;
        next = to_ms_since_boot(get_absolute_time()) + REPEAT_MS;
        while ((int32_t)(to_ms_since_boot(get_absolute_time()) - next) < 0)
        {
            if (!gpio_get(BTN_RUN_PIN))
            {
                sleep_ms(50u);
                while (!gpio_get(BTN_RUN_PIN))
                {
                    sleep_ms(10u);
                }
                break;
            }
            sleep_ms(10u);
        }
    }
}
