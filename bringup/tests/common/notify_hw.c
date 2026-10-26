/*
 * NeoPixel (PIO) and buzzer (PWM) helpers. See notify_hw.h.
 */
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include "notify_hw.h"
#include "ws2812.pio.h"

static PIO  g_pio = pio0;
static uint g_sm;
static bool g_led_ok;

bool neopixel_init(void)
{
    int sm;
    uint offset;

    if (!pio_can_add_program(g_pio, &ws2812_program))
    {
        return false;
    }
    sm = pio_claim_unused_sm(g_pio, false);
    if (sm < 0)
    {
        return false;
    }
    offset = pio_add_program(g_pio, &ws2812_program);
    g_sm = (uint)sm;
    ws2812_program_init(g_pio, g_sm, offset, NEOPIXEL_PIN, 800000.0f);
    g_led_ok = true;
    return true;
}

void neopixel_set_all(uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;
    uint32_t i;

    if (!g_led_ok)
    {
        return;
    }
    for (i = 0u; i < NEOPIXEL_COUNT; i++)
    {
        pio_sm_put_blocking(g_pio, g_sm, grb << 8u);
    }
}

void buzzer_init(void)
{
    uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);

    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    pwm_set_clkdiv(slice, (float)clock_get_hz(clk_sys) / 1000000.0f);   /* 1 MHz tick */
    pwm_set_wrap(slice, 499u);
    pwm_set_gpio_level(BUZZER_PIN, 0u);
    pwm_set_enabled(slice, true);
}

void buzzer_tone(uint32_t hz)
{
    uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);

    if ((hz < 20u) || (hz > 20000u))
    {
        pwm_set_gpio_level(BUZZER_PIN, 0u);
        return;
    }
    pwm_set_wrap(slice, (uint16_t)((1000000u / hz) - 1u));
    pwm_set_gpio_level(BUZZER_PIN, (uint16_t)(500000u / hz));
}
