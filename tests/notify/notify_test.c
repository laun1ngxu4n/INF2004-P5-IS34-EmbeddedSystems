/*
 * P5-IS34 bring-up test: status LED + buzzer (Buddy 1, A15)
 *
 * A15: "driving the NeoPixel via PIO and the buzzer via PWM adds no
 * measurable jitter to the 10 ms Motion Control loop. Pass: jitter <= 1 ms
 * with and without LED/buzzer activity."
 *
 * How it models that without the RTOS:
 *   - The "Motion loop" is a 10 ms hardware-timer interrupt (higher
 *     priority than anything in main, as Motion P3 is above Notification).
 *     Each activation does ~300 us of busy work, like a PID update, and
 *     records how late it started and how far its period was from 10 ms.
 *   - main() plays the Notification task. Three 10 s phases repeat:
 *       QUIET   no LED/buzzer writes (baseline)
 *       DESIGN  LED colour change every 100 ms (the design's blink tick)
 *               and a buzzer tone change every 50 ms
 *       STRESS  LED + buzzer writes back to back as fast as possible
 *     So anything in the LED/buzzer path that blocks interrupts or the
 *     timer shows up as Motion jitter in DESIGN/STRESS but not QUIET.
 *   - It also times each LED update and buzzer write (CPU cost, which
 *     matters for the v3 proposal to merge Notification into Telemetry).
 *
 * Not RTOS evidence: micro T-Kernel dispatch and its critical sections are
 * not present here. Repeat with the real tasks once integrated.
 *
 * Buttons (Robo Pico onboard, active low):
 *   GP20  mute/unmute the buzzer (the PWM writes still happen when muted)
 *   GP21  eye/ear check: red, green, blue, white, off, then a "fault" beep
 *         pattern and a "mission complete" pattern. Phases restart after.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/sync.h"

#include "notify_hw.h"

#define BTN_MUTE_PIN        20u
#define BTN_CHECK_PIN       21u
#define BTN_DEBOUNCE_MS     50u

#define MOTION_PERIOD_US    10000
#define MOTION_WORK_US      300u
#define PHASE_MS            10000u
#define LED_TICK_MS         100u
#define BUZZ_TICK_MS        50u
#define JITTER_LIMIT_US     1000u

typedef enum
{
    PH_QUIET = 0,
    PH_DESIGN,
    PH_STRESS,
    PH_COUNT
} phase_t;

static const char *const g_phase_name[PH_COUNT] = {"QUIET ", "DESIGN", "STRESS"};

/* Written by the timer IRQ, read and reset by main with IRQs off. */
static volatile uint32_t g_runs;
static volatile uint32_t g_late_max_us;
static volatile uint32_t g_period_dev_max_us;
static volatile uint64_t g_expected_us;
static volatile uint64_t g_prev_start_us;

static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static bool motion_tick(repeating_timer_t *p_rt)
{
    uint64_t start = time_us_64();
    uint32_t late;

    if (g_expected_us == 0u)
    {
        g_expected_us = start;
    }
    late = (start > g_expected_us) ? (uint32_t)(start - g_expected_us) : 0u;
    if (late > g_late_max_us)
    {
        g_late_max_us = late;
    }
    if (g_prev_start_us != 0u)
    {
        int64_t period = (int64_t)(start - g_prev_start_us);
        uint32_t dev = (uint32_t)((period > MOTION_PERIOD_US) ? (period - MOTION_PERIOD_US)
                                                             : (MOTION_PERIOD_US - period));
        if (dev > g_period_dev_max_us)
        {
            g_period_dev_max_us = dev;
        }
    }
    g_prev_start_us = start;
    g_expected_us += (uint64_t)MOTION_PERIOD_US;
    g_runs++;

    busy_wait_us_32(MOTION_WORK_US);   /* stand-in for the PID update */
    return true;
}

static void motion_stats_reset(void)
{
    uint32_t irq = save_and_disable_interrupts();
    g_runs = 0u;
    g_late_max_us = 0u;
    g_period_dev_max_us = 0u;
    restore_interrupts(irq);
}

static bool button_pressed(uint32_t pin, bool *p_was_down, uint32_t *p_last_ms)
{
    bool down = !gpio_get(pin);
    bool pressed = false;
    uint32_t t = now_ms();

    if (down && !*p_was_down && ((t - *p_last_ms) > BTN_DEBOUNCE_MS))
    {
        pressed = true;
        *p_last_ms = t;
    }
    *p_was_down = down;
    return pressed;
}

static void button_init(uint32_t pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

static void beep(uint32_t hz, uint32_t on_ms, uint32_t off_ms)
{
    buzzer_tone(hz);
    sleep_ms(on_ms);
    buzzer_tone(0u);
    sleep_ms(off_ms);
}

static void eye_ear_check(void)
{
    static const uint8_t colours[5][3] =
    {
        {64u, 0u, 0u}, {0u, 64u, 0u}, {0u, 0u, 64u}, {48u, 48u, 48u}, {0u, 0u, 0u}
    };
    static const char *const names[5] = {"RED", "GREEN", "BLUE", "WHITE", "OFF"};
    uint32_t i;

    printf("[check] LEDs (both should show each colour; GRB order is handled in code):\n");
    for (i = 0u; i < 5u; i++)
    {
        printf("[check]   %s\n", names[i]);
        neopixel_set_all(colours[i][0], colours[i][1], colours[i][2]);
        sleep_ms(1000u);
    }
    printf("[check] buzzer 'fault' pattern: 3 short 2 kHz beeps (silent? check the mute switch)\n");
    for (i = 0u; i < 3u; i++)
    {
        beep(2000u, 120u, 120u);
    }
    sleep_ms(400u);
    printf("[check] buzzer 'mission complete' pattern: rising 3 tones\n");
    beep(1000u, 150u, 50u);
    beep(1500u, 150u, 50u);
    beep(2000u, 300u, 0u);
    printf("[check] done - write down what you saw and heard\n");
}

int main(void)
{
    static const uint8_t cycle[4][3] =
    {
        {32u, 0u, 0u}, {0u, 32u, 0u}, {0u, 0u, 32u}, {0u, 0u, 0u}
    };
    repeating_timer_t timer;
    phase_t phase = PH_QUIET;
    uint32_t t_phase;
    uint32_t next_led = 0u;
    uint32_t next_buzz = 0u;
    uint32_t led_idx = 0u;
    uint32_t tone_idx = 0u;
    uint32_t led_max_us = 0u;
    uint32_t buzz_max_us = 0u;
    uint32_t led_writes = 0u;
    uint32_t buzz_writes = 0u;
    bool muted = false;
    bool mute_down = false;
    bool check_down = false;
    uint32_t mute_last = 0u;
    uint32_t check_last = 0u;
    bool led_ok;

    stdio_init_all();
    button_init(BTN_MUTE_PIN);
    button_init(BTN_CHECK_PIN);
    led_ok = neopixel_init();
    buzzer_init();
    sleep_ms(3000u);

    printf("\n=== P5-IS34 status LED + buzzer test (A15) ===\n");
    printf("[init] NeoPixel PIO on GP%u: %s, buzzer PWM on GP%u\n", NEOPIXEL_PIN,
           led_ok ? "OK" : "FAILED (no PIO space)", BUZZER_PIN);
    printf("10 ms 'Motion' timer IRQ with %u us work. Phases QUIET/DESIGN/STRESS, %u s each.\n",
           MOTION_WORK_US, PHASE_MS / 1000u);
    printf("GP20 = mute buzzer, GP21 = eye/ear check.\n");

    if (!add_repeating_timer_us(-MOTION_PERIOD_US, motion_tick, NULL, &timer))
    {
        printf("[init] could not start the 10 ms timer\n");
        return 1;
    }
    motion_stats_reset();
    t_phase = now_ms();

    for (;;)
    {
        uint32_t t = now_ms();

        if (phase == PH_DESIGN)
        {
            if ((int32_t)(t - next_led) >= 0)
            {
                uint64_t a = time_us_64();
                neopixel_set_all(cycle[led_idx][0], cycle[led_idx][1], cycle[led_idx][2]);
                uint32_t d = (uint32_t)(time_us_64() - a);
                led_max_us = (d > led_max_us) ? d : led_max_us;
                led_writes++;
                led_idx = (led_idx + 1u) % 4u;
                next_led = t + LED_TICK_MS;
            }
            if ((int32_t)(t - next_buzz) >= 0)
            {
                uint64_t a = time_us_64();
                buzzer_tone(muted ? 0u : ((tone_idx & 1u) ? 2000u : 1000u));
                uint32_t d = (uint32_t)(time_us_64() - a);
                buzz_max_us = (d > buzz_max_us) ? d : buzz_max_us;
                buzz_writes++;
                tone_idx++;
                next_buzz = t + BUZZ_TICK_MS;
            }
        }
        else if (phase == PH_STRESS)
        {
            uint64_t a = time_us_64();
            uint32_t d;
            neopixel_set_all(cycle[led_idx][0], cycle[led_idx][1], cycle[led_idx][2]);
            d = (uint32_t)(time_us_64() - a);
            led_max_us = (d > led_max_us) ? d : led_max_us;
            led_writes++;
            led_idx = (led_idx + 1u) % 4u;
            a = time_us_64();
            buzzer_tone(muted ? 0u : ((tone_idx & 1u) ? 2000u : 1000u));
            d = (uint32_t)(time_us_64() - a);
            buzz_max_us = (d > buzz_max_us) ? d : buzz_max_us;
            buzz_writes++;
            tone_idx++;
            busy_wait_us_32(100u);   /* >50 us WS2812 latch gap between frames */
        }

        if ((t - t_phase) >= PHASE_MS)
        {
            uint32_t runs;
            uint32_t late;
            uint32_t dev;
            uint32_t irq = save_and_disable_interrupts();
            runs = g_runs;
            late = g_late_max_us;
            dev = g_period_dev_max_us;
            restore_interrupts(irq);

            printf("[%s] motion runs %lu (expect ~%lu) | start late max %lu us | period dev max %lu us"
                   " -> %s | LED writes %lu (max %lu us) buzzer writes %lu (max %lu us)%s\n",
                   g_phase_name[phase], (unsigned long)runs,
                   (unsigned long)(PHASE_MS * 1000u / (uint32_t)MOTION_PERIOD_US),
                   (unsigned long)late, (unsigned long)dev,
                   (dev <= JITTER_LIMIT_US) ? "within 1 ms" : "OVER 1 ms",
                   (unsigned long)led_writes, (unsigned long)led_max_us,
                   (unsigned long)buzz_writes, (unsigned long)buzz_max_us,
                   muted ? " (muted)" : "");

            buzzer_tone(0u);
            neopixel_set_all(0u, 0u, 0u);
            phase = (phase_t)((phase + 1) % PH_COUNT);
            led_max_us = 0u;
            buzz_max_us = 0u;
            led_writes = 0u;
            buzz_writes = 0u;
            motion_stats_reset();
            t_phase = now_ms();
        }

        if (button_pressed(BTN_MUTE_PIN, &mute_down, &mute_last))
        {
            muted = !muted;
            buzzer_tone(0u);
            printf("[mute] buzzer %s\n", muted ? "muted" : "on");
        }
        if (button_pressed(BTN_CHECK_PIN, &check_down, &check_last))
        {
            eye_ear_check();
            phase = PH_QUIET;
            motion_stats_reset();
            t_phase = now_ms();
        }

        if (phase != PH_STRESS)
        {
            sleep_ms(1u);
        }
    }
}
