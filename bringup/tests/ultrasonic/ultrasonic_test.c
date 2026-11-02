/*
 * P5-IS34 bring-up test: HC-SR04 ultrasonic + scan servo (Buddy 5)
 *
 * v3.2 (25 Oct): the timed scans first move the servo to centre from where
 * it really is (review finding 12; this move is not part of the timed scan),
 * and the echo-width limits come from common/sonar_class.h, shared with
 * parts_check (finding 13).
 *
 * WIRING: TRIG -> GP6, ECHO -> GP7 with jumper wires, NOT Grove
 * (Grove 5 shares GP26 IMU SDA, Grove 7 shares GP28 battery sense).
 * Start with VCC = 3.3 V. If you power the module from 5 V, ECHO MUST go
 * through a divider (10k ECHO->GP7, 15k GP7->GND) or a level shifter.
 * A plain 5 V HC-SR04 driving GP7 directly can damage the Pico.
 *
 * What it measures (A5, NFR6):
 *   - Does this module work at 3.3 V at all (valid echoes, range)?
 *   - Accuracy and spread at known distances: put a flat target at
 *     50, 100, 150, 300 and 500 mm and press GP20 at each
 *   - Trigger-to-echo-rise latency and the no-target waveform
 *     (what ECHO does with nothing in front)
 *   - Raw outcomes kept separate: valid, too short, long pulse, no fall
 *     (timeout high), no rise (missing echo), stuck high before trigger.
 *     None of the non-valid outcomes is treated as "clear".
 *
 *   - NFR3 scan timing: the design's coarse scan (rays at 30, 60, 90,
 *     120, 150 deg) and fine scan (65..115 deg in 5 deg steps), two pings
 *     per ray in 60 ms slots, timed from scan start until the sensor is
 *     centred again (limits 1.2 s and 2.0 s). Servo settle time is a
 *     design assumption (SERVO_SETTLE_MS_PER_DEG): check the two pings per
 *     ray agree, and retry with a lower/higher value to find the real one.
 *
 * Not NFR6 evidence: there is no motor stop or braking here.
 *
 * Buttons (Robo Pico onboard, active low):
 *   GP20  capture the next 20 readings and print statistics
 *   GP21  short press: step the servo centre -> left 45 -> right 45
 *         hold >= 0.8 s and release: run a timed coarse scan, then fine scan
 *
 * Readings run every 60 ms, the design's Obstacle task period.
 * This test busy-waits for the echo; the real task will not.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "../common/sonar_class.h"

#define US_TRIG_PIN         6u
#define US_ECHO_PIN         7u
#define SERVO_PIN           14u
#define BTN_BATCH_PIN       20u
#define BTN_SERVO_PIN       21u

#define READ_PERIOD_MS      60u
#define TRIG_PULSE_US       10u
#define RISE_TIMEOUT_US     5000u
#define HIGH_TIMEOUT_US     40000u
#define MIN_VALID_US        SONAR_MIN_VALID_US    /* shared with parts_check */
#define LONG_PULSE_US       SONAR_LONG_PULSE_US
#define BATCH_SIZE          20u
#define BTN_DEBOUNCE_MS     50u

/* Speed of sound: 331.3 + 0.606 * T(degC). Singapore indoors ~ 28 C. */
#define AIR_TEMP_C          28.0f
#define SOUND_MM_PER_US     ((331.3f + 0.606f * AIR_TEMP_C) / 1000.0f)

/* Servo: 50 Hz, 1 us resolution (clock divided down to 1 MHz, wrap 20000). */
#define SERVO_WRAP          19999u
#define SERVO_CENTRE_US     1500u
#define SERVO_45_DEG_US     500u
#define SERVO_US_PER_DEG    ((float)SERVO_45_DEG_US / 45.0f)   /* nominal, not calibrated */

/* NFR3 scan settings (design 6.5 / NFR3 estimate). */
#define PING_SLOT_MS            60u     /* one ping per 60 ms slot */
#define PINGS_PER_RAY           2u
#define SERVO_SETTLE_MS_PER_DEG 2u      /* assumption: 30 deg -> 60 ms */
#define SERVO_SETTLE_MIN_MS     40u     /* assumption: 5 deg step -> 40 ms */
#define PING_AGREE_MM           30.0f   /* two pings on one ray must agree */
#define COARSE_LIMIT_MS         1200u
#define FINE_LIMIT_MS           2000u
#define LONG_PRESS_MS           800u
#define CENTRE_DEG              90

/*
 * Raw outcomes are kept separate on purpose. None of the non-VALID ones
 * proves clear space: what "no target" looks like depends on the module
 * and must be measured before any of these is treated as clearance.
 */
typedef enum
{
    US_VALID = 0,
    US_TOO_SHORT,      /* completed pulse < MIN_VALID_US (glitch/ambiguous edges) */
    US_LONG_PULSE,     /* completed pulse > LONG_PULSE_US */
    US_NO_FALL,        /* rose but never fell within HIGH_TIMEOUT_US */
    US_NO_RISE,        /* never rose after the trigger (missing echo) */
    US_STUCK_HIGH,     /* already high before the trigger */
    US_CLASS_COUNT
} us_class_t;

static const char *const g_class_name[US_CLASS_COUNT] =
{
    "VALID", "TOO_SHORT (glitch)", "LONG_PULSE (>25 ms)", "NO_FALL (timeout high)",
    "NO_RISE (missing echo)", "STUCK_HIGH (before trigger)"
};

static volatile uint32_t g_rise_us;
static volatile uint32_t g_fall_us;
static volatile bool g_rise_seen;
static volatile bool g_fall_seen;

static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static void echo_isr(uint gpio, uint32_t events)
{
    uint32_t t = timer_hw->timerawl;

    if (gpio != US_ECHO_PIN)
    {
        return;
    }
    if ((events & GPIO_IRQ_EDGE_RISE) != 0u)
    {
        g_rise_us = t;
        g_rise_seen = true;
    }
    if ((events & GPIO_IRQ_EDGE_FALL) != 0u)
    {
        g_fall_us = t;
        g_fall_seen = true;
    }
}

/* One ranging cycle. Returns the class, fills width and latency. */
static us_class_t us_measure(uint32_t *p_width_us, uint32_t *p_latency_us)
{
    uint32_t t_trig;

    *p_width_us = 0u;
    *p_latency_us = 0u;

    if (gpio_get(US_ECHO_PIN))
    {
        return US_STUCK_HIGH;
    }

    g_rise_seen = false;
    g_fall_seen = false;
    gpio_put(US_TRIG_PIN, 1);
    busy_wait_us_32(TRIG_PULSE_US);
    gpio_put(US_TRIG_PIN, 0);
    t_trig = timer_hw->timerawl;

    while (!g_rise_seen)
    {
        if ((timer_hw->timerawl - t_trig) > RISE_TIMEOUT_US)
        {
            return US_NO_RISE;
        }
    }
    *p_latency_us = g_rise_us - t_trig;

    while (!g_fall_seen)
    {
        if ((timer_hw->timerawl - g_rise_us) > HIGH_TIMEOUT_US)
        {
            *p_width_us = HIGH_TIMEOUT_US;
            return US_NO_FALL;
        }
    }
    *p_width_us = g_fall_us - g_rise_us;
    switch (sonar_classify_width(*p_width_us))
    {
        case SONAR_WIDTH_TOO_SHORT:
            return US_TOO_SHORT;   /* includes rise and fall reported in one event */
        case SONAR_WIDTH_LONG:
            return US_LONG_PULSE;
        default:
            return US_VALID;
    }
}

static void servo_init(void)
{
    uint32_t slice = pwm_gpio_to_slice_num(SERVO_PIN);
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    pwm_set_clkdiv(slice, (float)clock_get_hz(clk_sys) / 1000000.0f);
    pwm_set_wrap(slice, SERVO_WRAP);
    pwm_set_gpio_level(SERVO_PIN, SERVO_CENTRE_US);
    pwm_set_enabled(slice, true);
}

static int32_t g_servo_deg = CENTRE_DEG;   /* last commanded angle */

/* 90 deg = straight ahead. Sign of left/right is not calibrated. */
static void servo_set_angle(int32_t deg)
{
    float us = (float)SERVO_CENTRE_US + ((float)(deg - CENTRE_DEG) * SERVO_US_PER_DEG);
    pwm_set_gpio_level(SERVO_PIN, (uint16_t)(us + 0.5f));
    g_servo_deg = deg;
}

static uint32_t settle_ms(int32_t from_deg, int32_t to_deg)
{
    uint32_t delta = (uint32_t)((to_deg > from_deg) ? (to_deg - from_deg) : (from_deg - to_deg));
    uint32_t ms = delta * SERVO_SETTLE_MS_PER_DEG;
    return (ms < SERVO_SETTLE_MIN_MS) ? SERVO_SETTLE_MIN_MS : ms;
}

typedef struct
{
    int32_t    deg;
    us_class_t cls[PINGS_PER_RAY];
    float      mm[PINGS_PER_RAY];
} ray_t;

/*
 * One timed scan. Results are stored and printed afterwards so USB output
 * does not add to the measured time. Starts and ends at CENTRE_DEG.
 */
static void run_scan(const char *p_name, const int32_t *p_deg, uint32_t n, uint32_t limit_ms)
{
    static ray_t rays[11];
    uint64_t t0;
    uint32_t total_ms;
    int32_t cur;
    uint32_t unknown = 0u;
    uint32_t i;
    uint32_t k;

    /* Precondition, not timed: start from centre, settled from where the
     * servo really is (a manual step can leave it at 45 or 135 deg). */
    if (g_servo_deg != CENTRE_DEG)
    {
        int32_t from = g_servo_deg;
        servo_set_angle(CENTRE_DEG);
        sleep_ms(settle_ms(from, CENTRE_DEG));
        printf("[scan] servo was at %ld deg: centred first (%lu ms, not timed)\n", (long)from,
               (unsigned long)settle_ms(from, CENTRE_DEG));
    }
    cur = CENTRE_DEG;
    t0 = time_us_64();

    for (i = 0u; i < n; i++)
    {
        uint32_t s_ms = settle_ms(cur, p_deg[i]);
        servo_set_angle(p_deg[i]);
        sleep_ms(s_ms);
        cur = p_deg[i];
        rays[i].deg = p_deg[i];
        for (k = 0u; k < PINGS_PER_RAY; k++)
        {
            absolute_time_t slot_end = make_timeout_time_ms(PING_SLOT_MS);
            uint32_t w;
            uint32_t lat;
            rays[i].cls[k] = us_measure(&w, &lat);
            rays[i].mm[k] = (float)w * SOUND_MM_PER_US / 2.0f;
            sleep_until(slot_end);
        }
    }
    servo_set_angle(CENTRE_DEG);
    sleep_ms(settle_ms(cur, CENTRE_DEG));
    total_ms = (uint32_t)((time_us_64() - t0) / 1000u);

    printf("[scan] %s: %lu rays x %u pings, settle %u ms/deg (min %u ms)\n", p_name,
           (unsigned long)n, PINGS_PER_RAY, SERVO_SETTLE_MS_PER_DEG, SERVO_SETTLE_MIN_MS);
    for (i = 0u; i < n; i++)
    {
        bool agree = (rays[i].cls[0] == US_VALID) && (rays[i].cls[1] == US_VALID) &&
                     (fabsf(rays[i].mm[0] - rays[i].mm[1]) <= PING_AGREE_MM);
        if (!agree)
        {
            unknown++;
        }
        printf("[scan]   %3ld deg: %-22s %7.1f mm | %-22s %7.1f mm  %s\n",
               (long)rays[i].deg,
               g_class_name[rays[i].cls[0]], (double)((rays[i].cls[0] == US_VALID) ? rays[i].mm[0] : 0.0f),
               g_class_name[rays[i].cls[1]], (double)((rays[i].cls[1] == US_VALID) ? rays[i].mm[1] : 0.0f),
               agree ? "agree" : "UNCONFIRMED (counts as unknown, not clear)");
    }
    printf("[scan] %s TIME %lu ms (start -> centred again), limit %lu ms -> %s | "
           "unconfirmed rays %lu/%lu\n",
           p_name, (unsigned long)total_ms, (unsigned long)limit_ms,
           (total_ms <= limit_ms) ? "WITHIN" : "OVER", (unsigned long)unknown, (unsigned long)n);
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

int main(void)
{
    static const int32_t servo_offsets_deg[] = {0, -45, 45};   /* same pulses as before: 1500/1000/2000 us */
    const char *const servo_names[] = {"centre", "left 45", "right 45"};
    uint32_t servo_idx = 0u;
    uint32_t batch_left = 0u;
    uint32_t class_count[US_CLASS_COUNT];
    float sum = 0.0f;
    float sum2 = 0.0f;
    float d_min = 0.0f;
    float d_max = 0.0f;
    uint32_t n_valid = 0u;
    uint32_t lat_sum = 0u;
    uint32_t reading = 0u;
    uint32_t overruns = 0u;
    uint32_t batch_overruns = 0u;
    uint32_t c;
    static const int32_t coarse_deg[] = {30, 60, 90, 120, 150};
    static const int32_t fine_deg[] = {65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115};
    bool batch_down = false;
    bool batch_done = false;
    bool servo_down = false;
    uint32_t batch_last = 0u;
    uint32_t servo_down_since = 0u;
    absolute_time_t next_read;

    stdio_init_all();
    button_init(BTN_BATCH_PIN);
    button_init(BTN_SERVO_PIN);

    gpio_init(US_TRIG_PIN);
    gpio_set_dir(US_TRIG_PIN, GPIO_OUT);
    gpio_put(US_TRIG_PIN, 0);
    gpio_init(US_ECHO_PIN);
    gpio_set_dir(US_ECHO_PIN, GPIO_IN);
    gpio_pull_down(US_ECHO_PIN); /* disconnected ECHO reads as never rising */
    gpio_set_irq_enabled_with_callback(US_ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true, echo_isr);
    servo_init();

    sleep_ms(3000u);
    printf("\n=== P5-IS34 ultrasonic bring-up test (v3) ===\n");
    printf("Sound speed %.4f mm/us at %.0f C. GP20 = 20-reading batch, GP21 = servo step, "
           "hold GP21 >= 0.8 s = timed coarse + fine scan.\n",
           (double)SOUND_MM_PER_US, (double)AIR_TEMP_C);

    for (c = 0u; c < US_CLASS_COUNT; c++)
    {
        class_count[c] = 0u;
    }
    next_read = get_absolute_time();

    for (;;)
    {
        uint32_t width_us;
        uint32_t latency_us;
        us_class_t cls;
        float d_mm;

        cls = us_measure(&width_us, &latency_us);
        d_mm = (float)width_us * SOUND_MM_PER_US / 2.0f;
        reading++;

        if ((batch_left > 0u) || ((reading % 5u) == 0u))
        {
            printf("[%5lu] %-28s width=%6lu us  dist=%7.1f mm  trig->rise=%4lu us\n",
                   (unsigned long)reading, g_class_name[cls], (unsigned long)width_us,
                   (double)((cls == US_VALID) ? d_mm : 0.0f), (unsigned long)latency_us);
        }

        if (batch_left > 0u)
        {
            class_count[cls]++;
            if (cls == US_VALID)
            {
                if ((n_valid == 0u) || (d_mm < d_min))
                {
                    d_min = d_mm;
                }
                if ((n_valid == 0u) || (d_mm > d_max))
                {
                    d_max = d_mm;
                }
                sum += d_mm;
                sum2 += d_mm * d_mm;
                lat_sum += latency_us;
                n_valid++;
            }
            batch_left--;
            batch_done = (batch_left == 0u);
        }

        if (button_pressed(BTN_BATCH_PIN, &batch_down, &batch_last) && (batch_left == 0u))
        {
            for (c = 0u; c < US_CLASS_COUNT; c++)
            {
                class_count[c] = 0u;
            }
            sum = 0.0f;
            sum2 = 0.0f;
            n_valid = 0u;
            lat_sum = 0u;
            batch_overruns = 0u;
            batch_left = BATCH_SIZE;
            printf("[batch] capturing %u readings - write down the true distance now\n",
                   BATCH_SIZE);
        }

        /* GP21: short press steps the servo, long press runs the scans. */
        {
            bool down = !gpio_get(BTN_SERVO_PIN);
            uint32_t t_now = now_ms();
            if (down && !servo_down)
            {
                servo_down_since = t_now;
            }
            else if (!down && servo_down && ((t_now - servo_down_since) > BTN_DEBOUNCE_MS))
            {
                if (batch_left > 0u)
                {
                    printf("[servo] ignored: angle is held during a batch\n");
                }
                else if ((t_now - servo_down_since) >= LONG_PRESS_MS)
                {
                    printf("[scan] starting coarse + fine scans (robot must be stationary)\n");
                    run_scan("COARSE", coarse_deg, 5u, COARSE_LIMIT_MS);
                    sleep_ms(500u);
                    run_scan("FINE", fine_deg, 11u, FINE_LIMIT_MS);
                    servo_idx = 0u;
                    next_read = get_absolute_time();   /* scan time is not an overrun */
                }
                else
                {
                    servo_idx = (servo_idx + 1u) % 3u;
                    servo_set_angle(CENTRE_DEG + servo_offsets_deg[servo_idx]);
                    printf("[servo] %s (nominal pulse; physical angle not calibrated)\n",
                           servo_names[servo_idx]);
                }
            }
            servo_down = down;
        }

        next_read = delayed_by_ms(next_read, READ_PERIOD_MS);
        if (absolute_time_diff_us(get_absolute_time(), next_read) < 0)
        {
            overruns++;
            if ((batch_left > 0u) || batch_done)
            {
                batch_overruns++;   /* includes the batch's last reading */
            }
            next_read = get_absolute_time();   /* no catch-up burst */
        }

        if (batch_done)
        {
            batch_done = false;
            {
                printf("[batch] valid %lu/%u | too_short %lu long_pulse %lu no_fall %lu "
                       "no_rise %lu stuck_high %lu | overruns %lu (servo held at %s)\n",
                       (unsigned long)n_valid, BATCH_SIZE,
                       (unsigned long)class_count[US_TOO_SHORT],
                       (unsigned long)class_count[US_LONG_PULSE],
                       (unsigned long)class_count[US_NO_FALL],
                       (unsigned long)class_count[US_NO_RISE],
                       (unsigned long)class_count[US_STUCK_HIGH],
                       (unsigned long)batch_overruns, servo_names[servo_idx]);
                if (n_valid > 0u)
                {
                    float mean = sum / (float)n_valid;
                    float var = (sum2 / (float)n_valid) - (mean * mean);
                    printf("[batch] mean %.1f mm  sd %.1f mm  min %.1f  max %.1f  "
                           "avg trig->rise %lu us\n",
                           (double)mean, (double)sqrtf((var > 0.0f) ? var : 0.0f),
                           (double)d_min, (double)d_max,
                           (unsigned long)(lat_sum / n_valid));
                }
            }
        }
        sleep_until(next_read);
    }
}
