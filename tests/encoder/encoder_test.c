/*
 * P5-IS34 bring-up test: wheel encoders (Buddy 2, with Buddy 3 for A11)
 *
 * What it measures:
 *   - Counts per wheel revolution (turn a wheel exactly one turn by hand)
 *   - Whether both A and B channels work (quadrature) and the count sign
 *   - Counts/s and edge rate per wheel at fixed PWM duty (30/50/70/100 %)
 *   - Time spent in the encoder ISR (worst case and % of CPU) at that rate
 *   - Invalid quadrature transitions (missed edges or noise)
 *   - Left/right mismatch at the same duty (how much PID has to correct)
 *   - Stall rule from the design (Motion 6.2: PWM above 30 % with no
 *     encoder change for 200 ms): the longest natural gap between edges at
 *     each duty (the margin below 200 ms), and the detection time when you
 *     grab a wheel during a run (A14). The run stops when a stall fires.
 *
 * Then, by hand: mm per count = pi * wheel diameter / counts per rev.
 * A11 needs this to be about 1 mm or finer to measure 3 mm barcode bars.
 *
 * Buttons (Robo Pico onboard, active low):
 *   GP20  run both motors forward for 3 s at the next duty step
 *         (30/40/50/70/100 %; 30 % is NOT above 30 %, so the stall rule
 *         is inactive there, exactly as the design rule is written)
 *   GP21  stop the motors now (holding it also blocks any start);
 *         when stopped, a press zeros the counts
 *
 * v3.2 (25 Oct, review findings 04 and 14): the stall check reads both
 * last-edge times and then the timer in one interrupts-off section, so an
 * edge between the reads can no longer look like a ~4,295 s stall; and the
 * ISR records the longest completed interval between count changes, which
 * the 1 ms sampled check could miss.
 *
 * Counts per revolution are 4x decoded (every A and B edge counts): do not
 * multiply by 4 again.
 *
 * Not NFR10 evidence: this is a polled stop with no independent safe-off.
 *
 * SAFETY: lift the wheels off the table before pressing GP20.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "../common/enc_timing.h"
#include "hardware/sync.h"

/* ---- Pins (design pin table) --------------------------------------- */
#define ENC_L_A_PIN         4u
#define ENC_L_B_PIN         5u
#define ENC_R_A_PIN         16u
#define ENC_R_B_PIN         17u
#define MOTOR_L_A_PIN       8u
#define MOTOR_L_B_PIN       9u
#define MOTOR_R_A_PIN       10u
#define MOTOR_R_B_PIN       11u
#define BTN_RUN_PIN         20u
#define BTN_STOP_PIN        21u

/* ---- Fill these in after measuring (0 = unknown) -------------------- */
#define WHEEL_DIAMETER_MM   0.0f
#define COUNTS_PER_REV      0

/* ---- Test settings ------------------------------------------------- */
#define PWM_WRAP            12499u   /* 125 MHz / 12500 = 10 kHz */
#define RUN_TIME_MS         3000u
#define STEADY_FROM_MS      1000u    /* ignore spin-up for the average */
#define SAMPLE_PERIOD_MS    100u
#define PRINT_PERIOD_MS     250u
#define BTN_DEBOUNCE_MS     50u

/* Stall rule as written in the Week 6 design (Motion Control, 6.2). */
#define STALL_MIN_DUTY      30u      /* rule applies only ABOVE this duty */
#define STALL_NO_EDGE_MS    200u

#define WHEEL_L             0u
#define WHEEL_R             1u
#define QDEC_INVALID        2

static const uint8_t g_duty_steps[] = {30u, 40u, 50u, 70u, 100u};
#define DUTY_STEPS          (sizeof(g_duty_steps) / sizeof(g_duty_steps[0]))

/*
 * Quadrature table indexed by (previous AB << 2) | new AB.
 * +1 / -1 = one count, 0 = no change, 2 = both bits changed (invalid).
 */
static const int8_t g_qdec[16] =
{
     0, +1, -1, QDEC_INVALID,
    -1,  0, QDEC_INVALID, +1,
    +1, QDEC_INVALID,  0, -1,
    QDEC_INVALID, -1, +1,  0
};

static volatile int32_t  g_count[2];
static volatile uint32_t g_edges_a[2];
static volatile uint32_t g_edges_b[2];
static volatile uint32_t g_invalid[2];
static volatile uint32_t g_isr_max_us;
static volatile uint32_t g_isr_total_us;
static volatile uint32_t g_last_edge_us[2];   /* time of the last valid count change */
static volatile uint32_t g_max_iv_us[2];      /* longest completed interval while armed */
static volatile bool     g_iv_armed;          /* true during the steady part of a run */
static uint8_t g_prev_ab[2];

static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static uint8_t read_ab(uint32_t wheel)
{
    uint32_t pin_a = (wheel == WHEEL_L) ? ENC_L_A_PIN : ENC_R_A_PIN;
    uint32_t pin_b = (wheel == WHEEL_L) ? ENC_L_B_PIN : ENC_R_B_PIN;
    return (uint8_t)((gpio_get(pin_a) ? 2u : 0u) | (gpio_get(pin_b) ? 1u : 0u));
}

/* One GPIO callback serves every pin (SDK limit), so keep it short. */
static void gpio_isr(uint gpio, uint32_t events)
{
    uint32_t t_start = timer_hw->timerawl;
    uint32_t wheel;
    uint8_t ab;
    int8_t step;
    uint32_t dt;

    if ((gpio == ENC_L_A_PIN) || (gpio == ENC_L_B_PIN))
    {
        wheel = WHEEL_L;
    }
    else if ((gpio == ENC_R_A_PIN) || (gpio == ENC_R_B_PIN))
    {
        wheel = WHEEL_R;
    }
    else
    {
        return;
    }

    if ((gpio == ENC_L_A_PIN) || (gpio == ENC_R_A_PIN))
    {
        g_edges_a[wheel]++;
    }
    else
    {
        g_edges_b[wheel]++;
    }

    ab = read_ab(wheel);
    step = g_qdec[(g_prev_ab[wheel] << 2) | ab];
    if (step == QDEC_INVALID)
    {
        g_invalid[wheel]++;
    }
    else if (step != 0)
    {
        g_count[wheel] += step;
        g_max_iv_us[wheel] = enc_interval_note(g_max_iv_us[wheel], g_last_edge_us[wheel], t_start,
                                               g_iv_armed);
        g_last_edge_us[wheel] = t_start;
    }
    g_prev_ab[wheel] = ab;

    dt = timer_hw->timerawl - t_start;
    g_isr_total_us += dt;
    if (dt > g_isr_max_us)
    {
        g_isr_max_us = dt;
    }
}

static void encoder_init(void)
{
    const uint32_t pins[4] = {ENC_L_A_PIN, ENC_L_B_PIN, ENC_R_A_PIN, ENC_R_B_PIN};
    const uint32_t edges = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    uint32_t i;

    for (i = 0u; i < 4u; i++)
    {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_pull_up(pins[i]); /* harmless for push-pull, needed for open-collector */
    }
    g_prev_ab[WHEEL_L] = read_ab(WHEEL_L);
    g_prev_ab[WHEEL_R] = read_ab(WHEEL_R);

    gpio_set_irq_enabled_with_callback(ENC_L_A_PIN, edges, true, gpio_isr);
    gpio_set_irq_enabled(ENC_L_B_PIN, edges, true);
    gpio_set_irq_enabled(ENC_R_A_PIN, edges, true);
    gpio_set_irq_enabled(ENC_R_B_PIN, edges, true);
}

static void motor_init(void)
{
    /* GP8/GP9 are PWM slice 4 (A/B), GP10/GP11 are slice 5 (A/B). */
    uint32_t slice_l = pwm_gpio_to_slice_num(MOTOR_L_A_PIN);
    uint32_t slice_r = pwm_gpio_to_slice_num(MOTOR_R_A_PIN);

    gpio_set_function(MOTOR_L_A_PIN, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_L_B_PIN, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_R_A_PIN, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_R_B_PIN, GPIO_FUNC_PWM);
    pwm_set_wrap(slice_l, PWM_WRAP);
    pwm_set_wrap(slice_r, PWM_WRAP);
    pwm_set_both_levels(slice_l, 0u, 0u);
    pwm_set_both_levels(slice_r, 0u, 0u);
    pwm_set_enabled(slice_l, true);
    pwm_set_enabled(slice_r, true);
}

/* Forward = A driven with PWM, B low. 0 % = both low (brake per design). */
static void motor_set_duty(uint32_t duty_percent)
{
    uint32_t level = (PWM_WRAP + 1u) * duty_percent / 100u;
    pwm_set_gpio_level(MOTOR_L_A_PIN, level);
    pwm_set_gpio_level(MOTOR_L_B_PIN, 0u);
    pwm_set_gpio_level(MOTOR_R_A_PIN, level);
    pwm_set_gpio_level(MOTOR_R_B_PIN, 0u);
}

static void snapshot(int32_t count[2], uint32_t edges[2])
{
    uint32_t irq = save_and_disable_interrupts();
    count[WHEEL_L] = g_count[WHEEL_L];
    count[WHEEL_R] = g_count[WHEEL_R];
    edges[WHEEL_L] = g_edges_a[WHEEL_L] + g_edges_b[WHEEL_L];
    edges[WHEEL_R] = g_edges_a[WHEEL_R] + g_edges_b[WHEEL_R];
    restore_interrupts(irq);
}

static void zero_counts(void)
{
    uint32_t irq = save_and_disable_interrupts();
    g_count[WHEEL_L] = 0;
    g_count[WHEEL_R] = 0;
    g_edges_a[WHEEL_L] = 0u;
    g_edges_a[WHEEL_R] = 0u;
    g_edges_b[WHEEL_L] = 0u;
    g_edges_b[WHEEL_R] = 0u;
    g_invalid[WHEEL_L] = 0u;
    g_invalid[WHEEL_R] = 0u;
    g_isr_max_us = 0u;
    g_isr_total_us = 0u;
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

static void print_scale(void)
{
#if (COUNTS_PER_REV > 0)
    if (WHEEL_DIAMETER_MM > 0.0f)
    {
        float mm_per_count = 3.14159f * WHEEL_DIAMETER_MM / (float)COUNTS_PER_REV;
        printf("[scale] %.3f mm per count (A11 wants <= ~1 mm for 3 mm bars) -> %s\n",
               (double)mm_per_count, (mm_per_count <= 1.0f) ? "OK" : "TOO COARSE");
    }
#endif
}

int main(void)
{
    bool running = false;
    uint32_t duty_idx = 0u;
    uint32_t duty = 0u;
    uint32_t t_run = 0u;
    uint32_t next_sample = 0u;
    uint32_t next_print = 0u;
    int32_t c_prev[2] = {0, 0};
    uint32_t e_prev[2] = {0u, 0u};
    int32_t c_steady[2] = {0, 0};
    uint32_t peak_eps[2] = {0u, 0u};
    int32_t last_printed[2] = {INT32_MIN, INT32_MIN};
    uint32_t isr_total_start = 0u;
    uint32_t t_last_sample = 0u;
    uint32_t t_steady = 0u;
    uint32_t late_samples = 0u;
    uint32_t max_gap_us[2] = {0u, 0u};
    bool stop_held = false;
    bool stop_pressed = false;
    bool run_pressed = false;
    bool run_down = false;
    bool stop_down = false;
    uint32_t run_last = 0u;
    uint32_t stop_last = 0u;

    stdio_init_all();
    button_init(BTN_RUN_PIN);
    button_init(BTN_STOP_PIN);
    motor_init();
    motor_set_duty(0u);
    encoder_init();
    sleep_ms(3000u);

    printf("\n=== P5-IS34 encoder bring-up test (v3) ===\n");
    printf("Hand test: turn one wheel exactly 1 rev (mark the tyre). GP21 zeros.\n");
    printf("Motor test: LIFT THE WHEELS, then press GP20 (3 s run, duty steps 30/40/50/70/100%%).\n");
    printf("Stall test: during a run above 30%%, grab one wheel and hold it; note the [stall] line.\n");
    print_scale();

    for (;;)
    {
        uint32_t t = now_ms();
        int32_t c[2];
        uint32_t e[2];

        snapshot(c, e);

        /*
         * STOP has priority over everything in this iteration: holding GP21
         * stops a run (level, not just the press edge) and blocks any start.
         * A new run needs GP21 released and a fresh GP20 press.
         */
        stop_held = !gpio_get(BTN_STOP_PIN);
        stop_pressed = button_pressed(BTN_STOP_PIN, &stop_down, &stop_last);
        run_pressed = button_pressed(BTN_RUN_PIN, &run_down, &run_last);

        if (running && stop_held)
        {
            motor_set_duty(0u);
            running = false;
            printf("[stop] motors stopped by GP21\n");
        }
        else if (!running && stop_pressed)
        {
            zero_counts();
            printf("[zero] counts cleared\n");
        }

        if (!running && run_pressed && !stop_held && !stop_pressed)
        {
            duty = g_duty_steps[duty_idx];
            duty_idx = (duty_idx + 1u) % DUTY_STEPS;
            zero_counts();
            snapshot(c, e);
            c_prev[WHEEL_L] = c[WHEEL_L];
            c_prev[WHEEL_R] = c[WHEEL_R];
            e_prev[WHEEL_L] = e[WHEEL_L];
            e_prev[WHEEL_R] = e[WHEEL_R];
            peak_eps[WHEEL_L] = 0u;
            peak_eps[WHEEL_R] = 0u;
            late_samples = 0u;
            max_gap_us[WHEEL_L] = 0u;
            max_gap_us[WHEEL_R] = 0u;
            {
                /* "No change" is counted from the run start, as the design
                 * rule would see it: a slow start-up is a real false trip. */
                uint32_t irq = save_and_disable_interrupts();
                g_last_edge_us[WHEEL_L] = timer_hw->timerawl;
                g_last_edge_us[WHEEL_R] = g_last_edge_us[WHEEL_L];
                g_max_iv_us[WHEEL_L] = 0u;
                g_max_iv_us[WHEEL_R] = 0u;
                g_iv_armed = false;
                restore_interrupts(irq);
            }
            running = true;
            t_run = t;
            t_last_sample = t;
            t_steady = t;
            next_sample = t + SAMPLE_PERIOD_MS;
            motor_set_duty(duty);
            printf("[run] %lu %% duty for %u ms\n", (unsigned long)duty, RUN_TIME_MS);
        }

        if (running)
        {
            /* ---- Stall rule, checked every loop pass (~1 ms) ----------
             * Read both last-edge times, THEN the timer, with interrupts off:
             * now can never be older than a last edge (review finding 04). */
            uint32_t last_us[2];
            uint32_t now_us;
            uint32_t w;
            {
                uint32_t irq = save_and_disable_interrupts();
                last_us[WHEEL_L] = g_last_edge_us[WHEEL_L];
                last_us[WHEEL_R] = g_last_edge_us[WHEEL_R];
                now_us = timer_hw->timerawl;
                restore_interrupts(irq);
            }
            if ((t - t_run) >= STEADY_FROM_MS)
            {
                g_iv_armed = true;   /* ISR now records completed intervals */
            }
            for (w = 0u; w < 2u; w++)
            {
                uint32_t gap_us = enc_edge_age_us(now_us, last_us[w]);
                if (((t - t_run) >= STEADY_FROM_MS) && (gap_us > max_gap_us[w]))
                {
                    max_gap_us[w] = gap_us;   /* longest open gap seen by this 1 ms check */
                }
                if (running && enc_stall_tripped(duty, STALL_MIN_DUTY, gap_us, STALL_NO_EDGE_MS))
                {
                    motor_set_duty(0u);
                    running = false;
                    printf("[stall] %s wheel: no count change for %lu ms at %lu %% duty, "
                           "%lu ms into the run%s -> motors stopped\n",
                           (w == WHEEL_L) ? "LEFT" : "RIGHT", (unsigned long)(gap_us / 1000u),
                           (unsigned long)duty, (unsigned long)(t - t_run),
                           ((t - t_run) < (STALL_NO_EDGE_MS + 50u))
                               ? " (START-UP: wheel never turned - false trip or deadband)" : "");
                }
            }
        }

        if (running)
        {
            if (running && ((int32_t)(t - next_sample) >= 0))
            {
                /* Rates use the real elapsed time, not the nominal period,
                 * so a slow USB print cannot inflate them. */
                uint32_t dt_ms = t - t_last_sample;
                uint32_t w;

                if (dt_ms == 0u)
                {
                    dt_ms = 1u;
                }
                if (dt_ms > (SAMPLE_PERIOD_MS + 5u))
                {
                    late_samples++;
                }
                for (w = 0u; w < 2u; w++)
                {
                    uint32_t eps = (e[w] - e_prev[w]) * 1000u / dt_ms;
                    if (eps > peak_eps[w])
                    {
                        peak_eps[w] = eps;
                    }
                    e_prev[w] = e[w];
                }
                if ((t - t_run) < (STEADY_FROM_MS + SAMPLE_PERIOD_MS))
                {
                    c_steady[WHEEL_L] = c[WHEEL_L];
                    c_steady[WHEEL_R] = c[WHEEL_R];
                    isr_total_start = g_isr_total_us;
                    t_steady = t;
                }
                printf("  t=%4lu ms  L=%6ld  R=%6ld  counts/s L=%6ld R=%6ld  (dt %lu ms)\n",
                       (unsigned long)(t - t_run), (long)c[WHEEL_L], (long)c[WHEEL_R],
                       (long)((c[WHEEL_L] - c_prev[WHEEL_L]) * 1000 / (int32_t)dt_ms),
                       (long)((c[WHEEL_R] - c_prev[WHEEL_R]) * 1000 / (int32_t)dt_ms),
                       (unsigned long)dt_ms);
                c_prev[WHEEL_L] = c[WHEEL_L];
                c_prev[WHEEL_R] = c[WHEEL_R];
                t_last_sample = t;
                next_sample = t + SAMPLE_PERIOD_MS;
            }

            if (running && ((t - t_run) >= RUN_TIME_MS))
            {
                uint32_t steady_ms = (t > t_steady) ? (t - t_steady) : 1u;
                float steady_s = (float)steady_ms / 1000.0f;
                float cps_l = (float)(c[WHEEL_L] - c_steady[WHEEL_L]) / steady_s;
                float cps_r = (float)(c[WHEEL_R] - c_steady[WHEEL_R]) / steady_s;
                float isr_pct = (float)(g_isr_total_us - isr_total_start) /
                                ((float)steady_ms * 10.0f);
                /* Compare magnitudes: the two encoders may count in opposite
                 * directions for "forward". */
                float mag_l = fabsf(cps_l);
                float mag_r = fabsf(cps_r);
                float mismatch = (mag_l + mag_r) != 0.0f
                                 ? 200.0f * (mag_l - mag_r) / (mag_l + mag_r) : 0.0f;

                motor_set_duty(0u);
                running = false;
                printf("[result] duty=%lu%%  steady counts/s L=%.0f R=%.0f  (L-R mismatch %.1f %%)\n",
                       (unsigned long)duty, (double)cps_l, (double)cps_r, (double)mismatch);
                printf("[result] peak edges/s L=%lu R=%lu  invalid L=%lu R=%lu\n",
                       (unsigned long)peak_eps[WHEEL_L], (unsigned long)peak_eps[WHEEL_R],
                       (unsigned long)g_invalid[WHEEL_L], (unsigned long)g_invalid[WHEEL_R]);
                printf("[result] steady window %lu ms, late samples %lu\n",
                       (unsigned long)steady_ms, (unsigned long)late_samples);
                {
                    /* Longest gap = the larger of the longest completed interval
                     * (ISR) and the longest open gap seen by the 1 ms check. */
                    uint32_t iv[2];
                    uint32_t worst[2];
                    uint32_t irq;
                    g_iv_armed = false;
                    irq = save_and_disable_interrupts();
                    iv[WHEEL_L] = g_max_iv_us[WHEEL_L];
                    iv[WHEEL_R] = g_max_iv_us[WHEEL_R];
                    restore_interrupts(irq);
                    worst[WHEEL_L] = (iv[WHEEL_L] > max_gap_us[WHEEL_L]) ? iv[WHEEL_L] : max_gap_us[WHEEL_L];
                    worst[WHEEL_R] = (iv[WHEEL_R] > max_gap_us[WHEEL_R]) ? iv[WHEEL_R] : max_gap_us[WHEEL_R];
                    printf("[result] longest gap between count changes (steady part): completed "
                           "interval L=%lu us R=%lu us, open gap at the 1 ms check L=%lu us R=%lu us\n",
                           (unsigned long)iv[WHEEL_L], (unsigned long)iv[WHEEL_R],
                           (unsigned long)max_gap_us[WHEEL_L], (unsigned long)max_gap_us[WHEEL_R]);
                    printf("[result] stall rule margin vs %u ms (worst of the two): L x%lu R x%lu%s\n",
                           STALL_NO_EDGE_MS,
                           (unsigned long)((worst[WHEEL_L] == 0u) ? 0u
                                           : (STALL_NO_EDGE_MS * 1000u / worst[WHEEL_L])),
                           (unsigned long)((worst[WHEEL_R] == 0u) ? 0u
                                           : (STALL_NO_EDGE_MS * 1000u / worst[WHEEL_R])),
                           (duty > STALL_MIN_DUTY) ? "" : "  (stall rule inactive at this duty)");
                }
                printf("[result] callback-body time: worst %lu us, load %.2f %% of CPU "
                       "(excludes IRQ entry and SDK dispatch)\n",
                       (unsigned long)g_isr_max_us, (double)isr_pct);
                if (((cps_l > 0.0f) && (cps_r < 0.0f)) || ((cps_l < 0.0f) && (cps_r > 0.0f)))
                {
                    printf("[result] note: L and R count in opposite directions for forward\n");
                }
#if (COUNTS_PER_REV > 0)
                if (WHEEL_DIAMETER_MM > 0.0f)
                {
                    float mm_per_count = 3.14159f * WHEEL_DIAMETER_MM / (float)COUNTS_PER_REV;
                    printf("[result] speed L=%.0f mm/s R=%.0f mm/s\n",
                           (double)(cps_l * mm_per_count), (double)(cps_r * mm_per_count));
                }
#endif
            }
        }
        else if ((int32_t)(t - next_print) >= 0)
        {
            /* Hand mode: print only when something moved. */
            if ((c[WHEEL_L] != last_printed[WHEEL_L]) || (c[WHEEL_R] != last_printed[WHEEL_R]))
            {
                printf("[hand] L=%6ld (A edges %5lu, B edges %5lu)   "
                       "R=%6ld (A edges %5lu, B edges %5lu)   invalid L=%lu R=%lu\n",
                       (long)c[WHEEL_L], (unsigned long)g_edges_a[WHEEL_L],
                       (unsigned long)g_edges_b[WHEEL_L],
                       (long)c[WHEEL_R], (unsigned long)g_edges_a[WHEEL_R],
                       (unsigned long)g_edges_b[WHEEL_R],
                       (unsigned long)g_invalid[WHEEL_L], (unsigned long)g_invalid[WHEEL_R]);
                last_printed[WHEEL_L] = c[WHEEL_L];
                last_printed[WHEEL_R] = c[WHEEL_R];
            }
            next_print = t + PRINT_PERIOD_MS;
        }

        sleep_ms(1u);
    }
}
