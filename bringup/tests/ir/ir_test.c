/*
 * P5-IS34 bring-up test: IR line/barcode sensors (Buddy 3)
 *
 * The three sensors are digital (D0 from an LM393 comparator), so a
 * "level" is 0 or 1 and the threshold is the module's potentiometer.
 *
 * What it measures:
 *   - Which level means black and which means white (polarity)
 *   - Whether each sensor separates the 18 mm line from the floor at the
 *     mounted height, under the room lighting (A3)
 *   - Every edge with a microsecond timestamp and how long the previous
 *     level lasted: drag the robot slowly over a printed Code 39 barcode
 *     to see bar widths in time, and the narrow/wide ratio (A7, A11)
 *   - Chatter: pulses shorter than 500 us, which happen when a comparator
 *     sits near its threshold at a black/white boundary
 *
 * Buttons (Robo Pico onboard, active low):
 *   GP20  reset the statistics
 *   GP21  toggle printing of IR_LEFT/IR_CENTER edges (barcode edges
 *         always print)
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

#define IR_LEFT_PIN         0u
#define IR_CENTER_PIN       1u
#define IR_BARCODE_PIN      2u
#define BTN_RESET_PIN       20u
#define BTN_TOGGLE_PIN      21u

#define SENSOR_COUNT        3u
#define EDGE_RING_SIZE      256u     /* power of two */
#define GLITCH_US           500u
#define STATUS_PERIOD_MS    500u
#define BTN_DEBOUNCE_MS     50u
#define MAX_DRAIN_PER_LOOP  16u      /* bound printing work per loop pass */

typedef struct
{
    uint32_t t_us;
    uint8_t  sensor;
    uint8_t  level;
    uint8_t  ambiguous;   /* rise and fall both reported in one event */
    uint8_t  after_gap;   /* one or more edges were dropped just before this one */
} ir_edge_t;

static const uint32_t g_pins[SENSOR_COUNT] = {IR_LEFT_PIN, IR_CENTER_PIN, IR_BARCODE_PIN};
static const char *const g_names[SENSOR_COUNT] = {"LEFT   ", "CENTER ", "BARCODE"};

static volatile ir_edge_t g_ring[EDGE_RING_SIZE];
static volatile uint32_t g_head;
static volatile uint32_t g_overflow;
/* Single core: the ISR only writes g_head, main only writes g_tail. */
static volatile uint32_t g_tail;
/* ISR-only: set when an edge is dropped, carried by the next stored edge,
 * so the consumer sees the loss at its true position in the stream. */
static bool g_gap_pending;

static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static void gpio_isr(uint gpio, uint32_t events)
{
    uint32_t t = timer_hw->timerawl;
    uint32_t s;

    for (s = 0u; s < SENSOR_COUNT; s++)
    {
        if (gpio == g_pins[s])
        {
            break;
        }
    }
    if (s == SENSOR_COUNT)
    {
        return;
    }

    if ((g_head - g_tail) >= EDGE_RING_SIZE)
    {
        g_overflow++;
        g_gap_pending = true;
        return;
    }
    g_ring[g_head & (EDGE_RING_SIZE - 1u)].after_gap = g_gap_pending ? 1u : 0u;
    g_gap_pending = false;
    g_ring[g_head & (EDGE_RING_SIZE - 1u)].t_us = t;
    g_ring[g_head & (EDGE_RING_SIZE - 1u)].sensor = (uint8_t)s;
    g_ring[g_head & (EDGE_RING_SIZE - 1u)].level =
        ((events & GPIO_IRQ_EDGE_RISE) != 0u) ? 1u : 0u;
    g_ring[g_head & (EDGE_RING_SIZE - 1u)].ambiguous =
        (((events & GPIO_IRQ_EDGE_RISE) != 0u) && ((events & GPIO_IRQ_EDGE_FALL) != 0u)) ? 1u : 0u;
    __dmb();   /* entry written before it is published */
    g_head++;
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
    const uint32_t edges = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    uint32_t last_edge_us[SENSOR_COUNT] = {0u, 0u, 0u};
    uint32_t edge_count[SENSOR_COUNT] = {0u, 0u, 0u};
    uint32_t glitches[SENSOR_COUNT] = {0u, 0u, 0u};
    uint32_t min_pulse_us[SENSOR_COUNT] = {UINT32_MAX, UINT32_MAX, UINT32_MAX};
    uint32_t high_ms[SENSOR_COUNT] = {0u, 0u, 0u};
    uint32_t samples = 0u;
    uint32_t ambiguous = 0u;
    uint32_t gaps_seen = 0u;
    uint32_t next_status = 0u;
    uint32_t next_sample = 0u;
    bool print_line_edges = true;
    bool reset_down = false;
    bool toggle_down = false;
    uint32_t reset_last = 0u;
    uint32_t toggle_last = 0u;
    uint32_t s;

    stdio_init_all();
    button_init(BTN_RESET_PIN);
    button_init(BTN_TOGGLE_PIN);

    for (s = 0u; s < SENSOR_COUNT; s++)
    {
        gpio_init(g_pins[s]);
        gpio_set_dir(g_pins[s], GPIO_IN);
        gpio_disable_pulls(g_pins[s]); /* module drives the pin */
    }
    gpio_set_irq_enabled_with_callback(IR_LEFT_PIN, edges, true, gpio_isr);
    gpio_set_irq_enabled(IR_CENTER_PIN, edges, true);
    gpio_set_irq_enabled(IR_BARCODE_PIN, edges, true);

    sleep_ms(3000u);
    printf("\n=== P5-IS34 IR sensor bring-up test (v3) ===\n");
    printf("Hold each sensor over white floor, then over the black line; note the level\n");
    printf("and the module LED. Then drag slowly across a printed Code 39 barcode.\n");

    for (;;)
    {
        uint32_t t = now_ms();


        /* Print queued edges with the duration of the previous level. */
        for (uint32_t drained = 0u; (g_tail != g_head) && (drained < MAX_DRAIN_PER_LOOP); drained++)
        {
            ir_edge_t ev;
            uint32_t prev_len;

            ev.t_us = g_ring[g_tail & (EDGE_RING_SIZE - 1u)].t_us;
            ev.sensor = g_ring[g_tail & (EDGE_RING_SIZE - 1u)].sensor;
            ev.level = g_ring[g_tail & (EDGE_RING_SIZE - 1u)].level;
            ev.ambiguous = g_ring[g_tail & (EDGE_RING_SIZE - 1u)].ambiguous;
            ev.after_gap = g_ring[g_tail & (EDGE_RING_SIZE - 1u)].after_gap;
            __dmb();   /* entry read before the slot is released */
            g_tail++;

            /*
             * Edges were dropped just before this one. Edges queued before
             * the loss stay valid; widths across the gap do not, for any
             * sensor (we do not know whose edges were lost).
             */
            if (ev.after_gap != 0u)
            {
                for (s = 0u; s < SENSOR_COUNT; s++)
                {
                    last_edge_us[s] = 0u;
                }
                gaps_seen++;
                printf("[edge] %10lu us  EDGES LOST before this point (ring overflow) - "
                       "pulse history restarted, discard this capture\n",
                       (unsigned long)ev.t_us);
            }

            if (ev.ambiguous != 0u)
            {
                /* Edge order unknown: this width is not trustworthy. */
                ambiguous++;
                printf("[edge] %10lu us  %s AMBIGUOUS (rise+fall in one event) - width invalid\n",
                       (unsigned long)ev.t_us, g_names[ev.sensor]);
                last_edge_us[ev.sensor] = 0u;
                continue;
            }

            prev_len = (last_edge_us[ev.sensor] == 0u) ? 0u : (ev.t_us - last_edge_us[ev.sensor]);
            last_edge_us[ev.sensor] = ev.t_us;
            edge_count[ev.sensor]++;
            if (prev_len != 0u)
            {
                if (prev_len < min_pulse_us[ev.sensor])
                {
                    min_pulse_us[ev.sensor] = prev_len;
                }
                if (prev_len < GLITCH_US)
                {
                    glitches[ev.sensor]++;
                }
            }

            if ((ev.sensor == 2u) || print_line_edges)
            {
                printf("[edge] %10lu us  %s %u->%u  previous level lasted %8lu us%s\n",
                       (unsigned long)ev.t_us, g_names[ev.sensor],
                       (unsigned)(1u - ev.level), (unsigned)ev.level,
                       (unsigned long)prev_len,
                       ((prev_len != 0u) && (prev_len < GLITCH_US)) ? "  <- CHATTER" : "");
            }
        }

        /* Sample levels every 1 ms to get the fraction of time high. */
        if ((int32_t)(t - next_sample) >= 0)
        {
            for (s = 0u; s < SENSOR_COUNT; s++)
            {
                high_ms[s] += gpio_get(g_pins[s]) ? 1u : 0u;
            }
            samples++;
            next_sample = t + 1u;
        }

        if ((int32_t)(t - next_status) >= 0)
        {
            printf("[level] L=%u C=%u B=%u | edges %lu/%lu/%lu | chatter %lu/%lu/%lu | "
                   "min pulse us %lu/%lu/%lu | %%high(samples) %lu/%lu/%lu | ambiguous %lu | "
                   "edges dropped %lu (gaps seen %lu)\n",
                   (unsigned)gpio_get(IR_LEFT_PIN), (unsigned)gpio_get(IR_CENTER_PIN),
                   (unsigned)gpio_get(IR_BARCODE_PIN),
                   (unsigned long)edge_count[0], (unsigned long)edge_count[1],
                   (unsigned long)edge_count[2],
                   (unsigned long)glitches[0], (unsigned long)glitches[1],
                   (unsigned long)glitches[2],
                   (unsigned long)((min_pulse_us[0] == UINT32_MAX) ? 0u : min_pulse_us[0]),
                   (unsigned long)((min_pulse_us[1] == UINT32_MAX) ? 0u : min_pulse_us[1]),
                   (unsigned long)((min_pulse_us[2] == UINT32_MAX) ? 0u : min_pulse_us[2]),
                   (unsigned long)((samples == 0u) ? 0u : (100u * high_ms[0] / samples)),
                   (unsigned long)((samples == 0u) ? 0u : (100u * high_ms[1] / samples)),
                   (unsigned long)((samples == 0u) ? 0u : (100u * high_ms[2] / samples)),
                   (unsigned long)ambiguous, (unsigned long)g_overflow,
                   (unsigned long)gaps_seen);
            next_status = t + STATUS_PERIOD_MS;
        }

        if (button_pressed(BTN_RESET_PIN, &reset_down, &reset_last))
        {
            for (s = 0u; s < SENSOR_COUNT; s++)
            {
                edge_count[s] = 0u;
                glitches[s] = 0u;
                min_pulse_us[s] = UINT32_MAX;
                high_ms[s] = 0u;
                last_edge_us[s] = 0u;
            }
            samples = 0u;
            ambiguous = 0u;
            {
                uint32_t irq = save_and_disable_interrupts();
                g_overflow = 0u;
                gaps_seen = 0u;
                restore_interrupts(irq);
            }
            printf("[reset] statistics cleared\n");
        }
        if (button_pressed(BTN_TOGGLE_PIN, &toggle_down, &toggle_last))
        {
            print_line_edges = !print_line_edges;
            printf("[toggle] LEFT/CENTER edge printing %s\n", print_line_edges ? "ON" : "OFF");
        }

        tight_loop_contents();
    }
}
