/* Host test: IR ring overflow marker, using the real ir_test.c source.
 * Reproduces the v2 verification case: 256 edges at 100 us, the edge at
 * 25,700 us dropped (ring full), 16 drained, edge at 25,800 us queued,
 * rest drained. v2 printed a 200 us width across the gap. */
#include <setjmp.h>
#include <string.h>
#include "sdk_stub.h"
stub_timer_t stub_timer;
uint64_t stub_now_us = 0u;
bool stub_gpio_level[32];
static jmp_buf g_exit;
static int g_iter;
static void scenario_step(void);
void tight_loop_contents(void) { scenario_step(); }

#define main ir_main
#include "../tests/ir/ir_test.c"
#undef main

static int g_level;

static void edge_at(uint32_t t_us)
{
    stub_timer.timerawl = t_us;
    g_level ^= 1;
    gpio_isr(IR_BARCODE_PIN, g_level ? GPIO_IRQ_EDGE_RISE : GPIO_IRQ_EDGE_FALL);
}

static void scenario_step(void)
{
    g_iter++;
    stub_now_us += 1u;  /* keep the status print from firing */
    if (g_iter == 1)
    {
        edge_at(25800u);    /* first edge after the loss */
    }
    if (g_tail == g_head)
    {
        longjmp(g_exit, 1);
    }
}

int main(void)
{
    uint32_t i;
    int k;
    for (k = 0; k < 32; k++) { stub_gpio_level[k] = true; }   /* buttons up */
    stub_now_us = 5000000u;
    for (i = 0u; i < 256u; i++)
    {
        edge_at(100u + i * 100u);   /* 100 .. 25,600 us */
    }
    edge_at(25700u);                /* ring full: dropped */
    printf("ring head=%lu overflow=%lu (want 256, 1)\n", (unsigned long)g_head, (unsigned long)g_overflow);
    if (setjmp(g_exit) == 0)
    {
        (void)ir_main();
    }
    return 0;
}
