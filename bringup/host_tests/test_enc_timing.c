/* Host test: encoder timing helpers (tests/common/enc_timing.h, used by
 * encoder_test). Review findings 04 (false stall from a read race) and 14
 * (sampled age misses the longest interval). */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "../tests/common/enc_timing.h"

static int g_fail;
static int g_checks;

static void check(bool ok, const char *what)
{
    g_checks++;
    if (!ok) { g_fail++; printf("FAIL %s\n", what); }
}

int main(void)
{
    /* --- 04: the reproduced race. now saved at 1,000,000 us, then an edge
     * at 1,000,001 us lands before last-edge is read. v3.1 computed
     * now - last = 4,294,967,295 us (~4,295 s) and stopped the motors. */
    {
        uint32_t now = 1000000u;
        uint32_t last = 1000001u;
        uint32_t old_gap = now - last;
        uint32_t age = enc_edge_age_us(now, last);
        check(old_gap == 4294967295u, "v3.1 arithmetic reproduces the 4,295 s gap");
        check(age == 0u, "edge newer than now gives age 0");
        check(!enc_stall_tripped(50u, 30u, age, 200u), "race does not trip the stall rule");
    }
    /* Timer wrap: last edge just before the 32-bit wrap, now just after. */
    check(enc_edge_age_us(100u, 0xFFFFFF00u) == 356u, "age across timer wrap");
    /* Stall boundaries at 50 % duty: 199 / 200 / 201 ms. */
    check(!enc_stall_tripped(50u, 30u, 199999u, 200u), "199.999 ms: no stall");
    check(enc_stall_tripped(50u, 30u, 200000u, 200u), "200 ms: stall");
    check(enc_stall_tripped(50u, 30u, 201000u, 200u), "201 ms: stall");
    /* Rule applies only ABOVE 30 % duty (as written in Week 6). */
    check(!enc_stall_tripped(30u, 30u, 500000u, 200u), "30 % duty: rule inactive");
    check(enc_stall_tripped(31u, 30u, 500000u, 200u), "31 % duty: rule active");

    /* --- 14: phase sweep. Edges every 2 ms, with one 47 ms interval; the
     * main loop samples the age only every 1 ms, at a phase that varies.
     * The ISR-side maximum must always find 47 ms; the sampled open age can
     * be lower and must never be higher. */
    {
        int phase_us;
        bool all_found = true;
        bool sampled_never_higher = true;
        for (phase_us = 0; phase_us < 1000; phase_us += 37)
        {
            uint32_t max_iv = 0u;
            uint32_t last = 0u;
            uint32_t sampled_max = 0u;
            uint32_t t_edge = 0u;
            uint32_t next_sample = (uint32_t)phase_us;
            int k;
            for (k = 1; k <= 60; k++)
            {
                uint32_t step = (k == 30) ? 47000u : 2000u;
                t_edge += step;
                while (next_sample < t_edge)   /* main-loop samples before this edge */
                {
                    uint32_t a = enc_edge_age_us(next_sample, last);
                    if (a > sampled_max) { sampled_max = a; }
                    next_sample += 1000u;
                }
                max_iv = enc_interval_note(max_iv, last, t_edge, true);
                last = t_edge;
            }
            if (max_iv != 47000u) { all_found = false; }
            if (sampled_max > max_iv) { sampled_never_higher = false; }
        }
        check(all_found, "ISR maximum finds the 47 ms interval at every phase");
        check(sampled_never_higher, "sampled age is a lower bound of the ISR maximum");
    }
    /* Not armed: nothing recorded (spin-up excluded). */
    check(enc_interval_note(0u, 0u, 90000u, false) == 0u, "unarmed interval not recorded");

    printf("enc_timing: %s (%d checks, %d failures)\n", g_fail ? "FAILED" : "all passed", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
