/* Host test: STOP-safe motor steps (tests/common/motor_seq.h, used by
 * parts_check). Review finding 03 cases: STOP held before the first step,
 * STOP pressed during the start delay, during a run, during the 400 ms
 * spin-down gap, and a short press between steps. A fake clock advances
 * 1 ms per sleep; a script decides when STOP is requested. */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "../tests/common/motor_seq.h"

static uint32_t g_now;
static uint32_t g_drive_calls;
static uint32_t g_drive_ms;        /* ms spent with a non-zero drive */
static bool     g_driving;
static uint32_t g_stop_from;       /* STOP requested from this time ... */
static uint32_t g_stop_to;         /* ... until this time (latched: UINT32_MAX) */
static int      g_fail;
static int      g_checks;

static void fake_drive(uint32_t w, bool f, uint32_t d) { (void)w; (void)f; if (d > 0u) { g_drive_calls++; g_driving = true; } }
static void fake_off(void) { g_driving = false; }
static bool fake_stop(void) { return (g_now >= g_stop_from) && (g_now < g_stop_to); }
static uint32_t fake_now(void) { return g_now; }
static void fake_sleep(uint32_t ms) { if (g_driving) { g_drive_ms += ms; } g_now += ms; }

static const motor_hal_t hal = {fake_drive, fake_off, fake_stop, fake_now, fake_sleep, 0};

static void reset(uint32_t stop_from, uint32_t stop_to)
{
    g_now = 0u; g_drive_calls = 0u; g_drive_ms = 0u; g_driving = false;
    g_stop_from = stop_from; g_stop_to = stop_to;
}

static void check(bool ok, const char *what)
{
    g_checks++;
    if (!ok) { g_fail++; printf("FAIL %s (drive calls %u, drive ms %u, now %u)\n", what,
                                (unsigned)g_drive_calls, (unsigned)g_drive_ms, (unsigned)g_now); }
}

/* The parts_check sequence: 1 s start delay, then 4 steps of 600 ms run +
 * 400 ms spin-down. Returns the number of completed steps. */
static int sequence(void)
{
    int steps = 0;
    uint32_t w;
    uint32_t d;
    if (!motor_wait_or_stop(&hal, 1000u)) { return steps; }
    for (w = 0u; w < 2u; w++)
    {
        for (d = 0u; d < 2u; d++)
        {
            if (!motor_run_step(&hal, w, d == 0u, 50u, 600u, 400u)) { return steps; }
            steps++;
        }
    }
    return steps;
}

int main(void)
{
    int steps;
    const uint32_t LATCH = UINT32_MAX;

    /* 1. Normal run: 4 steps, 4 drive calls, motors off at the end. */
    reset(LATCH, LATCH);
    steps = sequence();
    check((steps == 4) && (g_drive_calls == 4u) && !g_driving, "normal sequence runs 4 steps");

    /* 2. STOP held from the start: never drives (v3.1 drove once first). */
    reset(0u, LATCH);
    steps = sequence();
    check((steps == 0) && (g_drive_calls == 0u), "STOP held before start: no drive at all");

    /* 3. Short press during the 1 s start delay (latched): no drive. */
    reset(500u, LATCH);
    steps = sequence();
    check((steps == 0) && (g_drive_calls == 0u), "STOP in start delay: no drive");

    /* 4. STOP during the first run: off within 1 ms, sequence aborted. */
    reset(1300u, LATCH);
    steps = sequence();
    check((steps == 0) && (g_drive_calls == 1u) && !g_driving && (g_drive_ms <= 301u),
          "STOP during a run: off within 1 ms, abort");

    /* 5. Short latched press in the 400 ms spin-down gap after step 1:
     *    step 2 must never start (v3.1 forgot this press). */
    reset(1800u, LATCH);
    steps = sequence();
    check((steps == 0) && (g_drive_calls == 1u), "STOP in spin-down gap: next step never starts");

    /* 6. A 5 ms press that is NOT latched still aborts if it overlaps a check
     *    (checks run every 1 ms). */
    reset(2700u, 2705u);
    steps = sequence();
    check((steps == 1) && (g_drive_calls == 2u) && !g_driving, "5 ms press during step 2: abort");

    /* 7. The step itself, called directly with STOP held (e.g. a press that
     *    lands right after the previous wait): must not drive even once. */
    reset(0u, LATCH);
    check(!motor_run_step(&hal, 0u, true, 50u, 600u, 400u) && (g_drive_calls == 0u),
          "motor_run_step with STOP held: no drive");

    printf("motor_seq: %s (%d checks, %d failures)\n", g_fail ? "FAILED" : "all passed", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
