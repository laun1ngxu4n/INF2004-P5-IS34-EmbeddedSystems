/**
 * @file    test_report.h
 *  Minimal on-target test helpers.
 *
 *  Every check prints ONE line in a fixed format, so a saved console log
 *  can be searched for results (every result line starts with [TEST]):
 *
 *      [TEST] S1-U01 PASS  ADC reads in range
 *      [TEST] S1-U02 FAIL  queue rejects overflow   (task_x.c:57)
 *      [TEST] subsystem1 SUMMARY 1 passed, 1 failed
 *
 *  Usage, inside a test function:
 *      TEST_INIT();
 *      TEST_CHECK("S1-U01", value >= 0 && value <= 4095, "ADC reads in range");
 *      TEST_SUMMARY("subsystem1");
 *
 *  Test IDs must match the ones in testing/<subsystem>/TEST_LOG.md.
 */

#ifndef TEST_REPORT_H
#define TEST_REPORT_H

#include <tm/tmonitor.h>

/* A failed check says where it is.  APP_COMPARE_BUILD (tools/barr/
   compare.sh) leaves the file and line out, so a reformatted source gives
   the same image as before and the two can be compared byte for byte. */
#ifdef APP_COMPARE_BUILD
#define TEST_WHERE_FILE ""
#define TEST_WHERE_LINE 0
#else
#define TEST_WHERE_FILE __FILE__
#define TEST_WHERE_LINE __LINE__
#endif

/* NOTE: the TEST_ checks are macros, not functions (Barr C 6.3a
   deviation): they count into the calling test's own test_pass_ /
   test_fail_ and report the caller's file and line. */
#define TEST_INIT()         \
    int32_t test_pass_ = 0; \
    int32_t test_fail_ = 0

#define TEST_CHECK(id, cond, desc)                                       \
    do                                                                   \
    {                                                                    \
        if (cond)                                                        \
        {                                                                \
            test_pass_++;                                                \
            tm_printf((uint8_t *)"[TEST] %s PASS  %s\n", (id), (desc));  \
        }                                                                \
        else                                                             \
        {                                                                \
            test_fail_++;                                                \
            tm_printf((uint8_t *)"[TEST] %s FAIL  %s   (%s:%d)\n", (id), \
                      (desc), TEST_WHERE_FILE, TEST_WHERE_LINE);         \
        }                                                                \
    } while (0)

#define TEST_SUMMARY(name)                                                   \
    tm_printf((uint8_t *)"[TEST] %s SUMMARY %d passed, %d failed\n", (name), \
              test_pass_, test_fail_)

/*
 * P5-IS34 Stage 0: checks for a task timed by app_timing (app_timing.h).
 * Watches the task for APP_TEST_WINDOW_MS while the whole system runs and
 * judges only that window, so a one-off delay at boot does not fail it
 * (the timing table's "boot max" still shows it).
 *
 *   TEST_PERIODIC   task woken by a cyclic handler every period_ms
 *     id_rate:  wakes in the window within 10 % of window / period
 *     id_time:  no overrun in the window, longest interval in the window
 *               <= period + APP_TEST_SLACK_MS
 *   TEST_TIMEOUT    event task that waits with a timeout of period_ms
 *                   (a kernel timeout ends on a 10 ms tick and lasts up to
 *                   one extra tick, so each wait is period .. period + 10 ms)
 *     id_rate:  at least 90 % of window / (period + 10 ms) wakes
 *     id_time:  longest interval in the window <= period + 10 ms + slack
 */
#define APP_TEST_WINDOW_MS 2000
#define APP_TEST_SLACK_MS  15 /* one 10 ms tick + 5 ms */

#define TEST_TIMED_(id_rate, id_time, slot, is_periodic)                       \
    do                                                                         \
    {                                                                          \
        app_timing_t before_;                                                  \
        app_timing_t after_;                                                   \
        int32_t      per_;                                                     \
        int32_t      exp_;                                                     \
        int32_t      got_;                                                     \
        int32_t      ovr_;                                                     \
        int32_t      lim_ms_;                                                  \
        app_timing_test_start(slot);                                           \
        app_timing_get((slot), &before_);                                      \
        tk_dly_tsk(APP_TEST_WINDOW_MS);                                        \
        app_timing_get((slot), &after_);                                       \
        per_    = (int32_t)after_.period_ms;                                   \
        got_    = (int32_t)(after_.count - before_.count);                     \
        ovr_    = (int32_t)(after_.overruns - before_.overruns);               \
        exp_    = APP_TEST_WINDOW_MS / ((is_periodic) ? per_ : (per_ + 10));   \
        lim_ms_ = per_ + APP_TEST_SLACK_MS + ((is_periodic) ? 0 : 10);         \
        tm_printf(                                                             \
            (uint8_t *)"[TEST] %s: %d wakes in %d ms (expected %s%d), "        \
                       "longest interval %d us (limit %d ms), overruns %d\n",  \
            after_.p_name, got_, APP_TEST_WINDOW_MS,                           \
            (is_periodic) ? "" : "about ", exp_,                               \
            (int32_t)after_.test_max_iv_us, lim_ms_, ovr_);                    \
        TEST_CHECK(                                                            \
            (id_rate),                                                         \
            ((got_ * 10) >= (exp_ * 9)) &&                                     \
                ((!(is_periodic)) || ((got_ * 10) <= ((exp_ * 11) + 10))),     \
            "wake rate");                                                      \
        TEST_CHECK((id_time),                                                  \
                   (0 == ovr_) &&                                              \
                       (after_.test_max_iv_us <= ((uint32_t)lim_ms_ * 1000u)), \
                   "no overrun, longest interval within limit");               \
    } while (0)

#define TEST_PERIODIC(id_rate, id_time, slot) \
    TEST_TIMED_(id_rate, id_time, slot, 1)
#define TEST_TIMEOUT(id_rate, id_time, slot) \
    TEST_TIMED_(id_rate, id_time, slot, 0)

#endif /* TEST_REPORT_H */

/*** end of file ***/

