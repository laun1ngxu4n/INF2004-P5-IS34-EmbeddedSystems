/*
 *	test_report.h
 *	Minimal on-target test helpers.
 *
 *	Every check prints ONE line in a fixed format, so a saved console log
 *	can be searched for results ("grep '\[TEST\]' log.txt"):
 *
 *	    [TEST] S1-U01 PASS  ADC reads in range
 *	    [TEST] S1-U02 FAIL  queue rejects overflow   (task_x.c:57)
 *	    [TEST] subsystem1 SUMMARY 1 passed, 1 failed
 *
 *	Usage, inside a test function:
 *	    TEST_INIT();
 *	    TEST_CHECK("S1-U01", value >= 0 && value <= 4095, "ADC reads in range");
 *	    TEST_SUMMARY("subsystem1");
 *
 *	Test IDs must match the ones in testing/<subsystem>/TEST_LOG.md.
 */

#ifndef TEST_REPORT_H
#define TEST_REPORT_H

#include <tm/tmonitor.h>

#define TEST_INIT()	INT test_pass_ = 0, test_fail_ = 0

#define TEST_CHECK(id, cond, desc)					\
	do {								\
		if (cond) {						\
			test_pass_++;					\
			tm_printf((UB*)"[TEST] %s PASS  %s\n", (id), (desc)); \
		} else {						\
			test_fail_++;					\
			tm_printf((UB*)"[TEST] %s FAIL  %s   (%s:%d)\n",	\
				  (id), (desc), __FILE__, __LINE__);	\
		}							\
	} while (0)

#define TEST_SUMMARY(name)						\
	tm_printf((UB*)"[TEST] %s SUMMARY %d passed, %d failed\n",	\
		  (name), test_pass_, test_fail_)

/*
 * P5-IS34 Stage 0: checks for a task timed by app_timing (app_timing.h).
 * Watches the task for APP_TEST_WINDOW_MS while the whole system runs.
 *
 *   TEST_PERIODIC   task woken by a cyclic handler every period_ms
 *     id_rate:  wakes in the window within 10 % of window / period
 *     id_time:  no overrun since boot, longest interval since boot
 *               <= period + APP_TEST_SLACK_MS
 *   TEST_TIMEOUT    event task that waits with a timeout of period_ms
 *                   (a kernel timeout ends on the next 10 ms tick, so each
 *                   wait lasts period .. period + 10 ms)
 *     id_rate:  at least window / (period + 10 ms) wakes
 *     id_time:  longest interval since boot <= period + 10 ms + slack
 */
#define APP_TEST_WINDOW_MS	2000
#define APP_TEST_SLACK_MS	15	/* one 10 ms tick + 5 ms */

#define TEST_TIMED_(id_rate, id_time, slot, is_periodic)		\
	do {								\
		APP_TIMING a_, b_;					\
		INT per_, exp_, got_, lim_ms_;				\
		app_timing_get((slot), &a_);				\
		tk_dly_tsk(APP_TEST_WINDOW_MS);				\
		app_timing_get((slot), &b_);				\
		per_ = (INT)b_.period_ms;				\
		got_ = (INT)(b_.count - a_.count);			\
		exp_ = APP_TEST_WINDOW_MS / ((is_periodic) ? per_ : per_ + 10); \
		lim_ms_ = per_ + APP_TEST_SLACK_MS + ((is_periodic) ? 0 : 10); \
		tm_printf((UB*)"[TEST] %s: %d wakes in %d ms (expected %s%d), "	\
			  "longest interval %d us (limit %d ms), overruns %d\n", \
			  b_.name, got_, APP_TEST_WINDOW_MS,		\
			  (is_periodic) ? "" : "at least ", exp_,	\
			  (INT)b_.max_iv_us, lim_ms_, (INT)b_.overruns); \
		TEST_CHECK((id_rate), (is_periodic) ?			\
			   (got_ * 10 >= exp_ * 9 && got_ * 10 <= exp_ * 11 + 10) : \
			   (got_ >= exp_), "wake rate");		\
		TEST_CHECK((id_time), b_.overruns == 0 &&		\
			   b_.max_iv_us <= (UW)lim_ms_ * 1000u,		\
			   "no overrun, longest interval within limit");	\
	} while (0)

#define TEST_PERIODIC(id_rate, id_time, slot)	TEST_TIMED_(id_rate, id_time, slot, 1)
#define TEST_TIMEOUT(id_rate, id_time, slot)	TEST_TIMED_(id_rate, id_time, slot, 0)

#endif /* TEST_REPORT_H */
