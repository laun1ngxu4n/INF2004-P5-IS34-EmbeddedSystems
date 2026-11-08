/*
 *	app_timing.c
 *	Per-task timing monitor (see app_timing.h).
 *
 *	Each slot is written by its own task (wake/done) and read and reset
 *	by Telemetry (report), so every access goes through APP_LOCK.  The
 *	locked sections are a few loads and stores.
 */

#include "app.h"

/* RP2040 TIMER block, TIMERAWL: free-running 1 MHz counter, read-only.
   Reading it has no side effects (unlike TIMELR/TIMEHR). */
#define RP2040_TIMERAWL		(*(volatile UW *)0x40054028)

LOCAL APP_TIMING	tmg[APP_TIMING_MAX];
LOCAL INT		tmg_used;

EXPORT UW app_time_us(void)
{
	return RP2040_TIMERAWL;
}

/* Called only from the subsystems' _init(), which usermain() runs one
   after another before any task starts, so no lock is needed here. */
EXPORT INT app_timing_add(const char *name, PRI pri, UINT period_ms)
{
	APP_TIMING	*t;

	if (tmg_used >= APP_TIMING_MAX) {
		return E_LIMIT;
	}
	t = &tmg[tmg_used];
	t->name = name;
	t->pri = pri;
	t->period_ms = period_ms;
	t->win_min_iv_us = 0xFFFFFFFFu;
	return tmg_used++;
}

EXPORT void app_timing_wake(INT slot, INT overruns)
{
	APP_TIMING	*t;
	UW		now = app_time_us();
	UW		iv;
	UINT		s;

	if ((slot < 0) || (slot >= tmg_used)) {
		return;
	}
	t = &tmg[slot];

	APP_LOCK(s);
	if (t->count > 0) {
		iv = now - t->last_us;
		if (iv < t->win_min_iv_us) t->win_min_iv_us = iv;
		if (iv > t->win_max_iv_us) t->win_max_iv_us = iv;
		if (iv > t->max_iv_us) {
			t->max_iv_us = iv;
			t->max_iv_at_us = now;
		}
		if (iv > t->test_max_iv_us) t->test_max_iv_us = iv;
	}
	if (overruns > 0) {
		t->overruns += (UW)overruns;
	}
	t->last_us = now;
	t->run_start_us = now;
	t->count++;
	APP_UNLOCK(s);
}

EXPORT void app_timing_done(INT slot)
{
	APP_TIMING	*t;
	UW		run;
	UINT		s;

	if ((slot < 0) || (slot >= tmg_used)) {
		return;
	}
	t = &tmg[slot];

	APP_LOCK(s);
	run = app_time_us() - t->run_start_us;
	if (run > t->win_max_run_us) t->win_max_run_us = run;
	if (run > t->max_run_us)     t->max_run_us = run;
	APP_UNLOCK(s);
}

EXPORT void app_timing_test_start(INT slot)
{
	UINT	s;

	if ((slot < 0) || (slot >= tmg_used)) {
		return;
	}
	APP_LOCK(s);
	tmg[slot].test_max_iv_us = 0;
	APP_UNLOCK(s);
}

EXPORT ER app_timing_get(INT slot, APP_TIMING *p_copy)
{
	UINT	s;

	if ((slot < 0) || (slot >= tmg_used)) {
		return E_PAR;
	}
	APP_LOCK(s);
	*p_copy = tmg[slot];
	APP_UNLOCK(s);
	return E_OK;
}

/* Print microseconds as milliseconds with two decimals, e.g. "10.02" */
LOCAL void print_ms(UW us)
{
	tm_printf((UB*)"%d.%02d", (INT)(us / 1000), (INT)((us % 1000) / 10));
}

EXPORT void app_timing_report(void)
{
	APP_TIMING	c;
	INT		i;
	UINT		s;

	tm_printf((UB*)"[tmg] t=%d ms | task pri period | wakes overruns | "
		  "interval min-max ms (boot max @ s) | run max us (boot max)\n",
		  (INT)(app_time_us() / 1000));
	for (i = 0; i < tmg_used; i++) {
		/* Copy and start a new window in one locked step, print after */
		APP_LOCK(s);
		c = tmg[i];
		tmg[i].win_min_iv_us = 0xFFFFFFFFu;
		tmg[i].win_max_iv_us = 0;
		tmg[i].win_max_run_us = 0;
		APP_UNLOCK(s);

		tm_printf((UB*)"[tmg] %-9s %2d %3d | %6d %3d | ",
			  c.name, (INT)c.pri, (INT)c.period_ms,
			  (INT)c.count, (INT)c.overruns);
		if (0 == c.win_max_iv_us) {
			tm_printf((UB*)"no wake this window");
		} else {
			print_ms(c.win_min_iv_us);
			tm_printf((UB*)"-");
			print_ms(c.win_max_iv_us);
			tm_printf((UB*)" (");
			print_ms(c.max_iv_us);
			tm_printf((UB*)" @ %d.%d", (INT)(c.max_iv_at_us / 1000000),
				  (INT)((c.max_iv_at_us % 1000000) / 100000));
			tm_printf((UB*)")");
		}
		tm_printf((UB*)" | %d (%d)\n", (INT)c.win_max_run_us, (INT)c.max_run_us);
	}
}
