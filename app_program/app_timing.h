/*
 *  app_timing.h
 *  Per-task timing monitor.  Shared by the whole team.
 *
 *  Every task calls app_timing_wake() right after it wakes and
 *  app_timing_done() when its work for that cycle is finished.  Telemetry
 *  prints the table once a second.  This is the "timing hooks from the
 *  start" item of the build plan: period, lateness, overruns and run time
 *  for every task, measured on the robot rather than estimated.
 *
 *  Times come from the RP2040's 1 MHz hardware timer, because the kernel
 *  clock only counts 10 ms ticks.
 */

#ifndef APP_TIMING_H
#define APP_TIMING_H

#include <tk/tkernel.h>

#define APP_TIMING_MAX 12 /* task slots */

typedef struct
{
    const char * name;
    PRI          pri;
    UINT         period_ms; /* expected wake period (timeout for event tasks) */
    UW           count;     /* wakes since boot */
    UW           overruns;  /* wake-ups that queued while still running */
    UW           last_us;   /* time of the last wake */
    UW           run_start_us;
    /* Since the last report (reset by app_timing_report) */
    UW win_min_iv_us;
    UW win_max_iv_us;
    UW win_max_run_us;
    /* Since boot */
    UW max_iv_us;
    UW max_iv_at_us; /* when the longest interval ended */
    UW max_run_us;
    /* Since app_timing_test_start (on-target tests only) */
    UW test_max_iv_us;
} APP_TIMING;

/* Microseconds since boot from the hardware timer (wraps after ~71 min) */
IMPORT UW app_time_us (void);

/* Register a task in a subsystem's _init(); returns its slot (or < 0) */
IMPORT INT app_timing_add (const char * name, PRI pri, UINT period_ms);

/* Call at the top of every cycle; overruns = value returned by tk_can_wup */
IMPORT void app_timing_wake (INT slot, INT overruns);

/* Call when the cycle's work is finished */
IMPORT void app_timing_done (INT slot);

/* Start a test window: clears test_max_iv_us (tests only) */
IMPORT void app_timing_test_start (INT slot);

/* Copy one slot (for tests); returns E_OK or E_PAR */
IMPORT ER app_timing_get (INT slot, APP_TIMING * p_copy);

/* Print one line per task, then start a new report window */
IMPORT void app_timing_report (void);

#endif /* APP_TIMING_H */

/*** end of file ***/

