/**
 * @file    app_main.c
 *  START HERE.  usermain() below is your program's entry point.
 *
 *  How micro T-Kernel starts your code
 *  -----------------------------------
 *    reset -> kernel initialises itself -> creates the "initial task"
 *          -> initial task calls usermain()
 *
 *  usermain() runs inside that initial task at priority 1 (the highest).
 *  On a single core nothing you create runs until usermain() blocks, but
 *  on the dual-core build the OTHER core may start a new task at once.
 *  So usermain() works in two phases:
 *    1. every subsystem creates its kernel objects  (subsystemN_init)
 *    2. every subsystem starts its tasks             (subsystemN_start)
 *  and then puts the initial task to sleep forever.
 *  If usermain() RETURNS, the kernel shuts the whole system down.
 *
 *  P5-IS34 Stage 0: the 8 design tasks, one subsystem per buddy
 *  -------------------------------------------------------------
 *    subsystem1/  B1  Telemetry (+ Notification)  100 ms, report 10 s
 *    subsystem2/  B2  Motion Control               10 ms
 *    subsystem3/  B3  Line Following 20 ms, Barcode Decoder (event, 10 ms)
 *    subsystem4/  B4  Safety / Fault Monitor 20 ms, IMU / Terrain 10 ms
 *    subsystem5/  B5  Mission Controller (event, 100 ms), Obstacle 60 ms
 *  Priorities and periods: app.h and TEAM.md section 4.
 *  Stage 0 tasks do no robot work: they wake, are timed (app_timing.c)
 *  and go back to sleep.  Telemetry prints the timing table every 10 s.
 *
 *  Interfaces (TEAM.md section 5):
 *    IF-01  S1 -> S5  event flag gh_flg_mission, bit MSN_FLG_PING (owned by S5)
 *                     Stage 0 check that an event wakes Mission; later
 *                     START/STOP/finish from MQTT use the same flag.
 *
 *  micro T-Kernel APIs used, and where to find them:
 *    tasks       tk_cre_tsk tk_sta_tsk tk_slp_tsk tk_wup_tsk tk_can_wup
 *                tk_dly_tsk tk_sus_tsk tk_rsm_tsk tk_chg_pri tk_ref_tsk
 *                tk_get_tid tk_get_prc                         app_main.c,
 * S1-S4 semaphore   tk_cre_sem tk_wai_sem tk_sig_sem tk_ref_sem       S2, S3,
 * S4 mem. pool   tk_cre_mpf tk_get_mpf tk_rel_mpf tk_ref_mpf       S2, S3, S4
 *    mailbox     tk_cre_mbx tk_snd_mbx tk_rcv_mbx                  S2, S3
 *    mutex       tk_cre_mtx tk_loc_mtx tk_unl_mtx                  S3, S4
 *    msg buffer  tk_cre_mbf tk_snd_mbf tk_rcv_mbf                  S3, S4
 *    event flag  tk_cre_flg tk_set_flg tk_wai_flg                  S3, S4
 *    cyclic      tk_cre_cyc tk_ref_cyc                             S1
 *    alarm       tk_cre_alm tk_sta_alm tk_stp_alm                  S4
 *    time        tk_get_tim                                        S2, S4
 *    system      tk_ref_ver                                        app_main.c
 *
 *  Output appears on the USB serial port (115200 8N1 in any terminal).
 */

#include "app.h"
#include "app_main.h"
#include "app_test.h"
#include "subsystem1/subsystem1_test.h"
#include "subsystem2/subsystem2_test.h"
#include "subsystem3/subsystem3_test.h"
#include "subsystem4/subsystem4_test.h"
#include "subsystem5/subsystem5_test.h"

/**
 * Create and start one task (declared in app.h for every subsystem).
 *   prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for "any core".
 */
EXPORT ID app_main_start_task (FP p_entry, PRI pri, uint32_t prc,
                               const char * p_name)
{
    T_CTSK ctsk = {0};
    ID     h_tid;
    ER     err;

    ctsk.task    = p_entry;         /* function the task runs */
    ctsk.itskpri = pri;             /* initial priority (1 = highest) */
    ctsk.stksz   = TASK_STACK_SIZE; /* kernel allocates the stack */
    ctsk.tskatr  = TA_HLNG          /* written in a high-level language (C) */
                   | TA_RNG3;       /* runs at user protection level */
#if TK_SUPPORT_SMP
    if (prc != 0)
    {
        ctsk.tskatr |= TA_ASSPRC; /* honour the affinity below */
        ctsk.assprc = prc;        /* which core(s) it may run on */
    }
#else
    (void)prc; /* single-core build: no affinity */
#endif

    /* tk_cre_tsk: create the task.  It starts in the DORMANT state. */
    h_tid = tk_cre_tsk(&ctsk);

    if (h_tid < E_OK)
    {
        app_check(h_tid, p_name);
        return h_tid;
    }

    /* tk_sta_tsk: DORMANT -> READY.  The int is passed as stacd. */
    err = tk_sta_tsk(h_tid, 0);
    app_check(err, p_name);
    return h_tid;
}

/** Cyclic handler body shared by every periodic task: exinf points at the
   task's ID variable.  Interrupt context: short, never waits. */
LOCAL void wake_handler (void * p_exinf)
{
    tk_wup_tsk(*(ID *)p_exinf); /* queues if the task is still busy */
}

EXPORT ID app_main_create_wake_cyclic (ID * p_p_tid, RELTIM period_ms)
{
    T_CCYC ccyc = {0};

    ccyc.exinf  = p_p_tid;
    ccyc.cycatr = TA_HLNG; /* created stopped; tk_sta_cyc in _start */
    /* T-Kernel keeps every handler as FP; with TA_HLNG it calls it with
       exinf, wake_handler's one parameter */
    ccyc.cychdr = (FP)wake_handler;
    ccyc.cyctim = period_ms; /* period, ms */
    ccyc.cycphs = period_ms; /* delay before the first call, ms */
    return tk_cre_cyc(&ccyc);
}

/** micro T-Kernel's entry point: starts every subsystem, then the tests or
 * the mission. */
EXPORT int32_t usermain (void)
{
    T_RVER ver;

    /* tk_ref_ver: ask the kernel who it is */
    tk_ref_ver(&ver);
    tm_printf(app_str("\n=== micro T-Kernel on Raspberry Pi Pico W"
                      " (spec 0x%04x, product 0x%04x) ===\n"),
              ver.spver, ver.prver);
#if TK_SUPPORT_SMP
    tm_printf(app_str("dual-core (SMP) build, usermain on core %d\n"),
              tk_get_prc() - 1); /* tk_get_prc() counts from 1 */
#else
    tm_printf(app_str("single-core build\n"));
#endif
#if APP_TEST
    /* the kernel tick, CNF_TIMER_PERIOD = 10 (ms): fits %d */
    tm_printf(app_str("P5-IS34 test image, hands-on TEST_STAGE=%d. "
                      "Kernel tick %d ms\n"),
              APP_TEST_STAGE, (int32_t)CNF_TIMER_PERIOD);
#else
    /* the kernel tick, CNF_TIMER_PERIOD = 10 (ms): fits %d */
    tm_printf(app_str("P5-IS34 mission image. "
                      "Kernel tick %d ms\n"),
              (int32_t)CNF_TIMER_PERIOD);
#endif

    /* The shared GPIO interrupt (app_gpio.c), before anyone registers */
    app_check(app_gpio_init(), "app_gpio_init");

    /* Phase 1: every subsystem creates its kernel objects */
    app_check(subsystem1_init(), "subsystem1_init");
    app_check(subsystem2_init(), "subsystem2_init");
    app_check(subsystem3_init(), "subsystem3_init");
    app_check(subsystem4_init(), "subsystem4_init");
    app_check(subsystem5_init(), "subsystem5_init");

    /* Phase 2: every subsystem starts its tasks */
    app_check(subsystem1_start(), "subsystem1_start");
    app_check(subsystem2_start(), "subsystem2_start");
    app_check(subsystem3_start(), "subsystem3_start");
    app_check(subsystem4_start(), "subsystem4_start");
    app_check(subsystem5_start(), "subsystem5_start");

    tm_printf(app_str("all subsystems started\n\n"));

#if APP_TEST
    /* Test build (make TEST=1): run every subsystem's tests in turn,
       here in the initial task, while the system is running.  Tests may
       call tk_dly_tsk() to let the other tasks work for a while. */
    /* Let start-up finish first: bringing up the WiFi radio pauses every
       task for about 120 ms at about 0.6 s (measured 26 Oct), which is
       not what the timing tests are meant to judge.  The timing table's
       "boot max" still records it. */
    tm_printf(app_str("[TEST] waiting %d ms for start-up to finish\n"),
              APP_TEST_SETTLE_MS);
    tk_dly_tsk(APP_TEST_SETTLE_MS);
    tm_printf(app_str("[TEST] === test run start ===\n"));
    subsystem1_test();
    subsystem2_test();
    subsystem3_test();
    subsystem4_test();
    subsystem5_test();
    tm_printf(app_str("[TEST] === test run end ===\n"));

    /* Hands-on test of the stage picked with TEST_STAGE=n; each move
       waits for the START button */
    app_test_stage();
#endif

    /* Sleep forever.  Returning from usermain() would shut down. */
    tk_slp_tsk(TMO_FEVR);
    return 0;
}

/*** end of file ***/

