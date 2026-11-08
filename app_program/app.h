/**
 * @file    app.h
 *  Definitions shared by the whole team.  Changed only by team agreement.
 *
 *  Anything that belongs to ONE subsystem goes in that subsystem's own
 *  header instead (app_program/subsystemN/subsystemN.h).
 *
 *  P5-IS34 Stage 0 (26 Oct): the 8 tasks of the design exist and run at
 *  their periods, but do no robot work yet.  Each later stage fills in a
 *  task body; the task list, priorities and periods below stay.
 */

#ifndef APP_H
#define APP_H

#include <tk/tkernel.h>  /* micro T-Kernel API (tk_xxx_yyy) */
#include <tm/tmonitor.h> /* T-Monitor API: tm_printf() console output */
#include <bsp/libbsp.h>  /* Board helpers: gpio_set_pin/val, pwm_... */

#include "app_types.h" /* int32_t, bool, float32_t ... (Barr C 5.2) */

#include "app_lock.h"   /* APP_LOCK / APP_UNLOCK for shared data */
#include "app_timing.h" /* per-task period and run-time monitor */
#include "app_gpio.h"   /* the shared GPIO interrupt dispatcher */
#include "app_stop.h"   /* the one STOP helper */

/* ---------------------------------------------------------------------
 *  Pins: free for projects are GP0..GP22 and GP26..GP28.  GP0/GP1 are
 *  taken by UART0 when built with "make CRASH_REPORT=1" (see README).
 *  Never use GP23/24/25/29: they are wired to the Pico W radio.
 *  Claim every pin in TEAM.md section 2 before using it.
 *  P5-IS34: GP0/GP1 are the IR sensors, so never build CRASH_REPORT=1
 *  with the IR sensors plugged in.
 * --------------------------------------------------------------------- */

/* ---------------------------------------------------------------------
 *  Task priorities: 1 is the highest; the kernel's USB console (3) and
 *  WiFi (4) tasks sit above every application task.
 *
 *  P5-IS34 (TEAM.md section 4, draft for the team to confirm): the Week 6
 *  order, shifted down to 5..12 below the kernel.  Notification is merged
 *  into Telemetry, so the design's 9 tasks become 8.
 * --------------------------------------------------------------------- */
#define PRI_SAFETY    5  /* S4  Safety / Fault Monitor */
#define PRI_MOTION    6  /* S2  Motion Control */
#define PRI_MISSION   7  /* S5  Mission Controller */
#define PRI_LINE      8  /* S3  Line Following */
#define PRI_IMU       9  /* S4  IMU / Terrain Monitor */
#define PRI_BARCODE   10 /* S3  Barcode Decoder */
#define PRI_OBSTACLE  11 /* S5  Obstacle Scanner */
#define PRI_TELEMETRY 12 /* S1  Telemetry + Notification */

/* Task periods in ms (Week 6 task table).  The kernel tick is 10 ms
   (CNF_TIMER_PERIOD in config/config.h), so every period is a multiple
   of 10 ms and a timeout is only exact to the next tick. */
#define PERIOD_SAFETY_MS    20
#define PERIOD_MOTION_MS    10
#define PERIOD_LINE_MS      20
#define PERIOD_IMU_MS       10
#define PERIOD_OBSTACLE_MS  60
#define PERIOD_TELEMETRY_MS 100 /* LED/buzzer tick; table every 10 s */
#define TIMEOUT_BARCODE_MS  10  /* event-driven, quiet-zone check */
#define TIMEOUT_MISSION_MS  100 /* event-driven, wait on gh_flg_mission */

#define TASK_STACK_SIZE 2048 /* bytes; tm_printf needs ~1 KB */

/** Create and start a task.  prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or
   0 for "any core".  Defined in app_main.c. */
IMPORT ID app_start_task (FP entry, PRI pri, uint32_t prc, const char * name);

/** Create a cyclic handler that wakes the task whose ID is stored at
   *p_tid every period_ms.  Create it in _init (it starts stopped); start
   it with tk_sta_cyc() in _start, after the task exists.  Defined in
   app_main.c. */
IMPORT ID app_create_wake_cyclic (ID * p_tid, RELTIM period_ms);

/** ---------------------------------------------------------------------
 *  Every subsystem provides these.  usermain() calls ALL the _init()
 *  functions first and only then ALL the _start() functions, so every
 *  kernel object exists before any task can use it.
 *    subsystemN_init()   create this subsystem's kernel objects
 *    subsystemN_start()  create and start this subsystem's tasks
 *    subsystemN_test()   on-target tests, only in test builds (make TEST=1)
 * --------------------------------------------------------------------- */
IMPORT ER subsystem1_init (void);
IMPORT ER subsystem2_init (void);
IMPORT ER subsystem3_init (void);
IMPORT ER subsystem4_init (void);
IMPORT ER subsystem5_init (void);

IMPORT ER subsystem1_start (void);
IMPORT ER subsystem2_start (void);
IMPORT ER subsystem3_start (void);
IMPORT ER subsystem4_start (void);
IMPORT ER subsystem5_start (void);

#if APP_TEST
#define APP_TEST_SETTLE_MS 3000 /* wait for WiFi start-up before testing */
IMPORT void subsystem1_test (void);
IMPORT void subsystem2_test (void);
IMPORT void subsystem3_test (void);
IMPORT void subsystem4_test (void);
IMPORT void subsystem5_test (void);
IMPORT void
app_stage_test (void); /* hands-on, TEST_STAGE=n (app_stage_test.c) */
#endif

/** Print a readable message if a kernel call failed (negative error code) */
Inline void app_check (int32_t err, const char * p_what)
{
    if (err < E_OK)
    {
        tm_printf((uint8_t *)"ERROR %s: %d\n", p_what, err);
    }
}

#endif /* APP_H */

/*** end of file ***/

