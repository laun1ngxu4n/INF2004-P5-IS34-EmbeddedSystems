/**
 * @file    app_main.h
 *  usermain() and the task helpers every subsystem uses (app_main.c).
 */

#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <tk/tkernel.h>
#include "app_types.h"

/** Create and start a task.  prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or
   0 for "any core".  Defined in app_main.c. */
IMPORT ID app_main_start_task (FP entry, PRI pri, uint32_t prc,
                               const char * name);

/** Create a cyclic handler that wakes the task whose ID is stored at
   *p_tid every period_ms.  Create it in _init (it starts stopped); start
   it with tk_sta_cyc() in _start, after the task exists.  Defined in
   app_main.c. */
IMPORT ID app_main_create_wake_cyclic (ID * p_tid, RELTIM period_ms);

#endif /* APP_MAIN_H */

/*** end of file ***/

