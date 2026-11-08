/**
 * @file    app_lock.h
 *  The one lock for shared data.  Shared by the whole team.
 *
 *  Use it for every variable that more than one task or interrupt
 *  handler WRITES (read-modify-write), e.g. stop_reasons or fault bits:
 *
 *      UINT s;
 *      APP_LOCK(s);
 *      stop_reasons |= STOP_BIT;
 *      APP_UNLOCK(s);
 *
 *  Keep the locked section a few lines long: no kernel waits, no
 *  tm_printf, no loops.  It is safe inside interrupt and cyclic handlers.
 *
 *  Never rely on task priority instead of this lock: on a dual-core build
 *  a lower-priority task can run at the same time on the other core.
 *
 *  Single core (SMP=0, the team's choice): turning interrupts off on this
 *  core is enough, because nothing else can run.  On SMP=1 this is NOT
 *  enough, so the build stops until a hardware spinlock is added here.
 */

#ifndef APP_LOCK_H
#define APP_LOCK_H

#include <tk/tkernel.h>

#if TK_SUPPORT_SMP
#error \
    "app_lock.h: SMP=1 needs a hardware spinlock in APP_LOCK/APP_UNLOCK first"
#endif

#define APP_LOCK(intsts)   DI(intsts)
#define APP_UNLOCK(intsts) EI(intsts)

#endif /* APP_LOCK_H */

/*** end of file ***/

