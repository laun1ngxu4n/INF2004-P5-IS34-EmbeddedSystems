/**
 * @file    safety.h
 *  Safety / Fault Monitor (safety.c).  Part of subsystem 4's
 *  interface (subsystem4.h includes it).
 */

#ifndef SAFETY_H
#define SAFETY_H

#include "../app.h"

IMPORT void safety_task (int32_t stacd, void * exinf);

#endif /* SAFETY_H */

/*** end of file ***/

