/**
 * @file    app_types.h
 * @brief   The data types our code uses (Barr C 5.2, 5.4, 5.6).
 *
 * Fixed-width integers come from <stdint.h> and Booleans from <stdbool.h>.
 * C99 has no name for a 32-bit float, so float32_t is defined here (the
 * Cortex-M0+ float is IEEE 754 single precision).
 *
 * The micro T-Kernel types (INT, UINT, ID, ER, BOOL, ...) are used only
 * where a kernel call needs them: docs/team/BARR_C_DEVIATIONS.md.
 */

#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdint.h>
#include <stdbool.h>

typedef float float32_t;

/* Change Requirement 1, Minimum-Bit Data Representation
   (docs/team/CHANGE_REQ1_PLAN.md).  APP_PACKED 1: each application data
   field takes only the bits its range needs (a C bit-field, written
   "uint32_t x APP_FIELD(5);").  APP_PACKED 0: every field gets its own
   full storage unit again, as before the change, so the two builds can be
   compared (make EXTRA_CFLAGS=-DAPP_PACKED=0).  NOTE: a function-like
   macro (Barr C deviation D4): it is part of a declaration, so it cannot
   be a function. */
#ifndef APP_PACKED
#define APP_PACKED 1
#endif

#if APP_PACKED
#define APP_FIELD(bits) : (bits)
#else
#define APP_FIELD(bits)
#endif

#endif /* APP_TYPES_H */

/*** end of file ***/

