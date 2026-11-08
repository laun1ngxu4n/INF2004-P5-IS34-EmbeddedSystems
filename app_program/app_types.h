/**
 * @file    app_types.h
 * @brief   The data types our code uses (Barr C 5.2, 5.4, 5.6).
 *
 * Fixed-width integers come from <stdint.h> and Booleans from <stdbool.h>.
 * C99 has no name for a 32-bit float, so float32_t is defined here (the
 * Cortex-M0+ float is IEEE 754 single precision).
 *
 * The micro T-Kernel types (INT, UINT, ID, ER, bool, ...) are used only
 * where a kernel call needs them: docs/team/BARR_C_DEVIATIONS.md.
 */

#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdint.h>
#include <stdbool.h>

typedef float float32_t;

#endif /* APP_TYPES_H */

/*** end of file ***/

