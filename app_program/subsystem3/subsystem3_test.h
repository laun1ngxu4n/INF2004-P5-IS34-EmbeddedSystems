/**
 * @file    subsystem3_test.h
 *  On-target tests for subsystem 3 (subsystem3_test.c).  Built only
 *  with: make TEST=1
 */

#ifndef SUBSYSTEM3_TEST_H
#define SUBSYSTEM3_TEST_H

#include "../app.h"

#if APP_TEST
IMPORT void subsystem3_test (void);       /* automatic, at start-up */
IMPORT void subsystem3_test_stage (void); /* stage 2 */
#endif

#endif /* SUBSYSTEM3_TEST_H */

/*** end of file ***/

