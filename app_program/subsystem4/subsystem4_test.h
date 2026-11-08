/**
 * @file    subsystem4_test.h
 *  On-target tests for subsystem 4 (subsystem4_test.c).  Built only
 *  with: make TEST=1
 */

#ifndef SUBSYSTEM4_TEST_H
#define SUBSYSTEM4_TEST_H

#include "../app.h"

#if APP_TEST
IMPORT void subsystem4_test (void);       /* automatic, at start-up */
IMPORT void subsystem4_test_stage (void); /* stage 4 */
#endif

#endif /* SUBSYSTEM4_TEST_H */

/*** end of file ***/

