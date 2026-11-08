/**
 * @file    subsystem5_test.h
 *  On-target tests for subsystem 5 (subsystem5_test.c).  Built only
 *  with: make TEST=1
 */

#ifndef SUBSYSTEM5_TEST_H
#define SUBSYSTEM5_TEST_H

#include "../app.h"

#if APP_TEST
IMPORT void subsystem5_test (void);       /* automatic, at start-up */
IMPORT void subsystem5_test_stage (void); /* stage 3 */
#endif

#endif /* SUBSYSTEM5_TEST_H */

/*** end of file ***/

