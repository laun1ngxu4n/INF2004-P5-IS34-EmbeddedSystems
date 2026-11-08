/**
 * @file    subsystem1_test.h
 *  On-target tests for subsystem 1 (subsystem1_test.c).  Built only
 *  with: make TEST=1
 */

#ifndef SUBSYSTEM1_TEST_H
#define SUBSYSTEM1_TEST_H

#include "../app.h"

#if APP_TEST
IMPORT void subsystem1_test (void); /* automatic, at start-up */
#endif

#endif /* SUBSYSTEM1_TEST_H */

/*** end of file ***/

