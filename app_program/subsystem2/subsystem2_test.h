/**
 * @file    subsystem2_test.h
 *  On-target tests for subsystem 2 (subsystem2_test.c).  Built only
 *  with: make TEST=1
 */

#ifndef SUBSYSTEM2_TEST_H
#define SUBSYSTEM2_TEST_H

#include "../app.h"

#if APP_TEST
IMPORT void subsystem2_test (void);       /* automatic, at start-up */
IMPORT void subsystem2_test_drive (void); /* stage 1 */
#endif

#endif /* SUBSYSTEM2_TEST_H */

/*** end of file ***/

