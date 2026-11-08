/**
 * @file    app_test.h
 *  Hands-on stage tests (make TEST=1 TEST_STAGE=n).  Shared by the team.
 *  They run after the automatic Stage 0 tests and wait for the START
 *  button (GP20) before anything moves.  STOP (GP21) works at any time.
 */

#ifndef APP_TEST_H
#define APP_TEST_H

#include "app.h"

#if APP_TEST
/** Runs the hands-on test chosen at build time (TEST_STAGE). */
IMPORT void app_test_stage (void);
IMPORT void
app_test_wait_start (const char * what);  /* START pressed and released */
IMPORT bool app_test_stop_pressed (void); /* a STOP reason is set */
IMPORT void app_test_print_x10 (const char * before, int32_t x10,
                                const char * after);
IMPORT void app_test_use_test_mapping (void); /* A=L,B=R,C=F,D=U, tests only */

IMPORT void app_test_course (bool obstacles,
                             bool imu); /* Mission run; stage 6 = all on */
#endif

#endif /* APP_TEST_H */

/*** end of file ***/

