/*
 *	app_stage_test.h
 *	Hands-on stage tests (make TEST=1 TEST_STAGE=n).  Shared by the team.
 *	They run after the automatic Stage 0 tests and wait for the START
 *	button (GP20) before anything moves.  STOP (GP21) works at any time.
 */

#ifndef APP_STAGE_TEST_H
#define APP_STAGE_TEST_H

#include "app.h"

#if APP_TEST
IMPORT void	test_wait_start(const char *what);	/* START pressed and released */
IMPORT BOOL	test_stop_pressed(void);		/* a STOP reason is set */
IMPORT void	test_print_x10(const char *before, INT x10, const char *after);
IMPORT void	test_use_test_mapping(void);		/* A=L,B=R,C=F,D=U, tests only */

IMPORT void	subsystem2_drive_test(void);		/* stage 1 */
IMPORT void	subsystem3_stage_test(void);		/* stage 2 */
IMPORT void	subsystem5_stage_test(void);		/* stage 3 */
IMPORT void	subsystem4_stage_test(void);		/* stage 4 */
IMPORT void	app_course_test(void);			/* stage 6 */
#endif

#endif /* APP_STAGE_TEST_H */
