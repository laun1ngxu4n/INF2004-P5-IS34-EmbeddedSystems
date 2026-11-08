/**
 * @file    mission.h
 *  Mission Controller (mission.c).  Part of subsystem 5's interface
 *  (subsystem5.h includes it).
 */

#ifndef MISSION_H
#define MISSION_H

#include "../app.h"

/** The mission state's name: "IDLE", "CAL", "RUN" or "MANOEUVRE". */
IMPORT const char * mission_state_name (void);
/** The mission state as a number: 0 IDLE, 1 CAL, 2 RUN, 3 MANOEUVRE
   (2 bits; the binary status message carries it). */
IMPORT uint32_t mission_state (void);
/** Mission reacts to START only when armed: always in the normal image;
   in a test image only after the test arms it (so tests can use START). */
IMPORT void mission_arm (bool on);
/** Which parts Mission uses (all true in the normal image).  Stage tests
   switch off the parts not tested yet, so a Stage 2 run does not depend
   on the untested obstacle (Stage 3) or IMU (Stage 4) code. */
IMPORT void mission_features (bool obstacles, bool imu);

IMPORT void mission_task (int32_t stacd, void * exinf);

#endif /* MISSION_H */

/*** end of file ***/

