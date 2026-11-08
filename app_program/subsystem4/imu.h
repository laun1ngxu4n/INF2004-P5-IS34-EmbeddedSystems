/**
 * @file    imu.h
 *  IMU / Terrain Monitor (imu.c): tilt, climbed height, ramp reports.
 *  Part of subsystem 4's interface (subsystem4.h includes it).
 */

#ifndef IMU_H
#define IMU_H

#include "../app.h"

IMPORT bool    imu_ok (void);        /* chip found and configured */
IMPORT void    imu_calibrate (void); /* robot level and still: 1 s */
IMPORT bool    imu_calibrated (void);
IMPORT int32_t imu_height_mm10 (void);    /* current height, 0.1 mm */
IMPORT int32_t imu_pitch_mdeg_sin (void); /* sin(pitch) x 1000, relative */
IMPORT int32_t imu_roll_sin_x1000 (void); /* sin(side tilt) x 1000, relative */
IMPORT int32_t imu_one_wheel_mm10 (void); /* one wheel up: T x sin(side tilt) */
IMPORT int32_t imu_ramp_peak_mm10 (void); /* last reported ramp peak, 0.1 mm */
IMPORT uint32_t imu_ramp_reports (void);  /* ramps reported since boot */
IMPORT uint32_t imu_jolts (void); /* samples skipped as jolts since boot */

IMPORT void imu_task (int32_t stacd, void * exinf);

#endif /* IMU_H */

/*** end of file ***/

