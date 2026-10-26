/*
 *	subsystem4.h
 *	Subsystem 4 -- Safety / Fault Monitor + IMU / Terrain Monitor
 *	(Buddy 4, Muhammad Haziq).
 *
 *	This header is subsystem 4's PUBLIC interface: only what other
 *	subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *	files.  Other subsystems include it as "../subsystem4/subsystem4.h".
 *
 *	Stage 0: Safety runs every 20 ms at the highest application priority;
 *	the IMU task runs every 10 ms.  Neither touches hardware yet.  The IMU
 *	is an LSM303D at 0x1D on I2C1 (GP26 SDA, GP27 SCL), use 400 kHz.
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

/* Private to subsystem 4, shared between its own .c files */
IMPORT ID	tid_safety;
IMPORT ID	cyc_safety;
IMPORT INT	tmg_safety;
IMPORT ID	tid_imu;
IMPORT ID	cyc_imu;
IMPORT INT	tmg_imu;
IMPORT void	safety_task(INT stacd, void *exinf);
IMPORT void	imu_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM4_H */
