/**
 * @file    subsystem4.h
 *  Subsystem 4 -- Safety / Fault Monitor + IMU / Terrain Monitor
 *  (Buddy 4, Muhammad Haziq).
 *
 *  This header is subsystem 4's PUBLIC interface: only what other
 *  subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *  files.  Other subsystems include it as "../subsystem4/subsystem4.h".
 *
 *  Stage 0: Safety runs every 20 ms at the highest application priority;
 *  the IMU task runs every 10 ms.  Neither touches hardware yet.  The IMU
 *  is an LSM303D at 0x1D on I2C1 (GP26 SDA, GP27 SCL), use 400 kHz.
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

/*
 * Stage 4: IMU (LSM303D at 0x1D on I2C1, GP26/GP27, 400 kHz, +/-4 g).
 * Ramp height (TEAM.md 7, ramp): height = sum of (distance driven x
 * sin(pitch)), pitch relative to the level pitch measured at START
 * (imu_calibrate).  Integration needs the encoders, so acceleration
 * spikes (the bench saw up to 6 deg of false pitch) are smoothed by a
 * low-pass filter and only count while the robot moves.  When the robot is
 * level again after a climb, the peak is reported (telemetry_event) and kept
 * for telemetry.  L x sin(peak pitch) (L = S4_L_MM) is reported beside it.
 * Safety (collision, stall) is not in the essentials plan: safety_task
 * stays the Stage 0 timing stub.
 */
#define S4_L_MM 80 /* axle to front support, measured 6 Nov */
/* One-wheel hump while driving: T x sin(side tilt) read about 1.6 times
 * the real height (9 Nov S4-R01, 5 runs: 16.6 / 17.2 / 17.6 mm for humps of
 * about 10 mm, 40.8 / 37.0 for about 25 mm), the same factor for both, while
 * standing still on a block it read right.  The wheel climbing jolts the
 * robot sideways, which the accelerometer adds to the tilt.  The reported
 * height is scaled by this (least squares over the 5 runs: 0.63); the live
 * one-wheel reading (tilt view) stays unscaled.  Re-check with S4-R01. */
#define S4_ONE_WHEEL_GAIN_PCT 63

/* Private to subsystem 4, shared between its own .c files */
IMPORT ID      gh_tid_safety;
IMPORT ID      gh_cyc_safety;
IMPORT int32_t g_tmg_safety;
IMPORT ID      gh_tid_imu;
IMPORT ID      gh_cyc_imu;
IMPORT int32_t g_tmg_imu;

#include "imu.h"
#include "safety.h"

#endif /* SUBSYSTEM4_H */

/*** end of file ***/
