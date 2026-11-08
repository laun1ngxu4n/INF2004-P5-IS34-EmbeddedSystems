/**
 * @file    imu_i2c.h
 *  I2C1 for the IMU (imu_i2c.c).  Private to subsystem 4.
 */

#ifndef IMU_I2C_H
#define IMU_I2C_H

#include "../app.h"

/** Sets up I2C1 for the IMU (pins, 100 kHz). */
IMPORT void imu_i2c_init (void);
/** Reads count (<= 8) registers from reg up; E_OK or an error. */
IMPORT ER imu_i2c_read (uint8_t addr, uint8_t reg, uint8_t * p_buf,
                        int32_t count);
/** Writes one register of an I2C device; E_OK or an error. */
IMPORT ER imu_i2c_write (uint8_t addr, uint8_t reg, uint8_t val);

#endif /* IMU_I2C_H */

/*** end of file ***/

