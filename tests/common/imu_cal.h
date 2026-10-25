/*
 * imu_cal.h - flat/still calibration for the IMU bring-up test.
 *
 * 25 Oct review finding 16: when the Z offset was rejected, imu_test still
 * changed pitch zero, the filter and the peak and printed a normal summary.
 * Now the whole candidate is computed and checked first; the caller commits
 * all fields only when this returns IMU_CAL_OK and keeps everything as it
 * was otherwise. Tested on the host by host_tests/test_imu_cal.c.
 *
 * Inputs are the mean accel X/Y/Z (g) over the calibration window, the pitch
 * standard deviation (deg) and the number of samples.
 */
#ifndef IMU_CAL_H
#define IMU_CAL_H

#include <math.h>
#include <stdint.h>

#define IMU_CAL_RAD_TO_DEG  57.29578f

typedef enum
{
    IMU_CAL_OK = 0,
    IMU_CAL_NO_DATA,      /* no samples */
    IMU_CAL_MOVED,        /* pitch sd above the limit */
    IMU_CAL_NOT_FLAT,     /* X/Y say the board is tilted more than max_tilt_deg */
    IMU_CAL_BAD_OFFSET    /* implied Z offset outside +/- max_off_g */
} imu_cal_status_t;

typedef struct
{
    float z_off_g;
    float pitch_zero_deg;
} imu_cal_t;

static inline const char *imu_cal_reason(imu_cal_status_t s)
{
    switch (s)
    {
        case IMU_CAL_OK:         return "ok";
        case IMU_CAL_NO_DATA:    return "no samples";
        case IMU_CAL_MOVED:      return "moved during calibration";
        case IMU_CAL_NOT_FLAT:   return "not flat";
        case IMU_CAL_BAD_OFFSET: return "Z offset out of range (tilted, moving or wrong axis?)";
        default:                 return "?";
    }
}

static inline imu_cal_status_t imu_cal_compute(float mx, float my, float mz, float pitch_sd_deg,
                                               uint32_t n, float max_sd_deg, float max_tilt_deg,
                                               float max_off_g, imu_cal_t *p_out)
{
    float horiz;
    float h2;
    float new_off;
    float zc;

    if (n == 0u)
    {
        return IMU_CAL_NO_DATA;
    }
    if (!(pitch_sd_deg <= max_sd_deg))           /* also rejects NaN */
    {
        return IMU_CAL_MOVED;
    }
    horiz = sqrtf((mx * mx) + (my * my));
    if (!(horiz <= sinf(max_tilt_deg / IMU_CAL_RAD_TO_DEG)))
    {
        return IMU_CAL_NOT_FLAT;
    }
    /* Assume X/Y read correctly and the scale is right (bench flip test:
     * +1 g and -1 g readings average 1.01 g), so the true |az| is
     * sqrt(1 - ax^2 - ay^2). */
    h2 = 1.0f - (mx * mx) - (my * my);
    new_off = mz - ((mz >= 0.0f) ? sqrtf(h2) : -sqrtf(h2));
    if (!(fabsf(new_off) <= max_off_g))
    {
        return IMU_CAL_BAD_OFFSET;
    }
    zc = mz - new_off;
    p_out->z_off_g = new_off;
    p_out->pitch_zero_deg = atan2f(mx, sqrtf((my * my) + (zc * zc))) * IMU_CAL_RAD_TO_DEG;
    return IMU_CAL_OK;
}

#endif /* IMU_CAL_H */
