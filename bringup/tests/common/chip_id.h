/*
 * chip_id.h - name the accel/mag chip from its ID registers (i2c_diag).
 *
 * 25 Oct review finding 20: i2c_diag recognised WHO_AM_I 0x49 but only
 * flagged "IMU found" for 0x19/0x1E, so our LSM303D at 0x1D ended with a
 * "different chip?" verdict. The classification now also says whether the
 * chip is one the bring-up firmware supports. Tested on the host by
 * host_tests/test_chip_id.c.
 *
 * Register order in ok[]/v[]: 0x00, 0x0D, 0x0F, 0x0A, 0x4F.
 */
#ifndef CHIP_ID_H
#define CHIP_ID_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CHIP_UNKNOWN = 0,
    CHIP_LSM303D,          /* accel + mag at one address; supported */
    CHIP_OTHER_KNOWN       /* recognised but not supported by our firmware */
} chip_kind_t;

typedef struct
{
    chip_kind_t kind;
    const char *p_name;
} chip_match_t;

static inline chip_match_t chip_from_ids(const bool ok[5], const uint8_t v[5])
{
    chip_match_t m = {CHIP_UNKNOWN, "no match in the table - send these bytes and the chip marking"};

    if (ok[2] && (v[2] == 0x49u))
    {
        m.kind = CHIP_LSM303D;
        m.p_name = "LSM303D (accel + mag, one address) or LSM9DS0 accel/mag";
    }
    else if (ok[2] && (v[2] == 0x41u))
    {
        m.kind = CHIP_OTHER_KNOWN;
        m.p_name = "LSM303C accelerometer (its mag should be at 0x1E)";
    }
    else if (ok[2] && (v[2] == 0x3Du))
    {
        m.kind = CHIP_OTHER_KNOWN;
        m.p_name = "LIS3MDL / LSM9DS1 / LSM303C magnetometer";
    }
    else if (ok[2] && (v[2] == 0x33u))
    {
        m.kind = CHIP_OTHER_KNOWN;
        m.p_name = "LIS3DH / LSM303AGR accelerometer";
    }
    else if (ok[0] && (v[0] == 0xE5u))
    {
        m.kind = CHIP_OTHER_KNOWN;
        m.p_name = "ADXL345 accelerometer (accel only, no mag)";
    }
    else if (ok[1] && (v[1] == 0x1Au))
    {
        m.kind = CHIP_OTHER_KNOWN;
        m.p_name = "MMA8451Q accelerometer (accel only)";
    }
    else if (ok[1] && (v[1] == 0x2Au))
    {
        m.kind = CHIP_OTHER_KNOWN;
        m.p_name = "MMA8452Q accelerometer (accel only)";
    }
    else if (ok[1] && (v[1] == 0x4Au))
    {
        m.kind = CHIP_OTHER_KNOWN;
        m.p_name = "MMA8653FC accelerometer (accel only)";
    }
    else if (ok[1] && (v[1] == 0xC7u))
    {
        m.kind = CHIP_OTHER_KNOWN;
        m.p_name = "FXOS8700 (accel + mag)";
    }
    else if (ok[2] && (v[2] == 0x55u))
    {
        m.kind = CHIP_OTHER_KNOWN;
        m.p_name = "MMA7455 accelerometer (accel only)";
    }
    else
    {
        /* keep the default */
    }
    return m;
}

#endif /* CHIP_ID_H */
