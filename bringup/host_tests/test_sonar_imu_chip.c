/* Host test: shared sonar classifier (finding 13), IMU calibration
 * all-or-nothing rule (finding 16) and i2c_diag chip naming (finding 20). */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "../tests/common/sonar_class.h"
#include "../tests/common/imu_cal.h"
#include "../tests/common/chip_id.h"

static int g_fail;
static int g_checks;

static void check(bool ok, const char *what)
{
    g_checks++;
    if (!ok) { g_fail++; printf("FAIL %s\n", what); }
}

int main(void)
{
    /* --- 13: boundaries, same answer for parts_check and ultrasonic_test. */
    check(sonar_classify_width(0u) == SONAR_WIDTH_TOO_SHORT, "0 us too short");
    check(sonar_classify_width(99u) == SONAR_WIDTH_TOO_SHORT, "99 us too short");
    check(sonar_classify_width(100u) == SONAR_WIDTH_VALID, "100 us valid");
    check(sonar_classify_width(25000u) == SONAR_WIDTH_VALID, "25,000 us valid");
    check(sonar_classify_width(25001u) == SONAR_WIDTH_LONG, "25,001 us long");
    check(sonar_classify_width(30000u) == SONAR_WIDTH_LONG, "30,000 us long (parts_check said OK in v3.1)");

    /* --- 16: IMU calibration. Bench case from 22 Oct: sensor upside down,
     * |a| 0.882 g raw, pitch about 2.4 deg -> offset +0.118 g. */
    {
        imu_cal_t c = {123.0f, 456.0f};
        float mx = 0.0376f;
        float my = 0.0230f;
        float mz = -0.8803f;
        imu_cal_status_t s = imu_cal_compute(mx, my, mz, 0.09f, 200u, 0.30f, 10.0f, 0.30f, &c);
        check(s == IMU_CAL_OK, "bench flat case accepted");
        check(fabsf(c.z_off_g - 0.119f) < 0.005f, "bench offset about +0.118 g");
        check((c.pitch_zero_deg > 1.5f) && (c.pitch_zero_deg < 3.0f), "bench pitch zero about 2.2 deg");
    }
    {
        imu_cal_t c = {123.0f, 456.0f};
        check(imu_cal_compute(0.03f, 0.02f, -0.88f, 0.50f, 200u, 0.30f, 10.0f, 0.30f, &c) == IMU_CAL_MOVED,
              "moved: rejected");
        check((c.z_off_g == 123.0f) && (c.pitch_zero_deg == 456.0f), "moved: nothing written");
        check(imu_cal_compute(0.50f, 0.0f, -0.75f, 0.05f, 200u, 0.30f, 10.0f, 0.30f, &c) == IMU_CAL_NOT_FLAT,
              "30 deg tilt: rejected");
        check(imu_cal_compute(0.02f, 0.02f, -0.50f, 0.05f, 200u, 0.30f, 10.0f, 0.30f, &c) == IMU_CAL_BAD_OFFSET,
              "implied 0.5 g offset: rejected (v3.1 still reset pitch zero)");
        check((c.z_off_g == 123.0f) && (c.pitch_zero_deg == 456.0f), "rejections leave the old values");
        check(imu_cal_compute(0.0f, 0.0f, -1.0f, 0.05f, 0u, 0.30f, 10.0f, 0.30f, &c) == IMU_CAL_NO_DATA,
              "no samples: rejected");
        check(imu_cal_compute(0.0f, 0.0f, -1.0f, NAN, 200u, 0.30f, 10.0f, 0.30f, &c) == IMU_CAL_MOVED,
              "NaN sd: rejected");
    }

    /* --- 20: chip naming. Our 22 Oct LSM303D bytes: [0x00]=00 [0x0D]=FE
     * [0x0F]=49 [0x0A]=D4 [0x4F]=00. */
    {
        bool ok[5] = {true, true, true, true, true};
        uint8_t v[5] = {0x00u, 0xFEu, 0x49u, 0xD4u, 0x00u};
        chip_match_t m = chip_from_ids(ok, v);
        check(m.kind == CHIP_LSM303D, "our board's bytes -> LSM303D (supported)");
        v[2] = 0x33u;
        check(chip_from_ids(ok, v).kind == CHIP_OTHER_KNOWN, "0x33 -> other known chip");
        v[2] = 0x12u;
        v[1] = 0x00u;
        check(chip_from_ids(ok, v).kind == CHIP_UNKNOWN, "unknown bytes -> unknown");
        ok[2] = false;
        v[2] = 0x49u;
        check(chip_from_ids(ok, v).kind != CHIP_LSM303D, "failed read is not an ID match");
    }

    printf("sonar/imu_cal/chip_id: %s (%d checks, %d failures)\n", g_fail ? "FAILED" : "all passed",
           g_checks, g_fail);
    return g_fail ? 1 : 0;
}
