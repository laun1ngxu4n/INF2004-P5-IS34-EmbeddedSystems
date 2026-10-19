/*
 * P5-IS34 bring-up test: GY-511 IMU (Buddy 4)
 *
 * What it measures:
 *   - I2C scan of I2C1 (GP26 SDA, GP27 SCL): which chip is really fitted
 *     (LSM303DLHC = accel 0x19 + mag 0x1E, mag ID bytes "H43")
 *   - Real accelerometer sample rate against the 100 Hz setting, and
 *     overruns (samples lost because they were read too late)
 *   - Time for one status + 6-byte accel read at 100 kHz and 400 kHz
 *     (per-transaction time only, not the whole task; the design budget
 *     is IMU task <= 1 ms per 10 ms, mostly I2C)
 *   - Accelerometer at +/-4 g, the design's range for the >2 g collision
 *     threshold (+/-2 g would saturate exactly at the threshold)
 *   - Pitch = atan2(ax, sqrt(ay^2 + az^2)) noise while stationary
 *     (standard deviation decides the 5 deg hump threshold margin)
 *   - Peak pitch and h = L * sin(peak pitch) when tilting the robot onto
 *     a block of known height (A4, wheelbase method)
 *   - Raw magnetometer heading, to compare with motors on and off
 *   - Collision rule from the design (IMU 6.4: acceleration change above
 *     2 g at +/-4 g): per-sample vector change |a(n) - a(n-1)| and the
 *     deviation of |a| from 1 g, with window peaks, a [shock] line on
 *     every change above 2 g (200 ms hold-off, as in the design), and a
 *     count of saturated samples (any axis at the +/-4 g limit). Use it
 *     for A14: bump the robot 10 times (expect >= 9 shocks), then roll it
 *     over the humps (expect none).
 *   - Whole loop work time (accel + mag transactions + maths, no USB
 *     printing), separate from the single accel transaction time
 *
 * Buttons (Robo Pico onboard, active low):
 *   GP20  toggle I2C speed 100 kHz <-> 400 kHz
 *   GP21  keep the robot flat and still, press: 200-sample calibration
 *         (sets the pitch zero) and resets the peak
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define IMU_I2C             i2c1
#define IMU_SDA_PIN         26u
#define IMU_SCL_PIN         27u
#define BTN_SPEED_PIN       20u
#define BTN_CAL_PIN         21u

#define I2C_SLOW_HZ         100000u
#define I2C_FAST_HZ         400000u
#define I2C_TIMEOUT_US      2000u

/* LSM303DLHC registers */
#define ACC_ADDR_DEFAULT    0x19u
#define ACC_ADDR_ALT        0x18u
#define MAG_ADDR            0x1Eu
#define ACC_CTRL_REG1       0x20u
#define ACC_CTRL_REG4       0x23u
#define ACC_STATUS_REG      0x27u
#define ACC_OUT_X_L         0x28u
#define ACC_AUTO_INC        0x80u
#define MAG_CRA_REG         0x00u
#define MAG_CRB_REG         0x01u
#define MAG_MR_REG          0x02u
#define MAG_OUT_X_H         0x03u
#define MAG_SR_REG          0x09u
#define MAG_IRA_REG         0x0Au

#define ACC_CFG_100HZ_XYZ   0x57u    /* ODR 100 Hz, normal mode, XYZ on */
#define ACC_CFG_BDU_HR_4G   0x98u    /* block update, high resolution, +/-4 g (design range) */
#define MAG_CFG_75HZ        0x18u
#define MAG_CFG_1_3GAUSS    0x20u
#define MAG_CONTINUOUS      0x00u
#define ACC_MG_PER_LSB      2.0f     /* +/-4 g, high resolution, 12-bit */

/* Measure the axle-to-front-support distance and set it (0 = unknown). */
#define WHEELBASE_MM        0.0f

#define CAL_SAMPLES         200u
#define PRINT_PERIOD_MS     100u
#define STATS_PERIOD_MS     2000u
#define BTN_DEBOUNCE_MS     50u
#define RAD_TO_DEG          57.2957795f

#define SHOCK_G             2.0f     /* design collision threshold */
#define SHOCK_HOLDOFF_MS    200u     /* design hold-off */
#define ACC_RAW_LIMIT       2047     /* 12-bit full scale */

static uint8_t g_acc_addr = ACC_ADDR_DEFAULT;

static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static bool reg_write(uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_write_timeout_us(IMU_I2C, addr, buf, 2u, false, I2C_TIMEOUT_US) == 2;
}

static bool reg_read(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint32_t len)
{
    if (i2c_write_timeout_us(IMU_I2C, addr, &reg, 1u, true, I2C_TIMEOUT_US) != 1)
    {
        return false;
    }
    return i2c_read_timeout_us(IMU_I2C, addr, p_buf, len, false, I2C_TIMEOUT_US) == (int)len;
}

static bool i2c_scan(void)
{
    uint8_t addr;
    uint8_t dummy;
    bool acc_found = false;

    printf("[scan] I2C1 devices:");
    for (addr = 0x08u; addr < 0x78u; addr++)
    {
        if (i2c_read_timeout_us(IMU_I2C, addr, &dummy, 1u, false, I2C_TIMEOUT_US) >= 0)
        {
            printf(" 0x%02X", addr);
            if (addr == ACC_ADDR_DEFAULT)
            {
                acc_found = true;
                g_acc_addr = ACC_ADDR_DEFAULT;
            }
            else if ((addr == ACC_ADDR_ALT) && !acc_found)
            {
                acc_found = true;
                g_acc_addr = ACC_ADDR_ALT;
            }
        }
    }
    printf("\n");
    return acc_found;
}

static void imu_init(void)
{
    uint8_t id[3] = {0u, 0u, 0u};
    bool ok = true;

    ok = ok && reg_write(g_acc_addr, ACC_CTRL_REG1, ACC_CFG_100HZ_XYZ);
    ok = ok && reg_write(g_acc_addr, ACC_CTRL_REG4, ACC_CFG_BDU_HR_4G);
    printf("[init] accel at 0x%02X configured (+/-4 g, as in the design): %s\n", g_acc_addr, ok ? "OK" : "FAILED");

    if (reg_read(MAG_ADDR, MAG_IRA_REG, id, 3u))
    {
        printf("[init] mag ID bytes: 0x%02X 0x%02X 0x%02X ('%c%c%c', DLHC expects 'H43')\n",
               id[0], id[1], id[2], id[0], id[1], id[2]);
        ok = reg_write(MAG_ADDR, MAG_CRA_REG, MAG_CFG_75HZ) &&
             reg_write(MAG_ADDR, MAG_CRB_REG, MAG_CFG_1_3GAUSS) &&
             reg_write(MAG_ADDR, MAG_MR_REG, MAG_CONTINUOUS);
        printf("[init] mag configured: %s\n", ok ? "OK" : "FAILED");
    }
    else
    {
        printf("[init] magnetometer at 0x1E not responding\n");
    }
}

static bool button_pressed(uint32_t pin, bool *p_was_down, uint32_t *p_last_ms)
{
    bool down = !gpio_get(pin);
    bool pressed = false;
    uint32_t t = now_ms();

    if (down && !*p_was_down && ((t - *p_last_ms) > BTN_DEBOUNCE_MS))
    {
        pressed = true;
        *p_last_ms = t;
    }
    *p_was_down = down;
    return pressed;
}

static void button_init(uint32_t pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

int main(void)
{
    uint32_t i2c_hz = I2C_FAST_HZ;
    float pitch = 0.0f;
    float pitch_filt = 0.0f;
    float roll = 0.0f;
    float mag_g = 0.0f;
    float heading = 0.0f;
    float pitch_zero = 0.0f;
    float peak_pitch = 0.0f;
    float cal_sum = 0.0f;
    uint32_t cal_left = 0u;
    uint32_t last_read_us = 0u;
    /* stats window */
    uint32_t n = 0u;
    float sum_p = 0.0f;
    float sum_p2 = 0.0f;
    float sum_a = 0.0f;
    float sum_a2 = 0.0f;
    uint32_t overruns = 0u;
    uint32_t read_fail = 0u;
    uint32_t status_fail = 0u;
    uint32_t mag_fail = 0u;
    uint64_t read_sum_us = 0u;
    uint32_t read_max_us = 0u;
    uint32_t next_print = 0u;
    uint32_t next_stats = 0u;
    uint32_t t_window = 0u;
    float ax_prev = 0.0f;
    float ay_prev = 0.0f;
    float az_prev = 0.0f;
    bool have_prev = false;
    float peak_da = 0.0f;
    float peak_dev = 0.0f;
    uint32_t shocks_win = 0u;
    uint32_t shocks_total = 0u;
    uint32_t saturated = 0u;
    uint32_t last_shock_ms = 0u;
    uint32_t work_max_us = 0u;
    bool speed_down = false;
    bool cal_down = false;
    uint32_t speed_last = 0u;
    uint32_t cal_last = 0u;

    stdio_init_all();
    button_init(BTN_SPEED_PIN);
    button_init(BTN_CAL_PIN);

    i2c_init(IMU_I2C, i2c_hz);
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);

    sleep_ms(3000u);
    printf("\n=== P5-IS34 IMU bring-up test (v3) ===\n");

    if (!i2c_scan())
    {
        printf("[scan] no accelerometer at 0x18/0x19 - check wiring (GP26/GP27, 3V3, GND).\n");
    }
    imu_init();
    printf("Keep it flat and still, press GP21 to calibrate. GP20 toggles 100/400 kHz.\n");
    t_window = now_ms();

    for (;;)
    {
        uint32_t t = now_ms();
        uint8_t status = 0u;
        uint8_t raw[6];
        uint64_t t_start = time_us_64();

        /* ---- Accelerometer: only read when new data is ready ------- */
        if (!reg_read(g_acc_addr, ACC_STATUS_REG, &status, 1u))
        {
            status_fail++;
        }
        else if ((status & 0x08u) != 0u)
        {
            if ((status & 0x80u) != 0u)
            {
                overruns++;
            }
            if (reg_read(g_acc_addr, (uint8_t)(ACC_OUT_X_L | ACC_AUTO_INC), raw, 6u))
            {
                int16_t ax_raw = (int16_t)((uint16_t)raw[1] << 8 | raw[0]) >> 4;
                int16_t ay_raw = (int16_t)((uint16_t)raw[3] << 8 | raw[2]) >> 4;
                int16_t az_raw = (int16_t)((uint16_t)raw[5] << 8 | raw[4]) >> 4;
                float ax = (float)ax_raw * ACC_MG_PER_LSB / 1000.0f;
                float ay = (float)ay_raw * ACC_MG_PER_LSB / 1000.0f;
                float az = (float)az_raw * ACC_MG_PER_LSB / 1000.0f;
                float rel;

                last_read_us = (uint32_t)(time_us_64() - t_start);
                read_sum_us += last_read_us;
                if (last_read_us > read_max_us)
                {
                    read_max_us = last_read_us;
                }

                pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
                roll = atan2f(ay, az) * RAD_TO_DEG;
                mag_g = sqrtf(ax * ax + ay * ay + az * az);
                pitch_filt += 0.2f * (pitch - pitch_filt);

                /* Collision rule inputs. */
                if ((ax_raw >= ACC_RAW_LIMIT) || (ax_raw <= -ACC_RAW_LIMIT) ||
                    (ay_raw >= ACC_RAW_LIMIT) || (ay_raw <= -ACC_RAW_LIMIT) ||
                    (az_raw >= ACC_RAW_LIMIT) || (az_raw <= -ACC_RAW_LIMIT))
                {
                    saturated++;
                }
                if (fabsf(mag_g - 1.0f) > peak_dev)
                {
                    peak_dev = fabsf(mag_g - 1.0f);
                }
                if (have_prev)
                {
                    float dx = ax - ax_prev;
                    float dy = ay - ay_prev;
                    float dz = az - az_prev;
                    float da = sqrtf(dx * dx + dy * dy + dz * dz);
                    if (da > peak_da)
                    {
                        peak_da = da;
                    }
                    if ((da > SHOCK_G) && ((t - last_shock_ms) >= SHOCK_HOLDOFF_MS))
                    {
                        last_shock_ms = t;
                        shocks_win++;
                        shocks_total++;
                        printf("[shock] #%lu at %lu ms: change %.2f g between samples, |a| %.2f g%s\n",
                               (unsigned long)shocks_total, (unsigned long)t, (double)da,
                               (double)mag_g, (saturated > 0u) ? " (saturation seen)" : "");
                    }
                }
                ax_prev = ax;
                ay_prev = ay;
                az_prev = az;
                have_prev = true;

                n++;
                sum_p += pitch;
                sum_p2 += pitch * pitch;
                sum_a += mag_g;
                sum_a2 += mag_g * mag_g;

                if (cal_left > 0u)
                {
                    cal_sum += pitch;
                    cal_left--;
                    if (cal_left == 0u)
                    {
                        pitch_zero = cal_sum / (float)CAL_SAMPLES;
                        peak_pitch = 0.0f;
                        printf("[cal] pitch zero = %.2f deg, peak reset\n", (double)pitch_zero);
                    }
                }

                rel = pitch_filt - pitch_zero;
                if (fabsf(rel) > fabsf(peak_pitch))
                {
                    peak_pitch = rel;
                }
            }
            else
            {
                read_fail++;
            }
        }

        /* ---- Magnetometer (75 Hz) --------------------------------- */
        if (!reg_read(MAG_ADDR, MAG_SR_REG, &status, 1u))
        {
            mag_fail++;
        }
        else if ((status & 0x01u) != 0u)
        {
            if (!reg_read(MAG_ADDR, MAG_OUT_X_H, raw, 6u))
            {
                mag_fail++;
            }
            else
            {
                int16_t mx = (int16_t)((uint16_t)raw[0] << 8 | raw[1]);
                int16_t my = (int16_t)((uint16_t)raw[4] << 8 | raw[5]);
                heading = atan2f((float)my, (float)mx) * RAD_TO_DEG;
                if (heading < 0.0f)
                {
                    heading += 360.0f;
                }
            }
        }

        {
            uint32_t work_us = (uint32_t)(time_us_64() - t_start);
            if (work_us > work_max_us)
            {
                work_max_us = work_us;
            }
        }

        /* ---- Output ----------------------------------------------- */
        if ((int32_t)(t - next_print) >= 0)
        {
            printf("pitch=%6.2f filt=%6.2f rel=%6.2f roll=%6.2f |a|=%.3f g "
                   "heading=%5.1f read=%3lu us peak=%6.2f",
                   (double)pitch, (double)pitch_filt, (double)(pitch_filt - pitch_zero),
                   (double)roll, (double)mag_g, (double)heading,
                   (unsigned long)last_read_us, (double)peak_pitch);
            if (WHEELBASE_MM > 0.0f)
            {
                printf(" h_est=%.1f mm",
                       (double)(WHEELBASE_MM * sinf(fabsf(peak_pitch) / RAD_TO_DEG)));
            }
            printf("\n");
            next_print = t + PRINT_PERIOD_MS;
        }

        if ((int32_t)(t - next_stats) >= 0)
        {
            uint32_t win_ms = t - t_window;
            if (n <= 1u)
            {
                printf("[stats] NO FRESH ACCEL SAMPLES in %lu ms (status fails %lu, data fails %lu, "
                       "mag fails %lu) - displayed values are stale\n",
                       (unsigned long)win_ms, (unsigned long)status_fail,
                       (unsigned long)read_fail, (unsigned long)mag_fail);
            }
            else
            {
                float mean_p = sum_p / (float)n;
                float mean_a = sum_a / (float)n;
                float var_p = (sum_p2 / (float)n) - (mean_p * mean_p);
                float var_a = (sum_a2 / (float)n) - (mean_a * mean_a);
                printf("[stats] %lu samples in %lu ms (%.1f Hz) | pitch mean %.2f sd %.3f deg | "
                       "|a| mean %.4f sd %.4f g | read avg %lu max %lu us @ %lu kHz | "
                       "overruns %lu | fails: status %lu data %lu mag %lu\n",
                       (unsigned long)n, (unsigned long)win_ms,
                       (double)(1000.0f * (float)n / (float)win_ms),
                       (double)mean_p, (double)sqrtf((var_p > 0.0f) ? var_p : 0.0f),
                       (double)mean_a, (double)sqrtf((var_a > 0.0f) ? var_a : 0.0f),
                       (unsigned long)(read_sum_us / n), (unsigned long)read_max_us,
                       (unsigned long)(i2c_hz / 1000u),
                       (unsigned long)overruns, (unsigned long)status_fail,
                       (unsigned long)read_fail, (unsigned long)mag_fail);
            }
            printf("[stats] shock: peak change %.2f g, peak | |a|-1g | %.2f g, shocks >%.1f g: "
                   "%lu (total %lu), saturated samples %lu | loop work max %lu us\n",
                   (double)peak_da, (double)peak_dev, (double)SHOCK_G,
                   (unsigned long)shocks_win, (unsigned long)shocks_total,
                   (unsigned long)saturated, (unsigned long)work_max_us);
            peak_da = 0.0f;
            peak_dev = 0.0f;
            shocks_win = 0u;
            saturated = 0u;
            work_max_us = 0u;
            n = 0u;
            sum_p = 0.0f;
            sum_p2 = 0.0f;
            sum_a = 0.0f;
            sum_a2 = 0.0f;
            overruns = 0u;
            read_fail = 0u;
            status_fail = 0u;
            mag_fail = 0u;
            read_sum_us = 0u;
            read_max_us = 0u;
            t_window = t;
            next_stats = t + STATS_PERIOD_MS;
        }

        /* ---- Buttons ---------------------------------------------- */
        if (button_pressed(BTN_SPEED_PIN, &speed_down, &speed_last))
        {
            i2c_hz = (i2c_hz == I2C_FAST_HZ) ? I2C_SLOW_HZ : I2C_FAST_HZ;
            (void)i2c_set_baudrate(IMU_I2C, i2c_hz);
            printf("[i2c] now %lu kHz (statistics window restarted)\n", (unsigned long)(i2c_hz / 1000u));
            n = 0u; sum_p = 0.0f; sum_p2 = 0.0f; sum_a = 0.0f; sum_a2 = 0.0f;
            overruns = 0u; read_fail = 0u; status_fail = 0u; mag_fail = 0u;
            read_sum_us = 0u; read_max_us = 0u;
            peak_da = 0.0f; peak_dev = 0.0f; shocks_win = 0u; saturated = 0u; work_max_us = 0u;
            t_window = now_ms(); next_stats = t_window + STATS_PERIOD_MS;
        }
        if (button_pressed(BTN_CAL_PIN, &cal_down, &cal_last))
        {
            cal_sum = 0.0f;
            cal_left = CAL_SAMPLES;
            printf("[cal] collecting %u samples, keep still...\n", CAL_SAMPLES);
        }

        sleep_us(500u);
    }
}
