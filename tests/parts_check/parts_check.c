/*
 * P5-IS34 parts check: every part on the robot in one image (all buddies)
 *
 * A quick go/no-go before the detailed per-buddy tests. It answers "is it
 * wired, powered and alive, and which way round is it?", not "does it
 * meet the requirement". Use the per-buddy tests for numbers.
 *
 * At start-up (and again on a GP21 press) it checks, in order:
 *   1. IMU on I2C1 (GP26/27): accel 0x19, mag 0x1E "H43", |a| ~ 1 g flat
 *   2. Ultrasonic (TRIG GP6, ECHO GP7): 5 pings; point it at a wall
 *   3. Servo (GP14): left 45, right 45, centre             -> LOOK
 *   4. NeoPixels (GP18): red, green, blue, off             -> LOOK
 *   5. Buzzer (GP22): two beeps (check the mute switch)    -> LISTEN
 *   6. Battery sense (GP28 / ADC2): mean and noise of 32 samples
 * Then a live line every 500 ms: IR L/C/B levels and edge counts,
 * encoder counts, buttons, battery. Hand-turn each wheel forward and move
 * each IR sensor over black/white while watching it.
 *
 * GP20 press: motor check. LIFT THE WHEELS FIRST. Each motor runs 0.6 s
 * at 50 % forward then reverse; it reports which encoder moved and with
 * which sign (catches swapped motors, swapped encoders, reversed
 * polarity), and the lowest battery reading under load (sag, A12).
 * Holding GP21 during the motor check stops the motors at once.
 *
 * All GPIO interrupts (encoders, IR, echo) go through ONE dispatcher, as
 * the integrated firmware must (the SDK allows one callback per core).
 *
 * Not evidence for any NFR: no timing or accuracy claims are made here.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"

#include "notify_hw.h"

/* ---- Pins (Week 6 pin table) --------------------------------------- */
#define IR_LEFT_PIN     0u
#define IR_CENTER_PIN   1u
#define IR_BARCODE_PIN  2u
#define ENC_L_A_PIN     4u
#define ENC_L_B_PIN     5u
#define US_TRIG_PIN     6u
#define US_ECHO_PIN     7u
#define MOTOR_L_A_PIN   8u
#define MOTOR_L_B_PIN   9u
#define MOTOR_R_A_PIN   10u
#define MOTOR_R_B_PIN   11u
#define SERVO_PIN       14u
#define ENC_R_A_PIN     16u
#define ENC_R_B_PIN     17u
#define BTN_START_PIN   20u
#define BTN_STOP_PIN    21u
#define IMU_SDA_PIN     26u
#define IMU_SCL_PIN     27u
#define VBAT_PIN        28u
#define VBAT_ADC_INPUT  2u

/* Set to the measured divider ratio (V_battery / V_pin). 0 = unknown. */
#define VBAT_DIVIDER    0.0f

#define IMU_I2C         i2c1
#define ACC_ADDR        0x19u
#define MAG_ADDR        0x1Eu

#define PWM_WRAP        12499u
#define MOTOR_TEST_DUTY 50u
#define MOTOR_TEST_MS   600u
#define MOVED_COUNTS    20

#define SERVO_CENTRE_US 1500u
#define SERVO_45_US     500u

#define WHEEL_L         0u
#define WHEEL_R         1u
#define QDEC_INVALID    2

typedef enum
{
    R_OK = 0,
    R_CHECK,
    R_FAIL,
    R_LOOK
} result_t;

static const char *const g_result_name[] = {"OK", "CHECK", "FAIL", "LOOK/LISTEN"};

static const int8_t g_qdec[16] =
{
     0, +1, -1, QDEC_INVALID,
    -1,  0, QDEC_INVALID, +1,
    +1, QDEC_INVALID,  0, -1,
    QDEC_INVALID, -1, +1,  0
};

/* ---- ISR-shared state ---------------------------------------------- */
static volatile int32_t  g_enc_count[2];
static volatile uint32_t g_enc_invalid[2];
static uint8_t           g_enc_prev[2];
static volatile uint32_t g_ir_edges[3];
static volatile uint32_t g_echo_rise_us;
static volatile uint32_t g_echo_fall_us;
static volatile bool     g_echo_rise_seen;
static volatile bool     g_echo_fall_seen;

static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static uint8_t enc_read_ab(uint32_t wheel)
{
    uint32_t a = (wheel == WHEEL_L) ? ENC_L_A_PIN : ENC_R_A_PIN;
    uint32_t b = (wheel == WHEEL_L) ? ENC_L_B_PIN : ENC_R_B_PIN;
    return (uint8_t)((gpio_get(a) ? 2u : 0u) | (gpio_get(b) ? 1u : 0u));
}

static void enc_edge(uint32_t wheel)
{
    uint8_t ab = enc_read_ab(wheel);
    int8_t step = g_qdec[(g_enc_prev[wheel] << 2) | ab];

    if (step == QDEC_INVALID)
    {
        g_enc_invalid[wheel]++;
    }
    else
    {
        g_enc_count[wheel] += step;
    }
    g_enc_prev[wheel] = ab;
}

/* The one GPIO dispatcher for every interrupt pin. */
static void gpio_dispatch(uint gpio, uint32_t events)
{
    uint32_t t = timer_hw->timerawl;

    switch (gpio)
    {
        case ENC_L_A_PIN:
        case ENC_L_B_PIN:
            enc_edge(WHEEL_L);
            break;
        case ENC_R_A_PIN:
        case ENC_R_B_PIN:
            enc_edge(WHEEL_R);
            break;
        case IR_LEFT_PIN:
        case IR_CENTER_PIN:
        case IR_BARCODE_PIN:
            g_ir_edges[gpio]++;   /* pins 0..2 map to index 0..2 */
            break;
        case US_ECHO_PIN:
            if ((events & GPIO_IRQ_EDGE_RISE) != 0u)
            {
                g_echo_rise_us = t;
                g_echo_rise_seen = true;
            }
            if ((events & GPIO_IRQ_EDGE_FALL) != 0u)
            {
                g_echo_fall_us = t;
                g_echo_fall_seen = true;
            }
            break;
        default:
            break;
    }
}

static void input_init(uint32_t pin, bool pull_up)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    if (pull_up)
    {
        gpio_pull_up(pin);
    }
    else
    {
        gpio_disable_pulls(pin);
    }
}

static void hw_init(void)
{
    const uint32_t both = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    uint32_t slice;

    input_init(BTN_START_PIN, true);
    input_init(BTN_STOP_PIN, true);
    input_init(IR_LEFT_PIN, false);
    input_init(IR_CENTER_PIN, false);
    input_init(IR_BARCODE_PIN, false);
    input_init(ENC_L_A_PIN, true);
    input_init(ENC_L_B_PIN, true);
    input_init(ENC_R_A_PIN, true);
    input_init(ENC_R_B_PIN, true);
    g_enc_prev[WHEEL_L] = enc_read_ab(WHEEL_L);
    g_enc_prev[WHEEL_R] = enc_read_ab(WHEEL_R);

    gpio_init(US_TRIG_PIN);
    gpio_set_dir(US_TRIG_PIN, GPIO_OUT);
    gpio_put(US_TRIG_PIN, 0);
    gpio_init(US_ECHO_PIN);
    gpio_set_dir(US_ECHO_PIN, GPIO_IN);
    gpio_pull_down(US_ECHO_PIN);

    gpio_set_irq_enabled_with_callback(ENC_L_A_PIN, both, true, gpio_dispatch);
    gpio_set_irq_enabled(ENC_L_B_PIN, both, true);
    gpio_set_irq_enabled(ENC_R_A_PIN, both, true);
    gpio_set_irq_enabled(ENC_R_B_PIN, both, true);
    gpio_set_irq_enabled(IR_LEFT_PIN, both, true);
    gpio_set_irq_enabled(IR_CENTER_PIN, both, true);
    gpio_set_irq_enabled(IR_BARCODE_PIN, both, true);
    gpio_set_irq_enabled(US_ECHO_PIN, both, true);

    /* Motors: both pins of each motor on PWM, all low = stopped. */
    gpio_set_function(MOTOR_L_A_PIN, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_L_B_PIN, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_R_A_PIN, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_R_B_PIN, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(MOTOR_L_A_PIN);
    pwm_set_wrap(slice, PWM_WRAP);
    pwm_set_both_levels(slice, 0u, 0u);
    pwm_set_enabled(slice, true);
    slice = pwm_gpio_to_slice_num(MOTOR_R_A_PIN);
    pwm_set_wrap(slice, PWM_WRAP);
    pwm_set_both_levels(slice, 0u, 0u);
    pwm_set_enabled(slice, true);

    /* Servo: 1 MHz tick, 20 ms period. */
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_set_clkdiv(slice, (float)clock_get_hz(clk_sys) / 1000000.0f);
    pwm_set_wrap(slice, 19999u);
    pwm_set_gpio_level(SERVO_PIN, SERVO_CENTRE_US);
    pwm_set_enabled(slice, true);

    i2c_init(IMU_I2C, 400000u);
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);

    adc_init();
    adc_gpio_init(VBAT_PIN);

    (void)neopixel_init();
    buzzer_init();
}

static void motors_off(void)
{
    pwm_set_gpio_level(MOTOR_L_A_PIN, 0u);
    pwm_set_gpio_level(MOTOR_L_B_PIN, 0u);
    pwm_set_gpio_level(MOTOR_R_A_PIN, 0u);
    pwm_set_gpio_level(MOTOR_R_B_PIN, 0u);
}

/* forward: A = PWM, B = low; reverse: A = low, B = PWM. */
static void motor_drive(uint32_t wheel, bool forward, uint32_t duty)
{
    uint32_t a = (wheel == WHEEL_L) ? MOTOR_L_A_PIN : MOTOR_R_A_PIN;
    uint32_t b = (wheel == WHEEL_L) ? MOTOR_L_B_PIN : MOTOR_R_B_PIN;
    uint32_t level = (PWM_WRAP + 1u) * duty / 100u;

    pwm_set_gpio_level(a, forward ? level : 0u);
    pwm_set_gpio_level(b, forward ? 0u : level);
}

static float vbat_pin_volts(void)
{
    adc_select_input(VBAT_ADC_INPUT);
    return (float)adc_read() * 3.3f / 4095.0f;
}

static void enc_snapshot(int32_t out[2])
{
    uint32_t irq = save_and_disable_interrupts();
    out[WHEEL_L] = g_enc_count[WHEEL_L];
    out[WHEEL_R] = g_enc_count[WHEEL_R];
    restore_interrupts(irq);
}

/* ---- Individual checks --------------------------------------------- */
static bool reg_write(uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_write_timeout_us(IMU_I2C, addr, buf, 2u, false, 2000u) == 2;
}

static bool reg_read(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint32_t len)
{
    if (i2c_write_timeout_us(IMU_I2C, addr, &reg, 1u, true, 2000u) != 1)
    {
        return false;
    }
    return i2c_read_timeout_us(IMU_I2C, addr, p_buf, len, false, 2000u) == (int)len;
}

static result_t check_imu(void)
{
    uint8_t id[3] = {0u, 0u, 0u};
    uint8_t raw[6];
    bool acc_ok;
    bool mag_ok;
    float sum_g = 0.0f;
    uint32_t n = 0u;
    uint32_t i;

    acc_ok = reg_write(ACC_ADDR, 0x20u, 0x57u) && reg_write(ACC_ADDR, 0x23u, 0x98u);
    mag_ok = reg_read(MAG_ADDR, 0x0Au, id, 3u);
    printf("[imu] accel 0x19: %s | mag 0x1E: %s, ID '%c%c%c' (expect 'H43')\n",
           acc_ok ? "responds" : "NO ANSWER", mag_ok ? "responds" : "NO ANSWER",
           mag_ok ? id[0] : '?', mag_ok ? id[1] : '?', mag_ok ? id[2] : '?');
    if (!acc_ok)
    {
        printf("[imu] check GP26 SDA / GP27 SCL, 3V3, GND (and Grove 5 is not also used)\n");
        return R_FAIL;
    }
    sleep_ms(50u);
    for (i = 0u; i < 20u; i++)
    {
        if (reg_read(ACC_ADDR, 0x28u | 0x80u, raw, 6u))
        {
            float ax = (float)((int16_t)((uint16_t)raw[1] << 8 | raw[0]) >> 4) * 0.002f;
            float ay = (float)((int16_t)((uint16_t)raw[3] << 8 | raw[2]) >> 4) * 0.002f;
            float az = (float)((int16_t)((uint16_t)raw[5] << 8 | raw[4]) >> 4) * 0.002f;
            sum_g += sqrtf(ax * ax + ay * ay + az * az);
            n++;
        }
        sleep_ms(10u);
    }
    if (n == 0u)
    {
        printf("[imu] accel data reads all failed\n");
        return R_FAIL;
    }
    printf("[imu] |a| = %.3f g over %lu reads (expect ~1.0 g when still)\n",
           (double)(sum_g / (float)n), (unsigned long)n);
    if ((sum_g / (float)n < 0.85f) || (sum_g / (float)n > 1.15f) || !mag_ok ||
        (id[0] != 'H') || (id[1] != '4') || (id[2] != '3'))
    {
        return R_CHECK;
    }
    return R_OK;
}

static int ping_once(float *p_mm)
{
    uint32_t t_trig;

    if (gpio_get(US_ECHO_PIN))
    {
        return -1;      /* stuck high */
    }
    g_echo_rise_seen = false;
    g_echo_fall_seen = false;
    gpio_put(US_TRIG_PIN, 1);
    busy_wait_us_32(10u);
    gpio_put(US_TRIG_PIN, 0);
    t_trig = timer_hw->timerawl;
    while (!g_echo_rise_seen)
    {
        if ((timer_hw->timerawl - t_trig) > 5000u)
        {
            return -2;  /* no rise */
        }
    }
    while (!g_echo_fall_seen)
    {
        if ((timer_hw->timerawl - g_echo_rise_us) > 40000u)
        {
            return -3;  /* no fall */
        }
    }
    *p_mm = (float)(g_echo_fall_us - g_echo_rise_us) * 0.3483f / 2.0f;
    return ((g_echo_fall_us - g_echo_rise_us) < 100u) ? -4 : 0;
}

static result_t check_ultrasonic(void)
{
    static const char *const why[] = {"", "stuck high", "no rise", "no fall", "too short"};
    uint32_t valid = 0u;
    uint32_t i;

    for (i = 0u; i < 5u; i++)
    {
        float mm = 0.0f;
        int r = ping_once(&mm);
        if (r == 0)
        {
            valid++;
            printf("[sonar] ping %lu: %.0f mm\n", (unsigned long)(i + 1u), (double)mm);
        }
        else
        {
            printf("[sonar] ping %lu: %s\n", (unsigned long)(i + 1u), why[-r]);
        }
        sleep_ms(60u);
    }
    if (valid == 0u)
    {
        printf("[sonar] no echoes: check jumper wiring (not Grove), supply, and point it at a wall\n");
        return R_FAIL;
    }
    return (valid >= 4u) ? R_OK : R_CHECK;
}

static result_t check_servo(void)
{
    printf("[servo] left 45 ... right 45 ... centre (nominal pulses) - did it move both ways?\n");
    pwm_set_gpio_level(SERVO_PIN, SERVO_CENTRE_US - SERVO_45_US);
    sleep_ms(600u);
    pwm_set_gpio_level(SERVO_PIN, SERVO_CENTRE_US + SERVO_45_US);
    sleep_ms(600u);
    pwm_set_gpio_level(SERVO_PIN, SERVO_CENTRE_US);
    sleep_ms(400u);
    return R_LOOK;
}

static result_t check_leds(void)
{
    printf("[led] red ... green ... blue ... off - both LEDs, right colours?\n");
    neopixel_set_all(48u, 0u, 0u);
    sleep_ms(500u);
    neopixel_set_all(0u, 48u, 0u);
    sleep_ms(500u);
    neopixel_set_all(0u, 0u, 48u);
    sleep_ms(500u);
    neopixel_set_all(0u, 0u, 0u);
    return R_LOOK;
}

static result_t check_buzzer(void)
{
    printf("[buzzer] two beeps (silent? check the buzzer mute switch on the board)\n");
    buzzer_tone(2000u);
    sleep_ms(150u);
    buzzer_tone(0u);
    sleep_ms(150u);
    buzzer_tone(2000u);
    sleep_ms(150u);
    buzzer_tone(0u);
    return R_LOOK;
}

static result_t check_battery(float *p_mean)
{
    float sum = 0.0f;
    float sum2 = 0.0f;
    float mean;
    float sd;
    uint32_t i;

    for (i = 0u; i < 32u; i++)
    {
        float v = vbat_pin_volts();
        sum += v;
        sum2 += v * v;
        sleep_us(200u);
    }
    mean = sum / 32.0f;
    sd = sqrtf(fmaxf(0.0f, (sum2 / 32.0f) - (mean * mean)));
    *p_mean = mean;
    if (VBAT_DIVIDER > 0.0f)
    {
        printf("[vbat] GP28 pin %.3f V (sd %.3f) -> battery %.2f V with divider %.2f\n",
               (double)mean, (double)sd, (double)(mean * VBAT_DIVIDER), (double)VBAT_DIVIDER);
    }
    else
    {
        printf("[vbat] GP28 pin %.3f V (sd %.3f). Divider ratio unknown: measure the battery with a "
               "meter and set VBAT_DIVIDER = V_batt / V_pin\n", (double)mean, (double)sd);
    }
    if ((mean < 0.05f) || (sd > 0.05f))
    {
        printf("[vbat] near 0 V or noisy: GP28 may not be connected to the battery on this board "
               "(check the jumper / Grove 7)\n");
        return R_CHECK;
    }
    return R_OK;
}

static void run_part_checks(void)
{
    result_t r[6];
    float vbat = 0.0f;
    static const char *const names[6] = {"IMU", "Ultrasonic", "Servo", "NeoPixels", "Buzzer",
                                          "Battery sense"};
    uint32_t i;

    printf("\n---- part checks ----\n");
    r[0] = check_imu();
    r[1] = check_ultrasonic();
    r[2] = check_servo();
    r[3] = check_leds();
    r[4] = check_buzzer();
    r[5] = check_battery(&vbat);
    printf("---- summary ----\n");
    for (i = 0u; i < 6u; i++)
    {
        printf("  %-14s %s\n", names[i], g_result_name[r[i]]);
    }
    printf("Live view below. Hand-turn each wheel FORWARD, move each IR sensor over black/white.\n");
    printf("GP20 = motor check (LIFT THE WHEELS), GP21 = repeat part checks.\n\n");
}

/* Returns false if aborted by GP21. */
static bool motor_step(uint32_t wheel, bool forward, int32_t delta[2], float *p_vmin)
{
    int32_t c0[2];
    int32_t c1[2];
    uint32_t t0;

    enc_snapshot(c0);
    motor_drive(wheel, forward, MOTOR_TEST_DUTY);
    t0 = now_ms();
    while ((now_ms() - t0) < MOTOR_TEST_MS)
    {
        float v = vbat_pin_volts();
        if (v < *p_vmin)
        {
            *p_vmin = v;
        }
        if (!gpio_get(BTN_STOP_PIN))
        {
            motors_off();
            return false;
        }
        sleep_ms(1u);
    }
    motors_off();
    sleep_ms(400u);     /* let it spin down before the next step */
    enc_snapshot(c1);
    delta[WHEEL_L] = c1[WHEEL_L] - c0[WHEEL_L];
    delta[WHEEL_R] = c1[WHEEL_R] - c0[WHEEL_R];
    return true;
}

static void run_motor_check(void)
{
    static const char *const wname[2] = {"LEFT ", "RIGHT"};
    float v_idle = 0.0f;
    float v_min = 10.0f;
    int32_t fwd_sign[2] = {0, 0};
    uint32_t wheel;

    printf("\n---- motor check: %u %% duty, %u ms per step. GP21 held = stop ----\n",
           MOTOR_TEST_DUTY, MOTOR_TEST_MS);
    (void)check_battery(&v_idle);
    sleep_ms(1000u);

    for (wheel = 0u; wheel < 2u; wheel++)
    {
        uint32_t d;
        for (d = 0u; d < 2u; d++)
        {
            int32_t delta[2];
            bool forward = (d == 0u);
            uint32_t other = 1u - wheel;
            if (!motor_step(wheel, forward, delta, &v_min))
            {
                printf("[motor] ABORTED by GP21 - motors off\n");
                return;
            }
            printf("[motor] %s %s: encoder L %+ld, R %+ld", wname[wheel],
                   forward ? "fwd(A=PWM)" : "rev(B=PWM)", (long)delta[WHEEL_L], (long)delta[WHEEL_R]);
            if ((delta[wheel] > -MOVED_COUNTS) && (delta[wheel] < MOVED_COUNTS))
            {
                if ((delta[other] <= -MOVED_COUNTS) || (delta[other] >= MOVED_COUNTS))
                {
                    printf("  <- WRONG ENCODER moved: motors or encoders swapped\n");
                }
                else
                {
                    printf("  <- own encoder did not move: motor not turning, or encoder dead\n");
                }
            }
            else
            {
                if (forward)
                {
                    fwd_sign[wheel] = (delta[wheel] > 0) ? 1 : -1;
                }
                printf("  OK\n");
            }
        }
    }
    printf("[motor] 'fwd(A=PWM)' counts L %s R %s. Look at the wheels: did fwd(A=PWM) drive the robot "
           "FORWARD on both sides? If not, swap that motor's A/B in software (note it for Buddy 2).\n",
           (fwd_sign[WHEEL_L] > 0) ? "up" : ((fwd_sign[WHEEL_L] < 0) ? "down" : "?"),
           (fwd_sign[WHEEL_R] > 0) ? "up" : ((fwd_sign[WHEEL_R] < 0) ? "down" : "?"));
    printf("[vbat] GP28 pin idle %.3f V, lowest under one motor %.3f V (sag %.3f V at pin)%s\n",
           (double)v_idle, (double)v_min, (double)(v_idle - v_min),
           (VBAT_DIVIDER > 0.0f) ? "" : " - scale by the divider once known");
}

static bool button_pressed(uint32_t pin, bool *p_was_down, uint32_t *p_last_ms)
{
    bool down = !gpio_get(pin);
    bool pressed = false;
    uint32_t t = now_ms();

    if (down && !*p_was_down && ((t - *p_last_ms) > 50u))
    {
        pressed = true;
        *p_last_ms = t;
    }
    *p_was_down = down;
    return pressed;
}

int main(void)
{
    uint32_t next_live = 0u;
    bool start_down = false;
    bool stop_down = false;
    uint32_t start_last = 0u;
    uint32_t stop_last = 0u;

    stdio_init_all();
    hw_init();
    motors_off();
    sleep_ms(3000u);

    printf("\n=== P5-IS34 parts check (all buddies) ===\n");
    printf("Robot on blocks, wheels off the table. Ultrasonic pointed at a wall 20-100 cm away.\n");
    run_part_checks();

    for (;;)
    {
        uint32_t t = now_ms();

        if ((int32_t)(t - next_live) >= 0)
        {
            int32_t c[2];
            enc_snapshot(c);
            printf("[live] IR L=%u C=%u B=%u (edges %lu/%lu/%lu) | enc L=%ld R=%ld (invalid %lu/%lu) | "
                   "GP20=%s GP21=%s | GP28 %.3f V\n",
                   (unsigned)gpio_get(IR_LEFT_PIN), (unsigned)gpio_get(IR_CENTER_PIN),
                   (unsigned)gpio_get(IR_BARCODE_PIN),
                   (unsigned long)g_ir_edges[0], (unsigned long)g_ir_edges[1],
                   (unsigned long)g_ir_edges[2],
                   (long)c[WHEEL_L], (long)c[WHEEL_R],
                   (unsigned long)g_enc_invalid[WHEEL_L], (unsigned long)g_enc_invalid[WHEEL_R],
                   gpio_get(BTN_START_PIN) ? "up" : "DOWN", gpio_get(BTN_STOP_PIN) ? "up" : "DOWN",
                   (double)vbat_pin_volts());
            next_live = t + 500u;
        }

        if (button_pressed(BTN_START_PIN, &start_down, &start_last))
        {
            run_motor_check();
            /* A GP21 hold that aborted the check is not a new press. */
            stop_down = !gpio_get(BTN_STOP_PIN);
            start_down = !gpio_get(BTN_START_PIN);
        }
        if (button_pressed(BTN_STOP_PIN, &stop_down, &stop_last))
        {
            run_part_checks();
        }
        sleep_ms(5u);
    }
}
