/*
 *  motion_hw.c
 *  Subsystem 2 -- motor PWM and wheel encoders.
 *
 *  Motors (Robo Pico M1/M2): forward = A driven with PWM and B low,
 *  reverse = A low and B PWM, off = both low (bring-up, after the 26 Oct
 *  M1 wire swap).  PWM 125 MHz / 12,500 = 10 kHz, as in bring-up.
 *
 *  Encoders: every A and B edge is decoded (4x), about 2,540 counts/rev.
 *  The edges arrive through the shared dispatcher (app_gpio.c).
 */

#include "subsystem2.h"
#include "../app_gpio.h"
#include "../app_stop.h"

#define PIN_M_L_A   8 /* PWM slice 4 */
#define PIN_M_L_B   9
#define PIN_M_R_A   10 /* PWM slice 5 */
#define PIN_M_R_B   11
#define PIN_ENC_L_A 4
#define PIN_ENC_L_B 5
#define PIN_ENC_R_A 16
#define PIN_ENC_R_B 17

#define PWM_WRAP  12499u /* 10 kHz */
#define DUTY_FULL 1000   /* duty is in per mille */

#define WHEEL_L  0
#define WHEEL_R  1
#define QDEC_BAD 2

/* (previous AB << 2) | new AB -> +1 / -1 count, 0 no change, 2 invalid */
LOCAL const int8_t g_qdec[16] = {0,        +1, -1, QDEC_BAD, -1, 0,
                                 QDEC_BAD, +1, +1, QDEC_BAD, 0,  -1,
                                 QDEC_BAD, -1, +1, 0};

LOCAL volatile int32_t
    g_enc_count[2]; /* raw: the left one counts down forward */
LOCAL volatile uint32_t g_enc_invalid;
LOCAL uint8_t           g_enc_prev[2];

LOCAL uint8_t enc_ab (int32_t wheel)
{
    uint32_t pins = in_w(GPIO_IN);

    if (WHEEL_L == wheel)
    {
        return (uint8_t)((((pins >> PIN_ENC_L_A) & 1u) << 1) |
                         ((pins >> PIN_ENC_L_B) & 1u));
    }
    return (uint8_t)((((pins >> PIN_ENC_R_A) & 1u) << 1) |
                     ((pins >> PIN_ENC_R_B) & 1u));
}

/* Interrupt context, from app_gpio.c */
LOCAL void encoder_edge_isr (uint32_t pin, uint32_t events)
{
    int32_t wheel =
        ((PIN_ENC_L_A == pin) || (PIN_ENC_L_B == pin)) ? WHEEL_L : WHEEL_R;
    uint8_t state_ab = enc_ab(wheel);
    int8_t  step     = g_qdec[(g_enc_prev[wheel] << 2) | state_ab];

    (void)events;
    if (QDEC_BAD == step)
    {
        g_enc_invalid++;
    }
    else
    {
        g_enc_count[wheel] += step;
    }
    g_enc_prev[wheel] = state_ab;
}

/* One write per slice sets A (low half) and B (high half) together, so a
   STOP in an interrupt can never be undone by a half-finished update. */
#define PWM_SLICE_CC(slice) (PWM_BASE + (slice) * 0x14 + PWM_CHx_CC)
#define SLICE_L             4 /* GP8 = A, GP9 = B */
#define SLICE_R             5 /* GP10 = A, GP11 = B */

LOCAL uint32_t cc_pair (int32_t duty)
{
    uint32_t level;

    if (duty > DUTY_FULL)
    {
        duty = DUTY_FULL;
    }
    if (duty < -DUTY_FULL)
    {
        duty = -DUTY_FULL;
    }
    level =
        (uint32_t)((duty >= 0) ? duty : -duty) * (PWM_WRAP + 1u) / DUTY_FULL;
    return (duty >= 0) ? level : (level << 16);
}

EXPORT void motor_write (int32_t duty_l, int32_t duty_r)
{
    uint32_t cc_l = cc_pair(duty_l);
    uint32_t cc_r = cc_pair(duty_r);
    uint32_t irq_state;

    APP_LOCK(irq_state);
    if (app_stop_reasons() != 0u)
    {
        cc_l = 0u; /* STOP always wins */
        cc_r = 0u;
    }
    out_w(PWM_SLICE_CC(SLICE_L), cc_l);
    out_w(PWM_SLICE_CC(SLICE_R), cc_r);
    APP_UNLOCK(irq_state);
}

EXPORT void motor_safe_off (void)
{
    out_w(PWM_SLICE_CC(SLICE_L), 0u);
    out_w(PWM_SLICE_CC(SLICE_R), 0u);
}

EXPORT int32_t motion_odometer (void)
{
    return (g_enc_count[WHEEL_R] - g_enc_count[WHEEL_L]) / 2;
}

/* Counts with + = forward for both wheels */
EXPORT void encoder_read (int32_t * p_left, int32_t * p_right,
                          uint32_t * p_invalid)
{
    uint32_t irq_state;

    APP_LOCK(irq_state);
    *p_left  = -g_enc_count[WHEEL_L];
    *p_right = g_enc_count[WHEEL_R];
    if (p_invalid != NULL)
    {
        *p_invalid = g_enc_invalid;
    }
    APP_UNLOCK(irq_state);
}

EXPORT ER motion_hw_init (void)
{
    const uint32_t mpins[4] = {PIN_M_L_A, PIN_M_L_B, PIN_M_R_A, PIN_M_R_B};
    const uint32_t epins[4] = {PIN_ENC_L_A, PIN_ENC_L_B, PIN_ENC_R_A,
                               PIN_ENC_R_B};
    ER             err;
    int32_t        idx;

    app_hw_release(RESETS_RESET_PWM); /* not released by the kernel */
    motor_safe_off();
    for (idx = 0; idx < 4; idx++)
    {
        pwm_set_pin(mpins[idx]);
    }
    pwm_set_wrap(PIN_M_L_A, PWM_WRAP);
    pwm_set_wrap(PIN_M_R_A, PWM_WRAP);
    pwm_set_enabled(PIN_M_L_A, TRUE);
    pwm_set_enabled(PIN_M_R_A, TRUE);

    for (idx = 0; idx < 4; idx++)
    {
        /* pull-up: harmless for push-pull, needed for open-collector */
        err = app_gpio_input(epins[idx], TRUE);
        if (err < E_OK)
        {
            return err;
        }
    }
    g_enc_prev[WHEEL_L] = enc_ab(WHEEL_L);
    g_enc_prev[WHEEL_R] = enc_ab(WHEEL_R);
    for (idx = 0; idx < 4; idx++)
    {
        err = app_gpio_irq_add(epins[idx], APP_GPIO_BOTH, encoder_edge_isr);
        if (err < E_OK)
        {
            return err;
        }
    }
    return E_OK;
}

/*** end of file ***/

