/**
 * @file    motor.c
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
#include "motor.h"

#define PIN_M_L_A   8 /* PWM slice 4 */
#define PIN_M_L_B   9
#define PIN_M_R_A   10 /* PWM slice 5 */
#define PIN_M_R_B   11
#define PIN_ENC_L_A 4
#define PIN_ENC_L_B 5
#define PIN_ENC_R_A 16
#define PIN_ENC_R_B 17
#define MOTOR_PINS  4 /* L A, L B, R A, R B */
#define ENC_PINS    4 /* L A, L B, R A, R B */

#define PWM_WRAP  12499u /* 10 kHz */
#define DUTY_FULL 1000   /* duty is in per mille */

#define WHEEL_L  0
#define WHEEL_R  1
#define QDEC_BAD 2

/* (previous AB << 2) | new AB -> +1 / -1 count, 0 no change, 2 invalid */
LOCAL const int8_t g_qdec[16] = {0,        +1, -1, QDEC_BAD, -1, 0,
                                 QDEC_BAD, +1, +1, QDEC_BAD, 0,  -1,
                                 QDEC_BAD, -1, +1, 0};

/* Encoder counts stay 32 bits (Change Requirement 1: packed where data is
   handed over, not in the encoder interrupt; Tribunal 1): 0.080 mm per
   count, so 2^31 counts are 171 km and no run comes near it, but a 20 m
   run is already 250,000 counts (19 bits + sign).  Read as one aligned
   word: atomic without a lock. */
LOCAL volatile int32_t
    g_enc_count[2]; /* raw: the left one counts down forward */
LOCAL volatile uint32_t g_enc_invalid; /* a counter: all 32 bits */
/* the last A/B levels, 2 bits per wheel (left in bits 0-1, right 2-3);
   written only by the encoder interrupt (and once before it is on) */
LOCAL uint8_t g_enc_prev;

/* The duties last written, for Safety's stall check: -1000..1000 per
   mille, 11 bits signed each.  Written by Motion and by motor_safe_off()
   (any context), so always under APP_LOCK. */
typedef struct
{
    int32_t left  APP_FIELD(11);
    int32_t right APP_FIELD(11);
} motor_duty_t;

LOCAL volatile motor_duty_t g_duty;

/** The A/B levels last seen on one wheel (2 bits). */
Inline uint32_t enc_prev_get (int32_t wheel)
{
    /* wheel is WHEEL_L or WHEEL_R (0 or 1): the shift is 0 or 2 */
    return ((uint32_t)g_enc_prev >> ((uint32_t)wheel * 2u)) & 3u;
}

/** Records the A/B levels of one wheel (2 bits). */
Inline void enc_prev_set (int32_t wheel, uint32_t state_ab)
{
    uint32_t shift = (uint32_t)wheel * 2u; /* wheel 0 or 1: shift 0 or 2 */

    /* only bits 0-3 are used: fits uint8_t */
    g_enc_prev = (uint8_t)(((uint32_t)g_enc_prev & ~(3u << shift)) |
                           ((state_ab & 3u) << shift));
}

/** The A/B level pair of one wheel's encoder. */
LOCAL uint8_t enc_ab (int32_t wheel)
{
    uint32_t pins = in_w(GPIO_IN);

    if (WHEEL_L == wheel)
    {
        /* two bits, 0..3: fit uint8_t */
        return (uint8_t)((((pins >> PIN_ENC_L_A) & 1u) << 1) |
                         ((pins >> PIN_ENC_L_B) & 1u));
    }

    /* two bits, 0..3: fit uint8_t */
    return (uint8_t)((((pins >> PIN_ENC_R_A) & 1u) << 1) |
                     ((pins >> PIN_ENC_R_B) & 1u));
}

/** Interrupt context, from app_gpio.c */
LOCAL void encoder_edge_isr (uint32_t pin, uint32_t events)
{
    int32_t wheel =
        ((PIN_ENC_L_A == pin) || (PIN_ENC_L_B == pin)) ? WHEEL_L : WHEEL_R;
    uint8_t state_ab = enc_ab(wheel);
    int8_t  step     = g_qdec[(enc_prev_get(wheel) << 2) | state_ab];

    (void)events;

    if (QDEC_BAD == step)
    {
        g_enc_invalid++;
    }
    else
    {
        g_enc_count[wheel] += step;
    }

    enc_prev_set(wheel, state_ab);
}

/* One write per slice sets A (low half) and B (high half) together, so a
   STOP in an interrupt can never be undone by a half-finished update. */
#define SLICE_L  4 /* GP8 = A, GP9 = B */
#define SLICE_R  5 /* GP10 = A, GP11 = B */
#define PWM_CC_L (PWM_BASE + (SLICE_L * 0x14) + PWM_CHx_CC)
#define PWM_CC_R (PWM_BASE + (SLICE_R * 0x14) + PWM_CHx_CC)

/** PWM compare value pair (A and B halves) for one motor's signed duty. */
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

    /* |duty| <= DUTY_FULL (clamped above): 0..1000, never negative */
    level =
        (uint32_t)((duty >= 0) ? duty : -duty) * (PWM_WRAP + 1u) / DUTY_FULL;
    return (duty >= 0) ? level : (level << 16);
}

/** Sets both motor duties, -1000..1000 per mille, in one write per motor. */
EXPORT void motor_write (int32_t duty_l, int32_t duty_r)
{
    uint32_t cc_l = cc_pair(duty_l);
    uint32_t cc_r = cc_pair(duty_r);
    uint32_t irq_state;

    APP_LOCK(irq_state);

    if (app_stop_reasons() != 0u)
    {
        cc_l   = 0u; /* STOP always wins */
        cc_r   = 0u;
        duty_l = 0;
        duty_r = 0;
    }

    out_w(PWM_CC_L, cc_l);
    out_w(PWM_CC_R, cc_r);
    /* cc_pair() clamped what reached the motors to -1000..1000: the same
       range is recorded */
    g_duty.left  = (duty_l > DUTY_FULL)    ? DUTY_FULL
                   : (duty_l < -DUTY_FULL) ? -DUTY_FULL
                                           : duty_l;
    g_duty.right = (duty_r > DUTY_FULL)    ? DUTY_FULL
                   : (duty_r < -DUTY_FULL) ? -DUTY_FULL
                                           : duty_r;
    APP_UNLOCK(irq_state);
}

EXPORT void motor_safe_off (void)
{
    uint32_t irq_state;

    out_w(PWM_CC_L, 0u);
    out_w(PWM_CC_R, 0u);
    APP_LOCK(irq_state);
    g_duty.left  = 0;
    g_duty.right = 0;
    APP_UNLOCK(irq_state);
}

/** The duties last written, -1000..1000 per mille (0 after a safe-off). */
EXPORT void motor_duties (int32_t * p_left, int32_t * p_right)
{
    uint32_t irq_state;

    APP_LOCK(irq_state);
    *p_left  = g_duty.left;
    *p_right = g_duty.right;
    APP_UNLOCK(irq_state);
}

EXPORT int32_t motor_odometer (void)
{
    return (g_enc_count[WHEEL_R] - g_enc_count[WHEEL_L]) / 2;
}

/** Counts with + = forward for both wheels */
EXPORT void motor_encoders (int32_t * p_left, int32_t * p_right,
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

/** Sets up the motor PWM and the encoder interrupts. */
EXPORT ER motor_init (void)
{
    const uint32_t mpins[MOTOR_PINS] = {PIN_M_L_A, PIN_M_L_B, PIN_M_R_A,
                                        PIN_M_R_B};
    const uint32_t epins[ENC_PINS]   = {PIN_ENC_L_A, PIN_ENC_L_B, PIN_ENC_R_A,
                                        PIN_ENC_R_B};
    ER             err;
    int32_t        idx;

    app_gpio_hw_release(RESETS_RESET_PWM); /* not released by the kernel */
    motor_safe_off();

    for (idx = 0; idx < MOTOR_PINS; idx++)
    {
        pwm_set_pin(mpins[idx]);
    }

    pwm_set_wrap(PIN_M_L_A, PWM_WRAP);
    pwm_set_wrap(PIN_M_R_A, PWM_WRAP);
    pwm_set_enabled(PIN_M_L_A, true);
    pwm_set_enabled(PIN_M_R_A, true);

    for (idx = 0; idx < ENC_PINS; idx++)
    {
        /* pull-up: harmless for push-pull, needed for open-collector */
        err = app_gpio_input(epins[idx], true);

        if (err < E_OK)
        {
            return err;
        }
    }

    enc_prev_set(WHEEL_L, enc_ab(WHEEL_L));
    enc_prev_set(WHEEL_R, enc_ab(WHEEL_R));

    for (idx = 0; idx < ENC_PINS; idx++)
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

