/*
 *	motion_hw.c
 *	Subsystem 2 -- motor PWM and wheel encoders.
 *
 *	Motors (Robo Pico M1/M2): forward = A driven with PWM and B low,
 *	reverse = A low and B PWM, off = both low (bring-up, after the 26 Oct
 *	M1 wire swap).  PWM 125 MHz / 12,500 = 10 kHz, as in bring-up.
 *
 *	Encoders: every A and B edge is decoded (4x), about 2,540 counts/rev.
 *	The edges arrive through the shared dispatcher (app_gpio.c).
 */

#include "subsystem2.h"
#include "../app_gpio.h"
#include "../app_stop.h"

#define PIN_M_L_A	8		/* PWM slice 4 */
#define PIN_M_L_B	9
#define PIN_M_R_A	10		/* PWM slice 5 */
#define PIN_M_R_B	11
#define PIN_ENC_L_A	4
#define PIN_ENC_L_B	5
#define PIN_ENC_R_A	16
#define PIN_ENC_R_B	17

#define PWM_WRAP	12499u		/* 10 kHz */
#define DUTY_FULL	1000		/* duty is in per mille */

#define WHEEL_L		0
#define WHEEL_R		1
#define QDEC_BAD	2

/* (previous AB << 2) | new AB -> +1 / -1 count, 0 no change, 2 invalid */
LOCAL const B qdec[16] = {
	 0, +1, -1, QDEC_BAD,
	-1,  0, QDEC_BAD, +1,
	+1, QDEC_BAD,  0, -1,
	QDEC_BAD, -1, +1,  0
};

LOCAL volatile W	enc_count[2];	/* raw: the left one counts down forward */
LOCAL volatile UW	enc_invalid;
LOCAL UB		enc_prev[2];

LOCAL UB enc_ab(INT wheel)
{
	UW in = in_w(GPIO_IN);

	if (wheel == WHEEL_L) {
		return (UB)((((in >> PIN_ENC_L_A) & 1u) << 1) | ((in >> PIN_ENC_L_B) & 1u));
	}
	return (UB)((((in >> PIN_ENC_R_A) & 1u) << 1) | ((in >> PIN_ENC_R_B) & 1u));
}

/* Interrupt context, from app_gpio.c */
LOCAL void enc_edge(UINT pin, UW events)
{
	INT	wheel = (pin == PIN_ENC_L_A || pin == PIN_ENC_L_B) ? WHEEL_L : WHEEL_R;
	UB	ab = enc_ab(wheel);
	B	step = qdec[(enc_prev[wheel] << 2) | ab];

	(void)events;
	if (step == QDEC_BAD) {
		enc_invalid++;
	} else {
		enc_count[wheel] += step;
	}
	enc_prev[wheel] = ab;
}

/* One write per slice sets A (low half) and B (high half) together, so a
   STOP in an interrupt can never be undone by a half-finished update. */
#define PWM_SLICE_CC(slice)	(PWM_BASE + (slice) * 0x14 + PWM_CHx_CC)
#define SLICE_L		4		/* GP8 = A, GP9 = B */
#define SLICE_R		5		/* GP10 = A, GP11 = B */

LOCAL UW cc_pair(INT duty)
{
	UW level;

	if (duty > DUTY_FULL) duty = DUTY_FULL;
	if (duty < -DUTY_FULL) duty = -DUTY_FULL;
	level = (UW)((duty >= 0) ? duty : -duty) * (PWM_WRAP + 1u) / DUTY_FULL;
	return (duty >= 0) ? level : (level << 16);
}

EXPORT void motor_write(INT duty_l, INT duty_r)
{
	UW	cc_l = cc_pair(duty_l);
	UW	cc_r = cc_pair(duty_r);
	UINT	s;

	APP_LOCK(s);
	if (app_stop_reasons() != 0u) {
		cc_l = 0u;			/* STOP always wins */
		cc_r = 0u;
	}
	out_w(PWM_SLICE_CC(SLICE_L), cc_l);
	out_w(PWM_SLICE_CC(SLICE_R), cc_r);
	APP_UNLOCK(s);
}

EXPORT void motor_safe_off(void)
{
	out_w(PWM_SLICE_CC(SLICE_L), 0u);
	out_w(PWM_SLICE_CC(SLICE_R), 0u);
}

/* Counts with + = forward for both wheels */
EXPORT void encoder_read(W *l, W *r, UW *invalid)
{
	UINT	s;

	APP_LOCK(s);
	*l = -enc_count[WHEEL_L];
	*r = enc_count[WHEEL_R];
	if (invalid != NULL) {
		*invalid = enc_invalid;
	}
	APP_UNLOCK(s);
}

EXPORT ER motion_hw_init(void)
{
	const UINT	mpins[4] = {PIN_M_L_A, PIN_M_L_B, PIN_M_R_A, PIN_M_R_B};
	const UINT	epins[4] = {PIN_ENC_L_A, PIN_ENC_L_B, PIN_ENC_R_A, PIN_ENC_R_B};
	ER		er;
	INT		i;

	motor_safe_off();
	for (i = 0; i < 4; i++) {
		pwm_set_pin(mpins[i]);
	}
	pwm_set_wrap(PIN_M_L_A, PWM_WRAP);
	pwm_set_wrap(PIN_M_R_A, PWM_WRAP);
	pwm_set_enabled(PIN_M_L_A, TRUE);
	pwm_set_enabled(PIN_M_R_A, TRUE);

	for (i = 0; i < 4; i++) {
		/* pull-up: harmless for push-pull, needed for open-collector */
		er = app_gpio_input(epins[i], TRUE);
		if (er < E_OK) {
			return er;
		}
	}
	enc_prev[WHEEL_L] = enc_ab(WHEEL_L);
	enc_prev[WHEEL_R] = enc_ab(WHEEL_R);
	for (i = 0; i < 4; i++) {
		er = app_gpio_irq_add(epins[i], APP_GPIO_BOTH, enc_edge);
		if (er < E_OK) {
			return er;
		}
	}
	return E_OK;
}
