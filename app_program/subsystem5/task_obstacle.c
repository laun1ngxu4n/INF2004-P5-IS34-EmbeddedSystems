/*
 *	task_obstacle.c
 *	Subsystem 5 -- the Obstacle Scanner task (every 60 ms) and the
 *	profiling scan.
 *
 *	Forward watch: one ping per 60 ms with the servo straight ahead; two
 *	readings in a row under S5_TRIP_MM tell Mission (MSN_FLG_OBSTACLE).
 *
 *	Profiling scan (TEAM.md 7.2), robot standing still:
 *	  O1  servo settle wait after every move (3 ms per degree, >= 60 ms)
 *	  O2  5 pings per ray (60 ms apart), median; >= 3 missing = open space
 *	  O3  background: a ray farther than front + BG_MM is open space
 *	  O4  an edge needs 2 rays in a row off the obstacle (hysteresis:
 *	      on the obstacle while within IN_MM of its front distance)
 *	  O6  width = left edge - right edge, with a +/- from the angle step,
 *	      the beam and (O5, obstacle_combine) the spread of two scans.
 */

#include "subsystem5.h"

#define PIN_TRIG	6
#define PIN_ECHO	7
#define PIN_SERVO	14		/* PWM slice 7 A */
#define SERVO_SLICE	7

#define SCAN_FROM	40		/* servo degrees */
#define SCAN_TO		140
#define SCAN_STEP	5
#define NRAYS		((SCAN_TO - SCAN_FROM) / SCAN_STEP + 1)
#define PINGS		5
#define SETTLE_MS_DEG	3
#define SETTLE_MIN_MS	60
#define IN_MM		150		/* on the obstacle while |d - front| <= this */
#define BG_MM		250		/* open space beyond front + this */
#define FAR		(-1)
#define MAX_FRONT_MM	600		/* nothing nearer = no obstacle */
#define BEAM_UNC_DEG	3		/* +/- on the beam correction */

LOCAL volatile UW	echo_rise_us;
LOCAL volatile UW	echo_width_us;
LOCAL volatile BOOL	echo_rose, echo_done;
LOCAL volatile BOOL	watch;
LOCAL volatile BOOL	scanning;
LOCAL volatile INT	front_mm = FAR;
LOCAL INT		servo_now = 90;

/* ECHO edges: interrupt context */
LOCAL void echo_edge(UINT pin, UW events)
{
	UW now = app_time_us();

	if (app_gpio_get(pin) != 0u) {
		echo_rise_us = now;
		echo_rose = TRUE;
	} else if (echo_rose) {
		echo_width_us = now - echo_rise_us;
		echo_done = TRUE;
	}
	(void)events;
}

LOCAL void trigger(void)
{
	UW t0;

	echo_rose = FALSE;
	echo_done = FALSE;
	gpio_set_val(PIN_TRIG, 1);
	t0 = app_time_us();
	while (app_time_us() - t0 < 12u) {
		;
	}
	gpio_set_val(PIN_TRIG, 0);
}

/* Result of the last trigger: mm, or FAR (no echo / out of range) */
LOCAL INT echo_mm(void)
{
	if (!echo_done || echo_width_us < 100u || echo_width_us > 25000u) {
		return FAR;
	}
	return (INT)(echo_width_us * 174u / 1000u);	/* 0.348 mm/us at 28 C, / 2 */
}

EXPORT INT sonar_ping_mm(void)
{
	trigger();
	tk_dly_tsk(60);
	return echo_mm();
}

EXPORT void servo_set_angle(INT deg)
{
	INT us;

	if (deg < 30) deg = 30;
	if (deg > 150) deg = 150;
	servo_now = deg;
	us = 1500 + (deg - 90) * S5_SERVO_US_PER_DEG * (S5_SERVO_LEFT_HIGH ? 1 : -1);
	pwm_set_cc(PIN_SERVO, (UW)us);			/* 1 count = 1 us */
}

EXPORT void obstacle_watch(BOOL on)	{ watch = on; }
EXPORT INT obstacle_front_mm(void)	{ return front_mm; }

EXPORT ER obstacle_hw_init(void)
{
	ER er;

	app_hw_release(RESETS_RESET_PWM);
	/* servo: 125 MHz / 125 = 1 MHz, wrap 20000 = 50 Hz */
	out_w(PWM_BASE + SERVO_SLICE * 0x14 + PWM_CHx_DIV, 125u << 4);
	pwm_set_wrap(PIN_SERVO, 19999);
	pwm_set_cc(PIN_SERVO, 1500);
	pwm_set_pin(PIN_SERVO);
	pwm_set_enabled(PIN_SERVO, TRUE);

	er = app_gpio_output(PIN_TRIG);
	if (er < E_OK) return er;
	er = app_gpio_input(PIN_ECHO, FALSE);
	if (er < E_OK) return er;
	clr_w(GPIO(PIN_ECHO), GPIO_PUE);
	set_w(GPIO(PIN_ECHO), GPIO_PDE);		/* as in bring-up */
	return app_gpio_irq_add(PIN_ECHO, APP_GPIO_BOTH, echo_edge);
}

/* ---- maths without libm -------------------------------------------- */

LOCAL float sin_deg(float d)
{
	float x = d * 3.14159265f / 180.0f;
	float x2 = x * x;

	return x * (1.0f - x2 / 6.0f * (1.0f - x2 / 20.0f * (1.0f - x2 / 42.0f *
		(1.0f - x2 / 72.0f))));
}

LOCAL float sqrt_f(float v)
{
	float	r = (v > 1.0f) ? v : 1.0f;
	INT	i;

	if (v <= 0.0f) return 0.0f;
	for (i = 0; i < 12; i++) r = 0.5f * (r + v / r);
	return r;
}

LOCAL INT median5(INT *v, INT n)
{
	INT i, j, t;

	for (i = 1; i < n; i++) {
		for (j = i; j > 0 && v[j] < v[j - 1]; j--) {
			t = v[j]; v[j] = v[j - 1]; v[j - 1] = t;
		}
	}
	return v[n / 2];
}

/* ---- the profiling scan --------------------------------------------- */

/* One side: walk outward from straight ahead.  dir = +1 / -1 is the
   direction in the ray array (which side that is depends on the servo).
   Returns the lateral edge (mm, signed) and its half-step uncertainty. */
LOCAL void side_edge(const INT *d, const INT *phi, INT centre, INT dir, INT front,
		     INT *edge_mm, INT *unc_mm, BOOL *clear)
{
	INT	i = centre, last_in = centre, out_run = 0, first_out = -1;
	BOOL	other = FALSE;

	for (i = centre + dir; i >= 0 && i < NRAYS; i += dir) {
		BOOL in = (d[i] != FAR) && (d[i] - front <= IN_MM) && (front - d[i] <= IN_MM);

		if (in && out_run < 2) {
			last_in = i;
			out_run = 0;
			first_out = -1;
		} else if (!in) {
			if (out_run == 0) first_out = i;
			out_run++;
			/* past the edge: open space, or something else nearer? */
			if (out_run >= 2 && d[i] != FAR && d[i] < front + BG_MM) {
				other = TRUE;
			}
		}
	}
	if (out_run < 2 || first_out < 0) {
		/* obstacle wider than the scan on this side */
		*edge_mm = (INT)((float)d[last_in] * sin_deg((float)phi[last_in]));
		*unc_mm = 50;
		*clear = FALSE;
		return;
	}
	{
		float	a_in = (float)phi[last_in], a_out = (float)phi[first_out];
		float	side = (a_out > 0.0f) ? 1.0f : -1.0f;	/* + = left */
		float	a_edge = 0.5f * (a_in + a_out) - side * (float)S5_BEAM_HALF_DEG;
		float	r = (float)d[last_in];

		if (a_edge * side < 0.0f) a_edge = 0.0f;	/* never past the centre */
		*edge_mm = (INT)(r * sin_deg(a_edge));
		*unc_mm = (INT)(r * (sin_deg(a_out) - sin_deg(a_in)) / 2.0f);
		if (*unc_mm < 0) *unc_mm = -*unc_mm;
		*clear = !other;
	}
}

EXPORT void obstacle_scan(OBSTACLE_PROFILE *p)
{
	INT	d[NRAYS], phi[NRAYS];
	INT	k, centre = (90 - SCAN_FROM) / SCAN_STEP;
	INT	ul, ur;

	scanning = TRUE;
	for (k = 0; k < NRAYS; k++) {
		INT	ang = SCAN_FROM + k * SCAN_STEP;
		INT	move = (ang > servo_now) ? ang - servo_now : servo_now - ang;
		INT	settle = move * SETTLE_MS_DEG;
		INT	v[PINGS], nv = 0, j;

		servo_set_angle(ang);
		tk_dly_tsk((settle > SETTLE_MIN_MS) ? settle : SETTLE_MIN_MS);	/* O1 */
		for (j = 0; j < PINGS; j++) {					/* O2 */
			INT mm = sonar_ping_mm();
			if (mm != FAR) v[nv++] = mm;
		}
		d[k] = (nv >= 3) ? median5(v, nv) : FAR;
		/* + = left of the robot */
		phi[k] = (ang - 90) * (S5_SERVO_LEFT_HIGH ? 1 : -1);
	}
	servo_set_angle(90);
	tk_dly_tsk(60 * SETTLE_MS_DEG);
	scanning = FALSE;

	tm_printf((UB*)"[SCAN] rays (deg:mm, -1 = open):");
	for (k = 0; k < NRAYS; k++) {
		tm_printf((UB*)" %d:%d", phi[k], d[k]);
	}
	tm_printf((UB*)"\n");

	p->rays = NRAYS;
	p->front_mm = d[centre];
	p->found = (d[centre] != FAR && d[centre] <= MAX_FRONT_MM);
	if (!p->found) {
		p->width_mm = p->unc_mm = p->left_mm = p->right_mm = 0;
		p->left_clear = p->right_clear = TRUE;
		return;
	}
	/* phi rises with k only if LEFT_HIGH: walk "left" in the right order */
	side_edge(d, phi, centre, S5_SERVO_LEFT_HIGH ? +1 : -1, d[centre],
		  &p->left_mm, &ul, &p->left_clear);
	side_edge(d, phi, centre, S5_SERVO_LEFT_HIGH ? -1 : +1, d[centre],
		  &p->right_mm, &ur, &p->right_clear);
	p->width_mm = p->left_mm - p->right_mm;
	{
		float beam = 2.0f * (float)d[centre] * sin_deg((float)BEAM_UNC_DEG);
		p->unc_mm = (INT)sqrt_f((float)(ul * ul + ur * ur) + beam * beam);
	}
}

EXPORT void obstacle_combine(const OBSTACLE_PROFILE *a, const OBSTACLE_PROFILE *b,
			     OBSTACLE_PROFILE *out)
{
	INT spread = a->width_mm - b->width_mm;

	if (spread < 0) spread = -spread;
	*out = *b;
	out->found = a->found && b->found;
	out->width_mm = (a->width_mm + b->width_mm) / 2;
	out->left_mm = (a->left_mm + b->left_mm) / 2;
	out->right_mm = (a->right_mm + b->right_mm) / 2;
	out->unc_mm = (a->unc_mm + b->unc_mm) / 2;
	if (spread / 2 > out->unc_mm) out->unc_mm = spread / 2;
	out->left_clear = a->left_clear && b->left_clear;
	out->right_clear = a->right_clear && b->right_clear;
}

/* ---- the task: forward watch ---------------------------------------- */

EXPORT void obstacle_task(INT stacd, void *exinf)
{
	INT	close_count = 0;
	BOOL	pinged = FALSE, told = FALSE;

	(void)stacd; (void)exinf;

	while (1) {
		tk_slp_tsk(TMO_FEVR);			/* woken every 60 ms */
		app_timing_wake(tmg_obstacle, tk_can_wup(TSK_SELF));

		if (scanning || !watch) {
			pinged = FALSE;
			close_count = 0;
			told = FALSE;
		} else {
			if (pinged) {
				INT mm = echo_mm();

				front_mm = mm;
				if (mm != FAR && mm < S5_TRIP_MM) {
					if (++close_count >= 2 && !told) {
						told = TRUE;
						tk_set_flg(flg_mission, MSN_FLG_OBSTACLE);
					}
				} else {
					close_count = 0;
				}
			}
			if (servo_now != 90) servo_set_angle(90);
			trigger();
			pinged = TRUE;
		}

		app_timing_done(tmg_obstacle);
	}
}
