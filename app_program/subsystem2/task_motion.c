/*
 *	task_motion.c
 *	Subsystem 2 -- the Motion Control task.
 *
 *	Every 10 ms:
 *	  1. read both encoders, measure each wheel's speed (40 ms average);
 *	  2. if any STOP reason is set: motors off, cancel the command;
 *	  3. otherwise run the command: a speed profile (ramp up, cruise,
 *	     slow down before the end), a correction that keeps both wheels
 *	     at the same distance (straight lines, clean spins), and a speed
 *	     controller per wheel (feed-forward + PI) that writes the PWM.
 *
 *	Commands come through a one-slot mailbox under APP_LOCK: the newest
 *	command replaces the old one.  Results are read with
 *	motion_get_status().
 */

#include "subsystem2.h"
#include "../app_stop.h"
#include "../app_math.h"
#include "../subsystem3/subsystem3.h"	/* IF-03: line_steer_mm_s() */

#define CMD_NONE	0
#define CMD_DRIVE	1
#define CMD_TURN	2
#define CMD_HALT	3
#define CMD_FOLLOW	4

#define DT_S		0.010f		/* task period */
#define SPEED_AVG_N	4		/* speed over the last 4 periods */
#define ACCEL_MM_S2	400.0f		/* ramp up and down */
#define V_MIN_MM_S	50.0f		/* slowest speed near the end */
#define KEEP_EQUAL	3.0f		/* mm/s per mm the wheels differ */

/* Per wheel speed controller, duty in per mille.  Feed-forward from the
   bring-up runs (wheels lifted: about 1.25 per mille per mm/s, through 0)
   plus an offset for the load on the floor; PI takes up the rest. */
#define FF_PER_MM_S	1.25f
#define FF_OFFSET	80.0f
#define KP		1.0f		/* per mille per mm/s error */
#define KI		6.0f		/* per mille per (mm/s * s) */
#define I_LIMIT		400.0f

typedef struct {
	INT	type;
	ID	id;
	INT	speed;		/* mm/s, sign = direction (drive) */
	INT	amount;		/* drive: mm; turn: deg (+ = left) */
} CMD;

LOCAL CMD		cmd_slot;	/* written by callers, under APP_LOCK */
LOCAL BOOL		cmd_new;
LOCAL ID		cmd_last_id;
LOCAL MOTION_STATUS	status;		/* written by the task, under APP_LOCK */

LOCAL ID cmd_post(INT type, INT speed, INT amount)
{
	UINT	s;
	ID	id;

	if (type != CMD_HALT && app_stop_reasons() != 0u) {
		return E_OBJ;
	}
	APP_LOCK(s);
	id = ++cmd_last_id;
	cmd_slot.type = type;
	cmd_slot.id = id;
	cmd_slot.speed = speed;
	cmd_slot.amount = amount;
	cmd_new = TRUE;
	status.id = (UW)id;
	status.result = MOT_RES_BUSY;
	APP_UNLOCK(s);
	return id;
}

EXPORT ID motion_drive(INT speed_mm_s, INT dist_mm)
{
	return cmd_post(CMD_DRIVE, speed_mm_s, dist_mm);
}

EXPORT ID motion_turn(INT deg)
{
	return cmd_post(CMD_TURN, S2_SPEED_TURN, deg);
}

EXPORT ID motion_follow(INT speed_mm_s)
{
	return cmd_post(CMD_FOLLOW, speed_mm_s, 0);
}

EXPORT void motion_halt(void)
{
	(void)cmd_post(CMD_HALT, 0, 0);
}

EXPORT void motion_get_status(MOTION_STATUS *st)
{
	UINT	s;

	APP_LOCK(s);
	*st = status;
	APP_UNLOCK(s);
}

LOCAL float clampf(float v, float lo, float hi)
{
	return (v < lo) ? lo : ((v > hi) ? hi : v);
}

LOCAL float absf(float v)
{
	return (v < 0.0f) ? -v : v;
}

typedef struct {
	float	integ;
} WHEEL_PI;

LOCAL INT wheel_duty(WHEEL_PI *w, float target, float measured)
{
	float	err = target - measured;
	float	ff;

	if (target == 0.0f) {
		w->integ = 0.0f;
		return 0;
	}
	ff = FF_PER_MM_S * target + ((target > 0.0f) ? FF_OFFSET : -FF_OFFSET);
	w->integ = clampf(w->integ + KI * err * DT_S, -I_LIMIT, I_LIMIT);
	return (INT)clampf(ff + KP * err + w->integ, -1000.0f, 1000.0f);
}

EXPORT void motion_task(INT stacd, void *exinf)
{
	W		hist_l[SPEED_AVG_N] = {0}, hist_r[SPEED_AVG_N] = {0};
	INT		h = 0;
	W		cl, cr, prev_l, prev_r, start_l = 0, start_r = 0;
	UW		invalid;
	CMD		cur = {0};
	WHEEL_PI	pi_l = {0}, pi_r = {0};
	float		v_prof = 0.0f;		/* profile speed now, mm/s */
	UW		elapsed_ms = 0, limit_ms = 0;
	INT		result = MOT_RES_NONE;
	UINT		s;

	(void)stacd; (void)exinf;
	encoder_read(&prev_l, &prev_r, NULL);

	while (1) {
		float	vl, vr, dl_mm, dr_mm;
		INT	duty_l = 0, duty_r = 0;
		W	sum_l = 0, sum_r = 0;
		INT	i;
		BOOL	fresh;

		tk_slp_tsk(TMO_FEVR);			/* woken every 10 ms */
		app_timing_wake(tmg_motion, tk_can_wup(TSK_SELF));

		/* 1. Encoders and speed */
		encoder_read(&cl, &cr, &invalid);
		hist_l[h] = cl - prev_l;
		hist_r[h] = cr - prev_r;
		h = (h + 1) % SPEED_AVG_N;
		prev_l = cl;
		prev_r = cr;
		for (i = 0; i < SPEED_AVG_N; i++) {
			sum_l += hist_l[i];
			sum_r += hist_r[i];
		}
		/* counts per 40 ms -> mm/s */
		vl = (float)sum_l * S2_UM_PER_COUNT / (1000.0f * SPEED_AVG_N * DT_S);
		vr = (float)sum_r * S2_UM_PER_COUNT / (1000.0f * SPEED_AVG_N * DT_S);

		/* Take a new command (it replaces a running one) */
		fresh = FALSE;
		APP_LOCK(s);
		if (cmd_new) {
			cur = cmd_slot;
			cmd_new = FALSE;
			fresh = TRUE;
		}
		APP_UNLOCK(s);

		/* 2. STOP wins over everything */
		if (app_stop_reasons() != 0u) {
			motor_safe_off();
			if (cur.type != CMD_NONE) {
				result = MOT_RES_STOPPED;
				cur.type = CMD_NONE;
			}
			pi_l.integ = pi_r.integ = 0.0f;
			v_prof = 0.0f;
			goto publish;
		}

		/* Start the new command from where the wheels are now */
		if (fresh) {
			start_l = cl;
			start_r = cr;
			elapsed_ms = 0;
			v_prof = 0.0f;
			pi_l.integ = pi_r.integ = 0.0f;
			result = MOT_RES_BUSY;
			if (cur.type == CMD_DRIVE) {
				limit_ms = (UW)(2000 * absf((float)cur.amount) /
					absf((float)cur.speed)) + 1000;
			} else if (cur.type == CMD_TURN) {
				limit_ms = (UW)(2000.0f * 3.14159f * S2_TRACK_MM *
					absf((float)cur.amount) / 360.0f /
					(float)cur.speed) + 1000;
			} else if (cur.type == CMD_FOLLOW) {
				limit_ms = 0;		/* runs until replaced */
			} else {			/* HALT */
				result = MOT_RES_DONE;
				cur.type = CMD_NONE;
			}
		}

		/* 3. Run the command */
		if (cur.type == CMD_FOLLOW) {
			float	steer = (float)line_steer_mm_s();	/* + = steer left */
			float	t_l, t_r;

			v_prof += ACCEL_MM_S2 * DT_S;
			if (v_prof > absf((float)cur.speed)) v_prof = absf((float)cur.speed);
			t_l = v_prof - steer;
			t_r = v_prof + steer;
			if (t_l < 0.0f) t_l = 0.0f;
			if (t_r < 0.0f) t_r = 0.0f;
			duty_l = wheel_duty(&pi_l, t_l, vl);
			duty_r = wheel_duty(&pi_r, t_r, vr);
			motor_write(duty_l, duty_r);
		} else if (cur.type == CMD_DRIVE || cur.type == CMD_TURN) {
			float	target, done, remain, v_cap, sl, sr, corr;

			elapsed_ms += 10;
			dl_mm = (float)(cl - start_l) * S2_UM_PER_COUNT / 1000.0f;
			dr_mm = (float)(cr - start_r) * S2_UM_PER_COUNT / 1000.0f;

			if (cur.type == CMD_DRIVE) {
				target = absf((float)cur.amount);
				sl = (cur.speed >= 0) ? dl_mm : -dl_mm;
				sr = (cur.speed >= 0) ? dr_mm : -dr_mm;
			} else {
				/* spin: + = left = left wheel back, right forward */
				target = 3.14159f * S2_TRACK_MM * absf((float)cur.amount) / 360.0f;
				sl = (cur.amount >= 0) ? -dl_mm : dl_mm;
				sr = (cur.amount >= 0) ? dr_mm : -dr_mm;
			}
			done = 0.5f * (sl + sr);
			remain = target - done;

			if (remain <= 0.0f || elapsed_ms > limit_ms) {
				motor_safe_off();
				result = (remain <= 0.0f) ? MOT_RES_DONE : MOT_RES_TIMEOUT;
				cur.type = CMD_NONE;
				pi_l.integ = pi_r.integ = 0.0f;
				v_prof = 0.0f;
				goto publish;
			}

			/* Profile: ramp up, never faster than can stop in 'remain' */
			v_cap = app_sqrtf(2.0f * ACCEL_MM_S2 * remain);
			if (v_cap < V_MIN_MM_S) v_cap = V_MIN_MM_S;
			v_prof += ACCEL_MM_S2 * DT_S;
			if (v_prof > absf((float)cur.speed)) v_prof = absf((float)cur.speed);
			if (v_prof > v_cap) v_prof = v_cap;

			/* Keep both wheels at the same distance */
			corr = KEEP_EQUAL * (sl - sr);
			{
				float	t_l = v_prof - corr;
				float	t_r = v_prof + corr;
				float	m_l, m_r;

				if (t_l < 0.0f) t_l = 0.0f;
				if (t_r < 0.0f) t_r = 0.0f;
				/* measured speeds in the same "along the move" sense */
				if (cur.type == CMD_DRIVE) {
					m_l = (cur.speed >= 0) ? vl : -vl;
					m_r = (cur.speed >= 0) ? vr : -vr;
				} else {
					m_l = (cur.amount >= 0) ? -vl : vl;
					m_r = (cur.amount >= 0) ? vr : -vr;
				}
				duty_l = wheel_duty(&pi_l, t_l, m_l);
				duty_r = wheel_duty(&pi_r, t_r, m_r);
			}
			/* back to wheel directions */
			if (cur.type == CMD_DRIVE) {
				if (cur.speed < 0) {
					duty_l = -duty_l;
					duty_r = -duty_r;
				}
			} else if (cur.amount >= 0) {
				duty_l = -duty_l;
			} else {
				duty_r = -duty_r;
			}
			motor_write(duty_l, duty_r);
		} else {
			motor_write(0, 0);
		}

publish:
		APP_LOCK(s);
		if (!cmd_new) {		/* a newer command keeps its BUSY */
			status.result = result;
		}
		status.count_l = cl;
		status.count_r = cr;
		status.speed_l = (INT)vl;
		status.speed_r = (INT)vr;
		status.invalid = invalid;
		APP_UNLOCK(s);

		app_timing_done(tmg_motion);
	}
}
