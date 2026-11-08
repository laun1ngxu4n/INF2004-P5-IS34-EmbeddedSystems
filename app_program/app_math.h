/*
 *	app_math.h
 *	Small maths for the whole team: the build is freestanding (no libm).
 *
 *	app_sqrtf: the start guess is scaled to the right power of 2 first, so
 *	6 Newton steps are exact over any range.  (A fixed 12 steps from x was
 *	far off for large x, e.g. IMU raw values: sqrt(6.7e7) gave 17,700
 *	instead of 8,185.  Found in review, 1 Nov.)
 */

#ifndef APP_MATH_H
#define APP_MATH_H

Inline float app_sqrtf(float x)
{
	float	r = 1.0f;
	float s = x;
	int	i;

	if (x <= 0.0f) {
		return 0.0f;
	}
	while (s >= 4.0f) {
		s *= 0.25f;
		r *= 2.0f;
	}
	while (s < 0.25f) {
		s *= 4.0f;
		r *= 0.5f;
	}
	for (i = 0; i < 6; i++) {
		r = 0.5f * (r + (x / r));
	}
	return r;
}

/* sin of an angle in degrees, |deg| <= 90 (Taylor to x^9) */
Inline float app_sin_deg(float deg)
{
	float x = deg * 3.14159265f / 180.0f;
	float x2 = x * x;

	return x * (1.0f - (x2 / 6.0f * (1.0f - (x2 / 20.0f * (1.0f - (x2 / 42.0f *
		(1.0f - (x2 / 72.0f))))))));
}

Inline float app_absf(float x)
{
	return (x < 0.0f) ? -x : x;
}

#endif /* APP_MATH_H */
