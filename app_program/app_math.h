/**
 * @file    app_math.h
 *  Small maths for the whole team: the build is freestanding (no libm).
 *
 *  app_sqrtf: the start guess is scaled to the right power of 2 first, so
 *  6 Newton steps are exact over any range.  (A fixed 12 steps from x was
 *  far off for large x, e.g. IMU raw values: sqrt(6.7e7) gave 17,700
 *  instead of 8,185.  Found in review, 1 Nov.)
 */

#ifndef APP_MATH_H
#define APP_MATH_H

#include "app_types.h"

/* app_sqrtf: the value is scaled by 4s into APP_SQRT_LO .. APP_SQRT_HI
   (the root by 2s), where APP_SQRT_STEPS Newton steps from 1 reach
   full precision */
#define APP_SQRT_HI    4.0f
#define APP_SQRT_LO    0.25f
#define APP_SQRT_STEPS 6

/** Square root (no libm): scaled guess, then Newton steps. */
Inline float32_t app_sqrtf (float32_t value)
{
    float32_t root   = 1.0f;
    float32_t scaled = value;
    int32_t   iter;

    if (value <= 0.0f)
    {
        return 0.0f;
    }

    while (scaled >= APP_SQRT_HI)
    {
        scaled *= 0.25f;
        root *= 2.0f;
    }

    while (scaled < APP_SQRT_LO)
    {
        scaled *= 4.0f;
        root *= 0.5f;
    }

    for (iter = 0; iter < APP_SQRT_STEPS; iter++)
    {
        root = 0.5f * (root + (value / root));
    }

    return root;
}

/** sin of an angle in degrees, |deg| <= 90 (Taylor to x^9) */
Inline float32_t app_sin_deg (float32_t deg)
{
    float32_t rad    = deg * 3.14159265f / 180.0f;
    float32_t rad_sq = rad * rad;

    return rad *
           (1.0f -
            (rad_sq / 6.0f *
             (1.0f - (rad_sq / 20.0f *
                      (1.0f - (rad_sq / 42.0f * (1.0f - (rad_sq / 72.0f))))))));
}

/** Absolute value of a float. */
Inline float32_t app_absf (float32_t value)
{
    return (value < 0.0f) ? -value : value;
}

#endif /* APP_MATH_H */

/*** end of file ***/

