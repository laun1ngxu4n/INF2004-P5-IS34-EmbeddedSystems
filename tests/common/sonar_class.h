/*
 * sonar_class.h - one HC-SR04 echo-width classifier for every program.
 *
 * 25 Oct review finding 13: parts_check accepted any completed pulse over
 * 100 us, while ultrasonic_test also rejected pulses over 25,000 us, so the
 * two tools disagreed on the same echo. Both now use this function.
 * Tested on the host by host_tests/test_sonar_class.c.
 */
#ifndef SONAR_CLASS_H
#define SONAR_CLASS_H

#include <stdint.h>

#define SONAR_MIN_VALID_US   100u     /* < ~17 mm: below the module's 20 mm minimum */
#define SONAR_LONG_PULSE_US  25000u   /* > ~4.3 m: completed but beyond useful range */

typedef enum
{
    SONAR_WIDTH_VALID = 0,
    SONAR_WIDTH_TOO_SHORT,   /* glitch, or rise and fall seen in one event */
    SONAR_WIDTH_LONG         /* completed but too long to be a real target */
} sonar_width_t;

static inline sonar_width_t sonar_classify_width(uint32_t width_us)
{
    if (width_us < SONAR_MIN_VALID_US)
    {
        return SONAR_WIDTH_TOO_SHORT;
    }
    if (width_us > SONAR_LONG_PULSE_US)
    {
        return SONAR_WIDTH_LONG;
    }
    return SONAR_WIDTH_VALID;
}

#endif /* SONAR_CLASS_H */
