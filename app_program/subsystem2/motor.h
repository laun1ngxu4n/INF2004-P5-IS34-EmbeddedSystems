/**
 * @file    motor.h
 *  The motors and wheel encoders (motor.c): PWM, the encoder
 *  interrupt, the odometer.  Part of subsystem 2's interface
 *  (subsystem2.h includes it).
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "../app.h"

/** Motors off now.  Any context, also interrupt handlers; used by the
   STOP helper (app_stop.c) and, later, Safety (TEAM.md section 3). */
IMPORT void motor_safe_off (void);

/** Distance driven since boot, mean of both wheels, in encoder counts
   (0.080 mm each).  Any context, also interrupt handlers (barcode edges). */
IMPORT int32_t motor_odometer (void);

IMPORT ER   motor_init (void);
IMPORT void motor_write (int32_t duty_l,
                         int32_t duty_r); /* -1000..1000 per mille */
IMPORT void motor_encoders (int32_t * l, int32_t * r, uint32_t * invalid);
/** The duties last written, -1000..1000 per mille (0 after a safe-off). */
IMPORT void motor_duties (int32_t * p_left, int32_t * p_right);

#endif /* MOTOR_H */

/*** end of file ***/

