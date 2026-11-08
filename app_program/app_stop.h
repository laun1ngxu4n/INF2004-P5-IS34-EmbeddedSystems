/**
 * @file    app_stop.h
 *  The one STOP helper (TEAM.md section 5, IF-02 / IF-05).  Shared by the
 *  whole team.
 *
 *  Anything that must stop the robot calls app_stop_request() with its
 *  reason bit: the motors go off at once, from any context (task, cyclic
 *  or interrupt handler), and stay off while any reason bit is set.
 *  Motion cancels its command when it sees a reason; whoever owns a
 *  reason clears it (STOP button: released and START pressed).
 */

#ifndef APP_STOP_H
#define APP_STOP_H

#include <tk/tkernel.h>

#define STOP_BUTTON 0x0001u /* GP21 pressed (S5) */
#define STOP_MQTT   0x0002u /* STOP from the PC (S1, later) */
#define STOP_FAULT  0x0004u /* Safety fault (S4, later) */

#define STOP_BITS 3 /* the STOP reasons' width */

/* Faults (Change Requirement 1): every fault condition is one bit of one
   shared fault word instead of a variable of its own.  Tasks and
   interrupts set and clear bits, so every change goes through
   app_stop_fault_set() / app_stop_fault_clear() (interrupts off: the Cortex-M0+
   has no atomic read-modify-write).  Each bit has one owner that sets and
   clears it. */
#define FAULT_MOTOR_STALL \
    0x01u /* a wheel driven hard does not turn (S4 Safety) */
#define FAULT_ENCODER   0x02u /* encoder edges missed too often (S4 Safety) */
#define FAULT_IMU       0x04u /* the IMU does not answer (S4 IMU) */
#define FAULT_NO_ECHO   0x08u /* sonar: 5 pings in a row, no ECHO (S5) */
#define FAULT_LINE_ONE  0x10u /* one line sensor black too long (S3) */
#define FAULT_LINE_BOTH 0x20u /* both line sensors black too long (S3) */
#define FAULT_WIFI_LOSS 0x40u /* no MQTT link (S1) */
#define FAULT_MOTION_HUNG \
    0x80u            /* the Motion task stopped running (S4 Safety) */
#define FAULT_BITS 8 /* the fault word's width */
/* Warnings: never stop the robot by themselves.  Wi-Fi loss never stops it
   (Week 6 rule, Tribunal 1); the encoder and IMU bits are reported. */
#define FAULT_WARNINGS (FAULT_ENCODER | FAULT_IMU | FAULT_WIFI_LOSS)

IMPORT void     app_stop_request (uint32_t reason); /* any context */
IMPORT void     app_stop_clear (uint32_t reason);
IMPORT uint32_t app_stop_reasons (void); /* 0 = free to move */
IMPORT uint32_t
app_stop_time_us (void); /* app_timing_now_us() of the last request */

IMPORT void     app_stop_fault_set (uint32_t mask);   /* any context */
IMPORT void     app_stop_fault_clear (uint32_t mask); /* any context */
IMPORT uint32_t app_stop_faults (void); /* FAULT_... bits set now */

#endif /* APP_STOP_H */

/*** end of file ***/

