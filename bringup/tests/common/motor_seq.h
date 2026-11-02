/*
 * motor_seq.h - STOP-safe motor test steps for the bring-up programs.
 *
 * Pure logic (no SDK calls), so host_tests/test_motor_seq.c can run it with
 * scripted button timing. The program supplies the hardware through
 * motor_hal_t.
 *
 * Rules (25 Oct review finding 03):
 *   - STOP is checked BEFORE every non-zero drive, including the first one.
 *   - Every wait inside a sequence (start delay, run, spin-down gap) checks
 *     STOP every millisecond, so a short press is never forgotten.
 *   - stop_requested() must report a LATCHED press (set in the GPIO
 *     interrupt) as well as a held button.
 *   - Once STOP is seen the step returns false and the caller must abort the
 *     whole sequence; a new sequence needs a fresh start press.
 *
 * Bench test only: this is not the independent safe-off path NFR10 needs.
 */
#ifndef MOTOR_SEQ_H
#define MOTOR_SEQ_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    void     (*p_drive)(uint32_t wheel, bool forward, uint32_t duty_pct);
    void     (*p_off)(void);
    bool     (*p_stop_requested)(void);
    uint32_t (*p_now_ms)(void);
    void     (*p_sleep_ms)(uint32_t ms);
    void     (*p_on_tick)(void);   /* optional, called once per ms while waiting */
} motor_hal_t;

/* Waits ms milliseconds. Returns false (motors off) as soon as STOP is seen. */
static inline bool motor_wait_or_stop(const motor_hal_t *p_hal, uint32_t ms)
{
    uint32_t t0 = p_hal->p_now_ms();

    for (;;)
    {
        if (p_hal->p_stop_requested())
        {
            p_hal->p_off();
            return false;
        }
        if ((p_hal->p_now_ms() - t0) >= ms)
        {
            return true;
        }
        if (p_hal->p_on_tick != 0)
        {
            p_hal->p_on_tick();
        }
        p_hal->p_sleep_ms(1u);
    }
}

/* One motor step: drive for run_ms, then off and wait spin_down_ms.
 * Never drives if STOP is already requested. Returns false if STOP was seen
 * at any point (motors are off). */
static inline bool motor_run_step(const motor_hal_t *p_hal, uint32_t wheel, bool forward,
                                  uint32_t duty_pct, uint32_t run_ms, uint32_t spin_down_ms)
{
    if (p_hal->p_stop_requested())
    {
        p_hal->p_off();
        return false;
    }
    p_hal->p_drive(wheel, forward, duty_pct);
    if (!motor_wait_or_stop(p_hal, run_ms))
    {
        return false;
    }
    p_hal->p_off();
    return motor_wait_or_stop(p_hal, spin_down_ms);
}

#endif /* MOTOR_SEQ_H */
