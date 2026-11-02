/*
 * enc_timing.h - encoder edge timing helpers (stall rule, longest interval).
 *
 * Pure logic, tested on the host by host_tests/test_enc_timing.c.
 *
 * 25 Oct review finding 04: the main loop read the time first and the
 * ISR-written last-edge time second. An edge in between made the last edge
 * "newer" than now, the unsigned subtraction wrapped, and the stall rule saw
 * a ~4,295 s gap. Fix: read both last-edge times and THEN the timer inside one
 * interrupts-off section (so now >= last edge), and clamp a negative age to
 * zero as a second line of defence.
 *
 * Finding 14: the main loop only samples the age of the latest edge about
 * once per ms, so a long interval that ends between samples was missed. The
 * ISR now records the longest completed interval itself.
 */
#ifndef ENC_TIMING_H
#define ENC_TIMING_H

#include <stdbool.h>
#include <stdint.h>

/* Age of the last edge at now_us. A last edge that is newer than now (it
 * arrived between two reads) gives 0, never a huge wrapped value. Valid for
 * ages below 2^31 us (about 35 min). */
static inline uint32_t enc_edge_age_us(uint32_t now_us, uint32_t last_edge_us)
{
    uint32_t age = now_us - last_edge_us;
    return ((int32_t)age < 0) ? 0u : age;
}

/* Design stall rule (Week 6, 6.2): PWM above min_duty and no count change
 * for at least limit_ms. */
static inline bool enc_stall_tripped(uint32_t duty_pct, uint32_t min_duty_pct, uint32_t age_us,
                                     uint32_t limit_ms)
{
    return (duty_pct > min_duty_pct) && (age_us >= (limit_ms * 1000u));
}

/* Called by the ISR on every valid count change, BEFORE it stores t_edge_us
 * as the new last edge. Returns the updated longest completed interval. */
static inline uint32_t enc_interval_note(uint32_t max_iv_us, uint32_t prev_edge_us,
                                         uint32_t t_edge_us, bool armed)
{
    uint32_t iv = t_edge_us - prev_edge_us;
    return (armed && (iv > max_iv_us)) ? iv : max_iv_us;
}

#endif /* ENC_TIMING_H */
