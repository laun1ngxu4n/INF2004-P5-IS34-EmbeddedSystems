/* Minimal host stand-ins for the Pico SDK / lwIP calls used by the tests. */
#ifndef SDK_STUB_H
#define SDK_STUB_H
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
typedef unsigned int uint;
typedef uint64_t absolute_time_t;
typedef struct { volatile uint32_t timerawl; } stub_timer_t;
extern stub_timer_t stub_timer;
#define timer_hw (&stub_timer)
extern uint64_t stub_now_us;
static inline absolute_time_t get_absolute_time(void) { return stub_now_us; }
static inline uint32_t to_ms_since_boot(absolute_time_t t) { return (uint32_t)(t / 1000u); }
static inline uint64_t time_us_64(void) { return stub_now_us; }
static inline absolute_time_t delayed_by_ms(absolute_time_t t, uint32_t ms) { return t + ms * 1000u; }
static inline int64_t absolute_time_diff_us(absolute_time_t a, absolute_time_t b) { return (int64_t)(b - a); }
static inline void sleep_until(absolute_time_t t) { (void)t; }
static inline void sleep_ms(uint32_t ms) { stub_now_us += ms * 1000u; }
static inline void sleep_us(uint64_t us) { stub_now_us += us; }
static inline void busy_wait_us_32(uint32_t us) { stub_now_us += us; }
static inline bool stdio_init_all(void) { return true; }
static inline void __dmb(void) {}
static inline uint32_t save_and_disable_interrupts(void) { return 0u; }
static inline void restore_interrupts(uint32_t s) { (void)s; }
extern bool stub_gpio_level[32];
static inline bool gpio_get(uint pin) { return stub_gpio_level[pin]; }
static inline void gpio_init(uint p) { (void)p; }
static inline void gpio_set_dir(uint p, bool o) { (void)p; (void)o; }
static inline void gpio_pull_up(uint p) { (void)p; }
static inline void gpio_disable_pulls(uint p) { (void)p; }
typedef void (*gpio_irq_callback_t)(uint gpio, uint32_t events);
static inline void gpio_set_irq_enabled_with_callback(uint p, uint32_t e, bool en, gpio_irq_callback_t cb)
{ (void)p; (void)e; (void)en; (void)cb; }
static inline void gpio_set_irq_enabled(uint p, uint32_t e, bool en) { (void)p; (void)e; (void)en; }
#define GPIO_IN 0
#define GPIO_IRQ_EDGE_FALL 0x4u
#define GPIO_IRQ_EDGE_RISE 0x8u
void tight_loop_contents(void);
#endif
