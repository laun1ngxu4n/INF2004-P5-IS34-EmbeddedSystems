/*
 * websim shim for <tk/tkernel.h>
 *
 * The firmware in app_program/ is compiled UNCHANGED against this header
 * instead of the real micro T-Kernel one.  It reuses the real type,
 * error-code and system-call headers from include/tk/, and replaces
 * everything that touches Cortex-M0+ or RP2040 hardware (interrupt masks,
 * register access) with calls into the simulator.
 */
#ifndef __TK_TKERNEL_H__
#define __TK_TKERNEL_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define CNF_TIMER_PERIOD 10 /* config/config.h: kernel tick, ms */
#define CNF_SMP          0
#define TK_SUPPORT_SMP   0

#include <tk/typedef.h>
#include <tk/errno.h>
#define TARGET_DIR pico_rp2040
#include <tk/syscall.h>

#ifndef Inline
#define Inline static inline
#endif

#define SIM_IMPORT(name) __attribute__((import_module("env"), import_name(name)))

/* ---- the simulator (src/ in tools/websim/web) --------------------------- */
SIM_IMPORT("hw_read") uint32_t hw_read(uint32_t addr);
SIM_IMPORT("hw_write") void hw_write(uint32_t addr, uint32_t val);
SIM_IMPORT("hw_set") void hw_set(uint32_t addr, uint32_t bits);
SIM_IMPORT("hw_clr") void hw_clr(uint32_t addr, uint32_t bits);
SIM_IMPORT("sim_yield") void sim_yield(void);

/* Register access (the real in_w/out_w dereference an address) */
Inline void out_w(UW port, UW data) { hw_write(port, data); }
Inline UW   in_w(UW port) { return hw_read(port); }
Inline void set_w(UW port, UW bits) { hw_set(port, bits); }
Inline void clr_w(UW port, UW bits) { hw_clr(port, bits); }

/* Interrupt masking: the simulator never interrupts a running task, so
   these only need to keep the calling convention. */
#define DI(intsts) ((intsts) = 0u)
#define EI(intsts) ((void)(intsts))
Inline void EnableInt(UINT intno, INT level) { (void)intno; (void)level; }
Inline void DisableInt(UINT intno) { (void)intno; }
Inline void ClearInt(UINT intno) { (void)intno; }

/* ---- register map (RP2040 datasheet; same values as sysdef.h) ----------- */
#define GPIO_NUM        30
#define IO_BANK0_BASE   0x40014000
#define GPIO_CTRL(n)    (IO_BANK0_BASE + 0x04 + ((n) * 8))
#define GPIO_CTRL_FUNCSEL_I2C 3
#define GPIO_IN         0xD0000004
#define PADS_BANK0_BASE 0x4001C000
#define GPIO(n)         (PADS_BANK0_BASE + 0x04 + ((n) * 4))
#define GPIO_OD         (1 << 7)
#define GPIO_IE         (1 << 6)
#define GPIO_PUE        (1 << 3)
#define GPIO_PDE        (1 << 2)
#define RESETS_BASE     0x4000C000
#define RESETS_RESET      (RESETS_BASE + 0x0)
#define RESETS_RESET_DONE (RESETS_BASE + 0x8)
#define RESETS_RESET_PWM  0x00004000
#define RESETS_RESET_I2C1 0x00000010
#define RESETS_RESET_ADC  0x00000001
#define PWM_BASE        0x40050000
#define PWM_CHx_CSR     0x00
#define PWM_CHx_DIV     0x04
#define PWM_CHx_CC      0x0C
#define PWM_CHx_TOP     0x10
#define I2C1_BASE       0x40048000

/* Which kernel entry points exist is decided by kernel.c; the rest of
   <tk/syscall.h> is declared but unused by app_program/. */

#endif /* __TK_TKERNEL_H__ */
