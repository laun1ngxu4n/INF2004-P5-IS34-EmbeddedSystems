/*
 *  app_gpio.h
 *  The one GPIO edge-interrupt dispatcher.  Shared by the whole team.
 *
 *  The RP2040 has ONE interrupt (IO_IRQ_BANK0) for every GPIO pin, so
 *  one handler must serve all of them and hand each pin to its owner
 *  (TEAM.md section 3, "GPIO interrupts").  Each owner registers its pins
 *  in its subsystemN_init():
 *
 *      app_gpio_input(PIN_STOP, TRUE);
 *      app_gpio_irq_add(PIN_STOP, APP_GPIO_FALL, stop_isr);
 *
 *  The callback runs in interrupt context: keep it a few lines long, never
 *  wait, no tm_printf.  It may call tk_set_flg / tk_sig_sem / tk_wup_tsk.
 *  The WiFi driver polls and the USB console has its own interrupt, so
 *  nothing else in the template uses IO_IRQ_BANK0.
 */

#ifndef APP_GPIO_H
#define APP_GPIO_H

#include <tk/tkernel.h>

#define APP_GPIO_FALL 0x4u /* falling edge */
#define APP_GPIO_RISE 0x8u /* rising edge */
#define APP_GPIO_BOTH (APP_GPIO_FALL | APP_GPIO_RISE)

typedef void (*APP_GPIO_CB)(UINT pin, UW events);

/* Called once by usermain() before any subsystemN_init() */
IMPORT ER app_gpio_init (void);

/* Make pin an input, with or without the internal pull-up */
IMPORT ER app_gpio_input (UINT pin, BOOL pull_up);

/* Call cb on the given edges of pin.  One callback per pin. */
IMPORT ER app_gpio_irq_add (UINT pin, UW events, APP_GPIO_CB cb);

/* Take peripheral blocks out of reset (RESETS_RESET_PWM, _I2C1, ...).
   The kernel only releases what its own drivers use: with USE_PTMR = 0
   the PWM block stays in reset, so motors and servo must call this. */
IMPORT void app_hw_release (UW resets_mask);

/* Make pin a plain output (low) */
IMPORT ER app_gpio_output (UINT pin);

/* Current level of an input pin (0 or 1) */
#define app_gpio_get(pin) ((in_w(GPIO_IN) >> (pin)) & 1u)

#endif /* APP_GPIO_H */

/*** end of file ***/

