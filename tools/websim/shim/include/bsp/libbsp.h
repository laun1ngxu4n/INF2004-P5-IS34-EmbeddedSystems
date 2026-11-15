/* websim shim for <bsp/libbsp.h>: the pin helpers the firmware uses */
#ifndef __BSP_LIBBSP_H__
#define __BSP_LIBBSP_H__

#include <tk/tkernel.h>

#define GPIO_MODE_OUT 1
#define GPIO_MODE_IN  0

ER gpio_set_pin(UINT no, UINT mode);
ER gpio_set_val(UINT no, UINT val);
UINT gpio_get_val(UINT no);
ER pwm_set_pin(UINT no);
ER pwm_set_wrap(UINT no, UW wrap);
ER pwm_set_cc(UINT no, UW cc);
ER pwm_set_enabled(UINT no, BOOL enable);

#endif
