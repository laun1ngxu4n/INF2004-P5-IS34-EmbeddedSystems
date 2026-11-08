/*
 *  app_gpio.c
 *  The shared GPIO edge-interrupt dispatcher (see app_gpio.h).
 */

#include "app.h"
#include "app_gpio.h"

#define INTNO_IO_BANK0  13 /* IO_IRQ_BANK0 */
#define INTPRI_IO_BANK0 1  /* above the kernel tick */

/* IO_BANK0 interrupt registers: 4 bits per pin, 8 pins per register */
#define IO_INTR(n)       (IO_BANK0_BASE + 0x0F0 + ((n) * 4))
#define IO_PROC0_INTE(n) (IO_BANK0_BASE + 0x100 + ((n) * 4))
#define IO_PROC0_INTS(n) (IO_BANK0_BASE + 0x120 + ((n) * 4))
#define IO_EDGE_BITS     0xCCCCCCCCu /* edge-low/edge-high of all 8 pins */

LOCAL APP_GPIO_CB gpio_cb[GPIO_NUM];

LOCAL void gpio_inthdr (UINT intno)
{
    UW   ints;
    UINT reg;
    UINT pin;
    UINT sh;

    for (reg = 0; reg < 4; reg++)
    {
        ints = in_w(IO_PROC0_INTS(reg));
        if (0u == ints)
        {
            continue;
        }
        /* Acknowledge first, so an edge during the callbacks is kept */
        out_w(IO_INTR(reg), ints & IO_EDGE_BITS);
        for (sh = 0; sh < 8; sh++)
        {
            UW ev = (ints >> (sh * 4)) & APP_GPIO_BOTH;
            pin   = (reg * 8) + sh;
            if ((ev != 0u) && (pin < GPIO_NUM) && (gpio_cb[pin] != NULL))
            {
                gpio_cb[pin](pin, ev);
            }
        }
    }
    ClearInt(intno);
}

EXPORT ER app_gpio_init (void)
{
    T_DINT dint;
    ER     er;

    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)gpio_inthdr;
    er          = tk_def_int(INTNO_IO_BANK0, &dint);
    if (er < E_OK)
    {
        return er;
    }
    EnableInt(INTNO_IO_BANK0, INTPRI_IO_BANK0);
    return E_OK;
}

EXPORT void app_hw_release (UW resets_mask)
{
    clr_w(RESETS_RESET, resets_mask);
    while ((in_w(RESETS_RESET_DONE) & resets_mask) != resets_mask)
    {
        ;
    }
}

EXPORT ER app_gpio_output (UINT pin)
{
    gpio_set_val(pin, 0);
    return gpio_set_pin(pin, GPIO_MODE_OUT);
}

EXPORT ER app_gpio_input (UINT pin, BOOL pull_up)
{
    ER er;

    er = gpio_set_pin(pin, GPIO_MODE_IN);
    if (er < E_OK)
    {
        return er;
    }
    clr_w(GPIO(pin),
          GPIO_PUE | GPIO_PDE); /* the pad resets with pull-down on */
    if (pull_up)
    {
        set_w(GPIO(pin), GPIO_PUE);
    }
    return E_OK;
}

EXPORT ER app_gpio_irq_add (UINT pin, UW events, APP_GPIO_CB cb)
{
    UINT s;
    UW   bits;

    if ((pin >= GPIO_NUM) || (NULL == cb) || ((events & ~APP_GPIO_BOTH) != 0u))
    {
        return E_PAR;
    }
    bits = events << ((pin % 8) * 4);
    APP_LOCK(s);
    if (gpio_cb[pin] != NULL)
    {
        APP_UNLOCK(s);
        return E_OBJ; /* pin already has an owner */
    }
    gpio_cb[pin] = cb;
    out_w(IO_INTR(pin / 8), bits); /* drop stale edges */
    set_w(IO_PROC0_INTE(pin / 8), bits);
    APP_UNLOCK(s);
    return E_OK;
}

/*** end of file ***/

