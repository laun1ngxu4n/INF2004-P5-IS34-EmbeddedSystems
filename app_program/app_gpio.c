/**
 * @file    app_gpio.c
 *  The shared GPIO edge-interrupt dispatcher (see app_gpio.h).
 */

#include "app.h"
#include "app_gpio.h"

#define INTNO_IO_BANK0  13 /* IO_IRQ_BANK0 */
#define INTPRI_IO_BANK0 1  /* above the kernel tick */

/* IO_BANK0 interrupt registers: 4 bits per pin, 8 pins per register */
#define IO_INT_REGS     4u /* GP0..GP29 */
#define IO_PINS_PER_REG 8u
#define IO_BITS_PER_PIN 4u

/** Interrupt registers of IO bank 0, 8 pins per register */
Inline uint32_t io_intr_reg (uint32_t reg)
{
    return IO_BANK0_BASE + 0x0F0u + (reg * 4u);
}
/** Address of the core 0 interrupt-enable register for pin bank reg. */
Inline uint32_t io_proc0_inte_reg (uint32_t reg)
{
    return IO_BANK0_BASE + 0x100u + (reg * 4u);
}
/** Address of the core 0 interrupt-status register for pin bank reg. */
Inline uint32_t io_proc0_ints_reg (uint32_t reg)
{
    return IO_BANK0_BASE + 0x120u + (reg * 4u);
}
#define IO_EDGE_BITS 0xCCCCCCCCu /* edge-low/edge-high of all 8 pins */

LOCAL app_gpio_cb_t g_gpio_cb[GPIO_NUM];

/** GPIO bank 0 interrupt: acknowledges each pin's edges and calls its
 * callback. */
LOCAL void gpio_bank0_isr (uint32_t intno)
{
    uint32_t ints;
    uint32_t reg;
    uint32_t pin;
    uint32_t slot;

    for (reg = 0; reg < IO_INT_REGS; reg++)
    {
        ints = in_w(io_proc0_ints_reg(reg));

        if (0u == ints)
        {
            continue;
        }

        /* Acknowledge first, so an edge during the callbacks is kept */
        out_w(io_intr_reg(reg), ints & IO_EDGE_BITS);

        for (slot = 0; slot < IO_PINS_PER_REG; slot++)
        {
            uint32_t events =
                (ints >> (slot * IO_BITS_PER_PIN)) & APP_GPIO_BOTH;
            pin = (reg * IO_PINS_PER_REG) + slot;

            if ((events != 0u) && (pin < GPIO_NUM) && (g_gpio_cb[pin] != NULL))
            {
                g_gpio_cb[pin](pin, events);
            }
        }
    }

    ClearInt(intno);
}

EXPORT ER app_gpio_init (void)
{
    T_DINT dint;
    ER     err;

    dint.intatr = TA_HLNG;
    /* T-Kernel keeps every handler as FP; with TA_HLNG it calls it with the
       interrupt number, gpio_bank0_isr's one parameter */
    dint.inthdr = (FP)gpio_bank0_isr;
    err         = tk_def_int(INTNO_IO_BANK0, &dint);

    if (err < E_OK)
    {
        return err;
    }

    EnableInt(INTNO_IO_BANK0, INTPRI_IO_BANK0);
    return E_OK;
}

EXPORT void app_gpio_hw_release (uint32_t resets_mask)
{
    clr_w(RESETS_RESET, resets_mask);

    while ((in_w(RESETS_RESET_DONE) & resets_mask) != resets_mask)
    {
        /* nothing: wait for the blocks to come out of reset */
    }
}

EXPORT ER app_gpio_output (uint32_t pin)
{
    gpio_set_val(pin, 0);
    return gpio_set_pin(pin, GPIO_MODE_OUT);
}

EXPORT ER app_gpio_input (uint32_t pin, bool b_pull_up)
{
    ER err;

    err = gpio_set_pin(pin, GPIO_MODE_IN);

    if (err < E_OK)
    {
        return err;
    }

    clr_w(GPIO(pin),
          GPIO_PUE | GPIO_PDE); /* the pad resets with pull-down on */

    if (b_pull_up)
    {
        set_w(GPIO(pin), GPIO_PUE);
    }

    return E_OK;
}

EXPORT ER app_gpio_irq_add (uint32_t pin, uint32_t events,
                            app_gpio_cb_t p_callback)
{
    uint32_t irq_state;
    uint32_t bits;

    if ((pin >= GPIO_NUM) || (NULL == p_callback) ||
        ((events & ~APP_GPIO_BOTH) != 0u))
    {
        return E_PAR;
    }

    bits = events << ((pin % 8) * 4);
    APP_LOCK(irq_state);

    if (g_gpio_cb[pin] != NULL)
    {
        APP_UNLOCK(irq_state);
        return E_OBJ; /* pin already has an owner */
    }

    g_gpio_cb[pin] = p_callback;
    out_w(io_intr_reg(pin / 8), bits); /* drop stale edges */
    set_w(io_proc0_inte_reg(pin / 8), bits);
    APP_UNLOCK(irq_state);
    return E_OK;
}

/*** end of file ***/

