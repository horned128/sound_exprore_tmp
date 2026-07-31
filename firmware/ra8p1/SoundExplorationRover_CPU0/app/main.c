/*
 * usermain.c
 * μT-Kernel 3.0 + Renesas RA BSP Blinky example
 */

#include "hal_data.h"
#include <tk/tkernel.h>

extern bsp_leds_t g_bsp_leds;

/*
 * μT-Kernel application entry point.
 *
 * This function is called after μT-Kernel starts.
 */
EXPORT INT usermain(void)
{
    /* LED type structure */
    bsp_leds_t leds = g_bsp_leds;

    /* Wake up 2nd core if this is first core and we are inside a multicore project. */
#if (0 == _RA_CORE) && (1 == BSP_MULTICORE_PROJECT) && !BSP_TZ_NONSECURE_BUILD
    R_BSP_SecondaryCoreStart();
#endif

    /* If this board has no LEDs then trap here */
    if (0 == leds.led_count)
    {
        while (1)
        {
            tk_dly_tsk(1000);
        }
    }

    /* Holds level to set for pins */
    bsp_io_level_t pin_level = BSP_IO_LEVEL_LOW;

    while (1)
    {
        /*
         * Enable access to the PFS registers.
         * If using r_ioport module then register protection is automatically
         * handled. This code uses BSP IO functions directly.
         */
        R_BSP_PinAccessEnable();

#if BSP_NUMBER_OF_CORES == 1

        /* Update all board LEDs */
        for (uint32_t i = 0; i < leds.led_count; i++)
        {
            uint32_t pin = leds.p_leds[i];

            R_BSP_PinWrite((bsp_io_port_pin_t) pin, pin_level);
        }

#else

        /* Update LED that is at the index of this core. */
        R_BSP_PinWrite((bsp_io_port_pin_t) leds.p_leds[_RA_CORE], pin_level);

#endif

        /* Protect PFS registers */
        R_BSP_PinAccessDisable();

        /* Toggle level for next write */
        if (BSP_IO_LEVEL_LOW == pin_level)
        {
            pin_level = BSP_IO_LEVEL_HIGH;
        }
        else
        {
            pin_level = BSP_IO_LEVEL_LOW;
        }

        /*
         * μT-Kernel task delay.
         * 500 ms ON + 500 ms OFF = 1 Hz blink.
         */
        tk_dly_tsk(1000);
    }

    return 0;
}
