/**
 * @file hal_entry.c
 * @brief CPU1 application entry point.
 */
#include "hal_data.h"
#include "config/actuator_config.h"
#include "drivers/servo.h"

#if !SERVO_DEMO_ENABLE
#include "app/actuator_app.h"
#endif

extern bsp_leds_t g_bsp_leds;

#define CPU1_STATUS_LED_INDEX (2U)                          /* 赤LED。青・緑はCPU0思考タスク専用 */

#if SERVO_DEMO_ENABLE

volatile fsp_err_t g_servo_demo_last_error = FSP_SUCCESS;

static void servo_demo_error_loop(bsp_leds_t const * p_leds)
{
    bsp_io_level_t level = BSP_IO_LEVEL_LOW;

    while (1)
    {
        if ((NULL != p_leds) && (p_leds->led_count > CPU1_STATUS_LED_INDEX))
        {
            (void) g_ioport.p_api->pinWrite(g_ioport.p_ctrl,
                                             p_leds->p_leds[CPU1_STATUS_LED_INDEX],
                                             level);
            level = (BSP_IO_LEVEL_LOW == level) ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW;
        }
        R_BSP_SoftwareDelay(100U, BSP_DELAY_UNITS_MILLISECONDS);
    }
}

static void servo_demo_run(bsp_leds_t const * p_leds)
{
    g_servo_demo_last_error = servo_init();
    if (FSP_SUCCESS != g_servo_demo_last_error)
    {
        servo_demo_error_loop(p_leds);
    }

    int16_t angle = SERVO_DEMO_MIN_DEG;
    int16_t step = SERVO_DEMO_STEP_DEG;

    while (1)
    {
        for (uint32_t i = 0U; i < SERVO_COUNT; i++)
        {
            g_servo_demo_last_error = servo_set_target_deg(i, angle);
            if (FSP_SUCCESS != g_servo_demo_last_error)
            {
                servo_demo_error_loop(p_leds);
            }
        }

        if (angle >= SERVO_DEMO_MAX_DEG)
        {
            step = -SERVO_DEMO_STEP_DEG;
        }
        else if (angle <= SERVO_DEMO_MIN_DEG)
        {
            step = SERVO_DEMO_STEP_DEG;
        }
        angle = (int16_t) (angle + step);

        R_BSP_SoftwareDelay(SERVO_DEMO_INTERVAL_MS, BSP_DELAY_UNITS_MILLISECONDS);
    }
}

#endif

void hal_entry(void)
{
#if BSP_TZ_SECURE_BUILD
    R_BSP_NonSecureEnter();
#endif

    bsp_leds_t const leds = g_bsp_leds;

#if SERVO_DEMO_ENABLE
    /* Directly drive all four PWM outputs; no CPU0 IPC command is required. */
    servo_demo_run(&leds);
#else
    (void) actuator_app_init();

    bsp_io_level_t led_level = BSP_IO_LEVEL_LOW;
    uint32_t heartbeat_ms = 0U;

    while (1)
    {
        actuator_app_run_1ms();

        if (FSP_SUCCESS != g_actuator_last_error)
        {
            heartbeat_ms += 9U;
        }

        heartbeat_ms++;
        if ((leds.led_count > CPU1_STATUS_LED_INDEX) && (heartbeat_ms >= 500U))
        {
            (void) g_ioport.p_api->pinWrite(g_ioport.p_ctrl,
                                             leds.p_leds[CPU1_STATUS_LED_INDEX],
                                             led_level);
            led_level = (BSP_IO_LEVEL_LOW == led_level) ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW;
            heartbeat_ms = 0U;
        }

        R_BSP_SoftwareDelay(ACTUATOR_LOOP_PERIOD_MS, BSP_DELAY_UNITS_MILLISECONDS);
    }
#endif
}
