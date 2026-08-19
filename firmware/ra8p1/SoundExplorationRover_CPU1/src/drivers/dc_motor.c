/** =================================================================*
 * @file   dc_motor.c
 * @brief  Two-side BTS7960 open-loop motor control
 *
 * Each BTS7960 module receives independent RPWM and LPWM signals.  Only one
 * direction input is driven at a time; the other direction is held at zero.
 * ================================================================= */
#include "dc_motor.h"
#include "../config/actuator_config.h"

volatile int16_t g_drive_left_duty_permille = 0;
volatile int16_t g_drive_right_duty_permille = 0;

static int16_t g_left_target_rpm;
static int16_t g_right_target_rpm;
static int16_t g_left_target_duty_permille;
static int16_t g_right_target_duty_permille;
static uint32_t g_pwm_update_elapsed_ms;
static bool g_pwm_running;

static int16_t motor_rpm_to_duty_permille(int16_t target_rpm, int8_t forward_sign)
{
    int32_t const signed_rpm = (int32_t) target_rpm * (int32_t) forward_sign;
    if (0 == signed_rpm)
    {
        return 0;
    }

    int32_t const magnitude = (signed_rpm < 0) ? -signed_rpm : signed_rpm;
    int32_t duty = (magnitude * 1000) / JGA25_TARGET_RPM_MAX;
    if (duty < MOTOR_PWM_MIN_DUTY_PERMILLE)
    {
        duty = MOTOR_PWM_MIN_DUTY_PERMILLE;
    }
    if (duty > MOTOR_PWM_MAX_DUTY_PERMILLE)
    {
        duty = MOTOR_PWM_MAX_DUTY_PERMILLE;
    }

    return (signed_rpm < 0) ? (int16_t) -duty : (int16_t) duty;
}

static int16_t motor_ramp_value(int16_t current, int16_t target)
{
    if (current < target)
    {
        int32_t next = (int32_t) current + MOTOR_PWM_RAMP_PER_MS;
        return (int16_t) ((next > target) ? target : next);
    }
    if (current > target)
    {
        int32_t next = (int32_t) current - MOTOR_PWM_RAMP_PER_MS;
        return (int16_t) ((next < target) ? target : next);
    }

    return current;
}

static fsp_err_t motor_pwm_apply(void)
{
#if SEROV_ENABLE_DRIVE_MOTORS
    timer_info_t rpwm_info = {0};
    timer_info_t lpwm_info = {0};
    fsp_err_t err = MOTOR_RPWM_INSTANCE->p_api->infoGet(MOTOR_RPWM_INSTANCE->p_ctrl, &rpwm_info);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = MOTOR_LPWM_INSTANCE->p_api->infoGet(MOTOR_LPWM_INSTANCE->p_ctrl, &lpwm_info);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    int16_t const left_duty = g_drive_left_duty_permille;
    int16_t const right_duty = g_drive_right_duty_permille;
    uint16_t const left_magnitude = (uint16_t) ((left_duty < 0) ? -left_duty : left_duty);
    uint16_t const right_magnitude = (uint16_t) ((right_duty < 0) ? -right_duty : right_duty);
    uint16_t const left_rpwm = (left_duty > 0) ? left_magnitude : 0U;
    uint16_t const left_lpwm = (left_duty < 0) ? left_magnitude : 0U;
    uint16_t const right_rpwm = (right_duty > 0) ? right_magnitude : 0U;
    uint16_t const right_lpwm = (right_duty < 0) ? right_magnitude : 0U;

    uint32_t const left_rpwm_counts = (uint32_t) (((uint64_t) rpwm_info.period_counts * left_rpwm) / 1000U);
    uint32_t const right_rpwm_counts = (uint32_t) (((uint64_t) rpwm_info.period_counts * right_rpwm) / 1000U);
    uint32_t const left_lpwm_counts = (uint32_t) (((uint64_t) lpwm_info.period_counts * left_lpwm) / 1000U);
    uint32_t const right_lpwm_counts = (uint32_t) (((uint64_t) lpwm_info.period_counts * right_lpwm) / 1000U);

    /* Clear all four outputs first.  This makes a direction change break
     * before-make: the old RPWM/LPWM pulse is removed before the new one is
     * applied, so both inputs of one BTS7960 are never high together. */
    err = MOTOR_RPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_RPWM_INSTANCE->p_ctrl,
                                                    0U,
                                                    MOTOR_LEFT_RPWM_OUTPUT);
    if (FSP_SUCCESS == err)
    {
        err = MOTOR_RPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_RPWM_INSTANCE->p_ctrl,
                                                        0U,
                                                        MOTOR_RIGHT_RPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err)
    {
        err = MOTOR_LPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_LPWM_INSTANCE->p_ctrl,
                                                        0U,
                                                        MOTOR_LEFT_LPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err)
    {
        err = MOTOR_LPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_LPWM_INSTANCE->p_ctrl,
                                                        0U,
                                                        MOTOR_RIGHT_LPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err)
    {
        err = MOTOR_RPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_RPWM_INSTANCE->p_ctrl,
                                                        left_rpwm_counts,
                                                        MOTOR_LEFT_RPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err)
    {
        err = MOTOR_RPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_RPWM_INSTANCE->p_ctrl,
                                                        right_rpwm_counts,
                                                        MOTOR_RIGHT_RPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err)
    {
        err = MOTOR_LPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_LPWM_INSTANCE->p_ctrl,
                                                        left_lpwm_counts,
                                                        MOTOR_LEFT_LPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err)
    {
        err = MOTOR_LPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_LPWM_INSTANCE->p_ctrl,
                                                        right_lpwm_counts,
                                                        MOTOR_RIGHT_LPWM_OUTPUT);
    }

    bool const should_run = (0 != left_magnitude) || (0 != right_magnitude);
    if ((FSP_SUCCESS == err) && should_run && !g_pwm_running)
    {
        err = MOTOR_RPWM_INSTANCE->p_api->start(MOTOR_RPWM_INSTANCE->p_ctrl);
        if (FSP_SUCCESS == err)
        {
            err = MOTOR_LPWM_INSTANCE->p_api->start(MOTOR_LPWM_INSTANCE->p_ctrl);
            if (FSP_SUCCESS != err)
            {
                (void) MOTOR_RPWM_INSTANCE->p_api->stop(MOTOR_RPWM_INSTANCE->p_ctrl);
            }
            else
            {
                g_pwm_running = true;
            }
        }
    }
    if ((FSP_SUCCESS == err) && should_run)
    {
        err = g_ioport.p_api->pinWrite(g_ioport.p_ctrl,
                                       MOTOR_ENABLE_PIN,
                                       BSP_IO_LEVEL_HIGH);
    }
    if ((FSP_SUCCESS == err) && !should_run)
    {
        (void) g_ioport.p_api->pinWrite(g_ioport.p_ctrl,
                                        MOTOR_ENABLE_PIN,
                                        BSP_IO_LEVEL_LOW);
        if (g_pwm_running)
        {
            err = MOTOR_RPWM_INSTANCE->p_api->stop(MOTOR_RPWM_INSTANCE->p_ctrl);
            if (FSP_SUCCESS == err)
            {
                err = MOTOR_LPWM_INSTANCE->p_api->stop(MOTOR_LPWM_INSTANCE->p_ctrl);
                if (FSP_SUCCESS == err)
                {
                    g_pwm_running = false;
                }
            }
        }
    }

    return err;
#else
    return FSP_SUCCESS;
#endif
}

fsp_err_t dc_motor_init(void)
{
    g_left_target_rpm = 0;
    g_right_target_rpm = 0;
    g_left_target_duty_permille = 0;
    g_right_target_duty_permille = 0;
    g_drive_left_duty_permille = 0;
    g_drive_right_duty_permille = 0;
    g_pwm_update_elapsed_ms = 0U;
    g_pwm_running = false;

#if SEROV_ENABLE_DRIVE_MOTORS
    fsp_err_t err = MOTOR_RPWM_INSTANCE->p_api->open(MOTOR_RPWM_INSTANCE->p_ctrl,
                                                     MOTOR_RPWM_INSTANCE->p_cfg);
    if (FSP_SUCCESS == err)
    {
        err = MOTOR_LPWM_INSTANCE->p_api->open(MOTOR_LPWM_INSTANCE->p_ctrl,
                                               MOTOR_LPWM_INSTANCE->p_cfg);
    }
    if (FSP_SUCCESS == err)
    {
        err = g_ioport.p_api->pinWrite(g_ioport.p_ctrl,
                                       MOTOR_ENABLE_PIN,
                                       BSP_IO_LEVEL_LOW);
    }
    return err;
#else
    return FSP_SUCCESS;
#endif
}

fsp_err_t dc_motor_stop(void)
{
    g_left_target_rpm = 0;
    g_right_target_rpm = 0;
    g_left_target_duty_permille = 0;
    g_right_target_duty_permille = 0;
    g_drive_left_duty_permille = 0;
    g_drive_right_duty_permille = 0;
    g_pwm_update_elapsed_ms = 0U;

#if SEROV_ENABLE_DRIVE_MOTORS
    (void) g_ioport.p_api->pinWrite(g_ioport.p_ctrl,
                                    MOTOR_ENABLE_PIN,
                                    BSP_IO_LEVEL_LOW);
    (void) MOTOR_RPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_RPWM_INSTANCE->p_ctrl,
                                                     0U,
                                                     MOTOR_LEFT_RPWM_OUTPUT);
    (void) MOTOR_RPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_RPWM_INSTANCE->p_ctrl,
                                                     0U,
                                                     MOTOR_RIGHT_RPWM_OUTPUT);
    (void) MOTOR_LPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_LPWM_INSTANCE->p_ctrl,
                                                     0U,
                                                     MOTOR_LEFT_LPWM_OUTPUT);
    (void) MOTOR_LPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_LPWM_INSTANCE->p_ctrl,
                                                     0U,
                                                     MOTOR_RIGHT_LPWM_OUTPUT);
    if (g_pwm_running)
    {
        fsp_err_t err = MOTOR_RPWM_INSTANCE->p_api->stop(MOTOR_RPWM_INSTANCE->p_ctrl);
        if (FSP_SUCCESS == err)
        {
            err = MOTOR_LPWM_INSTANCE->p_api->stop(MOTOR_LPWM_INSTANCE->p_ctrl);
        }
        if (FSP_SUCCESS != err)
        {
            return err;
        }
        g_pwm_running = false;
    }
#endif

    return FSP_SUCCESS;
}

fsp_err_t dc_motor_request_rpm(int16_t left_rpm, int16_t right_rpm)
{
    if ((left_rpm < -JGA25_TARGET_RPM_MAX) ||
        (left_rpm > JGA25_TARGET_RPM_MAX) ||
        (right_rpm < -JGA25_TARGET_RPM_MAX) ||
        (right_rpm > JGA25_TARGET_RPM_MAX))
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    g_left_target_rpm = left_rpm;
    g_right_target_rpm = right_rpm;
    g_left_target_duty_permille = motor_rpm_to_duty_permille(left_rpm,
                                                              MOTOR_LEFT_FORWARD_SIGN);
    g_right_target_duty_permille = motor_rpm_to_duty_permille(right_rpm,
                                                               MOTOR_RIGHT_FORWARD_SIGN);
    return FSP_SUCCESS;
}

fsp_err_t dc_motor_housekeeping_1ms(void)
{
    g_drive_left_duty_permille = motor_ramp_value(g_drive_left_duty_permille,
                                                   g_left_target_duty_permille);
    g_drive_right_duty_permille = motor_ramp_value(g_drive_right_duty_permille,
                                                    g_right_target_duty_permille);

    g_pwm_update_elapsed_ms++;
    if (g_pwm_update_elapsed_ms < MOTOR_PWM_UPDATE_PERIOD_MS)
    {
        return FSP_SUCCESS;
    }

    g_pwm_update_elapsed_ms = 0U;
    return motor_pwm_apply();
}

int16_t dc_motor_left_target_rpm_get(void)
{
    return g_left_target_rpm;
}

int16_t dc_motor_right_target_rpm_get(void)
{
    return g_right_target_rpm;
}
