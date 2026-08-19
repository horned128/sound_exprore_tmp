/** =================================================================*
 * @file   encoder.c
 * @brief  左右代表モーターのA/B相エンコーダ取得・速度算出
 * ================================================================= */
#include "encoder.h"                                        /* エンコーダAPI */
#include "../config/actuator_config.h"                      /* エンコーダ設定 */

volatile int32_t g_jga25_encoder_count = 0;                 /**< 左代表モーター累積カウント */
volatile int32_t g_jga25_rpm_x10 = 0;                       /**< 左代表モーター推定回転数の10倍 */
volatile int32_t g_jga25_right_encoder_count = 0;            /**< 右代表モーター累積カウント */
volatile int32_t g_jga25_right_rpm_x10 = 0;                  /**< 右代表モーター推定回転数の10倍 */

#if SEROV_ENABLE_MOTOR_ENCODER
static uint8_t g_left_encoder_previous_ab;
static uint8_t g_right_encoder_previous_ab;
static uint32_t g_speed_elapsed_ms;
static int32_t g_left_speed_previous_count;
static int32_t g_right_speed_previous_count;

/**< A/B相の遷移量テーブル（4逓倍） */
static int8_t const encoder_transition_delta[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0,
};
#endif

/** =================================================================*
 * @brief  指定した代表モーターのA/B相を読み取りカウント更新
 * @param[in] left trueなら左、falseなら右
 * ================================================================= */
static void encoder_update(bool left)
{
#if SEROV_ENABLE_MOTOR_ENCODER
    bsp_io_port_pin_t const pin_a = left ? JGA25_ENCODER_LEFT_A_PIN : JGA25_ENCODER_RIGHT_A_PIN;
    bsp_io_port_pin_t const pin_b = left ? JGA25_ENCODER_LEFT_B_PIN : JGA25_ENCODER_RIGHT_B_PIN;
    bsp_io_level_t level_a = BSP_IO_LEVEL_LOW;
    bsp_io_level_t level_b = BSP_IO_LEVEL_LOW;

    if ((FSP_SUCCESS != g_ioport.p_api->pinRead(g_ioport.p_ctrl, pin_a, &level_a)) ||
        (FSP_SUCCESS != g_ioport.p_api->pinRead(g_ioport.p_ctrl, pin_b, &level_b)))
    {
        return;
    }

    uint8_t const current_ab = (uint8_t) (((uint8_t) level_a << 1U) | (uint8_t) level_b);
    uint8_t * p_previous_ab = left ? &g_left_encoder_previous_ab : &g_right_encoder_previous_ab;
    uint8_t const transition = (uint8_t) (((*p_previous_ab) << 2U) | current_ab);
    if (left)
    {
        g_jga25_encoder_count += encoder_transition_delta[transition];
    }
    else
    {
        g_jga25_right_encoder_count += encoder_transition_delta[transition];
    }
    *p_previous_ab = current_ab;
#else
    FSP_PARAMETER_NOT_USED(left);
#endif
}

/** =================================================================*
 * @brief  速度推定値を更新
 * @param[in] count 現在の累積カウント
 * @param[in] p_previous_count 前回速度算出時の累積カウント
 * @param[in,out] p_rpm_x10 10倍RPM出力
 * @param[in] elapsed_ms サンプル経過時間
 * ================================================================= */
#if SEROV_ENABLE_MOTOR_ENCODER
static void encoder_speed_update(int32_t count,
                                 int32_t * p_previous_count,
                                 volatile int32_t * p_rpm_x10,
                                 uint32_t elapsed_ms)
{
    int32_t const delta = count - *p_previous_count;
    int64_t const numerator = (int64_t) delta * 600000LL;
    int64_t const denominator = (int64_t) JGA25_ENCODER_COUNTS_PER_REV * elapsed_ms;
    *p_rpm_x10 = (int32_t) (numerator / denominator);
    *p_previous_count = count;
}
#endif

/** =================================================================*
 * @brief  エンコーダ初期化
 * @return FSPエラーコード
 * ================================================================= */
fsp_err_t encoder_init(void)
{
    g_jga25_encoder_count = 0;
    g_jga25_rpm_x10 = 0;
    g_jga25_right_encoder_count = 0;
    g_jga25_right_rpm_x10 = 0;

#if SEROV_ENABLE_MOTOR_ENCODER
    bsp_io_level_t left_a = BSP_IO_LEVEL_LOW;
    bsp_io_level_t left_b = BSP_IO_LEVEL_LOW;
    bsp_io_level_t right_a = BSP_IO_LEVEL_LOW;
    bsp_io_level_t right_b = BSP_IO_LEVEL_LOW;
    fsp_err_t err = g_ioport.p_api->pinRead(g_ioport.p_ctrl, JGA25_ENCODER_LEFT_A_PIN, &left_a);
    if (FSP_SUCCESS == err)
    {
        err = g_ioport.p_api->pinRead(g_ioport.p_ctrl, JGA25_ENCODER_LEFT_B_PIN, &left_b);
    }
    if (FSP_SUCCESS == err)
    {
        err = g_ioport.p_api->pinRead(g_ioport.p_ctrl, JGA25_ENCODER_RIGHT_A_PIN, &right_a);
    }
    if (FSP_SUCCESS == err)
    {
        err = g_ioport.p_api->pinRead(g_ioport.p_ctrl, JGA25_ENCODER_RIGHT_B_PIN, &right_b);
    }
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    g_left_encoder_previous_ab = (uint8_t) (((uint8_t) left_a << 1U) | (uint8_t) left_b);
    g_right_encoder_previous_ab = (uint8_t) (((uint8_t) right_a << 1U) | (uint8_t) right_b);
    g_speed_elapsed_ms = 0U;
    g_left_speed_previous_count = 0;
    g_right_speed_previous_count = 0;

    err = g_encoder_a_irq.p_api->open(g_encoder_a_irq.p_ctrl, g_encoder_a_irq.p_cfg);
    if (FSP_SUCCESS == err)
    {
        err = g_encoder_a_irq.p_api->enable(g_encoder_a_irq.p_ctrl);
    }
    if (FSP_SUCCESS == err)
    {
        err = g_encoder_b_irq.p_api->open(g_encoder_b_irq.p_ctrl, g_encoder_b_irq.p_cfg);
    }
    if (FSP_SUCCESS == err)
    {
        err = g_encoder_b_irq.p_api->enable(g_encoder_b_irq.p_ctrl);
    }
    if (FSP_SUCCESS == err)
    {
        err = g_encoder_right_a_irq.p_api->open(g_encoder_right_a_irq.p_ctrl,
                                                g_encoder_right_a_irq.p_cfg);
    }
    if (FSP_SUCCESS == err)
    {
        err = g_encoder_right_a_irq.p_api->enable(g_encoder_right_a_irq.p_ctrl);
    }
    if (FSP_SUCCESS == err)
    {
        err = g_encoder_right_b_irq.p_api->open(g_encoder_right_b_irq.p_ctrl,
                                                g_encoder_right_b_irq.p_cfg);
    }
    if (FSP_SUCCESS == err)
    {
        err = g_encoder_right_b_irq.p_api->enable(g_encoder_right_b_irq.p_ctrl);
    }

    return err;
#else
    return FSP_SUCCESS;
#endif
}

/** =================================================================*
 * @brief  エンコーダ1 ms周期処理
 * ================================================================= */
void encoder_housekeeping_1ms(void)
{
#if SEROV_ENABLE_MOTOR_ENCODER
    g_speed_elapsed_ms++;
    if (g_speed_elapsed_ms >= JGA25_SPEED_SAMPLE_PERIOD_MS)
    {
        encoder_speed_update(g_jga25_encoder_count,
                             &g_left_speed_previous_count,
                             &g_jga25_rpm_x10,
                             g_speed_elapsed_ms);
        encoder_speed_update(g_jga25_right_encoder_count,
                             &g_right_speed_previous_count,
                             &g_jga25_right_rpm_x10,
                             g_speed_elapsed_ms);
        g_speed_elapsed_ms = 0U;
    }
#endif
}

static int16_t encoder_rpm_from_x10(volatile int32_t const * p_rpm_x10)
{
    int32_t rpm = *p_rpm_x10 / 10;
    if (rpm > INT16_MAX)
    {
        rpm = INT16_MAX;
    }
    else if (rpm < INT16_MIN)
    {
        rpm = INT16_MIN;
    }
    return (int16_t) rpm;
}

int32_t encoder_count_get(void)
{
    return g_jga25_encoder_count;
}

int32_t encoder_right_count_get(void)
{
    return g_jga25_right_encoder_count;
}

int16_t encoder_rpm_get(void)
{
    return encoder_rpm_from_x10(&g_jga25_rpm_x10);
}

int16_t encoder_right_rpm_get(void)
{
    return encoder_rpm_from_x10(&g_jga25_right_rpm_x10);
}

void jga25_encoder_a_callback(external_irq_callback_args_t * p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    encoder_update(true);
}

void jga25_encoder_b_callback(external_irq_callback_args_t * p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    encoder_update(true);
}

void jga25_encoder_right_a_callback(external_irq_callback_args_t * p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    encoder_update(false);
}

void jga25_encoder_right_b_callback(external_irq_callback_args_t * p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    encoder_update(false);
}
