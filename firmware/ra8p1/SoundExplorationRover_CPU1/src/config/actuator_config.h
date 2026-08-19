/** =================================================================*
 * @file   actuator_config.h
 * @brief  CPU1アクチュエータ設定
 * ================================================================= */
#ifndef SEROV_ACTUATOR_CONFIG_H
#define SEROV_ACTUATOR_CONFIG_H

#include "hal_data.h"                                       /* FSP生成のHAL/BSPインスタンス、ピン定義、周辺機器設定 */
#include "../../../common/ipc_message.h"                    /* CPU間通信で共有するサーボ数 */

/* 安全設定と周期設定。CPU0指令タスクは現在50 ms周期で指令を送信する。 */
#define ACTUATOR_COMMAND_TIMEOUT_MS          (1500U)
#define ACTUATOR_LOOP_PERIOD_MS              (1U)

/* 操舵サーボ設定。DS3225MGの180度範囲から安全側の範囲を使用する。 */
#define SERVO_PWM_PERIOD_US                  (20000U)
#define SERVO_PULSE_MIN_SAFE_US              (1000U)
#define SERVO_PULSE_CENTER_US                (1500U)
#define SERVO_PULSE_MAX_SAFE_US              (2000U)
#define STEERING_MIN_DEG                     (-45)
#define STEERING_CENTER_DEG                  (0)
#define STEERING_MAX_DEG                     (45)
#define STEERING_MIN_PULSE_US                (1200U)
#define STEERING_MAX_PULSE_US                (1800U)

/* 4サーボ試験用PWM割り当て。配列順はFR、FL、RR、RL。 */
#define SERVO_COUNT                          ACTUATOR_SERVO_COUNT
/* FSP instance names are normalized to the logical wheel names. */
#define SERVO_PWM_INSTANCE_FR               (&g_servo_pwm_fr) /* GPT12B / P803 */
#define SERVO_PWM_INSTANCE_FL               (&g_servo_pwm_fl) /* GPT9B  / P110 */
#define SERVO_PWM_INSTANCE_RR               (&g_servo_pwm_rr) /* GPT11B / P801 */
#define SERVO_PWM_INSTANCE_RL               (&g_servo_pwm_rl) /* GPT13B / P808 */
#define SERVO_PWM_OUTPUT_FR                 (GPT_IO_PIN_GTIOCB)
#define SERVO_PWM_OUTPUT_FL                 (GPT_IO_PIN_GTIOCB)
#define SERVO_PWM_OUTPUT_RR                 (GPT_IO_PIN_GTIOCB)
#define SERVO_PWM_OUTPUT_RL                 (GPT_IO_PIN_GTIOCB)

/*
 * Per-wheel steering calibration.  First mount each horn near straight at
 * 1500 us, then adjust these trims in 5-10 us steps.  Use -1 for a servo
 * whose mechanical installation reverses the requested steering direction.
 */
#define SERVO_CENTER_TRIM_US_FR             (0)
#define SERVO_CENTER_TRIM_US_FL             (0)
#define SERVO_CENTER_TRIM_US_RR             (0)
#define SERVO_CENTER_TRIM_US_RL             (0)
#define SERVO_DIRECTION_FR                  (1)
#define SERVO_DIRECTION_FL                  (1)
#define SERVO_DIRECTION_RR                  (1)
#define SERVO_DIRECTION_RL                  (1)

/* Standalone four-servo wiring test. Set to 0 to return to CPU0/CPU1 IPC control. */
#define SERVO_DEMO_ENABLE                   (0)
#define SERVO_DEMO_MIN_DEG                  (-30)
#define SERVO_DEMO_MAX_DEG                  (30)
#define SERVO_DEMO_STEP_DEG                 (2)
#define SERVO_DEMO_INTERVAL_MS              (20U)

/* 左右BTS7960の20 kHzオープンループ動作確認設定。 */
#define SEROV_ENABLE_DRIVE_MOTORS            (1)
#define SEROV_ENABLE_MOTOR_ENCODER           (0) /* 配線確認後に1へ変更 */
#define JGA25_MOTOR_PWM_FREQUENCY_HZ         (20000U)
#define JGA25_TARGET_RPM_MAX                 (300)
/* 300 RPM品の減速比は未確定。出力軸10回転で実測するまでRPM換算を禁止する。 */
#define JGA25_ENCODER_COUNTS_PER_REV         (0U)
#define JGA25_SPEED_SAMPLE_PERIOD_MS         (100U)

#if SEROV_ENABLE_MOTOR_ENCODER && (0U == JGA25_ENCODER_COUNTS_PER_REV)
 #error "Measure the 300 RPM JGA25 encoder counts/rev before enabling the encoder."
#endif

#define MOTOR_RPWM_INSTANCE                  (&g_motor_pwm)
#define MOTOR_LPWM_INSTANCE                  (&g_motor_pwm_lpwm)
#define MOTOR_LEFT_RPWM_OUTPUT               (GPT_IO_PIN_GTIOCB) /* P811 / GPT10B */
#define MOTOR_RIGHT_RPWM_OUTPUT              (GPT_IO_PIN_GTIOCA) /* P810 / GPT10A */
#define MOTOR_LEFT_LPWM_OUTPUT               (GPT_IO_PIN_GTIOCB) /* P602 / GPT7B */
#define MOTOR_RIGHT_LPWM_OUTPUT              (GPT_IO_PIN_GTIOCA) /* P603 / GPT7A */
/*
 * Logical positive RPM means rover-forward. The physical front/rear reference
 * is opposite to the initial software reference, so invert the chassis axis.
 * The two motor mounts also have opposite electrical polarities. Keep these
 * signs in configuration instead of changing the IPC or circle trajectory.
 */
#define MOTOR_CHASSIS_FORWARD_SIGN          (+1)
#define MOTOR_LEFT_MOUNT_SIGN               (-1)
#define MOTOR_RIGHT_MOUNT_SIGN              (+1)
#define MOTOR_LEFT_FORWARD_SIGN             (MOTOR_CHASSIS_FORWARD_SIGN * MOTOR_LEFT_MOUNT_SIGN)
#define MOTOR_RIGHT_FORWARD_SIGN            (MOTOR_CHASSIS_FORWARD_SIGN * MOTOR_RIGHT_MOUNT_SIGN)
/* 左右BTS7960のR_EN/L_ENを外部で共通接続し、PD01で一括制御する。 */
#define MOTOR_ENABLE_PIN                    (ARDUINO_D8_MIKROBUS_INT)
#define MOTOR_PWM_MIN_DUTY_PERMILLE          (0)
#define MOTOR_PWM_MAX_DUTY_PERMILLE          (700)
#define MOTOR_PWM_RAMP_PER_MS                (2)
#define MOTOR_PWM_UPDATE_PERIOD_MS           (5U)

/*
 * エンコーダ入力は左右の代表モーターを1組ずつ測定する。
 * 現在の6輪構成は左右各3台を1台のBTS7960へ並列接続するため、6台を
 * 個別制御できない現状で6組を混ぜて1入力へ接続してはいけない。
 * A/Bは両エッジの四逓倍クアドラチャ計数に使用する。
 */
#define JGA25_ENCODER_LEFT_A_PIN             (ARDUINO_D2_INT0)           /* P011 / IRQ16 */
#define JGA25_ENCODER_LEFT_B_PIN             (ARDUINO_D1TX_MIKROBUS_TX)  /* P809 / IRQ20 */
#define JGA25_ENCODER_RIGHT_A_PIN            (PMOD1_IRQ)                 /* P006 / IRQ11 */
#define JGA25_ENCODER_RIGHT_B_PIN            (PMOD1_GPIO2)               /* P413 / IRQ18 */

/* 旧名称は左エンコーダA/Bとの互換用。 */
#define JGA25_ENCODER_A_PIN                  JGA25_ENCODER_LEFT_A_PIN
#define JGA25_ENCODER_B_PIN                  JGA25_ENCODER_LEFT_B_PIN

#define ACTUATOR_RIGHT_MOTOR_CONFIGURED      (1)

#endif /* SEROV_ACTUATOR_CONFIG_H */
