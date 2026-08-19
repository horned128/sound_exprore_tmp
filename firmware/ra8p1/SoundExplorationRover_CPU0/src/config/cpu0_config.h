/** =================================================================*
 * @file   cpu0_config.h
 * @brief  CPU0動作設定
 * ================================================================= */
#ifndef SEROV_CPU0_CONFIG_H
#define SEROV_CPU0_CONFIG_H

#define CPU0_ACTUATOR_STARTUP_DELAY_MS     (250U)
#define CPU0_COMMAND_PERIOD_MS             (50U)
#define CPU0_COMMAND_TARGET_TIMEOUT_MS     (500U)
#define CPU0_THINK_PERIOD_MS               (100U)
#define CPU0_IPC_RETRY_DELAY_MS            (1U)

/* 数値が小さいほど高優先度。IPC keep-aliveを思考処理より優先する。 */
#define CPU0_INIT_TASK_PRIORITY            (5)
#define CPU0_COMMAND_TASK_PRIORITY         (6)
#define CPU0_THINK_TASK_PRIORITY           (10)
#define CPU0_INIT_TASK_STACK_SIZE          (1024U)
#define CPU0_COMMAND_TASK_STACK_SIZE       (1024U)
#define CPU0_THINK_TASK_STACK_SIZE         (1024U)

/* tk_thinkのサンプル動作を切り替える。通常は円軌道を使用する。 */
#define CPU0_THINK_MOTION_MODE_CIRCLE       (0)
#define CPU0_THINK_MOTION_MODE_SQUARE       (1)
#define CPU0_THINK_MOTION_MODE              (CPU0_THINK_MOTION_MODE_SQUARE)

/*
 * 実機の安全操舵限界内で、300 rpm実測モーターの負荷を考慮した
 * 左円軌道を作る。値はエンコーダ無効時のオープンループ換算値である。
 */
#define CPU0_CIRCLE_STEERING_DEG           (-45)
#define CPU0_CIRCLE_LEFT_RPM               (-130)
#define CPU0_CIRCLE_RIGHT_RPM              (-90)
#define CPU0_CIRCLE_CENTER_HOLD_MS         (1500U)
#define CPU0_CIRCLE_STEER_HOLD_MS          (1500U)

/* 正方形軌道: 45%で3秒直進し、左旋回を時刻で近似して4辺を繰り返す。 */
#define CPU0_OPEN_LOOP_RPM_FULL_SCALE      (300)
#define CPU0_SQUARE_SPEED_PERCENT          (45)
#define CPU0_SQUARE_SPEED_RPM              ((CPU0_OPEN_LOOP_RPM_FULL_SCALE * CPU0_SQUARE_SPEED_PERCENT) / 100)
#define CPU0_SQUARE_FORWARD_RPM            (-CPU0_SQUARE_SPEED_RPM)
#define CPU0_SQUARE_STRAIGHT_MS            (3000U)
#define CPU0_SQUARE_TURN_STEERING_DEG      (-45)
#define CPU0_SQUARE_TURN_MS                (1000U)
#define CPU0_SQUARE_CENTER_HOLD_MS         (1500U)

/* 青LED: 思考状態、緑LED: heartbeatまたはfault。赤LEDはCPU1専用。 */
#define CPU0_THINK_BLUE_LED_INDEX          (0U)
#define CPU0_THINK_GREEN_LED_INDEX         (1U)
#define CPU0_LED_HEARTBEAT_PERIOD_MS       (1000U)
#define CPU0_LED_HEARTBEAT_PULSE_MS        (100U)
#define CPU0_LED_CENTER_BLINK_MS           (500U)
#define CPU0_LED_STEER_BLINK_MS            (100U)
#define CPU0_LED_FAULT_PULSE_MS            (100U)
#define CPU0_LED_FAULT_GAP_MS              (1000U)

#endif /* SEROV_CPU0_CONFIG_H */
