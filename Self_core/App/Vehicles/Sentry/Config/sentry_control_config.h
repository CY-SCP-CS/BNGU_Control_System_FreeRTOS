/**
 * @file    sentry_control_config.h
 * @brief   控制时间配置；调度节拍和算法积分时间使用同一来源
 */
#ifndef SENTRY_CONTROL_CONFIG_H
#define SENTRY_CONTROL_CONFIG_H
#define APP_CONTROL_GIM_PERIOD_MS 1U
#define APP_CONTROL_CHAS_PERIOD_MS 1U
#define APP_CONTROL_SHOOT_PERIOD_MS 5U
#define APP_CONTROL_FEEDBACK_PERIOD_MS 5U
#define APP_CONTROL_GIM_DT_S (APP_CONTROL_GIM_PERIOD_MS * 0.001f)
#define APP_CONTROL_CHAS_DT_S (APP_CONTROL_CHAS_PERIOD_MS * 0.001f)
#define APP_CONTROL_SHOOT_DT_S (APP_CONTROL_SHOOT_PERIOD_MS * 0.001f)
#define SENTRY_CHASSIS_RAMP_ENABLE 1
#define SENTRY_CHASSIS_LINEAR_ACCEL_MM_S2 3000.0f // 3 m/s^2。
#define SENTRY_CHASSIS_ANGULAR_ACCEL_RAD_S2 9.0f  // rad/s^2。
#define SENTRY_GIMBAL_PITCH_JOINT_MIN                                                              \
    (-35.0f * LIB_PI / 180.0f) // 相对编码器机械零位的关节下限，rad。
#define SENTRY_GIMBAL_PITCH_JOINT_MAX                                                              \
    (25.0f * LIB_PI / 180.0f)             // 相对编码器机械零位的关节上限，rad。
#define SENTRY_GIMBAL_STARTUP_WAIT_MS 20U // 反馈持续有效后的启动等待时间。
#define SENTRY_GIMBAL_SCAN_ENABLE 1U
#define SENTRY_GIMBAL_SCAN_YAW_SPEED (20.0f * LIB_PI / 180.0f) // 持续左转。
#define SENTRY_GIMBAL_SCAN_PITCH_SPEED (20.0f * LIB_PI / 180.0f)
#define SENTRY_GIMBAL_SCAN_PITCH_MIN SENTRY_GIMBAL_PITCH_MIN
#define SENTRY_GIMBAL_SCAN_PITCH_MAX SENTRY_GIMBAL_PITCH_MAX
#define SENTRY_GIMBAL_CAN_POSE_TIMEOUT_MS 200U

#ifndef SENTRY_CAN_TEST_ENABLE
#define SENTRY_CAN_TEST_ENABLE 0U
#endif
#endif
