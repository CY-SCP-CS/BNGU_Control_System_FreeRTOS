/** @file app_input_config.h
 * @brief Maximum sample ages at the required 1 kHz kernel tick. Inclusive limits.
 * @note All timestamps are reception/sample times, never refreshed by a reader.
 */
#ifndef NAVIGATION_INPUT_CONFIG_H
#define NAVIGATION_INPUT_CONFIG_H
#define APP_INPUT_IMU_MAX_AGE_MS 10U
#define APP_INPUT_NAV_IMU_MAX_AGE_MS 20U
#define APP_INPUT_DBUS_MAX_AGE_MS 100U
#define APP_INPUT_MOTOR_MAX_AGE_MS 20U
#define APP_INPUT_CAN_MAX_AGE_MS 100U
#define APP_INPUT_POWER_MAX_AGE_MS 100U
#define APP_INPUT_OMEGA_MAX_AGE_MS 20U
#define APP_INPUT_SHOOT_MAX_AGE_MS 20U
#endif
