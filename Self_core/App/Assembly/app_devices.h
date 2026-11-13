/** @file app_devices.h
 * @brief 装配内部设备操作；业务消费者不包含本头。
 */
#ifndef APP_DEVICES_H
#define APP_DEVICES_H
#include <stdbool.h>
#include "drv_imu.h"
bool app_devices_start(void);
void app_devices_stop(void);
bool app_devices_dbus_poll(void);
bool app_devices_referee_poll(void);
bool app_devices_imu_prepare(drv_imu_t *imu);
void app_devices_imu_poll(void);
void app_devices_imu_trigger(uint32_t tick, uint32_t sequence, uint32_t cycles);
uint32_t app_devices_imu_errors(void);
uint32_t app_devices_imu_busy(void);
/**
 * @brief Read the latest magnetic sensor data in sensor axes.
 * @param[out] field_tesla Three components [X,Y,Z], unit T; unchanged on failure.
 * @param[out] sample_tick HAL 1 ms tick of measurement; unchanged on failure.
 * @return true=valid sample copied; false=no valid sample or invalid pointers.
 * @note Task context; does not start I2C or fuse heading into six-axis AHRS.
 */
bool app_devices_mag_read(float field_tesla[3], uint32_t *sample_tick);
#endif
