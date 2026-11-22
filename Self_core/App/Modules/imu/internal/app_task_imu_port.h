#ifndef APP_TASK_IMU_PORT_H
#define APP_TASK_IMU_PORT_H
#include "app_task_imu_io.h"
#include "drv_imu_port.h"
void app_task_imu_port_configure(const drv_imu_port_config_t *config);
void app_task_imu_port_trigger(uint32_t tick, uint32_t sequence, uint32_t cycles);
#endif
