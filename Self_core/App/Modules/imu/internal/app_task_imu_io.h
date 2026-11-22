#ifndef APP_TASK_IMU_IO_H
#define APP_TASK_IMU_IO_H
#include <stdbool.h>
#include "drv_imu.h"
bool app_task_imu_port_prepare(drv_imu_t *imu);
void app_task_imu_port_poll(void);
uint32_t app_task_imu_port_errors(void);
uint32_t app_task_imu_port_busy(void);
#endif
