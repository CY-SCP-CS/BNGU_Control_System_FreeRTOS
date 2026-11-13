/**
 * @file    app_task_imu.h
 * @brief   IMU 服务；私有算法实例、样本队列和姿态锁
 */
#ifndef APP_TASK_IMU_INTERNAL_H
#define APP_TASK_IMU_INTERNAL_H
#include <stdbool.h>
#include "app_imu_data.h"
#include "drv_imu.h"

/** @brief 初始化任务完成 IMU 配置、3000 次零偏采样与对齐；运行任务尚未启动。 */
bool app_rtos_imu_prepare(void);
/** @brief 完整 DMA 样本立即复制入队；NULL 只通知任务推进驱动恢复。 */
void app_rtos_imu_receive(const drv_imu_dma_sample_t *sample);
bool app_rtos_imu_read(app_imu_snapshot_t *value);        /**< 控制任务非阻塞取得姿态副本。 */
void app_rtos_imu_write(const app_imu_snapshot_t *value); /**< 初始化/IMU 任务发布完整姿态。 */

/**
 * @brief  IMU 通信任务入口；标定、样本交付和姿态快照接口见本头文件
 * @param  argument CMSIS-RTOS2 任务入口参数，当前创建时传 NULL。
 * @note   由任务表注册并由调度器调用，业务代码不直接调用任务入口。
 */
void Start_comm_imu_Task(void *argument);

/** @brief 调度器启动前创建本模块的对象与初始数据。 */
bool app_task_imu_init(void);
/** @brief 仅启动失败回滚时调用，调用前确保所属任务未运行。 */
void app_task_imu_deinit(void);

#endif
