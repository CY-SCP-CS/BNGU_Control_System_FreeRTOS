/**
 * @file    app_task_imu_internal.h
 * @brief   IMU 服务；私有算法实例、样本队列和姿态锁
 */
#ifndef APP_TASK_IMU_INTERNAL_H
#define APP_TASK_IMU_INTERNAL_H
#include "app_task_imu.h"
#include "drv_imu.h"

/**
 * @brief  配置 IMU 并完成零偏标定和初始姿态对齐。
 * @return true=完成准备；false=设备或算法配置失败。
 * @note   仅初始化任务在 SYS_READY 前调用；标定使用任务延时，运行采样尚未启动。
 */
bool app_rtos_imu_prepare(void);
/**
 * @brief  复制完整 DMA 样本，或唤醒 IMU 任务推进错误恢复。
 * @param[in] sample 完整样本，立即复制；NULL 只通知恢复，不发布数据。
 * @return 无返回值；队列满累计丢样计数，通知失败由统一接口计数。
 * @note   合法 RTOS 中断或驱动恢复上下文调用，不等待，不保留 DMA 地址。
 */
void app_rtos_imu_receive(const drv_imu_dma_sample_t *sample);

/**
 * @brief  IMU 通信任务入口；标定、样本交付和姿态快照接口见本头文件
 * @param[in] argument CMSIS-RTOS2 任务入口参数，当前创建时传 NULL。
 * @return 无返回值；任务持续运行。
 * @note   由任务表注册并由调度器调用，业务代码不直接调用任务入口。
 */
void Start_comm_imu_Task(void *argument);

/**
 * @brief  创建本模块通信对象并复位初始状态。
 * @return true=全部创建成功；false=创建失败，由装配统一回滚。
 * @note   仅内核就绪、调度器启动前调用，不支持运行中重置。
 */
bool app_task_imu_init(void);
/**
 * @brief  删除本模块已创建的通信对象。
 * @return 无返回值；空句柄安全跳过。
 * @note   仅启动失败回滚使用，必须先终止所有已创建任务。
 */
void app_task_imu_deinit(void);

#endif
