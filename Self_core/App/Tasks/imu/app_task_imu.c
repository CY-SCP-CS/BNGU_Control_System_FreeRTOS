/**
 * @file    app_task_imu.c
 * @brief   完整样本队列、姿态解算和快照；TIM14 保持唯一采样触发
 * @note    入口名、周期、业务算法及原通信对象名保持；通信对象句柄仅所属模块可访问。
 */
#include "app_rtos.h"
#include "app_task_imu.h"
#include "app_tasks.h"
#include "app_rtos_objects.h"
#include "app_rtos_config.h"
#include "app_board_config.h"
#include "app_rtos_time.h"
#include "app_task_monitor.h"
#include <string.h>
#include <math.h>

static osMessageQueueId_t imu_rx_queue;
static osMutexId_t imu_to_gim_mutex;
static app_imu_snapshot_t s_imu;
static drv_imu_t app_rtos_imu_instance; //算法实例仅初始化阶段和运行 IMU 任务访问。

/** @brief 采样回调可在 ISR 或驱动恢复路径调用；只复制完整样本或发唤醒信号。 */
void app_rtos_imu_receive(const drv_imu_dma_sample_t *sample)
{
    if (!sample)
    {
        app_rtos_notify_task(TASK_IMU, APP_RTOS_TF_DATA_READY);
        return;
    }
    if (osMessageQueuePut(imu_rx_queue, sample, 0, 0) != osOK)
    {
        app_rtos_count(COUNT_IMU_DROP);
    }
    else
    {
        app_rtos_notify_task(TASK_IMU, APP_RTOS_TF_DATA_READY);
    }
}

/**
 * @brief  读取一份完整的 IMU 姿态快照
 * @param  v            输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由云台控制及诊断读取者在任务上下文调用；使用 imu_to_gim_mutex，获取锁超时为 0。
 */
bool app_rtos_imu_read(app_imu_snapshot_t *v)
{
    return app_rtos_snapshot_read(imu_to_gim_mutex, &s_imu, v, sizeof(*v));
}

/**
 * @brief  发布 IMU 姿态快照
 * @param  v            输入快照，函数在锁内复制其内容
 * @note   写入者：comm_imu_Task；初始化任务在启动前也发布初始对齐姿态；使用 imu_to_gim_mutex，仅允许任务上下文调用。
 * @note   等待取得锁后只复制；参数/句柄无效或获取失败时忽略本次更新。
 */
void app_rtos_imu_write(const app_imu_snapshot_t *v)
{
    app_rtos_snapshot_write(imu_to_gim_mutex, &s_imu, v, sizeof(*v));
}

/** @brief 只有初始化任务调用；SYS_READY 之前完成配置和对齐，尚未有运行采样。 */
bool app_rtos_imu_prepare(void)
{
#if APP_BOARD_HAS_IMU
    if (!drv_imu_port_init(&app_rtos_imu_instance))
    {
        return false;
    }
    app_imu_snapshot_t initial = {0};
    initial.acc = app_rtos_imu_instance.acc;
    initial.gyro = app_rtos_imu_instance.gyro;
    initial.quat = app_rtos_imu_instance.quat;
    initial.euler = app_rtos_imu_instance.euler;
    initial.tick = osKernelGetTickCount();
    initial.valid = 1;
    app_rtos_imu_write(&initial);
#endif
    return true;
}

/**
 * @brief  IMU 样本处理与姿态解算任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   仅云台板创建；SYS_READY 后处理完整 DMA 样本；1 tick 超时用于恢复检查。
 * @note   解析 ACC/GYRO、转换单位、Mahony 和欧拉角解算；仅 TIM14 启动新采样，任务不另建采样节拍。
 */
void Start_comm_imu_Task(void *argument)
{
    (void)argument;
#if APP_BOARD_HAS_IMU
    static app_imu_snapshot_t snapshot;
    app_tasks_wait_start(TASK_IMU);
    uint32_t previous_cycles = 0;
    for (;;)
    {
        drv_imu_dma_sample_t sample;
        for (unsigned n = 0; n < APP_RTOS_IMU_QUEUE_DEPTH &&
                             osMessageQueueGet(imu_rx_queue, &sample, NULL, 0) == osOK;
             n++)
        {
            uint32_t started = app_rtos_cycles();
            drv_imu_t *imu = &app_rtos_imu_instance;
            /* 原始六轴数据按传感器小端组合，解析与物理量转换均在任务中完成。 */
            drv_imu_parse_acc_raw(imu, sample.acc);
            drv_imu_parse_gyro_raw(imu, sample.gyro);
            /* 姿态积分 dt 来自硬件触发的 DWT 时间差，与任务被调度的延迟无关。 */
            float dt =
                previous_cycles ? app_rtos_sample_dt(sample.cycles, previous_cycles) : 0.001f;
            previous_cycles = sample.cycles;
            /* 长时间中断无法重建丢失运动，异常间隔恢复时按标称 1 ms 积分。 */
            if (dt <= 0 || dt > 0.01f)
            {
                dt = 0.001f;
            }
            drv_imu_data_convert(imu);
            drv_imu_mahony_update(imu, dt);
            drv_imu_quat_to_euler(imu);
            snapshot.acc = imu->acc;
            snapshot.gyro = imu->gyro;
            snapshot.quat = imu->quat;
            snapshot.euler = imu->euler;
            snapshot.tick = sample.tick;
            snapshot.sequence = sample.sequence;
            snapshot.sampled = 1;
            /* 芯片 ID 有效性只用于健康监测；sampled 单独记录运行采样，避免序号回绕误报。 */
            snapshot.device_valid = sample.acc_chip_id == DRV_IMU_ACC_CHIP_ID &&
                                    sample.gyro_chip_id == DRV_IMU_GYRO_CHIP_ID;
            snapshot.valid =
                isfinite(imu->euler.yaw) && isfinite(imu->euler.roll) && isfinite(imu->gyro.z);
            app_rtos_imu_write(&snapshot);
            app_rtos_module_update(MODULE_IMU,
                                   snapshot.valid && snapshot.sampled && snapshot.device_valid,
                                   snapshot.tick);
            app_tasks_profile_end(TASK_IMU, started);
        }
        drv_imu_port_poll();
        app_rtos_module_diagnostics(MODULE_IMU, drv_imu_port_get_error_count(),
                                    drv_imu_port_get_busy_count());
        app_tasks_wait_event(TASK_IMU,
                             APP_RTOS_IMU_POLL_WAIT); /* 任务超时只推进恢复，不启动新采样。 */
    }
#else
    app_tasks_idle_failed(TASK_IMU);
#endif
}

/**
 * @brief  创建本模块全部通信对象并复位数据
 * @return true=全部创建成功，false=创建失败，由装配层统一回滚。
 * @note   只在调度器启动前调用一次；不支持运行中重新初始化。
 */
bool app_task_imu_init(void)
{
    memset(&s_imu, 0, sizeof(s_imu));
    memset(&app_rtos_imu_instance, 0, sizeof(app_rtos_imu_instance));
    return app_rtos_object_queue(&imu_rx_queue, "imu_rx_queue", APP_RTOS_IMU_QUEUE_DEPTH,
                                 sizeof(drv_imu_dma_sample_t)) &&
           app_rtos_object_mutex(&imu_to_gim_mutex, "imu_to_gim_mutex");
}

/** @brief 启动失败时删除已创建对象；句柄为 NULL 时安全跳过。 */
void app_task_imu_deinit(void)
{
    app_rtos_object_queue_delete(&imu_rx_queue);
    app_rtos_object_mutex_delete(&imu_to_gim_mutex);
}
