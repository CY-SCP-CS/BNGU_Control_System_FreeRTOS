/**
 * @file    app_imu_data.h
 * @brief   姿态快照契约
 */
#ifndef APP_IMU_DATA_H
#define APP_IMU_DATA_H
#include <stdint.h>
#include "drv_imu.h"

/**
 * @brief  IMU 任务发布的物理量与姿态快照
 * @note   acc 使用 g，gyro 使用 rad/s，euler 使用度；云台控制输出角度使用 rad。
 *          初始化可先发布对齐姿态，sampled 标记随后是否已处理运行阶段完整样本。
 *          device_valid 记录芯片 ID 检查结果，供健康监测使用。
 */
typedef struct
{
    drv_imu_real_t acc,    // 传感器三轴加速度转换值，单位 g。
        gyro;              // 传感器三轴角速度转换值，单位 rad/s。
    drv_imu_quat_t quat;   // Mahony 更新后的姿态四元数，无量纲。
    drv_imu_euler_t euler; // 姿态欧拉角，单位度。
    uint32_t tick,         // 运行样本触发时间戳；初始姿态使用初始化发布时间。
        sequence;          // 运行采样序号，与硬件采样触发对应。
    uint8_t valid,         // 姿态可供控制使用；运行更新时检查 yaw/roll/gyro.z 为有限数。
        sampled,           // 已处理过运行阶段完整样本的标志，序号回绕时仍保持置位。
        device_valid;      // 加速度计和陀螺仪芯片 ID 均匹配的标志。
} app_imu_snapshot_t;
#endif
