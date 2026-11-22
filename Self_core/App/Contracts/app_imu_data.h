/**
 * @file    app_imu_data.h
 * @brief   姿态快照契约
 */
#ifndef APP_IMU_DATA_H
#define APP_IMU_DATA_H
#include <stdint.h>
#include "lib_physical.h"

/**
 * @brief  IMU 任务发布的物理量与姿态快照
 * @note   acc 使用 m/s²，gyro 使用 rad/s，euler 使用 rad；云台控制输出角度使用 rad。
 *          初始化可先发布对齐姿态，sampled 标记随后是否已处理运行阶段完整样本。
 *          device_valid 记录芯片 ID 检查结果，参与控制有效性判断。
 */
typedef struct
{
    lib_vec3f_t acc,       // 安装补偿后的机体系加速度，单位 m/s²。
        gyro;              // 扣除静态和在线零偏后、安装补偿后的机体系角速度，rad/s。
    lib_quatf_t quat;      // 所选 AHRS 的安装补偿后机体系到世界系四元数，无量纲。
    lib_euler_rad_t euler; // 安装补偿后的欧拉角，rad；哨兵 XYZ、导航 ZYX 模式沿用驱动约定。
    uint32_t tick,         // 运行样本触发时间戳；初始姿态使用初始化发布时间。
        sequence;          // 运行采样序号，与硬件采样触发对应。
    uint8_t valid,         // 姿态可供控制使用；运行更新时检查 yaw/roll/gyro.z 为有限数。
        sampled,           // 已处理过运行阶段完整样本的标志，序号回绕时仍保持置位。
        device_valid,
        filter_ready; // AHRS startup history established.      // 加速度计和陀螺仪芯片 ID
                      // 均匹配的标志。
} app_imu_snapshot_t;
#endif
