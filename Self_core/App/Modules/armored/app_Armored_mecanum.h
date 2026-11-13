//
// Created by 33140 on 2026/10/26.
//

#ifndef BNGU_CONTROL_SYSTEM_APP_ARMORED_MECANUM_C_H
#define BNGU_CONTROL_SYSTEM_APP_ARMORED_MECANUM_C_H

#include "legacy_control_types.h"
#include "legacy_motor_limits.h"
#include "lib_pid.h"

typedef struct
{
    legacy_motor_t motor_data;
    lib_pid_t pid;
    uint16_t feedback_id;
    int16_t send_current;
    float tar_omega_speed;
} M3508_wheel_drive;

/**
 * @brief 将麦轮电机初始化；不注册硬件回调
 */
void mecnum_init(void);

/**
 * @brief 执行一次世界坐标系麦轮底盘控制：坐标转换、逆运动学解算、PID 计算及 电流计算。
 * @param vx_world 世界坐标系 x 方向目标线速度，单位 m/s。
 * @param vy_world 世界坐标系 y 方向目标线速度，x方向从上往下看逆时针转90°方向为正 ，单位 m/s。
 * @param wz 底盘绕竖直 z 轴的目标角速度，单位 rad/s，从上往下看逆时针为正；
 * @param yaw 底盘当前偏航角，单位度；世界坐标系 +x 方向为零，逆时针为正。
 * @param pitch 底盘当前俯仰角，单位度，用于坡度前馈补偿。
 * @return 计算状态：0 表示已计算，1 表示尚未初始化；不代表电调已收到或输入有效。
 * @note 调用前须调用 mecnum_init()；当前 PID 按 4 ms 控制周期计算。
 *       任一输入为 NaN 或无穷大时，重置四路 PID、清零目标转速和发送电流，
 *       并尝试生成零电流结果，返回计算状态。
 */
uint8_t mecanum_ctrl_world(float vx_world, float vy_world, float wz, float yaw, float pitch);

/**
 * @brief 执行一次车体坐标系麦轮底盘控制：逆运动学解算、轮速限幅、PID 计算及 电流计算。
 * @param vx_body 车体 x 方向目标线速度，向前为正，单位 m/s。
 * @param vy_body 车体 y 方向目标线速度，向左为正，单位 m/s。
 * @param wz 底盘绕竖直 z 轴的目标角速度，从上往下看逆时针为正，单位 rad/s。
 * @param pitch 底盘当前俯仰角，单位度，用于坡度前馈补偿。
 * @return 计算状态：0 表示已计算，1 表示尚未初始化；不代表电调已收到或输入有效。
 * @note 调用前须调用 mecnum_init()；当前 PID 按 4 ms 控制周期计算。
 *       与 mecanum_ctrl_world() 共用四路轮速 PID，每个控制周期只调用其中一个控制接口。
 *       任一输入为 NaN 或无穷大时，重置四路 PID、清零目标转速和发送电流，
 *       并尝试生成零电流结果，返回计算状态。
 */
uint8_t mecanum_ctrl_body(float vx_body, float vy_body, float wz, float pitch);

/**
 * @brief 清零四路目标转速和电流、重置 PID，并尝试生成零电流结果。
 * @return 0 表示已计算，1 表示尚未初始化。
 */
uint8_t mecanum_stop(void);

/** @brief 复制本轮四电机反馈，供旧算法速度环读取。
 * @param[in] feedback 非 NULL 完整副本；speed_rad_s 为 rad/s，返回前转换为旧 RPM 缓存。
 * @return 无返回值。有效参数下无失败。
 * @note 控制任务独占；导航索引 FR/RL/RR/FL，麦轮按其原索引；不保留地址。 */
void app_armored_mecanum_feedback(const app_can_feedback_t *feedback);
/** @brief 复制四电机计算结果。
 * @param[out] current 非 NULL，至少 4 个 int16；协议 raw，不是 A。
 * @return 无返回值。有效参数下总是覆盖全部 4 项。
 * @note 控制任务调用；没有 CAN 发送，任务须将结果提交给 CAN 所有者。 */
void app_armored_mecanum_currents(int16_t current[4]);
#endif // BNGU_CONTROL_SYSTEM_APP_ARMORED_MECANUM_C_H
