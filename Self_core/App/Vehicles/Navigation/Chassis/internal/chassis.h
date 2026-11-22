//
// Created by 33140 on 2026/11/3.
//

#ifndef BNGU_CONTROL_SYSTEM_CHASSIS_H
#define BNGU_CONTROL_SYSTEM_CHASSIS_H

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
} NAV_M3508_wheel_drive;

/** @brief 初始化四轮 PID 和控制就绪状态。
 * @return 无返回值。静态参数配置无运行时失败。
 * @note 系统初始化任务调用，运行时控制任务独占；不操作 CAN。 */
void chassis_init(void);

/**
 * @brief 执行一次世界坐标系底盘控制：坐标转换、逆解算、PID 计算及 电流计算。
 * @param vx_world 世界坐标系 x 方向目标线速度，单位 m/s。
 * @param vy_world 世界坐标系 y 方向目标线速度，+x 逆时针转 90° 为 +y，单位 m/s。
 * @param wz 底盘绕中心的目标角速度，俯视逆时针为正，单位 rad/s。
 * @param yaw 底盘偏航角，车头沿世界 +x 为零、逆时针为正，单位度。
 * @return 0 表示 计算完成，1 表示尚未初始化；不代表电调已收到。
 * @note 调用前须调用 chassis_init()；每 5 ms 调用一次。
 *       任一输入为 NaN 或无穷大时调用 chassis_stop()，清零目标和电流并重置 PID。
 */
uint8_t chassis_ctrl_world(float vx_world, float vy_world, float wz, float yaw);

/**
 * @brief 执行一次车体坐标系底盘控制：逆解算、轮速限幅、PID 计算及 电流计算。
 * @param vx_body 车体 x 方向目标线速度，向前为正，单位 m/s。
 * @param vy_body 车体 y 方向目标线速度，向左为正，单位 m/s。
 * @param wz 底盘绕中心的目标角速度，俯视逆时针为正，单位 rad/s。
 * @return 0 表示 计算完成，1 表示尚未初始化；不代表电调已收到。
 * @note 调用前须调用 chassis_init()；每 5 ms 与世界坐标系接口二选一调用。
 *       任一输入为 NaN 或无穷大时调用 chassis_stop()，清零目标和电流并重置 PID。
 */
uint8_t chassis_ctrl_body(float vx_body, float vy_body, float wz);

/**
 * @brief 清零四轮目标转速和电流、重置 PID，并生成零电流结果。
 * @return 0 表示 计算完成，1 表示尚未初始化。
 */
uint8_t chassis_stop(void);

/** @brief 复制本轮四电机反馈，供旧算法速度环读取。
 * @param[in] feedback 非 NULL 完整副本；speed_rad_s 为 rad/s，返回前转换为旧 RPM 缓存。
 * @return 无返回值。有效参数下无失败。
 * @note 控制任务独占；导航索引 FR/RL/RR/FL，麦轮按其原索引；不保留地址。 */
void navigation_chassis_feedback(const app_can_feedback_t *feedback);
/** @brief 复制四电机计算结果。
 * @param[out] current 非 NULL，至少 4 个 int16；协议 raw，不是 A。
 * @return 无返回值。有效参数下总是覆盖全部 4 项。
 * @note 控制任务调用；没有 CAN 发送，任务须将结果提交给 CAN 所有者。 */
void navigation_chassis_currents(int16_t current[4]);
#endif // BNGU_CONTROL_SYSTEM_CHASSIS_H
