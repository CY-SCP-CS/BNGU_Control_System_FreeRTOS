/**
 * @file    app_gimbal_data.h
 * @brief   云台控制输入与输出
 */
#ifndef APP_GIMBAL_DATA_H
#define APP_GIMBAL_DATA_H
#include "app_gimbal_comm.h"
#include <stdint.h>
#include "app_dbus_data.h"
#include "app_imu_data.h"
#include "app_feedback_data.h"
#include "app_shoot_data.h"
#include "app_chassis_comm.h"

/**
 * @brief  云台单次控制计算使用的完整输入
 * @note   任务先取得各输入副本，再调用算法；updated_mask 每周期重新汇总。
 *          增量指令只在本周期对应更新位有效时应用一次。
 */
typedef struct
{
    app_dbus_snapshot_t dbus;    // 最近取得的完整遥控输入副本。
    app_imu_snapshot_t imu;      // 最近取得的完整 IMU 物理量和姿态副本。
    app_can_feedback_t feedback; // 本板电机及底盘角速度反馈副本。
    app_gimbal_comm_rx_t can;    // 控制任务保存的各类最新 CAN 指令。
    uint8_t updated_mask;        // 本周期收到的 APP_GIMBAL_COMM_UPDATE_* 位组合。
    uint32_t tick;               // 本次云台控制执行的系统时间戳。
} app_gimbal_input_t;

/**
 * @brief  云台单次控制计算产生的电流、命令和状态
 * @note   current 使用 DJI 电流控制协议原始值，角度使用 rad，角速度使用 rad/s。
 *          任务分别发布发射命令、控制状态，并提交需要 CAN 编码的结果。
 */
typedef struct
{
    int16_t current[3];        // 电流顺序：大 yaw、小 yaw、pitch。
    app_shoot_command_t shoot; // 已裁决的发射模式、控制源、就绪和急停状态。
    float yaw;                 /**< 当前对地 yaw，rad。 */
    float pitch;               /**< IMU roll 与 pitch 关节融合后的角度，rad。 */
    float roll;                /**< 控制机体系横滚角，rad。 */
    float yaw_target;          /**< yaw 目标，rad。 */
    float pitch_target;        /**< pitch 目标，rad。 */
    float yaw_rate;            /**< 滤波后的 yaw 角速度，rad/s。 */
    float pitch_rate;          /**< pitch 电机角速度，rad/s。 */
    lib_quatf_t quat; /**< 控制 yaw/pitch/roll 重建的 ZYX 四元数，包含 pitch 关节补偿，无量纲。 */
    app_chassis_speed_cmd_t chassis; // 准备下发的底盘速度字段，SI 单位，保留急停语义。
} app_gimbal_output_t;
#endif
