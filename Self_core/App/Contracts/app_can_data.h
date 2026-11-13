/**
 * @file    app_can_data.h
 * @brief   CAN 传输值对象；仅通信适配使用 ID，控制算法不读取队列或帧
 */
#ifndef APP_CAN_DATA_H
#define APP_CAN_DATA_H
#include "app_can_frame.h"
#include <stdint.h>
#include "app_gimbal_comm.h"
#include "app_chassis_comm.h"
#include "lib_physical.h"

/** @brief 控制指令的语义类别；控制任务不需要了解线上 ID。 */
typedef enum
{
    APP_CAN_CMD_RADAR_SPEED,
    APP_CAN_CMD_SPEED_NO_SHOOT,
    APP_CAN_CMD_SPEED_SHOOT,
    APP_CAN_CMD_ANGLE_NO_SHOOT,
    APP_CAN_CMD_ANGLE_SHOOT,
    APP_CAN_CMD_SHOOT,
    APP_CAN_CMD_CHASSIS_SPEED,
    APP_CAN_CMD_ACKERMANN,
    APP_CAN_CMD_FOLLOW
} app_can_ctrl_type_t;

/**
 * @brief  CAN 任务交给本板主控制任务的解包指令
 * @note   type 决定 value 中哪个成员有效；每条消息携带一类指令。
 *          云台板使用 APP_GIMBAL_COMM_UPDATE_* 掩码；底盘板 bit0/1/2 对应 0x111/113/115。
 */
typedef struct
{
    app_can_ctrl_type_t type; // 解包后的业务类别，决定联合体成员。
    uint32_t tick,            // 原始指令帧的接收时间戳。
        sequence;             // 原始指令帧的接收序号。
    uint8_t updated_mask;     // 该指令更新的类别位，供控制周期汇总新指令。

    union
    {
        app_gimbal_radar_cmd_t radar;          // 0x120：雷达/上位机底盘速度指令。
        app_gimbal_speed_cmd_t delta;          // 0x121/125：云台角度增量，单位 rad。
        app_gimbal_angle_cmd_t angle;          // 0x123/127：云台绝对角度，单位 rad。
        app_gimbal_shoot_cmd_t shoot;          // 0x129：发射模式控制字。
        app_chassis_speed_cmd_t speed;         // 0x111：底盘速度指令，SI 单位。
        app_chassis_ackermann_cmd_t ackermann; // 0x113：阿克曼模式参数。
        app_chassis_follow_cmd_t follow;       // 0x115：底盘跟随模式参数。
    } value;                                   // 仅按 type 读取对应成员。
} app_can_ctrl_msg_t;

/**
 * @brief  控制任务提交给 CAN 任务的结果类型
 * @note   类型决定 app_ctrl_can_msg_t.value 的有效成员和后续 CAN 编码方式。
 */
typedef enum
{
    APP_CAN_GIM_CURRENT,   // 云台三电机电流，CAN2 ID 0x1FE。
    APP_CAN_SHOOT_CURRENT, // 发射三电机电流，CAN2 ID 0x200。
    APP_CAN_CHAS_CURRENT,  // 底盘四电机电流，拆为 CAN2 ID 0x200 和 0x1FF。
    APP_CAN_CHASSIS_CMD,   // 云台下发或转发底盘速度，CAN1 ID 0x111。
    APP_CAN_GIM_FEEDBACK,  // 云台角速度、角度和四元数，CAN1 ID 0x122/124/233。
    APP_CAN_CHAS_FEEDBACK  // 底盘功率和角速度，CAN1 ID 0x112/119。
} app_ctrl_can_type_t;

/**
 * @brief  控制任务交给 CAN 任务编码的完整输出消息
 * @note   source 当前使用 TASK_* 任务索引；type 决定 value 中哪个成员有效。
 *          电流使用 DJI 电流控制协议原始值；云台/发射使用前三项，底盘使用四项。
 */
typedef struct
{
    uint32_t tick,            // 本次控制结果的生成时间戳。
        sequence;             // 来源控制任务的输出序号。
    uint8_t source;           // 输出来源任务索引，用于识别消息生产者。
    app_ctrl_can_type_t type; // 输出类别，决定联合体成员和编码协议。

    union
    {
        int16_t current[4];              // 机构电流，排列顺序与对应控制输出结构体的 current 一致。
        app_chassis_speed_cmd_t chassis; // 0x111 底盘指令，vx/vy 为 m/s，vz 为 rad/s。

        struct
        {
            float yaw_rate,   // yaw 角速度反馈，单位 rad/s。
                pitch_rate,   // pitch 电机角速度反馈，单位 rad/s。
                yaw,          // 云台 yaw 角度反馈，单位 rad。
                pitch;        // 云台 pitch 角度反馈，单位 rad。
            lib_quatf_t quat; // IMU 四元数；发送时由 CAN 编码阶段量化。
        } gimbal;             // APP_CAN_GIM_FEEDBACK 使用的姿态反馈。

        struct
        {
            float power, // 底盘滤波后的实测功率，单位 W。
                omega;   // 底盘滤波后的角速度，单位 rad/s。
        } chassis_state; // APP_CAN_CHAS_FEEDBACK 使用的底盘反馈。
    } value;             // 仅按 type 读取对应成员。
} app_ctrl_can_msg_t;
#endif
