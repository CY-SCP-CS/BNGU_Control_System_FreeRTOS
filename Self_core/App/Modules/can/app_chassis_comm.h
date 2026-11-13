/**
 * @file    app_chassis_comm.h
 * @brief   底盘 CAN1 板间通信协议：接收控制指令，发送功率与角速度反馈
 * @note    底盘坐标系：前为 x、左为 y、上为 z；绕 z 轴逆时针为正。
 *          多字节数据按小端序编码；STM32F4 可直接复制整数和 float 字节。
 */
#ifndef APP_CHASSIS_COMM_H
#define APP_CHASSIS_COMM_H

#include "lib_typedef.h"

#define APP_CHASSIS_CAN_ID_SPEED_CMD 0x111U      // 底盘速度指令。
#define APP_CHASSIS_CAN_ID_POWER_FEEDBACK 0x112U // 实际功率反馈。
#define APP_CHASSIS_CAN_ID_ACKERMANN_CMD 0x113U  // 阿克曼底盘指令。
#define APP_CHASSIS_CAN_ID_FOLLOW_CMD 0x115U     // 跟随模式指令。
#define APP_CHASSIS_CAN_ID_OMEGA_FEEDBACK 0x119U // 底盘角速度反馈。
#define APP_CHASSIS_OMEGA_RAD_S_PER_LSB 0.001f   // 0x111 中 vz 的量化单位，rad/s。

/** 0x111 底盘速度指令。 */
typedef struct
{
    float vx;        // x 方向速度，单位 m/s，向前为正。
    float vy;        // y 方向速度，单位 m/s，向左为正。
    float vz;        // z 轴角速度 ，单位 rad/s，逆时针为正。
    float power_pct; // 功率比例，无量纲，1 表示 100%，当前由底盘协议保留。
} app_chassis_speed_cmd_t;

/** 0x113 阿克曼底盘指令。 */
typedef struct
{
    float speed;       // 车体线速度，单位 m/s，向前为正。
    float steer_angle; // 转向角，单位 rad，向左为正。
} app_chassis_ackermann_cmd_t;

/** 0x115 跟随模式指令。 */
typedef struct
{
    float vx;           // x 方向速度，单位 m/s，向前为正。
    float vy;           // y 方向速度，单位 m/s，向左为正。
    float gimbal_angle; // 云台角度 ，单位 rad，向左为正。
    int16_t custom;     // 协议预留字段。
} app_chassis_follow_cmd_t;

/** 底盘板接收并解包后的全部控制数据。 */
typedef struct
{
    app_chassis_speed_cmd_t speed;         // 0x111。
    app_chassis_ackermann_cmd_t ackermann; // 0x113。
    app_chassis_follow_cmd_t follow;       // 0x115。
} app_chassis_comm_rx_t;

/* 纯协议接口：不保存全局命令状态，rx 由调用任务持有。 */
/**
 * @brief  解包一帧底盘 CAN1 指令，更新对应字段及本帧更新掩码。
 * @param  id           CAN 标准 ID，支持 0x111、0x113、0x115。
 * @param  bytes        只读载荷，函数内部复制数据，不保存该地址。
 * @param  len          载荷长度，必须为 8 字节。
 * @param  rx           输入/输出：调用者持有的完整协议状态，仅覆盖本 ID 对应字段。
 * @param  updated_mask 输出：0x111/0x113/0x115 分别对应位 0/1/2；成功时赋值。
 * @return 1=识别并接受该帧，0=参数、长度、ID 或浮点数据无效；失败时不要使用输出掩码。
 * @note   纯协议函数，不操作队列、锁或 HAL。当前控制算法使用 0x111，另两类指令只解析存储。
 * @note   多字节字段沿用 STM32 小端协议；多个帧的掩码应由调用者按位或汇总。
 */
uint8_t app_chassis_comm_decode(uint32_t id, const uint8_t *bytes, uint8_t len,
                                app_chassis_comm_rx_t *rx, uint8_t *updated_mask);

#endif
