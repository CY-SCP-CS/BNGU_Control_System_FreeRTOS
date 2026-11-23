/**
 * @file    app_gimbal_comm.h
 * @brief   云台 CAN1 板间通信协议：接收控制指令，发送云台状态反馈
 * @note    云台坐标系：前为 x、左为 y、上为 z；yaw 左转为正，pitch 下转为正。
 *          多字节数据按小端序编码；STM32F4 可直接复制整数和 float 字节。
 */
#ifndef APP_GIMBAL_COMM_H
#define APP_GIMBAL_COMM_H

#include "App/Modules/can/app_can_commands.h"

#define APP_GIMBAL_CAN_ID_RADAR_SPEED 0x120U           // 雷达/上位机底盘速度指令。
#define APP_GIMBAL_CAN_ID_SPEED_NO_SHOOT 0x121U        // 无射击云台角速度指令。
#define APP_GIMBAL_CAN_ID_SPEED_FEEDBACK 0x122U        // 云台角速度反馈。
#define APP_GIMBAL_CAN_ID_ANGLE_NO_SHOOT 0x123U        // 无射击云台绝对角度指令。
#define APP_GIMBAL_CAN_ID_ANGLE_FEEDBACK 0x124U        // 云台绝对角度反馈。
#define APP_GIMBAL_CAN_ID_SPEED_SHOOT 0x125U           // 带射击云台角速度指令。
#define APP_GIMBAL_CAN_ID_SHOOT_FEEDBACK 0x126U        // 发射机构状态反馈。
#define APP_GIMBAL_CAN_ID_ANGLE_SHOOT 0x127U           // 带射击云台绝对角度指令。
#define APP_GIMBAL_CAN_ID_SHOOT_CMD 0x129U             // 发射机构控制指令，DLC 为 4。
#define APP_GIMBAL_CAN_ID_ANGLE_FEEDBACK_V2 0x130U     // 预留的角度反馈 V2。
#define APP_GIMBAL_CAN_ID_IMU_QUATERNION 0x233U        // IMU 四元数反馈。
#define APP_GIMBAL_CAN_ID_DM_IMU_REQUEST 0x31U         // 预留的达妙 IMU 请求。
#define APP_GIMBAL_CAN_ID_DM_IMU_RESPONSE 0x39U        // 预留的达妙 IMU 响应。
#define APP_GIMBAL_ANGLE_SHOOT_TARGET_DETECTED 0xFFFFU // 角度射击协议的目标检测标志。

/* 纯协议接口：不保存全局命令状态，rx 由调用任务持有。 */
/**
 * @brief  按 CAN ID 解包云台指令，返回本帧对应的更新位。
 * @param  id           CAN 标准 ID，支持 0x120/0x121/0x123/0x125/0x127/0x129。
 * @param  bytes        只读载荷，数据复制到 rx，函数不保留输入指针。
 * @param  len          0x120 为 8 字节；0x121/123/125/127 为 4 字节；0x129 为 2 字节。
 * @param  rx           输入/输出：调用者的协议缓存，仅覆盖当前 ID 对应字段。
 * @param  updated_mask 输出：APP_GIMBAL_COMM_UPDATE_* 中对应的一位；成功时赋值。
 * @return 1=解包成功，0=空参数、未知 ID、长度不符或角度含非有限浮点数。
 * @note   任务上下文的纯协议函数；0x121/0x125 表示目标角速度，单位 rad/s。
 * @note   每帧的更新位由调用者汇总；同 ID 的最新目标覆盖旧值，速度在有效期内按控制间隔积分。
 */
uint8_t app_gimbal_comm_decode(uint32_t id, const uint8_t *bytes, uint8_t len,
                               app_gimbal_comm_rx_t *rx, uint8_t *updated_mask);

#endif
