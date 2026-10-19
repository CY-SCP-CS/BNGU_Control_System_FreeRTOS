/**
 * @file    app_control_input.h
 * @brief   控制源和归一化遥控输入；独立于 CAN 协议和 RTOS 快照
 */
#ifndef APP_CONTROL_INPUT_H
#define APP_CONTROL_INPUT_H
#include <stdint.h>

/** S2 三挡输入源；遥控器的物理挡位从上到下为 1、3、2。 */
typedef enum
{
    APP_GIMBAL_INPUT_DBUS = 1U, // S2 上挡：使用遥控器。
    APP_GIMBAL_INPUT_CAN = 3U,  // S2 中挡：使用 CAN 上位机指令。
    APP_GIMBAL_INPUT_ESTOP = 2U // S2 下挡：急停。
} app_gimbal_input_source_t;

/** 遥控器采样值，通信层只归一化，不乘各机构的速度上限。 */
typedef struct
{
    app_gimbal_input_source_t source; // 当前 S2 选择的控制源。
    float chassis_vx_norm;            // ch0 归一化后的底盘 x 指令，范围 [-1, 1]。
    float chassis_vy_norm;            // ch1 归一化后的底盘 y 指令，范围 [-1, 1]。
    float chassis_omega_norm;         // 拨轮归一化后的底盘角速度指令，范围 [-1, 1]。
    float yaw_rate_norm;              // ch2 归一化后的 yaw 角速度指令，范围 [-1, 1]。
    float pitch_rate_norm;            // ch3 归一化后的 pitch 角速度指令，范围 [-1, 1]。
    uint8_t launcher_mode;            // S1 三挡发射机构控制字。
} app_gimbal_dbus_input_t;

#endif
