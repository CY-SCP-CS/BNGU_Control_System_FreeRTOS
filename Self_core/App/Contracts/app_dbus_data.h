/**
 * @file    app_dbus_data.h
 * @brief   遥控输入快照契约
 */
#ifndef APP_DBUS_DATA_H
#define APP_DBUS_DATA_H
#include <stdint.h>

/** S2 三挡输入源；遥控器的物理挡位从上到下为 1、3、2。 */
typedef enum
{
    APP_REMOTE_INPUT_DBUS = 1U, // S2 上挡：使用遥控器。
    APP_REMOTE_INPUT_CAN = 3U,  // S2 中挡：使用 CAN 上位机指令。
    APP_REMOTE_INPUT_ESTOP = 2U // S2 下挡：急停。
} app_remote_input_source_t;

/** 遥控器采样值，通信层只归一化，不乘各机构的速度上限。 */
typedef struct
{
    app_remote_input_source_t source; // 当前 S2 选择的控制源。
    float chassis_vx_norm;            // ch0 归一化后的底盘 x 指令，范围 [-1, 1]。
    float chassis_vy_norm;            // ch1 归一化后的底盘 y 指令，范围 [-1, 1]。
    float chassis_omega_norm;         // 拨轮归一化后的底盘角速度指令，范围 [-1, 1]。
    float yaw_rate_norm;              // ch2 归一化后的 yaw 角速度指令，范围 [-1, 1]。
    float pitch_rate_norm;            // ch3 归一化后的 pitch 角速度指令，范围 [-1, 1]。
    uint8_t launcher_mode;            // S1 三挡发射机构控制字。
} app_remote_input_t;

/**
 * @brief  DBUS 任务发布给云台控制的遥控输入快照
 * @note   一次发布完整解包值和归一化值，读者不会读到半更新数据。
 *          input 保存完整归一化输入；原解包结构由 DBUS 模块独占；机构速度上限由控制算法应用。
 */
typedef struct app_dbus_snapshot
{
    float
        channel_norm[4]; /**< DBUS channels 0..3 normalized to [-1,1]; no driver type dependency. */
    uint8_t switch_s1;   /**< Physical switch encoding: up=1, down=2, middle=3. */
    uint8_t switch_s2;   /**< Physical switch encoding: up=1, down=2, middle=3. */
    app_remote_input_t input; // 控制源、归一化摇杆/拨轮输入和发射模式。
    uint32_t tick,            // 本帧接收时间戳。
        sequence;             // 本帧接收序号。
    uint8_t seen;             // At least one frame observed; independent of sequence wrap.
    uint8_t valid;            // DBUS 解包与输入校验通过标志。
} app_dbus_snapshot_t;
#endif
