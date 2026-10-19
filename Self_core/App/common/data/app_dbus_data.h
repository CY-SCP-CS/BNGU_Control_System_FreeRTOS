/**
 * @file    app_dbus_data.h
 * @brief   遥控输入快照契约
 */
#ifndef APP_DBUS_DATA_H
#define APP_DBUS_DATA_H
#include <stdint.h>
#include "drv_dbus.h"
#include "app_control_input.h"

/**
 * @brief  DBUS 任务发布给云台控制的遥控输入快照
 * @note   一次发布完整解包值和归一化值，读者不会读到半更新数据。
 *          data 保存解包结果，input 保存归一化输入；机构速度上限由控制算法应用。
 */
typedef struct app_dbus_snapshot
{
    drv_dbus_data_t data;          // 解包后的通道、开关、鼠标和键盘数据。
    app_gimbal_dbus_input_t input; // 控制源、归一化摇杆/拨轮输入和发射模式。
    uint32_t tick,                 // 本帧接收时间戳。
        sequence;                  // 本帧接收序号。
    uint8_t valid;                 // DBUS 解包与输入校验通过标志。
} app_dbus_snapshot_t;
#endif
