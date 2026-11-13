/**
 * @file    app_dbus_data.h
 * @brief   遥控输入快照契约
 */
#ifndef APP_DBUS_DATA_H
#define APP_DBUS_DATA_H
#include <stdint.h>
#include "app_control_input.h"

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
    app_gimbal_dbus_input_t input; // 控制源、归一化摇杆/拨轮输入和发射模式。
    uint32_t tick,                 // 本帧接收时间戳。
        sequence;                  // 本帧接收序号。
    uint8_t valid;                 // DBUS 解包与输入校验通过标志。
} app_dbus_snapshot_t;
#endif
