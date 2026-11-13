/**
 * @file    app_control.h
 * @brief   控制输入有效性检查：不依赖调度器和硬件。
 */
#ifndef APP_CONTROL_H
#define APP_CONTROL_H
#include "app_control_input.h"
#include "drv_dbus.h"

/**
 * @brief  检查 DBUS 开关与四个摇杆通道是否处于当前允许范围。
 * @param  data         已完成 18 字节解包的遥控数据，调用者必须提供非 NULL 指针。
 * @return 1=S1/S2 均为 1~3 且 ch[0..3] 均在 [364,1684]；否则返回 0。
 * @note   由 DBUS 任务在发布快照前调用；本检查不校验拨轮、鼠标和键盘，也不代表模块仍在线。
 */
uint8_t app_control_dbus_valid(const drv_dbus_data_t *data);
/**
 * @brief  将已校验的 DBUS 数据转换为归一化输入
 * @param  dbus  解包且通过有效性检查的遥控数据。
 * @param  input 输出控制源、发射模式及 [-1,1] 的摇杆/拨轮值。
 * @return 1=成功；0=空指针。输入校验和快照有效位由调用者负责。
 * @note   保留原函数名；此转换属于输入处理，不依赖 CAN 或快照类型。
 */
uint8_t app_gimbal_comm_dbus_rx(const drv_dbus_data_t *dbus, app_gimbal_dbus_input_t *input);
#endif
