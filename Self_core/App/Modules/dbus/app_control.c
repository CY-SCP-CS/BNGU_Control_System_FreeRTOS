/**
 * @file    app_control.c
 * @brief   控制输入有效性检查：不依赖调度器和硬件。
 */
#include "lib_math.h"
#include "app_control.h"

/**
 * @brief  检查 DBUS 开关与四个摇杆通道是否处于当前允许范围。
 * @param  data         已完成 18 字节解包的遥控数据，调用者必须提供非 NULL 指针。
 * @return 1=S1/S2 均为 1~3 且 ch[0..3] 均在 [364,1684]；否则返回 0。
 * @note   由 DBUS 任务在发布快照前调用；本检查不校验拨轮、鼠标和键盘，也不代表模块仍在线。
 */
uint8_t app_control_dbus_valid(const drv_dbus_data_t *data)
{
    if (data->rc.s1 < 1 || data->rc.s1 > 3 || data->rc.s2 < 1 || data->rc.s2 > 3)
    {
        return 0;
    }
    for (unsigned i = 0; i < 4; i++)
    {
        if (data->rc.ch[i] < 364 || data->rc.ch[i] > 1684)
        {
            return 0;
        }
    }
    return 1;
}

/**
 * @brief  将已校验的 DBUS 数据转换为归一化输入
 * @param  dbus  解包且通过有效性检查的遥控数据。
 * @param  input 输出控制源、发射模式及 [-1,1] 的摇杆/拨轮值。
 * @return 1=成功；0=空指针。输入校验和快照有效位由调用者负责。
 * @note   保留原函数名；此转换属于输入处理，不依赖 CAN 或快照类型。
 */
uint8_t app_dbus_map_input(const drv_dbus_data_t *dbus, app_remote_input_t *input)
{
    if (!dbus || !input)
    {
        return 0;
    }
    input->launcher_mode = dbus->rc.s1;
    input->chassis_vx_norm = lib_clamp(((float)dbus->rc.ch[0] - 1024) / 660, -1, 1);
    input->chassis_vy_norm = lib_clamp(((float)dbus->rc.ch[1] - 1024) / 660, -1, 1);
    input->yaw_rate_norm = lib_clamp(((float)dbus->rc.ch[2] - 1024) / 660, -1, 1);
    input->pitch_rate_norm = lib_clamp(((float)dbus->rc.ch[3] - 1024) / 660, -1, 1);
    input->chassis_omega_norm = lib_clamp(((float)dbus->rc.rolling_wheel - 1024) / 660, -1, 1);
    if (dbus->rc.s2 == DRV_DBUS_SWITCH_UP)
    {
        input->source = APP_REMOTE_INPUT_DBUS;
    }
    else if (dbus->rc.s2 == DRV_DBUS_SWITCH_MIDDLE)
    {
        input->source = APP_REMOTE_INPUT_CAN;
    }
    else
    {
        input->source = APP_REMOTE_INPUT_ESTOP;
    }
    return 1;
}
