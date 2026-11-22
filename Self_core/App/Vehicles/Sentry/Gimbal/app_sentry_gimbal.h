/**
 * @file    app_sentry_gimbal.h
 * @brief   哨兵云台算法接口：输入完整快照，输出电流与已裁决命令。
 */
#ifndef APP_SENTRY_GIMBAL_H
#define APP_SENTRY_GIMBAL_H
#include "App/Vehicles/Sentry/Contracts/app_gimbal_data.h"

/** @brief 云台算法内部使用的控制命令，角度为 rad，角速度为 rad/s。 */

/**
 * @brief  清零云台内部状态，装载 PID 参数并初始化角速度滤波器。
 * @note   仅在控制任务开始前调用；后续内部目标、PID 积分和滤波状态由 ctrl_gim_Task 独占。
 */
void app_sentry_gimbal_init(void);
/**
 * @brief  完成一次 1 ms 云台计算，并给出发射命令和底盘转发命令。
 * @param  input        本周期完整私有快照：DBUS、IMU、CAN 反馈、协议状态及本周期更新掩码；不可为
 * NULL。
 * @param  output       输出：大 yaw/小 yaw/pitch 电流、目标与反馈、发射命令及底盘命令；不可为
 * NULL。
 * @note   由 ctrl_gim_Task 按 osDelayUntil() 调用；函数内部不等待 DMA、不操作 RTOS 对象，也不发送
 * CAN。
 * @note   算法步长来自 APP_CONTROL_GIM_DT_S；输入通过参数传递，不保存其地址。
 * @note   output->shoot.ready 表示首次反馈目标对齐完成；CAN 编码和快照发布由任务层处理。
 */
void app_sentry_gimbal_ctrl_1khz(const app_gimbal_input_t *input, app_gimbal_output_t *output);
/* Entry validates sample ages using app_input_config.h. Callers supply kernel tick
 * timestamps and explicit valid bits; zero timestamps are valid. Single owning task only.
 * Expired required inputs stop/reset this mechanism; see docs/ARCHITECTURE_REFACTOR_20261113.md. */
#endif
