/**
 * @file    app_sentry_shoot.h
 * @brief   哨兵发射机构算法：按已裁决模式计算摩擦轮和拨弹轮电流。
 * @note    发射任务独占 PID；调度、快照获取和 CAN 提交由 App/Tasks 完成。
 */
#ifndef APP_SENTRY_SHOOT_H
#define APP_SENTRY_SHOOT_H
#include "App/Vehicles/Sentry/Contracts/app_shoot_data.h"

/**
 * @brief  清空发射反馈缓存并初始化拨弹轮和两只摩擦轮速度 PID。
 * @note   由初始化任务调用；后续 PID 和内部电流状态只由 ctrl_shoot_Task 修改。
 */
void app_sentry_shoot_init(void);
/**
 * @brief  完成一次 5 ms 发射控制并输出三路电流和七个波形值。
 * @param  input        已裁决发射命令及电机反馈快照，不可为 NULL。
 * @param  output       输出电流顺序为左摩擦轮、拨弹轮、右摩擦轮；不可为 NULL。
 * @note   由 ctrl_shoot_Task 周期调用；步长来自 APP_CONTROL_SHOOT_DT_S，默认 5 ms。
 * @note   ready 未置位或 emergency_stop 有效时清零电流并复位 PID；其他情况按 fire 模式控制。
 * @note   本函数不再直接读取 DBUS，也不发送 CAN；模式已由云台控制任务统一裁决。
 */
void app_sentry_launcher_ctrl_200hz(const app_shoot_input_t *, app_shoot_output_t *);
/* Entry validates sample ages using app_input_config.h. Callers supply kernel tick
 * timestamps and explicit valid bits; zero timestamps are valid. Single owning task only.
 * Expired required inputs stop/reset this mechanism; see docs/ARCHITECTURE_REFACTOR_20261113.md. */
#endif
