/** @file app_task_telemetry.h
 * @brief telemetry 跨模块服务；无设备实例、任务入口或通信句柄。 */
#ifndef APP_TASK_TELEMETRY_H
#define APP_TASK_TELEMETRY_H
#include <stdbool.h>
#include "app_ctrl_data.h"
/**
 * @brief  读取完整控制状态快照
 * @param[in] value            输出控制状态，锁不可用时保留旧副本
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   任务上下文调用，ctrl_state_mutex 保护；超时为 0。
 */
bool app_rtos_ctrl_read(app_ctrl_state_t *value);
/**
 * @brief  读取控制快照中的 VOFA 九通道
 * @param[in] value            至少容纳 8 个 float 的输出数组，失败保留旧值
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   使用 ctrl_state_mutex，任务上下文获取锁超时为 0。
 */
bool app_rtos_vofa_read(float value[9]);
/**
 * @brief  发布云台控制输出与 IMU roll 波形
 * @param[in] value            有效的云台输出指针
 * @param[in] roll         IMU roll，单位度，写入 VOFA 通道 7
 * @param[in] tick         本轮控制时间，单位内核 tick
 * @note   由云台控制任务调用；持有 ctrl_state_mutex 只更新云台字段，保留发射字段。
  * @return 无返回值。失败按上述接口策略处理。
 */
void app_rtos_gimbal_publish(const app_gimbal_output_t *value, float roll, uint32_t tick);
/**
 * @brief  发布底盘控制输出及 VOFA 九通道
 * @param[in] value            有效的底盘输出指针
 * @param[in] tick         本轮控制时间，单位内核 tick
 * @note   由底盘控制任务调用；通道依次为 x/y/omega 目标与实际值、功率、缩放系数。
  * @return 无返回值。失败按上述接口策略处理。
 */
void app_rtos_chassis_publish(const app_chassis_output_t *value, uint32_t tick);
/**
 * @brief  发布发射输出及原发射 VOFA 七通道
 * @param[in] value            有效的发射输出指针，包含电流及具名遥测值
 * @param[in] tick         本轮控制时间，单位内核 tick
 * @note   由发射任务调用；持有 ctrl_state_mutex 更新发射字段，保留云台发布的通道 7。
  * @return 无返回值。失败按上述接口策略处理。
 */
void app_rtos_shoot_publish(const app_shoot_output_t *value, uint32_t tick);
#endif
