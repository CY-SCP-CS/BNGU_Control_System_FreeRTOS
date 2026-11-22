/**
 * @file    app_task_telemetry.h
 * @brief   控制遥测服务；不公开任务入口、设备实例或通信对象句柄。
 */
#ifndef APP_TASK_TELEMETRY_H
#define APP_TASK_TELEMETRY_H
#include <stdbool.h>
#include "App/Modules/telemetry/app_ctrl_data.h"

/**
 * @brief  读取最近发布的完整控制状态。
 * @param[out] value 非 NULL 输出，容量为一份 app_ctrl_state_t；失败时不改写。
 * @return true=完整复制成功；false=参数无效、尚未创建对象或锁忙。
 * @note   仅任务上下文调用，取锁超时为 0；返回值属于各机构独立发布时间。
 */
bool app_rtos_ctrl_read(app_ctrl_state_t *value);
/**
 * @brief  提取原 VOFA 九通道显示值，不复制整份控制状态。
 * @param[out] value 至少容纳 9 个 float 的独占输出缓冲；失败时全部保留原值。
 * @return true=九通道已复制；false=参数无效、尚未创建对象或锁忙。
 * @note   仅任务上下文调用，取锁超时为 0，无共享读取工作区。
 *         底盘通道 0..3 为目标/实际 vx、vy（mm/s），4..5 为目标/实际角速度（rad/s），
 *         6 为功率（W），7 为功率缩放比，8 为 0。
 *         云台板通道 0..1 为拨弹目标/实际角速度（rad/s），2 为 PID 积分输出 raw，
 *         3 为反馈电流 raw，4..7 为右目标/右实际/左目标/左实际角速度（rad/s），
 *         8 为拨弹编码器 raw；这些显示映射不改变业务契约单位。
 */
bool app_rtos_vofa_read(float value[9]);
/**
 * @brief  发布云台控制结果和 IMU roll 诊断值。
 * @param[in] value 非 NULL 云台输出；立即复制，不保留地址。
 * @param[in] roll IMU roll，单位 rad；只保存诊断，不占用发射波形通道。
 * @param[in] tick 本轮控制时间，单位内核 tick。
 * @return 无返回值；锁忙保留旧结果并累计 COUNT_TELEMETRY_DROP。
 * @note   仅云台控制任务调用，取锁超时为 0，不修改发射和底盘字段。
 */
void app_rtos_gimbal_publish(const app_gimbal_output_t *value, float roll, uint32_t tick);
/**
 * @brief  发布底盘控制结果。
 * @param[in] value 非 NULL 底盘输出；立即复制，不保留地址，物理量采用业务 SI 单位。
 * @param[in] tick 本轮控制时间，单位内核 tick。
 * @return 无返回值；锁忙保留旧结果并累计 COUNT_TELEMETRY_DROP。
 * @note   仅底盘控制任务调用，取锁超时为 0，不修改云台和发射字段。
 */
void app_rtos_chassis_publish(const app_chassis_output_t *value, uint32_t tick);
/**
 * @brief  发布发射控制结果及其具名遥测字段。
 * @param[in] value 非 NULL 发射输出；立即复制，不保留地址，字段单位见业务契约。
 * @param[in] tick 本轮控制时间，单位内核 tick。
 * @return 无返回值；锁忙保留旧结果并累计 COUNT_TELEMETRY_DROP。
 * @note   仅发射任务调用，取锁超时为 0，不修改云台和底盘字段。
 */
void app_rtos_shoot_publish(const app_shoot_output_t *value, uint32_t tick);
#endif
