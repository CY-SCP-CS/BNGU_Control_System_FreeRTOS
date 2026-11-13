/** @file app_task_shoot.h
 * @brief shoot 跨模块服务；无设备实例、任务入口或通信句柄。 */
#ifndef APP_TASK_SHOOT_H
#define APP_TASK_SHOOT_H
#include <stdbool.h>
#include "app_shoot_data.h"
/**
 * @brief  发布 已裁决发射命令快照
 * @param[in] value            输入快照，函数在锁内复制其内容
 * @note   写入者：ctrl_gim_Task；使用 dbus_to_shoot_mutex，仅允许任务上下文调用。
 * @note   等待取得锁后只复制；参数/句柄无效或获取失败时忽略本次更新。
  * @return 无返回值。失败按上述接口策略处理。
 */
void app_rtos_shoot_write(const app_shoot_command_t *value);
/**
 * @brief  读取一份完整的 已裁决发射命令快照
 * @param[in] value            输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由发射控制任务在任务上下文调用；使用 dbus_to_shoot_mutex，获取锁超时为 0。
 */
bool app_rtos_shoot_read(app_shoot_command_t *value);
#endif
