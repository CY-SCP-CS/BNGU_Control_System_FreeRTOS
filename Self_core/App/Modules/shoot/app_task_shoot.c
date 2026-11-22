/**
 * @file    app_task_shoot.c
 * @brief   发射命令快照及独立 5 ms 发射控制
 * @note    入口名、周期、业务算法及原通信对象名保持；通信对象句柄仅所属模块可访问。
 */
#include "app_rtos.h"
#include "app_task_shoot_internal.h"
#include "app_rtos_objects.h"
#include <string.h>

static osMutexId_t gimbal_to_shoot_mutex;
static app_shoot_command_t s_shoot;

/**
 * @brief  读取一份完整的 已裁决发射命令快照
 * @param  v            输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由发射控制任务在任务上下文调用；使用 gimbal_to_shoot_mutex，获取锁超时为 0。
 */
bool app_rtos_shoot_read(app_shoot_command_t *v)
{
    return app_rtos_snapshot_read(gimbal_to_shoot_mutex, &s_shoot, v, sizeof(*v));
}

/**
 * @brief  发布 已裁决发射命令快照
 * @param  v            输入快照，函数在锁内复制其内容
 * @note   写入者：ctrl_gim_Task；使用 gimbal_to_shoot_mutex，仅允许任务上下文调用。
 * @note   等待取得锁后只复制；参数/句柄无效或获取失败时忽略本次更新。
 */
void app_rtos_shoot_write(const app_shoot_command_t *v)
{
    app_rtos_snapshot_write(gimbal_to_shoot_mutex, &s_shoot, v, sizeof(*v));
}

/**
 * @brief  创建本模块全部通信对象并复位数据
 * @return true=全部创建成功，false=创建失败，由装配层统一回滚。
 * @note   只在调度器启动前调用一次；不支持运行中重新初始化。
 */
bool app_task_shoot_init(void)
{
    memset(&s_shoot, 0, sizeof(s_shoot));
    return app_rtos_object_mutex(&gimbal_to_shoot_mutex, "gimbal_to_shoot_mutex");
}

/** @brief 启动失败时删除已创建对象；句柄为 NULL 时安全跳过。 */
void app_task_shoot_deinit(void)
{
    app_rtos_object_mutex_delete(&gimbal_to_shoot_mutex);
}
