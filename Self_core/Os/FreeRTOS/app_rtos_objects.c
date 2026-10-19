/**
 * @file    app_rtos_objects.c
 * @brief   对象创建与快照复制的公共机制，不保存业务数据
 */

#include "app_rtos_objects.h"
#include <string.h>

/** @brief 按模块传入的类型大小与容量创建队列，允许启动阶段重复调用。 */
bool app_rtos_object_queue(osMessageQueueId_t *queue, const char *name, uint32_t depth,
                           uint32_t size)
{
    const osMessageQueueAttr_t attr = {.name = name};
    if (!*queue)
    {
        *queue = osMessageQueueNew(depth, size, &attr);
    }
    return *queue != NULL;
}

/** @brief 创建短快照复制使用的优先级继承互斥量。 */
bool app_rtos_object_mutex(osMutexId_t *mutex, const char *name)
{
    const osMutexAttr_t attr = {.name = name, .attr_bits = osMutexPrioInherit};
    if (!*mutex)
    {
        *mutex = osMutexNew(&attr);
    }
    return *mutex != NULL;
}

/** @brief 启动失败回滚；任务全部终止后才可删除对象。 */
void app_rtos_object_queue_delete(osMessageQueueId_t *queue)
{
    if (*queue)
    {
        osMessageQueueDelete(*queue);
        *queue = NULL;
    }
}

/** @brief 启动失败回滚；运行时不得删除仍被使用的互斥量。 */
void app_rtos_object_mutex_delete(osMutexId_t *mutex)
{
    if (*mutex)
    {
        osMutexDelete(*mutex);
        *mutex = NULL;
    }
}

/** @brief 任务非阻塞读取完整副本；失败不改写 local，锁内只复制。 */
bool app_rtos_snapshot_read(osMutexId_t mutex, const void *shared, void *local, size_t size)
{
    if (!mutex || !shared || !local || osMutexAcquire(mutex, 0) != osOK)
    {
        return false;
    }
    memcpy(local, shared, size);
    osMutexRelease(mutex);
    return true;
}

/** @brief 任务取得锁后复制完整快照；计算、格式化和外设发送必须在锁外。 */
void app_rtos_snapshot_write(osMutexId_t mutex, void *shared, const void *local, size_t size)
{
    if (!mutex || !shared || !local)
    {
        return;
    }
    if (osMutexAcquire(mutex, osWaitForever) == osOK)
    {
        memcpy(shared, local, size);
        osMutexRelease(mutex);
    }
}
