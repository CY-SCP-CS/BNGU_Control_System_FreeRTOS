/**
 * @file    app_rtos_objects.h
 * @brief   通用队列、优先级继承互斥量及快照复制；不依赖具体任务或板型
 */
#ifndef APP_RTOS_OBJECTS_H
#define APP_RTOS_OBJECTS_H
#include <stdbool.h>
#include <stddef.h>
#include "cmsis_os2.h"

/* 创建只在调度器启动前进行；删除只用于任务尚未运行时的失败回滚。
 * 快照锁内仅复制：读者超时 0，写者等待锁；均不得在中断中调用。 */
bool app_rtos_object_queue(osMessageQueueId_t *queue, const char *name, uint32_t depth,
                           uint32_t size);
bool app_rtos_object_mutex(osMutexId_t *mutex, const char *name);
void app_rtos_object_queue_delete(osMessageQueueId_t *queue);
void app_rtos_object_mutex_delete(osMutexId_t *mutex);
bool app_rtos_snapshot_read(osMutexId_t mutex, const void *shared, void *local, size_t size);
void app_rtos_snapshot_write(osMutexId_t mutex, void *shared, const void *local, size_t size);

#endif
