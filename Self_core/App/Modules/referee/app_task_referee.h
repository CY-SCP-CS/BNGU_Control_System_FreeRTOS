/** @file app_task_referee.h
 * @brief referee 跨模块服务；无设备实例、任务入口或通信句柄。 */
#ifndef APP_TASK_REFEREE_H
#define APP_TASK_REFEREE_H
#include <stdbool.h>
#include "app_referee_data.h"
/**
 * @brief  读取一份完整的 裁判状态快照
 * @param[out] value           输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由需要裁判数据的任务（控制读取接口预留）在任务上下文调用；使用 ref_to_chas_mutex，获取锁超时为 0。
 */
bool app_rtos_ref_read(app_ref_snapshot_t *value);

/* Service-owned resource capacities and per-cycle budgets. */
#define APP_RTOS_REF_FIFO_SIZE 1024U
#define APP_RTOS_REF_CHUNK_SIZE 128U
#define APP_RTOS_REF_BATCH_CHUNKS 8U
#ifndef APP_CAN_REF_HEAT_BARREL_MM
#define APP_CAN_REF_HEAT_BARREL_MM 0U /* Select 17 or 42 after the source is confirmed. */
#endif
#if APP_CAN_REF_HEAT_BARREL_MM != 0 && APP_CAN_REF_HEAT_BARREL_MM != 17 &&                         \
    APP_CAN_REF_HEAT_BARREL_MM != 42
#error "APP_CAN_REF_HEAT_BARREL_MM must be 0, 17 or 42"
#endif
#endif
