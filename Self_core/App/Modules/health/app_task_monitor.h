/**
 * @file    app_task_monitor.h
 * @brief   统一数据源状态；设备在线与任务心跳分别维护
 */
#ifndef APP_TASK_MONITOR_H
#define APP_TASK_MONITOR_H
#include <stdbool.h>
#include <stdint.h>

enum
{
#define APP_MODULE(id, number, name, threshold, board) id = number,
#include "app_rtos_module_list.def"
#undef APP_MODULE
    APP_HEALTH_MODULE_COUNT = 0
#define APP_MODULE(...) +1
#include "app_rtos_module_list.def"
#undef APP_MODULE
};

typedef struct
{
    const char *name;
    uint32_t threshold;
    uint8_t enabled;
} app_rtos_module_desc_t;

typedef struct
{
    uint32_t tick;             /**< 最近有效数据时间，不是任务唤醒时间。 */
    uint32_t errors, overruns; /**< 生产者发布的累计错误/忙或丢样次数；未提供时为 0。 */
    uint8_t valid;             /**< 最近发布的数据是否有效，false 不推进 tick。 */
} app_rtos_module_status_t;

extern const app_rtos_module_desc_t app_rtos_module_table[APP_HEALTH_MODULE_COUNT];
void app_rtos_monitor_init(void); /**< 接收启动前清零状态。 */
void app_rtos_module_update(unsigned module, bool valid,
                            uint32_t tick); /**< 任务/合法 ISR 短临界区发布。 */
void app_rtos_module_diagnostics(unsigned module, uint32_t errors, uint32_t overruns);
/** @brief 一次只复制一个小状态，避免健康任务长时间关闭中断复制大快照。  * @return true=完成；false=参数或资源当前不可用。
 */
bool app_rtos_module_read(unsigned module,
                          app_rtos_module_status_t *value); /**< 短复制，健康任务不读设备大快照。 */

#endif
