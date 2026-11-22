/**
 * @file    app_task_monitor.c
 * @brief   生产者发布通用有效数据状态，健康任务只读取本表
 */

#include "app_task_monitor.h"
#include "app_vehicle_config.h"
#include "app_rtos_time.h"
#include <string.h>

const app_rtos_module_desc_t app_rtos_module_table[APP_HEALTH_MODULE_COUNT] = {
#define APP_MODULE(id, number, label, timeout, board)                                              \
    [id] = {label, timeout, ((board) & APP_BOARD_CURRENT_MASK) != 0},
#include "app_rtos_module_list.def"
#undef APP_MODULE
};
static app_rtos_module_status_t s_module_status[APP_HEALTH_MODULE_COUNT];

void app_rtos_monitor_init(void)
{
    memset(s_module_status, 0, sizeof(s_module_status));
}

/** @brief 更新有效性；非法输入可以置 invalid，但不能伪造有效数据的新时间。 */
void app_rtos_module_update(unsigned module, bool valid, uint32_t tick)
{
    if (module >= APP_HEALTH_MODULE_COUNT)
    {
        return;
    }
    uint32_t key = app_rtos_irq_lock();
    s_module_status[module].valid = valid;
    if (valid)
    {
        s_module_status[module].tick = tick;
    }
    app_rtos_irq_unlock(key);
}

/** @brief 发布累计硬件错误/忙计数，计算和外设访问在调用前完成。 */
void app_rtos_module_diagnostics(unsigned module, uint32_t errors, uint32_t overruns)
{
    if (module >= APP_HEALTH_MODULE_COUNT)
    {
        return;
    }
    uint32_t key = app_rtos_irq_lock();
    s_module_status[module].errors = errors;
    s_module_status[module].overruns = overruns;
    app_rtos_irq_unlock(key);
}

/** @brief 一次只复制一个小状态，避免健康任务长时间关闭中断复制大快照。 */
bool app_rtos_module_read(unsigned module, app_rtos_module_status_t *value)
{
    if (module >= APP_HEALTH_MODULE_COUNT || !value)
    {
        return false;
    }
    uint32_t key = app_rtos_irq_lock();
    *value = s_module_status[module];
    app_rtos_irq_unlock(key);
    return true;
}
