/**
 * @file    app_rtos_kernel.c
 * @brief   FreeRTOS 专有接口集中适配；上层不直接调用 FreeRTOS 专用堆接口
 */
#include "app_rtos_kernel.h"
#include "app_rtos_time.h"
#include "FreeRTOS.h"
#include "task.h"
_Static_assert(configTICK_RATE_HZ == APP_RTOS_KERNEL_TICK_HZ, "Kernel tick must be 1 kHz");
volatile uint32_t app_rtos_fatal_code;

/**
 * @brief  读取 FreeRTOS 堆的当前及历史余量
 * @param  value 输出记录；为 NULL 时不操作。
 * @note   供任务诊断使用；三项查询不保证处于同一分配瞬间，不统计静态 SRAM/MSP。
 */
void app_rtos_memory_read(app_rtos_memory_t *value)
{
    if (value)
    {
        value->total = (uint32_t)configTOTAL_HEAP_SIZE;
        value->free = (uint32_t)xPortGetFreeHeapSize();
        value->minimum_free = (uint32_t)xPortGetMinimumEverFreeHeapSize();
    }
}

/**
 * @brief  处理内核检测到的任务栈溢出
 * @param  task         发生栈溢出的原生 FreeRTOS 任务句柄
 * @param  name         任务名，当前只保留参数不输出
 * @note   记录故障码 2，关闭中断后停止；此路径不依赖可能已损坏的日志或堆。
 */
void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;
    app_rtos_fatal_code = 2;
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}

/**
 * @brief  处理 FreeRTOS 动态内存分配失败
 * @note   记录故障码 1，关闭中断后停止；通过 configUSE_MALLOC_FAILED_HOOK 启用。
 */
void vApplicationMallocFailedHook(void)
{
    app_rtos_fatal_code = 1;
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}

/**
 * @brief  在内核 tick 中断中更新微秒测时锚点
 * @note   中断上下文，只记录 DWT 时间，不执行控制计算或阻塞操作。
 */
void vApplicationTickHook(void)
{
    app_rtos_kernel_tick();
}
