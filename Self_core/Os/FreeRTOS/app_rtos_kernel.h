/**
 * @file    app_rtos_kernel.h
 * @brief   内核内存统计与故障状态；不引用具体应用任务
 */
#ifndef APP_RTOS_KERNEL_H
#define APP_RTOS_KERNEL_H
#include <stdint.h>
/* 当前工程内核为 1 kHz；实现中静态检查 FreeRTOSConfig.h，避免配置分离。 */
#define APP_RTOS_KERNEL_TICK_HZ 1000U

typedef struct
{
    uint32_t total;
    uint32_t free;
    uint32_t minimum_free;
} app_rtos_memory_t;

/** @brief 任务中读取 FreeRTOS 管理的堆；不包含链接器静态 SRAM 或 MSP 栈。 */
void app_rtos_memory_read(app_rtos_memory_t *value);
extern volatile uint32_t app_rtos_fatal_code;
#endif
