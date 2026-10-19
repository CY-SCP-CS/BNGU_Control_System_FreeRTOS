/**
 * @file    app_rtos_time.h
 * @brief   测时与短临界区移植契约；实现位于 STM32 port，头文件不暴露 HAL
 */
#ifndef APP_RTOS_TIME_H
#define APP_RTOS_TIME_H
#include <stdint.h>

uint32_t app_rtos_cycles(void);                     /**< 当前硬件周期计数，计数器可自然回绕。 */
uint32_t app_rtos_elapsed_us(uint32_t start);       /**< 无符号差换算 us，包含抢占和阻塞。 */
void app_rtos_kernel_tick(void);                    /**< tick hook 更新硬件测时锚点。 */
uint32_t app_rtos_wakeup_delay_us(uint32_t target); /**< 目标 tick 到实际执行之间的延迟。 */
float app_rtos_sample_dt(uint32_t cycles, uint32_t previous); /**< 相邻采样触发间隔，单位 s。 */
uint32_t app_rtos_irq_lock(void);       /**< 保存屏蔽状态并进入短临界区，不在其中等待。 */
void app_rtos_irq_unlock(uint32_t key); /**< 恢复原屏蔽状态，不无条件打开中断。 */
void app_rtos_memory_barrier(void);     /**< FIFO 字节/索引发布顺序屏障。 */

/** @brief 初始化 CPU 测时硬件，不配置业务外设或任务回调。 */
void app_rtos_port_init(void);

#endif
