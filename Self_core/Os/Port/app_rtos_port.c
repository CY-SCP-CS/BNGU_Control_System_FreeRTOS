/**
 * @file    app_rtos_port.c
 * @brief   STM32F407 的 DWT/PRIMASK 适配，仅依赖 CPU 和内核
 * @note    更换 MCU 时适配此实现；应用外设及任务回调连接位于 App/Tasks。
 */
#include "app_rtos_time.h"
#include "cmsis_os2.h"
#include "stm32f4xx.h"

static volatile uint32_t s_kernel_tick_cycles; //tick hook 更新的 DWT 锚点

/**
 * @brief  读取 DWT 32 位 CPU 周期计数
 * @return 当前 CYCCNT 值
 * @note   必须先由 app_rtos_port_init() 启用 DWT；计数随 uint32_t 回绕。
 */
uint32_t app_rtos_cycles(void)
{
    return DWT->CYCCNT;
}

/**
 * @brief  计算自起点以来的经过时间
 * @param  start        起点的 DWT 周期计数
 * @return 经过时间，单位 us
 * @note   使用无符号差处理一次回绕；间隔需小于完整 DWT 计数周期，时间包含抢占和阻塞。
 */
uint32_t app_rtos_elapsed_us(uint32_t start)
{
    return (uint32_t)(DWT->CYCCNT - start) / (SystemCoreClock / 1000000);
}

/**
 * @brief  记录内核 tick 对应的 DWT 时间锚点
 * @note   由 FreeRTOS tick hook 在中断中调用，用于测量任务实际开始执行的延迟。
 */
void app_rtos_kernel_tick(void)
{
    s_kernel_tick_cycles = DWT->CYCCNT;
}

/**
 * @brief  计算周期任务相对目标 tick 的执行延迟
 * @param  target       已到期的绝对内核 tick
 * @return 延迟，单位 us，包含整 tick 延迟及当前 tick 内时间
 * @note   在短临界区读取 tick、DWT 和锚点；target 应是当前周期已经到达的截止时间。
 */
uint32_t app_rtos_wakeup_delay_us(uint32_t target)
{
    /* 保证 tick 与其 DWT 锚点来自同一短采样区，随后恢复原中断状态。 */
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    uint32_t tick = osKernelGetTickCount(), cycles = DWT->CYCCNT, anchor = s_kernel_tick_cycles;
    __set_PRIMASK(key);
    return (uint32_t)(tick - target) * 1000U +
           (uint32_t)(cycles - anchor) / (SystemCoreClock / 1000000U);
}

/**
 * @brief  计算相邻硬件采样触发之间的时间差
 * @param  cycles       本次触发的 DWT 周期计数
 * @param  previous     上次触发的 DWT 周期计数
 * @return 时间间隔，单位 s
 * @note   使用采样触发时间，而非姿态任务处理时间；无符号减法处理一次 DWT 回绕。
 */
float app_rtos_sample_dt(uint32_t cycles, uint32_t previous)
{
    return (float)(uint32_t)(cycles - previous) / (float)SystemCoreClock;
}

/**
 * @brief  保存 PRIMASK 并屏蔽可屏蔽中断
 * @return 进入前的 PRIMASK，供 app_rtos_irq_unlock() 恢复
 * @note   适用于任务和中断中的短临界区；必须成对恢复，临界区内不得等待或阻塞。
 */
uint32_t app_rtos_irq_lock(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    return key;
}

/**
 * @brief  恢复进入临界区前的中断屏蔽状态
 * @param  key          app_rtos_irq_lock() 返回的 PRIMASK
 * @note   恢复原状态，不能简单调用开中断，以免破坏外层临界区。
 */
void app_rtos_irq_unlock(uint32_t key)
{
    __set_PRIMASK(key);
}

/**
 * @brief  执行数据内存屏障
 * @note   用于裁判 FIFO 的字节与读写索引发布顺序，不替代互斥量或临界区。
 */
void app_rtos_memory_barrier(void)
{
    __DMB();
}

/** @brief 启用 CPU 周期计数；板级装配在需要测时前调用。 */
void app_rtos_port_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
