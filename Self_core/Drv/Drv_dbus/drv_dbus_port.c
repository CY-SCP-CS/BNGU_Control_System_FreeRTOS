/**
 * @file    drv_dbus_port.c
 * @brief   DBUS 硬件接收适配：USART3 普通 DMA 双缓冲与完整帧交付。
 * @note    ISR 只收集完整 18 字节帧；解包、范围检查和快照发布由 comm_dbus_Task 完成。
 */
#include "drv_dbus.h"
#include "bsp_uart.h"
#include <string.h>

/* 两块静态普通 SRAM 缓冲：一块由 DMA 写入，另一块在回调期间只读交付；不是动态内存池。 */
static uint8_t s_rx[2][18];
/* 当前准备接收的缓冲索引，收帧后异或 1 切换到另一块。 */
static uint8_t s_index;
/* ISR 出错时清零；任务看到未运行状态后执行恢复。 */
static volatile uint8_t s_started;
static drv_dbus_frame_callback_t s_deliver;

/**
 * @brief  绑定 DBUS 完整帧与接收错误的上交入口。
 * @param  callback     完整帧交付函数，NULL 表示不交付。
 * @note   在启动接收前由 RTOS 端口层绑定；当前回调复制到 dbus_rx_queue 并设置任务标志。
 */
void drv_dbus_port_bind(drv_dbus_frame_callback_t callback)
{
    s_deliver = callback;
}

/**
 * @brief  使用当前活动缓冲启动下一轮固定 18 字节 DMA。
 * @return 1=HAL 启动成功，0=启动失败；同时更新 s_started。
 * @note   由初始化、接收事件或任务恢复路径调用；调用者负责与 ISR 串行访问接收状态。
 */
static uint8_t restart(void)
{
    s_started = bsp_uart_rx_dma(&huart3, s_rx[s_index], 18) == HAL_OK;
    return s_started;
}

/**
 * @brief  在 USART3 接收事件中筛选完整帧、切换缓冲并重启接收。
 * @param  uart         USART3 句柄。
 * @param  event        BSP_UART_RX_HALF/FULL/IDLE/ERROR 事件。
 * @note   半满事件忽略；完整帧才上交，短帧停止后丢弃并开始下一轮，不跨 IDLE 拼接短帧。
 * @note   TC 后紧接着产生且字节数为 0 的 IDLE 会忽略，避免重复交付。
 * @note   先用另一块缓冲重启 DMA，再同步回调交付旧缓冲；回调返回后不得继续引用旧地址。
 */
static void rx_event(UART_HandleTypeDef *uart, uint32_t event)
{
    if (event == BSP_UART_RX_HALF)
    {
        return;
    }
    if (event == BSP_UART_ERROR)
    {
        s_started = 0;
        if (s_deliver)
        {
            s_deliver(NULL);
        }
        return;
    }
    /* NDTR 为剩余未接收字节数，因此 18-NDTR 为本轮已收到的字节数。 */
    uint16_t count = 18 - bsp_uart_get_rx_dma_remain(uart);
    if (event == BSP_UART_RX_IDLE && !count)
    {
        return; /* TC 后的空 IDLE 不重复交付。 */
    }
    (void)bsp_uart_stop_dma(uart);
    uint8_t index = s_index;
    s_index ^= 1;
    (void)restart();
    if (count == 18 && s_deliver)
    {
        s_deliver(s_rx[index]);
    }
}

/**
 * @brief  复位接收双缓冲，注册 USART3 事件并启动第一轮 18 字节 DMA。
 * @return 1=DMA 启动成功，0=启动失败。
 * @note   初始化任务调用；USART3 的波特率、DMA 和中断配置由 CubeMX 外设初始化先完成。
 */
uint8_t drv_dbus_port_init(void)
{
    s_index = 0;
    memset(s_rx, 0, sizeof(s_rx));
    bsp_uart_reg_rx_event_callback(&huart3, rx_event);
    bsp_uart_clear_idle(&huart3);
    bsp_uart_enable_idle_it(&huart3);
    return restart();
}

/**
 * @brief  在接收停止后尝试重新启动 USART3 DMA。
 * @return 1=接收已运行或重启成功，0=本次重启失败。
 * @note   由 DBUS 任务定期调用；短临界区防止与接收中断同时改变 DMA 状态，本函数不解包。
 */
uint8_t drv_dbus_port_poll(void)
{
    if (s_started)
    {
        return 1;
    }
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    (void)bsp_uart_stop_dma(&huart3);
    uint8_t ok = restart();
    __set_PRIMASK(key);
    return ok;
}
