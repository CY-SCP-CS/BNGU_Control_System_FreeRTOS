/**
 * @file    app_task_log_port.c
 * @brief   UART1 硬件发送事务；故障后如何运行由日志任务决定
 */
#include "app_task_log_port.h"
#include "app_rtos.h"
#include "bsp_uart.h"

/** @brief 完成中断只通知，不改写发送缓冲。 */
static void uart_done(UART_HandleTypeDef *uart)
{
    (void)uart;
    app_rtos_notify_task(TASK_LOG, APP_RTOS_TF_TX_DONE);
}

/** @brief 错误中断只累计诊断并通知，终止事务在任务上下文完成。 */
static void uart_error(UART_HandleTypeDef *uart, uint32_t event)
{
    (void)uart;
    if (event == BSP_UART_ERROR)
    {
        app_rtos_count(COUNT_UART_ERROR);
        app_rtos_notify_task(TASK_LOG, APP_RTOS_TF_IO_ERROR);
    }
}

void app_task_log_port_init(void)
{
    bsp_uart_reg_tx_cplt_callback(&huart1, uart_done);
    bsp_uart_reg_rx_event_callback(&huart1, uart_error);
}

app_log_tx_result_t app_rtos_uart_send(uint8_t *bytes, uint16_t len)
{
    if (!bytes || !len || !huart1.Init.BaudRate)
    {
        app_rtos_count(COUNT_UART_ERROR);
        return APP_LOG_TX_FAILED;
    }
    uint32_t tick_freq = osKernelGetTickFreq();
    /* 每字节按最多 12 位保守估算；当前 UART1 为 115200、8N1（实际 10 位）。 */
    uint32_t wire_ticks = (uint32_t)(((uint64_t)len * 12U * tick_freq + huart1.Init.BaudRate - 1U) /
                                     huart1.Init.BaudRate);
    uint32_t margin_ticks = (tick_freq + 99U) / 100U; //10 ms 向上取整
    uint32_t timeout = wire_ticks + (margin_ticks ? margin_ticks : 1U);
    /* 清除上一笔事务的完成/错误标志，防止把旧通知误认为本次发送完成。 */
    (void)osThreadFlagsClear(APP_RTOS_TF_TX_DONE | APP_RTOS_TF_IO_ERROR);
    if (bsp_uart_tx(&huart1, bytes, len) != HAL_OK)
    {
        app_rtos_count(COUNT_UART_ERROR);
        return APP_LOG_TX_FAILED;
    }
    uint32_t result =
        osThreadFlagsWait(APP_RTOS_TF_TX_DONE | APP_RTOS_TF_IO_ERROR, osFlagsWaitAny, timeout);
    if (!(result & osFlagsError) && (result & APP_RTOS_TF_TX_DONE) &&
        !(result & APP_RTOS_TF_IO_ERROR))
    {
        return APP_LOG_TX_OK;
    }
    if (result == osFlagsErrorTimeout)
    {
        app_rtos_count(COUNT_UART_TIMEOUT);
    }
    /* 发送错误或超时后确认中止，确保 UART IT 不再引用缓冲才允许复用。 */
    if (HAL_UART_AbortTransmit(&huart1) != HAL_OK)
    {
        return APP_LOG_TX_BUFFER_BUSY;
    }
    return APP_LOG_TX_FAILED;
}
