/**
 * @file    bsp_uart.c
 * @brief   UART 硬件收发与事件分发：IT 发送、DMA 接收、HT/TC/IDLE/ERROR 回调。
 * @note    BSP 只报告字节接收事件；完整帧收集、循环 DMA 位置计算和协议解包在驱动端口或任务完成。
 */
#include "bsp_uart.h"

typedef struct
{
    UART_HandleTypeDef *huart;
    bsp_uart_tx_cplt_callback_t callback;
} bsp_uart_tx_callback_entry_t;

typedef struct
{
    UART_HandleTypeDef *huart;
    bsp_uart_rx_idle_callback_t callback;
} bsp_uart_rx_idle_callback_entry_t;

static bsp_uart_tx_callback_entry_t s_tx_callbacks[BSP_UART_TX_CALLBACK_MAX];

/* 三个 UART 的统一事件槽；初始化时注册，ISR 只读取并同步分发。 */
static struct
{
    UART_HandleTypeDef *uart;
    bsp_uart_rx_event_callback_t callback;
} s_events[3];

/**
 * @brief  按 UART 句柄查找唯一事件入口并同步调用。
 * @param  uart         事件来源 UART。
 * @param  event        单个 BSP_UART_* 事件编号。
 * @note   中断上下文执行；未注册或回调为空时忽略，不缓存事件。
 */
static void notify_rx(UART_HandleTypeDef *uart, uint32_t event)
{
    for (unsigned i = 0; i < 3; i++)
    {
        if (s_events[i].uart == uart && s_events[i].callback)
        {
            s_events[i].callback(uart, event);
            return;
        }
    }
}

/**
 * @brief  为指定 UART 注册 HT/TC/IDLE/ERROR 统一回调。
 * @param  uart         目标 UART 句柄，必须有效。
 * @param  callback     同步事件回调，NULL 表示该槽不交付事件。
 * @note   最多保存三个 UART；同一 UART 再注册覆盖旧回调，表满时静默忽略。
 * @note   须在接收启动前注册；当前分别供 UART1 日志、USART3 DBUS 和 USART6 裁判使用。
 */
void bsp_uart_reg_rx_event_callback(UART_HandleTypeDef *uart, bsp_uart_rx_event_callback_t callback)
{
    for (unsigned i = 0; i < 3; i++)
    {
        if (!s_events[i].uart || s_events[i].uart == uart)
        {
            s_events[i].uart = uart;
            s_events[i].callback = callback;
            return;
        }
    }
}

/**
 * @brief  把 HAL DMA 接收半满转换为统一事件 BSP_UART_RX_HALF。
 * @param  uart         事件来源 UART 句柄。
 * @note   UART/DMA ISR 上下文；是否拼帧、重启接收或唤醒任务由已绑定的端口回调决定。
 */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *uart)
{
    notify_rx(uart, BSP_UART_RX_HALF);
}

/**
 * @brief  把 HAL DMA 接收全满转换为统一事件 BSP_UART_RX_FULL。
 * @param  uart         事件来源 UART 句柄。
 * @note   UART/DMA ISR 上下文；是否拼帧、重启接收或唤醒任务由已绑定的端口回调决定。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    notify_rx(uart, BSP_UART_RX_FULL);
}

/**
 * @brief  把 HAL UART 错误转换为统一事件 BSP_UART_ERROR。
 * @param  uart         事件来源 UART 句柄。
 * @note   UART/DMA ISR 上下文；是否拼帧、重启接收或唤醒任务由已绑定的端口回调决定。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    notify_rx(uart, BSP_UART_ERROR);
}

static uint8_t s_tx_callback_count;
static bsp_uart_rx_idle_callback_entry_t s_rx_idle_callbacks[BSP_UART_RX_IDLE_CALLBACK_MAX];
static uint8_t s_rx_idle_callback_count;

/**
 * @brief  启动 UART 中断发送，返回后由硬件中断继续取用原缓冲。
 * @param  huart        目标 UART。
 * @param  data         发送缓冲，发送完成或确认中止前保持有效且不改写。
 * @param  len          发送字节数，必须大于 0。
 * @return HAL_OK=已启动，其余为无效参数、忙或 HAL 错误；并不表示发送完成。
 * @note   当前 UART1 由日志任务独占；完成事件经注册回调通知任务后才能复用缓冲。
 */
HAL_StatusTypeDef bsp_uart_tx(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len)
{
    if (!huart || !data || len == 0U)
    {
        return HAL_ERROR;
    }
    return HAL_UART_Transmit_IT(huart, data, len);
}

/**
 * @brief  登记指定 UART 的发送完成回调。
 * @param  huart        目标 UART 句柄。
 * @param  callback     中断中同步调用的处理函数，NULL 时忽略注册。
 * @note   同 UART 覆盖旧函数；表满时静默忽略，接收/发送启动前完成注册。
 */
void bsp_uart_reg_tx_cplt_callback(UART_HandleTypeDef *huart, bsp_uart_tx_cplt_callback_t callback)
{
    uint8_t i;
    if (!huart || !callback)
    {
        return;
    }
    for (i = 0U; i < s_tx_callback_count; i++)
    {
        if (s_tx_callbacks[i].huart == huart)
        {
            s_tx_callbacks[i].callback = callback;
            return;
        }
    }
    if (s_tx_callback_count < BSP_UART_TX_CALLBACK_MAX)
    {
        s_tx_callbacks[s_tx_callback_count].huart = huart;
        s_tx_callbacks[s_tx_callback_count].callback = callback;
        s_tx_callback_count++;
    }
}

/**
 * @brief  登记指定 UART 的兼容 IDLE回调。
 * @param  huart        目标 UART 句柄。
 * @param  callback     中断中同步调用的处理函数，NULL 时忽略注册。
 * @note   同 UART 覆盖旧函数；表满时静默忽略，接收/发送启动前完成注册。
 */
void bsp_uart_reg_rx_idle_callback(UART_HandleTypeDef *huart, bsp_uart_rx_idle_callback_t callback)
{
    uint8_t i;
    if (!huart || !callback)
    {
        return;
    }
    for (i = 0U; i < s_rx_idle_callback_count; i++)
    {
        if (s_rx_idle_callbacks[i].huart == huart)
        {
            s_rx_idle_callbacks[i].callback = callback;
            return;
        }
    }
    if (s_rx_idle_callback_count < BSP_UART_RX_IDLE_CALLBACK_MAX)
    {
        s_rx_idle_callbacks[s_rx_idle_callback_count].huart = huart;
        s_rx_idle_callbacks[s_rx_idle_callback_count].callback = callback;
        s_rx_idle_callback_count++;
    }
}

/**
 * @brief  启动 DMA，把串口字节直接写入指定缓冲。
 * @param  huart        目标 UART 句柄。
 * @param  data         DMA 可访问的静态 SRAM 缓冲。
 * @param  len          DMA 接收长度，单位字节，必须大于 0。
 * @return HAL 状态；HAL_OK 表示已启动接收。
 * @note   普通/循环模式由外设 DMA 配置决定：DBUS 为普通 18 字节，裁判为循环 512 字节。
 * @note   缓冲须在接收停止前一直有效；不能使用会退出作用域的栈数组。
 */
HAL_StatusTypeDef bsp_uart_rx_dma(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len)
{
    if (!huart || !data || len == 0U)
    {
        return HAL_ERROR;
    }
    return HAL_UART_Receive_DMA(huart, data, len);
}

/**
 * @brief  调用 HAL 停止该 UART 的 DMA 事务。
 * @param  huart        目标 UART，NULL 返回 HAL_ERROR。
 * @return HAL_UART_DMAStop() 的状态。
 * @note   使用的是 HAL 的 DMA 停止接口，可能影响该 UART 的收/发 DMA；本工程用于端口接收重启。
 */
HAL_StatusTypeDef bsp_uart_stop_dma(UART_HandleTypeDef *huart)
{
    return huart ? HAL_UART_DMAStop(huart) : HAL_ERROR;
}

/**
 * @brief  读取 RX DMA NDTR 剩余计数。
 * @param  huart        带 RX DMA 的 UART 句柄。
 * @return 尚未传输的字节数；句柄或 RX DMA 缺失时返回 0。
 * @note   当前写入位置通常由缓冲长度减 NDTR 得到；循环 DMA 到尾部后硬件自动重装。
 */
uint16_t bsp_uart_get_rx_dma_remain(const UART_HandleTypeDef *huart)
{
    if (!huart || !huart->hdmarx)
    {
        return 0U;
    }
    return (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);
}

/**
 * @brief  检查 UART IDLE 硬件标志是否置位。
 * @param  huart        待检查 UART。
 * @return 1=IDLE 已置位，0=未置位或句柄无效。
 * @note   只检查标志，不清除、不交付字节，也不判定协议帧是否完整。
 */
uint8_t bsp_uart_is_idle(const UART_HandleTypeDef *huart)
{
    return huart && __HAL_UART_GET_FLAG((UART_HandleTypeDef *)huart, UART_FLAG_IDLE) != RESET;
}

/**
 * @brief  按 STM32 HAL 规定清除 UART IDLE 标志。
 * @param  huart        目标 UART，NULL 时忽略。
 */
void bsp_uart_clear_idle(UART_HandleTypeDef *huart)
{
    if (huart)
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
    }
}

/**
 * @brief  使能 UART IDLE 中断，为不满 DMA 缓冲的片段提供交付机会。
 * @param  huart        目标 UART，NULL 时忽略。
 */
void bsp_uart_enable_idle_it(UART_HandleTypeDef *huart)
{
    if (huart)
    {
        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    }
}

/**
 * @brief  分发 UART IDLE 事件，并清除硬件空闲标志。
 * @param  huart        发生 UART 中断的句柄，NULL 时忽略。
 * @note   由 USARTx_IRQHandler 在 HAL_UART_IRQHandler() 前调用；本函数不代替 HAL 处理其他中断。
 * @note   先调用兼容 IDLE 回调，再清除 IDLE，最后通知统一事件回调；未发现 IDLE 时不交付事件。
 */
void bsp_uart_rx_irq_dispatch(UART_HandleTypeDef *huart)
{
    uint8_t i;

    if (!huart)
    {
        return;
    }
    if (bsp_uart_is_idle(huart))
    {
        for (i = 0U; i < s_rx_idle_callback_count; i++)
        {
            if (s_rx_idle_callbacks[i].huart == huart && s_rx_idle_callbacks[i].callback)
            {
                s_rx_idle_callbacks[i].callback(huart);
                break;
            }
        }
        bsp_uart_clear_idle(huart);
        notify_rx(huart, BSP_UART_RX_IDLE);
    }
}

/**
 * @brief  执行 BSP IDLE 分发，然后继续 HAL 的 UART 中断处理。
 * @param  huart        目标 UART 句柄，调用者须提供有效句柄。
 * @note   也可以在 Core 中分别调用 dispatch 和 HAL；两种接法只选一组，避免重复执行 HAL 处理。
 */
void bsp_uart_irq_handler(UART_HandleTypeDef *huart)
{
    bsp_uart_rx_irq_dispatch(huart);
    HAL_UART_IRQHandler(huart);
}

/**
 * @brief  查找 UART 对应的发送完成回调并通知持有发送缓冲的上层。
 * @param  huart        已完成中断发送的 UART。
 * @note   当前 UART1 转为日志任务 TX_DONE 标志；仅表示本机发送完成，不表示上位机确认接收。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t i;
    for (i = 0U; i < s_tx_callback_count; i++)
    {
        if (s_tx_callbacks[i].huart == huart && s_tx_callbacks[i].callback)
        {
            s_tx_callbacks[i].callback(huart);
            return;
        }
    }
}
