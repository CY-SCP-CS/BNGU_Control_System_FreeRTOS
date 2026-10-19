/**
 * @file    drv_referee_port.c
 * @brief   裁判串口接收适配：USART6 循环 DMA 分段交付，任务负责协议解析。
 * @note    512 字节 DMA 缓冲由硬件循环写入；HT/TC/IDLE 中断只把新增字节复制到 RTOS 字节 FIFO。
 */
#include "drv_referee.h"
#include "bsp_uart.h"

/**
 * @brief  裁判串口端口的私有接收状态。
 * @note   静态存储在普通 SRAM；s_dma 供 DMA 持续写入，不能放进 STM32F407 CCM。
 * @note   保留原成员名；ISR 交付新增字节，任务恢复路径在短临界区重启接收。
 */
typedef struct
{
    uint8_t s_dma[512];                     /**< 循环 DMA 接收缓冲，接收期间始终有效。 */
    uint16_t s_position;                    /**< 已交付到的位置 [0,511]，不属于 RTOS FIFO 索引。 */
    volatile uint8_t s_started;             /**< 接收运行标记，ISR 出错清零，任务尝试恢复。 */
    drv_referee_bytes_callback_t s_deliver; /**< 接收片段/错误的同步交付入口。 */
} drv_referee_port_context_t;

static drv_referee_port_context_t s_referee_port;

/**
 * @brief  绑定裁判字节流交付入口。
 * @param  callback     新增字节交付函数，当前绑定 app_rtos_ref_push_isr；NULL 时停止上交。
 * @note   在启动接收前调用；回调立即复制数据，不能保留循环 DMA 缓冲的地址。
 */
void drv_referee_port_bind(drv_referee_bytes_callback_t callback)
{
    s_referee_port.s_deliver = callback;
}

/**
 * @brief  依据当前 DMA 写入位置交付自上次事件以来新增的有序字节。
 * @param  uart         USART6 接收句柄。
 * @param  event        半满、全满、空闲或错误事件。
 * @note   每次读取实时 NDTR，now=(512-NDTR) 映射到 [0,511]；now 等于旧位置时不重复交付。
 * @note   未回绕交付 [s_position,now)；回绕时依次交付尾段和头段，然后保存 now。
 * @note   UART 错误置接收停止并交付 (NULL,0)，由任务负责清理协议半帧及重新启动。
 * @note   位置比较不能识别已被覆盖的完整多圈数据，依赖中断及时处理；本回调不识别协议帧。
 */
static void rx_event(UART_HandleTypeDef *uart, uint32_t event)
{
    if (event == BSP_UART_ERROR)
    {
        s_referee_port.s_started = 0;
        if (s_referee_port.s_deliver)
        {
            s_referee_port.s_deliver(NULL, 0);
        }
        return;
    }

    /* 每个 HT/TC/IDLE 都读取当前 NDTR，用位置差去重，而不按事件种类固定交付半块。 */
    uint16_t now = (uint16_t)(512 - bsp_uart_get_rx_dma_remain(uart));
    /* TC 时 NDTR 可为 0，也可能已自动重装；缓冲末尾统一映射到下标 0。 */
    if (now == 512)
    {
        now = 0;
    }
    if (now == s_referee_port.s_position)
    {
        return;
    }

    /* 回调按字节顺序同步复制；回绕时先尾后头，FIFO 的数据仍连续有序。 */
    if (s_referee_port.s_deliver)
    {
        if (now > s_referee_port.s_position)
        {
            s_referee_port.s_deliver(s_referee_port.s_dma + s_referee_port.s_position,
                                     now - s_referee_port.s_position);
        }
        else
        {
            s_referee_port.s_deliver(s_referee_port.s_dma + s_referee_port.s_position,
                                     512 - s_referee_port.s_position);
            if (now)
            {
                s_referee_port.s_deliver(s_referee_port.s_dma, now);
            }
        }
    }
    s_referee_port.s_position = now;
}

/**
 * @brief  初始化解析状态、注册 USART6 事件并启动 512 字节循环 DMA。
 * @return 1=DMA 启动成功，0=启动失败。
 * @note   初始化任务调用；循环模式由 CubeMX 的 USART6 RX DMA 配置决定，本函数不修改 DMA 模式。
 */
uint8_t drv_referee_port_init(void)
{
    drv_referee_init();
    s_referee_port.s_position = 0;
    bsp_uart_reg_rx_event_callback(&huart6, rx_event);
    bsp_uart_clear_idle(&huart6);
    bsp_uart_enable_idle_it(&huart6);
    s_referee_port.s_started =
        bsp_uart_rx_dma(&huart6, s_referee_port.s_dma, sizeof(s_referee_port.s_dma)) == HAL_OK;
    return s_referee_port.s_started;
}

/**
 * @brief  在 UART 错误导致接收停止后，丢弃解析半帧并重启循环 DMA。
 * @return 1=接收仍运行或重启成功，0=本次重启失败。
 * @note   由裁判任务调用，未发生错误时直接返回；不解析新数据，也不主动读取 DMA 片段。
 * @note   短临界区串行化 DMA 复位与接收 ISR；已解析的业务数据和累计计数保留。
 */
uint8_t drv_referee_port_poll(void)
{
    if (s_referee_port.s_started)
    {
        return 1;
    }
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    (void)bsp_uart_stop_dma(&huart6);
    s_referee_port.s_position = 0;
    drv_referee_reset_stream(); /* 只在任务上下文清理解析半帧，UART ISR 不操作解析器。 */
    s_referee_port.s_started =
        bsp_uart_rx_dma(&huart6, s_referee_port.s_dma, sizeof(s_referee_port.s_dma)) == HAL_OK;
    __set_PRIMASK(key);
    return s_referee_port.s_started;
}
