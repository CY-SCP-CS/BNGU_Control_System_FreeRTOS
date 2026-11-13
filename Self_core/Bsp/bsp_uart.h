/**
 * @file    bsp_uart.h
 * @brief   UART 驱动：DMA 接收、非阻塞发送与中断回调分发
 */
#ifndef BSP_UART_H
#define BSP_UART_H

#include "bsp_cfg.h"

#define BSP_UART_TX_CALLBACK_MAX 3U
#define BSP_UART_RX_IDLE_CALLBACK_MAX 3U

/** @brief UART 发送完成回调类型。 */
typedef void (*bsp_uart_tx_cplt_callback_t)(UART_HandleTypeDef *huart);

/** @brief UART 空闲中断回调类型。 */
typedef void (*bsp_uart_rx_idle_callback_t)(UART_HandleTypeDef *huart);

/**
 * @brief  以中断方式发送数据，不阻塞调用者。
 * @param  huart UART 句柄。
 * @param  data  发送缓冲区。
 * @param  len   发送字节数。
 * @return HAL 库执行状态。
 * @note   HAL_OK 仅表示已启动；缓冲必须保持有效且不改写，直到发送完成或确认中止。
 */
HAL_StatusTypeDef bsp_uart_tx(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len);

/**
 * @brief  注册指定 UART 的发送完成回调。
 * @param  huart    UART 句柄。
 * @param  callback 发送完成后调用的函数。
 */
void bsp_uart_reg_tx_cplt_callback(UART_HandleTypeDef *huart, bsp_uart_tx_cplt_callback_t callback);

/**
 * @brief  启动 DMA 接收。
 * @param  huart UART 句柄。
 * @param  data  接收缓冲区。
 * @param  len   接收缓冲区长度，单位字节。
 * @return HAL 库执行状态。
 * @note   DMA 模式沿用外设配置，接收缓冲须有静态生命周期且位于 DMA 可访问的 SRAM。
 */
HAL_StatusTypeDef bsp_uart_rx_dma(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len);

/**
 * @brief  停止 UART 的 DMA 接收。
 * @param  huart UART 句柄。
 * @return HAL 库执行状态。
 */
HAL_StatusTypeDef bsp_uart_stop_dma(UART_HandleTypeDef *huart);

/**
 * @brief  获取 DMA 尚未写入的接收缓冲区字节数。
 * @param  huart UART 句柄。
 * @return DMA 剩余传输字节数；句柄或 DMA 无效时返回 0。
 */
uint16_t bsp_uart_get_rx_dma_remain(const UART_HandleTypeDef *huart);

/**
 * @brief  判断 UART 是否产生空闲标志。
 * @param  huart UART 句柄。
 * @return 1 表示空闲标志有效，0 表示未产生或句柄无效。
 */
uint8_t bsp_uart_is_idle(const UART_HandleTypeDef *huart);

/**
 * @brief 清除 UART 空闲中断标志。
 * @param huart UART 句柄。
 */
void bsp_uart_clear_idle(UART_HandleTypeDef *huart);

/**
 * @brief 使能 UART 的空闲中断。
 * @param huart UART 句柄。
 */
void bsp_uart_enable_idle_it(UART_HandleTypeDef *huart);

/**
 * @brief 处理 USARTx_IRQHandler。
 * @note  该函数先分发空闲中断；其余中断继续交由 HAL_UART_IRQHandler 处理。
 * @param  huart UART 句柄。
 */
void bsp_uart_irq_handler(UART_HandleTypeDef *huart);

/**
 * @brief  注册指定 UART 的空闲中断回调。
 * @param  huart    UART 句柄。
 * @param  callback 空闲中断发生后调用的函数。
 */
void bsp_uart_reg_rx_idle_callback(UART_HandleTypeDef *huart, bsp_uart_rx_idle_callback_t callback);

/** @brief 接收事件编号，属于枚举值，不能按标志位组合。 */
enum
{
    BSP_UART_RX_HALF = 1, /**< DMA 半缓冲接收完成 HT。 */
    BSP_UART_RX_FULL = 2, /**< DMA 全缓冲接收完成 TC；循环模式随后继续覆盖缓冲。 */
    BSP_UART_RX_IDLE = 3, /**< UART 检测到一段线路空闲，不保证等于一帧协议。 */
    BSP_UART_ERROR = 4    /**< HAL 报告 UART 接收/传输错误。 */
};

/**
 * @brief  UART 统一事件回调类型。
 * @note   参数依次为 UART 句柄和一个 BSP_UART_* 事件编号；在 UART 或 DMA ISR 中调用。
 * @note   回调可读取 NDTR 并复制数据，但不要阻塞、取互斥量或执行协议业务。
 */
typedef void (*bsp_uart_rx_event_callback_t)(UART_HandleTypeDef *, uint32_t event);
/**
 * @brief  分发 UART IDLE 事件，并清除硬件空闲标志。
 * @param  huart        发生 UART 中断的句柄，NULL 时忽略。
 * @note   由 USARTx_IRQHandler 在 HAL_UART_IRQHandler() 前调用；本函数不代替 HAL 处理其他中断。
 * @note   先调用兼容 IDLE 回调，再清除 IDLE，最后通知统一事件回调；未发现 IDLE 时不交付事件。
 */
void bsp_uart_rx_irq_dispatch(UART_HandleTypeDef *);
/**
 * @brief  为指定 UART 注册 HT/TC/IDLE/ERROR 统一回调。
 * @param  uart         目标 UART 句柄，必须有效。
 * @param  callback     同步事件回调，NULL 表示该槽不交付事件。
 * @note   最多保存三个 UART；同一 UART 再注册覆盖旧回调，表满时静默忽略。
 * @note   须在接收启动前注册；当前分别供 UART1 日志、USART3 DBUS 和 USART6 裁判使用。
 */
void bsp_uart_reg_rx_event_callback(UART_HandleTypeDef *, bsp_uart_rx_event_callback_t);
/**
 * @brief 独占注册某 UART 的接收事件；相同回调可重复注册。
 * @param[in] uart 已初始化 UART，非 NULL。
 * @param[in] callback 非 NULL 的同步 ISR 回调，接收期间不变。
 * @return HAL_OK=注册成功；HAL_BUSY=已有不同接收者；HAL_ERROR=参数无效或表满。
 * @note 仅启动前调用；DMA 缓冲和恢复由接收者维护，不分配内存。
 */
HAL_StatusTypeDef bsp_uart_claim_rx(UART_HandleTypeDef *uart,
                                    bsp_uart_rx_event_callback_t callback);
/**
 * @brief 停止接收后释放该回调的注册槽。
 * @param[in] uart 目标 UART。
 * @param[in] callback 当前拥有者的回调；不匹配时保留原注册。
 * @return 无返回值。未匹配时不修改。
 * @note 仅任务上下文，调用前停用 DMA 并与 ISR 串行化。
 */
void bsp_uart_release_rx(UART_HandleTypeDef *uart, bsp_uart_rx_event_callback_t callback);
#endif
