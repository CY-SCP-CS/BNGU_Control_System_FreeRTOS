/**
 * @file    app_task_log_port.h
 * @brief   日志串口发送契约，返回缓冲能否安全复用
 */
#ifndef APP_TASK_LOG_PORT_H
#define APP_TASK_LOG_PORT_H
#include <stdint.h>

typedef enum
{
    APP_LOG_TX_OK,         /**< 本机发送完成，缓冲可复用。 */
    APP_LOG_TX_FAILED,     /**< 发送失败但已确认终止，缓冲可复用。 */
    APP_LOG_TX_BUFFER_BUSY /**< 无法确认终止，调用者必须保留缓冲。 */
} app_log_tx_result_t;

/** @brief 板级装配注册 UART1 事务回调；UART1 由日志任务独占。 */
void app_task_log_port_init(void);
/** @brief 等待完成/错误；超时按报文长度及波特率计算，返回前尝试确认中止。 */
app_log_tx_result_t app_rtos_uart_send(uint8_t *bytes, uint16_t len);
#endif
