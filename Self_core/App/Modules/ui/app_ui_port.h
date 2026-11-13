/** @file app_ui_port.h
 * @brief USART6 UI transmitter; static buffer retained until completion or confirmed abort.
 */
#ifndef APP_UI_PORT_H
#define APP_UI_PORT_H
#include <stdint.h>
/** @brief 注册 USART6 发送完成回调并清空发送状态。
 * @return 无返回值。注册接口无错误返回；端口启动失败由 send 报告。
 * @note 裁判任务在 PORT_READY 后调用一次；不替换接收回调。 */
void app_ui_port_init(void);
/** @brief 复制完整帧并启动 USART6 中断发送。
 * @param[in] frame 非 NULL 完整帧，仅本次调用借用，返回前复制。
 * @param[in] length 长度，B，范围 1..REFEREE_TX_BUF_SIZE。
 * @return 0=已启动，1=上一帧仍在发送，-1=参数或 HAL 错误。
 * @note 仅裁判任务调用，非阻塞；静态缓冲在完成或确认终止前不复用。 */
int app_ui_port_send(const void *frame, uint16_t length);
/** @brief 检查 100 ms 发送超时并尝试终止发送。
 * @return 无返回值。错误累加 COUNT_UART_TIMEOUT；终止失败保留忙状态。
 * @note 裁判任务调用；仅终止 TX，不影响循环 DMA 接收。 */
void app_ui_port_poll(void);
#endif
