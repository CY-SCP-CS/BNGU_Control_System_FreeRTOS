/** @file app_ui_io.h
 * @brief UI 同步构帧与输出契约；由裁判任务独占，不持有硬件句柄。
 */
#ifndef APP_UI_IO_H
#define APP_UI_IO_H
#include <stdint.h>
/** @brief 接受一帧待发送数据。
 * @param[in] frame 完整帧，非 NULL，仅本次调用期间借用；接受前必须复制。
 * @param[in] length 帧长度，B，大于 0。
 * @return 0=已接受，1=忙，-1=参数或传输错误；接受不代表发送完成。
 * @note 裁判任务上下文；不得阻塞或保存 frame 地址。
 */
typedef int (*app_ui_send_t)(const void *frame, uint16_t length);
/** @brief 绑定 UI 的时钟与输出端口。
 * @param[in] clock_tick 单调 1 ms 计数回调，借用至重新绑定；NULL 时返回时间 0。
 * @param[in] send 输出回调，借用至重新绑定；NULL 时拒绝输出。
 * @return 无返回值。无运行时失败；不启动传输。
 * @note 初始化时调用，禁止与构帧并发；时钟回绕按无符号差值处理。
 */
void app_ui_bind(uint32_t (*clock_tick)(void), app_ui_send_t send);
/** @brief 读取 UI 的 1 ms 时间。
 * @return 绑定时钟的计数；未绑定为 0。
 * @note 裁判任务调用，不等待。
 */
uint32_t app_ui_now_tick(void);
/** @brief 将完整帧交给绑定的输出端口。
 * @param[in] frame 完整帧，仅调用期间借用，输出回调须复制。
 * @param[in] length 有效长度，B，大于 0；容量限制由端口检查。
 * @return 0=接受，1=忙，-1=未绑定、参数或端口错误；成功不代表线缆传输完成。
 * @note 裁判任务独占，非阻塞。
 */
int app_ui_send_frame(const void *frame, uint16_t length);
/** @brief 构造带 CRC 的裁判协议帧并尝试提交。
 * @param[in] cmd_id 裁判协议命令 ID。
 * @param[in] payload 载荷，借用至返回；长度为 0 时允许 NULL。
 * @param[in] length 载荷长度，B；总帧长不得超过 REFEREE_TX_BUF_SIZE。
 * @return 0=接受且推进序号，1=忙且不推进序号，-1=构帧或端口错误。
 * @note 裁判任务独占静态构帧缓冲；输出回调须复制，不等待。
 */
int app_ui_send_payload(uint16_t cmd_id, const void *payload, uint16_t length);
#endif
