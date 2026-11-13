/** @file app_ui_service.h
 * @brief Optional referee-task UI; original simulator demo is explicitly selected.
 */
#ifndef APP_UI_SERVICE_H
#define APP_UI_SERVICE_H
#include "app_referee_data.h"
/** @brief 初始化静态 UI 数据并绑定 UART6 输出。
 * @return 无返回值。APP_UI_MODE 为 0 时无操作，发送错误在端口计数。
 * @note 裁判任务在 PORT_READY 后调用一次；DEMO 模式使用模拟数据。 */
void app_ui_service_init(void);
/** @brief Poll transfer recovery and advance UI at most one frame per 40 ms.
 * @param[in] referee Latest referee business snapshot, borrowed for this call.
 * @return No value. Busy leaves dirty state for retry; errors are counted.
 * @note Referee task only; no waits, no motor control, no RX buffer ownership. */
void app_ui_service_poll(const app_ref_snapshot_t *referee);
#endif
