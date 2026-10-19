/**
 * @file    app_rtos_board.h
 * @brief   板级装配入口；启动顺序/板型角色集中于此
 */
#ifndef APP_RTOS_BOARD_H
#define APP_RTOS_BOARD_H
#include <stdbool.h>

void app_rtos_board_init(void);      /**< 初始化本板业务及硬件适配，先绑定回调再启动接收。 */
bool app_rtos_ports_start(void);     /**< CAN1/CAN2 + 本板 DBUS/裁判接收，实际失败返回 false。 */
bool app_rtos_board_calibrate(void); /**< 云台完成 IMU 标定，底盘直接成功。 */
bool app_rtos_sampling_start(void);  /**< 云台启动 TIM14，底盘不启动采样定时器。 */

#endif
