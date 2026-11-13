/**
 * @file    app_init.h
 * @brief   按 CURRENT_BOARD 初始化本板应用算法状态。
 */
#ifndef APP_INIT_H
#define APP_INIT_H
/**
 * @brief  初始化当前板型使用的机构算法、PID 和滤波状态。
 * @note   由 sys_init_Task 在启动接收和发布系统 READY 前调用一次。
 * @note   云台板初始化云台与发射机构，底盘板初始化底盘；硬件启动、IMU 标定和任务创建由其他层负责。
 */
void app_init(void);
#endif
