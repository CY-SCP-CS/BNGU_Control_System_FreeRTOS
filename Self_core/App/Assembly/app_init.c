/**
 * @file    app_init.c
 * @brief   按 CURRENT_BOARD 初始化本板应用算法状态。
 */
#include "app_init.h"
#include "project_cfg.h"
#include "app_sentry_gimbal.h"
#include "app_sentry_chassis.h"
#include "app_sentry_shoot.h"
#include "navigation_ctrl.h"

/**
 * @brief  初始化当前板型使用的机构算法、PID 和滤波状态。
 * @note   由 sys_init_Task 在启动接收和发布系统 READY 前调用一次。
 * @note   云台板初始化云台与发射机构，底盘板初始化底盘；硬件启动、IMU 标定和任务创建由其他层负责。
 */
void app_init(void)
{
#if CURRENT_ROBOT == ROBOT_NAVIGATION
    navigation_ctrl_init();
#elif CURRENT_BOARD == BOARD_GIMBAL
    app_sentry_gimbal_init();
    app_sentry_shoot_init();
#else
    app_sentry_chassis_init();
#endif
}
