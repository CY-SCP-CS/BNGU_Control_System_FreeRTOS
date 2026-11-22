/**
 * @file    navigation_control_config.h
 * @brief   控制时间配置；调度节拍和算法积分时间使用同一来源
 */
#ifndef NAVIGATION_CONTROL_CONFIG_H
#define NAVIGATION_CONTROL_CONFIG_H
#define APP_CONTROL_GIM_PERIOD_MS 1U
#define APP_CONTROL_CHAS_PERIOD_MS 1U
#define APP_CONTROL_SHOOT_PERIOD_MS 5U
#define APP_CONTROL_FEEDBACK_PERIOD_MS 5U
#define APP_CONTROL_GIM_DT_S (APP_CONTROL_GIM_PERIOD_MS * 0.001f)
#define APP_CONTROL_CHAS_DT_S (APP_CONTROL_CHAS_PERIOD_MS * 0.001f)
#define APP_CONTROL_SHOOT_DT_S (APP_CONTROL_SHOOT_PERIOD_MS * 0.001f)
/* Task RX service and wheel-control periods have different responsibilities. */
#define NAV_RX_SERVICE_PERIOD_MS APP_CONTROL_CHAS_PERIOD_MS
#define NAV_CONTROL_PERIOD_MS 5U
#define NAV_CONTROL_DT_S (NAV_CONTROL_PERIOD_MS * 0.001f)
#endif
