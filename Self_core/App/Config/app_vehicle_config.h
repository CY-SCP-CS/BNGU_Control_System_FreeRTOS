/**
 * @file    app_board_config.h
 * @brief   板型能力配置；新增板型在此明确能力与装配关系
 */
#ifndef APP_VEHICLE_CONFIG_H
#define APP_VEHICLE_CONFIG_H
#include "project_cfg.h"

/* 板型能力是任务、端口绑定和诊断启用条件的共同入口。 */
#define APP_BOARD_GIMBAL_MASK (1U << BOARD_GIMBAL)
#define APP_BOARD_CHASSIS_MASK (1U << BOARD_CHASSIS)
#define APP_BOARD_ALL_MASK (APP_BOARD_GIMBAL_MASK | APP_BOARD_CHASSIS_MASK)
#define APP_BOARD_CURRENT_MASK (1U << CURRENT_BOARD)
/* 同一能力掩码同时供任务创建、诊断清单和硬件装配使用。 */
#define APP_BOARD_CAN_MASK APP_BOARD_ALL_MASK
/* No armored motor protocol is bound until its vehicle controller is integrated. */
#define APP_BOARD_MOTOR_MASK (APP_BOARD_GIMBAL_CONTROL_MASK | APP_BOARD_CHASSIS_CONTROL_MASK)
#if CURRENT_ROBOT == ROBOT_SENTRY
#include "../Vehicles/Sentry/Config/sentry_board_config.h"
#include "../Vehicles/Sentry/Config/sentry_control_config.h"
#include "../Vehicles/Sentry/Config/sentry_input_config.h"
#elif CURRENT_ROBOT == ROBOT_NAVIGATION
#include "../Vehicles/Navigation/Config/navigation_board_config.h"
#include "../Vehicles/Navigation/Config/navigation_control_config.h"
#include "../Vehicles/Navigation/Config/navigation_input_config.h"
#elif CURRENT_ROBOT == ROBOT_ARMORED
#include "../Vehicles/Armored/Config/armored_board_config.h"
#include "../Vehicles/Armored/Config/armored_control_config.h"
#include "../Vehicles/Armored/Config/armored_input_config.h"
#else
#error "Unsupported vehicle board policy"
#endif

#define APP_BOARD_REFEREE_MASK APP_BOARD_CHASSIS_MASK
#define APP_BOARD_HAS_GIMBAL ((APP_BOARD_CURRENT_MASK & APP_BOARD_GIMBAL_CONTROL_MASK) != 0)
#define APP_BOARD_HAS_CHASSIS ((APP_BOARD_CURRENT_MASK & APP_BOARD_CHASSIS_CONTROL_MASK) != 0)
#define APP_BOARD_HAS_CAN ((APP_BOARD_CURRENT_MASK & APP_BOARD_CAN_MASK) != 0)
#define APP_BOARD_HAS_SHOOT ((APP_BOARD_CURRENT_MASK & APP_BOARD_SHOOT_MASK) != 0)
#define APP_BOARD_HAS_DBUS ((APP_BOARD_CURRENT_MASK & APP_BOARD_DBUS_MASK) != 0)
#define APP_BOARD_HAS_IMU ((APP_BOARD_CURRENT_MASK & APP_BOARD_IMU_MASK) != 0)
#define APP_BOARD_HAS_REFEREE ((APP_BOARD_CURRENT_MASK & APP_BOARD_REFEREE_MASK) != 0)

#if APP_BOARD_HAS_GIMBAL
#define APP_TASK_GIMBAL_ENTRY Start_ctrl_gim_Task
#else
#define APP_TASK_GIMBAL_ENTRY NULL
#endif
#if APP_BOARD_HAS_SHOOT
#define APP_TASK_SHOOT_ENTRY Start_ctrl_shoot_Task
#else
#define APP_TASK_SHOOT_ENTRY NULL
#endif
#if CURRENT_ROBOT == ROBOT_NAVIGATION && APP_BOARD_HAS_CHASSIS
#define APP_TASK_CHASSIS_ENTRY Start_navigation_Task
#define APP_TASK_CHASSIS_NAME "navigation_Task"
#elif CURRENT_ROBOT == ROBOT_SENTRY && APP_BOARD_HAS_CHASSIS
#define APP_TASK_CHASSIS_ENTRY Start_ctrl_chas_Task
#define APP_TASK_CHASSIS_NAME "ctrl_chas_Task"
#else
#define APP_TASK_CHASSIS_ENTRY NULL
#define APP_TASK_CHASSIS_NAME "chassis_control_disabled"
#endif
#endif
