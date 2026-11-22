/**
 * @file    project_cfg.h
 * @brief   唯一配置入口 — 选择板型 + 车组
 */
#ifndef PROJECT_CFG_H
#define PROJECT_CFG_H

// ========== [必改] 选择主板 ==========
#define BOARD_CHASSIS 0
#define BOARD_GIMBAL 1

/* CMake 可通过编译定义覆盖 CURRENT_BOARD；未传入时保留当前底盘板默认值。
 * Os 任务表和 App 初始化共用此值，决定本板实际创建哪些任务及初始化哪些机构。 */
#ifndef CURRENT_BOARD
#define CURRENT_BOARD BOARD_CHASSIS // 可选: BOARD_CHASSIS / BOARD_GIMBAL
#endif

// ========== [必改] 选择车组 ==========
#define ROBOT_ARMORED 4
#define ROBOT_INFANTRY 1
#define ROBOT_SENTRY 2
#define ROBOT_NAVIGATION 3

/* 哨兵、导航、重装为三个独立车型。车组与板型分别选择：同一哨兵工程可构建云台板或底盘板。 */
#ifndef CURRENT_ROBOT
#define CURRENT_ROBOT                                                                              \
    ROBOT_SENTRY // ROBOT_SENTRY / ROBOT_NAVIGATION / ROBOT_ARMORED；重装整车控制尚未接入调度。
#endif

/* 编译时尽早拒绝无效板型，防止任务表与硬件角色不一致。 */
#if CURRENT_BOARD != BOARD_CHASSIS && CURRENT_BOARD != BOARD_GIMBAL
#error "Unsupported CURRENT_BOARD"
#endif

#if CURRENT_ROBOT != ROBOT_SENTRY && CURRENT_ROBOT != ROBOT_NAVIGATION &&                          \
    CURRENT_ROBOT != ROBOT_ARMORED
#error "Unsupported CURRENT_ROBOT"
#endif

#if defined(APP_UI_MODE) && APP_UI_MODE != 0 && CURRENT_BOARD != BOARD_CHASSIS
#error "Referee UI requires BOARD_CHASSIS"
#endif

#ifndef IMU_AHRS_ALGORITHM
#define IMU_AHRS_ALGORITHM DRV_IMU_ALGO_VQF
#endif
#ifndef IMU_MOUNT_ROLL_DEG
#define IMU_MOUNT_ROLL_DEG 0.0f
#endif
#ifndef IMU_MOUNT_PITCH_DEG
#if CURRENT_ROBOT == ROBOT_NAVIGATION
#define IMU_MOUNT_PITCH_DEG 0.0f
#else
#define IMU_MOUNT_PITCH_DEG (-90.0f)
#endif
#endif
#ifndef IMU_MOUNT_YAW_DEG
#define IMU_MOUNT_YAW_DEG 0.0f
#endif

#ifndef APP_UI_MODE
#define APP_UI_MODE 0
#endif

#if APP_UI_MODE < 0 || APP_UI_MODE > 2
#error "Unsupported APP_UI_MODE"
#endif

#endif /* PROJECT_CFG_H */
