/**
 * @file    project_cfg.h
 * @brief   唯一配置入口 — 选择板型 + 车组
 */
#ifndef PROJECT_CFG_H
#define PROJECT_CFG_H

// ========== [必改] 选择主板 ==========
#define BOARD_CHASSIS 0
#define BOARD_GIMBAL 1

/* CMake 可通过编译定义覆盖 CURRENT_BOARD；未传入时默认云台板。
 * Os 任务表和 App 初始化共用此值，决定本板实际创建哪些任务及初始化哪些机构。 */
#ifndef CURRENT_BOARD
#define CURRENT_BOARD BOARD_CHASSIS // 可选: BOARD_CHASSIS / BOARD_GIMBAL
#endif

// ========== [必改] 选择车组 ==========
#define ROBOT_HERO 0
#define ROBOT_INFANTRY 1
#define ROBOT_SENTRY 2

/* 当前仅完成哨兵算法。车组与板型分别选择：同一哨兵工程可构建云台板或底盘板。 */
#ifndef CURRENT_ROBOT
#define CURRENT_ROBOT ROBOT_SENTRY // 当前只实现哨兵；其他车组选择会报错。
#endif

/* 编译时尽早拒绝无效板型，防止任务表与硬件角色不一致。 */
#if CURRENT_BOARD != BOARD_CHASSIS && CURRENT_BOARD != BOARD_GIMBAL
#error "Unsupported CURRENT_BOARD"
#endif

#if CURRENT_ROBOT != ROBOT_SENTRY
#error "Only ROBOT_SENTRY is implemented"
#endif

#endif
