/**
 * @file    app_motor_role.h
 * @brief   电机反馈快照的稳定槽位；协议解码和控制计算共用
 */
#ifndef APP_MOTOR_ROLE_H
#define APP_MOTOR_ROLE_H

/* 云台与发射共用一份六电机反馈，槽位不再在三个模块分别定义。 */
enum
{
    MOTOR_YAW_L = 0,
    MOTOR_YAW_S = 1,
    MOTOR_PITCH = 2,
    MOTOR_FRIC_L = 3,
    MOTOR_DISC = 4,
    MOTOR_FRIC_R = 5,
    GIMBAL_MOTOR_COUNT = 6
};

/* 底盘板独立的反馈排列；名称表达实际部件，不是 CAN ID。 */
enum
{
    M_DRIVE_R = 0,
    M_DRIVE_L = 1,
    M_STEER_L = 2,
    M_STEER_R = 3,
    CHASSIS_MOTOR_COUNT = 4
};
#endif
