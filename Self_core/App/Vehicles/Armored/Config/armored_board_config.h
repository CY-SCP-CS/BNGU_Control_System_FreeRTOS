/** Armored board capability policy. */
#ifndef ARMORED_BOARD_CONFIG_H
#define ARMORED_BOARD_CONFIG_H
/* ARMORED algorithms are independent; no complete vehicle control task is wired yet. */
#define APP_BOARD_SHOOT_MASK 0U
#define APP_BOARD_GIMBAL_CONTROL_MASK 0U
#define APP_BOARD_CHASSIS_CONTROL_MASK 0U

#define APP_BOARD_DBUS_MASK 0U
#define APP_BOARD_IMU_MASK 0U

#define APP_BOARD_EXTRA_MOTOR_MASK 0U
#define APP_BOARD_OMEGA_MASK 0U
#define APP_BOARD_POWER_MASK 0U
#endif
