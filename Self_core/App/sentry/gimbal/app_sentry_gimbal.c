/**
 * @file    app_sentry_gimbal.c
 * @brief   哨兵云台纯算法：双 yaw PID、pitch 控制与控制源裁决。
 * @note    调度、IMU 解算、发射计算和 CAN 发送由各自任务负责。
 */
#include "app_control_config.h"
#include "app_motor_role.h"
#include "app_sentry_gimbal.h"

#include "lib_filter.h"
#include "lib_math.h"

#include "drv_imu.h"
#include "drv_motor.h"

#include "app_sentry_common.h"
#include "app_gimbal_comm.h"
#include "app_chassis_comm.h"

#include <math.h>
#include <string.h>

#define GIMBAL_CHASSIS_OMEGA_TIMEOUT_MS 20U

/** 云台电机在反馈表和发送电流帧中的索引。 */

/** 电机反馈快照、CAN 接收缓冲和电流输出。 */
typedef struct
{
    drv_motor_data_t feedback[GIMBAL_MOTOR_COUNT]; // 控制循环使用的稳定反馈快照。
    int16_t current[3];                            // 大 yaw、小 yaw、pitch 电流。
    float pitch_cur_speed;                         // Pitch angular speed from motor feedback.
    lib_lpf_t pitch_speed_lpf;                     // Pitch speed low-pass filter.
} app_sentry_gimbal_motor_state_t;

/** IMU 角度、角速度及其低通滤波状态。 */
typedef struct
{
    const app_imu_snapshot_t *instance; // 只在本次计算期间引用输入快照。
    float yaw_cur_angle;                // 当前 yaw 角度，rad。
    float pitch_cur_angle;              // 当前 pitch 角度，rad。
    float yaw_gyro;                     // 当前 yaw 陀螺角速度，rad/s。
    lib_lpf_t yaw_gyro_lpf;             // yaw 角速度低通滤波器。
} app_sentry_gimbal_imu_state_t;

/** 云台协议输入与控制器内部目标。 */
typedef struct
{
    app_sentry_gimbal_cmd_t data; // 当前控制输入。
    float yaw_tar_angle;          // yaw 目标角度，rad。
    float pitch_tar_angle;        // pitch 目标角度，rad。
} app_sentry_gimbal_command_state_t;

/** 云台转发给底盘的命令及从底盘收到的角速度。 */
typedef struct
{
    float vw_cur_speed;                     // 底盘当前对地角速度，rad/s。
    float yaw_rel_angle;                    // 相机（小 yaw）相对底盘 x 轴的朝向，rad。
    uint32_t vw_rx_tick_ms;                 // 最近一次 0x119 接收时间。
    uint8_t vw_is_valid;                    // 已收到有效的 0x119 角速度。
    int16_t vx_tar_speed;                   // 底盘 x 方向目标速度，mm/s。
    int16_t vy_tar_speed;                   // 底盘 y 方向目标速度，mm/s。
    int16_t vw_tar_speed;                   // 底盘目标角速度的 0.001 rad/s 协议值。
    uint8_t emergency_stop;                 // 底盘急停状态。
    app_gimbal_input_source_t input_source; // 当前控制源。
} app_sentry_gimbal_chassis_state_t;

/** 需要跨控制周期保存的控制状态。 */
typedef struct
{
    uint8_t is_ready; // 首次有效反馈完成目标对齐后置位。
} app_sentry_gimbal_control_state_t;

/** 云台各控制环 PID 状态。 */
typedef struct
{
    lib_pid_t yaw_large; // 大 yaw 绝对角度环。
    lib_pid_t yaw_small; // 小 yaw 绝对角度环。
    lib_pid_t pitch;     // pitch 角度环。
} app_sentry_gimbal_pid_state_t;

/**
 * @brief 云台全局调试快照。
 *
 * 保留外部链接便于调试器 Watch 在任意断点查看；不在头文件声明，
 * 不作为模块间访问接口。
 */
typedef struct
{
    app_sentry_gimbal_motor_state_t motor;
    app_sentry_gimbal_imu_state_t imu;
    app_sentry_gimbal_command_state_t command;
    app_sentry_gimbal_chassis_state_t chassis;
    app_sentry_gimbal_control_state_t control;
    app_sentry_gimbal_pid_state_t pid;
} app_sentry_gimbal_debug_t;

app_sentry_gimbal_debug_t app_sentry_gimbal_debug = {
    .chassis =
        {
            .input_source = APP_GIMBAL_INPUT_CAN,
        },
};

#define s_motor_state app_sentry_gimbal_debug.motor
#define s_imu_state app_sentry_gimbal_debug.imu
#define s_command_state app_sentry_gimbal_debug.command
#define s_chassis_state app_sentry_gimbal_debug.chassis
#define s_control_state app_sentry_gimbal_debug.control
#define s_pid_yaw_large app_sentry_gimbal_debug.pid.yaw_large
#define s_pid_yaw_small app_sentry_gimbal_debug.pid.yaw_small
#define s_pid_pitch app_sentry_gimbal_debug.pid.pitch

/* 临时借用当前控制调用的输入；只有云台任务访问，公开入口返回前置 NULL。 */
static const app_gimbal_input_t *s_input;
static void gimbal_reset(void);
static void gimbal_update_cmd(const app_gimbal_dbus_input_t *input);
static void gimbal_apply_can_cmd(const app_gimbal_comm_rx_t *, uint8_t);
static float gimbal_coord_trans(void);
static void imu_fusion(void);
static void yaw_pid_control(void);
static void pitch_control(void);

/**
 * @brief  装载三个云台位置环的参数，并初始化 yaw/pitch 角速度低通滤波器。
 * @note   仅操作云台任务私有状态；运行中的急停复位使用 gimbal_reset()。
 */
static void pid_init_all(void)
{
    lib_pid_init(&s_pid_yaw_large, 30000.0f, 1500.0f, 7200.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX, 1000.0f);
    lib_pid_init(&s_pid_yaw_small, 30000.0f, 2000.0f, 1700.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX, 1000.0f);
    lib_pid_init(&s_pid_pitch, 32000.0f, 2000.0f, 1550.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX, 1000.0f);
    lib_lpf_init(&s_imu_state.yaw_gyro_lpf, 25.0f);
    lib_lpf_init(&s_motor_state.pitch_speed_lpf, 25.0f);
}

/**
 * @brief  清零云台电流、底盘目标和发射命令，复位 PID 并要求重新对齐目标。
 * @note   在急停或姿态/角速度无效时调用；只生成当前算法状态，不直接向 CAN 总线发送。
 */
static void gimbal_reset(void)
{
    memset(s_motor_state.current, 0, sizeof(s_motor_state.current));
    s_command_state.data.fire = APP_SENTRY_FIRE_OFF;
    s_command_state.data.yaw_tar_speed = 0.0f;
    s_chassis_state.vx_tar_speed = 0;
    s_chassis_state.vy_tar_speed = 0;
    s_chassis_state.vw_tar_speed = 0;
    s_chassis_state.input_source = APP_GIMBAL_INPUT_ESTOP;
    s_chassis_state.emergency_stop = 1U;
    s_control_state.is_ready = 0U;
    lib_pid_reset(&s_pid_yaw_large);
    lib_pid_reset(&s_pid_yaw_small);
    lib_pid_reset(&s_pid_pitch);
}

/**
 * @brief  裁决 DBUS、CAN 与急停输入，并更新云台目标和底盘转发速度。
 * @param  input        归一化遥控输入及 S2 控制源；NULL 时直接返回。
 * @note   CAN 模式按本周期 updated_mask 应用新指令；遥控模式将角速度按 0.001 s 积分为角度目标。
 * @note   遥控底盘平移指令从云台坐标系旋转到底盘坐标系；急停状态由输出接口传给任务。
 */
static void gimbal_update_cmd(const app_gimbal_dbus_input_t *input)
{
    app_gimbal_comm_rx_t can_rx = s_input->can;
    uint8_t updated_mask = s_input->updated_mask;

    if (!input)
    {
        return;
    }

    /* 非 CAN 模式也清空更新标志，防止切换控制源时执行旧指令。 */

    s_chassis_state.input_source = input->source;
    s_chassis_state.emergency_stop = (input->source == APP_GIMBAL_INPUT_ESTOP);
    if (s_chassis_state.emergency_stop)
    {
        s_chassis_state.vx_tar_speed = 0;
        s_chassis_state.vy_tar_speed = 0;
        s_chassis_state.vw_tar_speed = 0;
        return;
    }
    if (input->source == APP_GIMBAL_INPUT_CAN)
    {
        s_command_state.data.yaw_tar_speed = 0.0f;
        gimbal_apply_can_cmd(&can_rx, updated_mask);
        return;
    }

    s_command_state.data.mode = APP_SENTRY_GIMBAL_MODE_SPEED;
    s_command_state.data.fire = input->launcher_mode;
    s_command_state.data.yaw_tar_speed = input->yaw_rate_norm * SENTRY_GIMBAL_YAW_TAR_SPEED_MAX;
    s_command_state.data.pitch_tar_speed =
        input->pitch_rate_norm * SENTRY_GIMBAL_PITCH_TAR_SPEED_MAX;

    float vx_gimbal_tar_speed = input->chassis_vx_norm * SENTRY_CHASSIS_VX_TAR_SPEED_MAX;
    float vy_gimbal_tar_speed = input->chassis_vy_norm * SENTRY_CHASSIS_VY_TAR_SPEED_MAX;
    float camera_yaw_rel_angle = gimbal_coord_trans();
    float cos_yaw = cosf(camera_yaw_rel_angle);
    float sin_yaw = sinf(camera_yaw_rel_angle);
    s_chassis_state.yaw_rel_angle = camera_yaw_rel_angle;
    s_chassis_state.vx_tar_speed =
        (int16_t)lroundf(cos_yaw * vx_gimbal_tar_speed - sin_yaw * vy_gimbal_tar_speed);
    s_chassis_state.vy_tar_speed =
        (int16_t)lroundf(sin_yaw * vx_gimbal_tar_speed + cos_yaw * vy_gimbal_tar_speed);
    s_chassis_state.vw_tar_speed =
        (int16_t)lroundf(input->chassis_omega_norm * SENTRY_CHASSIS_VW_TAR_SPEED_MAX /
                         APP_CHASSIS_OMEGA_RAD_S_PER_LSB);

    s_command_state.yaw_tar_angle += s_command_state.data.yaw_tar_speed * APP_CONTROL_GIM_DT_S;
    s_command_state.pitch_tar_angle += s_command_state.data.pitch_tar_speed * APP_CONTROL_GIM_DT_S;
    s_command_state.yaw_tar_angle = lib_rad_norm(s_command_state.yaw_tar_angle);
}

/**
 * @brief  按固定顺序应用本周期新增的增量、绝对角度和发射指令。
 * @param  rx           本周期协议缓存，NULL 时忽略。
 * @param  updated_mask 本周期新到指令的位掩码；为 0 的字段不会再次应用。
 * @note   先处理两种增量，再处理两种绝对角度，最后处理 0x129；因此同周期绝对角度可覆盖前面的增量结果。
 * @note   0x120 底盘指令由任务层转发，不在此函数中发送或重编码。
 */
static void gimbal_apply_can_cmd(const app_gimbal_comm_rx_t *rx, uint8_t updated_mask)
{
    if (!rx)
    {
        return;
    }
    if (updated_mask & APP_GIMBAL_COMM_UPDATE_SPEED_NO_SHOOT)
    {
        s_command_state.yaw_tar_angle =
            lib_rad_norm(s_command_state.yaw_tar_angle + rx->speed_no_shoot.yaw_inc);
        s_command_state.pitch_tar_angle += rx->speed_no_shoot.pitch_inc;
    }
    if (updated_mask & APP_GIMBAL_COMM_UPDATE_SPEED_SHOOT)
    {
        s_command_state.yaw_tar_angle =
            lib_rad_norm(s_command_state.yaw_tar_angle + rx->speed_shoot.yaw_inc);
        s_command_state.pitch_tar_angle += rx->speed_shoot.pitch_inc;
    }
    if (updated_mask & APP_GIMBAL_COMM_UPDATE_ANGLE_NO_SHOOT)
    {
        s_command_state.yaw_tar_angle = lib_rad_norm(rx->angle_no_shoot.yaw_abs);
        s_command_state.pitch_tar_angle = rx->angle_no_shoot.pitch_abs;
    }
    if (updated_mask & APP_GIMBAL_COMM_UPDATE_ANGLE_SHOOT)
    {
        s_command_state.yaw_tar_angle = lib_rad_norm(rx->angle_shoot.yaw_abs);
        s_command_state.pitch_tar_angle = rx->angle_shoot.pitch_abs;
    }
    if (updated_mask & APP_GIMBAL_COMM_UPDATE_SHOOT)
    {
        if (rx->shoot.shoot_switch == 0xFFU && rx->shoot.retreat == 0x00U)
        {
            s_command_state.data.fire = APP_SENTRY_FIRE_ON;
        }
        else if (rx->shoot.shoot_switch == 0xFFU && rx->shoot.retreat == 0xFFU)
        {
            s_command_state.data.fire = APP_SENTRY_FIRE_FIR;
        }
        else if (rx->shoot.shoot_switch == 0x00U && rx->shoot.retreat == 0xFFU)
        {
            s_command_state.data.fire = APP_SENTRY_FIRE_REVERSE;
        }
        else
        {
            s_command_state.data.fire = APP_SENTRY_FIRE_OFF;
        }
    }
}

/**
 * @brief  根据大小 yaw 编码器计算云台相对底盘的平面朝向。
 * @return 按机械零位及方向修正后的相对角，单位 rad，并归一化到角度环范围。
 * @note   用于把遥控平移速度旋转到底盘坐标系；只读取本周期电机反馈副本。
 */
static float gimbal_coord_trans(void)
{
    float large_yaw_cur_angle = lib_get_shortest_path(
        lib_enc_conv((float)s_motor_state.feedback[MOTOR_YAW_L].angle, LIB_ENC13_TO_RAD),
        lib_enc_conv((float)SENTRY_GIMBAL_L_YAW_ZERO, LIB_ENC13_TO_RAD));
    float small_yaw_cur_angle = lib_get_shortest_path(
        lib_enc_conv((float)s_motor_state.feedback[MOTOR_YAW_S].angle, LIB_ENC13_TO_RAD),
        lib_enc_conv((float)SENTRY_GIMBAL_S_YAW_ZERO, LIB_ENC13_TO_RAD));

    return lib_rad_norm(-SENTRY_GIMBAL_L_YAW_DIR * large_yaw_cur_angle -
                        SENTRY_GIMBAL_S_YAW_DIR * small_yaw_cur_angle);
}

/**
 * @brief  从 IMU 快照与 pitch 编码器构造云台控制使用的姿态和角速度。
 * @note   IMU yaw 从度转换为 rad；pitch 使用 IMU roll 转 rad 后叠加 pitch 编码器相对角。
 * @note   yaw 角速度取 gyro.z 并作低通滤波；这里不运行 Mahony，姿态解算已由 IMU 任务完成。
 */
static void imu_fusion(void)
{
    if (!s_imu_state.instance)
    {
        return;
    }

    s_imu_state.yaw_cur_angle = lib_deg_to_rad(s_imu_state.instance->euler.yaw);

    float pitch_cur_angle = lib_get_shortest_path(
        lib_enc_conv((float)s_motor_state.feedback[MOTOR_PITCH].angle, LIB_ENC13_TO_RAD),
        lib_enc_conv((float)SENTRY_GIMBAL_PITCH_ZERO, LIB_ENC13_TO_RAD));
    s_imu_state.pitch_cur_angle =
        lib_deg_to_rad(s_imu_state.instance->euler.roll) + pitch_cur_angle;

    s_imu_state.yaw_gyro = lib_lpf_update(&s_imu_state.yaw_gyro_lpf, s_imu_state.instance->gyro.z,
                                          APP_CONTROL_GIM_DT_S);
}

/**
 * @brief  计算双 yaw 位置环电流，并处理底盘角速度补偿和小 yaw 机械边界。
 * @note   输入角度单位 rad、速度单位 rad/s；底盘角速度过期时按现有逻辑不参与补偿。
 * @note   计算过程使用本任务独占的 PID；结果写入大 yaw、小 yaw 电流槽。
 */
static void yaw_pid_control(void)
{
    float yaw_cur_angle = s_imu_state.yaw_cur_angle;
    float small_yaw_cur_angle =
        SENTRY_GIMBAL_S_YAW_DIR *
        lib_enc13_relative_rad(s_motor_state.feedback[MOTOR_YAW_S].angle, SENTRY_GIMBAL_S_YAW_ZERO);
    float small_yaw_tar_angle =
        yaw_cur_angle + lib_get_shortest_path(s_command_state.yaw_tar_angle, yaw_cur_angle);
    float large_yaw_tar_angle =
        yaw_cur_angle +
        lib_get_shortest_path(lib_rad_norm(s_command_state.yaw_tar_angle + small_yaw_cur_angle),
                              yaw_cur_angle);
    float large_yaw_cur_speed = SENTRY_GIMBAL_L_YAW_DIR *
                                lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_YAW_L].speed);
    float small_yaw_cur_speed = SENTRY_GIMBAL_S_YAW_DIR *
                                lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_YAW_S].speed);
    float chassis_yaw_speed = 0.0f;
    float large_yaw_ground_speed;
    float small_yaw_ground_speed;
    float chassis_yaw_feedback;
    uint32_t chassis_omega_rx_tick_ms;
    uint8_t has_chassis_omega;
    uint8_t chassis_omega_is_valid;
    float small_yaw_current;
    float limit_weight = 0.0f;

    chassis_yaw_feedback = s_chassis_state.vw_cur_speed;
    chassis_omega_rx_tick_ms = s_chassis_state.vw_rx_tick_ms;
    chassis_omega_is_valid = s_chassis_state.vw_is_valid;

    has_chassis_omega =
        chassis_omega_is_valid &&
        (uint32_t)(s_input->tick - chassis_omega_rx_tick_ms) <= GIMBAL_CHASSIS_OMEGA_TIMEOUT_MS;
    if (has_chassis_omega)
    {
        chassis_yaw_speed = chassis_yaw_feedback;
    }
    if (has_chassis_omega)
    {
        large_yaw_ground_speed = chassis_yaw_speed + large_yaw_cur_speed;
        small_yaw_ground_speed = large_yaw_ground_speed + small_yaw_cur_speed;
    }
    else
    {
        /* 没有可信底盘速度时，维持原来的相对速度阻尼。 */
        large_yaw_ground_speed = large_yaw_cur_speed;
        small_yaw_ground_speed = small_yaw_cur_speed;
    }

    small_yaw_current = lib_pid_pos_calc(&s_pid_yaw_small, small_yaw_tar_angle, yaw_cur_angle, 0.0f,
                                         0.0f, small_yaw_ground_speed, APP_CONTROL_GIM_DT_S);
    s_motor_state.current[MOTOR_YAW_L] =
        (int16_t)(SENTRY_GIMBAL_L_YAW_DIR *
                  lib_pid_pos_calc(&s_pid_yaw_large, large_yaw_tar_angle, yaw_cur_angle, 0.0f, 0.0f,
                                   large_yaw_ground_speed, APP_CONTROL_GIM_DT_S));

    if (small_yaw_cur_angle > SENTRY_GIMBAL_S_YAW_LIMIT - SENTRY_GIMBAL_S_YAW_SOFT_ZONE)
    {
        limit_weight = lib_remap_clamp(small_yaw_cur_angle,
                                       SENTRY_GIMBAL_S_YAW_LIMIT - SENTRY_GIMBAL_S_YAW_SOFT_ZONE,
                                       SENTRY_GIMBAL_S_YAW_LIMIT, 0.0f, 1.0f);
        if (small_yaw_current > 0.0f)
        {
            small_yaw_current *= 1.0f - limit_weight;
            s_pid_yaw_small.integral = 0.0f;
        }
        if (small_yaw_cur_speed > 0.0f)
        {
            small_yaw_current -=
                SENTRY_GIMBAL_S_YAW_BRAKE_GAIN * limit_weight * small_yaw_cur_speed;
            s_pid_yaw_small.integral = 0.0f;
        }
    }
    else if (small_yaw_cur_angle < -SENTRY_GIMBAL_S_YAW_LIMIT + SENTRY_GIMBAL_S_YAW_SOFT_ZONE)
    {
        limit_weight =
            lib_remap_clamp(small_yaw_cur_angle, -SENTRY_GIMBAL_S_YAW_LIMIT,
                            -SENTRY_GIMBAL_S_YAW_LIMIT + SENTRY_GIMBAL_S_YAW_SOFT_ZONE, 1.0f, 0.0f);
        if (small_yaw_current < 0.0f)
        {
            small_yaw_current *= 1.0f - limit_weight;
            s_pid_yaw_small.integral = 0.0f;
        }
        if (small_yaw_cur_speed < 0.0f)
        {
            small_yaw_current -=
                SENTRY_GIMBAL_S_YAW_BRAKE_GAIN * limit_weight * small_yaw_cur_speed;
            s_pid_yaw_small.integral = 0.0f;
        }
    }

    small_yaw_current =
        lib_clamp(small_yaw_current, DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX);
    s_pid_yaw_small.out = small_yaw_current;
    s_motor_state.current[MOTOR_YAW_S] = (int16_t)(SENTRY_GIMBAL_S_YAW_DIR * small_yaw_current);
}

/**
 * @brief  计算 pitch 位置环电流并在机械边界附近削弱向外推动的电流。
 * @note   先限幅目标角，再加入现有重力前馈；软限位处按方向缩放电流，必要时清除积分项。
 */
static void pitch_control(void)
{
    float pitch_cur_angle = s_imu_state.pitch_cur_angle;
    float pitch_tar_angle;
    float pitch_current;

    s_command_state.pitch_tar_angle = lib_clamp(s_command_state.pitch_tar_angle,
                                                SENTRY_GIMBAL_PITCH_MIN, SENTRY_GIMBAL_PITCH_MAX);
    pitch_tar_angle = s_command_state.pitch_tar_angle;

    pitch_current =
        lib_pid_pos_calc(&s_pid_pitch, pitch_tar_angle, pitch_cur_angle, cosf(pitch_cur_angle),
                         0.0f, s_motor_state.pitch_cur_speed, APP_CONTROL_GIM_DT_S);

    /* In the soft zone, progressively remove only the current driving farther into a limit. */
    if (pitch_current > 0.0f &&
        pitch_cur_angle > SENTRY_GIMBAL_PITCH_MAX - SENTRY_GIMBAL_PITCH_SOFT_ZONE)
    {
        pitch_current *= lib_remap_clamp(pitch_cur_angle,
                                         SENTRY_GIMBAL_PITCH_MAX - SENTRY_GIMBAL_PITCH_SOFT_ZONE,
                                         SENTRY_GIMBAL_PITCH_MAX, 1.0f, 0.0f);
        if (pitch_tar_angle >= pitch_cur_angle)
        {
            s_pid_pitch.integral = 0.0f;
        }
    }
    else if (pitch_current < 0.0f &&
             pitch_cur_angle < SENTRY_GIMBAL_PITCH_MIN + SENTRY_GIMBAL_PITCH_SOFT_ZONE)
    {
        pitch_current *=
            lib_remap_clamp(pitch_cur_angle, SENTRY_GIMBAL_PITCH_MIN,
                            SENTRY_GIMBAL_PITCH_MIN + SENTRY_GIMBAL_PITCH_SOFT_ZONE, 0.0f, 1.0f);
        if (pitch_tar_angle <= pitch_cur_angle)
        {
            s_pid_pitch.integral = 0.0f;
        }
    }

    s_pid_pitch.out = pitch_current;
    s_motor_state.current[MOTOR_PITCH] = (int16_t)pitch_current;
}

/**
 * @brief  执行姿态检查、初始目标对齐、控制源选择和三轴控制。
 * @note   仅由公开的单周期入口调用；使用 s_input 指向的本周期快照。
 * @note   没有有效 DBUS 时沿用默认 CAN 控制源；姿态无效或急停时调用 gimbal_reset()。
 */
static void gimbal_compute(void)
{
    app_gimbal_dbus_input_t input = {
        /* 未连接 DBUS 时默认使用 CAN；没有 CAN 指令时各目标保持初始零值。 */
        .source = APP_GIMBAL_INPUT_CAN,
    }; //感觉逻辑还是有点问题
    s_motor_state.pitch_cur_speed = lib_lpf_update(
        &s_motor_state.pitch_speed_lpf,
        lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_PITCH].speed), APP_CONTROL_GIM_DT_S);

    imu_fusion();
    if (s_input->dbus.valid)
    {
        input = s_input->dbus.input;
    }

    if (!s_imu_state.instance || !isfinite(s_imu_state.yaw_cur_angle) ||
        !isfinite(s_imu_state.pitch_cur_angle) || !isfinite(s_imu_state.yaw_gyro) ||
        !isfinite(s_motor_state.pitch_cur_speed))
    {
        gimbal_reset();

        return;
    }
    if (!s_control_state.is_ready)
    {
        s_command_state.yaw_tar_angle = s_imu_state.yaw_cur_angle;
        s_command_state.pitch_tar_angle = 0.0f;
        lib_pid_reset(&s_pid_yaw_large);
        lib_pid_reset(&s_pid_yaw_small);
        lib_pid_reset(&s_pid_pitch);
        s_control_state.is_ready = 1;
    }
    gimbal_update_cmd(&input);
    if (s_chassis_state.emergency_stop)
    {
        gimbal_reset();

        return;
    }

    yaw_pid_control();

    pitch_control();
}

/**
 * @brief  清零云台内部状态，装载 PID 参数并初始化角速度滤波器。
 * @note   仅在控制任务开始前调用；后续内部目标、PID 积分和滤波状态由 ctrl_gim_Task 独占。
 */
void app_sentry_gimbal_init(void)
{
    memset(&app_sentry_gimbal_debug, 0, sizeof(app_sentry_gimbal_debug));
    pid_init_all();
}

/**
 * @brief  完成一次 1 ms 云台计算，并给出发射命令和底盘转发命令。
 * @param  input        本周期完整私有快照：DBUS、IMU、CAN 反馈、协议状态及本周期更新掩码；不可为 NULL。
 * @param  output       输出：大 yaw/小 yaw/pitch 电流、目标与反馈、发射命令及底盘命令；不可为 NULL。
 * @note   由 ctrl_gim_Task 按 osDelayUntil() 调用；函数内部不等待 DMA、不操作 RTOS 对象，也不发送 CAN。
 * @note   算法步长来自 APP_CONTROL_GIM_DT_S；输入地址只在本次调用中借用，返回前清除引用。
 * @note   output->shoot.ready 表示首次反馈目标对齐完成；CAN 编码和快照发布由任务层处理。
 */
void app_sentry_gimbal_ctrl_1khz(const app_gimbal_input_t *input, app_gimbal_output_t *output)
{
    s_input = input;
    s_imu_state.instance = input->imu.valid ? &input->imu : NULL;
    memcpy(s_motor_state.feedback, input->feedback.motor, sizeof(s_motor_state.feedback));
    s_chassis_state.vw_cur_speed = input->feedback.omega;
    s_chassis_state.vw_rx_tick_ms = input->feedback.omega_tick;
    s_chassis_state.vw_is_valid = input->feedback.omega_valid;
    gimbal_compute();
    memset(output, 0, sizeof(*output));
    memcpy(output->current, s_motor_state.current, sizeof(output->current));
    output->yaw = s_imu_state.yaw_cur_angle;
    output->pitch = s_imu_state.pitch_cur_angle;
    output->yaw_target = s_command_state.yaw_tar_angle;
    output->pitch_target = s_command_state.pitch_tar_angle;
    output->yaw_rate = s_imu_state.yaw_gyro;
    output->pitch_rate = s_motor_state.pitch_cur_speed;
    output->shoot.source = s_chassis_state.input_source;
    output->shoot.fire = s_command_state.data.fire;
    output->shoot.ready = s_control_state.is_ready;
    output->shoot.emergency_stop = s_chassis_state.emergency_stop;
    output->shoot.tick = input->tick;
    output->chassis.vx =
        s_chassis_state.emergency_stop ? SENTRY_CHASSIS_ESTOP : s_chassis_state.vx_tar_speed;
    output->chassis.vy = s_chassis_state.vy_tar_speed;
    output->chassis.vz = s_chassis_state.vw_tar_speed;
    if (output->shoot.source == APP_GIMBAL_INPUT_CAN)
    {
        output->chassis.vx = input->can.radar_speed.vx;
        output->chassis.vy = input->can.radar_speed.vy;
        output->chassis.vz = input->can.radar_speed.vz;
        output->chassis.power_pct = input->can.radar_speed.power_pct;
    }
    /* 输入仅在本周期调用期间借用，返回前清除引用，避免保存调用者临时对象的地址。 */
    s_imu_state.instance = NULL;
    s_input = NULL;
}
