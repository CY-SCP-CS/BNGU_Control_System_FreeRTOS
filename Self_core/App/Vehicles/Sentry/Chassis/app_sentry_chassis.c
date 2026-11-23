/**
 * @file    app_sentry_chassis.c
 * @brief   哨兵双舵轮底盘控制实现
 * @note    控制流程：底盘目标速度 → 逆运动学 → 正运动学 →
 *           驱动前馈与轮级 PID → 功率缩放 → CAN2 电流输出。
 */
#include "App/Vehicles/Sentry/Config/sentry_control_config.h"
#include "App/Vehicles/Sentry/Config/app_sentry_common.h"
#include "App/Vehicles/Sentry/Contracts/app_motor_role.h"
#include "app_sentry_chassis.h"
#include "legacy_control_types.h"
#include "legacy_motor_limits.h"

#include "lib_filter.h"
#include "lib_math.h"

#include <math.h>
#include <string.h>

#define SENTRY_CHASSIS_OMEGA_TX_LPF_CUTOFF_HZ 20.0f

/** 底盘电机在反馈表和电流帧中的索引。 */

/** 电机反馈快照、CAN 接收缓冲、电流输出和驱动前馈。 */
typedef struct
{
    legacy_motor_t feedback[CHASSIS_MOTOR_COUNT]; // 控制循环使用的稳定反馈快照。
    int16_t current[CHASSIS_MOTOR_COUNT];         // 本周期电流输出。
    int16_t cur_speed[CHASSIS_MOTOR_COUNT];       // 电机当前转速，RPM。
    int16_t drive_current_ff[2];                  // 两个驱动轮的电流前馈。
} app_sentry_chassis_motor_state_t;

/** 底盘目标/当前状态和动力学中间量。 */
typedef struct
{
    legacy_sentry_swerve_wheel_t wheel_cur[2]; // 当前舵轮状态。
    legacy_sentry_swerve_wheel_t wheel_tar[2]; // 目标舵轮状态。
    legacy_sentry_chassis_speed_t cmd_speed;
    legacy_sentry_chassis_speed_t tar_speed; // 车体目标速度。
    legacy_sentry_chassis_speed_t cur_speed; // 正运动学估算的车体速度。
    lib_lpf_t omega_tx_lpf;                  // CAN1 角速度反馈低通滤波器。
} app_sentry_chassis_motion_state_t;

/** 功率计反馈、滤波状态和电流缩放系数。 */
typedef struct
{
    legacy_power_t rx;     // 功率计 CAN 接收缓冲。
    lib_lpf_t measure_lpf; // 实测功率低通滤波器。
    float measured_power;  // 滤波后的实测功率，W。
    float tar_power;       // 功率控制目标，W。
    float scale;           // 电机电流缩放系数，范围 0~1。
    float battery_voltage; // 电池电压，V。
    float battery_current; // 电池电流，A。
} app_sentry_chassis_power_state_t;

/** 底盘各级 PID 状态。 */
typedef struct
{
    lib_pid_t steer[2]; // 两个转向角度环。
    lib_pid_t drive[2]; // 两个驱动速度环。
} app_sentry_chassis_control_state_t;

/**
 * @brief 底盘全局调试快照。
 *
 * 任务私有状态可通过调试器查看；不在头文件声明，
 * 不作为模块间访问接口。
 */
typedef struct
{
    app_sentry_chassis_motor_state_t motor;
    app_sentry_chassis_motion_state_t motion;
    app_sentry_chassis_power_state_t power;
    app_sentry_chassis_control_state_t pid;
    legacy_sentry_chassis_state_t state;
} app_sentry_chassis_debug_t;

static app_sentry_chassis_debug_t app_sentry_chassis_debug = {
    .power =
        {
            .tar_power = SENTRY_CHASSIS_TAR_POWER,
            .scale = 1.0f,
        },
};

#define s_motor_state app_sentry_chassis_debug.motor
#define s_motion_state app_sentry_chassis_debug.motion
#define s_power_state app_sentry_chassis_debug.power
#define s_control_state app_sentry_chassis_debug.pid
#define s_chassis_state app_sentry_chassis_debug.state

static const uint8_t s_drive_index[2] = {M_DRIVE_L, M_DRIVE_R};
static const float s_drive_direction[2] = {-1.0f, 1.0f};
static const float s_steer_direction[2] = {
    SENTRY_STEER_L_DIRECTION,
    SENTRY_STEER_R_DIRECTION,
};

/**
 * @brief  初始化车体、轮级和功率控制 PID，参数沿用当前底盘配置。
 */
static void pid_init_all(void)
{

    lib_pid_init(&s_control_state.steer[0], 20000.0f, 20.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX, 1000.0f);
    lib_pid_init(&s_control_state.steer[1], 20000.0f, 20.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX, 1000.0f);
    lib_pid_init(&s_control_state.drive[0], 20.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                 DRV_MOTOR_M3508_CURRENT_MAX, 0.0f, DRV_MOTOR_M3508_CURRENT_MIN,
                 DRV_MOTOR_M3508_CURRENT_MAX, 0.0f);
    lib_pid_init(&s_control_state.drive[1], 20.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                 DRV_MOTOR_M3508_CURRENT_MAX, 0.0f, DRV_MOTOR_M3508_CURRENT_MIN,
                 DRV_MOTOR_M3508_CURRENT_MAX, 0.0f);
}

static void chassis_ramp_update(float dt);
static void chassis_reset(void);
static void inverse_kinematics(const legacy_sentry_chassis_speed_t *tar_speed);
static void forward_kinematics(void);
static void wheel_control(void);
static void power_update(void);
static void power_apply_limit(void);
static void power_publish(void);

/**
 * @brief  初始化底盘算法状态、PID、功率和角速度滤波器。
 * @note   由初始化任务调用一次，控制开始后状态由 ctrl_chas_Task 独占。
 * @note   功率目标沿用 SENTRY_CHASSIS_TAR_POWER=100 W，不在此处启动硬件。
 */
void app_sentry_chassis_init(void)
{
    memset(&app_sentry_chassis_debug, 0, sizeof(app_sentry_chassis_debug));
    s_power_state.tar_power = SENTRY_CHASSIS_TAR_POWER;
    s_power_state.scale = 1;
    lib_lpf_init(&s_power_state.measure_lpf, 35.0f);
    lib_lpf_init(&s_motion_state.omega_tx_lpf, SENTRY_CHASSIS_OMEGA_TX_LPF_CUTOFF_HZ);
    pid_init_all();
}

/**
 * @brief  读取稳定反馈并执行一次目标限幅、舵轮运动学、PID 和功率缩放。
 * @param  input        本周期完整私有输入；只读取 0x111 速度命令和 CAN 反馈。
 * @note   vx 为急停保留值时清零输出并重置控制状态；裁判数据暂不参与新的功率/热量限制。
 */
static void chassis_compute(const legacy_chassis_input_t *input)
{
    legacy_chassis_speed_cmd_t can_cmd = input->can.speed;

    /* 输入已经由任务复制成完整快照；算法不访问中断缓冲或 RTOS 对象。 */

    memcpy(s_motor_state.feedback, input->feedback.motor, sizeof(s_motor_state.feedback));
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        s_motor_state.cur_speed[i] = s_motor_state.feedback[i].speed;
    }

    uint8_t is_emergency_stop = can_cmd.vx == SENTRY_CHASSIS_ESTOP;
    s_power_state.rx = input->feedback.power;
    power_update();

    if (is_emergency_stop)
    {
        chassis_reset();
        power_publish();

        return;
    }

    /* 将 0x111 命令解码为车体坐标系目标速度并限幅。 */
    s_motion_state.cmd_speed.vx_speed = lib_clamp(
        (float)can_cmd.vx, -SENTRY_CHASSIS_VX_TAR_SPEED_MAX, SENTRY_CHASSIS_VX_TAR_SPEED_MAX);
    s_motion_state.cmd_speed.vy_speed = lib_clamp(
        (float)can_cmd.vy, -SENTRY_CHASSIS_VY_TAR_SPEED_MAX, SENTRY_CHASSIS_VY_TAR_SPEED_MAX);
    s_motion_state.cmd_speed.vw_speed =
        lib_clamp((float)can_cmd.vz * LEGACY_OMEGA_RAD_S_PER_LSB, -SENTRY_CHASSIS_VW_TAR_SPEED_MAX,
                  SENTRY_CHASSIS_VW_TAR_SPEED_MAX);

    chassis_ramp_update(APP_CONTROL_CHAS_DT_S);
    /* 根据转向编码器和驱动转速更新两个舵轮当前状态。 */
    s_motion_state.wheel_cur[0].steer_angle = lib_rad_norm(
        SENTRY_SWERVE_L_CAL_ANGLE +
        SENTRY_STEER_L_DIRECTION * lib_enc13_relative_rad(s_motor_state.feedback[M_STEER_L].angle,
                                                          SENTRY_SWERVE_0_OFFSET));
    s_motion_state.wheel_cur[1].steer_angle = lib_rad_norm(
        SENTRY_SWERVE_R_CAL_ANGLE +
        SENTRY_STEER_R_DIRECTION * lib_enc13_relative_rad(s_motor_state.feedback[M_STEER_R].angle,
                                                          SENTRY_SWERVE_1_OFFSET));
    s_motion_state.wheel_cur[0].drive_speed = -lib_motor_rpm_to_mm_s(
        (float)s_motor_state.cur_speed[M_DRIVE_L], SENTRY_WHEEL_RADIUS, DRV_MOTOR_M3508_REDUCTION);
    s_motion_state.wheel_cur[1].drive_speed = lib_motor_rpm_to_mm_s(
        (float)s_motor_state.cur_speed[M_DRIVE_R], SENTRY_WHEEL_RADIUS, DRV_MOTOR_M3508_REDUCTION);
    s_motion_state.wheel_cur[0].drive_rev = 1;
    s_motion_state.wheel_cur[1].drive_rev = 1;

    inverse_kinematics(&s_motion_state.tar_speed);
    forward_kinematics();
    (void)lib_lpf_update(&s_motion_state.omega_tx_lpf, s_motion_state.cur_speed.vw_speed,
                         APP_CONTROL_CHAS_DT_S);
    memset(s_motor_state.drive_current_ff, 0, sizeof(s_motor_state.drive_current_ff));
    wheel_control();
    power_apply_limit();

    memcpy(s_chassis_state.wheel_cur, s_motion_state.wheel_cur, sizeof(s_motion_state.wheel_cur));
    s_chassis_state.cur_speed = s_motion_state.cur_speed;
    power_publish();
}

/**
 * @brief  清零电流和车体状态，复位角速度滤波器及全部底盘 PID。
 * @note   急停分支使用；只复位内部算法状态，任务层仍负责提交当前输出。
 */
static void chassis_reset(void)
{
    memset(s_motor_state.current, 0, sizeof(s_motor_state.current));
    memset(&s_motion_state.cmd_speed, 0, sizeof(s_motion_state.cmd_speed));
    memset(&s_motion_state.tar_speed, 0, sizeof(s_motion_state.tar_speed));
    memset(&s_motion_state.cur_speed, 0, sizeof(s_motion_state.cur_speed));
    lib_lpf_init(&s_motion_state.omega_tx_lpf, SENTRY_CHASSIS_OMEGA_TX_LPF_CUTOFF_HZ);
    memset(&s_chassis_state, 0, sizeof(s_chassis_state));
    s_power_state.scale = 0.0f;
    for (unsigned i = 0; i < 2; i++)
    {
        lib_pid_reset(&s_control_state.drive[i]);
        lib_pid_reset(&s_control_state.steer[i]);
    }
}

/**
 * @brief  把车体目标速度分解为左右两个舵轮的目标角度和驱动速度。
 * @param  tar_speed    车体速度，vx/vy 为 mm/s，vw 为 rad/s；必须有效。
 * @note   转向误差超过 90 度时翻转驱动方向，以减小舵轮旋转距离；近零速时保持当前舵角。
 */
static void inverse_kinematics(const legacy_sentry_chassis_speed_t *tar_speed)
{
    float vx_tar_speed = lib_clamp(tar_speed->vx_speed, -SENTRY_CHASSIS_VX_TAR_SPEED_MAX,
                                   SENTRY_CHASSIS_VX_TAR_SPEED_MAX);
    float vy_tar_speed = lib_clamp(tar_speed->vy_speed, -SENTRY_CHASSIS_VY_TAR_SPEED_MAX,
                                   SENTRY_CHASSIS_VY_TAR_SPEED_MAX);
    float vw_tar_speed = lib_clamp(tar_speed->vw_speed, -SENTRY_CHASSIS_VW_TAR_SPEED_MAX,
                                   SENTRY_CHASSIS_VW_TAR_SPEED_MAX);

    for (int i = 0; i < 2; i++)
    {
        float sign = (i == 0) ? 1.0f : -1.0f;
        float wheel_vx_tar_speed = vx_tar_speed - sign * vw_tar_speed * SENTRY_WHEEL_HALF_TRACK;
        float wheel_vy_tar_speed = vy_tar_speed + sign * vw_tar_speed * SENTRY_WHEEL_HALF_BASE;

        float wheel_tar_speed = sqrtf(wheel_vx_tar_speed * wheel_vx_tar_speed +
                                      wheel_vy_tar_speed * wheel_vy_tar_speed);
        if (wheel_tar_speed < 1.0f)
        {
            s_motion_state.wheel_tar[i].steer_angle = s_motion_state.wheel_cur[i].steer_angle;
            s_motion_state.wheel_tar[i].drive_speed = 0.0f;
            s_motion_state.wheel_tar[i].drive_rev = 1;
            continue;
        }
        float wheel_tar_angle = atan2f(wheel_vy_tar_speed, wheel_vx_tar_speed);

        float steer_angle_error =
            lib_get_shortest_path(wheel_tar_angle, s_motion_state.wheel_cur[i].steer_angle);

        if (fabsf(steer_angle_error) > LIB_PI * 0.5f)
        {
            s_motion_state.wheel_tar[i].drive_rev = -1;
            s_motion_state.wheel_tar[i].steer_angle =
                lib_rad_norm(wheel_tar_angle + (steer_angle_error > 0.0f ? -LIB_PI : LIB_PI));
        }
        else
        {
            s_motion_state.wheel_tar[i].drive_rev = 1;
            s_motion_state.wheel_tar[i].steer_angle = wheel_tar_angle;
        }
        s_motion_state.wheel_tar[i].drive_speed =
            wheel_tar_speed * (float)s_motion_state.wheel_tar[i].drive_rev;
    }
}

/**
 * @brief  由两轮当前舵角和驱动转速估算车体 vx、vy 与角速度。
 * @note   使用减速比、轮半径及左右驱动符号；结果保存在 s_motion_state.cur_speed。
 */
static void forward_kinematics(void)
{
    float wheel_vx_cur_speed[2], wheel_vy_cur_speed[2];
    for (int i = 0; i < 2; i++)
    {
        float wheel_cur_speed =
            lib_motor_rpm_to_mm_s((float)s_motor_state.cur_speed[s_drive_index[i]],
                                  SENTRY_WHEEL_RADIUS, DRV_MOTOR_M3508_REDUCTION) *
            s_drive_direction[i];
        float wheel_cur_angle = s_motion_state.wheel_cur[i].steer_angle;
        wheel_vx_cur_speed[i] = wheel_cur_speed * cosf(wheel_cur_angle);
        wheel_vy_cur_speed[i] = wheel_cur_speed * sinf(wheel_cur_angle);
    }
    float half_track = SENTRY_WHEEL_HALF_TRACK;
    float half_base = SENTRY_WHEEL_HALF_BASE;
    s_motion_state.cur_speed.vx_speed = (wheel_vx_cur_speed[0] + wheel_vx_cur_speed[1]) * 0.5f;
    s_motion_state.cur_speed.vy_speed = (wheel_vy_cur_speed[0] + wheel_vy_cur_speed[1]) * 0.5f;
    s_motion_state.cur_speed.vw_speed =
        (half_track * (wheel_vx_cur_speed[1] - wheel_vx_cur_speed[0]) +
         half_base * (wheel_vy_cur_speed[0] - wheel_vy_cur_speed[1])) /
        (2.0f * (half_track * half_track + half_base * half_base));
}

/**
 * @brief  用车体速度误差计算合力方向、合力大小及绕 z 轴力矩。
 * @note   三个 PID 使用 APP_CONTROL_CHAS_DT_S 步长，输出供后续轮级分配使用。
 */

/**
 * @brief  把车体合力和力矩投影到左右轮，形成驱动电流前馈。
 * @note   沿用当前底盘分配关系、轮半径及减速比；电流按 M3508 协议输出范围限幅。
 */

/**
 * @brief  计算两个驱动速度环及两个转向位置环的电流。
 * @note   内部轮序为左、右，通过 s_drive_index 映射到输出电机索引；电流加入前馈后按电机范围限幅。
 */
static void wheel_control(void)
{
    int i;
    for (i = 0; i < 2; i++)
    {
        uint8_t drive_index = s_drive_index[i];
        float drive_tar_speed = s_motion_state.wheel_tar[i].drive_speed * s_drive_direction[i];
        float drive_cur_speed =
            lib_motor_rpm_to_mm_s((float)s_motor_state.cur_speed[drive_index], SENTRY_WHEEL_RADIUS,
                                  DRV_MOTOR_M3508_REDUCTION);
        float drive_current_ff = (float)s_motor_state.drive_current_ff[i] * s_drive_direction[i];
        float drive_current =
            lib_pid_ff_calc(&s_control_state.drive[i], drive_tar_speed, drive_cur_speed,
                            drive_current_ff, 0.0f, APP_CONTROL_CHAS_DT_S);
        drive_current =
            lib_clamp(drive_current, DRV_MOTOR_M3508_CURRENT_MIN, DRV_MOTOR_M3508_CURRENT_MAX);
        s_motor_state.current[drive_index] = (int16_t)drive_current;

        float steer_target = s_motion_state.wheel_tar[i].steer_angle;
        float steer_measure = s_motion_state.wheel_cur[i].steer_angle;
        float steer_speed = (float)s_motor_state.cur_speed[M_STEER_L + i];

        float steer_current =
            lib_pid_pos_calc(&s_control_state.steer[i], steer_target, steer_measure, 0.0f, 0.0f,
                             steer_speed, APP_CONTROL_CHAS_DT_S);
        steer_current *= s_steer_direction[i];
        steer_current =
            lib_clamp(steer_current, DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX);
        s_motor_state.current[M_STEER_L + i] = (int16_t)steer_current;
    }
}

/**
 * @brief  读取本周期功率计副本并更新功率低通滤波、电压和电流。
 * @note   功率为 W，原始 bat_v/bat_i 除以 100 后得到 V/A；负的滤波功率限制为 0。
 */
static void power_update(void)
{
    /* 任务已取得完整输入；算法读取自己持有的功率计副本。 */
    const legacy_power_t measured_power = s_power_state.rx;
    s_power_state.measure_lpf.out =
        lib_lpf_update(&s_power_state.measure_lpf, measured_power.power, APP_CONTROL_CHAS_DT_S);
    s_power_state.measured_power = fmaxf(s_power_state.measure_lpf.out, 0.0f);
    s_power_state.battery_voltage = measured_power.bat_v / 100.0f;
    s_power_state.battery_current = measured_power.bat_i / 100.0f;
}

/**
 * @brief  根据固定功率目标与实测功率计算保守缩放系数，并统一缩放四路电流。
 * @note   系数限制在 [0,1]；目标功率不大于 0 时清零系数并清零输出。
 */
static void power_apply_limit(void)
{
    if (s_power_state.tar_power <= 0.0f)
    {
        s_power_state.scale = 0.0f;
    }
    else
    {
        float demand = s_power_state.measured_power > s_power_state.tar_power
                           ? sqrtf(s_power_state.tar_power / s_power_state.measured_power)
                           : 1.0f;
        s_power_state.scale =
            demand < s_power_state.scale ? demand : fminf(demand, s_power_state.scale + .001f);
        s_power_state.scale = lib_clamp(s_power_state.scale, 0, 1);
    }

    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        s_motor_state.current[i] = (int16_t)(s_motor_state.current[i] * s_power_state.scale);
    }
}

/**
 * @brief  把当前功率、目标功率、缩放系数和电池信息复制到底盘输出状态。
 */
static void power_publish(void)
{
    s_chassis_state.cur_power = s_power_state.measured_power;
    s_chassis_state.tar_power = s_power_state.tar_power;
    s_chassis_state.power_scale = s_power_state.scale;
    s_chassis_state.battery_voltage = s_power_state.battery_voltage;
    s_chassis_state.battery_current = s_power_state.battery_current;
}

/**
 * @brief  完成一次 1 ms 双舵轮计算并输出本周期结果。
 * @param  input        完整底盘指令及 CAN 反馈副本；不可为 NULL。
 * @param  output       输出：右/左驱动、左/右转向电流，底盘状态、目标速度和反馈发送标记；不可为
 * NULL。
 * @note   由 ctrl_chas_Task 周期调用，步长来自 APP_CONTROL_CHAS_DT_S，不调用 HAL 或 RTOS。
 * @note   板间反馈分频由任务层维护；输入过期由 SI 适配入口转换为急停并复位 PID。
 */
static void legacy_app_sentry_chassis_ctrl(const legacy_chassis_input_t *input,
                                    legacy_chassis_output_t *output)
{
    s_power_state.tar_power =
        SENTRY_CHASSIS_TAR_POWER * lib_clamp(input->can.speed.power_pct * .0001f, 0, 1);
    chassis_compute(input);
    memcpy(output->current, s_motor_state.current, sizeof(output->current));
    output->state = s_chassis_state;
    output->target = s_motion_state.tar_speed;
    output->omega = s_motion_state.omega_tx_lpf.out;
}

static void chassis_ramp_update(float dt)
{
#if SENTRY_CHASSIS_RAMP_ENABLE
    float dx = s_motion_state.cmd_speed.vx_speed - s_motion_state.tar_speed.vx_speed;
    float dy = s_motion_state.cmd_speed.vy_speed - s_motion_state.tar_speed.vy_speed;
    float distance = sqrtf(dx * dx + dy * dy);
    float max_step = SENTRY_CHASSIS_LINEAR_ACCEL_MM_S2 * dt;

    /* x/y 共用比例，斜向变化率不会增大到 sqrt(2) 倍。 */
    if (distance > max_step)
    {
        float scale = max_step / distance;
        s_motion_state.tar_speed.vx_speed += dx * scale;
        s_motion_state.tar_speed.vy_speed += dy * scale;
    }
    else
    {
        s_motion_state.tar_speed.vx_speed = s_motion_state.cmd_speed.vx_speed;
        s_motion_state.tar_speed.vy_speed = s_motion_state.cmd_speed.vy_speed;
    }
    float dw = s_motion_state.cmd_speed.vw_speed - s_motion_state.tar_speed.vw_speed;
    float max_angular_step = SENTRY_CHASSIS_ANGULAR_ACCEL_RAD_S2 * dt;
    s_motion_state.tar_speed.vw_speed += lib_clamp(dw, -max_angular_step, max_angular_step);
#else
    (void)dt;
    s_motion_state.tar_speed = s_motion_state.cmd_speed;
#endif
}

/* Public SI entry and conversion to the retained control kernel. */
/** @file app_sentry_chassis_adapter.c
 * @brief SI input policy and legacy scale conversion. Private scratch storage is owned
 *        by the sole control task; this API is single-instance and non-reentrant.
 */
#include "app_sentry_chassis.h"
#include "App/Vehicles/Sentry/Config/app_sentry_common.h"
#include "legacy_adapter.h"
#include "app_data_age.h"

void app_sentry_chassis_ctrl(const app_chassis_input_t *input, app_chassis_output_t *output)
{
    static legacy_chassis_input_t s_legacy_input;
    static legacy_chassis_output_t s_legacy_output;
    legacy_chassis_input_t *v = &s_legacy_input;
    legacy_feedback(&input->feedback, &v->feedback);
    v->can.speed = (legacy_chassis_speed_cmd_t){
        lib_quantize_i16(input->can.speed.vx, .001f), lib_quantize_i16(input->can.speed.vy, .001f),
        lib_quantize_i16(input->can.speed.vz, .001f),
        lib_quantize_i16(input->can.speed.power_pct, .0001f)};
    if (v->can.speed.vx == SENTRY_CHASSIS_ESTOP)
    {
        v->can.speed.vx = SENTRY_CHASSIS_ESTOP + 1;
    }
    v->can.ackermann = (legacy_chassis_ackermann_cmd_t){lib_m_to_mm(input->can.ackermann.speed),
                                                        input->can.ackermann.steer_angle};
    v->can.follow =
        (legacy_chassis_follow_cmd_t){lib_quantize_i16(input->can.follow.vx, .001f),
                                      lib_quantize_i16(input->can.follow.vy, .001f),
                                      lib_quantize_i16(input->can.follow.gimbal_angle, .001f),
                                      lib_quantize_i16(input->can.follow.wz, .001f)};
    if (!isfinite(input->can.speed.power_pct) || input->can.speed.power_pct <= 0 ||
        input->can.speed.power_pct > 1 ||
        !app_data_fresh(input->command_valid, input->command_tick, input->tick,
                        APP_INPUT_CAN_MAX_AGE_MS) ||
        !app_motor_feedback_fresh(&input->feedback, 0, 4, input->tick) ||
        !app_data_fresh(input->feedback.power_valid, input->feedback.power_tick, input->tick,
                        APP_INPUT_POWER_MAX_AGE_MS))
    {
        v->can.speed.vx = SENTRY_CHASSIS_ESTOP;
    }
    legacy_app_sentry_chassis_ctrl(v, &s_legacy_output);
    const legacy_chassis_output_t *o = &s_legacy_output;
    memcpy(output->current, o->current, sizeof(output->current));
    for (unsigned i = 0; i < 2; i++)
    {
        output->state.wheel_cur[i] = (app_swerve_wheel_state_t){
            o->state.wheel_cur[i].steer_angle, lib_mm_to_m(o->state.wheel_cur[i].drive_speed),
            o->state.wheel_cur[i].drive_rev};
    }
    output->state.cur_speed = (app_chassis_velocity_t){lib_mm_to_m(o->state.cur_speed.vx_speed),
                                                       lib_mm_to_m(o->state.cur_speed.vy_speed),
                                                       o->state.cur_speed.vw_speed};
    output->state.cur_power = o->state.cur_power;
    output->state.tar_power = o->state.tar_power;
    output->state.power_scale = o->state.power_scale;
    output->state.battery_voltage = o->state.battery_voltage;
    output->state.battery_current = o->state.battery_current;
    output->target = (app_chassis_velocity_t){lib_mm_to_m(o->target.vx_speed),
                                              lib_mm_to_m(o->target.vy_speed), o->target.vw_speed};
    output->omega = o->omega;
}
