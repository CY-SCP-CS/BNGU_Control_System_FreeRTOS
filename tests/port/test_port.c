#include "app_gimbal_command_merge.h"
/** @file test_port.c
 * @brief 裸机功能迁移回归：输入映射、控制单位、协议及导航仲裁。
 */
#include <assert.h>
#include <stdlib.h>
#undef assert
#define assert(x)                                                                                  \
    do                                                                                             \
    {                                                                                              \
        if (!(x))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #x);                                   \
            exit(1);                                                                               \
        }                                                                                          \
    } while (0)
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "project_cfg.h"
#include "app_control.h"
#include "app_sentry_gimbal.h"
#include "app_sentry_shoot.h"
#include "App/Vehicles/Sentry/Config/app_sentry_common.h"
#include "app_vehicle_can.h"
#include "navigation_ctrl.h"
#include "drv_motor.h"
#include "drv_referee.h"
#include "drv_vofa.h"
#include "lib_pid.h"
#include "app_sentry_chassis.h"
#include "app_data_age.h"
static void recent(app_can_feedback_t *f, uint32_t tick)
{
    for (unsigned i = 0; i < 6; i++)
    {
        f->motor_valid[i] = 1;
        f->motor_tick[i] = tick;
    }
    f->power_valid = 1;
    f->power_tick = tick;
}

static void near(float a, float b, float tolerance)
{
    if (!(isfinite(a) && fabsf(a - b) <= tolerance))
    {
        fprintf(stderr, "near: %g != %g tol %g\n", a, b, tolerance);
        exit(1);
    }
}
static void test_input(void)
{
    drv_dbus_data_t d = {0};
    app_remote_input_t o;
    for (unsigned i = 0; i < 4; i++)
    {
        d.rc.ch[i] = 1024;
    }
    d.rc.rolling_wheel = 1024;
    d.rc.s1 = 1;
    d.rc.s2 = 1;
    assert(app_dbus_map_input(&d, &o));
    assert(o.source == APP_REMOTE_INPUT_DBUS);
    d.rc.s2 = 3;
    assert(app_dbus_map_input(&d, &o));
    assert(o.source == APP_REMOTE_INPUT_CAN);
    d.rc.s2 = 2;
    assert(app_dbus_map_input(&d, &o));
    assert(o.source == APP_REMOTE_INPUT_ESTOP);
}
static void test_gimbal(void)
{
    app_gimbal_input_t in = {0};
    app_gimbal_output_t out = {0};
    recent(&in.feedback, 0);
    in.imu.valid = in.imu.sampled = in.imu.device_valid = in.imu.filter_ready = 1;
    in.dbus.valid = 1;
    in.dbus.input.source = APP_REMOTE_INPUT_DBUS;
    in.dbus.input.launcher_mode = APP_SENTRY_FIRE_OFF;
    in.feedback.motor[0].angle_rad = SENTRY_GIMBAL_L_YAW_ZERO * LIB_RAD_PER_ENC13;
    in.feedback.motor[1].angle_rad = SENTRY_GIMBAL_S_YAW_ZERO * LIB_RAD_PER_ENC13;
    in.feedback.motor[2].angle_rad = SENTRY_GIMBAL_PITCH_ZERO * LIB_RAD_PER_ENC13;
    app_sentry_gimbal_init();
    in.tick = in.imu.tick = in.dbus.tick = 0;
    recent(&in.feedback, 0);
    app_sentry_gimbal_ctrl_1khz(&in, &out);
    assert(!out.shoot.ready);
    in.tick = in.imu.tick = in.dbus.tick = 20;
    recent(&in.feedback, 20);
    app_sentry_gimbal_ctrl_1khz(&in, &out);
    near(out.quat.q0, 1, 1e-5f);
    near(out.pitch, 0, 1e-5f);
    in.dbus.input.chassis_vx_norm = .25f;
    in.dbus.input.chassis_vy_norm = .5f;
    app_sentry_gimbal_ctrl_1khz(&in, &out);
    near(out.chassis.vx, -SENTRY_CHASSIS_VX_TAR_SPEED_MAX * .25f * .001f, .0011f);
    near(out.chassis.vy, SENTRY_CHASSIS_VY_TAR_SPEED_MAX * .5f * .001f, .0011f);
    in.can_tick[0] = in.tick;
    in.can_valid_mask = APP_GIMBAL_COMM_UPDATE_RADAR_SPEED;
    in.dbus.input.source = APP_REMOTE_INPUT_CAN;
    in.can.radar_speed.vx = 1.123f;
    in.can.radar_speed.vy = -.456f;
    app_sentry_gimbal_ctrl_1khz(&in, &out);
    near(out.chassis.vx, -1.123f, .0011f);
    near(out.chassis.vy, -.456f, .0011f);
    in.imu.euler.yaw = .3f;
    in.imu.euler.roll = .1f;
    in.imu.euler.pitch = -.2f;
    in.feedback.motor[2].angle_rad = (SENTRY_GIMBAL_PITCH_ZERO + 100) * LIB_RAD_PER_ENC13;
    app_sentry_gimbal_ctrl_1khz(&in, &out);
    near(out.yaw, .3f, 1e-5f);
    near(out.pitch, .1f + 100 * LIB_RAD_PER_ENC13, 1e-5f);
    near(out.roll, .2f, 1e-5f);
    float norm = out.quat.q0 * out.quat.q0 + out.quat.q1 * out.quat.q1 + out.quat.q2 * out.quat.q2 +
                 out.quat.q3 * out.quat.q3;
    near(norm, 1, 1e-5f);
    in.dbus.input.source = APP_REMOTE_INPUT_ESTOP;
    app_sentry_gimbal_ctrl_1khz(&in, &out);
    assert(out.current[0] == 0 && out.current[1] == 0 && out.current[2] == 0 &&
           out.shoot.emergency_stop);
}
static void test_shoot(void)
{
    app_shoot_input_t in = {0};
    app_shoot_output_t out;
    recent(&in.feedback, 0);
    in.command.ready = 1;
    in.command.fire = APP_SENTRY_FIRE_ON;
    app_sentry_shoot_init();
    app_sentry_launcher_ctrl_200hz(&in, &out);
    assert(out.current[0] > 0 && out.current[1] > 0 && out.current[2] < 0);
    lib_pid_t pid;
    lib_pid_init(&pid, 85, 150, 0, 1, 0, 3000, 0, -10000, 10000, 7000);
    assert(out.current[1] ==
           (int16_t)lib_pid_ff_calc(&pid, SENTRY_DISC_TAR_SPEED, 0, 3000, 0, .005f));
    near(out.disc_integral, pid.ki * pid.integral, .01f);
    in.command.fire = APP_SENTRY_FIRE_FIR;
    app_sentry_launcher_ctrl_200hz(&in, &out);
    assert(out.current[1] == 0);
    in.command.fire = APP_SENTRY_FIRE_REVERSE;
    app_sentry_launcher_ctrl_200hz(&in, &out);
    assert(out.current[1] < 0);
    in.command.emergency_stop = 1;
    app_sentry_launcher_ctrl_200hz(&in, &out);
    assert(out.current[0] == 0 && out.current[1] == 0 && out.current[2] == 0);
    near(out.disc_integral, 0, 0);
}
static void test_wire(void)
{
    app_ctrl_can_msg_t m = {.type = APP_CAN_GIM_FEEDBACK};
    app_can_frame_t f[APP_VEHICLE_CAN_MAX_FRAMES];
    m.value.gimbal.yaw = LIB_RAD_PER_DEG * 90;
    m.value.gimbal.pitch = LIB_RAD_PER_DEG * -30;
    m.value.gimbal.quat.q0 = 1;
#if CURRENT_ROBOT == ROBOT_SENTRY && CURRENT_BOARD == BOARD_GIMBAL
    assert(app_vehicle_can_encode(&m, f) == 4);
    assert(f[0].std_id == 0x122 && f[1].std_id == 0x124 && f[2].std_id == 0x233 &&
           f[3].std_id == 0x139);
    float x, y;
    memcpy(&x, f[3].data, 4);
    memcpy(&y, f[3].data + 4, 4);
    near(x, 90, .0001f);
    near(y, -30, .0001f);
    assert(f[3].bus == 1);
    memcpy(&x, f[1].data, 4);
    near(x, LIB_RAD_PER_DEG * 90, .0001f);
#else
    assert(app_vehicle_can_encode(&m, f) == 0);
#endif
    float data[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    uint8_t frame[DRV_VOFA_FRAME_SIZE];
    assert(sizeof(frame) == 40);
    assert(drv_vofa_pack(data, frame) == DRV_VOFA_OK);
    assert(memcmp(data, frame, 36) == 0);
    const uint8_t tail[] = {0, 0, 0x80, 0x7f};
    assert(memcmp(frame + 36, tail, 4) == 0);
#if CURRENT_ROBOT == ROBOT_NAVIGATION
    m.type = APP_CAN_CHAS_CURRENT;
    for (unsigned i = 0; i < 4; i++)
    {
        m.value.current[i] = 100 + i;
    }
    assert(app_vehicle_can_encode(&m, f) == 1);
    assert(f[0].std_id == 0x200 && f[0].data[7] == 103);
#endif
    uint8_t command[8];
    memset(command, 0x55, 8);
    assert(!drv_motor_dm_command_frame(command, 0));
    assert(command[0] == 0x55);
    assert(drv_motor_dm_command_frame(command, DRV_MOTOR_DM_CMD_ENABLE));
    for (unsigned i = 0; i < 7; i++)
    {
        assert(command[i] == 255);
    }
    assert(command[7] == 0xfc);
    uint8_t packet[128];
    drv_referee_context_t parser;
    drv_referee_init(&parser);
    const uint8_t payload[] = {1, 2, 3};
    uint16_t n = drv_referee_build_frame(packet, sizeof(packet), 0x7777, payload, 3, 7);
    assert(n == 12);
    (void)drv_referee_process(&parser, packet, 4);
    (void)drv_referee_process(&parser, packet + 4, n - 4);
    assert(parser.s_frames == 1);
}
static void test_navigation(void)
{
    app_navigation_input_t in = {0};
    app_dbus_snapshot_t dbus = {0};
    app_imu_snapshot_t imu = {0};
    app_chassis_output_t out;
    recent(&in.chassis.feedback, 100);
    in.chassis.command_valid = 1;
    in.chassis.command_tick = 100;
    dbus.tick = 100;
    navigation_ctrl_init();
    dbus.valid = 1;
    dbus.switch_s1 = 3;
    dbus.switch_s2 = 3;
    dbus.channel_norm[3] = .2f;
    navigation_ctrl_step(&in, &dbus, &imu, 100, &out);
    near(out.target.vx_speed, 1.2f, 1e-5f);
    assert(out.current[0] < 0 && out.current[1] > 0 && out.current[2] < 0 && out.current[3] > 0);
    dbus.switch_s2 = 1;
    navigation_ctrl_step(&in, &dbus, &imu, 100, &out);
    for (unsigned i = 0; i < 4; i++)
    {
        assert(out.current[i] == 0);
    }
    imu.valid = 1;
    imu.sampled = imu.device_valid = imu.filter_ready = 1;
    imu.tick = 100;
    imu.euler.yaw = LIB_RAD_PER_DEG * 90;
    navigation_ctrl_step(&in, &dbus, &imu, 100, &out);
    assert(out.current[0] > 0 && out.current[1] < 0 && out.current[2] < 0 && out.current[3] > 0);
    navigation_ctrl_step(&in, &dbus, &imu, 121, &out);
    for (unsigned i = 0; i < 4; i++)
    {
        assert(out.current[i] == 0);
    }
    dbus.switch_s1 = 1;
    navigation_ctrl_step(&in, &dbus, &imu, 121, &out);
    for (unsigned i = 0; i < 4; i++)
    {
        assert(out.current[i] == 0);
    }
    dbus.switch_s1 = 2;
    recent(&in.chassis.feedback, 121);
    in.chassis.can.speed.vx = .7f;
    navigation_ctrl_step(&in, &dbus, &imu, 121, &out);
    near(out.target.vx_speed, .7f, 1e-6f);
    dbus.valid = 0;
    navigation_ctrl_step(&in, &dbus, &imu, 121, &out);
    for (unsigned i = 0; i < 4; i++)
    {
        assert(out.current[i] == 0);
    }
}
static void test_freshness(void)
{
    assert(app_data_fresh(1, 0, 10, 10));
    assert(!app_data_fresh(1, 0, 11, 10));
    assert(!app_data_fresh(0, 0, 0, 10));
    assert(app_data_fresh(1, UINT32_MAX - 3U, 6U, 10U));
    assert(!app_data_fresh(1, UINT32_MAX - 3U, 7U, 10U));
    app_shoot_input_t s = {0};
    app_shoot_output_t so;
    s.tick = 120;
    s.command.tick = 100;
    s.command.ready = 1;
    s.command.fire = APP_SENTRY_FIRE_ON;
    recent(&s.feedback, 120);
    app_sentry_shoot_init();
    app_sentry_launcher_ctrl_200hz(&s, &so);
    assert(so.current[0] != 0);
    s.tick = 121;
    app_sentry_launcher_ctrl_200hz(&s, &so);
    assert(so.current[0] == 0 && so.current[1] == 0 && so.current[2] == 0);
    app_chassis_input_t c = {0};
    app_chassis_output_t co;
    c.tick = 200;
    c.command_tick = 100;
    c.command_valid = 1;
    c.can.speed.vx = .6f;
    recent(&c.feedback, 200);
    app_sentry_chassis_init();
    app_sentry_chassis_ctrl(&c, &co);
    near(co.target.vx_speed, .003f, .00001f);
    c.tick = 301;
    app_sentry_chassis_ctrl(&c, &co);
    assert(co.current[0] == 0 && co.current[1] == 0 && co.current[2] == 0 && co.current[3] == 0);
    near(co.target.vx_speed, 0, 0);
    c.command_tick = 301;
    recent(&c.feedback, 301);
    app_sentry_chassis_ctrl(&c, &co);
    near(co.target.vx_speed, .003f, .00001f);
    c.feedback.power_valid = 0;
    app_sentry_chassis_ctrl(&c, &co);
    near(co.target.vx_speed, 0, 0);
    /* Navigation also stops on radio/CAN/motor expiry. */
    app_navigation_input_t nav = {0};
    app_dbus_snapshot_t d = {0};
    app_imu_snapshot_t imu = {0};
    navigation_ctrl_init();
    d.valid = 1;
    d.tick = 100;
    d.switch_s1 = 3;
    d.switch_s2 = 3;
    d.channel_norm[3] = .2f;
    recent(&c.feedback, 201);
    nav.chassis = c;
    navigation_ctrl_step(&nav, &d, &imu, 201, &co);
    near(co.target.vx_speed, 0, 0);
    d.tick = 201;
    d.switch_s1 = 2;
    c.command_valid = 0;
    nav.chassis = c;
    navigation_ctrl_step(&nav, &d, &imu, 201, &co);
    near(co.target.vx_speed, 0, 0);
    c.command_valid = 1;
    c.command_tick = 201;
    c.feedback.motor_valid[0] = 0;
    nav.chassis = c;
    navigation_ctrl_step(&nav, &d, &imu, 201, &co);
    near(co.target.vx_speed, 0, 0);
}

#include "Drv/Drv_game_controller/drv_game_controller.h"
#include "app_can_output_policy.h"
static void gimbal_refresh(app_gimbal_input_t *in, uint32_t tick)
{
    in->tick = in->imu.tick = in->dbus.tick = tick;
    recent(&in->feedback, tick);
}
static void test_migrated_control(void)
{
    app_gimbal_input_t g = {0};
    app_gimbal_output_t o = {0};
    g.imu.valid = g.imu.sampled = g.imu.device_valid = g.imu.filter_ready = 1;
    g.dbus.valid = g.dbus.seen = 1;
    g.dbus.input.source = APP_REMOTE_INPUT_DBUS;
    g.dbus.input.launcher_mode = APP_SENTRY_FIRE_ON;
    g.feedback.motor[0].angle_rad = SENTRY_GIMBAL_L_YAW_ZERO * LIB_RAD_PER_ENC13;
    g.feedback.motor[1].angle_rad = SENTRY_GIMBAL_S_YAW_ZERO * LIB_RAD_PER_ENC13;
    g.feedback.motor[2].angle_rad = SENTRY_GIMBAL_PITCH_ZERO * LIB_RAD_PER_ENC13;
    g.imu.euler.roll = .3f;
    app_sentry_gimbal_init();
    for (unsigned t = 100; t <= 120; t++)
    {
        gimbal_refresh(&g, t);
        app_sentry_gimbal_ctrl_1khz(&g, &o);
        if (t < 120)
        {
            assert(!o.shoot.ready && o.current[0] == 0 && o.current[2] == 0);
        }
    }
    assert(o.shoot.ready);
    near(o.pitch_target, .3f, 1e-5f);
    gimbal_refresh(&g, 121);
    g.can_valid_mask = g.updated_mask = APP_GIMBAL_COMM_UPDATE_SHOOT;
    g.can_tick[5] = 121;
    app_sentry_gimbal_ctrl_1khz(&g, &o);
    assert(o.shoot.fire == APP_SENTRY_FIRE_ON);
    gimbal_refresh(&g, 122);
    g.updated_mask = 0;
    g.dbus.input.source = APP_REMOTE_INPUT_CAN;
    app_sentry_gimbal_ctrl_1khz(&g, &o);
    assert(o.shoot.fire == APP_SENTRY_FIRE_OFF);
    gimbal_refresh(&g, 123);
    g.updated_mask = APP_GIMBAL_COMM_UPDATE_SHOOT;
    g.can_tick[5] = 123;
    g.can.shoot.shoot_switch = 255;
    app_sentry_gimbal_ctrl_1khz(&g, &o);
    assert(o.shoot.fire == APP_SENTRY_FIRE_ON);
    g.imu.device_valid = 0;
    app_sentry_gimbal_ctrl_1khz(&g, &o);
    assert(!o.shoot.ready && o.shoot.emergency_stop);
    g.imu.device_valid = 1;
    g.updated_mask = 0;
    for (unsigned t = 124; t <= 144; t++)
    {
        gimbal_refresh(&g, t);
        app_sentry_gimbal_ctrl_1khz(&g, &o);
    }
    assert(o.shoot.ready);
    near(o.pitch_target, .3f, 1e-5f);
    /* No command allows idle scan, but never authorizes firing. */
    g.can_valid_mask = 0;
    for (unsigned t = 145; t <= 147; t++)
    {
        gimbal_refresh(&g, t);
        app_sentry_gimbal_ctrl_1khz(&g, &o);
    }
    assert(o.yaw_target > 0 && o.shoot.fire == APP_SENTRY_FIRE_OFF);
    gimbal_refresh(&g, 148);
    g.can_valid_mask = g.updated_mask = APP_GIMBAL_COMM_UPDATE_ANGLE_SHOOT;
    g.can_tick[4] = 148;
    g.can.angle_shoot.pitch_abs = .3f;
    app_sentry_gimbal_ctrl_1khz(&g, &o);
    assert(o.shoot.fire == APP_SENTRY_FIRE_ON);
    gimbal_refresh(&g, 149);
    g.can_valid_mask |= APP_GIMBAL_COMM_UPDATE_SHOOT;
    g.updated_mask = APP_GIMBAL_COMM_UPDATE_SHOOT;
    g.can_tick[5] = 149;
    g.can.shoot.shoot_switch = 0;
    app_sentry_gimbal_ctrl_1khz(&g, &o);
    assert(o.shoot.fire == APP_SENTRY_FIRE_OFF);
    gimbal_refresh(&g, 150);
    g.updated_mask = APP_GIMBAL_COMM_UPDATE_ANGLE_SHOOT;
    g.can_tick[4] = 150;
    app_sentry_gimbal_ctrl_1khz(&g, &o);
    assert(o.shoot.fire == APP_SENTRY_FIRE_OFF);
    /* Sensor expiry still requires reset, independent of fresh CAN commands. */
    g.tick = 161;
    app_sentry_gimbal_ctrl_1khz(&g, &o);
    assert(!o.shoot.ready);
    app_chassis_input_t c = {0};
    app_chassis_output_t co;
    c.tick = c.command_tick = 100;
    c.command_valid = 1;
    c.can.speed.vx = 1;
    recent(&c.feedback, 100);
    c.feedback.power.power_w = 300;
    app_sentry_chassis_init();
    app_sentry_chassis_ctrl(&c, &co);
    near(co.target.vx_speed, .003f, 1e-6f);
    assert(co.state.power_scale < .6f);
    c.command_valid = 0;
    app_sentry_chassis_ctrl(&c, &co);
    near(co.target.vx_speed, 0, 0);
    c.command_valid = 1;
    app_sentry_chassis_ctrl(&c, &co);
    near(co.target.vx_speed, .003f, 1e-6f);
    app_can_frame_t f = {.bus = 2, .std_id = 0x1fe, .tick = 100};
    memset(f.data, 255, 8);
    assert(app_can_frame_prepare(&f, 120) && f.data[0] == 255);
    assert(app_can_frame_prepare(&f, 121) && f.data[0] == 0);
    f.bus = 1;
    f.std_id = 0x111;
    assert(app_can_frame_prepare(&f, 201) && f.data[1] == 128);
    f.std_id = 0x124;
    assert(!app_can_frame_prepare(&f, 121));
    drv_game_controller_buttons_t pad = {0};
    assert(drv_game_controller_decode((const uint8_t *)"JOY,1,-1,0,.5", &pad));
    assert(!drv_game_controller_decode((const uint8_t *)"JOY,nan,0,0,0", &pad));
    assert(!drv_game_controller_decode((const uint8_t *)"JOY,0,0,0,0garbage", &pad));
    assert(!drv_game_controller_decode((const uint8_t *)"JOY,2,0,0,0", &pad));

    app_can_ctrl_msg_t delta = {.type = APP_CAN_CMD_SPEED_NO_SHOOT,
                                .updated_mask = APP_GIMBAL_COMM_UPDATE_SPEED_NO_SHOOT,
                                .tick = 100};
    delta.value.delta.yaw_inc = .1f;
    uint8_t radar = 0;
    app_gimbal_input_t merged = {0};
    app_gimbal_command_merge(&merged, &delta, &radar);
    app_gimbal_command_merge(&merged, &delta, &radar);
    near(merged.can.speed_no_shoot.yaw_inc, .2f, 1e-6f);
    app_dbus_snapshot_t radio = {0};
    app_imu_snapshot_t nav_imu = {0};
    radio.valid = 1;
    radio.tick = 100;
    radio.switch_s1 = 1;
    radio.switch_s2 = 3;
    app_navigation_input_t nav = {.chassis = c};
    nav.gamepad = (app_gamepad_snapshot_t){.ly = .5f, .tick = 100, .valid = 1};
    recent(&nav.chassis.feedback, 100);
    navigation_ctrl_init();
    navigation_ctrl_step(&nav, &radio, &nav_imu, 100, &co);
    near(co.target.vx_speed, -3, 1e-6f);
    radio.tick = 201;
    recent(&nav.chassis.feedback, 201);
    navigation_ctrl_step(&nav, &radio, &nav_imu, 201, &co);
    near(co.target.vx_speed, 0, 0);

    /* Exact 2027 referee payload sizes, interaction validation and context isolation. */
    drv_referee_context_t ctx;
    drv_referee_init(&ctx);
    uint8_t buf[160];
    drv_referee_robot_status_t status = {0};
    status.robot_id = 7;
    uint16_t n =
        drv_referee_build_frame(buf, sizeof(buf), CMD_ROBOT_STATUS, &status, sizeof(status), 0);
    assert(n == 26);
    drv_referee_parse_result_t result = drv_referee_process(&ctx, buf, n);
    assert(result.updated_mask & REFEREE_FLAG_ROBOT_STATUS);
    uint8_t payload[8] = {0};
    payload[0] = 0;
    payload[1] = 2; /* robot data subcommand 0x0200 */
    n = drv_referee_build_frame(buf, sizeof(buf), CMD_ROBOT_INTERACTION, payload, 8, 0);
    assert(n == 17);
    result = drv_referee_process(&ctx, buf, n);
    assert(result.updated_mask & REFEREE_FLAG_INTERACTION);
    payload[0] = 1;
    payload[1] = 1; /* one graphic requires 15 user bytes */
    assert(!drv_referee_build_frame(buf, sizeof(buf), CMD_ROBOT_INTERACTION, payload, 8, 0));
}

int main(void)
{
    test_migrated_control();
    test_input();
    test_gimbal();
    test_shoot();
    test_wire();
    test_navigation();
    test_freshness();
    puts("PASS: input, gimbal, launcher, CAN, VOFA, referee, DM and navigation");
    return 0;
}
