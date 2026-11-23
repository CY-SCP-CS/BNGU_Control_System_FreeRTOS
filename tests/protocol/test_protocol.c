#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "app_vehicle_config.h"
#include "app_vehicle_can.h"
#include "app_core_protocol.h"
#include "app_chassis_comm.h"
#include "app_gimbal_comm.h"
static void near(float a, float b, float e)
{
    assert(isfinite(a) && fabsf(a - b) <= e);
}
int main(void)
{
    app_chassis_comm_rx_t c = {0};
    app_gimbal_comm_rx_t g = {0};
    uint8_t mask = 0;
    const uint8_t speed[] = {0xE8, 3, 0x0C, 0xFE, 0xE8, 3, 0x10, 0x27};
    assert(app_chassis_comm_decode(0x111, speed, 8, &c, &mask));
    near(c.speed.vx, 1, .000001f);
    near(c.speed.vy, -.5, .000001f);
    near(c.speed.vz, 1, .000001f);
    near(c.speed.power_pct, 1, .000001f);
    assert(app_gimbal_comm_decode(0x120, speed, 8, &g, &mask));
    near(g.radar_speed.power_pct, 1, .000001f);
    const uint8_t ack[] = {0x10, 0x27, 0xE8, 3};
    assert(app_chassis_comm_decode(0x113, ack, 4, &c, &mask));
    near(c.ackermann.speed, 10, .000001f);
    near(c.ackermann.steer_angle, .1, .000001f);
    assert(!app_chassis_comm_decode(0x113, speed, 8, &c, &mask));
    const uint8_t follow[] = {0xE8, 3, 0x0C, 0xFE, 0xE8, 3, 0x18, 0xFC};
    assert(app_chassis_comm_decode(0x115, follow, 8, &c, &mask));
    near(c.follow.wz, 1, .000001f);
    near(c.follow.gimbal_angle, -.1, .000001f);
    const uint8_t rate[] = {0x64, 0, 0x38, 0xFF};
    assert(app_gimbal_comm_decode(0x121, rate, 4, &g, &mask));
    near(g.speed_no_shoot.yaw_rate, .1, .000001f);
    near(g.speed_no_shoot.pitch_rate, -.2, .000001f);
    assert(app_gimbal_comm_decode(0x125, rate, 4, &g, &mask));
    assert(!app_gimbal_comm_decode(0x121, speed, 8, &g, &mask));
    const uint8_t angle[] = {0x10, 0x27, 0x18, 0xFC};
    assert(app_gimbal_comm_decode(0x123, angle, 4, &g, &mask));
    near(g.angle_no_shoot.yaw_abs, 1, .000001f);
    near(g.angle_no_shoot.pitch_abs, -.1, .000001f);
    const uint8_t minus_one[] = {0xFF, 0xFF, 0, 0};
    assert(app_gimbal_comm_decode(0x127, minus_one, 4, &g, &mask));
    near(g.angle_shoot.yaw_abs, -.0001, .0000001f);
    near(g.angle_shoot.pitch_abs, 0, 0);
    for (unsigned friction = 0; friction < 2; friction++)
    {
        for (unsigned i = 0; i < 3; i++)
        {
            uint8_t b[] = {friction ? 255 : 0, i == 0 ? 0 : i == 1 ? 170 : 255};
            assert(app_gimbal_comm_decode(0x129, b, 2, &g, &mask));
            assert(g.shoot.friction_switch == b[0] && g.shoot.feed_state == b[1]);
        }
    }
    uint8_t bad[] = {1, 0};
    assert(!app_gimbal_comm_decode(0x129, bad, 2, &g, &mask));
    assert(!app_gimbal_comm_decode(0x129, speed, 8, &g, &mask));
    app_can_frame_t f = {
        .bus = 1, .std_id = 0x112, .dlc = 4, .tick = 7, .data = {0x30, 0x75, 0xF0, 0xD8}};
    app_can_feedback_t feedback = {0};
    app_can_ctrl_msg_t command;
    unsigned index;
    app_can_frame_t auxiliary = {.bus = 1, .std_id = 0x157, .dlc = 1, .tick = 12, .data = {107}};
    assert(app_vehicle_can_decode(&auxiliary, &feedback, &command, &index) == APP_VEHICLE_CAN_AUX);
    assert(feedback.core_referee.robot_id == 107 && feedback.core_referee.tick[3] == 12);
#if CURRENT_BOARD == BOARD_GIMBAL
    assert(app_vehicle_can_decode(&f, &feedback, &command, &index) == APP_VEHICLE_CAN_OMEGA);
    near(feedback.power.power_w, 300, .0001f);
    near(feedback.omega, -10, .00001f);
    f.std_id = 0x119;
    assert(app_vehicle_can_decode(&f, &feedback, &command, &index) == APP_VEHICLE_CAN_IGNORED);
    app_ctrl_can_msg_t m = {.type = APP_CAN_GIM_FEEDBACK};
    m.value.gimbal.yaw_rate = .1;
    m.value.gimbal.pitch_rate = -.2;
    m.value.gimbal.yaw = 1;
    m.value.gimbal.pitch = -.1;
    m.value.gimbal.quat.q0 = 2;
    app_can_frame_t frames[APP_VEHICLE_CAN_MAX_FRAMES];
    assert(app_vehicle_can_encode(&m, frames) == 3);
    assert(frames[0].dlc == 4 && memcmp(frames[0].data, rate, 4) == 0);
    assert(frames[1].dlc == 4 && memcmp(frames[1].data, angle, 4) == 0);
    assert(frames[2].data[0] == 0x30 && frames[2].data[1] == 0x75);
    m.value.gimbal.quat.q0 = 0;
    assert(app_vehicle_can_encode(&m, frames) == 0);
#else
    app_ctrl_can_msg_t m = {.type = APP_CAN_CHAS_FEEDBACK};
    m.value.chassis_state.power = 300;
    m.value.chassis_state.omega = -10;
    app_can_frame_t frames[APP_VEHICLE_CAN_MAX_FRAMES];
    assert(app_vehicle_can_encode(&m, frames) == 1);
    assert(frames[0].std_id == 0x112 && frames[0].dlc == 4 &&
           memcmp(frames[0].data, f.data, 4) == 0);
#endif
    assert(app_core_imu_read_request(2, 3, &f));
    const uint8_t request[] = {0xCC, 3, 0, 0xDD};
    assert(f.std_id == 0x31 && f.dlc == 4 && memcmp(f.data, request, 4) == 0);
    assert(!app_core_imu_read_request(0, 3, &f));
    f = (app_can_frame_t){
        .bus = 2, .std_id = 0x39, .dlc = 8, .data = {3, 0, 0, 0, 255, 255, 0, 128}};
    app_core_euler_t e;
    assert(app_core_imu_euler_decode(&f, &e));
    assert(app_vehicle_can_decode(&f,&feedback,&command,&index)==APP_VEHICLE_CAN_AUX);
    assert(feedback.dm_valid);
    near(e.pitch, -1.5707963268f, .000001f);
    near(e.yaw, 3.1415926536f, .000001f);
    assert(e.roll > 0 && e.roll < .0001f);
    f.data[1] = 1;
    assert(!app_core_imu_euler_decode(&f, &e));
    f = (app_can_frame_t){.bus = 1, .std_id = 0x233, .dlc = 8, .data = {0x30, 0x75}};
    float q[4];
    assert(app_core_quaternion_decode(&f, q));
    near(q[0], 1, .000001f);
    memset(f.data, 0, 8);
    assert(!app_core_quaternion_decode(&f, q));
    app_core_referee_t ref = {.valid_mask = 15,
                              .heat_remaining = -5,
                              .stage = 4,
                              .hurt_type = 5,
                              .remaining_s = 300,
                              .warning_level = 2,
                              .warning_robot = 7,
                              .flags = 0x3f,
                              .recovery_pct = 10,
                              .heat_ratio = 128,
                              .hp = 300,
                              .max_hp = 400,
                              .ally_1_hp = 500,
                              .ally_3_hp = 600,
                              .robot_id = 107},
                       out = {0};
    const uint32_t ids[] = {0x151, 0x153, 0x155, 0x157};
    for (unsigned i = 0; i < 4; i++)
    {
        assert(app_core_referee_encode(ids[i], &ref, &f));
        f.tick = 100 + i;
        assert(app_core_referee_decode(&f, &out));
        assert(out.tick[i] == 100 + i);
    }
    assert(out.heat_remaining == -5 && out.stage == 4 && out.hurt_type == 5 &&
           out.remaining_s == 300 && out.flags == 63 && out.hp == 300 && out.ally_3_hp == 600 &&
           out.robot_id == 107);
    assert(app_core_referee_encode(0x153, &ref, &f));
    const uint8_t field[] = {0x54, 0x2c, 1, 2, 7, 0x3f, 10, 128};
    assert(memcmp(f.data, field, 8) == 0);
    f.dlc = 7;
    assert(!app_core_referee_decode(&f, &out));
    ref.flags = 0x80;
    assert(!app_core_referee_encode(0x153, &ref, &f));
    puts("PASS core protocol: document byte vectors, exact DLC, signed fixed-point, DM mapping, "
         "quaternion and referee layouts");
    return 0;
}
