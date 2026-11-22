#include <assert.h>
#include "app_vehicle_config.h"
#include "app_vehicle_can.h"

#ifdef ROBOT_HERO
#error "HERO must be removed"
#endif

int main(void)
{
    app_can_frame_t frames[APP_VEHICLE_CAN_MAX_FRAMES] = {0};
    app_ctrl_can_msg_t out = {.type = APP_CAN_CHAS_CURRENT};
    out.value.current[0] = 0x1234;
    out.value.current[2] = 0x5678;
    app_can_frame_t in = {.bus = 2, .std_id = 0x203, .dlc = 8};
    app_can_feedback_t feedback = {0};
    app_can_ctrl_msg_t command = {0};
    unsigned index = 99;
#if CURRENT_ROBOT == ROBOT_ARMORED
    assert(app_vehicle_can_encode(&out, frames) == 0);
    assert(app_vehicle_can_decode(&in, &feedback, &command, &index) == APP_VEHICLE_CAN_IGNORED);
    assert(!APP_BOARD_HAS_CHASSIS && !APP_BOARD_HAS_SHOOT && !APP_BOARD_HAS_GIMBAL);
#elif CURRENT_ROBOT == ROBOT_NAVIGATION
    assert(app_vehicle_can_encode(&out, frames) == 1);
    assert(frames[0].std_id == 0x200 && frames[0].data[4] == 0x56 && frames[0].data[5] == 0x78);
    assert(app_vehicle_can_decode(&in, &feedback, &command, &index) == APP_VEHICLE_CAN_MOTOR);
    assert(index == 2);
    in.bus = 1;
    in.std_id = 0x113; /* Sentry-only Ackermann command. */
    assert(app_vehicle_can_decode(&in, &feedback, &command, &index) == APP_VEHICLE_CAN_IGNORED);
    out.type = APP_CAN_SHOOT_CURRENT;
    assert(app_vehicle_can_encode(&out, frames) == 0);
    assert(APP_BOARD_HAS_CHASSIS && !APP_BOARD_HAS_GIMBAL && !APP_BOARD_HAS_SHOOT);
#else
    in.std_id = 0x201;
    assert(app_vehicle_can_decode(&in, &feedback, &command, &index) == APP_VEHICLE_CAN_MOTOR);
    assert(app_vehicle_can_encode(&out, frames) == 2);
    assert(frames[0].std_id == 0x200 && frames[1].std_id == 0x1FF);
    assert(frames[0].data[0] == 0x12 && frames[1].data[0] == 0x56);
    assert(APP_BOARD_HAS_CHASSIS && !APP_BOARD_HAS_GIMBAL && !APP_BOARD_HAS_SHOOT);
#endif
    return 0;
}
