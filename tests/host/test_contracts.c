/** @file test_contracts.c
 * @brief SI 边界、CAN 量化和裁判多实例回归。
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "lib_physical.h"
#include "legacy_adapter.h"
#include "app_sentry_can.h"
#include "drv_referee.h"
int main(void)
{
    assert(lib_quantize_i16(NAN, .001f) == 0);
    assert(lib_quantize_i16(1000, .001f) == 32767);
    assert(lib_quantize_i16(-1000, .001f) == -32768);
    for (int raw = -32768; raw <= 32767; raw++)
    {
        assert(lib_quantize_i16(raw * .001f, .001f) == raw);
        assert(lib_quantize_i16(raw * .0001f, .0001f) == raw);
        assert(lib_quantize_i16(raw * LIB_RAD_S_PER_RPM, LIB_RAD_S_PER_RPM) == raw);
    }
    app_can_feedback_t feedback = {0};
    legacy_can_feedback_t old = {0};
    for (int raw = 0; raw < 8192; raw++)
    {
        feedback.motor[0].angle_rad = raw * LIB_RAD_PER_ENC13;
        legacy_feedback(&feedback, &old);
        assert(old.motor[0].angle == raw);
    }
    for (int raw = -32768; raw <= 32767; raw++)
    {
        int16_t wire[4] = {(int16_t)raw, (int16_t)-raw, 123, 10000};
        app_chassis_comm_rx_t rx = {0};
        uint8_t mask = 0;
        assert(app_chassis_comm_decode(0x111, (uint8_t *)wire, 8, &rx, &mask));
        app_ctrl_can_msg_t message = {.type = APP_CAN_CHASSIS_CMD};
        message.value.chassis = rx.speed;
        app_can_frame_t frames[APP_SENTRY_CAN_MAX_FRAMES];
        assert(app_sentry_can_encode(&message, frames) == 1);
        assert(memcmp(frames[0].data, wire, 8) == 0);
    }
    drv_referee_context_t first, second;
    drv_referee_init(&first);
    drv_referee_init(&second);
    first.s_stream_length = 3;
    first.s_frames = 7;
    drv_referee_reset_stream(&first);
    assert(first.s_stream_length == 0 && first.s_frames == 7);
    assert(second.s_stream_length == 0 && second.s_frames == 0);
    const uint8_t fragment[] = {0xa5, 0, 0};
    (void)drv_referee_process(&first, fragment, sizeof(fragment));
    assert(first.s_stream_length == 3 && second.s_stream_length == 0);
    puts("PASS: SI integer round trips, 65536 CAN commands, parser isolation");
    return 0;
}
