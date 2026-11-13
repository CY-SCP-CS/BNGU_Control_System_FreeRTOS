#include "app_ui_io.h"
#include "drv_referee.h"
static uint32_t (*s_clock)(void);
static app_ui_send_t s_send;
static uint8_t s_sequence;
static uint8_t s_frame[REFEREE_TX_BUF_SIZE];

void app_ui_bind(uint32_t (*clock_tick)(void), app_ui_send_t send)
{
    s_clock = clock_tick;
    s_send = send;
    s_sequence = 0;
}

uint32_t app_ui_now_tick(void)
{
    return s_clock ? s_clock() : 0;
}

int app_ui_send_frame(const void *frame, uint16_t length)
{
    return s_send && frame && length ? s_send(frame, length) : -1;
}

int app_ui_send_payload(uint16_t cmd_id, const void *payload, uint16_t length)
{
    uint16_t n =
        drv_referee_build_frame(s_frame, sizeof(s_frame), cmd_id, payload, length, s_sequence);
    if (!n)
    {
        return -1;
    }
    int result = app_ui_send_frame(s_frame, n);
    if (!result)
    {
        ++s_sequence;
    }
    return result;
}
