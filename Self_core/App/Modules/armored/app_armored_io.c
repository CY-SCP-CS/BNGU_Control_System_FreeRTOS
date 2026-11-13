#include "app_armored_io.h"
static app_armored_submit_t s_submit;
static void *s_context;

void app_armored_bind(app_armored_submit_t submit, void *context)
{
    s_submit = submit;
    s_context = context;
}

uint8_t app_armored_can_submit(uint32_t id, const uint8_t data[8])
{
    return s_submit && data && id <= 0x7ff ? s_submit(id, data, s_context) : 1;
}
