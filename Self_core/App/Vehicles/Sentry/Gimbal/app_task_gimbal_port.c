#include "app_task_gimbal_io.h"
#include "drv_buzzer.h"
#include "cmsis_os2.h"

void app_task_gimbal_ready_signal(bool ready)
{

    static uint8_t announced, sounding;
    static uint32_t since;
    uint32_t now = osKernelGetTickCount();
    if (ready && !announced)
    {
        drv_buzzer_set_freq(2000);
        drv_buzzer_on();
        announced = sounding = 1;
        since = now;
    }
    if (sounding && (uint32_t)(now - since) >= 500)
    {
        drv_buzzer_off();
        sounding = 0;
    }
}
