/** Navigation vehicle owns gamepad RX; log owns UART1 TX. */
#include "app_task_navigation_io.h"
#include "bsp_uart.h"
#include "cmsis_os2.h"
#include "drv_game_controller.h"
#include <string.h>

void app_navigation_gamepad_poll(app_gamepad_snapshot_t *out)
{

    static uint8_t dma[128], line[64], started;
    static uint16_t consumed, length;
    static uint8_t discard;
    if (!started)
    {
        if (bsp_uart_rx_dma(&huart1, dma, sizeof(dma)) != HAL_OK)
        {
            out->valid = 0;
            return;
        }
        started = 1;
        consumed = 0;
    }
    if (huart1.ErrorCode != HAL_UART_ERROR_NONE)
    {
        (void)HAL_UART_AbortReceive(&huart1);
        started = 0;
        length = 0;
        discard = 1;
        out->valid = 0;
        return;
    }
    static uint32_t previous_poll;
    uint32_t now = osKernelGetTickCount();
    uint16_t end = sizeof(dma) - bsp_uart_get_rx_dma_remain(&huart1);
    if (end > sizeof(dma))
    {
        out->valid = 0;
        return;
    }
    end %= sizeof(dma);
    consumed %= sizeof(dma);
    if ((uint32_t)(now - previous_poll) >= 10 && previous_poll)
    {
        consumed = end;
        length = 0;
        discard = 1;
        out->valid = 0;
    }
    previous_poll = now;
    /* UART1 RX DMA is circular. Poll every 1 ms, before navigation decimation. */
    while (consumed != end)
    {
        consumed %= sizeof(dma);
        if (consumed == end)
        {
            break;
        }
        uint8_t c = dma[consumed++];
        if (c == '\n' || c == '\r')
        {
            if (!discard && length)
            {
                line[length] = 0;
                drv_game_controller_buttons_t decoded = {0};
                if (drv_game_controller_decode(line, &decoded))
                {
                    *out = (app_gamepad_snapshot_t){
                        decoded.lx, decoded.ly, decoded.rx, decoded.ry, osKernelGetTickCount(), 1};
                }
            }
            length = 0;
            discard = 0;
        }
        else if (length < sizeof(line) - 1 && !discard)
        {
            line[length++] = c;
        }
        else
        {
            length = 0;
            discard = 1;
        }
    }
}
