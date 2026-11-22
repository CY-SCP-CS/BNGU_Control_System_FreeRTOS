/** @file app_ui_port.c
 * @brief Optional UI USART6 transfer; owned exclusively by referee task.
 */
#include "app_ui_port.h"
#include "bsp_uart.h"
#include "app_rtos.h"
#include "drv_referee.h"
#include <string.h>
static uint8_t s_buffer[REFEREE_TX_BUF_SIZE];
static volatile uint8_t s_busy;
static uint32_t s_start_tick;

/** @brief ISR completion releases ownership, then wakes the owner for continued processing. */
static void tx_done(UART_HandleTypeDef *uart)
{
    (void)uart;
    s_busy = 0;
    app_rtos_notify_task(TASK_REF, APP_RTOS_TF_DATA_READY);
}

void app_ui_port_init(void)
{
    s_busy = 0;
    bsp_uart_reg_tx_cplt_callback(&huart6, tx_done);
}

int app_ui_port_send(const void *frame, uint16_t length)
{
    if (!frame || !length || length > sizeof(s_buffer))
    {
        return -1;
    }
    if (length < PROTOCOL_MIN_FRAME_LEN) return -1;
    uint16_t command;
    memcpy(&command, (const uint8_t *)frame + PROTOCOL_HEADER_LEN, sizeof(command));
    if (command == CMD_ROBOT_INTERACTION &&
        length > REFEREE_INTERACTION_PAYLOAD_MAX + PROTOCOL_MIN_FRAME_LEN) return -1;
    if (s_busy)
    {
        return 1;
    }
    memcpy(s_buffer, frame, length);
    s_start_tick = osKernelGetTickCount();
    s_busy = 1;
    if (bsp_uart_tx(&huart6, s_buffer, length) != HAL_OK)
    {
        s_busy = 0;
        app_rtos_count(COUNT_UART_ERROR);
        return -1;
    }
    return 0;
}

void app_ui_port_poll(void)
{
    if (s_busy && (uint32_t)(osKernelGetTickCount() - s_start_tick) > 100U)
    {
        /* TX uses IT, so AbortTransmit confirms no ISR retains s_buffer. */
        if (HAL_UART_AbortTransmit(&huart6) == HAL_OK)
        {
            s_busy = 0;
            app_rtos_count(COUNT_UART_TIMEOUT);
        }
    }
}
