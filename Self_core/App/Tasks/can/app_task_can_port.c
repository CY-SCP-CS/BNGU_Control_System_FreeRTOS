/**
 * @file    app_task_can_port.c
 * @brief   CAN 服务到 STM32 CAN1/CAN2 的硬件接入
 */
#include "app_task_can_port.h"
#include "app_task_can.h"
#include "app_board_config.h"
#include "cmsis_os2.h"
#include "bsp_can.h"

static void can_rx(CAN_HandleTypeDef *can, uint32_t id, const uint8_t *bytes, uint8_t length)
{
    app_rtos_can_receive_isr(can == &hcan1 ? 1 : 2, id, bytes, length, osKernelGetTickCount());
}

static void can_event(CAN_HandleTypeDef *can, uint32_t error)
{
    (void)can;
    app_rtos_can_event_isr(error == 0                       ? APP_RTOS_CAN_TX_COMPLETE
                           : error == BSP_CAN_EVENT_INVALID ? APP_RTOS_CAN_INVALID_FRAME
                                                            : APP_RTOS_CAN_IO_ERROR);
}

void app_task_can_port_bind(void)
{
    bsp_can_reg_raw_callbacks(can_rx, can_event);
}

bool app_task_can_port_start(void)
{
    return bsp_can_start(&hcan1, 0, 14) == HAL_OK && bsp_can_start(&hcan2, 14, 14) == HAL_OK;
}

bool app_rtos_can_transmit(const app_can_frame_t *frame)
{
    return bsp_can_tx_frame(frame->bus == 1 ? &hcan1 : &hcan2, frame->std_id, frame->data,
                            frame->dlc) == 0;
}
