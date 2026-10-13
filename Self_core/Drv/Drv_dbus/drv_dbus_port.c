#include "drv_dbus.h"
#include "bsp_uart.h"
#include <string.h>
static uint8_t s_rx[2][18];
static uint8_t s_index;
static volatile uint8_t s_started;
static drv_dbus_frame_callback_t s_deliver;
void drv_dbus_port_bind(drv_dbus_frame_callback_t callback) { s_deliver=callback; }
static uint8_t restart(void) {
    s_started=bsp_uart_rx_dma(&huart3,s_rx[s_index],18)==HAL_OK;
    return s_started;
}
static void rx_event(UART_HandleTypeDef *uart,uint32_t event)
{
    if (event==BSP_UART_RX_HALF) return;
    if (event==BSP_UART_ERROR) { s_started=0; if(s_deliver)s_deliver(NULL); return; }
    uint16_t count=18-bsp_uart_get_rx_dma_remain(uart);
    if(event==BSP_UART_RX_IDLE && !count) return; /* IDLE following TC: do not duplicate. */
    (void)bsp_uart_stop_dma(uart);
    uint8_t index=s_index; s_index^=1;
    (void)restart();
    if(count==18 && s_deliver) s_deliver(s_rx[index]);
}
uint8_t drv_dbus_port_init(void)
{
    s_index=0; memset(s_rx,0,sizeof(s_rx));
    bsp_uart_reg_rx_event_callback(&huart3,rx_event);
    bsp_uart_clear_idle(&huart3); bsp_uart_enable_idle_it(&huart3);
    return restart();
}
uint8_t drv_dbus_port_poll(void)
{
    if(s_started) return 1;
    uint32_t key=__get_PRIMASK(); __disable_irq();
    (void)bsp_uart_stop_dma(&huart3); uint8_t ok=restart(); __set_PRIMASK(key); return ok;
}
