#include "drv_referee.h"
#include "bsp_uart.h"
/* Static ordinary SRAM: DMA cannot access STM32F407 CCM. */
static uint8_t s_dma[512];
static uint16_t s_position;
static volatile uint8_t s_started;
static drv_referee_bytes_callback_t s_deliver;
void drv_referee_port_bind(drv_referee_bytes_callback_t callback) { s_deliver=callback; }
static void rx_event(UART_HandleTypeDef *uart,uint32_t event)
{
    if(event==BSP_UART_ERROR) { s_started=0; if(s_deliver)s_deliver(NULL,0); return; }
    /* Read NDTR for every HT/TC/IDLE, so simultaneous pending IRQs never duplicate bytes. */
    uint16_t now=(uint16_t)(512-bsp_uart_get_rx_dma_remain(uart));
    if(now==512) now=0;
    if(now==s_position) return;
    if(s_deliver) {
        if(now>s_position) s_deliver(s_dma+s_position,now-s_position);
        else { s_deliver(s_dma+s_position,512-s_position); if(now)s_deliver(s_dma,now); }
    }
    s_position=now;
}
uint8_t drv_referee_port_init(void)
{
    drv_referee_init(); s_position=0;
    bsp_uart_reg_rx_event_callback(&huart6,rx_event);
    bsp_uart_clear_idle(&huart6); bsp_uart_enable_idle_it(&huart6);
    s_started=bsp_uart_rx_dma(&huart6,s_dma,sizeof(s_dma))==HAL_OK; return s_started;
}
uint8_t drv_referee_port_poll(void)
{
    if(s_started) return 1;
    uint32_t key=__get_PRIMASK(); __disable_irq();
    (void)bsp_uart_stop_dma(&huart6); s_position=0;
    drv_referee_reset_stream(); /* Called by the task, never by the UART IRQ. */
    s_started=bsp_uart_rx_dma(&huart6,s_dma,sizeof(s_dma))==HAL_OK;
    __set_PRIMASK(key); return s_started;
}
void drv_referee_port_process(void) { (void)drv_referee_port_poll(); }
