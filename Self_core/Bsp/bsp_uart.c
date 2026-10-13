#include "bsp_uart.h"

typedef struct {
    UART_HandleTypeDef *huart;
    bsp_uart_tx_cplt_callback_t callback;
} bsp_uart_tx_callback_entry_t;

typedef struct {
    UART_HandleTypeDef *huart;
    bsp_uart_rx_idle_callback_t callback;
} bsp_uart_rx_idle_callback_entry_t;

static bsp_uart_tx_callback_entry_t s_tx_callbacks[BSP_UART_TX_CALLBACK_MAX];

static struct { UART_HandleTypeDef *uart; bsp_uart_rx_event_callback_t callback; } s_events[3];
static void notify_rx(UART_HandleTypeDef *uart,uint32_t event) {
    for(unsigned i=0;i<3;i++) if(s_events[i].uart==uart && s_events[i].callback) { s_events[i].callback(uart,event); return; }
}
void bsp_uart_reg_rx_event_callback(UART_HandleTypeDef *uart,bsp_uart_rx_event_callback_t callback) {
    for(unsigned i=0;i<3;i++) if(!s_events[i].uart || s_events[i].uart==uart) { s_events[i].uart=uart; s_events[i].callback=callback; return; }
}
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *uart) { notify_rx(uart,BSP_UART_RX_HALF); }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart) { notify_rx(uart,BSP_UART_RX_FULL); }
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart) { notify_rx(uart,BSP_UART_ERROR); }
static uint8_t s_tx_callback_count;
static bsp_uart_rx_idle_callback_entry_t s_rx_idle_callbacks[BSP_UART_RX_IDLE_CALLBACK_MAX];
static uint8_t s_rx_idle_callback_count;

HAL_StatusTypeDef bsp_uart_tx(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len)
{
    if (!huart || !data || len == 0U) {
        return HAL_ERROR;
    }
    return HAL_UART_Transmit_IT(huart, data, len);
}

void bsp_uart_reg_tx_cplt_callback(UART_HandleTypeDef *huart,
                                   bsp_uart_tx_cplt_callback_t callback)
{
    uint8_t i;
    if (!huart || !callback) {
        return;
    }
    for (i = 0U; i < s_tx_callback_count; i++) {
        if (s_tx_callbacks[i].huart == huart) {
            s_tx_callbacks[i].callback = callback;
            return;
        }
    }
    if (s_tx_callback_count < BSP_UART_TX_CALLBACK_MAX) {
        s_tx_callbacks[s_tx_callback_count].huart = huart;
        s_tx_callbacks[s_tx_callback_count].callback = callback;
        s_tx_callback_count++;
    }
}

void bsp_uart_reg_rx_idle_callback(UART_HandleTypeDef *huart,
                                   bsp_uart_rx_idle_callback_t callback)
{
    uint8_t i;
    if (!huart || !callback) {
        return;
    }
    for (i = 0U; i < s_rx_idle_callback_count; i++) {
        if (s_rx_idle_callbacks[i].huart == huart) {
            s_rx_idle_callbacks[i].callback = callback;
            return;
        }
    }
    if (s_rx_idle_callback_count < BSP_UART_RX_IDLE_CALLBACK_MAX) {
        s_rx_idle_callbacks[s_rx_idle_callback_count].huart = huart;
        s_rx_idle_callbacks[s_rx_idle_callback_count].callback = callback;
        s_rx_idle_callback_count++;
    }
}

HAL_StatusTypeDef bsp_uart_rx_dma(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len)
{
    if (!huart || !data || len == 0U) {
        return HAL_ERROR;
    }
    return HAL_UART_Receive_DMA(huart, data, len);
}

HAL_StatusTypeDef bsp_uart_stop_dma(UART_HandleTypeDef *huart)
{
    return huart ? HAL_UART_DMAStop(huart) : HAL_ERROR;
}

uint16_t bsp_uart_get_rx_dma_remain(const UART_HandleTypeDef *huart)
{
    if (!huart || !huart->hdmarx) {
        return 0U;
    }
    return (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);
}

uint8_t bsp_uart_is_idle(const UART_HandleTypeDef *huart)
{
    return huart && __HAL_UART_GET_FLAG((UART_HandleTypeDef *)huart, UART_FLAG_IDLE) != RESET;
}

void bsp_uart_clear_idle(UART_HandleTypeDef *huart)
{
    if (huart) {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
    }
}

void bsp_uart_enable_idle_it(UART_HandleTypeDef *huart)
{
    if (huart) {
        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    }
}

void bsp_uart_rx_irq_dispatch(UART_HandleTypeDef *huart)
{
    uint8_t i;

    if (!huart) {
        return;
    }
    if (bsp_uart_is_idle(huart)) {
        for (i = 0U; i < s_rx_idle_callback_count; i++) {
            if (s_rx_idle_callbacks[i].huart == huart && s_rx_idle_callbacks[i].callback) {
                s_rx_idle_callbacks[i].callback(huart);
                break;
            }
        }
        bsp_uart_clear_idle(huart);
        notify_rx(huart,BSP_UART_RX_IDLE);
    }
}
void bsp_uart_irq_handler(UART_HandleTypeDef *huart) {
    bsp_uart_rx_irq_dispatch(huart); HAL_UART_IRQHandler(huart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t i;
    for (i = 0U; i < s_tx_callback_count; i++) {
        if (s_tx_callbacks[i].huart == huart && s_tx_callbacks[i].callback) {
            s_tx_callbacks[i].callback(huart);
            return;
        }
    }
}
