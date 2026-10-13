/**
 * @file    bsp_spi.c
 * @brief   SPI 驱动实现
 */
#include "bsp_spi.h"

#define BSP_SPI_TIMEOUT_MS  10

static SPI_HandleTypeDef *s_spi_dma_handle;
static bsp_spi_callback_t s_spi_dma_complete_callback;
static bsp_spi_callback_t s_spi_dma_error_callback;
static bsp_spi_callback_t s_spi_dma_wake_callback;
static volatile uint8_t s_close_pending, s_abort_pending;

void bsp_spi_reg_dma_wake_callback(bsp_spi_callback_t callback) { s_spi_dma_wake_callback=callback; }

static void finish_rx_dma(DMA_HandleTypeDef *dma)
{
    SPI_HandleTypeDef *spi=(SPI_HandleTypeDef *)dma->Parent;
    uint32_t status=spi->Instance->SR;
    if(!(status&SPI_SR_TXE) || (status&SPI_SR_BSY)) {
        s_close_pending=1;
        if(s_spi_dma_wake_callback)s_spi_dma_wake_callback(spi);
        return;
    }
    s_close_pending=0;
    __HAL_SPI_DISABLE_IT(spi,SPI_IT_ERR);
    CLEAR_BIT(spi->Instance->CR2,SPI_CR2_RXDMAEN|SPI_CR2_TXDMAEN);
    spi->TxXferCount=spi->RxXferCount=0;
    spi->State=HAL_SPI_STATE_READY;
    if(spi->ErrorCode!=HAL_SPI_ERROR_NONE) {
        (void)bsp_spi_abort_dma(spi);
    } else if(s_spi_dma_complete_callback)s_spi_dma_complete_callback(spi);
}

static void reset_dma(DMA_HandleTypeDef *dma)
{
    __HAL_DMA_CLEAR_FLAG(dma,__HAL_DMA_GET_TC_FLAG_INDEX(dma)|__HAL_DMA_GET_HT_FLAG_INDEX(dma)|
        __HAL_DMA_GET_TE_FLAG_INDEX(dma)|__HAL_DMA_GET_DME_FLAG_INDEX(dma)|__HAL_DMA_GET_FE_FLAG_INDEX(dma));
    dma->State=HAL_DMA_STATE_READY;
    __HAL_UNLOCK(dma);
}

void bsp_spi_poll_dma(SPI_HandleTypeDef *spi)
{
    if(spi!=s_spi_dma_handle)return;
    if(s_abort_pending) {
        /* DMA stream disable is asynchronous; never spin with interrupts masked. */
        if((spi->hdmarx->Instance->CR&DMA_SxCR_EN) || (spi->hdmatx->Instance->CR&DMA_SxCR_EN))return;
        reset_dma(spi->hdmarx); reset_dma(spi->hdmatx);
        __HAL_SPI_DISABLE(spi); __HAL_SPI_CLEAR_OVRFLAG(spi); __HAL_SPI_CLEAR_FREFLAG(spi);
        spi->TxXferCount=spi->RxXferCount=0; spi->State=HAL_SPI_STATE_READY; __HAL_UNLOCK(spi);
        s_abort_pending=s_close_pending=0;
        if(s_spi_dma_error_callback)s_spi_dma_error_callback(spi);
    } else if(s_close_pending)finish_rx_dma(spi->hdmarx);
}

void bsp_spi_transceive(SPI_HandleTypeDef *hspi,
                        const uint8_t *tx, uint8_t *rx, uint16_t size)
{
    HAL_SPI_TransmitReceive(hspi, (uint8_t *)tx, rx, size, BSP_SPI_TIMEOUT_MS);
}

HAL_StatusTypeDef bsp_spi_transceive_dma(SPI_HandleTypeDef *hspi,
                                         const uint8_t *tx, uint8_t *rx, uint16_t size)
{
    if (!hspi || !hspi->hdmarx || !hspi->hdmatx || !tx || !rx || size == 0U) {
        return HAL_ERROR;
    }
    uint32_t key=__get_PRIMASK(); __disable_irq();
    HAL_StatusTypeDef status=HAL_SPI_TransmitReceive_DMA(hspi, (uint8_t *)tx, rx, size);
    if(status==HAL_OK && hspi==s_spi_dma_handle && hspi->Init.Mode==SPI_MODE_MASTER &&
        hspi->Init.Direction==SPI_DIRECTION_2LINES && hspi->Init.CRCCalculation==SPI_CRCCALCULATION_DISABLE) {
        /* The vendor RX completion path waits up to SPI_DEFAULT_TIMEOUT for BSY.
         * Replace only this dedicated, non-CRC master path with a bounded check. */
        hspi->hdmarx->XferCpltCallback=finish_rx_dma;
        hspi->hdmarx->XferHalfCpltCallback=NULL;
        __HAL_DMA_DISABLE_IT(hspi->hdmarx,DMA_IT_HT);
    }
    __set_PRIMASK(key); return status;
}

void bsp_spi_reg_dma_callbacks(SPI_HandleTypeDef *hspi,
                                    bsp_spi_callback_t complete,
                                    bsp_spi_callback_t error)
{
    s_spi_dma_handle = hspi;
    s_spi_dma_complete_callback = complete;
    s_spi_dma_error_callback = error;
}

HAL_StatusTypeDef bsp_spi_abort_dma(SPI_HandleTypeDef *hspi)
{
    if (!hspi) {
        return HAL_ERROR;
    }
    if(hspi!=s_spi_dma_handle || !hspi->hdmarx || !hspi->hdmatx)return HAL_ERROR;
    s_abort_pending=1; s_close_pending=0;
    __HAL_SPI_DISABLE_IT(hspi,SPI_IT_ERR);
    CLEAR_BIT(hspi->Instance->CR2,SPI_CR2_RXDMAEN|SPI_CR2_TXDMAEN);
    __HAL_DMA_DISABLE_IT(hspi->hdmarx,DMA_IT_TC|DMA_IT_HT|DMA_IT_TE|DMA_IT_DME);
    __HAL_DMA_DISABLE_IT(hspi->hdmatx,DMA_IT_TC|DMA_IT_HT|DMA_IT_TE|DMA_IT_DME);
    __HAL_DMA_DISABLE_IT(hspi->hdmarx,DMA_IT_FE);
    __HAL_DMA_DISABLE_IT(hspi->hdmatx,DMA_IT_FE);
    __HAL_DMA_DISABLE(hspi->hdmarx); __HAL_DMA_DISABLE(hspi->hdmatx);
    bsp_spi_poll_dma(hspi);
    if(s_abort_pending && s_spi_dma_wake_callback)s_spi_dma_wake_callback(hspi);
    return HAL_OK;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == s_spi_dma_handle && s_spi_dma_complete_callback) {
        s_spi_dma_complete_callback(hspi);
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == s_spi_dma_handle && s_spi_dma_error_callback) {
        (void)bsp_spi_abort_dma(hspi);
    }
}

void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == s_spi_dma_handle && s_spi_dma_error_callback) {
        s_spi_dma_error_callback(hspi);
    }
}
