/**
 * @file    bsp_spi.c
 * @brief   SPI 驱动实现
 */
#include "bsp_spi.h"

#define BSP_SPI_TIMEOUT_MS 10

static SPI_HandleTypeDef *s_spi_dma_handle;
static bsp_spi_callback_t s_spi_dma_complete_callback;
static bsp_spi_callback_t s_spi_dma_error_callback;
static bsp_spi_callback_t s_spi_dma_wake_callback;
/* s_close_pending：DMA 完成但 SPI 尚忙；s_abort_pending：停止 DMA 后等待使能位清零。 */
static volatile uint8_t s_close_pending, s_abort_pending;

/**
 * @brief  绑定需要任务继续处理 DMA 收尾时的唤醒入口。
 * @param  callback     状态检查唤醒函数，NULL 时不通知。
 * @note   仅保存函数指针，不创建任务；当前由 IMU 端口转换成线程标志。
 */
void bsp_spi_reg_dma_wake_callback(bsp_spi_callback_t callback)
{
    s_spi_dma_wake_callback = callback;
}

/**
 * @brief  用一次 TXE/BSY 检查完成 SPI 收尾，未就绪时留给任务再处理。
 * @param  dma          已完成的 RX DMA 句柄，其 Parent 指向 SPI 句柄。
 * @note   专用于已注册的无 CRC、双线主机全双工 DMA；替换会在 ISR 内等待 SPI BSY 的 HAL 路径。
 * @note   可从 DMA ISR 或 poll 路径进入；完成后通知驱动进入下一采样阶段。
 */
static void finish_rx_dma(DMA_HandleTypeDef *dma)
{
    SPI_HandleTypeDef *spi = (SPI_HandleTypeDef *)dma->Parent;
    uint32_t status = spi->Instance->SR;
    if (!(status & SPI_SR_TXE) || (status & SPI_SR_BSY))
    {
        s_close_pending = 1;
        if (s_spi_dma_wake_callback)
        {
            s_spi_dma_wake_callback(spi);
        }
        return;
    }
    s_close_pending = 0;
    __HAL_SPI_DISABLE_IT(spi, SPI_IT_ERR);
    CLEAR_BIT(spi->Instance->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    spi->TxXferCount = spi->RxXferCount = 0;
    spi->State = HAL_SPI_STATE_READY;
    if (spi->ErrorCode != HAL_SPI_ERROR_NONE)
    {
        (void)bsp_spi_abort_dma(spi);
    }
    else if (s_spi_dma_complete_callback)
    {
        s_spi_dma_complete_callback(spi);
    }
}

/**
 * @brief  在 DMA 流使能位已经清零后，清中断标志并复位 HAL 状态及锁。
 * @param  dma          已经确认停止的 DMA 句柄。
 * @note   仅用于中止收尾；本函数本身不等待，也不能替代停止一个仍运行的 DMA 流。
 */
static void reset_dma(DMA_HandleTypeDef *dma)
{
    __HAL_DMA_CLEAR_FLAG(dma, __HAL_DMA_GET_TC_FLAG_INDEX(dma) | __HAL_DMA_GET_HT_FLAG_INDEX(dma) |
                                  __HAL_DMA_GET_TE_FLAG_INDEX(dma) |
                                  __HAL_DMA_GET_DME_FLAG_INDEX(dma) |
                                  __HAL_DMA_GET_FE_FLAG_INDEX(dma));
    dma->State = HAL_DMA_STATE_READY;
    __HAL_UNLOCK(dma);
}

/**
 * @brief  尝试一次 DMA 完成收尾或中止收尾，硬件尚未就绪就立即返回。
 * @param  spi          已通过 bsp_spi_reg_dma_callbacks() 绑定的 SPI 句柄。
 * @note   由拥有 SPI 的驱动任务重试；中止入口也可立即调用一次，因此回调可能来自任务或 ISR。
 * @note   调用者需与 DMA/SPI 状态变更串行化；当前 IMU 端口在短临界区调用，不忙等 DMA 流关闭。
 */
void bsp_spi_poll_dma(SPI_HandleTypeDef *spi)
{
    if (spi != s_spi_dma_handle)
    {
        return;
    }
    if (s_abort_pending)
    {
        /* DMA stream disable is asynchronous; never spin with interrupts masked. */
        if ((spi->hdmarx->Instance->CR & DMA_SxCR_EN) || (spi->hdmatx->Instance->CR & DMA_SxCR_EN))
        {
            return;
        }
        reset_dma(spi->hdmarx);
        reset_dma(spi->hdmatx);
        __HAL_SPI_DISABLE(spi);
        __HAL_SPI_CLEAR_OVRFLAG(spi);
        __HAL_SPI_CLEAR_FREFLAG(spi);
        spi->TxXferCount = spi->RxXferCount = 0;
        spi->State = HAL_SPI_STATE_READY;
        __HAL_UNLOCK(spi);
        s_abort_pending = s_close_pending = 0;
        if (s_spi_dma_error_callback)
        {
            s_spi_dma_error_callback(spi);
        }
    }
    else if (s_close_pending)
    {
        finish_rx_dma(spi->hdmarx);
    }
}

/**
 * @brief  使用 HAL 同步全双工 SPI 收发，超时参数为 10 ms。
 * @param  hspi         SPI 句柄。
 * @param  tx           发送缓冲。
 * @param  rx           接收缓冲。
 * @param  size         收发字节数。
 * @note   该兼容接口不返回 HAL 状态；运行时严格采样使用下面的 DMA 接口。
 */
void bsp_spi_transceive(SPI_HandleTypeDef *hspi, const uint8_t *tx, uint8_t *rx, uint16_t size)
{
    HAL_SPI_TransmitReceive(hspi, (uint8_t *)tx, rx, size, BSP_SPI_TIMEOUT_MS);
}

/**
 * @brief  启动 SPI 全双工 DMA，并为专用主机路径安装有界完成回调。
 * @param  hspi         带有效 RX/TX DMA 句柄的 SPI。
 * @param  tx           发送缓冲，完成/确认中止前必须有效。
 * @param  rx           接收缓冲，必须位于 DMA 可访问的 SRAM。
 * @param  size         本次收发字节数，必须大于 0。
 * @return HAL_OK=启动成功，其余为参数、忙或 HAL 启动错误。
 * @note   仅对已注册且主机/双线/无 CRC 的 SPI 替换 RX DMA 完成回调；其他配置沿用 HAL 路径。
 * @note   短临界区防止传输极快完成时，在安装专用回调前进入旧完成路径。
 */
HAL_StatusTypeDef bsp_spi_transceive_dma(SPI_HandleTypeDef *hspi, const uint8_t *tx, uint8_t *rx,
                                         uint16_t size)
{
    if (!hspi || !hspi->hdmarx || !hspi->hdmatx || !tx || !rx || size == 0U)
    {
        return HAL_ERROR;
    }
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(hspi, (uint8_t *)tx, rx, size);
    if (status == HAL_OK && hspi == s_spi_dma_handle && hspi->Init.Mode == SPI_MODE_MASTER &&
        hspi->Init.Direction == SPI_DIRECTION_2LINES &&
        hspi->Init.CRCCalculation == SPI_CRCCALCULATION_DISABLE)
    {
        /* The vendor RX completion path waits up to SPI_DEFAULT_TIMEOUT for BSY.
         * Replace only this dedicated, non-CRC master path with a bounded check. */
        hspi->hdmarx->XferCpltCallback = finish_rx_dma;
        hspi->hdmarx->XferHalfCpltCallback = NULL;
        __HAL_DMA_DISABLE_IT(hspi->hdmarx, DMA_IT_HT);
    }
    __set_PRIMASK(key);
    return status;
}

/**
 * @brief  登记一组 SPI DMA 完成和错误收尾回调。
 * @param  hspi         唯一绑定的 SPI 句柄，当前为 SPI1。
 * @param  complete     完成回调。
 * @param  error        错误/中止收尾回调。
 * @note   当前实现只有一个全局注册槽，重新注册会覆盖旧绑定；启动 DMA 前调用。
 */
void bsp_spi_reg_dma_callbacks(SPI_HandleTypeDef *hspi, bsp_spi_callback_t complete,
                               bsp_spi_callback_t error)
{
    s_spi_dma_handle = hspi;
    s_spi_dma_complete_callback = complete;
    s_spi_dma_error_callback = error;
}

/**
 * @brief  关闭 SPI DMA 请求并请求停用双 DMA 流，剩余收尾由 poll 推进。
 * @param  hspi         已注册且带 RX/TX DMA 的 SPI 句柄。
 * @return HAL_OK=已接受中止请求，不保证此时已完全停止；HAL_ERROR=句柄或绑定无效。
 * @note   不循环等待 DMA EN 或 SPI BSY；只有错误回调确认收尾后，驱动才可复用缓冲并开始新事务。
 */
HAL_StatusTypeDef bsp_spi_abort_dma(SPI_HandleTypeDef *hspi)
{
    if (!hspi)
    {
        return HAL_ERROR;
    }
    if (hspi != s_spi_dma_handle || !hspi->hdmarx || !hspi->hdmatx)
    {
        return HAL_ERROR;
    }
    s_abort_pending = 1;
    s_close_pending = 0;
    __HAL_SPI_DISABLE_IT(hspi, SPI_IT_ERR);
    CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    __HAL_DMA_DISABLE_IT(hspi->hdmarx, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME);
    __HAL_DMA_DISABLE_IT(hspi->hdmatx, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME);
    __HAL_DMA_DISABLE_IT(hspi->hdmarx, DMA_IT_FE);
    __HAL_DMA_DISABLE_IT(hspi->hdmatx, DMA_IT_FE);
    __HAL_DMA_DISABLE(hspi->hdmarx);
    __HAL_DMA_DISABLE(hspi->hdmatx);
    bsp_spi_poll_dma(hspi);
    if (s_abort_pending && s_spi_dma_wake_callback)
    {
        s_spi_dma_wake_callback(hspi);
    }
    return HAL_OK;
}

/**
 * @brief  分发 HAL 标准全双工完成回调。
 * @param  hspi         HAL 回调所属的 SPI 句柄。
 * @note   专用 BMI088 DMA 路径通常由 finish_rx_dma 直接通知；此入口保留给 HAL 标准路径。
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == s_spi_dma_handle && s_spi_dma_complete_callback)
    {
        s_spi_dma_complete_callback(hspi);
    }
}

/**
 * @brief  把 HAL SPI 错误转换为非等待 DMA 中止。
 * @param  hspi         HAL 回调所属的 SPI 句柄。
 * @note   错误回调在中止真正收尾后执行，避免仍传输时提前复用缓冲。
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == s_spi_dma_handle && s_spi_dma_error_callback)
    {
        (void)bsp_spi_abort_dma(hspi);
    }
}

/**
 * @brief  把 HAL 已完成的中止转交给驱动错误收尾入口。
 * @param  hspi         HAL 回调所属的 SPI 句柄。
 * @note   仅分发当前登记的 SPI，驱动负责释放片选并恢复采样状态。
 */
void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == s_spi_dma_handle && s_spi_dma_error_callback)
    {
        s_spi_dma_error_callback(hspi);
    }
}
