/**
 * @file    bsp_spi.h
 * @brief   SPI 收发
 */
#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "bsp_cfg.h"

/**
 * @brief  SPI DMA 完成、错误或任务唤醒回调类型。
 * @note   参数为所属 SPI 句柄；可能由 DMA/SPI ISR 或任务延后收尾路径调用。
 * @note   回调应只推进状态或复制数据，不阻塞等待硬件标志。
 */
typedef void (*bsp_spi_callback_t)(SPI_HandleTypeDef *hspi);

/**
 * @brief  SPI 同步收发 (全双工)
 * @param  hspi  SPI 句柄
 * @param  tx    发送缓冲区
 * @param  rx    接收缓冲区
 * @param  size  收发字节数
 */
void bsp_spi_transceive(SPI_HandleTypeDef *hspi, const uint8_t *tx, uint8_t *rx, uint16_t size);

/** @brief 启动 SPI DMA 全双工收发，缓冲区在完成前必须保持有效。 
 * @param  hspi  SPI 句柄
 * @param  tx    发送缓冲区
 * @param  rx    接收缓冲区
 * @param  size  收发字节数
 * @return HAL 状态码
 * @note   接收缓冲必须位于 DMA 可访问的 SRAM，不能放入 STM32F407 CCM；专用 BMI088 路径使用有界收尾。
 * @note   HAL_OK 仅表示启动成功，缓冲仍归该事务占用，直到完成或确认中止。
 */
HAL_StatusTypeDef bsp_spi_transceive_dma(SPI_HandleTypeDef *hspi, const uint8_t *tx, uint8_t *rx,
                                         uint16_t size);

/** @brief 注册 SPI DMA 完成和错误回调。
 * @param  hspi      SPI 句柄
 * @param  complete  DMA 完成回调
 * @param  error     DMA 错误回调
 * @note   当前只有一个注册槽，绑定 SPI1 后再启动传输；回调可能由 ISR 或任务收尾调用。
 */
void bsp_spi_reg_dma_callbacks(SPI_HandleTypeDef *hspi, bsp_spi_callback_t complete,
                               bsp_spi_callback_t error);

/** @brief 异步中止 SPI DMA。 
 * @param  hspi  SPI 句柄
 * @return HAL 状态码
 * @note   HAL_OK 只表示中止请求已接受，真正收尾由错误回调确认。
 * @note   不在中断中等待 DMA 流或 SPI 标志；调用 poll 推进尚未完成的关闭过程。
 */
HAL_StatusTypeDef bsp_spi_abort_dma(SPI_HandleTypeDef *hspi);
/* BMI088 full-duplex DMA completion/abort never waits for SPI flags in an ISR.
 * A deferred closure is resumed by the owning task. */
/**
 * @brief  尝试一次 DMA 完成收尾或中止收尾，硬件尚未就绪就立即返回。
 * @param  hspi         已通过 bsp_spi_reg_dma_callbacks() 绑定的 SPI 句柄。
 * @note   由拥有 SPI 的驱动任务重试；中止入口也可立即调用一次，因此回调可能来自任务或 ISR。
 * @note   调用者需与 DMA/SPI 状态变更串行化；当前 IMU 端口在短临界区调用，不忙等 DMA 流关闭。
 */
void bsp_spi_poll_dma(SPI_HandleTypeDef *hspi);
/**
 * @brief  绑定需要任务继续处理 DMA 收尾时的唤醒入口。
 * @param  callback     状态检查唤醒函数，NULL 时不通知。
 * @note   仅保存函数指针，不创建任务；当前由 IMU 端口转换成线程标志。
 */
void bsp_spi_reg_dma_wake_callback(bsp_spi_callback_t callback);

#endif
