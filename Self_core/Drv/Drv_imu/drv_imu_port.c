/**
 * @file    drv_imu_port.c
 * @brief   BMI088 硬件适配：初始化标定、TIM14 触发和 ACC/GYRO 两阶段 SPI DMA。
 * @note    硬件节拍只启动采样；完整原始样本交给 IMU 任务解析并运行姿态算法。
 * @note    DMA 完成与中止采用非等待状态推进，任务负责延后收尾和错误恢复。
 */
#include "drv_imu_port.h"
#include "bsp_spi.h"
#include "bsp_gpio.h"
#include "bsp_tim.h"
#include <string.h>

/** @brief 单份 IMU 原始样本的采集状态；由硬件中断和任务恢复路径串行推进。 */
enum
{
    IDLE,         /**< 无进行中的样本，可以接受下一次 TIM14 触发。 */
    ACC,          /**< 加速度计 DMA 正在传输。 */
    GYRO_PENDING, /**< ACC 数据已保存，等待 SPI/DMA 真正空闲后启动 GYRO。 */
    GYRO,         /**< 陀螺仪 DMA 正在传输。 */
    ABORTING      /**< 中止请求已发出，等待 DMA 使能位清除再复位。 */
};

/** @brief 物理 SPI 的唯一进行中事务；不同设备共享总线时忙即拒绝，不自动排队。 */
static struct
{
    SPI_HandleTypeDef *spi;
    drv_imu_port_t *owner;
} s_owners[3];

/** @brief 查找事件所属采集实例。
 * @param[in] spi 硬件 SPI。
 * @return 进行中实例；无事务为 NULL。
 * @note ISR 只读，owner 在短临界区或初始化设置。 */
static drv_imu_port_t *owner(SPI_HandleTypeDef *spi)
{
    for (unsigned i = 0; i < 3; i++)
    {
        if (s_owners[i].spi == spi)
        {
            return s_owners[i].owner;
        }
    }
    return NULL;
}

/** @brief 取得/释放总线完整采样的独占权。
 * @param[in,out] p 实例，非 NULL。
 * @param[in] acquire 1=尝试占用；0=仅释放自己的事务。
 * @return 1=成功；0=表满或其他实例正在占用。
 * @note 调用者串行化；不排队、不抢占其他实例。 */
static uint8_t own(drv_imu_port_t *p, uint8_t acquire)
{
    for (unsigned i = 0; i < 3; i++)
    {
        if (s_owners[i].spi == p->config.spi || !s_owners[i].spi)
        {
            s_owners[i].spi = p->config.spi;
            if (acquire)
            {
                if (s_owners[i].owner && s_owners[i].owner != p)
                {
                    return 0;
                }
                s_owners[i].owner = p;
            }
            else if (s_owners[i].owner == p)
            {
                s_owners[i].owner = NULL;
            }
            return 1;
        }
    }
    return 0;
}

/**
 * @brief  把 SPI 未完成收尾状态转换为上层任务唤醒请求。
 * @param  spi_handle   SPI 句柄，当前仅有 SPI1，函数不使用该参数。
 * @note   以 NULL 调用样本回调，仅通知任务检查状态；不表示采集到新样本。
 */
static void wake_task(SPI_HandleTypeDef *spi_handle)
{
    drv_imu_port_t *p = owner(spi_handle);
    if (!p)
    {
        return;
    }
    (void)spi_handle;
    if (p->receive)
    {
        p->receive(p->context, DRV_IO_SERVICE, NULL);
    }
}

/**
 * @brief  绑定完整样本交付入口和初始化时使用的毫秒等待函数。
 * @param  callback     样本/状态回调，NULL 表示不交付。
 * @param  delay        初始化/标定延时函数；RTOS 适配传入可调度等待。
 * @note   必须在初始化和采样启动前绑定；未提供 delay 时退回 BSP 毫秒延时。
 */

/**
 * @brief  设置 BMI088 PA4 加速度计片选电平。
 * @param  ctx          通用总线接口上下文，当前硬件映射未使用。
 * @param  state        0=选中，1=释放片选。
 * @note   仅作 GPIO 操作，供初始化同步访问和运行时 DMA 状态机使用。
 */
static void acc_cs(void *ctx, uint8_t state)
{
    drv_imu_port_t *p = ctx;
    bsp_gpio_write_pin(p->config.acc_gpio, p->config.acc_pin, state);
}

/**
 * @brief  设置 BMI088 PB0 陀螺仪片选电平。
 * @param  ctx          通用总线接口上下文，当前硬件映射未使用。
 * @param  state        0=选中，1=释放片选。
 * @note   仅作 GPIO 操作，供初始化同步访问和运行时 DMA 状态机使用。
 */
static void gyro_cs(void *ctx, uint8_t state)
{
    drv_imu_port_t *p = ctx;
    bsp_gpio_write_pin(p->config.gyro_gpio, p->config.gyro_pin, state);
}

/**
 * @brief  通过绑定函数等待指定毫秒数，供初始化和标定使用。
 * @param  ms           等待时长，单位 ms。
 * @note   RTOS 环境绑定到任务延时；若未绑定则调用 BSP 延时，不应在采样 ISR 中调用。
 */
static void drv_imu_port_delay_ms(void *ctx, uint32_t ms)
{
    drv_imu_port_t *p = ctx;
    if (p->delay)
    {
        p->delay(p->delay_context, ms);
    }
    else
    {
        bsp_tim_delay_ms(ms);
    }
}

/**
 * @brief  执行 BMI088 初始化阶段的同步 SPI1 访问并记录 HAL 失败。
 * @param  ctx          通用总线上下文，当前未使用。
 * @param  tx           发送缓冲。
 * @param  rx           接收缓冲。
 * @param  len          本次全双工收发字节数。
 * @note   HAL 超时参数为 10 ms；运行时采样使用 DMA，不调用此同步接口。
 */
static void spi(void *ctx, const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    drv_imu_port_t *p = ctx;
    if (HAL_SPI_TransmitReceive(p->config.spi, (uint8_t *)tx, rx, len, 10) != HAL_OK)
    {
        p->init_io_failed = 1;
    }
}

/**
 * @brief  检查 SPI1、两条 DMA 的 HAL 状态及硬件流使能位是否都已空闲。
 * @return 1=可安全启动下一阶段，0=仍在收尾或尚未空闲。
 * @note   使用前必须已初始化 SPI1 和其 RX/TX DMA 句柄；不轮询等待。
 */
static uint8_t ready(drv_imu_port_t *p)
{
    return p->config.spi->State == HAL_SPI_STATE_READY &&
           p->config.spi->hdmarx->State == HAL_DMA_STATE_READY &&
           p->config.spi->hdmatx->State == HAL_DMA_STATE_READY &&
           !(p->config.spi->hdmarx->Instance->CR & DMA_SxCR_EN) &&
           !(p->config.spi->hdmatx->Instance->CR & DMA_SxCR_EN);
}

/**
 * @brief  释放两个片选、累计错误并把采样状态恢复为空闲。
 * @param  spi_handle   回调来源 SPI，当前实现未使用该参数。
 * @note   DMA 确认中止后调用；通过 NULL 样本回调唤醒任务，不交付部分样本。
 */
static void finished_error(SPI_HandleTypeDef *spi_handle)
{
    drv_imu_port_t *p = owner(spi_handle);
    if (!p)
    {
        return;
    }
    (void)spi_handle;
    acc_cs(p, 1);
    gyro_cs(p, 1);
    p->error_count++;
    p->state = IDLE;
    if (p->receive)
    {
        p->receive(p->context, DRV_IO_ERROR, NULL);
    }
    (void)own(p, 0);
}

/**
 * @brief  进入 ABORTING 并发起非等待的 SPI DMA 中止。
 * @note   若中止接口失败且硬件已经空闲则直接完成错误收尾；其余情况由后续 poll 推进。
 */
static void abort_dma(drv_imu_port_t *p)
{
    p->state = ABORTING;
    if (bsp_spi_abort_dma(p->config.spi) != HAL_OK && ready(p))
    {
        finished_error(p->config.spi);
    }
}

/**
 * @brief  在 ACC 完成后拉低陀螺片选并启动 GYRO DMA。
 * @return 1=GYRO DMA 启动成功，0=启动失败且已请求中止。
 * @note   由完成回调或任务恢复路径调用；失败不交付半份样本。
 */
static uint8_t start_gyro(drv_imu_port_t *p)
{
    p->state = GYRO;
    gyro_cs(p, 0);
    if (bsp_spi_transceive_dma(p->config.spi, p->gyro_tx, p->gyro_rx, sizeof(p->gyro_tx)) != HAL_OK)
    {
        abort_dma(p);
        return 0;
    }
    return 1;
}

/**
 * @brief  处理两阶段 DMA 完成：先保存 ACC，再保存 GYRO 并交付完整样本。
 * @param  spi_handle   完成回调来源 SPI，当前只绑定 SPI1。
 * @note   ACC 六字节轴数据位于突发读取的偏移 20，GYRO 六字节位于偏移 3；同时保存两个芯片 ID。
 * @note   若 ACC 已完成但 SPI/DMA 尚未空闲，进入 GYRO_PENDING，由任务继续启动 GYRO。
 * @note   完整样本交付回调必须同步复制；可能在 DMA ISR 或任务延后收尾路径执行。
 */
static void completed(SPI_HandleTypeDef *spi_handle)
{
    drv_imu_port_t *p = owner(spi_handle);
    if (!p)
    {
        return;
    }
    (void)spi_handle;
    if (p->state == ACC)
    {
        acc_cs(p, 1);
        memcpy(p->sample.acc, p->acc_rx, 2);
        memcpy(p->sample.acc + 2, p->acc_rx + 20, 6);
        p->sample.acc_chip_id = p->acc_rx[2];
        p->state = GYRO_PENDING;
        if (ready(p))
        {
            (void)start_gyro(p);
        }
        else if (p->receive)
        {
            p->receive(p->context, DRV_IO_SERVICE, NULL);
        }
    }
    else if (p->state == GYRO)
    {
        gyro_cs(p, 1);
        p->sample.gyro[0] = p->gyro_rx[0];
        memcpy(p->sample.gyro + 1, p->gyro_rx + 3, 6);
        p->sample.gyro_chip_id = p->gyro_rx[1];
        if (p->receive)
        {
            p->receive(p->context, DRV_IO_DATA, &p->sample);
        }
        p->state = IDLE;
        (void)own(p, 0);
    }
}

/**
 * @brief  配置 BMI088，完成陀螺零偏标定和初始姿态对齐，再转入 DMA 采样。
 * @param  imu          由初始化任务持有并随后交给 IMU 任务的算法句柄；不可为 NULL。
 * @return 1=同步 SPI 访问未报告失败，0=至少一次初始化访问失败。
 * @note   SPI1、PA4(ACC_CS)、PB0(GYRO_CS) 为当前硬件映射；进行 3000 次陀螺零偏采样。
 * @note   本函数不启动 TIM14；标定完成后，系统初始化流程另行启动严格采样节拍。
 */
uint8_t drv_imu_port_init(drv_imu_port_t *p, drv_imu_t *imu, const drv_imu_port_config_t *config,
                          drv_imu_receive_t receive, void *context, drv_imu_delay_t delay,
                          void *delay_context)
{
    if (!p || !imu || !config || !config->spi || !config->spi->hdmarx || !config->spi->hdmatx ||
        !receive)
    {
        return 0;
    }
    memset(p, 0, sizeof(*p));
    p->config = *config;
    p->receive = receive;
    p->context = context;
    p->delay = delay;
    p->delay_context = delay_context;
    p->acc_tx[0] = 0x80;
    p->gyro_tx[0] = 0x80;
    if (!own(p, 1))
    {
        return 0;
    }
    if (bsp_spi_reg_dma_callbacks(p->config.spi, completed, finished_error) != HAL_OK)
    {
        (void)own(p, 0);
        return 0;
    }

    p->init_io_failed = 0;
    drv_imu_bus_t bus = {.spi_ctx = p,
                         .spi_xfer = spi,
                         .acc_cs_ctx = p,
                         .acc_cs = acc_cs,
                         .gyro_cs_ctx = p,
                         .gyro_cs = gyro_cs,
                         .delay_ctx = p,
                         .delay_ms = drv_imu_port_delay_ms};
    drv_imu_init(imu, &bus);
    drv_imu_start(imu);
    drv_imu_calibrate_gyro(imu, 3000);
    drv_imu_read_acc_raw(imu);
    drv_imu_data_convert(imu);
    drv_imu_initial_alignment(imu);
    drv_imu_quat_to_euler(imu);

    bsp_spi_reg_dma_wake_callback(p->config.spi, wake_task);
    (void)own(p, 0);
    return !p->init_io_failed;
}

/**
 * @brief  接受一次硬件采样节拍并尝试启动 ACC DMA。
 * @param  tick         触发时内核 tick。
 * @param  sequence     本次触发序号。
 * @param  cycles       触发时 DWT 周期计数。
 * @return 1=ACC DMA 已启动，0=设备忙或启动失败；忙和错误分别累计统计。
 * @note   运行时只由 TIM14 中断调用；不等待前一笔传输，不解包，不执行姿态或控制算法。
 * @note   本次触发的时间和序号随完整样本传递，ACC 完成后再启动 GYRO。
 */
uint8_t drv_imu_port_sample_trigger(drv_imu_port_t *p, uint32_t tick, uint32_t sequence,
                                    uint32_t cycles)
{
    if (p->state != IDLE || !ready(p) || !own(p, 1))
    {
        p->busy_count++;
        return 0;
    }
    p->sample.tick = tick;
    p->sample.sequence = sequence;
    p->sample.cycles = cycles;
    p->start_tick = bsp_tim_get_tick_ms();
    p->state = ACC;
    acc_cs(p, 0);
    if (bsp_spi_transceive_dma(p->config.spi, p->acc_tx, p->acc_rx, sizeof(p->acc_tx)) != HAL_OK)
    {
        abort_dma(p);
        return 0;
    }
    return 1;
}

/**
 * @brief  推进延后的 SPI DMA 收尾、GYRO 阶段和超时恢复。
 * @note   由 IMU 任务周期检查；短临界区与 TIM14/DMA 回调串行修改采样状态。
 * @note   进行中的样本按 HAL 毫秒时基判断是否超过 2 ms；本函数不产生新的 ACC 采样节拍。
 */
void drv_imu_port_poll(drv_imu_port_t *p)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    if (owner(p->config.spi) == p)
    {
        bsp_spi_poll_dma(p->config.spi);
    }
    if (p->state == GYRO_PENDING && ready(p))
    {
        (void)start_gyro(p);
    }
    else if (p->state != IDLE && p->state != ABORTING &&
             (uint32_t)(bsp_tim_get_tick_ms() - p->start_tick) > 2)
    {
        abort_dma(p);
    }
    else if (p->state == ABORTING && ready(p))
    {
        finished_error(p->config.spi);
    }
    __set_PRIMASK(key);
}

/**
 * @brief  读取因前一笔采样尚未空闲而跳过的硬件触发次数。
 * @return 累计忙计数，uint32_t 回绕。
 * @note   任务诊断读取；不清除计数，不改变采样状态。
 */
uint32_t drv_imu_port_get_busy_count(drv_imu_port_t *p)
{
    return p->busy_count;
}

/**
 * @brief  读取 DMA 启动、传输或超时恢复的累计错误次数。
 * @return 错误收尾累计次数，uint32_t 回绕。
 * @note   任务诊断读取；不清除计数，不改变采样状态。
 */
uint32_t drv_imu_port_get_error_count(drv_imu_port_t *p)
{
    return p->error_count;
}

/**
 * @brief  启动 TIM10_CH1 加热 PWM 输出。
 * @note   由硬件/初始化流程在需要加热时调用；本接口只启动 PWM，不实现温度闭环。
 */
void drv_imu_port_heater_start(drv_imu_port_t *p)
{
    if (p->config.heater)
    {
        bsp_tim_pwm_start(p->config.heater, p->config.heater_channel);
    }
}

/**
 * @brief  设置 TIM10_CH1 的加热 PWM 比较值。
 * @param  value        CCR 比较值，单位为定时器计数；调用者依据 ARR 确保范围正确。
 */
void drv_imu_port_heater_set(drv_imu_port_t *p, uint16_t value)
{
    if (p->config.heater)
    {
        bsp_tim_pwm_set_compare(p->config.heater, p->config.heater_channel, value);
    }
}
