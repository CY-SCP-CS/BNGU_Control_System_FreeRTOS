/** @file drv_imu_port.h
 * @brief BMI088 多实例硬件端口；实例静态 SRAM，SPI 总线同一时刻仅一笔完整 ACC/GYRO 事务。
 */
#ifndef DRV_IMU_PORT_H
#define DRV_IMU_PORT_H
#include "drv_imu.h"
#include "drv_io_event.h"
#include "bsp_spi.h"
#include "bsp_gpio.h"
#include "bsp_tim.h"

/** @brief 静态硬件配置，端口复制；片选映射与加热输出由装配提供。 */
typedef struct
{
    SPI_HandleTypeDef *spi;    /**< 借用的已初始化 SPI，含 RX/TX DMA。 */
    GPIO_TypeDef *acc_gpio;    /**< ACC 片选 GPIO，低有效。 */
    uint16_t acc_pin;          /**< GPIO pin 掩码。 */
    GPIO_TypeDef *gyro_gpio;   /**< GYRO 片选 GPIO，低有效。 */
    uint16_t gyro_pin;         /**< GPIO pin 掩码。 */
    TIM_HandleTypeDef *heater; /**< 可选加热 PWM，NULL 表示不用。 */
    uint32_t heater_channel;   /**< HAL PWM 通道常量。 */
} drv_imu_port_config_t;

/**
 * @brief 同步交付完整样本或请求恢复服务。
 * @param[in,out] context 接收者静态上下文，只透传。
 * @param[in] event DATA=完整样本；ERROR=已完成错误收尾；SERVICE=任务需推进收尾。
 * @param[in] sample DATA 时有效，仅借用至回调返回；其他事件不得解引用。
 * @return 无返回值。接收者记录队列失败。
 * @note DMA ISR 或拥有者任务调用，不阻塞，样本必须立即复制。
 */
typedef void (*drv_imu_receive_t)(void *context, drv_io_event_t event,
                                  const drv_imu_dma_sample_t *sample);

/** @brief 一个 BMI088 采集实例；DMA 缓冲仅空闲或确认中止后复用。 */
typedef struct
{
    drv_imu_port_config_t config;
    drv_imu_receive_t receive;
    void *context;
    drv_imu_delay_t delay;
    void *delay_context;
    volatile uint32_t state; /**< IDLE/ACC/GYRO_PENDING/GYRO/ABORTING，见实现状态机。 */
    uint32_t busy_count;     /**< 忙时跳过次数，可回绕。 */
    uint32_t error_count;    /**< 已确认错误收尾次数，可回绕。 */
    uint32_t start_tick;     /**< HAL 毫秒 tick，非 RTOS tick。 */
    drv_imu_dma_sample_t sample;
    uint8_t acc_tx[26]; /**< 突发 ACC 命令，DMA 普通 SRAM。 */
    uint8_t acc_rx[26];
    uint8_t gyro_tx[9];
    uint8_t gyro_rx[9];
    uint8_t init_io_failed;
} drv_imu_port_t;

/**
 * @brief 配置实例、3000 次零偏标定与初始对齐，并登记 DMA 收尾回调。
 * @param[in,out] port 静态零初始化端口，运行后不得重初始化。
 * @param[in,out] imu 算法实例，初始化后交给唯一 IMU 任务。
 * @param[in] config 硬件映射，非 NULL，配置被复制。
 * @param[in] receive 同步事件回调，非 NULL。
 * @param[in,out] context 接收者上下文，生命周期覆盖运行。
 * @param[in] delay 标定延时，可 NULL 回退 BSP。
 * @param[in,out] delay_context 延时上下文，仅透传。
 * @return 1=同步初始化未报错；0=参数、总线注册/占用或 SPI 失败。不是 CHIP_ID 校验通过。
 * @note 初始化任务串行调用，耗时数秒；不启动采样定时器。
 */
uint8_t drv_imu_port_init(drv_imu_port_t *port, drv_imu_t *imu, const drv_imu_port_config_t *config,
                          drv_imu_receive_t receive, void *context, drv_imu_delay_t delay,
                          void *delay_context);
/**
 * @brief 非阻塞接受一次硬件采样触发。
 * @param[in,out] port 已初始化实例，非 NULL。
 * @param[in] tick 内核 tick。
 * @param[in] sequence 触发序号，可回绕。
 * @param[in] cycles DWT 周期计数。
 * @return 1=ACC DMA 已启动；0=忙/失败，当前节拍跳过，不排队。
 * @note 定时器 ISR 调用；总线归本次完整事务占用，GYRO 完成或错误确认后释放。
 */
uint8_t drv_imu_port_sample_trigger(drv_imu_port_t *port, uint32_t tick, uint32_t sequence,
                                    uint32_t cycles);
/**
 * @brief 尝试一次 DMA 收尾/超时恢复。
 * @param[in,out] port 已初始化实例。
 * @return 无返回值。错误用 receive(ERROR) 和 error_count 报告。
 * @note 拥有者任务调用，短临界区，不忙等；2ms 超时先中止再确认空闲。
 */
void drv_imu_port_poll(drv_imu_port_t *port);
/** @brief 读取忙计数。
 * @param[in] port 实例，非 NULL。
 * @return uint32_t 累计次数。
 * @note 不清零；诊断近似读取。 */
uint32_t drv_imu_port_get_busy_count(drv_imu_port_t *port);
/** @brief 读取错误计数。
 * @param[in] port 实例，非 NULL。
 * @return uint32_t 累计次数。
 * @note 不清零；诊断近似读取。 */
uint32_t drv_imu_port_get_error_count(drv_imu_port_t *port);
/** @brief 启动可选加热 PWM。
 * @param[in,out] port 实例，非 NULL。
 * @return 无返回值。未配置 PWM 时忽略；硬件错误由板级诊断处理。
 * @note 初始化任务调用。 */
void drv_imu_port_heater_start(drv_imu_port_t *port);
/** @brief 更新加热 PWM 比较值。
 * @param[in,out] port 实例，非 NULL。
 * @param[in] value CCR 原始计数，调用者保证不超过 ARR。
 * @return 无返回值。未配置 PWM 时忽略。
 * @note 无温度闭环；任务调用。 */
void drv_imu_port_heater_set(drv_imu_port_t *port, uint16_t value);
#endif
