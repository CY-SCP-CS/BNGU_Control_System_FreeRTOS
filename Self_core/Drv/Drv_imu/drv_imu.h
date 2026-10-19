/**
 * @file    drv_imu.h
 * @brief   BMI088 六轴 IMU 驱动 (纯逻辑层)
 * @note    通过函数指针解耦 SPI 访问, 不依赖 BSP / HAL
 *          支持三种欧拉角模式, 可选择万向锁锁定的轴
 */
#ifndef DRV_IMU_H
#define DRV_IMU_H

#include "lib_typedef.h"

//少一个封装的更彻底的函数，还可以增加其他陀螺仪算法，考虑增加二阶龙格库塔法做积分
// ─── 数据类型 ─────────────────────────────────────

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} drv_imu_raw_t;

typedef struct
{
    float x;
    float y;
    float z;
} drv_imu_real_t;

typedef struct
{
    float q0; /* 实部 */
    float q1; /* 虚部 i */
    float q2; /* 虚部 j */
    float q3; /* 虚部 k */
} drv_imu_quat_t;

typedef struct
{
    float pitch; /* 俯仰角 (度) */
    float roll;  /* 横滚角 (度) */
    float yaw;   /* 偏航角 (度) */
} drv_imu_euler_t;

// ─── 欧拉角模式 ──────────────────────────────────

/**
 * @enum   drv_imu_euler_mode_t
 * @brief  欧拉角转换模式, 决定万向锁锁哪个轴
 * @note   XYZ → 锁 pitch (默认)
 *         ZYX → 锁 roll
 *         YXZ → 锁 yaw
 */
typedef enum
{
    DRV_IMU_EULER_XYZ = 0, /* 锁 pitch */
    DRV_IMU_EULER_ZYX = 1, /* 锁 roll  */
    DRV_IMU_EULER_YXZ = 2, /* 锁 yaw   */
} drv_imu_euler_mode_t;

// ─── 总线函数指针类型 ─────────────────────────────

typedef void (*drv_imu_spi_xfer_t)(void *ctx, const uint8_t *tx, uint8_t *rx, uint16_t len);
typedef void (*drv_imu_gpio_write_t)(void *ctx, uint8_t state);
typedef void (*drv_imu_delay_t)(uint32_t ms);

typedef struct
{
    void *spi_ctx;
    drv_imu_spi_xfer_t spi_xfer;

    void *acc_cs_ctx;
    drv_imu_gpio_write_t acc_cs;

    void *gyro_cs_ctx;
    drv_imu_gpio_write_t gyro_cs;

    drv_imu_delay_t delay_ms;
} drv_imu_bus_t;

// ─── IMU 句柄 ─────────────────────────────────────

typedef struct
{
    drv_imu_bus_t bus;
    drv_imu_euler_mode_t euler_mode;

    drv_imu_raw_t acc_raw;
    drv_imu_raw_t gyro_raw;

    drv_imu_real_t acc;  /* g */
    drv_imu_real_t gyro; /* rad/s */

    drv_imu_real_t gyro_offset;
    drv_imu_real_t acc_offset;

    float acc_lsb_to_g;
    float gyro_lsb_to_radps;

    drv_imu_quat_t quat;
    drv_imu_euler_t euler;
    drv_imu_euler_t euler_offset; /* 零位偏置, 由 calibrate_pose 记录 */

    float temperature;

    float integral_fb[3]; /* Mahony 积分项 */
} drv_imu_t;

// ─── 接口声明 ─────────────────────────────────────

/**
 * @brief  初始化 IMU
 * @param  imu  IMU 句柄
 * @param  bus  总线配置
 */
void drv_imu_init(drv_imu_t *imu, const drv_imu_bus_t *bus);
/**
 * @brief  启动 IMU
 * @param  imu  IMU 句柄
 */
void drv_imu_start(drv_imu_t *imu);
/**
 * @brief  设置欧拉角模式
 * @param  imu  IMU 句柄
 * @param  mode 欧拉角模式
 */
void drv_imu_set_euler_mode(drv_imu_t *imu, drv_imu_euler_mode_t mode);

/** @brief 解析 SPI 返回的 8 字节 ACC 缓冲（含命令/空字节），纯转换，不保存指针。 */
void drv_imu_parse_acc_raw(drv_imu_t *imu, const uint8_t bytes[8]);
/** @brief 解析 SPI 返回的 7 字节 GYRO 缓冲（含命令返回字节），纯转换，不保存指针。 */
void drv_imu_parse_gyro_raw(drv_imu_t *imu, const uint8_t bytes[7]);

/**
 * @brief  读取加速度计原始数据
 * @param  imu  IMU 句柄
 */
void drv_imu_read_acc_raw(drv_imu_t *imu);
/**
 * @brief  读取陀螺仪原始数据
 * @param  imu  IMU 句柄
 */
void drv_imu_read_gyro_raw(drv_imu_t *imu);
/**
 * @brief  转换 IMU 数据
 * @param  imu  IMU 句柄
 */
void drv_imu_data_convert(drv_imu_t *imu);
/**
 * @brief  读取温度数据
 * @param  imu  IMU 句柄
 */
void drv_imu_read_temp(drv_imu_t *imu);
/**
 * @brief  Mahony 四元数更新
 * @param  imu  IMU 句柄
 * @param  dt   时间间隔 (秒)
 */
void drv_imu_mahony_update(drv_imu_t *imu, float dt);
/**
 * @brief  四元数转欧拉角
 * @param  imu  IMU 句柄
 */
void drv_imu_quat_to_euler(drv_imu_t *imu);
/**
 * @brief  陀螺仪零偏标定
 * @param  imu          IMU 句柄
 * @param  sample_count 样本数量
 */
void drv_imu_calibrate_gyro(drv_imu_t *imu, uint16_t sample_count);
/**
 * @brief  姿态零位标定
 * @param  imu  IMU 句柄
 */
void drv_imu_initial_alignment(drv_imu_t *imu);
/**
 * @brief  姿态零位标定 (记录当前欧拉角为零位偏置)
 * @param  imu  IMU 句柄
 */
void drv_imu_calibrate_pose(drv_imu_t *imu);
/**
 * @brief  重启 IMU (软复位)
 * @param  imu  IMU 句柄
 */
void drv_imu_restart(drv_imu_t *imu);

// ─── Port: BSP 适配 ─────────────────────────────

/** @brief BMI088 芯片身份字节，用于区别真实设备响应与全 0/全 FF 的 DMA 数据。 */
enum
{
    DRV_IMU_ACC_CHIP_ID = 0x1E,
    DRV_IMU_GYRO_CHIP_ID = 0x0F
};

/**
 * @brief 一份完整 ACC/GYRO 原始样本，按值复制到 IMU 接收队列。
 * @note acc/gyro 保留当前解析器使用的命令占位字节；六个轴数据随后由任务提取。
 */
typedef struct
{
    uint8_t acc[8],   /**< ACC 两个占位字节 + XYZ 六字节。 */
        gyro[7],      /**< GYRO 一个占位字节 + XYZ 六字节。 */
        acc_chip_id,  /**< 本次 ACC 突发读取的芯片 ID。 */
        gyro_chip_id; /**< 本次 GYRO 突发读取的芯片 ID。 */
    uint32_t tick,    /**< 硬件触发时的内核 tick，当前 1 tick=1 ms。 */
        sequence,     /**< 触发序号，包含忙时跳过的触发，可用于诊断缺样。 */
        cycles;       /**< 硬件触发时的 DWT 周期计数，用于积分时间和延迟测量。 */
} drv_imu_dma_sample_t;

/**
 * @brief  IMU 原始样本/状态推进回调类型。
 * @note   参数非 NULL 时是一份完整样本，接收者立即复制，不保留指向驱动内部样本的地址。
 * @note   参数为 NULL 时只表示需要唤醒任务处理未完成收尾或错误；可在 ISR 或任务恢复路径执行。
 */
typedef void (*drv_imu_sample_callback_t)(const drv_imu_dma_sample_t *);
/**
 * @brief  绑定完整样本交付入口和初始化时使用的毫秒等待函数。
 * @param  callback     样本/状态回调，NULL 表示不交付。
 * @param  delay        初始化/标定延时函数；RTOS 适配传入可调度等待。
 * @note   必须在初始化和采样启动前绑定；未提供 delay 时退回 BSP 毫秒延时。
 */
void drv_imu_port_bind(drv_imu_sample_callback_t callback, drv_imu_delay_t delay);
/**
 * @brief  初始化 BMI088 (BSP 适配版)
 * @param  imu  IMU 句柄
 * @note   内部填充 drv_imu_bus_t, 挂接 SPI1 / PA4(ACC_CS) / PB0(GYRO_CS)
 * @note   配置 BMI088，完成 3000 次陀螺零偏采样及初始姿态对齐，再注册 DMA 回调。
 * @return 1=初始化期间的同步 SPI 访问未报告失败，0=至少一次同步访问失败。
 * @note   在初始化任务调用，可耗时数秒；等待通过绑定的延时函数让出 CPU。
 * @note   该返回值不等同于芯片身份验证通过，运行样本还会独立检查 CHIP_ID。
 */
uint8_t drv_imu_port_init(drv_imu_t *imu);
/**
 * @brief  接受一次硬件采样节拍并尝试启动 ACC DMA。
 * @param  tick         触发时内核 tick。
 * @param  sequence     本次触发序号。
 * @param  cycles       触发时 DWT 周期计数。
 * @return 1=ACC DMA 已启动，0=设备忙或启动失败；忙和错误分别累计统计。
 * @note   运行时只由 TIM14 中断调用；不等待前一笔传输，不解包，不执行姿态或控制算法。
 * @note   本次触发的时间和序号随完整样本传递，ACC 完成后再启动 GYRO。
 */
uint8_t drv_imu_port_sample_trigger(uint32_t tick, uint32_t sequence, uint32_t cycles);
/**
 * @brief  推进延后的 SPI DMA 收尾、GYRO 阶段和超时恢复。
 * @note   由 IMU 任务周期检查；短临界区与 TIM14/DMA 回调串行修改采样状态。
 * @note   进行中的样本按 HAL 毫秒时基判断是否超过 2 ms；本函数不产生新的 ACC 采样节拍。
 */
void drv_imu_port_poll(void);

/* Only TIM14 calls sample_trigger; completed bytes arrive through the callback. */

/** @brief 获取 DMA 忙导致的采样跳过次数。 */
uint32_t drv_imu_port_get_busy_count(void);

/** @brief 获取 DMA 启动、传输或超时错误次数。 */
uint32_t drv_imu_port_get_error_count(void);

/**
 * @brief  启动加热 PWM (TIM10_CH1)
 * @note   当前实现只经 BSP 调用 HAL_TIM_PWM_Start，不启动额外的基定时中断。
 */
void drv_imu_port_heater_start(void);

/**
 * @brief  设置加热 PWM 比较值
 * @param  val  比较值
 */
void drv_imu_port_heater_set(uint16_t val);

/**
 * @brief  延时 (毫秒)
 * @param  ms  毫秒数
 */
void drv_imu_port_delay_ms(uint32_t ms);

#endif
