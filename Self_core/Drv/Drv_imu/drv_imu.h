/**
 * @file    drv_imu.h
 * @brief   BMI088 六轴 IMU 驱动 (纯逻辑层)
 * @note    通过函数指针解耦 SPI 访问, 不依赖 BSP / HAL
 *          支持三种欧拉角模式, 可选择万向锁锁定的轴
 */
#ifndef DRV_IMU_H
#define DRV_IMU_H

#include "lib_typedef.h"
#include "lib_physical.h"
// ─── 数据类型 ─────────────────────────────────────

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} drv_imu_raw_t;

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

/** 六轴姿态解算器；滤波器 quat 为传感器系到世界系的旋转，标量在前。 */
typedef enum
{
    DRV_IMU_ALGO_MAHONY = 0,
    DRV_IMU_ALGO_EKF = 1,
    DRV_IMU_ALGO_VQF = 2,
} drv_imu_algorithm_t;

/** 乘法误差状态 EKF 参数；噪声参数为标准差，不是方差。 */
typedef struct
{
    float gyro_noise;           /* 陀螺白噪声密度，rad/s/sqrt(Hz) */
    float bias_noise;           /* 零偏随机游走，rad/s/sqrt(s) */
    float acc_noise;            /* 归一化重力方向测量噪声，无量纲 */
    float initial_attitude_std; /* 初始姿态误差，rad */
    float initial_bias_std;     /* 初始残余零偏误差，rad/s */
    float acc_rejection;        /* |模长 - 1g| 超过此值则仅预测，g */
    float innovation_gate;      /* 三维创新的 Mahalanobis 距离平方门限 */
    float bias_limit;           /* 残余零偏各轴限幅，rad/s */
} drv_imu_ekf_config_t;

/** VQF 六轴版参数，与作者实现的参数单位保持一致。 */
typedef struct
{
    float tau_acc;              /* 惯性系加速度二阶低通时间常数，s */
    float bias_sigma_init;      /* 初始零偏不确定度，deg/s */
    float bias_forgetting_time; /* 零偏协方差遗忘时间，s */
    float bias_clip;            /* 残余零偏限幅，deg/s */
    float bias_sigma_motion;    /* 运动零偏估计不确定度，deg/s */
    float bias_vertical_forgetting_factor;
    float bias_sigma_rest;     /* 静止零偏估计不确定度，deg/s */
    float rest_min_time;       /* 静止判定所需持续时间，s */
    float rest_filter_tau;     /* 静止检测二阶低通时间常数，s */
    float rest_threshold_gyro; /* 静止角速度偏差门限，deg/s */
    float rest_threshold_acc;  /* 静止加速度偏差门限，m/s^2 */
    uint8_t motion_bias_enabled;
    uint8_t rest_bias_enabled;
} drv_imu_vqf_config_t;

typedef struct
{
    float bias[3];            /* 叠加在 gyro_offset 之后的残余零偏，rad/s */
    float covariance[6][6];   /* 状态顺序：[局部姿态误差，零偏误差] */
    float innovation_squared; /* 最近一次创新的 Mahalanobis 距离平方 */
    uint8_t acc_used;         /* 最近一次更新是否接受了重力测量 */
} drv_imu_ekf_state_t;

typedef struct
{
    lib_quatf_t gyr_quat;
    lib_quatf_t acc_quat;
    float bias[3];   /* rad/s；不写回静态 gyro_offset */
    float bias_p[9]; /* VQF 内部缩放后的零偏协方差 */
    float last_acc_lp[3];
    float rest_gyr_lp[3];
    float rest_acc_lp[3];
    float rest_deviation[2]; /* 角速度/加速度偏差的模长平方 */
    float rest_time;
    uint8_t rest_detected;

    /* 低截止频率下，二阶滤波系数和历史必须保留 double 精度。 */
    double acc_lp_state[6];
    double rest_gyr_state[6];
    double rest_acc_state[6];
    double motion_r_state[18];
    double motion_bias_state[4];
    double init_time[5];       /* 五组低通的启动平均累计时间，s */
    uint8_t lp_initialized[5]; /* 0：启动平均；1：二阶滤波历史已建立，不用 NaN 作哨兵 */
    double acc_b[3], acc_a[2];
    double rest_b[3], rest_a[2];
    float coeff_dt;
    float bias_p0, bias_v, bias_motion_w, bias_vertical_w, bias_rest_w;
} drv_imu_vqf_state_t;

// ─── 总线函数指针类型 ─────────────────────────────

typedef void (*drv_imu_spi_xfer_t)(void *ctx, const uint8_t *tx, uint8_t *rx, uint16_t len);
typedef void (*drv_imu_gpio_write_t)(void *ctx, uint8_t state);
typedef void (*drv_imu_delay_t)(void *context, uint32_t ms);

typedef struct
{
    void *spi_ctx;
    drv_imu_spi_xfer_t spi_xfer;

    void *acc_cs_ctx;
    drv_imu_gpio_write_t acc_cs;

    void *gyro_cs_ctx;
    drv_imu_gpio_write_t gyro_cs;

    void *delay_ctx; /**< 延时上下文，借用到设备停止。 */
    drv_imu_delay_t delay_ms;
} drv_imu_bus_t;

// ─── IMU 句柄 ─────────────────────────────────────

typedef struct
{
    drv_imu_bus_t bus;
    drv_imu_euler_mode_t euler_mode;

    drv_imu_raw_t acc_raw;
    drv_imu_raw_t gyro_raw;

    lib_vec3f_t acc;  /* g */
    lib_vec3f_t gyro; /* rad/s */

    lib_vec3f_t gyro_offset;
    lib_vec3f_t acc_offset;

    float acc_lsb_to_g;
    float gyro_lsb_to_radps;

    lib_quatf_t quat;             /* 滤波器姿态：传感器系 -> 世界系，不含安装补偿 */
    lib_quatf_t mounting_quat;    /* 固定安装旋转：传感器系 -> 等效水平机体系 */
    drv_imu_euler_t euler;        /* 安装补偿后的机体系姿态，单位度 */
    drv_imu_euler_t euler_offset; /* 零位偏置, 由 calibrate_pose 记录 */

    float temperature;

    float integral_fb[3]; /* Mahony 积分项 */

    drv_imu_algorithm_t algorithm;
    drv_imu_ekf_config_t ekf_config;
    drv_imu_vqf_config_t vqf_config;

    union
    {
        drv_imu_ekf_state_t ekf;
        drv_imu_vqf_state_t vqf;
    } filter; /* 仅保存当前算法的状态，无动态内存 */
} drv_imu_t;

/** @brief 解包 ACC 原始 SPI 帧；不保留缓冲地址。 */
void drv_imu_parse_acc_raw(drv_imu_t *imu, const uint8_t bytes[8]);
/** @brief 解包 GYRO 原始 SPI 帧；不保留缓冲地址。 */
void drv_imu_parse_gyro_raw(drv_imu_t *imu, const uint8_t bytes[7]);

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

/** 设置固定安装角，单位度；q_mount = qz(yaw) * qy(pitch) * qx(roll)。
 *  描述传感器系到等效水平机体系的旋转；水平时传感器 roll=+90 度则设 roll=+90。
 *  初始化默认无安装旋转；成功返回 1，非有限参数返回 0 且不修改实例。
 *  只改变输出坐标，不重置三种滤波器；应在 AHRS 所在上下文或暂停更新时配置。
 */
uint8_t drv_imu_set_mounting(drv_imu_t *imu, float roll_deg, float pitch_deg, float yaw_deg);
/** 获取安装补偿后的机体系 -> 世界系四元数：q_body = quat * conjugate(q_mount)。 */
void drv_imu_get_body_quat(const drv_imu_t *imu, lib_quatf_t *out);
/** 获取机体系角速度(rad/s)：扣除 gyro_offset，再应用当前算法的在线零偏补偿。
 *  EKF/VQF 减去 filter.bias；Mahony 加上 integral_fb，最后旋转到安装补偿后的机体系。
 *  不改写原始 gyro、零偏估计或滤波状态。
 */
void drv_imu_get_body_gyro(const drv_imu_t *imu, lib_vec3f_t *out);

/** 切换算法并重置其内部状态，保留当前姿态及静态零偏；成功返回 1。
 *  同一种算法重复设置不会重置。需在 AHRS 更新所在上下文或停用更新时调用。
 */
uint8_t drv_imu_set_algorithm(drv_imu_t *imu, drv_imu_algorithm_t algorithm);
/** 清除当前算法的历史、协方差和在线零偏，保留当前四元数与配置。 */
void drv_imu_reset_filter(drv_imu_t *imu);
/** 统一姿态更新入口；输入取自 acc(g)、gyro(rad/s)，dt 单位 s。
 *  有效 dt 范围 (0, 0.1]；无效 dt/陀螺数据跳过更新。
 *  不包含采样、单位换算和欧拉角转换。
 */
void drv_imu_update(drv_imu_t *imu, float dt);

void drv_imu_ekf_default_config(drv_imu_ekf_config_t *config);
void drv_imu_vqf_default_config(drv_imu_vqf_config_t *config);
/** 校验并应用配置；无效参数返回 0，当前算法匹配时重置滤波历史。
 *  config=NULL 恢复默认值。应在暂停 AHRS 更新时调用。
 */
uint8_t drv_imu_ekf_configure(drv_imu_t *imu, const drv_imu_ekf_config_t *config);
uint8_t drv_imu_vqf_configure(drv_imu_t *imu, const drv_imu_vqf_config_t *config);
/** 直接调用会自动切换到相应算法，参数约定同 drv_imu_update。 */
void drv_imu_ekf_update(drv_imu_t *imu, float dt);
void drv_imu_vqf_update(drv_imu_t *imu, float dt);

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
 * @brief  传感器四元数先做固定安装补偿，再转换机体系欧拉角；不修改滤波器四元数。
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

#endif
