/**
 * @file    bsp_tim.h
 * @brief   定时器 BSP：PWM、基础定时器中断与系统节拍接口
 */
#ifndef BSP_TIM_H
#define BSP_TIM_H

#include "bsp_cfg.h"

/** @brief 定时器更新回调表容量，单位为定时器个数；可在编译配置中覆盖。 */
#ifndef BSP_TIM_REG_MAX
#define BSP_TIM_REG_MAX 14U
#endif

#if BSP_TIM_REG_MAX < 1
#error "BSP_TIM_REG_MAX must be greater than zero"
#endif

/** @brief 定时器更新中断回调类型；在对应定时器的 ISR 上下文执行。 */
typedef void (*bsp_tim_period_callback_t)(void);

/**
 * @brief  启动 PWM 输出。
 * @param  htim     定时器句柄。
 * @param  channel  PWM 通道，如 TIM_CHANNEL_1。
 */
void bsp_tim_pwm_start(TIM_HandleTypeDef *htim, uint32_t channel);

/**
 * @brief  停止 PWM 输出。
 * @param  htim     定时器句柄。
 * @param  channel  PWM 通道。
 */
void bsp_tim_pwm_stop(TIM_HandleTypeDef *htim, uint32_t channel);

/**
 * @brief  设置 PWM 频率，并保持原有占空比。
 * @param  htim     定时器句柄。
 * @param  channel  PWM 通道。
 * @param  freq_hz  目标频率，单位 Hz。
 */
void bsp_tim_pwm_set_freq(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t freq_hz);

/**
 * @brief  设置 PWM 比较值。
 * @param  htim     定时器句柄。
 * @param  channel  PWM 通道。
 * @param  compare  比较值，范围为 0 到 ARR。
 */
void bsp_tim_pwm_set_compare(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t compare);

/**
 * @brief  启动基础定时器更新中断。
 * @param  htim  定时器句柄。
 */
void bsp_tim_it_start(TIM_HandleTypeDef *htim);

/**
 * @brief  停止基础定时器更新中断。
 * @param  htim  定时器句柄。
 */
void bsp_tim_it_stop(TIM_HandleTypeDef *htim);

/**
 * @brief  注册、更新或注销指定定时器的更新中断回调。
 * @param  htim      长期有效的定时器句柄，必须与 HAL 分发时使用的句柄一致。
 * @param  callback  更新中断处理函数；传 NULL 时仅注销此 htim 的回调。
 * @return HAL_OK=注册/更新/注销成功（未注册时注销也成功）；
 *         HAL_ERROR=htim 或其 Instance 为 NULL；HAL_BUSY=新增时注册表已满。
 * @note   不同句柄独立保存，每个句柄最多一个回调；重复注册更新该条目，不重复占槽。
 * @note   在初始化阶段或停止目标定时器中断后调用；本函数只修改注册表，不启停硬件。
 * @note   表访问使用短 IRQ 临界区；回调在临界区外的 ISR 中执行，不能延时或等待互斥量。
 */
HAL_StatusTypeDef bsp_tim_reg_callback(TIM_HandleTypeDef *htim,
                                       bsp_tim_period_callback_t callback);

/**
 * @brief  根据定时器句柄查表并调用其更新回调。
 * @param  htim      HAL 本次更新事件对应的定时器句柄；NULL 或未注册时忽略。
 * @note   由唯一的 HAL_TIM_PeriodElapsedCallback() 调用，在 ISR 上下文执行。
 */
void bsp_tim_period_elapsed_callback(TIM_HandleTypeDef *htim);

/**
 * @brief  获取 HAL 系统节拍。
 * @return 自系统启动以来的毫秒数。
 */
uint32_t bsp_tim_get_tick_ms(void);

/**
 * @brief  阻塞延时。
 * @param  ms  延时时间，单位 ms。
 */
void bsp_tim_delay_ms(uint32_t ms);

#endif
