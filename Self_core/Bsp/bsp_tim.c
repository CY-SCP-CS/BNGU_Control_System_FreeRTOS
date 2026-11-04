/**
 * @file    bsp_tim.c
 * @brief   定时器 BSP 实现：PWM、更新中断注册表与分发
 */
#include "bsp_tim.h"

/** @brief 一个定时器对应一个更新回调，htim 是查找键。 */
typedef struct
{
    TIM_HandleTypeDef *htim;
    bsp_tim_period_callback_t callback;
} bsp_tim_callback_entry_t;

/** @brief 静态回调表；entries[0, count) 是有效条目，不进行动态分配。 */
typedef struct
{
    bsp_tim_callback_entry_t entries[BSP_TIM_REG_MAX];
    uint32_t count;
} bsp_tim_callback_table_t;

static bsp_tim_callback_table_t s_period_table;

/**
 * @brief  获取定时器实际输入时钟。
 * @note   当 APB 预分频不为 1 时，STM32 的定时器时钟为 APB 时钟的两倍。
 */
static uint32_t get_timer_clock(TIM_HandleTypeDef *htim)
{
    uint32_t clk;

    /* APB2：TIM1、TIM8~TIM11。 */
    switch ((uint32_t)htim->Instance)
    {
        case (uint32_t)TIM1:
        case (uint32_t)TIM8:
        case (uint32_t)TIM9:
        case (uint32_t)TIM10:
        case (uint32_t)TIM11:
            clk = HAL_RCC_GetPCLK2Freq();
            if (READ_BIT(RCC->CFGR, RCC_CFGR_PPRE2) != 0U)
            {
                clk *= 2U;
            }
            break;

        /* APB1：其余通用和基础定时器。 */
        default:
            clk = HAL_RCC_GetPCLK1Freq();
            if (READ_BIT(RCC->CFGR, RCC_CFGR_PPRE1) != 0U)
            {
                clk *= 2U;
            }
            break;
    }

    return clk;
}

void bsp_tim_pwm_start(TIM_HandleTypeDef *htim, uint32_t channel)
{
    (void)HAL_TIM_PWM_Start(htim, channel);
}

void bsp_tim_pwm_stop(TIM_HandleTypeDef *htim, uint32_t channel)
{
    (void)HAL_TIM_PWM_Stop(htim, channel);
}

void bsp_tim_pwm_set_compare(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t compare)
{
    __HAL_TIM_SET_COMPARE(htim, channel, compare);
}

void bsp_tim_pwm_set_freq(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t freq_hz)
{
    uint32_t timer_clk;
    uint32_t psc;
    uint32_t arr;
    uint32_t old_arr;
    uint32_t old_ccr;
    uint32_t new_ccr;

    if (!htim || freq_hz == 0U)
    {
        return;
    }

    timer_clk = get_timer_clock(htim);
    old_arr = __HAL_TIM_GET_AUTORELOAD(htim);
    old_ccr = __HAL_TIM_GET_COMPARE(htim, channel);

    /* 选择预分频，使 16 位 ARR 不超过 65535。 */
    psc = (timer_clk / freq_hz / 65536U) + 1U;
    arr = (timer_clk / (psc + 1U) / freq_hz) - 1U;

    /* 改频后按比例保留原有占空比。 */
    if (old_arr > 0U && old_ccr > 0U)
    {
        new_ccr = (uint32_t)((uint64_t)old_ccr * arr / old_arr);
    }
    else
    {
        new_ccr = 0U;
    }

    __HAL_TIM_SET_PRESCALER(htim, psc);
    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COMPARE(htim, channel, new_ccr);
}

void bsp_tim_it_start(TIM_HandleTypeDef *htim)
{
    (void)HAL_TIM_Base_Start_IT(htim);
}

void bsp_tim_it_stop(TIM_HandleTypeDef *htim)
{
    (void)HAL_TIM_Base_Stop_IT(htim);
}

/**
 * @brief  在静态表中登记或注销一个定时器更新回调。
 * @param  htim      与 HAL 中断分发一致、生命周期覆盖注册期的句柄。
 * @param  callback  对应处理函数，NULL 表示注销该句柄。
 * @return HAL_OK=成功；HAL_ERROR=无效句柄；HAL_BUSY=新增时表已满。
 * @note   只在初始化或停止目标定时器中断后修改注册；临界区保护查找和表项更新。
 */
HAL_StatusTypeDef bsp_tim_reg_callback(TIM_HandleTypeDef *htim,
                                       bsp_tim_period_callback_t callback)
{
    uint32_t irq_state;

    if (htim == NULL || htim->Instance == NULL)
    {
        return HAL_ERROR;
    }

    irq_state = __get_PRIMASK();
    __disable_irq();

    for (uint32_t i = 0; i < s_period_table.count; i++)
    {
        if (s_period_table.entries[i].htim == htim)
        {
            if (callback != NULL)
            {
                s_period_table.entries[i].callback = callback;
            }
            else
            {
                /* 用末项补空位，保持有效区间连续；末槽清零后可再次使用。 */
                s_period_table.count--;
                s_period_table.entries[i] = s_period_table.entries[s_period_table.count];
                s_period_table.entries[s_period_table.count] =
                    (bsp_tim_callback_entry_t){0};
            }

            __set_PRIMASK(irq_state);
            return HAL_OK;
        }
    }

    if (callback == NULL)
    {
        /* 注销不存在的句柄没有副作用，允许调用方统一执行清理。 */
        __set_PRIMASK(irq_state);
        return HAL_OK;
    }

    if (s_period_table.count >= BSP_TIM_REG_MAX)
    {
        __set_PRIMASK(irq_state);
        return HAL_BUSY;
    }

    s_period_table.entries[s_period_table.count] =
        (bsp_tim_callback_entry_t){.htim = htim, .callback = callback};
    s_period_table.count++;

    __set_PRIMASK(irq_state);
    return HAL_OK;
}

/**
 * @brief  HAL 更新事件的 BSP 分发入口，只调用与当前句柄匹配的处理函数。
 * @param  htim      产生更新事件的定时器句柄。
 * @note   临界区内仅查表并取得函数指针，恢复原 PRIMASK 后执行回调。
 * @note   本函数由 HAL 更新回调在 ISR 中调用，不负责清中断标志或维护 HAL tick。
 */
void bsp_tim_period_elapsed_callback(TIM_HandleTypeDef *htim)
{
    bsp_tim_period_callback_t callback = NULL;
    uint32_t irq_state;

    if (htim == NULL)
    {
        return;
    }

    irq_state = __get_PRIMASK();
    __disable_irq();

    for (uint32_t i = 0; i < s_period_table.count; i++)
    {
        if (s_period_table.entries[i].htim == htim)
        {
            callback = s_period_table.entries[i].callback;
            break;
        }
    }

    __set_PRIMASK(irq_state);

    if (callback != NULL)
    {
        callback();
    }
}

uint32_t bsp_tim_get_tick_ms(void)
{
    return HAL_GetTick();
}

void bsp_tim_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}
