/** @file app_data_age.h
 * @brief Wrap-safe age checks on the 1 kHz kernel timebase; no RTOS/HAL dependency.
 * @note Limits and real sample intervals must be below 2^31 ticks. Timestamp zero is valid.
 */
#ifndef APP_DATA_AGE_H
#define APP_DATA_AGE_H
#include <stdbool.h>
#include <stdint.h>
#include "app_feedback_data.h"
#include "app_vehicle_config.h"

static inline bool app_data_fresh(bool valid, uint32_t sampled, uint32_t now, uint32_t limit)
{
    return valid && (uint32_t)(now - sampled) <= limit;
}

static inline bool app_motor_feedback_fresh(const app_can_feedback_t *feedback, unsigned first,
                                            unsigned count, uint32_t now)
{
    for (unsigned i = first; i < first + count; ++i)
    {
        if (!app_data_fresh(feedback->motor_valid[i], feedback->motor_tick[i], now,
                            APP_INPUT_MOTOR_MAX_AGE_MS))
        {
            return false;
        }
    }
    return true;
}

static inline uint8_t app_command_fresh_mask(uint8_t valid, const uint32_t ticks[6], uint32_t now)
{
    uint8_t result = 0;
    for (unsigned i = 0; i < 6; ++i)
    {
        if (app_data_fresh(valid & (1U << i), ticks[i], now, APP_INPUT_CAN_MAX_AGE_MS))
        {
            result |= (uint8_t)(1U << i);
        }
    }
    return result;
}
#endif
