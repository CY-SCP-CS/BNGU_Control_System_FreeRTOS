/** @file legacy_adapter.h
 * @brief SI -> 旧算法数值适配，仅控制模块内部使用；不改变 PID 参数。
 */
#ifndef LEGACY_ADAPTER_H
#define LEGACY_ADAPTER_H
#include "legacy_control_types.h"
#include <string.h>

/** @brief 恢复旧算法需要的编码器、RPM 与功率计原始尺度。
 * @param[in] src 完整 SI 反馈副本，非 NULL，不保留地址。
 * @param[out] dst 完整旧尺度反馈，非 NULL，全部覆盖。
 * @return 无返回值。调用者保证输入来自已校验设备边界。
 * @note 纯同步函数；无锁；由各控制任务独占自己的目标缓冲。 */
static inline void legacy_feedback(const app_can_feedback_t *src, legacy_can_feedback_t *dst)
{
    memset(dst, 0, sizeof(*dst));
    for (unsigned i = 0; i < GIMBAL_MOTOR_COUNT; i++)
    {
        dst->motor[i].angle = (uint16_t)lroundf(src->motor[i].angle_rad / LIB_RAD_PER_ENC13);
        dst->motor[i].speed = lib_quantize_i16(src->motor[i].speed_rad_s, LIB_RAD_S_PER_RPM);
        dst->motor[i].current = src->motor[i].effort_raw;
        dst->motor[i].temperature = (uint8_t)src->motor[i].temperature_c;
        dst->motor_tick[i] = src->motor_tick[i];
        dst->motor_valid[i] = src->motor_valid[i];
    }
    dst->power.bat_v = (uint16_t)lroundf(src->power.voltage_v * 100.0f);
    dst->power.bat_i = (uint16_t)lroundf(src->power.current_a * 100.0f);
    dst->power.power = src->power.power_w;
    dst->power_tick = src->power_tick;
    dst->power_valid = src->power_valid;
    dst->omega = src->omega;
    dst->omega_tick = src->omega_tick;
    dst->omega_valid = src->omega_valid;
}
#endif
