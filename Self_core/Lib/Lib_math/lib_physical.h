/**
 * @file lib_physical.h
 * @brief SI 数值类型和边界换算；纯数学，不依赖设备、板型或调度器。
 */
#ifndef LIB_PHYSICAL_H
#define LIB_PHYSICAL_H
#include <math.h>
#include <stdint.h>

/** @brief 三维向量；单位由所属字段标注，x/y/z 顺序不变。 */
typedef struct
{
    float x;
    float y;
    float z;
} lib_vec3f_t;

/** @brief 单位四元数；q0 为实部，q1/q2/q3 对应 i/j/k，无量纲。 */
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} lib_quatf_t;

/** @brief 欧拉角，均为 rad；旋转顺序由 IMU 配置决定，不在此改变零位。 */
typedef struct
{
    float pitch;
    float roll;
    float yaw;
} lib_euler_rad_t;

#define LIB_STANDARD_GRAVITY_M_S2 9.80665f
#define LIB_RAD_PER_DEG (0.01745329251994329577f)
#define LIB_RAD_S_PER_RPM (0.1047197551196597746f)
#define LIB_RAD_PER_ENC13 (0.0007669903939428206f)

/** @brief 毫米数值换算为米。
 * @param[in] value 毫米数值；调用者负责有限性。
 * @return 米数值，保留符号。
 * @note 纯函数；同样适用于 mm/s 到 m/s，不改变时间量纲。 */
static inline float lib_mm_to_m(float value)
{
    return value * 0.001f;
}

/** @brief 米数值换算为毫米。
 * @param[in] value 米数值；调用者负责有限性。
 * @return 毫米数值，保留符号。
 * @note 仅设备/旧算法边界使用；不修改输入。 */
static inline float lib_m_to_mm(float value)
{
    return value * 1000.0f;
}

/** @brief 将 SI 量量化为有符号 16 位协议整数。
 * @param[in] value SI 数值；非有限值编码为零。
 * @param[in] per_lsb 每 LSB 的 SI 单位，必须为有限正数。
 * @return 四舍五入、饱和到 [-32768,32767] 的原始值；无效参数返回零。
 * @note 纯函数；lroundf 的半值远离零，禁止直接截断越界浮点数。 */
static inline int16_t lib_quantize_i16(float value, float per_lsb)
{
    if (!isfinite(value) || !isfinite(per_lsb) || per_lsb <= 0)
    {
        return 0;
    }
    float raw = value / per_lsb;
    if (raw >= 32767.0f)
    {
        return 32767;
    }
    if (raw <= -32768.0f)
    {
        return -32768;
    }
    return (int16_t)lroundf(raw);
}
#endif
