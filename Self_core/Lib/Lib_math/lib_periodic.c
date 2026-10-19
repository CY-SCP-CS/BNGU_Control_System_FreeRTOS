/**
 * @file    lib_periodic.c
 * @brief   绝对 tick 周期辅助：跳过已过期节拍，支持 uint32_t tick 回绕。
 */
#include "lib_periodic.h"

/**
 * @brief  若截止时间已经过期，将其直接推进到下一个未来节拍。
 * @param  now          当前 uint32_t 内核 tick。
 * @param  period       周期，单位 tick；为 0 时不推进。
 * @param  next         输入/输出：待执行截止 tick 的地址；必须非 NULL。
 * @return 跳过的截止次数；now 恰好等于 *next、尚未到期或 period=0 时返回 0。
 * @note   以有符号差值判断先后，要求所比较时间间隔小于 2^31 ticks；函数不调用任何 RTOS API。
 * @note   已过期时按 lateness/period+1 推进，不连续补算旧周期。当前时刻仍可执行与它相等的截止时间。
 */
uint32_t lib_periodic_skip_expired(uint32_t now, uint32_t period, uint32_t *next)
{
    /* uint32_t 差值再转 int32_t，可在半个计数范围内正确判断回绕后的先后。 */
    int32_t lateness = (int32_t)(now - *next);
    if (lateness <= 0 || !period)
    {
        return 0;
    }
    /* 严格越过 now；若已晚整整一周期，也跳过那个已经到期的旧节拍。 */
    uint32_t skipped = (uint32_t)lateness / period + 1;
    *next += skipped * period;
    return skipped;
}
