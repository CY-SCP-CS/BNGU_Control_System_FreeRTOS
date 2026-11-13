/** @file app_referee_data.h
 * @brief 裁判业务投影；完整协议记录仅由裁判模块解析器保留。
 */
#ifndef APP_REFEREE_DATA_H
#define APP_REFEREE_DATA_H
#include <stdint.h>

/** @brief 裁判任务维护，ref_to_chas_mutex 保护；消费者只读完整副本。 */
typedef struct
{
    uint32_t valid_mask;        /**< 已接收过的协议类别位，与 updated_mask 共用固定 20 位布局。 */
    uint32_t tick;              /**< 最近 CRC 正确帧处理时内核 tick；不是 DMA 触发时间。 */
    uint32_t sequence;          /**< 本地累计完整帧数，可回绕。 */
    uint32_t updated_mask;      /**< 本批实际更新类别，保留原 REFEREE_FLAG 位数值。 */
    uint32_t field_tick[20];    /**< 各协议类别最近更新 tick，bit i 对应下标 i。 */
    uint8_t robot_id;           /**< 机器人协议 ID，未更新时为零。 */
    float power_limit_w;        /**< 裁判功率上限 W；当前控制仍使用原固定 100 W。 */
    float buffer_energy_j;      /**< 缓冲能量 J。 */
    uint16_t heat_17_raw;       /**< 17mm 枪口协议热量计数，不将其误标 J。 */
    uint16_t heat_42_raw;       /**< 42mm 枪口协议热量计数，不将其误标 J。 */
    uint16_t heat_limit_raw;    /**< Current robot shooter cooling limit, protocol heat count. */
    uint16_t projectile_42_raw; /**< Remaining 42 mm allowance, projectile count. */
    uint8_t valid;              /**< 曾发布过 CRC 正确帧；各字段有效性还须检查 valid_mask。 */
} app_ref_snapshot_t;
#endif
