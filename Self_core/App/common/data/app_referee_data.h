/**
 * @file    app_referee_data.h
 * @brief   裁判业务快照契约
 */
#ifndef APP_REFEREE_DATA_H
#define APP_REFEREE_DATA_H
#include <stdint.h>
#include "drv_referee.h"

/**
 * @brief  裁判任务发布的协议数据与字段更新快照
 * @note   data.data_valid_flags 保存累计有效类别，updated_mask 保存本次解析批次更新类别。
 *          field_tick[i] 对应 REFEREE_FLAG_* 的第 i 位，用于独立判断各类数据更新时间。
 */
typedef struct
{
    drv_referee_global_t data; // 裁判协议已解析数据及累计有效标志。
    uint32_t tick,             // 最近一批有效更新的发布时间戳。
        sequence,              // 发布时对应的裁判解析器帧计数。
        updated_mask;          // 本次发布有更新的 REFEREE_FLAG_* 类别位。
    uint32_t field_tick[20];   // 20 类协议数据各自最近更新的系统时间戳。
    uint8_t valid;             // 已发布过有效裁判数据的标志。
} app_ref_snapshot_t;
#endif
