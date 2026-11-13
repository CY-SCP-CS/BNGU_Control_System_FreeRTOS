/**
 * @file    drv_referee.c
 * @brief   裁判系统协议解析 (RoboMaster 2026 官方协议)
 * @note    SOF=0xA5 帧协议
 *          CRC8(多项式0x31)校验帧头, CRC16(多项式0x1021)校验整帧
 *          协议定义参考: D:/STM32project/referee_system/MDK-ARM/referee_system_data.c
 */
#include "drv_referee.h"

#include <string.h>

_Static_assert(sizeof(drv_referee_frame_header_t) == PROTOCOL_HEADER_LEN,
               "Referee header must be packed");
_Static_assert(sizeof(drv_referee_game_status_t) == 11U, "Referee game status must be packed");
_Static_assert(sizeof(drv_referee_robot_status_t) == 13U, "Referee robot status must be packed");

/* ════════════════════════════════════════════════════
 * CRC 查表与计算
 * ════════════════════════════════════════════════════ */

/**
 * CRC8 配置
 * - 多项式: x^8 + x^5 + x^4 + 1  (0x31)
 * - 初值: 0xFF
 * - 按最低有效位优先计算（反射实现）
 * 用于校验帧头前 4 字节 (SOF + data_length + seq)
 */
#define DRV_REFEREE_CRC8_INIT 0xFF

static const uint8_t s_referee_crc8_table[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35};

/**
 * CRC16 配置
 * - 多项式: x^16 + x^12 + x^5 + 1  (0x1021)
 * - 初值: 0xFFFF
 * - 按最低有效位优先计算（反射实现）
 * 用于校验从 SOF 开始、除末尾 CRC16 外的整帧数据
 */
#define DRV_REFEREE_CRC16_INIT 0xFFFF

/* ── CRC 计算函数 ──────────────────────────────── */

/**
 * @brief  计算裁判帧头 CRC8，初值 0xFF，使用反射查表。
 * @param  data         只读待校验数据，必须有至少 len 字节。
 * @param  len          参与校验的字节数。
 * @return CRC8 结果；帧头校验时 len 为前 4 字节。
 */
static uint8_t calc_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = DRV_REFEREE_CRC8_INIT;
    while (len--)
    {
        crc = s_referee_crc8_table[crc ^ *data++];
    }
    return crc;
}

/**
 * @brief  计算裁判整帧 CRC16，初值 0xFFFF，反射多项式为 0x8408。
 * @param  data         只读待校验数据，必须有至少 len 字节。
 * @param  len          参与校验的字节数。
 * @return CRC16 结果；整帧校验时不包含末尾两个 CRC 字节。
 */
static uint16_t calc_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = DRV_REFEREE_CRC16_INIT;
    while (len--)
    {
        crc ^= *data++;
        for (int i = 0; i < 8; i++)
        {
            crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0x8408U) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

/* ════════════════════════════════════════════════════
 * 协议缓存与全局数据
 * ════════════════════════════════════════════════════ */

/**
 * @brief  裁判协议解析器的私有状态，由 comm_ref_Task 独占。
 * @note   只包含协议数据与普通内存，不依赖 HAL 或 RTOS 对象。
 * @note   result 是本次 process 的结果；data_valid_flags 另行累计，供旧查询接口消费。
 */

/* ════════════════════════════════════════════════════
 * 各命令处理器 (按 CMD_ID 分派)
 * ════════════════════════════════════════════════════ */

/* ── 0x0001 比赛状态 ──────────────────────────── */
/**
 * @brief  保存比赛状态载荷，并置 REFEREE_FLAG_GAME_STATUS 更新位。
 * @param  data         已经过帧 CRC 校验的数据段地址，不包含帧头、命令码或帧尾。
 * @param  len          数据段长度，小于对应结构体大小时直接忽略，保留旧值。
 * @note   仅由裁判任务内的解析器调用；复制协议结构所需字节，额外载荷不保存。
 */
static void handle_game_status(drv_referee_context_t *ctx, const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_game_status_t))
    {
        return;
    }
    memcpy(&ctx->s_referee_data.game_status, data, sizeof(drv_referee_game_status_t));
    ctx->result.updated_mask |= REFEREE_FLAG_GAME_STATUS;
}

/* ── 0x0002 比赛结果 ──────────────────────────── */
/**
 * @brief  保存比赛结果载荷，并置 REFEREE_FLAG_GAME_RESULT 更新位。
 * @param  data         已经过帧 CRC 校验的数据段地址，不包含帧头、命令码或帧尾。
 * @param  len          数据段长度，小于对应结构体大小时直接忽略，保留旧值。
 * @note   仅由裁判任务内的解析器调用；复制协议结构所需字节，额外载荷不保存。
 */
static void handle_game_result(drv_referee_context_t *ctx, const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_game_result_t))
    {
        return;
    }
    memcpy(&ctx->s_referee_data.game_result, data, sizeof(drv_referee_game_result_t));
    ctx->result.updated_mask |= REFEREE_FLAG_GAME_RESULT;
}

/* ── 0x0003 全场机器人血量 ───────────────────── */
/**
 * @brief  保存全场机器人血量载荷，并置 REFEREE_FLAG_GAME_ROBOT_HP 更新位。
 * @param  data         已经过帧 CRC 校验的数据段地址，不包含帧头、命令码或帧尾。
 * @param  len          数据段长度，小于对应结构体大小时直接忽略，保留旧值。
 * @note   仅由裁判任务内的解析器调用；复制协议结构所需字节，额外载荷不保存。
 */
static void handle_game_robot_hp(drv_referee_context_t *ctx, const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_game_robot_hp_t))
    {
        return;
    }
    memcpy(&ctx->s_referee_data.game_robot_hp, data, sizeof(drv_referee_game_robot_hp_t));
    ctx->result.updated_mask |= REFEREE_FLAG_GAME_ROBOT_HP;
}

/* ── 0x0101 场地事件 ──────────────────────────── */
/**
 * @brief  保存场地事件载荷，并置 REFEREE_FLAG_EVENT_DATA 更新位。
 * @param  data         已经过帧 CRC 校验的数据段地址，不包含帧头、命令码或帧尾。
 * @param  len          数据段长度，小于对应结构体大小时直接忽略，保留旧值。
 * @note   仅由裁判任务内的解析器调用；复制协议结构所需字节，额外载荷不保存。
 */
static void handle_event_data(drv_referee_context_t *ctx, const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_event_data_t))
    {
        return;
    }
    memcpy(&ctx->s_referee_data.event_data, data, sizeof(drv_referee_event_data_t));
    ctx->result.updated_mask |= REFEREE_FLAG_EVENT_DATA;
}

/* ── 0x0104 裁判警告 ──────────────────────────── */
/**
 * @brief  保存裁判警告载荷，并置 REFEREE_FLAG_REFEREE_WARNING 更新位。
 * @param  data         已经过帧 CRC 校验的数据段地址，不包含帧头、命令码或帧尾。
 * @param  len          数据段长度，小于对应结构体大小时直接忽略，保留旧值。
 * @note   仅由裁判任务内的解析器调用；复制协议结构所需字节，额外载荷不保存。
 */
static void handle_referee_warning(drv_referee_context_t *ctx, const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_warning_t))
    {
        return;
    }
    memcpy(&ctx->s_referee_data.referee_warning, data, sizeof(drv_referee_warning_t));
    ctx->result.updated_mask |= REFEREE_FLAG_REFEREE_WARNING;
}

/* ── 0x0105 飞镖发射信息 ──────────────────────── */
/**
 * @brief  保存飞镖发射信息载荷，并置 REFEREE_FLAG_DART_INFO 更新位。
 * @param  data         已经过帧 CRC 校验的数据段地址，不包含帧头、命令码或帧尾。
 * @param  len          数据段长度，小于对应结构体大小时直接忽略，保留旧值。
 * @note   仅由裁判任务内的解析器调用；复制协议结构所需字节，额外载荷不保存。
 */
static void handle_dart_info(drv_referee_context_t *ctx, const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_dart_info_t))
    {
        return;
    }
    memcpy(&ctx->s_referee_data.dart_info, data, sizeof(drv_referee_dart_info_t));
    ctx->result.updated_mask |= REFEREE_FLAG_DART_INFO;
}

/* ── 0x0201 机器人性能状态 ───────────────────── */
/**
 * @brief  保存机器人性能状态载荷，并置 REFEREE_FLAG_ROBOT_STATUS 更新位。
 * @param  data         已经过帧 CRC 校验的数据段地址，不包含帧头、命令码或帧尾。
 * @param  len          数据段长度，小于对应结构体大小时直接忽略，保留旧值。
 * @note   仅由裁判任务内的解析器调用；复制协议结构所需字节，额外载荷不保存。
 */
static void handle_robot_status(drv_referee_context_t *ctx, const uint8_t *data, uint16_t len)
{
    drv_referee_robot_status_t robot_status;

    if (len < sizeof(robot_status))
    {
        return;
    }
    memcpy(&robot_status, data, sizeof(robot_status));

    /* 此处仅更新裁判任务私有解析状态；跨任务发布由裁判任务复制到互斥快照。 */
    ctx->s_referee_data.robot_status = robot_status;
    ctx->result.updated_mask |= REFEREE_FLAG_ROBOT_STATUS;
}

/* ── 0x0202 功率热量数据 ──────────────────────── */
/**
 * @brief  保存功率与发射热量载荷，并置 REFEREE_FLAG_POWER_HEAT 更新位。
 * @param  data         已经过帧 CRC 校验的数据段地址，不包含帧头、命令码或帧尾。
 * @param  len          数据段长度，小于对应结构体大小时直接忽略，保留旧值。
 * @note   仅由裁判任务内的解析器调用；复制协议结构所需字节，额外载荷不保存。
 */
static void handle_power_heat(drv_referee_context_t *ctx, const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_power_heat_t))
    {
        return;
    }
    memcpy(&ctx->s_referee_data.power_heat_data, data, sizeof(drv_referee_power_heat_t));
    ctx->result.updated_mask |= REFEREE_FLAG_POWER_HEAT;
}

/* ── 0x0203 机器人位置 ────────────────────────── */
/**
 * @brief  保存机器人位置载荷，并置 REFEREE_FLAG_ROBOT_POS 更新位。
 * @param  data         已经过帧 CRC 校验的数据段地址，不包含帧头、命令码或帧尾。
 * @param  len          数据段长度，小于对应结构体大小时直接忽略，保留旧值。
 * @note   仅由裁判任务内的解析器调用；复制协议结构所需字节，额外载荷不保存。
 */
static void handle_robot_pos(drv_referee_context_t *ctx, const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_robot_pos_t))
    {
        return;
    }
    memcpy(&ctx->s_referee_data.robot_pos, data, sizeof(drv_referee_robot_pos_t));
    ctx->result.updated_mask |= REFEREE_FLAG_ROBOT_POS;
}

/* ── 0x0204 增益/减益状态 ─────────────────────── */
/**
 * @brief  保存增益/减益状态载荷，并置 REFEREE_FLAG_BUFF 更新位。
 * @param  data         已经过帧 CRC 校验的数据段地址，不包含帧头、命令码或帧尾。
 * @param  len          数据段长度，小于对应结构体大小时直接忽略，保留旧值。
 * @note   仅由裁判任务内的解析器调用；复制协议结构所需字节，额外载荷不保存。
 */
static void handle_buff(drv_referee_context_t *ctx, const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_buff_t))
    {
        return;
    }
    memcpy(&ctx->s_referee_data.buff, data, sizeof(drv_referee_buff_t));
    ctx->result.updated_mask |= REFEREE_FLAG_BUFF;
}

/* ── 0x0205 空中机器人能量 ────────────────────── */
/**
 * @brief  保存空中支援状态载荷，并置 REFEREE_FLAG_AIR_SUPPORT 更新位。
 * @param  data         已经过帧 CRC 校验的数据段地址，不包含帧头、命令码或帧尾。
 * @param  len          数据段长度，小于对应结构体大小时直接忽略，保留旧值。
 * @note   仅由裁判任务内的解析器调用；复制协议结构所需字节，额外载荷不保存。
 */
static void handle_air_support(drv_referee_context_t *ctx, const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_air_support_data_t))
    {
        return;
    }
    memcpy(&ctx->s_referee_data.air_support, data, sizeof(drv_referee_air_support_data_t));
    ctx->result.updated_mask |= REFEREE_FLAG_AIR_SUPPORT;
}

/* ── 0x0206 伤害状态 ──────────────────────────── */
/**
 * @brief  保存伤害状态载荷，并置 REFEREE_FLAG_HURT_DATA 更新位。
 * @param  data         已经过帧 CRC 校验的数据段地址，不包含帧头、命令码或帧尾。
 * @param  len          数据段长度，小于对应结构体大小时直接忽略，保留旧值。
 * @note   仅由裁判任务内的解析器调用；复制协议结构所需字节，额外载荷不保存。
 */
static void handle_hurt_data(drv_referee_context_t *ctx, const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_hurt_data_t))
    {
        return;
    }
    memcpy(&ctx->s_referee_data.hurt_data, data, sizeof(drv_referee_hurt_data_t));
    ctx->result.updated_mask |= REFEREE_FLAG_HURT_DATA;
}

/* ── 0x0207 实时射击数据 ──────────────────────── */
/**
 * @brief  保存实时射击数据载荷，并置 REFEREE_FLAG_SHOOT_DATA 更新位。
 * @param  data         已经过帧 CRC 校验的数据段地址，不包含帧头、命令码或帧尾。
 * @param  len          数据段长度，小于对应结构体大小时直接忽略，保留旧值。
 * @note   仅由裁判任务内的解析器调用；复制协议结构所需字节，额外载荷不保存。
 */
static void handle_shoot_data(drv_referee_context_t *ctx, const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_shoot_data_t))
    {
        return;
    }
    memcpy(&ctx->s_referee_data.shoot_data, data, sizeof(drv_referee_shoot_data_t));
    ctx->result.updated_mask |= REFEREE_FLAG_SHOOT_DATA;
}

/* ── 0x0208 弹丸许可量 ────────────────────────── */
/**
 * @brief  保存弹丸许可量载荷，并置 REFEREE_FLAG_PROJECTILE_ALLOWANCE 更新位。
 * @param  data         已经过帧 CRC 校验的数据段地址，不包含帧头、命令码或帧尾。
 * @param  len          数据段长度，小于对应结构体大小时直接忽略，保留旧值。
 * @note   仅由裁判任务内的解析器调用；复制协议结构所需字节，额外载荷不保存。
 */
static void handle_projectile_allowance(drv_referee_context_t *ctx, const uint8_t *data,
                                        uint16_t len)
{
    if (len < sizeof(drv_referee_projectile_allowance_t))
    {
        return;
    }
    memcpy(&ctx->s_referee_data.projectile, data, sizeof(drv_referee_projectile_allowance_t));
    ctx->result.updated_mask |= REFEREE_FLAG_PROJECTILE_ALLOWANCE;
}

/* ── 0x0209 RFID状态 ──────────────────────────── */
/**
 * @brief  保存RFID 状态载荷，并置 REFEREE_FLAG_RFID_STATUS 更新位。
 * @param  data         已经过帧 CRC 校验的数据段地址，不包含帧头、命令码或帧尾。
 * @param  len          数据段长度，小于对应结构体大小时直接忽略，保留旧值。
 * @note   仅由裁判任务内的解析器调用；复制协议结构所需字节，额外载荷不保存。
 */
static void handle_rfid_status(drv_referee_context_t *ctx, const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_rfid_status_t))
    {
        return;
    }
    memcpy(&ctx->s_referee_data.rfid_status, data, sizeof(drv_referee_rfid_status_t));
    ctx->result.updated_mask |= REFEREE_FLAG_RFID_STATUS;
}

/* ── 0x020A 飞镖选手端指令 ────────────────────── */
/**
 * @brief  保存飞镖选手端指令载荷，并置 REFEREE_FLAG_DART_CLIENT_CMD 更新位。
 * @param  data         已经过帧 CRC 校验的数据段地址，不包含帧头、命令码或帧尾。
 * @param  len          数据段长度，小于对应结构体大小时直接忽略，保留旧值。
 * @note   仅由裁判任务内的解析器调用；复制协议结构所需字节，额外载荷不保存。
 */
static void handle_dart_client_cmd(drv_referee_context_t *ctx, const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_dart_client_cmd_t))
    {
        return;
    }
    memcpy(&ctx->s_referee_data.dart_client_cmd, data, sizeof(drv_referee_dart_client_cmd_t));
    ctx->result.updated_mask |= REFEREE_FLAG_DART_CLIENT_CMD;
}

/* ── 0x020B 地面机器人位置 ────────────────────── */
/**
 * @brief  保存地面机器人位置载荷，并置 REFEREE_FLAG_GROUND_ROBOT_POS 更新位。
 * @param  data         已经过帧 CRC 校验的数据段地址，不包含帧头、命令码或帧尾。
 * @param  len          数据段长度，小于对应结构体大小时直接忽略，保留旧值。
 * @note   仅由裁判任务内的解析器调用；复制协议结构所需字节，额外载荷不保存。
 */
static void handle_ground_robot_pos(drv_referee_context_t *ctx, const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_ground_robot_pos_t))
    {
        return;
    }
    memcpy(&ctx->s_referee_data.ground_robot_pos, data, sizeof(drv_referee_ground_robot_pos_t));
    ctx->result.updated_mask |= REFEREE_FLAG_GROUND_ROBOT_POS;
}

/* ── 0x020C 雷达标记进度 ──────────────────────── */
/**
 * @brief  保存雷达标记进度载荷，并置 REFEREE_FLAG_RADAR_MARK 更新位。
 * @param  data         已经过帧 CRC 校验的数据段地址，不包含帧头、命令码或帧尾。
 * @param  len          数据段长度，小于对应结构体大小时直接忽略，保留旧值。
 * @note   仅由裁判任务内的解析器调用；复制协议结构所需字节，额外载荷不保存。
 */
static void handle_radar_mark(drv_referee_context_t *ctx, const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_radar_mark_data_t))
    {
        return;
    }
    memcpy(&ctx->s_referee_data.radar_mark, data, sizeof(drv_referee_radar_mark_data_t));
    ctx->result.updated_mask |= REFEREE_FLAG_RADAR_MARK;
}

/* ── 0x020D 哨兵信息 ──────────────────────────── */
/**
 * @brief  保存哨兵自主决策信息载荷，并置 REFEREE_FLAG_SENTRY_INFO 更新位。
 * @param  data         已经过帧 CRC 校验的数据段地址，不包含帧头、命令码或帧尾。
 * @param  len          数据段长度，小于对应结构体大小时直接忽略，保留旧值。
 * @note   仅由裁判任务内的解析器调用；复制协议结构所需字节，额外载荷不保存。
 */
static void handle_sentry_info(drv_referee_context_t *ctx, const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_sentry_info_t))
    {
        return;
    }
    memcpy(&ctx->s_referee_data.sentry_info, data, sizeof(drv_referee_sentry_info_t));
    ctx->result.updated_mask |= REFEREE_FLAG_SENTRY_INFO;
}

/* ── 0x020E 雷达站信息 ────────────────────────── */
/**
 * @brief  保存雷达状态载荷，并置 REFEREE_FLAG_RADAR_INFO 更新位。
 * @param  data         已经过帧 CRC 校验的数据段地址，不包含帧头、命令码或帧尾。
 * @param  len          数据段长度，小于对应结构体大小时直接忽略，保留旧值。
 * @note   仅由裁判任务内的解析器调用；复制协议结构所需字节，额外载荷不保存。
 */
static void handle_radar_info(drv_referee_context_t *ctx, const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_radar_info_t))
    {
        return;
    }
    memcpy(&ctx->s_referee_data.radar_info, data, sizeof(drv_referee_radar_info_t));
    ctx->result.updated_mask |= REFEREE_FLAG_RADAR_INFO;
}

/* ── 0x0301 机器人交互 (含UI绘制) ────────────────
 * 结构: data_cmd_id(2) + sender_id(2) + receiver_id(2) + user_data(N)
 * 当前仅做解析框架, 具体交互处理留待后续实现
 */
/**
 * @brief  读取 0x0301 的交互子命令并分派到预留分支。
 * @param  data         数据命令 ID、发送者、接收者及用户数据组成的载荷。
 * @param  len          载荷长度，至少 6 字节。
 * @note   当前 UI、哨兵和雷达交互分支仅有框架，不发布业务字段或更新位。
 */
static void handle_robot_interaction(drv_referee_context_t *ctx, const uint8_t *data, uint16_t len)
{
    (void)ctx;
    if (len < 6)
    {
        return; /* 至少需要 data_cmd_id + sender_id + receiver_id */
    }

    uint16_t sub_cmd_id;
    memcpy(&sub_cmd_id, data, sizeof(sub_cmd_id)); /* 前2字节为子命令ID */

    uint16_t user_data_len = len - 6;

    switch (sub_cmd_id)
    {
        case SUB_CMD_LAYER_DELETE:
            /* 删除图层, 暂不处理 */
            break;
        case SUB_CMD_DRAW_ONE_GRAPHIC:
            /* 绘制1个图形, 暂不处理 */
            break;
        case SUB_CMD_DRAW_TWO_GRAPHIC:
            /* 绘制2个图形, 暂不处理 */
            break;
        case SUB_CMD_DRAW_FIVE_GRAPHIC:
            /* 绘制5个图形, 暂不处理 */
            break;
        case SUB_CMD_DRAW_SEVEN_GRAPHIC:
            /* 绘制7个图形, 暂不处理 */
            break;
        case SUB_CMD_SENTRY_CMD:
            if (user_data_len >= sizeof(drv_referee_sentry_cmd_t))
            {
                /* 哨兵命令, 暂不处理 */
            }
            break;
        case SUB_CMD_RADAR_CMD:
            if (user_data_len >= sizeof(drv_referee_radar_cmd_t))
            {
                /* 雷达命令, 暂不处理 */
            }
            break;
        default:
            break;
    }
}

/* ════════════════════════════════════════════════════
 * 已校验完整帧的命令分派
 * ════════════════════════════════════════════════════ */

/**
 * @brief  按命令码分派一份已经通过帧校验的完整裁判数据。
 * @param  buffer  完整帧地址，必须由 drv_referee_process(ctx) 先验证 SOF、长度和 CRC。
 * @param  length  已验证的完整帧长度，范围 PROTOCOL_MIN_FRAME_LEN~REFEREE_RX_BUF_SIZE。
 * @return 0=已知命令完成分派，-2=未知命令；未知命令仍计为 CRC 正确的完整帧。
 * @note   帧头和全帧校验统一在流式入口执行，此处不重复计算 CRC。
 * @note   过短的业务载荷由各 handle_* 忽略；实际字段更新以 result.updated_mask 为准。
 */
static int parse_referee_packet(drv_referee_context_t *ctx, const uint8_t *buffer, uint32_t length)
{
    const uint8_t *cmd_ptr = buffer + PROTOCOL_HEADER_LEN; /* 命令码位置 */
    uint16_t cmd_id;
    memcpy(&cmd_id, cmd_ptr, sizeof(cmd_id));             /* 小端序直接读取 */
    const uint8_t *data_ptr = cmd_ptr + PROTOCOL_CMD_LEN; /* 数据段位置 */
    uint16_t data_len = (uint16_t)(length - PROTOCOL_MIN_FRAME_LEN);

    /* 完整帧已经校验，只检查各命令所需的业务载荷长度。 */
    switch (cmd_id)
    {
        case CMD_GAME_STATUS:
            handle_game_status(ctx, data_ptr, data_len);
            break;
        case CMD_GAME_RESULT:
            handle_game_result(ctx, data_ptr, data_len);
            break;
        case CMD_GAME_ROBOT_HP:
            handle_game_robot_hp(ctx, data_ptr, data_len);
            break;
        case CMD_EVENT_DATA:
            handle_event_data(ctx, data_ptr, data_len);
            break;
        case CMD_REFEREE_WARNING:
            handle_referee_warning(ctx, data_ptr, data_len);
            break;
        case CMD_DART_INFO:
            handle_dart_info(ctx, data_ptr, data_len);
            break;
        case CMD_ROBOT_STATUS:
            handle_robot_status(ctx, data_ptr, data_len);
            break;
        case CMD_POWER_HEAT_DATA:
            handle_power_heat(ctx, data_ptr, data_len);
            break;
        case CMD_ROBOT_POS:
            handle_robot_pos(ctx, data_ptr, data_len);
            break;
        case CMD_BUFF_DATA:
            handle_buff(ctx, data_ptr, data_len);
            break;
        case CMD_AIR_SUPPORT_DATA:
            handle_air_support(ctx, data_ptr, data_len);
            break;
        case CMD_HURT_DATA:
            handle_hurt_data(ctx, data_ptr, data_len);
            break;
        case CMD_SHOOT_DATA:
            handle_shoot_data(ctx, data_ptr, data_len);
            break;
        case CMD_PROJECTILE_ALLOWANCE:
            handle_projectile_allowance(ctx, data_ptr, data_len);
            break;
        case CMD_RFID_STATUS:
            handle_rfid_status(ctx, data_ptr, data_len);
            break;
        case CMD_DART_CLIENT_CMD:
            handle_dart_client_cmd(ctx, data_ptr, data_len);
            break;
        case CMD_GROUND_ROBOT_POS:
            handle_ground_robot_pos(ctx, data_ptr, data_len);
            break;
        case CMD_RADAR_MARK_DATA:
            handle_radar_mark(ctx, data_ptr, data_len);
            break;
        case CMD_SENTRY_INFO:
            handle_sentry_info(ctx, data_ptr, data_len);
            break;
        case CMD_RADAR_INFO:
            handle_radar_info(ctx, data_ptr, data_len);
            break;
        case CMD_ROBOT_INTERACTION:
            handle_robot_interaction(ctx, data_ptr, data_len);
            break;
        default:
            return -2; /* 未知命令ID */
    }

    return 0;
}

/* ════════════════════════════════════════════════════
 * 公有接口
 * ════════════════════════════════════════════════════ */

/**
 * @brief  清零裁判解析状态、拼帧缓存和累计统计。
 * @note   接收启动前调用一次；运行时只需丢弃半帧时使用 drv_referee_reset_stream(ctx)。
 */
void drv_referee_init(drv_referee_context_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
}

/**
 * @brief  追加有序字节片段，完成裁判组帧、CRC 校验和解包，并返回本次处理结果。
 * @param  data  字节片段，函数复制内容，不保留输入地址。
 * @param  len   片段长度，范围 1~REFEREE_RX_BUF_SIZE；当前任务每次最多输入 128 字节。
 * @return 按值返回本次 updated_mask、frames、errors；空参数、零长度或超长输入返回全零结果。
 * @note   仅裁判任务串行调用；半帧保留到下次输入，坏帧逐字节重新同步。
 * @note   CRC 正确的未知命令也计入 frames；实际业务更新以 updated_mask 为准。
 * @note   相同字段多次更新可在每次返回中置位；旧 get_and_clear_flags() 仍独立累计到显式清除。
 */
drv_referee_parse_result_t drv_referee_process(drv_referee_context_t *ctx, const uint8_t *data,
                                               uint16_t len)
{
    /* 本次结果与兼容接口的待消费更新位分开，避免同一字段连续更新时漏报。 */
    memset(&ctx->result, 0, sizeof(ctx->result));
    if (!data || len == 0U || len > REFEREE_RX_BUF_SIZE)
    {
        return ctx->result;
    }
    if (ctx->s_stream_length + len > sizeof(ctx->s_stream_buf))
    {
        ctx->s_stream_length = 0;
    }

    /* 线性拼帧缓存只保存尚未消费的字节，与 ISR 到任务的环形 FIFO 独立。 */
    memcpy(ctx->s_stream_buf + ctx->s_stream_length, data, len);
    ctx->s_stream_length += len;
    uint16_t offset = 0;
    while (ctx->s_stream_length - offset >= PROTOCOL_HEADER_LEN)
    {
        const uint8_t *frame = ctx->s_stream_buf + offset;

        /* 先校验帧头，再信任其中的长度；普通噪声按字节跳过。 */
        if (frame[0] != PROTOCOL_SOF ||
            calc_crc8(frame, PROTOCOL_HEADER_LEN - 1U) != frame[PROTOCOL_HEADER_LEN - 1U])
        {
            if (frame[0] == PROTOCOL_SOF)
            {
                ctx->result.errors++;
            }
            offset++;
            continue;
        }
        uint16_t payload_length;
        memcpy(&payload_length, frame + 1, sizeof(payload_length));
        uint32_t frame_length = (uint32_t)payload_length + PROTOCOL_MIN_FRAME_LEN;
        if (frame_length > REFEREE_RX_BUF_SIZE)
        {
            offset++;
            continue;
        }
        if ((uint32_t)(ctx->s_stream_length - offset) < frame_length)
        {
            break; /* 帧头可信但全帧未到齐，从当前 SOF 起保留到下一次输入。 */
        }

        /* 全帧 CRC16 仅在这里计算一次；校验通过后才允许处理器改写业务数据。 */
        uint16_t received_crc;
        memcpy(&received_crc, frame + frame_length - PROTOCOL_CRC16_LEN, sizeof(received_crc));
        if (calc_crc16(frame, (uint16_t)(frame_length - PROTOCOL_CRC16_LEN)) != received_crc)
        {
            ctx->result.errors++;
            offset++;
            continue;
        }
        (void)parse_referee_packet(ctx, frame, frame_length);
        ctx->result.frames++;
        offset += (uint16_t)frame_length;
    }

    /* 移除已消费前缀；保留完整的未收齐尾部，供下次片段继续拼接。 */
    ctx->s_stream_length -= offset;
    memmove(ctx->s_stream_buf, ctx->s_stream_buf + offset, ctx->s_stream_length);

    /* 累计查询接口继续可用；只有显式 get_and_clear_flags() 才清除待消费更新位。 */
    ctx->s_referee_data.data_valid_flags |= ctx->result.updated_mask;
    ctx->s_frames += ctx->result.frames;
    ctx->s_errors += ctx->result.errors;
    return ctx->result;
}

/**
 * @brief  取得当前解析器的私有业务数据地址。
 * @return 静态存储的只读视图；后续 process/init 会改写其内容。
 * @note   仅裁判任务直接读取并复制；其他任务应通过 app_rtos_ref_read() 获取互斥快照。
 */
const drv_referee_global_t *drv_referee_get_data(drv_referee_context_t *ctx)
{
    return &ctx->s_referee_data;
}

/**
 * @brief  取走从上次清除后成功更新过的字段位，并清零更新位。
 * @return REFEREE_FLAG_* 按位或组成的掩码。
 * @note   不清除业务数据或 process 返回的本次结果；新任务直接使用返回结果，旧调用者仍可消费累计更新位。
 */
uint32_t drv_referee_get_and_clear_flags(drv_referee_context_t *ctx)
{
    uint32_t flags = ctx->s_referee_data.data_valid_flags;
    ctx->s_referee_data.data_valid_flags = 0;
    return flags;
}

/**
 * @brief  检查当前尚未清除的字段更新位。
 * @param  flag         一个或多个 REFEREE_FLAG_*。
 * @return 1=至少有一位已更新，0=均未更新。
 * @note   不消费更新位，也不判断在线状态；调用 get_and_clear_flags() 后相应位会清除。
 */
int drv_referee_is_data_updated(drv_referee_context_t *ctx, uint32_t flag)
{
    return (ctx->s_referee_data.data_valid_flags & flag) ? 1 : 0;
}

/**
 * @brief  把单帧解析返回码转换为固定诊断字符串。
 * @param  result       兼容的历史单帧解析错误码；当前流式入口返回结构体统计。
 * @return 静态字符串地址，调用者不应修改或释放。
 */
const char *drv_referee_parse_error_string(int result)
{
    switch (result)
    {
        case 0:
            return "Parse success";
        case -1:
            return "Length insufficient";
        case -2:
            return "Unknown command ID";
        case -3:
            return "SOF error (not 0xA5)";
        case -4:
            return "CRC mismatch";
        case -5:
            return "Frame too large";
        default:
            return "Unknown error";
    }
}

/**
 * @brief  丢弃解析器尚未收齐的半帧，下一次从新的字节片段重新同步。
 * @note   仅清零线性拼帧长度，不清空已解析业务数据、字段标志、帧数或错误数。
 * @note   任务发现 FIFO 溢出/接收错误后，必须先调用本函数，再处理复位后取出的新字节。
 */
void drv_referee_reset_stream(drv_referee_context_t *ctx)
{
    ctx->s_stream_length = 0;
}

/**
 * @brief  读取已通过 CRC 的完整帧累计数。
 * @return 本地 uint32_t 累计帧数；包含未知命令，不等于协议帧头 8 位 seq。
 * @note   由裁判任务读取，初始化时清零；统计已接受的帧，不保证所有帧都更新了业务字段。
 */
uint32_t drv_referee_get_frame_count(drv_referee_context_t *ctx)
{
    return ctx->s_frames;
}

/**
 * @brief  读取解析路径累计错误数。
 * @return 帧头 CRC8 或全帧 CRC16 失败的累计次数；普通噪声和越界长度跳过不计数。
 * @note   初始化时清零；UART 错误、FIFO 丢弃由任务集成层另行计数。
 */
uint32_t drv_referee_get_error_count(drv_referee_context_t *ctx)
{
    return ctx->s_errors;
}

uint16_t drv_referee_build_frame(uint8_t *frame, uint16_t capacity, uint16_t cmd_id,
                                 const void *payload, uint16_t payload_len, uint8_t seq)

{
    uint32_t frame_len = (uint32_t)payload_len + 9U;

    if (frame == NULL)
    {
        return 0U;
    }

    if (payload_len > 0U && payload == NULL)
    {
        return 0U;
    }

    if (frame_len > capacity)
    {
        return 0U;
    }

    frame[0] = 0xA5;
    memcpy(frame + 1U, &payload_len, sizeof(payload_len));
    frame[3] = seq;
    frame[4] = calc_crc8(frame, 4U);
    memcpy(frame + 5U, &cmd_id, sizeof(cmd_id));
    if (payload_len > 0U)
    {
        memcpy(frame + 7U, payload, payload_len);
    }
    uint16_t crc16 = calc_crc16(frame, 7U + payload_len);
    memcpy(frame + 7U + payload_len, &crc16, sizeof(crc16));

    return (uint16_t)frame_len;
}
