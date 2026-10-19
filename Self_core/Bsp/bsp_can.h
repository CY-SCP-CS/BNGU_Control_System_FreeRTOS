/**
 * @file    bsp_can.h
 * @brief   CAN 驱动: 初始化 / 发送 / 回调订阅
 */
#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "bsp_cfg.h"

#define BSP_CAN_REG_MAX 16 //CAN1/CAN2最大回调注册数量

typedef void (*bsp_can_rx_callback_t)(uint32_t std_id, uint8_t *data,
                                      uint8_t len); //CAN接收回调函数指针

/**
 * @brief  CAN 初始化
 * @param  hcan               CAN 句柄
 * @param  filter_bank        滤波器组号
 * @param  slave_filter_bank  CAN2 滤波器起始组号 (仅双 CAN 时 CAN1 需要, 单 CAN 传 0)
 * @return HAL_StatusTypeDef
 * @note  滤波器用 16-bit IDMASK 模式, 接收所有 ID，还没留接口，感觉不太需要
 */
HAL_StatusTypeDef bsp_can_start(CAN_HandleTypeDef *hcan, uint8_t filter_bank,
                                uint8_t slave_filter_bank);

/**
 * @brief  发送 CAN 帧 (非阻塞, 固定 8 字节)
 * @param  hcan   CAN 句柄
 * @param  std_id 标准 ID
 * @param  data   数据 (8 字节)
 * @return 0=发送成功，1=发送失败
 */
uint8_t bsp_can_tx(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t data[8]);

/**
 * @brief  注册 CAN 接收回调 (中断中直接分发)
 * @param  hcan     CAN 句柄
 * @param  std_id   要监听的标准 ID
 * @param  callback 回调函数
    * @note   同一总线同 ID 只允许注册一个回调, 重复注册会覆盖旧回调
    * @note   回调函数在中断中执行, 尽量短小, 避免阻塞
 */
void bsp_can_rx_reg(CAN_HandleTypeDef *hcan, uint32_t std_id, bsp_can_rx_callback_t callback);

/**
 * @brief  CAN 接收中断入口 (在 HAL 回调中调用)
 * @param  hcan   CAN 句柄
 */
void bsp_can_rx_irq_handler(CAN_HandleTypeDef *hcan);

/* Raw capture callbacks run in IRQ context; receivers must immediately copy. */
/**
 * @brief  CAN 原始标准数据帧交付回调，CAN1/CAN2 共用一个入口。
 * @note   参数依次为总线句柄、标准 ID、只读载荷地址、DLC；在 CAN RX ISR 中同步调用。
 * @note   载荷指向 BSP 接收入口的局部数组，回调必须立即复制；不能把该地址排队后返回。
 */
typedef void (*bsp_can_raw_callback_t)(CAN_HandleTypeDef *, uint32_t, const uint8_t *, uint8_t);
/**
 * @brief  CAN 邮箱完成或错误事件回调。
 * @note   参数依次为总线句柄和事件值：0=邮箱发送完成，非 0=中止、接收失败或 HAL 错误。
 * @note   BSP_CAN_EVENT_INVALID 表示非标准数据帧或 DLC 不合法；事件值并非统一的 HAL 错误枚举。
 */
typedef void (*bsp_can_event_callback_t)(CAN_HandleTypeDef *, uint32_t);
/* BSP 自定义非法帧事件位；交给任务计数，原始数据不继续上交。 */
#define BSP_CAN_EVENT_INVALID 0x80000000UL
/**
 * @brief  绑定 RTOS 使用的原始收帧和邮箱/错误事件入口。
 * @param  rx           收帧回调，NULL 时退回原有按 ID 回调表分发。
 * @param  event        发送完成/错误回调，NULL 时不通知。
 * @note   在启动 CAN 接收前绑定，CAN1/CAN2 共用这组函数，回调通过 hcan 区分总线。
 * @note   绑定 raw 回调后原有按 ID 回调表不会同时接收该帧；解包转移到 CAN 任务。
 */
void bsp_can_reg_raw_callbacks(bsp_can_raw_callback_t rx, bsp_can_event_callback_t event);
/**
 * @brief  尝试把一帧标准数据帧提交到空闲 CAN 邮箱。
 * @param  hcan         目标 CAN 句柄。
 * @param  id           11 位标准 ID，范围 0~0x7FF。
 * @param  data         载荷地址；必须非 NULL，HAL 同步装载邮箱后即可复用。
 * @param  dlc          有效载荷长度，范围 0~8 字节。
 * @return 0=已装入邮箱，1=参数无效、邮箱占满或 HAL 提交失败；不表示总线发送已完成。
 * @note   本函数不等待空闲邮箱；短临界区覆盖邮箱检查与 HAL 提交，避免并发发送竞争。
 * @note   RTOS 工程由 CAN 任务管理重试；发送完成后通过事件回调通知继续处理待发帧。
 */
uint8_t bsp_can_tx_frame(CAN_HandleTypeDef *, uint32_t id, const uint8_t *, uint8_t dlc);
#endif
