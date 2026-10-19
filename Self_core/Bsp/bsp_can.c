/**
 * @file    bsp_can.c
 * @brief   CAN 驱动实现: 初始化 / 发送 / 回调分发
 * @note    滤波器用 16-bit IDMASK 模式, 接收所有 ID
 *          所有接收走回调注册, 无轮询缓冲区
 *          CAN1 / CAN2 使用独立回调表, 互不占用
 */
#include "bsp_can.h"

typedef struct
{
    uint32_t std_id;
    bsp_can_rx_callback_t callback;
} bsp_can_callback_entry_t; //CAN回调表条目

typedef struct
{
    bsp_can_callback_entry_t entries[BSP_CAN_REG_MAX];
    uint8_t count;
} bsp_can_callback_table_t; //CAN回调表

/* 两条 CAN 共用的 RTOS 原始入口；安装后旁路按 ID 兼容表，业务解包由任务执行。 */
static bsp_can_raw_callback_t s_raw_rx;
static bsp_can_event_callback_t s_event;
static bsp_can_callback_table_t s_can1_table; //CAN1回调表
static bsp_can_callback_table_t s_can2_table; //CAN2回调表

/**
 * @brief  根据 hcan 选择对应回调表
 * @return 回调表指针, 未知总线返回 NULL
 */
static bsp_can_callback_table_t *get_table(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1)
    {
        return &s_can1_table;
    }
    if (hcan == &hcan2)
    {
        return &s_can2_table;
    }
    return NULL;
}

/**
 * @brief  获取空闲邮箱号
 * @return 邮箱位掩码 (CAN_TX_MAILBOX0/1/2), 无空闲返回 0xFFFFFFFF
 */
static uint32_t get_free_mbox(CAN_HandleTypeDef *hcan)
{
    if (!HAL_CAN_IsTxMessagePending(hcan, CAN_TX_MAILBOX0))
    {
        return CAN_TX_MAILBOX0;
    }
    if (!HAL_CAN_IsTxMessagePending(hcan, CAN_TX_MAILBOX1))
    {
        return CAN_TX_MAILBOX1;
    }
    if (!HAL_CAN_IsTxMessagePending(hcan, CAN_TX_MAILBOX2))
    {
        return CAN_TX_MAILBOX2;
    }
    return 0xFFFFFFFFu;
}

/**
 * @brief  配置全接收滤波器，启动 CAN，并使能 FIFO0、邮箱完成和错误中断。
 * @param  hcan         CAN1 或 CAN2 句柄。
 * @param  filter_bank  本总线使用的滤波器组。
 * @param  slave_filter_bank 双 CAN 共享滤波器的 CAN2 起始组。
 * @return 首个失败步骤的 HAL 状态，或中断通知使能结果；HAL_OK 表示这三步完成。
 * @note   当前适配使用 CAN1 组 0、CAN2 组 14，分界为 14；必须先绑定接收/事件回调再启动。
 */
HAL_StatusTypeDef bsp_can_start(CAN_HandleTypeDef *hcan, uint8_t filter_bank,
                                uint8_t slave_filter_bank)
{
    CAN_FilterTypeDef filter;
    HAL_StatusTypeDef can_status_return;

    filter.FilterActivation = CAN_FILTER_ENABLE;
    filter.FilterBank = filter_bank;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.SlaveStartFilterBank = slave_filter_bank;

    can_status_return = HAL_CAN_ConfigFilter(hcan, &filter);
    if (can_status_return != HAL_OK)
    {
        return can_status_return; //检查寄存器配置是否成功
    }

    can_status_return = HAL_CAN_Start(hcan);
    if (can_status_return != HAL_OK)
    {
        return can_status_return; //检查CAN启动是否成功
    }

    can_status_return = HAL_CAN_ActivateNotification(
        hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_ERROR | CAN_IT_BUSOFF |
                  CAN_IT_LAST_ERROR_CODE | CAN_IT_RX_FIFO0_OVERRUN); //开启接收中断
    return can_status_return;
}

/**
 * @brief  尝试把一帧标准数据帧提交到空闲 CAN 邮箱。
 * @param  hcan         目标 CAN 句柄。
 * @param  std_id       11 位标准 ID，范围 0~0x7FF。
 * @param  data         载荷地址；必须非 NULL，HAL 同步装载邮箱后即可复用。
 * @param  dlc          有效载荷长度，范围 0~8 字节。
 * @return 0=已装入邮箱，1=参数无效、邮箱占满或 HAL 提交失败；不表示总线发送已完成。
 * @note   本函数不等待空闲邮箱；短临界区覆盖邮箱检查与 HAL 提交，避免并发发送竞争。
 * @note   RTOS 工程由 CAN 任务管理重试；发送完成后通过事件回调通知继续处理待发帧。
 */
uint8_t bsp_can_tx_frame(CAN_HandleTypeDef *hcan, uint32_t std_id, const uint8_t *data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef tx_hdr = {0};
    HAL_StatusTypeDef hal_status;
    uint32_t irq_state;
    uint32_t mbox;

    if (!hcan || !data || std_id > 0x7FFU || dlc > 8U)
    {
        return 1U;
    }

    tx_hdr.StdId = std_id;
    tx_hdr.IDE = CAN_ID_STD;
    tx_hdr.RTR = CAN_RTR_DATA;
    tx_hdr.DLC = dlc;

    /* 邮箱检查与设置 TXRQ 必须连续完成，防止中断嵌套发送使用同一邮箱*/
    irq_state = __get_PRIMASK();
    __disable_irq();

    mbox = get_free_mbox(hcan);
    if (mbox == 0xFFFFFFFFU)
    {
        __set_PRIMASK(irq_state);
        return 1U;
    }
    hal_status = HAL_CAN_AddTxMessage(hcan, &tx_hdr, (uint8_t *)data, &mbox);
    __set_PRIMASK(irq_state);

    return hal_status == HAL_OK ? 0U : 1U;
}

/**
 * @brief  向指定总线的兼容回调表登记标准 ID 处理函数。
 * @param  hcan         CAN1/CAN2 句柄。
 * @param  std_id       要匹配的标准 ID。
 * @param  callback     中断内调用的接收函数。
 * @note   同 ID 更新旧回调；表满、未知总线或 NULL 回调时静默忽略。
 * @note   只在未绑定 raw 回调时使用；注册应在接收开始前完成。
 */
void bsp_can_rx_reg(CAN_HandleTypeDef *hcan, uint32_t std_id, bsp_can_rx_callback_t callback)
{
    bsp_can_callback_table_t *table = get_table(hcan);
    uint8_t i;

    if (!table || !callback)
    {
        return;
    }

    for (i = 0; i < table->count; i++)
    {
        if (table->entries[i].std_id == std_id)
        {
            table->entries[i].callback = callback;
            return; //覆盖旧回调
        }
    } //

    if (table->count >= BSP_CAN_REG_MAX)
    {
        return;
    } //超出限制静默失败

    table->entries[table->count].std_id = std_id;
    table->entries[table->count].callback = callback;
    table->count++; //正常添加
}

/**
 * @brief  在 FIFO0 中断中取出并交付最多三帧，限制一次回调处理量。
 * @param  hcan         产生接收中断的 CAN 句柄。
 * @note   只接受标准数据帧且 DLC<=8；非法帧或读取失败通过事件回调报告。
 * @note   优先交给 raw 回调，否则匹配旧的总线/ID 回调表；本地 bytes 数组必须在回调返回前复制。
 */
void bsp_can_rx_irq_handler(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t bytes[8];
    bsp_can_callback_table_t *table = get_table(hcan);
    for (unsigned n = 0; n < 3 && HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0); n++)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, bytes) != HAL_OK)
        {
            if (s_event)
            {
                s_event(hcan, 1);
            }
            break;
        }
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC > 8)
        {
            if (s_event)
            {
                s_event(hcan, BSP_CAN_EVENT_INVALID);
            }
            continue;
        }
        if (s_raw_rx)
        {
            s_raw_rx(hcan, header.StdId, bytes, (uint8_t)header.DLC);
        }
        else if (table)
        {
            for (unsigned i = 0; i < table->count; i++)
            {
                if (table->entries[i].std_id == header.StdId)
                {
                    table->entries[i].callback(header.StdId, bytes, (uint8_t)header.DLC);
                }
            }
        }
    }
}

/**
 * @brief  把 HAL FIFO0 有帧事件转入 BSP 的有界接收分发。
 * @param  hcan         触发 FIFO0 事件的 CAN 句柄。
 * @note   在 CAN RX ISR 中执行，不在此处解析电机或板间业务。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    bsp_can_rx_irq_handler(hcan);
}

/**
 * @brief  兼容原接口，以固定 DLC=8 提交标准 CAN 数据帧。
 * @param  hcan         目标 CAN 句柄。
 * @param  id           标准 ID。
 * @param  data         至少 8 字节的输入载荷。
 * @return 0=已提交邮箱，1=提交失败；语义与 bsp_can_tx_frame() 相同。
 */
uint8_t bsp_can_tx(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t data[8])
{
    return bsp_can_tx_frame(hcan, id, data, 8);
}

/**
 * @brief  绑定 RTOS 使用的原始收帧和邮箱/错误事件入口。
 * @param  rx           收帧回调，NULL 时退回原有按 ID 回调表分发。
 * @param  event        发送完成/错误回调，NULL 时不通知。
 * @note   在启动 CAN 接收前绑定，CAN1/CAN2 共用这组函数，回调通过 hcan 区分总线。
 * @note   绑定 raw 回调后原有按 ID 回调表不会同时接收该帧；解包转移到 CAN 任务。
 */
void bsp_can_reg_raw_callbacks(bsp_can_raw_callback_t rx, bsp_can_event_callback_t event)
{
    s_raw_rx = rx;
    s_event = event;
}

/**
 * @brief  把邮箱 0 发送完成转为 CAN 任务可接收的完成事件。
 * @param  hcan         该邮箱所属 CAN 句柄。
 * @note   CAN TX ISR 上下文，以事件值 0 调用已注册入口；回调只通知，不在这里排队重发。
 */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if (s_event)
    {
        s_event(hcan, 0);
    }
}

/**
 * @brief  把邮箱 1 发送完成转为 CAN 任务可接收的完成事件。
 * @param  hcan         该邮箱所属 CAN 句柄。
 * @note   CAN TX ISR 上下文，以事件值 0 调用已注册入口；回调只通知，不在这里排队重发。
 */
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if (s_event)
    {
        s_event(hcan, 0);
    }
}

/**
 * @brief  把邮箱 2 发送完成转为 CAN 任务可接收的完成事件。
 * @param  hcan         该邮箱所属 CAN 句柄。
 * @note   CAN TX ISR 上下文，以事件值 0 调用已注册入口；回调只通知，不在这里排队重发。
 */
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if (s_event)
    {
        s_event(hcan, 0);
    }
}

/**
 * @brief  把邮箱 0 中止转为错误事件，交由任务记录与后续处理。
 * @param  hcan         该邮箱所属 CAN 句柄。
 * @note   CAN ISR 上下文，以非 0 事件值 1 通知上层。
 */
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan)
{
    if (s_event)
    {
        s_event(hcan, 1);
    }
}

/**
 * @brief  把邮箱 1 中止转为错误事件，交由任务记录与后续处理。
 * @param  hcan         该邮箱所属 CAN 句柄。
 * @note   CAN ISR 上下文，以非 0 事件值 1 通知上层。
 */
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan)
{
    if (s_event)
    {
        s_event(hcan, 1);
    }
}

/**
 * @brief  把邮箱 2 中止转为错误事件，交由任务记录与后续处理。
 * @param  hcan         该邮箱所属 CAN 句柄。
 * @note   CAN ISR 上下文，以非 0 事件值 1 通知上层。
 */
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan)
{
    if (s_event)
    {
        s_event(hcan, 1);
    }
}

/**
 * @brief  上报 HAL CAN 错误码，供通信任务计数并检查发送状态。
 * @param  hcan         报告错误的 CAN 句柄。
 * @note   CAN ISR 上下文；ErrorCode 为 0 时仍以事件值 1 表示发生过错误。
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (s_event)
    {
        s_event(hcan, hcan->ErrorCode ? hcan->ErrorCode : 1);
    }
}
