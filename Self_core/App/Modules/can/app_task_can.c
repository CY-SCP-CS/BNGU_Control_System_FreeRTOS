#include "app_can_output_policy.h"
/**
 * @file    app_task_can.c
 * @brief   CAN 独占接收、解码、编码、待发槽与反馈快照
 * @note    入口名、周期、业务算法及原通信对象名保持；通信对象句柄仅所属模块可访问。
 */
#include "app_vehicle_can.h"
#include "app_rtos.h"
#include "app_task_can_internal.h"
#include "app_tasks.h"
#include "app_rtos_objects.h"
#include "app_rtos_config.h"
#include "app_vehicle_config.h"
#include "app_rtos_time.h"
#include "app_task_can_port.h"
#include "app_task_monitor.h"
#include <string.h>

static osMessageQueueId_t can1_rx_queue;
static osMessageQueueId_t can2_rx_queue;
static osMessageQueueId_t can_to_ctrl_queue;
static osMutexId_t ctrl_output_mutex;

static struct
{
    app_ctrl_can_msg_t message;
    uint8_t valid, dirty, expired;
} s_latest[7];

static osMutexId_t can_feedback_mutex;
static app_can_feedback_t s_feedback;
static uint32_t s_can_seq[2];

/** @brief 中断入口只复制原始帧、计数和通知；协议在本任务解析。 */
void app_rtos_can_receive_isr(uint8_t bus, uint32_t id, const uint8_t *bytes, uint8_t len,
                              uint32_t tick)
{
    if ((bus != 1 && bus != 2) || id > 0x7FFU || len > 8 || !bytes)
    {
        app_rtos_count(COUNT_CAN_INVALID);
        return;
    }
    app_can_frame_t frame = {
        .bus = bus, .std_id = id, .dlc = len, .tick = tick, .sequence = ++s_can_seq[bus - 1]};
    memcpy(frame.data, bytes, len);
    if (osMessageQueuePut(bus == 1 ? can1_rx_queue : can2_rx_queue, &frame, 0, 0) != osOK)
    {
        app_rtos_count(bus == 1 ? COUNT_CAN1_RX_DROP : COUNT_CAN2_RX_DROP);
    }
    else
    {
        app_rtos_notify_task(TASK_CAN, APP_RTOS_TF_DATA_READY);
    }
}

/** @brief 邮箱释放不是业务数据，使用可合并的线程标志。 */
void app_rtos_can_event_isr(app_rtos_can_event_t event)
{
    if (event != APP_RTOS_CAN_TX_COMPLETE)
    {
        app_rtos_count(event == APP_RTOS_CAN_INVALID_FRAME ? COUNT_CAN_INVALID : COUNT_CAN_ERROR);
    }
    app_rtos_notify_task(TASK_CAN, APP_RTOS_TF_DATA_READY);
}

void app_rtos_ctrl_send(const app_ctrl_can_msg_t *message)
{
    if (!message || (unsigned)message->type >= 7U)
    {
        return;
    }
    if (osMutexAcquire(ctrl_output_mutex, osWaitForever) != osOK)
    {
        return;
    }
    unsigned i = message->type;
    s_latest[i].message = *message;
    s_latest[i].valid = 1;
    s_latest[i].dirty = 1;
    s_latest[i].expired = 0;
    osMutexRelease(ctrl_output_mutex);
    app_rtos_notify_task(TASK_CAN, APP_RTOS_TF_DATA_READY);
}

bool app_rtos_can_ctrl_receive(app_can_ctrl_msg_t *message)
{
    return message && osMessageQueueGet(can_to_ctrl_queue, message, NULL, 0) == osOK;
}

/**
 * @brief  读取一份完整的 CAN 电机/功率计/底盘角速度反馈快照
 * @param  v            输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由控制及诊断读取者在任务上下文调用；使用 can_feedback_mutex，获取锁超时为 0。
 */
bool app_rtos_feedback_read(app_can_feedback_t *v)
{
    return app_rtos_snapshot_read(can_feedback_mutex, &s_feedback, v, sizeof(*v));
}

/**
 * @brief  发布 CAN 电机/功率计/底盘角速度反馈快照
 * @param[in] v            输入快照，函数在锁内复制其内容
 * @note   写入者：comm_can_Task；使用 can_feedback_mutex，仅允许任务上下文调用。
 * @note   等待取得锁后只复制；参数/句柄无效或获取失败时忽略本次更新。
 * @return 无返回值；参数/句柄无效或取锁失败时保留旧快照。
 */
static void app_rtos_feedback_write(const app_can_feedback_t *v)
{
    app_rtos_snapshot_write(can_feedback_mutex, &s_feedback, v, sizeof(*v));
}

/* 以下 CAN 解码、编码、邮箱提交和待发槽全部由同一个 CAN 任务独占。 */
static app_can_feedback_t s_can_feedback; // CAN 任务私有反馈工作副本，各电机拥有独立更新时间

/** @brief 调用纯协议解析器，再把更新发布给健康监测或唯一控制消费者。 */
static void can_decode(const app_can_frame_t *frame)
{
    app_can_ctrl_msg_t command;
    unsigned motor_index = 0;
    app_vehicle_can_result_t result =
        app_vehicle_can_decode(frame, &s_can_feedback, &command, &motor_index);
    switch (result)
    {
        case APP_VEHICLE_CAN_MOTOR:
            app_rtos_module_update(MODULE_MOTOR0 + motor_index, true, frame->tick);
            break;
        case APP_VEHICLE_CAN_POWER:
            app_rtos_module_update(MODULE_POWER, true, frame->tick);
            break;
        case APP_VEHICLE_CAN_OMEGA:
            app_rtos_module_update(MODULE_CHASSIS_OMEGA, true, frame->tick);
            break;
        case APP_VEHICLE_CAN_COMMAND:
            app_rtos_module_update(MODULE_CAN_COMMAND, true, frame->tick);
            /* 标定期间更新反馈，但不积压旧控制增量。 */
            if ((app_rtos_init_status() & APP_RTOS_EVT_SYS_READY) &&
                osMessageQueuePut(can_to_ctrl_queue, &command, 0, 0) != osOK)
            {
                app_rtos_count(COUNT_CAN_CTRL_DROP);
            }
            break;
        case APP_VEHICLE_CAN_INVALID:
            app_rtos_count(COUNT_CAN_INVALID);
            break;
        case APP_VEHICLE_CAN_IGNORED:
            break;
    }
}

static void can_pending_put(const app_can_frame_t *frame);

/** @brief 编码结果直接按总线/ID 更新最新待发槽。 */
static void can_encode(const app_ctrl_can_msg_t *message)
{
    app_can_frame_t frames[APP_VEHICLE_CAN_MAX_FRAMES];
    unsigned count = app_vehicle_can_encode(message, frames);
    if (!count)
    {
        app_rtos_count(COUNT_CAN_INVALID);
    }
    for (unsigned i = 0; i < count; i++)
    {
        can_pending_put(&frames[i]);
    }
}

/* 每条总线独立 16 个待发槽，used 表示仍待提交的最新值，只有 CAN 任务访问。 */
static struct
{
    app_can_frame_t frame;
    uint8_t used;
} s_pending[2][APP_RTOS_CAN_PENDING_DEPTH];

static void can_pending_put(const app_can_frame_t *frame)
{
    unsigned bus = frame->bus - 1;
    int free_slot = -1;
    for (unsigned i = 0; i < APP_RTOS_CAN_PENDING_DEPTH; i++)
    {
        if (s_pending[bus][i].used && s_pending[bus][i].frame.std_id == frame->std_id)
        {
            s_pending[bus][i].frame = *frame;
            return;
        }
        if (!s_pending[bus][i].used)
        {
            free_slot = (int)i;
        }
    }
    if (free_slot < 0)
    {
        app_rtos_count(bus ? COUNT_CAN2_TX_DROP : COUNT_CAN1_TX_DROP);
        return;
    }
    s_pending[bus][free_slot].frame = *frame;
    s_pending[bus][free_slot].used = 1;
}

static uint8_t s_tx_cursor[2]; // 两条总线独立的待发槽轮转起点

/**
 * @brief  合并待发帧并向空闲 CAN 邮箱提交
 * @note   按总线/ID 保留最新值，无额外发送 FIFO，待发槽由 CAN 任务独占。
 * @note   轮转游标避免长期偏向某个 ID；邮箱忙/提交失败时保留帧，等待下一事件继续。
 * @note   已经交给硬件邮箱的帧不撤销，本函数不是额外的发送任务。
 */
static void can_transmit(void)
{
    for (unsigned bus = 0; bus < 2; bus++)
    {
        app_can_frame_t frame;
        /* 从上次游标轮转提交；失败时保留 used，邮箱完成事件后继续尝试。 */
        unsigned origin = s_tx_cursor[bus];
        for (unsigned i = 0; i < APP_RTOS_CAN_PENDING_DEPTH; i++)
        {
            unsigned slot = (origin + i) % APP_RTOS_CAN_PENDING_DEPTH;
            if (!s_pending[bus][slot].used)
            {
                continue;
            }
            frame = s_pending[bus][slot].frame;
            if (!app_can_frame_prepare(&frame, osKernelGetTickCount()))
            {
                s_pending[bus][slot].used = 0;
                continue;
            }
            if (!app_rtos_can_transmit(&frame))
            {
                break;
            }
            s_pending[bus][slot].used = 0;
            s_tx_cursor[bus] = (slot + 1) % APP_RTOS_CAN_PENDING_DEPTH;
        }
    }
}

/**
 * @brief  CAN 通信任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   两种板型均创建；PORT_READY 后由接收、邮箱完成或错误事件唤醒。
 * @note   独占 CAN 解码、编码和待发帧；每 tick 每条 RX 队列最多处理 8 条，输出按固定类型槽处理。
 */
void Start_comm_can_Task(void *argument)
{
    (void)argument;
    app_tasks_wait_start(TASK_CAN);
    uint32_t window = osKernelGetTickCount();
    uint8_t used[2] = {0};
    for (;;)
    {
        uint32_t now = osKernelGetTickCount();
        /* 每个 1 ms 窗口重置两路接收和控制输出预算，持续流量也不能无限占用 CPU。 */
        if (now != window)
        {
            window = now;
            memset(used, 0, sizeof(used));
        }
        uint32_t started = app_rtos_cycles();
        app_can_frame_t frame;
        for (unsigned bus = 0; bus < 2; bus++)
        {
            while (used[bus] < APP_RTOS_CAN_RX_BUDGET &&
                   osMessageQueueGet(bus ? can2_rx_queue : can1_rx_queue, &frame, NULL, 0) == osOK)
            {
                used[bus]++;
                can_decode(&frame);
            }
        }
        /* 一次处理批次结束后发布完整反馈，控制任务只读快照，不直接读私有解码状态。 */
        app_rtos_feedback_write(&s_can_feedback);
        for (unsigned i = 0; i < 7; ++i)
        {
            app_ctrl_can_msg_t message;
            bool send = false;
            osMutexAcquire(ctrl_output_mutex, osWaitForever);
            uint32_t age = osKernelGetTickCount() - s_latest[i].message.tick;
            if (s_latest[i].valid && i <= APP_CAN_CHASSIS_CMD &&
                age > (i == APP_CAN_CHASSIS_CMD ? 100U : APP_CAN_OUTPUT_MAX_AGE_MS) &&
                !s_latest[i].expired)
            {
                memset(&s_latest[i].message.value, 0, sizeof(s_latest[i].message.value));
                if (i == APP_CAN_CHASSIS_CMD)
                {
                    s_latest[i].message.value.chassis.vx = -32.768f;
                }
                s_latest[i].message.tick = osKernelGetTickCount();
                s_latest[i].dirty = 1;
                s_latest[i].expired = 1;
            }
            if (s_latest[i].valid && s_latest[i].dirty)
            {
                message = s_latest[i].message;
                s_latest[i].dirty = 0;
                send = true;
            }
            osMutexRelease(ctrl_output_mutex);
            if (send)
            {
                can_encode(&message);
            }
        }
        can_transmit();
        app_tasks_profile_end(TASK_CAN, started);
        app_rtos_heartbeat(TASK_CAN);
        bool budget_blocked =
            (used[0] == APP_RTOS_CAN_RX_BUDGET && osMessageQueueGetCount(can1_rx_queue)) ||
            (used[1] == APP_RTOS_CAN_RX_BUDGET && osMessageQueueGetCount(can2_rx_queue));
        /* 已用完预算且还有积压则等下一 tick；其他情况等待数据/邮箱事件并维护心跳。 */
        if (budget_blocked)
        {
            osDelay(1);
        }
        else
        {
            app_tasks_wait_event(TASK_CAN, 1);
        }
    }
}

/**
 * @brief  创建本模块全部通信对象并复位数据
 * @return true=全部创建成功，false=创建失败，由装配层统一回滚。
 * @note   只在调度器启动前调用一次；不支持运行中重新初始化。
 */
bool app_task_can_init(void)
{
    memset(s_latest, 0, sizeof(s_latest));
    memset(&s_feedback, 0, sizeof(s_feedback));
    memset(&s_can_feedback, 0, sizeof(s_can_feedback));
    memset(s_can_seq, 0, sizeof(s_can_seq));
    memset(s_pending, 0, sizeof(s_pending));
    memset(s_tx_cursor, 0, sizeof(s_tx_cursor));
    return app_rtos_object_queue(&can1_rx_queue, "can1_rx_queue", APP_RTOS_CAN_RX_QUEUE_DEPTH,
                                 sizeof(app_can_frame_t)) &&
           app_rtos_object_queue(&can2_rx_queue, "can2_rx_queue", APP_RTOS_CAN_RX_QUEUE_DEPTH,
                                 sizeof(app_can_frame_t)) &&
           app_rtos_object_queue(&can_to_ctrl_queue, "can_to_ctrl_queue", APP_RTOS_CTRL_QUEUE_DEPTH,
                                 sizeof(app_can_ctrl_msg_t)) &&
           app_rtos_object_mutex(&ctrl_output_mutex, "ctrl_output_mutex") &&
           app_rtos_object_mutex(&can_feedback_mutex, "can_feedback_mutex");
}

/** @brief 启动失败时删除已创建对象；句柄为 NULL 时安全跳过。 */
void app_task_can_deinit(void)
{
    app_rtos_object_queue_delete(&can1_rx_queue);
    app_rtos_object_queue_delete(&can2_rx_queue);
    app_rtos_object_queue_delete(&can_to_ctrl_queue);
    app_rtos_object_mutex_delete(&ctrl_output_mutex);
    app_rtos_object_mutex_delete(&can_feedback_mutex);
}
