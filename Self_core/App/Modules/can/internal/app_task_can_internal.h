/**
 * @file    app_task_can_internal.h
 * @brief   CAN 服务语义接口；六个队列和反馈锁由 CAN 模块私有管理
 */
#ifndef APP_TASK_CAN_INTERNAL_H
#define APP_TASK_CAN_INTERNAL_H
#include "app_task_can.h"

typedef enum
{
    APP_RTOS_CAN_BUS_1 = 1,
    APP_RTOS_CAN_BUS_2 = 2
} app_rtos_can_bus_t;

typedef enum
{
    APP_RTOS_CAN_TX_COMPLETE,
    APP_RTOS_CAN_IO_ERROR,
    APP_RTOS_CAN_INVALID_FRAME
} app_rtos_can_event_t;

/**
 * @brief  将标准 CAN 帧复制到对应接收队列并唤醒 CAN 任务。
 * @param[in] bus 总线编号，1 或 2。
 * @param[in] id 标准帧 ID，0..0x7FF。
 * @param[in] bytes 至少 len 字节的有效缓冲；立即复制，不保留地址。
 * @param[in] len 数据长度，0..8 B。
 * @param[in] tick 接收时间，单位内核 tick。
 * @return 无返回值；无效参数或队列满时累计对应诊断计数。
 * @note   合法 RTOS 中断上下文调用，入队不等待。
 */
void app_rtos_can_receive_isr(uint8_t bus, uint32_t id, const uint8_t *bytes, uint8_t len,
                              uint32_t tick);
/**
 * @brief  将 CAN 邮箱完成或错误事件通知 CAN 任务。
 * @param[in] event 明确的完成、外设错误或非法帧事件。
 * @return 无返回值；通知失败由统一通知接口计数。
 * @note   合法 RTOS 中断上下文调用，不执行协议计算或阻塞发送。
 */
void app_rtos_can_event_isr(app_rtos_can_event_t event);

/**
 * @brief  CAN 通信任务入口；帧交付、控制提交和反馈快照接口见本头文件
 * @param[in] argument CMSIS-RTOS2 任务入口参数，当前创建时传 NULL。
 * @return 无返回值；任务持续运行。
 * @note   由任务表注册并由调度器调用，业务代码不直接调用任务入口。
 */
void Start_comm_can_Task(void *argument);

/**
 * @brief  创建本模块通信对象并复位初始状态。
 * @return true=全部创建成功；false=创建失败，由装配统一回滚。
 * @note   仅内核就绪、调度器启动前调用，不支持运行中重置。
 */
bool app_task_can_init(void);
/**
 * @brief  删除本模块已创建的通信对象。
 * @return 无返回值；空句柄安全跳过。
 * @note   仅启动失败回滚使用，必须先终止所有已创建任务。
 */
void app_task_can_deinit(void);

#endif
