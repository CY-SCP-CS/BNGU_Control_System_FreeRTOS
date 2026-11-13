/**
 * @file    app_rtos_types.h
 * @brief   任务元数据、线程标志和诊断索引；不包含业务数据或设备头
 */
#ifndef APP_RTOS_TYPES_H
#define APP_RTOS_TYPES_H
#include <stdbool.h>
#include <stdint.h>
#include "cmsis_os2.h"
#define APP_RTOS_EVT_SYS_READY (1UL << 0)       //事件组：系统初始化成功
#define APP_RTOS_EVT_SYS_INIT_FAILED (1UL << 1) //事件组：系统初始化失败
#define APP_RTOS_EVT_PORT_READY (1UL << 2)      //持久事件：端口可用
#define APP_RTOS_TF_DATA_READY (1UL << 0)       //线程标志：有接收数据或驱动状态待处理
#define APP_RTOS_TF_STOP_REQUEST (1UL << 1)     //预留停止请求位，当前任务循环未消费
#define APP_RTOS_TF_TX_DONE (1UL << 5)          //线程标志：本机 UART 发送完成
#define APP_RTOS_TF_IO_ERROR (1UL << 6)         //线程标志：UART1 发送错误

/* 从统一清单生成任务编号，供创建、通知、日志和统计共用。
 * 编号按清单顺序从 0 递增；新任务追加到清单末尾，保持已有编号稳定。
 * 此处只展开第一个参数，不依赖板型能力或入口函数声明。
 */
enum
{
#define APP_TASK(task_id, task_handle, task_entry, ...) task_id,
#include "app_task_list.def"
#undef APP_TASK
    TASK_COUNT /* 清单中的任务总数，包含当前板未启用的任务。 */
};

/** @brief 创建、启动门控和诊断共用的任务元数据，栈单位为字节。 */
typedef struct
{
    const char *name;              /**< 调试器中显示的任务名。 */
    osThreadId_t *handle;          /**< 句柄变量的地址；创建后向该变量写入线程 ID。 */
    osThreadFunc_t entry;          /**< 任务入口函数。 */
    uint32_t stack;                /**< 栈大小，单位字节。 */
    osPriority_t priority;         /**< CMSIS-RTOS2 线程优先级。 */
    uint8_t enabled;               /**< 当前板型是否创建此任务。 */
    uint32_t start_flags;          /**< 首次等待的 PORT_READY 或 SYS_READY。 */
    uint8_t diagnostic_on_failure; /**< 初始化失败后是否仍运行诊断。 */
    uint8_t one_shot;              /**< 是否为执行完毕后退出的一次性任务。 */
} app_rtos_task_desc_t;

extern const app_rtos_task_desc_t app_rtos_task_table[TASK_COUNT];

/**
 * @brief  单任务运行统计，由任务维护自己的条目，健康任务读取
 * @note   这是多字段统计记录，读取时不保证所有字段属于同一瞬间。
 */
typedef struct
{
    uint32_t heartbeat,        //最近心跳时间，单位内核 tick
        heartbeat_count,       //累计心跳更新次数
        cycles,                //累计完成的业务轮数；事件任务按处理批次计
        missed,                //周期任务跳过的过期节拍数量
        delay_errors,          //排除正常到期竞争后的延时 API 错误
        snapshot_failures;     //快照非阻塞读取失败次数
    uint32_t max_execution_us, //最大经过时间，含抢占/阻塞，单位 us
        max_wakeup_us,         //相对目标 tick 的最大开始执行延迟，单位 us
        max_imu_age_ms;        //云台使用 IMU 数据的最大年龄，单位 ms
    uint32_t final_stack_free; //一次性任务退出前缓存的栈水位，单位字节
    uint8_t completed;         //一次性任务已完成，不再查询失效句柄
} app_rtos_task_stats_t;

extern app_rtos_task_stats_t app_rtos_task_stats[TASK_COUNT]; //任务运行统计表，供健康监测/调试读取

/* 诊断计数索引：由发生问题的任务/中断累计，健康任务汇总。 */
enum
{
    COUNT_CAN1_RX_DROP,     //CAN1 原始接收队列投递失败
    COUNT_CAN2_RX_DROP,     //CAN2 原始接收队列投递失败
    COUNT_CAN1_TX_DROP,     //CAN1 发送队列或待发槽满
    COUNT_CAN2_TX_DROP,     //CAN2 发送队列或待发槽满
    COUNT_CAN_CTRL_DROP,    //CAN 指令交付控制任务失败
    COUNT_CTRL_CAN_DROP,    //控制结果交付 CAN 任务失败
    COUNT_DBUS_DROP,        //完整 DBUS 帧入队失败
    COUNT_IMU_DROP,         //完整 IMU 样本入队失败
    COUNT_REF_OVERFLOW,     //裁判 FIFO 溢出，请求重新同步
    COUNT_LOG_DROP,         //日志队列投递失败
    COUNT_CAN_INVALID,      //已知 CAN 帧的格式/数值或输出类型非法
    COUNT_CAN_ERROR,        //CAN 外设错误事件
    COUNT_UART_ERROR,       //UART 接收/发送错误
    COUNT_UART_TIMEOUT,     //UART1 等待发送完成超时
    COUNT_DBUS_INVALID,     //DBUS 解包结果未通过校验
    COUNT_REF_CRC,          //裁判解析器累计错误增量，包含 CRC/格式错误
    COUNT_FLAG_ERROR,       //线程标志设置/等待 API 错误
    COUNT_IO_RESTART_ERROR, //接收端口重启失败
    COUNT_TELEMETRY_DROP,   // 诊断锁忙，保留旧遥测；不影响控制交付
    COUNT_COUNT             //计数项总数
};

extern volatile uint32_t app_rtos_counters[COUNT_COUNT]; //共享诊断计数，使用 app_rtos_count() 增加

#endif
