/**
 * @file    app_rtos.h
 * @brief   应用 RTOS 对象与配置接口: 队列 / 互斥量 / 标志 / 任务
 * @note    基于 CMSIS-RTOS2，保持现有任务和通信对象命名。
 * @note    对象在内核启动前创建；任务业务和调度入口位于 app_tasks.c。
 */
#ifndef APP_RTOS_H
#define APP_RTOS_H
#include <stdbool.h>
#include <stdint.h>
#include "cmsis_os2.h"
#include "app_data.h"
typedef app_can_frame_t app_rtos_can_frame_t; //保留 RTOS 帧类型名，包含总线/ID/DLC/字节/tick/序号
typedef app_rtos_can_frame_t app_rtos_can_rx_msg_t; //原始接收队列消息
typedef app_rtos_can_frame_t app_rtos_can_tx_msg_t; //已编码发送队列消息
typedef app_can_ctrl_msg_t app_rtos_can_ctrl_msg_t; //CAN -> 控制的应用指令类型
typedef app_ctrl_can_msg_t app_rtos_ctrl_can_msg_t; //控制 -> CAN 的应用结果类型
/** @brief DBUS 队列消息；保存自有字节，不传递 DMA 缓冲指针。 */
typedef struct {
    uint8_t bytes[18];          //完整 DBUS 帧
    uint32_t tick,              //接收时间，单位内核 tick
             sequence;         //本接收端口的递增帧序号
} app_rtos_dbus_frame_t;
/** @brief 固定长度日志事件；不在控制任务中格式化文本。 */
typedef struct {
    uint32_t tick;              //事件时间，单位内核 tick
    uint16_t code;              //事件码，具体意义由事件类型决定
    uint8_t source,             //来源任务编号 TASK_*
            level;             //1=信息，2=告警，3=错误
    int32_t value0,             //事件整数参数 0
            value1;            //事件整数参数 1
} app_rtos_log_msg_t;
/** @brief CAN 帧中的总线编号，1/2 分别选择 CAN1/CAN2。 */
typedef enum { APP_RTOS_CAN_BUS_1=1, APP_RTOS_CAN_BUS_2=2 } app_rtos_can_bus_t;
#define APP_RTOS_CAN_RX_QUEUE_DEPTH 32U //每条原始 CAN 接收队列容量
#define APP_RTOS_CAN_TX_QUEUE_DEPTH 16U //每条已编码 CAN 发送队列容量
#define APP_RTOS_CTRL_QUEUE_DEPTH 16U //两个应用方向队列各自的容量
#define APP_RTOS_LOG_QUEUE_DEPTH 16U //日志事件队列容量
#define APP_RTOS_CAN_DATA_SIZE 8U //经典 CAN 单帧最大载荷字节数
#define APP_RTOS_EVT_SYS_READY (1UL<<0) //事件组：系统初始化成功
#define APP_RTOS_EVT_SYS_INIT_FAILED (1UL<<1) //事件组：系统初始化失败
#define APP_RTOS_TF_DATA_READY (1UL<<0) //线程标志：有接收数据或驱动状态待处理
#define APP_RTOS_TF_STOP_REQUEST (1UL<<1) //预留停止请求位，当前任务循环未消费
#define APP_RTOS_TF_PORT_READY (1UL<<2) //线程标志：端口可用，允许通信/健康/TEXT 日志运行
#define APP_RTOS_TF_SYS_READY (1UL<<3) //线程标志：业务初始化完成，允许控制/IMU/VOFA 运行
#define APP_RTOS_TF_INIT_FAILED (1UL<<4) //线程标志：初始化失败，业务任务停止业务
#define APP_RTOS_TF_TX_DONE (1UL<<5) //线程标志：CAN 邮箱释放或本机 UART 发送完成
#define APP_RTOS_TF_IO_ERROR (1UL<<6) //线程标志：外设错误，需要任务处理
#define APP_RTOS_WAKE_FLAGS (APP_RTOS_TF_DATA_READY|APP_RTOS_TF_TX_DONE|APP_RTOS_TF_IO_ERROR) //事件任务统一等待的标志组合

extern osMessageQueueId_t can1_rx_queue; //CAN1 中断 -> CAN 任务；原始帧，深度 32
extern osMessageQueueId_t can2_rx_queue; //CAN2 中断 -> CAN 任务；原始反馈帧，深度 32
extern osMessageQueueId_t can1_tx_queue; //CAN 任务编码阶段 -> 发送阶段；已编码 CAN1 帧，深度 16
extern osMessageQueueId_t can2_tx_queue; //CAN 任务编码阶段 -> 发送阶段；已编码 CAN2 帧，深度 16
extern osMessageQueueId_t can_to_ctrl_queue; //CAN -> 本板主控制任务；解码指令，深度 16，仅一个消费者
extern osMessageQueueId_t ctrl_to_can_queue; //各控制任务 -> CAN；电流/命令/状态结果，深度 16
extern osMessageQueueId_t sys_log_queue; //日志事件 -> 日志任务；固定长度事件，深度 16
extern osMessageQueueId_t dbus_rx_queue; //USART3 回调 -> DBUS；完整 18 字节帧，深度 4
extern osMessageQueueId_t imu_rx_queue; //SPI DMA 回调 -> IMU；完整采样数据，深度 4

extern osMutexId_t dbus_to_gim_mutex; //保护 DBUS 遥控快照，写入者为 DBUS 任务
extern osMutexId_t dbus_to_shoot_mutex; //保护云台裁决后的发射命令，写入者为云台任务，名称保留
extern osMutexId_t imu_to_gim_mutex; //保护 IMU 姿态快照，控制读取完整副本
extern osMutexId_t ref_to_chas_mutex; //保护裁判快照，目前供健康读取，保留控制读取接口
extern osMutexId_t can_feedback_mutex; //保护各电机、功率计和底盘角速度反馈
extern osMutexId_t ctrl_state_mutex; //保护各控制任务发布的状态和 VOFA 波形
extern osMutexId_t sys_health_mutex; //保护完整健康快照，供诊断查询

extern osEventFlagsId_t sys_init_event; //系统 READY / INIT_FAILED 状态查询事件组

extern osSemaphoreId_t app_rtos_example_binary_sem; //保留的二值信号量示例，初值 0，当前业务未使用

extern osThreadId_t ctrl_chas_TaskHandle; //底盘板任务句柄；未创建时为 NULL
extern osThreadId_t ctrl_gim_TaskHandle; //云台板任务句柄；未创建时为 NULL
extern osThreadId_t ctrl_shoot_TaskHandle; //云台板任务句柄；未创建时为 NULL
extern osThreadId_t comm_can_TaskHandle; //两板任务句柄；未创建时为 NULL
extern osThreadId_t comm_dbus_TaskHandle; //云台板任务句柄；未创建时为 NULL
extern osThreadId_t comm_ref_TaskHandle; //底盘板任务句柄；未创建时为 NULL
extern osThreadId_t comm_imu_TaskHandle; //云台板任务句柄；未创建时为 NULL
extern osThreadId_t sys_init_TaskHandle; //初始化任务句柄；两板创建，退出前清空
extern osThreadId_t sys_health_TaskHandle; //两板任务句柄；未创建时为 NULL
extern osThreadId_t sys_log_TaskHandle; //两板任务句柄；未创建时为 NULL

/* 以下创建接口在内核初始化后、调度器启动前调用，已有对象不重复创建。 */
/**
 * @brief  创建 CAN、控制、日志、DBUS 和 IMU 消息队列
 * @return true=全部队列可用，false=有队列创建失败
 * @note   在 osKernelInitialize() 之后、osKernelStart() 之前调用；已有句柄不重复创建。
 * @note   消息大小使用实际类型的 sizeof，队列传递数据副本；部分创建失败的回收由 app_rtos_task_init() 负责。
 */
bool app_rtos_queue_init(void);
/**
 * @brief  创建共享快照互斥量
 * @return true=全部互斥量可用，false=有互斥量创建失败
 * @note   仅在调度器启动前创建；互斥量启用优先级继承。
 * @note   运行时只在任务上下文使用，锁内只复制数据，不进行计算或外设收发。
 */
bool app_rtos_mutex_init(void);
/**
 * @brief  创建保留的二值信号量示例
 * @return true=信号量可用，false=创建失败
 * @note   最大计数 1、初始计数 0；当前业务完成通知使用线程标志。
 */
bool app_rtos_semaphore_init(void);
/**
 * @brief  创建系统初始化状态事件标志组
 * @return true=事件组可用，false=创建失败
 * @note   sys_init_event 保存 READY / INIT_FAILED 状态；等待查询可使用 osFlagsNoClear。
 */
bool app_rtos_event_init(void);
/**
 * @brief  依次创建队列、互斥量、信号量和事件组
 * @return true=全部通信对象可用，false=某一步创建失败
 * @note   调度器启动前调用；失败时可能已有部分对象，由任务创建入口统一回收。
 */
bool app_rtos_ipc_init(void);
/**
 * @brief  创建通信对象并按 CURRENT_BOARD 创建本板任务
 * @return true=已完成或本次创建成功，false=创建失败并回收对象
 * @note   由 freertos.c 在 osKernelInitialize() 之后、osKernelStart() 之前调用。
 * @note   云台板创建 8 个任务，底盘板创建 6 个任务；未创建句柄保持 NULL，栈大小单位为字节。
 * @note   成功后重复调用直接返回；不用于调度器运行期间重新创建任务。
 * @note   启用分配失败钩子时，内存不足可能先由钩子记录故障码并停止，不返回到回滚分支。
 */
bool app_rtos_task_init(void);
/* 任务索引与 app_rtos_task_table 的顺序一致，修改时须同时核对配置表。 */
enum {
    TASK_CHAS,    //底盘控制
    TASK_GIM,     //云台控制
    TASK_SHOOT,   //发射控制
    TASK_CAN,     //CAN 通信
    TASK_DBUS,    //遥控接收
    TASK_REF,     //裁判接收
    TASK_IMU,     //IMU 解算
    TASK_INIT,    //一次性初始化
    TASK_HEALTH,  //健康监测
    TASK_LOG,     //日志/波形
    TASK_COUNT    //任务种类数量，不是当前板型实际创建数
};
/** @brief 任务创建与健康查询共用的配置条目。 */
typedef struct {
    const char *name;           //保持原任务名称，供内核/调试器显示
    osThreadId_t *handle;       //对应全局任务句柄的地址
    osThreadFunc_t entry;       //CMSIS 任务入口函数
    uint32_t stack;            //任务栈大小，单位字节
    osPriority_t priority;     //CMSIS 任务优先级
    uint8_t enabled;           //当前板型是否创建本任务
} app_rtos_task_desc_t;
extern const app_rtos_task_desc_t app_rtos_task_table[TASK_COUNT]; //板型任务配置表，顺序对应 TASK_*
/**
 * @brief  单任务运行统计，由任务维护自己的条目，健康任务读取
 * @note   这是多字段统计记录，读取时不保证所有字段属于同一瞬间。
 */
typedef struct {
    uint32_t heartbeat,        //最近心跳时间，单位内核 tick
             heartbeat_count, //累计心跳更新次数
             cycles,          //累计完成的业务轮数；事件任务按处理批次计
             missed,          //周期任务跳过的过期节拍数量
             delay_errors,    //排除正常到期竞争后的延时 API 错误
             snapshot_failures; //快照非阻塞读取失败次数
    uint32_t max_execution_us, //最大经过时间，含抢占/阻塞，单位 us
             max_wakeup_us,    //相对目标 tick 的最大开始执行延迟，单位 us
             max_imu_age_ms;   //云台使用 IMU 数据的最大年龄，单位 ms
} app_rtos_task_stats_t;
extern app_rtos_task_stats_t app_rtos_task_stats[TASK_COUNT]; //任务运行统计表，供健康监测/调试读取
/* 诊断计数索引：由发生问题的任务/中断累计，健康任务汇总。 */
enum {
    COUNT_CAN1_RX_DROP,      //CAN1 原始接收队列投递失败
    COUNT_CAN2_RX_DROP,      //CAN2 原始接收队列投递失败
    COUNT_CAN1_TX_DROP,      //CAN1 发送队列或待发槽满
    COUNT_CAN2_TX_DROP,      //CAN2 发送队列或待发槽满
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
    COUNT_COUNT            //计数项总数
};
extern volatile uint32_t app_rtos_counters[COUNT_COUNT]; //共享诊断计数，使用 app_rtos_count() 增加
/**
 * @brief  增加指定诊断计数
 * @param  index        COUNT_* 索引，越界时忽略
 * @note   允许任务和合法 RTOS 中断上下文调用；保存并恢复原中断屏蔽状态，避免计数读改写丢失。
 */
void app_rtos_count(unsigned index);
/**
 * @brief  更新当前任务的心跳时间与次数
 * @param  task         调用者对应的 TASK_* 索引，必须小于 TASK_COUNT
 * @note   由任务更新自己的统计条目，健康任务据此监测存活状态。
 */
void app_rtos_heartbeat(unsigned task);
/**
 * @brief  向日志队列投递一条固定长度日志事件
 * @param  source       来源任务编号 TASK_*
 * @param  level        日志级别，当前约定 1=信息、2=告警、3=错误
 * @param  code         事件码，由日志任务输出并由上位机解释
 * @param  a            事件整数参数 0
 * @param  b            事件整数参数 1
 * @note   投递超时为 0，队列满或投递失败只增加 COUNT_LOG_DROP，不等待日志发送。
 * @note   TEXT 模式输出文本；VOFA 模式消费事件，保持串口字节流为纯波形数据。
 */
void app_rtos_log(uint8_t source,uint8_t level,uint16_t code,int32_t a,int32_t b);
/**
 * @brief  投递控制结果并唤醒 CAN 任务
 * @param  message      有效的控制结果指针，消息内容复制到 ctrl_to_can_queue
 * @note   控制任务调用，超时为 0；失败增加 COUNT_CTRL_CAN_DROP，下一控制周期再提交当前结果。
 */
void app_rtos_ctrl_send(const app_rtos_ctrl_can_msg_t *message);
extern uint32_t app_rtos_start_tick, app_rtos_init_stack_free; //共同起始 tick；初始化任务退出前保存的栈余量，单位字节
extern uint8_t app_rtos_init_completed; //初始化任务已完成，健康任务不再访问已删除的句柄
extern volatile uint32_t app_rtos_fatal_code; //1=分配失败，2=栈溢出，3=初始化延时异常，4=UART 中止失败
/* 中断与任务之间统一使用 CMSIS 线程标志通知，实际载荷由队列/快照/FIFO 保存。 */
/**
 * @brief  设置目标任务的线程标志并记录通知失败
 * @param  handle       目标任务句柄；NULL 表示未启用，直接忽略
 * @param  flags        APP_RTOS_TF_* 标志组合
 * @note   允许任务和合法 RTOS 中断调用；失败增加 COUNT_FLAG_ERROR。
 * @note   CMSIS 线程标志占用适配层任务通知，勿对同一任务混用原生 FreeRTOS task notification。
 */
void app_rtos_notify(osThreadId_t handle,uint32_t flags);
/* 通信示例模板：二值信号量保留，当前业务完成通知使用线程标志。
 * osSemaphoreRelease(app_rtos_example_binary_sem);
 * osSemaphoreAcquire(app_rtos_example_binary_sem, osWaitForever);
 * osMessageQueuePut(queue, &typed_value, 0, 0); // 中断/控制任务：超时 0，不等待
 * osThreadFlagsSet(handle, APP_RTOS_TF_DATA_READY);
 * osThreadFlagsWait(APP_RTOS_TF_DATA_READY, osFlagsWaitAny, 50);
 * osEventFlagsWait(sys_init_event, APP_RTOS_EVT_SYS_READY,
 *                 osFlagsWaitAny|osFlagsNoClear, osWaitForever);
 * 互斥量只保护数据复制，禁止在中断中获取；计算与外设操作在锁外执行。
 */
#endif
