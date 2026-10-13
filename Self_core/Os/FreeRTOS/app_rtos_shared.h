/**
 * @file    app_rtos_shared.h
 * @brief   任务共享数据接口: 完整快照 / 健康状态 / 裁判 FIFO
 * @note    读取快照超时为 0，失败时调用者保留旧副本；写者锁内只复制。
 * @note    快照接口只用于任务；裁判 FIFO 使用单生产者中断、单消费者任务。
 */
#ifndef APP_RTOS_SHARED_H
#define APP_RTOS_SHARED_H
#include "app_rtos.h"
/** @brief 单个外部模块的在线诊断，不用于新增控制停机。 */
typedef struct {
    const char *name;       //模块名称，指向长期有效的字符串
    uint32_t tick,          //最近有效数据的内核 tick
             threshold;   //最大允许数据年龄，当前 tick 频率为 1000 Hz
    uint8_t enabled,       //当前板型是否包含此模块
            valid,        //是否已经收到有效数据
            online;       //enabled && valid && 数据年龄不超过 threshold
} app_health_module_t;
#define APP_HEALTH_MODULE_COUNT 12 //IMU、6 个电机、DBUS、CAN1 指令、底盘角速度、功率计、裁判
/** @brief 任务心跳、完成状态和栈水位。 */
typedef struct {
    uint8_t enabled,       //当前板型是否创建该任务
            completed,     //一次性初始化任务是否已正常结束
            alive,         //启动阶段视为存活；READY 后检查心跳，初始化完成也算存活
            stack_warning; //栈余量不足 128 字节或不足分配值 20%
    uint32_t stack_free;   //栈历史最小余量，单位字节
    app_rtos_task_stats_t stats; //本轮读取的任务运行统计
} app_health_task_t;
/** @brief 健康任务发布的一份完整诊断快照。 */
typedef struct {
    uint32_t tick,          //本轮健康检查时间，单位内核 tick
             heap_total,   //配置的 FreeRTOS 堆总量，单位字节
             heap_used,    //heap_total - heap_free，包含堆管理/对齐开销
             heap_free,    //当前 FreeRTOS 堆余量，单位字节
             heap_min,     //历史最小 FreeRTOS 堆余量，单位字节
             init_stack_free; //初始化任务退出前保存的栈余量，单位字节
    uint8_t ready,          //系统业务初始化成功
            init_failed,   //系统初始化失败
            heap_warning;  //当前堆余量不足 1024 字节
    app_health_module_t module[APP_HEALTH_MODULE_COUNT]; //模块在线表
    app_health_task_t task[TASK_COUNT]; //任务健康表，包含未启用任务的状态
    uint32_t counters[COUNT_COUNT], //队列/协议/外设错误与丢包计数副本
             imu_busy,     //DMA 忙导致的采样跳过次数
             imu_errors;   //IMU DMA 启动、传输或超时错误次数
} app_sys_health_snapshot_t;
/**
 * @brief  清零各共享快照及裁判 FIFO
 * @note   由初始化任务在启动接收和发布 READY 前调用，此时没有并发读写。
 * @note   运行期间不能无锁调用本函数清空正在使用的共享状态。
 */
void app_rtos_shared_init(void);
/**
 * @brief  读取一份完整的 DBUS 遥控快照
 * @param  v            输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由云台控制及健康任务在任务上下文调用；使用 dbus_to_gim_mutex，获取锁超时为 0。
 */
bool app_rtos_dbus_read(app_dbus_snapshot_t *);
/**
 * @brief  读取一份完整的 IMU 姿态快照
 * @param  v            输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由云台控制及健康任务在任务上下文调用；使用 imu_to_gim_mutex，获取锁超时为 0。
 */
bool app_rtos_imu_read(app_imu_snapshot_t *);
/**
 * @brief  读取一份完整的 已裁决发射命令快照
 * @param  v            输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由发射控制任务在任务上下文调用；使用 dbus_to_shoot_mutex，获取锁超时为 0。
 */
bool app_rtos_shoot_read(app_shoot_command_t *);
/**
 * @brief  读取一份完整的 CAN 电机/功率计/底盘角速度反馈快照
 * @param  v            输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由控制及健康任务在任务上下文调用；使用 can_feedback_mutex，获取锁超时为 0。
 */
bool app_rtos_feedback_read(app_can_feedback_t *);
/**
 * @brief  读取一份完整的 裁判状态快照
 * @param  v            输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由健康任务（控制读取接口预留）在任务上下文调用；使用 ref_to_chas_mutex，获取锁超时为 0。
 */
bool app_rtos_ref_read(app_ref_snapshot_t *);
/**
 * @brief  读取完整控制状态快照
 * @param  v            输出控制状态，锁不可用时保留旧副本
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   任务上下文调用，ctrl_state_mutex 保护；超时为 0。
 */
bool app_rtos_ctrl_read(app_ctrl_state_t *);
/**
 * @brief  读取一份完整的 系统健康快照
 * @param  v            输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由诊断查询任务在任务上下文调用；使用 sys_health_mutex，获取锁超时为 0。
 */
bool app_rtos_health_read(app_sys_health_snapshot_t *);
/**
 * @brief  发布 DBUS 遥控快照
 * @param  v            输入快照，函数在锁内复制其内容
 * @note   写入者：comm_dbus_Task；使用 dbus_to_gim_mutex，仅允许任务上下文调用。
 * @note   等待取得锁后只复制；参数/句柄无效或获取失败时忽略本次更新。
 */
void app_rtos_dbus_write(const app_dbus_snapshot_t *);
/**
 * @brief  发布 IMU 姿态快照
 * @param  v            输入快照，函数在锁内复制其内容
 * @note   写入者：comm_imu_Task；初始化任务在启动前也发布初始对齐姿态；使用 imu_to_gim_mutex，仅允许任务上下文调用。
 * @note   等待取得锁后只复制；参数/句柄无效或获取失败时忽略本次更新。
 */
void app_rtos_imu_write(const app_imu_snapshot_t *);
/**
 * @brief  发布 已裁决发射命令快照
 * @param  v            输入快照，函数在锁内复制其内容
 * @note   写入者：ctrl_gim_Task；使用 dbus_to_shoot_mutex，仅允许任务上下文调用。
 * @note   等待取得锁后只复制；参数/句柄无效或获取失败时忽略本次更新。
 */
void app_rtos_shoot_write(const app_shoot_command_t *);
/**
 * @brief  发布 CAN 电机/功率计/底盘角速度反馈快照
 * @param  v            输入快照，函数在锁内复制其内容
 * @note   写入者：comm_can_Task；使用 can_feedback_mutex，仅允许任务上下文调用。
 * @note   等待取得锁后只复制；参数/句柄无效或获取失败时忽略本次更新。
 */
void app_rtos_feedback_write(const app_can_feedback_t *);
/**
 * @brief  发布 裁判状态快照
 * @param  v            输入快照，函数在锁内复制其内容
 * @note   写入者：comm_ref_Task；使用 ref_to_chas_mutex，仅允许任务上下文调用。
 * @note   等待取得锁后只复制；参数/句柄无效或获取失败时忽略本次更新。
 */
void app_rtos_ref_write(const app_ref_snapshot_t *);
/**
 * @brief  发布 系统健康快照
 * @param  v            输入快照，函数在锁内复制其内容
 * @note   写入者：sys_health_Task；使用 sys_health_mutex，仅允许任务上下文调用。
 * @note   等待取得锁后只复制；参数/句柄无效或获取失败时忽略本次更新。
 */
void app_rtos_health_write(const app_sys_health_snapshot_t *);
/**
 * @brief  发布云台控制输出与 IMU roll 波形
 * @param  v            有效的云台输出指针
 * @param  roll         IMU roll，单位度，写入 VOFA 通道 7
 * @param  tick         本轮控制时间，单位内核 tick
 * @note   由云台控制任务调用；持有 ctrl_state_mutex 只更新云台字段，保留发射字段。
 */
void app_rtos_gimbal_publish(const app_gimbal_output_t *,float roll,uint32_t tick);
/**
 * @brief  发布发射输出及原发射 VOFA 七通道
 * @param  v            有效的发射输出指针，vofa 数组包含通道 0～6
 * @param  tick         本轮控制时间，单位内核 tick
 * @note   由发射任务调用；持有 ctrl_state_mutex 更新发射字段，保留云台发布的通道 7。
 */
void app_rtos_shoot_publish(const app_shoot_output_t *,uint32_t tick);
/**
 * @brief  发布底盘控制输出及 VOFA 八通道
 * @param  v            有效的底盘输出指针
 * @param  tick         本轮控制时间，单位内核 tick
 * @note   由底盘控制任务调用；通道依次为 x/y/omega 目标与实际值、功率、缩放系数。
 */
void app_rtos_chassis_publish(const app_chassis_output_t *,uint32_t tick);
/**
 * @brief  读取控制快照中的 VOFA 八通道
 * @param  data         输出数组，至少容纳 8 个 float；失败时保留旧值
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   sys_log_Task 在任务中读取，超时为 0；数据映射由本板发布函数维护。
 */
bool app_rtos_vofa_read(float data[8]);
extern uint32_t app_rtos_can_command_tick; //最近有效 CAN1 控制指令时间，由 CAN 任务写、健康任务读
extern uint8_t app_rtos_can_command_valid; //是否曾收到本板有效 CAN1 控制指令
/* 裁判字节 FIFO：USART6 DMA/IDLE 单生产者 -> 裁判任务单消费者。
 * 溢出或 UART 错误请求丢弃旧片段并重新同步，避免断裂字节流继续解析。 */
/**
 * @brief  把 USART6 DMA 接收片段追加到裁判 FIFO
 * @param  bytes        接收字节指针；NULL 表示 UART 错误并请求重同步
 * @param  n            本片段字节数；bytes 非 NULL 且 n=0 时忽略
 * @note   单生产者中断上下文调用，数据立即复制，不使用互斥量；中断优先级须允许 RTOS API。
 * @note   FIFO 满时丢弃本片段并置复位请求，等待消费者丢弃旧片段和重新同步；不越权改写消费者索引。
 */
void app_rtos_ref_push_isr(const uint8_t *,uint16_t);
/**
 * @brief  从裁判 FIFO 取出有序字节并消费复位请求
 * @param  bytes        输出缓冲，cap 大于 0 时必须有效
 * @param  cap          缓冲最大容量，单位字节
 * @param  reset        有效输出指针；1=解析器需先丢弃旧解析片段并重新同步
 * @return 本次复制的字节数，不超过 cap
 * @note   仅 comm_ref_Task 作为单消费者调用；短临界区处理复位，字节复制不持有互斥量。
 * @note   复位后可能已收到新字节，调用者应先处理 reset 再解析返回的字节。
 */
uint16_t app_rtos_ref_pop(uint8_t *,uint16_t,uint8_t *reset);
#endif
