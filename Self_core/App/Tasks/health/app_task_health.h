/**
 * @file    app_task_health.h
 * @brief   健康快照与监测接口；不依赖 IMU/DBUS/裁判/电机的业务结构
 */
#ifndef APP_TASK_HEALTH_H
#define APP_TASK_HEALTH_H
#include <stdbool.h>
#include "app_rtos_types.h"
#include "app_task_monitor.h"

/** @brief 单个外部模块的在线诊断，不用于新增控制停机。 */
typedef struct
{
    const char *name;          //模块名称，指向长期有效的字符串
    uint32_t errors, overruns; //生产者提供的累计错误/忙或丢样统计
    uint32_t tick,             //最近有效数据的内核 tick
        threshold;             //最大允许数据年龄，当前 tick 频率为 1000 Hz
    uint8_t enabled,           //当前板型是否包含此模块
        valid,                 //是否已经收到有效数据
        online;                //enabled && valid && 数据年龄不超过 threshold
} app_health_module_t;

/** @brief 任务心跳、完成状态和栈水位。 */
typedef struct
{
    uint8_t enabled,             //当前板型是否创建该任务
        completed,               //一次性初始化任务是否已正常结束
        alive,                   //启动阶段视为存活；READY 后检查心跳，初始化完成也算存活
        stack_warning;           //栈余量不足 128 字节或不足分配值 20%
    uint32_t stack_free;         //栈历史最小余量，单位字节
    app_rtos_task_stats_t stats; //本轮读取的任务运行统计
} app_health_task_t;

/** @brief 健康任务发布的一份完整诊断快照。 */
typedef struct
{
    uint32_t tick,       //本轮健康检查时间，单位内核 tick
        heap_total,      //配置的 FreeRTOS 堆总量，单位字节
        heap_used,       //heap_total - heap_free，包含堆管理/对齐开销
        heap_free,       //当前 FreeRTOS 堆余量，单位字节
        heap_min,        //历史最小 FreeRTOS 堆余量，单位字节
        init_stack_free; //初始化任务退出前保存的栈余量，单位字节
    uint8_t ready,       //系统业务初始化成功
        init_failed,     //系统初始化失败
        heap_warning;    //当前堆余量不足 1024 字节
    app_health_module_t module[APP_HEALTH_MODULE_COUNT]; //模块在线表
    app_health_task_t task[TASK_COUNT];                  //任务健康表，包含未启用任务的状态
    uint32_t counters[COUNT_COUNT],                      //队列/协议/外设错误与丢包计数副本
        imu_busy,                                        //DMA 忙导致的采样跳过次数
        imu_errors;                                      //IMU DMA 启动、传输或超时错误次数
} app_sys_health_snapshot_t;

bool app_rtos_health_read(app_sys_health_snapshot_t *value); /**< 非阻塞读取；失败保留输出。 */
void app_rtos_health_write(
    const app_sys_health_snapshot_t *value); /**< 健康任务发布，锁内只复制。 */

/**
 * @brief  系统健康任务入口；在线状态、任务栈和堆的快照接口见本头文件
 * @param  argument CMSIS-RTOS2 任务入口参数，当前创建时传 NULL。
 * @note   由任务表注册并由调度器调用，业务代码不直接调用任务入口。
 */
void Start_sys_health_Task(void *argument);

/** @brief 调度器启动前创建本模块的对象与初始数据。 */
bool app_task_health_init(void);
/** @brief 仅启动失败回滚时调用，调用前确保所属任务未运行。 */
void app_task_health_deinit(void);

#endif
