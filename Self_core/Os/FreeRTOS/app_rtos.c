/**
 * @file    app_rtos.c
 * @brief   应用 RTOS 对象实现: 对象创建 / 板型任务创建 / 诊断 / 故障钩子
 * @note    CURRENT_BOARD 决定启用任务，属性表保留原栈分配，栈单位为字节。
 * @note    创建失败统一回收；运行时提供非阻塞消息提交和短临界区计数。
 */
#include "app_rtos.h"
#include "app_tasks.h"
#include "app_rtos_port.h"
#include "FreeRTOS.h"
#include "task.h"
/* 本工程周期参数直接以 tick 表达，要求 1 tick=1 ms；变更频率须同步调整周期。 */
_Static_assert(configTICK_RATE_HZ==1000, "This application's periods require a 1 kHz kernel tick");
/* 对象句柄只在本文件定义；其他模块通过 app_rtos.h 的 extern 声明引用。 */
osMessageQueueId_t can1_rx_queue;
osMessageQueueId_t can2_rx_queue;
osMessageQueueId_t can1_tx_queue;
osMessageQueueId_t can2_tx_queue;
osMessageQueueId_t can_to_ctrl_queue;
osMessageQueueId_t ctrl_to_can_queue;
osMessageQueueId_t sys_log_queue;
osMessageQueueId_t dbus_rx_queue;
osMessageQueueId_t imu_rx_queue;
/* 快照互斥量：全部启用优先级继承，运行时仅用于任务间的短数据复制。 */
osMutexId_t dbus_to_gim_mutex;
osMutexId_t dbus_to_shoot_mutex;
osMutexId_t imu_to_gim_mutex;
osMutexId_t ref_to_chas_mutex;
osMutexId_t can_feedback_mutex;
osMutexId_t ctrl_state_mutex;
osMutexId_t sys_health_mutex;
osEventFlagsId_t sys_init_event;
osSemaphoreId_t app_rtos_example_binary_sem;
/* 全局任务句柄初值为 NULL，只有当前板型启用的条目会创建线程。 */
osThreadId_t ctrl_chas_TaskHandle;
osThreadId_t ctrl_gim_TaskHandle;
osThreadId_t ctrl_shoot_TaskHandle;
osThreadId_t comm_can_TaskHandle;
osThreadId_t comm_dbus_TaskHandle;
osThreadId_t comm_ref_TaskHandle;
osThreadId_t comm_imu_TaskHandle;
osThreadId_t sys_init_TaskHandle;
osThreadId_t sys_health_TaskHandle;
osThreadId_t sys_log_TaskHandle;

/* 运行统计与故障代码保留在 RAM，日志不可用时也可通过调试器读取。 */
app_rtos_task_stats_t app_rtos_task_stats[TASK_COUNT];
volatile uint32_t app_rtos_counters[COUNT_COUNT];
uint32_t app_rtos_start_tick, app_rtos_init_stack_free;
uint8_t app_rtos_init_completed;
volatile uint32_t app_rtos_fatal_code;
#define GIM (CURRENT_BOARD==BOARD_GIMBAL) //云台板任务启用条件
#define CHAS (CURRENT_BOARD==BOARD_CHASSIS) //底盘板任务启用条件
/**
 * @brief  全部十种任务的名称、入口、字节栈大小、优先级及板型启用条件
 * @note   云台板启用 8 个、底盘板启用 6 个；枚举索引与表项位置保持一致。
 * @note   通信/IMU 优先于控制；云台高于发射，在同一到期 tick 先发布发射命令。
 */
const app_rtos_task_desc_t app_rtos_task_table[TASK_COUNT]={
    {"ctrl_chas_Task", &ctrl_chas_TaskHandle, Start_ctrl_chas_Task, 1536, osPriorityHigh, CHAS},
    {"ctrl_gim_Task", &ctrl_gim_TaskHandle, Start_ctrl_gim_Task, 1536, osPriorityHigh, GIM},
    {"ctrl_shoot_Task", &ctrl_shoot_TaskHandle, Start_ctrl_shoot_Task, 1024, osPriorityAboveNormal3, GIM},
    {"comm_can_Task", &comm_can_TaskHandle, Start_comm_can_Task, 1024, osPriorityHigh1, 1},
    {"comm_dbus_Task", &comm_dbus_TaskHandle, Start_comm_dbus_Task, 1024, osPriorityAboveNormal2, GIM},
    {"comm_ref_Task", &comm_ref_TaskHandle, Start_comm_ref_Task, 1024, osPriorityAboveNormal1, CHAS},
    {"comm_imu_Task", &comm_imu_TaskHandle, Start_comm_imu_Task, 1536, osPriorityHigh2, GIM},
    {"sys_init_Task", &sys_init_TaskHandle, Start_sys_init_Task, 1024, osPriorityHigh3, 1},
    {"sys_health_Task", &sys_health_TaskHandle, Start_sys_health_Task, 768, osPriorityBelowNormal, 1},
    {"sys_log_Task", &sys_log_TaskHandle, Start_sys_log_Task, 1024, osPriorityLow, 1},
};
/**
 * @brief  创建 CAN、控制、日志、DBUS 和 IMU 消息队列
 * @return true=全部队列可用，false=有队列创建失败
 * @note   在 osKernelInitialize() 之后、osKernelStart() 之前调用；已有句柄不重复创建。
 * @note   消息大小使用实际类型的 sizeof，队列传递数据副本；部分创建失败的回收由 app_rtos_task_init() 负责。
 */
bool app_rtos_queue_init(void) {
    /* 先创建两条总线的原始帧接收队列，再创建发送与应用方向队列。 */
    if(!can1_rx_queue) { const osMessageQueueAttr_t attr={.name="can1_rx_queue"}; can1_rx_queue=osMessageQueueNew(32,sizeof(app_rtos_can_frame_t),&attr); }
    if(!can1_rx_queue)return false;
    if(!can2_rx_queue) { const osMessageQueueAttr_t attr={.name="can2_rx_queue"}; can2_rx_queue=osMessageQueueNew(32,sizeof(app_rtos_can_frame_t),&attr); }
    if(!can2_rx_queue)return false;
    if(!can1_tx_queue) { const osMessageQueueAttr_t attr={.name="can1_tx_queue"}; can1_tx_queue=osMessageQueueNew(16,sizeof(app_rtos_can_frame_t),&attr); }
    if(!can1_tx_queue)return false;
    if(!can2_tx_queue) { const osMessageQueueAttr_t attr={.name="can2_tx_queue"}; can2_tx_queue=osMessageQueueNew(16,sizeof(app_rtos_can_frame_t),&attr); }
    if(!can2_tx_queue)return false;
    /* can_to_ctrl 只由本板主控制任务消费；反馈通过快照供多个控制任务读取。 */
    if(!can_to_ctrl_queue) { const osMessageQueueAttr_t attr={.name="can_to_ctrl_queue"}; can_to_ctrl_queue=osMessageQueueNew(16,sizeof(app_rtos_can_ctrl_msg_t),&attr); }
    if(!can_to_ctrl_queue)return false;
    if(!ctrl_to_can_queue) { const osMessageQueueAttr_t attr={.name="ctrl_to_can_queue"}; ctrl_to_can_queue=osMessageQueueNew(16,sizeof(app_rtos_ctrl_can_msg_t),&attr); }
    if(!ctrl_to_can_queue)return false;
    if(!sys_log_queue) { const osMessageQueueAttr_t attr={.name="sys_log_queue"}; sys_log_queue=osMessageQueueNew(16,sizeof(app_rtos_log_msg_t),&attr); }
    if(!sys_log_queue)return false;
    /* 队列持有完整自有帧/样本，DMA 缓冲可以在回调返回后继续接收。 */
    if(!dbus_rx_queue) { const osMessageQueueAttr_t attr={.name="dbus_rx_queue"}; dbus_rx_queue=osMessageQueueNew(4,sizeof(app_rtos_dbus_frame_t),&attr); }
    if(!dbus_rx_queue)return false;
    if(!imu_rx_queue) { const osMessageQueueAttr_t attr={.name="imu_rx_queue"}; imu_rx_queue=osMessageQueueNew(4,sizeof(drv_imu_dma_sample_t),&attr); }
    if(!imu_rx_queue)return false;
    return true;
}
/**
 * @brief  创建共享快照互斥量
 * @return true=全部互斥量可用，false=有互斥量创建失败
 * @note   仅在调度器启动前创建；互斥量启用优先级继承。
 * @note   运行时只在任务上下文使用，锁内只复制数据，不进行计算或外设收发。
 */
bool app_rtos_mutex_init(void) {
    /* 低优先级持锁者可继承读取者优先级，减少短复制区导致的优先级反转。 */
    if(!dbus_to_gim_mutex) { const osMutexAttr_t attr={.name="dbus_to_gim_mutex",.attr_bits=osMutexPrioInherit}; dbus_to_gim_mutex=osMutexNew(&attr); }
    if(!dbus_to_gim_mutex)return false;
    if(!dbus_to_shoot_mutex) { const osMutexAttr_t attr={.name="dbus_to_shoot_mutex",.attr_bits=osMutexPrioInherit}; dbus_to_shoot_mutex=osMutexNew(&attr); }
    if(!dbus_to_shoot_mutex)return false;
    if(!imu_to_gim_mutex) { const osMutexAttr_t attr={.name="imu_to_gim_mutex",.attr_bits=osMutexPrioInherit}; imu_to_gim_mutex=osMutexNew(&attr); }
    if(!imu_to_gim_mutex)return false;
    if(!ref_to_chas_mutex) { const osMutexAttr_t attr={.name="ref_to_chas_mutex",.attr_bits=osMutexPrioInherit}; ref_to_chas_mutex=osMutexNew(&attr); }
    if(!ref_to_chas_mutex)return false;
    if(!can_feedback_mutex) { const osMutexAttr_t attr={.name="can_feedback_mutex",.attr_bits=osMutexPrioInherit}; can_feedback_mutex=osMutexNew(&attr); }
    if(!can_feedback_mutex)return false;
    if(!ctrl_state_mutex) { const osMutexAttr_t attr={.name="ctrl_state_mutex",.attr_bits=osMutexPrioInherit}; ctrl_state_mutex=osMutexNew(&attr); }
    if(!ctrl_state_mutex)return false;
    if(!sys_health_mutex) { const osMutexAttr_t attr={.name="sys_health_mutex",.attr_bits=osMutexPrioInherit}; sys_health_mutex=osMutexNew(&attr); }
    if(!sys_health_mutex)return false;

    return true;
}
/**
 * @brief  创建保留的二值信号量示例
 * @return true=信号量可用，false=创建失败
 * @note   最大计数 1、初始计数 0；当前业务完成通知使用线程标志。
 */
bool app_rtos_semaphore_init(void) {
    const osSemaphoreAttr_t attr={.name="app_rtos_example_binary_sem"};
    if(!app_rtos_example_binary_sem)app_rtos_example_binary_sem=osSemaphoreNew(1,0,&attr);
    return app_rtos_example_binary_sem!=NULL;
}
/**
 * @brief  创建系统初始化状态事件标志组
 * @return true=事件组可用，false=创建失败
 * @note   sys_init_event 保存 READY / INIT_FAILED 状态；等待查询可使用 osFlagsNoClear。
 */
bool app_rtos_event_init(void) {
    const osEventFlagsAttr_t attr={.name="sys_init_event"};
    if(!sys_init_event)sys_init_event=osEventFlagsNew(&attr);
    return sys_init_event!=NULL;
}
/**
 * @brief  依次创建队列、互斥量、信号量和事件组
 * @return true=全部通信对象可用，false=某一步创建失败
 * @note   调度器启动前调用；失败时可能已有部分对象，由任务创建入口统一回收。
 */
bool app_rtos_ipc_init(void) {
    return app_rtos_queue_init() && app_rtos_mutex_init() && app_rtos_semaphore_init() && app_rtos_event_init();
}
/**
 * @brief  回收已创建的通信对象并清空句柄
 * @note   仅供调度器启动前的创建失败回滚；不能在任务仍使用对象时调用。
 */
static void delete_objects(void) {
    if(can1_rx_queue) { osMessageQueueDelete(can1_rx_queue); can1_rx_queue=NULL; }
    if(can2_rx_queue) { osMessageQueueDelete(can2_rx_queue); can2_rx_queue=NULL; }
    if(can1_tx_queue) { osMessageQueueDelete(can1_tx_queue); can1_tx_queue=NULL; }
    if(can2_tx_queue) { osMessageQueueDelete(can2_tx_queue); can2_tx_queue=NULL; }
    if(can_to_ctrl_queue) { osMessageQueueDelete(can_to_ctrl_queue); can_to_ctrl_queue=NULL; }
    if(ctrl_to_can_queue) { osMessageQueueDelete(ctrl_to_can_queue); ctrl_to_can_queue=NULL; }
    if(sys_log_queue) { osMessageQueueDelete(sys_log_queue); sys_log_queue=NULL; }
    if(dbus_rx_queue) { osMessageQueueDelete(dbus_rx_queue); dbus_rx_queue=NULL; }
    if(imu_rx_queue) { osMessageQueueDelete(imu_rx_queue); imu_rx_queue=NULL; }
    if(dbus_to_gim_mutex) { osMutexDelete(dbus_to_gim_mutex); dbus_to_gim_mutex=NULL; }
    if(dbus_to_shoot_mutex) { osMutexDelete(dbus_to_shoot_mutex); dbus_to_shoot_mutex=NULL; }
    if(imu_to_gim_mutex) { osMutexDelete(imu_to_gim_mutex); imu_to_gim_mutex=NULL; }
    if(ref_to_chas_mutex) { osMutexDelete(ref_to_chas_mutex); ref_to_chas_mutex=NULL; }
    if(can_feedback_mutex) { osMutexDelete(can_feedback_mutex); can_feedback_mutex=NULL; }
    if(ctrl_state_mutex) { osMutexDelete(ctrl_state_mutex); ctrl_state_mutex=NULL; }
    if(sys_health_mutex) { osMutexDelete(sys_health_mutex); sys_health_mutex=NULL; }

    if(sys_init_event) { osEventFlagsDelete(sys_init_event); sys_init_event=NULL; }
    if(app_rtos_example_binary_sem) { osSemaphoreDelete(app_rtos_example_binary_sem); app_rtos_example_binary_sem=NULL; }
}
/**
 * @brief  创建通信对象并按 CURRENT_BOARD 创建本板任务
 * @return true=已完成或本次创建成功，false=创建失败并回收对象
 * @note   由 freertos.c 在 osKernelInitialize() 之后、osKernelStart() 之前调用。
 * @note   云台板创建 8 个任务，底盘板创建 6 个任务；未创建句柄保持 NULL，栈大小单位为字节。
 * @note   成功后重复调用直接返回；不用于调度器运行期间重新创建任务。
 * @note   启用分配失败钩子时，内存不足可能先由钩子记录故障码并停止，不返回到回滚分支。
 */
bool app_rtos_task_init(void) {
    static bool initialized;
    if(initialized)return true;
    /* 所有通信对象先就绪，再创建任务；内核启动后任务不会看到半套对象。 */
    if(!app_rtos_ipc_init())goto failed;
    for(unsigned i=0;i<TASK_COUNT;i++) {
        const app_rtos_task_desc_t *d=&app_rtos_task_table[i];
        if(!d->enabled)continue;
        const osThreadAttr_t attr={.name=d->name,.stack_size=d->stack,.priority=d->priority};
        *d->handle=osThreadNew(d->entry,NULL,&attr);
        if(!*d->handle)goto failed;
    }
    initialized=true; return true;
failed:
    /* 创建阶段回滚：先终止已创建线程，再回收它们引用的通信对象。 */
    for(unsigned i=0;i<TASK_COUNT;i++)if(*app_rtos_task_table[i].handle) {
        osThreadTerminate(*app_rtos_task_table[i].handle); *app_rtos_task_table[i].handle=NULL;
    }
    delete_objects(); return false;
}
/* PRIMASK 临界区保存原状态，可供任务/中断使用；内部只进行短计数操作。 */
/**
 * @brief  增加指定诊断计数
 * @param  index        COUNT_* 索引，越界时忽略
 * @note   允许任务和合法 RTOS 中断上下文调用；保存并恢复原中断屏蔽状态，避免计数读改写丢失。
 */
void app_rtos_count(unsigned index) {
    uint32_t key=app_rtos_irq_lock();
    if(index<COUNT_COUNT)app_rtos_counters[index]++;
    app_rtos_irq_unlock(key);
}
/**
 * @brief  设置目标任务的线程标志并记录通知失败
 * @param  handle       目标任务句柄；NULL 表示未启用，直接忽略
 * @param  flags        APP_RTOS_TF_* 标志组合
 * @note   允许任务和合法 RTOS 中断调用；失败增加 COUNT_FLAG_ERROR。
 * @note   CMSIS 线程标志占用适配层任务通知，勿对同一任务混用原生 FreeRTOS task notification。
 */
void app_rtos_notify(osThreadId_t handle,uint32_t flags) {
    if(handle && (osThreadFlagsSet(handle,flags)&osFlagsError)) app_rtos_count(COUNT_FLAG_ERROR);
}
/**
 * @brief  更新当前任务的心跳时间与次数
 * @param  task         调用者对应的 TASK_* 索引，必须小于 TASK_COUNT
 * @note   由任务更新自己的统计条目，健康任务据此监测存活状态。
 */
void app_rtos_heartbeat(unsigned task) {
    app_rtos_task_stats[task].heartbeat=osKernelGetTickCount();
    app_rtos_task_stats[task].heartbeat_count++;
}
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
void app_rtos_log(uint8_t source,uint8_t level,uint16_t code,int32_t a,int32_t b) {
    app_rtos_log_msg_t m={osKernelGetTickCount(),code,source,level,a,b};
    if(osMessageQueuePut(sys_log_queue,&m,0,0)!=osOK)app_rtos_count(COUNT_LOG_DROP);
}
/**
 * @brief  投递控制结果并唤醒 CAN 任务
 * @param  message      有效的控制结果指针，消息内容复制到 ctrl_to_can_queue
 * @note   控制任务调用，超时为 0；失败增加 COUNT_CTRL_CAN_DROP，下一控制周期再提交当前结果。
 */
void app_rtos_ctrl_send(const app_rtos_ctrl_can_msg_t *message) {
    if(osMessageQueuePut(ctrl_to_can_queue,message,0,0)!=osOK)app_rtos_count(COUNT_CTRL_CAN_DROP);
    else app_rtos_notify(comm_can_TaskHandle,APP_RTOS_TF_DATA_READY);
}
/**
 * @brief  处理内核检测到的任务栈溢出
 * @param  task         发生栈溢出的原生 FreeRTOS 任务句柄
 * @param  name         任务名，当前只保留参数不输出
 * @note   记录故障码 2，关闭中断后停止；此路径不依赖可能已损坏的日志或堆。
 */
void vApplicationStackOverflowHook(TaskHandle_t task,char *name) {
    (void)task; (void)name; app_rtos_fatal_code=2; taskDISABLE_INTERRUPTS(); for(;;){}
}
/**
 * @brief  处理 FreeRTOS 动态内存分配失败
 * @note   记录故障码 1，关闭中断后停止；通过 configUSE_MALLOC_FAILED_HOOK 启用。
 */
void vApplicationMallocFailedHook(void) {
    app_rtos_fatal_code=1; taskDISABLE_INTERRUPTS(); for(;;){}
}
/**
 * @brief  在内核 tick 中断中更新微秒测时锚点
 * @note   中断上下文，只记录 DWT 时间，不执行控制计算或阻塞操作。
 */
void vApplicationTickHook(void) { app_rtos_kernel_tick(); }
