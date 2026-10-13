/**
 * @file    app_rtos_shared.c
 * @brief   任务共享数据实现: 互斥快照 / 控制波形合并 / 有序字节 FIFO
 * @note    快照存储私有，通过读写接口复制；禁止向外暴露可变数据指针。
 * @note    每份快照使用对应互斥量；裁判 FIFO 用索引归属、短临界区及内存屏障保护。
 */
#include "app_rtos_shared.h"
#include "app_rtos_port.h"
#include <string.h>
/* 各共享对象私有；读写者通过接口取得完整副本，不能直接保存共享地址。 */
static app_dbus_snapshot_t s_dbus; //DBUS -> 云台/健康
static app_imu_snapshot_t s_imu; //IMU -> 云台/健康
static app_shoot_command_t s_shoot; //云台裁决结果 -> 发射
static app_can_feedback_t s_feedback; //CAN -> 控制/健康
static app_ref_snapshot_t s_ref; //裁判状态，当前主要供健康读取
static app_ctrl_state_t s_ctrl; //各控制任务独立字段与 VOFA 合并状态
static app_sys_health_snapshot_t s_health; //健康 -> 诊断查询
uint32_t app_rtos_can_command_tick;
uint8_t app_rtos_can_command_valid;
static uint8_t s_fifo[1024]; //容量为 2 的幂，索引通过 &1023 回绕
static volatile uint32_t s_read,s_write; //消费者只推进 s_read，生产者只推进 s_write
static volatile uint8_t s_reset; //溢出/错误复位请求，由消费者在短临界区消费
/* 写者等待锁后只复制；读者获取锁超时为 0，失败保留调用者上一份副本。 */
/**
 * @brief  在短互斥区内复制共享数据到任务私有副本
 * @param  mutex        保护该数据的互斥量
 * @param  shared       共享存储地址
 * @param  local        任务私有输出地址
 * @param  n            复制字节数
 * @return true=复制完成，false=参数/句柄无效或锁暂不可用
 * @note   获取锁超时为 0，失败不改写 local；仅任务上下文使用。
 */
static bool copy_read(osMutexId_t mutex,const void *shared,void *local,size_t n) {
    if(!mutex || !shared || !local)return false;
    if(osMutexAcquire(mutex,0)!=osOK)return false;
    memcpy(local,shared,n); osMutexRelease(mutex); return true;
}
/**
 * @brief  取得互斥量后发布完整共享副本
 * @param  mutex        保护该数据的互斥量
 * @param  shared       共享输出地址
 * @param  local        完整输入副本
 * @param  n            复制字节数
 * @note   任务上下文等待锁，锁内只 memcpy；计算和设备发送必须在锁外完成。
 */
static void copy_write(osMutexId_t mutex,void *shared,const void *local,size_t n) {
    if(!mutex || !shared || !local)return;
    if(osMutexAcquire(mutex,osWaitForever)==osOK) { memcpy(shared,local,n); osMutexRelease(mutex); }
}
/**
 * @brief  读取一份完整的 DBUS 遥控快照
 * @param  v            输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由云台控制及健康任务在任务上下文调用；使用 dbus_to_gim_mutex，获取锁超时为 0。
 */
bool app_rtos_dbus_read(app_dbus_snapshot_t *v) { return copy_read(dbus_to_gim_mutex,&s_dbus,v,sizeof(*v)); }
/**
 * @brief  发布 DBUS 遥控快照
 * @param  v            输入快照，函数在锁内复制其内容
 * @note   写入者：comm_dbus_Task；使用 dbus_to_gim_mutex，仅允许任务上下文调用。
 * @note   等待取得锁后只复制；参数/句柄无效或获取失败时忽略本次更新。
 */
void app_rtos_dbus_write(const app_dbus_snapshot_t *v) { copy_write(dbus_to_gim_mutex,&s_dbus,v,sizeof(*v)); }
/**
 * @brief  读取一份完整的 IMU 姿态快照
 * @param  v            输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由云台控制及健康任务在任务上下文调用；使用 imu_to_gim_mutex，获取锁超时为 0。
 */
bool app_rtos_imu_read(app_imu_snapshot_t *v) { return copy_read(imu_to_gim_mutex,&s_imu,v,sizeof(*v)); }
/**
 * @brief  发布 IMU 姿态快照
 * @param  v            输入快照，函数在锁内复制其内容
 * @note   写入者：comm_imu_Task；初始化任务在启动前也发布初始对齐姿态；使用 imu_to_gim_mutex，仅允许任务上下文调用。
 * @note   等待取得锁后只复制；参数/句柄无效或获取失败时忽略本次更新。
 */
void app_rtos_imu_write(const app_imu_snapshot_t *v) { copy_write(imu_to_gim_mutex,&s_imu,v,sizeof(*v)); }
/**
 * @brief  读取一份完整的 已裁决发射命令快照
 * @param  v            输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由发射控制任务在任务上下文调用；使用 dbus_to_shoot_mutex，获取锁超时为 0。
 */
bool app_rtos_shoot_read(app_shoot_command_t *v) { return copy_read(dbus_to_shoot_mutex,&s_shoot,v,sizeof(*v)); }
/**
 * @brief  发布 已裁决发射命令快照
 * @param  v            输入快照，函数在锁内复制其内容
 * @note   写入者：ctrl_gim_Task；使用 dbus_to_shoot_mutex，仅允许任务上下文调用。
 * @note   等待取得锁后只复制；参数/句柄无效或获取失败时忽略本次更新。
 */
void app_rtos_shoot_write(const app_shoot_command_t *v) { copy_write(dbus_to_shoot_mutex,&s_shoot,v,sizeof(*v)); }
/**
 * @brief  读取一份完整的 CAN 电机/功率计/底盘角速度反馈快照
 * @param  v            输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由控制及健康任务在任务上下文调用；使用 can_feedback_mutex，获取锁超时为 0。
 */
bool app_rtos_feedback_read(app_can_feedback_t *v) { return copy_read(can_feedback_mutex,&s_feedback,v,sizeof(*v)); }
/**
 * @brief  发布 CAN 电机/功率计/底盘角速度反馈快照
 * @param  v            输入快照，函数在锁内复制其内容
 * @note   写入者：comm_can_Task；使用 can_feedback_mutex，仅允许任务上下文调用。
 * @note   等待取得锁后只复制；参数/句柄无效或获取失败时忽略本次更新。
 */
void app_rtos_feedback_write(const app_can_feedback_t *v) { copy_write(can_feedback_mutex,&s_feedback,v,sizeof(*v)); }
/**
 * @brief  读取一份完整的 裁判状态快照
 * @param  v            输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由健康任务（控制读取接口预留）在任务上下文调用；使用 ref_to_chas_mutex，获取锁超时为 0。
 */
bool app_rtos_ref_read(app_ref_snapshot_t *v) { return copy_read(ref_to_chas_mutex,&s_ref,v,sizeof(*v)); }
/**
 * @brief  发布 裁判状态快照
 * @param  v            输入快照，函数在锁内复制其内容
 * @note   写入者：comm_ref_Task；使用 ref_to_chas_mutex，仅允许任务上下文调用。
 * @note   等待取得锁后只复制；参数/句柄无效或获取失败时忽略本次更新。
 */
void app_rtos_ref_write(const app_ref_snapshot_t *v) { copy_write(ref_to_chas_mutex,&s_ref,v,sizeof(*v)); }
/**
 * @brief  读取一份完整的 系统健康快照
 * @param  v            输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由诊断查询任务在任务上下文调用；使用 sys_health_mutex，获取锁超时为 0。
 */
bool app_rtos_health_read(app_sys_health_snapshot_t *v) { return copy_read(sys_health_mutex,&s_health,v,sizeof(*v)); }
/**
 * @brief  发布 系统健康快照
 * @param  v            输入快照，函数在锁内复制其内容
 * @note   写入者：sys_health_Task；使用 sys_health_mutex，仅允许任务上下文调用。
 * @note   等待取得锁后只复制；参数/句柄无效或获取失败时忽略本次更新。
 */
void app_rtos_health_write(const app_sys_health_snapshot_t *v) { copy_write(sys_health_mutex,&s_health,v,sizeof(*v)); }

/**
 * @brief  读取完整控制状态快照
 * @param  v            输出控制状态，锁不可用时保留旧副本
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   任务上下文调用，ctrl_state_mutex 保护；超时为 0。
 */
bool app_rtos_ctrl_read(app_ctrl_state_t *v) { return copy_read(ctrl_state_mutex,&s_ctrl,v,sizeof(*v)); }
/**
 * @brief  读取控制快照中的 VOFA 八通道
 * @param  v            至少容纳 8 个 float 的输出数组，失败保留旧值
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   使用 ctrl_state_mutex，任务上下文获取锁超时为 0。
 */
bool app_rtos_vofa_read(float v[8]) { return copy_read(ctrl_state_mutex,s_ctrl.vofa,v,sizeof(s_ctrl.vofa)); }
/**
 * @brief  清零各共享快照及裁判 FIFO
 * @note   由初始化任务在启动接收和发布 READY 前调用，此时没有并发读写。
 * @note   运行期间不能无锁调用本函数清空正在使用的共享状态。
 */
void app_rtos_shared_init(void) {
    memset(&s_dbus,0,sizeof(s_dbus)); memset(&s_imu,0,sizeof(s_imu)); memset(&s_shoot,0,sizeof(s_shoot));
    memset(&s_feedback,0,sizeof(s_feedback)); memset(&s_ref,0,sizeof(s_ref)); memset(&s_ctrl,0,sizeof(s_ctrl));
    memset(&s_health,0,sizeof(s_health)); s_read=s_write=0; s_reset=0;
}
/**
 * @brief  发布云台控制输出与 IMU roll 波形
 * @param  v            有效的云台输出指针
 * @param  roll         IMU roll，单位度，写入 VOFA 通道 7
 * @param  tick         本轮控制时间，单位内核 tick
 * @note   由云台控制任务调用；持有 ctrl_state_mutex 只更新云台字段，保留发射字段。
 */
void app_rtos_gimbal_publish(const app_gimbal_output_t *v,float roll,uint32_t tick) {
    if(osMutexAcquire(ctrl_state_mutex,osWaitForever)==osOK) {
        s_ctrl.gimbal=*v; s_ctrl.gimbal_tick=tick; s_ctrl.vofa[7]=roll; osMutexRelease(ctrl_state_mutex);
    }
}
/**
 * @brief  发布发射输出及原发射 VOFA 七通道
 * @param  v            有效的发射输出指针，vofa 数组包含通道 0～6
 * @param  tick         本轮控制时间，单位内核 tick
 * @note   由发射任务调用；持有 ctrl_state_mutex 更新发射字段，保留云台发布的通道 7。
 */
void app_rtos_shoot_publish(const app_shoot_output_t *v,uint32_t tick) {
    if(osMutexAcquire(ctrl_state_mutex,osWaitForever)==osOK) {
        s_ctrl.shoot=*v; s_ctrl.shoot_tick=tick; memcpy(s_ctrl.vofa,v->vofa,sizeof(v->vofa)); osMutexRelease(ctrl_state_mutex);
    }
}
/**
 * @brief  发布底盘控制输出及 VOFA 八通道
 * @param  v            有效的底盘输出指针
 * @param  tick         本轮控制时间，单位内核 tick
 * @note   由底盘控制任务调用；通道依次为 x/y/omega 目标与实际值、功率、缩放系数。
 */
void app_rtos_chassis_publish(const app_chassis_output_t *v,uint32_t tick) {
    if(osMutexAcquire(ctrl_state_mutex,osWaitForever)==osOK) {
        s_ctrl.chassis=*v; s_ctrl.chassis_tick=tick;
        s_ctrl.vofa[0]=v->target.vx_speed; s_ctrl.vofa[1]=v->state.cur_speed.vx_speed;
        s_ctrl.vofa[2]=v->target.vy_speed; s_ctrl.vofa[3]=v->state.cur_speed.vy_speed;
        s_ctrl.vofa[4]=v->target.vw_speed; s_ctrl.vofa[5]=v->state.cur_speed.vw_speed;
        s_ctrl.vofa[6]=v->state.cur_power; s_ctrl.vofa[7]=v->state.power_scale;
        osMutexRelease(ctrl_state_mutex);
    }
}
/**
 * @brief  把 USART6 DMA 接收片段追加到裁判 FIFO
 * @param  bytes        接收字节指针；NULL 表示 UART 错误并请求重同步
 * @param  n            本片段字节数；bytes 非 NULL 且 n=0 时忽略
 * @note   单生产者中断上下文调用，数据立即复制，不使用互斥量；中断优先级须允许 RTOS API。
 * @note   FIFO 满时丢弃本片段并置复位请求，等待消费者丢弃旧片段和重新同步；不越权改写消费者索引。
 */
void app_rtos_ref_push_isr(const uint8_t *bytes,uint16_t n) {
    if(!bytes) { s_reset=1; app_rtos_count(COUNT_UART_ERROR); app_rtos_notify(comm_ref_TaskHandle,APP_RTOS_TF_IO_ERROR); return; }
    if(!n)return;
    if(s_reset)return;
    if(n>sizeof(s_fifo)-(uint32_t)(s_write-s_read)) {
        s_reset=1; app_rtos_count(COUNT_REF_OVERFLOW);
        /* 生产者只提出复位请求，不改写消费者索引；复位期间新片段暂不写入。 */
        app_rtos_notify(comm_ref_TaskHandle,APP_RTOS_TF_DATA_READY); return;
    }
    for(unsigned i=0;i<n;i++)s_fifo[(s_write+i)&1023]=bytes[i];
    /* 先写完字节再发布 s_write，消费者看到新索引时才可读取对应内容。 */
    app_rtos_memory_barrier(); s_write+=n; app_rtos_notify(comm_ref_TaskHandle,APP_RTOS_TF_DATA_READY);
}
/**
 * @brief  从裁判 FIFO 取出有序字节并消费复位请求
 * @param  bytes        输出缓冲，cap 大于 0 时必须有效
 * @param  cap          缓冲最大容量，单位字节
 * @param  reset        有效输出指针；1=解析器需先丢弃旧解析片段并重新同步
 * @return 本次复制的字节数，不超过 cap
 * @note   仅 comm_ref_Task 作为单消费者调用；短临界区处理复位，字节复制不持有互斥量。
 * @note   复位后可能已收到新字节，调用者应先处理 reset 再解析返回的字节。
 */
uint16_t app_rtos_ref_pop(uint8_t *bytes,uint16_t cap,uint8_t *reset) {
    uint32_t key=app_rtos_irq_lock();
    *reset=s_reset;
    if(s_reset) { s_read=s_write; s_reset=0; }
    app_rtos_irq_unlock(key);
    uint32_t n=s_write-s_read; if(n>cap)n=cap;
    for(unsigned i=0;i<n;i++)bytes[i]=s_fifo[(s_read+i)&1023];
    /* 先完成字节读取再发布 s_read，生产者才能复用这些 FIFO 空间。 */
    app_rtos_memory_barrier(); s_read+=n; return (uint16_t)n;
}
