/**
 * @file    app_tasks.c
 * @brief   应用任务实现: 启动门控 / 周期调度 / 数据搬运 / 模块调用
 * @note    云台/底盘 1 ms、发射 5 ms 独立使用 osDelayUntil()；TIM14 只触发云台 IMU 采样。
 * @note    任务层读取输入快照、调用 App 算法并提交结果；控制算法与 PID 状态由对应任务独占。
 * @note    事件任务使用队列/FIFO 保存数据、线程标志唤醒；健康仅告警，不新增控制策略。
 */
#include "app_tasks.h"
#include "app_rtos.h"
#include "app_rtos_shared.h"
#include "app_rtos_port.h"
#include "app_init.h"
#include "app_control.h"
#include "app_sentry_gimbal.h"
#include "app_sentry_chassis.h"
#include "app_sentry_shoot.h"
#include "lib_periodic.h"
#include "lib_math.h"
#include "drv_vofa.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* 较大的任务私有工作副本使用静态 SRAM 存储，保留原栈给算法调用和上下文保存。
 * 私有工作副本不跨任务共享，跨任务数据统一通过队列或受保护快照传递。 */
static uint32_t s_sequence[TASK_COUNT]; //各任务输出序号；生成新结果时只推进自己的条目
/**
 * @brief  让初始化失败或误调用的业务任务停留在安全等待
 * @param  task         当前 TASK_* 索引
 * @note   持续维护心跳并等待 50 ticks，不执行业务；此函数不返回。
 */
static void idle_failed(unsigned task) {
    for(;;) { app_rtos_heartbeat(task); osDelay(50); }
}
/**
 * @brief  等待当前任务所需的初始化线程标志
 * @param  task         当前 TASK_* 索引
 * @param  wanted       所需启动条件，PORT_READY 或 SYS_READY
 * @note   每 50 ticks 维护一次心跳；超时是正常等待，其他标志 API 错误累计诊断。
 * @note   收到 INIT_FAILED 时普通业务任务停止业务；健康/日志入口可继续处理诊断。
 */
static void wait_start(unsigned task,uint32_t wanted) {
    for(;;) {
        app_rtos_heartbeat(task);
        uint32_t f=osThreadFlagsWait(wanted|APP_RTOS_TF_INIT_FAILED,osFlagsWaitAny,50);
        if(f&osFlagsError) { if(f!=osFlagsErrorTimeout)app_rtos_count(COUNT_FLAG_ERROR); continue; }
        if((f&APP_RTOS_TF_INIT_FAILED) && task!=TASK_HEALTH && task!=TASK_LOG)idle_failed(task);
        if(f&wanted)return;
        if((f&APP_RTOS_TF_INIT_FAILED) && (task==TASK_HEALTH || task==TASK_LOG))return;
    }
}
/**
 * @brief  等待接收、发送完成或错误事件并维护心跳
 * @param  task         当前 TASK_* 索引
 * @param  timeout      等待上限，单位内核 tick
 * @note   线程标志只负责唤醒；数据保存在队列/FIFO，多个事件合并不会丢失已入队的数据。
 */
static void wait_event(unsigned task,uint32_t timeout) {
    app_rtos_heartbeat(task);
    uint32_t f=osThreadFlagsWait(APP_RTOS_WAKE_FLAGS,osFlagsWaitAny,timeout);
    if((f&osFlagsError) && f!=osFlagsErrorTimeout)app_rtos_count(COUNT_FLAG_ERROR);
}
/* 截止时间始终沿共同相位推进。当前 CMSIS 包装对已到/过去的 tick 返回
 * osErrorParameter：恰好到期直接执行，过期推进并计漏周期，真实 API 错误另行计数。 */
/**
 * @brief  等待下一绝对截止 tick，跳过已过期周期
 * @param  task         当前周期任务 TASK_* 索引
 * @param  next         输入/输出截止 tick；过期时推进到下一未来节拍
 * @param  period       周期 tick 数，必须大于 0
 * @note   目标恰好已到就执行；已经过期只计漏周期，不连续补算旧控制。
 * @note   区分 osErrorParameter 的到期竞争和真实 API 错误；所有 tick 间隔应小于 2^31。
 * @note   返回后由调用者 next += period，保持共同相位，不以业务完成时间重定周期。
 */
static void period_wait(unsigned task,uint32_t *next,uint32_t period) {
    for(;;) {
        uint32_t now=osKernelGetTickCount();
        app_rtos_task_stats[task].missed+=lib_periodic_skip_expired(now,period,next);
        now=osKernelGetTickCount();
        if(now==*next)break;
        osStatus_t status=osDelayUntil(*next);
        if(status==osOK)break;
        now=osKernelGetTickCount();
        if(status==osErrorParameter && (int32_t)(now-*next)>=0) {
            if(now==*next)break;
            continue;
        }
        app_rtos_task_stats[task].delay_errors++; osDelay(1);
    }
    uint32_t late=app_rtos_wakeup_delay_us(*next);
    if(late>app_rtos_task_stats[task].max_wakeup_us)app_rtos_task_stats[task].max_wakeup_us=late;
    app_rtos_heartbeat(task);
}
/**
 * @brief  统计本轮任务执行次数和最大耗时
 * @param  task         当前 TASK_* 索引
 * @param  start        本轮开始时读取的 DWT 周期计数
 * @note   经过时间包含执行期间的抢占和阻塞，不是独占 CPU 的算法耗时。
 */
static void profile_end(unsigned task,uint32_t start) {
    uint32_t elapsed=app_rtos_elapsed_us(start);
    if(elapsed>app_rtos_task_stats[task].max_execution_us)app_rtos_task_stats[task].max_execution_us=elapsed;
    app_rtos_task_stats[task].cycles++;
}
/**
 * @brief  向本板已创建的其他任务发布初始化线程标志
 * @param  flags        PORT_READY / SYS_READY / INIT_FAILED 组合
 * @note   遍历板型任务表，跳过初始化任务及未启用任务；每个接收者使用自己的线程标志。
 */
static void publish_init(uint32_t flags) {
    for(unsigned i=0;i<TASK_COUNT;i++)if(i!=TASK_INIT && app_rtos_task_table[i].enabled)
        app_rtos_notify(*app_rtos_task_table[i].handle,flags);
}
/**
 * @brief  一次性系统初始化任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   两种板型均创建；初始化业务/PID/回调及接收端口，云台完成 3000 次零偏标定。
 * @note   发布共同起始 tick 和系统状态，保存初始化栈余量后退出；未在线的外部模块不阻塞就绪。
 */
void Start_sys_init_Task(void *argument) {
    (void)argument; app_rtos_heartbeat(TASK_INIT);
    /* 先建立业务状态和回调，再打开接收端口；通信/健康可在后续 IMU 标定期间运行。 */
    app_rtos_shared_init(); app_init(); app_rtos_port_init();
    bool ok=app_rtos_ports_start();
    if(ok)publish_init(APP_RTOS_TF_PORT_READY);
#if CURRENT_BOARD==BOARD_GIMBAL
    if(ok)ok=drv_imu_port_init(&app_rtos_imu_instance)!=0;
    if(ok) {
        static app_imu_snapshot_t initial;
        initial.acc=app_rtos_imu_instance.acc; initial.gyro=app_rtos_imu_instance.gyro;
        initial.quat=app_rtos_imu_instance.quat; initial.euler=app_rtos_imu_instance.euler;
        initial.tick=osKernelGetTickCount(); initial.valid=1; app_rtos_imu_write(&initial);
    }
#endif
    if(ok) {
        /* 共同相位留 2 ticks 余量；控制首轮为起点+1，发射/VOFA 为起点+5。 */
        app_rtos_start_tick=osKernelGetTickCount()+2;
        ok=app_rtos_sampling_start();
    }
    if(ok) {
        osEventFlagsSet(sys_init_event,APP_RTOS_EVT_SYS_READY);
        publish_init(APP_RTOS_TF_SYS_READY);
        app_rtos_log(TASK_INIT,1,1,CURRENT_BOARD,osKernelGetTickFreq());
    } else {
        osEventFlagsSet(sys_init_event,APP_RTOS_EVT_SYS_INIT_FAILED);
        publish_init(APP_RTOS_TF_PORT_READY|APP_RTOS_TF_INIT_FAILED);
        app_rtos_log(TASK_INIT,3,2,CURRENT_BOARD,0);
    }
    /* 先缓存初始化栈水位，再标记完成并清空句柄，健康任务不查询已删除任务。 */
    app_rtos_init_stack_free=osThreadGetStackSpace(sys_init_TaskHandle);
    app_rtos_task_stats[TASK_INIT].cycles=1;
    app_rtos_init_completed=1; sys_init_TaskHandle=NULL;
    osThreadExit();
}

/* CAN 指令队列只有本板主控制任务消费，发射任务读取云台发布的已裁决快照。 */
#if CURRENT_BOARD==BOARD_GIMBAL
static app_gimbal_input_t s_gim_input; //云台任务私有输入；快照读失败时保留上周期副本
static app_gimbal_output_t s_gim_output; //云台任务私有输出
/**
 * @brief  合并本周期云台 CAN 指令并生成更新掩码
 * @param  radar_pending 有效输出状态指针；收到 0x120 时置 1，供 200 Hz 转发阶段消费
 * @note   本板唯一的 can_to_ctrl_queue 消费者；每次最多取 16 条，同 ID 保留最后一条。
 * @note   每周期先清更新掩码，增量只应用一次，实际顺序由 App 算法保持裸机规则。
 */
static void gimbal_drain(uint8_t *radar_pending) {
    app_rtos_can_ctrl_msg_t m;
    s_gim_input.updated_mask=0;
    for(unsigned i=0;i<16 && osMessageQueueGet(can_to_ctrl_queue,&m,NULL,0)==osOK;i++) {
        switch(m.id) {
        case 0x120: s_gim_input.can.radar_speed=m.value.radar; *radar_pending=1; break;
        case 0x121: s_gim_input.can.speed_no_shoot=m.value.delta; break;
        case 0x125: s_gim_input.can.speed_shoot=m.value.delta; break;
        case 0x123: s_gim_input.can.angle_no_shoot=m.value.angle; break;
        case 0x127: s_gim_input.can.angle_shoot=m.value.angle; break;
        case 0x129: s_gim_input.can.shoot=m.value.shoot; break;
        default: continue;
        }
        s_gim_input.updated_mask|=m.updated_mask;
    }
}
#endif
/**
 * @brief  云台控制任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   仅云台板创建；SYS_READY 后独立使用 osDelayUntil()，周期 1 tick / 1 ms。
 * @note   读取 DBUS/IMU/CAN 快照，裁决控制源并执行双 yaw/pitch 控制；发布发射命令和 200 Hz 板间反馈。
 */
void Start_ctrl_gim_Task(void *argument) {
    (void)argument;
#if CURRENT_BOARD==BOARD_GIMBAL
    wait_start(TASK_GIM,APP_RTOS_TF_SYS_READY);
    uint32_t next=app_rtos_start_tick+1; uint8_t divider=0,radar_pending=0;
    for(;;) {
        period_wait(TASK_GIM,&next,1); uint32_t started=app_rtos_cycles();
        /* 快照读锁超时为 0，失败沿用上一份完整输入并计数，不等待本周期 IMU DMA。 */
        s_gim_input.tick=osKernelGetTickCount();
        if(!app_rtos_dbus_read(&s_gim_input.dbus))app_rtos_task_stats[TASK_GIM].snapshot_failures++;
        if(!app_rtos_imu_read(&s_gim_input.imu))app_rtos_task_stats[TASK_GIM].snapshot_failures++;
        if(!app_rtos_feedback_read(&s_gim_input.feedback))app_rtos_task_stats[TASK_GIM].snapshot_failures++;
        uint32_t age=s_gim_input.tick-s_gim_input.imu.tick;
        if(age>app_rtos_task_stats[TASK_GIM].max_imu_age_ms)app_rtos_task_stats[TASK_GIM].max_imu_age_ms=age;
        /* 汇总本周期 CAN 更新掩码后进入 App；控制源、急停和限位仍由业务算法裁决。 */
        gimbal_drain(&radar_pending);
        app_sentry_gimbal_ctrl_1khz(&s_gim_input,&s_gim_output);
        s_gim_output.shoot.sequence=++s_sequence[TASK_GIM];
        /* 发射只使用已裁决命令；本任务优先级高于发射，同 tick 到期时先发布。 */
        app_rtos_shoot_write(&s_gim_output.shoot);
        app_rtos_gimbal_publish(&s_gim_output,s_gim_input.imu.euler.roll,s_gim_input.tick);
        app_rtos_ctrl_can_msg_t m={.tick=s_gim_input.tick,.sequence=s_sequence[TASK_GIM],.source=TASK_GIM,.type=APP_CAN_GIM_CURRENT};
        memcpy(m.value.current,s_gim_output.current,sizeof(s_gim_output.current)); app_rtos_ctrl_send(&m);
        /* 控制每 1 ms 运行，5 轮分频为 200 Hz，保持原板间命令/姿态反馈节奏。 */
        if(++divider==5) {
            divider=0;
            if(s_gim_output.shoot.source!=APP_GIMBAL_INPUT_CAN || radar_pending) {
                m.type=APP_CAN_CHASSIS_CMD;
                /* CAN 源透传最新 0x120 载荷到 0x111；其他控制源使用本轮转换结果。 */
                if(s_gim_output.shoot.source==APP_GIMBAL_INPUT_CAN)memcpy(&m.value.chassis,&s_gim_input.can.radar_speed,8);
                else m.value.chassis=s_gim_output.chassis;
                app_rtos_ctrl_send(&m);
            }
            radar_pending=0;
            m.type=APP_CAN_GIM_FEEDBACK;
            m.value.gimbal.yaw_rate=s_gim_output.yaw_rate; m.value.gimbal.pitch_rate=s_gim_output.pitch_rate;
            m.value.gimbal.yaw=s_gim_output.yaw; m.value.gimbal.pitch=s_gim_output.pitch;
            m.value.gimbal.quat=s_gim_input.imu.quat; app_rtos_ctrl_send(&m);
        }
        profile_end(TASK_GIM,started); next+=1;
    }
#else
    idle_failed(TASK_GIM);
#endif
}
/**
 * @brief  发射控制任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   仅云台板创建；SYS_READY 后独立使用 osDelayUntil()，周期 5 ticks / 5 ms。
 * @note   读取云台裁决后的发射命令与电机反馈；摩擦轮、拨弹轮 PID 状态由本任务独占。
 */
void Start_ctrl_shoot_Task(void *argument) {
    (void)argument;
#if CURRENT_BOARD==BOARD_GIMBAL
    static app_shoot_input_t input; static app_shoot_output_t output;
    wait_start(TASK_SHOOT,APP_RTOS_TF_SYS_READY); uint32_t next=app_rtos_start_tick+5;
    for(;;) {
        period_wait(TASK_SHOOT,&next,5); uint32_t started=app_rtos_cycles();
        /* 发射任务独占摩擦轮/拨弹轮 PID；不从 CAN 指令队列争抢消息。 */
        if(!app_rtos_shoot_read(&input.command))app_rtos_task_stats[TASK_SHOOT].snapshot_failures++;
        if(!app_rtos_feedback_read(&input.feedback))app_rtos_task_stats[TASK_SHOOT].snapshot_failures++;
        app_sentry_launcher_ctrl_200hz(&input,&output);
        uint32_t tick=osKernelGetTickCount(); app_rtos_shoot_publish(&output,tick);
        app_rtos_ctrl_can_msg_t m={.tick=tick,.sequence=++s_sequence[TASK_SHOOT],.source=TASK_SHOOT,.type=APP_CAN_SHOOT_CURRENT};
        memcpy(m.value.current,output.current,sizeof(output.current)); app_rtos_ctrl_send(&m);
        profile_end(TASK_SHOOT,started); next+=5;
    }
#else
    idle_failed(TASK_SHOOT);
#endif
}
/**
 * @brief  底盘控制任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   仅底盘板创建；SYS_READY 后独立使用 osDelayUntil()，周期 1 tick / 1 ms。
 * @note   消费本板唯一的 CAN 指令队列，读取电机/功率计快照，计算并提交电流及 200 Hz 状态反馈。
 */
void Start_ctrl_chas_Task(void *argument) {
    (void)argument;
#if CURRENT_BOARD==BOARD_CHASSIS
    static app_chassis_input_t input; static app_chassis_output_t output;
    wait_start(TASK_CHAS,APP_RTOS_TF_SYS_READY); uint32_t next=app_rtos_start_tick+1;
    for(;;) {
        period_wait(TASK_CHAS,&next,1); uint32_t started=app_rtos_cycles();
        if(!app_rtos_feedback_read(&input.feedback))app_rtos_task_stats[TASK_CHAS].snapshot_failures++;
        app_rtos_can_ctrl_msg_t received;
        /* 底盘板唯一指令消费者；0x113/0x115 只保存参数，不新增未实现模式。 */
        for(unsigned i=0;i<16 && osMessageQueueGet(can_to_ctrl_queue,&received,NULL,0)==osOK;i++) {
            if(received.id==0x111)input.can.speed=received.value.speed;
            if(received.id==0x113)input.can.ackermann=received.value.ackermann;
            if(received.id==0x115)input.can.follow=received.value.follow;
        }
        /* App 保留原固定 100 W 功率目标和功率计闭环，此处未加入裁判限功率。 */
        app_sentry_chassis_ctrl(&input,&output);
        uint32_t tick=osKernelGetTickCount(); app_rtos_chassis_publish(&output,tick);
        app_rtos_ctrl_can_msg_t m={.tick=tick,.sequence=++s_sequence[TASK_CHAS],.source=TASK_CHAS,.type=APP_CAN_CHAS_CURRENT};
        memcpy(m.value.current,output.current,sizeof(output.current)); app_rtos_ctrl_send(&m);
        if(output.feedback_due) {
            m.type=APP_CAN_CHAS_FEEDBACK; m.value.chassis_state.power=output.state.cur_power;
            m.value.chassis_state.omega=output.omega; app_rtos_ctrl_send(&m);
        }
        profile_end(TASK_CHAS,started); next+=1;
    }
#else
    idle_failed(TASK_CHAS);
#endif
}

/* 以下 CAN 解码、编码、邮箱提交和待发槽全部由同一个 CAN 任务独占。 */
static app_can_feedback_t s_can_feedback; //CAN 任务私有反馈工作副本，各电机拥有独立更新时间
/**
 * @brief  解析原始 CAN 帧并发布反馈或控制指令
 * @param  frame        队列取出的完整有效帧，含总线、ID、DLC、时间和序号
 * @note   仅 CAN 任务调用；CAN2 解析电机/功率计，CAN1 解析本板指令或底盘角速度。
 * @note   反馈写入 CAN 任务私有工作副本后批量发布，控制指令发送到本板唯一消费者队列。
 * @note   无关 ID 忽略，已知协议非法 DLC/数值计数；READY 前不积压控制增量。
 */
static void can_decode(const app_rtos_can_frame_t *frame) {
    if(frame->bus==2) {
        int index=-1;
#if CURRENT_BOARD==BOARD_GIMBAL
        /* 云台反馈索引：大 yaw、小 yaw、pitch、左摩擦、拨弹、右摩擦。 */
        const uint16_t ids[6]={0x205,0x206,0x207,0x201,0x202,0x203};
        for(unsigned i=0;i<6;i++)if(frame->std_id==ids[i])index=(int)i;
#else
        /* 底盘反馈索引：右驱动、左驱动、左舵向、右舵向。 */
        const uint16_t ids[4]={0x201,0x202,0x205,0x206};
        for(unsigned i=0;i<4;i++)if(frame->std_id==ids[i])index=(int)i;
        if(frame->std_id==0x212) {
            if(frame->dlc!=8) { app_rtos_count(COUNT_CAN_INVALID); return; }
            drv_power_data_t power; drv_power_solve(frame->data,&power);
            if(power.bat_v<500 || power.bat_v>6000 || !isfinite(power.power)) { app_rtos_count(COUNT_CAN_INVALID); return; }
            s_can_feedback.power=power; s_can_feedback.power_tick=frame->tick; s_can_feedback.power_valid=1; return;
        }
#endif
        if(index>=0 && frame->dlc==8) {
            drv_motor_data_t data; drv_motor_solve_dji(frame->data,&data);
            if(data.angle>8191) { app_rtos_count(COUNT_CAN_INVALID); return; }
            s_can_feedback.motor[index]=data; s_can_feedback.motor_tick[index]=frame->tick; s_can_feedback.motor_valid[index]=1;
        } else if(index>=0)app_rtos_count(COUNT_CAN_INVALID);
        return;
    }
    app_rtos_can_ctrl_msg_t m={.id=frame->std_id,.tick=frame->tick,.sequence=frame->sequence};
#if CURRENT_BOARD==BOARD_GIMBAL
    if(frame->std_id==0x119) {
        float omega;
        if(frame->dlc<4) { app_rtos_count(COUNT_CAN_INVALID); return; }
        memcpy(&omega,frame->data,4);
        if(!isfinite(omega)||fabsf(omega)>SENTRY_CHASSIS_VW_TAR_SPEED_MAX*1.5f) { app_rtos_count(COUNT_CAN_INVALID); return; }
        s_can_feedback.omega=omega; s_can_feedback.omega_tick=frame->tick; s_can_feedback.omega_valid=1; return;
    }
    if(frame->std_id!=0x120 && frame->std_id!=0x121 && frame->std_id!=0x125 &&
        frame->std_id!=0x123 && frame->std_id!=0x127 && frame->std_id!=0x129)return;
    app_gimbal_comm_rx_t rx;
    if(!app_gimbal_comm_decode(frame->std_id,frame->data,frame->dlc,&rx,&m.updated_mask)) { app_rtos_count(COUNT_CAN_INVALID); return; }
    switch(frame->std_id) {
    case 0x120: m.value.radar=rx.radar_speed; break;
    case 0x121: m.value.delta=rx.speed_no_shoot; break;
    case 0x125: m.value.delta=rx.speed_shoot; break;
    case 0x123: m.value.angle=rx.angle_no_shoot; break;
    case 0x127: m.value.angle=rx.angle_shoot; break;
    case 0x129: m.value.shoot=rx.shoot; break;
    default: return;
    }
#else
    if(frame->std_id!=0x111 && frame->std_id!=0x113 && frame->std_id!=0x115)return;
    app_chassis_comm_rx_t rx;
    if(!app_chassis_comm_decode(frame->std_id,frame->data,frame->dlc,&rx,&m.updated_mask)) { app_rtos_count(COUNT_CAN_INVALID); return; }
    if(frame->std_id==0x111)m.value.speed=rx.speed;
    if(frame->std_id==0x113)m.value.ackermann=rx.ackermann;
    if(frame->std_id==0x115)m.value.follow=rx.follow;
#endif
    app_rtos_can_command_tick=frame->tick; app_rtos_can_command_valid=1;
    /* 云台约 6 秒标定期间端口已接收，反馈/健康照常更新；READY 前不把旧角度
     * 增量或雷达命令堆入控制队列，避免启动时执行陈旧指令。 */
    if(!(osEventFlagsGet(sys_init_event)&APP_RTOS_EVT_SYS_READY))return;
    if(osMessageQueuePut(can_to_ctrl_queue,&m,0,0)!=osOK)app_rtos_count(COUNT_CAN_CTRL_DROP);
}
/**
 * @brief  把编码好的 CAN 帧写入对应发送队列
 * @param  bus          1=CAN1，2=CAN2
 * @param  id           标准帧 ID
 * @param  bytes        编码载荷指针，至少包含 dlc 字节
 * @param  dlc          有效字节数，调用者保证不超过 8
 * @param  source       原控制结果，提供 tick 与序号
 * @note   CAN 任务内部调用，入队超时为 0；失败累计对应 TX_DROP。
 */
static void queue_frame(uint8_t bus,uint32_t id,const void *bytes,uint8_t dlc,const app_rtos_ctrl_can_msg_t *source) {
    app_rtos_can_frame_t frame={.bus=bus,.std_id=id,.dlc=dlc,.tick=source->tick,.sequence=source->sequence};
    memcpy(frame.data,bytes,dlc);
    if(osMessageQueuePut(bus==1 ? can1_tx_queue : can2_tx_queue,&frame,0,0)!=osOK)
        app_rtos_count(bus==1 ? COUNT_CAN1_TX_DROP : COUNT_CAN2_TX_DROP);
}
/**
 * @brief  将应用控制结果编码为裸机兼容 CAN 帧
 * @param  m            完整控制结果消息，以 type 选择编码方式
 * @note   CAN2 电流使用 DJI 大端格式；CAN1 板间数据保持原小端单位和协议 ID。
 * @note   一个应用结果可拆成多帧；非有限云台浮点数据和未知类型记录非法计数。
 */
static void can_encode(const app_rtos_ctrl_can_msg_t *m) {
    uint8_t bytes[8]={0};
    switch(m->type) {
    case APP_CAN_GIM_CURRENT:
    case APP_CAN_SHOOT_CURRENT:
        for(unsigned i=0;i<3;i++)drv_motor_set_dji(bytes,i,m->value.current[i]);
        queue_frame(2,m->type==APP_CAN_GIM_CURRENT ? 0x1FE : 0x200,bytes,8,m); break;
    case APP_CAN_CHAS_CURRENT:
        drv_motor_set_dji(bytes,0,m->value.current[0]); drv_motor_set_dji(bytes,1,m->value.current[1]);
        queue_frame(2,0x200,bytes,8,m); memset(bytes,0,8);
        drv_motor_set_dji(bytes,0,m->value.current[2]); drv_motor_set_dji(bytes,1,m->value.current[3]);
        queue_frame(2,0x1FF,bytes,8,m); break;
    case APP_CAN_CHASSIS_CMD: queue_frame(1,0x111,&m->value.chassis,8,m); break;
    case APP_CAN_GIM_FEEDBACK: {
        if(!isfinite(m->value.gimbal.yaw_rate) || !isfinite(m->value.gimbal.pitch_rate) ||
            !isfinite(m->value.gimbal.yaw) || !isfinite(m->value.gimbal.pitch) ||
            !isfinite(m->value.gimbal.quat.q0) || !isfinite(m->value.gimbal.quat.q1) ||
            !isfinite(m->value.gimbal.quat.q2) || !isfinite(m->value.gimbal.quat.q3)) {
            app_rtos_count(COUNT_CAN_INVALID); break;
        }
        memcpy(bytes,&m->value.gimbal.yaw_rate,4); memcpy(bytes+4,&m->value.gimbal.pitch_rate,4); queue_frame(1,0x122,bytes,8,m);
        memcpy(bytes,&m->value.gimbal.yaw,4); memcpy(bytes+4,&m->value.gimbal.pitch,4); queue_frame(1,0x124,bytes,8,m);
        int16_t q[4]={(int16_t)(m->value.gimbal.quat.q0*30000),(int16_t)(m->value.gimbal.quat.q1*30000),
            (int16_t)(m->value.gimbal.quat.q2*30000),(int16_t)(m->value.gimbal.quat.q3*30000)};
        queue_frame(1,0x233,q,8,m); break;
    }
    case APP_CAN_CHAS_FEEDBACK: {
        int16_t power=(int16_t)lib_clamp(m->value.chassis_state.power*100,-32768,32767);
        memcpy(bytes,&power,2); queue_frame(1,0x112,bytes,8,m); memset(bytes,0,8);
        memcpy(bytes,&m->value.chassis_state.omega,4); queue_frame(1,0x119,bytes,8,m); break;
    }
    default: app_rtos_count(COUNT_CAN_INVALID); break;
    }
}
/* 每条总线独立 16 个待发槽，used 表示仍待提交的最新值，只有 CAN 任务访问。 */
static struct { app_rtos_can_frame_t frame; uint8_t used; } s_pending[2][16];
static uint8_t s_tx_cursor[2]; //两条总线独立的待发槽轮转起点
/**
 * @brief  合并待发帧并向空闲 CAN 邮箱提交
 * @note   每条总线最多取 16 个队列帧；按总线/ID 合并最新值，待发槽由 CAN 任务独占。
 * @note   轮转游标避免长期偏向某个 ID；邮箱忙/提交失败时保留帧，等待下一事件继续。
 * @note   已经交给硬件邮箱的帧不撤销，本函数不是额外的发送任务。
 */
static void can_transmit(void) {
    for(unsigned bus=0;bus<2;bus++) {
        app_rtos_can_frame_t frame;
        for(unsigned n=0;n<16 && osMessageQueueGet(bus ? can2_tx_queue : can1_tx_queue,&frame,NULL,0)==osOK;n++) {
            int slot=-1,free_slot=-1;
            for(unsigned i=0;i<16;i++) {
                if(s_pending[bus][i].used && s_pending[bus][i].frame.std_id==frame.std_id) { slot=(int)i; break; }
                if(!s_pending[bus][i].used)free_slot=(int)i;
            }
            if(slot<0)slot=free_slot;
            if(slot<0) { app_rtos_count(bus ? COUNT_CAN2_TX_DROP : COUNT_CAN1_TX_DROP); continue; }
            s_pending[bus][slot].frame=frame; s_pending[bus][slot].used=1;
        }
        /* 从上次游标轮转提交；失败时保留 used，邮箱完成事件后继续尝试。 */
        unsigned origin=s_tx_cursor[bus];
        for(unsigned i=0;i<16;i++) {
            unsigned slot=(origin+i)%16;
            if(!s_pending[bus][slot].used)continue;
            frame=s_pending[bus][slot].frame;
            if(!app_rtos_can_transmit(&frame))break;
            s_pending[bus][slot].used=0; s_tx_cursor[bus]=(slot+1)%16;
        }
    }
}
/**
 * @brief  CAN 通信任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   两种板型均创建；PORT_READY 后由接收、邮箱完成或错误事件唤醒。
 * @note   独占 CAN 解码、编码和待发帧；每 tick 每条 RX 队列及控制输出各最多处理 8 条。
 */
void Start_comm_can_Task(void *argument) {
    (void)argument; wait_start(TASK_CAN,APP_RTOS_TF_PORT_READY);
    uint32_t window=osKernelGetTickCount(); uint8_t used[3]={0};
    for(;;) {
        uint32_t now=osKernelGetTickCount();
        /* 每个 1 ms 窗口重置两路接收和控制输出预算，持续流量也不能无限占用 CPU。 */
        if(now!=window) { window=now; memset(used,0,sizeof(used)); }
        uint32_t started=app_rtos_cycles(); app_rtos_can_frame_t frame;
        for(unsigned bus=0;bus<2;bus++)while(used[bus]<8 && osMessageQueueGet(bus ? can2_rx_queue : can1_rx_queue,&frame,NULL,0)==osOK) {
            used[bus]++; can_decode(&frame);
        }
        /* 一次处理批次结束后发布完整反馈，控制任务只读快照，不直接读私有解码状态。 */
        app_rtos_feedback_write(&s_can_feedback);
        app_rtos_ctrl_can_msg_t message;
        while(used[2]<8 && osMessageQueueGet(ctrl_to_can_queue,&message,NULL,0)==osOK) { used[2]++; can_encode(&message); }
        can_transmit(); profile_end(TASK_CAN,started); app_rtos_heartbeat(TASK_CAN);
        bool budget_blocked=(used[0]==8 && osMessageQueueGetCount(can1_rx_queue)) ||
            (used[1]==8 && osMessageQueueGetCount(can2_rx_queue)) || (used[2]==8 && osMessageQueueGetCount(ctrl_to_can_queue));
        /* 已用完预算且还有积压则等下一 tick；其他情况等待数据/邮箱事件并维护心跳。 */
        if(budget_blocked)osDelay(1); else wait_event(TASK_CAN,50);
    }
}

/**
 * @brief  遥控接收与解包任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   仅云台板创建；PORT_READY 后处理完整 18 字节帧，事件等待最长 50 ticks。
 * @note   执行解包、合法性校验和归一化，发布遥控快照；短帧对齐与接收重启由驱动端口处理。
 */
void Start_comm_dbus_Task(void *argument) {
    (void)argument;
#if CURRENT_BOARD==BOARD_GIMBAL
    static app_dbus_snapshot_t snapshot;
    wait_start(TASK_DBUS,APP_RTOS_TF_PORT_READY);
    for(;;) {
        app_rtos_dbus_frame_t frame;
        for(unsigned n=0;n<4 && osMessageQueueGet(dbus_rx_queue,&frame,NULL,0)==osOK;n++) {
            uint32_t started=app_rtos_cycles();
            drv_dbus_data_t data; drv_dbus_decode(frame.bytes,&data);
            /* 非法帧不覆盖旧摇杆值，但发布 valid=0；有效帧才更新输入和有效数据时间。 */
            snapshot.valid=app_control_dbus_valid(&data); snapshot.sequence=frame.sequence;
            if(snapshot.valid) {
                snapshot.data=data; snapshot.tick=frame.tick;
                (void)app_gimbal_comm_dbus_rx(&snapshot,&snapshot.input);
            } else app_rtos_count(COUNT_DBUS_INVALID);
            app_rtos_dbus_write(&snapshot); profile_end(TASK_DBUS,started);
        }
        if(!drv_dbus_port_poll())app_rtos_count(COUNT_IO_RESTART_ERROR);
        wait_event(TASK_DBUS,50);
    }
#else
    idle_failed(TASK_DBUS);
#endif
}
/**
 * @brief  裁判系统组帧与解包任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   仅底盘板创建；PORT_READY 后消费有序字节 FIFO，事件等待最长 50 ticks。
 * @note   处理分包/粘包/CRC，保存完整状态和各字段更新时间；目前不新增裁判限功率或限热量策略。
 */
void Start_comm_ref_Task(void *argument) {
    (void)argument;
#if CURRENT_BOARD==BOARD_CHASSIS
    static app_ref_snapshot_t snapshot; static uint8_t chunk[128];
    wait_start(TASK_REF,APP_RTOS_TF_PORT_READY);
    uint32_t previous_frames=0,previous_errors=0;
    for(;;) {
        uint8_t reset; uint16_t n;
        /* 每批最多 8 次、每次 128 字节，限制一次解包工作量；控制任务优先级更高。 */
        for(unsigned pass=0;pass<8;pass++) {
            uint32_t started=app_rtos_cycles();
            n=app_rtos_ref_pop(chunk,sizeof(chunk),&reset);
            if(reset)drv_referee_reset_stream();
            if(!n)break;
            drv_referee_process(chunk,n); profile_end(TASK_REF,started);
            uint32_t mask=drv_referee_get_and_clear_flags();
            uint32_t frames=drv_referee_get_frame_count();
            if(frames!=previous_frames) {
                /* 累计字段有效掩码与本次更新掩码分开，未出现在本帧的字段保留既有有效状态。 */
                uint32_t valid_mask=snapshot.data.data_valid_flags|mask;
                snapshot.data=*drv_referee_get_data(); snapshot.data.data_valid_flags=valid_mask;
                snapshot.updated_mask=mask; snapshot.tick=osKernelGetTickCount(); snapshot.sequence=frames; snapshot.valid=1;
                for(unsigned i=0;i<20;i++)if(mask&(1UL<<i))snapshot.field_tick[i]=snapshot.tick;
                app_rtos_ref_write(&snapshot); previous_frames=frames;
            }
        }
        uint32_t errors=drv_referee_get_error_count();
        uint32_t key=app_rtos_irq_lock();
        app_rtos_counters[COUNT_REF_CRC]+=errors-previous_errors;
        app_rtos_irq_unlock(key); previous_errors=errors;
        if(!drv_referee_port_poll()) { app_rtos_count(COUNT_IO_RESTART_ERROR); drv_referee_reset_stream(); }
        wait_event(TASK_REF,50);
    }
#else
    idle_failed(TASK_REF);
#endif
}
/**
 * @brief  IMU 样本处理与姿态解算任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   仅云台板创建；SYS_READY 后处理完整 DMA 样本；1 tick 超时用于恢复检查。
 * @note   解析 ACC/GYRO、转换单位、Mahony 和欧拉角解算；仅 TIM14 启动新采样，任务不另建采样节拍。
 */
void Start_comm_imu_Task(void *argument) {
    (void)argument;
#if CURRENT_BOARD==BOARD_GIMBAL
    static app_imu_snapshot_t snapshot;
    wait_start(TASK_IMU,APP_RTOS_TF_SYS_READY);
    uint32_t previous_cycles=0;
    for(;;) {
        drv_imu_dma_sample_t sample;
        for(unsigned n=0;n<4 && osMessageQueueGet(imu_rx_queue,&sample,NULL,0)==osOK;n++) {
            uint32_t started=app_rtos_cycles(); drv_imu_t *imu=&app_rtos_imu_instance;
            /* 原始六轴数据按传感器小端组合，解析与物理量转换均在任务中完成。 */
            imu->acc_raw.x=(int16_t)((sample.acc[3]<<8)|sample.acc[2]);
            imu->acc_raw.y=(int16_t)((sample.acc[5]<<8)|sample.acc[4]);
            imu->acc_raw.z=(int16_t)((sample.acc[7]<<8)|sample.acc[6]);
            imu->gyro_raw.x=(int16_t)((sample.gyro[2]<<8)|sample.gyro[1]);
            imu->gyro_raw.y=(int16_t)((sample.gyro[4]<<8)|sample.gyro[3]);
            imu->gyro_raw.z=(int16_t)((sample.gyro[6]<<8)|sample.gyro[5]);
            /* 姿态积分 dt 来自硬件触发的 DWT 时间差，与任务被调度的延迟无关。 */
            float dt=previous_cycles ? app_rtos_sample_dt(sample.cycles,previous_cycles) : 0.001f;
            previous_cycles=sample.cycles;
            /* 长时间中断无法重建丢失运动，异常间隔恢复时按标称 1 ms 积分。 */
            if(dt<=0 || dt>0.01f)dt=0.001f;
            drv_imu_data_convert(imu); drv_imu_mahony_update(imu,dt); drv_imu_quat_to_euler(imu);
            snapshot.acc=imu->acc; snapshot.gyro=imu->gyro; snapshot.quat=imu->quat; snapshot.euler=imu->euler;
            snapshot.tick=sample.tick; snapshot.sequence=sample.sequence;
            snapshot.sampled=1;
            /* 芯片 ID 有效性只用于健康监测；sampled 单独记录运行采样，避免序号回绕误报。 */
            snapshot.device_valid=sample.acc_chip_id==DRV_IMU_ACC_CHIP_ID && sample.gyro_chip_id==DRV_IMU_GYRO_CHIP_ID;
            snapshot.valid=isfinite(imu->euler.yaw)&&isfinite(imu->euler.roll)&&isfinite(imu->gyro.z);
            app_rtos_imu_write(&snapshot); profile_end(TASK_IMU,started);
        }
        drv_imu_port_poll(); wait_event(TASK_IMU,1); /* 任务超时只推进恢复，不启动新采样。 */
    }
#else
    idle_failed(TASK_IMU);
#endif
}

static app_sys_health_snapshot_t s_health; //健康任务组装后整体发布，避免暴露半更新诊断表
static app_can_feedback_t s_health_can;
static app_dbus_snapshot_t s_health_dbus;
static app_imu_snapshot_t s_health_imu;
static app_ref_snapshot_t s_health_ref;
static uint32_t s_reported_counters[COUNT_COUNT],s_reported_misses[TASK_COUNT]; //已报告计数水位，用于只输出新的变化
static uint8_t s_reported_stack_warning[TASK_COUNT],s_reported_alive[TASK_COUNT]; //上一轮已报告的告警/存活状态
/* 芯片应答只用于在线诊断，控制维持原裸机策略；初始姿态不等同于运行采样。 */
/**
 * @brief  判断 IMU 数据是否满足在线诊断条件
 * @param  snapshot     有效的完整姿态快照指针
 * @return 1=数值有效、已完成运行采样且芯片 ID 有效，0=不满足
 * @note   sampled 与序号独立，序号回绕为 0 仍有效；device_valid 只用于诊断，不新增控制停机策略。
 */
static uint8_t imu_sample_valid(const app_imu_snapshot_t *snapshot) {
    return snapshot->valid && snapshot->sampled && snapshot->device_valid;
}
/**
 * @brief  更新一个模块的在线状态并记录状态变化
 * @param  index        健康模块索引，调用者保证小于 APP_HEALTH_MODULE_COUNT
 * @param  name         模块名称，指针在健康快照使用期间保持有效
 * @param  threshold    允许的数据年龄，单位内核 tick
 * @param  enabled      该模块是否属于当前板型
 * @param  valid        是否已经有有效数据
 * @param  tick         最近一次有效数据时间
 * @note   使用健康任务本轮时间判断在线；未启用模块不告警，READY 后状态变化才产生日志。
 */
static void module(unsigned index,const char *name,uint32_t threshold,uint8_t enabled,uint8_t valid,uint32_t tick) {
    app_health_module_t *m=&s_health.module[index];
    uint8_t online=enabled && valid && (uint32_t)(s_health.tick-tick)<=threshold;
    if(s_health.ready && m->enabled && m->online!=online)app_rtos_log(TASK_HEALTH,online ? 1 : 2,100+index,online,0);
    *m=(app_health_module_t){name,tick,threshold,enabled,valid,online};
}
/**
 * @brief  系统健康监测任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   两种板型均创建；PORT_READY 后使用 osDelayUntil()，周期 20 ticks / 20 ms。
 * @note   监测模块在线、心跳、漏周期、栈/堆和丢包错误；只告警，不改变原控制策略。
 */
void Start_sys_health_Task(void *argument) {
    (void)argument; wait_start(TASK_HEALTH,APP_RTOS_TF_PORT_READY);
    uint32_t next=osKernelGetTickCount()+20; uint32_t report=osKernelGetTickCount()+1000;
    for(;;) {
        period_wait(TASK_HEALTH,&next,20); uint32_t started=app_rtos_cycles(); s_health.tick=osKernelGetTickCount();
        uint32_t flags=osEventFlagsGet(sys_init_event);
        s_health.ready=(flags&APP_RTOS_EVT_SYS_READY)!=0; s_health.init_failed=(flags&APP_RTOS_EVT_SYS_INIT_FAILED)!=0;
        (void)app_rtos_feedback_read(&s_health_can); (void)app_rtos_dbus_read(&s_health_dbus);
        (void)app_rtos_imu_read(&s_health_imu); (void)app_rtos_ref_read(&s_health_ref);
        /* 按当前板型启用模块；离线阈值在 1000 Hz 内核下分别对应毫秒。 */
        const uint8_t gimbal=CURRENT_BOARD==BOARD_GIMBAL;
        module(0,"IMU",10,gimbal,imu_sample_valid(&s_health_imu),s_health_imu.tick);
        static const char *const motor_names[6]={"motor0","motor1","motor2","motor3","motor4","motor5"};
        for(unsigned i=0;i<6;i++)module(i+1,motor_names[i],50,gimbal || i<4,s_health_can.motor_valid[i],s_health_can.motor_tick[i]);
        module(7,"DBUS",100,gimbal,s_health_dbus.valid,s_health_dbus.tick);
        module(8,"CAN1 command",200,1,app_rtos_can_command_valid,app_rtos_can_command_tick);
        module(9,"chassis omega",20,gimbal,s_health_can.omega_valid,s_health_can.omega_tick);
        module(10,"power meter",100,!gimbal,s_health_can.power_valid,s_health_can.power_tick);
        module(11,"referee",1000,!gimbal,s_health_ref.valid,s_health_ref.tick);
        uint32_t key=app_rtos_irq_lock();
        for(unsigned i=0;i<COUNT_COUNT;i++)s_health.counters[i]=app_rtos_counters[i];
        app_rtos_irq_unlock(key);
        unsigned warnings=0;
        for(unsigned i=0;i<TASK_COUNT;i++) {
            const app_rtos_task_desc_t *d=&app_rtos_task_table[i]; app_health_task_t *t=&s_health.task[i];
            t->enabled=d->enabled; t->completed=i==TASK_INIT && app_rtos_init_completed;
            t->stats=app_rtos_task_stats[i];
            /* 当前 CMSIS 适配层已返回字节，不能再乘 4；初始化已退出则读取缓存水位。 */
            t->stack_free=t->completed ? app_rtos_init_stack_free : *d->handle ? osThreadGetStackSpace(*d->handle) : 0;
            /* 栈余量不足分配值 20% 或不足 128 字节即告警，未启用任务不告警。 */
            t->stack_warning=t->enabled && (t->stack_free<128 || t->stack_free<d->stack/5);
            t->alive=t->enabled && (t->completed || !s_health.ready || (uint32_t)(s_health.tick-t->stats.heartbeat)<=100);
            if(t->enabled && (t->stack_warning || !t->alive))warnings++;
        }
        s_health.heap_total=(uint32_t)configTOTAL_HEAP_SIZE;
        s_health.heap_free=(uint32_t)xPortGetFreeHeapSize();
        s_health.heap_min=(uint32_t)xPortGetMinimumEverFreeHeapSize();
        /* 配置堆大小含管理/对齐开销；空闲堆不是整颗芯片的剩余 SRAM。 */
        s_health.heap_used=s_health.heap_free<s_health.heap_total ? s_health.heap_total-s_health.heap_free : 0;
        s_health.heap_warning=s_health.heap_free<1024; s_health.init_stack_free=app_rtos_init_stack_free;
        s_health.imu_busy=drv_imu_port_get_busy_count(); s_health.imu_errors=drv_imu_port_get_error_count();
        app_rtos_health_write(&s_health);
        /* 20 ms 更新快照，1 秒汇总日志；通过已报告水位区分新错误与历史累计。 */
        if((int32_t)(s_health.tick-report)>=0) {
            report+=1000;
            app_rtos_log(TASK_HEALTH,warnings||s_health.heap_warning ? 2 : 1,20,warnings,s_health.heap_free);
            for(unsigned i=0;i<TASK_COUNT;i++)if(s_health.task[i].enabled) {
                app_health_task_t *t=&s_health.task[i];
                if(t->stack_warning && !s_reported_stack_warning[i])app_rtos_log(TASK_HEALTH,2,21,i,t->stack_free);
                if(s_health.ready && !t->alive && s_reported_alive[i])app_rtos_log(TASK_HEALTH,2,22,i,s_health.tick-t->stats.heartbeat);
                if(t->stats.missed!=s_reported_misses[i])app_rtos_log(TASK_HEALTH,2,23,i,t->stats.missed-s_reported_misses[i]);
                s_reported_stack_warning[i]=t->stack_warning; s_reported_alive[i]=t->alive; s_reported_misses[i]=t->stats.missed;
            }
            for(unsigned i=0;i<COUNT_COUNT;i++)if(s_health.counters[i]!=s_reported_counters[i]) {
                app_rtos_log(TASK_HEALTH,2,24,i,s_health.counters[i]-s_reported_counters[i]);
                s_reported_counters[i]=s_health.counters[i];
            }
            if(s_health.ready)for(unsigned i=0;i<APP_HEALTH_MODULE_COUNT;i++)if(s_health.module[i].enabled&&!s_health.module[i].online)
                app_rtos_log(TASK_HEALTH,2,100+i,0,s_health.tick-s_health.module[i].tick);
        }
        profile_end(TASK_HEALTH,started); next+=20;
    }
}
#if !APP_LOG_VOFA
/**
 * @brief  发送一行已经格式化的文本，并维护日志任务心跳
 * @param  buffer       日志任务独占的静态发送缓冲
 * @param  capacity     缓冲容量，包含字符串终止符空间
 * @param  length       snprintf() 的返回值
 * @return true=本机发送完成，false=格式化或发送失败
 * @note   不发送字符串终止符；发送完成/确认中止后才允许改写缓冲。
 */
static bool log_text_line(uint8_t *buffer,size_t capacity,int length) {
    if(length<=0 || capacity<2)return false;
    uint16_t count=(uint16_t)((size_t)length<capacity ? (size_t)length : capacity-1);
    app_rtos_heartbeat(TASK_LOG);
    return app_rtos_uart_send(buffer,count);
}
/**
 * @brief  输出一份堆及各业务任务栈的内存监控报告
 * @param  buffer       日志任务独占的静态发送缓冲
 * @param  capacity     发送缓冲容量
 * @return true=整份报告发送完成，false=快照读取或任意一行发送失败
 * @note   大快照使用静态 SRAM；锁内只复制，整数格式化和 UART 发送在锁外执行。
 * @note   单位为字节；free_min/used_max 是任务自创建以来的栈历史水位。
 * @note   heap_used 包含堆管理开销；任务栈已在 FreeRTOS 堆内分配，不能重复相加。
 */
static bool log_memory_report(uint8_t *buffer,size_t capacity) {
    static app_sys_health_snapshot_t snapshot;
    if(!app_rtos_health_read(&snapshot)) {
        app_rtos_task_stats[TASK_LOG].snapshot_failures++;
        return false;
    }
    int length=snprintf((char *)buffer,capacity,
        "[MEM %lu] units=B heap_total=%lu used=%lu free=%lu min=%lu %s\r\n",
        (unsigned long)snapshot.tick,(unsigned long)snapshot.heap_total,
        (unsigned long)snapshot.heap_used,(unsigned long)snapshot.heap_free,
        (unsigned long)snapshot.heap_min,snapshot.heap_warning ? "WARN" : "OK");
    if(!log_text_line(buffer,capacity,length))return false;
    for(unsigned i=0;i<TASK_COUNT;i++) {
        const app_rtos_task_desc_t *d=&app_rtos_task_table[i];
        const app_health_task_t *t=&snapshot.task[i];
        if(!t->enabled) {
            length=snprintf((char *)buffer,capacity,"[STACK %lu] %s DISABLED\r\n",
                (unsigned long)snapshot.tick,d->name);
        } else {
            /* 空闲量为历史最小值；初始化退出后仍显示健康任务保存的水位。 */
            uint32_t free_min=t->stack_free<d->stack ? t->stack_free : d->stack;
            uint32_t used_max=d->stack-free_min;
            uint32_t percent=d->stack ? free_min*100U/d->stack : 0;
            const char *state=t->completed ? "EXITED" : t->alive ? "ALIVE" : "STALE";
            length=snprintf((char *)buffer,capacity,
                "[STACK %lu] %s total=%lu used_max=%lu free_min=%lu (%lu%%) %s %s\r\n",
                (unsigned long)snapshot.tick,d->name,(unsigned long)d->stack,
                (unsigned long)used_max,(unsigned long)free_min,(unsigned long)percent,
                state,t->stack_warning ? "WARN" : "OK");
        }
        /* 分行维护心跳；发送失败停止本轮，下一秒重新报告最新数据。 */
        if(!log_text_line(buffer,capacity,length))return false;
    }
    return true;
}
#endif
/**
 * @brief  UART1 日志与波形发送任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   两种板型均创建，独占 UART1；TEXT 输出日志事件并每秒输出堆/任务栈报告。
 * @note   VOFA 在 SYS_READY 后以 5 ms 周期发送原八通道波形。
 * @note   发送缓冲由本任务持有直到完成或确认中止；VOFA 字节流不混入文本。
 */
void Start_sys_log_Task(void *argument) {
    (void)argument; static uint8_t buffer[160];
    wait_start(TASK_LOG,APP_RTOS_TF_PORT_READY);
#if APP_LOG_VOFA
    wait_start(TASK_LOG,APP_RTOS_TF_SYS_READY);
    static float channels[8]; uint32_t next=app_rtos_start_tick+5;
    for(;;) {
        period_wait(TASK_LOG,&next,5);
        uint32_t started=app_rtos_cycles();
        if(!app_rtos_vofa_read(channels))app_rtos_task_stats[TASK_LOG].snapshot_failures++;
        (void)drv_vofa_pack(channels,buffer); (void)app_rtos_uart_send(buffer,DRV_VOFA_FRAME_SIZE);
        /* VOFA 串口保持纯二进制波形；消费日志事件释放队列，诊断仍保留于
         * 健康快照和计数器，不在波形流中混入 ASCII 文本。 */
        app_rtos_log_msg_t event;
        for(unsigned n=0;n<16 && osMessageQueueGet(sys_log_queue,&event,NULL,0)==osOK;n++){}
        profile_end(TASK_LOG,started); next+=5;
    }
#else
    const uint32_t report_period=osKernelGetTickFreq(); //一秒；当前内核频率 1000 Hz
    uint32_t memory_report=osKernelGetTickCount()+report_period;
    for(;;) {
        app_rtos_heartbeat(TASK_LOG); app_rtos_log_msg_t event;
        /* 无事件也必须检查报告期限；内存报告直接读取快照，不占用日志队列。 */
        osStatus_t status=osMessageQueueGet(sys_log_queue,&event,NULL,50);
        uint32_t started=app_rtos_cycles();
        if(status==osOK) {
            int length=snprintf((char *)buffer,sizeof(buffer),"[%lu] src=%u lvl=%u code=%u a=%ld b=%ld\r\n",
                (unsigned long)event.tick,(unsigned)event.source,(unsigned)event.level,
                (unsigned)event.code,(long)event.value0,(long)event.value1);
            (void)log_text_line(buffer,sizeof(buffer),length);
        }
        uint32_t now=osKernelGetTickCount();
        if((int32_t)(now-memory_report)>=0) {
            (void)log_memory_report(buffer,sizeof(buffer));
            now=osKernelGetTickCount();
            /* 保持一秒相位；拥塞时跳过旧报告，避免连续补发历史数据。 */
            memory_report+=((uint32_t)(now-memory_report)/report_period+1U)*report_period;
        }
        profile_end(TASK_LOG,started);
    }
#endif
}
