/**
 * @file    app_tasks.h
 * @brief   应用任务入口声明: 控制 / 通信 / 初始化 / 健康 / 日志
 * @note    入口由 app_rtos.c 的任务表注册，不在 CubeMX 任务函数内编写业务。
 * @note    CURRENT_BOARD 决定实际创建哪些任务，未启用任务句柄保持 NULL。
 */
#ifndef APP_TASKS_H
#define APP_TASKS_H
/**
 * @brief  底盘控制任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   仅底盘板创建；SYS_READY 后独立使用 osDelayUntil()，周期 1 tick / 1 ms。
 * @note   消费本板唯一的 CAN 指令队列，读取电机/功率计快照，计算并提交电流及 200 Hz 状态反馈。
 */
void Start_ctrl_chas_Task(void *argument);
/**
 * @brief  云台控制任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   仅云台板创建；SYS_READY 后独立使用 osDelayUntil()，周期 1 tick / 1 ms。
 * @note   读取 DBUS/IMU/CAN 快照，裁决控制源并执行双 yaw/pitch 控制；发布发射命令和 200 Hz 板间反馈。
 */
void Start_ctrl_gim_Task(void *argument);
/**
 * @brief  发射控制任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   仅云台板创建；SYS_READY 后独立使用 osDelayUntil()，周期 5 ticks / 5 ms。
 * @note   读取云台裁决后的发射命令与电机反馈；摩擦轮、拨弹轮 PID 状态由本任务独占。
 */
void Start_ctrl_shoot_Task(void *argument);
/**
 * @brief  CAN 通信任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   两种板型均创建；PORT_READY 后由接收、邮箱完成或错误事件唤醒。
 * @note   独占 CAN 解码、编码和待发帧；每 tick 每条 RX 队列及控制输出各最多处理 8 条。
 */
void Start_comm_can_Task(void *argument);
/**
 * @brief  遥控接收与解包任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   仅云台板创建；PORT_READY 后处理完整 18 字节帧，事件等待最长 50 ticks。
 * @note   执行解包、合法性校验和归一化，发布遥控快照；短帧对齐与接收重启由驱动端口处理。
 */
void Start_comm_dbus_Task(void *argument);
/**
 * @brief  裁判系统组帧与解包任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   仅底盘板创建；PORT_READY 后消费有序字节 FIFO，事件等待最长 50 ticks。
 * @note   处理分包/粘包/CRC，保存完整状态和各字段更新时间；目前不新增裁判限功率或限热量策略。
 */
void Start_comm_ref_Task(void *argument);
/**
 * @brief  IMU 样本处理与姿态解算任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   仅云台板创建；SYS_READY 后处理完整 DMA 样本；1 tick 超时用于恢复检查。
 * @note   解析 ACC/GYRO、转换单位、Mahony 和欧拉角解算；仅 TIM14 启动新采样，任务不另建采样节拍。
 */
void Start_comm_imu_Task(void *argument);
/**
 * @brief  一次性系统初始化任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   两种板型均创建；初始化业务/PID/回调及接收端口，云台完成 3000 次零偏标定。
 * @note   发布共同起始 tick 和系统状态，保存初始化栈余量后退出；未在线的外部模块不阻塞就绪。
 */
void Start_sys_init_Task(void *argument);
/**
 * @brief  系统健康监测任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   两种板型均创建；PORT_READY 后使用 osDelayUntil()，周期 20 ticks / 20 ms。
 * @note   监测模块在线、心跳、漏周期、栈/堆和丢包错误；只告警，不改变原控制策略。
 */
void Start_sys_health_Task(void *argument);
/**
 * @brief  UART1 日志与波形发送任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   两种板型均创建，独占 UART1；TEXT 输出事件及每秒内存报告，VOFA 以 5 ms 周期发送。
 * @note   内存报告包含堆总量/已用/当前空闲/最低空闲及本板业务任务栈水位，单位字节。
 * @note   发送缓冲由本任务持有直到完成或确认中止；VOFA 字节流不混入文本。
 */
void Start_sys_log_Task(void *argument);
#endif
