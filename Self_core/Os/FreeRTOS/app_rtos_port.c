/**
 * @file    app_rtos_port.c
 * @brief   RTOS 硬件适配实现: BSP 回调与 CMSIS 通信对象连接
 * @note    中断只复制数据、投递队列和通知；解包、姿态解算及控制计算在任务中执行。
 * @note    STM32 DWT/PRIMASK、CAN/SPI/UART/TIM 绑定集中在本文件，换 MCU 时替换对应实现。
 */
#include "app_rtos_port.h"
#include "app_rtos_shared.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "bsp_tim.h"
#include "drv_led.h"
#include "drv_buzzer.h"
/* IMU 算法实例不直接交给控制任务；运行期间经快照读取，避免半更新姿态。 */
drv_imu_t app_rtos_imu_instance;
static uint32_t s_can_seq[2],s_dbus_seq,s_sample_seq; //各总线/端口独立序号，采样序号在硬件触发时递增
static volatile uint32_t s_kernel_tick_cycles; //tick hook 更新的 DWT 锚点
/**
 * @brief  读取 DWT 32 位 CPU 周期计数
 * @return 当前 CYCCNT 值
 * @note   必须先由 app_rtos_port_init() 启用 DWT；计数随 uint32_t 回绕。
 */
uint32_t app_rtos_cycles(void) { return DWT->CYCCNT; }
/**
 * @brief  计算自起点以来的经过时间
 * @param  start        起点的 DWT 周期计数
 * @return 经过时间，单位 us
 * @note   使用无符号差处理一次回绕；间隔需小于完整 DWT 计数周期，时间包含抢占和阻塞。
 */
uint32_t app_rtos_elapsed_us(uint32_t start) { return (uint32_t)(DWT->CYCCNT-start)/(SystemCoreClock/1000000); }
/**
 * @brief  记录内核 tick 对应的 DWT 时间锚点
 * @note   由 FreeRTOS tick hook 在中断中调用，用于测量任务实际开始执行的延迟。
 */
void app_rtos_kernel_tick(void) { s_kernel_tick_cycles=DWT->CYCCNT; }
/**
 * @brief  计算周期任务相对目标 tick 的执行延迟
 * @param  target       已到期的绝对内核 tick
 * @return 延迟，单位 us，包含整 tick 延迟及当前 tick 内时间
 * @note   在短临界区读取 tick、DWT 和锚点；target 应是当前周期已经到达的截止时间。
 */
uint32_t app_rtos_wakeup_delay_us(uint32_t target) {
    /* 保证 tick 与其 DWT 锚点来自同一短采样区，随后恢复原中断状态。 */
    uint32_t key=__get_PRIMASK(); __disable_irq();
    uint32_t tick=osKernelGetTickCount(),cycles=DWT->CYCCNT,anchor=s_kernel_tick_cycles;
    __set_PRIMASK(key);
    return (uint32_t)(tick-target)*1000U+(uint32_t)(cycles-anchor)/(SystemCoreClock/1000000U);
}
/**
 * @brief  计算相邻硬件采样触发之间的时间差
 * @param  cycles       本次触发的 DWT 周期计数
 * @param  previous     上次触发的 DWT 周期计数
 * @return 时间间隔，单位 s
 * @note   使用采样触发时间，而非姿态任务处理时间；无符号减法处理一次 DWT 回绕。
 */
float app_rtos_sample_dt(uint32_t cycles,uint32_t previous) { return (float)(uint32_t)(cycles-previous)/(float)SystemCoreClock; }
/**
 * @brief  保存 PRIMASK 并屏蔽可屏蔽中断
 * @return 进入前的 PRIMASK，供 app_rtos_irq_unlock() 恢复
 * @note   适用于任务和中断中的短临界区；必须成对恢复，临界区内不得等待或阻塞。
 */
uint32_t app_rtos_irq_lock(void) { uint32_t key=__get_PRIMASK(); __disable_irq(); return key; }
/**
 * @brief  恢复进入临界区前的中断屏蔽状态
 * @param  key          app_rtos_irq_lock() 返回的 PRIMASK
 * @note   恢复原状态，不能简单调用开中断，以免破坏外层临界区。
 */
void app_rtos_irq_unlock(uint32_t key) { __set_PRIMASK(key); }
/**
 * @brief  执行数据内存屏障
 * @note   用于裁判 FIFO 的字节与读写索引发布顺序，不替代互斥量或临界区。
 */
void app_rtos_memory_barrier(void) { __DMB(); }
/**
 * @brief  将一帧提交到对应 CAN 总线的空闲邮箱
 * @param  frame        有效帧指针，bus=1/2，标准 ID，DLC 不超过 8
 * @return true=成功交给邮箱，false=邮箱忙或提交失败
 * @note   由 CAN 任务调用，提交不阻塞；false 时由调用者保留待发帧。
 * @note   成功提交不代表总线已完成发送，完成/错误由中断另行通知。
 */
bool app_rtos_can_transmit(const app_rtos_can_frame_t *frame) {
    return bsp_can_tx_frame(frame->bus==1 ? &hcan1 : &hcan2,frame->std_id,frame->data,frame->dlc)==0;
}
/**
 * @brief  把 BSP CAN 接收帧复制到对应 RTOS 队列
 * @param  can          CAN1/CAN2 HAL 句柄
 * @param  id           标准帧 ID
 * @param  bytes        BSP 临时数据指针，回调返回前复制
 * @param  len          有效数据长度，由 BSP 保证不超过 8
 * @note   在 CAN 接收中断中调用，附加 tick 和每条总线独立的序号；入队超时为 0。
 * @note   队列满记录该总线 RX_DROP，成功入队后只设置 DATA_READY，不在此处解析。
 */
static void can_rx(CAN_HandleTypeDef *can,uint32_t id,const uint8_t *bytes,uint8_t len) {
    unsigned bus=can==&hcan1 ? 0 : 1;
    app_rtos_can_frame_t frame={.std_id=id,.bus=bus+1,.dlc=len,.tick=osKernelGetTickCount(),.sequence=++s_can_seq[bus]};
    for(unsigned i=0;i<len;i++)frame.data[i]=bytes[i];
    if(osMessageQueuePut(bus ? can2_rx_queue : can1_rx_queue,&frame,0,0)!=osOK)
        app_rtos_count(bus ? COUNT_CAN2_RX_DROP : COUNT_CAN1_RX_DROP);
    else app_rtos_notify(comm_can_TaskHandle,APP_RTOS_TF_DATA_READY);
}
/**
 * @brief  将 CAN 邮箱完成或错误转换为线程标志
 * @param  can          触发事件的 CAN 句柄，当前通知由同一个 CAN 任务接收
 * @param  error        0=邮箱释放，BSP_CAN_EVENT_INVALID=非法帧，其余非零=CAN 错误
 * @note   中断上下文调用；记录错误计数后通知 TX_DONE 或 IO_ERROR。
 */
static void can_event(CAN_HandleTypeDef *can,uint32_t error) {
    (void)can;
    if(error)app_rtos_count(error==BSP_CAN_EVENT_INVALID ? COUNT_CAN_INVALID : COUNT_CAN_ERROR);
    app_rtos_notify(comm_can_TaskHandle,error ? APP_RTOS_TF_IO_ERROR : APP_RTOS_TF_TX_DONE);
}
/**
 * @brief  交付完整 DBUS 帧或接收错误
 * @param  bytes        完整 18 字节数据；NULL 表示接收错误
 * @note   USART3 接收回调上下文立即复制字节、tick、序号；成功后唤醒 DBUS 任务。
 * @note   NULL 只通知 IO_ERROR；短帧已经由驱动端口丢弃，本回调不解包。
 */
static void dbus_rx(const uint8_t bytes[18]) {
    if(!bytes) { app_rtos_count(COUNT_UART_ERROR); app_rtos_notify(comm_dbus_TaskHandle,APP_RTOS_TF_IO_ERROR); return; }
    app_rtos_dbus_frame_t frame={.tick=osKernelGetTickCount(),.sequence=++s_dbus_seq};
    for(unsigned i=0;i<18;i++)frame.bytes[i]=bytes[i];
    if(osMessageQueuePut(dbus_rx_queue,&frame,0,0)!=osOK)app_rtos_count(COUNT_DBUS_DROP);
    else app_rtos_notify(comm_dbus_TaskHandle,APP_RTOS_TF_DATA_READY);
}
/**
 * @brief  交付完整 IMU DMA 样本或推进恢复处理
 * @param  sample       完整 ACC/GYRO/芯片 ID 样本；NULL 表示驱动需要任务处理状态
 * @note   可能由 DMA 中断或驱动任务恢复路径调用；有效样本立即复制到 imu_rx_queue。
 * @note   NULL 只唤醒 IMU 任务处理未完成 DMA/错误，不代表获得了一份有效样本。
 */
static void imu_rx(const drv_imu_dma_sample_t *sample) {
    if(!sample) { app_rtos_notify(comm_imu_TaskHandle,APP_RTOS_TF_DATA_READY); return; }
    if(osMessageQueuePut(imu_rx_queue,sample,0,0)!=osOK)app_rtos_count(COUNT_IMU_DROP);
    else app_rtos_notify(comm_imu_TaskHandle,APP_RTOS_TF_DATA_READY);
}
/**
 * @brief  响应 TIM14 硬件节拍并启动一次有界采样
 * @note   云台板中断上下文记录内核 tick、采样序号、DWT 时间，尝试启动 SPI DMA。
 * @note   DMA 忙时由驱动记录跳过次数，不在中断中等待，也不进行姿态或控制计算。
 */
static void imu_tick(void) {
    (void)drv_imu_port_sample_trigger(osKernelGetTickCount(),++s_sample_seq,DWT->CYCCNT);
}
/**
 * @brief  为 IMU 初始化/标定提供可调度的毫秒等待
 * @param  ms           等待毫秒数，当前 1000 Hz 内核下等于 tick 数
 * @note   初始化任务上下文调用；等待前维护心跳，延时 API 失败记录故障码 3。
 */
static void task_delay(uint32_t ms) { app_rtos_heartbeat(TASK_INIT); if(osDelay(ms)!=osOK)app_rtos_fatal_code=3; }
/**
 * @brief  把 UART1 发送完成转换为日志任务标志
 * @param  uart         UART1 句柄
 * @note   发送完成中断中调用，只设置 TX_DONE，不复用或修改发送缓冲。
 */
static void uart_done(UART_HandleTypeDef *uart) { (void)uart; app_rtos_notify(sys_log_TaskHandle,APP_RTOS_TF_TX_DONE); }
/**
 * @brief  把 UART1 错误转换为日志任务标志
 * @param  uart         UART1 句柄
 * @param  event        BSP UART 事件，只有 BSP_UART_ERROR 需要处理
 * @note   中断中记录错误并通知 IO_ERROR；日志任务负责确认中止及缓冲回收。
 */
static void uart_error(UART_HandleTypeDef *uart,uint32_t event) {
    (void)uart;
    if(event==BSP_UART_ERROR) { app_rtos_count(COUNT_UART_ERROR); app_rtos_notify(sys_log_TaskHandle,APP_RTOS_TF_IO_ERROR); }
}
/**
 * @brief  初始化 RTOS 硬件适配并注册外设回调
 * @note   由初始化任务调用；配置 DWT 测时、LED/蜂鸣器及 CAN/UART/SPI/TIM 回调绑定。
 * @note   本函数只建立绑定，接收启动由 app_rtos_ports_start() 完成。
 */
void app_rtos_port_init(void) {
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk; DWT->CYCCNT=0; DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    drv_led_port_init(); drv_buzzer_port_init();
    /* 建立回调后再启动接收，保证第一份中断数据就有正确交付目标。 */
    bsp_can_reg_raw_callbacks(can_rx,can_event);
    bsp_uart_reg_tx_cplt_callback(&huart1,uart_done); bsp_uart_reg_rx_event_callback(&huart1,uart_error);
    drv_dbus_port_bind(dbus_rx); drv_referee_port_bind(app_rtos_ref_push_isr); drv_imu_port_bind(imu_rx,task_delay);
    bsp_tim_reg_callback(&htim14,imu_tick);
}
/**
 * @brief  启动 CAN1/CAN2 和本板串口接收
 * @return true=接收端口启动成功，false=外设启动失败
 * @note   由初始化任务调用；云台板启动 DBUS，底盘板启动裁判接收。
 * @note   返回 false 时可能已有部分端口启动，调用者发布 INIT_FAILED 状态。
 */
bool app_rtos_ports_start(void) {
    if(bsp_can_start(&hcan1,0,14)!=HAL_OK || bsp_can_start(&hcan2,14,14)!=HAL_OK)return false;
#if CURRENT_BOARD==BOARD_GIMBAL
    return drv_dbus_port_init()!=0;
#else
    return drv_referee_port_init()!=0;
#endif
}
/**
 * @brief  启动本板严格采样的硬件节拍
 * @return true=启动成功或底盘板无需启动，false=TIM14 启动失败
 * @note   云台板在 IMU 标定后清除 TIM14 计数和更新标志，再启动 1 kHz 中断；底盘板不启动 TIM14。
 */
bool app_rtos_sampling_start(void) {
#if CURRENT_BOARD==BOARD_GIMBAL
    __HAL_TIM_SET_COUNTER(&htim14,0); __HAL_TIM_CLEAR_FLAG(&htim14,TIM_FLAG_UPDATE);
    return HAL_TIM_Base_Start_IT(&htim14)==HAL_OK;
#else
    return true;
#endif
}
/**
 * @brief  使用 UART1 中断发送并等待本机发送完成
 * @param  bytes        发送缓冲，成功完成或确认中止前必须保持有效且不改写
 * @param  len          发送字节数
 * @return true=本机发送完成，false=启动失败、发送错误或超时
 * @note   由 sys_log_Task 独占调用；按报文长度/波特率估算传输时间并加 10 ms 余量。
 * @note   超时后确认中止发送；较长的文本报告不会套用固定 10 ticks 超时。
 * @note   中止失败时记录故障码 4，日志任务进入持续延时，避免复用仍被外设占用的缓冲。
 * @note   TX_DONE 只表示本机发送完成，不表示上位机已经接收。
 */
bool app_rtos_uart_send(uint8_t *bytes,uint16_t len) {
    if(!bytes || !len || !huart1.Init.BaudRate) { app_rtos_count(COUNT_UART_ERROR); return false; }
    uint32_t tick_freq=osKernelGetTickFreq();
    /* 每字节按最多 12 位保守估算；当前 UART1 为 115200、8N1（实际 10 位）。 */
    uint32_t wire_ticks=(uint32_t)(((uint64_t)len*12U*tick_freq+huart1.Init.BaudRate-1U)/huart1.Init.BaudRate);
    uint32_t margin_ticks=(tick_freq+99U)/100U; //10 ms 向上取整
    uint32_t timeout=wire_ticks+(margin_ticks ? margin_ticks : 1U);
    /* 清除上一笔事务的完成/错误标志，防止把旧通知误认为本次发送完成。 */
    (void)osThreadFlagsClear(APP_RTOS_TF_TX_DONE|APP_RTOS_TF_IO_ERROR);
    if(bsp_uart_tx(&huart1,bytes,len)!=HAL_OK) { app_rtos_count(COUNT_UART_ERROR); return false; }
    uint32_t result=osThreadFlagsWait(APP_RTOS_TF_TX_DONE|APP_RTOS_TF_IO_ERROR,osFlagsWaitAny,timeout);
    if(!(result&osFlagsError) && (result&APP_RTOS_TF_TX_DONE))return true;
    if(result==osFlagsErrorTimeout)app_rtos_count(COUNT_UART_TIMEOUT);
    /* 发送错误或超时后确认中止，确保 UART IT 不再引用缓冲才允许复用。 */
    if(HAL_UART_AbortTransmit(&huart1)!=HAL_OK) {
        app_rtos_fatal_code=4; for(;;)osDelay(50);
    }
    return false;
}
