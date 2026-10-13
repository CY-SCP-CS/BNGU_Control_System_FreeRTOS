/**
 * @file    app_rtos_port.h
 * @brief   RTOS 硬件适配接口: 外设启动 / 中断交付 / 发送 / 测时
 * @note    硬件相关实现集中于 app_rtos_port.c，任务层通过此接口访问硬件。
 * @note    调用 RTOS API 的外设中断须满足内核优先级要求，当前工程使用优先级 5。
 */
#ifndef APP_RTOS_PORT_H
#define APP_RTOS_PORT_H
#include "app_rtos.h"
/**
 * @brief  初始化 RTOS 硬件适配并注册外设回调
 * @note   由初始化任务调用；配置 DWT 测时、LED/蜂鸣器及 CAN/UART/SPI/TIM 回调绑定。
 * @note   本函数只建立绑定，接收启动由 app_rtos_ports_start() 完成。
 */
void app_rtos_port_init(void);
/**
 * @brief  启动 CAN1/CAN2 和本板串口接收
 * @return true=接收端口启动成功，false=外设启动失败
 * @note   由初始化任务调用；云台板启动 DBUS，底盘板启动裁判接收。
 * @note   返回 false 时可能已有部分端口启动，调用者发布 INIT_FAILED 状态。
 */
bool app_rtos_ports_start(void);
/**
 * @brief  启动本板严格采样的硬件节拍
 * @return true=启动成功或底盘板无需启动，false=TIM14 启动失败
 * @note   云台板在 IMU 标定后清除 TIM14 计数和更新标志，再启动 1 kHz 中断；底盘板不启动 TIM14。
 */
bool app_rtos_sampling_start(void);
/**
 * @brief  使用 UART1 中断发送并等待本机发送完成
 * @param  bytes        发送缓冲，成功完成或确认中止前必须保持有效且不改写
 * @param  len          发送字节数
 * @return true=本机发送完成，false=启动失败、发送错误或超时
 * @note   由 sys_log_Task 独占调用；按报文长度/波特率估算传输时间并加 10 ms 余量。
 * @note   超时后确认中止发送，之后才能复用缓冲。
 * @note   中止失败时记录故障码 4，日志任务进入持续延时，避免复用仍被外设占用的缓冲。
 * @note   TX_DONE 只表示本机发送完成，不表示上位机已经接收。
 */
bool app_rtos_uart_send(uint8_t *,uint16_t);
/**
 * @brief  读取 DWT 32 位 CPU 周期计数
 * @return 当前 CYCCNT 值
 * @note   必须先由 app_rtos_port_init() 启用 DWT；计数随 uint32_t 回绕。
 */
uint32_t app_rtos_cycles(void);
/**
 * @brief  计算自起点以来的经过时间
 * @param  start        起点的 DWT 周期计数
 * @return 经过时间，单位 us
 * @note   使用无符号差处理一次回绕；间隔需小于完整 DWT 计数周期，时间包含抢占和阻塞。
 */
uint32_t app_rtos_elapsed_us(uint32_t start);
/**
 * @brief  记录内核 tick 对应的 DWT 时间锚点
 * @note   由 FreeRTOS tick hook 在中断中调用，用于测量任务实际开始执行的延迟。
 */
void app_rtos_kernel_tick(void);
/**
 * @brief  计算周期任务相对目标 tick 的执行延迟
 * @param  target       已到期的绝对内核 tick
 * @return 延迟，单位 us，包含整 tick 延迟及当前 tick 内时间
 * @note   在短临界区读取 tick、DWT 和锚点；target 应是当前周期已经到达的截止时间。
 */
uint32_t app_rtos_wakeup_delay_us(uint32_t target);
/**
 * @brief  计算相邻硬件采样触发之间的时间差
 * @param  cycles       本次触发的 DWT 周期计数
 * @param  previous     上次触发的 DWT 周期计数
 * @return 时间间隔，单位 s
 * @note   使用采样触发时间，而非姿态任务处理时间；无符号减法处理一次 DWT 回绕。
 */
float app_rtos_sample_dt(uint32_t cycles, uint32_t previous);
/**
 * @brief  保存 PRIMASK 并屏蔽可屏蔽中断
 * @return 进入前的 PRIMASK，供 app_rtos_irq_unlock() 恢复
 * @note   适用于任务和中断中的短临界区；必须成对恢复，临界区内不得等待或阻塞。
 */
uint32_t app_rtos_irq_lock(void);
/**
 * @brief  恢复进入临界区前的中断屏蔽状态
 * @param  key          app_rtos_irq_lock() 返回的 PRIMASK
 * @note   恢复原状态，不能简单调用开中断，以免破坏外层临界区。
 */
void app_rtos_irq_unlock(uint32_t key);
/**
 * @brief  执行数据内存屏障
 * @note   用于裁判 FIFO 的字节与读写索引发布顺序，不替代互斥量或临界区。
 */
void app_rtos_memory_barrier(void);
/**
 * @brief  将一帧提交到对应 CAN 总线的空闲邮箱
 * @param  frame        有效帧指针，bus=1/2，标准 ID，DLC 不超过 8
 * @return true=成功交给邮箱，false=邮箱忙或提交失败
 * @note   由 CAN 任务调用，提交不阻塞；false 时由调用者保留待发帧。
 * @note   成功提交不代表总线已完成发送，完成/错误由中断另行通知。
 */
bool app_rtos_can_transmit(const app_rtos_can_frame_t *frame);
extern drv_imu_t app_rtos_imu_instance; //初始化任务配置/标定，运行期间 IMU 任务独占解算，控制读取姿态快照
#endif
