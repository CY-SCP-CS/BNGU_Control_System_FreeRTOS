/** @file drv_referee_port.h
 * @brief 多实例串口接收端口；静态 SRAM 缓冲归实例，纯解析头不依赖硬件。
 */
#ifndef DRV_REFEREE_PORT_H
#define DRV_REFEREE_PORT_H
#include "bsp_uart.h"
#include "drv_io_event.h"
/**
 * @brief 同步交付接收片段或传输错误。
 * @param[in,out] context 注册的接收者上下文，端口不解释，必须覆盖接收生命周期。
 * @param[in] event DATA=字节有效，ERROR=接收停止；不使用 NULL 推断事件。
 * @param[in] bytes DATA 时指向 length B 临时缓冲；其余事件不读取。
 * @param[in] length DATA 的有效 B 数，DBUS 为 18；裁判为有序片段。
 * @return 无返回值。接收者自行记录入队/复制失败。
 * @note ISR 同步调用，返回前复制，不保留指针，不阻塞。
 */
typedef void (*drv_referee_receive_t)(void *context, drv_io_event_t event, const uint8_t *bytes,
                                      uint16_t length);

/** @brief 接收实例；装配静态分配，ISR 与恢复任务短临界区串行化。 */
typedef struct
{
    UART_HandleTypeDef *uart;      /**< 借用至 stop；同一 UART 只能有一个接收者。 */
    drv_referee_receive_t receive; /**< 同步交付回调。 */
    void *context;                 /**< 接收者上下文。 */
    uint8_t dma[512];              /**< 普通 SRAM，DMA 占用期间不可由业务写入。 */
    uint16_t position;             /**< 已交付写位置 [0,511]。 */
    volatile uint8_t started;      /**< DMA 正在接收；错误 ISR 清零。 */
} drv_referee_port_t;

/**
 * @brief 注册并启动一个独立串口接收实例。
 * @param[in,out] port 静态实例，非 NULL；首次须零初始化，失败可 stop 回滚。
 * @param[in] uart 已配置 DMA 的 UART，非 NULL，接收期间借用。
 * @param[in] receive 非 NULL 同步回调，接收期间保持有效。
 * @param[in,out] context 接收者上下文，仅透传，可为 NULL。
 * @return 1=DMA 已启动；0=参数、注册冲突、表满或 DMA 启动失败。
 * @note 初始化任务调用；最多三个 UART；成功不表示已收到数据。
 */
uint8_t drv_referee_port_init(drv_referee_port_t *port, UART_HandleTypeDef *uart,
                              drv_referee_receive_t receive, void *context);
/**
 * @brief 接收错误后尝试一次 DMA 重启。
 * @param[in,out] port 已注册实例，非 NULL。
 * @return 1=运行或重启已启动；0=参数错误或启动失败，下一任务轮重试。
 * @note 任务调用，短临界区，不解析字节，不等待完成。
 */
uint8_t drv_referee_port_poll(drv_referee_port_t *port);
/**
 * @brief 停止 DMA 并解除接收实例注册。
 * @param[in,out] port 实例；NULL 时忽略。调用者确保所属业务任务尚未运行。
 * @return 无返回值。未注册时安全忽略。
 * @note 启动失败回滚调用，短临界区后 DMA 缓冲才可回收。
 */
void drv_referee_port_stop(drv_referee_port_t *port);
#endif
