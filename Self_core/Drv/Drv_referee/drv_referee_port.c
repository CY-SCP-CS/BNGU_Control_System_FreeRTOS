/** @file drv_referee_port.c
 * @brief 多实例 裁判循环 DMA 片段接收；回调只交付，任务解析。
 */
#include "drv_referee_port.h"
#include <string.h>
static drv_referee_port_t *s_ports[3];

/** @brief 查找 UART 对应的已注册接收实例。
 * @param[in] uart 事件所属 UART。
 * @return 实例；未注册返回 NULL。
 * @note 注册表只在启动/停止时写，ISR 只读。 */
static drv_referee_port_t *find_port(UART_HandleTypeDef *uart)
{
    for (unsigned i = 0; i < 3; i++)
    {
        if (s_ports[i] && s_ports[i]->uart == uart)
        {
            return s_ports[i];
        }
    }
    return NULL;
}

/** @brief 启动当前实例缓冲的 DMA 接收。
 * @param[in,out] port 已配置实例，非 NULL。
 * @return 1=已启动，0=启动失败；更新 started。
 * @note 调用者与 ISR 串行化，不等待。 */
static uint8_t restart(drv_referee_port_t *port)
{
    port->started = bsp_uart_rx_dma(port->uart, port->dma, sizeof(port->dma)) == HAL_OK;
    return port->started;
}

/** @brief 分发 UART 接收事件并同步交付新增有效字节。
 * @param[in] uart 事件所属 UART。
 * @param[in] event BSP_UART_* 事件。
 * @return 无返回值。错误通过 receive(ERROR) 报告。
 * @note ISR 有界执行；DBUS 短帧丢弃；循环 DMA 先尾后头，HT/TC/IDLE 去重。 */
static void rx_event(UART_HandleTypeDef *uart, uint32_t event)
{
    drv_referee_port_t *port = find_port(uart);
    if (!port)
    {
        return;
    }
    if (event == BSP_UART_ERROR)
    {
        port->started = 0;
        port->receive(port->context, DRV_IO_ERROR, NULL, 0);
        return;
    }
    uint16_t now = (uint16_t)(512 - bsp_uart_get_rx_dma_remain(uart));
    if (now == 512)
    {
        now = 0;
    }
    if (now == port->position)
    {
        return;
    }
    if (now > port->position)
    {
        port->receive(port->context, DRV_IO_DATA, port->dma + port->position, now - port->position);
    }
    else
    {
        port->receive(port->context, DRV_IO_DATA, port->dma + port->position, 512 - port->position);
        if (now)
        {
            port->receive(port->context, DRV_IO_DATA, port->dma, now);
        }
    }
    port->position = now;
}

uint8_t drv_referee_port_init(drv_referee_port_t *port, UART_HandleTypeDef *uart,
                              drv_referee_receive_t receive, void *context)
{
    if (!port || !uart || !receive)
    {
        return 0;
    }
    if (find_port(uart))
    {
        return 0;
    }
    int free_slot = -1;
    for (unsigned i = 0; i < 3; i++)
    {
        if (!s_ports[i])
        {
            free_slot = (int)i;
            break;
        }
    }
    if (free_slot < 0 || bsp_uart_claim_rx(uart, rx_event) != HAL_OK)
    {
        return 0;
    }
    memset(port, 0, sizeof(*port));
    port->uart = uart;
    port->receive = receive;
    port->context = context;
    s_ports[free_slot] = port;
    bsp_uart_clear_idle(uart);
    bsp_uart_enable_idle_it(uart);
    return restart(port);
}

uint8_t drv_referee_port_poll(drv_referee_port_t *port)
{
    if (!port || !port->uart)
    {
        return 0;
    }
    if (port->started)
    {
        return 1;
    }
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    (void)bsp_uart_stop_dma(port->uart);
    port->position = 0;
    uint8_t ok = restart(port);
    __set_PRIMASK(key);
    return ok;
}

void drv_referee_port_stop(drv_referee_port_t *port)
{
    if (!port || !port->uart || find_port(port->uart) != port)
    {
        return;
    }
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    (void)bsp_uart_stop_dma(port->uart);
    bsp_uart_release_rx(port->uart, rx_event);
    for (unsigned i = 0; i < 3; i++)
    {
        if (s_ports[i] == port)
        {
            s_ports[i] = NULL;
        }
    }
    port->started = 0;
    port->uart = NULL;
    __set_PRIMASK(key);
}
