/** @file drv_io_event.h
 * @brief 同步设备端口回调的事件判别；数值仅内部使用。
 */
#ifndef DRV_IO_EVENT_H
#define DRV_IO_EVENT_H

/** @brief DATA 携带有效数据；ERROR 表示传输失败；SERVICE 请求推进非阻塞收尾。 */
typedef enum
{
    DRV_IO_DATA,
    DRV_IO_ERROR,
    DRV_IO_SERVICE
} drv_io_event_t;
#endif
