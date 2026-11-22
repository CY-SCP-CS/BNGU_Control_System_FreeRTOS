#ifndef APP_TASK_DBUS_PORT_H
#define APP_TASK_DBUS_PORT_H
#include "app_task_dbus_io.h"
#include "drv_dbus_port.h"
bool app_task_dbus_port_start(UART_HandleTypeDef *uart);
void app_task_dbus_port_stop(void);
#endif
