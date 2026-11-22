#ifndef APP_TASK_REFEREE_PORT_H
#define APP_TASK_REFEREE_PORT_H
#include "app_task_referee_io.h"
#include "drv_referee_port.h"
bool app_task_referee_port_start(UART_HandleTypeDef *uart);
void app_task_referee_port_stop(void);
#endif
