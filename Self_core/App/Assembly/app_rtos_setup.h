/** @file app_rtos_setup.h
 * @brief Application composition entry called after kernel initialization and before scheduling.
 */
#ifndef APP_RTOS_SETUP_H
#define APP_RTOS_SETUP_H
#include <stdbool.h>
/* Failure terminates created threads before deleting module resources. */
bool app_rtos_create_tasks(void);
#endif
