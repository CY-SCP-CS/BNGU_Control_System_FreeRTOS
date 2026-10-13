#ifndef APP_SENTRY_CHASSIS_H
#define APP_SENTRY_CHASSIS_H
#include "app_data.h"
#include "lib_pid.h"
void app_sentry_chassis_init(void);
void app_sentry_chassis_ctrl(const app_chassis_input_t *, app_chassis_output_t *);
#endif
