#ifndef APP_SENTRY_SHOOT_H
#define APP_SENTRY_SHOOT_H
#include "app_data.h"
void app_sentry_shoot_init(void);
void app_sentry_launcher_ctrl_200hz(const app_shoot_input_t *, app_shoot_output_t *);
#endif
