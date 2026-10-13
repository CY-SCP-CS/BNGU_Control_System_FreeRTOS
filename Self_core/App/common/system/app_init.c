#include "app_init.h"
#include "app_sentry_gimbal.h"
#include "app_sentry_chassis.h"
#include "app_sentry_shoot.h"
void app_init(void) {
#if CURRENT_BOARD==BOARD_GIMBAL
    app_sentry_gimbal_init(); app_sentry_shoot_init();
#else
    app_sentry_chassis_init();
#endif
}
