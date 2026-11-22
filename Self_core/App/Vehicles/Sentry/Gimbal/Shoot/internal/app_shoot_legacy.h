/** @file app_shoot_legacy.h
 * @brief Private legacy step; SI consumers use app_sentry_shoot.h only.
 */
#ifndef APP_SHOOT_LEGACY_H
#define APP_SHOOT_LEGACY_H
#include "legacy_control_types.h"
void legacy_app_sentry_launcher_ctrl_200hz(const legacy_shoot_input_t *input,
                                           legacy_shoot_output_t *output);
#endif
