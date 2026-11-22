/** @file app_chassis_legacy.h
 * @brief Private legacy step; SI consumers use app_sentry_chassis.h only.
 */
#ifndef APP_CHASSIS_LEGACY_H
#define APP_CHASSIS_LEGACY_H
#include "legacy_control_types.h"
void legacy_app_sentry_chassis_ctrl(const legacy_chassis_input_t *input,
                                    legacy_chassis_output_t *output);
#endif
