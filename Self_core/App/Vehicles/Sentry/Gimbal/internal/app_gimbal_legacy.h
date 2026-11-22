/** @file app_gimbal_legacy.h
 * @brief Private legacy step; SI consumers use app_sentry_gimbal.h only.
 */
#ifndef APP_GIMBAL_LEGACY_H
#define APP_GIMBAL_LEGACY_H
#include "legacy_control_types.h"
void legacy_app_sentry_gimbal_ctrl_1khz(const legacy_gimbal_input_t *input,
                                        legacy_gimbal_output_t *output);
#endif
