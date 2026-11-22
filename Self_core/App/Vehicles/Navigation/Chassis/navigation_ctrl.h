/** @file navigation_ctrl.h
 * @brief Navigation robot synchronous control; task and HAL independent.
 */
#ifndef NAVIGATION_CTRL_H
#define NAVIGATION_CTRL_H
#include "App/Vehicles/Navigation/Contracts/app_navigation_data.h"
#include "app_dbus_data.h"
#include "app_imu_data.h"
/** @brief Reset the navigation state and four wheel PID controllers.
 * @return No value. Configuration is static and cannot fail.
 * @note Initialization task only, before control starts. */
void navigation_ctrl_init(void);
/** @brief Execute one 5 ms navigation control step.
 * @param[in] input Feedback and latest CAN command in SI; borrowed for this call.
 * @param[in] dbus Latest complete normalized DBUS input; borrowed for this call.
 * @param[in] imu Body attitude in rad, timestamp in kernel ticks; borrowed for this call.
 * @param[in] now_tick Current 1 kHz kernel time, used for world-frame IMU freshness.
 * @param[out] output Four current raw values in CAN ID 1,2,3,4 order and commanded speed.
 * @return No value. Invalid/missing DBUS or stale world-frame IMU produces stop.
 * @note Chassis task owns all state. Game-controller mode remains unimplemented and stops.
 *       DBUS/CAN/motor freshness is checked at entry using app_input_config.h.
 */
void navigation_ctrl_step(const app_navigation_input_t *input, const app_dbus_snapshot_t *dbus,
                          const app_imu_snapshot_t *imu, uint32_t now_tick,
                          app_chassis_output_t *output);
#endif
