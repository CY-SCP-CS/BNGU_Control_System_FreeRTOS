//
// Created by 33140 on 2026/10/31.
//

#ifndef BNGU_CONTROL_SYSTEM_APP_ARMORED_JOINT_H
#define BNGU_CONTROL_SYSTEM_APP_ARMORED_JOINT_H

#include "drv_motor.h"
#include "lib_pid.h"

typedef struct
{
    drv_motor_dm_t dm_motor;
    lib_pid_t Height_ctrl;
    lib_pid_t pitch_ctrl;
    lib_pid_t roll_ctrl;
} dm_joint_motor;

/** @brief 配置四个关节实例并设置待使能请求。
 * @return 无返回值。配置失败保持原模块未就绪，不发送 CAN。
 * @note 装配先绑定输出回调；实验模块不自动接入任何任务。 */
void joint_motor_init(void);
/** @brief 设置关节使能请求。
 * @return 无返回值。实际提交由 process 完成，当前函数不发送。
 * @note 仅模块所有者任务调用；禁止在中断中并发修改状态。 */
void joint_motor_enable_request(void);
/** @brief 按 Master_id 分发关节反馈。
 * @param[in] id 已校验标准 CAN 反馈 ID。
 * @param[in] bytes 帧数据，仅调用期间借用；NULL 时忽略。
 * @param[in] length 有效长度，B，必须为 8，否则忽略。
 * @return 无返回值。未知 ID 或无效长度不更新实例。
 * @note 仅关节所有者任务调用；不在 CAN 中断中解析。 */
void app_armored_joint_receive(uint32_t id, const uint8_t *bytes, uint8_t length);
/** @brief 推进使能或交替提交两个关节控制帧。
 * @param[in] count 偶数为 FL/RL，奇数为 FR/RR。
 * @param[in] fl 前左 8B 控制帧，仅调用期间借用。
 * @param[in] rl 后左 8B 控制帧，仅调用期间借用。
 * @param[in] fr 前右 8B 控制帧，仅调用期间借用。
 * @param[in] rr 后右 8B 控制帧，仅调用期间借用。
 * @return 0=两帧已接受；1=空指针或提交失败；2=尚未全部使能，跳过控制。
 * @note 仅所有者任务，非阻塞；使能过程不读取控制帧；成功不代表动作完成。 */
uint8_t app_armored_joint_process(uint8_t count, uint8_t fl[8], uint8_t rl[8], uint8_t fr[8],
                                  uint8_t rr[8]);
#endif // BNGU_CONTROL_SYSTEM_APP_ARMORED_JOINT_H
