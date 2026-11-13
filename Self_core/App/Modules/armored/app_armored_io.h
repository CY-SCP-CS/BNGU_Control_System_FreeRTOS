/** @file app_armored_io.h
 * @brief Experimental armored actuator output injection, without HAL or scheduler ownership.
 */
#ifndef APP_ARMORED_IO_H
#define APP_ARMORED_IO_H
#include <stdint.h>
/** @brief Accept a copied eight-byte CAN2 frame; return 0 accepted, nonzero rejected.
 * @param[in] id Standard CAN ID.
 * @param[in] data Borrowed eight-byte payload, copy before returning.
 * @param[in,out] context Borrowed context for the configured lifetime.
 * @return 0=accepted; nonzero=busy/error. Success is not motor acknowledgement.
 * @note Task context only; callback must not block. */
typedef uint8_t (*app_armored_submit_t)(uint32_t id, const uint8_t data[8], void *context);
/** @brief Configure the experimental output. NULL disables it; call before any joint operation.
 * @param[in] submit Callback borrowed until rebound.
 * @param[in,out] context Callback context borrowed until rebound.
 * @return No value. Does not enable motors. */
void app_armored_bind(app_armored_submit_t submit, void *context);
/** @brief 向注入端口提交一份 CAN2 帧。
 * @param[in] id 标准 CAN ID，由调用者保证范围 0..0x7FF。
 * @param[in] data 非 NULL，8B，仅本次调用期间借用，回调接受前须复制。
 * @return 0=已接受；非零=未绑定、忙或端口错误。成功不代表电机收到。
 * @note 仅所属任务调用，非阻塞；本接口不加入 CAN 队列，由装配回调负责交付。 */
uint8_t app_armored_can_submit(uint32_t id, const uint8_t data[8]);
#endif
