/** @file test_port.c
 * @brief Async IST8310 port regression with explicitly delivered HAL completion/error events.
 */
#include "drv_mag_port.h"
#include "bsp_gpio.h"
#include "bsp_i2c.h"
#include "fake_mag.h"
#include <stdio.h>
I2C_HandleTypeDef hi2c3 = {I2C3};
GPIO_TypeDef test_gpio_g;
uint32_t test_primask;
static fake_mag_t sensor;
static HAL_StatusTypeDef start_status;
static uint8_t io_kind;
static uint16_t io_reg, io_length;
static uint8_t *io_bytes;
static bool hold_completion;
static unsigned aborts;
uint32_t bsp_tim_get_tick_ms(void) { return sensor.now; }

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *port, uint16_t addr, uint16_t reg,
                                  uint16_t size, uint8_t *bytes, uint16_t n, uint32_t timeout)
{
    (void)port; (void)addr; (void)reg; (void)size; (void)bytes; (void)n; (void)timeout;
    assert(!"Acquisition must never use blocking I2C");
    return HAL_ERROR;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *port, uint16_t addr, uint16_t reg,
                                   uint16_t size, uint8_t *bytes, uint16_t n, uint32_t timeout)
{
    return HAL_I2C_Mem_Read(port, addr, reg, size, bytes, n, timeout);
}
static HAL_StatusTypeDef start(I2C_HandleTypeDef *port, uint16_t address, uint16_t reg,
                               uint16_t size, uint8_t *data, uint16_t length, uint8_t kind)
{
    assert(test_primask == 0 && port == &hi2c3 && address == 0x1c && size == 1);
    assert(data && length && io_kind == 0);
    if (start_status != HAL_OK) { return start_status; }
    io_kind = kind; io_reg = reg; io_bytes = data; io_length = length;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *p, uint16_t a, uint16_t r,
                                     uint16_t s, uint8_t *b, uint16_t n)
{ return start(p, a, r, s, b, n, 1); }
HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *p, uint16_t a, uint16_t r,
                                      uint16_t s, uint8_t *b, uint16_t n)
{ return start(p, a, r, s, b, n, 2); }
HAL_StatusTypeDef HAL_I2C_Master_Abort_IT(I2C_HandleTypeDef *port, uint16_t address)
{
    assert(port == &hi2c3 && address == 0x1c);
    io_kind = 0; ++aborts;
    HAL_I2C_AbortCpltCallback(port);
    return HAL_OK;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    assert(port == GPIOG && pin == GPIO_PIN_6);
    sensor.reset_asserted = state == GPIO_PIN_RESET;
    if (sensor.reset_asserted) { ++sensor.resets; sensor.measuring = false; }
}
void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin) { (void)port; (void)pin; }
void HAL_Delay(uint32_t ms) { (void)ms; assert(!"No blocking delay in acquisition"); }
static drv_mag_snapshot_t snapshot(void)
{
    drv_mag_snapshot_t value;
    assert(drv_mag_port_read_snapshot(&value));
    return value;
}
static void completion(void)
{
    if (!io_kind || hold_completion) { return; }
    uint8_t kind = io_kind; io_kind = 0;
    IST8310_Status_t result = kind == 1 ? fake_read(&sensor, io_reg, io_bytes, io_length)
                                       : fake_write(&sensor, io_reg, io_bytes, io_length);
    if (result != IST8310_OK) { HAL_I2C_ErrorCallback(&hi2c3); }
    else if (kind == 1) { HAL_I2C_MemRxCpltCallback(&hi2c3); }
    else { HAL_I2C_MemTxCpltCallback(&hi2c3); }
}
static void step(void)
{
    ++sensor.now;
    drv_mag_port_process();
    completion();
}
static void wait_sample(uint32_t expected)
{
    for (unsigned i = 0; i < 1000 && snapshot().sample_count < expected; ++i) { step(); }
    assert(snapshot().sample_count == expected);
}
static void boot(uint32_t epoch)
{
    fake_init(&sensor); sensor.now = epoch;
    start_status = HAL_OK; io_kind = 0; hold_completion = false;
    assert(drv_mag_port_init() == IST8310_OK);
    assert(sensor.reset_asserted && !sensor.reads && !sensor.writes);
    for (unsigned i = 0; i < 49; ++i) { step(); }
    assert(sensor.reset_asserted);
    step(); assert(!sensor.reset_asserted);
    for (unsigned i = 0; i < 50; ++i) { step(); }
    assert(!sensor.reads); /* configuration is asynchronous and follows settling */
    wait_sample(1);
}
int main(void)
{
    drv_mag_snapshot_t value;
    assert(!drv_mag_port_read_snapshot(&value));
    assert(!drv_mag_port_read_snapshot(NULL));
    boot(0);
    value = snapshot();
    assert(value.valid && value.online && value.device_id == 0x10);
    assert(value.sample.raw.x == 123 && value.sample.raw.y == -456 && value.sample.raw.z == 789);
    unsigned reads = sensor.reads, writes = sensor.writes;
    HAL_GPIO_EXTI_Callback(GPIO_PIN_3);
    assert(reads == sensor.reads && writes == sensor.writes); /* IRQ performs no I2C */
    test_primask = 1; assert(drv_mag_port_read_snapshot(&value) && test_primask == 1); test_primask = 0;
    sensor.fail_read = 3;
    for (unsigned i = 0; i < 100 && snapshot().state != DRV_MAG_RECOVERING; ++i) { step(); }
    value = snapshot();
    assert(value.state == DRV_MAG_RECOVERING && value.valid && !value.online && value.error_count == 1);
    assert(value.sample_count == 1 && value.sample.raw.x == 123);
    sensor.fail_read = -1;
    wait_sample(2);
    assert(snapshot().recovery_count == 1);
    hold_completion = true;
    for (unsigned i = 0; i < 30 && !aborts; ++i) { step(); }
    assert(aborts == 1 && snapshot().last_error == IST8310_ERR_TIMEOUT);
    hold_completion = false;
    wait_sample(3);
    assert(sensor.delays == 0);
    sensor.now += 100;
    assert(!snapshot().online);
    boot(UINT32_MAX - 70U);
    assert(snapshot().valid && snapshot().online && sensor.now < 300U);
    bsp_i2c_transfer_t invalid = {0};
    assert(bsp_i2c_transfer(NULL) == HAL_ERROR && bsp_i2c_transfer(&invalid) == HAL_ERROR);
    assert(bsp_i2c_transfer_async(&invalid) == HAL_ERROR);
    puts("PASS async magnetometer: settling, completion ownership, IRQ bounds, decode, transfer errors, abort/recovery, stale data, tick wrap");
    return 0;
}
