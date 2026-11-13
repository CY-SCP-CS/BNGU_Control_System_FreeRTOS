/* Compile this test and the real driver WITHOUT any HAL include directory. */
#include "fake_mag.h"
#include <math.h>
#include <stdio.h>

int main(void)
{
    fake_mag_t f;
    fake_init(&f);
    IST8310_Config_t bus = fake_bus(&f);
    IST8310_Handle_t dev = {0};
    IST8310_Data_t data, before;
    memset(&data, 0x55, sizeof(data));
    before = data;
    assert(IST8310_Init(NULL, &bus) == IST8310_ERR_ARGUMENT);
    assert(IST8310_Init(&dev, NULL) == IST8310_ERR_ARGUMENT);
    assert(IST8310_Read(&dev, &data) == IST8310_ERR_NOT_INITIALIZED);
    f.regs[0] = 0x42;
    assert(IST8310_Init(&dev, &bus) == IST8310_ERR_DEVICE_ID && !dev.initialized);
    assert(f.writes == 0);
    f.regs[0] = 0x10;
    f.fail_read = 0;
    f.error = IST8310_ERR_TIMEOUT;
    assert(IST8310_Init(&dev, &bus) == IST8310_ERR_TIMEOUT && !dev.initialized);
    f.fail_read = -1;
    f.fail_write = 0x41;
    assert(IST8310_Init(&dev, &bus) == IST8310_ERR_TIMEOUT && !dev.initialized);
    f.fail_write = -1;
    f.mismatch = 0x42;
    assert(IST8310_Init(&dev, &bus) == IST8310_ERR_VERIFY && !dev.initialized);
    f.mismatch = -1;
    assert(IST8310_Init(&dev, &bus) == IST8310_OK);
    assert(dev.device_id == 0x10 && dev.initialized);
    assert(f.regs[0x0B] == 0x08 && f.regs[0x41] == 0x24 && f.regs[0x42] == 0xC0);
    assert(IST8310_Read(&dev, &data) == IST8310_NOT_READY);
    assert(memcmp(&before, &data, sizeof(data)) == 0);
    assert(IST8310_StartMeasurement(&dev) == IST8310_OK);
    f.now += 6;
    f.fail_read = 0x03;
    assert(IST8310_Read(&dev, &data) == IST8310_ERR_TIMEOUT);
    assert(memcmp(&before, &data, sizeof(data)) == 0);
    f.fail_read = -1;
    f.raw = (IST8310_RawData_t){-32768, 32767, -1};
    assert(IST8310_Read(&dev, &data) == IST8310_OK);
    assert(data.raw.x == -32768 && data.raw.y == 32767 && data.raw.z == -1);
    assert(fabsf(data.x_uT + 9830.4f) < 0.01f);
    assert(data.overrun && data.field_interrupt && data.stat1 == 3 && data.stat2 == 8);
    assert(fabsf(data.norm_uT - sqrtf(data.x_uT*data.x_uT + data.y_uT*data.y_uT + data.z_uT*data.z_uT)) < 0.01f);
    assert(IST8310_Read(&dev, &data) == IST8310_NOT_READY);

    IST8310_SelfTestResult_t result;
    f.raw = (IST8310_RawData_t){123, -456, 789};
    assert(IST8310_SelfTest(&dev, &result) == IST8310_OK);
    assert(result.x_passed && result.y_passed && result.z_passed);
    assert(f.regs[0x40] == 0 && f.regs[0x0C] == 0 && !f.measuring);
    f.wrong_polarity = true;
    assert(IST8310_SelfTest(&dev, &result) == IST8310_ERR_SELF_TEST);
    f.wrong_polarity = false;
    f.raw = (IST8310_RawData_t){0};
    assert(IST8310_SelfTest(&dev, &result) == IST8310_SELF_TEST_INCONCLUSIVE);
    f.never_ready = true;
    assert(IST8310_SelfTest(&dev, &result) == IST8310_ERR_TIMEOUT);
    assert(f.regs[0x40] == 0 && f.regs[0x0C] == 0);
    f.never_ready = false;
    f.fail_read = 0x03;
    assert(IST8310_SelfTest(&dev, &result) == IST8310_ERR_TIMEOUT);
    assert(f.regs[0x40] == 0 && f.regs[0x0C] == 0);
    f.fail_read = -1;
    f.fail_write = 0x40;
    assert(IST8310_SelfTest(&dev, &result) == IST8310_ERR_TIMEOUT && !dev.initialized);
    f.fail_write = -1;
    assert(IST8310_Init(&dev, &dev.config) == IST8310_OK);
    assert(IST8310_SoftReset(&dev) == IST8310_OK && !dev.initialized && f.regs[0x0B] == 1);
    assert(IST8310_StartMeasurement(&dev) == IST8310_ERR_NOT_INITIALIZED);
    puts("PASS magnetometer core: ID, config verify, errors, signed decode, status, self-test cleanup, soft reset; no HAL");
    return 0;
}
