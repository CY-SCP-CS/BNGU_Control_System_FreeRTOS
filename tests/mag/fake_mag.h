#ifndef TEST_FAKE_MAG_H
#define TEST_FAKE_MAG_H
#include "drv_mag.h"
#include <assert.h>
#include <string.h>
#ifdef TEST_IRQ_STATE
extern uint32_t test_primask;
#define ASSERT_BUS_UNLOCKED() assert(test_primask == 0)
#else
#define ASSERT_BUS_UNLOCKED() ((void)0)
#endif

typedef struct {
    uint8_t regs[256];
    uint32_t now, due, reads, writes, delays, resets;
    bool measuring, never_ready, ready_irq, reset_asserted, wrong_polarity;
    int fail_read, fail_write, mismatch;
    IST8310_Status_t error;
    IST8310_RawData_t raw;
} fake_mag_t;

static void fake_init(fake_mag_t *f)
{
    memset(f, 0, sizeof(*f));
    f->regs[0] = IST8310_DEVICE_ID;
    f->fail_read = f->fail_write = f->mismatch = -1;
    f->error = IST8310_ERR_IO;
    f->raw = (IST8310_RawData_t){123, -456, 789};
}

static IST8310_Status_t fake_read(void *context, uint8_t reg, uint8_t *data, uint16_t length)
{
    fake_mag_t *f = context;
    ASSERT_BUS_UNLOCKED();
    ++f->reads;
    if (f->fail_read == reg) { memset(data, 0xAA, length); return f->error; }
    if (reg == 0x02) {
        assert(length == 1);
        data[0] = f->measuring && !f->never_ready &&
                  (int32_t)(f->now - f->due) >= 0 ? 0x03 : 0x00;
    } else if (reg == 0x03) {
        assert(length == 7);
        int32_t axes[] = {f->raw.x, f->raw.y, f->raw.z};
        for (int i = 0; i < 3; ++i) {
            if (f->regs[0x0C] == 0x40 && !f->wrong_polarity) axes[i] = -axes[i];
            uint16_t value = (uint16_t)axes[i];
            data[2*i] = value & 0xFF;
            data[2*i+1] = value >> 8;
        }
        data[6] = 0x08;
        f->measuring = false;
        f->regs[0x0A] = 0;
    } else {
        memcpy(data, &f->regs[reg], length);
        if (reg == f->mismatch) data[0] ^= 1;
    }
    return IST8310_OK;
}

static IST8310_Status_t fake_write(void *context, uint8_t reg, const uint8_t *data, uint16_t length)
{
    fake_mag_t *f = context;
    ASSERT_BUS_UNLOCKED();
    ++f->writes;
    if (f->fail_write == reg) return f->error;
    memcpy(&f->regs[reg], data, length);
    if (reg == 0x0A) {
        f->measuring = data[0] == 1;
        if (f->measuring) f->due = f->now + 6;
    }
    return IST8310_OK;
}

static void fake_delay(void *context, uint32_t ms)
{
    fake_mag_t *f = context;
    f->now += ms;
    ++f->delays;
}

static IST8310_Config_t fake_bus(fake_mag_t *f)
{
    IST8310_Config_t bus = {f, fake_read, fake_write, fake_delay};
    return bus;
}
#endif
