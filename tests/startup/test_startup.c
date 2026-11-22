/** @file test_startup.c
 * @brief Fault-inject every module/thread creation step in the real Assembly implementation.
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "app_rtos_setup.h"
#include "app_vehicle_config.h"
#include "app_rtos.h"
#include "app_rtos_runtime_internal.h"
#include "app_task_health_internal.h"

static unsigned fail_at, step, created, terminated, destroyed;
static int next_destroy = 7;
static bool ready = true, bound;
static bool accept(void)
{
    return ++step != fail_at;
}

osKernelState_t osKernelGetState(void)
{
    return ready ? osKernelReady : osKernelRunning;
}
osThreadId_t osThreadNew(osThreadFunc_t func, void *arg, const osThreadAttr_t *attr)
{
    (void)func;
    (void)arg;
    assert(attr && attr->name && attr->stack_size);
    if (!accept())
    {
        return NULL;
    }
    return (osThreadId_t)(uintptr_t)(++created);
}
osStatus_t osThreadTerminate(osThreadId_t id)
{
    assert(id);
    ++terminated;
    return osOK;
}
bool app_rtos_startup_objects_init(void)
{
    return accept();
}
void app_rtos_startup_objects_deinit(void)
{
    assert(next_destroy == -1);
    ++destroyed;
}
void app_rtos_monitor_init(void)
{
}
void app_rtos_log(uint8_t source, uint8_t level, uint16_t code, int32_t a, int32_t b)
{
    (void)source;
    (void)level;
    (void)code;
    (void)a;
    (void)b;
}
void app_task_health_set_log_sink(app_health_log_sink_t sink)
{
    assert(sink == app_rtos_log);
    bound = true;
}

#define MODULE(name, index)                                                                        \
    bool app_task_##name##_init(void)                                                              \
    {                                                                                              \
        return accept();                                                                           \
    }                                                                                              \
    void app_task_##name##_deinit(void)                                                            \
    {                                                                                              \
        assert(next_destroy-- == index);                                                           \
        ++destroyed;                                                                               \
    }
MODULE(can, 0)
MODULE(dbus, 1)
MODULE(imu, 2)
MODULE(shoot, 3)
MODULE(referee, 4)
MODULE(health, 5)
MODULE(log, 6)
MODULE(telemetry, 7)

#define ENTRY(name)                                                                                \
    void name(void *argument)                                                                      \
    {                                                                                              \
        (void)argument;                                                                            \
    }
ENTRY(Start_ctrl_chas_Task)
ENTRY(Start_navigation_Task)
ENTRY(Start_ctrl_gim_Task)
ENTRY(Start_ctrl_shoot_Task)
ENTRY(Start_comm_can_Task)
ENTRY(Start_comm_dbus_Task)
ENTRY(Start_comm_ref_Task)
ENTRY(Start_comm_imu_Task)
ENTRY(Start_sys_init_Task)
ENTRY(Start_sys_health_Task)
ENTRY(Start_sys_log_Task)

int main(int argc, char **argv)
{
    assert(argc == 2);
    fail_at = (unsigned)strtoul(argv[1], NULL, 10);
    unsigned enabled = 0;
    for (unsigned i = 0; i < TASK_COUNT; ++i)
    {
        enabled += !!app_rtos_task_table[i].enabled;
    }
#if CURRENT_ROBOT == ROBOT_NAVIGATION && CURRENT_BOARD == BOARD_CHASSIS
    assert(app_rtos_task_table[TASK_CHAS].entry == Start_navigation_Task);
    assert(app_rtos_task_table[TASK_CHAS].enabled);
    assert(!app_rtos_task_table[TASK_GIM].enabled && !app_rtos_task_table[TASK_SHOOT].enabled);
#elif CURRENT_ROBOT == ROBOT_ARMORED || CURRENT_ROBOT == ROBOT_NAVIGATION
    assert(!app_rtos_task_table[TASK_CHAS].enabled && !app_rtos_task_table[TASK_CHAS].entry);
    assert(!app_rtos_task_table[TASK_GIM].enabled && !app_rtos_task_table[TASK_SHOOT].enabled);
#endif
    ready = false;
    assert(!app_rtos_create_tasks() && step == 0);
    ready = true;
    bool result = app_rtos_create_tasks();
    if (fail_at)
    {
        assert(!result && step == fail_at);
        assert(terminated == created && destroyed == 9);
        for (unsigned i = 0; i < TASK_COUNT; ++i)
        {
            assert(!*app_rtos_task_table[i].handle);
        }
        /* A startup failure can be retried; it is not a running-system restart. */
        fail_at = 0;
        assert(app_rtos_create_tasks());
    }
    else
    {
        assert(result && created == enabled && !destroyed && bound);
        printf("STARTUP_STEPS=%u\n", 9 + enabled);
    }
    unsigned previous = step;
    assert(app_rtos_create_tasks() && step == previous);
    return 0;
}
