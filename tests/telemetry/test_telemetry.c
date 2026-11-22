/**
 * @file    test_telemetry.c
 * @brief   用真实遥测、快照和 VOFA 打包实现检查协议映射与非阻塞失败策略。
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "app_task_telemetry_internal.h"
#include "app_rtos.h"
#include "app_vehicle_config.h"
#include "drv_vofa.h"

static unsigned s_drops;
static bool s_created;
static bool s_locked;
static bool s_busy;
static int s_mutex;

/**
 * @brief  创建只供本测试使用的互斥量替身并核对继承属性。
 * @param[in] attr 待验证的互斥量属性，立即读取，不保留。
 * @return 唯一的测试互斥量句柄。
 * @note   主机单线程测试，不模拟任务调度。
 */
osMutexId_t osMutexNew(const osMutexAttr_t *attr)
{
    assert(!s_created);
    assert(attr && attr->attr_bits == osMutexPrioInherit);
    assert(strcmp(attr->name, "ctrl_state_mutex") == 0);
    s_created = true;
    return &s_mutex;
}

/**
 * @brief  验证所有遥测读写都采用非阻塞取锁。
 * @param[in] mutex 被测模块传入的互斥量。
 * @param[in] timeout 必须为 0 tick。
 * @return osOK=取得；osErrorResource=本轮注入锁忙。
 * @note   不等待，通过 s_busy 注入可重复的竞争失败。
 */
osStatus_t osMutexAcquire(osMutexId_t mutex, uint32_t timeout)
{
    assert(mutex == &s_mutex && s_created && !s_locked);
    assert(timeout == 0);
    if (s_busy)
    {
        return osErrorResource;
    }
    s_locked = true;
    return osOK;
}

/**
 * @brief  确认被测实现只释放已经取得的互斥量。
 * @param[in] mutex 当前持有的测试互斥量。
 * @return osOK=释放完成。
 * @note   单线程测试，错误释放通过断言失败报告。
 */
osStatus_t osMutexRelease(osMutexId_t mutex)
{
    assert(mutex == &s_mutex && s_locked);
    s_locked = false;
    return osOK;
}

/**
 * @brief  确认回滚时不存在未释放的锁。
 * @param[in] mutex 被删除的测试互斥量。
 * @return osOK=删除完成。
 * @note   仅测试结束调用，不模拟运行中删除。
 */
osStatus_t osMutexDelete(osMutexId_t mutex)
{
    assert(mutex == &s_mutex && s_created && !s_locked);
    s_created = false;
    return osOK;
}

/**
 * @brief  拒绝本测试不应创建的消息队列。
 * @param[in] count 请求深度。
 * @param[in] size 请求消息大小，B。
 * @param[in] attr 请求属性。
 * @return NULL；调用本身会触发断言失败。
 * @note   链接真实对象辅助函数需要此符号，遥测不得使用它。
 */
osMessageQueueId_t osMessageQueueNew(uint32_t count, uint32_t size,
                                     const osMessageQueueAttr_t *attr)
{
    (void)count;
    (void)size;
    (void)attr;
    assert(0);
    return NULL;
}

/**
 * @brief  拒绝本测试不应删除的消息队列。
 * @param[in] queue 请求删除的队列。
 * @return osErrorResource；调用本身会触发断言失败。
 * @note   仅满足真实对象辅助函数的链接依赖。
 */
osStatus_t osMessageQueueDelete(osMessageQueueId_t queue)
{
    (void)queue;
    assert(0);
    return osErrorResource;
}

/**
 * @brief  记录遥测发布失败次数。
 * @param[in] index 必须为 COUNT_TELEMETRY_DROP。
 * @return 无返回值；错误计数类型通过断言报告。
 * @note   单线程计数，不模拟临界区。
 */
void app_rtos_count(unsigned index)
{
    assert(index == COUNT_TELEMETRY_DROP);
    s_drops++;
}

/**
 * @brief  比较九通道并核对实际打包的 40 B 线上帧。
 * @param[in] actual 被测映射输出的九个 float。
 * @param[in] expected 预先确定的九通道参考值，索引对应既有协议。
 * @return 无返回值；数值或协议变化通过断言报告。
 * @note   参考帧独立按 float[9] 和固定帧尾构造，不调用被测打包器。
 */
static void check_frame(const float actual[9], const float expected[9])
{
    uint8_t actual_frame[DRV_VOFA_FRAME_SIZE];
    uint8_t expected_frame[40];
    const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    assert(sizeof(actual_frame) == sizeof(expected_frame));
    for (unsigned i = 0; i < 9; i++)
    {
        assert(fabsf(actual[i] - expected[i]) < 0.0001f);
    }
    memcpy(expected_frame, expected, 9 * sizeof(float));
    memcpy(expected_frame + 9 * sizeof(float), tail, sizeof(tail));
    assert(drv_vofa_pack(actual, actual_frame) == DRV_VOFA_OK);
    assert(memcmp(actual_frame, expected_frame, sizeof(actual_frame)) == 0);
}

/**
 * @brief  验证板型映射、锁忙保留、副本隔离和发布失败策略。
 * @return 0=全部通过；断言失败终止进程。
 * @note   使用真实模块源文件；不验证硬件发送时序。
 */
int main(void)
{
    float channels[9];
    float previous[9];
    for (unsigned i = 0; i < 9; i++)
    {
        channels[i] = 123.0f;
    }
    memcpy(previous, channels, sizeof(previous));
    assert(!app_rtos_vofa_read(channels));
    assert(memcmp(channels, previous, sizeof(channels)) == 0);
    assert(app_task_telemetry_init());
    assert(!app_rtos_vofa_read(NULL));

    app_shoot_output_t shoot = {.disc_nominal = 11.0f,
                                .disc_speed = -12.0f,
                                .disc_integral = 13.0f,
                                .disc_effort_raw = -14.0f,
                                .friction_nominal = 15.0f,
                                .friction_r_speed = -16.0f,
                                .friction_l_speed = 17.0f,
                                .disc_encoder_raw = 8191.0f};
    app_chassis_output_t chassis = {
        .target = {.vx_speed = 1.25f, .vy_speed = -0.5f, .vw_speed = 2.0f},
        .state = {.cur_speed = {.vx_speed = -0.25f, .vy_speed = 0.125f, .vw_speed = -3.0f},
                  .cur_power = 75.0f,
                  .power_scale = 0.5f}};
    app_gimbal_output_t gimbal = {0};
    app_rtos_shoot_publish(&shoot, 101);
    app_rtos_chassis_publish(&chassis, 102);
    app_rtos_gimbal_publish(&gimbal, 0.25f, 103);
    assert(app_rtos_vofa_read(channels));
#if APP_BOARD_HAS_CHASSIS
    const float expected[9] = {1250.0f, -250.0f, -500.0f, 125.0f, 2.0f, -3.0f, 75.0f, 0.5f, 0.0f};
#else
    const float expected[9] = {11.0f, -12.0f, 13.0f, -14.0f, -15.0f, -16.0f, 15.0f, 17.0f, 8191.0f};
#endif
    check_frame(channels, expected);
    app_ctrl_state_t snapshot;
    assert(app_rtos_ctrl_read(&snapshot));
    assert(snapshot.shoot_tick == 101 && snapshot.chassis_tick == 102 &&
           snapshot.gimbal_tick == 103);
    assert(snapshot.imu_roll_rad == 0.25f);
    memcpy(previous, channels, sizeof(previous));
    s_busy = true;
    assert(!app_rtos_vofa_read(channels));
    assert(memcmp(previous, channels, sizeof(channels)) == 0);
    app_ctrl_state_t prior = snapshot;
    assert(!app_rtos_ctrl_read(&snapshot));
    assert(memcmp(&prior, &snapshot, sizeof(snapshot)) == 0);
    shoot.disc_speed = 99.0f;
    app_rtos_shoot_publish(&shoot, 200);
    app_rtos_chassis_publish(&chassis, 200);
    app_rtos_gimbal_publish(&gimbal, 1.0f, 200);
    assert(s_drops == 3);
    s_busy = false;
    assert(app_rtos_vofa_read(channels));
    check_frame(channels, expected);
    /* A caller's previous result is independent of the next reader's output. */
    channels[0] = 999.0f;
    assert(app_rtos_vofa_read(previous));
    assert(channels[0] == 999.0f);
    check_frame(previous, expected);
    app_task_telemetry_deinit();
    assert(!s_created && !app_rtos_vofa_read(previous));
    puts("PASS: nine-channel mapping/frame, nonblocking failure, private copies, telemetry drops");
    return 0;
}
