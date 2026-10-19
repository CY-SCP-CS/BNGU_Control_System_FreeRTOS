/**
 * @file    app_task_init.c
 * @brief   一次性初始化编排；板型差异由板级装配负责
 * @note    入口名、周期、业务算法及原通信对象名保持；通信对象句柄仅所属模块可访问。
 */
#include "app_rtos.h"
#include "app_task_init.h"
#include "app_tasks.h"
#include "app_rtos_config.h"
#include "app_board_config.h"
#include "app_rtos_time.h"
#include "app_rtos_board.h"
#include "app_task_monitor.h"
#include "app_task_log.h"
#include <string.h>

/**
 * @brief  一次性系统初始化任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   两种板型均创建；初始化业务/PID/回调及接收端口，云台完成 3000 次零偏标定。
 * @note   发布共同起始 tick 和系统状态，保存初始化栈余量后退出；未在线的外部模块不阻塞就绪。
 */
void Start_sys_init_Task(void *argument)
{
    (void)argument;
    app_rtos_heartbeat(TASK_INIT);
    /* 先建立业务状态和回调，再打开接收端口；通信/健康可在后续 IMU 标定期间运行。 */
    app_rtos_board_init();
    bool ok = app_rtos_ports_start();
    if (ok)
    {
        app_rtos_publish_init(APP_RTOS_EVT_PORT_READY);
    }
    if (ok)
    {
        ok = app_rtos_board_calibrate();
    }
    if (ok)
    {
        /* 共同相位留 2 ticks 余量；控制首轮为起点+1，发射/VOFA 为起点+5。 */
        app_rtos_start_tick = osKernelGetTickCount() + APP_RTOS_START_MARGIN;
        ok = app_rtos_sampling_start();
    }
    if (ok)
    {
        app_rtos_publish_init(APP_RTOS_EVT_SYS_READY);
        app_rtos_log(TASK_INIT, 1, 1, CURRENT_BOARD, osKernelGetTickFreq());
    }
    else
    {
        app_rtos_publish_init(APP_RTOS_EVT_PORT_READY | APP_RTOS_EVT_SYS_INIT_FAILED);
        app_rtos_log(TASK_INIT, 3, 2, CURRENT_BOARD, 0);
    }
    /* 先缓存初始化栈水位，再标记完成并清空句柄，健康任务不查询已删除任务。 */
    app_rtos_task_stats[TASK_INIT].cycles = 1;
    app_rtos_task_complete(TASK_INIT);
    osThreadExit();
}
