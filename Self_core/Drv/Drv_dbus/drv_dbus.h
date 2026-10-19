/**
 * @file    drv_dbus.h
 * @brief   DBUS 遥控器数据解码 (纯逻辑层)
 * @note    不依赖 BSP / HAL, 输入 18 字节原始数据, 输出解码后的遥控数据
 */
#ifndef DRV_DBUS_H
#define DRV_DBUS_H

#include "lib_typedef.h"

#define DRV_DBUS_BUFFER_SIZE 18
#define DRV_DBUS_CHANNEL_CENTER 1024
#define DRV_DBUS_CHANNEL_RANGE 660
#define DRV_DBUS_SWITCH_UP 1U
#define DRV_DBUS_SWITCH_DOWN 2U
#define DRV_DBUS_SWITCH_MIDDLE 3U

/**
 * @brief  DBUS 解码数据结构
 */
typedef struct
{
    struct
    {
        uint16_t ch[4];         /* 摇杆通道 (0~1684, 中点≈1024) */
        uint8_t s1;             /* 拨轮开关 S1 (1=上, 2=下, 3=中) */
        uint8_t s2;             /* 拨轮开关 S2 */
        uint16_t rolling_wheel; /* 滚轮 */
    } rc;

    struct
    {
        int16_t x;     /* 鼠标 X 位移 (像素) */
        int16_t y;     /* 鼠标 Y 位移 */
        int16_t z;     /* 鼠标滚轮 */
        uint8_t left;  /* 鼠标左键 (0=松开, 1=按下) */
        uint8_t right; /* 鼠标右键 */
    } mouse;

    struct
    {
        uint8_t w;     /* W 键 */
        uint8_t s;     /* S 键 */
        uint8_t a;     /* A 键 */
        uint8_t d;     /* D 键 */
        uint8_t q;     /* Q 键 */
        uint8_t e;     /* E 键 */
        uint8_t shift; /* Shift 键 */
        uint8_t ctrl;  /* Ctrl 键 */
    } keyboard;
} drv_dbus_data_t;

/**
 * @brief  解码 DBUS 18 字节原始数据
 * @param  buffer  串口 DMA 接收缓冲区 (18 bytes)
 * @param  data    解码结果输出
 */
void drv_dbus_decode(const uint8_t buffer[DRV_DBUS_BUFFER_SIZE], drv_dbus_data_t *data);

// ─── Port: BSP 适配 ─────────────────────────────

/**
 * @brief  DBUS 完整帧交付回调类型。
 * @note   在接收中断中调用；bytes 指向临时使用的 DMA 缓冲，接收者必须立即复制 18 字节。
 * @note   bytes=NULL 表示 UART 接收错误，不能对其解引用；这里不交付不足 18 字节的帧。
 */
typedef void (*drv_dbus_frame_callback_t)(const uint8_t bytes[18]);
/* Callback runs in IRQ context: bytes=NULL signals a receive error. */
/**
 * @brief  绑定 DBUS 完整帧与接收错误的上交入口。
 * @param  callback     完整帧交付函数，NULL 表示不交付。
 * @note   在启动接收前由 RTOS 端口层绑定；当前回调复制到 dbus_rx_queue 并设置任务标志。
 */
void drv_dbus_port_bind(drv_dbus_frame_callback_t callback);
/**
 * @brief  初始化 DBUS (UART DMA + IDLE 中断)
 * @note   挂接 USART3 + DMA, 硬编码硬件映射
 * @note   复位接收双缓冲，注册 USART3 事件并启动第一轮 18 字节 DMA。
 * @return 1=DMA 启动成功，0=启动失败。
 * @note   初始化任务调用；USART3 的波特率、DMA 和中断配置由 CubeMX 外设初始化先完成。
 */
uint8_t drv_dbus_port_init(void);
/**
 * @brief  在接收停止后尝试重新启动 USART3 DMA。
 * @return 1=接收已运行或重启成功，0=本次重启失败。
 * @note   由 DBUS 任务定期调用；短临界区防止与接收中断同时改变 DMA 状态，本函数不解包。
 */
uint8_t drv_dbus_port_poll(void);

#endif
