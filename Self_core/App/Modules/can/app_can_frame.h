/**
 * @file    app_can_frame.h
 * @brief   CAN 原始帧值类型，不依赖控制业务和具体芯片
 */
#ifndef APP_CAN_FRAME_H
#define APP_CAN_FRAME_H
#include <stdint.h>

/**
 * @brief  CAN 原始帧值对象，供 CAN1 / CAN2 接收和发送队列复制
 * @note   data 持有本帧自己的载荷；有效字节数由 dlc 指定。
 *          接收帧记录接收时间和序号；发送帧继承产生它的控制结果时间和序号。
 */
typedef struct
{
    uint32_t std_id; // 标准帧 ID，范围 0x000~0x7FF。
    uint8_t bus,     // 总线编号：1=CAN1，2=CAN2。
        dlc,         // 有效载荷长度，范围 0~8 字节。
        data[8];     // 自有载荷数组，入队时复制全部结构体内容。
    uint32_t tick,   // 接收或生成该帧的系统节拍时间戳。
        sequence;    // 接收帧序号，或对应控制结果的输出序号。
} app_can_frame_t;

#endif
