//
// Created by 33140 on 2026/11/7.
//

#ifndef BNGU_CONTROL_SYSTEM_DRV_GAME_CONTROLLER_H
#define BNGU_CONTROL_SYSTEM_DRV_GAME_CONTROLLER_H

#include <stdint.h>

typedef struct
{
    float lx;
    float ly;
    float rx;
    float ry;

    float lt;
    uint8_t lb;
    float rt;
    uint8_t rb;

    uint8_t a;
    uint8_t b;
    uint8_t x;
    uint8_t y;

    uint8_t up;
    uint8_t down;
    uint8_t left;
    uint8_t right;

    uint8_t view;  // 视图键，两个方块
    uint8_t menu;  // 菜单键，三条横线
    uint8_t share; // 分享键，上传图标
    uint8_t xbox;
} drv_game_controller_buttons_t;

/**
 * @brief 第一版：解析 JOY,lx,ly,rx,ry 文本帧，后续协议与接口还会修改。
 * @param buffer 完整帧，以 '\0' 结束；四轴范围 [-1, 1]。
 * @param data 成功时仅更新 lx、ly、rx、ry，其余字段不变。
 * @return 1 成功；0 失败，原数据不变。
 */
uint8_t drv_game_controller_decode(const uint8_t *buffer, drv_game_controller_buttons_t *data);

#endif
