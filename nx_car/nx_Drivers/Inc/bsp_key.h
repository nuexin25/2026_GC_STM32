/**
 ******************************************************************************
 * @file    bsp_key.h
 * @brief   按键扫描驱动：消抖、短按、长按检测
 * @author  nx_car
 * @date    2026-09-05
 ******************************************************************************
 * @note   硬件：KEY1~KEY5 = PE7~PE11，上拉输入，按下为低电平（GPIO_PIN_RESET）。
 *         扫描节拍 10ms，消抖需连续 2 次同电平确认。
 ******************************************************************************
 */
#ifndef __BSP_KEY_H
#define __BSP_KEY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* ===================== 按键 ID 枚举 ===================== */
typedef enum {
    KEY_NONE = 0,   /* 无按键 */
    KEY1 = 1,       /* PE7 */
    KEY2 = 2,       /* PE8 */
    KEY3 = 3,       /* PE9 */
    KEY4 = 4,       /* PE10 */
    KEY5 = 5        /* PE11 */
} KeyID_t;

/* ===================== 可调参数 ===================== */
#define KEY_SCAN_PERIOD_MS      2U      /* 按键扫描周期，单位 ms，可调范围 2~50（越小响应越快） */
#define KEY_DEBOUNCE_COUNT      2U      /* 消抖确认次数，可调范围 1~5（连续 N 次同电平） */
#define KEY_LONG_PRESS_MS       1000U   /* 长按判定阈值，单位 ms，可调范围 300~5000 */

/* 按键有效电平（上拉输入，按下为低） */
#define KEY_PRESSED_LEVEL       GPIO_PIN_RESET
#define KEY_RELEASED_LEVEL      GPIO_PIN_SET

/* ===================== 函数声明 ===================== */

/**
 * @brief  按键初始化
 * @param  无
 * @retval 无
 * @note   GPIO 已由 CubeMX 配置为上拉输入，本函数可留空，仅保留接口供后续扩展。
 */
void Key_Init(void);

/**
 * @brief  按键扫描（主循环周期调用，内部按 KEY_SCAN_PERIOD_MS 节流）
 * @param  无
 * @retval 短按事件的按键 ID（KEY_NONE 表示无短按事件）
 * @note   - 消抖：连续 KEY_DEBOUNCE_COUNT 次同电平才确认状态变化；
 *         - 短按：按键释放时若按住时长 < KEY_LONG_PRESS_MS 则触发短按；
 *         - 长按：按住 >= KEY_LONG_PRESS_MS 时通过 Key_GetLongPress() 获取；
 *         - 同一时刻仅响应一个按键（优先级 KEY1 > KEY2 > ... > KEY5）。
 */
KeyID_t Key_Scan(void);

/**
 * @brief  获取长按事件
 * @param  无
 * @retval 触发长按的按键 ID（KEY_NONE 表示无长按事件）
 * @note   调用后清除长按标志，供 P9 一键启动复用。
 */
KeyID_t Key_GetLongPress(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_KEY_H */
