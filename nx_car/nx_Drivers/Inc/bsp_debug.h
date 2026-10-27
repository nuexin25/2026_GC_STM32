/**
 ******************************************************************************
 * @file    bsp_debug.h
 * @brief   调试系统驱动头文件：printf 重定向、统一日志、LED 流水、PWM 蜂鸣器、板载 BZ_LED
 * @author  nx_car
 * @date    2026-09-05
 ******************************************************************************
 * @note    蜂鸣器 WHT101 为无源器件（谐振 2.731kHz），由 PA7=TIM3_CH2 输出 PWM 方波驱动；
 *          电平直驱（直流）不发声。本驱动采用非阻塞状态机，禁止 HAL_Delay。
 ******************************************************************************
 */
#ifndef __BSP_DEBUG_H
#define __BSP_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdint.h>

/* ===================== 外接 LED 极性配置（PA0~PA2 流水灯） ===================== */
#define LED_ACTIVE_LEVEL      GPIO_PIN_SET     /* 外接 LED 点亮电平，SET=高电平点亮 */
#define LED_INACTIVE_LEVEL    GPIO_PIN_RESET   /* 外接 LED 熄灭电平 */

/* ===================== 蜂鸣器 PWM 配置（PA7 = TIM3_CH2） ===================== */
/* PWM 频率由 CubeMX 生成的 TIM3 配置（tim.c 的 PSC/ARR）决定，软件侧只调占空比。 */
#define BEEP_DUTY_PERCENT     50U     /* 蜂鸣器 PWM 占空比，单位 %，默认 50，可调范围 10~90 */
#define BEEP_TIM              htim3   /* 蜂鸣器所用定时器句柄（TIM3） */
#define BEEP_TIM_CHANNEL      TIM_CHANNEL_2  /* 蜂鸣器 PWM 通道（TIM3_CH2 = PA7） */

/* ===================== 板载 BZ_LED 配置（PE12/PE13/PE14，低电平点亮） ===================== */
#define BZ_LED_ACTIVE_LEVEL   GPIO_PIN_RESET  /* 板载 LED 点亮电平：低电平点亮 */
#define BZ_LED_INACTIVE_LEVEL GPIO_PIN_SET    /* 板载 LED 熄灭电平：高电平熄灭 */

/* 板载 LED 编号（PE12/PE13/PE14） */
typedef enum {
    BZ_LED1 = 0,   /* 心跳指示：每 HEARTBEAT_PERIOD_MS 翻转，与 [SYS] alive 同步 */
    BZ_LED2 = 1,   /* 蜂鸣同步：鸣叫期间点亮、静默熄灭 */
    BZ_LED3 = 2,   /* 预留：BSP_LED_Set 可控，供后续模块指示状态 */
    BZ_LED_MAX
} BZ_LED_ID_t;

/* ===================== 周期/时长参数 ===================== */
#define LED_FLOW_PERIOD_MS    500U    /* 外接 LED 流水切换周期，单位 ms，可调范围 50~5000 */
#define HEARTBEAT_PERIOD_MS   1000U   /* 心跳日志输出周期，单位 ms，可调范围 100~10000 */
#define BEEP_ON_MS           500U    /* 上电单次蜂鸣时长，单位 ms，可调范围 10~1000 */

/* 外接 LED 流水索引（对应 PA0=G, PA1=B, PA2=Y） */
typedef enum {
    LED_FLOW_G = 0,   /* 绿色 LED (PA0) */
    LED_FLOW_B = 1,   /* 蓝色 LED (PA1) */
    LED_FLOW_Y = 2,   /* 黄色 LED (PA2) */
    LED_FLOW_MAX
} LED_FlowIndex_t;

/* ===================== 调试宏 ===================== */
/* DEBUG_BEEP_TEST: 1=上电启动持续断续鸣叫验证，0=正常模式（默认） */
#ifndef DEBUG_BEEP_TEST
#define DEBUG_BEEP_TEST       0
#endif

/* ===================== 函数声明 ===================== */

/**
 * @brief  调试系统初始化：熄灭全部 LED、上电蜂鸣一次、记录起始节拍
 * @param  无
 * @retval 无
 * @note   须在主循环前调用一次；BSP_Beep_Update() 须在主循环周期调用
 */
void BSP_Debug_Init(void);

/**
 * @brief  统一日志输出接口
 * @param  tag: 日志标签，固定 3~4 字符（如 SYS/IMU/MOT/QR/SRV/FSM）
 * @param  fmt: printf 风格格式串，禁止使用 %f（小数用整数放大法）
 * @retval 无
 * @note   输出格式：[TAG] 内容\r\n，经 UART5 输出
 */
void Debug_Log(const char *tag, const char *fmt, ...);

/**
 * @brief  蜂鸣器鸣叫配置（非阻塞，立即返回）
 * @param  times:  鸣叫次数；0=持续断续鸣叫（循环模式，供故障报警），>0 鸣叫 times 次
 * @param  on_ms:  每次鸣响时长，单位 ms
 * @param  off_ms: 相邻两次鸣叫的间隔时长，单位 ms
 * @retval 无
 * @note   非阻塞：仅设置模式并启动节拍，实际 PWM 启停由 BSP_Beep_Update() 管理；
 *         禁止与 HAL_Delay 配合使用；持续模式须用 BSP_Beep_Stop() 停止
 */
void BSP_Beep(uint8_t times, uint16_t on_ms, uint16_t off_ms);

/**
 * @brief  立即停止蜂鸣器（静音）
 * @param  无
 * @retval 无
 * @note   停止 PWM 输出并清除鸣叫状态，BZ_LED2 同步熄灭
 */
void BSP_Beep_Stop(void);

/**
 * @brief  蜂鸣器非阻塞节拍更新（主循环周期调用）
 * @param  无
 * @retval 无
 * @note   基于 HAL_GetTick() 管理 on/off 时序：鸣叫段启动 PWM，间歇段停止 PWM；
 *         同步控制 BZ_LED2（鸣叫亮、静默灭）。不得在中断中调用
 */
void BSP_Beep_Update(void);

/**
 * @brief  板载 LED 通用设置接口（BZ_LED3 预留，供后续模块使用）
 * @param  id: 板载 LED 编号（BZ_LED1~BZ_LED3）
 * @param  on: 1=点亮，0=熄灭（内部按低电平点亮极性转换）
 * @retval 无
 * @note   BZ_LED1（心跳）、BZ_LED2（蜂鸣同步）由系统自动管理，外部不应直接设置
 */
void BSP_LED_Set(uint8_t id, uint8_t on);

/**
 * @brief  外接 LED 流水灯非阻塞更新（在主循环中周期调用）
 * @param  无
 * @retval 无
 * @note   基于 HAL_GetTick()，每 LED_FLOW_PERIOD_MS 切换点亮一个 LED（G→B→Y 循环）
 */
void BSP_LED_Flow_Update(void);

/**
 * @brief  心跳日志非阻塞更新（在主循环中周期调用）
 * @param  无
 * @retval 无
 * @note   每 HEARTBEAT_PERIOD_MS 输出一条 [SYS] alive, t=xxxxx ms，同时翻转 BZ_LED1
 */
void BSP_Heartbeat_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_DEBUG_H */
