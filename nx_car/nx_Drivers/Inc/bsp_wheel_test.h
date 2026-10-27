/**
 ******************************************************************************
 * @file    bsp_wheel_test.h
 * @brief   OLED 单轮点动测试页（P3 电机单轮调试）
 * @author  nx_car
 * @date    2026-09-09
 ******************************************************************************
 * @note   本页从"模块测试页"MOT 项进入，可单独选择 4 个车轮中的任意一个，
 *         控制其正转/反转/停止。页面四行 8x16 大字体：
 *           第1行 标题    WHEEL TEST
 *           第2行 当前电机 MOT:1 FL（1=FL 2=FR 3=RL 4=RR）
 *           第3行 当前方向 DIR:FWD / REV / STOP
 *           第4行 点动速度 RPM:80
 *         本页按键语义独立（不复用测试页语义）：
 *           KEY1=上一个电机(1->4->3->2->1)、KEY2=下一个(1->2->3->4->1)、
 *           KEY3=正转 toggle、KEY4=反转 toggle、KEY5=全停退出回测试页。
 *         注意：KEY4 在本页是"反转"，不是退出；退出一律 KEY5。
 ******************************************************************************
 */
#ifndef __BSP_WHEEL_TEST_H
#define __BSP_WHEEL_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "bsp_key.h"

/* ===================== 方向状态枚举 ===================== */
typedef enum {
    WHEEL_DIR_STOP = 0,   /* 停止 */
    WHEEL_DIR_FWD,        /* 正转 */
    WHEEL_DIR_REV         /* 反转 */
} WheelDir_t;

/* ===================== 函数声明 ===================== */

/**
 * @brief  进入单轮点动测试页（测试页 MOT 项 run 回调调用）
 * @param  无
 * @retval 无
 * @note   进入时自动使能 1 号电机并切到 PAGE_WHEEL 页面；
 *         使能只发生在进入本页或切换电机时，上电/初始化阶段不使能。
 */
void WheelTest_Enter(void);

/**
 * @brief  退出单轮点动测试页（KEY5 或测试框架 stop 回调调用）
 * @param  无
 * @retval 无
 * @note   四个电机全部立即停止并失能；测试页 MOT 项状态复位 IDLE。
 *         不调用 Test_Enter（测试页活动标志未退出、光标保持在 MOT）。
 */
void WheelTest_Exit(void);

/**
 * @brief  单轮点动页按键处理（本页活动时由 main 路由调用）
 * @param  key: 按键 ID（KEY1~KEY5）
 * @retval 无
 * @note   切换电机先停旧轮再使能新轮；正反转切换先停再反向防冲击。
 */
void WheelTest_HandleKey(KeyID_t key);

/**
 * @brief  查询单轮点动页是否活动
 * @param  无
 * @retval 1=本页活动（按键交本页处理），0=非本页
 */
uint8_t WheelTest_IsActive(void);

/**
 * @brief  获取当前选中的电机地址（供页面绘制）
 * @param  无
 * @retval 电机地址 1~4
 */
uint8_t WheelTest_GetMotor(void);

/**
 * @brief  获取当前方向状态（供页面绘制）
 * @param  无
 * @retval WHEEL_DIR_STOP / WHEEL_DIR_FWD / WHEEL_DIR_REV
 */
WheelDir_t WheelTest_GetDirState(void);

/**
 * @brief  获取点动速度设定值（供页面绘制）
 * @param  无
 * @retval 点动转速 RPM（MOTOR_JOG_RPM）
 */
uint16_t WheelTest_GetRpm(void);

/**
 * @brief  获取电机地址对应的车轮位置缩写（供页面绘制）
 * @param  addr: 电机地址 1~4
 * @retval "FL"/"FR"/"RL"/"RR"；非法地址返回 "??"
 */
const char *WheelTest_MotorName(uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_WHEEL_TEST_H */
