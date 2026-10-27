/**
 ******************************************************************************
 * @file    bsp_chassis_test.h
 * @brief   OLED 车机测试页（P4 麦轮底盘位置模式调试）
 * @author  nx_car
 * @date    2026-09-16
 ******************************************************************************
 * @note   本页从"模块测试页"CHS 项进入，可选择 7 个整车动作并启动，
 *         页面实时显示动作名、状态与进度。页面四行 8x16 大字体：
 *           第1行 标题   CHASSIS TEST
 *           第2行 动作名 FWD/BACK/LEFT/RIGHT/ROT/DIAG/ARC
 *           第3行 状态   IDLE / RUN / DONE / ABORT
 *           第4行 进度   d=320 a=45（已走 mm / 已转 deg）
 *         本页按键语义独立（不复用测试页语义）：
 *           KEY1=上一个动作、KEY2=下一个动作（运行中忽略）、
 *           KEY3=启动 / 中止（运行中）、KEY4 或 KEY5=中止并退出回测试页。
 *         注意：进入本页不自动启动运动，必须按 KEY3 启动。
 ******************************************************************************
 */
#ifndef __BSP_CHASSIS_TEST_H
#define __BSP_CHASSIS_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "bsp_key.h"
#include "bsp_chassis.h"

/* ===================== 函数声明 ===================== */

/**
 * @brief  进入车机测试页（测试页 CHS 项 run 回调调用）
 * @param  无
 * @retval 无
 * @note   只切页并选定第一个动作，不自动启动运动（防误触发车轮）。
 */
void CHS_Test(void);

/**
 * @brief  退出车机测试页（测试框架 stop 回调 / KEY4 / KEY5 调用）
 * @param  无
 * @retval 无
 * @note   中止运动 + 四轮失能；测试页 CHS 项状态复位 IDLE，
 *         光标保持 CHS，可再次按 KEY3 进入。
 *         不调用 Test_Exit（测试页活动标志与光标位置均保持）。
 */
void CHS_Test_Stop(void);

/**
 * @brief  车机测试页按键处理（本页活动时由 main 路由调用）
 * @param  key: 按键 ID（KEY1~KEY5）
 * @retval 无
 * @note   运行中屏蔽动作切换；KEY3 在运行中为"中止"，非运行为"启动"。
 */
void ChassisTest_HandleKey(KeyID_t key);

/**
 * @brief  查询车机测试页是否活动
 * @param  无
 * @retval 1=本页活动（按键交本页处理），0=非本页
 */
uint8_t ChassisTest_IsActive(void);

/**
 * @brief  取当前选中的动作编号（供页面绘制）
 * @param  无
 * @retval ChassisAct_t 值
 */
uint8_t ChassisTest_GetAct(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CHASSIS_TEST_H */
