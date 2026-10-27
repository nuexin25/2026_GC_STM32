/**
 ******************************************************************************
 * @file    bsp_route_test.h
 * @brief   OLED 跑图测试页（P7 无障碍物跑图）
 * @author  nx_car
 * @date    2026-09-17
 ******************************************************************************
 * @note   本页从"模块测试页"ROUTE 项进入，按 KEY3 启动跑图，页面四行 8x16 大字体：
 *           第1行 标题   ROUTE TEST
 *           第2行 进度   SEG 2/4（当前段/总段数，0 起）
 *           第3行 状态   IDLE / RUN / DONE / ABORT + 阶段 TURN / MOVE / WAIT
 *           第4行 当前段目标（如 "to(500,500)" / "turn90" / "T90 (500,500)"）
 *         本页按键语义独立（不复用测试页语义）：
 *           KEY3=空闲时启动跑图 / 跑图中中止；KEY4 或 KEY5=中止并退出回测试页；
 *           KEY1/KEY2 本页不用。
 *         注意：进入本页不自动启动跑图，必须按 KEY3 启动（防误触发车轮）。
 ******************************************************************************
 */
#ifndef __BSP_ROUTE_TEST_H
#define __BSP_ROUTE_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "bsp_key.h"
#include "bsp_route.h"

/* ===================== 函数声明 ===================== */

/**
 * @brief  进入跑图测试页（测试页 ROUTE 项 run 回调 ROUTE_Test）
 * @param  无
 * @retval 无
 * @note   切到 PAGE_ROUTE；跑图状态复位为 IDLE，不自动启动（试车安全）。
 */
void ROUTE_Test(void);

/**
 * @brief  退出跑图测试页（测试框架 stop 回调 / KEY4 / KEY5 调用）
 * @param  无
 * @retval 无
 * @note   中止跑图 + 四轮失能；测试页 ROUTE 项状态复位 IDLE，
 *         光标保持 ROUTE，可再次按 KEY3 进入。
 *         不调用 Test_Exit（测试页活动标志与光标位置均保持）。
 */
void ROUTE_Test_Stop(void);

/**
 * @brief  跑图测试页按键处理（本页活动时由 main 路由调用）
 * @param  key: 按键 ID（KEY1~KEY5）
 * @retval 无
 * @note   KEY3=启动/中止；KEY4、KEY5=中止并退出回测试页；KEY1/KEY2 忽略。
 */
void RouteTest_HandleKey(KeyID_t key);

/**
 * @brief  查询跑图测试页是否活动
 * @param  无
 * @retval 1=本页活动（按键交本页处理），0=非本页
 */
uint8_t RouteTest_IsActive(void);

/**
 * @brief  取路段进度文本（供页面绘制第2行）
 * @param  无
 * @retval "SEG 2/4" 格式静态缓冲指针
 */
const char *RouteTest_GetSegText(void);

/**
 * @brief  取状态+阶段文本（供页面绘制第3行）
 * @param  无
 * @retval 形如 "RUN MOVE" / "IDLE WAIT" 的静态缓冲指针
 */
const char *RouteTest_GetStatusText(void);

/**
 * @brief  取当前段目标文本（供页面绘制第4行）
 * @param  无
 * @retval 形如 "to(500,500)" / "turn90" 的静态缓冲指针
 */
const char *RouteTest_GetTargetText(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ROUTE_TEST_H */
