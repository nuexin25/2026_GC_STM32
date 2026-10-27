/**
 ******************************************************************************
 * @file    bsp_qr_test.h
 * @brief   OLED 扫码测试页（P5 GM65 扫码人工验证）
 * @author  nx_car
 * @date    2026-09-10
 ******************************************************************************
 * @note   本页从"模块测试页"QR 项进入，用于人工对码验证扫码稳定性。
 *         扫码模块上电即后台运行（QR_Init 在上电调用），进入/退出本页
 *         不影响 UART4 接收，退出后再次扫到码仍会更新任务码页。
 *
 *         页面四行 8x16 大字体：
 *           第1行 标题   QR SCAN TEST
 *           第2行 状态   WAIT / OK / BAD / OFFLINE
 *           第3行 原始码 有则显示 "134+123+314+231"，无则 "---"
 *           第4行 解析   第一批颜色顺序缩写，如 "1:R 2:B 3:G"
 *         本页按键语义独立：
 *           KEY3=手动重扫触发（清当前结果回 WAIT，等待下一帧）；
 *           KEY4/KEY5=退出回测试页（KEY4 遵循"统一退出"规范，KEY5 等效）；
 *           KEY1/KEY2 本页不使用。
 ******************************************************************************
 */
#ifndef __BSP_QR_TEST_H
#define __BSP_QR_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "bsp_key.h"

/* ===================== 函数声明 ===================== */

/**
 * @brief  进入扫码测试页（测试页 QR 项 run 回调 QR_Test 调用）
 * @param  无
 * @retval 无
 * @note   切到 PAGE_QR（SetPage 内置脏标志）；不自动启动扫码
 *         （QR_Init 在上电已调用，扫码一直在跑），显示当前缓存结果。
 */
void QR_Test(void);

/**
 * @brief  退出扫码测试页（KEY4/KEY5 或测试框架 stop 回调调用）
 * @param  无
 * @retval 无
 * @note   回测试页，不停 UART4 接收（扫码始终后台运行）；
 *         QR 测试项状态复位 IDLE，退回后光标仍在 QR。
 */
void QR_Test_Stop(void);

/**
 * @brief  扫码测试页按键处理（本页活动时由 main 路由调用）
 * @param  key: 按键 ID（KEY1~KEY5）
 * @retval 无
 * @note   KEY3=手动重扫触发；KEY4/KEY5=退出回测试页；KEY1/KEY2 忽略。
 */
void QRTest_HandleKey(KeyID_t key);

/**
 * @brief  查询扫码测试页是否活动
 * @param  无
 * @retval 1=本页活动（按键交本页处理），0=非本页
 */
uint8_t QRTest_IsActive(void);

/**
 * @brief  获取状态文本（供页面绘制第2行）
 * @param  无
 * @retval "WAIT" / "OK" / "BAD" / "OFFLINE"
 */
const char *QRTest_GetStatusText(void);

/**
 * @brief  获取原始码字符串（供页面绘制第3行）
 * @param  无
 * @retval 原始码指针；无码时返回 "---"
 */
const char *QRTest_GetRawText(void);

/**
 * @brief  获取第一批颜色顺序缩写串（供页面绘制第4行）
 * @param  无
 * @retval 静态缓冲指针，如 "1:R 2:B 3:G"；无码时返回 "---"
 * @note   格式：{放置位}:{颜色缩写}，颜色缩写 1=R 2=B 3=G 4=Y。
 */
const char *QRTest_GetBatch1Text(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_QR_TEST_H */
