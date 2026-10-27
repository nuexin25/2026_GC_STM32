/**
 ******************************************************************************
 * @file    bsp_rpi_test.h
 * @brief   OLED 树莓派通讯测试页（P6 视觉链路人工验证，协议 v2.0）
 * @author  nx_car
 * @date    2026-09-15
 ******************************************************************************
 * @note   本页从"模块测试页"RPI 项进入，用于人工验证与视觉端的收发链路（v2.0）：
 *           · 看它发上来的 $RCP / $RZ / $CCG / $SHOT 数据与 $PING 心跳；
 *           · 手工给它下发 7 条请求帧，观察视觉端反应。
 *         通讯模块上电即后台运行（RPi_Init 在上电调用），进入/退出本页
 *         不影响 USART1 接收。**本页只验证链路与数据，不驱动底盘、不联动机械臂。**
 *
 *         页面四行 8x16 大字体（每行最多 16 字符）：
 *           第1行  "RPI v2.0" + 在线状态  RPI v2.0 ONLINE / NOHB / OFFLINE
 *           第2行  视觉端状态 + 帧计数  TURN RX:12 B:0
 *           第3行  $RZ 结果（位号:颜色）  RZ 1:1 2:3 3:4（无数据 RZ --）
 *           第4行  当前选中项 + $RCP/$CCG/$SHOT 摘要  CCG3 R42,-15 C3=1
 *         本页按键语义独立：
 *           KEY1/KEY2 = 上一项 / 下一项（RCP/RZ/CCG,1/CCG,2/CCG,3/SHOT/CODE 循环）；
 *           KEY3      = 发送当前选中项（v2.0 直接发帧，不做任何机械臂联动）；
 *           KEY4/KEY5 = 退出回测试页。
 *
 *         【行长限制说明】8x16 字体下每行只能放 16 字符。第4行要同时容纳
 *         "选中项短名 + RCP/CCG/SHOT 三段摘要"，三项都有效时尾部会被屏幕截断——
 *         此时完整值请看 UART5 日志（[RPi] 前缀）。
 ******************************************************************************
 */
#ifndef __BSP_RPI_TEST_H
#define __BSP_RPI_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "bsp_key.h"

/* ===================== 函数声明 ===================== */

/**
 * @brief  进入树莓派测试页（测试页 RPI 项 run 回调）
 * @param  无
 * @retval 无
 * @note   切到 PAGE_RPI；通讯模块上电已启动，本页只读显示当前缓存数据，
 *         **不自动发送任何指令**。
 */
void RPI_Test(void);

/**
 * @brief  树莓派测试页周期刷新（测试页 RPI 项的 update 回调）
 * @param  无
 * @retval 无
 * @note   只在"在线状态发生变化"时置脏，不做周期高频重绘
 *         （数据本身的新鲜度刷新由 bsp_rpi 收到新帧/数据过期时驱动）。
 */
void RPI_Test_Update(void);

/**
 * @brief  退出树莓派测试页（KEY4/KEY5 或测试框架 stop 回调）
 * @param  无
 * @retval 无
 * @note   回测试页，不停 USART1 接收（通讯始终后台运行）；
 *         RPI 测试项状态复位 IDLE，光标保持 RPI。
 */
void RPI_Test_Stop(void);

/**
 * @brief  树莓派测试页按键处理（本页活动时由 main 路由调用）
 * @param  key: 按键 ID（KEY1~KEY5）
 * @retval 无
 * @note   KEY1/KEY2=切换指令项；KEY3=发送；KEY4/KEY5=退出回测试页。
 */
void RPiTest_HandleKey(KeyID_t key);

/**
 * @brief  查询树莓派测试页是否活动
 * @param  无
 * @retval 1=本页活动（按键交本页处理），0=非本页
 */
uint8_t RPiTest_IsActive(void);

/**
 * @brief  获取第1行文本（标题 + 在线状态）
 * @param  无
 * @retval "RPI v2.0 ONLINE"（心跳正常）/ "RPI v2.0 NOHB"（有数据无心跳）/ "RPI v2.0 OFFLINE"（无任何数据）
 */
const char *RPiTest_GetTitleText(void);

/**
 * @brief  获取第2行文本（视觉端状态 + 帧计数）
 * @param  无
 * @retval 静态缓冲指针，如 "TURN RX:12 B:0"（状态取自 $PING 的 IDLE/TURN/SHOT）
 */
const char *RPiTest_GetColorText(void);

/**
 * @brief  获取第3行文本（$RZ 结果：位号:颜色）
 * @param  无
 * @retval 静态缓冲指针，如 "RZ 1:1 2:3 3:4"；无数据时 "RZ --"
 */
const char *RPiTest_GetTargetText(void);

/**
 * @brief  获取第4行文本（当前选中项 + $RCP/$CCG/$SHOT 摘要）
 * @param  无
 * @retval 静态缓冲指针，如 "CCG3 R42,-15 C3=1"
 */
const char *RPiTest_GetCmdText(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_RPI_TEST_H */
