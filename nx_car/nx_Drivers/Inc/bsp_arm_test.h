/**
 ******************************************************************************
 * @file    bsp_arm_test.h
 * @brief   OLED 机械臂测试页（P8：4 个执行器逐个单独调试）
 * @author  nx_car
 * @date    2026-09-18
 ******************************************************************************
 * @note   本页从"模块测试页"的 SRV(ARM) 项按 KEY3 进入，页面四行 8x16 大字体：
 *           第1行 标题   ARM TEST
 *           第2行 执行器   TURN 1/4 5=EXIT（名字 + 序号 + KEY5 短按退出提示）
 *           第3行 状态     电机类：p=位置 H=回零完成 R=本段到位
 *                          舵机类：pos=上次目标位置 e=发送错误计数
 *           第4行 按键提示 K3=A K4=B K5L=H 之类（含义见下）
 *
 *         按键语义独立（**本页 KEY4 是副动作，退出一律用 KEY5**）：
 *           KEY1 短按=上一个执行器        KEY2 短按=下一个执行器
 *           KEY3 短按=主动作              KEY3 长按=电机5 手动回零（仅 TURN，备用入口）
 *           KEY4 短按=副动作
 *           KEY5 短按=停止（两个电机急停）+ 退出回测试页
 *           KEY5 长按=让本执行器归到 0 点（TURN=把当前位置标定为 0 点并写 flash 保存；
 *                                     LIFT=碰撞回零；舵机忽略）
 *
 *         各执行器的主/副动作：
 *           TURN：主=向 A 方向小幅度点动      副=向 B 方向小幅度点动
 *           LIFT：主=上升小幅度点动           副=下降小幅度点动
 *           CLAW：主=爪子伸出                 副=爪子后缩
 *           GRIP：主=爪子抓紧                 副=爪子松开
 ******************************************************************************
 */
#ifndef __BSP_ARM_TEST_H
#define __BSP_ARM_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "bsp_key.h"

/* ===================== 执行器列表 ===================== */

typedef enum {
    ARMT_ITEM_TURN = 0,   /* 电机5：手臂左右转向 */
    ARMT_ITEM_LIFT,       /* 电机6：爪子高度（丝杆） */
    ARMT_ITEM_CLAW,       /* 舵机 ID1：爪子前伸 / 后缩 */
    ARMT_ITEM_GRIP,       /* 舵机 ID2：爪子抓紧 / 松开 */
    ARMT_ITEM_MAX
} ArmTestItem_t;

/* ===================== 函数声明 ===================== */

/**
 * @brief  进入机械臂测试页（测试页 SRV/ARM 项 run 回调 ARM_Test）
 * @param  无
 * @retval 无
 * @note   切到 PAGE_ARM；默认选中 TURN 项，**不自动执行任何动作**；
 *         使能电机 5/6；舵机侧不发帧（调试板协议没有力矩装卸载指令）。 */
void ARM_Test(void);

/**
 * @brief  退出机械臂测试页（测试框架 stop 回调 / KEY5 短按调用）
 * @param  无
 * @retval 无
 * @note   两个电机急停；测试页 SRV 项状态复位 IDLE，光标保持 SRV。
 *         舵机无法软件急停（调试板协议没有停止指令），单次动作只有 800ms。
 */
void ARM_Test_Stop(void);

/**
 * @brief  机械臂测试页短按处理（本页活动时由 main 路由调用）
 * @param  key: 按键 ID（KEY1~KEY5）
 * @retval 无
 * @note   KEY1/KEY2=切换执行器；KEY3/KEY4=主动作/副动作；
 *         KEY5=停止两个电机并退出回测试页（本页 KEY4 不是退出键）。
 */
void ArmTest_HandleKey(KeyID_t key);

/**
 * @brief  机械臂测试页长按处理
 * @param  longKey: 长按事件对应的按键 ID（KEY_NONE 表示无长按事件）
 * @retval 无
 * @note   【为什么长按要单独一个入口】bsp_key 的长按事件由 Key_GetLongPress()
 *         读取并**消费一次**，长按不会同时产生短按事件；而 main 里在进入按键
 *         路由前已经取过一次长按（用于菜单判断），所以长按必须在那一处就派发，
 *         不能靠 ArmTest_HandleKey 的形参带进来。
 *         本页长按语义：KEY5 = "让本执行器归到 0 点"（TURN=标定当前位置为0点并写
 *         flash 保存；LIFT=碰撞回零；舵机忽略）；KEY3 = 电机5 手动回零（备用）。
 */
void ArmTest_HandleLongKey(KeyID_t longKey);

/**
 * @brief  本页主循环节拍（置脏标志，让状态行/位置跟得上变化）
 * @param  无
 * @retval 无
 * @note   只置脏，实际重绘仍由 OLED 侧按 REFRESH_PERIOD_MS 节流，
 *         不会高频全量重绘。
 */
void ArmTest_Update(void);

/**
 * @brief  查询机械臂测试页是否活动
 * @param  无
 * @retval 1=本页活动（按键交本页处理），0=非本页
 */
uint8_t ArmTest_IsActive(void);

/**
 * @brief  取当前执行器文本（供页面第2行）
 * @param  无
 * @retval 形如 "TURN 1/4 5=EXIT" 的静态缓冲指针
 */
const char *ArmTest_GetItemText(void);

/**
 * @brief  取当前执行器状态文本（供页面第3行）
 * @param  无
 * @retval 电机类："p=200 H1 R0"（p=位置(下发脉冲域)，读不到显示 --）
 *         舵机类："pos=800 e0"（上次目标位置 + 发送错误计数）
 */
const char *ArmTest_GetStateText(void);

/**
 * @brief  取按键提示文本（供页面第4行）
 * @param  无
 * @retval 形如 "K3=A K4=B K5L=H" / "K3=OUT K4=IN" 的静态缓冲指针
 * @note   K5L=H 表示"长按 KEY5 回零(Home)"；退出见第2行末尾的 5=EXIT。
 */
const char *ArmTest_GetHintText(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ARM_TEST_H */
