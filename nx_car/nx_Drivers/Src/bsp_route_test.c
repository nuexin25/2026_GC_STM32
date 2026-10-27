/**
 ******************************************************************************
 * @file    bsp_route_test.c
 * @brief   OLED 跑图测试页实现（P7 无障碍物跑图）
 * @author  nx_car
 * @date    2026-09-17
 ******************************************************************************
 * @note   页面只在按键/跑图状态变化时通过 Display_RouteRefresh() 置脏标志触发
 *         重绘（仿车机页/扫码页做法），不做周期性全量重绘。
 *         所有运动经 bsp_route 执行器下发，日志统一 [ROUTE] 标签。
 ******************************************************************************
 */
#include "bsp_route_test.h"
#include "bsp_route.h"
#include "bsp_chassis.h"    /* 取相对坐标朝向 Chassis_GetOdomYaw（页面显示用） */
#include "bsp_oled_ui.h"
#include "bsp_test.h"
#include "bsp_debug.h"
#include <stdio.h>

/* ===================== 内部状态 ===================== */

static uint8_t routeTestActive = 0U;   /* 本页活动标志：0=否，1=是 */

/* ===================== 公开接口 ===================== */

/**
 * @brief  进入跑图测试页（测试页 ROUTE 项 run 回调）
 * @param  无
 * @retval 无
 * @note   切到 PAGE_ROUTE；先把跑图状态复位为 IDLE（若上次是 DONE/ABORT），
 *         不自动启动跑图，避免误触发车轮（试车安全）。
 */
void ROUTE_Test(void)
{
    routeTestActive = 1U;

    /* 若仍在跑（异常路径）先停掉，再复位状态机，保证页面显示与电机状态一致 */
    if (Route_IsActive())
    {
        Route_Abort();
    }
    Route_Init();

    Display_SetPage(PAGE_ROUTE);
    Debug_Log("ROUTE", "route test enter, segs=%u", (unsigned int)Route_GetSegCount());
}

/**
 * @brief  退出跑图测试页
 * @param  无
 * @retval 无
 * @note   中止跑图 + 四轮失能；ROUTE 测试项状态复位 IDLE（光标保持 ROUTE）。
 */
void ROUTE_Test_Stop(void)
{
    Route_Abort();

    routeTestActive = 0U;

    /* 复位测试页 ROUTE 项状态（不复位光标，光标保持在 ROUTE） */
    testItems[TEST_ID_ROUTE].state = TEST_STATE_IDLE;

    Debug_Log("ROUTE", "route test exit");
}

/**
 * @brief  跑图测试页按键处理
 * @param  key: 按键 ID（KEY1~KEY5，本页语义独立）
 * @retval 无
 * @note   KEY3=启动 / 中止；KEY4、KEY5=中止并退出回测试页；KEY1/KEY2 忽略。
 */
void RouteTest_HandleKey(KeyID_t key)
{
    if (key == KEY_NONE)
    {
        return;
    }

    switch (key)
    {
        case KEY3:   /* 空闲/完成 -> 启动跑图；跑图中 -> 中止 */
            if (Route_IsActive())
            {
                Route_Abort();
            }
            else
            {
                Route_Start();
            }
            Display_RouteRefresh();
            break;

        case KEY4:   /* 中止 + 退出回测试页（KEY4 统一退出规范） */
        case KEY5:   /* 同上（等效退出） */
            ROUTE_Test_Stop();
            Display_SetPage(PAGE_TEST);
            break;

        default:     /* KEY1/KEY2 本页不用（跑图顺序由路段表决定，不做人工切换） */
            break;
    }
}

/**
 * @brief  查询跑图测试页是否活动
 * @param  无
 * @retval 1=本页活动，0=非本页
 */
uint8_t RouteTest_IsActive(void)
{
    return routeTestActive;
}

/**
 * @brief  取路段进度文本（页面第2行）
 * @param  无
 * @retval "SEG 2/4 w:1111" 格式静态缓冲指针
 * @note   末尾 `w:` 后 4 位依次是 FL/FR/RL/RR 是否**被下过命令**（1=该轮本次应转动，
 *         0=位移为 0、不下发命令）。用来判断"轮子不动"是正常还是异常：
 *         位为 0 的轮不动是**正常**（例如 45° 斜行只需两个轮）；
 *         位为 1 的轮不动才是丢帧/驱动忽略/电机异常，需要查 CAN 总线与电机。
 */
const char *RouteTest_GetSegText(void)
{
    static char buf[20];
    uint8_t     m = Route_GetWheelMask();

    snprintf(buf, sizeof(buf), "SEG %u/%u w:%u%u%u%u",
             (unsigned int)Route_GetSegIdx(), (unsigned int)Route_GetSegCount(),
             (unsigned int)((m >> 0) & 1U), (unsigned int)((m >> 1) & 1U),
             (unsigned int)((m >> 2) & 1U), (unsigned int)((m >> 3) & 1U));

    return buf;
}

/**
 * @brief  取状态+阶段+朝向文本（页面第3行）
 * @param  无
 * @retval 形如 "RUN MOVE H=-90" / "IDLE WAIT H=0" 的静态缓冲指针
 * @note   末尾的 H 是"相对跑图起点的朝向"（单位 deg，取整）：
 *         转弯后 H 应立刻跟着变（表里转角 +90 的左转对应 H≈-90），
 *         若转弯前后 H 不变，说明 IMU 朝向没更新，坐标与行进方向都会算错。
 */
const char *RouteTest_GetStatusText(void)
{
    static char buf[24];

    snprintf(buf, sizeof(buf), "%s %s H=%d",
             Route_GetStateText(), Route_GetPhaseText(),
             (int)Chassis_GetOdomYaw());

    return buf;
}

/**
 * @brief  取当前段目标文本（页面第4行）
 * @param  无
 * @retval 形如 "to(500,500)" / "turn90" 的静态缓冲指针
 * @note   直接复用执行器的目标文本（路段表在 bsp_route.c 内部，不对外暴露）。
 */
const char *RouteTest_GetTargetText(void)
{
    return Route_GetSegText();
}
