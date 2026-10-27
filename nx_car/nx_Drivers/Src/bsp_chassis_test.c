/**
 ******************************************************************************
 * @file    bsp_chassis_test.c
 * @brief   OLED 车机测试页实现（P4 麦轮底盘位置模式调试）
 * @author  nx_car
 * @date    2026-09-16
 ******************************************************************************
 * @note   页面只在按键/运动状态变化时通过 Display_ChassisRefresh() 置脏标志
 *         触发重绘（仿单轮页/扫码页做法），不做周期性全量重绘。
 *         所有运动经 bsp_chassis 编排层下发，日志统一 [CHS] 标签。
 ******************************************************************************
 */
#include "bsp_chassis_test.h"
#include "bsp_chassis.h"
#include "bsp_oled_ui.h"
#include "bsp_test.h"
#include "bsp_debug.h"

/* ===================== 内部状态 ===================== */

static uint8_t chsTestActive = 0U;            /* 本页活动标志：0=否，1=是 */
static uint8_t chsActSel     = CHS_ACT_FWD;   /* 当前选中动作（0~CHS_ACT_COUNT-1） */

/* ===================== 公开接口 ===================== */

/**
 * @brief  进入车机测试页（测试页 CHS 项 run 回调调用）
 * @param  无
 * @retval 无
 * @note   切到 PAGE_CHASSIS 并默认选中第一个动作；不自动启动运动，
 *         避免误触发电机转动（试车安全）。
 */
void CHS_Test(void)
{
    chsTestActive = 1U;
    chsActSel     = (uint8_t)CHS_ACT_FWD;

    Display_SetPage(PAGE_CHASSIS);

    /* 进入本页即把"当前位置 + 当前朝向"设为相对坐标的原点(0,0)/0°，
       之后所有动作的位移都累加到这个坐标系里（退出页面不保留） */
    Chassis_OdomReset();

    Debug_Log("CHS", "chassis test enter");
}

/**
 * @brief  退出车机测试页
 * @param  无
 * @retval 无
 * @note   中止运动 + 四轮失能；CHS 测试项状态复位 IDLE（光标保持 CHS）。
 */
void CHS_Test_Stop(void)
{
    Chassis_Abort();

    chsTestActive = 0U;

    /* 复位测试页 CHS 项状态（不复位光标，光标保持在 CHS） */
    testItems[TEST_ID_CHS].state = TEST_STATE_IDLE;

    Debug_Log("CHS", "chassis test exit");
}

/**
 * @brief  车机测试页按键处理
 * @param  key: 按键 ID（KEY1~KEY5，本页语义独立）
 * @retval 无
 * @note   KEY1/KEY2=切换动作（运行中忽略，防运动中被改目标）；
 *         KEY3=启动 / 中止；KEY4、KEY5=中止并退出回测试页。
 */
void ChassisTest_HandleKey(KeyID_t key)
{
    if (key == KEY_NONE)
    {
        return;
    }

    switch (key)
    {
        case KEY1:   /* 上一个动作（循环） */
        case KEY2:   /* 下一个动作（循环） */
            if (Chassis_IsActive())
            {
                /* 运行中禁止切换动作：目标已下发，切换会导致显示与实际不符 */
                Debug_Log("CHS", "act switch ignored while running");
                break;
            }

            if (key == KEY1)
            {
                chsActSel = (chsActSel == 0U) ? (uint8_t)(CHS_ACT_COUNT - 1U)
                                              : (uint8_t)(chsActSel - 1U);
            }
            else
            {
                chsActSel = (uint8_t)((chsActSel + 1U) % (uint8_t)CHS_ACT_COUNT);
            }

            Debug_Log("CHS", "select %s", Chassis_GetActName(chsActSel));
            Display_ChassisRefresh();
            break;

        case KEY3:   /* 空闲/到位 -> 启动；运行中 -> 中止 */
            if (Chassis_IsActive())
            {
                Chassis_Abort();
            }
            else
            {
                Chassis_Start(chsActSel);
            }
            Display_ChassisRefresh();
            break;

        case KEY4:   /* 中止 + 退出回测试页（KEY4 统一退出规范） */
        case KEY5:   /* 同上（等效退出） */
            CHS_Test_Stop();
            Display_SetPage(PAGE_TEST);
            break;

        default:
            break;
    }
}

/**
 * @brief  查询车机测试页是否活动
 * @param  无
 * @retval 1=本页活动，0=非本页
 */
uint8_t ChassisTest_IsActive(void)
{
    return chsTestActive;
}

/**
 * @brief  取当前选中的动作编号
 * @param  无
 * @retval ChassisAct_t 值
 */
uint8_t ChassisTest_GetAct(void)
{
    return chsActSel;
}
