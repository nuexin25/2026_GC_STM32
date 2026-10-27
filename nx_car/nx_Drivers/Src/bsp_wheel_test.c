/**
 ******************************************************************************
 * @file    bsp_wheel_test.c
 * @brief   OLED 单轮点动测试页实现（P3 电机单轮调试）
 * @author  nx_car
 * @date    2026-09-09
 ******************************************************************************
 * @note   页面状态机：当前电机地址（1~4）+ 方向状态（STOP/FWD/REV）。
 *         页面只在按键后变化，通过 Display_WheelTestRefresh() 置脏标志触发
 *         重绘（仿 Display_TestRefresh），不做周期性全量重绘。
 *         所有电机动作经 bsp_motor.h 封装层下发，日志统一 [MOT] 标签。
 ******************************************************************************
 */
#include "bsp_wheel_test.h"
#include "bsp_motor.h"
#include "bsp_oled_ui.h"
#include "bsp_test.h"
#include "bsp_debug.h"

/* ===================== 内部状态 ===================== */
static uint8_t    wheelActive = 0;              /* 本页活动标志：0=否，1=是 */
static uint8_t    curMotor    = MOTOR_ADDR_FL;  /* 当前选中电机地址 1~4 */
static WheelDir_t curDir      = WHEEL_DIR_STOP; /* 当前方向状态 */

/* ===================== 内部工具 ===================== */

/**
 * @brief  切换当前电机：先停旧轮（防带速切换），再使能新轮，方向复位 STOP
 * @param  newAddr: 新电机地址 1~4（调用前保证与当前不同且合法）
 * @retval 无
 */
static void WheelTest_SwitchMotor(uint8_t newAddr)
{
    /* 先停当前电机 */
    Motor_Stop(curMotor);
    /* 使能新电机（使能只发生在进入本页或切换电机时） */
    curMotor = newAddr;
    curDir   = WHEEL_DIR_STOP;
    Motor_Enable(curMotor, 1U);

    Debug_Log("MOT", "switch mot=%u", (unsigned int)curMotor);
    Display_WheelTestRefresh();
}

/* ===================== 公开接口 ===================== */

/**
 * @brief  进入单轮点动测试页
 * @param  无
 * @retval 无
 * @note   默认选中 1 号电机并自动使能；切到 PAGE_WHEEL（SetPage 内置脏标志）。
 */
void WheelTest_Enter(void)
{
    wheelActive = 1U;
    curMotor    = MOTOR_ADDR_FL;
    curDir      = WHEEL_DIR_STOP;

    /* 进入页面自动使能 1 号电机（上电初始化阶段不使能，使能只在此处发生） */
    Motor_Enable(curMotor, 1U);

    Display_SetPage(PAGE_WHEEL);
    Debug_Log("MOT", "wheel test enter, mot=%u", (unsigned int)curMotor);
}

/**
 * @brief  退出单轮点动测试页
 * @param  无
 * @retval 无
 * @note   四个电机全部立即停止并失能；MOT 测试项状态复位 IDLE，
 *         这样退回测试页后光标仍在 MOT、状态符为 '.'，按 KEY3 可直接再次进入。
 *         不调用 Test_Enter/Test_Exit（测试页活动标志与光标位置均保持）。
 */
void WheelTest_Exit(void)
{
    uint8_t addr;

    /* 全停 + 失能四轮 */
    for (addr = MOTOR_ADDR_MIN; addr <= MOTOR_ADDR_MAX; addr++)
    {
        Motor_Stop(addr);
        Motor_Disable(addr);
    }

    wheelActive = 0U;
    curDir      = WHEEL_DIR_STOP;

    /* 复位测试页 MOT 项状态（不复位光标，光标保持在 MOT） */
    testItems[TEST_ID_MOT].state = TEST_STATE_IDLE;

    Debug_Log("MOT", "wheel test exit, all stop");
}

/**
 * @brief  单轮点动页按键处理
 * @param  key: 按键 ID（KEY1~KEY5，本页语义独立）
 * @retval 无
 * @note   KEY1=上一个电机(1->4->3->2->1)、KEY2=下一个(1->2->3->4->1)、
 *         KEY3=正转 toggle、KEY4=反转 toggle、KEY5=全停退出回测试页。
 */
void WheelTest_HandleKey(KeyID_t key)
{
    if (key == KEY_NONE)
    {
        return;
    }

    switch (key)
    {
        case KEY1:   /* 上一个电机：1->4->3->2->1 循环 */
        {
            uint8_t newAddr = (curMotor == MOTOR_ADDR_MIN)
                              ? MOTOR_ADDR_MAX
                              : (uint8_t)(curMotor - 1U);
            WheelTest_SwitchMotor(newAddr);
            break;
        }

        case KEY2:   /* 下一个电机：1->2->3->4->1 循环 */
        {
            uint8_t newAddr = (uint8_t)(curMotor % MOTOR_NUM) + 1U;
            WheelTest_SwitchMotor(newAddr);
            break;
        }

        case KEY3:   /* 正转 toggle：未正转则正转，已正转则停止 */
            if (curDir == WHEEL_DIR_FWD)
            {
                Motor_Stop(curMotor);
                curDir = WHEEL_DIR_STOP;
                Debug_Log("MOT", "mot=%u fwd stop", (unsigned int)curMotor);
            }
            else
            {
                /* 正反转切换：先停止再反向，防冲击 */
                if (curDir == WHEEL_DIR_REV)
                {
                    Motor_Stop(curMotor);
                }
                Motor_SetVel(curMotor, (int16_t)MOTOR_JOG_RPM, MOTOR_JOG_ACC);
                curDir = WHEEL_DIR_FWD;
                Debug_Log("MOT", "mot=%u fwd start", (unsigned int)curMotor);
            }
            Display_WheelTestRefresh();
            break;

        case KEY4:   /* 反转 toggle：未反转则反转，已反转则停止（本页 KEY4 不是退出） */
            if (curDir == WHEEL_DIR_REV)
            {
                Motor_Stop(curMotor);
                curDir = WHEEL_DIR_STOP;
                Debug_Log("MOT", "mot=%u rev stop", (unsigned int)curMotor);
            }
            else
            {
                /* 正反转切换：先停止再反向，防冲击 */
                if (curDir == WHEEL_DIR_FWD)
                {
                    Motor_Stop(curMotor);
                }
                Motor_SetVel(curMotor, -(int16_t)MOTOR_JOG_RPM, MOTOR_JOG_ACC);
                curDir = WHEEL_DIR_REV;
                Debug_Log("MOT", "mot=%u rev start", (unsigned int)curMotor);
            }
            Display_WheelTestRefresh();
            break;

        case KEY5:   /* 四个电机全部停止，退出本页回测试页（光标仍在 MOT） */
            WheelTest_Exit();
            Display_SetPage(PAGE_TEST);
            break;

        default:
            break;
    }
}

/**
 * @brief  查询单轮点动页是否活动
 * @param  无
 * @retval 1=本页活动，0=非本页
 */
uint8_t WheelTest_IsActive(void)
{
    return wheelActive;
}

/**
 * @brief  获取当前选中的电机地址
 * @param  无
 * @retval 电机地址 1~4
 */
uint8_t WheelTest_GetMotor(void)
{
    return curMotor;
}

/**
 * @brief  获取当前方向状态
 * @param  无
 * @retval WHEEL_DIR_STOP / WHEEL_DIR_FWD / WHEEL_DIR_REV
 */
WheelDir_t WheelTest_GetDirState(void)
{
    return curDir;
}

/**
 * @brief  获取点动速度设定值
 * @param  无
 * @retval 点动转速 RPM（MOTOR_JOG_RPM）
 */
uint16_t WheelTest_GetRpm(void)
{
    return (uint16_t)MOTOR_JOG_RPM;
}

/**
 * @brief  获取电机地址对应的车轮位置缩写
 * @param  addr: 电机地址 1~4
 * @retval "FL"/"FR"/"RL"/"RR"；非法地址返回 "??"
 */
const char *WheelTest_MotorName(uint8_t addr)
{
    switch (addr)
    {
        case MOTOR_ADDR_FL: return "FL";
        case MOTOR_ADDR_FR: return "FR";
        case MOTOR_ADDR_RL: return "RL";
        case MOTOR_ADDR_RR: return "RR";
        default:            return "??";
    }
}
