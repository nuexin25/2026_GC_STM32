/**
 ******************************************************************************
 * @file    bsp_arm_test.c
 * @brief   OLED 机械臂测试页实现（P8：4 个执行器逐个单独调试）
 * @author  nx_car
 * @date    2026-09-18
 ******************************************************************************
 * @note   页面只在按键/状态变化时通过 Display_ArmRefresh() 置脏标志触发重绘
 *         （仿车机页/扫码页/跑图页做法），实际重绘受 REFRESH_PERIOD_MS 节流。
 *         所有动作都经 bsp_arm（电机 5/6）与官方库 LobotServoController
 *         （舵机 ID1/ID2，经幻尔 Bus-Servo-Controller 调试板转发）下发，
 *         日志分别用 [ARM] / [SRV] 标签。
 ******************************************************************************
 */
#include "bsp_arm_test.h"
#include "bsp_arm.h"               /* 电机 5/6 接口 + 舵机统一下发入口 Arm_ServoMove */
#include "bsp_oled_ui.h"
#include "bsp_test.h"
#include "bsp_debug.h"
#include "bsp_arm_action.h"   /* ★ArmAct_TrimCancelAll：进本页手动点动前先丢掉动作库的"后台精修" */
#include <stdio.h>

/* ===================== 内部状态 ===================== */

static uint8_t armTestActive = 0U;                  /* 本页活动标志：0=否，1=是 */
static uint8_t armTestItem   = ARMT_ITEM_TURN;      /* 当前选中的执行器 */

/* ===================== 内部工具 ===================== */

/**
 * @brief  取当前执行器对应的电机地址
 * @param  无
 * @retval ARM_ADDR_TURN / ARM_ADDR_LIFT；当前项是舵机时返回 0
 */
static uint8_t ArmTest_CurAddr(void)
{
    if (armTestItem == ARMT_ITEM_TURN)
    {
        return ARM_ADDR_TURN;
    }
    if (armTestItem == ARMT_ITEM_LIFT)
    {
        return ARM_ADDR_LIFT;
    }
    return 0U;
}

/**
 * @brief  取当前执行器对应的舵机 ID
 * @param  无
 * @retval SERVO_ID_CLAW / SERVO_ID_GRIP；当前项是电机时返回 0
 */
static uint8_t ArmTest_CurServoId(void)
{
    if (armTestItem == ARMT_ITEM_CLAW)
    {
        return SERVO_ID_CLAW;
    }
    if (armTestItem == ARMT_ITEM_GRIP)
    {
        return SERVO_ID_GRIP;
    }
    return 0U;
}

/**
 * @brief  执行当前执行器的"主动作"（KEY3 短按）
 * @param  无
 * @retval 无
 * @note   TURN=向 A 方向点动；LIFT=上升点动；CLAW=伸出；GRIP=抓紧。
 */
static void ArmTest_DoMainAction(void)
{
    switch (armTestItem)
    {
        case ARMT_ITEM_TURN:
            Arm_TurnTest(1U);
            break;

        case ARMT_ITEM_LIFT:
            Arm_LiftTest(1U);
            break;

        case ARMT_ITEM_CLAW:
            Arm_ServoMove(SERVO_ID_CLAW, (uint16_t)SERVO_CLAW_OUT_POS);
            break;

        default:   /* ARMT_ITEM_GRIP */
            Arm_ServoMove(SERVO_ID_GRIP, (uint16_t)SERVO_GRIP_CLOSE_POS);
            break;
    }
}

/**
 * @brief  执行当前执行器的"副动作"（KEY4 短按）
 * @param  无
 * @retval 无
 * @note   TURN=向 B 方向点动；LIFT=下降点动；CLAW=后缩；GRIP=松开。
 */
static void ArmTest_DoSubAction(void)
{
    switch (armTestItem)
    {
        case ARMT_ITEM_TURN:
            Arm_TurnTest(0U);
            break;

        case ARMT_ITEM_LIFT:
            Arm_LiftTest(0U);
            break;

        case ARMT_ITEM_CLAW:
            Arm_ServoMove(SERVO_ID_CLAW, (uint16_t)SERVO_CLAW_IN_POS);
            break;

        default:   /* ARMT_ITEM_GRIP */
            Arm_ServoMove(SERVO_ID_GRIP, (uint16_t)SERVO_GRIP_OPEN_POS);
            break;
    }
}

/**
 * @brief  切换执行器（KEY1 上一个 / KEY2 下一个，循环）
 * @param  next: 1=下一个；0=上一个
 * @retval 无
 */
static void ArmTest_SwitchItem(uint8_t next)
{
    if (next != 0U)
    {
        armTestItem++;
        if (armTestItem >= (uint8_t)ARMT_ITEM_MAX)
        {
            armTestItem = 0U;
        }
    }
    else
    {
        if (armTestItem == 0U)
        {
            armTestItem = (uint8_t)(ARMT_ITEM_MAX - 1U);
        }
        else
        {
            armTestItem--;
        }
    }

    /* 切到电机项时立刻补一次位置查询，页面马上能看到当前坐标 */
    if (ArmTest_CurAddr() != 0U)
    {
        Arm_ReadPos(ArmTest_CurAddr());
    }

    Debug_Log("ARM", "test item=%u", (unsigned int)armTestItem);
    Display_ArmRefresh();
}

/* ===================== 公开接口 ===================== */

/**
 * @brief  进入机械臂测试页
 * @param  无
 * @retval 无
 * @note   默认选中 TURN 项，不自动执行动作；
 *         只在进页时发使能指令给电机 5/6（位置模式必须先使能）。
 *         舵机侧不发任何帧（本页不做舵机卸载，爪子保持最后状态；
 *         需要"卸载舵机、手动扳爪子"请用 `ARMPARA` 手臂参数页，那里会发
 *         调试板协议 `Cmd 0x14` 卸载力矩）。 */
void ARM_Test(void)
{
    armTestActive = 1U;
    armTestItem   = ARMT_ITEM_TURN;

    Display_SetPage(PAGE_ARM);

    /* ★【2026-10-15】本页是**手动点动**（Arm_TurnTest/Arm_LiftTest，不经动作库的
       `Act_SendStep`，因此不会自动清"后台精修"）—— 进页先把它取消，
       否则后台精修会把用户刚点动过的轴**拽回上一段任务动作的旧目标**。 */
    ArmAct_TrimCancelAll();

    /* 使能两个电机（只在这里发生，初始化阶段不发） */
    Arm_Enable(ARM_ADDR_TURN, 1U);
    Arm_Enable(ARM_ADDR_LIFT, 1U);

    /* 立刻查一次位置，页面第3行不用等一个查询周期 */
    Arm_ReadPos(ARM_ADDR_TURN);
    Arm_ReadPos(ARM_ADDR_LIFT);

    Display_ArmRefresh();

    Debug_Log("ARM", "test enter");
}

/**
 * @brief  退出机械臂测试页
 * @param  无
 * @retval 无
 * @note   两个电机急停；SRV 项状态复位 IDLE，光标保持 SRV。
 *         舵机侧无法软件急停：调试板上位机协议未提供"停止舵机"指令，
 *         而单次运动时间只有 SERVO_MOVE_TIME_MS（默认 800ms），
 *         正常不会长时间停不下来；要立刻断电请用硬件断电。 */
void ARM_Test_Stop(void)
{
    Arm_Stop(ARM_ADDR_TURN);
    Arm_Stop(ARM_ADDR_LIFT);

    armTestActive = 0U;

    /* 复位测试页 SRV 项状态（不复位光标，光标保持在 SRV） */
    testItems[TEST_ID_SRV].state = TEST_STATE_IDLE;

    Debug_Log("ARM", "test exit");
}

/**
 * @brief  机械臂测试页短按处理
 * @param  key: 按键 ID（KEY1~KEY5）
 * @retval 无
 */
void ArmTest_HandleKey(KeyID_t key)
{
    if (key == KEY_NONE)
    {
        return;
    }

    switch (key)
    {
        case KEY1:   /* 上一个执行器 */
            ArmTest_SwitchItem(0U);
            break;

        case KEY2:   /* 下一个执行器 */
            ArmTest_SwitchItem(1U);
            break;

        case KEY3:   /* 主动作 */
            ArmTest_DoMainAction();
            Display_ArmRefresh();
            break;

        case KEY4:   /* 副动作（注意：本页 KEY4 不是退出键） */
            ArmTest_DoSubAction();
            Display_ArmRefresh();
            break;

        case KEY5:   /* 停止两个电机 + 退出回测试页 */
            ARM_Test_Stop();
            Display_SetPage(PAGE_TEST);
            break;

        default:
            break;
    }
}

/**
 * @brief  机械臂测试页长按处理
 * @param  longKey: 长按事件对应的按键 ID
 * @retval 无
 * @note   KEY3 长按（仅 TURN）：把电机5 当前位置设为单圈零点；
 *         KEY5 长按：当前执行器回零（TURN=单圈回零 / LIFT=碰撞回零 / 舵机忽略）。
 */
void ArmTest_HandleLongKey(KeyID_t longKey)
{
    if (armTestActive == 0U)
    {
        return;
    }

    if (longKey == KEY5)
    {
        /* 长按 KEY5 = "让本执行器归到 0 点"，两个电机的做法不同：
             电机5（TURN）：把**当前位置标定为**单圈零点（写电机 flash，掉电不忘）
             电机6（LIFT）：触发**碰撞回零**（丝杆没有绝对零点，只能靠撞停标定）
             舵机项没有归零概念，忽略 */
        if (armTestItem == ARMT_ITEM_TURN)
        {
            Arm_SetZero(ARM_ADDR_TURN);
            Display_ArmRefresh();
        }
        else if (armTestItem == ARMT_ITEM_LIFT)
        {
            Arm_LiftHome();
            Display_ArmRefresh();
        }
        else
        {
            Debug_Log("ARM", "zero ignored (servo item=%u)", (unsigned int)armTestItem);
        }
    }
    else if (longKey == KEY3)
    {
        /* 长按 KEY3（仅 TURN）= 让电机5 按 ARM_TURN_HOME_MODE 回零。
           这是**备用入口**：零点丢失或被手扳偏时用；
           正常标定过的零点（存在电机 flash 里）每次开机不需要回零。 */
        if (armTestItem == ARMT_ITEM_TURN)
        {
            Arm_TurnHome();
            Display_ArmRefresh();
        }
        else
        {
            Debug_Log("ARM", "turn home ignored (item=%u)", (unsigned int)armTestItem);
        }
    }
    else
    {
        /* 其它长按（含 KEY1/KEY2/KEY4）本页不使用 */
    }
}

/**
 * @brief  本页主循环节拍（置脏标志刷新状态行）
 * @param  无
 * @retval 无
 * @note   只置脏；OLED 侧按 REFRESH_PERIOD_MS 节流，不会高频全量重绘。
 *         电机位置一直在变，需要持续刷新才能看到实时值。
 */
void ArmTest_Update(void)
{
    if (armTestActive != 0U)
    {
        Display_ArmRefresh();
    }
}

/**
 * @brief  查询机械臂测试页是否活动
 * @param  无
 * @retval 1=本页活动，0=非本页
 */
uint8_t ArmTest_IsActive(void)
{
    return armTestActive;
}

/**
 * @brief  取当前执行器文本（页面第2行）
 * @param  无
 * @retval 形如 "TURN 1/4 5=EXIT" 的静态缓冲指针
 * @note   末尾 `5=EXIT` 是**退出提示**：本页 KEY4 是副动作不是退出，
 *         退出一律按 KEY5 短按，避免误操作。
 */
const char *ArmTest_GetItemText(void)
{
    static char buf[20];
    static const char *names[ARMT_ITEM_MAX] = { "TURN", "LIFT", "CLAW", "GRIP" };

    snprintf(buf, sizeof(buf), "%s %u/%u 5=EXIT",
             names[armTestItem],
             (unsigned int)(armTestItem + 1U), (unsigned int)ARMT_ITEM_MAX);

    return buf;
}

/**
 * @brief  取当前执行器状态文本（页面第3行）
 * @param  无
 * @retval 电机类 "p=200 H1 R0"：p=当前位置(下发脉冲域，读不到显示 --)、
 *                            H=回零完成、R=本段运动到位
 *         舵机类 "pos=800"：pos=上次下发给该舵机的目标位置（0~1000，
 *                            本次上电还没发过显示 --）
 */
const char *ArmTest_GetStateText(void)
{
    static char buf[20];

    if (ArmTest_CurAddr() != 0U)
    {
        uint8_t addr = ArmTest_CurAddr();

        if (Arm_IsPosValid(addr) != 0U)
        {
            snprintf(buf, sizeof(buf), "p=%d H%u R%u",
                     (int)Arm_GetPos(addr),
                     (unsigned int)Arm_IsHomeDone(addr),
                     (unsigned int)Arm_IsReachDone(addr));
        }
        else
        {
            /* 还没收到过 0x36 位置应答：此时位置显示 0 会误导，显示 -- */
            snprintf(buf, sizeof(buf), "p=-- H%u R%u",
                     (unsigned int)Arm_IsHomeDone(addr),
                     (unsigned int)Arm_IsReachDone(addr));
        }
    }
    else
    {
        uint16_t p = Arm_GetServoLastPos(ArmTest_CurServoId());

        if (p == 0xFFFFU)
        {
            snprintf(buf, sizeof(buf), "pos=--");
        }
        else
        {
            snprintf(buf, sizeof(buf), "pos=%u", (unsigned int)p);
        }
    }

    return buf;
}

/**
 * @brief  取按键提示文本（页面第4行）
 * @param  无
 * @retval 静态字符串指针
 * @note   TURN 的 `K5L=0` = 长按 KEY5 把当前位置**标定为 0 点**（写 flash 保存）；
 *         LIFT 的 `K5LH` = 长按 KEY5 触发碰撞回零(Home)。
 *         长按 KEY3（TURN 手动回零）是备用入口，屏上调不出空间，写在《调参手册》里。
 */
const char *ArmTest_GetHintText(void)
{
    switch (armTestItem)
    {
        case ARMT_ITEM_TURN: return "K3=A K4=B K5L=0";
        case ARMT_ITEM_LIFT: return "K3=UP K4=DN K5LH";
        case ARMT_ITEM_CLAW: return "K3=OUT K4=IN";
        default:             return "K3=CLS K4=OPN";
    }
}
