/**
 ******************************************************************************
 * @file    bsp_arm_para.c
 * @brief   手臂参数页实现（P8 联调）：电机 5/6 失能 + 读数刷新
 * @author  nx_car
 * @date    2026-09-18
 ******************************************************************************
 * @note   读数是"相对电机上次回零点的绝对位置"（下发脉冲域）。
 *         要让读数可复现、能直接抄进动作表，**进本页前应先跑一次回原点**
 *         （用手臂调试页的 KEY3 重新回原点），否则读数与动作表基准点不一致。
 *
 *         硬约束：禁止 HAL_Delay；日志禁 %f（位置用整数打印）。
 ******************************************************************************
 */
#include "bsp_arm_para.h"
#include "bsp_arm.h"
#include "bsp_servo.h"        /* 舵机回读：Servo_GetPos 取实际位置 */
#include "bsp_oled_ui.h"
#include "bsp_debug.h"
#include "bsp_motor.h"      /* ★本页进入时使能四轮：Motor_Enable / MOTOR_ADDR_MIN..MAX */
#include <stdio.h>

/* 回读时要读的舵机 ID 列表（爪子伸缩 ID1、抓松 ID2、车载转盘 ID3） */
static const uint8_t armParaReadIds[3] = { SERVO_ID_CLAW, SERVO_ID_GRIP, SERVO_ID_TURNTABLE };

/* ===================== 内部状态 ===================== */

static uint8_t  armParaActive = 0U;    /* 1=本页活动 */
static uint32_t armParaTick   = 0U;    /* 上次读数刷新时刻（节流用） */

/* ===================== 内部工具 ===================== */

/**
 * @brief  拼一行电机位置文本
 * @param  addr: 电机地址 ARM_ADDR_TURN(5) / ARM_ADDR_LIFT(6)
 * @param  tag : 行首标签（"M5" / "M6"）
 * @param  buf : 输出缓冲
 * @param  len : 缓冲长度（字节）
 * @retval 无
 * @note   位置无效（还没收到过 0x36 应答）时显示 "--"，
 *         避免把"读不到"误显示成"真的在 0 位"；
 *         本行数值与手臂调试页 `ARM TEST` 的 `p=` **同源同口径**（都是 Arm_GetPos），
 *         两页显示的坐标应当一致。
 */
static void ArmPara_MakeMotorText(uint8_t addr, const char *tag, char *buf, uint16_t len)
{
    if (Arm_IsPosValid(addr) != 0U)
    {
        snprintf(buf, (size_t)len, "%s p=%d", tag, (int)Arm_GetPos(addr));
    }
    else
    {
        snprintf(buf, (size_t)len, "%s p=--", tag);
    }
}

/**
 * @brief  把某舵机的**真实回读位置**转成字符串
 * @param  id : 舵机 ID（SERVO_ID_CLAW / SERVO_ID_GRIP）
 * @param  buf: 输出缓冲
 * @param  len: 缓冲长度（字节）
 * @retval 无
 * @note   回读有效时输出数值 0~1000；无有效回读（未读到 / 超时 / 数据过期）
 *         输出 "--"，避免把"读不到"显示成"真的在某个位置"。
 */
static void ArmPara_ReadBackToStr(uint8_t id, char *buf, uint16_t len)
{
    uint16_t pos = 0U;

    if (Servo_GetPos(id, &pos) != 0U)
    {
        snprintf(buf, (size_t)len, "%u", (unsigned int)pos);
    }
    else
    {
        snprintf(buf, (size_t)len, "--");
    }
}

/**
 * @brief  取某舵机真实回读位置（整数形式，供日志用）
 * @param  id: 舵机 ID
 * @retval 0~1000；无有效回读返回 -1
 */
static int ArmPara_ReadBackInt(uint8_t id)
{
    uint16_t pos = 0U;

    return (Servo_GetPos(id, &pos) != 0U) ? (int)pos : -1;
}

/**
 * @brief  取某舵机"最后一次下发的目标位置"（整数形式，供日志用）
 * @param  id: 舵机 ID
 * @retval 0~1000；本次上电没下发过返回 -1
 * @note   与 ArmPara_ReadBackInt 配对打印，就是"指令 vs 实际"的对比。
 */
static int ArmPara_LastSentInt(uint8_t id)
{
    uint16_t v = Arm_GetServoLastPos(id);

    return (v == 0xFFFFU) ? -1 : (int)v;
}

/* ===================== 公开接口 ===================== */

/**
 * @brief  进入手臂参数页
 * @param  无
 * @retval 无
 * @note   电机 5/6 失能（可手动扳动）；舵机无软件失能（官方库无该指令）。
 *         ★【2026-10-10 晚 用户要求】**本页进入时把四个轮子使能**
 *         （不使能时车是"自由轮"，会被推动/滑动，看参数时机构姿态会漂）。
 *         四个轮子只"使能抱轴"，本页**不发任何速度指令**，车不会自己走。
 */
void ArmPara_Enter(void)
{
    /* ★【2026-10-10 晚 用户要求】**四轮使能**（抱轴，防止在参数页里车被推着滑） */
    {
        uint8_t addr;

        for (addr = (uint8_t)MOTOR_ADDR_MIN; addr <= (uint8_t)MOTOR_ADDR_MAX; addr++)
        {
            Motor_Enable(addr, 1U);
        }
        Debug_Log("PARA", "4 wheels enabled (hold)");
    }

    armParaActive = 1U;

    Display_SetPage(PAGE_ARM_PARA);

    /* 失能两个电机，方便手动扳动机构读位置（退出时也保持失能） */
    Arm_Enable(ARM_ADDR_LIFT, 0U);
    Arm_Enable(ARM_ADDR_TURN, 0U);

    /* 立刻插两条**电机**位置查询，页面不用等一个查询周期 */
    Arm_ReadPos(ARM_ADDR_TURN);
    Arm_ReadPos(ARM_ADDR_LIFT);

    /* 同时主动触发一次**舵机**位置回读，让 S1/S2/S3 尽快显示真实值 */
    Servo_ReadPos(3U, armParaReadIds);

    /* 卸载三个舵机（`CMD_MULT_SERVO_UNLOAD = 0x14`）：关掉保持力矩，
       本页本来就是"用手扳机构读位置"，舵机抱着力矩会扳不动。
       退出时（ArmPara_Exit）会按最后下发位置把它们重新锁住。
       ★【2026-10-23】先清掉"没动复查表"：否则旧的补发条目会在 250ms 后
       把舵机拽回旧目标，和手扳操作打架。 */
    Arm_ServoTrackClear();
    Servo_Unload(3U, armParaReadIds);

    armParaTick = HAL_GetTick();

    /* 电机位置与舵机回读都是异步应答，此刻还没回来（由 ArmPara_Update 周期刷新），
       这里打印"指令 vs 实际"的对比：r=真实回读值、c=最后下发值，-1 表示暂无 */
    Debug_Log("PARA", "enter, motors disabled, servos unloaded, s1 r=%d c=%d, s2 r=%d c=%d, s3 r=%d c=%d",
              ArmPara_ReadBackInt(SERVO_ID_CLAW), ArmPara_LastSentInt(SERVO_ID_CLAW),
              ArmPara_ReadBackInt(SERVO_ID_GRIP), ArmPara_LastSentInt(SERVO_ID_GRIP),
              ArmPara_ReadBackInt(SERVO_ID_TURNTABLE), ArmPara_LastSentInt(SERVO_ID_TURNTABLE));
}

/**
 * @brief  退出手臂参数页
 * @param  无
 * @retval 无
 * @note   电机保持失能；需要重新抱轴请进手臂调试页。
 */
void ArmPara_Exit(void)
{
    uint8_t  i;
    uint8_t  id;
    uint16_t last;

    armParaActive = 0U;

    /* 退出时把两个舵机的力矩**恢复回来**（进页时卸载过，见 ArmPara_Enter）。
       协议里**没有**单独的"装载力矩"指令，所以做法是：按"最后下发位置"
       再发一条运动指令 —— 舵机收到运动指令就会重新出力并转到该位置。
       没下发过位置的舵机（last == 0xFFFF）保持卸载，只打日志说明
       （之后执行一次爪子动作就会重新出力）。 */
    for (i = 0U; i < (uint8_t)(sizeof(armParaReadIds)); i++)
    {
        id   = armParaReadIds[i];
        last = Arm_GetServoLastPos(id);

        if (last != 0xFFFFU)
        {
            Arm_ServoMove(id, last);
        }
        else
        {
            Debug_Log("PARA", "servo id=%u keeps unloaded (no target yet)",
                      (unsigned int)id);
        }
    }

    Debug_Log("PARA", "exit");

    Display_MenuEnter();
}

/**
 * @brief  本页主循环节拍（读数节流刷新）
 * @param  无
 * @retval 无
 */
void ArmPara_Update(void)
{
    uint32_t now;

    if (armParaActive == 0U)
    {
        return;
    }

    now = HAL_GetTick();
    if ((now - armParaTick) < (uint32_t)ARM_PARA_REFRESH_MS)
    {
        return;
    }
    armParaTick = now;

    /* 机构可能被手扳动，周期重发位置查询并刷新显示 */
    Arm_ReadPos(ARM_ADDR_TURN);
    Arm_ReadPos(ARM_ADDR_LIFT);
    Display_ArmParaRefresh();
}

/**
 * @brief  本页按键处理（短按）
 * @param  key: 按键 ID
 * @retval 无
 */
void ArmPara_HandleKey(KeyID_t key)
{
    if (key == KEY_NONE)
    {
        return;
    }

    switch (key)
    {
        case KEY3:   /* 立即刷新读数 */
            /* 注意：电机位置与舵机回读都是异步应答，这一拍显示/打印的是
               "上一次收到的值"，真正的更新要等应答回来（约 100~200ms 后由 Update 刷新）。
               日志里 r=真实回读值、c=最后下发值，两者对比可看出舵机有没有走到位 */
            Debug_Log("PARA", "refresh, m5=%d(v%u) m6=%d(v%u), s1 r=%d c=%d, s2 r=%d c=%d, s3 r=%d c=%d",
                      (int)Arm_GetPos(ARM_ADDR_TURN),
                      (unsigned int)Arm_IsPosValid(ARM_ADDR_TURN),
                      (int)Arm_GetPos(ARM_ADDR_LIFT),
                      (unsigned int)Arm_IsPosValid(ARM_ADDR_LIFT),
                      ArmPara_ReadBackInt(SERVO_ID_CLAW), ArmPara_LastSentInt(SERVO_ID_CLAW),
                      ArmPara_ReadBackInt(SERVO_ID_GRIP), ArmPara_LastSentInt(SERVO_ID_GRIP),
                      ArmPara_ReadBackInt(SERVO_ID_TURNTABLE), ArmPara_LastSentInt(SERVO_ID_TURNTABLE));

            Arm_ReadPos(ARM_ADDR_TURN);
            Arm_ReadPos(ARM_ADDR_LIFT);
            Servo_ReadPos(3U, armParaReadIds);   /* 顺手主动读一次舵机实际位置 */
            Display_ArmParaRefresh();
            break;

        case KEY4:   /* 退出回主菜单 */
            ArmPara_Exit();
            break;

        default:
            /* KEY1/KEY2 本页预留不用 */
            break;
    }
}

/**
 * @brief  本页长按处理（由 main.c 的长按分支派发）
 * @param  longKey: 长按事件对应的按键 ID
 * @retval 无
 * @note   **长按 KEY3 = 把电机5 当前位置设为"绝对 0 点"**：
 *         发送驱动器"单圈零点设置"命令 `addr 93 88 01 6B`
 *         （手动菜单 `O_Set → SetO`，svF=01 = **存进电机 flash、掉电不忘**），
 *         并由 Arm_SetZero 内部顺带把回零参数（含 potF）也存进电机。
 *
 *         放在本页的原因：本页电机5 已**失能**，可以**用手把手臂扳到期望角度**、
 *         对着 `M5 p=` 一边看一边标定，比在调试页用点动凑角度精确得多。
 *
 *         ⚠ 短按 KEY3 只是刷新读数，**长按 KEY3 会改零点** —— 别按错。
 */
void ArmPara_HandleLongKey(KeyID_t longKey)
{
    if (armParaActive == 0U)
    {
        return;
    }

    if (longKey == KEY3)
    {
        /* 设单圈零点（存电机 flash）：本页电机5 已失能，不影响该命令生效 */
        Arm_SetZero(ARM_ADDR_TURN);

        /* 立刻插一次位置查询，让页面尽快显示新基准（p 应变 0） */
        Arm_ReadPos(ARM_ADDR_TURN);
        Display_ArmParaRefresh();

        Debug_Log("PARA", "long KEY3 -> set m5 zero at cur pos (was p=%d)",
                  (int)Arm_GetPos(ARM_ADDR_TURN));
    }
    else
    {
        /* 其它长按键本页不使用 */
    }
}

/**
 * @brief  查询本页是否活动
 * @param  无
 * @retval 1=活动，0=非本页
 */
uint8_t ArmPara_IsActive(void)
{
    return armParaActive;
}

/**
 * @brief  取页面第2行文本（电机5 位置）
 * @param  无
 * @retval 静态缓冲指针
 */
const char *ArmPara_GetMotor5Text(void)
{
    static char buf[20];

    ArmPara_MakeMotorText(ARM_ADDR_TURN, "M5", buf, (uint16_t)sizeof(buf));

    return buf;
}

/**
 * @brief  取页面第3行文本（电机6 位置）
 * @param  无
 * @retval 静态缓冲指针
 */
const char *ArmPara_GetMotor6Text(void)
{
    static char buf[20];

    ArmPara_MakeMotorText(ARM_ADDR_LIFT, "M6", buf, (uint16_t)sizeof(buf));

    return buf;
}

/**
 * @brief  取页面第4行文本（两个舵机的**真实回读位置**）
 * @param  无
 * @retval 静态缓冲指针，形如 "S1=105 S2=990"；无有效回读显示 "--"
 */
const char *ArmPara_GetServoText(void)
{
    static char buf[24];
    char t1[8];
    char t2[8];

    ArmPara_ReadBackToStr(SERVO_ID_CLAW, t1, (uint16_t)sizeof(t1));
    ArmPara_ReadBackToStr(SERVO_ID_GRIP, t2, (uint16_t)sizeof(t2));

    snprintf(buf, sizeof(buf), "S1=%s S2=%s", t1, t2);

    return buf;
}

/**
 * @brief  取页面第1行文本（标题 + **舵机3 真实回读位置**）
 * @param  无
 * @retval 静态缓冲指针，形如 "ARM PARA S3=500"；无有效回读显示 "S3=--"
 * @note   ★【2026-10-21 硬件改动】车载 3 个位置改由**舵机3（车载转盘）**选位，
 *         第4行已被 S1/S2 占满（8x16 每行最多 16 字符），所以把 S3 放在标题行。
 */
const char *ArmPara_GetTitleText(void)
{
    static char buf[20];
    char t3[8];

    ArmPara_ReadBackToStr(SERVO_ID_TURNTABLE, t3, (uint16_t)sizeof(t3));

    snprintf(buf, sizeof(buf), "ARM PARA S3=%s", t3);

    return buf;
}
