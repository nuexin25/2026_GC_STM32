/**
 ******************************************************************************
 * @file    bsp_task_dbg.c
 * @brief   任务点调试页实现（P9）：6 个任务点的单点调试状态机（骨架）
 * @author  nx_car
 * @date    2026-10-10
 ******************************************************************************
 * @note   状态机在 TaskDbg_Update() 里按主循环节拍推进，**完全非阻塞**：
 *           - 每个任务点分三段：阶段1 发指令 -> 阶段2 等数据 + 对位 -> 阶段3 动作 + 收尾；
 *           - 每段用 tdbgStage 计数、tdbgStageTick 记起始时刻；
 *           - 超时（宏见 bsp_task_cfg.h 的转盘/打靶两节）-> 停机器 + 日志 + FAIL，不卡死；
 *           - 机械臂动作**全部留桩**：编号 0 打 `arm act not configured` 并跳过，
 *             非 0 打 `not implemented` 后放行（本次不实做动作）。
 *
 *         数据来源与动作全部复用现有模块，**不重复实现**：
 *           协议 bsp_rpi：RPi_ReqRcp/ReqRz/ReqCcg/ReqShot、RPi_GetRcp/Rz/Ccg/Shot、
 *                          RPi_FindColorPos；
 *           对位 bsp_align：Align_Start（转盘，数据源 $RCP）/ Align_StartShot（打靶区，数据源 $SHOT）
 *                           / IsBusy/IsDone/IsFailed/Abort/SetBudget；
 *           导航 bsp_mission：Mission_NavGoTo（回原点）；
 *           车载位记录 bsp_task：Task_SetCarSlotColor/GetCarSlotColor、
 *                          Task_ColorOfBatch/Task_PlaceOfBatch（任务码）。
 *
 *         硬约束：禁止 HAL_Delay；日志前缀 [TDBG]；日志禁 %f（全部整数打印）。
 ******************************************************************************
 */
#include "bsp_task_dbg.h"
#include "bsp_task.h"       /* Task_SetCarSlotColor / Task_GetCarSlotColor / 任务码查询；Task_Stop */
#include "bsp_motor.h"      /* Motor_Enable：启动时使能四轮（Motor_Update 由 main 调度） */
#include "bsp_rpi.h"        /* 树莓派协议 v2.0：$RCP/$RZ/$CCG/$SHOT + 便捷查询 */
#include "bsp_align.h"      /* 视觉闭环对位：Align_Start/IsBusy/IsDone/IsFailed/Abort */
#include "bsp_arm_action.h" /* 机械臂动作库：ArmAct_GotoTtReady（全局观察位）/ ArmAct_Run(ARM_OP_*,pos) */
#include "bsp_tt.h"         /* ★转盘取料序列（TRAY PICK **唯一实现**）：TT_Start/Update/IsDone/IsFailed/Abort */
#include "bsp_mission.h"    /* 单段导航：Mission_NavGoTo */
#include "bsp_oled_ui.h"    /* Display_SetPage / Display_MenuEnter / Display_TaskDbgRefresh */
#include "bsp_debug.h"      /* Debug_Log */
#include "bsp_imu.h"       /* IMU_StaticZeroTo：任务点静态置角（零漂治理） */
#include "bsp_chassis.h"   /* Chassis_StaticZeroYaw：置角时同步里程计朝向基准 */
#include <stdio.h>

/* ===================== 内部状态 ===================== */

/* 本页运行状态 */
typedef enum {
    TDBG_ST_IDLE = 0,   /* 空闲：可 KEY1/KEY2 选择、KEY5（或 KEY3）启动 */
    TDBG_ST_RUN,        /* 执行中：状态机推进中 */
    TDBG_ST_DONE,       /* 当前任务点执行完成 */
    TDBG_ST_FAIL        /* 超时 / 中止 / 失败 */
} TdbgSt_t;

static uint8_t        tdbgActive = 0U;                        /* 本页活动标志 */
static TaskDbgItem_t  tdbgItem   = TDBG_TRAY_PICK;            /* 当前选中任务点 */
static TdbgSt_t       tdbgState  = TDBG_ST_IDLE;              /* 运行状态 */
static uint8_t        tdbgStage  = 0U;                        /* 当前阶段序号 */
static uint32_t       tdbgStageTick = 0U;                     /* 当前阶段起始时刻(ms) */
static uint8_t        tdbgTrayStarted = 0U;                   /* ★TRAY PICK：TT_ 转盘取料序列是否已启动 */
static uint8_t        tdbgShotStarted = 0U;                   /* ★STORE/ROUGH PLACE：SP_ 打靶放置序列是否已启动 */
static uint8_t        tdbgBackStarted = 0U;                   /* ★STORE BACK：SB_ 打靶取回序列是否已启动 */
static char           tdbgMsg[20]   = "";                     /* 第3行：状态 + 阶段 */
static char           tdbgData[20]  = "";                     /* 第4行：关键数据 */

/* 任务点名（下标 = TaskDbgItem_t 值；TDBG_NONE 位不用） */
static const char *const tdbgItemName[TDBG_COUNT] = {
    "NONE",          /* TDBG_NONE（占位） */
    "TRAY PICK",     /* TDBG_TRAY_PICK */
    "STORE PLACE",   /* TDBG_STORE_PLACE */
    "STORE BACK",    /* TDBG_STORE_BACK */
    "ROUGH PLACE",   /* TDBG_ROUGH_PLACE */
    "ROUGH STACK",   /* TDBG_ROUGH_STACK */
    "HOME"           /* TDBG_HOME */
};

/* ===================== 内部辅助 ===================== */

/**
 * @brief  进入指定阶段并重置阶段计时
 * @param  st: 阶段序号
 * @retval 无
 */
static void Tdbg_SetStage(uint8_t st)
{
    tdbgStage     = st;
    tdbgStageTick = HAL_GetTick();
}

/**
 * @brief  取当前阶段已耗时
 * @retval 毫秒
 */
static uint32_t Tdbg_Elapsed(void)
{
    return HAL_GetTick() - tdbgStageTick;
}

/**
 * @brief  机械臂动作留桩：编号 0 = 未配置；否则 = 未实现；都只打日志后放行
 * @param  act:  动作编号宏（0 表示未配置）
 * @param  what: 动作名（日志用）
 * @retval 无
 * @note   **绝不阻塞、绝不卡死**：目的是让协议与调试入口先能独立跑通。
 */
static void Tdbg_ArmAct(uint8_t act, const char *what)
{
    if (act == 0U)
    {
        Debug_Log("TDBG", "arm act not configured (%s)", what);
    }
    else
    {
        Debug_Log("TDBG", "arm act %u not implemented (%s)", (unsigned)act, what);
    }
}

/**
 * @brief  置失败：停机器（对位中止 + 四轮急停）+ 日志 + 状态 FAIL（不卡死）
 * @param  why: 简短原因（会进日志与第3行）
 * @retval 无
 */
static void Tdbg_Fail(const char *why)
{
    Align_Abort();     /* 若在对位，先停对位 */
    TT_Abort();        /* 若 TRAY PICK 在跑（bsp_tt 序列），一并中止 */
    SP_Abort();        /* 若 STORE/ROUGH PLACE 在跑（bsp_tt 序列），一并中止 */
    SB_Abort();        /* 若 STORE BACK 在跑（bsp_tt 序列），一并中止 */
    Task_Stop();       /* 四轮急停 + 失能（bsp_task 的通用停车） */
    tdbgState = TDBG_ST_FAIL;
    (void)snprintf(tdbgMsg, sizeof(tdbgMsg), "FAIL %.11s", why);
    Debug_Log("TDBG", "FAIL %s", why);
    Display_TaskDbgRefresh();
}

/**
 * @brief  置完成
 * @param  what: 简短描述（进日志）
 * @retval 无
 */
static void Tdbg_Done(const char *what)
{
    tdbgState = TDBG_ST_DONE;
    (void)snprintf(tdbgMsg, sizeof(tdbgMsg), "DONE");
    Debug_Log("TDBG", "DONE %s", what);
    Display_TaskDbgRefresh();
}

/**
 * @brief  启动当前任务点（复位内部阶段，幂等可反复执行）
 * @param  无
 * @retval 无
 */
static void Tdbg_Start(void)
{
    uint8_t addr;

    Align_Abort();             /* 清掉可能残留的对位 */

    /* 使能四轮（幂等）：本页是"单点调试"入口，之前没跑过导航时四轮还是失能的，
       而对位（速度模式）与放料都要动轮子 —— 不先使能会表现为"命令发了、轮子不动"。
       中止/失败/退出（运行态）由 Task_Stop() 急停 + 失能。 */
    for (addr = MOTOR_ADDR_MIN; addr <= MOTOR_ADDR_MAX; addr++)
    {
        Motor_Enable(addr, 1U);
    }
    Debug_Log("TDBG", "wheels enabled");

    TT_Abort();                /* ★清掉可能残留的转盘取料序列（TRAY PICK 由 bsp_tt 实现） */
    SP_Abort();                /* ★清掉可能残留的打靶放置序列（STORE/ROUGH PLACE 由 bsp_tt 实现） */
    SB_Abort();                /* ★清掉可能残留的打靶取回序列（STORE BACK 由 bsp_tt 实现） */

    /* ★任务点陀螺仪静态置角（与赛题页同口径）：
       - TRAY PICK / ROUGH PLACE / ROUGH STACK：对位成功后置 0 / 90 / 90；
       - STORE BACK：本段开始执行时立即置 180。 */
    Align_ClearDoneYawRef();
    switch (tdbgItem)
    {
        case TDBG_TRAY_PICK:
            Align_SetDoneYawRef(0.0f);
            break;
        case TDBG_ROUGH_PLACE:
        case TDBG_ROUGH_STACK:
            Align_SetDoneYawRef(90.0f);
            break;
        case TDBG_STORE_BACK:
            (void)Chassis_StaticZeroYaw(180.0f);
            break;
        default:
            break;
    }

    /* ★【2026-10-11 用户要求】ROUGH PLACE：放完 3 件后手臂回车载 3 号位、不抓
       （STORE PLACE 不做这段 —— 它要"只松抓"以衔接 STORE BACK） */
    SP_SetRoughMode((tdbgItem == TDBG_ROUGH_PLACE) ? 1U : 0U);

    tdbgState     = TDBG_ST_RUN;
    tdbgStage     = 0U;
    tdbgStageTick = HAL_GetTick();
    tdbgTrayStarted = 0U;
    tdbgShotStarted = 0U;
    tdbgBackStarted = 0U;
    tdbgMsg[0]    = '\0';
    tdbgData[0]   = '\0';
    (void)snprintf(tdbgMsg, sizeof(tdbgMsg), "RUN start");
    Debug_Log("TDBG", "run item=%u(%s)", (unsigned)tdbgItem, tdbgItemName[tdbgItem]);
    Display_TaskDbgRefresh();
}

/**
 * @brief  切换选中任务点（仅非运行态调用）
 * @param  dir: +1=下一项，-1=上一项
 * @retval 无
 */
static void Tdbg_Select(int8_t dir)
{
    int v = (int)tdbgItem + (int)dir;

    if (v < (int)TDBG_TRAY_PICK)
    {
        v = (int)TDBG_HOME;
    }
    if (v > (int)TDBG_HOME)
    {
        v = (int)TDBG_TRAY_PICK;
    }

    tdbgItem  = (TaskDbgItem_t)v;
    tdbgState = TDBG_ST_IDLE;
    tdbgStage = 0U;
    tdbgMsg[0]  = '\0';
    tdbgData[0] = '\0';
    Debug_Log("TDBG", "sel %u/%u %s", (unsigned)tdbgItem,
              (unsigned)(TDBG_COUNT - 1U), tdbgItemName[tdbgItem]);
    Display_TaskDbgRefresh();
}

/**
 * @brief  转盘取料（TDBG_TRAY_PICK）：直接驱动 `bsp_tt` 的转盘取料序列（**唯一实现**）
 * @param  无
 * @retval 无
 * @note   ★【2026-10-10 统一实现】本页不再自带一套流程，改为调用 `bsp_tt` 的
 *         `TT_Start()/TT_IsDone()/TT_IsFailed()/TT_Abort()`。`bsp_tt` 内部就是现场调好的
 *         那一套：**先进观察位** -> `$RCP` -> 对位 -> （周期）`$RZ` -> 准备位 ->
 *         （周期）`$CCG` -> 抓住 + 颜色硬校验 -> 放车载 1/2/3 -> 连抓 3 件。
 *         赛题流程 `PICK_TRAY_ALL()`（bsp_task.c）调的也是它 —— 两个入口共用同一份代码，
 *         不会再出现两套流程不一致。
 *         `TT_Update()` 由 main 主循环无条件调度，本页只需"启动 + 显示 + 判结束"。
 */
static void Tdbg_UpdateTrayPick(void)
{
    /* ---- ① 首次进入：启动序列 ---- */
    if (tdbgTrayStarted == 0U)
    {
        (void)snprintf(tdbgMsg, sizeof(tdbgMsg), "RUN tt start");
        if (TT_Start() != 0U)
        {
            tdbgTrayStarted = 1U;
            return;
        }
        /* 启动被拒（任务码未扫到 / 对位或机械臂正忙）：下一拍重试，超时兜底 */
        if (Tdbg_Elapsed() >= (uint32_t)TRAY_ARM_TIMEOUT_MS)
        {
            Tdbg_Fail("tt start");
        }
        return;
    }

    /* ---- ② 显示进度：第3行 = TT 状态，第4行 = 车载位↔颜色绑定 ---- */
    (void)snprintf(tdbgMsg, sizeof(tdbgMsg), "RUN %.15s", TT_GetStateText());
    (void)snprintf(tdbgData, sizeof(tdbgData), "%.19s", TT_GetMapText());

    /* ---- ③ 等序列结束（推进在 main 的 TT_Update）---- */
    if (TT_IsDone() != 0U)
    {
        tdbgTrayStarted = 0U;
        Tdbg_Done("tray 3/3");
    }
    else if (TT_IsFailed() != 0U)
    {
        tdbgTrayStarted = 0U;
        Tdbg_Fail("tray fail");
    }
}

/**
 * @brief  打靶类放置（TDBG_STORE_PLACE / TDBG_ROUGH_PLACE）：直接驱动 `bsp_tt` 的 SP_ 序列
 * @param  无
 * @retval 无
 * @note   ★【2026-10-10 统一实现】本页不再自带一套流程，改为调用 `bsp_tt` 的
 *         `SP_Start()/SP_IsDone()/SP_IsFailed()/SP_Abort()`。`SP_` 内部就是现场调好的那一套：
 *           **臂进观察位** -> `$SHOT` -> 等有效 -> 精确对位（严格档 1px、**不限时**）
 *           -> **车载位 1→2→3**：读该位绑定色 -> 任务码第 1 组查靶位 ->
 *              `ArmAct_Run(ARM_OP_CAR_PICK, slot)` 取件 -> `ArmAct_Run(ARM_OP_TGT_PLACE, tgt)` 放靶位。
 *         赛题流程 `T_B1_PLACE` 的 `PLACE_STORE_ALL()`（bsp_task.c）调的也是它 —— 共用一份代码。
 *         ⚠ 前提：**先跑过 TRAY PICK**（颜色↔车载位号绑定），否则会 `FAIL no binding`。
 *         `SP_Update()` 由 main 主循环无条件调度，本页只需"启动 + 显示 + 判结束"。
 */
static void Tdbg_UpdateShotPlace(void)
{
    /* ---- ① 首次进入：启动序列 ---- */
    if (tdbgShotStarted == 0U)
    {
        (void)snprintf(tdbgMsg, sizeof(tdbgMsg), "RUN sp start");
        if (SP_Start() != 0U)
        {
            tdbgShotStarted = 1U;
            return;
        }
        /* 启动被拒（对位或机械臂正忙）：下一拍重试，超时兜底 */
        if (Tdbg_Elapsed() >= (uint32_t)TRAY_ARM_TIMEOUT_MS)
        {
            Tdbg_Fail("sp start");
        }
        return;
    }

    /* ---- ② 显示进度：第3行 = SP 状态，第4行 = 车载位/颜色/靶位 ---- */
    (void)snprintf(tdbgMsg, sizeof(tdbgMsg), "RUN %.15s", SP_GetStateText());
    (void)snprintf(tdbgData, sizeof(tdbgData), "%.19s", SP_GetDataText());

    /* ---- ③ 等序列结束（推进在 main 的 SP_Update）---- */
    if (SP_IsDone() != 0U)
    {
        tdbgShotStarted = 0U;
        Tdbg_Done("place 3/3");
    }
    else if (SP_IsFailed() != 0U)
    {
        tdbgShotStarted = 0U;
        Tdbg_Fail("place fail");
    }
}

/**
 * @brief  第二批粗加工区堆叠（TDBG_ROUGH_STACK）三段式状态机
 * @param  无
 * @retval 无
 * @note   ★【2026-10-21 用户要求】**对位方式与 ROUGH PLACE 完全一致**：
 *         发 `$SHOT` -> 等 `RPi_GetShot()->valid` -> 对位（`Align_StartShot`，严格档 1px、不限时）
 *         -> 按任务码第4组靶位号（place2）堆叠 3 件。
 *         （原实现是 `$SHOT2` + `Align_StartShot2`，已按要求同步为 `$SHOT`。） */
static void Tdbg_UpdateRoughStack(void)
{
    switch (tdbgStage)
    {
        case 0:
            Debug_Log("TDBG", "stack: tx $SHOT");
            RPi_ReqShot();                       /* ★与 ROUGH PLACE 同一数据源 */
            Tdbg_SetStage(1U);
            break;

        case 1:
            if (RPi_GetShot()->valid != 0U)     /* 等的是 $SHOT 那一份数据 */
            {
                Debug_Log("TDBG", "stack: SHOT ok dx=%d dy=%d q=%d",
                          (int)RPi_GetShot()->dxPx, (int)RPi_GetShot()->dyPx,
                          (int)RPi_GetShot()->q[0]);
                Tdbg_SetStage(2U);
            }
            else if (Tdbg_Elapsed() >= (uint32_t)SHOT_TIMEOUT_MS)
            {
                Tdbg_Fail("shot to");
            }
            else
            {
                /* 继续等 */
            }
            break;

        case 2:   /* 阶段2：对位（数据源 = $SHOT -> 严格档；**不限时**，必须磨到 (0,0)） */
            Align_SetBudget(0U);                 /* 0 = 不限时 */
            if (Align_StartShot() != 0U)
            {
                Debug_Log("TDBG", "stack: align start (SHOT, no time limit)");
                Tdbg_SetStage(3U);
            }
            else
            {
                /* $SHOT 帧失效就重新请求一帧，下一拍再试（不是失败） */
                RPi_ReqShot();
            }
            break;

        case 3:   /* 等对位结束：进 1px 死区才放行；只有视觉断流才会 FAIL（同 STORE PLACE） */
            if (Align_IsBusy() == 0U)
            {
                if (Align_IsDone() != 0U)
                {
                    Debug_Log("TDBG", "stack: aligned");
                    Tdbg_SetStage(4U);
                }
                else
                {
                    Tdbg_Fail("align fail");
                }
            }
            break;

        case 4:   /* 按任务码第3组颜色 + 第4组靶位号堆叠 3 件（留桩） */
        {
            uint8_t i;
            for (i = 0U; i < 3U; i++)
            {
                uint8_t c   = Task_ColorOfBatch(2U, i);
                uint8_t t   = Task_PlaceOfBatch(2U, i);
                uint8_t act = (t == 1U) ? SHOT_ARM_STACK_ACT1 :
                              (t == 2U) ? SHOT_ARM_STACK_ACT2 :
                                          SHOT_ARM_STACK_ACT3;

                Debug_Log("TDBG", "stack: color %u -> tgt %u", (unsigned)c, (unsigned)t);
                Tdbg_ArmAct(act, "shot stack");
                (void)snprintf(tdbgData, sizeof(tdbgData), "c%u t%u",
                               (unsigned)c, (unsigned)t);
            }
            Tdbg_Done("rough stack");
            break;
        }

        default:
            Tdbg_Done("rough stack");
            break;
    }
}

/**
 * @brief  打靶取回（TDBG_STORE_BACK）：直接驱动 `bsp_tt` 的 SB_ 序列
 * @param  无
 * @retval 无
 * @note   ★【2026-10-10 实做】本页不再自带流程，改为调用 `bsp_tt` 的
 *         `SB_Start()/SB_IsDone()/SB_IsFailed()/SB_Abort()`。`SB_` 内部：
 *           **按 STORE PLACE 的逆序** —— 车载位 **3→2→1**：读该位绑定色 ->
 *           任务码第 1 组**反查靶位** -> `ArmAct_SbPick(tgt,..)` 从靶位取件
 *           -> `ArmAct_SbPlace(slot)` 放回车载该号位。**无需视觉**。
 *         赛题流程 `T_B1_PBACK` 的 `PLACE_BACK_ALL()`（bsp_task.c）调的也是它 —— 共用一份代码。
 *         ⚠ 前提：先跑过 TRAY PICK + STORE PLACE，否则会 `FAIL no binding`。
 *         ★【2026-10-10 晚 用户要求】**第一件"直接从抓取开始"** —— 不要"先缩回、再抬升"
 *           （现场在调试页跑 STORE BACK 时看到的就是这两步，它们来自 `flowSbPick` 的第 1/2 步）。
 *           ⇒ 本页也传 **`SB_Start(1)`**（= `chained`），第一件改走 `flowSbPickFirst`
 *           （只有 **夹紧 -> 抬到安全高 -> 缩回** 共 3 步）。
 *         ⚠ **前提：跑本任务点之前先跑过 `STORE PLACE`**，臂要停在"最后一次放置的那个靶位姿态"
 *           （S1 伸在靶位、M6 在取件高度、S2 已松开）—— 这正是 STORE BACK 的真实使用姿势。
 *           若臂不在该姿态（冷启动直接跑 STORE BACK），把这里的 **1 改回 0**（走完整流程，
 *           会先"缩回 + 抬到安全高"再转向靶位，按臂的姿态自适应）。
 *         `SB_Update()` 由 main 主循环无条件调度，本页只需"启动 + 显示 + 判结束"。
 */
static void Tdbg_UpdateStoreBack(void)
{
    /* ---- ① 首次进入：启动序列 ---- */
    if (tdbgBackStarted == 0U)
    {
        (void)snprintf(tdbgMsg, sizeof(tdbgMsg), "RUN sb start");
        if (SB_Start(1U) != 0U)
        {
            tdbgBackStarted = 1U;
            return;
        }
        /* 启动被拒（机械臂正忙）：下一拍重试，超时兜底 */
        if (Tdbg_Elapsed() >= (uint32_t)TRAY_ARM_TIMEOUT_MS)
        {
            Tdbg_Fail("sb start");
        }
        return;
    }

    /* ---- ② 显示进度：第3行 = SB 状态，第4行 = 车载位/颜色/靶位 ---- */
    (void)snprintf(tdbgMsg, sizeof(tdbgMsg), "RUN %.15s", SB_GetStateText());
    (void)snprintf(tdbgData, sizeof(tdbgData), "%.19s", SB_GetDataText());

    /* ---- ③ 等序列结束（推进在 main 的 SB_Update）---- */
    if (SB_IsDone() != 0U)
    {
        tdbgBackStarted = 0U;
        Tdbg_Done("back 3/3");
    }
    else if (SB_IsFailed() != 0U)
    {
        tdbgBackStarted = 0U;
        Tdbg_Fail("back fail");
    }
}

/**
 * @brief  回启停区（TDBG_HOME）状态机
 * @param  无
 * @retval 无
 * @note   调 Mission_NavGoTo(0,0) 回原点；超时用 TASK_MOVE_TIMEOUT_MS 兜底。
 */
static void Tdbg_UpdateHome(void)
{
    switch (tdbgStage)
    {
        case 0:
            Debug_Log("TDBG", "home: nav goto 0,0");
            (void)snprintf(tdbgMsg, sizeof(tdbgMsg), "RUN home");
            Tdbg_SetStage(1U);
            break;

        case 1:
            if (Mission_NavGoTo(0, 0) != 0U)
            {
                Tdbg_Done("home");
            }
            else if (Tdbg_Elapsed() >= (uint32_t)TASK_MOVE_TIMEOUT_MS)
            {
                Tdbg_Fail("home to");
            }
            else
            {
                /* 导航中 */
            }
            break;

        default:
            Tdbg_Done("home");
            break;
    }
}

/* ===================== 对外接口 ===================== */

/**
 * @brief  任务调试模块初始化（上电调一次）
 * @param  无
 * @retval 无
 * @note   只复位状态，不发电机/机械臂命令。
 */
void TaskDbg_Init(void)
{
    tdbgActive    = 0U;
    tdbgItem      = TDBG_TRAY_PICK;
    tdbgState     = TDBG_ST_IDLE;
    tdbgStage     = 0U;
    tdbgTrayStarted = 0U;
    tdbgShotStarted = 0U;
    tdbgBackStarted = 0U;
    tdbgMsg[0]    = '\0';
    tdbgData[0]   = '\0';
}

/**
 * @brief  进入任务调试页（主菜单 TDBG 项调用）
 * @param  无
 * @retval 无
 */
void TaskDbg_Enter(void)
{
    Align_Abort();             /* 清掉可能残留的对位 */
    tdbgActive    = 1U;
    tdbgState     = TDBG_ST_IDLE;
    tdbgStage     = 0U;
    tdbgTrayStarted = 0U;
    tdbgShotStarted = 0U;
    tdbgBackStarted = 0U;
    tdbgMsg[0]    = '\0';
    tdbgData[0]   = '\0';

    Display_SetPage(PAGE_TASK_DBG);
    Debug_Log("TDBG", "enter, item=%u(%s)", (unsigned)tdbgItem, tdbgItemName[tdbgItem]);
    Display_TaskDbgRefresh();
}

/**
 * @brief  退出任务调试页（KEY4/KEY5 调用）
 * @param  无
 * @retval 无
 * @note   运行中先中止（对位 Abort + 四轮急停），再回主菜单。
 */
void TaskDbg_Exit(void)
{
    if (tdbgState == TDBG_ST_RUN)
    {
        Align_Abort();
        TT_Abort();            /* TRAY PICK 由 bsp_tt 驱动：退出时必须一并中止 */
        SP_Abort();            /* STORE/ROUGH PLACE 也由 bsp_tt 驱动 */
        SB_Abort();            /* STORE BACK 也由 bsp_tt 驱动 */
        Task_Stop();
        Debug_Log("TDBG", "abort on exit");
    }
    tdbgState  = TDBG_ST_IDLE;
    tdbgActive = 0U;
    Debug_Log("TDBG", "exit");
    Display_MenuEnter();
}

/**
 * @brief  本页主循环节拍（非阻塞）
 * @param  无
 * @retval 无
 */
void TaskDbg_Update(void)
{
    if (tdbgActive == 0U)
    {
        return;
    }
    if (tdbgState != TDBG_ST_RUN)
    {
        return;
    }

    switch (tdbgItem)
    {
        case TDBG_TRAY_PICK:
            Tdbg_UpdateTrayPick();
            break;

        case TDBG_STORE_PLACE:
        case TDBG_ROUGH_PLACE:
            Tdbg_UpdateShotPlace();
            break;

        case TDBG_ROUGH_STACK:
            Tdbg_UpdateRoughStack();
            break;

        case TDBG_STORE_BACK:
            Tdbg_UpdateStoreBack();
            break;

        case TDBG_HOME:
            Tdbg_UpdateHome();
            break;

        default:
            Tdbg_Done("none");
            break;
    }

    Display_TaskDbgRefresh();   /* 阶段变化时刷新（受 OLED 节流） */
}

/**
 * @brief  本页按键处理（短按）
 * @param  key: 按键 ID
 * @retval 无
 */
void TaskDbg_HandleKey(KeyID_t key)
{
    if (key == KEY_NONE)
    {
        return;
    }

    switch (key)
    {
        case KEY1:   /* 上一项（运行中忽略） */
            if (tdbgState != TDBG_ST_RUN)
            {
                Tdbg_Select(-1);
            }
            break;

        case KEY2:   /* 下一项（运行中忽略） */
            if (tdbgState != TDBG_ST_RUN)
            {
                Tdbg_Select(+1);
            }
            break;

        case KEY3:   /* 启动当前任务点；运行中 = 中止（与 KEY5 同义） */
        case KEY5:   /* 启动当前任务点；运行中 = 中止（现场主用键） */
            if (tdbgState == TDBG_ST_RUN)
            {
                Tdbg_Fail("abort");
            }
            else
            {
                Tdbg_Start();
            }
            break;

        case KEY4:   /* 退出回主菜单（运行中会先中止 + 停车） */
            TaskDbg_Exit();
            break;

        default:
            break;
    }
}

/**
 * @brief  查询任务调试页是否活动
 * @param  无
 * @retval 1=本页活动，0=非本页
 */
uint8_t TaskDbg_IsActive(void)
{
    return tdbgActive;
}

/**
 * @brief  取页面第 1 行文本：当前项序号 / 总数
 * @param  无
 * @retval 形如 "TASK DBG 2/6"
 */
const char *TaskDbg_GetTitleText(void)
{
    static char line[20];

    (void)snprintf(line, sizeof(line), "TASK DBG %u/%u",
                   (unsigned)tdbgItem, (unsigned)(TDBG_COUNT - 1U));
    return line;
}

/**
 * @brief  取页面第 2 行文本：当前任务点名
 * @param  无
 * @retval 形如 "TRAY PICK"
 */
const char *TaskDbg_GetNameText(void)
{
    return tdbgItemName[tdbgItem];
}

/**
 * @brief  取页面第 3 行文本：状态 + 阶段说明
 * @param  无
 * @retval 形如 "IDLE" / "RUN ccg Q2" / "DONE" / "FAIL ccg to"
 */
const char *TaskDbg_GetStateText(void)
{
    static const char *const stName[] = { "IDLE", "RUN", "DONE", "FAIL" };

    if (tdbgMsg[0] != '\0')
    {
        return tdbgMsg;      /* RUN/FAIL 期间带阶段说明；DONE 为 "DONE" */
    }
    return stName[(uint8_t)tdbgState];
}

/**
 * @brief  取页面第 4 行文本：关键数据
 * @param  无
 * @retval 形如 "c1 P2 Q3" / "c1->S3"；无数据 "---"
 */
const char *TaskDbg_GetDataText(void)
{
    return (tdbgData[0] != '\0') ? tdbgData : "---";
}
