/**
 ******************************************************************************
 * @file    bsp_tt.c
 * @brief   转盘取料序列实现（P10）：协议 v2.0 §6.1 请求/轮询式，连抓 3 件放车载 1/2/3
 * @author  nx_car
 * @date    2026-10-06
 ******************************************************************************
 * @note   状态机（全部非阻塞，主循环 TT_Update 推进）：
 *           TT_OBSERVE 手臂回【观察区对位】+ 舵盘选号位（ArmAct_TrayReady(slot)）
 *           -> TT_REQ_RCP / TT_WAIT_RCP 发 `$RCP` 等应答
 *           -> TT_RCP_SHIFT ★一直没数据（视觉没看到转盘）就向车身右侧平移
 *              TRAY_RCP_SHIFT_MM，然后重发 `$RCP` 重新识别（最多 TRAY_RCP_SHIFT_MAX 次）
 *           -> TT_ALIGN 车身对位（Align_Start，数据源 `$RCP`）
 *              ★★【2026-10-29 用户要求】"颜色顺序记忆 + 定时"在**对位阶段就开始**：
 *              TT_WAIT_RCP / TT_ALIGN 期间并行周期发 `$RZ`（`Tt_RzTrack`）：
 *                ① 同一份"位号↔颜色"映射连续两帧一致 -> 锁 `ttMapBase`（= 记住颜色顺序）；
 *                ② 再等到"完美循环移位"且连续两帧一致 -> 认 t0 并分配 3 件位号（= 开始计时）。
 *              对位成功时若 t0 已就绪 -> **直接进 TT_PREP 抓第 1 件**，不再等帧。
 *           -> TT_RZ_BASE 对位期间没锁成基准时，仍在此锁一份**基准映射**
 *           -> TT_RZ_EDGE 对位期间只锁到基准、没认到 t0 时，仍在此等换位边沿定 t0
 *           -> TT_PREP 手臂去本件目标位号：位号 = 基准位号 + 由 (t0 -> 现在) 时间推算的步数
 *              （★每个周期**前 TRAY_ROT_LEAD_MS（默认 2s）** 就地抓算出来的位号；
 *                之后提前去"算出来的下一个位号"并**停在那里等料转到位**再抓）。
 *              ★第 2/3 件**不再回观察位、不再对位**，S3 在本状态内预转。
 *           -> TT_GRAB 抓取+放车载一整套（ArmAct_TrayPickPlace(slot)）
 *           -> TT_PLACE 等这套跑完 + 记账 -> 未满 3 件直接回 TT_PREP；满 3 件 TT_DONE。
 *         ★【2026-10-10 顺序改为"先观察位"】观察位是 $RCP/$RZ 能拿到数据的前提，
 *           必须先摆臂再看；本文件是 **TRAY PICK 的唯一实现**，任务调试页（TDBG）
 *           与赛题流程（PICK_TRAY_ALL）都调它。
 *
 *         机械臂动作**不在这里定义**：全部直接调 bsp_arm_action 的接口
 *         （ArmAct_TrayReady / ArmAct_GotoTtPos / ArmAct_TrayPickPlace），
 *         TRAY PICK 的动作组与参数都在 bsp_arm_action.c 的 TRAY 专用流程里，
 *         **改其它任务点不会影响这里**。
 *
 *         v1.0 的"视觉持续推送颜色 + 转盘偏移"语义已删除，本文件不再读写 $COLOR/$TURNTABLE；
 *         ★2026-10-24 起也不再使用 `$CCG`（不再"等目标色转到准备位"），
 *         只通过 RPi_ReqRcp/ReqRz 请求并用 RPi_GetRcp/GetRz 取应答。
 *
 *         硬约束：禁止 HAL_Delay；日志禁 %f；全部非阻塞；每个等待都有超时宏兜底。
 ******************************************************************************
 */
#include "bsp_tt.h"
#include "bsp_align.h"        /* Align_Start/IsBusy/IsDone/IsFailed/Abort/SetBudget */
#include "bsp_arm_action.h"   /* ArmAct_GotoTtPos/TrayReady/TrayPickPlace/PreSelectSlot */
#include "bsp_rpi.h"          /* RPi_ReqRcp/ReqRz + RPi_GetRcp/GetRz */
#include "bsp_task.h"         /* Task_SetCarSlotColor/GetCarSlotColor、Task_ColorOfBatch、Task_GetCode */
#include "bsp_task_cfg.h"     /* TRAY_*：转盘周期/超时/右移参数（机械臂动作组在 bsp_arm_action.h） */
#include "bsp_mission.h"      /* Mission_NavGoTo：$RCP 无数据时"右移一小段"的载体 */
#include "bsp_chassis.h"      /* Chassis_GetOdomX/Y/YawTrue：按当前朝向算右移目标点 */
#include "bsp_mecanum.h"      /* MEC_RAD_PER_DEG */
#include "bsp_debug.h"
#include <math.h>
#include <stdio.h>

/* ===================== 内部状态 ===================== */

typedef enum {
    TT_IDLE = 0,      /* 没在跑 */
    TT_OBSERVE,       /* ① 手臂到【观察位】（ArmAct_TrayReady）—— **必须最先做**：
                          摄像头要先看到转盘/物料，后面的 $RCP / $RZ 才有意义 */
    TT_REQ_RCP,       /* ② 发 $RCP 请求转盘中心偏移 */
    TT_WAIT_RCP,      /* ③ 等 $RCP,dx,dy 有效 */
    TT_RCP_SHIFT,     /* ③' ★一直没看到转盘 -> 向车身右侧平移一小段 */
    TT_ALIGN,         /* ④ 车身对位（Align_Start，数据源 $RCP） */
    TT_RZ_BASE,       /* ⑤ ★周期发 $RZ，锁**基准映射**（连续两帧一致才认） */
    TT_RZ_EDGE,       /* ⑥ ★继续发 $RZ，等"完美循环移位"发生 -> 记 t0，抓第 1 件 */
    TT_PREP,          /* ⑦ 手臂去本件目标位号（第 2/3 件在这里按时间推算位置） */
    TT_GRAB,          /* ⑧ 抓取+放车载一整套（ArmAct_TrayPickPlace） */
    TT_PLACE,         /* ⑨ 等那套动作跑完 + 记账（TT_SLOT_FIRST 起的号位） */
    TT_DONE,          /* 3 件全部上车 */
    TT_FAILED         /* 失败中止 */
} TT_State_t;

static TT_State_t ttState     = TT_IDLE;         /* 当前状态 */
static uint8_t    ttColors[3] = { 0U, 0U, 0U };  /* 本轮 3 件的目标颜色顺序（1~4） */
static uint8_t    ttItem      = 0U;              /* 已抓件数 0~TT_ITEM_COUNT */
static uint8_t    ttTargetPos = 0U;              /* 本件**实际去抓**的转盘位号 1~3 */
static uint8_t    ttPickColor = 0U;              /* 本件记账用的颜色（= ttColors[ttItem]） */
static uint32_t   ttStateTick = 0U;              /* 进入本状态的时刻（超时计时） */
static uint32_t   ttLimitMs   = 0U;              /* 本状态超时上限（0=不超时） */
static uint32_t   ttRzTick    = 0U;              /* $RZ 上次发送时刻（单帧快照 -> 周期重发） */
static uint8_t    ttRzSent    = 0U;              /* $RZ 已发送次数 */
static uint8_t    ttAlignOn   = 0U;              /* 1=本次对位已启动（避免重复 Align_Start） */
static uint8_t    ttArmOn     = 0U;              /* 1=机械臂本段已启动（避免重复下发） */
static uint8_t    ttTailOnly  = 0U;              /* ★1=最后一件已到手且 S1 已回 0，
                                                    只剩"放回车载"这类多余动作 -> 可让底盘先动 */
static uint8_t    ttTailCand  = 0U;              /* ★1=最后一件已到手（"只剩收尾"候选），
                                                    还要在 TT_PLACE 每拍复核 S1 才置 ttTailOnly */
static uint8_t    ttShiftCnt  = 0U;              /* ★本件已"右移搜索"的次数 */
static uint8_t    ttShiftOn   = 0U;              /* ★1=本次右移段已下发（避免重复下发） */
static int32_t    ttShiftX    = 0;               /* ★本次右移的绝对目标 X（里程计坐标 mm） */
static int32_t    ttShiftY    = 0;               /* ★本次右移的绝对目标 Y（里程计坐标 mm） */

/* ---- ★【2026-10-24】"计时推算"用的映射与时间基准：ttMap[0] 恒为 0，只用 [1..3] ---- */
static uint8_t    ttMapBase[4] = { 0U, 0U, 0U, 0U }; /* 基准映射：位号->颜色（TT_RZ_BASE 锁定） */
static uint8_t    ttMapRef[4]  = { 0U, 0U, 0U, 0U }; /* t0 时刻的映射：3 件位号的推算基准 */
static uint8_t    ttMapCand[4] = { 0U, 0U, 0U, 0U }; /* "连续两帧一致"用的候选映射 */
static uint8_t    ttRefPos[3]  = { 0U, 0U, 0U };     /* 3 件目标色在 ttMapRef 里的位号（推算基准） */
static uint32_t   ttT0        = 0U;                  /* 转动计时起点 t0（变化首次出现时刻） */
static uint32_t   ttCandTick  = 0U;                  /* 候选映射首次出现时刻（用于定 t0） */
static uint32_t   ttRzRxTick  = 0U;                  /* 已处理过的 `$RZ` 帧时间戳（判"新帧"用） */
static uint32_t   ttWaitTick  = 0U;                  /* ★>0 = 已在"下一格"待命，等到本时刻（料转到位）再抓 */
static uint32_t   ttGrabHoldTick = 0U;               /* ★【2026-11-02】颜色确认后的"保持"起点
                                                        （0=还没开始保持；等满 TT_GRAB_HOLD_MS 才夹紧） */
static uint32_t   ttCcgTick   = 0U;                  /* ★等待位 `$CCG` 上次发送时刻（单帧快照 -> 周期重发） */
static uint8_t    ttCcgSent   = 0U;                  /* ★等待位 `$CCG` 已发送次数（0=还没发过，立即发） */
static uint32_t   ttCcgRxTick = 0U;                  /* ★本次等待位"首次请求前"的应答时间戳（判新应答用） */
static uint8_t    ttCcgArmed  = 0U;                  /* ★【2026-10-26】1=已到等待位、颜色等待已开启
                                                        （0=手臂还在行程中，不许发 `$CCG`/不许抓） */
static uint32_t   ttArmIdleTick = 0U;                /* ★【2026-10-26】手臂首次"空闲"的时刻（0=还在动），
                                                        静置 TT_PREP_ARRIVE_MS 后认定"到达等待位" */
/* ★【2026-10-29 用户要求】"颜色顺序记忆 + 定时"在**对位阶段**就开始做：
   对位期间并行跑 `$RZ` 跟踪，进度记在 ttRzPhase 里。 */
#define TT_RZ_WAIT   0U     /* 还没锁到"连续两帧一致"的基准映射 */
#define TT_RZ_BASED  1U     /* 基准映射已锁，继续等"完美循环移位" */
#define TT_RZ_T0     2U     /* 已认出换位边沿、t0 已定：对位一结束就能直接抓 */

static uint8_t    ttRzPhase     = TT_RZ_WAIT;        /* ★对位阶段 $RZ 跟踪进度 */
static uint8_t    ttRzOddCnt    = 0U;                /* ★TT_RZ_EDGE 里"连续既非基准、
                                                        也非循环移位"的帧数（判基准是否失效） */

/* ===================== 内部函数 ===================== */

/**
 * @brief  切状态（同时重置计时与超时上限）
 * @param  st     : 目标状态
 * @param  limitMs: 本状态超时上限 ms（0=不超时）
 * @retval 无
 */
static void Tt_GoTo(TT_State_t st, uint32_t limitMs)
{
    ttState     = st;
    ttStateTick = HAL_GetTick();
    ttLimitMs   = limitMs;
}

/**
 * @brief  本状态是否已超时
 * @retval 1=超时；0=未到（ttLimitMs=0 时恒不超时）
 */
static uint8_t Tt_Timeout(void)
{
    if (ttLimitMs == 0U)
    {
        return 0U;
    }
    return ((HAL_GetTick() - ttStateTick) >= ttLimitMs) ? 1U : 0U;
}

/**
 * @brief  失败中止（把对位/机械臂都停下来，报出原因）
 * @param  why: 简短原因（进日志）
 * @retval 无
 */
static void Tt_Fail(const char *why)
{
    Align_Abort();
    ArmAct_Abort();
    ttState = TT_FAILED;
    Debug_Log("TT", "FAIL: %s (item %u/%u, map %s)",
              why, (unsigned int)ttItem, (unsigned int)TT_ITEM_COUNT, TT_GetMapText());
}

/**
 * @brief  周期发 `$RZ`（单帧快照：只反映"请求那一瞬间"的识别结果，必须周期重发）
 * @param  无
 * @retval 无
 */
static void Tt_RzPoll(void)
{
    if ((ttRzSent == 0U) ||
        ((HAL_GetTick() - ttRzTick) >= (uint32_t)TRAY_RZ_POLL_MS))
    {
        ttRzTick = HAL_GetTick();
        ttRzSent++;
        RPi_ReqRz();
    }
}

/**
 * @brief  ★【2026-10-24 用户要求】"等待位有颜色反馈，根据颜色反馈来抓"
 * @param  pos  : 等待位号 1~3
 * @param  color: 本件目标色 1~4
 * @retval 1=相机确认该位号现在是目标色（可以抓）；0=还没确认
 * @note   `$CCG,<pos>` -> `$CCG,c` 只返回"颜色"，且是单帧快照，必须周期重发；
 *         判据 = 本次等待位**开新应答**（lastRxTick 变化）且 c 等于目标色。
 *         这样能挡掉"请求前残留的旧应答"和"料还没转到位时的旧颜色"。
 */
static uint8_t Tt_ColorReady(uint8_t pos, uint8_t color)
{
    const RPi_Ccg_t *ccg = RPi_GetCcg();

    if ((ttCcgSent == 0U) ||
        ((HAL_GetTick() - ttCcgTick) >= (uint32_t)TRAY_RZ_POLL_MS))
    {
        if (ttCcgSent == 0U)
        {
            ttCcgRxTick = ccg->lastRxTick;  /* 首次请求前的应答戳：之后必须变才算新应答 */
        }
        ttCcgTick = HAL_GetTick();
        ttCcgSent++;
        RPi_ReqCcg(pos);
    }

    if ((ccg->valid == 0U) || (ccg->reqPos != pos))
    {
        return 0U;
    }
    if (ccg->lastRxTick == ttCcgRxTick)
    {
        return 0U;                          /* 还没收到本次等待位的新应答 */
    }
    if (ccg->color != color)
    {
        return 0U;
    }

    Debug_Log("TT", "item%u wait P%u color C%u confirmed",
              (unsigned int)(ttItem + 1U), (unsigned int)pos, (unsigned int)color);
    return 1U;
}

/**
 * @brief  一帧 `$RZ` 是否"三个位号都认出来了"（做映射比较的前提）
 * @param  byPos: `RPi_GetRz()->byPos`（只用 [1..3]）
 * @retval 1=三位号全非 0；0=有漏识别
 */
static uint8_t Tt_MapReady(const uint8_t *byPos)
{
    if ((byPos[1] == 0U) || (byPos[2] == 0U) || (byPos[3] == 0U))
    {
        return 0U;
    }
    return 1U;
}

/**
 * @brief  两份"位号->颜色"映射是否完全一致（只用 [1..3]）
 * @param  a/b: 映射数组
 * @retval 1=一致；0=不同
 */
static uint8_t Tt_MapEqual(const uint8_t *a, const uint8_t *b)
{
    if ((a[1] != b[1]) || (a[2] != b[2]) || (a[3] != b[3]))
    {
        return 0U;
    }
    return 1U;
}

/**
 * @brief  位号沿转盘前进方向前进 steps 步（按 3 循环）
 * @param  pos  : 起始位号 1~3
 * @param  steps: 前进步数（0=原位）
 * @retval 目标位号 1~3；方向由 `TRAY_ROT_DIR` 决定
 */
static uint8_t Tt_ShiftPos(uint8_t pos, uint32_t steps)
{
    int32_t p = (int32_t)pos - 1 + ((int32_t)steps * (int32_t)TRAY_ROT_DIR);

    p = ((p % 3) + 3) % 3;
    return (uint8_t)(p + 1);
}

/**
 * @brief  cur 是否为 base 的**完美循环移位**（= 整盘整体转了一步或两步）
 * @param  base: 基准映射（位号->颜色）
 * @param  cur : 当前映射
 * @retval 1=是循环移位（转盘真的转了）；0=不是（识别错/还没转/过渡帧）
 * @note   逐位核对"base 的 p 号位颜色现在出现在 p+DIR*k 号位"，k=1/2 任一成立即认。
 *         这样能挡掉"某个位号识别错色"或"转到一半的混合帧"这类假变化。
 */
static uint8_t Tt_IsRotChanged(const uint8_t *base, const uint8_t *cur)
{
    uint8_t k;
    uint8_t p;
    uint8_t ok;

    for (k = 1U; k <= 2U; k++)
    {
        ok = 1U;
        for (p = 1U; p <= 3U; p++)
        {
            if (cur[Tt_ShiftPos(p, k)] != base[p])
            {
                ok = 0U;
                break;
            }
        }
        if (ok != 0U)
        {
            return 1U;
        }
    }
    return 0U;
}

/* 前置声明：`Tt_RzTrack` 内部要用（定义在它之后） */
static uint8_t Tt_AssignRef(const uint8_t *map);

/**
 * @brief  ★【2026-10-29 用户要求】对位阶段就"记住颜色顺序 + 开始计时"
 * @param  无
 * @retval 当前跟踪进度（TT_RZ_WAIT / TT_RZ_BASED / TT_RZ_T0）
 * @note   周期发 `$RZ`（单帧快照必须重发），分两步：
 *         ① 等**同一份"位号->颜色"映射连续两帧一致** -> 锁 `ttMapBase`（= 记住颜色顺序）；
 *         ② 继续等**完美循环移位**且连续两帧一致 -> 把这次变化**首次出现**的时刻记为
 *            `ttT0`、把 t0 那帧存进 `ttMapRef` 并分配好 3 件位号（= 开始计时）。
 *         对位一结束（`TT_ALIGN` done）若已到 TT_RZ_T0，就**直接进 TT_PREP 抓第 1 件**，
 *         不再走 TT_RZ_BASE / TT_RZ_EDGE 的等帧。第 2/3 件仍按 `Tt_PredictPos` 推算。
 */
static uint8_t Tt_RzTrack(void)
{
    const RPi_Rz_t *rz;

    Tt_RzPoll();

    rz = RPi_GetRz();
    if ((rz->valid == 0U) || (rz->lastRxTick == ttRzRxTick))
    {
        return ttRzPhase;                   /* 没新帧：等下一拍 */
    }
    ttRzRxTick = rz->lastRxTick;

    if (Tt_MapReady(rz->byPos) == 0U)
    {
        return ttRzPhase;                   /* 有漏识别：不参与比较 */
    }

    /* ---- ① 锁基准映射（"连续两帧一致"才认，排除正在转的混合帧）---- */
    if (ttRzPhase == TT_RZ_WAIT)
    {
        if (Tt_MapEqual(ttMapCand, rz->byPos) != 0U)
        {
            ttMapBase[1] = rz->byPos[1];
            ttMapBase[2] = rz->byPos[2];
            ttMapBase[3] = rz->byPos[3];
            ttRzPhase    = TT_RZ_BASED;
            ttRzOddCnt   = 0U;
            Debug_Log("TT", "base map locked (align): P1=C%u P2=C%u P3=C%u",
                      (unsigned int)ttMapBase[1], (unsigned int)ttMapBase[2],
                      (unsigned int)ttMapBase[3]);
        }
        else
        {
            ttMapCand[1] = rz->byPos[1];
            ttMapCand[2] = rz->byPos[2];
            ttMapCand[3] = rz->byPos[3];
        }
        return ttRzPhase;
    }

    /* ---- ② 等"完美循环移位" -> 记 t0（同样要连续两帧一致）---- */
    if (Tt_MapEqual(ttMapCand, rz->byPos) != 0U)
    {
        if (Tt_IsRotChanged(ttMapBase, rz->byPos) == 0U)
        {
            return ttRzPhase;               /* 和候选一致、但相对基准没转：继续等 */
        }

        ttMapRef[1] = rz->byPos[1];
        ttMapRef[2] = rz->byPos[2];
        ttMapRef[3] = rz->byPos[3];
        ttT0        = ttCandTick;

        if (Tt_AssignRef(ttMapRef) == 0U)
        {
            Debug_Log("TT", "WARN align-phase assign ref failed (colors %u,%u,%u map %u,%u,%u)",
                      (unsigned int)ttColors[0], (unsigned int)ttColors[1],
                      (unsigned int)ttColors[2], (unsigned int)ttMapRef[1],
                      (unsigned int)ttMapRef[2], (unsigned int)ttMapRef[3]);
            return ttRzPhase;
        }

        ttRzPhase = TT_RZ_T0;
        Debug_Log("TT", "align-phase t0: ref P%u/P%u/%u (colors %u,%u,%u)",
                  (unsigned int)ttRefPos[0], (unsigned int)ttRefPos[1],
                  (unsigned int)ttRefPos[2], (unsigned int)ttColors[0],
                  (unsigned int)ttColors[1], (unsigned int)ttColors[2]);
        return ttRzPhase;
    }

    if (Tt_IsRotChanged(ttMapBase, rz->byPos) != 0U)
    {
        ttMapCand[1] = rz->byPos[1];
        ttMapCand[2] = rz->byPos[2];
        ttMapCand[3] = rz->byPos[3];
        ttCandTick   = HAL_GetTick();       /* t0 取"这份映射首次出现"的时刻 */
    }
    return ttRzPhase;
}

/**
 * @brief  按本轮目标色顺序，给 3 件分配"基准位号"（写 ttRefPos[]）
 * @param  map: 映射（位号->颜色），通常是 t0 时刻的 ttMapRef
 * @retval 1=3 件都分到位号；0=有目标色在图上找不到（视觉漏识别/颜色不符）
 * @note   同一颜色出现两次时，按"未占用位号从小到大"依次占用 —— 保证 3 件不撞位。
 */
static uint8_t Tt_AssignRef(const uint8_t *map)
{
    uint8_t used[4] = { 0U, 0U, 0U, 0U };
    uint8_t i;
    uint8_t p;

    for (i = 0U; i < (uint8_t)TT_ITEM_COUNT; i++)
    {
        uint8_t found = 0U;

        for (p = 1U; p <= 3U; p++)
        {
            if ((used[p] == 0U) && (map[p] == ttColors[i]))
            {
                used[p]     = 1U;
                ttRefPos[i] = p;
                found       = 1U;
                break;
            }
        }
        if (found == 0U)
        {
            return 0U;
        }
    }
    return 1U;
}

/**
 * @brief  按"记住的顺序 + 计时"推算本件目标位号（第 2/3 件用）
 * @param  item    : 件号 0~2
 * @param  waitTick: 输出，>0 = 到位后还要等到这个时刻（料转到位）再抓；0 = 到了就抓
 * @retval 目标位号 1~3
 * @note   elapsed = 现在 - t0；steps = elapsed / T；off = elapsed % T。
 *         ★【2026-10-24 用户要求】**前 TRAY_ROT_LEAD_MS（默认 2s）**在 steps 位置就地抓；
 *           **之后**提前去 steps+1 位置，停在那里等"料转到位"（= t0 + (steps+1)*T）再抓。
 *         例 T=6000 / LEAD=2000：6~8s 在 +1 位抓；8~12s 去 +2 位等到 12s 再抓；
 *                              12~14s 在 +2 位抓；14~18s 去 +3 位等到 18s 再抓。
 */
static uint8_t Tt_PredictPos(uint8_t item, uint32_t *waitTick)
{
    uint32_t elapsed = HAL_GetTick() - ttT0;
    uint32_t period  = (uint32_t)TRAY_ROT_PERIOD_MS;
    uint32_t steps   = elapsed / period;
    uint8_t  preempt = 0U;
    uint8_t  pos;

    if ((elapsed % period) >= (uint32_t)TRAY_ROT_LEAD_MS)
    {
        steps++;
        preempt   = 1U;
        *waitTick = ttT0 + (steps * period);   /* 料转到该位号的那一刻 */
    }
    else
    {
        *waitTick = 0U;                        /* 就地抓，不用等 */
    }

    pos = Tt_ShiftPos(ttRefPos[item], steps);
    Debug_Log("TT", "item%u predict: t=%ums steps=%u base P%u -> P%u%s",
              (unsigned int)(item + 1U), (unsigned int)elapsed, (unsigned int)steps,
              (unsigned int)ttRefPos[item], (unsigned int)pos,
              (preempt != 0U) ? " (wait there)" : "");
    return pos;
}

/**
 * @brief  记录"本件已放到车载 slot 号位"（颜色↔位号绑定）
 * @param  slot : 车载号位 1~3
 * @param  color: 颜色 1~4
 * @retval 无
 */
static void Tt_RecordSlot(uint8_t slot, uint8_t color)
{
    Task_SetCarSlotColor(slot, color);
    Debug_Log("TT", "item%u/%u placed C%u -> car slot %u, map %s",
              (unsigned int)(ttItem + 1U), (unsigned int)TT_ITEM_COUNT,
              (unsigned int)color, (unsigned int)slot, TT_GetMapText());
}

/**
 * @brief  ★【2026-10-24 新增】在当前位置向车身侧平移一段（非阻塞，一次一段）
 * @param  无
 * @retval 1=本段已到位；0=还在走
 * @note   用途：转盘区发 $RCP 一直拿不到有效数据（视觉没看到转盘）时，
 *         向左/右挪一小段换个视角，再重发 $RCP 重新识别。
 *         移动走 `Mission_NavGoTo`（与 GO 同一条位置模式闭环：电机到位回传 +
 *         四轮同拍瞬停 + 静置 + 落点核对），但**不会**像 GO 那样超时 Task_Stop，
 *         不会打断正在跑的赛题流程；到位超时由 TT 状态机自己的 Tt_Timeout 兜底。
 *         目标点 = 当前里程计坐标 + 名义"右"向量 (sinθ, cosθ) * 距离 * 符号，
 *         该向量与 `Ms_IssueMoveAbs` 的逆变换严格配套（见 bsp_mission.c），
 *         所以现场真往哪边走由底盘标定 MEC_SIGN_VY 决定 —— **方向反了只改
 *         TRAY_RCP_SHIFT_SIGN，不要改这里的数学**。
 */
static uint8_t Tt_ShiftSide(void)
{
    if (ttShiftOn == 0U)
    {
        float th = Chassis_GetOdomYawTrue() * MEC_RAD_PER_DEG;
        float c  = cosf(th);
        float s  = sinf(th);
        float mm = (float)TRAY_RCP_SHIFT_MM * (float)TRAY_RCP_SHIFT_SIGN;

        ttShiftX  = (int32_t)(Chassis_GetOdomX() + (mm * s));
        ttShiftY  = (int32_t)(Chassis_GetOdomY() + (mm * c));
        ttShiftOn = 1U;
        Debug_Log("TT", "rcp no data -> shift %s %dmm (h=%d) -> abs %d,%d",
                  ((int)TRAY_RCP_SHIFT_SIGN >= 0) ? "right" : "left",
                  (int)TRAY_RCP_SHIFT_MM, (int)Chassis_GetOdomYawTrue(),
                  (int)ttShiftX, (int)ttShiftY);
    }

    if (Mission_NavGoTo(ttShiftX, ttShiftY) != 0U)
    {
        ttShiftOn = 0U;
        return 1U;
    }
    return 0U;
}

/* ===================== 对外接口 ===================== */

void TT_Init(void)
{
    ttState     = TT_IDLE;
    ttItem      = 0U;
    ttTargetPos = 0U;
    ttPickColor = 0U;
    ttRzSent    = 0U;
    ttRzTick    = 0U;
    ttAlignOn   = 0U;
    ttArmOn     = 0U;
    ttTailOnly  = 0U;
    ttTailCand  = 0U;
    ttShiftCnt  = 0U;
    ttShiftOn   = 0U;
    ttColors[0] = 0U;
    ttColors[1] = 0U;
    ttColors[2] = 0U;
    ttLimitMs   = 0U;
    ttMapBase[1] = 0U;
    ttMapBase[2] = 0U;
    ttMapBase[3] = 0U;
    ttMapRef[1]  = 0U;
    ttMapRef[2]  = 0U;
    ttMapRef[3]  = 0U;
    ttMapCand[1] = 0U;
    ttMapCand[2] = 0U;
    ttMapCand[3] = 0U;
    ttRefPos[0]  = 0U;
    ttRefPos[1]  = 0U;
    ttRefPos[2]  = 0U;
    ttT0         = 0U;
    ttCandTick   = 0U;
    ttRzRxTick   = 0U;
    ttWaitTick   = 0U;
    ttGrabHoldTick = 0U;
    ttCcgTick    = 0U;
    ttCcgSent    = 0U;
    ttCcgRxTick  = 0U;
    ttCcgArmed   = 0U;
    ttArmIdleTick = 0U;
    ttRzPhase     = TT_RZ_WAIT;
    ttRzOddCnt    = 0U;

    Debug_Log("TT", "init (items=%u, slot order %u->%u)",
              (unsigned int)TT_ITEM_COUNT,
              (unsigned int)TT_SLOT_FIRST,
              (unsigned int)((uint32_t)TT_SLOT_FIRST + (uint32_t)TT_ITEM_COUNT - 1U));
}

uint8_t TT_Start(void)
{
    uint8_t colors[3];
    uint8_t i;

    if (Task_GetCode()->valid == 0U)
    {
        Debug_Log("TT", "start refused: no valid task code");
        return 0U;
    }

    for (i = 0U; i < 3U; i++)
    {
        colors[i] = Task_ColorOfBatch(1U, i);   /* 第 1 组颜色顺序 */
    }

    return TT_StartWithColors(colors);
}

uint8_t TT_StartWithColors(const uint8_t *colors3)
{
    uint8_t i;

    if (colors3 == NULL)
    {
        return 0U;
    }
    if (TT_IsBusy() != 0U)
    {
        return 0U;                              /* 已在跑 */
    }
    if ((Align_IsBusy() != 0U) || (ArmAct_IsBusy() != 0U))
    {
        return 0U;                              /* 对位或机械臂正忙：不抢 */
    }

    for (i = 0U; i < 3U; i++)
    {
        if (colors3[i] == 0U)
        {
            Debug_Log("TT", "start refused: color%u unknown", (unsigned int)i);
            return 0U;
        }
        ttColors[i] = colors3[i];
    }

    ttItem      = 0U;
    ttTargetPos = 0U;
    ttPickColor = 0U;
    ttRzSent    = 0U;
    ttRzTick    = 0U;
    ttAlignOn   = 0U;
    ttArmOn     = 0U;
    ttTailOnly  = 0U;
    ttTailCand  = 0U;
    ttShiftCnt  = 0U;
    ttShiftOn   = 0U;
    ttMapBase[1] = 0U;
    ttMapBase[2] = 0U;
    ttMapBase[3] = 0U;
    ttMapRef[1]  = 0U;
    ttMapRef[2]  = 0U;
    ttMapRef[3]  = 0U;
    ttMapCand[1] = 0U;
    ttMapCand[2] = 0U;
    ttMapCand[3] = 0U;
    ttRefPos[0]  = 0U;
    ttRefPos[1]  = 0U;
    ttRefPos[2]  = 0U;
    ttT0         = 0U;
    ttCandTick   = 0U;
    ttRzRxTick   = 0U;
    ttWaitTick   = 0U;
    ttGrabHoldTick = 0U;
    ttCcgTick    = 0U;
    ttCcgSent    = 0U;
    ttCcgRxTick  = 0U;
    ttCcgArmed   = 0U;
    ttArmIdleTick = 0U;
    ttRzPhase     = TT_RZ_WAIT;
    ttRzOddCnt    = 0U;

    /* 本轮取料重新记账：车载位↔颜色表清零 */
    Task_SetCarSlotColor(1U, 0U);
    Task_SetCarSlotColor(2U, 0U);
    Task_SetCarSlotColor(3U, 0U);

    /* ★【2026-10-10 顺序改为"先观察位"】观察位是 $RCP/$RZ 能拿到数据的前提，
       所以先摆臂到观察位，再发 $RCP（与任务调试页现场验证过的流程一致）。 */
    Debug_Log("TT", "start: arm->observe, colors %u,%u,%u",
              (unsigned int)ttColors[0], (unsigned int)ttColors[1],
              (unsigned int)ttColors[2]);
    Tt_GoTo(TT_OBSERVE, (uint32_t)TT_ARM_TIMEOUT_MS);
    return 1U;
}

void TT_Abort(void)
{
    if (ttState == TT_IDLE)
    {
        return;
    }
    Align_Abort();
    ArmAct_Abort();
    ttState = TT_IDLE;
    ttShiftCnt = 0U;
    ttShiftOn  = 0U;
    Debug_Log("TT", "aborted by user (item %u/%u)", (unsigned int)ttItem,
              (unsigned int)TT_ITEM_COUNT);
}

void TT_Update(void)
{
    if ((ttState == TT_IDLE) || (ttState == TT_DONE) || (ttState == TT_FAILED))
    {
        return;
    }

    /* 通用超时兜底：每个等待状态都被它覆盖（超时 -> 停机器 + 日志 + 置失败） */
    if (Tt_Timeout() != 0U)
    {
        Tt_Fail("state timeout");
        return;
    }

    switch (ttState)
    {
        /* ---- ① 手臂到【观察区对位】+ 舵盘选号位（TRAY PICK 专用动作组） ----
           ★【2026-10-24 用户要求】"电机5 和舵机1 同时动作到观察区对位，
             **同时**舵机3 转到相应的车载位号（第一个物料为1号位）"。
             由 `ArmAct_TrayReady(slot)` 一次下发：S2 松开 -> M6 观察高 ->
             [M5 ‖ S1]；S3 由预选服务同时启动并按回读确认补发。
             本来就已经在观察位时，各步会被"目标已满足"直接跳过，不额外耗时。 */
        case TT_OBSERVE:
        {
            uint8_t slot = (uint8_t)((uint32_t)TT_SLOT_FIRST + (uint32_t)ttItem);

            if (ttArmOn == 0U)
            {
                /* ★【2026-10-29 用户要求】"每个任务点快到达的时候，就提前将手臂转到
                   观察位，并且任务点**不重复**操作此动作"：路上 `Tsk_ArmPreTry()` 已经
                   调过 `ArmAct_GotoTtReady()`，到点时若已在观察位就**整段跳过**
                   `ArmAct_TrayReady()`（只补一次舵盘预选），省掉重复摆臂的时间。
                   与 `SP_OBSERVE` / `STACK_STORE` 的写法一致。 */
                if (ArmAct_IsAtTtReady() != 0U)
                {
                    (void)ArmAct_PreSelectSlot(slot);
                    Debug_Log("TT", "already at observe -> skip tray ready (slot%u)",
                              (unsigned int)slot);
                    ttShiftCnt = 0U;
                    ttShiftOn  = 0U;
                    Tt_GoTo(TT_REQ_RCP, (uint32_t)TRAY_RCP_TIMEOUT_MS);
                    break;
                }
                if (ArmAct_TrayReady(slot) != 0U)
                {
                    ttArmOn = 1U;
                    Debug_Log("TT", "tray ready slot%u (M5|S1 observe + S3)",
                              (unsigned int)slot);
                }
                break;                      /* 启动不了（忙）：下一拍再试，状态超时兜底 */
            }
            if (ArmAct_IsBusy() != 0U)
            {
                break;
            }
            ttArmOn = 0U;
            if (ArmAct_IsAborted() != 0U)
            {
                /* ★【2026-10-23 用户要求】"不要 FAIL，没达成就下一步" */
                Debug_Log("TT", "WARN ready aborted -> continue");
            }
            ttShiftCnt = 0U;            /* 每件重新计"右移搜索"次数 */
            ttShiftOn  = 0U;
            Tt_GoTo(TT_REQ_RCP, (uint32_t)TRAY_RCP_TIMEOUT_MS);
            break;
        }

        /* ---- ② 发 $RCP 请求转盘中心偏移 ---- */
        case TT_REQ_RCP:
            RPi_ReqRcp();
            Debug_Log("TT", "tx $RCP");
            Tt_GoTo(TT_WAIT_RCP, (uint32_t)TRAY_RCP_TIMEOUT_MS);
            break;

        /* ---- ③ 等 $RCP,dx,dy 有效 -> 开始对位 ----
           ★【2026-10-24 新增】等 TRAY_RCP_SHIFT_WAIT_MS 还是没数据，说明视觉没看到转盘：
             原地死等只会超时 FAIL，改为"向右挪一小段换视角 -> 重发 $RCP 重新识别"，
             最多挪 TRAY_RCP_SHIFT_MAX 次；用完还看不到才判失败。 */
        case TT_WAIT_RCP:
            /* ★【2026-10-29】等 $RCP 的这段时间手臂已在观察位、相机看得到转盘：
               并行开始"颜色顺序记忆 + 计时"（锁基准映射 + 认 t0），对位结束就不用再等帧。 */
            Tt_RzTrack();
            if (RPi_GetRcp()->valid != 0U)
            {
                Debug_Log("TT", "RCP ok dx=%d dy=%d -> align",
                          (int)RPi_GetRcp()->dxPx, (int)RPi_GetRcp()->dyPx);
                Align_Abort();
                Align_SetBudget((uint32_t)TT_ALIGN_BUDGET_MS);
                ttAlignOn = 0U;
                Tt_GoTo(TT_ALIGN, (uint32_t)TT_ALIGN_TIMEOUT_MS);
            }
            else if ((HAL_GetTick() - ttStateTick) >= (uint32_t)TRAY_RCP_SHIFT_WAIT_MS)
            {
                if (ttShiftCnt < (uint8_t)TRAY_RCP_SHIFT_MAX)
                {
                    ttShiftCnt++;
                    ttShiftOn = 0U;         /* 允许 Tt_ShiftSide 重新规划本段目标点 */
                    Tt_GoTo(TT_RCP_SHIFT, (uint32_t)TRAY_RCP_SHIFT_TIMEOUT_MS);
                }
                else
                {
                    Tt_Fail("rcp not found (shift used up)");
                }
            }
            break;

        /* ---- ③' 右移一段（换个视角）-> 到位后重发 $RCP 回 ③ ---- */
        case TT_RCP_SHIFT:
            if (Tt_ShiftSide() != 0U)
            {
                Debug_Log("TT", "shift done (%u/%u) -> tx $RCP again",
                          (unsigned int)ttShiftCnt, (unsigned int)TRAY_RCP_SHIFT_MAX);
                RPi_ReqRcp();
                Tt_GoTo(TT_WAIT_RCP, (uint32_t)TRAY_RCP_TIMEOUT_MS);
            }
            break;

        /* ---- ④ 对位（数据源 $RCP；本次只做一次）---- */
        case TT_ALIGN:
            /* ★【2026-10-29】对位期间继续盯着 `$RZ`：颜色顺序（基准映射）和 t0（计时起点）
               都在这里定好，对位成功就能直接按"4s 转 / 2s 停"的窗口去抓目标色。 */
            Tt_RzTrack();
            if (ttAlignOn == 0U)
            {
                if (Align_Start() != 0U)
                {
                    ttAlignOn = 1U;
                    Debug_Log("TT", "align start");
                }
                break;                          /* 数据可能刚过期：下一拍再试 */
            }
            if (Align_IsBusy() != 0U)
            {
                break;
            }
            if (Align_IsDone() != 0U)
            {
                Debug_Log("TT", "align done");
                /* ★【2026-10-24】S3 的换位已经并入 TT_OBSERVE 的 `ArmAct_TrayReady()`
                   （"回到观察区的同时转到该车载号位"），这里**不再重复下发**，
                   保证舵盘每个物料只执行一次换位命令。 */
                ttRzSent     = 0U;
                ttRzOddCnt   = 0U;
                if (ttRzPhase == TT_RZ_T0)
                {
                    /* ★★【2026-10-29 用户要求】"颜色顺序和定时在对位阶段就记忆，对位成功
                       后直接抓取"：t0 已在对位期间定好、3 件位号也已分配好 —— 这里
                       直接进 TT_PREP 抓第 1 件（位号在 TT_PREP 里按 t0 推算），
                       不再走 TT_RZ_BASE / TT_RZ_EDGE 的等帧。 */
                    ttItem      = 0U;
                    ttTargetPos = 0U;
                    ttArmOn     = 0U;
                    ttWaitTick  = 0U;
                    ttCcgSent   = 0U;
                    ttCcgTick   = 0U;
                    Debug_Log("TT", "align done -> t0 ready, grab item1 (ref P%u)",
                              (unsigned int)ttRefPos[0]);
                    Tt_GoTo(TT_PREP, (uint32_t)TT_ARM_TIMEOUT_MS);
                }
                else if (ttRzPhase == TT_RZ_BASED)
                {
                    /* 颜色顺序已记住、还差一个换位边沿：只剩等边沿 */
                    Debug_Log("TT", "align done -> base ready, wait edge only");
                    Tt_GoTo(TT_RZ_EDGE, (uint32_t)TRAY_RZ_EDGE_TIMEOUT_MS);
                }
                else
                {
                    ttRzRxTick   = 0U;
                    ttMapCand[1] = 0U;
                    ttMapCand[2] = 0U;
                    ttMapCand[3] = 0U;
                    Tt_GoTo(TT_RZ_BASE, (uint32_t)TRAY_RZ_BASE_TIMEOUT_MS);
                }
            }
            else
            {
                Tt_Fail("align failed");
            }
            break;

        /* ---- ⑤ ★锁**基准映射**（位号->颜色）----
           ★【2026-10-24 新抓取逻辑】周期发 `$RZ`（单帧快照，必须重发）拿到"位号↔颜色"。
             转盘可能还在转（视觉单帧会给出"部分位号已换料"的混合映射），所以要求
             **同一份映射连续两帧出现**才认定为静止基准，避免把过渡帧当基准。
             `byPos[1..3]` 必须全非 0（三个位号都认出来了）才参与比较。 */
        case TT_RZ_BASE:
        {
            const RPi_Rz_t *rz = RPi_GetRz();

            Tt_RzPoll();

            if ((rz->valid == 0U) || (rz->lastRxTick == ttRzRxTick))
            {
                break;                      /* 没新帧：等下一拍 */
            }
            ttRzRxTick = rz->lastRxTick;

            if (Tt_MapReady(rz->byPos) == 0U)
            {
                Debug_Log("TT", "WARN rz incomplete (%u,%u,%u)",
                          (unsigned int)rz->byPos[1], (unsigned int)rz->byPos[2],
                          (unsigned int)rz->byPos[3]);
                break;
            }

            if (Tt_MapEqual(ttMapCand, rz->byPos) != 0U)
            {
                ttMapBase[1] = rz->byPos[1];
                ttMapBase[2] = rz->byPos[2];
                ttMapBase[3] = rz->byPos[3];
                Debug_Log("TT", "base map locked: P1=C%u P2=C%u P3=C%u",
                          (unsigned int)ttMapBase[1], (unsigned int)ttMapBase[2],
                          (unsigned int)ttMapBase[3]);
                ttRzRxTick = rz->lastRxTick;
                Tt_GoTo(TT_RZ_EDGE, (uint32_t)TRAY_RZ_EDGE_TIMEOUT_MS);
            }
            else
            {
                ttMapCand[1] = rz->byPos[1];
                ttMapCand[2] = rz->byPos[2];
                ttMapCand[3] = rz->byPos[3];
            }
            break;
        }

        /* ---- ⑥ ★等"转了一步" -> 记 t0 -> 直接抓第 1 件 ----
           判据（抗误检）：当前帧相对基准是**完美循环移位**（整盘整体转了 k=1/2 步），
           且这份映射**连续两帧**出现（排除"某一个位号识别错色"的假变化）。
           t0 取这份映射**首次出现**的时刻（= 转盘刚停稳那一刻），后续 3 件全靠它推算。 */
        case TT_RZ_EDGE:
        {
            const RPi_Rz_t *rz = RPi_GetRz();

            Tt_RzPoll();

            if ((rz->valid == 0U) || (rz->lastRxTick == ttRzRxTick))
            {
                break;                      /* 没新帧：等下一拍 */
            }
            ttRzRxTick = rz->lastRxTick;

            if (Tt_MapReady(rz->byPos) == 0U)
            {
                break;
            }

            if (Tt_MapEqual(ttMapCand, rz->byPos) != 0U)
            {
                if (Tt_IsRotChanged(ttMapBase, rz->byPos) == 0U)
                {
                    ttRzOddCnt = 0U;
                    break;                  /* 只是和候选一致、相对基准并没转：继续等 */
                }

                ttMapRef[1] = rz->byPos[1];
                ttMapRef[2] = rz->byPos[2];
                ttMapRef[3] = rz->byPos[3];
                ttT0        = ttCandTick;

                if (Tt_AssignRef(ttMapRef) == 0U)
                {
                    /* 本轮目标色在图上找不到（视觉漏识别/颜色不符）：再等下一帧重试 */
                    Debug_Log("TT", "WARN assign ref failed (colors %u,%u,%u map %u,%u,%u)",
                              (unsigned int)ttColors[0], (unsigned int)ttColors[1],
                              (unsigned int)ttColors[2], (unsigned int)ttMapRef[1],
                              (unsigned int)ttMapRef[2], (unsigned int)ttMapRef[3]);
                    break;
                }

                ttItem      = 0U;
                ttTargetPos = ttRefPos[0];
                ttArmOn     = 0U;
                ttWaitTick  = 0U;           /* 第 1 件：就地抓，不等 */
                ttCcgSent   = 0U;           /* 等待位颜色反馈重新开始 */
                ttCcgTick   = 0U;
                Debug_Log("TT", "rot detected -> t0, item1 C%u at P%u (ref P%u/%u/%u)",
                          (unsigned int)ttColors[0], (unsigned int)ttTargetPos,
                          (unsigned int)ttRefPos[0], (unsigned int)ttRefPos[1],
                          (unsigned int)ttRefPos[2]);
                Tt_GoTo(TT_PREP, (uint32_t)TT_ARM_TIMEOUT_MS);
                break;
            }

            if (Tt_IsRotChanged(ttMapBase, rz->byPos) != 0U)
            {
                /* 出现"疑似转过一步"的新映射：登记为候选并记下首次出现时刻 */
                ttMapCand[1] = rz->byPos[1];
                ttMapCand[2] = rz->byPos[2];
                ttMapCand[3] = rz->byPos[3];
                ttCandTick   = HAL_GetTick();
                ttRzOddCnt   = 0U;
                break;
            }

            if (Tt_MapEqual(ttMapBase, rz->byPos) != 0U)
            {
                ttRzOddCnt = 0U;            /* 和基准一致（还没转）：继续等 */
                break;
            }

            /* ★【2026-10-29】这一帧**既不是基准、也不是完美循环移位**：如果用对位阶段
               提前锁的基准，可能是车身对位过程改变了视角（位号映射整体变了），此时
               永远等不到"完美循环移位" -> 只能 20s 超时 FAIL。连续 3 帧都这样才认定
               基准失效，退回 TT_RZ_BASE 重新锁一份再继续等。 */
            if ((ttRzPhase != TT_RZ_T0) && (ttRzPhase != TT_RZ_BASED))
            {
                break;                      /* 基准是原地锁的：按原逻辑继续等 */
            }
            ttRzOddCnt++;
            if (ttRzOddCnt >= 3U)
            {
                Debug_Log("TT", "WARN early base stale (%u,%u,%u) -> relock base",
                          (unsigned int)rz->byPos[1], (unsigned int)rz->byPos[2],
                          (unsigned int)rz->byPos[3]);
                ttRzPhase     = TT_RZ_WAIT;
                ttRzOddCnt    = 0U;
                ttMapCand[1]  = 0U;
                ttMapCand[2]  = 0U;
                ttMapCand[3]  = 0U;
                Tt_GoTo(TT_RZ_BASE, (uint32_t)TRAY_RZ_BASE_TIMEOUT_MS);
            }
            break;
        }

        /* ---- ⑦ 手臂去本件目标位号（ArmAct_GotoTtPos，**不含舵机2 抓紧**）----
           ★【2026-10-24 新抓取逻辑】第 1 件的位号在 TT_RZ_EDGE 就定好了（= t0 那帧看到的）；
             第 2/3 件**不再回观察位、不再对位**，位号在这里按"记住的基准位号 + 计时"推算：
               steps = (now - t0) / T；周期内已过时间 < TRAY_ROT_LEAD_MS -> 就地抓 steps 位；
               ≥ TRAY_ROT_LEAD_MS -> 提前去 steps+1 位，**停在那儿等料转到位**（= t0+(steps+1)*T）再抓。
             等待时还要 `$CCG` 颜色反馈确认（"等待位有颜色反馈，根据颜色反馈来抓"）。
             推算时刻尽量晚（就在摆臂这一拍算），把手臂行程也算进"现在"。 */
        case TT_PREP:
            if (ttArmOn == 0U)
            {
                if (ttTargetPos == 0U)
                {
                    ttTargetPos = Tt_PredictPos(ttItem, &ttWaitTick);
                }
                if (ArmAct_GotoTtPos(ttTargetPos) != 0U)
                {
                    ttArmOn      = 1U;
                    ttCcgArmed   = 0U;  /* ★新的一段行程：颜色等待重新"上锁" */
                    ttArmIdleTick = 0U;
                    ttGrabHoldTick = 0U;   /* ★保持计时重新开始 */
                    Debug_Log("TT", "item%u goto P%u (TTPREP)%s",
                              (unsigned int)(ttItem + 1U), (unsigned int)ttTargetPos,
                              (ttWaitTick != 0U) ? " -> wait for material" : "");
                }
                break;                      /* 启动不了：下一拍再试，状态超时兜底 */
            }
            if ((ArmAct_IsBusy() != 0U) || (ArmAct_IsAtTtPos(ttTargetPos) == 0U))
            {
                ttArmIdleTick = 0U;         /* 还在动 / 电机5、6 还没真到位：到位时刻重新计 */
                ttCcgArmed    = 0U;
                break;
            }
            /* ★【2026-10-26 用户要求】"请在**手臂到达等待位**再开启等待颜色"：
               到位判据 = ①没有动作在跑 ②电机5/6 **回读**已在该号位取料姿态（`ArmAct_IsAtTtPos`）。
               只判 `ArmAct_IsBusy()` 不够：动作"跑完"时电机可能还在走最后一段
               （并行对超宽限、电机步没到位也会收尾），那时就开始等色/抓，现场就是
               "手臂还在去等待位途中、相机已经看到目标色 -> 直接抓"。
               真到位后再**静置 `TT_PREP_ARRIVE_MS`**，这一拍之前**一帧 `$CCG` 都不发**。 */
            if (ttArmIdleTick == 0U)
            {
                ttArmIdleTick = HAL_GetTick();
                break;
            }
            if ((uint32_t)(HAL_GetTick() - ttArmIdleTick) < (uint32_t)TT_PREP_ARRIVE_MS)
            {
                break;
            }
            if (ttCcgArmed == 0U)
            {
                /* 到达等待位这一拍：**重置 `$CCG` 会话**（丢弃到位前的旧应答），
                   从这里开始请求的颜色才算数 */
                ttCcgArmed = 1U;
                ttCcgSent  = 0U;
                ttCcgTick  = 0U;
                ttGrabHoldTick = 0U;    /* ★保持计时从"开启等色"这一拍重新计 */
                if (ttWaitTick != 0U)
                {
                    Debug_Log("TT", "item%u at wait P%u -> enable color wait",
                              (unsigned int)(ttItem + 1U), (unsigned int)ttTargetPos);
                }
            }
            /* ★到位了但料还没转到这一格：停在位号处等（ttArmOn 保持 1，不重复下发动作） */
            if (ttWaitTick != 0U)
            {
                if ((int32_t)(HAL_GetTick() - ttWaitTick) < 0)
                {
                    break;                  /* 时间没到：先等 */
                }
                /* ★【2026-10-24 用户要求】"等待位有颜色反馈，根据颜色反馈来抓"：
                   时间到了还不算数，必须 `$CCG` 回报"本位号就是本件目标色"才抓。
                   一直不确认则由本状态超时兜底（屏上显示 `TT prep P# wait`）。 */
                if (Tt_ColorReady(ttTargetPos, ttColors[ttItem]) == 0U)
                {
                    break;
                }
            }

            /* ★★【2026-11-02 用户要求】"每个物料等 500ms，不要颜色出现就抓"：
               颜色已确认（或本条不需要等色）之后，**再保持 `TT_GRAB_HOLD_MS`** 才夹紧，
               避开"料刚转到位/刚被识别就抓"造成的抓偏、抓边。 */
            if (ttGrabHoldTick == 0U)
            {
                ttGrabHoldTick = HAL_GetTick();
                Debug_Log("TT", "item%u color ok -> hold %ums before grab",
                          (unsigned int)(ttItem + 1U), (unsigned int)TT_GRAB_HOLD_MS);
            }
            if ((uint32_t)(HAL_GetTick() - ttGrabHoldTick) < (uint32_t)TT_GRAB_HOLD_MS)
            {
                break;                  /* 还没保持够：先别抓 */
            }
            ttGrabHoldTick = 0U;

            ttWaitTick = 0U;
            ttArmOn    = 0U;
            if (ArmAct_IsAborted() != 0U)
            {
                /* ★【2026-10-23 用户要求】"不要 FAIL，没达成就下一步" */
                Debug_Log("TT", "WARN prep aborted -> continue");
            }
            Tt_GoTo(TT_GRAB, (uint32_t)TT_ARM_TIMEOUT_MS);
            break;

        /* ---- ⑧ 抓取 + 放车载（**一整套** `ArmAct_TrayPickPlace`）----
           ★【2026-10-24 用户要求】"看到目标物料舵机2抓取 -> 电机6抬升和舵机1缩回
             同时运行，舵机1<500 时电机5开始转向 -> 电机5 到目标±100 内电机6下降 ->
             到位直接松手舵机2"。整个段由这一个动作组完成，启动后转到 TT_PLACE 等它跑完。
           ★【2026-10-24 新抓取逻辑】位号是"观察位 + 计时"定的，抓前若在"等待位"
             已由 TT_PREP 用 `$CCG` 颜色反馈确认过目标色；记账颜色取 `ttColors[ttItem]`。 */
        case TT_GRAB:
        {
            uint8_t slot = (uint8_t)((uint32_t)TT_SLOT_FIRST + (uint32_t)ttItem);

            if (ttArmOn != 0U)
            {
                break;
            }

            ttPickColor = ttColors[ttItem];

            if (ArmAct_TrayPickPlace(slot) != 0U)
            {
                ttArmOn         = 1U;
                Debug_Log("TT", "grab+place slot%u (TRAYPICK)", (unsigned int)slot);

                /* 最后一件已到手：只记"候选"，真正的放行判据（S1 已回 0）在 TT_PLACE 每拍复核 */
                if ((TT_TAIL_EARLY_EN != 0U) &&
                    ((uint8_t)(ttItem + 1U) >= (uint8_t)TT_ITEM_COUNT) && (ttTailCand == 0U))
                {
                    ttTailCand = 1U;
                    Debug_Log("TT", "last item in hand -> tail candidate (wait S1=0)");
                }
                Tt_GoTo(TT_PLACE, (uint32_t)TT_ARM_TIMEOUT_MS);
            }
            break;
        }

        /* ---- ⑩ 等"抓 + 放"跑完 -> 记账 -> 下一件 ---- */
        case TT_PLACE:
        {
            uint8_t slot = (uint8_t)((uint32_t)TT_SLOT_FIRST + (uint32_t)ttItem);

            /* ★【2026-10-15 用户要求】**每拍复核**"只剩收尾 + S1 已回 0"（**严格档**：
               回读不可用不放行）。通过了才置 `ttTailOnly` -> 任务点可让底盘先动、手臂并行。
               ★【2026-10-21 规则3】放行判据 = **S1 已回 0** 或 **手臂已进入电机5 转向那一步**。
               ★★【2026-10-29 用户要求】"抓最后一个物料时，一定要保证**物料已经抓在爪子上**，
               并且**手臂抬高了**，再进行提前跑路段操作"：上面两条**只看 S1 回读**，
               而 S1 的回读可能是**上一件留下的 0**（缓存有效期 1000ms），进 TT_PLACE 的
               第一拍就能命中 -> 车立刻开跑，此时爪子还没夹紧、手臂还低在转盘里
               （现场现象："最后一个还没抓到车就跑了"）。所以必须再加：
               `ArmAct_GripHeld()`（料确实夹住） + `ArmAct_LiftAboveSafe()`（M6 已抬高）。 */
            if ((TT_TAIL_EARLY_EN != 0U) && (ttTailCand != 0U) && (ttTailOnly == 0U) &&
                (ArmAct_GripHeld() != 0U) && (ArmAct_LiftAboveSafe() != 0U) &&
                ((ArmAct_S1Retracted() != 0U) || (ArmAct_IsTurning() != 0U)))
            {
                ttTailOnly = 1U;
                Debug_Log("TT", "tail only: grip held + lifted + %s -> chassis may start",
                          (ArmAct_IsTurning() != 0U) ? "M5 turning" : "S1=0");
            }

            if ((ttArmOn == 0U) || (ArmAct_IsBusy() != 0U))
            {
                break;
            }

            /* ★【2026-10-24 用户要求】"舵盘(舵机3)转向还是太快 —— 等手臂把物料松开、
               抬起来了再转舵盘，不要同时"。原实现是"放料动作跑完后再静置
               `TT_PLACE_S3_DELAY_MS`(100ms)"；★★【2026-10-29 用户要求】"手臂运行过程中
               有很多不必要的等待，能优化的都优化掉"：`flowTrayPickPlace` 的末步就是
               "电机6 抬回安全高"，松爪步还自带 `ACT_PLACE_SETTLE_MS` 静置 —— 等它
               `ArmAct_IsBusy()==0` 时"抬臂"早已完成，这 100ms 纯属重复，**整段删除**
               （`ttPlaceIdleTick` 变量与 `TT_PLACE_S3_DELAY_MS` 宏一并去掉）。
               现场若再觉得"抬臂和转舵盘挨太近"，请在 `flowTrayPickPlace` 末步加
               `ACT_STEP_SETTLE`，而不是恢复这里的空等。 */
            ttArmOn = 0U;
            /* ★★【2026-10-24 用户要求】"没抓住物料就停下报 FAIL，绝不动别的关节"：
               夹紧判据已重做（见 `Act_ServoDoneGrip`），只有"确实夹到"才放行；
               重夹用尽仍没夹到（`ArmAct_GripFailed()`）-> **整段停下**，不记账、不继续下一件。 */
            if (ArmAct_GripFailed() != 0U)
            {
                Tt_Fail("grip failed");
                break;
            }
            if (ArmAct_IsAborted() != 0U)
            {
                /* ★【2026-10-23 用户要求】"不要 FAIL，没达成就下一步"：
                   放料被中止也**照常记到车载位表**并继续下一件。 */
                Debug_Log("TT", "WARN place aborted -> continue (item%u)",
                          (unsigned int)(ttItem + 1U));
            }

            Tt_RecordSlot(slot, ttPickColor);
            ttItem++;
            if (ttItem >= (uint8_t)TT_ITEM_COUNT)
            {
                ttState = TT_DONE;
                Debug_Log("TT", "DONE: %u items on car, map %s",
                          (unsigned int)ttItem, TT_GetMapText());
            }
            else
            {
                /* 未满 3 件：★【2026-10-24 新抓取逻辑】**不再回观察位、不再对位** ——
                   直接按"记住的顺序 + 计时"去下一件；顺便把舵盘（舵机3）转到下一个
                   车载号位，与随后的摆臂并行。 */
                uint8_t nextSlot = (uint8_t)((uint32_t)TT_SLOT_FIRST + (uint32_t)ttItem);

                ttArmOn     = 0U;
                ttTargetPos = 0U;               /* 让 TT_PREP 在那一拍按时间重新推算 */
                ttWaitTick  = 0U;               /* 与位号一起清，等 TT_PREP 重新推算 */
                ttGrabHoldTick = 0U;            /* ★下一件的"颜色保持"重新计 */
                ttCcgSent   = 0U;               /* 下一个等待位：颜色反馈从"立即发"重新开始 */
                ttCcgTick   = 0U;
                (void)ArmAct_PreSelectSlot(nextSlot);
                /* 本状态现在可能包含"停在下一格等料转到位"（最长约一个周期），超时按 30s 算 */
                Tt_GoTo(TT_PREP, (uint32_t)TT_ARM_TIMEOUT_MS);
            }
            break;
        }

        default:
            break;
    }
}

uint8_t TT_IsBusy(void)
{
    return ((ttState != TT_IDLE) && (ttState != TT_DONE) &&
            (ttState != TT_FAILED)) ? 1U : 0U;
}

uint8_t TT_IsDone(void)
{
    return (ttState == TT_DONE) ? 1U : 0U;
}

uint8_t TT_IsFailed(void)
{
    return (ttState == TT_FAILED) ? 1U : 0U;
}

/* ★【2026-10-15 用户要求】是否只剩"多余动作"（最后一件已到手 + S1 已回 0）。
   注意：只作"可以放行"的判据，**不改变 ttState**，TT_Update 仍会把收尾动作跑完。 */
uint8_t TT_TailOnly(void)
{
    /* ★【2026-11-02 用户要求】`TT_TAIL_EARLY_EN = 0` 时恒不放行：
       必须等 3 件全部放进车载位、手臂整段跑完，车轮才动。 */
    if ((TT_TAIL_EARLY_EN == 0U) || (ttTailOnly == 0U))
    {
        return 0U;
    }
    return 1U;
}

const char *TT_GetStateText(void)
{
    static char line[24];

    switch (ttState)
    {
        case TT_OBSERVE:
            return "TT arm observe";

        case TT_REQ_RCP:
            return "TT req RCP";

        case TT_WAIT_RCP:
            return "TT wait RCP";

        case TT_RCP_SHIFT:
            (void)snprintf(line, sizeof(line), "TT shift %u/%u",
                           (unsigned int)ttShiftCnt, (unsigned int)TRAY_RCP_SHIFT_MAX);
            return line;

        case TT_ALIGN:
            return Align_GetStateText();        /* 对位的详细进度交给它显示 */

        case TT_RZ_BASE:
            return "TT rz base";

        case TT_RZ_EDGE:
            return "TT rz edge";

        case TT_PREP:
            (void)snprintf(line, sizeof(line), "TT prep P%u%s", (unsigned int)ttTargetPos,
                           (ttWaitTick != 0U) ? " wait" : "");
            return line;

        case TT_GRAB:
            (void)snprintf(line, sizeof(line), "TT grab %u/3",
                           (unsigned int)(ttItem + 1U));
            return line;

        case TT_PLACE:
            (void)snprintf(line, sizeof(line), "TT put S%u",
                           (unsigned int)((uint32_t)TT_SLOT_FIRST + (uint32_t)ttItem));
            return line;

        case TT_DONE:
            (void)snprintf(line, sizeof(line), "TT %s", TT_GetMapText());
            return line;

        case TT_FAILED:
            return "TT FAIL (see log)";

        default:
            return "TT idle";
    }
}

uint8_t TT_GetSlotColor(uint8_t slot)
{
    return Task_GetCarSlotColor(slot);
}

const char *TT_GetMapText(void)
{
    static char map[24];

    (void)snprintf(map, sizeof(map), "3:C%u 2:C%u 1:C%u",
                   (unsigned int)Task_GetCarSlotColor(3U),
                   (unsigned int)Task_GetCarSlotColor(2U),
                   (unsigned int)Task_GetCarSlotColor(1U));
    return map;
}

/* ============================================================================
 *               ② 打靶放置序列（STORE PLACE / ROUGH PLACE）
 *   与上面的"转盘取料"同属**作业点序列**，故放在本文件，两个入口共用一份代码：
 *     · 任务调试页 `TDBG_STORE_PLACE` / `TDBG_ROUGH_PLACE`（bsp_task_dbg.c）
 *     · 赛题流程 `T_B1_PLACE`（`PLACE_STORE_ALL()`，bsp_task.c）
 *   流程（现场按 $SHOT 调试确定）：
 *     ① **★先发 `$SHOT`**（2026-10-10 晚用户要求）—— 让树莓派**先把程序启起来/开始出帧**，
 *        解决"**到观察位后无响应**"（用户判断是树莓派开程序超时）。
 *     ② 臂进【全局观察位】(ArmAct_GotoTtReady) —— 视觉要先看到靶区圆环；
 *        **手臂走的这段时间就是给树莓派启动的提前量**。
 *     ③ 等 `$SHOT,...,dx,dy` 有效（`SP_WAIT_SHOT`，**期间周期重发**，见 `SP_SHOT_REQ_MS`）
 *     ④ 精确对位（`Align_StartShot`，数据源 `$SHOT` -> 严格档：死区 1px + **不限时**
 *        + 关闭放宽出口 —— "不到 (0,0) 不放过"）
 *     ⑤~⑧ 逐件搬，**车载位 3 -> 2 -> 1**（★2026-10-10 晚用户要求，原来是 1->2->3）：
 *        读该位绑定的颜色 -> 在任务码第 1 组按颜色查靶位 ->
 *        `ArmAct_Run(ARM_OP_CAR_PICK, slot)` 取件 -> `ArmAct_Run(ARM_OP_TGT_PLACE, tgt)` 放靶位
 *   ⚠ 前提：**先跑过 TRAY PICK**（它把颜色↔车载位号绑定记进 `Task_SetCarSlotColor`），
 *      否则第 ⑤ 步会以 `no binding` 失败。
 * ==========================================================================*/

typedef enum {
    SP_IDLE = 0,
    SP_REQ_SHOT,      /* ① ★【2026-10-10】**先发 $SHOT**：让树莓派先起动/开始出帧（防"到观察位后无响应"） */
    SP_OBSERVE,       /* ② 臂进【观察位】(ArmAct_GotoTtReady)：这期间视觉那边已经在算 */
    SP_WAIT_SHOT,     /* ③ 等 $SHOT 有效（**周期重发**，见 SP_SHOT_REQ_MS / SP_SHOT_TIMEOUT_MS） */
    SP_ALIGN,         /* ④ 启动对位（Align_StartShot，严格档） */
    SP_WAIT_ALIGN,    /* ⑤ 等对位结束（**不限时**） */
    SP_BIND,          /* ⑥ 读车载位绑定色 -> 任务码第1组查靶位 */
    SP_PICK,          /* ⑦ 车载取件 -> 靶位放置（★2026-10-24 合并成一条连续动作 CAR2TGT） */
    SP_ARM_IDLE,      /* ⑧ ★ROUGH PLACE 收尾：手臂回车载 3 号位、**不抓** */
    SP_DONE,          /* 3 件全部放完 */
    SP_FAILED         /* 失败中止 */
} SP_State_t;

static SP_State_t spState   = SP_IDLE;
static uint8_t    spSlot    = 0U;   /* 当前在搬的车载位号 1~3 */
static uint8_t    spTgtPos  = 0U;   /* 本件要放的靶位号 1~3 */
static uint8_t    spColor   = 0U;   /* 本件车载位绑定的颜色 1~4（显示用） */
static uint8_t    spArmOn   = 0U;   /* 1=本段机械臂动作已启动 */
static uint8_t    spRough   = 0U;   /* ★1=本段是 ROUGH PLACE（放完 3 件后做"回车载位不抓"收尾） */
/* ★【2026-10-23】本段按**任务码第几批**取颜色/靶位号（1=第1/2组；2=第3/4组）。
   第二批要在粗加工区按第 4 组放置，所以不能写死第 1 批。 */
static uint8_t    spBatch   = 1U;
static uint8_t    spTailOnly = 0U;  /* ★【2026-10-15】1=3 件已放完、正在做"回车载位不抓"收尾
                                       且 S1 已回 0，只剩多余动作 -> 可让底盘先动 */
static uint32_t   spStateTick = 0U;
static uint32_t   spLimitMs   = 0U;
static uint32_t   spShotTick  = 0U;   /* ★上次发 $SHOT 的时刻（SP_WAIT_SHOT 周期重发用） */

static void Sp_GoTo(SP_State_t st, uint32_t limitMs)
{
    spState     = st;
    spStateTick = HAL_GetTick();
    spLimitMs   = limitMs;
}

static uint8_t Sp_Timeout(void)
{
    if (spLimitMs == 0U)
    {
        return 0U;                  /* 0 = 不超时（对位段用它，做到"不限时"） */
    }
    return ((HAL_GetTick() - spStateTick) >= spLimitMs) ? 1U : 0U;
}

static void Sp_Fail(const char *why)
{
    Align_Abort();
    ArmAct_Abort();
    spState = SP_FAILED;
    Debug_Log("SP", "FAIL: %s (slot %u/3)", why, (unsigned int)spSlot);
}

void SP_SetRoughMode(uint8_t on)
{
    spRough = (on != 0U) ? 1U : 0U;
}

void SP_SetBatch(uint8_t batch)
{
    spBatch = (batch == 2U) ? 2U : 1U;
}

void SP_Init(void)
{
    spState   = SP_IDLE;
    spSlot    = 0U;
    spTgtPos  = 0U;
    spColor   = 0U;
    spArmOn   = 0U;
    spLimitMs = 0U;
    spTailOnly = 0U;
    spShotTick = 0U;
    Debug_Log("SP", "init (打靶放置序列)");
}

uint8_t SP_Start(void)
{
    if (SP_IsBusy() != 0U)
    {
        return 0U;
    }
    if ((Align_IsBusy() != 0U) || (ArmAct_IsBusy() != 0U))
    {
        return 0U;                  /* 对位或机械臂正忙：不抢 */
    }

    spSlot    = 1U;   /* ★【2026-10-24 用户要求】按任务码第 2 组数字的**顺序**放：
                         第 1 件 -> 第 1 个靶位号，第 2 件 -> 第 2 个……（车载位 1->2->3） */
    spTgtPos  = 0U;
    spColor   = 0U;
    spArmOn   = 0U;
    spTailOnly = 0U;
    spShotTick = 0U;

    /* ★【2026-10-10 晚 用户要求】**先发 $SHOT，再进观察位**：
       原来"先进观察位、再发 $SHOT"，现场出现"**到观察位后无响应**"
       （用户判断是**树莓派开程序超时**）。现在先发，手臂去观察位这段时间就是提前量，
       且 `SP_WAIT_SHOT` 期间**每 SP_SHOT_REQ_MS 重发一次**（防首帧被丢）。 */
    Debug_Log("SP", "start: $SHOT first, then arm->observe + align + place x%u",
              (unsigned int)SP_ITEM_COUNT);
    Sp_GoTo(SP_REQ_SHOT, 0U);       /* 立刻发 $SHOT；本态不做等待，下一拍即进入观察位 */
    return 1U;
}

void SP_Abort(void)
{
    if (spState == SP_IDLE)
    {
        return;
    }
    Align_Abort();
    ArmAct_Abort();
    spState = SP_IDLE;
    Debug_Log("SP", "aborted by user (slot %u/3)", (unsigned int)spSlot);
}

void SP_Update(void)
{
    if ((spState == SP_IDLE) || (spState == SP_DONE) || (spState == SP_FAILED))
    {
        return;
    }

    if (Sp_Timeout() != 0U)
    {
        Sp_Fail("state timeout");
        return;
    }

    switch (spState)
    {
        /* ---- ① ★先发 $SHOT（让树莓派先把程序启起来/开始出帧）---- */
        case SP_REQ_SHOT:
            RPi_ReqShot();
            spShotTick = HAL_GetTick();     /* 记发送时刻：SP_WAIT_SHOT 的"首次补发/重发"计时基准 */
            Debug_Log("SP", "tx $SHOT first -> arm->observe");
            Sp_GoTo(SP_OBSERVE, (uint32_t)TT_ARM_TIMEOUT_MS);
            break;

        /* ---- ② 臂进【全局观察位】（打靶区圆环要视觉先看到）
                ★手臂走的这段时间 = 给树莓派的**启动/出首帧提前量**（用户要求先发 $SHOT）---- */
        case SP_OBSERVE:
            /* ★【2026-10-21 用户要求】路上已提前摆到观察位 -> 本步直接跳过 */
            if (ArmAct_IsAtTtReady() != 0U)
            {
                spArmOn = 0U;
                Debug_Log("SP", "already at observe -> skip");
                Sp_GoTo(SP_WAIT_SHOT, (uint32_t)SP_SHOT_TIMEOUT_MS);
                break;
            }
            if (spArmOn == 0U)
            {
                if (ArmAct_GotoTtReady() != 0U)
                {
                    spArmOn = 1U;
                    Debug_Log("SP", "goto observe (TTREADY)");
                }
                break;                  /* 启动不了：下一拍再试（状态超时兜底） */
            }
            if (ArmAct_IsBusy() != 0U)
            {
                break;
            }
            spArmOn = 0U;
            if (ArmAct_IsAborted() != 0U)
            {
                Sp_Fail("observe aborted");
                break;
            }
            /* 到观察位后马上进"等帧"：因为 spShotTick 是"出发前"记的，多半已超 SP_SHOT_REQ_MS，
               所以下一拍会**立刻补发一次 $SHOT**（此时树莓派应该已经在监听了）——正是我们要的。 */
            Sp_GoTo(SP_WAIT_SHOT, (uint32_t)SP_SHOT_TIMEOUT_MS);
            break;

        /* ---- ③ 等 $SHOT,dx,dy 有效 -> 开始严格对位
                ★**周期重发**：v2.0 是一问一答、无 ACK，首帧可能被丢（树莓派还没在监听），
                 不重发就只能干等到超时失败 —— 见 bsp_tt.h 的 SP_SHOT_REQ_MS / SP_SHOT_TIMEOUT_MS ---- */
        case SP_WAIT_SHOT:
            if (RPi_GetShot()->valid != 0U)
            {
                Debug_Log("SP", "SHOT ok dx=%d dy=%d -> align",
                          (int)RPi_GetShot()->dxPx, (int)RPi_GetShot()->dyPx);
                Align_SetBudget(0U);            /* 打靶档恒不限时（这里显式表态） */
                Sp_GoTo(SP_ALIGN, (uint32_t)SP_ALIGN_START_TIMEOUT_MS);
                break;
            }
            if ((HAL_GetTick() - spShotTick) >= (uint32_t)SP_SHOT_REQ_MS)
            {
                spShotTick = HAL_GetTick();
                RPi_ReqShot();                  /* 补发（v2.0：不重发就永远没有） */
            }
            break;

        /* ---- ④ 启动对位（$SHOT 帧 500ms 失效 -> 重发再试，不是失败）---- */
        case SP_ALIGN:
            if (Align_StartShot() != 0U)
            {
                Debug_Log("SP", "align start (SHOT, no time limit)");
                Sp_GoTo(SP_WAIT_ALIGN, 0U);     /* 0 = **不限时**，磨到进 1px 死区为止 */
            }
            else
            {
                RPi_ReqShot();
            }
            break;

        /* ---- ⑤ 等对位结束（不限时；只有"真正进死区"或"视觉断流"两种出路）---- */
        case SP_WAIT_ALIGN:
            if (Align_IsBusy() != 0U)
            {
                break;
            }
            if (Align_IsDone() != 0U)
            {
                Debug_Log("SP", "aligned");
                spSlot  = 1U;   /* ★按任务码第 2 组的顺序：1 -> 2 -> 3 */
                spArmOn = 0U;
                Sp_GoTo(SP_BIND, (uint32_t)TT_ARM_TIMEOUT_MS);
            }
            else
            {
                Sp_Fail("align failed");
            }
            break;

        /* ---- ⑥ 按**任务码件号**取靶位（不再按颜色反查）---- */
        case SP_BIND:
        {
            /* ★★【2026-10-24 用户要求】**严格按任务码放置**：
               车载位 k 放的就是任务码第 1 批的**第 k-1 件**（TRAY PICK 按 i 顺序上盘 1→2→3），
               所以这件要放的靶位 = `place1[k-1]`，**与颜色无关**。
               原来"用绑定色去 color1[] 里反查下标"的做法，在**两种颜色相同时会取到前一个下标**
               —— 两件都放到同一个靶位、另一个靶位空着，现场就是"没按任务码放"。
               绑定色只用来**核对/告警**，不再参与选靶位。 */
            uint8_t idx   = (uint8_t)(spSlot - 1U);
            uint8_t color = Task_GetCarSlotColor(spSlot);
            uint8_t want  = Task_ColorOfBatch(spBatch, idx);
            uint8_t tgt   = Task_PlaceOfBatch(spBatch, idx);

            if (tgt == 0U)
            {
                Debug_Log("SP", "slot%u task code invalid (idx%u tgt=0)",
                          (unsigned int)spSlot, (unsigned int)idx);
                Sp_Fail("task code invalid");
                break;
            }

            if ((want != 0U) && (color != 0U) && (want != color))
            {
                /* 只告警，仍按任务码件号放（上料时已有颜色硬校验，正常不会不一致） */
                Debug_Log("SP", "WARN slot%u bound C%u != code C%u (idx%u) -> place by code",
                          (unsigned int)spSlot, (unsigned int)color,
                          (unsigned int)want, (unsigned int)idx);
            }

            spColor  = (want != 0U) ? want : color;
            spTgtPos = tgt;
            spArmOn  = 0U;
            Debug_Log("SP", "slot%u (item%u) color%u -> tgt%u [by task code]",
                      (unsigned int)spSlot, (unsigned int)(idx + 1U),
                      (unsigned int)spColor, (unsigned int)spTgtPos);
            Sp_GoTo(SP_PICK, (uint32_t)(TT_ARM_TIMEOUT_MS * 2U));  /* 合并动作：取+放，给双倍预算 */
            break;
        }

        /* ---- ⑦⑧ 车载取件 + 靶位放置（★【2026-10-24】合并成**一条连续动作**）----
           为什么要合并：用户要求"抓完 [抬升 ‖ 转向靶位 ‖ 舵盘回0] 同拍、
           放料 [伸爪 ‖ 下降] 同拍、放完 [抬升 ‖ 缩回] 同拍"——
           这些跨在原来的 CAR_PICK / TGT_PLACE **两个动作**之间，分开调用做不到。
           现在由 `ArmAct_RunCar2Tgt(slot, tgt, keepPose, area)` 一次跑完，结束后才做"记账 + 下一件"。
           ★【2026-10-31 用户要求】参数表按**区域**选（不是按批次）：
             `area = (spRough != 0) ? 1 : 2`
               · 1 = 粗加工区（ROUGH PLACE）-> `ARM_OP_TGT_PLACE`
               · 2 = 暂存区（STORE PLACE，第一批/第二批都是它）-> `ARM_OP_TGT_PLACE2`
           最后一传 `keepPose=1`：放完只松抓、不收爪不抬升（接 STORE BACK 的"直接抓"）。 */
        case SP_PICK:
            if (spArmOn == 0U)
            {
                /* ★【2026-10-23 用户要求】STORE PLACE **最后一个物料松抓就行、手臂不用上台**：
                   第 3 件（且不是 ROUGH PLACE）传 keepPose=1 -> 放完只松抓、不收爪不抬升，
                   手臂留在靶位取件姿态，正好接 STORE BACK 的"直接抓"。 */
                uint8_t keepPose = ((spRough == 0U) &&
                                    (spSlot == (uint8_t)SP_ITEM_COUNT)) ? 1U : 0U;
                uint8_t area     = (spRough != 0U) ? 1U : 2U;

                if (ArmAct_RunCar2Tgt(spSlot, spTgtPos, keepPose, area) != 0U)
                {
                    spArmOn = 1U;
                    Debug_Log("SP", "car slot%u -> tgt%u (C2T continuous, last=%u, %s)",
                              (unsigned int)spSlot, (unsigned int)spTgtPos,
                              (unsigned int)keepPose, (area == 2U) ? "STORE" : "ROUGH");
                }
                break;
            }
            if (ArmAct_IsBusy() != 0U)
            {
                break;
            }
            spArmOn = 0U;
            /* ★★【2026-10-24 用户要求】车载取件没夹到物料 -> **停下报 FAIL**，不往下走 */
            if (ArmAct_GripFailed() != 0U)
            {
                Sp_Fail("grip failed");
                break;
            }
            if (ArmAct_IsAborted() != 0U)
            {
                /* ★【2026-10-23 用户要求】"不要 FAIL，没达成就下一步"：
                   动作被中止也**当作本件处理完**，记录后继续下一件。 */
                Debug_Log("SP", "WARN car->tgt aborted -> continue (%u/%u)",
                          (unsigned int)spSlot, (unsigned int)SP_ITEM_COUNT);
            }

            Debug_Log("SP", "slot%u -> tgt%u done (%u/%u)",
                      (unsigned int)spSlot, (unsigned int)spTgtPos,
                      (unsigned int)spSlot, (unsigned int)SP_ITEM_COUNT);
            spSlot++;                   /* ★按任务码第 2 组的顺序：1 -> 2 -> 3 */
            if (spSlot > (uint8_t)SP_ITEM_COUNT)   /* 3 件搬完 */
            {
                /* ★【2026-10-11 用户要求】ROUGH PLACE：放完 3 件后，手臂回"车载
                   SP_ARM_IDLE_SLOT 号位、不抓"（先抬 M6 -> 收 S1 -> 转 M5 -> 张 S2）。
                   **STORE PLACE（spRough=0）不做这段**，保持"只松抓"以衔接 STORE BACK。 */
                if (spRough != 0U)
                {
                    spArmOn = 0U;
                    Sp_GoTo(SP_ARM_IDLE, (uint32_t)SP_ARM_IDLE_TIMEOUT_MS);
                    Debug_Log("SP", "3/3 placed -> arm back to car slot%u (idle, no grip)",
                              (unsigned int)SP_ARM_IDLE_SLOT);
                }
                else
                {
                    spState = SP_DONE;
                    Debug_Log("SP", "DONE: %u/%u placed",
                              (unsigned int)SP_ITEM_COUNT, (unsigned int)SP_ITEM_COUNT);
                }
            }
            else
            {
                spArmOn = 0U;
                Sp_GoTo(SP_BIND, (uint32_t)TT_ARM_TIMEOUT_MS);
            }
            break;

        /* ---- ⑨ ★ROUGH PLACE 收尾：手臂回车载号位、**不抓**（见 SP_SetRoughMode）---- */
        case SP_ARM_IDLE:
            if (spArmOn == 0U)
            {
                if (ArmAct_GotoCarIdle((uint8_t)SP_ARM_IDLE_SLOT) != 0U)
                {
                    spArmOn = 1U;
                    Debug_Log("SP", "arm -> car slot%u idle (no grip)",
                              (unsigned int)SP_ARM_IDLE_SLOT);
                }
                break;
            }
            /* ★【2026-10-15 用户要求】3 件已放完，只剩"回车载位不抓"这串多余动作
               （M6 抬高 -> S1 收回 -> M5 转向 -> S2 张开）。
               **每拍复核 + 严格档**（`ArmAct_S1Retracted()` 回读不可用返回 0）：
               动作还在半路、S1 可能还伸着，回读拿不到就**不放行**（退回"等整段跑完"），
               绝不带着伸出的爪子动车。 */
            /* ★【2026-10-21 规则3】与 TRAY PICK 同口径：**S1 已回 0** 或 **已进入电机5 转向**，
               两种情形都说明"关键作业已完成、剩下只是收臂"，底盘可以先走。
               严格档（`ArmAct_S1Retracted()` 回读不可用返回 0）仍然保留：
               拿不到回读时只能靠"确实转起来了"来放行，绝不带着伸出的爪子动车。 */
            if ((spTailOnly == 0U) &&
                ((ArmAct_S1Retracted() != 0U) || (ArmAct_IsTurning() != 0U)))
            {
                spTailOnly = 1U;
                Debug_Log("SP", "rough idle: %s -> tail only (chassis may start)",
                          (ArmAct_IsTurning() != 0U) ? "M5 turning" : "S1=0");
            }
            if (ArmAct_IsBusy() != 0U)
            {
                break;
            }
            spArmOn = 0U;
            if (ArmAct_IsAborted() != 0U)
            {
                Sp_Fail("arm idle aborted");
                break;
            }
            spState = SP_DONE;
            Debug_Log("SP", "DONE: %u/%u placed (arm idle at car slot%u)",
                      (unsigned int)SP_ITEM_COUNT, (unsigned int)SP_ITEM_COUNT,
                      (unsigned int)SP_ARM_IDLE_SLOT);
            break;

        default:
            break;
    }
}

uint8_t SP_IsBusy(void)
{
    return ((spState != SP_IDLE) && (spState != SP_DONE) &&
            (spState != SP_FAILED)) ? 1U : 0U;
}

uint8_t SP_IsDone(void)
{
    return (spState == SP_DONE) ? 1U : 0U;
}

uint8_t SP_IsFailed(void)
{
    return (spState == SP_FAILED) ? 1U : 0U;
}

/* ★【2026-10-15 用户要求】是否只剩"多余动作"（3 件已放完 + 收尾中 S1 已回 0）。
   只作"可以放行"的判据，**不改变 spState**，SP_Update 仍会把收尾动作跑完。 */
uint8_t SP_TailOnly(void)
{
    return spTailOnly;
}

const char *SP_GetStateText(void)
{
    static char line[24];

    switch (spState)
    {
        case SP_OBSERVE:
            return "SP arm observe";

        case SP_REQ_SHOT:
            return "SP req SHOT";

        case SP_WAIT_SHOT:
            return "SP wait SHOT";

        case SP_ALIGN:
        case SP_WAIT_ALIGN:
            return Align_GetStateText();        /* 对位详细进度交给它显示 */

        case SP_BIND:
            (void)snprintf(line, sizeof(line), "SP bind S%u", (unsigned int)spSlot);
            return line;

        case SP_PICK:
            (void)snprintf(line, sizeof(line), "SP S%u->T%u",
                           (unsigned int)spSlot, (unsigned int)spTgtPos);
            return line;

        case SP_ARM_IDLE:
            return "SP arm idle";

        case SP_DONE:
            return "SP DONE 3/3";

        case SP_FAILED:
            return "SP FAIL (see log)";

        default:
            return "SP idle";
    }
}

const char *SP_GetDataText(void)
{
    static char line[24];

    if (spState == SP_IDLE)
    {
        return "---";
    }
    (void)snprintf(line, sizeof(line), "S%u c%u t%u %u/3",
                   (unsigned int)spSlot, (unsigned int)spColor,
                   (unsigned int)spTgtPos, (unsigned int)(spSlot - 1U));
    return line;
}

/* ============================================================================
 *               ③ 打靶取回序列（**STORE BACK**，2026-10-10 实做）
 *   【做什么】**按 STORE PLACE 的逆序**，把已经放到靶位上的 3 件料**搬回车载 3 个号位**：
 *     STORE PLACE 是"车载 **3→2→1** 取件 -> 放到任务码指定的靶位"，
 *     所以逆序就是"**车载 1→2→3**：从它原先放的那个靶位抓回 -> 放回车载该号位"。
 *   【为什么不用视觉】STORE BACK 紧跟 STORE PLACE 执行（车已在打靶位对好），
 *     靶位号由"车载位绑定色 + 任务码第 1 组"直接算出，**不需要再 `$SHOT` 对位**。
 *   【第一件的特殊处理】用户要求"放完最后一件手臂**不上抬**、**直接抓**"：
 *     此时臂正停在**最后一次放置的那个靶位**姿态上，所以 `SB_Start(chained=1)` 时
 *     每件都传 `atPose=1`：`ArmAct_SbPick` 跳过"摆位"步骤，**直接抓、不再等 1s**
 *     （`SB_SETTLE_MS` 默认 0，2026-10-15 用户要求）。
 *     ⚠ **独立跑（chained=0）时 `atPose=0`** —— 臂可能停在任意姿态，
 *       先由 `flowSbPrep` 把手臂摆到该靶位取件姿态，避免在空中抓。
 *   【每件的收尾顺手带下一件（2026-10-23 用户要求）】`ArmAct_SbPlace(slot, nextTgt)`：
 *     松手后立刻"转向下一件 ‖ 舵盘回 0 -> 伸出 ‖ 下降"，下一件直接接着抓。
 *   【两个入口共用】任务调试页 `TDBG_STORE_BACK` / 赛题 `T_B1_PBACK`（`PLACE_BACK_ALL()`）。
 *   ⚠ 前提：先跑过 TRAY PICK（颜色↔车载位号绑定）+ STORE PLACE（料已在靶位上）。
 * ==========================================================================*/

typedef enum {
    SB_IDLE = 0,
    SB_SETTLE,        /* ① 等 SB_SETTLE_MS（只 chained 时；默认 0 = 不停，直接抓） */
    SB_BIND,          /* ② 读车载位绑定色 -> 任务码第 1 组**反查靶位** */
    SB_PICK,          /* ③ 从靶位取件（ArmAct_SbPick） */
    SB_PLACE,         /* ④ 放回车载该号位（ArmAct_SbPlace） */
    SB_DONE,          /* 3 件全部放回 */
    SB_FAILED         /* 失败中止 */
} SB_State_t;

static SB_State_t sbState   = SB_IDLE;
static uint8_t    sbSlot    = 0U;   /* 当前在搬的车载位号（3 -> 2 -> 1） */
static uint8_t    sbTgtPos  = 0U;   /* 本件从哪个靶位取回 */
static uint8_t    sbNextTgt = 0U;   /* ★下一件的靶位号（0=本件是最后一件）：
                                       STORE BACK 的"放完就顺手摆好下一件"要用它 */
static uint8_t    sbColor   = 0U;   /* 本件车载位绑定的颜色（显示用） */
static uint8_t    sbChained = 0U;   /* 1=紧跟 STORE PLACE 执行（手臂已在该靶位取件姿态） */
/* ★【2026-10-23】本段按**任务码第几批**取颜色/靶位号（1=第1/2组；2=第3/4组）。 */
static uint8_t    sbBatch   = 1U;
static uint8_t    sbArmOn   = 0U;   /* 1=本段机械臂动作已启动 */
static uint8_t    sbCount   = 0U;   /* 已放回几件（0~3） */
static uint8_t    sbTailOnly = 0U;  /* ★【2026-10-15】1=最后一件已从靶位抓到手里、S1 已回 0，
                                       只剩"放回车载"这类多余动作 -> 可让底盘先动 */
static uint8_t    sbTailCand = 0U;  /* ★1=最后一件已到手（"只剩收尾"候选），
                                       还要在 SB_PLACE 每拍复核 S1 才置 sbTailOnly */
static uint32_t   sbStateTick = 0U;
static uint32_t   sbLimitMs   = 0U;

static void Sb_GoTo(SB_State_t st, uint32_t limitMs)
{
    sbState     = st;
    sbStateTick = HAL_GetTick();
    sbLimitMs   = limitMs;
}

static uint8_t Sb_Timeout(void)
{
    if (sbLimitMs == 0U)
    {
        return 0U;
    }
    return ((HAL_GetTick() - sbStateTick) >= sbLimitMs) ? 1U : 0U;
}

static void Sb_Fail(const char *why)
{
    ArmAct_Abort();
    sbState = SB_FAILED;
    Debug_Log("SB", "FAIL: %s (slot %u/3)", why, (unsigned int)sbSlot);
}

void SB_Init(void)
{
    sbState     = SB_IDLE;
    sbSlot      = 0U;
    sbTgtPos    = 0U;
    sbNextTgt   = 0U;
    sbColor     = 0U;
    sbChained   = 0U;
    sbArmOn     = 0U;
    sbCount     = 0U;
    sbTailOnly  = 0U;
    sbTailCand  = 0U;
    sbLimitMs   = 0U;
    Debug_Log("SB", "init (STORE BACK 打靶取回序列)");
}

void SB_SetBatch(uint8_t batch)
{
    sbBatch = (batch == 2U) ? 2U : 1U;
}

uint8_t SB_Start(uint8_t chained)
{
    if (SB_IsBusy() != 0U)
    {
        return 0U;
    }
    if ((Align_IsBusy() != 0U) || (ArmAct_IsBusy() != 0U))
    {
        return 0U;                  /* 对位或机械臂正忙：不抢 */
    }

    sbSlot    = (uint8_t)SB_ITEM_COUNT;   /* ★【2026-10-24】**放置的逆序**：STORE PLACE 现在是 1->2->3，
                                             所以取回是 **3 -> 2 -> 1**（第一件 = 最后放上去的那件） */
    sbTgtPos  = 0U;
    sbNextTgt = 0U;
    sbColor   = 0U;
    sbChained = (chained != 0U) ? 1U : 0U;
    sbArmOn   = 0U;
    sbCount   = 0U;
    sbTailOnly = 0U;
    sbTailCand = 0U;

    Debug_Log("SB", "start: back x%u (slot 3->2->1) chained=%u",
              (unsigned int)SB_ITEM_COUNT, (unsigned int)sbChained);

    if (sbChained != 0U)
    {
        /* ① 先静置（外层兜底给足余量） */
        Sb_GoTo(SB_SETTLE, (uint32_t)SB_SETTLE_MS + 2000U);
    }
    else
    {
        Sb_GoTo(SB_BIND, (uint32_t)SB_ARM_TIMEOUT_MS);
    }
    return 1U;
}

void SB_Abort(void)
{
    if (sbState == SB_IDLE)
    {
        return;
    }
    ArmAct_Abort();
    sbState = SB_IDLE;
    Debug_Log("SB", "aborted by user (slot %u/3)", (unsigned int)sbSlot);
}

void SB_Update(void)
{
    if ((sbState == SB_IDLE) || (sbState == SB_DONE) || (sbState == SB_FAILED))
    {
        return;
    }

    if (Sb_Timeout() != 0U)
    {
        Sb_Fail("state timeout");
        return;
    }

    switch (sbState)
    {
        /* ---- ① 静置（只 chained；对应"放完最后一件不上抬、直接抓"）
                ★【2026-10-15 用户要求】不再等 1s：`SB_SETTLE_MS` 默认 0 ->
                本态直接放行，**上个任务点放置完就马上抓**。
                （`SB_SETTLE_MS` 若改回 >0，这里仍按原逻辑静置那么久。） ---- */
        case SB_SETTLE:
#if (SB_SETTLE_MS > 0U)
            if ((HAL_GetTick() - sbStateTick) < (uint32_t)SB_SETTLE_MS)
            {
                break;                  /* 还在静置 */
            }
#endif
            Debug_Log("SB", "start -> pick tgt (no wait, no raise, no turn)");
            sbArmOn = 0U;
            Sb_GoTo(SB_BIND, (uint32_t)SB_ARM_TIMEOUT_MS);
            break;

        /* ---- ② 按**任务码件号**取回靶位（与 SP_BIND 同一口径，不再按颜色反查）---- */
        case SB_BIND:
        {
            /* ★【2026-10-24】车载位 k 装的是任务码第 1 批的**第 k-1 件**，
               它当初被放在 `place1[k-1]` -> 现在就从那个靶位取回。
               （颜色重复时，"按颜色反查"会取到前一个下标，导致取错靶位。） */
            uint8_t idx   = (uint8_t)(sbSlot - 1U);
            uint8_t color = Task_GetCarSlotColor(sbSlot);
            uint8_t want  = Task_ColorOfBatch(sbBatch, idx);
            uint8_t tgt   = Task_PlaceOfBatch(sbBatch, idx);

            if (tgt == 0U)
            {
                Debug_Log("SB", "slot%u task code invalid (idx%u tgt=0)",
                          (unsigned int)sbSlot, (unsigned int)idx);
                Sb_Fail("task code invalid");
                break;
            }

            if ((want != 0U) && (color != 0U) && (want != color))
            {
                Debug_Log("SB", "WARN slot%u bound C%u != code C%u (idx%u) -> pick by code",
                          (unsigned int)sbSlot, (unsigned int)color,
                          (unsigned int)want, (unsigned int)idx);
            }

            sbColor  = (want != 0U) ? want : color;
            sbTgtPos = tgt;
            sbArmOn  = 0U;

            /* ★下一件（车载位 sbSlot-1）的靶位号：本件放完就顺手"转过去 + 伸出 + 下降"，
               所以现在就要算出来；sbSlot==1（最后一件）填 0 = 没有下一件。 */
            sbNextTgt = 0U;
            if (sbSlot > 1U)
            {
                /* ★【2026-10-24 修复】原来写死 batch=1，B2 取回时"下一件预转靶位"全按
                   第 1 批算 -> 手臂被预转到错误靶位，下一件直接抓错（现场：两次都抓
                   一号靶位）。必须用本批 sbBatch。 */
                sbNextTgt = Task_PlaceOfBatch(sbBatch, (uint8_t)(sbSlot - 2U));
            }

            Debug_Log("SB", "slot%u (item%u) color%u -> tgt%u next%u [by task code] (chained=%u)",
                      (unsigned int)sbSlot, (unsigned int)(idx + 1U),
                      (unsigned int)sbColor, (unsigned int)sbTgtPos,
                      (unsigned int)sbNextTgt, (unsigned int)sbChained);
            Sb_GoTo(SB_PICK, (uint32_t)SB_ARM_TIMEOUT_MS);
            break;
        }

        /* ---- ③ 从靶位取件 ---- */
        case SB_PICK:
            if (sbArmOn == 0U)
            {
                if (ArmAct_SbPick(sbTgtPos, sbSlot, sbChained) != 0U)
                {
                    sbArmOn = 1U;
                    Debug_Log("SB", "pick tgt%u -> slot%u", (unsigned int)sbTgtPos,
                              (unsigned int)sbSlot);
                }
                break;                  /* 启动不了：下一拍再试（状态超时兜底） */
            }
            if (ArmAct_IsBusy() != 0U)
            {
                break;
            }
            sbArmOn = 0U;
            /* ★★【2026-10-24 用户要求】靶位取件没夹到物料 -> **停下报 FAIL**，不往下走 */
            if (ArmAct_GripFailed() != 0U)
            {
                Sb_Fail("grip failed");
                break;
            }
            if (ArmAct_IsAborted() != 0U)
            {
                /* ★【2026-10-23 用户要求】"不要 FAIL，没达成就下一步"：
                   靶位取件被中止也**继续走放回流程**（不再整段失败）。 */
                Debug_Log("SB", "WARN tgt pick aborted -> continue to place (slot%u)",
                          (unsigned int)sbSlot);
            }

            /* ★【2026-10-15 用户要求·这正是用户举的例子】最后一件（第 3 件）已从靶位
               抓到手里 = **"关键作业已完成"**。这里**只记"候选"**，放行判据（S1 已回 0）
               留到 `SB_PLACE` **每拍复核**（原因同 TT_：只判一次会被
               "步完成容差 80 vs 放行门限 20"漏掉）。 */
            if ((SB_TAIL_EARLY_EN != 0U) &&
                ((uint8_t)(sbCount + 1U) >= (uint8_t)SB_ITEM_COUNT) && (sbTailCand == 0U))
            {
                sbTailCand = 1U;
                Debug_Log("SB", "last item in hand -> tail candidate (wait S1=0)");
            }

            Sb_GoTo(SB_PLACE, (uint32_t)SB_ARM_TIMEOUT_MS);
            break;

        /* ---- ④ 放回车载该号位 -> 下一件（**逆序 3 -> 2 -> 1**）---- */
        case SB_PLACE:
            /* ★【2026-10-15 用户要求】**每拍复核**"只剩收尾 + S1 已回 0"（**严格档**：
               回读不可用不放行）。通过了才置 `sbTailOnly` -> 任务点可让底盘先动
               （`T_B1_STO_GO`），手臂并行去"放回车载号位"。
               复核窗口 = 本状态全程：`flowSbPlace` 开头有 rule3 自动插的"收爪"步，
               S1 会在 0 停留到"伸爪"那一步，足够复核上（周期 100ms）。 */
            if ((SB_TAIL_EARLY_EN != 0U) && (sbTailCand != 0U) && (sbTailOnly == 0U) &&
                (ArmAct_GripHeld() != 0U) && (ArmAct_LiftAboveSafe() != 0U) &&
                (ArmAct_S1Retracted() != 0U))
            {
                sbTailOnly = 1U;
                Debug_Log("SB", "tail only: grip held + lifted + S1=0 -> chassis may start");
            }

            if (sbArmOn == 0U)
            {
                if (ArmAct_SbPlace(sbSlot, sbNextTgt) != 0U)
                {
                    sbArmOn = 1U;
                    Debug_Log("SB", "put back slot%u (from tgt%u, next tgt%u)",
                              (unsigned int)sbSlot, (unsigned int)sbTgtPos,
                              (unsigned int)sbNextTgt);
                }
                break;
            }
            if (ArmAct_IsBusy() != 0U)
            {
                break;
            }
            sbArmOn = 0U;
            if (ArmAct_IsAborted() != 0U)
            {
                /* ★【2026-10-23 用户要求】"不要 FAIL，没达成就下一步"：
                   放回车载被中止也**照常记账、继续下一件**。 */
                Debug_Log("SB", "WARN car place aborted -> continue (slot%u)",
                          (unsigned int)sbSlot);
            }

            sbCount++;
            Debug_Log("SB", "slot%u <- tgt%u done (%u/%u)",
                      (unsigned int)sbSlot, (unsigned int)sbTgtPos,
                      (unsigned int)sbCount, (unsigned int)SB_ITEM_COUNT);

            if (sbCount >= (uint8_t)SB_ITEM_COUNT)
            {
                sbState = SB_DONE;
                Debug_Log("SB", "DONE: %u/%u back to car",
                          (unsigned int)SB_ITEM_COUNT, (unsigned int)SB_ITEM_COUNT);
            }
            else
            {
                sbSlot--;               /* ★逆序：放置 1->2->3，取回 3 -> 2 -> 1 */
                sbArmOn = 0U;
                Sb_GoTo(SB_BIND, (uint32_t)SB_ARM_TIMEOUT_MS);
            }
            break;

        default:
            break;
    }
}

uint8_t SB_IsBusy(void)
{
    return ((sbState != SB_IDLE) && (sbState != SB_DONE) &&
            (sbState != SB_FAILED)) ? 1U : 0U;
}

uint8_t SB_IsDone(void)
{
    return (sbState == SB_DONE) ? 1U : 0U;
}

uint8_t SB_IsFailed(void)
{
    return (sbState == SB_FAILED) ? 1U : 0U;
}

/* ★【2026-10-15 用户要求】是否只剩"多余动作"（最后一件已到手 + S1 已回 0）。
   只作"可以放行"的判据，**不改变 sbState**，SB_Update 仍会把"放回车载"跑完。 */
uint8_t SB_TailOnly(void)
{
    /* ★【2026-11-02 用户要求】`SB_TAIL_EARLY_EN = 0` 时恒不放行（等料真正放上车载盘） */
    if ((SB_TAIL_EARLY_EN == 0U) || (sbTailOnly == 0U))
    {
        return 0U;
    }
    return 1U;
}

const char *SB_GetStateText(void)
{
    static char buf[20];

    switch (sbState)
    {
        case SB_IDLE:   return "SB idle";
        case SB_SETTLE: return "SB start";
        case SB_BIND:   return "SB bind";
        case SB_DONE:   return "SB DONE 3/3";
        case SB_FAILED: return "SB FAIL";
        default:        break;
    }

    if (sbState == SB_PICK)
    {
        (void)snprintf(buf, sizeof(buf), "SB pick t%u", (unsigned int)sbTgtPos);
    }
    else
    {
        (void)snprintf(buf, sizeof(buf), "SB put s%u", (unsigned int)sbSlot);
    }
    return buf;
}

const char *SB_GetDataText(void)
{
    static char line[24];

    if (sbState == SB_IDLE)
    {
        return "---";
    }
    (void)snprintf(line, sizeof(line), "S%u c%u t%u %u/3",
                   (unsigned int)sbSlot, (unsigned int)sbColor,
                   (unsigned int)sbTgtPos, (unsigned int)sbCount);
    return line;
}

