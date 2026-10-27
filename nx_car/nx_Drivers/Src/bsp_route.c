/**
 ******************************************************************************
 * @file    bsp_route.c
 * @brief   跑图执行器实现（P7：按路段表顺序自动行驶 + 到位判定）
 * @author  nx_car
 * @date    2026-09-17
 ******************************************************************************
 * @note   非阻塞状态机，无 HAL_Delay。
 *         运动复用 P4 的位置模式（Mecanum_MoveBy / Mecanum_MoveByRot），
 *         到位优先等电机主动回传（Motor_ReachOk），超时退脉冲容差兜底。
 *         位置读取只解析接收缓冲（查询由 Motor_PeriodicQuery 统一负责），
 *         并按 MOTOR_READ_SCALE_NUM/DEN 换算到下发脉冲域（与 P4 一致）。
 *         日志统一 [ROUTE] 标签，禁 %f。
 ******************************************************************************
 */
#include "bsp_route.h"
#include "bsp_motor.h"
#include "bsp_mecanum.h"
#include "bsp_chassis.h"
#include "bsp_imu.h"        /* IMU_IsOnline：日志里标注朝向数据是否新鲜，便于排查 */
#include "bsp_oled_ui.h"
#include "bsp_debug.h"
#include <math.h>
#include <stdio.h>

/* ===================== 路段表（由 X-macro 展开生成数组） ===================== */

/* 单个路段的内容 */
typedef struct {
    int16_t turn_deg;   /* 本段先原地旋转的角度，单位 deg，正=逆时针，0=不转 */
    int32_t x;          /* 本段目标点 x，单位 mm（相对跑图起点），ROUTE_NO_MOVE=无位移 */
    int32_t y;          /* 本段目标点 y，单位 mm（相对跑图起点），ROUTE_NO_MOVE=无位移 */
} RouteSeg_t;

/* 把 ROUTE_SEG_LIST 的每一行 ROUTE_SEG(a,b,c) 展开成结构体初始化项 */
#define ROUTE_SEG(a, b, c)   { (a), (b), (c) },
static const RouteSeg_t routeTable[] = { ROUTE_SEG_LIST };
#undef ROUTE_SEG

/* 总路段数（自动计算，增删路段不用改这里） */
#define ROUTE_SEG_COUNT     ((uint8_t)(sizeof(routeTable) / sizeof(routeTable[0])))

/* ---- 运行期可调参数（在线调参用，初值 = bsp_route.h 里的宏，见 Route_Init） ---- */
static RouteCfg_t routeCfg;

/* ---- 运行期"缩放后的路段表"（tableScalePct != 100 时用它，调车用小圈） ---- */
static RouteSeg_t routeCfgTable[ROUTE_SEG_COUNT];
static const RouteSeg_t *routeTbl = routeTable;   /* 实际使用的表（默认就是上面的常量表） */

/* ===================== 运行期可调参数：默认值 ===================== */
/* 【思路】把 bsp_route.h 里的编译期默认值先存到 ROUTE_DEF_*，供 routeCfgDefault 取快照。
 *   在线调参（bsp_tune）与 PC 自动调车脚本直接改 routeCfg，业务代码按字段读。 */
#define ROUTE_DEF_ROT_SCALE_NUM    100U
#define ROUTE_DEF_ROT_SCALE_DEN    100U
#define ROUTE_DEF_DIST_SCALE_NUM   100U
#define ROUTE_DEF_DIST_SCALE_DEN   100U
#define ROUTE_DEF_TABLE_SCALE      100U
#define ROUTE_DEF_ROT_VEL          MEC_ROT_VEL
#define ROUTE_DEF_HOLD_MS          30U     /* 【2026-10-15 提速】100 -> 30：见 bsp_route.h
                                              ROUTE_DEF_HOLD_MS 说明。只压缩"停稳死等"，
                                              不改变"四轮同拍瞬停(0xFE)"本身。 */
#define ROUTE_DEF_YAW_ENABLE       1U
/* 2026-10-07 retune: "one shot, no waste" (see tuning manual P9 / route chapter) */
#define ROUTE_DEF_YAW_TOL          0U      /* 2026-10-07: user asked for tolerance = 0
                                              (keep compensating until the compensation itself
                                              gets tiny). NOTE: the "tiny" guard in
                                              Route_YawLoop / Ms_YawLoop still releases once the
                                              residual is under 1 deg, so the effective stop is
                                              "<1 deg", not literally 0. */
#define ROUTE_DEF_YAW_CORR_PCT     100U    /* was 70: compensate the FULL error in one round */
#define ROUTE_DEF_YAW_MAXCORR_X10  150U
#define ROUTE_DEF_YAW_MAX_ROUND    3U      /* was 2: allow one more fixup round if overshoot */
#define ROUTE_DEF_YAW_BUDGET_MS    1500U   /* was 1200 ms: enough for up to 3 rounds */

/* 默认值快照。字段顺序必须与 bsp_route.h 的 RouteCfg_t 完全一致（位置初始化）。
 * 用途：① Route_Init 用它初始化；② 在线调参的 `DEFAULT` 命令一键恢复出厂值。 */
static const RouteCfg_t routeCfgDefault = {
    ROUTE_DEF_ROT_SCALE_NUM,    /* rotScaleNum      */
    ROUTE_DEF_ROT_SCALE_DEN,    /* rotScaleDen      */
    ROUTE_DEF_ROT_VEL,          /* rotVel           */
    ROUTE_DEF_DIST_SCALE_NUM,   /* distScaleNum     */
    ROUTE_DEF_DIST_SCALE_DEN,   /* distScaleDen     */
    ROUTE_DEF_TABLE_SCALE,      /* tableScalePct    */
    0U,                         /* lapRepeat = 只跑一次 */
    ROUTE_DEF_HOLD_MS,          /* holdMs = 到位后停稳 30ms（2026-10-15 by 100） */
    ROUTE_DEF_YAW_ENABLE,       /* yawEnable = 开 */
    ROUTE_DEF_YAW_TOL,          /* yawTolDeg = 1 deg */
    ROUTE_DEF_YAW_CORR_PCT,     /* yawCorrPct = 只补 70% */
    ROUTE_DEF_YAW_MAXCORR_X10,  /* yawMaxCorrX10 = 15° */
    ROUTE_DEF_YAW_MAX_ROUND,    /* yawMaxRound = 最多补 2 轮 */
    ROUTE_DEF_YAW_BUDGET_MS     /* yawBudgetMs = 1200ms */
};

/* ===================== 内部状态 ===================== */

static RouteState_t routeState = ROUTE_STATE_IDLE;   /* 跑图总状态 */
static RoutePhase_t routePhase = ROUTE_PHASE_WAIT;   /* 当前段所处阶段 */
static uint8_t      routePosRound = 0U;              /* [2026-10-07] landing-point loop rounds used in this segment */

static uint8_t  routeSegIdx      = 0U;    /* 当前路段号（0 起） */
static uint8_t  routeLapIdx      = 1U;    /* 当前第几圈（1 起，PC 自动调参按圈统计用） */
static uint8_t  routeLapTotal    = 1U;    /* 本次要跑的圈数（= routeCfg.lapRepeat 换算而来） */
static uint8_t  routePhaseSent   = 0U;    /* 本阶段命令是否已下发：0=未发，1=已发等待到位 */
static uint8_t  routeFallbackLogged = 0U; /* 本阶段是否已打印"回传超时退脉冲"提示 */
static uint8_t  routeBaseValid   = 0U;    /* 脉冲兜底基准是否有效（读位置成功） */

/* ---- 到位后的"统一停轮 + 停稳"（见 RouteCfg_t.holdMs 说明） ---- */
static uint8_t  routeHoldActive  = 0U;    /* 1=四轮已同拍停住，正在静置等残余运动结束 */
static uint32_t routeHoldTick    = 0U;    /* 开始静置的时刻 */

/* ---- 转角闭环（按 HWT101CT 回传的 θ 补转，见 RouteCfg_t 的 yaw* 字段） ---- */
static float    routeYawTarget   = 0.0f;  /* 本段旋转结束时应达到的**绝对 θ**（deg） */
static uint8_t  routeYawRound    = 0U;    /* 本段已补转轮次 */
static uint32_t routeYawStart    = 0U;    /* 本段转角闭环开始时刻（预算计时） */
static int32_t  routeYawErrX10   = 0;     /* 最近一次角度残差（0.1°，日志/页面用） */

static int32_t  routeSegBase[MOTOR_NUM];  /* 本阶段起点四轮位置（下发脉冲域） */
static int32_t  routeSegClk[MOTOR_NUM];   /* 本阶段目标增量脉冲（判到位用） */

static uint32_t routeSegStartTick = 0U;   /* 本段开始时刻（段超时计时） */
static uint32_t routePhaseTick    = 0U;   /* 本阶段下发时刻（回传超时计时） */
static float    routeTurnYaw0     = 0.0f; /* 本段旋转**下发前**的 θ（用于量实测转角） */
static float    routeTurnRaw0     = 0.0f; /* 本段旋转**下发前**的 IMU 原始偏航角（诊断用） */

/* ---- 转弯后的"朝向稳定等待"（见 ROUTE_YAW_SETTLE_MS 说明） ---- */
static uint8_t  routeSettleActive = 0U;   /* 1=正在等朝向稳定（本段平移先不下发） */
static float    routeSettleYaw    = 0.0f; /* 上次观察到的朝向 */
static float    routeSettleNeed   = 0.0f; /* 本段期望的 θ 变化量（= 表里转角） */
static uint32_t routeSettleTick   = 0U;   /* 朝向最近一次"变了"的时刻 */
static uint32_t routeSettleStart  = 0U;   /* 开始等待的时刻（硬上限计时） */

/* 【2026-10-15 提速】静止零漂测量窗口"每圈只强制一次"的标志：
 *   0 = 本圈还没测过（第一次转弯要停满 ROUTE_DRIFT_CAL_MS）；
 *   1 = 本圈已测过（后续转弯不再为它停，衔接时间只留 holdMs）。
 *   一圈测一次足够（零漂是缓变量），见 bsp_route.h ROUTE_DRIFT_CAL_MS 说明。 */
static uint8_t  routeDriftCalUsed = 0U;

/* ---- velocity-mode (pure gyro) state, see ROUTE_VEL_MODE ---- */
#if (ROUTE_VEL_MODE != 0U)
static uint32_t rvTick       = 0U;    /* last velocity-loop tick */
static uint32_t rvPhaseStart = 0U;    /* phase start (profile time base) */
static uint32_t rvCtrlTick   = 0U;    /* last control tick (for D term) */
static uint32_t rvOkTick     = 0U;    /* time the phase first reached tolerance */
static float    rvYawTarget  = 0.0f;  /* hold / target heading (deg) */
static float    rvYawPlan    = 0.0f;  /* ABSOLUTE planned heading of the current segment
                                         (= sum of the table turn_deg, CCW +, wrapped).
                                         Turn AND straight hold this angle, so a turn error
                                         never shifts the heading of the following legs. */
static float    rvYawPrev    = 0.0f;  /* previous measured yaw (rate estimate) */
static float    rvRateF      = 0.0f;  /* low-pass filtered yaw rate (deg/s, CCW +) */
static float    rvYaw0Raw    = 0.0f;  /* RAW IMU yaw at the route origin (lap start) */
static uint8_t  rvHomeActive = 0U;    /* 1 = final (0,0) approach running */
static uint8_t  rvHomeDone   = 0U;    /* 1 = the final approach has been performed */
static uint32_t rvHomeTick   = 0U;    /* final approach start time */
static float    rvDirF       = 0.0f;  /* unit direction (body frame) */
static float    rvDirR       = 0.0f;
static float    rvDist       = 0.0f;  /* segment distance after scale (mm) */
static float    rvPeak       = 0.0f;  /* profile peak speed (mm/s) */
static float    rvTAcc       = 0.0f;  /* accel time (s) */
static float    rvTDecel     = 0.0f;  /* decel time (s) */
static float    rvTCruise    = 0.0f;  /* cruise time (s) */
static float    rvVStart     = 0.0f;  /* profile start speed (0 or corner speed) */
static float    rvVEnd       = 0.0f;  /* profile end speed (0 or corner speed) */
static float    rvCornerSpeed = 0.0f; /* speed handed over to the next segment */
static float    rvTTotal     = 0.0f;  /* total profile time (s) */
static uint8_t  rvMoving     = 0U;    /* 0=phase not issued yet, 1=running */
static uint8_t  rvBlend      = 0U;    /* 1=rotating for the NEXT corner while translating */
static uint8_t  rvSkipTurn   = 0U;    /* next segment's in-place TURN is already done */
static uint8_t  rvKeepYaw    = 0U;    /* next move keeps rvYawTarget (finish the corner) */
static uint8_t  rvFromCorner = 0U;    /* next move starts at corner speed (not 0) */
static float    rvBlendYaw0  = 0.0f;  /* yaw when the corner blend started */
static float    rvBlendTurn = 0.0f;   /* turn angle being blended (deg) */
static uint32_t rvBlendT0   = 0U;     /* blend start tick */
static uint32_t rvBlendDur  = 0U;     /* blend duration (ms), = remaining segment time */

/* Segmented mode: "the straight leg must REALLY be stopped before turning".
   We sample the four wheel positions during the stop and only advance once the
   wheels have not moved for ROUTE_VEL_STOP_HOLD_MS (or the hard cap expires). */
static int32_t  rvStopPos[MOTOR_NUM];
static uint8_t  rvStopValid = 0U;     /* 1 = rvStopPos holds a valid sample */
static uint32_t rvStopTick  = 0U;     /* time the wheels were first seen still */
static uint32_t rvHoldLogTick = 0U;   /* last "yaw hold" log time (straight legs) */
#endif  /* ROUTE_VEL_MODE */

/* ===================== 内部工具 ===================== */

/**
 * @brief  读四轮当前位置（换算到下发脉冲域）
 * @param  out: 输出四轮位置（FL/FR/RL/RR），下标 0~3
 * @retval 1=四轮全部读到；0=有任一读不到
 * @note   与 P4 bsp_chassis 的 Chassis_ReadPos 做法一致（该函数是 static 不可复用）：
 *         0x36 上报的是编码器计数，比下发脉冲大 MOTOR_READ_SCALE_DEN/NUM 倍，
 *         这里统一换算到下发脉冲域，才能与 Mecanum_MoveBy 的目标脉冲直接比较。
 */
static uint8_t Route_ReadPos(int32_t out[MOTOR_NUM])
{
    int32_t raw;
    uint8_t i;

    for (i = 0; i < MOTOR_NUM; i++)
    {
        if (Motor_GetAngle_x10deg((uint8_t)(MOTOR_ADDR_MIN + i), &raw) != 0)
        {
            return 0U;
        }

        out[i] = (raw * (int32_t)MOTOR_READ_SCALE_NUM) / (int32_t)MOTOR_READ_SCALE_DEN;
    }

    return 1U;
}

/**
 * @brief  清除四轮"到位回传"标志
 * @param  无
 * @retval 无
 * @note   **必须在下发本阶段命令之前调用**：否则上一阶段的到位标志会被当成
 *         "本阶段已到位"，出现"刚下发就判到位、实际没走"。
 */
static void Route_ReachClearAll(void)
{
    uint8_t addr;

    for (addr = MOTOR_ADDR_MIN; addr <= MOTOR_ADDR_MAX; addr++)
    {
        Motor_ReachClear(addr);
    }
}

/**
 * @brief  本次下发了命令的轮是否都已回传"到位"
 * @param  无
 * @retval 1=所有**有命令的轮**都到位；0=还有没到位的
 * @note   位移为 0 的轮（例如 45° 斜行只用两个轮、某些合成运动）**不会收到命令**，
 *         自然也不会有到位帧 —— 必须把它们排除，否则每段都要白等到
 *         ROUTE_REACH_FALLBACK_MS 才靠脉冲兜底过去（现象：动一下要停好几秒）。
 */
static uint8_t Route_ReachAll(void)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t)MOTOR_NUM; i++)
    {
        if (routeSegClk[i] == 0)
        {
            continue;               /* 本阶段没给这个轮下命令 */
        }
        if (Motor_ReachOk((uint8_t)(MOTOR_ADDR_MIN + i)) == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief  脉冲容差判到位（兜底判据）
 * @param  无
 * @retval 1=四轮"已走脉冲 − 目标脉冲"都小于 ROUTE_PULSE_TOL；0=未到位
 * @note   读不到位置时返回 0（等下一拍再判），不用旧值蒙。
 */
static uint8_t Route_PulseArrived(void)
{
    int32_t cur[MOTOR_NUM];
    int32_t err;
    uint8_t i;

    if (Route_ReadPos(cur) == 0U)
    {
        return 0U;
    }

    for (i = 0; i < MOTOR_NUM; i++)
    {
        if (routeSegClk[i] == 0)
        {
            continue;               /* 本阶段没给这个轮下命令，不参与判定 */
        }
        err = (cur[i] - routeSegBase[i]) - routeSegClk[i];
        if (err < 0)
        {
            err = -err;
        }
        if (err > (int32_t)ROUTE_PULSE_TOL)
        {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief  本阶段是否已到位
 * @param  无
 * @retval 1=已到位；0=还没到
 * @note   优先用电机主动回传的到位帧（电机自己的判据，窗口仅 0.1~0.3°，最可靠）；
 *         下发后超过 ROUTE_REACH_FALLBACK_MS 还没收齐，就退回脉冲容差判定并
 *         打印一次提示（防漏帧/菜单没配 Reached 时挂死）。
 *         注意：本函数只管当前**阶段**（旋转或平移）是否到位。
 */
static uint8_t Route_PhaseArrived(void)
{
    if (Route_ReachAll() != 0U)
    {
        return 1U;
    }

    if ((HAL_GetTick() - routePhaseTick) < ROUTE_REACH_FALLBACK_MS)
    {
        return 0U;
    }

    if (routeFallbackLogged == 0U)
    {
        routeFallbackLogged = 1U;
        Debug_Log("ROUTE", "reach timeout, fallback to pulse");
    }

    if (routeBaseValid == 0U)
    {
        /* 连脉冲基准都没有：只能继续等回传，最终由段超时兜底 */
        return 0U;
    }

    if (Route_PulseArrived() != 0U)
    {
        return 1U;
    }

    return 0U;
}

/**
 * @brief  到位后的"四轮同拍瞬停 + 静置等待"（治"减速区末端横移/走偏"）
 * @param  无
 * @retval 1=静置结束，**可以推进下一段**；0=还在静置（本拍不要推进）
 * @note   【为什么要它】四个轮子是**各自独立的闭环位置模式**，脉冲数不同
 *         （斜行、或车在横向上有偏差时，四轮脉冲就不相等）到位时刻也就不同：
 *         先到位的轮停下并保持，后到位的轮还在转 —— 在麦轮上这会被分解成
 *         "整车横移 + 微转"，现场表现就是**直行到点位的减速段出现微小横移**，
 *         最后没能正好停在点位上。
 *         本函数在**判定到位的那一刻**给四轮同拍发 0xFE 瞬停（把还在收尾/被推着
 *         走的轮子一起按住），再静置 `routeCfg.holdMs`(默认 100ms) 等残余运动
 *         彻底结束，然后才允许进入下一段 —— 也顺带避免"上一段的余动和下一段的
 *         指令叠在一起"。
 *         【为什么不会截掉距离】停轮发生在"**所有有命令的轮**都已回传到位"
 *         （或脉冲进容差）之后；电机的到位窗口只有 0.1~0.3°（≈0.3mm 轮面位移），
 *         所以最多截掉不到 1mm，可忽略；真正被消掉的是"互相推挤"那部分。
 *         `holdMs = 0` 表示"仍然同拍停四轮，但不额外等待"（下一拍就进下一段）。
 */
static uint8_t Route_StopAndSettle(void)
{
    if (routeHoldActive == 0U)
    {
        uint8_t addr;

        for (addr = MOTOR_ADDR_MIN; addr <= MOTOR_ADDR_MAX; addr++)
        {
            Motor_Stop(addr);           /* 0xFE 同拍瞬停（内部已等空闲发送邮箱） */
        }
        routeHoldActive = 1U;
        routeHoldTick   = HAL_GetTick();
        return 0U;
    }

    if ((HAL_GetTick() - routeHoldTick) < (uint32_t)routeCfg.holdMs)
    {
        return 0U;
    }

    routeHoldActive = 0U;
    return 1U;
}

/**
 * @brief  阶段完成时打印一行证据日志（段落排错用）
 * @param  无
 * @retval 无
 * @note   一并给出两个掩码，用来判断"轮子不动"到底是谁的问题：
 *           `wheels=` 本次**被下过命令**的轮（1=应转动、0=位移为 0，正常不动）；
 *           `reach=`  **主动回传"到位"**的轮（bit0~3 = FL/FR/RL/RR）。
 *         典型判读：
 *           - `wheels=0011` → 本段是斜行，另两轮本来就不该动（正常）；
 *           - `wheels=1111 reach=F` → 四轮都下发了、都回传到位（正常）；
 *           - `wheels=1111 reach=7` → 有一轮没回传（该轮可能丢帧/被忽略，随后走了脉冲兜底）。
 */
static void Route_LogPhaseDone(void)
{
    uint8_t  i;
    uint8_t  wheels = Route_GetWheelMask();
    uint8_t  reach  = 0U;

    for (i = 0U; i < (uint8_t)MOTOR_NUM; i++)
    {
        if (Motor_ReachOk((uint8_t)(MOTOR_ADDR_MIN + i)) != 0U)
        {
            reach |= (uint8_t)(1U << i);
        }
    }

    Debug_Log("ROUTE", "seg%u phase done wheels=%u%u%u%u reach=%X",
              (unsigned int)routeSegIdx,
              (unsigned int)((wheels >> 0) & 1U), (unsigned int)((wheels >> 1) & 1U),
              (unsigned int)((wheels >> 2) & 1U), (unsigned int)((wheels >> 3) & 1U),
              (unsigned int)reach);
}

#if (ROUTE_SEG_RELATIVE == 0)
/**
 * @brief  浮点转 int32（四舍五入，避免正负都向 0 截断带来的 1mm 偏差）
 * @param  v: 输入值
 * @retval 最接近的整数
 * @note   只在**绝对坐标点模式**（需要坐标变换）下使用。
 */
static int32_t Route_RoundF(float v)
{
    return (int32_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}
#endif

/**
 * @brief  放行"朝向等待"（统一出口：结束静止零漂测量 + 清标志）
 * @param  无
 * @retval 恒为 0（供调用处直接 return）
 * @note   无论走哪条放行分支（已到位/已稳定/超时）都要经过这里，
 *         保证静止零漂测量一定有头有尾（Chassis_YawDriftEnd 内部会判断窗口是否够长）。
 */
static uint8_t Route_SettleRelease(void)
{
    routeSettleActive = 0U;
    Chassis_YawDriftEnd();
    routeDriftCalUsed = 1U;     /* 【2026-10-15 提速】本圈已做过零漂静止窗口，后续转弯不再强等 */
    return 0U;
}

/**
 * @brief  转弯后的"朝向等待"（等 IMU 把转弯后的新朝向送上来再规划平移方向）
 * @param  无
 * @retval 1=还要继续等（本拍不下发平移）；0=可以规划下发
 * @note   三级判据：
 *         1) 朝向已到达本段期望值（|实测 − 期望| ≤ ROUTE_TURN_TOL_DEG）→ 立即放行；
 *         2) 朝向**确实动过**（方向一致、且已走过命令角的一半）后，连续
 *            ROUTE_YAW_SETTLE_MS 变化 ≤0.5° → 视为稳定放行（兼容转过头的情况）；
 *         3) 超过 ROUTE_YAW_SETTLE_MAX_MS 仍未达标 → 放行并报警 `yaw NOT ready`。
 *         **关键**：旧版只看"稳定"，会把"新值还没送到（读到的仍是转弯前的旧值）"
 *         误判成稳定，于是按旧朝向规划出横向平移（转两次 90° 后走反方向）。
 *         现在必须先确认"朝向动过"，稳定判据才生效。
 */
static uint8_t Route_YawSettleWait(void)
{
    float y;
    float diff;
    float dNow;
    float dErr;
    float needAbs;
    float doneAbs;

    if (routeSettleActive == 0U)
    {
        return 0U;
    }

    /* 静止零漂测量窗口：转弯到位后至少静置 ROUTE_DRIFT_CAL_MS 再放行。
       这段静止时间里车不动，yaw 若还在缓慢变化就是陀螺零漂（由 Chassis_YawDriftEnd 估计）。
       【2026-10-15 提速】每圈只在第一次转弯时强制停满该窗口；之后转弯不再为它停
       （routeDriftCalUsed 已置 1），衔接时间只留 holdMs。见 bsp_route.h 说明。 */
    if ((routeDriftCalUsed == 0U) &&
        ((uint32_t)(HAL_GetTick() - routeSettleStart) < (uint32_t)ROUTE_DRIFT_CAL_MS))
    {
        return 1U;
    }

    y = Chassis_GetOdomYaw();

    /* 实测转角（相对本段旋转下发前），归一化到 ±180 */
    dNow = y - routeTurnYaw0;
    while (dNow > 180.0f)
    {
        dNow -= 360.0f;
    }
    while (dNow < -180.0f)
    {
        dNow += 360.0f;
    }

    /* 1) 已到达期望值：立刻放行（正常情况走这条，不额外等待）。
       判据用**转角闭环的容差** `yawTolDeg`（与 Route_YawLoop 一致）：
       转过弯且闭环已判达标时，这里就没必要再等"稳定"，直接放行。
       （`yawEnable = 0` 时这个容差比原来的 ROUTE_TURN_TOL_DEG 宽，
         等于退回"只等新朝向、不卡精度"的老行为，可接受。） */
    dErr = dNow - routeSettleNeed;
    if (dErr < 0.0f)
    {
        dErr = -dErr;
    }
    if (dErr <= (float)routeCfg.yawTolDeg)
    {
        Debug_Log("ROUTE", "seg%u yaw ready yaw=%d (waited %u ms)",
                  (unsigned int)routeSegIdx, (int)(y * 10.0f),
                  (unsigned int)(HAL_GetTick() - routeSettleStart));
        return Route_SettleRelease();
    }

    /* 2) 只有"朝向确实动过"才允许用稳定判据收尾，避免把旧值当稳定提前放行 */
    needAbs = (routeSettleNeed < 0.0f) ? -routeSettleNeed : routeSettleNeed;
    doneAbs = (dNow < 0.0f) ? -dNow : dNow;
    if (((dNow >= 0.0f) == (routeSettleNeed >= 0.0f)) &&
        (doneAbs >= (needAbs * 0.5f)))
    {
        diff = y - routeSettleYaw;
        if (diff < 0.0f)
        {
            diff = -diff;
        }
        if (diff > 0.5f)
        {
            /* 朝向还在变：记下新值并重新计时 */
            routeSettleYaw  = y;
            routeSettleTick = HAL_GetTick();
            return 1U;
        }
        if ((HAL_GetTick() - routeSettleTick) >= (uint32_t)ROUTE_YAW_SETTLE_MS)
        {
            Debug_Log("ROUTE", "seg%u yaw settled yaw=%d d=%d (waited %u ms)",
                      (unsigned int)routeSegIdx, (int)(y * 10.0f), (int)(dNow * 10.0f),
                      (unsigned int)(HAL_GetTick() - routeSettleStart));
            return Route_SettleRelease();
        }
    }

    /* 3) 等太久仍未到位：放行并报警，避免整条路径一直等下去 */
    if ((HAL_GetTick() - routeSettleStart) >= (uint32_t)ROUTE_YAW_SETTLE_MAX_MS)
    {
        Debug_Log("ROUTE", "seg%u yaw NOT ready yaw=%d d=%d (cmd=%d) -> check IMU refresh",
                  (unsigned int)routeSegIdx, (int)(y * 10.0f), (int)(dNow * 10.0f),
                  (int)(routeSettleNeed * 10.0f));
        return Route_SettleRelease();
    }

    return 1U;
}

/**
 * @brief  按"角度"下发一次原地旋转（只负责脉冲换算 + 下发，不动基准/标志）
 * @param  deg: 要转的角度，单位 deg（**正 = 逆时针 / 左转**，与页面/日志的 θ 同号）
 * @retval 本次下发的等效弧长（mm，带符号，仅用于日志打印）
 * @note   **调用方负责**：先 `Route_ReachClearAll()` 清到位标志、按需刷新
 *         `routeSegBase`（判到位基准）。本函数会写 `routeSegClk[]`。
 *         符号与刻度：表里/θ **正 = 逆时针**，而电机侧旋转正方向实测为**顺时针**，
 *         故取反；再乘**运行期旋转刻度** `rotScaleNum/Den`（在线调参：转不够就加大分子）。
 *         麦轮逆解（dx=dy=0）：FL/RL = +arc、FR/RR = −arc，与 bsp_mecanum.c 的
 *         `Mecanum_MoveByRot` 完全一致。**为什么自己算**：`rotVel` / `rotScale` 是
 *         **运行期在线可调量**，而 `Mecanum_MoveByRot` 里只有编译期宏；
 *         加减速度档则直接复用 bsp_mecanum 的编译期宏 `MEC_ROT_ACC`（与 MoveByRot 一致）。
 *         ⚠ 若以后 bsp_mecanum 的旋转逆解改了，这里必须同步改。
 */
static int32_t Route_SendRotDeg(float deg)
{
    float   arc;
    float   scale;
    int32_t clk[MOTOR_NUM];
    uint8_t i;

    arc = -(float)Mecanum_DegToArcMm(deg) *
          (float)routeCfg.rotScaleNum / (float)routeCfg.rotScaleDen;

    scale = MEC_PULSE_PER_MM * (float)MEC_PULSE_SCALE_NUM / (float)MEC_PULSE_SCALE_DEN;

    for (i = 0U; i < (uint8_t)MOTOR_NUM; i++)
    {
        float sg = ((i == 0U) || (i == 2U)) ? 1.0f : -1.0f;
        float p  = sg * (float)MEC_SIGN_WZ * arc * scale;

        clk[i] = (int32_t)((p >= 0.0f) ? (p + 0.5f) : (p - 0.5f));
        routeSegClk[i] = clk[i];             /* 判到位基准，必须与下发一致 */
    }

    Mecanum_MoveByPulse(clk, routeCfg.rotVel, (uint8_t)MEC_ROT_ACC);

    return (int32_t)arc;
}

/**
 * @brief  下发本阶段的"原地旋转"命令
 * @param  turn_deg: 目标转角，单位 deg（**正=逆时针/左转，负=顺时针/右转**，
 *                   与页面/日志的 θ（H）**同号**）
 * @retval 无
 * @note   先清到位标志、再记录脉冲基准与目标，最后下发（顺序不能反）。
 *         转速与旋转刻度可**在线调参**（`rotVel` / `rotScaleNum/Den`，见第 7 节）。
 *         记录下发前的 θ routeTurnYaw0，供到位后量"实测转角"（见 Route_CheckTurnAngle）
 *         以及**转角闭环**设定目标（见 Route_YawLoop）。
 */
static void Route_IssueRot(int16_t turn_deg)
{
    int32_t arc;

    Route_ReachClearAll();
    routeBaseValid = Route_ReadPos(routeSegBase);
    routeTurnYaw0  = Chassis_GetOdomYaw();   /* 记下发前 θ：到位后用它量实测转角 */
    routeTurnRaw0  = IMU_GetYaw();           /* 记下发前原始偏航角：诊断"yaw 是否真的变了" */

    arc = Route_SendRotDeg((float)turn_deg);

    Debug_Log("ROUTE", "seg%u rot arc=%d clk=%d,%d,%d,%d vel=%u sc=%u/%u",
              (unsigned int)routeSegIdx, (int)arc,
              (int)routeSegClk[0], (int)routeSegClk[1],
              (int)routeSegClk[2], (int)routeSegClk[3],
              (unsigned int)routeCfg.rotVel,
              (unsigned int)routeCfg.rotScaleNum, (unsigned int)routeCfg.rotScaleDen);
}

/**
 * @brief  量本段"实测转角"并**只打印提示**（不做任何中止）
 * @param  cmdDeg: 本段命令转角，单位 deg（正=逆时针/左转，与 θ 同号）
 * @retval 无
 * @note   纯诊断函数，**不会中止跑图**，判定结果只体现在日志里：
 *           - 命令转角够大却几乎没转（|d| < ROUTE_TURN_ALIVE_DEG）
 *             ⇒ 打印 `yaw not updating`（朝向数据可疑，查 IMU）；
 *           - |d| 与 |cmd| 相当但**符号相反** ⇒ 打印 `turn direction inverted`
 *             （偏航角正方向可疑，给出要改的宏值）；
 *           - |d − cmd| > ROUTE_TURN_TOL_DEG ⇒ 打印幅值偏差与**转角刻度建议值**
 *             （`MEC_ROT_CMD_GAIN_NUM/DEN`），供 3.10 节标定；
 *         提示后一律继续执行——后续路段用的是**实测 θ** 算增量，
 *         所以"转多/转少"不会把车带离目标点。
 */
static void Route_CheckTurnAngle(int16_t cmdDeg)
{
    float d      = Chassis_GetOdomYaw() - routeTurnYaw0;
    float expect = (float)cmdDeg;       /* 表里转角与 θ 同号 */
    float rawNow = IMU_GetYaw();
    float dRaw   = rawNow - routeTurnRaw0;   /* 原始偏航角的变化（诊断"yaw 是否真的变了"） */
    float ad;
    float ae;

    while (d > 180.0f)
    {
        d -= 360.0f;
    }
    while (d < -180.0f)
    {
        d += 360.0f;
    }

    ad = (d < 0.0f) ? -d : d;
    ae = (expect < 0.0f) ? -expect : expect;

    /* raw= 当前原始偏航角、draw= 本次旋转期间原始偏航角的变化量：
       若 draw≈0 而车确实转了 -> **IMU 的 yaw 没更新**（模块侧问题/被自动归零）；
       若 draw≠0 而 d≈0    -> 是 θ 换算/原点基准的问题（代码侧）。 */
    Debug_Log("ROUTE", "seg%u turn done, yaw d=%d (cmd=%d) raw=%d draw=%d imu=%u ccw=%u",
              (unsigned int)routeSegIdx, (int)(d * 10.0f), (int)(expect * 10.0f),
              (int)(rawNow * 10.0f), (int)(dRaw * 10.0f),
              (unsigned int)IMU_IsOnline(), (unsigned int)IMU_YAW_CCW_POSITIVE);

    if ((ae >= (float)ROUTE_TURN_TOL_DEG) && (ad < (float)ROUTE_TURN_ALIVE_DEG))
    {
        Debug_Log("ROUTE", "seg%u hint: yaw not updating (d=%d cmd=%d), check IMU",
                  (unsigned int)routeSegIdx, (int)(d * 10.0f), (int)(expect * 10.0f));
    }
    else if ((ae >= (float)ROUTE_TURN_TOL_DEG) &&
             (((d > 0.0f) && (expect < 0.0f)) || ((d < 0.0f) && (expect > 0.0f))) &&
             (((ad - ae) < (float)ROUTE_TURN_TOL_DEG) && ((ae - ad) < (float)ROUTE_TURN_TOL_DEG)))
    {
        Debug_Log("ROUTE", "seg%u hint: turn direction inverted, check IMU_YAW_CCW_POSITIVE (now %u)",
                  (unsigned int)routeSegIdx, (unsigned int)IMU_YAW_CCW_POSITIVE);
    }

    /* 残差超过**转角闭环的容差**才算"没转准"（否则闭环已经收进容差了，不该再报）：
       `yawEnable = 1` 时本行通常不出现；出现即说明闭环也放弃了（看 `yaw giveup`）。 */
    if (((d - expect) > (float)routeCfg.yawTolDeg) ||
        ((expect - d) > (float)routeCfg.yawTolDeg))
    {
        int32_t ratio = (ae > 1.0f) ? (int32_t)((ad * 100.0f) / ae) : 0;

        Debug_Log("ROUTE", "seg%u turn angle off by %d deg (cmd=%d achieved=%d%%) -- continue, check turn calib",
                  (unsigned int)routeSegIdx, (int)((d - expect) * 10.0f),
                  (int)(expect * 10.0f), (int)ratio);

        /* 建议值：gain = 100/ratio（例：实测 200% -> 100/200 = 1/2） */
        if (ratio > 0)
        {
            Debug_Log("ROUTE", "  -> set MEC_ROT_CMD_GAIN_NUM/DEN to 100/%d", (int)ratio);
        }
    }
}

/**
 * @brief  转角闭环：按 HWT101CT 回传的 θ 与目标比，超容差就补转一次
 * @param  无
 * @retval 1=转角已达标（或超轮次/超预算，**放行**）；0=刚下发补转，等它走完再来判
 * @note   【为什么要它】拐角是**开环**的（角度 → 弧长 → 脉冲，只靠标定），
 *         原地转是靠麦轮横向刮擦、打滑不可避免，实测常常差 2°~10°；
 *         而绝对点模式下"下一段往哪走"是用**实测量到的 θ** 反算的 ——
 *         θ 偏 Δ° ⇒ 平移方向歪 Δ°（走 1740mm、Δ=2° 就横偏约 61mm）。
 *         所以必须把 θ 校到位。
 *         【怎么校】纯 **P + 死区 + 限幅**，不做 PID：
 *           读 θ → 误差 e = 目标θ − 实测θ（归一化 ±180°）；
 *           · |e| ≤ `yawTolDeg`(2°)            → 达标，`yaw ok`
 *           · 轮次用尽 / 超 `yawBudgetMs`(1200ms) → `yaw giveup`，**放行不中止跑图**
 *           · 否则下发补转 `e × yawCorrPct%`（默认只补 70%，留 30% 防"补过头来回摆"），
 *             单次幅值限幅 `yawMaxCorrX10`(15°)，超过打 `yaw corr clamp`
 *           · 补转走完 → 外层再"等到位 + 同拍停 + 静置 holdMs"后才回到这里复测，
 *             保证用的是补转**之后**的新 IMU 样本（不再是旧值）
 *         【为什么不做 PID】本环是"每段一次、间隔几百 ms"的离散低频回路，
 *         被控对象（驱动器内部脉冲位置环）本身已是闭环，测量还有 20~50ms 死区：
 *         加 I 会 windup 冲过头、加 D 会被 0.1° 量化噪声放大成抖动。
 *         【为什么关不掉】`yawEnable = 0` 即整段跳过本函数，退回纯开环转角。
 */
static uint8_t Route_YawLoop(void)
{
    float   err;
    float   corr;
    int32_t corrX10;
    int32_t arc;

    if (routeCfg.yawEnable == 0U)
    {
        return 1U;                  /* 关掉转角闭环：直接放行 */
    }

    /* 误差 = 目标 θ − 实测 θ，归一化到 ±180°（正 = 需要逆时针补转 = 左转） */
    err = routeYawTarget - Chassis_GetOdomYaw();
    while (err > 180.0f)
    {
        err -= 360.0f;
    }
    while (err < -180.0f)
    {
        err += 360.0f;
    }
    routeYawErrX10 = (int32_t)(err * 10.0f);

    /* ---- check 1: inside tolerance -> OK, release ---- */
    if ((err <= (float)routeCfg.yawTolDeg) && (err >= -(float)routeCfg.yawTolDeg))
    {
        Debug_Log("ROUTE", "seg%u yaw ok r=%u e=%d target=%d now=%d",
                  (unsigned int)routeSegIdx, (unsigned int)routeYawRound,
                  (int)routeYawErrX10, (int)(routeYawTarget * 10.0f),
                  (int)(Chassis_GetOdomYaw() * 10.0f));
        return 1U;
    }

    /* ---- 判据②：轮次用尽 / 超预算 → 放行并报警（绝不中止跑图） ---- */
    if ((routeYawRound >= routeCfg.yawMaxRound) ||
        ((HAL_GetTick() - routeYawStart) >= (uint32_t)routeCfg.yawBudgetMs))
    {
        Debug_Log("ROUTE", "seg%u yaw giveup r=%u e=%d (%s), check IMU/rotScale",
                  (unsigned int)routeSegIdx, (unsigned int)routeYawRound,
                  (int)routeYawErrX10,
                  (routeYawRound >= routeCfg.yawMaxRound) ? "round" : "budget");
        return 1U;
    }

    /* ---- 判据③：下发补转（只补误差的 yawCorrPct%，并做单次限幅） ---- */
    corr = err * (float)routeCfg.yawCorrPct / 100.0f;
    corrX10 = (int32_t)(corr * 10.0f);
    if (corrX10 > (int32_t)routeCfg.yawMaxCorrX10)
    {
        corrX10 = (int32_t)routeCfg.yawMaxCorrX10;
    }
    else if (corrX10 < -(int32_t)routeCfg.yawMaxCorrX10)
    {
        corrX10 = -(int32_t)routeCfg.yawMaxCorrX10;
    }

    /* skip corrections below 1 deg (avoid hunting around the target) */
    if ((corrX10 < 10) && (corrX10 > -10))
    {
        Debug_Log("ROUTE", "seg%u yaw ok r=%u e=%d (tiny)",
                  (unsigned int)routeSegIdx, (unsigned int)routeYawRound,
                  (int)routeYawErrX10);
        return 1U;
    }

    corr = (float)corrX10 / 10.0f;

    /* 下发补转：清到位标志 + 用"当前四轮位置"当新的判到位基准（顺序不能反） */
    Route_ReachClearAll();
    routeBaseValid = Route_ReadPos(routeSegBase);
    arc = Route_SendRotDeg(corr);

    routeYawRound++;

    Debug_Log("ROUTE", "seg%u yaw corr d=%d e=%d r=%u arc=%d",
              (unsigned int)routeSegIdx, (int)corrX10, (int)routeYawErrX10,
              (unsigned int)routeYawRound, (int)arc);

    return 0U;
}

/**
 * @brief  下发本阶段的车身系平移命令
 * @param  dx_body: 车身系前进位移 mm（正=车头方向）
 * @param  dy_body: 车身系右移位移 mm（正=车身右侧）
 * @retval 无
 * @note   先清到位标志、再记录脉冲基准与目标，最后下发（顺序不能反）。
 *         走短/走长的比例误差按《调参手册》P4 章 3.1 标定
 *         `MEC_PULSE_SCALE_NUM/DEN`（已含在 Mecanum_MoveBy 内）；
 *         这里另有**运行期平移刻度** distScaleNum/Den（在线调参用）。
 */
static void Route_IssueMove(int32_t dx_body, int32_t dy_body)
{
    int32_t dx = dx_body;
    int32_t dy = dy_body;

    /* 运行期**平移刻度** distScaleNum/Den（在线调参：走短了加大分子）：
       只作用在"最终下发的车身系增量"上，不改里程计与目标点计算方式，
       所以绝对点模式下下一段会自动把这段误差纠回来（不会越调越乱）。 */
    if (routeCfg.distScaleDen != 0U)
    {
        dx = (int32_t)(((float)dx_body * (float)routeCfg.distScaleNum) /
                       (float)routeCfg.distScaleDen);
        dy = (int32_t)(((float)dy_body * (float)routeCfg.distScaleNum) /
                       (float)routeCfg.distScaleDen);
    }

    Route_ReachClearAll();
    routeBaseValid = Route_ReadPos(routeSegBase);
    Mecanum_MoveBy(dx, dy, 0, routeSegClk);
}

/**
 * @brief  下发本段的平移命令（按 ROUTE_SEG_RELATIVE 选择语义）
 * @param  tx: 路段表第 2 个字段：绝对模式下是目标点 x（mm，相对跑图起点）；
 *             相对模式下是本段前进位移 mm
 * @param  ty: 路段表第 3 个字段：绝对模式下是目标点 y（mm，相对跑图起点）；
 *             相对模式下是本段右移位移 mm
 * @param  outDxb: 输出算出的车身系前进位移 mm，可传 NULL
 * @param  outDyb: 输出算出的车身系右移位移 mm，可传 NULL
 * @retval 1=已下发位移命令；0=车身系增量四舍五入后为 0（无需下发）
 * @note   **绝对模式（ROUTE_SEG_RELATIVE=0）**：全局增量 = 目标点 − 当前 ODOM 坐标，
 *         再按 spec §3 的逆变换转到车身系：
 *             dx = X·cosθ − Y·sinθ
 *             dy = X·sinθ + Y·cosθ
 *         θ = Chassis_GetOdomYaw()（俯视逆时针为正），与里程计 Chassis_OdomUpdate
 *         里的正变换严格互逆，两处共用同一个 θ。此模式依赖 IMU 朝向正确。
 *         **相对模式（ROUTE_SEG_RELATIVE=1）**：tx/ty 直接就是车身系的前进/右移位移，
 *         不做坐标变换 → **运动不依赖 IMU 朝向**。
 *         增量为 0 时不下发：位置模式没有运动就不会回到位帧，等待会白等 2s。
 */
static uint8_t Route_IssueMoveAbs(int32_t tx, int32_t ty,
                                  int32_t *outDxb, int32_t *outDyb)
{
    int32_t bx;
    int32_t by;

#if (ROUTE_SEG_RELATIVE != 0)
    /* 车体系（相对）模式：x=前进 mm、y=右移 mm，直接用，不做坐标变换。
       这样运动**不依赖 IMU 朝向**（转弯后一定朝车头走），IMU 只用于坐标显示。 */
    bx = tx;
    by = ty;
#else
    {
        float theta = Chassis_GetOdomYawTrue() * MEC_RAD_PER_DEG;
        float c     = cosf(theta);
        float s     = sinf(theta);
        float dX    = (float)tx - Chassis_GetOdomX();
        float dY    = (float)ty - Chassis_GetOdomY();

        /* 逆变换（spec §3）：dx = X·cosθ − Y·sinθ；dy = X·sinθ + Y·cosθ */
        bx = Route_RoundF((dX * c) - (dY * s));
        by = Route_RoundF((dX * s) + (dY * c));
    }
#endif

    if (outDxb != NULL)
    {
        *outDxb = bx;
    }
    if (outDyb != NULL)
    {
        *outDyb = by;
    }

    if ((bx == 0) && (by == 0))
    {
        return 0U;
    }

    Route_IssueMove(bx, by);
    return 1U;
}

#if (ROUTE_SEG_RELATIVE == 0)
/**
 * @brief  本段到位后核对落点误差（仅日志提示，不影响推进）
 * @param  seg: 当前路段
 * @retval 无
 * @note   只在**绝对坐标点模式**下有意义（相对模式没有"目标点"）。
 *         误差 = 目标点 − 实测 ODOM；超过 ROUTE_DIST_TOL_MM 才打印，避免刷屏。
 */
static void Route_CheckPosErr(const RouteSeg_t *seg)
{
    int32_t ex = seg->x - (int32_t)Chassis_GetOdomX();
    int32_t ey = seg->y - (int32_t)Chassis_GetOdomY();

    if ((ex > (int32_t)ROUTE_DIST_TOL_MM) || (ex < -(int32_t)ROUTE_DIST_TOL_MM) ||
        (ey > (int32_t)ROUTE_DIST_TOL_MM) || (ey < -(int32_t)ROUTE_DIST_TOL_MM))
    {
        Debug_Log("ROUTE", "seg%u pos err x=%d y=%d",
                  (unsigned int)routeSegIdx, (int)ex, (int)ey);
    }
}
#endif

#if (ROUTE_VEL_MODE != 0U)
/* forward declarations (definitions are inside the velocity-mode block below) */
static float Route_Wrap180(float a);
static void  Route_VelHomeBegin(void);
#endif

/**
 * @brief  准备进入某个路段（阶段复位到 TURN，重新计时）
 * @param  无
 * @retval 无
 */
static void Route_ResetSeg(void)
{
    routePhase          = ROUTE_PHASE_TURN;
    routePhaseSent      = 0U;
    routeFallbackLogged = 0U;
    routeBaseValid      = 0U;
    routeSettleActive   = 0U;      /* 朝向稳定等待随段复位 */
    routeHoldActive     = 0U;      /* 停稳等待随段复位（防上一段的残留状态带进来） */
    routeYawRound       = 0U;      /* 转角闭环轮次随段复位（目标 θ 在下发旋转时重设） */
    routeYawErrX10      = 0;
    routePosRound       = 0U;      /* [2026-10-07] reset landing-point loop rounds per segment */
    routeSegStartTick   = HAL_GetTick();
    routePhaseTick      = HAL_GetTick();

    /* [2026-10-07] Safety net: a segment always starts with odom x/y NOT frozen.
       The turn phase freezes x/y (see Route_Update) so that rotating in place does
       not drift the coordinates; this call makes sure a frozen state can never leak
       into the next segment. */
    Chassis_OdomHoldXY(0U);

#if (ROUTE_VEL_MODE != 0U)
    /* velocity mode: each phase re-issues its command from scratch */
    rvMoving     = 0U;
    rvPhaseStart = HAL_GetTick();
    rvCtrlTick   = HAL_GetTick();
    rvOkTick     = 0U;
    rvStopValid  = 0U;      /* fresh wheel-stop detection per segment */
    rvStopTick   = 0U;
    rvHoldLogTick = 0U;

    /* ABSOLUTE planned heading for THIS segment = previous plan + its turn_deg.
       Both the turn phase and the following straight leg drive to this angle, so
       a small turn error is corrected while driving and never shifts the heading
       of the next legs. */
    rvYawPlan = Route_Wrap180(rvYawPlan + (float)routeTbl[routeSegIdx].turn_deg);
#endif
}

/**
 * @brief  跑图全部完成
 * @param  无
 * @retval 无
 * @note   位置模式电机到点自停，这里不需要急停；保持四轮使能（抱轴）。
 */
static void Route_Finish(void)
{
#if (ROUTE_VEL_MODE != 0U)
#if (ROUTE_VEL_HOME_ENABLE != 0U)
    /* POSITION CLOSED LOOP: before declaring the lap finished, use the encoder
       odometry to drive the car to (0,0). One attempt per lap; if the error is
       already inside ROUTE_VEL_POS_TOL_MM we finish immediately. */
    if (rvHomeDone == 0U)
    {
        float ex = Chassis_GetOdomX();
        float ey = Chassis_GetOdomY();

        if ((((ex < 0.0f) ? -ex : ex) > ROUTE_VEL_POS_TOL_MM) ||
            (((ey < 0.0f) ? -ey : ey) > ROUTE_VEL_POS_TOL_MM))
        {
            rvHomeDone = 1U;
            Route_VelHomeBegin();
            return;             /* stay RUN; the approach finishes in Route_VelUpdate */
        }
    }
#endif
#endif

    Debug_Log("ROUTE", "lap end x=%d y=%d yaw=%d lap=%u/%u",
              (int)Chassis_GetOdomX(), (int)Chassis_GetOdomY(),
              (int)(Chassis_GetOdomYaw() * 10.0f),
              (unsigned int)routeLapIdx, (unsigned int)routeLapTotal);

    /* 连跑模式（routeCfg.lapRepeat）：跑完一圈自动再跑下一圈，
       供 PC 自动调参脚本连续采集数据；按 KEY3 随时可以中止。 */
    if (routeLapIdx < routeLapTotal)
    {
        uint8_t addr;

        routeLapIdx++;
        routeSegIdx = 0U;
        routeDriftCalUsed = 0U;     /* 【2026-10-15 提速】新一圈重新允许一次零漂静止窗口 */
#if (ROUTE_VEL_MODE != 0U)
        rvYawPlan = 0.0f;           /* new lap: planned heading starts at 0 */
#endif
        Route_ResetSeg();

        /* 每圈重建里程计原点：这样每圈的"落点误差/转角误差"都是同一套口径，
           PC 端按圈比较才有意义（否则第 2 圈开始坐标就是累计值） */
        Chassis_OdomReset();

#if (ROUTE_VEL_MODE != 0U)
        /* new lap: odometry origin + raw yaw reference + home state */
        rvYaw0Raw    = IMU_GetYaw();
        rvYawPlan    = 0.0f;
        rvHomeActive = 0U;
        rvHomeDone   = 0U;
#endif
        for (addr = MOTOR_ADDR_MIN; addr <= MOTOR_ADDR_MAX; addr++)
        {
            Motor_Enable(addr, 1U);
        }

        Debug_Log("ROUTE", "lap %u/%u start",
                  (unsigned int)routeLapIdx, (unsigned int)routeLapTotal);
        Display_RouteRefresh();
        return;
    }

    routeState = ROUTE_STATE_DONE;
    routePhase = ROUTE_PHASE_WAIT;

    /* [2026-10-07] run finished -> never leave odom x/y frozen */
    Chassis_OdomHoldXY(0U);

    Debug_Log("ROUTE", "all done, x=%d y=%d",
              (int)Chassis_GetOdomX(), (int)Chassis_GetOdomY());
    Display_RouteRefresh();
}

/**
 * @brief  本段完成：段号 +1，进下一段或整体完成
 * @param  无
 * @retval 无
 */
static void Route_SegDone(void)
{
    Debug_Log("ROUTE", "seg%u done", (unsigned int)routeSegIdx);

    routeSegIdx++;
    if (routeSegIdx >= ROUTE_SEG_COUNT)
    {
        Route_Finish();
        return;
    }

    Route_ResetSeg();
    Display_RouteRefresh();
}

/**
 * @brief  Runtime read of ROUTE_VEL_MODE (keeps the position-mode path compiled
 *         without triggering an "unreachable statement" warning)
 */
#if (ROUTE_VEL_MODE != 0U)
static uint8_t Route_VelModeOn(void)
{
    return (uint8_t)ROUTE_VEL_MODE;
}
#endif

#if (ROUTE_VEL_MODE != 0U)
/* ===================== velocity-mode executor (pure gyro) ===================== */

/**
 * @brief  Runtime read of ROUTE_VEL_SEG_STOP (same trick as Route_VelModeOn:
 *         keeps both branches compiled, no "unreachable statement" warning)
 */
static uint8_t Route_VelSegStop(void)
{
    return (uint8_t)ROUTE_VEL_SEG_STOP;
}

/* unit conversions: body mm/s -> wheel RPM, body deg/s -> wheel RPM (in-place rotation) */
#define RV_RPM_PER_MMPS   (60.0f / MEC_WHEEL_CIRC_MM)
#define RV_RPM_PER_DPS    ((MEC_PI / 180.0f) * MEC_ROT_RADIUS_MM * 60.0f / MEC_WHEEL_CIRC_MM)

/**
 * @brief  Wrap an angle to (-180, +180]
 */
static float Route_Wrap180(float a)
{
    while (a > 180.0f)
    {
        a -= 360.0f;
    }
    while (a < -180.0f)
    {
        a += 360.0f;
    }
    return a;
}

/**
 * @brief  Heading used by velocity mode: RAW IMU yaw minus the yaw captured at
 *         the lap origin (one subtraction, no drift compensation, no integration)
 */
static float Route_VelYaw(void)
{
    return Route_Wrap180(IMU_GetYaw() - rvYaw0Raw);
}

/**
 * @brief  Stop all four wheels with speed-0 frames (soft stop, decelerate by acc)
 */
static void Route_VelStop(void)
{
    Mecanum_VelBodyRpm(0.0f, 0.0f, 0.0f, (uint8_t)ROUTE_VEL_ACC);
}

/**
 * @brief  Begin the final position approach to (0,0) using encoder odometry
 */
static void Route_VelHomeBegin(void)
{
    uint32_t now = HAL_GetTick();

    rvHomeActive = 1U;
    rvHomeTick   = now;
    rvYawTarget  = 0.0f;                 /* finish facing the start heading */
    rvYawPrev    = Route_VelYaw();
    rvRateF      = 0.0f;
    rvCtrlTick   = now;

    Debug_Log("ROUTE", "pos home start x=%d y=%d",
              (int)Chassis_GetOdomX(), (int)Chassis_GetOdomY());
}

/**
 * @brief  Final position approach: P-controlled speed toward (0,0),
 *         position feedback = encoder odometry, heading held at 0.
 * @note   Ends when the odometry is inside ROUTE_VEL_POS_TOL_MM, or after
 *         ROUTE_VEL_HOME_MAX_MS (never hangs).
 */
static void Route_VelHomeUpdate(void)
{
    uint32_t now = HAL_GetTick();
    float yaw = Route_VelYaw();
    float ex  = 0.0f - Chassis_GetOdomX();
    float ey  = 0.0f - Chassis_GetOdomY();
    float rem = sqrtf((ex * ex) + (ey * ey));
    float theta;
    float c;
    float s;
    float bx;
    float by;
    float d;
    float v;
    float err;
    float wz;
    float dt;

    if ((rem <= ROUTE_VEL_POS_TOL_MM) ||
        ((now - rvHomeTick) >= (uint32_t)ROUTE_VEL_HOME_MAX_MS))
    {
        Route_VelStop();
        rvHomeActive = 0U;
        Debug_Log("ROUTE", "pos home done x=%d y=%d rem=%d",
                  (int)Chassis_GetOdomX(), (int)Chassis_GetOdomY(), (int)rem);
        Route_Finish();
        return;
    }

    /* world error -> body frame (same inverse transform as the leg planner) */
    theta = yaw * MEC_RAD_PER_DEG;
    c = cosf(theta);
    s = sinf(theta);
    bx = (ex * c) - (ey * s);
    by = (ex * s) + (ey * c);
    d  = sqrtf((bx * bx) + (by * by));
    if (d < 1.0f)
    {
        d = 1.0f;
    }

    v = ROUTE_VEL_HOME_KP * rem;         /* slow down as it approaches */
    if (v > ROUTE_VEL_MMPS)
    {
        v = ROUTE_VEL_MMPS;
    }
    if (v < ROUTE_VEL_HOME_MIN_MMPS)
    {
        v = ROUTE_VEL_HOME_MIN_MMPS;
    }

    /* heading hold (same law as the straight legs) */
    err = Route_Wrap180(rvYawTarget - yaw);
    dt  = (float)(now - rvCtrlTick) / 1000.0f;
    if (dt < 0.001f)
    {
        dt = 0.001f;
    }
    {
        float dyaw = Route_Wrap180(yaw - rvYawPrev);

        rvRateF   = (0.6f * rvRateF) + (0.4f * (dyaw / dt));
        rvYawPrev = yaw;
    }
    rvCtrlTick = now;

    wz = (ROUTE_VEL_YAW_KP * err) - (ROUTE_VEL_YAW_KD * rvRateF);
    if (wz > ROUTE_VEL_YAW_MAX_DPS)        { wz =  ROUTE_VEL_YAW_MAX_DPS; }
    else if (wz < -ROUTE_VEL_YAW_MAX_DPS)  { wz = -ROUTE_VEL_YAW_MAX_DPS; }

    Mecanum_VelBodyRpm(v * (bx / d) * RV_RPM_PER_MMPS,
                       v * (by / d) * RV_RPM_PER_MMPS,
                       -wz * RV_RPM_PER_DPS, (uint8_t)ROUTE_VEL_ACC);
}

/**
 * @brief  Velocity-mode turn phase: gyro PID until the target heading is reached
 * @param  seg: current segment
 * @note   No wheels-position feedback: the phase ends when |yaw err| stays inside
 *         ROUTE_VEL_TURN_TOL_DEG for ROUTE_VEL_STOP_MS. Timeout -> give up but
 *         still advance (never abort the whole run for one turn).
 */
static void Route_VelTurn(const RouteSeg_t *seg)
{
    uint32_t now = HAL_GetTick();
    float    yaw = Route_VelYaw();
    float    err;
    float    aerr;
    float    wz;
    float    dt;

    if (rvMoving == 0U)
    {
        /* Drive to the ABSOLUTE planned heading of this segment (not "current yaw
           + turn"): a previous turn error is corrected here instead of being
           carried into every following leg. */
        rvYawTarget  = rvYawPlan;
        rvYawPrev    = yaw;
        rvRateF      = 0.0f;
        rvPhaseStart = now;
        rvCtrlTick   = now;
        rvOkTick     = 0U;
        rvMoving     = 1U;
        Debug_Log("ROUTE", "seg%u vel turn %d tgt=%d yaw=%d",
                  (unsigned int)routeSegIdx, (int)seg->turn_deg,
                  (int)(rvYawTarget * 10.0f), (int)(yaw * 10.0f));
    }

    if ((now - rvPhaseStart) >= ROUTE_VEL_TURN_TIMEOUT_MS)
    {
        Route_VelStop();
        Debug_Log("ROUTE", "seg%u vel turn giveup err=%d",
                  (unsigned int)routeSegIdx,
                  (int)(Route_Wrap180(rvYawTarget - yaw) * 10.0f));
        rvMoving   = 0U;
        routePhase = ROUTE_PHASE_MOVE;
        return;
    }

    /* measured yaw rate (low-pass) -> real damping term.
       Using the measured rate instead of d(err)/dt is what stops the "swing back
       and forth": the old KD=0.08 on d(err) was almost no brake at all. */
    dt = (float)(now - rvCtrlTick) / 1000.0f;
    if (dt < 0.001f)
    {
        dt = 0.001f;
    }
    {
        float dyaw = Route_Wrap180(yaw - rvYawPrev);
        float rate = dyaw / dt;
        rvRateF   = (0.6f * rvRateF) + (0.4f * rate);
        rvYawPrev = yaw;
    }

    err  = Route_Wrap180(rvYawTarget - yaw);
    aerr = (err < 0.0f) ? -err : err;
    rvCtrlTick = now;

    /* ---- arrive: deadband + rate gate + confirmed hold ---- */
    if ((aerr <= ROUTE_VEL_TURN_TOL_DEG) &&
        (((rvRateF < 0.0f) ? -rvRateF : rvRateF) <= ROUTE_VEL_TURN_ARRIVE_DPS))
    {
        if (rvOkTick == 0U)
        {
            rvOkTick = now;
        }
        if ((now - rvOkTick) >= ROUTE_VEL_TURN_HOLD_MS)
        {
            Route_VelStop();
            rvMoving     = 0U;
            routePhase   = ROUTE_PHASE_MOVE;
            rvPhaseStart = now;
            Debug_Log("ROUTE", "seg%u vel turn ok err=%d rate=%d yaw=%d",
                      (unsigned int)routeSegIdx, (int)(err * 10.0f),
                      (int)(rvRateF * 10.0f), (int)(yaw * 10.0f));
            return;
        }
    }
    else
    {
        rvOkTick = 0U;
    }

    /* ---- continuous cascade: position P -> rate reference -> rate loop ----
       rate_ref = min(KP_POS*|err|, sqrt(2*ACC*|err|), MAX), not below MIN while
       outside the deadband.  wz = rate_ref + KP_RATE*(rate_ref - measured rate):
       when the car is faster than the reference the command goes negative, i.e.
       REAL braking (that is what removes the overshoot / oscillation). */
    {
        float ref = ROUTE_VEL_TURN_KP_POS * aerr;
        float env = sqrtf(2.0f * ROUTE_VEL_TURN_ACC_DPS2 * aerr);

        if (ref > env)
        {
            ref = env;
        }
        if (ref > ROUTE_VEL_TURN_MAX_DPS)
        {
            ref = ROUTE_VEL_TURN_MAX_DPS;
        }
        if ((aerr > ROUTE_VEL_TURN_TOL_DEG) && (ref < ROUTE_VEL_TURN_MIN_DPS))
        {
            ref = ROUTE_VEL_TURN_MIN_DPS;
        }
        if (err < 0.0f)
        {
            ref = -ref;
        }

        wz = ref + (ROUTE_VEL_TURN_KP_RATE * (ref - rvRateF));
    }
    if (wz > ROUTE_VEL_TURN_MAX_DPS)        { wz =  ROUTE_VEL_TURN_MAX_DPS; }
    else if (wz < -ROUTE_VEL_TURN_MAX_DPS)  { wz = -ROUTE_VEL_TURN_MAX_DPS; }

    /* Differential in-place turn about the BODY CENTER:
       pure yaw command, no body vx/vy -> the chassis centre stays put and all
       four wheels take the rotation share (FL/RL one way, FR/RR the other).
       NOTE THE MINUS: the mecanum inverse kinematics used here has W>0 = CW
       (see Route_SendRotDeg / bsp_chassis), while wz is CCW-positive. */
    Mecanum_VelBodyRpm(0.0f, 0.0f, -wz * RV_RPM_PER_DPS, (uint8_t)ROUTE_VEL_ACC);
}

/**
 * @brief  Velocity-mode move phase: trapezoid speed profile + gyro heading hold
 * @param  seg: current segment
 * @note   Distance comes from the route table and the PLANNED position (open loop),
 *         NOT from wheel positions -- this is the "pure gyro" scheme.
 *         Direction is computed once at phase start (body frame); the heading PID
 *         keeps that body direction pointing at the planned world direction.
 */
static void Route_VelMove(const RouteSeg_t *seg)
{
    uint32_t now = HAL_GetTick();
    float    yaw = Route_VelYaw();
    float    t;
    float    v;
    float    err;
    float    wz;
    float    dt;
    float    rpmF;
    float    rpmR;
    uint8_t  i;

    if (rvMoving == 0U)
    {
        float   bx;
        float   by;
        float   d;
        float   yawRef;
        float   vStart;
        float   vEnd;

        /* Heading reference for this leg:
           - ROUTE_VEL_SEG_STOP=1 (segmented test): keep the ABSOLUTE heading set
             by the turn phase, so the straight leg also pulls any residual turn
             error back while driving (gyro straight-line hold).
           - otherwise: keep the old blend behaviour (if the previous move already
             blended this segment's turn, keep that target). */
        if (Route_VelSegStop() != 0U)
        {
            yawRef    = rvYawTarget;
            rvKeepYaw = 0U;
        }
        else if (rvKeepYaw != 0U)
        {
            yawRef     = rvYawTarget;
            rvKeepYaw  = 0U;
        }
        else
        {
            yawRef      = yaw;
            rvYawTarget = yaw;
        }

#if (ROUTE_SEG_RELATIVE != 0)
        bx = (float)seg->x;         /* body: x = forward, y = right */
        by = (float)seg->y;
#else
        {
            /* POSITION CLOSED LOOP: the leg starts from the MEASURED position
               (encoder odometry), not from the previous plan, so the error of
               the previous legs is corrected here instead of accumulating. */
            float theta = yawRef * MEC_RAD_PER_DEG;
            float c     = cosf(theta);
            float s     = sinf(theta);
            float dX    = (float)seg->x - Chassis_GetOdomX();
            float dY    = (float)seg->y - Chassis_GetOdomY();

            bx = (dX * c) - (dY * s);
            by = (dX * s) + (dY * c);
        }
#endif
        d = sqrtf((bx * bx) + (by * by));

        if (d < 2.0f)
        {
            Debug_Log("ROUTE", "seg%u vel move skip d=%d",
                      (unsigned int)routeSegIdx, (int)d);
            Route_SegDone();
            return;
        }

        rvDirF = bx / d;
        rvDirR = by / d;
        rvDist = d * ((float)ROUTE_VEL_DIST_PCT / 100.0f);

        /* End speed depends on what comes next:
           - next segment turns   -> slow to corner speed and blend the turn
           - next segment goes on -> keep cruise speed (waypoint on a straight line)
           - next segment only turns (no move) or there is none -> 0 (full stop) */
        {
            uint8_t hasNext  = (((uint8_t)(routeSegIdx + 1U)) < ROUTE_SEG_COUNT) ? 1U : 0U;
            uint8_t turnNext = (uint8_t)((hasNext != 0U) &&
                              (routeTbl[routeSegIdx + 1U].turn_deg != 0)) ? 1U : 0U;
            uint8_t nextNoMove = (uint8_t)((hasNext != 0U) &&
                                ((routeTbl[routeSegIdx + 1U].x == ROUTE_NO_MOVE) ||
                                 (routeTbl[routeSegIdx + 1U].y == ROUTE_NO_MOVE))) ? 1U : 0U;

            vEnd = (Route_VelSegStop() != 0U)
                 ? 0.0f     /* segmented test: full stop at the end of EVERY leg */
                 : (((hasNext == 0U) || (nextNoMove != 0U))
                    ? 0.0f                     /* last leg / final in-place turn -> stop */
                    : ((turnNext != 0U)
                       ? ROUTE_VEL_CORNER_MMPS /* corner ahead: slow down, keep driving */
                       : ROUTE_VEL_MMPS));
        }
        vStart       = (rvFromCorner != 0U) ? rvCornerSpeed : 0.0f;
        rvFromCorner = 0U;
        rvVStart     = vStart;
        rvVEnd       = vEnd;

        /* speed profile with start/end speeds (trapezoid or triangle) */
        {
            float a    = ROUTE_VEL_ACC_MMPS2;
            float vmax = ROUTE_VEL_MMPS;
            float tA   = (vmax - vStart) / a;
            float dA;
            float tD;
            float dD;
            float tC;

            if (tA < 0.0f) { tA = 0.0f; }
            dA = 0.5f * (vStart + vmax) * tA;
            tD = (vmax - vEnd) / a;
            if (tD < 0.0f) { tD = 0.0f; }
            dD = 0.5f * (vmax + vEnd) * tD;

            if (rvDist >= (dA + dD))
            {
                rvPeak    = vmax;
                tC        = (rvDist - dA - dD) / vmax;
                rvTAcc    = tA;
                rvTDecel  = tD;
                rvTCruise = tC;
                rvTTotal  = tA + tC + tD;
            }
            else
            {
                /* peak between vStart and vmax such that accel + decel = D */
                rvPeak = sqrtf(((2.0f * a * rvDist) +
                                (vStart * vStart) + (vEnd * vEnd)) * 0.5f);
                rvTAcc    = (rvPeak - vStart) / a;
                if (rvTAcc < 0.0f) { rvTAcc = 0.0f; }
                tD        = (rvPeak - vEnd) / a;
                if (tD < 0.0f) { tD = 0.0f; }
                rvTDecel  = tD;
                rvTCruise = 0.0f;
                rvTTotal  = rvTAcc + tD;
            }
        }

        rvYawPrev    = yaw;
        rvRateF      = 0.0f;
        rvPhaseStart = now;
        rvCtrlTick   = now;
        rvOkTick     = 0U;
        rvMoving     = 1U;
        rvBlend      = 0U;

        Debug_Log("ROUTE", "seg%u vel move fwd=%d right=%d d=%d t=%d v0=%d ve=%d yaw=%d",
                  (unsigned int)routeSegIdx,
                  (int)(rvDirF * rvDist), (int)(rvDirR * rvDist),
                  (int)rvDist, (int)(rvTTotal * 1000.0f),
                  (int)vStart, (int)vEnd, (int)(yaw * 10.0f));
    }

    t = (float)(now - rvPhaseStart) / 1000.0f;

    if (t >= rvTTotal)
    {
        /* Segmented test mode: stop (and let the wheels settle) after EVERY
           segment, then advance -- the next segment starts from rest. */
        if (Route_VelSegStop() != 0U)
        {
            int32_t cur[MOTOR_NUM];
            uint8_t still = 0U;
            uint8_t moved = 0U;

            Route_VelStop();                    /* keep commanding speed 0 */

            if (rvOkTick == 0U)
            {
                rvOkTick = now;
            }

            /* ---- the straight leg must REALLY be stopped before the turn ----
               The 200ms timer alone is not enough: the motors decelerate by
               their own ramp, so the car can still be rolling when the turn
               starts. Sample the four wheel positions and require that none of
               them moved more than ROUTE_VEL_STOP_TOL_PULSE for
               ROUTE_VEL_STOP_HOLD_MS. */
            if (Route_ReadPos(cur) != 0U)
            {
                if (rvStopValid == 0U)
                {
                    for (i = 0U; i < (uint8_t)MOTOR_NUM; i++)
                    {
                        rvStopPos[i] = cur[i];
                    }
                    rvStopValid = 1U;
                    rvStopTick  = 0U;
                }
                else
                {
                    for (i = 0U; i < (uint8_t)MOTOR_NUM; i++)
                    {
                        int32_t d = cur[i] - rvStopPos[i];

                        if (d < 0)
                        {
                            d = -d;
                        }
                        if (d > (int32_t)ROUTE_VEL_STOP_TOL_PULSE)
                        {
                            moved = 1U;
                        }
                    }

                    if (moved != 0U)
                    {
                        for (i = 0U; i < (uint8_t)MOTOR_NUM; i++)
                        {
                            rvStopPos[i] = cur[i];
                        }
                        rvStopTick = 0U;
                    }
                    else if (rvStopTick == 0U)
                    {
                        rvStopTick = now;
                    }
                    else if ((now - rvStopTick) >= ROUTE_VEL_STOP_HOLD_MS)
                    {
                        still = 1U;
                    }
                }
            }

            if ((((now - rvOkTick) >= ROUTE_VEL_SEG_STOP_MS) && (still != 0U)) ||
                ((now - rvOkTick) >= ROUTE_VEL_STOP_MAX_MS))
            {
                rvMoving    = 0U;
                rvStopValid = 0U;
                rvStopTick  = 0U;
                Debug_Log("ROUTE", "seg%u vel stop still=%u x=%d y=%d yaw=%d",
                          (unsigned int)routeSegIdx, (unsigned int)still,
                          (int)Chassis_GetOdomX(), (int)Chassis_GetOdomY(),
                          (int)(yaw * 10.0f));
                Route_SegDone();
            }
            return;
        }

        if (((uint8_t)(routeSegIdx + 1U)) < ROUTE_SEG_COUNT)
        {
            /* hand over to the next segment WITHOUT stopping:
               - straight continuation keeps cruise speed
               - corner continuation keeps the corner speed and finishes the turn
                 while driving (rvSkipTurn tells the next TURN phase to skip) */
            rvCornerSpeed = rvVEnd;
            rvFromCorner  = 1U;
            if ((rvBlend != 0U) &&
                (routeTbl[routeSegIdx + 1U].x != ROUTE_NO_MOVE) &&
                (routeTbl[routeSegIdx + 1U].y != ROUTE_NO_MOVE))
            {
                /* normal continuation: the next leg carries on without an in-place turn */
                rvSkipTurn = 1U;
                rvKeepYaw  = 1U;
            }
            /* If the next segment only turns (no move), do NOT skip: after the car
               has stopped, that final in-place turn corrects any residual angle. */
            rvMoving = 0U;
            Debug_Log("ROUTE", "seg%u vel continue v=%d blend=%u",
                      (unsigned int)routeSegIdx, (int)rvVEnd, (unsigned int)rvBlend);
            Route_SegDone();
            return;
        }

        Route_VelStop();
        if (rvOkTick == 0U)
        {
            rvOkTick = now;
        }
        if ((now - rvOkTick) >= ROUTE_VEL_STOP_MS)
        {
            rvMoving = 0U;
            Route_SegDone();
        }
        return;
    }
    rvOkTick = 0U;

    /* speed profile: vStart -> cruise -> vEnd, SINE (S-curve) ramps.
       Same area as a linear ramp of the same duration (average speed is
       (v1+v2)/2), so the solved distance/time stays valid; only the shape is
       smoother: zero jerk at both ends of each ramp -> less wheel slip. */
    if (t < rvTAcc)
    {
        float s = (rvTAcc > 0.001f) ? (t / rvTAcc) : 1.0f;

        v = rvVStart + ((rvPeak - rvVStart) * (0.5f - (0.5f * cosf(MEC_PI * s))));
    }
    else if (t < (rvTAcc + rvTCruise))
    {
        v = rvPeak;
    }
    else
    {
        float s = (rvTDecel > 0.001f) ? ((rvTTotal - t) / rvTDecel) : 0.0f;

        if (s > 1.0f)
        {
            s = 1.0f;
        }
        v = rvVEnd + ((rvPeak - rvVEnd) * (0.5f - (0.5f * cosf(MEC_PI * s))));
    }
    if (v < 0.0f)
    {
        v = 0.0f;
    }

    /* corner blending: start rotating for the NEXT segment while still translating */
    if ((rvBlend == 0U) &&
        (((rvTTotal - t) * 1000.0f) <= (float)ROUTE_VEL_BLEND_MS) &&
        (((uint8_t)(routeSegIdx + 1U)) < ROUTE_SEG_COUNT) &&
        (routeTbl[routeSegIdx + 1U].turn_deg != 0))
    {
        /* blend toward the ABSOLUTE planned heading of the NEXT segment */
        rvBlend     = 1U;
        rvBlendYaw0 = yaw;
        rvYawTarget = Route_Wrap180(rvYawPlan + (float)routeTbl[routeSegIdx + 1U].turn_deg);
        rvBlendTurn = Route_Wrap180(rvYawTarget - yaw);
        rvBlendT0   = now;
        rvBlendDur  = (uint32_t)((rvTTotal - t) * 1000.0f);
        if (rvBlendDur < 200U)
        {
            rvBlendDur = 200U;
        }
        rvYawTarget = Route_Wrap180(yaw + rvBlendTurn);
        Debug_Log("ROUTE", "seg%u vel blend turn %d v=%d win=%d",
                  (unsigned int)routeSegIdx,
                  (int)rvBlendTurn, (int)v, (int)rvBlendDur);
    }

    /* heading command: during a blend follow a LINEAR yaw ramp so the turn is
       finished by the time the segment ends (a pure P target lags behind and
       leaves 20~40 deg for the next segment -> the corner looks wrong) */
    if (rvBlend != 0U)
    {
        uint32_t el   = now - rvBlendT0;
        float    frac = (rvBlendDur != 0U) ? ((float)el / (float)rvBlendDur) : 1.0f;

        if (frac > 1.0f)
        {
            frac = 1.0f;
        }
        /* SINE ease instead of a linear ramp: the corner starts and ends with
           zero yaw rate -> less lateral impact / slip at both ends */
        frac = 0.5f - (0.5f * cosf(MEC_PI * frac));
        err = Route_Wrap180((rvBlendYaw0 + (rvBlendTurn * frac)) - yaw);
    }
    else
    {
        err = Route_Wrap180(rvYawTarget - yaw);
    }
    dt  = (float)(now - rvCtrlTick) / 1000.0f;
    if (dt < 0.001f)
    {
        dt = 0.001f;
    }
    {
        float dyaw = Route_Wrap180(yaw - rvYawPrev);
        float rate = dyaw / dt;
        rvRateF   = (0.6f * rvRateF) + (0.4f * rate);
        rvYawPrev = yaw;
    }
    wz = (ROUTE_VEL_YAW_KP * err) - (ROUTE_VEL_YAW_KD * rvRateF);
    {
        float cap = (rvBlend != 0U) ? ROUTE_VEL_BLEND_DPS : ROUTE_VEL_YAW_MAX_DPS;

        if (wz > cap)       { wz =  cap; }
        else if (wz < -cap) { wz = -cap; }
    }

    /* Segmented mode: NO heading correction once the decel ramp starts.
       Otherwise the yaw PID keeps adding a rotation term while the speed goes
       to zero, and the car visibly "turns" at the end of a straight leg.
       The leg then just decelerates straight and stops on all four wheels. */
    if ((Route_VelSegStop() != 0U) && (t >= (rvTAcc + rvTCruise)))
    {
        wz = 0.0f;
    }

    /* diagnostic: show that the straight leg is really correcting its heading
       (target / measured / error in 0.1 deg, correction rate in 0.1 deg/s) */
    if ((now - rvHoldLogTick) >= 500U)
    {
        rvHoldLogTick = now;
        Debug_Log("ROUTE", "seg%u yaw hold tgt=%d yaw=%d err=%d wz=%d",
                  (unsigned int)routeSegIdx, (int)(rvYawTarget * 10.0f),
                  (int)(yaw * 10.0f), (int)(err * 10.0f), (int)(wz * 10.0f));
    }

    rvCtrlTick = now;

    rpmF = v * rvDirF * RV_RPM_PER_MMPS;
    rpmR = v * rvDirR * RV_RPM_PER_MMPS;

    /* heading-hold: same W sign convention as the turn phase (W>0 = CW) */
    Mecanum_VelBodyRpm(rpmF, rpmR, -wz * RV_RPM_PER_DPS, (uint8_t)ROUTE_VEL_ACC);
}

/**
 * @brief  Velocity-mode state machine tick (called at ROUTE_VEL_TICK_MS)
 */
static void Route_VelUpdate(void)
{
    const RouteSeg_t *seg;

    Display_RouteRefresh();

    if ((HAL_GetTick() - routeSegStartTick) >= ROUTE_SEG_TIMEOUT_MS)
    {
        Debug_Log("ROUTE", "seg%u timeout", (unsigned int)routeSegIdx);
        Route_Abort();
        return;
    }

    /* final position closed loop to (0,0): runs after the last segment */
    if (rvHomeActive != 0U)
    {
        Route_VelHomeUpdate();
        return;
    }

    seg = &routeTbl[routeSegIdx];

    switch (routePhase)
    {
        case ROUTE_PHASE_TURN:
            /* turn already blended into the previous move -> go straight on */
            if ((seg->turn_deg == 0) || (rvSkipTurn != 0U))
            {
                /* no turn here: the straight leg still holds the ABSOLUTE
                   planned heading of this segment */
                rvYawTarget = rvYawPlan;
                rvSkipTurn  = 0U;
                routePhase  = ROUTE_PHASE_MOVE;
                rvMoving    = 0U;
            }
            else
            {
                Route_VelTurn(seg);
            }
            break;

        case ROUTE_PHASE_MOVE:
            if ((seg->x == ROUTE_NO_MOVE) || (seg->y == ROUTE_NO_MOVE))
            {
                Route_SegDone();
            }
            else
            {
                Route_VelMove(seg);
            }
            break;

        default:
            break;
    }
}
#endif /* ROUTE_VEL_MODE */

/* ===================== 公开接口 ===================== */

/**
 * @brief  跑图模块初始化（只复位状态机，不发电机命令）
 * @param  无
 * @retval 无
 */
void Route_Init(void)
{
    routeState          = ROUTE_STATE_IDLE;
    routePhase          = ROUTE_PHASE_WAIT;
    routeSegIdx         = 0U;
    routePhaseSent      = 0U;
    routeFallbackLogged = 0U;
    routeBaseValid      = 0U;
    routeSettleActive   = 0U;
    routeHoldActive     = 0U;
    routeYawRound       = 0U;
    routeYawErrX10      = 0;
    routeDriftCalUsed   = 0U;      /* 【2026-10-15 提速】零漂窗口"每圈一次"标志复位 */

    /* [2026-10-07] init -> make sure odom x/y is not frozen */
    Chassis_OdomHoldXY(0U);

#if (ROUTE_VEL_MODE != 0U)
    rvHomeActive = 0U;
    rvHomeDone   = 0U;
    rvYawPlan    = 0.0f;
#endif

    /* 运行期可调参数恢复成 bsp_route.h 里的默认值（在线调参改的是 routeCfg） */
    routeCfg = routeCfgDefault;
    routeLapIdx   = 1U;
    routeLapTotal = 1U;
    routeTbl      = routeTable;
}

/**
 * @brief  取运行期可调参数（在线调参用）
 * @param  无
 * @retval 指向全局配置的指针（恒非空）
 */
RouteCfg_t *Route_GetCfg(void)
{
    return &routeCfg;
}

/**
 * @brief  把运行期可调参数恢复成 bsp_route.h 里的默认值
 * @param  无
 * @retval 无
 */
void Route_CfgReset(void)
{
    routeCfg = routeCfgDefault;
}

/**
 * @brief  启动跑图（使能四轮 + 清零相对坐标 + 段索引归 0 + 置 RUN）
 * @param  无
 * @retval 无
 * @note   四轮使能只在本函数发生（位置模式必须先使能），
 *         退出/中止时由 Route_Abort 失能。
 */
void Route_Start(void)
{
    uint8_t addr;

    if (ROUTE_SEG_COUNT == 0U)
    {
        Debug_Log("ROUTE", "no segment defined");
        return;
    }

    /* 位置模式的一切（里程计坐标 + 目标增量）都依赖位置数据：
       读不到四轮位置就拒绝启动，否则会朝错误的坐标走（与 P4 一致）。
       ★速度模式（ROUTE_VEL_MODE=1）按"表里距离 + 陀螺仪航向"跑，不需要位置数据。 */
#if (ROUTE_VEL_MODE == 0U)
    if (Route_ReadPos(routeSegBase) == 0U)
    {
        Debug_Log("ROUTE", "no pos data, start rejected");
        return;
    }
#else
    routeBaseValid = 0U;
#endif

    /* 使能四轮（与 P4 一致：使能只发生在启动时） */
    for (addr = MOTOR_ADDR_MIN; addr <= MOTOR_ADDR_MAX; addr++)
    {
        Motor_Enable(addr, 1U);
    }

    /* 原点 = 当前位置，朝向基准 = 当前 yaw；此后所有路段目标都相对本原点 */
    Chassis_OdomReset();

    /* 路段表运行期缩放（routeCfg.tableScalePct）：调车时缩成小圈跑，不用重编译。
       100% 时直接用常量表；否则把每个目标点**按跑图原点 (0,0) 等比缩放**。

       为什么锚在原点而不是表里第一个点：本表是"起步于原点、跑完回原点"的闭环，
       只有锚在原点才能保证**缩放后仍然闭环**（每圈结束回到起点、下一圈原地重跑）。
       若锚在表内首个点，末点 (0,0) 会被缩到非零位置 → 每跑一圈整车就平移一段距离，
       连跑几圈后车会慢慢走出小圈（自动调参连跑时尤其致命）。 */
    if ((routeCfg.tableScalePct >= 5U) && (routeCfg.tableScalePct <= 100U) &&
        (routeCfg.tableScalePct != 100U))
    {
        uint8_t i;

        for (i = 0U; i < ROUTE_SEG_COUNT; i++)
        {
            routeCfgTable[i].turn_deg = routeTable[i].turn_deg;
            if ((routeTable[i].x == ROUTE_NO_MOVE) || (routeTable[i].y == ROUTE_NO_MOVE))
            {
                routeCfgTable[i].x = routeTable[i].x;
                routeCfgTable[i].y = routeTable[i].y;
            }
            else
            {
                routeCfgTable[i].x = (int32_t)((routeTable[i].x *
                                    (int32_t)routeCfg.tableScalePct) / 100);
                routeCfgTable[i].y = (int32_t)((routeTable[i].y *
                                    (int32_t)routeCfg.tableScalePct) / 100);
            }
        }
        routeTbl = routeCfgTable;
        Debug_Log("ROUTE", "table scaled to %u%% (about origin)",
                  (unsigned int)routeCfg.tableScalePct);
    }
    else
    {
        routeTbl = routeTable;
    }

    /* 连跑圈数（PC 自动调参用）：0/1 => 只跑一圈；255 => 无限圈 */
    if (routeCfg.lapRepeat == 0U)
    {
        routeLapTotal = 1U;
    }
    else
    {
        routeLapTotal = routeCfg.lapRepeat;      /* 255 = 无限（靠 KEY3 中止） */
    }
    routeLapIdx = 1U;
    routeDriftCalUsed = 0U;     /* 【2026-10-15 提速】每圈第一次转弯才强制零漂静止窗口 */

    routeSegIdx = 0U;
    routeState  = ROUTE_STATE_RUN;

#if (ROUTE_VEL_MODE != 0U)
    /* new run: the first segment's planned heading is 0 (relative to start) */
    rvYawPlan = 0.0f;
#endif
    Route_ResetSeg();

#if (ROUTE_VEL_MODE != 0U)
    /* velocity mode: odom origin = now, raw yaw reference = now, no home yet */
    rvYaw0Raw    = IMU_GetYaw();
    rvHomeActive = 0U;
    rvHomeDone   = 0U;
    rvTick       = HAL_GetTick();
#endif

    Debug_Log("ROUTE", "start seg=0/%u laps=%u", (unsigned int)ROUTE_SEG_COUNT,
              (unsigned int)routeLapTotal);
    Display_RouteRefresh();
}

/**
 * @brief  中止跑图（四轮急停 + 失能 + 状态置 ABORT）
 * @param  无
 * @retval 无
 * @note   仅在确实跑图中时才发急停，避免空闲时刷一堆无意义的停止日志。
 */
void Route_Abort(void)
{
    uint8_t addr;

    if (routeState == ROUTE_STATE_RUN)
    {
        Mecanum_Stop();
        routeState = ROUTE_STATE_ABORT;
        Debug_Log("ROUTE", "abort seg=%u", (unsigned int)routeSegIdx);
    }

    routePhase      = ROUTE_PHASE_WAIT;
    routePhaseSent  = 0U;
    routeHoldActive = 0U;       /* 清掉"停稳等待"，避免下次启动带残留 */

    /* [2026-10-07] aborted in the middle of a turn -> odom x/y must be unfrozen,
       otherwise the car would drive with frozen coordinates afterwards. */
    Chassis_OdomHoldXY(0U);

    /* 失能四轮：避免退出页面后电机长时间抱轴发热 */
    for (addr = MOTOR_ADDR_MIN; addr <= MOTOR_ADDR_MAX; addr++)
    {
        Motor_Disable(addr);
    }

    Display_RouteRefresh();
}

/**
 * @brief  跑图状态机推进（主循环调用，非阻塞）
 * @param  无
 * @retval 无
 * @note   每拍只推进"一个动作"：要么下发当前阶段的命令，要么判一次是否到位，
 *         保证任何一次调用都很快返回（OLED 刷新/按键响应不受影响）。
 */
void Route_Update(void)
{
    const RouteSeg_t *seg;

    if (routeState != ROUTE_STATE_RUN)
    {
        return;
    }

    /* 跑图过程中刷新页面（路段/状态行）：只置脏标志，实际重绘由 OLED 侧
       按 REFRESH_PERIOD_MS 节流，不会高频全量重绘 */
    Display_RouteRefresh();

    /* 本段总超时兜底（旋转阶段 + 平移阶段一起计时） */
    if ((HAL_GetTick() - routeSegStartTick) >= ROUTE_SEG_TIMEOUT_MS)
    {
        Debug_Log("ROUTE", "seg%u timeout", (unsigned int)routeSegIdx);
        Route_Abort();
        return;
    }

    /* ★速度模式（纯陀螺仪）：固定 20ms 节拍跑独立执行器，不再走位置模式状态机。
       下面这段位置模式代码保留编译（切 ROUTE_VEL_MODE=0 即可回退），不参与运行。 */
#if (ROUTE_VEL_MODE != 0U)
    if (Route_VelModeOn() != 0U)
    {
        if ((HAL_GetTick() - rvTick) < ROUTE_VEL_TICK_MS)
        {
            return;
        }
        rvTick = HAL_GetTick();
        Route_VelUpdate();
        return;
    }
#endif

    seg = &routeTbl[routeSegIdx];

    switch (routePhase)
    {
        case ROUTE_PHASE_TURN:
            if (routePhaseSent == 0U)
            {
                if (seg->turn_deg != 0)
                {
                    /* 下发原地旋转并等回传 */
                    Route_IssueRot(seg->turn_deg);
                    routePhaseSent      = 1U;
                    routePhaseTick      = HAL_GetTick();
                    routeFallbackLogged = 0U;

                    /* 设定**转角闭环**的目标 θ = 转前 θ + 表里转角（Route_IssueRot
                       刚刚记下了转前的 θ），并复位轮次/预算计时 */
                    routeYawTarget = routeTurnYaw0 + (float)seg->turn_deg;
                    routeYawRound  = 0U;
                    routeYawStart  = HAL_GetTick();

                    Debug_Log("ROUTE", "seg%u turn %d",
                              (unsigned int)routeSegIdx, (int)seg->turn_deg);
                    Display_RouteRefresh();

                    /* [2026-10-07] Freeze odom x/y for the whole turn phase:
                       rotating in place must not change the coordinates at all
                       (only the heading, which comes from the IMU, changes).
                       Driving/strafe still accumulate normally. */
                    Chassis_OdomHoldXY(1U);
                }
                else
                {
                    /* 本段不需要转：直接换到平移阶段（下一拍下发） */
                    routePhase     = ROUTE_PHASE_MOVE;
                    routePhaseSent = 0U;
                }
                break;
            }

            if (Route_PhaseArrived() != 0U)
            {
                /* 到位 -> **四轮同拍瞬停 + 静置**，等残余运动结束再往下走。
                   这是"直行/转弯到点位减速段出现微小横移"的直接对治手段，
                   见 Route_StopAndSettle 的说明；holdMs=0 时本函数立刻放行。 */
                if (Route_StopAndSettle() == 0U)
                {
                    break;                  /* 还在静置，本拍不推进 */
                }

                /* **转角闭环**：按 HWT101CT 回传的 θ 校转角。
                   返回 0 = 刚下发补转，要重新"等到位 + 停稳"再复测，
                   所以这里要把"等到位"的计时一并重开（否则会立刻被段超时/兜底判到位）。 */
                if (Route_YawLoop() == 0U)
                {
                    routePhaseTick      = HAL_GetTick();
                    routeFallbackLogged = 0U;
                    Display_RouteRefresh();
                    break;
                }

                /* 只做诊断提示（不中止），随后继续下一阶段 */
                Route_CheckTurnAngle(seg->turn_deg);
                Route_LogPhaseDone();       /* 证据日志：哪些轮被命令 / 哪些回传到位 */

                /* 进入"朝向稳定等待"：等 IMU 把转弯后的新朝向送上来，
                   再规划本段平移方向（否则会按转弯前的旧朝向算，出现横向平移）。
                   若朝向其实已经到位，Route_YawSettleWait 会立刻放行、不额外等待。 */
                routeSettleActive = 1U;
                routeSettleYaw    = Chassis_GetOdomYaw();
                routeSettleNeed   = (float)seg->turn_deg;
                routeSettleTick   = HAL_GetTick();
                routeSettleStart  = HAL_GetTick();

                /* 顺势开始"静止零漂"测量：转弯到位后车静止的这段时间里，
                   测 yaw 自身的漂移速率（见 ROUTE_DRIFT_CAL_MS / Chassis_YawDriftEnd） */
                Chassis_YawDriftBegin();

                /* 旋转到位 -> 进入本段平移阶段（下一拍下发） */
                routePhase     = ROUTE_PHASE_MOVE;
                routePhaseSent = 0U;
                routePhaseTick = HAL_GetTick();

                /* [2026-10-07] Turn finished -> unfreeze odom x/y so that the
                   following translation accumulates position again. */
                Chassis_OdomHoldXY(0U);
            }
            break;

        case ROUTE_PHASE_MOVE:
            if (routePhaseSent == 0U)
            {
                /* 若本段前面刚做过旋转，先等朝向稳定再规划方向 */
                if (Route_YawSettleWait() != 0U)
                {
                    break;
                }

                if ((seg->x == ROUTE_NO_MOVE) || (seg->y == ROUTE_NO_MOVE))
                {
                    /* 本段只转不走：直接完成本段 */
                    Route_SegDone();
                }
                else
                {
                    int32_t bx = 0;
                    int32_t by = 0;

                    /* 先转后走的第二段 / 只走不转：按路段表语义下发本段位移
                       （车体系模式直接用表里数值，绝对点模式先做坐标变换） */
                    if (Route_IssueMoveAbs(seg->x, seg->y, &bx, &by) == 0U)
                    {
                        /* 已在目标点 / 本段位移为 0：不下发、不等待，直接完成本段 */
                        Debug_Log("ROUTE", "seg%u already at target",
                                  (unsigned int)routeSegIdx);
                        Route_SegDone();
                        break;
                    }

                    routePhaseSent      = 1U;
                    routePhaseTick      = HAL_GetTick();
                    routeFallbackLogged = 0U;

                    /* 日志把"本段字段"和"算出的车身系增量"都打出来：
                       字段含义随 ROUTE_SEG_RELATIVE 变化（相对=前进/右移；绝对=目标点），
                       fwd/right 始终是最终下发的车身系增量 */
                    Debug_Log("ROUTE", "seg%u move in=%d,%d fwd=%d right=%d yaw=%d imu=%u",
                              (unsigned int)routeSegIdx, (int)seg->x, (int)seg->y,
                              (int)bx, (int)by,
                              (int)(Chassis_GetOdomYaw() * 10.0f),
                              (unsigned int)IMU_IsOnline());
                    Display_RouteRefresh();
                }
                break;
            }

            if (Route_PhaseArrived() != 0U)
            {
                /* 到位 -> **四轮同拍瞬停 + 静置**：直行到点位的减速段会出现
                   微小横移（四轮独立闭环、后停的推先停的），这里把它按住并等稳。
                   见 Route_StopAndSettle 的说明；holdMs=0 时本函数立刻放行。 */
                if (Route_StopAndSettle() == 0U)
                {
                    break;                  /* 还在静置，本拍不推进 */
                }

                Route_LogPhaseDone();       /* 证据日志：哪些轮被命令 / 哪些回传到位 */

#if (ROUTE_SEG_RELATIVE == 0)
                /* [2026-10-07] Landing-point closed loop (absolute mode only):
                   re-issue the SAME target. Route_IssueMoveAbs recomputes the body-frame
                   delta from the MEASURED position, so slip / scale error is corrected.
                   Returns 1 -> still off by a non-zero rounded delta: walk one more leg
                   and go back to "arrived + settle". Returns 0 -> rounded delta is 0,
                   i.e. already at pulse resolution, done. */
                if ((routePosRound < (uint8_t)ROUTE_POS_LOOP_MAX) &&
                    (seg->x != ROUTE_NO_MOVE) && (seg->y != ROUTE_NO_MOVE))
                {
                    int32_t bx2 = 0;
                    int32_t by2 = 0;

                    routePosRound++;

                    if (Route_IssueMoveAbs(seg->x, seg->y, &bx2, &by2) != 0U)
                    {
                        routePhaseSent      = 1U;
                        routePhaseTick      = HAL_GetTick();
                        routeFallbackLogged = 0U;
                        routeHoldActive     = 0U;   /* new sub-leg: reset the settle timer,
                                                       otherwise Route_StopAndSettle would pass
                                                       immediately using the stale tick */
                        Debug_Log("ROUTE", "seg%u pos loop %u/%u fwd=%d right=%d",
                                  (unsigned int)routeSegIdx, (unsigned int)routePosRound,
                                  (unsigned int)ROUTE_POS_LOOP_MAX, (int)bx2, (int)by2);
                        Display_RouteRefresh();
                        break;              /* one more leg queued: back to "arrived + settle" */
                    }
                }

                /* 绝对模式才有"目标点"，才做落点误差提示。
                   注意：必须在"停稳"之后读，否则读到的是还在收尾的坐标 */
                Route_CheckPosErr(seg);
#endif
                Route_SegDone();
                Display_RouteRefresh();
            }
            break;

        default:
            break;
    }
}

/**
 * @brief  查询是否正在跑图
 * @param  无
 * @retval 1=跑图中（RUN），0=其它状态
 */
uint8_t Route_IsActive(void)
{
    return (routeState == ROUTE_STATE_RUN) ? 1U : 0U;
}

/**
 * @brief  查询是否跑图完成
 * @param  无
 * @retval 1=全部路段跑完（DONE），0=其它状态
 */
uint8_t Route_IsDone(void)
{
    return (routeState == ROUTE_STATE_DONE) ? 1U : 0U;
}

/**
 * @brief  取当前路段号（0 起）
 * @param  无
 * @retval 当前段索引；跑完后等于总段数
 */
uint8_t Route_GetSegIdx(void)
{
    return routeSegIdx;
}

/**
 * @brief  取总路段数
 * @param  无
 * @retval 路段数（由 ROUTE_SEG_LIST 自动计算）
 */
uint8_t Route_GetSegCount(void)
{
    return ROUTE_SEG_COUNT;
}

/**
 * @brief  取本段/本阶段"哪些轮被下过命令"的位掩码
 * @param  无
 * @retval bit0~bit3 依次对应 FL/FR/RL/RR：1=该轮本次有命令（应该转动），
 *         0=该轮位移为 0（不下发命令，保持不动）
 * @note   用来区分"轮子不动"的两种完全不同的原因：
 *           - 对应位为 0 → **正常**（例如 45° 斜行只需两个轮，另两个轮位移恰好为 0）；
 *           - 对应位为 1 却没动 → 命令丢失/被驱动忽略/电机异常，需要查 CAN 与电机。
 *         页面第 2 行把它显示成 `w:1001`（依次 FL/FR/RL/RR）。
 */
uint8_t Route_GetWheelMask(void)
{
    uint8_t i;
    uint8_t m = 0U;

    for (i = 0U; i < (uint8_t)MOTOR_NUM; i++)
    {
        if (routeSegClk[i] != 0)
        {
            m |= (uint8_t)(1U << i);
        }
    }

    return m;
}

/**
 * @brief  取跑图状态文字（页面显示用）
 * @param  无
 * @retval "IDLE" / "RUN" / "DONE" / "ABORT"
 */
const char *Route_GetStateText(void)
{
    switch (routeState)
    {
        case ROUTE_STATE_RUN:   return "RUN";
        case ROUTE_STATE_DONE:  return "DONE";
        case ROUTE_STATE_ABORT: return "ABORT";
        default:                return "IDLE";
    }
}

/**
 * @brief  取当前段阶段文字（页面显示用）
 * @param  无
 * @retval "TURN" / "MOVE" / "WAIT"
 */
const char *Route_GetPhaseText(void)
{
    switch (routePhase)
    {
        case ROUTE_PHASE_TURN:  return "TURN";
        case ROUTE_PHASE_MOVE:  return "MOVE";
        default:                return "WAIT";
    }
}

/**
 * @brief  取当前段目标文字（页面第4行显示用）
 * @param  无
 * @retval 形如 "T90 (500,500)" / "to(0,500)" / "turn90" / "-" 的静态缓冲指针
 * @note   跑完后显示最后一段的目标，便于人工核对终态。
 */
const char *Route_GetSegText(void)
{
    static char buf[24];
    const RouteSeg_t *seg;
    uint8_t idx;
    uint8_t hasMove;

    if (ROUTE_SEG_COUNT == 0U)
    {
        return "-";
    }

    idx = (routeSegIdx < ROUTE_SEG_COUNT) ? routeSegIdx
                                          : (uint8_t)(ROUTE_SEG_COUNT - 1U);
    seg     = &routeTbl[idx];
    hasMove = ((seg->x != ROUTE_NO_MOVE) && (seg->y != ROUTE_NO_MOVE)) ? 1U : 0U;

#if (ROUTE_SEG_RELATIVE != 0)
    /* 车体系（相对）模式：显示"转角 + 前进/右移" */
    if ((seg->turn_deg != 0) && (hasMove != 0U))
    {
        snprintf(buf, sizeof(buf), "T%d f%d r%d",
                 (int)seg->turn_deg, (int)seg->x, (int)seg->y);
    }
    else if (seg->turn_deg != 0)
    {
        snprintf(buf, sizeof(buf), "turn%d", (int)seg->turn_deg);
    }
    else if (hasMove != 0U)
    {
        snprintf(buf, sizeof(buf), "f%d r%d", (int)seg->x, (int)seg->y);
    }
    else
    {
        snprintf(buf, sizeof(buf), "-");
    }
#else
    if ((seg->turn_deg != 0) && (hasMove != 0U))
    {
        snprintf(buf, sizeof(buf), "T%d (%d,%d)",
                 (int)seg->turn_deg, (int)seg->x, (int)seg->y);
    }
    else if (seg->turn_deg != 0)
    {
        snprintf(buf, sizeof(buf), "turn%d", (int)seg->turn_deg);
    }
    else if (hasMove != 0U)
    {
        snprintf(buf, sizeof(buf), "to(%d,%d)", (int)seg->x, (int)seg->y);
    }
    else
    {
        snprintf(buf, sizeof(buf), "-");
    }
#endif

    return buf;
}
