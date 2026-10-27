/**
 ******************************************************************************
 * @file    bsp_mission.c
 * @brief   赛题任务执行器实现（P9 步骤1：跑点位 —— STEP 单步 + AUTO 连续双模式）
 * @author  nx_car
 * @date    2026-09-14
 ******************************************************************************
 * @note   【本文件是后续所有赛题开发的主战场】步骤1 只实现"点位导航 + 到点停 2s"。
 *
 *         【两种模式】
 *           - STEP 单步：按 KEY5 执行当前点位，执行完**停在 DONE** 等下一次按键；
 *           - AUTO 连续：按 KEY5 一键启动，`Ms_PointDone()` 里自动 `msIdx++`
 *             并复位阶段接着跑，直到最后一个点位；跑完进 MS_STATE_ALLDONE。
 *           两者共用同一套"启动 + 单点执行"逻辑，唯一分叉点就是 `Ms_PointDone()`。
 *
 *         【点位类型】
 *           - MS_PT_WORK 作业点：到位停 MS_ARRIVE_HOLD_MS(2s) -> 执行 act -> 完成；
 *           - MS_PT_VIA  中转点：**只路过** —— 到位后不停留、不执行动作，直接进下一个。
 *             用途：场地只有中间十字灰色车道可行驶，两点之间插中转点把斜线拆成
 *             正交折线，车就不会开出车道压到浅色区/圆环区。
 *
 *         【运动策略为什么与 bsp_route.c 长得几乎一样】
 *           需求要求"点位之间运动要丝滑"，且已确认**沿用 P7 既有策略**。所以这里的
 *           到位判定 / 同拍停 / 转角闭环 / 朝向稳定等待**逐条对应** bsp_route.c：
 *             ① Ms_PhaseArrived      <- Route_PhaseArrived（回传优先 + 脉冲容差兜底）
 *             ② Ms_StopAndSettle     <- Route_StopAndSettle（四轮同拍 0xFE 停 + 静置）
 *             ③ Ms_YawLoop           <- Route_YawLoop（按 HWT101CT 的 θ 补转）
 *             ④ Ms_YawSettleWait     <- Route_YawSettleWait（等新朝向再规划方向）
 *           **参数不重复定义**：全部读 `Route_GetCfg()` 那一套运行期参数，
 *           所以"跑图"与"赛题"改一处两处都生效（见 bsp_mission_cfg.h 文件头）。
 *           为什么复制而不是调用：bsp_route 的这些函数都是 static，且跑图是
 *           "按编译期路段表走"，与赛题"按点位表走"的调度语义不同；
 *           bsp_mecanum / bsp_chassis / bsp_motor 这些底层则**直接复用**。
 *
 *         【与 P7 的差异（有意为之）】
 *           - 旋转目标是**绝对朝向**（点位表的 yaw_deg），转角增量由
 *             "目标 yaw − 实测 yaw" 现算，因此每到一个点位都把朝向拉回同一基准，
 *             朝向误差不逐点累积；
 *           - 平移目标是**绝对坐标点**，用"目标点 − 实测 ODOM 坐标"算车身系增量，
 *             每步都朝同一个世界点回纠，位置误差不逐点累积；
 *           - 作业点到点后多一个**停留阶段**（MS_ARRIVE_HOLD_MS，默认 2s）。
 *
 *         硬约束：全部函数非阻塞，禁止 HAL_Delay；日志统一 [MS] 标签，禁 %f。
 ******************************************************************************
 */
#include "bsp_mission.h"
#include "bsp_motor.h"
#include "bsp_mecanum.h"
#include "bsp_chassis.h"
#include "bsp_imu.h"        /* IMU_IsOnline / IMU_GetYaw：日志标注朝向数据是否新鲜 */
#include "bsp_route.h"      /* Route_GetCfg() 运行期参数 + ROUTE_* 编译期阈值（口径一致） */
#include "bsp_rpi.h"        /* RPi_ReqRcp/ReqRz/ReqShot/ReqCcg：到点停稳后发 v2.0 视觉请求（协议 §3） */
#include "bsp_oled_ui.h"
#include "bsp_debug.h"
#include <math.h>
#include <stdio.h>

/* ===================== 点位表（由 X-macro 展开生成数组） ===================== */

/* 单个点位的内容（字段顺序必须与 bsp_mission_cfg.h 的 MS_POINT(...) 参数顺序一致） */
typedef struct {
    const char *name;   /* 点位显示名（≤4 字符，页面第 1 行用） */
    int32_t     x;      /* 目标 x，单位 mm，相对出发点，正 = 出发点前方 */
    int32_t     y;      /* 目标 y，单位 mm，相对出发点，正 = 出发点右侧 */
    int16_t     yaw;    /* 目标朝向，单位 deg，±180，俯视逆时针为正 */
    uint8_t     kind;   /* 点位类型，MsPtKind_t：VIA=只路过 / WORK=停 2s + 执行动作 */
    uint8_t     act;    /* 到位后应执行的动作，MsAct_t（**只对 WORK 有效**；步骤1 仅占位） */
    uint8_t     vcmd;   /* 到点**停稳后**要向树莓派发出的 v2.0 请求，MsReq_t；
                           MS_REQ_NONE = 不发（协议 §3：请求由"车停稳在作业点"驱动） */
} MsPoint_t;

/* 把 MS_POINT_LIST 的每一行 MS_POINT(a,...,h) 展开成结构体初始化项 */
#define MS_POINT(id, name, x, y, yaw, kind, act, vcmd) \
    { (name), (x), (y), (yaw), (uint8_t)(kind), (uint8_t)(act), (uint8_t)(vcmd) },
static const MsPoint_t msTable[] = { MS_POINT_LIST };
#undef MS_POINT

/* 总分点数（自动计算，增删点位不用改这里） */
#define MS_POINT_COUNT      ((uint8_t)MS_POINT_NUM)

/* ===================== 内部状态 ===================== */

static uint8_t   msActive    = 0U;              /* 页面活动标志：1=本页活动 */
static MsMode_t  msMode      = MS_MODE_STEP;    /* 运行模式：STEP 单步 / AUTO 连续（KEY3 切换） */
static MsState_t msState     = MS_STATE_IDLE;   /* 总状态 */
static MsPhase_t msPhase     = MS_PHASE_WAIT;   /* 当前点位所处阶段 */
static uint8_t   msIdx       = 0U;              /* 当前选中的点位号（0 起，跨进出页面保留） */

static uint8_t   msPhaseSent = 0U;              /* 本阶段命令是否已下发：0=未发，1=已发等待到位 */
static uint8_t   msFallbackLogged = 0U;         /* 本阶段是否已打印"回传超时退脉冲"提示 */
static uint8_t   msBaseValid = 0U;              /* 脉冲兜底基准是否有效（读位置成功） */

/* ---- 到位后的"统一停轮 + 停稳"（对应 RouteCfg_t.holdMs） ---- */
static uint8_t   msHoldActive = 0U;             /* 1=四轮已同拍停住，正在静置等残余运动结束 */
static uint32_t  msHoldTick   = 0U;             /* 开始静置的时刻 */

/* ---- 到点停留（赛题要求的"停 2 秒"） ---- */
static uint32_t  msArriveTick = 0U;             /* 进入 HOLD 阶段的时刻 */

/* ---- 转角闭环（按 HWT101CT 回传的 θ 补转，对应 Route_YawLoop） ---- */
static float     msYawTarget  = 0.0f;           /* 本阶段旋转结束时应达到的**绝对 θ**（deg） */
static uint8_t   msYawRound   = 0U;             /* 本阶段已补转轮次 */
static uint32_t  msYawStart   = 0U;             /* 本阶段转角闭环开始时刻（预算计时） */
static float     msTurnYaw0   = 0.0f;           /* 旋转**下发前**的 θ（量实测转角用） */
static float     msTurnNeed   = 0.0f;           /* 本次命令的转角（正=逆时针），朝向等待用 */

static int32_t   msSegBase[MOTOR_NUM];          /* 本阶段起点四轮位置（下发脉冲域） */
static int32_t   msSegClk[MOTOR_NUM];           /* 本阶段目标增量脉冲（判到位用） */

static uint32_t  msStartTick  = 0U;             /* 本点位开始时刻（点位超时计时） */
static uint32_t  msPhaseTick  = 0U;             /* 本阶段下发时刻（回传超时计时） */

/* ---- 转弯后的"朝向稳定等待"（对应 Route_YawSettleWait） ---- */
static uint8_t   msSettleActive = 0U;           /* 1=正在等朝向稳定（本段平移先不下发） */
static float     msSettleYaw    = 0.0f;         /* 上次观察到的朝向 */
static uint32_t  msSettleTick   = 0U;           /* 朝向最近一次"变了"的时刻 */
static uint32_t  msSettleStart  = 0U;           /* 开始等待的时刻（硬上限计时） */

/* ===================== 内部工具 ===================== */

/**
 * @brief  浮点转 int32（四舍五入，避免正负都向 0 截断带来的 1mm 偏差）
 * @param  v: 输入值
 * @retval 最接近的整数
 */
static int32_t Ms_RoundF(float v)
{
    return (int32_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

/**
 * @brief  把角度归一化到 ±180° 区间
 * @param  d: 输入角度，单位 deg
 * @retval 归一化后的角度，范围 (-180, 180]
 */
static float Ms_NormDeg(float d)
{
    while (d > 180.0f)
    {
        d -= 360.0f;
    }
    while (d < -180.0f)
    {
        d += 360.0f;
    }
    return d;
}

/**
 * @brief  读四轮当前位置（换算到下发脉冲域）
 * @param  out: 输出四轮位置（FL/FR/RL/RR），下标 0~3
 * @retval 1=四轮全部读到；0=有任一读不到
 * @note   与 P4 bsp_chassis 的 Chassis_ReadPos、P7 的 Route_ReadPos 做法一致：
 *         0x36 上报的是编码器计数，比下发脉冲大 MOTOR_READ_SCALE_DEN/NUM 倍，
 *         统一换算到下发脉冲域，才能与 Mecanum_MoveBy 的目标脉冲直接比较。
 */
static uint8_t Ms_ReadPos(int32_t out[MOTOR_NUM])
{
    int32_t raw;
    uint8_t i;

    for (i = 0U; i < (uint8_t)MOTOR_NUM; i++)
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
static void Ms_ReachClearAll(void)
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
 * @note   位移为 0 的轮（如横移只需部分轮）不会收到命令，也不会有到位帧，
 *         必须排除，否则每段都要白等到 ROUTE_REACH_FALLBACK_MS 才靠脉冲兜底过去。
 */
static uint8_t Ms_ReachAll(void)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t)MOTOR_NUM; i++)
    {
        if (msSegClk[i] == 0)
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
static uint8_t Ms_PulseArrived(void)
{
    int32_t cur[MOTOR_NUM];
    int32_t err;
    uint8_t i;

    if (Ms_ReadPos(cur) == 0U)
    {
        return 0U;
    }

    for (i = 0U; i < (uint8_t)MOTOR_NUM; i++)
    {
        if (msSegClk[i] == 0)
        {
            continue;               /* 本阶段没给这个轮下命令，不参与判定 */
        }
        err = (cur[i] - msSegBase[i]) - msSegClk[i];
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
 * @brief  本阶段是否已到位（对应 Route_PhaseArrived）
 * @param  无
 * @retval 1=已到位；0=还没到
 * @note   优先用电机主动回传的到位帧（电机自己的判据，窗口仅 0.1~0.3°，最可靠）；
 *         下发后超过 ROUTE_REACH_FALLBACK_MS 还没收齐，就退回脉冲容差判定并
 *         打印一次提示（防漏帧/菜单没配 Reached 时挂死）。
 */
static uint8_t Ms_PhaseArrived(void)
{
    if (Ms_ReachAll() != 0U)
    {
        return 1U;
    }

    if ((HAL_GetTick() - msPhaseTick) < ROUTE_REACH_FALLBACK_MS)
    {
        return 0U;
    }

    if (msFallbackLogged == 0U)
    {
        msFallbackLogged = 1U;
        Debug_Log("MS", "reach timeout, fallback to pulse");
    }

    if (msBaseValid == 0U)
    {
        return 0U;      /* 连脉冲基准都没有：只能继续等回传，最终由点位超时兜底 */
    }

    return Ms_PulseArrived();
}

/**
 * @brief  到位后的"四轮同拍瞬停 + 静置等待"（对应 Route_StopAndSettle）
 * @param  无
 * @retval 1=静置结束，可以推进下一阶段；0=还在静置（本拍不要推进）
 * @note   四个轮子是各自独立的闭环位置模式，脉冲数不同则到位时刻不同：先到位的
 *         轮停下并保持，后到位的轮还在转 —— 在麦轮上会被分解成"整车横移 + 微转"，
 *         现场表现就是"直行到点位的减速段出现微小横移"。
 *         本函数在判定到位的那一刻给四轮同拍发 0xFE 瞬停，再静置
 *         `RouteCfg_t.holdMs`（默认 100ms）等残余运动结束，然后才允许推进。
 *         【为什么不会截掉距离】停轮发生在"所有有命令的轮都已到位"之后，
 *         电机的到位窗口只有 0.1~0.3°（≈0.3mm 轮面位移），最多截掉不到 1mm。
 */
static uint8_t Ms_StopAndSettle(void)
{
    if (msHoldActive == 0U)
    {
        uint8_t addr;

        for (addr = MOTOR_ADDR_MIN; addr <= MOTOR_ADDR_MAX; addr++)
        {
            Motor_Stop(addr);           /* 0xFE 同拍瞬停（内部已等空闲发送邮箱） */
        }
        msHoldActive = 1U;
        msHoldTick   = HAL_GetTick();
        return 0U;
    }

    if ((HAL_GetTick() - msHoldTick) < (uint32_t)Route_GetCfg()->holdMs)
    {
        return 0U;
    }

    msHoldActive = 0U;
    return 1U;
}

/**
 * @brief  放行"朝向等待"（统一出口：结束静止零漂测量 + 清标志）
 * @param  无
 * @retval 恒为 0（供调用处直接 return）
 * @note   无论走哪条放行分支都要经过这里，保证 Chassis_YawDriftEnd() 有头有尾。
 */
static uint8_t Ms_SettleRelease(void)
{
    msSettleActive = 0U;
    Chassis_YawDriftEnd();
    return 0U;
}

/**
 * @brief  转弯后的"朝向等待"（对应 Route_YawSettleWait）
 * @param  无
 * @retval 1=还要继续等（本拍不下发平移）；0=可以规划下发
 * @note   背景：IMU 偏航角按帧输出，转弯刚结束时读到的可能还是转弯前的旧值；
 *         若立刻按该值规划方向，就会出现"明明转了 90°，却按 0° 算"的横向平移。
 *         三级判据：
 *           1) 朝向已到达本点位目标（|误差| ≤ RouteCfg_t.yawTolDeg）→ 立即放行；
 *           2) 朝向**确实动过**（方向一致、已走过命令角一半）后，连续
 *              ROUTE_YAW_SETTLE_MS 变化 ≤0.5° → 视为稳定放行（兼容转过头）；
 *           3) 超过 ROUTE_YAW_SETTLE_MAX_MS 仍未达标 → 放行并报警。
 */
static uint8_t Ms_YawSettleWait(void)
{
    float y;
    float diff;
    float dNow;
    float dErr;
    float needAbs;
    float doneAbs;

    if (msSettleActive == 0U)
    {
        return 0U;
    }

    /* 静止零漂测量窗口：转弯到位后至少静置 ROUTE_DRIFT_CAL_MS 再放行。
       这段静止时间里车不动，yaw 若还在缓慢变化就是陀螺零漂。 */
    if ((uint32_t)(HAL_GetTick() - msSettleStart) < (uint32_t)ROUTE_DRIFT_CAL_MS)
    {
        return 1U;
    }

    y    = Chassis_GetOdomYaw();
    dNow = Ms_NormDeg(y - msTurnYaw0);          /* 实测转角（相对本次旋转下发前） */

    /* 1) 已到达期望值：立刻放行（正常情况走这条，不额外等待）。
       判据用"目标 θ − 实测 θ"再归一化，与转角闭环同一口径：
       像"目标 180°、实测 −179.5°"这种回绕情况不会被误判成"差 359.5°"，
       否则每转 180° 都会白等到 ROUTE_YAW_SETTLE_MAX_MS 并报 yaw NOT ready。 */
    dErr = Ms_NormDeg(msYawTarget - y);
    if (dErr < 0.0f)
    {
        dErr = -dErr;
    }
    if (dErr <= (float)Route_GetCfg()->yawTolDeg)
    {
        Debug_Log("MS", "pt%u yaw ready yaw=%d (waited %u ms)",
                  (unsigned int)msIdx, (int)(y * 10.0f),
                  (unsigned int)(HAL_GetTick() - msSettleStart));
        return Ms_SettleRelease();
    }

    /* 2) 只有"朝向确实动过"才允许用稳定判据收尾，避免把旧值当稳定提前放行 */
    needAbs = (msTurnNeed < 0.0f) ? -msTurnNeed : msTurnNeed;
    doneAbs = (dNow < 0.0f) ? -dNow : dNow;
    if (((dNow >= 0.0f) == (msTurnNeed >= 0.0f)) && (doneAbs >= (needAbs * 0.5f)))
    {
        diff = y - msSettleYaw;
        if (diff < 0.0f)
        {
            diff = -diff;
        }
        if (diff > 0.5f)
        {
            msSettleYaw  = y;               /* 朝向还在变：记下新值并重新计时 */
            msSettleTick = HAL_GetTick();
            return 1U;
        }
        if ((HAL_GetTick() - msSettleTick) >= (uint32_t)ROUTE_YAW_SETTLE_MS)
        {
            Debug_Log("MS", "pt%u yaw settled yaw=%d d=%d (waited %u ms)",
                      (unsigned int)msIdx, (int)(y * 10.0f), (int)(dNow * 10.0f),
                      (unsigned int)(HAL_GetTick() - msSettleStart));
            return Ms_SettleRelease();
        }
    }

    /* 3) 等太久仍未到位：放行并报警，避免一直等下去 */
    if ((HAL_GetTick() - msSettleStart) >= (uint32_t)ROUTE_YAW_SETTLE_MAX_MS)
    {
        Debug_Log("MS", "pt%u yaw NOT ready yaw=%d d=%d (cmd=%d) -> check IMU refresh",
                  (unsigned int)msIdx, (int)(y * 10.0f), (int)(dNow * 10.0f),
                  (int)(msTurnNeed * 10.0f));
        return Ms_SettleRelease();
    }

    return 1U;
}

/**
 * @brief  按"角度"下发一次原地旋转（只负责脉冲换算 + 下发，不动基准/标志）
 * @param  deg: 要转的角度，单位 deg（**正 = 逆时针 / 左转**，与页面/日志的 θ 同号）
 * @retval 本次下发的等效弧长（mm，带符号，仅用于日志打印）
 * @note   **调用方负责**：先 Ms_ReachClearAll() 清到位标志、按需刷新 msSegBase。
 *         本函数会写 msSegClk[]。
 *         符号与刻度：页面/θ 正 = 逆时针，而电机侧旋转正方向实测为顺时针，故取反；
 *         再乘**运行期旋转刻度** RouteCfg_t.rotScaleNum/Den（在线调参：转不够加大分子）。
 *         麦轮逆解（dx=dy=0）：FL/RL = +arc、FR/RR = −arc，与 bsp_mecanum.c 的
 *         Mecanum_MoveByRot 完全一致（为什么自己算：rotScale / rotVel 要在线可调，
 *         而 bsp_mecanum 里的对应量是编译期宏；加减速度档同样取编译期宏 MEC_ROT_ACC）。
 *         ⚠ 若以后 bsp_mecanum 的旋转逆解改了，这里必须同步改。
 */
static int32_t Ms_SendRotDeg(float deg)
{
    RouteCfg_t *cfg = Route_GetCfg();
    float   arc;
    float   scale;
    int32_t clk[MOTOR_NUM];
    uint8_t i;

    arc = -Mecanum_DegToArcMm(deg) * (float)cfg->rotScaleNum / (float)cfg->rotScaleDen;

    scale = MEC_PULSE_PER_MM * (float)MEC_PULSE_SCALE_NUM / (float)MEC_PULSE_SCALE_DEN;

    for (i = 0U; i < (uint8_t)MOTOR_NUM; i++)
    {
        float sg = ((i == 0U) || (i == 2U)) ? 1.0f : -1.0f;
        float p  = sg * (float)MEC_SIGN_WZ * arc * scale;

        clk[i] = (int32_t)((p >= 0.0f) ? (p + 0.5f) : (p - 0.5f));
        msSegClk[i] = clk[i];             /* 判到位基准，必须与下发一致 */
    }

    Mecanum_MoveByPulse(clk, cfg->rotVel, (uint8_t)MEC_ROT_ACC);

    return (int32_t)arc;
}

/**
 * @brief  下发"转到目标绝对朝向"的命令
 * @param  targetYaw: 目标朝向，单位 deg，±180，俯视逆时针为正（点位表的 yaw_deg）
 * @retval 1=已下发旋转命令；0=已在目标朝向上（无需旋转），直接进平移阶段
 * @note   转角增量 = 归一化(目标朝向 − 实测朝向)，所以**每到一个点位都把朝向
 *         拉回同一基准**，朝向误差不逐点累积。
 *         绝对值小于 MS_TURN_SKIP_DEG 时不下发（避免驱动器在目标附近反复找位、
 *         越找越抖），残差由 RouteCfg_t.yawTolDeg（默认 2°）覆盖。
 *         记下发前的 θ（msTurnYaw0），供到位后量实测转角与**转角闭环**设定目标。
 */
static uint8_t Ms_IssueRot(float targetYaw)
{
    int32_t arc;
    float   d = Ms_NormDeg(targetYaw - Chassis_GetOdomYaw());

    if ((d < MS_TURN_SKIP_DEG) && (d > -MS_TURN_SKIP_DEG))
    {
        Debug_Log("MS", "pt%u turn skip d=%d (already aligned)",
                  (unsigned int)msIdx, (int)(d * 10.0f));
        return 0U;
    }

    Ms_ReachClearAll();
    msBaseValid = Ms_ReadPos(msSegBase);
    msTurnYaw0  = Chassis_GetOdomYaw();   /* 记下发前 θ：到位后用它量实测转角 */
    msTurnNeed  = d;
    msYawTarget = targetYaw;              /* 闭环直接朝这个**绝对**目标补 */
    msYawRound  = 0U;
    msYawStart  = HAL_GetTick();

    arc = Ms_SendRotDeg(d);

    Debug_Log("MS", "pt%u rot d=%d target=%d arc=%d clk=%d,%d,%d,%d vel=%u",
              (unsigned int)msIdx, (int)(d * 10.0f), (int)(targetYaw * 10.0f), (int)arc,
              (int)msSegClk[0], (int)msSegClk[1], (int)msSegClk[2], (int)msSegClk[3],
              (unsigned int)Route_GetCfg()->rotVel);

    return 1U;
}

/**
 * @brief  转角闭环：按 HWT101CT 回传的 θ 与目标比，超容差就补转（对应 Route_YawLoop）
 * @param  无
 * @retval 1=转角已达标（或超轮次/超预算，**放行**）；0=刚下发补转，等它走完再来判
 * @note   为什么必须做：转角本身是开环的（角度→弧长→脉冲，只靠标定），原地转靠
 *         麦轮横向刮擦、打滑不可避免，实测常差 2°~10°；而"下一段往哪走"是用
 *         **实测 θ** 反算的 —— θ 偏 Δ° 则平移方向歪 Δ°（走 1740mm、Δ=2° 横偏约 61mm）。
 *         控制律：纯 **P + 死区 + 限幅**，不做 PID。本环是"每点位一次、间隔几百 ms"
 *         的离散低频回路，被控对象（驱动器内部脉冲位置环）本身已是闭环，测量还有
 *         20~50ms 死区：加 I 会 windup 冲过头、加 D 会被 0.1° 量化噪声放大成抖动。
 *         防"补过头"四道防护：
 *           · 只补误差的 RouteCfg_t.yawCorrPct%（默认 70%）；
 *           · 单次限幅 RouteCfg_t.yawMaxCorrX10（默认 15°），超了打 `yaw corr clamp`；
 *           · 最多 RouteCfg_t.yawMaxRound 轮（默认 2）；
 *           · 总预算 RouteCfg_t.yawBudgetMs（默认 1200ms），超了 `yaw giveup` 放行，
 *             **绝不中止**；
 *           每次补转都要重新"等到位 + 同拍停 + 静置"后才复测，保证读到的是
 *           补转**之后**的新 IMU 样本。
 *         RouteCfg_t.yawEnable = 0 即整段跳过本函数，退回纯开环转角。
 */
static uint8_t Ms_YawLoop(void)
{
    RouteCfg_t *cfg = Route_GetCfg();
    float   err;
    float   corr;
    int32_t corrX10;

    if (cfg->yawEnable == 0U)
    {
        return 1U;                  /* 关掉转角闭环：直接放行 */
    }

    /* 误差 = 目标 θ − 实测 θ，归一化到 ±180°（正 = 需要逆时针补转 = 左转） */
    err = Ms_NormDeg(msYawTarget - Chassis_GetOdomYaw());

    /* ---- 判据①：达标（容差内） ---- */
    if ((err <= (float)cfg->yawTolDeg) && (err >= -(float)cfg->yawTolDeg))
    {
        Debug_Log("MS", "pt%u yaw ok r=%u e=%d target=%d now=%d",
                  (unsigned int)msIdx, (unsigned int)msYawRound, (int)(err * 10.0f),
                  (int)(msYawTarget * 10.0f), (int)(Chassis_GetOdomYaw() * 10.0f));
        return 1U;
    }

    /* ---- 判据②：轮次用尽 / 超预算 → 放行并报警（绝不中止） ---- */
    if ((msYawRound >= cfg->yawMaxRound) ||
        ((HAL_GetTick() - msYawStart) >= (uint32_t)cfg->yawBudgetMs))
    {
        Debug_Log("MS", "pt%u yaw giveup r=%u e=%d (%s), check IMU/rotScale",
                  (unsigned int)msIdx, (unsigned int)msYawRound, (int)(err * 10.0f),
                  (msYawRound >= cfg->yawMaxRound) ? "round" : "budget");
        return 1U;
    }

    /* ---- 判据③：下发补转（只补误差的 yawCorrPct%，并做单次限幅） ---- */
    corr    = err * (float)cfg->yawCorrPct / 100.0f;
    corrX10 = (int32_t)(corr * 10.0f);
    if (corrX10 > (int32_t)cfg->yawMaxCorrX10)
    {
        corrX10 = (int32_t)cfg->yawMaxCorrX10;
        Debug_Log("MS", "pt%u yaw corr clamp", (unsigned int)msIdx);
    }
    else if (corrX10 < -(int32_t)cfg->yawMaxCorrX10)
    {
        corrX10 = -(int32_t)cfg->yawMaxCorrX10;
        Debug_Log("MS", "pt%u yaw corr clamp", (unsigned int)msIdx);
    }

    /* 单次补转都不到 1° 就不补了（免得驱动器在目标附近来回找位、越补越抖） */
    if ((corrX10 < 10) && (corrX10 > -10))
    {
        Debug_Log("MS", "pt%u yaw ok r=%u e=%d (tiny)",
                  (unsigned int)msIdx, (unsigned int)msYawRound, (int)(err * 10.0f));
        return 1U;
    }

    corr = (float)corrX10 / 10.0f;

    /* 下发补转：清到位标志 + 用"当前四轮位置"当新的判到位基准（顺序不能反） */
    Ms_ReachClearAll();
    msBaseValid = Ms_ReadPos(msSegBase);
    (void)Ms_SendRotDeg(corr);

    msYawRound++;

    Debug_Log("MS", "pt%u yaw corr d=%d e=%d r=%u",
              (unsigned int)msIdx, (int)corrX10, (int)(err * 10.0f),
              (unsigned int)msYawRound);

    return 0U;
}

/**
 * @brief  下发本阶段的车身系平移命令（含运行期平移刻度）
 * @param  dx_body: 车身系前进位移 mm（正=车头方向）
 * @param  dy_body: 车身系右移位移 mm（正=车身右侧）
 * @retval 无
 * @note   先清到位标志、再记录脉冲基准与目标，最后下发（顺序不能反）。
 *         走短/走长的比例误差按《调参手册》P4 章标定 MEC_PULSE_SCALE_NUM/DEN
 *         （已含在 Mecanum_MoveBy 内）；这里另有**运行期平移刻度**
 *         RouteCfg_t.distScaleNum/Den（在线调参用），与跑图共用同一套值。
 */
static void Ms_IssueMove(int32_t dx_body, int32_t dy_body)
{
    RouteCfg_t *cfg = Route_GetCfg();
    int32_t dx = dx_body;
    int32_t dy = dy_body;

    if (cfg->distScaleDen != 0U)
    {
        dx = (int32_t)(((float)dx_body * (float)cfg->distScaleNum) / (float)cfg->distScaleDen);
        dy = (int32_t)(((float)dy_body * (float)cfg->distScaleNum) / (float)cfg->distScaleDen);
    }

    Ms_ReachClearAll();
    msBaseValid = Ms_ReadPos(msSegBase);
    Mecanum_MoveBy(dx, dy, 0, msSegClk);
}

/**
 * @brief  下发"走到目标坐标点"的命令（绝对点语义）
 * @param  无
 * @retval 1=已下发位移命令；0=已在目标点（车身系增量四舍五入为 0，无需下发）
 * @note   全局增量 = 目标点 − 当前 ODOM 坐标，再按 bsp_chassis.h 的逆变换转到车身系：
 *             dx = dX·cosθ − dY·sinθ
 *             dy = dX·sinθ + dY·cosθ
 *         θ = Chassis_GetOdomYawTrue()（**真实坐标系**，= IMU 域朝向 − 坐标补偿，
 *             俯视逆时针为正），与里程计的位置口径一致（位置也走 Chassis_GetOdomX/Y 的真实坐标系）。
 *         增量为 0 时不下发：位置模式没有运动就不会回到位帧，等待会白等几秒。
 */
static uint8_t Ms_IssueMoveAbs(const MsPoint_t *p)
{
    float   theta = Chassis_GetOdomYawTrue() * MEC_RAD_PER_DEG;
    float   c     = cosf(theta);
    float   s     = sinf(theta);
    float   dX    = (float)p->x - Chassis_GetOdomX();
    float   dY    = (float)p->y - Chassis_GetOdomY();
    int32_t bx    = Ms_RoundF((dX * c) - (dY * s));
    int32_t by    = Ms_RoundF((dX * s) + (dY * c));

    if ((bx == 0) && (by == 0))
    {
        return 0U;
    }

    Ms_IssueMove(bx, by);

    Debug_Log("MS", "pt%u move tgt=%d,%d fwd=%d right=%d yaw=%d imu=%u",
              (unsigned int)msIdx, (int)p->x, (int)p->y, (int)bx, (int)by,
              (int)(Chassis_GetOdomYaw() * 10.0f), (unsigned int)IMU_IsOnline());

    return 1U;
}

/**
 * @brief  本点位到位后核对落点误差（仅日志提示，不影响推进）
 * @param  p: 当前点位
 * @retval 无
 * @note   误差 = 目标点 − 实测 ODOM；超过 MS_DIST_TOL_MM 才打印，避免刷屏。
 */
static void Ms_CheckPosErr(const MsPoint_t *p)
{
    int32_t ex = p->x - (int32_t)Chassis_GetOdomX();
    int32_t ey = p->y - (int32_t)Chassis_GetOdomY();

    if ((ex > (int32_t)MS_DIST_TOL_MM) || (ex < -(int32_t)MS_DIST_TOL_MM) ||
        (ey > (int32_t)MS_DIST_TOL_MM) || (ey < -(int32_t)MS_DIST_TOL_MM))
    {
        Debug_Log("MS", "pt%u pos err x=%d y=%d",
                  (unsigned int)msIdx, (int)ex, (int)ey);
    }
}

/**
 * @brief  执行"到位后应做的工艺动作"（**步骤1 全部未实现，只打日志**）
 * @param  act: 动作类型，MsAct_t
 * @retval 无
 * @note   步骤1 只做移动，不做扫码/抓取/放置/视觉对位。本函数故意留空并打印
 *         `not implemented` —— 目的是"防止误调用"：后续步骤若忘了实现，
 *         日志里一眼能看出来，而不是静默什么都不做。
 *         后续步骤实现时：把这里换成"启动对应动作的状态机 + 返回是否完成"，
 *         并把 Mission_Update 的 HOLD 分支改成"等动作完成再置 DONE"。
 */
static void Ms_ExecAction(uint8_t act)
{
    switch (act)
    {
        case MS_ACT_NONE:
            break;                                  /* 无动作：到位即可 */

        case MS_ACT_SCAN:
            Debug_Log("MS", "pt%u act SCAN not implemented", (unsigned int)msIdx);
            break;

        case MS_ACT_PICK:
            Debug_Log("MS", "pt%u act PICK not implemented", (unsigned int)msIdx);
            break;

        case MS_ACT_PLACE:
            Debug_Log("MS", "pt%u act PLACE not implemented", (unsigned int)msIdx);
            break;

        case MS_ACT_VISION:
            Debug_Log("MS", "pt%u act VISION not implemented", (unsigned int)msIdx);
            break;

        default:
            Debug_Log("MS", "pt%u act %u not implemented",
                      (unsigned int)msIdx, (unsigned int)act);
            break;
    }
}

/**
 * @brief  复位"当前点位"的阶段与计时（启动本点 / AUTO 自动进下一点都用它）
 * @param  无
 * @retval 无
 * @note   阶段回到 TURN、清各等待标志、重开"点位超时"与"阶段回传超时"计时。
 *         注意**不**动 msIdx、不动四轮使能、不读位置（那两件事只在启动整轮时做一次）。
 */
static void Ms_ResetPoint(void)
{
    msPhase          = MS_PHASE_TURN;
    msPhaseSent      = 0U;
    msFallbackLogged = 0U;
    msBaseValid      = 0U;
    msHoldActive     = 0U;
    msSettleActive   = 0U;
    msYawRound       = 0U;
    msStartTick      = HAL_GetTick();
    msPhaseTick      = msStartTick;
}

/**
 * @brief  本点位执行完成：STEP 停下等按键 / AUTO 自动进下一个点位
 * @param  无
 * @retval 无
 * @note   **两种模式唯一的分叉点**：
 *           STEP -> 状态置 DONE，等人工按 KEY5 再执行（或 KEY1/KEY2 换点）；
 *           AUTO -> 还有下一个点位就 msIdx++ 并 Ms_ResetPoint() 接着跑；
 *                   已是最后一个则置 ALLDONE（全部跑完，停在终点）。
 *         位置模式电机到点自停，这里不需要急停；保持四轮使能（抱轴）。
 */
static void Ms_PointDone(void)
{
    Debug_Log("MS", "pt%u done, x=%d y=%d h=%d",
              (unsigned int)msIdx, (int)Chassis_GetOdomX(),
              (int)Chassis_GetOdomY(), (int)(Chassis_GetOdomYaw() * 10.0f));

    if (msMode == MS_MODE_AUTO)
    {
        if ((uint8_t)(msIdx + 1U) < MS_POINT_COUNT)
        {
            msIdx++;
            Ms_ResetPoint();
            Debug_Log("MS", "auto next pt%u %s (%u/%u)",
                      (unsigned int)msIdx, msTable[msIdx].name,
                      (unsigned int)(msIdx + 1U), (unsigned int)MS_POINT_COUNT);
            Display_MissionRefresh();
            return;
        }

        msState = MS_STATE_ALLDONE;
        msPhase = MS_PHASE_WAIT;
        Debug_Log("MS", "auto all done, x=%d y=%d",
                  (int)Chassis_GetOdomX(), (int)Chassis_GetOdomY());
        Display_MissionRefresh();
        return;
    }

    /* STEP：停下等人工继续 */
    msState = MS_STATE_DONE;
    msPhase = MS_PHASE_WAIT;
    Display_MissionRefresh();
}

/**
 * @brief  取本点位的"到点停留时长"
 * @param  p: 当前点位
 * @retval 单位 ms；**中转点返回 0**（只路过，不停留）
 */
static uint32_t Ms_HoldMsOf(const MsPoint_t *p)
{
    return (p->kind == MS_PT_VIA) ? 0U : (uint32_t)MS_ARRIVE_HOLD_MS;
}

/**
 * @brief  取"到点视觉请求"枚举的文字名（日志/页面用）
 * @param  req: MsReq_t 值（见 bsp_mission.h）
 * @retval 静态字符串 "NONE"/"RCP"/"RZ"/"SHOT"/"CCG3"
 */
static const char *Mission_ReqName(uint8_t req)
{
    switch (req)
    {
        case (uint8_t)MS_REQ_RCP:  return "RCP";
        case (uint8_t)MS_REQ_RZ:   return "RZ";
        case (uint8_t)MS_REQ_SHOT: return "SHOT";
        case (uint8_t)MS_REQ_CCG3: return "CCG3";
        default:                   return "NONE";
    }
}

/**
 * @brief  按点位配置发出一次 v2.0 视觉请求（协议 §3）
 * @param  req: MsReq_t 值；MS_REQ_NONE 时不发
 * @retval 无
 * @note   非阻塞：各 RPi_Req* 内部为短帧发送（带超时保护），不在本函数内等待应答。
 */
static void Ms_IssueReq(uint8_t req)
{
    switch (req)
    {
        case (uint8_t)MS_REQ_RCP:  RPi_ReqRcp();   break;   /* $RCP      */
        case (uint8_t)MS_REQ_RZ:   RPi_ReqRz();    break;   /* $RZ       */
        case (uint8_t)MS_REQ_SHOT: RPi_ReqShot();  break;   /* $SHOT     */
        case (uint8_t)MS_REQ_CCG3: RPi_ReqCcg(3U); break;   /* $CCG,3    */
        default:                                  break;   /* 不发       */
    }
}

/**
 * @brief  进入"到点停留"阶段（赛题要求的"停 2 秒"）
 * @param  无
 * @retval 无
 * @note   与"四轮同拍停 + 静置"（RouteCfg_t.holdMs）是两件事：后者是到位收尾，
 *         本阶段是到点后的**工艺停留**，两者先后发生。
 *         **中转点**（MS_PT_VIA）停留时长为 0，下一拍就通过本阶段继续走。
 */
static void Ms_StartHold(void)
{
    const MsPoint_t *p = &msTable[msIdx];

    msPhase     = MS_PHASE_HOLD;
    msPhaseSent = 0U;
    msArriveTick = HAL_GetTick();

    /* ★ 到点**停稳**后按点位配置发一次 v2.0 视觉请求（协议 §3：请求由"车停稳在作业点"驱动）。
       本函数只在"运动结束、四轮已停住"之后被调用，是"停稳"这一时刻的唯一入口。
       请求类型由点位表第 8 个字段配置（见 bsp_mission_cfg.h），多数点位是 MS_REQ_NONE。 */
    if (p->vcmd != (uint8_t)MS_REQ_NONE)
    {
        Debug_Log("MS", "pt%u req %s", (unsigned int)msIdx, Mission_ReqName(p->vcmd));
        Ms_IssueReq(p->vcmd);
    }

    Debug_Log("MS", "pt%u %s arrived, hold %u ms (odom x=%d y=%d h=%d)",
              (unsigned int)msIdx, p->name, (unsigned int)Ms_HoldMsOf(p),
              (int)Chassis_GetOdomX(), (int)Chassis_GetOdomY(),
              (int)(Chassis_GetOdomYaw() * 10.0f));

    Display_MissionRefresh();
}

/* ===================== 公开接口：状态机 / 控制 ===================== */

/**
 * @brief  赛题模块初始化（只复位状态机，不发电机命令）
 * @param  无
 * @retval 无
 */
void Mission_Init(void)
{
    msActive         = 0U;
    msState          = MS_STATE_IDLE;
    msPhase          = MS_PHASE_WAIT;
    msIdx            = 0U;
    msPhaseSent      = 0U;
    msFallbackLogged = 0U;
    msBaseValid      = 0U;
    msHoldActive     = 0U;
    msSettleActive   = 0U;
    msYawRound       = 0U;
    msTurnNeed       = 0.0f;
}

/**
 * @brief  进入赛题页（主菜单 MISSION 项调用）
 * @param  无
 * @retval 无
 */
void Mission_Enter(void)
{
    uint8_t addr;

    /* 若上次残留运行态，先停掉，保证页面显示与电机状态一致 */
    if ((msState == MS_STATE_RUN) || (msPhase == MS_PHASE_HOLD))
    {
        Mission_Abort();
    }

    /* 复位状态机，但**保留 msIdx**（反复调同一个点位时不用重新选） */
    msState          = MS_STATE_IDLE;
    msPhase          = MS_PHASE_WAIT;
    msPhaseSent      = 0U;
    msFallbackLogged = 0U;
    msBaseValid      = 0U;
    msHoldActive     = 0U;
    msSettleActive   = 0U;
    msYawRound       = 0U;

    /* 原点 = 出发点：把**当前位置/朝向**当作出发点（与"原点 = 出发点"的约定一致），
       省掉一次忘按清零；车已在别处、想换原点时进页后按 KEY5 重新清零即可。
       失能四轮，防上电/进页车轮抱轴发热（使能只在 Mission_StartPoint）。 */
    Chassis_OdomReset();
    for (addr = MOTOR_ADDR_MIN; addr <= MOTOR_ADDR_MAX; addr++)
    {
        Motor_Disable(addr);
    }

    msActive = 1U;
    Display_SetPage(PAGE_MISSION);

    Debug_Log("MS", "enter, pts=%u idx=%u, odom zeroed",
              (unsigned int)MS_POINT_COUNT, (unsigned int)msIdx);
}

/**
 * @brief  退出赛题页（KEY4 调用）
 * @param  无
 * @retval 无
 */
void Mission_Exit(void)
{
    Mission_Abort();
    msActive = 0U;
    Debug_Log("MS", "exit");
}

/**
 * @brief  启动当前点位（STEP 执行本步 / AUTO 从本步连续跑到底）
 * @param  无
 * @retval 无
 */
void Mission_StartPoint(void)
{
    const MsPoint_t *p;
    uint8_t addr;

    if ((msState == MS_STATE_RUN) || (msPhase == MS_PHASE_HOLD))
    {
        return;                     /* 执行中重复调用：忽略 */
    }

    if (MS_POINT_COUNT == 0U)
    {
        Debug_Log("MS", "no point defined");
        return;
    }

    if (msIdx >= MS_POINT_COUNT)
    {
        msIdx = 0U;
    }

    p = &msTable[msIdx];

    /* 位置模式的一切（里程计坐标 + 目标增量）都依赖位置数据：
       读不到四轮位置就拒绝启动，否则会朝错误的坐标走（与 P4/P7 一致） */
    if (Ms_ReadPos(msSegBase) == 0U)
    {
        Debug_Log("MS", "no pos data, start rejected");
        return;
    }

    /* 使能四轮（位置模式必须先使能，否则电机不收位置命令） */
    for (addr = MOTOR_ADDR_MIN; addr <= MOTOR_ADDR_MAX; addr++)
    {
        Motor_Enable(addr, 1U);
    }

    msState = MS_STATE_RUN;
    Ms_ResetPoint();

    Debug_Log("MS", "start %s pt%u %s tgt=%d,%d h=%d %s act=%s",
              Mission_GetModeText(), (unsigned int)msIdx, p->name,
              (int)p->x, (int)p->y, (int)p->yaw,
              (p->kind == MS_PT_VIA) ? "VIA" : "WORK",
              Mission_GetActName(msIdx));

    Display_MissionRefresh();
}

/**
 * @brief  中止当前点位执行（四轮急停 + 失能 + 状态置 ABORT）
 * @param  无
 * @retval 无
 */
void Mission_Abort(void)
{
    uint8_t addr;

    if (msState == MS_STATE_RUN)
    {
        Mecanum_Stop();
        msState = MS_STATE_ABORT;
        Debug_Log("MS", "abort pt=%u phase=%u", (unsigned int)msIdx, (unsigned int)msPhase);
    }

    msPhase         = MS_PHASE_WAIT;
    msPhaseSent     = 0U;
    msHoldActive    = 0U;
    msSettleActive  = 0U;
    msFallbackLogged = 0U;

    /* 中止/退出时**解冻平移坐标**：旋转中途被中止的话，冻结状态会被一直带下去，
       之后就成了"车在走、坐标不动"。 */
    Chassis_OdomHoldXY(0U);

    /* 失能四轮：避免退出页面后电机长时间抱轴发热 */
    for (addr = MOTOR_ADDR_MIN; addr <= MOTOR_ADDR_MAX; addr++)
    {
        Motor_Disable(addr);
    }

    Display_MissionRefresh();
}

/**
 * @brief  里程计清零：原点 = 当前位置，朝向基准 = 当前 yaw
 * @param  无
 * @retval 无
 */
void Mission_OdomZero(void)
{
    if (msState == MS_STATE_RUN)
    {
        Debug_Log("MS", "odom zero ignored (running)");
        return;
    }

    Chassis_OdomReset();
    Debug_Log("MS", "odom zeroed (origin = now)");
    Display_MissionRefresh();
}

/**
 * @brief  赛题状态机推进（主循环调用，非阻塞）
 * @param  无
 * @retval 无
 * @note   每拍只推进"一个动作"：要么下发当前阶段的命令，要么判一次是否到位，
 *         保证任何一次调用都很快返回（OLED 刷新/按键响应不受影响）。
 */
void Mission_Update(void)
{
    const MsPoint_t *p;

    if (msState != MS_STATE_RUN)
    {
        return;
    }

    /* 执行过程中刷新页面：只置脏标志，实际重绘由 OLED 侧按 REFRESH_PERIOD_MS
       节流，不会高频全量重绘（HOLD 阶段有 0.1s 倒计时，需要持续刷新） */
    Display_MissionRefresh();

    /* 本点位总超时兜底（旋转 + 平移 + 到点停留一起计时） */
    if ((HAL_GetTick() - msStartTick) >= MS_POINT_TIMEOUT_MS)
    {
        Debug_Log("MS", "pt%u timeout", (unsigned int)msIdx);
        Mission_Abort();
        return;
    }

    p = &msTable[msIdx];

    switch (msPhase)
    {
        case MS_PHASE_TURN:
            if (msPhaseSent == 0U)
            {
                if (Ms_IssueRot((float)p->yaw) != 0U)
                {
                    msPhaseSent      = 1U;
                    msPhaseTick      = HAL_GetTick();
                    msFallbackLogged = 0U;
                }
                else
                {
                    msPhase     = MS_PHASE_MOVE;   /* 已在目标朝向：直接进平移 */
                    msPhaseSent = 0U;
                }
                break;
            }

            if (Ms_PhaseArrived() != 0U)
            {
                /* 到位 -> 四轮同拍瞬停 + 静置，等残余运动结束再往下走 */
                if (Ms_StopAndSettle() == 0U)
                {
                    break;                          /* 还在静置，本拍不推进 */
                }

                /* 转角闭环：按 HWT101CT 回传的 θ 校到目标朝向。
                   返回 0 = 刚下发补转，要重新"等到位 + 停稳"再复测，
                   所以要把"等到位"的计时一并重开。 */
                if (Ms_YawLoop() == 0U)
                {
                    msPhaseTick      = HAL_GetTick();
                    msFallbackLogged = 0U;
                    Display_MissionRefresh();
                    break;
                }

                /* 进入"朝向稳定等待"：等 IMU 把补转后的新朝向送上来，
                   再规划平移方向（否则会按旧朝向算，出现横向平移）。
                   若朝向其实已经到位，Ms_YawSettleWait 会立刻放行、不额外等待。 */
                msSettleActive = 1U;
                msSettleYaw    = Chassis_GetOdomYaw();
                msSettleTick   = HAL_GetTick();
                msSettleStart  = HAL_GetTick();

                /* 顺势开始"静止零漂"测量：转弯到位后车静止的这段时间里，
                   测 yaw 自身的漂移速率（见 ROUTE_DRIFT_CAL_MS） */
                Chassis_YawDriftBegin();

                msPhase     = MS_PHASE_MOVE;
                msPhaseSent = 0U;
                msPhaseTick = HAL_GetTick();
            }
            break;

        case MS_PHASE_MOVE:
            if (msPhaseSent == 0U)
            {
                /* 若前面刚做过旋转，先等朝向稳定再规划方向 */
                if (Ms_YawSettleWait() != 0U)
                {
                    break;
                }

                if (Ms_IssueMoveAbs(p) == 0U)
                {
                    /* 已在目标点：不下发、不等待，直接进到点停留 */
                    Debug_Log("MS", "pt%u already at target", (unsigned int)msIdx);
                    Ms_StartHold();
                    break;
                }

                msPhaseSent      = 1U;
                msPhaseTick      = HAL_GetTick();
                msFallbackLogged = 0U;
                Display_MissionRefresh();
                break;
            }

            if (Ms_PhaseArrived() != 0U)
            {
                /* 到位 -> 四轮同拍瞬停 + 静置（治减速段末端横移） */
                if (Ms_StopAndSettle() == 0U)
                {
                    break;
                }

                /* 落点误差提示必须在"停稳"之后读，否则读到的是还在收尾的坐标 */
                Ms_CheckPosErr(p);
                Ms_StartHold();
            }
            break;

        case MS_PHASE_HOLD:
            /* 作业点：停 MS_ARRIVE_HOLD_MS(默认 2s)；中转点：停留 0ms，本拍直接通过 */
            if ((HAL_GetTick() - msArriveTick) < Ms_HoldMsOf(p))
            {
                break;
            }

            /* 停留结束 -> 执行该点位应做的工艺动作（步骤1 全部未实现，只打日志）。
               中转点（MS_PT_VIA）**不执行动作**，只把 act 字段忽略掉。
               后续步骤实现工艺动作时，这里要改成"启动动作 + 等动作完成再置 DONE"。 */
            if (p->kind == MS_PT_WORK)
            {
                Ms_ExecAction(p->act);

                /* 【v2.0 变更】原 v1.0 在取料动作（MS_ACT_PICK）完成后补发一条
                   $CMD,COLOR_STOP，让视觉端停发 $COLOR + $TURNTABLE。
                   v2.0 取消了推送式 $CMD：数据帧本身按 RPI_DATA_EXPIRE_MS(500ms)
                   超时自动判无效，无需"停止"指令；打靶阶段也由打靶作业点自行发
                   $SHOT（点位表 MS_REQ_SHOT / 后续作业点实现）。
                   故此处**不再发送任何帧**（阶段结束只由流程推进体现）。 */
            }

            /* 本点位完成：STEP 停下等按键 / AUTO 自动进下一个（见 Ms_PointDone） */
            Ms_PointDone();
            break;

        default:
            break;
    }
}

/**
 * @brief  赛题页按键处理（短按）
 * @param  key: 按键 ID（KEY1~KEY5）
 * @retval 无
 * @note   布局按《赛题开发指南》3.4：
 *           KEY1/KEY2 = 上/下一个点位（只移光标不执行；执行中忽略）
 *           KEY3      = 切换模式 STEP <-> AUTO（执行中忽略）
 *           KEY4      = 中止并退出回主菜单
 *           KEY5      = 执行当前点位 / 一键启动；执行中 = 中止
 *         里程计清零在长按 KEY5（Mission_HandleLongKey）。
 */
void Mission_HandleKey(KeyID_t key)
{
    if (key == KEY_NONE)
    {
        return;
    }

    /* 运行中只允许"中止"和"退出"，其余按键一律忽略，防状态错乱 */
    if (msState == MS_STATE_RUN)
    {
        if (key == KEY5)
        {
            Mission_Abort();
        }
        else if (key == KEY4)
        {
            Mission_Exit();
            Display_MenuEnter();
        }
        else
        {
            /* KEY1/KEY2/KEY3 运行中无效 */
        }
        return;
    }

    switch (key)
    {
        case KEY1:   /* 上一个点位（到头循环到最后一个） */
        case KEY2:   /* 下一个点位（到底循环到第一个） */
            if (key == KEY1)
            {
                msIdx = (msIdx == 0U) ? (uint8_t)(MS_POINT_COUNT - 1U)
                                      : (uint8_t)(msIdx - 1U);
            }
            else
            {
                msIdx = (uint8_t)((msIdx + 1U) % MS_POINT_COUNT);
            }

            msState = MS_STATE_IDLE;            /* 换点位后回到空闲，等 KEY5 执行 */
            msPhase = MS_PHASE_WAIT;
            Debug_Log("MS", "select pt%u %s (%s)",
                      (unsigned int)msIdx, msTable[msIdx].name,
                      (msTable[msIdx].kind == MS_PT_VIA) ? "VIA" : "WORK");
            Display_MissionRefresh();
            break;

        case KEY3:   /* 切换模式 STEP <-> AUTO */
            msMode = (msMode == MS_MODE_STEP) ? MS_MODE_AUTO : MS_MODE_STEP;

            /* 上一次跑完的"全部完成"状态在换模式后没有意义，退回空闲 */
            if (msState == MS_STATE_ALLDONE)
            {
                msState = MS_STATE_IDLE;
                msIdx   = 0U;
            }

            Debug_Log("MS", "mode %s", Mission_GetModeText());
            Display_MissionRefresh();
            break;

        case KEY4:   /* 中止 + 退出回主菜单（KEY4 统一退出规范） */
            Mission_Exit();
            Display_MenuEnter();
            break;

        case KEY5:   /* STEP：执行当前点位；AUTO：一键启动（从当前点位连续跑到底） */
            if (msState == MS_STATE_ALLDONE)
            {
                /* AUTO 跑完后再按 = 从头再来一遍 */
                msIdx = 0U;
            }
            Mission_StartPoint();
            break;

        default:
            break;
    }
}

/**
 * @brief  赛题页**长按**按键处理
 * @param  key: 按键 ID（KEY1~KEY5）
 * @retval 无
 * @note   只用到一条：**长按 KEY5 = 里程计清零**（执行中忽略）。
 *         短按 KEY5 被"执行/一键启动"占了，标定用的清零就放到长按上，
 *         避免和常用键抢短按。
 */
void Mission_HandleLongKey(KeyID_t key)
{
    if (key == KEY5)
    {
        Mission_OdomZero();
        Debug_Log("MS", "long KEY5 -> odom zero");
    }
}

/**
 * @brief  查询赛题页是否活动
 * @param  无
 * @retval 1=本页活动，0=非本页
 */
uint8_t Mission_IsActive(void)
{
    return msActive;
}

/* ===================== 公开接口：页面显示 ===================== */

/**
 * @brief  取点位总数
 * @param  无
 * @retval 点位个数（由 MS_POINT_LIST 自动计算）
 */
uint8_t Mission_GetPointCount(void)
{
    return MS_POINT_COUNT;
}

/**
 * @brief  取当前选中的点位号（0 起）
 * @param  无
 * @retval 当前点位下标
 */
uint8_t Mission_GetPointIdx(void)
{
    return msIdx;
}

/**
 * @brief  取点位显示名
 * @param  idx: 点位下标，越界返回 "??"
 * @retval 静态字符串指针
 */
const char *Mission_GetPointName(uint8_t idx)
{
    if (idx >= MS_POINT_COUNT)
    {
        return "??";
    }
    return msTable[idx].name;
}

/**
 * @brief  取点位的动作名
 * @param  idx: 点位下标，越界返回 "?"
 * @retval "NONE" / "SCAN" / "PICK" / "PLACE" / "VIS" / "?"
 */
const char *Mission_GetActName(uint8_t idx)
{
    if (idx >= MS_POINT_COUNT)
    {
        return "?";
    }

    switch (msTable[idx].act)
    {
        case MS_ACT_NONE:   return "NONE";
        case MS_ACT_SCAN:   return "SCAN";
        case MS_ACT_PICK:   return "PICK";
        case MS_ACT_PLACE:  return "PLACE";
        case MS_ACT_VISION: return "VIS";
        default:            return "?";
    }
}

/**
 * @brief  取当前运行模式文字
 * @param  无
 * @retval "STEP" / "AUTO"
 */
const char *Mission_GetModeText(void)
{
    return (msMode == MS_MODE_AUTO) ? "AUTO" : "STEP";
}

/**
 * @brief  取页面第 1 行文本：模式 + 点位序号/总数 + 当前点位名
 * @param  无
 * @retval 形如 "MS STEP 3/7 ROU" / "MS AUTO 7/7 BACK"
 * @note   用 "MS" 而不是 "MSN"：最宽的情况 "MS AUTO 7/7 BACK" 正好 16 字符，
 *         8x16 字体一行只有 16 个字符位，多一个字母就会被截断。
 */
const char *Mission_GetTitleText(void)
{
    static char buf[20];

    snprintf(buf, sizeof(buf), "MS %s %u/%u %s",
             Mission_GetModeText(), (unsigned int)(msIdx + 1U),
             (unsigned int)MS_POINT_COUNT, msTable[msIdx].name);

    return buf;
}

/**
 * @brief  取页面第 2 行文本：**目标**坐标与朝向
 * @param  无
 * @retval 形如 "T1200,1600h90"
 */
const char *Mission_GetTargetText(void)
{
    static char buf[20];

    snprintf(buf, sizeof(buf), "T%d,%dh%d",
             (int)msTable[msIdx].x, (int)msTable[msIdx].y, (int)msTable[msIdx].yaw);

    return buf;
}

/**
 * @brief  取页面第 3 行文本：**实测**坐标与朝向（实测替换点位用）
 * @param  无
 * @retval 形如 "A800,0h0"
 */
const char *Mission_GetOdomText(void)
{
    static char buf[24];

    snprintf(buf, sizeof(buf), "A%d,%dh%d",
             (int)Chassis_GetOdomX(), (int)Chassis_GetOdomY(),
             (int)Chassis_GetOdomYaw());

    return buf;
}

/**
 * @brief  取页面第 4 行文本：阶段 / 到点停留倒计时 / 按键提示
 * @param  无
 * @retval 空闲 "IDLE K5=GO"（AUTO：'READY K5=RUN'）；执行中 "RUN TURN"/"RUN MOVE"；
 *         停留 "HOLD 1.2s"；STEP 完成 "DONE K5=GO"；AUTO 跑完 "ALL DONE"；中止 "ABORT"
 */
const char *Mission_GetStateText(void)
{
    static char buf[20];

    switch (msState)
    {
        case MS_STATE_RUN:
            if (msPhase == MS_PHASE_HOLD)
            {
                /* 中转点停留 0ms，HOLD 阶段几乎不可见；作业点显示 2s 倒计时 */
                uint32_t holdMs = Ms_HoldMsOf(&msTable[msIdx]);
                uint32_t el     = HAL_GetTick() - msArriveTick;
                uint32_t left   = (el < holdMs) ? (holdMs - el) : 0U;

                /* 剩余时间按 0.1s 分辨率显示（禁 %f，用整数拆位） */
                snprintf(buf, sizeof(buf), "HOLD %u.%us",
                         (unsigned int)(left / 1000U), (unsigned int)((left % 1000U) / 100U));
            }
            else if (msPhase == MS_PHASE_TURN)
            {
                snprintf(buf, sizeof(buf), "RUN TURN");
            }
            else
            {
                snprintf(buf, sizeof(buf), "RUN MOVE");
            }
            break;

        case MS_STATE_DONE:
            snprintf(buf, sizeof(buf), "DONE K5=GO");
            break;

        case MS_STATE_ALLDONE:
            snprintf(buf, sizeof(buf), "ALL DONE");
            break;

        case MS_STATE_ABORT:
            snprintf(buf, sizeof(buf), "ABORT");
            break;

        default:
            /* 空闲：两种模式的"下一步动作"不同，提示词也不同 */
            snprintf(buf, sizeof(buf), (msMode == MS_MODE_AUTO) ? "READY K5=RUN"
                                                                 : "IDLE K5=GO");
            break;
    }

    return buf;
}

/* ===================== 公开接口：单段导航（bsp_task 复用） ===================== */

/* 【为什么单独一套小状态】
 * 本文件原有的点位流程（Mission_Update）是"按点位表 + 阶段"驱动的；bsp_task 的流程
 * 是"按步骤表"驱动的，两者调度源不同，不能共用 msPhase。
 * 但**被判定的量是同一批全局量**（msSegBase/msSegClk/msPhaseTick/msYawTarget/msSettle*…），
 * 所以下面用一组很小的状态位（navActive/navSent/navPhase）来区分
 * "首次进入本段（要下发命令）"与"推进中（只判到位）"，其余全部交给已验证的静态内核。
 * 前提：bsp_task 接管后 Mission_Update() 不再被调用，这些全局量只被本组接口使用。 */

/* 单段导航所处阶段 */
typedef enum {
    MS_NAV_TURN = 0,   /* 转向段：下发旋转 + 等到位 + 同拍停 + 转角闭环 */
    MS_NAV_MOVE,       /* 平移段：下发平移 + 等到位 + 同拍停（粗定位） */
    MS_NAV_SETTLE      /* 收尾段：转向后等 IMU 送上补转后的新朝向（只 NavRotate 用） */
} MsNavPhase_t;

/* MISSION_NAV_VEL=1 时这两个接口走速度模式，这一组位置模式的状态就没人用了，
   一起关掉，避免 #177-D "declared but never referenced" 警告 */
#if (MISSION_NAV_VEL == 0)
static uint8_t      navActive = 0U;        /* 0=本段未启动/已完成，1=进行中（幂等标志） */
static uint8_t      navSent   = 0U;        /* 本段命令是否已下发：0=未发（首次进入），1=已发 */
#if (MISSION_ROT_VEL == 0)      /* 转向也走速度模式时，位置模式的阶段变量没人用 */
static MsNavPhase_t navPhase  = MS_NAV_TURN;  /* 本段所处阶段 */
#endif
static MsPoint_t    navPt;                 /* 单段平移的临时目标点（供 Ms_IssueMoveAbs/CheckPosErr 用） */
#endif

/* ===================== ★速度模式导航（MISSION_NAV_VEL / MISSION_VEL_API） =====================
   GO  : 0xF6 速度闭环 + IMU 航向保持 + 里程计距离判到点
   ROT : 原地差速转 + IMU 航向闭环（用连续航向，180° 也不会反向）
   这套控制律与 TEST 页 VELTEST 完全一致，参数在 bsp_mission.h 的 MNV_* 。
   两个开关：MISSION_NAV_VEL=赛题点位表是否用它；MISSION_VEL_API=内核是否编译（VELROUTE 页用）。 */
#if ((MISSION_NAV_VEL != 0) || (MISSION_VEL_API != 0))

#define MNV_RPM_PER_MMPS   (60.0f / MEC_WHEEL_CIRC_MM)
#define MNV_RPM_PER_DPS    ((MEC_PI / 180.0f) * MEC_ROT_RADIUS_MM * 60.0f / MEC_WHEEL_CIRC_MM)

static uint8_t  mnvOn       = 0U;     /* 本段是否已启动 */
static uint8_t  mnvKind     = 0U;     /* 0=GO 1=ROT */
static uint8_t  mnvStopping = 0U;     /* 已到点：正在补发 0 速度 */
static float    mnvLegF     = 0.0f;   /* 本段车身系前后分量 mm（前正/后负）—— 日志用 */
static float    mnvLegL     = 0.0f;   /* 本段车身系左右分量 mm（右正/左负）—— 日志用 */
static float    mnvSegPsi   = 0.0f;   /* ★本段车身系行驶角 rad（atan2(l,f)）：斜向直连 + 弧线转向用 */
static float    mnvLegDist  = 0.0f;   /* 本段直线长度 mm */
static float    mnvLegTravel= 0.0f;   /* 本段已走 mm（速度×时间开环积分） */
static float    mnvOdomYaw0 = 0.0f;   /* ★段首里程计朝向 deg：算"车头已经把行驶方向带偏多少" */
static float    mnvYawC0    = 0.0f;   /* ★段首连续航向 deg：弧线转向的航向参考起点 */
static float    mnvTurnTot  = 0.0f;   /* ★本段要转的总角度 deg（0=不转，走直行航向保持） */
static float    mnvSpeed    = 0.0f;
static float    mnvSegVel   = 0.0f;   /* ★本段巡航速度上限 mm/s（0=用运行期 mnvP_Vel） */
static uint8_t  mnvSegChain = 0U;     /* ★1=段末不停车（爬行速度直接交给下一段，拐弯/直行衔接用） */
static float    mnvYawRef   = 0.0f;   /* GO 期间要守住的航向 */
static float    mnvRotDeg   = 0.0f;   /* 本次转向的目标朝向（日志用） */
static float    mnvYawCont  = 0.0f;   /* 连续航向（跨 ±180 不跳变）—— 转向闭环用它 */
static float    mnvYawTgtC  = 0.0f;   /* 转向目标（连续航向坐标系） */
static float    mnvYawRaw   = 0.0f;
static float    mnvRate     = 0.0f;
static float    mnvWz       = 0.0f;
static uint32_t mnvTick     = 0U;
static uint32_t mnvCtrl     = 0U;
static uint32_t mnvOkTick   = 0U;
static uint32_t mnvStopTick = 0U;
static uint32_t mnvRotTick  = 0U;     /* 本段转向起点（统计"转得多快"） */
static uint8_t  mnvRedo     = 0U;     /* 本段转向已补转次数（最多 2） */
#if (MNV_ROT_PRE_MS > 0U)
static uint32_t mnvPreTick  = 0U;     /* 转向预置起点 */
#endif

/* ★运行期参数（串口 VRCFG 在线调；默认值 = bsp_mission.h 的 MNV_*，掉电回默认）。
   注意：这些是"跑的时候要改"的量，全部集中在这里，Mnv_Go/Mnv_Rot 只读它们。 */
static float    mnvP_Vel    = MNV_VEL_MMPS;
static float    mnvP_Acc    = MNV_ACC_MMPS2;
static float    mnvP_FAcc   = MNV_FINAL_ACC_MMPS2;
static float    mnvP_Creep  = MNV_CREEP_MMPS;
static float    mnvP_Tol    = MNV_STOP_TOL_MM;
static float    mnvP_Kp     = MNV_YAW_KP;
static float    mnvP_Kd     = MNV_YAW_KD;
static float    mnvP_YMax   = MNV_YAW_MAX_DPS;
static float    mnvP_TMax   = MNV_TURN_MAX_DPS;
static float    mnvP_TKp    = MNV_TURN_KP;
static float    mnvP_TKd    = MNV_TURN_KD;
static float    mnvP_TAcc   = MNV_TURN_ACC_DPS2;

/* 遥测快照（串口 VRTELE 用） */
static int32_t  mnvTeleDist = 0;      /* 剩余距离 mm（ROT 段为角度误差 0.1°） */
static int32_t  mnvTeleErr  = 0;      /* 航向误差 0.1° */
static int32_t  mnvTeleVel  = 0;      /* mm/s */
static int32_t  mnvTeleWz   = 0;      /* 0.1 dps */
static int32_t  mnvTeleDec  = 0;      /* 本拍允许速度（制动包络）mm/s —— 诊断用 */
static int32_t  mnvTeleLat  = 0;      /* 横向偏差 mm（正=偏右）—— 看"蛇形/晃动"用 */

static float Mnv_Wrap(float a)
{
    while (a > 180.0f)  { a -= 360.0f; }
    while (a < -180.0f) { a += 360.0f; }
    return a;
}

/** 下发车体速度（vF 前、vR 右，mm/s）+ 转向角速度（wz 逆时针正，dps） */
static void Mnv_Send(float vF, float vR, float wz)
{
    Mecanum_VelBodyRpm(vF * MNV_RPM_PER_MMPS, vR * MNV_RPM_PER_MMPS,
                       -wz * MNV_RPM_PER_DPS, (uint8_t)MNV_ACC_LVL);
}

/** 每拍更新连续航向与实测角速度（低通） */
static void Mnv_YawTrack(uint32_t now, float yaw)
{
    float dts  = (float)(now - mnvCtrl) / 1000.0f;
    float dyaw = Mnv_Wrap(yaw - mnvYawRaw);

    if (dts < 0.001f) { dts = 0.001f; }

    mnvYawCont += dyaw;
    mnvYawRaw   = yaw;
    mnvRate     = (0.8f * mnvRate) + (0.2f * (dyaw / dts));
    mnvCtrl     = now;
}

#if (MNV_YAW_HOLD != 0)
/** 直行航向保持（PD + 小死区）—— 仅 MNV_YAW_HOLD=1 时编译使用 */
static float Mnv_YawHold(float yaw)
{
    float err  = Mnv_Wrap(mnvYawRef - yaw);
    float aerr = (err < 0.0f) ? -err : err;
    float ar   = (mnvRate < 0.0f) ? -mnvRate : mnvRate;
    float wz;

    if ((aerr <= MNV_YAW_DEAD_DEG) && (ar <= MNV_YAW_DEAD_DPS))
    {
        return 0.0f;
    }

    wz = (mnvP_Kp * err) - (mnvP_Kd * mnvRate);

    if (wz > mnvP_YMax)       { wz =  mnvP_YMax; }
    else if (wz < -mnvP_YMax) { wz = -mnvP_YMax; }
    return wz;
}
#endif  /* MNV_YAW_HOLD */

/** 到点停车：0xF6 无应答，必须连续补发 0 速度一段时间，否则丢一帧轮子就保持原速。
 *  返回 1 = 补发结束（由调用方决定"真的到点"还是"冲过头再贴一次"）。 */
static uint8_t Mnv_Stop(uint32_t now)
{
    if (mnvStopping == 0U)
    {
        mnvStopping = 1U;
        mnvStopTick = now;
    }

    Mnv_Send(0.0f, 0.0f, 0.0f);

    if ((now - mnvStopTick) >= (uint32_t)MNV_STOP_RESEND_MS)
    {
        mnvStopping = 0U;
        return 1U;
    }
    return 0U;
}

/**
 * @brief  速度模式：走到**绝对坐标** (x,y)，段内**一条直线**走完（斜向直连，不再拆 L 形两腿）
 * @param  x,y:      目标绝对坐标（里程计坐标系，与路段表 GO(x,y) 同口径）。
 * @param  cap:      本段巡航速度上限 mm/s（0=用运行期 mnvP_Vel）——进弯/进点降速用。
 * @param  chain:    1=段末**不停车**：跑完直接返回 1，速度从爬行段交给下一段；
 *                   0=段末 Mnv_Stop 补发 0 速度停稳（转向/机械臂作业前必须用）。
 * @param  turn_deg: 下一步 ROT 的目标朝向 deg（仅在 turn_on=1 且段足够长时生效）。
 * @param  turn_on:  1=**边走边转**：行进中把车头转到 turn_deg，到点时已对准；
 *                   0=不转，车头守住段首朝向（直行航向保持）。
 * @retval 1=已到点；0=进行中
 * @note   【斜向直连】段首**读一次**里程计算增量 (dX,dY)，用当前朝向旋到车身系得到
 *          前后分量 f（前正）、左右分量 l（右正），合成长度 + 行驶角 psi=atan2(l,f)，
 *          **只算这一次**；之后跑满 `速度×时间` 的开环积分即停，**过程不读里程计纠偏**。
 *         【边走边转】车头转 Δθ 时，世界系要保持同一条直线，车身系行驶角必须变成
 *          psi+Δθ（世界系行驶角 = 车身系行驶角 − 车头角，两者相消），否则一转头车就
 *          拐弯走了。航向参考按"已走比例"线性推进，前馈抵消稳态滞后，P/D 修扰动。
 */
static uint8_t Mnv_Go(int32_t x, int32_t y, int32_t cap, uint8_t chain,
                      int32_t turn_deg, uint8_t turn_on)
{
    uint32_t now = HAL_GetTick();
    float    dt;

    if ((mnvOn == 0U) || (mnvKind != 0U))
    {
        float theta;
        float c;
        float s;
        float dX;
        float dY;

        /* 绝对坐标 → 段首读一次里程计算增量，再旋到车身系（f=前正 l=右正） */
        theta = Chassis_GetOdomYawTrue() * MEC_RAD_PER_DEG;
        c     = cosf(theta);
        s     = sinf(theta);
        dX    = (float)x - Chassis_GetOdomX();
        dY    = (float)y - Chassis_GetOdomY();

        mnvLegF    = (dX * c) - (dY * s);
        mnvLegL    = (dX * s) + (dY * c);
        mnvSegPsi  = atan2f(mnvLegL, mnvLegF);
        mnvLegDist = sqrtf((mnvLegF * mnvLegF) + (mnvLegL * mnvLegL));

        mnvStopping  = 0U;
        mnvOn        = 1U;
        mnvKind      = 0U;
        mnvSpeed     = 0.0f;
        /* ★本段规划量：段首锁存，段中不变 */
        mnvSegVel    = (cap > 0) ? (float)cap : 0.0f;
        mnvSegChain  = (chain != 0U) ? 1U : 0U;
        mnvRate      = 0.0f;
        mnvWz        = 0.0f;
        mnvYawRef    = IMU_GetYaw();
        mnvYawRaw    = mnvYawRef;
        mnvCtrl      = now;
        mnvTick      = now;
        mnvOkTick    = 0U;
        mnvLegTravel = 0.0f;
        mnvTeleLat   = 0;

        /* ★弧线转向：段首就把"下一步 ROT 的目标朝向"变成行进中的航向目标 */
        mnvOdomYaw0 = Chassis_GetOdomYawTrue();
        mnvYawC0    = mnvYawCont;
        mnvTurnTot  = 0.0f;

        if ((turn_on != 0U) && (MNV_ARC_TURN_EN != 0) && (mnvLegDist >= MNV_ARC_MIN_MM))
        {
            float d = Mnv_Wrap((float)turn_deg - Chassis_GetOdomYaw());

            if ((d > MNV_ARC_MIN_DEG) || (d < -MNV_ARC_MIN_DEG))
            {
                mnvTurnTot  = d;
                /* ★弧线收尾必须停稳：ROT 已对准时会**直接返回 1、不补发 0 速度**，
                   若这里还 chain，上一段的爬行速度会一直留在驱动器里 */
                mnvSegChain = 0U;
            }
        }

        Chassis_OdomHoldXY(0U);             /* 平移段解冻坐标（显示/后续转向用） */
        Debug_Log("MS", "navvel GO f=%d l=%d arc=%d",
                  (int)mnvLegF, (int)mnvLegL, (int)(mnvTurnTot * 10.0f));

        if (mnvLegDist < 1.0f)
        {
            mnvOn = 0U;
            return 1U;                      /* 本来就在点上 */
        }
    }

    if ((now - mnvTick) < (uint32_t)MNV_TICK_MS)
    {
        return 0U;
    }

    dt      = (float)(now - mnvTick) / 1000.0f;
    mnvTick = now;

    {
        float yaw    = IMU_GetYaw();
        float remain = mnvLegDist - mnvLegTravel;
        float decel;
        float vcap;
        float wz     = 0.0f;

        Mnv_YawTrack(now, yaw);

        mnvTeleDist = (int32_t)remain;
        mnvTeleErr  = (int32_t)(Mnv_Wrap(mnvYawRef - yaw) * 10.0f);

        if (remain <= 0.0f)
        {
            if (mnvSegChain != 0U)
            {
                /* ★段末不停车：直接返回，下一段段首重新读里程计分解，天然吸收少量过冲 */
                mnvOn = 0U;
                Debug_Log("MS", "navvel GO chain f=%d l=%d",
                          (int)mnvLegF, (int)mnvLegL);
                return 1U;
            }

            if (Mnv_Stop(now) != 0U)
            {
                mnvOn = 0U;
                Debug_Log("MS", "navvel GO done f=%d l=%d arc=%d",
                          (int)mnvLegF, (int)mnvLegL, (int)(mnvTurnTot * 10.0f));
                return 1U;
            }
            return 0U;
        }

        /* 梯形速度：允许速度 = √(2·A·剩余)，实际速度按 A 逼近它；
           ★上限用"本段规划上限"（进弯/进点降速），0 表示用运行期巡航速度 */
        decel = sqrtf(2.0f * mnvP_Acc * remain);
        vcap  = (mnvSegVel > 0.0f) ? mnvSegVel : mnvP_Vel;
        if (vcap > mnvP_Vel) { vcap = mnvP_Vel; }
        if (decel > vcap)    { decel = vcap; }

        if (mnvSpeed > decel)
        {
            mnvSpeed -= mnvP_Acc * dt;
            if (mnvSpeed < decel) { mnvSpeed = decel; }
        }
        else
        {
            mnvSpeed += mnvP_Acc * dt;
            if (mnvSpeed > decel) { mnvSpeed = decel; }
        }

        if (mnvSpeed < mnvP_Creep) { mnvSpeed = mnvP_Creep; }

        /* ★开环积分：已走距离 += 速度 × 时间 */
        mnvLegTravel += mnvSpeed * dt;
        if (mnvLegTravel > mnvLegDist) { mnvLegTravel = mnvLegDist; }

        mnvTeleVel = (int32_t)mnvSpeed;
        mnvTeleDec = (int32_t)decel;

        if (mnvTurnTot != 0.0f)
        {
            /* ★边走边转：航向参考按"已走比例"线性推进（与开环积分严格同步）
               ref  = 起点 + 总转角 × 已走比例
               ff   = 总转角 × (速度/段长)  <- 等速段的理论角速度，抵消"追参考"的稳态滞后
               P/D 只修扰动（打滑、加减速段） */
            float p   = mnvLegTravel / mnvLegDist;
            float ref = mnvYawC0 + (mnvTurnTot * p);
            float err = Mnv_Wrap(ref - mnvYawCont);
            float ff  = mnvTurnTot * (mnvSpeed / mnvLegDist);

            wz = (MNV_ARC_KP * err) - (MNV_ARC_KD * mnvRate) + ff;

            if (wz > MNV_ARC_YMX_DPS)       { wz =  MNV_ARC_YMX_DPS; }
            else if (wz < -MNV_ARC_YMX_DPS) { wz = -MNV_ARC_YMX_DPS; }

            mnvTeleErr = (int32_t)(err * 10.0f);
        }
        else
        {
#if (MNV_YAW_HOLD != 0)
            wz = Mnv_YawHold(yaw);
#endif
        }
        mnvTeleWz = (int32_t)(wz * 10.0f);

        /* ★世界系直线 + 车头转动：车身系行驶角 = 段首行驶角 + 车头已转过的角度。
           (dX,dY) 是段首用**当时**朝向旋到车身系的，而目标点在世界系固定 ->
           世界系行驶角 = 车身系行驶角 − 车头角，要让它保持不变，
           车头转 Δθ 就必须让 φ = φ0 + Δθ。 */
        {
            float dth = Mnv_Wrap(Chassis_GetOdomYawTrue() - mnvOdomYaw0) * MEC_RAD_PER_DEG;
            float phi = mnvSegPsi + dth;

            Mnv_Send(mnvSpeed * cosf(phi), mnvSpeed * sinf(phi), wz);
        }
    }

    return 0U;
}

/**
 * @brief  速度模式：原地转到绝对朝向 deg（相对出发点，逆时针为正）
 * @retval 1=已到位；0=进行中
 */
static uint8_t Mnv_Rot(float deg)
{
    uint32_t now = HAL_GetTick();
    float    yaw;
    float    dt;
    float    err, aerr, ar, wz;

    if ((mnvOn == 0U) || (mnvKind != 1U))
    {
        float d;

        mnvStopping = 0U;
        mnvOn     = 1U;
        mnvKind   = 1U;
        mnvRate   = 0.0f;
        mnvWz     = 0.0f;
        mnvYawRaw = IMU_GetYaw();
        mnvCtrl   = now;
        mnvTick   = now;
        mnvRotTick = now;
        mnvOkTick = 0U;
        mnvRedo   = 0U;

        /* 目标 = 当前连续航向 + （目标朝向 − 里程计朝向）的**最短角差**。
           ⚠**必须用 IMU 连续航向闭环**，不能直接追 Chassis_GetOdomYaw()：
           赛题页每次停稳都会测陀螺零漂并持续补偿，里程计朝向本身在"漂"，
           拿它当参考会变成"追一个移动目标"（实测来回摆 ±20°）。 */
        mnvRotDeg = deg;
#if (MNV_ROT_PRE_MS > 0U)
        mnvPreTick = now;
#endif

        d = Mnv_Wrap(deg - Chassis_GetOdomYaw());
        if ((d < 0.5f) && (d > -0.5f))
        {
            mnvOn = 0U;
            return 1U;                      /* 已经对准 */
        }
        mnvYawTgtC = mnvYawCont + d;

        Chassis_OdomHoldXY(1U);             /* 原地转：冻结 x/y，避免转一次漂一点 */
        Debug_Log("MS", "navvel ROT %d (now h=%d d=%d)",
                  (int)deg, (int)Chassis_GetOdomYaw(), (int)d);
    }

#if (MNV_ROT_PRE_MS > 0U)
    /* ★预置段：只发 0 速度（20ms 一帧），等位置模式残留的 CAN 查询排空、
       驱动器切到速度模式。 */
    if ((now - mnvPreTick) < (uint32_t)MNV_ROT_PRE_MS)
    {
        if ((now - mnvTick) >= (uint32_t)MNV_TICK_MS)
        {
            mnvTick = now;
            Mnv_Send(0.0f, 0.0f, 0.0f);
        }
        return 0U;
    }
#endif

    if ((now - mnvTick) < (uint32_t)MNV_TICK_MS)
    {
        return 0U;
    }

    dt      = (float)(now - mnvTick) / 1000.0f;
    mnvTick = now;

    if (mnvStopping != 0U)
    {
        if (Mnv_Stop(now) != 0U)
        {
            mnvOn = 0U;
            Debug_Log("MS", "navvel ROT done tgt=%d t=%ums h=%d",
                      (int)mnvRotDeg, (unsigned int)(now - mnvRotTick),
                      (int)Chassis_GetOdomYaw());
            return 1U;
        }
        return 0U;
    }

    yaw     = IMU_GetYaw();
    Mnv_YawTrack(now, yaw);

    err  = mnvYawTgtC - mnvYawCont;
    aerr = (err < 0.0f) ? -err : err;
    ar   = (mnvRate < 0.0f) ? -mnvRate : mnvRate;

    if ((aerr <= MNV_TURN_TOL_DEG) && (ar <= MNV_TURN_ARRIVE_DPS))
    {
        if (mnvOkTick == 0U)
        {
            mnvOkTick = now;
        }
        if ((now - mnvOkTick) >= (uint32_t)MNV_TURN_HOLD_MS)
        {
            /* ★按**里程计朝向**复核一次：IMU 有零漂（实测 ~1.2°/s），
               赛题页停稳时会测出这个零漂并持续补偿里程计，
               于是"IMU 转够了"物理上却少了 2~3°（实测转到 -87°）。
               这里用里程计残差补转一次（最多 2 次，每次参考固定，不会来回摆）。 */
            float res = Mnv_Wrap(mnvRotDeg - Chassis_GetOdomYaw());

            if (((res > MNV_ROT_RECHECK_DEG) || (res < -MNV_ROT_RECHECK_DEG)) &&
                (mnvRedo < 2U))
            {
                mnvRedo++;
                mnvYawTgtC = mnvYawCont + res;
                mnvOkTick  = 0U;
                mnvRate    = 0.0f;
                mnvWz      = 0.0f;
                Debug_Log("MS", "navvel ROT redo%u res=%d",
                          (unsigned int)mnvRedo, (int)(res * 10.0f));
                return 0U;
            }

            Chassis_OdomHoldXY(0U);         /* 旋转段结束 -> 解冻，恢复平移坐标累加 */
            Debug_Log("MS", "navvel ROT arrive err=%d", (int)(err * 10.0f));
            Mnv_Stop(now);
            return 0U;
        }
    }
    else
    {
        mnvOkTick = 0U;
    }

    /* 位置 P + 角速度阻尼 D -> 参考角速度（再套制动包络）。
       ★加了**延迟补偿**：用"实测角速度×MNV_TURN_LEAD_S"把剩余角度提前扣掉，
         等于提前刹车。纯 P/包络实测 160°/s 会冲过 4~7°（延迟 100~150ms 造成），
         冲过去再回头要 300~400ms —— 那才是"不一步到位"的原因。 */
    {
        float ePred = aerr;
        float ref;
        float env;

        /* 只有"正在朝目标转"时才扣提前量（反向修正时不扣，否则会削弱回正力度） */
        if (((err > 0.0f) && (mnvRate > 0.0f)) || ((err < 0.0f) && (mnvRate < 0.0f)))
        {
            ePred = aerr - (mnvRate * MNV_TURN_LEAD_S);
            if (ePred < 0.0f) { ePred = 0.0f; }
        }

        env = sqrtf(2.0f * mnvP_TAcc * ePred);
        ref = (mnvP_TKp * ePred) - (mnvP_TKd * mnvRate);

        if (ref < 0.0f) { ref = 0.0f; }
        if (ref > env) { ref = env; }
        if (ref > mnvP_TMax) { ref = mnvP_TMax; }

        /* ★末端减速带：最后 MNV_TURN_NEAR_DEG 内角速度封顶，
           用"低速爬进"换"不冲过头"（冲过去再回头才是现场看到的震荡）。 */
        if ((aerr < MNV_TURN_NEAR_DEG) && (ref > MNV_TURN_NEAR_DPS))
        {
            ref = MNV_TURN_NEAR_DPS;
        }

        /* ★"一拍内不许越过目标"：赛题页主循环实测 80~100ms 一拍（VELROUTE 页 20~50ms），
           200°/s 时一拍就转 20°，末端减速带被整段跨过 -> 冲 14° 再回头。
           按实际 dt 算"这一拍最多能转多少"，指令再大也没用（看不见就来不及收）。
           这样循环快时自动放宽、循环慢时自动收窄，两个页面表现一致。 */
        if (dt > 0.005f)
        {
            float vReach = (aerr - (MNV_TURN_TOL_DEG * 0.5f)) / dt;

            if (vReach < MNV_TURN_MIN_DPS) { vReach = MNV_TURN_MIN_DPS; }
            if (ref > vReach) { ref = vReach; }
        }

        if ((aerr > MNV_TURN_TOL_DEG) && (ref < MNV_TURN_MIN_DPS))
        {
            ref = MNV_TURN_MIN_DPS;
        }
        if (err < 0.0f) { ref = -ref; }
        wz = ref;
    }
    if (wz > mnvP_TMax)       { wz =  mnvP_TMax; }
    else if (wz < -mnvP_TMax) { wz = -mnvP_TMax; }

    /* 斜率限制：**只限加速**，减速立即跟随。
       原来上下行都限 800dps² —— 从 200°/s 收到 30°/s 要 0.21s，这 0.21s 里车还在转 20°，
       必然冲过点（现场看到的"转过去再回来"）。减速不限斜率，由驱动器自己尽最大能力刹。 */
    {
        float step   = MNV_WZ_SLEW_DPS2 * dt;
        float aTarget;
        float aNow;

        if (step < 1.0f) { step = 1.0f; }

        aTarget = (wz < 0.0f) ? -wz : wz;
        aNow    = (mnvWz < 0.0f) ? -mnvWz : mnvWz;

        if (aTarget > aNow)
        {
            /* 幅值变大 = 加速：限斜率，防起步猛甩 */
            if (wz > 0.0f)
            {
                mnvWz += step;
                if (mnvWz > wz) { mnvWz = wz; }
            }
            else
            {
                mnvWz -= step;
                if (mnvWz < wz) { mnvWz = wz; }
            }
        }
        else
        {
            mnvWz = wz;                     /* 幅值变小 / 反向：立即跟随（刹不住就必过冲） */
        }
    }

    mnvTeleDist = (int32_t)(err * 10.0f);   /* ROT 段：距离位放角度误差 0.1° */
    mnvTeleErr  = (int32_t)(err * 10.0f);
    mnvTeleVel  = 0;
    mnvTeleWz   = (int32_t)(mnvWz * 10.0f);

    Mnv_Send(0.0f, 0.0f, mnvWz);
    return 0U;
}

/* ---- 对外：速度模式内核（VELROUTE 页调用；与 MISSION_NAV_VEL 无关） ---- */

uint8_t Mission_VelGo(int32_t x_mm, int32_t y_mm)
{
    return Mnv_Go(x_mm, y_mm, 0, 0U, 0, 0U);
}

/* 带"本段降速上限 + 段末不停车 + 行进中转向"的重载版：赛题连跑（VELROUTE AUTO）衔接用 */
uint8_t Mission_VelGoEx(int32_t x_mm, int32_t y_mm, int32_t cap_mmps, uint8_t chain,
                        int32_t turn_deg, uint8_t turn_on)
{
    return Mnv_Go(x_mm, y_mm, cap_mmps, chain, turn_deg, turn_on);
}

int32_t Mission_VelGetCruise(void)
{
    return (int32_t)mnvP_Vel;
}

uint8_t Mission_VelRot(float deg)
{
    return Mnv_Rot(deg);
}

void Mission_VelStop(void)
{
    Mecanum_VelBodyRpm(0.0f, 0.0f, 0.0f, (uint8_t)MNV_ACC_LVL);
    mnvOn        = 0U;
    mnvStopping  = 0U;
    mnvSpeed     = 0.0f;
    mnvSegVel    = 0.0f;
    mnvSegChain  = 0U;
    mnvWz        = 0.0f;
    mnvLegTravel = 0.0f;
    mnvTurnTot   = 0.0f;
    mnvTeleVel   = 0;
    mnvTeleWz    = 0;
    Chassis_OdomHoldXY(0U);             /* 别把冻结状态留给下一段 */
}

/** 取值范围（越界就夹到边上，不报错） */
static void Mnv_ParamRange(uint8_t idx, int32_t *lo, int32_t *hi)
{
    switch (idx)
    {
        case MV_VEL:   *lo = 200;  *hi = 1500; break;
        case MV_ACC:   *lo = 100;  *hi = 2000; break;
        case MV_FACC:  *lo = 100;  *hi = 2000; break;
        case MV_CREEP: *lo = 30;   *hi = 300;  break;
        case MV_TOL:   *lo = 5;    *hi = 60;   break;
        case MV_KP:    *lo = 2;    *hi = 50;   break;
        case MV_KD:    *lo = 0;    *hi = 50;   break;
        case MV_YMAX:  *lo = 5;    *hi = 120;  break;
        case MV_TMAX:  *lo = 40;   *hi = 300;  break;
        case MV_TKP:   *lo = 10;   *hi = 80;   break;
        case MV_TKD:   *lo = 0;    *hi = 50;   break;
        case MV_TACC:  *lo = 30;   *hi = 300;  break;
        default:       *lo = 0;    *hi = 0;    break;
    }
}

uint8_t Mission_VelSetParam(uint8_t idx, int32_t v)
{
    int32_t lo;
    int32_t hi;

    if (idx >= (uint8_t)MV_COUNT)
    {
        return 0U;
    }

    Mnv_ParamRange(idx, &lo, &hi);
    if (v < lo) { v = lo; }
    if (v > hi) { v = hi; }

    switch (idx)
    {
        case MV_VEL:   mnvP_Vel   = (float)v;        break;
        case MV_ACC:   mnvP_Acc   = (float)v;        break;
        case MV_FACC:  mnvP_FAcc  = (float)v;        break;
        case MV_CREEP: mnvP_Creep = (float)v;        break;
        case MV_TOL:   mnvP_Tol   = (float)v;        break;
        case MV_KP:    mnvP_Kp    = (float)v / 10.0f; break;
        case MV_KD:    mnvP_Kd    = (float)v / 10.0f; break;
        case MV_YMAX:  mnvP_YMax  = (float)v;        break;
        case MV_TMAX:  mnvP_TMax  = (float)v;        break;
        case MV_TKP:   mnvP_TKp   = (float)v / 10.0f; break;
        case MV_TKD:   mnvP_TKd   = (float)v / 10.0f; break;
        case MV_TACC:  mnvP_TAcc  = (float)v * 10.0f; break;
        default: return 0U;
    }
    return 1U;
}

void Mission_VelDump(void)
{
    Debug_Log("MS", "velcfg V=%d A=%d FA=%d C=%d T=%d K=%d D=%d Y=%d M=%d TK=%d TD=%d TA=%d",
              (int)mnvP_Vel, (int)mnvP_Acc, (int)mnvP_FAcc, (int)mnvP_Creep,
              (int)mnvP_Tol, (int)(mnvP_Kp * 10.0f), (int)(mnvP_Kd * 10.0f),
              (int)mnvP_YMax, (int)mnvP_TMax,
              (int)(mnvP_TKp * 10.0f), (int)(mnvP_TKd * 10.0f), (int)(mnvP_TAcc / 10.0f));
}

void Mission_VelGetTele(int32_t *dist, int32_t *err, int32_t *yaw,
                        int32_t *vel, int32_t *wz)
{
    if (dist != NULL) { *dist = mnvTeleDist; }
    if (err  != NULL) { *err  = mnvTeleErr;  }
    if (yaw  != NULL) { *yaw  = (int32_t)(IMU_GetYaw() * 10.0f); }
    if (vel  != NULL) { *vel  = mnvTeleVel;  }
    if (wz   != NULL) { *wz   = mnvTeleWz;   }
}

int32_t Mission_VelGetDecel(void)
{
    return mnvTeleDec;
}

int32_t Mission_VelGetLat(void)
{
    return mnvTeleLat;
}


#endif  /* (MISSION_NAV_VEL != 0) || (MISSION_VEL_API != 0) */


uint8_t Mission_NavGoTo(int32_t x_mm, int32_t y_mm)
{
#if (MISSION_NAV_VEL != 0)
    return Mnv_Go(x_mm, y_mm, 0, 0U, 0, 0U);
#else
    if (navActive == 0U)
    {
        /* 首次进入本段：准备临时目标点、清掉可能残留的"朝向等待"状态 */
        navActive      = 1U;
        navSent        = 0U;
#if (MISSION_ROT_VEL == 0)
        navPhase       = MS_NAV_MOVE;
#endif
        msSettleActive = 0U;
        msHoldActive   = 0U;        /* 清"停稳"状态，防上一段的计时残留被本段直接用掉 */

        /* 平移段必须"不冻结"坐标。这里是**防御性解冻**：上一次旋转若因超时/中止
           没走到 return 1，冻结状态会残留下来，那就会变成"车在走、坐标不动"。 */
        Chassis_OdomHoldXY(0U);

        navPt.name = "NAV";
        navPt.x    = x_mm;
        navPt.y    = y_mm;
        navPt.yaw  = 0;
        navPt.kind = (uint8_t)MS_PT_WORK;
        navPt.act  = (uint8_t)MS_ACT_NONE;
        navPt.vcmd = (uint8_t)MS_REQ_NONE;
    }

    /* ---- 首次进入本段：下发一次平移命令（之后每拍只判到位，绝不重复下发）---- */
    if (navSent == 0U)
    {
        if (Ms_IssueMoveAbs(&navPt) == 0U)
        {
            /* 已在目标点（车身系增量四舍五入为 0）：不下发、不等待，直接算到达 */
            navActive = 0U;
            return 1U;
        }

        navSent          = 1U;
        msPhaseTick      = HAL_GetTick();
        msFallbackLogged = 0U;
        return 0U;
    }

    /* ---- 推进中：等到位 -> 四轮同拍停 + 静置 -> 落点误差提示 -> 收尾 ---- */
    if (Ms_PhaseArrived() != 0U)
    {
        if (Ms_StopAndSettle() == 0U)
        {
            return 0U;              /* 还在静置，本拍不推进 */
        }

        Ms_CheckPosErr(&navPt);     /* 只在"停稳"之后读，否则读到的是还在收尾的坐标 */
        navActive = 0U;             /* 自复位：下一次调用会重新启动本段 */
        return 1U;
    }

    return 0U;
#endif  /* MISSION_NAV_VEL */
}

/**
 * @brief  单段导航：原地转到目标朝向 deg（相对出发点朝向）
 * @param  deg: 目标朝向，单位 deg，俯视逆时针为正，范围 -180~+180
 * @retval 1=已到位（含转角闭环 + 朝向稳定等待）；0=进行中
 */
uint8_t Mission_NavRotate(float deg)
{
#if ((MISSION_NAV_VEL != 0) || (MISSION_ROT_VEL != 0))
    return Mnv_Rot(deg);
#else
    if (navActive == 0U)
    {
        /* 首次进入本段：记目标朝向，清掉可能残留的"朝向等待"状态 */
        navActive      = 1U;
        navSent        = 0U;
        navPhase       = MS_NAV_TURN;
        msSettleActive = 0U;

        /* 旋转段：**冻结平移坐标** —— 转多少度 (x,y) 都不变，只有朝向 yaw 在变。
           原因见 Chassis_OdomHoldXY 的说明（原地转时四轮反向，正解出的残余位移
           会被积分进坐标，表现就是"转一次漂一点"）。 */
        Chassis_OdomHoldXY(1U);
    }

    /* ---- 收尾段：等 IMU 把补转后的新朝向送上来，再放行 ---- */
    if (navPhase == MS_NAV_SETTLE)
    {
        if (Ms_YawSettleWait() != 0U)
        {
            return 0U;              /* 朝向还没稳定，继续等 */
        }

        Chassis_OdomHoldXY(0U);     /* 旋转段结束 -> 解冻，恢复平移坐标累加 */
        navActive = 0U;
        return 1U;
    }

    /* ---- 首次进入本段：下发一次旋转命令 ---- */
    if (navSent == 0U)
    {
        if (Ms_IssueRot(deg) == 0U)
        {
            /* 已在目标朝向（差 < MS_TURN_SKIP_DEG）：不下发、不等待 */
            Chassis_OdomHoldXY(0U);     /* 没转 -> 立刻解冻，别把冻结状态留下 */
            navActive = 0U;
            return 1U;
        }

        navSent          = 1U;
        msPhaseTick      = HAL_GetTick();
        msFallbackLogged = 0U;
        return 0U;
    }

    /* ---- 推进中：等到位 -> 同拍停 + 静置 -> 转角闭环 -> 朝向稳定等待 ---- */
    if (Ms_PhaseArrived() != 0U)
    {
        if (Ms_StopAndSettle() == 0U)
        {
            return 0U;
        }

        /* 转角闭环：按 HWT101CT 回传的 θ 补到目标朝向（位置模式补转）。
           返回 0 = 刚下发补转，要重新"等到位 + 停稳"再复测，所以把计时一并重开。 */
        if (Ms_YawLoop() == 0U)
        {
            msPhaseTick      = HAL_GetTick();
            msFallbackLogged = 0U;
            return 0U;
        }

        /* 进入"朝向稳定等待"：等新朝向样本，保证后续按补转后的朝向规划方向 */
        msSettleActive = 1U;
        msSettleYaw    = Chassis_GetOdomYaw();
        msSettleTick   = HAL_GetTick();
        msSettleStart  = HAL_GetTick();
        Chassis_YawDriftBegin();    /* 顺势开始"静止零漂"测量，与 Ms_SettleRelease 配对 */

        navPhase = MS_NAV_SETTLE;
        return 0U;
    }

    return 0U;
#endif  /* MISSION_NAV_VEL */
}
