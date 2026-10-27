/**
 ******************************************************************************
 * @file    bsp_chassis.c
 * @brief   车机运动序列实现（P4：整车位置模式运动 + 到位判定 + 航向纠偏）
 * @author  nx_car
 * @date    2026-09-16
 ******************************************************************************
 * @note   非阻塞状态机，无 HAL_Delay。所有 CAN 命令经 bsp_mecanum / bsp_motor
 *         下发；位置读取只解析接收缓冲（不发查询，查询由 Motor_PeriodicQuery
 *         统一 round-robin 负责）。
 *         日志统一 [CHS] 标签，禁 %f。
 ******************************************************************************
 */
#include "bsp_chassis.h"
#include "bsp_motor.h"
#include "bsp_mecanum.h"
#include "bsp_imu.h"
#include "bsp_oled_ui.h"
#include "bsp_debug.h"
#include <math.h>

/* ===================== 内部阶段枚举 ===================== */
typedef enum {
    CHS_PHASE_IDLE = 0,   /* 无进行中的段 */
    CHS_PHASE_MOVE,       /* 主运动段 */
    CHS_PHASE_YAWFIX,     /* 航向纠偏段 */
    CHS_PHASE_ARCROT,     /* ARC 两段式回退的第二段（原地旋转） */
    CHS_PHASE_SPINROT     /* SPIN 第二段：原地自转到目标角度 */
} ChassisPhase_t;

/* ===================== 内部状态 ===================== */

static ChassisState_t chsState = CHS_STATE_IDLE;   /* 运动状态 */
static ChassisAct_t   chsAct   = CHS_ACT_FWD;      /* 当前/上次动作 */
static ChassisPhase_t chsPhase = CHS_PHASE_IDLE;   /* 当前段 */

static int32_t  chsSegBase[MOTOR_NUM];   /* 判到位基准（本段起点） */
static int32_t  chsSegClk[MOTOR_NUM];    /* 判到位目标（本段增量脉冲） */
static int32_t  chsStartBase[MOTOR_NUM]; /* 本次动作起始四轮位置（进度基准） */

static uint32_t chsSegTick = 0U;         /* 本段开始时间戳 ms */
static float    chsYaw0    = 0.0f;       /* 本次动作起始 yaw（deg） */

static float    chsProgDist = 0.0f;      /* 进度：已走平移 mm */
static float    chsProgDeg  = 0.0f;      /* 进度：已转角度 deg */

/* ---- 相对坐标（里程计）：原点 = 进页面时刻的位置，轴取当时车身方向 ---- */
static float   chsOdomX       = 0.0f;    /* 相对原点 x，mm（正=当时前方） */
static float   chsOdomY       = 0.0f;    /* 相对原点 y，mm（正=当时右侧） */
static float   chsOdomYaw0    = 0.0f;    /* 原点处的 yaw（全局朝向基准） */
static int32_t chsOdomPrevClk[MOTOR_NUM];/* 上一拍四轮脉冲（求增量用） */
static uint8_t chsOdomReady   = 0U;      /* 1=已建基准，可以开始累加 */
static uint8_t chsOdomHoldXY  = 0U;      /* 1=冻结平移坐标累加（原地旋转期间用，见 Chassis_OdomHoldXY） */

/* ---- 静止零漂补偿（每次转弯后车停稳时测 yaw 自身漂移，线性扣除） ----
 * 思路：车静止时 yaw 的缓慢变化就是陀螺零漂，测出速率后按时间线性扣除，
 *       抑制长路径上"世界坐标系缓慢旋转"造成的累计误差。 */
static float    chsYawDriftDps   = 0.0f;  /* 零偏估计，单位 °/s（平滑后） */
static float    chsYawCorrDeg    = 0.0f;  /* 已累计的补偿量（度），yaw_used = raw − 本值 */
static float    chsYawCompDeg    = 0.0f;  /* coordinate-frame compensation (deg); see bsp_chassis.h */
static uint32_t chsYawCorrTick   = 0U;    /* 补偿累计的时间戳 ms */
static float    chsYawCalRefYaw  = 0.0f;  /* 本次静止测量开始时的**原始** yaw */
static uint32_t chsYawCalRefTick = 0U;    /* 本次静止测量开始时刻 ms */
static uint8_t  chsYawCalActive  = 0U;    /* 1=正在做静止零漂测量 */

/* ---- 速度模式（单步调试）内部状态 ----
 * 与跑图速度模式同思路：位移来自动作表、速度走梯形曲线，航向由陀螺仪闭环。
 * 这里的角度都用**相对动作起点的连续角**（不折返），所以 360°/720° 也能一次转完。 */
#if (CHS_VEL_MODE != 0U)
static uint32_t chsVelTick       = 0U;   /* 20ms 节拍 */
static uint32_t chsVelPhaseStart = 0U;   /* 当前段起始时刻 */
static uint32_t chsVelCtrlTick   = 0U;   /* 上次控制时刻（D 项 dt） */
static uint32_t chsVelOkTick     = 0U;   /* 首次进入到位判据的时刻（停稳用） */
static uint32_t chsVelRotLimitMs = 0U;   /* 本段旋转的总时限 */
static float    chsVelYawPrev    = 0.0f; /* 上一拍**原始** yaw（只用于求角速度做阻尼） */
static float    chsVelYawRef     = 0.0f; /* 动作起点的**原始** yaw（直行航向基准） */
static float    chsVelRateF      = 0.0f; /* 低通后的实测角速度 deg/s */
static float    chsVelTurnTarget = 0.0f; /* 旋转段目标：**原始 yaw 的绝对值** */
static float    chsVelTurnRemain = 0.0f; /* 本动作还剩多少度没转（>180° 分段转） */
static float    chsVelDirF       = 0.0f; /* 平移方向（车体系单位向量） */
static float    chsVelDirR       = 0.0f;
static float    chsVelDist       = 0.0f; /* 本段距离 mm（含补偿） */
static float    chsVelPeak       = 0.0f; /* 速度曲线峰值 mm/s */
static float    chsVelTAcc       = 0.0f; /* 加速时间 s */
static float    chsVelTDecel     = 0.0f; /* 减速时间 s */
static float    chsVelTCruise    = 0.0f; /* 匀速时间 s */
static float    chsVelTTotal     = 0.0f; /* 总时间 s */
static float    chsVelArcDeg     = 0.0f; /* ARC：本段要转的角度 deg（0=不转） */

#define CHS_RPM_PER_MMPS  (60.0f / MEC_WHEEL_CIRC_MM)
#define CHS_RPM_PER_DPS   ((MEC_PI / 180.0f) * MEC_ROT_RADIUS_MM * 60.0f / MEC_WHEEL_CIRC_MM)
#endif

/* ===================== 内部工具 ===================== */

/**
 * @brief  读四轮当前位置
 * @param  out: 输出四轮位置（FL/FR/RL/RR），单位=**下发脉冲域**
 * @retval 1=四轮全部读到；0=有任一读不到（输出不改动）
 * @note   只解析接收缓冲，不发 CAN 帧，非阻塞。位置数据的刷新由
 *         Motor_PeriodicQuery（每 80ms 一轮）负责，所以这里不做 round-robin，
 *         每拍全读四轮即可（判到位更快）。
 *         单位换算：驱动器 0x36 上报的是编码器计数（65536/转 = 6.55 万/转），比下发脉冲
 *         （3200/转）大 MOTOR_READ_SCALE_DEN/NUM 倍（本项目实测结构比值 **20.48**），这里统一
 *         换算到**下发脉冲域**，让坐标、进度 D、脉冲判到位三者与 chsSegClk 同域，
 *         否则会出现"实际走 500mm、坐标读 5000mm"。
 *         【2026-09-30】MOTOR_READ_SCALE 由 1/20 修正为 25/512(=1/20.48)：
 *         用 1/20 时里程计多读 2.4%，跑图/赛题的点位会**少走 2.4%**。
 */
static uint8_t Chassis_ReadPos(int32_t out[MOTOR_NUM])
{
    int32_t raw[MOTOR_NUM];
    uint8_t i;

    /* 先四轮全部读出：任一轮读不到就整次作废，
       避免"读到一半返回"让调用方拿到半新半旧的基准 */
    for (i = 0; i < MOTOR_NUM; i++)
    {
        if (Motor_GetAngle_x10deg((uint8_t)(MOTOR_ADDR_MIN + i), &raw[i]) != 0)
        {
            return 0U;
        }
    }

    for (i = 0; i < MOTOR_NUM; i++)
    {
        out[i] = (raw[i] * (int32_t)MOTOR_READ_SCALE_NUM) / (int32_t)MOTOR_READ_SCALE_DEN;
    }

    return 1U;
}

/**
 * @brief  开始一个新运动段：重读位置基准并重启段计时
 * @param  无
 * @retval 1=成功；0=位置数据不可用（调用方应中止）
 */
static uint8_t Chassis_BeginSeg(void)
{
    if (Chassis_ReadPos(chsSegBase) == 0U)
    {
        return 0U;
    }
    chsSegTick = HAL_GetTick();
    return 1U;
}

/**
 * @brief  在当前位置基础上下发一段整车位移（位置模式）
 * @param  dx_mm:   前进位移 mm
 * @param  dy_mm:   右移位移 mm
 * @param  dRot_mm: 旋转等效弧长 mm
 * @retval 无
 * @note   位置模式是"相对当前位置"，所以目标 = 本段起始位置 + 本段增量脉冲。
 *         增量脉冲由 Mecanum_MoveBy 算出并回传，存进 chsSegClk 供判到位。
 *         下发前**必须清除四轮的到位回传标志**：否则上一段的到位标志会被
 *         当成"本段已到位"，出现"刚下发就判到位、实际没走"。
 */
static void Chassis_IssueMove(int32_t dx_mm, int32_t dy_mm, int32_t dRot_mm)
{
    int32_t clk[MOTOR_NUM];
    uint8_t addr;
    uint8_t i;

    /* 清到位标志（在发命令之前清，保证收到的到位帧一定属于本段） */
    for (addr = MOTOR_ADDR_MIN; addr <= MOTOR_ADDR_MAX; addr++)
    {
        Motor_ReachClear(addr);
    }

    Mecanum_MoveBy(dx_mm, dy_mm, dRot_mm, clk);

    for (i = 0; i < MOTOR_NUM; i++)
    {
        chsSegClk[i] = clk[i];
    }
}

/**
 * @brief  判断本段是否到位
 * @param  tolPulse: 到位容差，单位脉冲（四轮误差都小于它才算到位）
 * @retval 1=四轮都到位；0=未到位
 * @note   任一轮本拍读不到位置就返回 0（等下一拍再判），不用旧值蒙。
 *         本函数是"脉冲容差"判据，作为**后备**；段到位优先用电机到位回传
 *         （见 Chassis_SegArrivedByReach）。
 */
static uint8_t Chassis_SegArrived(int32_t tolPulse)
{
    int32_t cur[MOTOR_NUM];
    int32_t delta;
    int32_t err;
    uint8_t i;

    /* 走 Chassis_ReadPos 而不是直接读电机：与本段基准 chsSegBase 同处
       滤除跳变后的坐标域，避免使能瞬间的位置基准跳变造成"永远差很远" */
    if (Chassis_ReadPos(cur) == 0U)
    {
        return 0U;
    }

    for (i = 0; i < MOTOR_NUM; i++)
    {
        delta = cur[i] - chsSegBase[i];
        err   = delta - chsSegClk[i];
        if (err < 0)
        {
            err = -err;
        }
        if (err > tolPulse)
        {
            return 0U;
        }
    }
    return 1U;
}

/**
 * @brief  段到位判定（**等电机主动回传的到位帧**）
 * @param  无
 * @retval 1=四轮都已回传"到位"；0=未收齐（或未启用回传）
 * @note   到位帧：`地址 + FD + 9F + 6B`，电机位置进入"位置到达窗口"
 *         （默认 0.1~0.3°，比脉冲容差严格得多）后由电机主动推送。
 *         本段下发时已由 Chassis_IssueMove 清除过标志，所以不会误判。
 *         需 4 个电机菜单 Response 设为 Reached/Both/Other。
 */
static uint8_t Chassis_SegArrivedByReach(void)
{
#if (CHS_USE_MOTOR_REACH != 0)
    uint8_t addr;

    for (addr = MOTOR_ADDR_MIN; addr <= MOTOR_ADDR_MAX; addr++)
    {
        if (Motor_ReachOk(addr) == 0U)
        {
            return 0U;
        }
    }

    return 1U;
#else
    return 0U;
#endif
}

/**
 * @brief  是否允许退回"脉冲容差"判到位
 * @param  无
 * @retval 1=允许；0=继续等到位回传
 * @note   未启用回传时恒为 1；启用时只在等待超过 CHS_REACH_FALLBACK_MS 后为 1，
 *         并打印一次提示（每段最多一条：判到位后本段立即结束/换段）。
 */
static uint8_t Chassis_PulseFallbackAllowed(void)
{
#if (CHS_USE_MOTOR_REACH != 0)
    if ((HAL_GetTick() - chsSegTick) < CHS_REACH_FALLBACK_MS)
    {
        return 0U;
    }

    Debug_Log("CHS", "reach timeout, fallback to pulse");
    return 1U;
#else
    return 1U;
#endif
}

/**
 * @brief  取四轮"到位回传"掩码（诊断用）
 * @param  无
 * @retval bit0~bit3 分别对应 1~4 号轮，1=该轮已回传到位
 * @note   段完成日志里打印：`reach=15` 表示四轮都是靠到位回传确认的；
 *         `reach=0` 说明回传没收到（走了脉冲兜底），需检查电机 Response 菜单。
 */
static uint8_t Chassis_ReachMask(void)
{
    uint8_t mask = 0U;
    uint8_t i;

    for (i = 0; i < MOTOR_NUM; i++)
    {
        if (Motor_ReachOk((uint8_t)(MOTOR_ADDR_MIN + i)) != 0U)
        {
            mask |= (uint8_t)(1U << i);
        }
    }
    return mask;
}

/**
 * @brief  更新进度显示（已走 mm / 已转 deg）
 * @param  无
 * @retval 无
 * @note   用"当前位置 − 动作起始位置"做麦轮正解，得到相对本次动作起点的
 *         总平移位移与总转角，跨段累加（纠偏段也会计入角度）。
 */
static void Chassis_UpdateProgress(void)
{
    int32_t cur[MOTOR_NUM];
    int32_t dClk[MOTOR_NUM];
    float   dx = 0.0f;
    float   dy = 0.0f;
    float   dr = 0.0f;
    uint8_t i;

    /* 走 Chassis_ReadPos：与动作基准 chsStartBase 同处滤除跳变后的坐标域，
       否则使能瞬间的位置基准跳变会被算成"已走几千毫米" */
    if (Chassis_ReadPos(cur) == 0U)
    {
        return;   /* 本拍读不到位置：保持上次进度值 */
    }

    for (i = 0; i < MOTOR_NUM; i++)
    {
        dClk[i] = cur[i] - chsStartBase[i];
    }

    Mecanum_ForwardPulse(dClk, &dx, &dy, &dr);

    chsProgDist = sqrtf((dx * dx) + (dy * dy));
    chsProgDeg  = Mecanum_ArcMmToDeg(dr);
}

/**
 * @brief  取"已做零漂补偿"的原始偏航角
 * @param  无
 * @retval 原始 IMU 偏航角 − 已累计的漂移补偿，单位 deg
 * @note   **唯一入口**：所有用到原始 yaw 的地方都必须走这里，
 *         这样静止零漂补偿（每次转弯后测得）才能同时作用于里程计与跑图规划。
 */
static float Chassis_YawUsedDeg(void)
{
#if (CHS_YAW_DRIFT_COMP_ENABLE != 0)
    return IMU_GetYaw() - chsYawCorrDeg;
#else
    return IMU_GetYaw();
#endif
}

/**
 * @brief  归一化的航向角 θ（**俯视逆时针为正**，相对参考朝向）
 * @param  refYaw: 参考航向基准（原始 IMU 偏航角），单位 deg
 * @retval θ，单位 deg，范围 ±180
 * @note   本函数是 **θ 的唯一出口**，符号基准由 bsp_chassis.h 的
 *         `IMU_YAW_CCW_POSITIVE` 决定（该宏必须实测确认，见 P7 章 5 节）：
 *           =1：IMU_GetYaw 逆时针增大，与 θ 同向，直接用；
 *           =0：IMU_GetYaw 顺时针增大，与 θ 反向，取反后用。
 *         所有使用者（里程计正变换、跑图逆变换、页面/日志 H、航向纠偏、
 *         SPIN 漂移补偿）都只认这个 θ，不允许任何一处再单独取反。
 *         约定（俯视、原点=启动点）：+X=车头、+Y=车右；θ>0 = 车头由 +X 转向 -Y（车左）。
 */
static float Chassis_YawDelta(float refYaw)
{
    float yawNow = Chassis_YawUsedDeg();   /* 已扣除零漂补偿的当前偏航角 */

#if (IMU_YAW_CCW_POSITIVE == 0)
    float err = refYaw - yawNow;   /* IMU 为顺时针为正：取反得到逆时针为正的 θ */
#else
    float err = yawNow - refYaw;   /* IMU 本身就是逆时针为正 */
#endif

    while (err > 180.0f)
    {
        err -= 360.0f;
    }
    while (err < -180.0f)
    {
        err += 360.0f;
    }
    return err;
}

/**
 * @brief  取本次动作的航向偏差 θ（同上符号约定）
 * @param  无
 * @retval 相对动作起始朝向的 θ，单位 deg（正=俯视逆时针）
 */
static float Chassis_YawErrorDeg(void)
{
    return Chassis_YawDelta(chsYaw0);
}

/**
 * @brief  取相对"坐标原点朝向"的航向角 θ（里程计正变换 / 跑图逆变换共用）
 * @param  无
 * @retval θ，单位 deg，范围 ±180（**正 = 俯视逆时针**，即车头由 +X 转向 -Y）
 * @note   两个变换必须共用本函数返回的同一个 θ，否则会出现"里程计显示到 A 点、
 *         按目标坐标反推的 dx/dy 却是反方向"的矛盾。
 */
static float Chassis_GlobalYawDeg(void)
{
    return Chassis_YawDelta(chsOdomYaw0);
}

/* ===================== 相对坐标（里程计） ===================== */

/**
 * @brief  相对坐标清零：原点 = 当前位置，朝向基准 = 当前陀螺仪角度
 * @param  无
 * @retval 无
 * @note   进入车机测试页时自动调用（见 bsp_chassis_test.c 的 CHS_Test）。
 *         清零后立即从 (0,0)/0° 开始累加；若此刻四轮位置读不到，
 *         基准会在第一次 Chassis_OdomUpdate 成功读到时补建。
 */
void Chassis_OdomReset(void)
{
    chsOdomX      = 0.0f;
    chsOdomY      = 0.0f;
    chsOdomYaw0   = Chassis_YawUsedDeg();  /* 与 Chassis_YawDelta 同源，避免补偿量造成基准错位 */
    chsOdomReady  = 0U;
    chsOdomHoldXY = 0U;                    /* 复位时一并解冻，避免残留"冻结"状态带进下一段 */

    if (Chassis_ReadPos(chsOdomPrevClk) != 0U)
    {
        chsOdomReady = 1U;
    }

    Debug_Log("CHS", "odom reset, x=0 y=0 yaw0=%d", (int)(chsOdomYaw0 * 10.0f));
}

/**
 * @brief  ★任务点"静态置角"：IMU 置 refDeg **并同步里程计朝向基准**
 * @param  refDeg: 期望当前朝向（deg）
 * @retval 1=已置角；0=车还在转被 IMU 拒绝（本拍不改任何基准）
 * @note   【根因】`Chassis_GetOdomYaw()` 算的是 `IMU yaw − chsOdomYaw0`，
 *         而 `chsOdomYaw0` 是"里程计清零那一刻的 IMU 原始 yaw"。
 *         以前只调 `IMU_StaticZeroTo()`：IMU 变成 0 了，但基准没变，
 *         于是页面航向角变成 `0 − chsOdomYaw0`——现场就是"TRAY PICK 置 0 后
 *         航向角被置成 180°"（清零时 IMU 原始 yaw 恰好在 −180 附近）。
 *         本函数在**置角成功后**把基准一起平移，使航向角 == refDeg，
 *         后续 Rotate/GO 的坐标变换才真正建立在"新设置的角度"上。
 *         ★只动朝向基准，不动原点坐标 x/y（x/y 由轮式里程累积，与 yaw 基准无关）。
 *         ★若正在做静止零漂测量，同步重置其参考点，避免把这次跳变当成"零漂"。
 */
uint8_t Chassis_StaticZeroYaw(float refDeg)
{
    if (IMU_StaticZeroTo(refDeg) == 0U)
    {
        return 0U;                       /* 车还在转：什么都不改，等下一拍重试 */
    }

#if (IMU_YAW_CCW_POSITIVE == 0)
    chsOdomYaw0 = Chassis_YawUsedDeg() + refDeg;
#else
    chsOdomYaw0 = Chassis_YawUsedDeg() - refDeg;
#endif

    if (chsYawCalActive != 0U)
    {
        chsYawCalRefYaw  = IMU_GetYaw(); /* 参考点跟着平移，否则会把本次跳变算成零漂 */
        chsYawCalRefTick = HAL_GetTick();
    }

    Debug_Log("CHS", "static zero yaw -> %d, odom now %d (yaw0=%d)",
              (int)(refDeg * 10.0f), (int)(Chassis_GlobalYawDeg() * 10.0f),
              (int)(chsOdomYaw0 * 10.0f));
    return 1U;
}

/**
 * @brief  开始一次"静止零漂"测量（车已停稳时调用）
 * @param  无
 * @retval 无
 * @note   记录当前**原始** yaw 与时刻，随后由 Chassis_YawDriftEnd() 结束测量。
 *         典型时机：每次转弯到位后、进入朝向等待/短暂停顿的那段静止时间。
 *         原理：车静止时 yaw 的缓慢变化就是陀螺零漂，测出速率后可线性扣除。
 */
void Chassis_YawDriftBegin(void)
{
    chsYawCalRefYaw  = IMU_GetYaw();     /* 用**原始**值：测的就是 yaw 自身的漂移 */
    chsYawCalRefTick = HAL_GetTick();
    chsYawCalActive  = 1U;
}

/**
 * @brief  结束"静止零漂"测量并更新零偏估计
 * @param  无
 * @retval 无
 * @note   测量窗口需 ≥ CHS_YAW_DRIFT_CAL_MS 才有效；估计值 =
 *         (窗口内 yaw 变化量) / (窗口秒数)，经**限幅**（超 CHS_YAW_DRIFT_MAX_DPS
 *         视为未停稳/干扰，丢弃本次）与**一阶平滑**（权重 CHS_YAW_DRIFT_SMOOTH）
 *         后，由 Chassis_OdomUpdate 按时间线性扣除。
 *         只在"车确实静止"时调用，否则会把真实转动误当零偏。
 */
void Chassis_YawDriftEnd(void)
{
    uint32_t dtMs;
    float    dtS;
    float    rate;

    if (chsYawCalActive == 0U)
    {
        return;
    }
    chsYawCalActive = 0U;

    dtMs = HAL_GetTick() - chsYawCalRefTick;
    if (dtMs < (uint32_t)CHS_YAW_DRIFT_CAL_MS)
    {
        return;                     /* 静止窗口太短，本次不更新估计 */
    }

    dtS  = (float)dtMs * 0.001f;
    rate = (IMU_GetYaw() - chsYawCalRefYaw) / dtS;      /* 单位 °/s */

    if ((rate > CHS_YAW_DRIFT_MAX_DPS) || (rate < -CHS_YAW_DRIFT_MAX_DPS))
    {
        Debug_Log("CHS", "yaw drift cal reject rate=%d mdeg/s", (int)(rate * 1000.0f));
        return;
    }

    chsYawDriftDps += CHS_YAW_DRIFT_SMOOTH * (rate - chsYawDriftDps);

    Debug_Log("CHS", "yaw drift cal %u ms: rate=%d bias=%d mdeg/s",
              (unsigned int)dtMs, (int)(rate * 1000.0f), (int)(chsYawDriftDps * 1000.0f));
}

/**
 * @brief  取当前零偏估计（整数打印用，禁 %f）
 * @param  无
 * @retval 零偏估计 ×1000，单位 0.001°/s
 */
int32_t Chassis_GetYawDrift_mdps(void)
{
    return (int32_t)(chsYawDriftDps * 1000.0f);
}

/**
 * @brief  相对坐标每拍累加（主循环周期调用，与运动状态无关）
 * @param  无
 * @retval 无
 * @note   原理：四轮脉冲增量 → 麦轮正解得到**本体坐标系**位移增量 →
 *         按当前全局朝向旋转回原点坐标系后累加：
 *           x += dx_body·cos(theta) − dy_body·sin(theta)
 *           y += dx_body·sin(theta) + dy_body·cos(theta)
 *         这样"转了弯再直行"也能算对（不能只用 dx_body 直接当 x）。
 *         任一轮位置读不到就整拍跳过（基准不动），不污染累计；
 *         本拍四轮都没动则直接返回。
 *         本函数还负责推进静止零漂补偿量（见 Chassis_YawDriftBegin/End）。
 */

/**
 * @brief  原地旋转期间"冻结 / 解冻"平移坐标累加
 * @param  on: 1=冻结（本拍起不再把四轮位移积分进 x/y）；0=恢复正常累加
 * @retval 无
 * @note   用途：赛题 / 跑图的"旋转步"期间调用，保证**转多少度坐标都不动**，
 *         只有"前后行进 + 左右平移"才改变 x/y（朝向仍由 IMU 正常更新）。
 *         为什么需要：原地转时四轮反向转动，理论上车心不动，但打滑 / 轮径差
 *         会让正解出一段残余位移，积分进去就是"转一次漂一点"。
 *         ⚠ 用完必须解冻（上层在进入平移前 / 中止时 / 复位时都要发 0），
 *           否则会变成"车在走、坐标不动"。
 */
void Chassis_OdomHoldXY(uint8_t on)
{
    uint8_t v = (on != 0U) ? 1U : 0U;

    if (v == chsOdomHoldXY)
    {
        return;                 /* 状态没变：不打日志（本函数会被上层频繁调用） */
    }

    chsOdomHoldXY = v;

    Debug_Log("CHS", "odom holdXY %s", (chsOdomHoldXY != 0U) ? "ON (freeze x/y)" : "OFF");
}

void Chassis_OdomUpdate(void)
{
    int32_t cur[MOTOR_NUM];
    int32_t dClk[MOTOR_NUM];
    float   dxb  = 0.0f;
    float   dyb  = 0.0f;
    float   drot = 0.0f;
    float   th;
    float   c;
    float   s;
    uint8_t moved = 0U;
    uint8_t i;
#if (CHS_ODOM_READ_MS > 0U)
    static uint32_t odomReadTick = 0U;
#endif
#if (CHS_ODOM_LOG_MS > 0U)
    static uint32_t odomLogTick = 0U;   /* 原始数据日志的上次打印时刻 */
#endif

#if (CHS_ODOM_READ_MS > 0U)
    /* ★限速读位置：每拍 4 帧 CAN 查询会把总线挤死（速度指令被推迟 ->
       四轮不同步 -> 车头左右甩）。100Hz 足够。 */
    {
        uint32_t nowRd = HAL_GetTick();

        if ((nowRd - odomReadTick) < (uint32_t)CHS_ODOM_READ_MS)
        {
            return;
        }
        odomReadTick = nowRd;
    }
#endif

    /* 静止零漂补偿推进：按当前零偏估计线性累加补偿量（每拍一次，非阻塞）。
       估计值由"每次转弯后车停稳"的静止窗口测得（见 Chassis_YawDriftBegin/End）。 */
#if (CHS_YAW_DRIFT_COMP_ENABLE != 0)
    {
        uint32_t nowMs = HAL_GetTick();
        uint32_t dtMs  = nowMs - chsYawCorrTick;

        if (dtMs > 0U)
        {
            chsYawCorrDeg += chsYawDriftDps * ((float)dtMs * 0.001f);
            chsYawCorrTick = nowMs;
        }
    }
#endif

    /* 四轮位置一次全读（Chassis_ReadPos 内部已滤除使能瞬间的位置基准跳变）。
       任一轮读不到就整拍作废：chsOdomPrevClk 不动，下一拍从同一基准重算增量 */
    if (Chassis_ReadPos(cur) == 0U)
    {
        return;
    }

    /* 第一拍（或复位后首拍）：只建基准，不累加 */
    if (chsOdomReady == 0U)
    {
        for (i = 0; i < MOTOR_NUM; i++)
        {
            chsOdomPrevClk[i] = cur[i];
        }
        chsOdomReady = 1U;
        return;
    }

    for (i = 0; i < MOTOR_NUM; i++)
    {
        dClk[i] = cur[i] - chsOdomPrevClk[i];
        if (dClk[i] != 0)
        {
            moved = 1U;
        }
        chsOdomPrevClk[i] = cur[i];     /* 每拍对齐，避免同一段增量被重复累计 */
    }

    if (moved == 0U)
    {
        return;                         /* 没动就不做三角函数 */
    }

    Mecanum_ForwardPulse(dClk, &dxb, &dyb, &drot);

    th = Chassis_GlobalYawDeg() * MEC_RAD_PER_DEG;   /* θ：俯视逆时针为正 */
    c  = cosf(th);
    s  = sinf(th);

    /* 正变换（车身位移 -> 全局位移，spec §3）：
         X = dx·cosθ + dy·sinθ
         Y = −dx·sinθ + dy·cosθ
       与跑图 Route_IssueMoveAbs 的逆变换严格互逆（互为转置）。

       【2026-10-07 新增：原地旋转期间冻结平移坐标】
       原因：原地转时四轮反向转动，理论上车心不动；但打滑 / 轮径差 / 减速比误差
       会让 Mecanum_ForwardPulse 解出一小段残余 (dxb, dyb)，积分进坐标后
       表现就是"每转一次坐标就漂一点"。旋转阶段由上层调用
       Chassis_OdomHoldXY(1) 置位 -> 本拍只推进 chsOdomPrevClk、不累加坐标
       （chsOdomPrevClk 已在上面每拍对齐，所以解冻后不会"补跳"）。
       朝向 yaw 不受本开关影响：它来自 IMU（Chassis_YawUsedDeg），不在此积分。 */
    if (chsOdomHoldXY == 0U)
    {
        chsOdomX += (dxb * c) + (dyb * s);
        chsOdomY += (-dxb * s) + (dyb * c);
    }

#if (CHS_ODOM_LOG_MS > 0U)
    /* 上报位置 + 坐标 + 朝向（h = 相对原点朝向 ×10，单位 0.1°），
       供标定几何参数/排查坐标异常（CHS_ODOM_LOG_MS 置 0 关闭）。
       判断"转完弯坐标还走 X"的关键：看本行 h 是否跟着转——h≈900 才是转了 90°，
       h≈0 说明 IMU 朝向没更新（此时旋转补偿做不出来，坐标必然不对） */
    if ((HAL_GetTick() - odomLogTick) >= CHS_ODOM_LOG_MS)
    {
        odomLogTick = HAL_GetTick();
        Debug_Log("CHS", "odom p=%d,%d,%d,%d x=%d y=%d h=%d",
                  (int)cur[0], (int)cur[1], (int)cur[2], (int)cur[3],
                  (int)chsOdomX, (int)chsOdomY,
                  (int)(Chassis_GlobalYawDeg() * 10.0f));
    }
#endif

    /* 坐标有变化才置脏，让车机页的 x/y/朝向跟着动（重绘频率由 OLED 侧节流）。
       这样空闲时手动推车、坐标也会实时更新 */
    Display_ChassisRefresh();
}

/**
 * @brief  取相对原点的 x 坐标（**真实坐标系**，见 bsp_chassis.h 的坐标域补偿说明）
 * @param  无
 * @retval x，单位 mm
 * @note   内部累计 chsOdomX/Y 在 IMU 域；这里按 −补偿 旋到真实坐标系
 *         （X_true = X·cos(c) − Y·sin(c)）。补偿为 0 时直接返回原值（零开销）。
 */
float Chassis_GetOdomX(void)
{
    float r, cf, sf;

    if (chsYawCompDeg == 0.0f)
    {
        return chsOdomX;
    }
    r  = chsYawCompDeg * MEC_RAD_PER_DEG;
    cf = cosf(r);
    sf = sinf(r);
    return (chsOdomX * cf) - (chsOdomY * sf);
}

/**
 * @brief  取相对原点的 y 坐标（**真实坐标系**）
 * @param  无
 * @retval y，单位 mm
 */
float Chassis_GetOdomY(void)
{
    float r, cf, sf;

    if (chsYawCompDeg == 0.0f)
    {
        return chsOdomY;
    }
    r  = chsYawCompDeg * MEC_RAD_PER_DEG;
    cf = cosf(r);
    sf = sinf(r);
    return (chsOdomX * sf) + (chsOdomY * cf);
}

/**
 * @brief  Set current coordinate-frame compensation (deg); English only (GBK file)
 * @param  deg: compensation in deg (0 = off = previous behaviour)
 * @note   Called by bsp_task's Rotate_Comp(). See bsp_chassis.h for the rationale.
 */
void Chassis_SetYawCompDeg(float deg)
{
    chsYawCompDeg = deg;
}

/**
 * @brief  Get current coordinate-frame compensation (deg)
 */
float Chassis_GetYawCompDeg(void)
{
    return chsYawCompDeg;
}

/**
 * @brief  取**真实坐标系**的朝向（给坐标变换用；旋转控制仍用 Chassis_GetOdomYaw）
 * @param  无
 * @retval TRUE heading = Chassis_GetOdomYaw() − 补偿，单位 deg
 */
float Chassis_GetOdomYawTrue(void)
{
    return Chassis_GetOdomYaw() - chsYawCompDeg;
}

/**
 * @brief  取相对原点朝向的转角
 * @param  无
 * @retval 角度 deg，范围 ±180
 */
float Chassis_GetOdomYaw(void)
{
    return Chassis_GlobalYawDeg();
}

/**
 * @brief  判断本段到位后是否需要进入下一段
 * @param  无
 * @retval 下一阶段；CHS_PHASE_IDLE 表示整个动作已完成
 * @note   顺序：
 *           1) SPIN 第一段（直行）到位 -> SPIN 第二段（原地自转）；
 *           2) 主运动段 + 平移类动作 + 航向偏差超阈值 -> 航向纠偏段；
 *           3) 主运动段 + ARC + 两段式开关打开 -> ARC 第二段（原地旋转）；
 *           4) 其它 -> 完成。
 *         ROT / ARC / SPIN 本身就在改朝向，不做航向纠偏
 *         （SPIN 的朝向在第二段里按目标角补偿航向漂移）。
 */
static ChassisPhase_t Chassis_NextPhase(void)
{
    /* SPIN：第一段（纯直行）到位后进入第二段（原地自转到目标角） */
    if ((chsPhase == CHS_PHASE_MOVE) && (chsAct == CHS_ACT_SPIN))
    {
        return CHS_PHASE_SPINROT;
    }

    /* 平移类动作到位后做航向纠偏（ROT / ARC / SPIN 跳过） */
    if ((chsPhase == CHS_PHASE_MOVE) &&
        (chsAct != CHS_ACT_ROT) && (chsAct != CHS_ACT_ARC) && (chsAct != CHS_ACT_SPIN))
    {
        float err = Chassis_YawErrorDeg();
        if (err < 0.0f)
        {
            err = -err;
        }
        if (err > CHS_YAW_TOL_DEG)
        {
            return CHS_PHASE_YAWFIX;
        }
    }

    /* ARC 两段式回退：第一段（直行）到位后补第二段（原地旋转） */
    if ((CHS_ARC_SPLIT != 0) &&
        (chsPhase == CHS_PHASE_MOVE) && (chsAct == CHS_ACT_ARC))
    {
        return CHS_PHASE_ARCROT;
    }

    return CHS_PHASE_IDLE;
}

/* ===================== 速度模式（单步调试执行器） ===================== */

/**
 * @brief  运行期读 CHS_VEL_MODE（避免位置模式分支被判"不可达"）
 */
static uint8_t Chassis_VelModeOn(void)
{
    return (uint8_t)CHS_VEL_MODE;
}

#if (CHS_VEL_MODE != 0U)

/**
 * @brief  把角度折到 (-180, +180]
 */
static float Chassis_Wrap180(float a)
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
 * @brief  四轮下发 0 速（软停，按 acc 档减速）
 */
static void Chassis_VelStop(void)
{
    Mecanum_VelBodyRpm(0.0f, 0.0f, 0.0f, (uint8_t)CHS_VEL_ACC);
}

/**
 * @brief  启动一次"平移段"（梯形速度曲线，航向锁定在动作起点）
 * @param  dx_mm: 车体系前进位移 mm
 * @param  dy_mm: 车体系右移位移 mm
 * @param  arcDeg: 本段同时要转的角度 deg（ARC 用，0=直行）
 * @retval 无
 */
static void Chassis_VelBeginMove(int32_t dx_mm, int32_t dy_mm, float arcDeg)
{
    float d = sqrtf(((float)dx_mm * (float)dx_mm) + ((float)dy_mm * (float)dy_mm));
    float a = CHS_VEL_ACC_MMPS2;
    float vmax = CHS_VEL_MMPS;
    float tA;
    float dA;

    if (d < 1.0f)
    {
        d = 1.0f;
    }

    chsVelDirF = (float)dx_mm / d;
    chsVelDirR = (float)dy_mm / d;
    chsVelDist = d * (float)CHS_VEL_DIST_PCT / 100.0f;
    chsVelArcDeg = arcDeg;

    /* 梯形 / 三角速度曲线（起止速度都是 0） */
    tA = vmax / a;
    dA = 0.5f * vmax * tA;
    if (chsVelDist >= (2.0f * dA))
    {
        chsVelPeak    = vmax;
        chsVelTAcc    = tA;
        chsVelTCruise = (chsVelDist - (2.0f * dA)) / vmax;
    }
    else
    {
        chsVelPeak    = sqrtf(a * chsVelDist);
        chsVelTAcc    = chsVelPeak / a;
        chsVelTCruise = 0.0f;
    }
    chsVelTDecel = chsVelTAcc;
    chsVelTTotal = (2.0f * chsVelTAcc) + chsVelTCruise;

    chsPhase         = CHS_PHASE_MOVE;
    chsVelPhaseStart = HAL_GetTick();
    chsVelCtrlTick   = chsVelPhaseStart;
    chsVelOkTick     = 0U;

    Debug_Log("CHS", "vel move d=%d dir=%d,%d arc=%d t=%d",
              (int)chsVelDist, (int)(chsVelDirF * 100.0f), (int)(chsVelDirR * 100.0f),
              (int)(arcDeg * 10.0f), (int)(chsVelTTotal * 1000.0f));
}

/**
 * @brief  启动一次"原地差速转"段
 * @param  deltaDeg: 从**当前朝向**起还要转的角度 deg（CCW 正）
 * @retval 无
 * @note   只用**陀螺仪原始 yaw**：目标 = 当前原始 yaw + 本次要转的角度（单次相减求误差），
 *         不做角度累加、不扣零漂补偿。>180° 的动作按 ≤170° 分段转到同一个目标。
 */
static void Chassis_VelBeginRot(float deltaDeg)
{
    float ad   = (deltaDeg < 0.0f) ? -deltaDeg : deltaDeg;
    float yaw0 = IMU_GetYaw();
    float step = deltaDeg;

    if (step > 170.0f)
    {
        step = 170.0f;
    }
    else if (step < -170.0f)
    {
        step = -170.0f;
    }

    chsPhase         = CHS_PHASE_SPINROT;
    chsVelTurnTarget = yaw0 + step;          /* 原始 yaw 的绝对目标 */
    chsVelTurnRemain = deltaDeg - step;      /* 剩下的部分下一段接着转 */
    chsVelPhaseStart = HAL_GetTick();
    chsVelCtrlTick   = chsVelPhaseStart;
    chsVelOkTick     = 0U;
    chsVelRotLimitMs = 2500U + (uint32_t)(ad * (1000.0f / 15.0f));   /* 至少按 15°/s 给时限 */

    Debug_Log("CHS", "vel turn %d yaw=%d tgt=%d",
              (int)(deltaDeg * 10.0f), (int)(yaw0 * 10.0f),
              (int)(chsVelTurnTarget * 10.0f));
}

/**
 * @brief  速度模式状态机（20ms 节拍）
 */
static void Chassis_VelUpdate(void)
{
    uint32_t now   = HAL_GetTick();
    float    yawNow;
    float    err;
    float    aerr;
    float    wz;
    float    lim;
    float    dt;
    float    t;
    float    v;

    if (chsState != CHS_STATE_RUN)
    {
        return;
    }

    /* ★ 直接使用陀螺仪模块回传的**原始 yaw**（不扣零漂、不做角度累加）。
       chsVelRateF 是由原始 yaw 差分的角速度，仅用于 D 项阻尼，不参与角度计算。 */
    yawNow = IMU_GetYaw();
    {
        float dYaw = Chassis_Wrap180(yawNow - chsVelYawPrev);

        dt = (float)(now - chsVelCtrlTick) / 1000.0f;
        if (dt < 0.001f)
        {
            dt = 0.001f;
        }
        chsVelRateF    = (0.6f * chsVelRateF) + (0.4f * (dYaw / dt));
        chsVelYawPrev  = yawNow;
        chsVelCtrlTick = now;

        /* 页面 `a=`：原始 yaw 与动作起点的差（只用于显示） */
        chsProgDeg = Chassis_Wrap180(yawNow - chsVelYawRef);
    }

    /* ---------- 旋转段 ---------- */
    if (chsPhase == CHS_PHASE_SPINROT)
    {
        if ((now - chsVelPhaseStart) >= chsVelRotLimitMs)
        {
            Chassis_VelStop();
            Debug_Log("CHS", "vel turn giveup err=%d yaw=%d",
                      (int)(Chassis_Wrap180(chsVelTurnTarget - yawNow) * 10.0f),
                      (int)(yawNow * 10.0f));
            chsState = CHS_STATE_DONE;
            chsPhase = CHS_PHASE_IDLE;
            return;
        }

        err  = Chassis_Wrap180(chsVelTurnTarget - yawNow);
        aerr = (err < 0.0f) ? -err : err;

        /* ---- 到位判定：死区 + 角速度门限 + 连续保持 HOLD_MS ---- */
        if ((aerr <= CHS_VEL_ROT_TOL_DEG) &&
            (((chsVelRateF < 0.0f) ? -chsVelRateF : chsVelRateF) <= CHS_VEL_ROT_ARRIVE_DPS))
        {
            if (((chsVelTurnRemain < 0.0f) ? -chsVelTurnRemain : chsVelTurnRemain) >
                CHS_VEL_ROT_TOL_DEG)
            {
                /* >180° 的动作：本段到了，接着转下一段（不停车） */
                float step = chsVelTurnRemain;

                if (step > 170.0f)
                {
                    step = 170.0f;
                }
                else if (step < -170.0f)
                {
                    step = -170.0f;
                }
                chsVelTurnTarget += step;
                chsVelTurnRemain -= step;
                err  = Chassis_Wrap180(chsVelTurnTarget - yawNow);
                aerr = (err < 0.0f) ? -err : err;
                chsVelOkTick = 0U;
            }
            else
            {
                if (chsVelOkTick == 0U)
                {
                    chsVelOkTick = now;
                }
                if ((now - chsVelOkTick) >= CHS_VEL_ROT_HOLD_MS)
                {
                    Chassis_VelStop();
                    Debug_Log("CHS", "vel turn ok err=%d rate=%d yaw=%d",
                              (int)(err * 10.0f), (int)(chsVelRateF * 10.0f),
                              (int)(yawNow * 10.0f));
                    chsState = CHS_STATE_DONE;
                    chsPhase = CHS_PHASE_IDLE;
                    return;
                }
            }
        }
        else
        {
            chsVelOkTick = 0U;
        }

        /* ---- 连续控制律：位置 P -> 速度参考 -> 速度环 P ----
           rate_ref = min(KP_POS*|err|, √(2·ACC*|err|), MAX)，死区外不低于 MIN；
           wz = rate_ref + KP_RATE·(rate_ref − 实测角速度)，限幅 ±MAX。
           实际转太快时 wz 会转负 -> 真正刹车；没有远近段切换，不会在死区边缘反复。 */
        {
            float ref = CHS_VEL_ROT_KP_POS * aerr;
            float env = sqrtf(2.0f * CHS_VEL_ROT_ACC_DPS2 * aerr);

            if (ref > env)
            {
                ref = env;
            }
            if (ref > CHS_VEL_ROT_MAX_DPS)
            {
                ref = CHS_VEL_ROT_MAX_DPS;
            }
            if ((aerr > CHS_VEL_ROT_TOL_DEG) && (ref < CHS_VEL_ROT_MIN_DPS))
            {
                ref = CHS_VEL_ROT_MIN_DPS;
            }
            if (err < 0.0f)
            {
                ref = -ref;
            }

            wz = ref + (CHS_VEL_ROT_KP_RATE * (ref - chsVelRateF));
        }
        if (wz > CHS_VEL_ROT_MAX_DPS)        { wz =  CHS_VEL_ROT_MAX_DPS; }
        else if (wz < -CHS_VEL_ROT_MAX_DPS)  { wz = -CHS_VEL_ROT_MAX_DPS; }

        /* ★ Mecanum 逆解的 W 正方向实测为**顺时针**，而 wz 是 CCW 正 → 取反 */
        Mecanum_VelBodyRpm(0.0f, 0.0f, -wz * CHS_RPM_PER_DPS, (uint8_t)CHS_VEL_ACC);
        return;
    }

    /* ---------- 平移段 ---------- */
    t = (float)(now - chsVelPhaseStart) / 1000.0f;

    if (t >= chsVelTTotal)
    {
        Chassis_VelStop();
        if (chsVelOkTick == 0U)
        {
            chsVelOkTick = now;
        }
        if ((now - chsVelOkTick) >= CHS_VEL_STOP_MS)
        {
            chsVelOkTick = 0U;

            /* SPIN：直行完接着转；ARC：先把航向残差补齐；其余动作直接完成 */
            if (chsAct == CHS_ACT_SPIN)
            {
                Chassis_VelBeginRot((float)CHS_SPIN_DEG);
                return;
            }
            if (chsAct == CHS_ACT_ARC)
            {
                float res = chsVelArcDeg - Chassis_Wrap180(yawNow - chsVelYawRef);

                if (((res < 0.0f) ? -res : res) > CHS_VEL_ROT_TOL_DEG)
                {
                    Chassis_VelBeginRot(res);
                    return;
                }
            }

            Debug_Log("CHS", "vel done %s dist=%d deg=%d",
                      Chassis_GetActName((uint8_t)chsAct),
                      (int)chsProgDist,
                      (int)(Chassis_Wrap180(yawNow - chsVelYawRef) * 10.0f));
            chsState = CHS_STATE_DONE;
            chsPhase = CHS_PHASE_IDLE;
        }
        return;
    }
    chsVelOkTick = 0U;

    /* 速度曲线：0 -> 峰值 -> 0，正弦（S 形）过渡。
       与同时间的线性斜坡面积相同（平均速度都是 (v1+v2)/2），距离不变，
       但两端 jerk 为零，起步/停稳更不容易打滑。 */
    if (t < chsVelTAcc)
    {
        float s = (chsVelTAcc > 0.001f) ? (t / chsVelTAcc) : 1.0f;

        v = chsVelPeak * (0.5f - (0.5f * cosf(MEC_PI * s)));
    }
    else if (t < (chsVelTAcc + chsVelTCruise))
    {
        v = chsVelPeak;
    }
    else
    {
        float s = (chsVelTDecel > 0.001f) ? ((chsVelTTotal - t) / chsVelTDecel) : 0.0f;

        if (s > 1.0f)
        {
            s = 1.0f;
        }
        v = chsVelPeak * (0.5f - (0.5f * cosf(MEC_PI * s)));
    }
    if (v < 0.0f)
    {
        v = 0.0f;
    }

    /* 航向：直行锁"动作起点的原始 yaw"；ARC 在它上面按距离线性加斜坡 */
    if ((chsVelArcDeg != 0.0f) && (chsVelTTotal > 0.0f))
    {
        err = Chassis_Wrap180((chsVelYawRef + (chsVelArcDeg * (t / chsVelTTotal))) - yawNow);
    }
    else
    {
        err = Chassis_Wrap180(chsVelYawRef - yawNow);
    }

    wz = (CHS_VEL_YAW_KP * err) - (CHS_VEL_YAW_KD * chsVelRateF);
    lim = (chsVelArcDeg != 0.0f) ? CHS_VEL_ARC_DPS_CAP : CHS_VEL_YAW_MAX_DPS;
    if (wz > lim)        { wz =  lim; }
    else if (wz < -lim)  { wz = -lim; }

    /* Straight actions: no heading correction during the decel ramp, otherwise
       the car visibly turns while stopping. It just decelerates and stops on
       all four wheels. (ARC keeps correcting because it is a curve by design.) */
    if ((chsVelArcDeg == 0.0f) && (t >= (chsVelTAcc + chsVelTCruise)))
    {
        wz = 0.0f;
    }

    Mecanum_VelBodyRpm(v * chsVelDirF * CHS_RPM_PER_MMPS,
                       v * chsVelDirR * CHS_RPM_PER_MMPS,
                       -wz * CHS_RPM_PER_DPS, (uint8_t)CHS_VEL_ACC);
}

#endif /* CHS_VEL_MODE */

/* ===================== 公开接口 ===================== */

/**
 * @brief  车机运动模块初始化（只复位状态，不发电机命令）
 * @param  无
 * @retval 无
 */
void Chassis_Init(void)
{
    chsState    = CHS_STATE_IDLE;
    chsAct      = CHS_ACT_FWD;
    chsPhase    = CHS_PHASE_IDLE;
    chsSegTick  = HAL_GetTick();
    chsProgDist = 0.0f;
    chsProgDeg  = 0.0f;

#if (CHS_VEL_MODE != 0U)
    chsVelTick     = HAL_GetTick();
    chsVelYawRef   = IMU_GetYaw();
    chsVelRateF    = 0.0f;
    chsVelOkTick   = 0U;
    chsVelTurnRemain = 0.0f;
#endif

    Debug_Log("CHS", "init, dist=%d rot=%d diag=%d arc=%d/%d spin=%d/%d tol=%d",
              (int)CHS_DIST_MM, (int)CHS_ROT_DEG, (int)CHS_DIAG_DIST_MM,
              (int)CHS_ARC_DIST_MM, (int)CHS_ARC_DEG,
              (int)CHS_SPIN_DIST_MM, (int)CHS_SPIN_DEG,
              (int)CHS_PULSE_TOL);
}

/**
 * @brief  启动一个动作（使能四轮 + 下发位置模式目标）
 * @param  act: 动作编号，超范围忽略
 * @retval 无
 */
void Chassis_Start(uint8_t act)
{
    int32_t dx     = 0;
    int32_t dy     = 0;
    int32_t dRotMm = 0;
    uint8_t addr;
    uint8_t i;

    if (act >= (uint8_t)CHS_ACT_COUNT)
    {
        Debug_Log("CHS", "start bad act=%u", (unsigned int)act);
        return;
    }

    /* 位置模式的到位判定完全依赖位置数据，读不到就不启动；
       读到的位置同时作为本次动作的进度基准 chsStartBase。
       速度模式（CHS_VEL_MODE=1）不依赖位置数据，读不到也允许启动（只影响 D= 显示）。 */
    if (Chassis_ReadPos(chsStartBase) == 0U)
    {
        if (Chassis_VelModeOn() == 0U)
        {
            Debug_Log("CHS", "no pos data, start rejected");
            return;
        }
        for (i = 0; i < MOTOR_NUM; i++)
        {
            chsStartBase[i] = 0;
        }
    }

    /* 使能四轮（使能只在这里发生；退出/中止时由 Chassis_Abort 失能） */
    for (addr = MOTOR_ADDR_MIN; addr <= MOTOR_ADDR_MAX; addr++)
    {
        Motor_Enable(addr, 1U);
    }

    /* 记录起始航向，进度清零（与 Chassis_YawDelta 同源） */
    chsYaw0     = Chassis_YawUsedDeg();
    chsProgDist = 0.0f;
    chsProgDeg  = 0.0f;

    /* 按动作换算整车位移（mm） */
    switch ((ChassisAct_t)act)
    {
        case CHS_ACT_FWD:
            dx = (int32_t)CHS_DIST_MM;
            break;
        case CHS_ACT_BACK:
            dx = -(int32_t)CHS_DIST_MM;
            break;
        case CHS_ACT_LEFT:
            dy = -(int32_t)CHS_DIST_MM;   /* 左移 = 车体 y 负方向 */
            break;
        case CHS_ACT_RIGHT:
            dy = (int32_t)CHS_DIST_MM;    /* 右移 = 车体 y 正方向 */
            break;
        case CHS_ACT_ROT:
            dRotMm = (int32_t)Mecanum_DegToArcMm((float)CHS_ROT_DEG);
            break;
        case CHS_ACT_DIAG:
            dx = (int32_t)CHS_DIAG_DIST_MM;
            dy = (int32_t)CHS_DIAG_DIST_MM;
            break;
        case CHS_ACT_ARC:
            dx = (int32_t)CHS_ARC_DIST_MM;
            if (CHS_ARC_SPLIT == 0)
            {
                /* 等时长法：平移与旋转一次下发，四轮同时到位 */
                dRotMm = (int32_t)Mecanum_DegToArcMm((float)CHS_ARC_DEG);
            }
            /* CHS_ARC_SPLIT != 0 时本段只直行，旋转放到第二段 */
            break;
        case CHS_ACT_SPIN:
            /* 第一段：纯直行到 CHS_SPIN_DIST_MM（等价于 FWD）；
               到位后由 Chassis_NextPhase 进入第二段（原地自转 CHS_SPIN_DEG） */
            dx = (int32_t)CHS_SPIN_DIST_MM;
            break;
        default:
            break;
    }

    chsAct   = (ChassisAct_t)act;
    chsState = CHS_STATE_RUN;
    chsPhase = CHS_PHASE_MOVE;

#if (CHS_VEL_MODE != 0U)
    if (Chassis_VelModeOn() != 0U)
    {
        /* 速度模式：清基准（判到位基准用不到，但保持结构干净），起计时 */
        for (i = 0; i < MOTOR_NUM; i++)
        {
            chsSegBase[i] = 0;
            chsSegClk[i]  = 0;
        }

        chsSegTick    = HAL_GetTick();
        chsVelTick    = chsSegTick;
        chsVelYawPrev = IMU_GetYaw();      /* 原始 yaw */
        chsVelYawRef  = chsVelYawPrev;
        chsVelRateF   = 0.0f;
        chsVelOkTick  = 0U;
        chsVelTurnRemain = 0.0f;

        if (act == (uint8_t)CHS_ACT_ROT)
        {
            Chassis_VelBeginRot((float)CHS_ROT_DEG);
        }
        else
        {
            Chassis_VelBeginMove(dx, dy,
                                 (act == (uint8_t)CHS_ACT_ARC) ? (float)CHS_ARC_DEG : 0.0f);
        }

        Debug_Log("CHS", "start %s vel dx=%d dy=%d yaw=%d",
                  Chassis_GetActName(act), (int)dx, (int)dy,
                  (int)(chsYaw0 * 10.0f));
        return;
    }
#endif

    /* 本段判到位基准 = 动作起始位置（第一段从动作起点开始） */
    chsSegTick = HAL_GetTick();
    for (i = 0; i < MOTOR_NUM; i++)
    {
        chsSegBase[i] = chsStartBase[i];
        chsSegClk[i]  = 0;
    }

    Chassis_IssueMove(dx, dy, dRotMm);

    Debug_Log("CHS", "start %s dx=%d dy=%d rot=%d yaw=%d",
              Chassis_GetActName(act), (int)dx, (int)dy, (int)dRotMm,
              (int)(chsYaw0 * 10.0f));
}

/**
 * @brief  中止当前动作（四轮急停 + 失能 + 状态复位）
 * @param  无
 * @retval 无
 * @note   仅在确实运动中时才发急停，避免空闲时刷一堆无意义的停止日志。
 */
void Chassis_Abort(void)
{
    uint8_t addr;

    if (chsState == CHS_STATE_RUN)
    {
        Mecanum_Stop();
        chsState = CHS_STATE_ABORT;
        Debug_Log("CHS", "abort %s", Chassis_GetActName((uint8_t)chsAct));
    }

    chsPhase = CHS_PHASE_IDLE;

#if (CHS_VEL_MODE != 0U)
    /* 速度模式：清掉段内状态，避免下次启动带着上一段的角速度/剩余角 */
    chsVelRateF      = 0.0f;
    chsVelOkTick     = 0U;
    chsVelTurnRemain = 0.0f;
#endif

    /* 失能四轮：避免退出页面后电机长时间抱轴发热 */
    for (addr = MOTOR_ADDR_MIN; addr <= MOTOR_ADDR_MAX; addr++)
    {
        Motor_Disable(addr);
    }
}

/**
 * @brief  车机运动状态机推进（主循环调用，非阻塞）
 * @param  无
 * @retval 无
 */
void Chassis_Update(void)
{
    ChassisPhase_t next;
    uint8_t        reached;

    if (chsState != CHS_STATE_RUN)
    {
        return;
    }

    /* 段超时兜底：位置数据异常时不允许车轮状态机一直挂着 */
    if ((HAL_GetTick() - chsSegTick) >= CHS_TIMEOUT_MS)
    {
        Mecanum_Stop();
        chsState = CHS_STATE_ABORT;
        chsPhase = CHS_PHASE_IDLE;
        Debug_Log("CHS", "timeout, abort %s", Chassis_GetActName((uint8_t)chsAct));
        Display_ChassisRefresh();
        return;
    }

#if (CHS_VEL_MODE != 0U)
    /* 速度模式：20ms 节拍；到位判据只看陀螺仪，不看电机回传/脉冲 */
    if (Chassis_VelModeOn() != 0U)
    {
        if ((HAL_GetTick() - chsVelTick) >= CHS_VEL_TICK_MS)
        {
            chsVelTick = HAL_GetTick();
            Chassis_UpdateProgress();   /* D= 仍按编码器统计（只用于显示） */
            Chassis_VelUpdate();
        }
        Display_ChassisRefresh();
        return;
    }
#endif

    /* ---- 各段统一处理：更新进度 -> 判到位 -> 决定下一段 ---- */
    Chassis_UpdateProgress();

    /* 段到位判定：**优先等电机主动回传的到位帧**（电机自己的判据，最可靠）；
       回传超时/未启用时才退回四轮脉冲容差判定，避免漏帧导致整段挂死 */
    reached = Chassis_SegArrivedByReach();

    if ((reached == 0U) &&
        (Chassis_SegArrived((int32_t)CHS_PULSE_TOL) != 0U) &&
        (Chassis_PulseFallbackAllowed() != 0U))
    {
        reached = 1U;
    }

    if (reached == 0U)
    {
        Display_ChassisRefresh();
        return;
    }

    /* 本段到位：决定是否进入下一段 */
    next = Chassis_NextPhase();

    /* 段完成诊断：next=0 表示整个动作结束；reach 为四轮到位回传掩码
       （15=四轮都靠回传确认，0=回传没收到来、走了脉冲兜底） */
    Debug_Log("CHS", "seg done next=%u act=%s reach=%u yaw0=%d yaw=%d",
              (unsigned int)next, Chassis_GetActName((uint8_t)chsAct),
              (unsigned int)Chassis_ReachMask(),
              (int)(chsYaw0 * 10.0f), (int)(Chassis_YawUsedDeg() * 10.0f));

    if (next == CHS_PHASE_IDLE)
    {
        chsState = CHS_STATE_DONE;
        chsPhase = CHS_PHASE_IDLE;
        Debug_Log("CHS", "done %s dist=%d deg=%d",
                  Chassis_GetActName((uint8_t)chsAct),
                  (int)chsProgDist, (int)(chsProgDeg * 10.0f));
        Display_ChassisRefresh();
        return;
    }

    /* 开新段前必须重新取位置基准（位置模式是相对当前位置） */
    if (Chassis_BeginSeg() == 0U)
    {
        Mecanum_Stop();
        chsState = CHS_STATE_ABORT;
        chsPhase = CHS_PHASE_IDLE;
        Debug_Log("CHS", "next seg failed, abort");
        Display_ChassisRefresh();
        return;
    }

    chsPhase = next;

    if (next == CHS_PHASE_YAWFIX)
    {
        /* 纠偏：往漂移的反方向转。
           Chassis_YawErrorDeg() 返回 θ（**俯视逆时针为正**）；
           而旋转指令的正方向实测为**顺时针**（表里写 -90 相当于逆时针左转），
           两者符号相反，所以"抵消 θ 的旋转量"= 指令 +θ。 */
        float err  = Chassis_YawErrorDeg();
        int32_t arc = (int32_t)Mecanum_DegToArcMm(err);

        Debug_Log("CHS", "yawfix err=%d arc=%d", (int)(err * 10.0f), (int)arc);
        Chassis_IssueMove(0, 0, arc);
    }
    else if (next == CHS_PHASE_SPINROT)
    {
        /* SPIN 第二段：原地自转 CHS_SPIN_DEG。
           若第一段有**合理范围内**的航向漂移，就从目标角里扣掉，使终态朝向
           正好是"动作起始朝向 + CHS_SPIN_DEG"。
           注意符号：`err` 是 θ（俯视逆时针为正），而旋转指令的正方向实测为
           顺时针，所以"用指令表示当前朝向"= −θ，剩余指令量 = 目标 − (−θ) = 目标 + θ。
           漂移值离谱（陀螺仪未初始化/数据异常）时必须忽略，否则会被吃成 0，
           导致第二段不下发、直接判到位
           （现象：直行正常但完全不转、页面 a= 停在 0） */
        float  err  = Chassis_YawErrorDeg();
        float  need = (float)CHS_SPIN_DEG;

        if ((err > -CHS_SPIN_DRIFT_MAX_DEG) && (err < CHS_SPIN_DRIFT_MAX_DEG))
        {
            need += err;
        }
        else
        {
            Debug_Log("CHS", "spin drift=%d out of range, ignore", (int)(err * 10.0f));
        }

        Debug_Log("CHS", "spin rot seg need=%d deg drift=%d arc=%d",
                  (int)(need * 10.0f), (int)(err * 10.0f),
                  (int)Mecanum_DegToArcMm(need));
        Chassis_IssueMove(0, 0, (int32_t)Mecanum_DegToArcMm(need));
    }
    else /* CHS_PHASE_ARCROT：ARC 两段式的第二段——原地旋转 */
    {
        Debug_Log("CHS", "arc rot seg, %d deg", (int)CHS_ARC_DEG);
        Chassis_IssueMove(0, 0, (int32_t)Mecanum_DegToArcMm((float)CHS_ARC_DEG));
    }

    Display_ChassisRefresh();
}

/**
 * @brief  查询是否正在运动
 * @param  无
 * @retval 1=运动中，0=非运动
 */
uint8_t Chassis_IsActive(void)
{
    return (chsState == CHS_STATE_RUN) ? 1U : 0U;
}

/**
 * @brief  查询是否已到位完成
 * @param  无
 * @retval 1=已到位（DONE），0=其它状态
 */
uint8_t Chassis_IsDone(void)
{
    return (chsState == CHS_STATE_DONE) ? 1U : 0U;
}

/**
 * @brief  取当前/上次动作编号
 * @param  无
 * @retval ChassisAct_t 值
 */
uint8_t Chassis_GetAct(void)
{
    return (uint8_t)chsAct;
}

/**
 * @brief  取当前状态文字
 * @param  无
 * @retval "IDLE" / "RUN" / "DONE" / "ABORT"
 */
const char *Chassis_GetStateText(void)
{
    switch (chsState)
    {
        case CHS_STATE_RUN:   return "RUN";
        case CHS_STATE_DONE:  return "DONE";
        case CHS_STATE_ABORT: return "ABORT";
        case CHS_STATE_IDLE:
        default:              return "IDLE";
    }
}

/**
 * @brief  取动作名称文字
 * @param  act: 动作编号
 * @retval "FWD"/"BACK"/"LEFT"/"RIGHT"/"ROT"/"DIAG"/"ARC"/"SPIN"；非法返回 "???"
 */
const char *Chassis_GetActName(uint8_t act)
{
    switch ((ChassisAct_t)act)
    {
        case CHS_ACT_FWD:   return "FWD";
        case CHS_ACT_BACK:  return "BACK";
        case CHS_ACT_LEFT:  return "LEFT";
        case CHS_ACT_RIGHT: return "RIGHT";
        case CHS_ACT_ROT:   return "ROT";
        case CHS_ACT_DIAG:  return "DIAG";
        case CHS_ACT_ARC:   return "ARC";
        case CHS_ACT_SPIN:  return "SPIN";
        default:            return "???";
    }
}

/**
 * @brief  取本次动作已走的平移距离（进度显示）
 * @param  无
 * @retval 已完成平移距离，单位 mm
 */
int16_t Chassis_GetDistMm(void)
{
    return (int16_t)chsProgDist;
}

/**
 * @brief  取本次动作已转的角度（进度显示）
 * @param  无
 * @retval 已完成转角，单位 deg
 */
int16_t Chassis_GetAngleDeg(void)
{
    return (int16_t)chsProgDeg;
}
