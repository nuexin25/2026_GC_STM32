/**
 ******************************************************************************
 * @file    bsp_vel_test.c
 * @brief   速度模式测试页实现（P25）：纯陀螺仪航向 + 时间开环距离
 * @author  nx_car
 * @date    2026-10-24
 ******************************************************************************
 * @note   【为什么单独一页】赛题跑图目前用位置模式；这一页用来验证
 *         "速度模式(0xF6) + IMU 航向 + 时间控距离" 是否够用，**完全不读轮编码器**，
 *         方便现场用皮尺对比"设定时间 vs 实际距离"。
 *
 *         【状态机】★【2026-10-24 用户要求】**过弯不停车**（低速转直角弯）：
 *           READY -> RUN1(加速到 V，跑 T1)
 *                 -> CORNER(先按 A 降到过弯速度 VC；再**边低速前进边转 G 度**)
 *                 -> RUN2(从 VC 加速到 V，跑 T2)
 *                 -> DEC2(减速停) -> DONE
 *         过弯前后**没有停稳等待**，也不把速度降到 0（VC 可调；VC=0 即原地慢转）。
 *
 *         【航向】全程 IMU：直行段做航向保持（KP/KD）；
 *                过弯段 = 低速前进 + 航向闭环（转弯半径 ≈ VC / 角速度）；
 *                最后一段减速时不再修正航向（否则"边减速边扭"）。
 *
 *         【距离】**开环**：只有速度指令和时间，没有任何轮速/轮位反馈。
 *
 *         硬约束：禁止 HAL_Delay；日志禁 %f；全部非阻塞。
 ******************************************************************************
 */
#include "bsp_vel_test.h"
#include "bsp_mecanum.h"      /* Mecanum_VelBodyRpm / MEC_WHEEL_CIRC_MM */
#include "bsp_motor.h"        /* Motor_Enable / Motor_Disable */
#include "bsp_imu.h"          /* IMU_GetYaw / IMU_IsOnline */
#include "bsp_chassis.h"      /* ★Chassis_OdomReset / GetOdomX/Y：正方形路线的距离闭环 */
#include "bsp_oled_ui.h"      /* Display_SetPage / Display_MenuEnter / Display_VelTestRefresh */
#include "bsp_debug.h"
#include <stdio.h>
#include <math.h>

/* ===================== 参数 ===================== */

#define VT_TICK_MS            20U     /* 速度环控制周期 ms（与跑图速度模式一致） */
#define VT_ACC_DEF            0U      /* ★0xF6 的加减速档（越大越缓）
                                         【2026-10-24 实测】40 -> 10 -> 0：
                                         40 时下发 100°/s 要 ~1s 才跟上（像大惯性对象，
                                         转弯必冲过头）；0 时轮速几乎立即跟随。
                                         可串口调 AC=。 */

/* 车体速度 -> 轮速（与 bsp_route.c 同一换算，避免两套口径） */
#define VT_RPM_PER_MMPS       (60.0f / MEC_WHEEL_CIRC_MM)
#define VT_RPM_PER_DPS        ((MEC_PI / 180.0f) * MEC_ROT_RADIUS_MM * 60.0f / MEC_WHEEL_CIRC_MM)

/* 直行航向保持（PD）
   ★【2026-10-24 修"转弯后走蛇形"】原来的 KP=1.6 / KD=0.30 阻尼比只有 ~0.1，
     属于**严重欠阻尼** —— 航向一有偏差就来回摆（现场就是蛇形）。
     按 ζ ≈ KD/(2·√KP) 取 ζ≈0.7：KP=1.2、KD=1.5（高速段用 1.2 才能拉回打滑造成的甩头）。
     另外加一个很小的死区（0.5° / 5dps），避免零漂引起的持续小幅摆动。 */
#define VT_YAW_KP_DEF         1.2f
#define VT_YAW_KD_DEF         1.5f
#define VT_YAW_MAX_DPS        40.0f    /* ★20 -> 40：高速段（V≥1200）加速时轮子打滑会
                                         把车头甩十几度，20°/s 的修正量压不住 -> 越跑越歪 */
#define VT_YAW_BRAKE_DPS      60.0f    /* ★刹车段专用：1300mm/s 急刹会自转 100°+，
                                         刹车时允许更大的航向修正量 */
#define VT_YAW_DEAD_DEG       0.5f
#define VT_YAW_DEAD_DPS       5.0f

/* 原地转向（可串口调：TRK/TRR/TRM/TRA）
   ★【2026-10-24 现场整定】纯位置 P + 制动包络（TRR=0）在这台车上最稳：
     · TRK=1.5：位置增益，太大出弯会摆；
     · TRM=90 ：角速度上限，90°/s 转 90° 约 1.5~1.8 s；
     · TRA=40 ：制动包络（×10 = 400 dps²），保证接近目标时提前减速、不过冲。 */
static float      vtTrKp     = 3.5f;      /* 位置环 P（dps per deg） */
static float      vtTrKr     = 0.0f;      /* 角速度环 P（0=不用，反馈噪声大） */
static float      vtTrMax    = 200.0f;    /* 角速度上限 dps（再大反而慢：轮子打滑跟不上） */
static float      vtTrAcc    = 1500.0f;   /* 制动包络 dps^2 */
static float      vtTrDec    = 800.0f;    /* ★过弯前的减速率 mm/s²（单独设，别用直行的 A 慢慢磨） */

#define VT_TURN_MIN_DPS        6.0f    /* 死区外的最低角速度（克服静摩擦） */
#define VT_CORNER_BLEND_MMPS   120.0f  /* ★离过弯速度还差 120mm/s 就开始带转（衔接不断档） */
#define VT_WZ_SLEW_DPS2        800.0f  /* ★转向指令斜率限制：0->200°/s 约 0.25 s，避免猛甩 */
#define VT_TURN_ARRIVE_DPS     15.0f   /* 出弯门限：滤波角速度低于此即认为转完
                                         （6 太小 —— 车早已不转，只是滤波尾巴在掉，
                                           每个弯多等 ~0.3 s） */
#define VT_TURN_TOL_DEG        1.0f    /* 到位死区 */
#define VT_TURN_HOLD_MS        60U     /* 到位需保持的时间 */
#define VT_TURN_TIMEOUT_MS     8000U   /* 过弯超时（低速转向留足时间） */

/* 参数表（页面可调） */
typedef enum {
    VT_P_V = 0,     /* 巡航速度 mm/s */
    VT_P_T1,        /* 第一段直行总时间 ms */
    VT_P_T2,        /* 第二段直行总时间 ms */
    VT_P_A,         /* 加减速度 mm/s^2 */
    VT_P_G,         /* 过弯转角 deg（正=左转，负=右转） */
    VT_P_VC,        /* 过弯速度 mm/s（0=原地慢转） */
    VT_P_KP,        /* 直行航向保持 KP（×0.1） */
    VT_P_KD,        /* 直行航向保持 KD（×0.1） */
    VT_P_SQ,        /* ★0=单段测试  1=正方形路线（里程计距离闭环） */
    VT_P_LEG,       /* ★正方形边长 mm */
    VT_P_COUNT
} VelParam_t;

/* 状态 */
typedef enum {
    VT_IDLE = 0,
    VT_RUN1,
    VT_CORNER,
    VT_RUN2,
    VT_DEC2,
    VT_RT_MOVE,     /* ★路线模式：直行到里程计距离 */
    VT_RT_TURN,     /* ★路线模式：原地转到目标航向 */
    VT_DONE,
    VT_ERR
} VelState_t;

static uint8_t    vtActive   = 0U;        /* 本页活动 */
static VelState_t vtState    = VT_IDLE;
static uint8_t    vtSel      = 0U;        /* 当前选中的参数（VelParam_t） */

static float      vtV        = 1200.0f;   /* mm/s（★现场整定：1200 内直线还能保持） */
static uint32_t   vtT1       = 2600U;     /* ms（约 1.7 m） */
static uint32_t   vtT2       = 0U;        /* ms（★0 = 转完就停，不再直行） */
static float      vtA        = 900.0f;    /* mm/s^2 */
static float      vtG        = 180;       /* deg（★180 = 原地掉头；90 = 直角） */
static float      vtVC       = 0.0f;      /* 进弯速度：0 = **原地转**（不打弧） */
static float      vtKP       = VT_YAW_KP_DEF;   /* 直行航向保持 KP */
static float      vtKD       = VT_YAW_KD_DEF;   /* 直行航向保持 KD */
static uint8_t    vtAccLvl   = VT_ACC_DEF;      /* 0xF6 加减速档（越大越缓，串口 AC= 可调） */

/* ---- 正方形路线（方案A：正交 + 里程计距离闭环） ---- */
#define VT_RT_STEPS      8U       /* MOVE/TURN × 4 = 8 步 */
#define VT_RT_TOL_MM     12.0f    /* 直行到位容差 mm */
#define VT_RT_ANGLE      90.0f    /* 每个角左转 90° */

static uint8_t    vtSQ       = 1U;        /* ★1=上电默认跑正方形路线；0=单段测试 */
static float      vtLeg      = 1200.0f;   /* 正方形边长 mm */
static uint8_t    vtRtIdx    = 0U;        /* 当前步 0..7（偶=直行，奇=转向） */
static float      vtRtTravel = 0.0f;      /* 本段已走 mm（里程计增量累加） */
static float      vtRtPrevX  = 0.0f;
static float      vtRtPrevY  = 0.0f;
static float      vtRtTgtC   = 0.0f;      /* 本段转向目标（连续航向） */

static uint32_t   vtTick        = 0U;     /* 上次控制节拍 */
static uint32_t   vtStateTick   = 0U;     /* 进入本状态时刻 */
static uint32_t   vtLegTick     = 0U;     /* 本段直行开始时刻 */
static uint32_t   vtRunTick     = 0U;     /* 整次启动时刻（日志用） */
static uint32_t   vtCtrlTick    = 0U;     /* 角速度估算上一拍 */
static uint32_t   vtOkTick      = 0U;     /* 转弯进入死区的时刻 */

static float      vtSpeed    = 0.0f;      /* 当前速度指令 mm/s */
static float      vtYawLeg   = 0.0f;      /* 本段直行航向基准（wrap 后） */
static float      vtYawCont  = 0.0f;      /* ★连续航向：从启动开始累加，不受 ±180 翻号影响 */
static float      vtYawRaw   = 0.0f;      /* 上一拍 IMU 原始 yaw（累加用） */
static float      vtTurnTgtC = 0.0f;      /* ★转弯目标（连续量）：vtYawCont + G */
static float      vtRateF    = 0.0f;      /* 实测角速度（低通） */
static float      vtWzLast   = 0.0f;      /* 最近一次下发的转向角速度（dps） */
static float      vtWzCmd    = 0.0f;      /* 过弯转向指令（经斜率限制后的实际下发值） */
static uint8_t    vtCornerTurning = 0U;   /* 过弯段：0=还在降速，1=已在边跑边转 */
static uint16_t   vtStopHold = 0U;        /* ★停车后"补发 0 速度"的剩余拍数 */

/* 串口实时数据 */
static uint16_t   vtTeleMs   = 50U;       /* 0=关闭；默认 50ms=20Hz */
static uint32_t   vtTeleTick = 0U;

static char       vtLine[4][20];

/* ===================== 内部工具 ===================== */

static float Vt_Wrap180(float a)
{
    while (a > 180.0f)  { a -= 360.0f; }
    while (a < -180.0f) { a += 360.0f; }
    return a;
}

static const char *Vt_StateName(void)
{
    switch (vtState)
    {
        case VT_RUN1:   return "RUN1";
        case VT_CORNER: return "CORNER";
        case VT_RUN2:   return "RUN2";
        case VT_DEC2:   return "DEC2";
        case VT_DONE:   return "DONE";
        case VT_RT_MOVE: return "RTMOV";
        case VT_RT_TURN: return "RTTURN";
        case VT_ERR:    return "ERR";
        default:        return "READY";
    }
}

static uint8_t Vt_Running(void)
{
    return ((vtState == VT_RUN1) || (vtState == VT_CORNER) ||
            (vtState == VT_RUN2) || (vtState == VT_DEC2) ||
            (vtState == VT_RT_MOVE) || (vtState == VT_RT_TURN)) ? 1U : 0U;
}

/* 下发速度（wz 为 CCW 正，麦轮混控里 W>0=CW，故取负） */
static void Vt_Send(float v, float wz)
{
    vtWzLast = wz;
    Mecanum_VelBodyRpm(v * VT_RPM_PER_MMPS, 0.0f,
                       -wz * VT_RPM_PER_DPS, vtAccLvl);
}

static void Vt_StopSpeed(void)
{
    vtSpeed = 0.0f;
    Mecanum_VelBodyRpm(0.0f, 0.0f, 0.0f, vtAccLvl);

    /* ★【2026-10-24 修"停车后轮子还在转"】0xF6 是 CAN 帧、没有应答，
       偶尔丢一帧就会有个轮子保持原速 -> 车原地打圈。
       这里登记"补发 0 速度"的拍数，由 VelTest_Update 连续再发 ~0.5s。 */
    vtStopHold = 25U;
}

/* 直行段的航向保持项（PD，带小死区防小幅摆动） */
static float Vt_YawHoldLim(float yaw, float lim)
{
    float err  = Vt_Wrap180(vtYawLeg - yaw);
    float aerr = (err < 0.0f) ? -err : err;
    float ar   = (vtRateF < 0.0f) ? -vtRateF : vtRateF;
    float wz;

    if ((aerr <= VT_YAW_DEAD_DEG) && (ar <= VT_YAW_DEAD_DPS))
    {
        return 0.0f;                    /* 已在小死区内：不再纠，避免来回摆 */
    }

    wz = (vtKP * err) - (vtKD * vtRateF);

    if (wz > lim)       { wz =  lim; }
    else if (wz < -lim) { wz = -lim; }
    return wz;
}

static float Vt_YawHold(float yaw)
{
    return Vt_YawHoldLim(yaw, VT_YAW_MAX_DPS);
}

/**
 * @brief  转向一拍的公共实现（直角弯 / 原地转 共用）
 * @param  fwd : 转向同时保持的前进速度（原地转传 0）
 * @param  tgtC: 目标航向（连续航向 vtYawCont 域）
 * @param  now : 当前时刻
 * @param  dt  : 控制周期 s
 * @retval 0=进行中 1=到位 2=超时
 * @note   位置 P + 制动包络 + 斜率限制；到位判据 = 死区内且角速度低于门限并保持。
 */
static uint8_t Vt_TurnTick(float fwd, float tgtC, uint32_t now, float dt)
{
    float err  = tgtC - vtYawCont;
    float aerr = (err < 0.0f) ? -err : err;
    float ar   = (vtRateF < 0.0f) ? -vtRateF : vtRateF;
    float wz;

    if ((now - vtStateTick) >= (uint32_t)VT_TURN_TIMEOUT_MS)
    {
        Vt_StopSpeed();
        return 2U;
    }

    if ((aerr <= VT_TURN_TOL_DEG) && (ar <= VT_TURN_ARRIVE_DPS))
    {
        if (vtOkTick == 0U)
        {
            vtOkTick = now;
        }
        if ((now - vtOkTick) >= (uint32_t)VT_TURN_HOLD_MS)
        {
            return 1U;
        }
    }
    else
    {
        vtOkTick = 0U;
    }

    /* 位置 P -> 参考角速度（带制动包络） -> 可选角速度环 */
    {
        float ref = vtTrKp * aerr;
        float env = sqrtf(2.0f * vtTrAcc * aerr);

        if (ref > env)             { ref = env; }
        if (ref > vtTrMax)         { ref = vtTrMax; }
        if ((aerr > VT_TURN_TOL_DEG) && (ref < VT_TURN_MIN_DPS))
        {
            ref = VT_TURN_MIN_DPS;
        }
        if (err < 0.0f) { ref = -ref; }

        wz = ref + (vtTrKr * (ref - vtRateF));
    }
    if (wz > vtTrMax)       { wz =  vtTrMax; }
    else if (wz < -vtTrMax) { wz = -vtTrMax; }

    /* 斜率限制：渐入/渐出，不猛甩 */
    {
        float step = VT_WZ_SLEW_DPS2 * dt;

        if (step < 1.0f) { step = 1.0f; }

        if (wz > vtWzCmd)
        {
            vtWzCmd += step;
            if (vtWzCmd > wz) { vtWzCmd = wz; }
        }
        else if (wz < vtWzCmd)
        {
            vtWzCmd -= step;
            if (vtWzCmd < wz) { vtWzCmd = wz; }
        }
        else
        {
            /* 相等：不动 */
        }
    }

    Vt_Send(fwd, vtWzCmd);
    return 0U;
}

/* ===================== 公开接口 ===================== */

void VelTest_Enter(void)
{
    uint8_t addr;

    for (addr = (uint8_t)MOTOR_ADDR_MIN; addr <= (uint8_t)MOTOR_ADDR_MAX; addr++)
    {
        Motor_Enable(addr, 1U);        /* 抱轴：进页不跑，但车不会被推动 */
    }

    vtActive    = 1U;
    vtState     = VT_IDLE;
    vtSel       = 0U;
    vtSpeed     = 0.0f;

    Display_SetPage(PAGE_VELTEST);

    Debug_Log("VT", "enter (pure gyro + time). K1/K2=sel K3=+ K4=- K5=run");
}

void VelTest_Exit(void)
{
    Vt_StopSpeed();
    vtState  = VT_IDLE;

    {
        uint8_t addr;

        for (addr = (uint8_t)MOTOR_ADDR_MIN; addr <= (uint8_t)MOTOR_ADDR_MAX; addr++)
        {
            Motor_Disable(addr);
        }
    }

    vtActive = 0U;
    Debug_Log("VT", "exit");
    Display_MenuEnter();
}

uint8_t VelTest_IsActive(void)
{
    return vtActive;
}

void VelTest_Update(void)
{
    uint32_t now;
    float    dt;
    float    yaw;

    if (vtActive == 0U)
    {
        return;
    }

    now = HAL_GetTick();

    /* ★【2026-10-24 关键修】运行中**不再周期性重绘 OLED**：
       整页刷新一次要 ~100ms（I2C 阻塞），会把 20ms 的控制环拖成 ~100ms，
       于是速度斜坡/转向斜率全变成"大步跳"，现场就是"衔接一顿一顿、不丝滑"。
       状态切换时（进弯、出弯、结束）已经各自调了 Display_VelTestRefresh，
       实时数据看串口 `[VT] T ...` 即可。 */

    if (Vt_Running() == 0U)
    {
        /* ★停车后补发 0 速度：0xF6 无应答，丢一帧就会有轮子继续转（车打圈）。
           这里再连续发 ~0.5s，确保四个轮子都收到"停"。 */
        if ((vtStopHold != 0U) && ((now - vtTick) >= (uint32_t)VT_TICK_MS))
        {
            vtTick = now;
            Mecanum_VelBodyRpm(0.0f, 0.0f, 0.0f, vtAccLvl);
            vtStopHold--;
        }
        return;
    }

    if ((now - vtTick) < (uint32_t)VT_TICK_MS)
    {
        return;
    }

    dt     = (float)(now - vtTick) / 1000.0f;
    vtTick = now;

    if (dt <= 0.0f)
    {
        return;
    }

    yaw = IMU_GetYaw();

    /* 实测角速度（低通，给航向保持和转向阻尼共用）
       ★【2026-10-24 修蛇形】系数由 0.6/0.4 加重到 0.8/0.2：
         IMU 角度量化到 0.1°，20ms 采样下原始角速度噪声可达 ±5°/s，
         乘 KD 后直接变成可见的左右摆动。加重滤波后噪声小一个量级。 */
    {
        float dts  = (float)(now - vtCtrlTick) / 1000.0f;
        float dyaw = Vt_Wrap180(yaw - vtYawRaw);

        if (dts < 0.001f) { dts = 0.001f; }

        vtYawCont += dyaw;          /* ★连续航向累加（跨 ±180 不跳变） */
        vtYawRaw   = yaw;
        vtRateF    = (0.8f * vtRateF) + (0.2f * (dyaw / dts));
        vtCtrlTick = now;
    }

    switch (vtState)
    {
        /* ---- 第一段：加速到 V 并保持，到 T1 时刻进入过弯 ---- */
        case VT_RUN1:
        {
            vtSpeed += vtA * dt;
            if (vtSpeed > vtV) { vtSpeed = vtV; }

            Vt_Send(vtSpeed, Vt_YawHold(yaw));

            if ((now - vtLegTick) >= vtT1)
            {
                /* ★目标用**连续航向**算：180° 掉头时若用 wrap 后的角度，
                   一过 ±180 会判成 -179°，控制器会反向狂打。 */
                vtTurnTgtC      = vtYawCont + (float)vtG;
                vtCornerTurning = 0U;                 /* 先降速，再转 */
                vtOkTick        = 0U;
                vtState         = VT_CORNER;
                vtStateTick     = now;
                Debug_Log("VT", "RUN1 -> CORNER t=%u v=%d yaw=%d vc=%d g=%d",
                          (unsigned int)(now - vtLegTick), (int)vtSpeed,
                          (int)(yaw * 10.0f), (int)vtVC, (int)vtG);
            }
            break;
        }

        /* ---- 过弯：先降到过弯速度 VC，再**边低速前进边转 G 度**（不停车） ----
           ★【2026-10-24 用户要求"直行→转弯衔接要丝滑"】
             ① 不用"收到 VC 再转"，而是**离 VC 还差 120mm/s 就开始带转**，
                转向与剩余降速同时进行（衔接不断档）；
             ② 转向指令加**斜率限制**（VT_WZ_SLEW_DPS2），0→满值约 0.4 s，
                不会再从"直线"一下子猛甩进弯。 */
        case VT_CORNER:
        {
            /* 速度：先按 TRD 收到 VC；转起来以后保持 VC */
            if (vtSpeed > (vtVC + 1.0f))
            {
                vtSpeed -= vtTrDec * dt;
                if (vtSpeed < vtVC) { vtSpeed = vtVC; }
            }

            /* 进入"带转"的门槛：有进弯速度时提前 120mm/s 带转（衔接不断档）；
               ★C=0（原地转）时**不提前**，必须停下来再转，否则会边挪边转。 */
            {
                float blend = (vtVC >= 50.0f) ? VT_CORNER_BLEND_MMPS : 0.0f;

                if ((vtCornerTurning == 0U) &&
                    (vtSpeed <= (vtVC + blend + 1.0f)))
                {
                    vtCornerTurning = 1U;
                    vtWzCmd         = 0.0f;
                    vtRateF         = 0.0f;
                    vtCtrlTick      = now;
                    Debug_Log("VT", "CORNER turn start v=%d tgt=%d yaw=%d",
                              (int)vtSpeed, (int)(vtTurnTgtC * 10.0f),
                              (int)(yaw * 10.0f));
                }
            }

            if (vtCornerTurning == 0U)
            {
                Vt_Send(vtSpeed, Vt_YawHold(yaw));   /* 还在降速：保持直线 */
                break;
            }

            /* 已在过弯：低速前进 + 航向闭环（误差用**连续航向**，180° 掉头不会反向） */
            {
                uint8_t r = Vt_TurnTick(vtSpeed, vtTurnTgtC, now, dt);

                if (r == 2U)
                {
                    vtState = VT_ERR;
                    Debug_Log("VT", "CORNER timeout err=%d",
                              (int)((vtTurnTgtC - vtYawCont) * 10.0f));
                    break;
                }

                if (r == 1U)
                {
                    float err = vtTurnTgtC - vtYawCont;

                    /* 不停车，直接接第二段直行。
                       ★【2026-10-24 修蛇形】基准取**过弯目标航向**（不是当前实测角）：
                         出弯时车身往往还带着几度/秒的残余角速度，
                         若把基准设成"当前角"，误差立刻变 0、只剩很弱的阻尼项，
                         车就会继续偏、再被拉回来 —— 正是蛇形的起点。 */
                    vtYawLeg        = Vt_Wrap180(yaw + err);   /* 目标航向（wrap 形式） */
                    vtRateF         = 0.0f;
                    vtCtrlTick      = now;
                    vtLegTick       = now;
                    vtCornerTurning = 0U;
                    vtWzCmd         = 0.0f;

                    /* ★T2=0：转完就结束（原地转 180° 这种测试用） */
                    if (vtT2 == 0U)
                    {
                        Vt_StopSpeed();
                        vtState     = VT_DONE;
                        vtStateTick = now;
                        Debug_Log("VT", "CORNER ok err=%d yaw=%d -> DONE (T2=0)",
                                  (int)(err * 10.0f), (int)(yaw * 10.0f));
                        Display_VelTestRefresh();
                        break;
                    }

                    vtState     = VT_RUN2;
                    vtStateTick = now;
                    Debug_Log("VT", "CORNER ok err=%d yaw=%d -> RUN2",
                              (int)(err * 10.0f), (int)(yaw * 10.0f));
                }
            }
            break;
        }

        /* ---- 第二段：从过弯速度加速到 V 并保持，到 T2 后减速停 ---- */
        case VT_RUN2:
            if (vtSpeed < vtVC) { vtSpeed = vtVC; }

            vtSpeed += vtA * dt;
            if (vtSpeed > vtV) { vtSpeed = vtV; }

            Vt_Send(vtSpeed, Vt_YawHold(yaw));

            if ((now - vtLegTick) >= vtT2)
            {
                vtState     = VT_DEC2;
                vtStateTick = now;
                Debug_Log("VT", "RUN2 -> DEC2 t=%u v=%d yaw=%d",
                          (unsigned int)(now - vtLegTick), (int)vtSpeed,
                          (int)(yaw * 10.0f));
            }
            break;

        /* ---- 最后一段减速停（不做航向修正） ---- */
        case VT_DEC2:
            vtSpeed -= vtA * dt;

            if (vtSpeed <= 0.0f)
            {
                Vt_StopSpeed();
                vtState     = VT_DONE;
                vtStateTick = now;
                Debug_Log("VT", "DONE total %u ms yaw=%d",
                          (unsigned int)(now - vtRunTick), (int)(yaw * 10.0f));
                Display_VelTestRefresh();   /* 跑完才刷一次（90ms 阻塞不影响控制） */
            }
            else
            {
                /* ★刹车段也做航向保持（限幅加大）：高速急刹时麦轮会自己甩头，
                   原来 wz=0 会把车甩出 100°+。 */
                Vt_Send(vtSpeed, Vt_YawHoldLim(yaw, VT_YAW_BRAKE_DPS));
            }
            break;

        /* ================= ★正方形路线（方案A：正交 + 里程计距离闭环） =================
           路线表固定为 4 个角：MOVE(L) -> TURN(+90) × 4。
           直行段**按里程计走的距离**判到点（不是时间），转向段用原地转。 */
        case VT_RT_MOVE:
        {
            float ox = Chassis_GetOdomX();
            float oy = Chassis_GetOdomY();
            float dx = ox - vtRtPrevX;
            float dy = oy - vtRtPrevY;
            float remain;
            float decel;

            vtRtPrevX   = ox;
            vtRtPrevY   = oy;
            /* 距离：按每拍位移长度累加（简单、单调、不会因航向约定算错方向）。
               ⚠ 高速时会把轻微横向摆动也算进去，实际会比设定值多 2~5%，
                 需要精确时把 L 按实测比例调小即可。 */
            vtRtTravel += sqrtf((dx * dx) + (dy * dy));

            remain = vtLeg - vtRtTravel;
            decel  = (vtSpeed * vtSpeed) / (2.0f * ((vtA > 1.0f) ? vtA : 1.0f));

            /* 安全兜底：里程计读不到（chsOdomReady=0）时 remain 永远不减少，
               这里 20s 直接判错停车，避免一直往前开。 */
            if ((now - vtLegTick) > 20000U)
            {
                Vt_StopSpeed();
                vtState = VT_ERR;
                Debug_Log("VT", "RT move timeout (odom d=%d/%d)",
                          (int)vtRtTravel, (int)vtLeg);
                break;
            }

            if (remain <= (decel + 10.0f))
            {
                vtSpeed -= vtA * dt;               /* 到点前减速 */
                if (vtSpeed < 0.0f) { vtSpeed = 0.0f; }
            }
            else
            {
                vtSpeed += vtA * dt;               /* 加速到巡航 */
                if (vtSpeed > vtV) { vtSpeed = vtV; }
            }

            /* 到点：容差内，或已经停住且只剩 <30mm（防止最后几毫米爬不动） */
            if ((remain <= VT_RT_TOL_MM) ||
                ((vtSpeed <= 0.0f) && (remain <= 30.0f)))
            {
                Vt_StopSpeed();
                Debug_Log("VT", "RT move %u done d=%d (tgt %d)",
                          (unsigned int)vtRtIdx, (int)vtRtTravel, (int)vtLeg);

                vtRtIdx++;
                if (vtRtIdx >= (uint8_t)VT_RT_STEPS)
                {
                    vtState     = VT_DONE;
                    vtStateTick = now;
                    Debug_Log("VT", "RT DONE (square) %u ms",
                              (unsigned int)(now - vtRunTick));
                    Display_VelTestRefresh();
                    break;
                }

                /* 准备转向 */
                vtRtTgtC    = vtYawCont + VT_RT_ANGLE;   /* 左转 90° */
                vtWzCmd     = 0.0f;
                vtRateF     = 0.0f;
                vtCtrlTick  = now;
                vtOkTick    = 0U;
                vtState     = VT_RT_TURN;
                vtStateTick = now;
                Debug_Log("VT", "RT turn %u tgt=%d yaw=%d",
                          (unsigned int)vtRtIdx, (int)(vtRtTgtC * 10.0f),
                          (int)(yaw * 10.0f));
                break;
            }

            Vt_Send(vtSpeed, Vt_YawHold(yaw));
            break;
        }

        case VT_RT_TURN:
        {
            uint8_t r = Vt_TurnTick(0.0f, vtRtTgtC, now, dt);   /* 原地转 */

            if (r == 2U)
            {
                vtState = VT_ERR;
                Debug_Log("VT", "RT turn timeout err=%d",
                          (int)((vtRtTgtC - vtYawCont) * 10.0f));
                break;
            }

            if (r == 1U)
            {
                float err = vtRtTgtC - vtYawCont;

                vtYawLeg       = Vt_Wrap180(yaw + err);
                vtRateF        = 0.0f;
                vtCornerTurning = 0U;
                vtWzCmd        = 0.0f;

                vtRtIdx++;
                if (vtRtIdx >= (uint8_t)VT_RT_STEPS)
                {
                    vtState     = VT_DONE;
                    vtStateTick = now;
                    Debug_Log("VT", "RT DONE (square) %u ms yaw=%d",
                              (unsigned int)(now - vtRunTick), (int)(yaw * 10.0f));
                    Display_VelTestRefresh();
                    break;
                }

                /* 下一段直行：重新计里程 */
                vtRtTravel = 0.0f;
                vtRtPrevX  = Chassis_GetOdomX();
                vtRtPrevY  = Chassis_GetOdomY();
                vtSpeed    = 0.0f;
                vtLegTick  = now;
                vtState    = VT_RT_MOVE;
                vtStateTick = now;
                Debug_Log("VT", "RT turn done err=%d yaw=%d -> move %u",
                          (int)(err * 10.0f), (int)(yaw * 10.0f),
                          (unsigned int)vtRtIdx);
            }
            break;
        }

        default:
            break;
    }

    /* ★串口实时数据（调 PID 用）：
       t=启动后 ms st=状态 y=航向0.1° r=滤波角速度0.1dps g=IMU角速度0.1dps
       e=航向误差0.1° w=下发角速度0.1dps v=速度mm/s */
    if ((vtTeleMs != 0U) && ((now - vtTeleTick) >= (uint32_t)vtTeleMs))
    {
        int32_t e10;
        float   gz  = IMU_GetGyroZ();

        if (vtState == VT_CORNER)
        {
            e10 = (int32_t)((vtTurnTgtC - vtYawCont) * 10.0f);   /* 连续量，180° 也不会翻号 */
        }
        else
        {
            e10 = (int32_t)(Vt_Wrap180(vtYawLeg - yaw) * 10.0f);
        }

        vtTeleTick = now;
        Debug_Log("VT", "T t=%u st=%u y=%d r=%d g=%d e=%d w=%d v=%d",
                  (unsigned int)(now - vtRunTick), (unsigned int)vtState,
                  (int)(yaw * 10.0f), (int)(vtRateF * 10.0f), (int)(gz * 10.0f),
                  (int)e10, (int)(vtWzLast * 10.0f), (int)vtSpeed);
    }
}

static void Vt_Start(void)
{
    uint8_t addr;

    if (Vt_Running() != 0U)
    {
        return;
    }

    if (IMU_IsOnline() == 0U)
    {
        vtState = VT_ERR;
        Debug_Log("VT", "start refused: IMU offline");
        Display_VelTestRefresh();
        return;
    }

    for (addr = (uint8_t)MOTOR_ADDR_MIN; addr <= (uint8_t)MOTOR_ADDR_MAX; addr++)
    {
        Motor_Enable(addr, 1U);
    }

    vtSpeed     = 0.0f;
    vtYawLeg    = IMU_GetYaw();
    vtYawRaw    = vtYawLeg;
    vtYawCont   = 0.0f;          /* 连续航向从 0 起算 */
    vtRateF     = 0.0f;
    vtTick      = HAL_GetTick();
    vtCtrlTick  = vtTick;
    vtLegTick   = vtTick;
    vtRunTick   = vtTick;
    vtStateTick = vtTick;
    vtOkTick    = 0U;
    vtCornerTurning = 0U;
    vtWzCmd     = 0.0f;

    if (vtSQ != 0U)
    {
        /* ★正方形路线：里程计从这里清零，之后每段按"走了多少 mm"判到点 */
        Chassis_OdomReset();

        vtRtIdx    = 0U;
        vtRtTravel = 0.0f;
        vtRtPrevX  = Chassis_GetOdomX();
        vtRtPrevY  = Chassis_GetOdomY();
        vtState    = VT_RT_MOVE;

        Debug_Log("VT", "start SQUARE leg=%d V=%d A=%d yaw=%d",
                  (int)vtLeg, (int)vtV, (int)vtA, (int)(vtYawLeg * 10.0f));
    }
    else
    {
        vtState = VT_RUN1;

        Debug_Log("VT", "start V=%d T1=%u T2=%u A=%d G=%d VC=%d yaw=%d",
                  (int)vtV, (unsigned int)vtT1, (unsigned int)vtT2,
                  (int)vtA, (int)vtG, (int)vtVC, (int)(vtYawLeg * 10.0f));
    }

    Display_VelTestRefresh();
}

static void Vt_Stop(const char *why)
{
    if (Vt_Running() != 0U)
    {
        Vt_StopSpeed();
        Debug_Log("VT", "stop (%s) at %s", why, Vt_StateName());
    }

    vtState = VT_IDLE;
    Display_VelTestRefresh();
}

/* ===================== 串口调参接口 ===================== */

uint8_t VelTest_CmdSet(uint8_t idx, int32_t v)
{
    switch ((VelCmdParam_t)idx)
    {
        case VT_CMD_V:
            if (v < 100)  { v = 100; }
            if (v > 1500) { v = 1500; }
            vtV = (float)v;
            if (vtVC > vtV) { vtVC = vtV; }
            break;

        case VT_CMD_T1:
            if (v < 200)  { v = 200; }
            if (v > 9999) { v = 9999; }
            vtT1 = (uint32_t)v;
            break;

        case VT_CMD_T2:
            if (v < 0)    { v = 0; }        /* 0 = 转完就停 */
            if (v > 9999) { v = 9999; }
            vtT2 = (uint32_t)v;
            break;

        case VT_CMD_A:
            if (v < 100)  { v = 100; }
            if (v > 1000) { v = 1000; }
            vtA = (float)v;
            break;

        case VT_CMD_G:
            if (v < -180) { v = -180; }
            if (v >  180) { v = 180; }
            vtG = (float)v;
            break;

        case VT_CMD_VC:
            if (v < 0)   { v = 0; }
            if (v > 600) { v = 600; }
            vtVC = (float)v;
            if (vtVC > vtV) { vtVC = vtV; }
            break;

        case VT_CMD_KP:     /* ×10 */
            if (v < 2)  { v = 2; }
            if (v > 20) { v = 20; }
            vtKP = (float)v / 10.0f;
            break;

        case VT_CMD_KD:
            if (v < 0)  { v = 0; }
            if (v > 30) { v = 30; }
            vtKD = (float)v / 10.0f;
            break;

        case VT_CMD_TRK:    /* 过弯位置环 KP ×10 */
            if (v < 1)  { v = 1; }
            if (v > 60) { v = 60; }
            vtTrKp = (float)v / 10.0f;
            break;

        case VT_CMD_TRR:    /* 过弯角速度环 KP ×10 */
            if (v < 0)  { v = 0; }
            if (v > 30) { v = 30; }
            vtTrKr = (float)v / 10.0f;
            break;

        case VT_CMD_TRM:    /* 过弯角速度上限 dps */
            if (v < 10)  { v = 10; }
            if (v > 220) { v = 220; }
            vtTrMax = (float)v;
            break;

        case VT_CMD_TRA:    /* 制动包络 dps^2 / 10 */
            if (v < 5)   { v = 5; }
            if (v > 200) { v = 200; }
            vtTrAcc = (float)v * 10.0f;
            break;

        case VT_CMD_TRD:    /* 过弯前减速率 mm/s^2 / 10 */
            if (v < 20)  { v = 20; }
            if (v > 200) { v = 200; }
            vtTrDec = (float)v * 10.0f;
            break;

        case VT_CMD_SQ:     /* 0=单段 1=正方形 */
            vtSQ = (v != 0) ? 1U : 0U;
            break;

        case VT_CMD_LEG:    /* 正方形边长 mm */
            if (v < 300)  { v = 300; }
            if (v > 3000) { v = 3000; }
            vtLeg = (float)v;
            break;

        default:
            return 0U;
    }

    Display_VelTestRefresh();
    return 1U;
}

uint8_t VelTest_CmdStart(void)
{
    if (vtActive == 0U)
    {
        VelTest_Enter();
    }

    Vt_Start();
    return (Vt_Running() != 0U) ? 1U : 0U;
}

void VelTest_CmdStop(void)
{
    Vt_Stop("cmd");
}

void VelTest_CmdExit(void)
{
    if (vtActive != 0U)
    {
        VelTest_Exit();
    }
}

void VelTest_CmdDump(void)
{
    Debug_Log("VT", "cfg V=%d T1=%u T2=%u A=%d G=%d C=%d K=%d D=%d AC=%u "
                    "TRK=%d TRR=%d TRM=%d TRA=%d TRD=%d SQ=%u LEG=%d tele=%u st=%u",
              (int)vtV, (unsigned int)vtT1, (unsigned int)vtT2, (int)vtA,
              (int)vtG, (int)vtVC, (int)((vtKP * 10.0f) + 0.5f),
              (int)((vtKD * 10.0f) + 0.5f), (unsigned int)vtAccLvl,
              (int)((vtTrKp * 10.0f) + 0.5f), (int)((vtTrKr * 10.0f) + 0.5f),
              (int)vtTrMax, (int)(vtTrAcc / 10.0f), (int)(vtTrDec / 10.0f),
              (unsigned int)vtSQ, (int)vtLeg,
              (unsigned int)vtTeleMs, (unsigned int)vtState);
}

void VelTest_SetTeleMs(uint16_t ms)
{
    vtTeleMs   = (ms > 1000U) ? 1000U : ms;
    vtTeleTick = 0U;
}

void VelTest_CmdSetAcc(uint8_t acc)
{
    vtAccLvl = acc;
}

uint16_t VelTest_GetTeleMs(void)
{
    return vtTeleMs;
}

void VelTest_HandleKey(KeyID_t key)
{
    if ((vtActive == 0U) || (key == KEY_NONE))
    {
        return;
    }

    switch (key)
    {
        case KEY1:      /* 上一个参数（运行中不许改） */
        case KEY2:
            if (Vt_Running() != 0U)
            {
                break;
            }
            if (key == KEY1)
            {
                vtSel = (vtSel == 0U) ? (uint8_t)(VT_P_COUNT - 1U)
                                      : (uint8_t)(vtSel - 1U);
            }
            else
            {
                vtSel = (uint8_t)((vtSel + 1U) % (uint8_t)VT_P_COUNT);
            }
            break;

        case KEY3:      /* 选中参数 +1 档 */
        case KEY4:
        {
            int32_t dir = (key == KEY3) ? 1 : -1;

            if (Vt_Running() != 0U)
            {
                break;
            }

            switch ((VelParam_t)vtSel)
            {
                case VT_P_V:
                    vtV += (float)(50 * dir);
                    if (vtV < 100.0f)  { vtV = 100.0f; }
                    if (vtV > 1500.0f) { vtV = 1500.0f; }
                    break;

                case VT_P_T1:
                case VT_P_T2:
                {
                    int32_t t = (int32_t)((vtSel == VT_P_T1) ? vtT1 : vtT2) + (100 * dir);

                    /* T1 最短 200ms；T2 允许 0（= 转完就停） */
                    if ((vtSel == VT_P_T1) && (t < 200)) { t = 200; }
                    if ((vtSel == VT_P_T2) && (t < 0))   { t = 0; }
                    if (t > 9999) { t = 9999; }

                    if (vtSel == VT_P_T1) { vtT1 = (uint32_t)t; }
                    else                  { vtT2 = (uint32_t)t; }
                    break;
                }

                case VT_P_A:
                    vtA += (float)(50 * dir);
                    if (vtA < 100.0f)  { vtA = 100.0f; }
                    if (vtA > 1000.0f) { vtA = 1000.0f; }
                    break;

                case VT_P_G:
                    vtG += (5 * dir);
                    if (vtG < -180) { vtG = -180; }
                    if (vtG >  180) { vtG = 180; }
                    break;

                case VT_P_VC:   /* 过弯速度（0=原地慢转） */
                    vtVC += (float)(50 * dir);
                    if (vtVC < 0.0f)   { vtVC = 0.0f; }
                    if (vtVC > 600.0f) { vtVC = 600.0f; }
                    if (vtVC > vtV)    { vtVC = vtV; }   /* 不能比巡航还快 */
                    break;

                case VT_P_KP:   /* 航向保持 KP（0.1 一档） */
                    vtKP += (0.1f * (float)dir);
                    if (vtKP < 0.2f) { vtKP = 0.2f; }
                    if (vtKP > 2.0f) { vtKP = 2.0f; }
                    break;

                case VT_P_KD:   /* 航向保持 KD（0.1 一档） */
                    vtKD += (0.1f * (float)dir);
                    if (vtKD < 0.0f) { vtKD = 0.0f; }
                    if (vtKD > 3.0f) { vtKD = 3.0f; }
                    break;

                case VT_P_SQ:   /* 单段/正方形 切换 */
                    vtSQ = (vtSQ != 0U) ? 0U : 1U;
                    break;

                default:        /* VT_P_LEG：正方形边长（100 一档） */
                    vtLeg += (float)(100 * dir);
                    if (vtLeg < 300.0f)  { vtLeg = 300.0f; }
                    if (vtLeg > 3000.0f) { vtLeg = 3000.0f; }
                    break;
            }

            Debug_Log("VT", "param %u -> V=%d T1=%u T2=%u A=%d G=%d VC=%d K=%d D=%d SQ=%u L=%d",
                      (unsigned int)vtSel, (int)vtV, (unsigned int)vtT1,
                      (unsigned int)vtT2, (int)vtA, (int)vtG, (int)vtVC,
                      (int)(vtKP * 10.0f), (int)(vtKD * 10.0f),
                      (unsigned int)vtSQ, (int)vtLeg);
            break;
        }

        case KEY5:      /* 启动 / 运行中停止 */
            if (Vt_Running() != 0U)
            {
                Vt_Stop("key5");
            }
            else
            {
                Vt_Start();
            }
            break;

        default:
            break;
    }

    Display_VelTestRefresh();
}

void VelTest_HandleLongKey(KeyID_t key)
{
    if ((vtActive == 0U) || (key == KEY_NONE))
    {
        return;
    }

    /* 长按 KEY4 / KEY5：停止并退出（本页 KEY4 短按是"减一档"，所以退出放长按） */
    if ((key == KEY4) || (key == KEY5))
    {
        VelTest_Exit();
    }
}

/* 参数窗口起始下标：保证选中项落在 3 行 × 2 个参数的可见范围内 */
static uint8_t Vt_Window(void)
{
    int8_t w;
    int8_t maxW = (int8_t)VT_P_COUNT - 6;

    if (maxW < 0) { maxW = 0; }

    w = ((int8_t)vtSel >= 6) ? (int8_t)((int8_t)vtSel - 5) : 0;
    if (w > maxW) { w = maxW; }
    return (uint8_t)w;
}

/* 单个参数的显示文本（前面带 '>' 表示当前选中） */
static void Vt_ParamStr(uint8_t idx, char *buf, size_t len)
{
    char m = (vtSel == idx) ? '>' : ' ';

    switch ((VelParam_t)idx)
    {
        case VT_P_V:  (void)snprintf(buf, len, "%cV=%d",  m, (int)vtV);  break;
        case VT_P_T1: (void)snprintf(buf, len, "%cT1=%u", m, (unsigned int)vtT1); break;
        case VT_P_T2: (void)snprintf(buf, len, "%cT2=%u", m, (unsigned int)vtT2); break;
        case VT_P_A:  (void)snprintf(buf, len, "%cA=%d",  m, (int)vtA);  break;
        case VT_P_G:  (void)snprintf(buf, len, "%cG=%d",  m, (int)vtG);  break;
        case VT_P_VC: (void)snprintf(buf, len, "%cC=%d",  m, (int)vtVC); break;
        case VT_P_KP: (void)snprintf(buf, len, "%cK=%d",  m, (int)((vtKP * 10.0f) + 0.5f)); break;
        case VT_P_KD: (void)snprintf(buf, len, "%cD=%d",  m, (int)((vtKD * 10.0f) + 0.5f)); break;
        case VT_P_SQ: (void)snprintf(buf, len, "%cSQ=%u", m, (unsigned int)vtSQ); break;
        default:      (void)snprintf(buf, len, "%cL=%d",  m, (int)vtLeg); break;
    }
}

const char *VelTest_GetLine(uint8_t idx)
{
    char a[12];
    char b[12];
    uint8_t w;

    if (idx > 3U) { idx = 3U; }

    w = Vt_Window();

    switch (idx)
    {
        case 0:
            (void)snprintf(vtLine[0], sizeof(vtLine[0]), "VELTEST %s", Vt_StateName());
            break;

        case 1:
        case 2:
        {
            uint8_t i0 = (uint8_t)(w + ((idx == 1U) ? 0U : 2U));

            Vt_ParamStr(i0, a, sizeof(a));
            Vt_ParamStr((uint8_t)(i0 + 1U), b, sizeof(b));
            (void)snprintf(vtLine[idx], sizeof(vtLine[idx]), "%s %s", a, b);
            break;
        }

        default:
            if (Vt_Running() != 0U)
            {
                int32_t y10 = (int32_t)(IMU_GetYaw() * 10.0f);
                int32_t yi  = y10 / 10;
                int32_t yf  = y10 % 10;

                if (yf < 0) { yf = -yf; }
                (void)snprintf(vtLine[3], sizeof(vtLine[3]), "Y=%ld.%ld K=%d",
                               (long)yi, (long)yf, (int)((vtKP * 10.0f) + 0.5f));
            }
            else
            {
                Vt_ParamStr((uint8_t)(w + 4U), a, sizeof(a));
                Vt_ParamStr((uint8_t)(w + 5U), b, sizeof(b));
                (void)snprintf(vtLine[3], sizeof(vtLine[3]), "%s %s", a, b);
            }
            break;
    }

    return vtLine[idx];
}
