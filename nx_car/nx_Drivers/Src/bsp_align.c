/**
 ******************************************************************************
 * @file    bsp_align.c
 * @brief   视觉闭环定位实现（P9）：把视觉像素偏移连续收敛到 (0,0)
 *          （转盘区数据源 $RCP、打靶区数据源 $SHOT，见 Align_Start/Align_StartShot）
 * @author  nx_car
 * @date    2026-09-28
 ******************************************************************************
 * @note   状态机（全部非阻塞，每拍只走一小步）：
 *           AL_RUN    速度段：每个节拍按**最新**偏移下发四轮**速度**，
 *                     **X/Y 分轴 + 像素→速度连续映射**（越近越慢，上下限之间线性插值）；
 *                     进死区才停（2026-10-09：原来的位置模式精调段已删除）
 *           AL_SETTLE 软停后静置 ALIGN_SETTLE_MS，等车真停稳、视觉也刷到新一帧
 *           AL_READ   判：已进死区 -> DONE；否则回 AL_RUN 继续追
 *           AL_DONE / AL_FAILED
 *
 *         【为什么全程用速度模式】
 *           位置模式：精度高，但每个动作"加速-减速-停"自带停顿，全程用它就是一顿一顿；
 *           速度模式：可以连续运转（丝滑），配合"1RPM 慢档 + 20ms 短节拍"每拍只走 0.31mm，
 *                     足以稳稳收进像素死区，不需要再切位置模式。
 *
 *         【三条容易搞错的点】
 *         ① **绝不能盲走**：数据无效/过期时必须先软停再等新帧，
 *            否则会按旧偏移一直开下去（跑出视野也停不下来）。
 *         ② **软停后必须等新帧**：移动结束后 offset 还是移动前的旧值，直接采信会越纠越偏，
 *            所以要求 `lastRxTick` 晚于"本阶段起始时刻"。
 *         ③ **判据用像素**：`RPi_IsOffsetAligned()` 比的是 px 死区，与 mm 系数无关；
 *            mm 系数只影响"速度方向/大小"，不影响"是否已经对准"。
 *
 *         硬约束：禁止 HAL_Delay；日志禁 %f；速度段四轮命令**背靠背**下发（中间不插日志）。
 ******************************************************************************
 */
#include "bsp_align.h"
#include "bsp_rpi.h"        /* RPi_GetRcp/GetShot（数据源）/ RPi_PxToBodyMm（唯一换算入口）/ RPi_GetCfg */
#include "bsp_chassis.h"    /* Chassis_GetOdomX/Y：状态文本显示对位中的绝对坐标 */
#include "bsp_mecanum.h"    /* Mecanum_Stop（停轮 / 急停） */
#include "bsp_motor.h"      /* MOTOR_ADDR_x / MOTOR_xx_DIR_FWD/REV（方向标定宏）/ MOTOR_NUM */
#include "bsp_Emm_V5.h"     /* Emm_V5_Vel_Control（速度模式 0xF6，静默下发用） */
#include "can.h"            /* hcan：下发前等空闲发送邮箱 */
#include "bsp_debug.h"
#include "bsp_imu.h"        /* IMU_StaticZeroTo：对位成功后一次性静态置角 */
#include <stdio.h>

/* ===================== 内部状态 ===================== */

typedef enum {
    AL_IDLE = 0,   /* 空闲 */
    AL_YAW,        /* ★车身矫正：按 $SHOT 的"平行度 q"原地把车转正（仅 $SHOT 档；在 XY 平移之前） */
    AL_RUN,        /* 速度段：X/Y 分轴 + 两段速连续平移 */
    AL_READ,       /* 软停后：等新帧 + 评估 */
    AL_SETTLE,     /* 等停稳 */
    AL_DONE,       /* 已收敛（成功） */
    AL_FAILED      /* 失败 */
} AlignState_t;

/* ★【2026-10-10 晚 用户要求】对位"**轮次**"：
 *   1 = 第 1 轮（**粗靠近**：死区放大 `ALIGN_PASS1_DB_MUL` 倍，快）；
 *   2 = 第 2 轮（**精定位**：死区回到严格值 `RPI_PX_DEADBAND`，准）。
 * 每次 `Al_Begin` 复位为 1；每轮结束（进"当前轮死区"）若还有下一轮就重跑一轮。
 * **仅 `$SHOT`**（STORE/ROUGH PLACE）走两轮；`$RCP`（TRAY PICK，有 7s 预算）保持单轮。 */
static uint8_t alPass = 1U;

/* 四轮电机地址（速度下发用） */
static const uint8_t alAddr[MOTOR_NUM] = {
    MOTOR_ADDR_FL, MOTOR_ADDR_FR, MOTOR_ADDR_RL, MOTOR_ADDR_RR
};

static uint8_t  alState     = AL_IDLE;
static uint16_t alIter      = 0U;    /* 从 AL_RUN 起步的次数（诊断用） */
static uint32_t alPhaseTick = 0U;    /* 当前阶段开始时刻 */
static int16_t  alLastPx    = 0;     /* 最近一次评估时的偏移（显示用） */
static int16_t  alLastPy    = 0;

/* ★【2026-10-15】调参用调试快照（对齐 bsp_align.h 的 Align_Dbg_t；只读，不影响控制） */
static Align_Dbg_t alDbg;

/* 时间预算（"几秒内必须完成对位"）。
   ★【2026-10-10 分档】`alBudgetReq` = 调用方通过 Align_SetBudget() **提出**的预算（**转盘档用**）；
   `alBudgetMs` = **本次对位实际生效**的预算，在 Al_Begin() 里按档位定：
     · $RCP（转盘 TRAY PICK）-> 用 `alBudgetReq`（转盘只停几秒时靠它按时收工，原行为）；
     · $SHOT（打靶 STORE/ROUGH PLACE）-> **恒为 0 = 不限时**（用户要求"必须磨到 (0,0)"）。
   两者都为 0 表示不限时。 */
static uint32_t alBudgetReq  = 0U;   /* 调用方提出的预算 ms（转盘档用） */
static uint32_t alBudgetMs   = 0U;   /* 本次对位实际生效的预算 ms（0 = 不限时） */
static uint32_t alBudgetTick = 0U;   /* 本次对位起始时刻 */

/* ★★ 本次对位的"精度档"，由 Al_Begin() 按**数据源**选定（调用方不用管）：
     · $SHOT -> 严格档（死区 1px）；· $RCP -> 常规档（死区 6px）。
   注：**死区本身**不放副本，直接写 bsp_rpi 的运行期配置 `RPi_GetCfg()->deadbandPx`
   （判定函数 Al_EffDeadband 读的就是它），避免两处不同步。
   【2026-10-10】原来的"末端放宽出口"（alHoldMs / Al_HoldDone）已**整体删除** ——
   对位只有一条成功判据：两轴都真正进死区，即"一步到位"。 */

/* 速度段状态 */
static int32_t  alRpmBase   = 0;     /* 上一拍下发的基准轮速（RPM，≥0），用于限斜率 */
static uint32_t alTick      = 0U;    /* 上一拍下发时刻（节拍计时） */
static uint32_t alRunLogTick = 0U;   /* 速度段日志限流 */

/* 卡住检测（偏差长时间不缩小 = 轮子没转/打滑） */
static uint32_t alProgTick  = 0U;    /* 上次"偏差明显变小"的时刻 */
static int16_t  alProgPx    = 0;     /* 上次记录的最小偏差幅值 */
static uint8_t  alBoost     = 0U;    /* 1=正在"踢一脚"（限时，见 ALIGN_PID_KICK_MAX_MS） */
static uint32_t alBoostTick = 0U;    /* ★本次踢脚的开始时刻（到点必须撤回，禁止锁存） */

/* 诊断计数（日志里打出来，用来判断"是不是视觉断断续续造成的走走停停"） */
static uint16_t alTickCnt   = 0U;    /* 已下发速度的拍数 */
static uint16_t alHoldCnt   = 0U;    /* 因"数据无效/过期"而软停等待的次数 */

/* "还在不在收新数据"的看门狗（默认只报警不中止，见 ALIGN_NO_DATA_FAIL_MS） */
static uint32_t alLastSeen    = 0U;  /* 上一次看到的帧时间戳（变了 = 收到新帧） */
static uint32_t alNoDataTick  = 0U;  /* 最近一次收到新帧的时刻 */
static uint8_t  alNoDataLogged = 0U; /* "在等数据"提示只打一次（收到新帧后清零） */

/* 实测视觉帧间隔：用来**自动放宽**"数据年龄"上限。
   为什么要自动放宽：如果视觉端回 $RCP / $SHOT 的周期比 ALIGN_VEL_DATA_MS 还长，
   那么每一拍都会判成"过期" -> 软停 -> 等 -> 再走 -> 又过期，
   表现出来就正是"走一段停一段"。放宽到 ≥3 个实测帧周期就不会误判。 */
static uint32_t alFrameTick   = 0U;  /* 上一帧到达时刻 */
static uint32_t alFramePeriod = 0U;  /* 实测帧间隔（ms，钳到 20~500） */

/* 上一次"主动重发请求"的时刻（对位期间每 ALIGN_REQ_POLL_MS 重发 $RCP/$SHOT）。
   【为什么必须有】协议 v2.0 是**请求/应答式**：树莓派只在收到请求时回一帧，
   不重发就永远只有第一帧；而数据 500ms 不更新即判无效，对位会卡在"等新帧"
   上完全不动（现象：车一动不动 / 走到一半停住）。 */
static uint32_t alReqTick     = 0U;

/* ===================== 内部工具 ===================== */

/**
 * @brief  取绝对值（避免引入 math.h）
 */
static int16_t Align_Abs16(int16_t v)
{
    return (v < 0) ? (int16_t)(-v) : v;
}

/* ===================== ★【2026-10-15】PID 控制器（对位三轴各一套） =====================
   ⚠ 整块只在 `ALIGN_PID_ENABLE=1`（默认）时编译；=0 时退回旧的"分轴线性查表"，
     本块的静态量/函数全部不参与编译（避免"未引用"告警）。
   为什么自己写而不用库：本工程没有浮点 PID 库；这里只需要 20 行，且要**非阻塞、无动态分配**。
   结构说明：
     · `iTerm` = ∫e·dt（单位 = 误差单位 × s），输出时乘 `ki`；限幅时**除以 ki** 得到积分项上限，
       这样"积分项输出 ≤ iMax RPM"是直观可调的；
     · 微分用"**测量变化**"算（本控制器设定值恒为 0，与"误差变化"等价），
       并加一阶低通 —— 视觉是**整数像素**、量化噪声大，裸微分会把噪声放大成抖动；
     · 积分分离：只有 |误差| ≤ `ALIGN_PID_I_SEP_MM`(旋转用 `ALIGN_YAW_DEADBAND`×若干) 才积分，
       避免大偏差阶段积分饱和 -> 过冲。
   ⚠ 本函数**只在收到新视觉帧时调用一次**（见 Align_RunPhase / Align_YawPhase）：
     同一帧重复调会把同一段误差积分/微分多次，必然振荡。 */
#if (ALIGN_PID_ENABLE != 0U)
typedef struct {
    float   kp;
    float   ki;
    float   kd;
    float   iMax;      /* 积分项**输出**上限（RPM） */
    float   iTerm;     /* ∫e·dt（误差单位 × s） */
    float   measPrev;  /* 上一次测量（做微分用） */
    float   dFilt;     /* 微分低通状态 */
    uint8_t started;   /* 0=还没建立基准（第一条测量只做基准，不算微分） */
} AlPid_t;

static float Al_AbsF(float v)
{
    return (v < 0.0f) ? -v : v;
}

/**
 * @brief  复位一个 PID（清积分/微分/基准）
 */
static void Al_PidReset(AlPid_t *p, float kp, float ki, float kd, float iMax)
{
    p->kp       = kp;
    p->ki       = ki;
    p->kd       = kd;
    p->iMax     = iMax;
    p->iTerm    = 0.0f;
    p->measPrev = 0.0f;
    p->dFilt    = 0.0f;
    p->started  = 0U;
}

/**
 * @brief  PID 一拍（**只在收到新视觉帧时调用**）
 * @param  p      : 控制器
 * @param  err    : 误差（mm 或 0.1°；带符号，目标恒为 0）
 * @param  dtSec  : 本拍时长（s）= 实测帧间隔，内部钳到 1ms~500ms
 * @param  outMax : 输出限幅（RPM）
 * @param  iSep   : 积分分离门限（|err| ≤ 本值才积分；单位同 err）
 * @retval 输出（RPM，带符号；已限幅）
 */
static float Al_PidStep(AlPid_t *p, float err, float dtSec, float outMax, float iSep)
{
    float d;
    float u;
    float iLimit;

    if (dtSec < 0.001f) { dtSec = 0.001f; }
    if (dtSec > 0.500f) { dtSec = 0.500f; }

    /* 微分：第一条测量只作基准（否则首拍会有一个巨大的假微分） */
    if (p->started == 0U)
    {
        p->measPrev = err;
        p->started  = 1U;
        d = 0.0f;
    }
    else
    {
        d = (err - p->measPrev) / dtSec;
    }
    p->measPrev = err;

    /* 微分低通（一阶）：D_FILTER = 新值权重 */
    p->dFilt = (p->dFilt * (1.0f - (float)ALIGN_PID_D_FILTER)) +
               (d * (float)ALIGN_PID_D_FILTER);

    /* 积分分离 + 积分项输出限幅（防饱和） */
    if (Al_AbsF(err) <= iSep)
    {
        p->iTerm += (err * dtSec);
    }
    iLimit = (p->ki > 0.000001f) ? (p->iMax / p->ki) : 0.0f;
    if (p->iTerm >  iLimit) { p->iTerm =  iLimit; }
    if (p->iTerm < -iLimit) { p->iTerm = -iLimit; }

    u = (p->kp * err) + (p->ki * p->iTerm) + (p->kd * p->dFilt);

    if (u >  outMax) { u =  outMax; }
    if (u < -outMax) { u = -outMax; }

    return u;
}

/* 三轴 PID 实例：F=前进、R=横移、W=旋转 */
static AlPid_t alPidF;
static AlPid_t alPidR;
static AlPid_t alPidW;

static uint32_t alPidFrame = 0U;   /* 上一次"喂给 PID"的视觉帧时间戳（同一帧不重复算） */
static uint32_t alPidTick  = 0U;   /* 上一次 PID 更新的时刻（算 dt 用） */

/* 上一拍下发的**单轮**速度（逐轮限斜率用；与 alRpmBase 的"基准速度"不同） */
static int16_t alWheelPrev[MOTOR_NUM] = { 0, 0, 0, 0 };

/* ===== ★【2026-10-15 用户要求】**三套 PID 参数分开调**（$RCP / $SHOT / $SHOT2）=====
 *   `Al_Begin()` 按 `alSrc` 选表；本文件里所有"用哪套参数"的地方都读 `alCfg`，
 *   所以三处互不影响，改 bsp_align.h 里对应的一组宏即可。
 *   ★ 表**不是 const**：`Align_SetPidCfg()` 可在运行期改（UART5 在线调参 / PC 自动调参）。 */
static Align_PidCfg_t alPidCfg[3] = {
    /* 0 = $RCP（转盘） */
    { (float)ALIGN_RCP_KP,   (float)ALIGN_RCP_KI,   (float)ALIGN_RCP_KD,
      (float)ALIGN_RCP_RPM_MAX, (int32_t)ALIGN_RCP_WHEEL_MAX,
      (float)ALIGN_RCP_YAW_KP, (float)ALIGN_RCP_YAW_KI, (float)ALIGN_RCP_YAW_KD,
      (float)ALIGN_RCP_YAW_RPM_MAX },
    /* 1 = $SHOT（打靶 STORE / ROUGH PLACE） */
    { (float)ALIGN_SHOT_KP,  (float)ALIGN_SHOT_KI,  (float)ALIGN_SHOT_KD,
      (float)ALIGN_SHOT_RPM_MAX, (int32_t)ALIGN_SHOT_WHEEL_MAX,
      (float)ALIGN_SHOT_YAW_KP, (float)ALIGN_SHOT_YAW_KI, (float)ALIGN_SHOT_YAW_KD,
      (float)ALIGN_SHOT_YAW_RPM_MAX },
    /* 2 = $SHOT2（ROUGH STACK） */
    { (float)ALIGN_SHOT2_KP, (float)ALIGN_SHOT2_KI, (float)ALIGN_SHOT2_KD,
      (float)ALIGN_SHOT2_RPM_MAX, (int32_t)ALIGN_SHOT2_WHEEL_MAX,
      (float)ALIGN_SHOT2_YAW_KP, (float)ALIGN_SHOT2_YAW_KI, (float)ALIGN_SHOT2_YAW_KD,
      (float)ALIGN_SHOT2_YAW_RPM_MAX }
};

/* 当前生效的那一套（Al_Begin 里按 alSrc 切换；默认先指向 RCP，防未初始化被读） */
static const Align_PidCfg_t *alCfg = &alPidCfg[0];

/**
 * @brief  复位三轴 PID（开新一段 / 开新一轮时调）
 */
static void Al_PidResetAll(uint32_t now)
{
    Al_PidReset(&alPidF, alCfg->kp, alCfg->ki, alCfg->kd, (float)ALIGN_PID_I_MAX);
    Al_PidReset(&alPidR, alCfg->kp, alCfg->ki, alCfg->kd, (float)ALIGN_PID_I_MAX);
    Al_PidReset(&alPidW, alCfg->ykp, alCfg->yki, alCfg->ykd, (float)ALIGN_PID_YAW_I_MAX);
    alPidFrame = 0U;
    alPidTick  = now;
}
#endif   /* ALIGN_PID_ENABLE */

/**
 * @brief  下发 CAN 命令前等一个空闲发送邮箱
 * @param  无
 * @retval 无
 * @note   STM32F1 只有 3 个发送邮箱，官方库 can_SendCmd 不检查发送结果，
 *         邮箱占满时该帧被**静默丢弃**（现象是"偶发某个轮子不动"）。
 *         一帧约 0.26ms @500kbps，正常几乎不等待，上限 MOTOR_TX_SLOT_WAIT_MS 兜底。
 *         （bsp_motor / bsp_arm 里各有一份同源的 static 版本，本文件需要自己一份。）
 */
static void Align_TxSlotWait(void)
{
    uint32_t t0 = HAL_GetTick();

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U)
    {
        while ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U) &&
               ((HAL_GetTick() - t0) < (uint32_t)MOTOR_TX_SLOT_WAIT_MS))
        {
            /* 空等极短时间，正常不会进入 */
        }
    }
}

/**
 * @brief  取某轮"正转"对应的官方库 dir 值
 * @param  addr: 1~4
 * @retval dir 值（0=CW / 1=CCW）
 * @note   与 bsp_motor 的 Motor_DirFwd 用**同一套标定宏**，所以方向标定仍然只有一处，
 *         改 bsp_motor.h 的 MOTOR_xx_DIR_FWD/REV 时本文件自动跟着变。
 */
static uint8_t Align_DirFwd(uint8_t addr)
{
    switch (addr)
    {
        case MOTOR_ADDR_FL: return (uint8_t)MOTOR_FL_DIR_FWD;
        case MOTOR_ADDR_FR: return (uint8_t)MOTOR_FR_DIR_FWD;
        case MOTOR_ADDR_RL: return (uint8_t)MOTOR_RL_DIR_FWD;
        case MOTOR_ADDR_RR: return (uint8_t)MOTOR_RR_DIR_FWD;
        default:            return (uint8_t)MOTOR_DIR_FWD_DEFAULT;
    }
}

/**
 * @brief  取某轮"反转"对应的官方库 dir 值
 */
static uint8_t Align_DirRev(uint8_t addr)
{
    switch (addr)
    {
        case MOTOR_ADDR_FL: return (uint8_t)MOTOR_FL_DIR_REV;
        case MOTOR_ADDR_FR: return (uint8_t)MOTOR_FR_DIR_REV;
        case MOTOR_ADDR_RL: return (uint8_t)MOTOR_RL_DIR_REV;
        case MOTOR_ADDR_RR: return (uint8_t)MOTOR_RR_DIR_REV;
        default:            return (uint8_t)MOTOR_DIR_REV_DEFAULT;
    }
}

/**
 * @brief  静默下发单轮速度（rpm=0 -> 该轮按加减速档软停）
 * @param  addr: 1~4
 * @param  rpm:  有符号转速（RPM），正=整车前进方向的正转侧
 * @retval 无
 * @note   为什么不用 bsp_motor 的 Motor_SetVel：
 *         ① Motor_SetVel **每轮都打一行日志**。本模块每个节拍要发 4 条、末端节拍 20ms，
 *            4 行日志 ≈ 11ms，等于把主循环卡死；而且日志夹在四条命令之间，会把第 4 轮
 *            拖后十几 ms —— 先动的轮先走，车就会歪。这里四条**背靠背连发**，
 *            日志由调用方汇总限流打一行。
 *         ② 方向标定仍走 bsp_motor.h 的宏，不存在"两套方向"的问题。
 *         语义与 Motor_SetVel 一致（方向映射、acc 档、0 速度=停）。
 */
static void Align_SendVel(uint8_t addr, int16_t rpm)
{
    uint8_t  dir;
    uint16_t speed;

    if (rpm < 0)
    {
        dir   = Align_DirRev(addr);
        speed = (uint16_t)(-rpm);
    }
    else
    {
        dir   = Align_DirFwd(addr);
        speed = (uint16_t)rpm;
    }

    Align_TxSlotWait();
    Emm_V5_Vel_Control(addr, dir, speed, (uint8_t)ALIGN_ACC, false);
}

/**
 * @brief  停四轮（速度模式发 0 速度；减速快慢由 ALIGN_ACC 决定，现为 0 = 直接停）
 * @param  无
 * @retval 无
 * @note   与 Mecanum_Stop（0xFE 瞬停）相对：正常收尾/交接用这条，紧急中止才用急停。
 *         没下发过速度（alRpmBase==0）就不发，避免四条无意义的命令。
 *         末段速度只有 NEAR 档（1RPM ≈ 5.2mm/s），即使 acc=0 直接停也很轻。
 */
static void Align_StopWheels(void)
{
    uint8_t i;

    if (alRpmBase == 0)
    {
        return;
    }

    for (i = 0U; i < (uint8_t)MOTOR_NUM; i++)
    {
        Align_SendVel(alAddr[i], 0);
#if (ALIGN_PID_ENABLE != 0U)
        alWheelPrev[i] = 0;         /* ★PID 版：停轮后逐轮限斜率从 0 重新起步 */
#endif
    }
    alRpmBase = 0;
}

#if (ALIGN_PID_ENABLE == 0U)
/**
 * @brief  速度段：按"**单轴**位移 + 指定速度"下发一次四轮速度（速度模式，连续平移）
 * @param  mmFwd:   还需向前的位移（mm，正=前进）—— **只传一个轴**，另一轴传 0
 * @param  mmRight: 还需向右的位移（mm，正=右移）—— 同上
 * @param  rotW:    ★旋转分量（**车身矫正用**）：平移段传 **0**；非 0 = 叠加原地旋转，
 *                  符号决定转向（幅值只用于过下面"是否值得动"的判据，不代表角度）。
 * @param  rpm:     本拍的四轮基准速度（RPM，>0）；由调用方按"偏差→速度"插值算出
 * @param  tickMs:  本拍节拍长度（ms），用于按节拍折算限斜率
 * @param  now:     当前时刻
 * @retval 无
 * @note   控制律（2026-10-09 二次改"**分轴 + 像素→速度连续映射**"）：
 *         速度由调用方按"像素偏差上下限 ↔ 速度上下限"线性插值算好传进来，
 *         本函数只负责**逆解分配 + 限斜率 + 四轮背靠背下发**：
 *           · 按各轮位移比例分配（等时长法：比例一致才能走直线、不走偏）；
 *           · 各轮**符号**由位移分量的符号决定，方向宏与位置模式共用同一套；
 *           · rotW = 0 -> 只平移不旋转（车身角度不歪）；≠0 -> 叠加原地旋转（车身矫正段用）。
 */
static void Align_VelMove(int16_t mmFwd, int16_t mmRight, int16_t rotW,
                          int32_t rpm, uint32_t tickMs, uint32_t now)
{
    float   axFwd   = (float)MEC_SIGN_VX * (float)mmFwd;
    float   axRight = (float)MEC_SIGN_VY * (float)mmRight;
    float   axRot   = (float)MEC_SIGN_WZ * (float)rotW;   /* ★车身矫正：旋转分量（平移段传 0） */
    float   c[MOTOR_NUM];
    float   cMax = 0.0f;
    int32_t rpmBase;
    int32_t slew;
    uint8_t i;

    /* 麦轮逆解（位移版，X 形安装）——与 bsp_mecanum 的 Mecanum_MoveBy 同一套符号：
         rotW = 0 -> 只平移不旋转（原行为，车身角度不歪）；
         rotW ≠ 0 -> 叠加原地旋转（FL/RL 一边、FR/RR 另一边），符号由整车标定 MEC_SIGN_WZ 决定。 */
    c[0] = axFwd + axRight + axRot;   /* FL 前左 */
    c[1] = axFwd - axRight - axRot;   /* FR 前右 */
    c[2] = axFwd - axRight + axRot;   /* RL 后左 */
    c[3] = axFwd + axRight - axRot;   /* RR 后右 */

    for (i = 0U; i < (uint8_t)MOTOR_NUM; i++)
    {
        float a = (c[i] < 0.0f) ? -c[i] : c[i];
        if (a > cMax) { cMax = a; }
    }

    /* 需要走的距离不足 0.5mm：不值得动（只会出现在死区附近） */
    if (cMax < 0.5f)
    {
        return;
    }

    /* ---- 速度：直接用调用方算好的值；只做上限保护（设了预算时按百分比提速）---- */
    {
        int32_t maxRpm = (alBudgetMs != 0U)
                         ? ((int32_t)ALIGN_XY_RPM_MAX * (int32_t)ALIGN_BUDGET_SPEED_PCT / 100)
                         : (int32_t)ALIGN_XY_RPM_MAX;

        rpmBase = (rpm > maxRpm) ? maxRpm : rpm;
        if (rpmBase < 1) { rpmBase = 1; }
    }

    /* ---- 软件限斜率：软起步 / 软收尾（丝滑的关键）----
       按节拍长度同比折算，末端换 20ms 短节拍时限幅也跟着变小，
       否则"短节拍 + 大限幅"等于没限。 */
#if (ALIGN_VEL_SLEW_RPM != 0U)
    slew = (int32_t)(((uint32_t)ALIGN_VEL_SLEW_RPM * tickMs) / (uint32_t)ALIGN_TICK_MS);
    if (slew < 1) { slew = 1; }

    if (rpmBase > (alRpmBase + slew))      { rpmBase = alRpmBase + slew; }
    else if (rpmBase < (alRpmBase - slew)) { rpmBase = alRpmBase - slew; }
#endif
    alRpmBase = rpmBase;

    /* ---- 四轮按位移比例分配速度（等时长法），四条**背靠背**下发 ---- */
    for (i = 0U; i < (uint8_t)MOTOR_NUM; i++)
    {
        float q   = (float)rpmBase * (c[i] / cMax);
        int32_t r = (int32_t)((q >= 0.0f) ? (q + 0.5f) : (q - 0.5f));

        /* 该轮确实要动、但四舍五入成 0 时保号给 1RPM（否则这一拍它就完全不动，
           末端会出现"有的轮在磨、有的轮不动"的扭动感） */
        if ((r == 0) && (c[i] != 0.0f))
        {
            r = (c[i] > 0.0f) ? 1 : -1;
        }

        Align_SendVel(alAddr[i], (int16_t)r);
    }

    alTickCnt++;

    /* 汇总日志（限流 500ms）：tick=拍数 hold=因数据失效软停次数 boost=抬速档 */
    if ((now - alRunLogTick) >= 500U)
    {
        alRunLogTick = now;
        Debug_Log("ALN", "run px=%d,%d rpm=%ld tick=%u hold=%u boost=%u",
                  (int)alLastPx, (int)alLastPy, (long)alRpmBase,
                  (unsigned)alTickCnt, (unsigned)alHoldCnt, (unsigned)alBoost);
    }
}
#endif   /* ALIGN_PID_ENABLE == 0 */

#if (ALIGN_PID_ENABLE != 0U)
/**
 * @brief  ★【2026-10-15】**PID 版**速度下发：按三轴**速度**直接算四轮 RPM
 * @param  rpmFwd  : 前进轴速度（RPM，带符号，PID 输出）
 * @param  rpmRight: 横移轴速度（RPM，带符号）
 * @param  rpmRot  : 旋转轴速度（RPM，带符号）
 * @param  now     : 当前时刻
 * @retval 无
 * @note   与 `Align_VelMove` 的区别：
 *           · 那边输入是"**位移**"，所以只能按 `rpmBase * (c/cMax)` **比例**分配（速度大小由偏差定）；
 *           · 这里输入已经是"**速度**"，所以**直接逆解** `c[i] = ±vF ± vR ± vW`：
 *             三轴的真实权重都保留（同时走 X+Y+转 才不会互相吞掉）；
 *           · 逆解后**逐轮限幅**（当前那套参数的 `wheelMax`）+ **逐轮限斜率**
 *             （`ALIGN_VEL_SLEW_RPM`，软起步/软收尾）—— 限斜率放在轮子上，
 *             所以三轴合成的方向不会因为"基准速度被限"而临时变形。
 *         符号标定与 `Align_VelMove` **完全一致**（同一套 `MEC_SIGN_VX/VY/WZ`），
 *         方向反了仍然只改 bsp_mecanum.h 的那三个宏。
 */
static void Align_VelMoveVel(float rpmFwd, float rpmRight, float rpmRot, uint32_t now)
{
    float    axF = (float)MEC_SIGN_VX * rpmFwd;
    float    axR = (float)MEC_SIGN_VY * rpmRight;
    float    axW = (float)MEC_SIGN_WZ * rpmRot;
    float    c[MOTOR_NUM];
    int32_t  r[MOTOR_NUM];
    int32_t  maxAbs = 0;
    uint8_t  i;

    c[0] = axF + axR + axW;   /* FL 前左 */
    c[1] = axF - axR - axW;   /* FR 前右 */
    c[2] = axF - axR + axW;   /* RL 后左 */
    c[3] = axF + axR - axW;   /* RR 后右 */

    for (i = 0U; i < (uint8_t)MOTOR_NUM; i++)
    {
        int32_t v = (int32_t)((c[i] >= 0.0f) ? (c[i] + 0.5f) : (c[i] - 0.5f));

        /* 单轮限幅（三轴叠加可能超上限） */
        if (v >  alCfg->wheelMax) { v =  alCfg->wheelMax; }
        if (v < -alCfg->wheelMax) { v = -alCfg->wheelMax; }

        /* 逐轮限斜率：每拍最多变 ALIGN_VEL_SLEW_RPM（软起步/软收尾） */
#if (ALIGN_VEL_SLEW_RPM != 0U)
        {
            int32_t dv  = v - (int32_t)alWheelPrev[i];
            int32_t lim = (int32_t)ALIGN_VEL_SLEW_RPM;

            if (dv >  lim) { v = (int32_t)alWheelPrev[i] + lim; }
            if (dv < -lim) { v = (int32_t)alWheelPrev[i] - lim; }
        }
#endif
        r[i] = v;

        {
            int32_t a = (v < 0) ? -v : v;
            if (a > maxAbs) { maxAbs = a; }
        }
    }

    /* ★调参快照：三轴 PID 输出（×100）与本拍最大单轮 RPM */
    alDbg.uF       = (int16_t)(rpmFwd   * 100.0f);
    alDbg.uR       = (int16_t)(rpmRight * 100.0f);
    alDbg.uW       = (int16_t)(rpmRot   * 100.0f);
    alDbg.wheelMax = (int16_t)maxAbs;

    /* 四轮都该是 0：软停一次就够，不必每拍刷 4 条无用命令 */
    if (maxAbs == 0)
    {
        Align_StopWheels();
        return;
    }

    /* 四条**背靠背**下发（中间不插日志：插日志会把第 4 轮拖后十几 ms，车就歪了） */
    for (i = 0U; i < (uint8_t)MOTOR_NUM; i++)
    {
        Align_SendVel(alAddr[i], (int16_t)r[i]);
        alWheelPrev[i] = (int16_t)r[i];
    }
    alRpmBase = maxAbs;
    alTickCnt++;

    /* 汇总日志（限流 500ms）：三轴 PID 输出 + 拍数 + 等待次数 */
    if ((now - alRunLogTick) >= 500U)
    {
        alRunLogTick = now;
        Debug_Log("ALN", "pid px=%d,%d u=%d,%d,%d w=%d tick=%u hold=%u",
                  (int)alLastPx, (int)alLastPy,
                  (int)rpmFwd, (int)rpmRight, (int)rpmRot,
                  (int)maxAbs, (unsigned)alTickCnt, (unsigned)alHoldCnt);
    }
}
#endif   /* ALIGN_PID_ENABLE */

/**
 * @brief  判断当前偏差是否已进"保底速度区"（两轴都在 ALIGN_XY_PX_MIN 内）
 * @note   用来把节拍切成更短的 ALIGN_TICK_FINE_MS（每拍位移更小，末端更稳）。
 */
static uint8_t Align_InSlowZone(int16_t px, int16_t py)
{
    return ((Align_Abs16(px) <= (int16_t)ALIGN_XY_PX_MIN) &&
            (Align_Abs16(py) <= (int16_t)ALIGN_XY_PX_MIN)) ? 1U : 0U;
}

/**
 * @brief  取偏移帧里更"新"的时间戳（与 alPhaseTick 比，处理溢出用差值）
 * @param  tick: 帧时间戳
 * @retval 1=比本阶段开始时刻更新
 */
static uint8_t Align_IsNewer(uint32_t tick)
{
    return ((int32_t)(tick - alPhaseTick) > 0) ? 1U : 0U;
}

/**
 * @brief  进入失败态（**只有一种原因**：连续收不到新的有效偏移）
 * @param  无
 * @retval 无
 */
static void Align_Fail(void)
{
    alState = AL_FAILED;
    Mecanum_Stop();
    Chassis_OdomHoldXY(0U);     /* ★对位失败：解冻平移坐标（对位期间是冻结的，见 Al_Begin） */

    Debug_Log("ALN", "fail: no new offset for >%u ms (vision lost / out of view) hold=%u",
              (unsigned)ALIGN_NO_DATA_FAIL_MS, (unsigned)alHoldCnt);
}

/**
 * @brief  记录"收到一帧新偏移"，并更新实测帧间隔
 * @param  now: 当前时刻
 * @retval 无
 * @note   帧间隔用来把"数据年龄"上限自动放宽（见 Align_DataLimit），
 *         避免视觉帧率比 ALIGN_VEL_DATA_MS 慢时被反复误判成过期 -> 走一段停一段。
 */
static void Align_MarkFrame(uint32_t now)
{
    uint32_t dt = now - alFrameTick;

    if ((dt >= 20U) && (dt <= 500U))
    {
        alFramePeriod = dt;
    }
    alFrameTick = now;
}

/**
 * @brief  本拍允许的最大"数据年龄"（ms）
 * @retval 固定值 ALIGN_VEL_DATA_MS 与"3 个实测帧周期"取大，上限 1000
 */
static uint32_t Align_DataLimit(void)
{
    uint32_t lim     = (uint32_t)ALIGN_VEL_DATA_MS;
    uint32_t autoLim = alFramePeriod * 3U;

    if (autoLim > lim) { lim = autoLim; }
    if (lim > 1000U)   { lim = 1000U; }
    return lim;
}

/* ===================== 对位数据源（v2.0 协议：$RCP / $SHOT） =====================
   【为什么要有这一层】v2.0 把"当前偏移"拆成两条独立的数据帧：
       · 转盘阶段（$RCP）：转盘中心偏移 —— 转盘区对位用；
       · 打靶阶段（$SHOT）：中间圆环偏移 —— 暂存区/粗加工区对位用。
   两者字段语义完全一样（dxPx/dyPx 右正/下正、valid、lastRxTick），
   所以这里把两种源**归一化成一个视图 `Al_Off_t`**，本文件其余代码只认它，
   不必到处写 `if (源 == RCP)`。每拍用 `Al_RefreshOff()` 按 `alSrc` 刷新快照。
   【选源】`Align_Start()` = $RCP（转盘）；`Align_StartShot()` = $SHOT（打靶区）。 */
typedef struct {
    int16_t  dxPx;         /* 右正 */
    int16_t  dyPx;         /* 下正 */
    int16_t  qAvg;         /* ★车身矫正用：三个圆环"平行度"的取值（见 ALIGN_YAW_Q_MODE；$RCP 恒 0） */
    uint8_t  valid;        /* 1=有效（未过期） */
    uint32_t lastRxTick;   /* 该帧最后到达时刻（判过期/新帧用） */
} Al_Off_t;

#define AL_SRC_RCP   0U    /* 转盘：$RCP（Align_Start） */
#define AL_SRC_SHOT  1U    /* 打靶：$SHOT（Align_StartShot） */
#define AL_SRC_SHOT2 2U    /* ★ROUGH STACK：$SHOT2（Align_StartShot2）—— 见视觉组《SHOT2通讯对接.md》 */

/* ★"打靶类"数据源（$SHOT / $SHOT2）：对位行为**完全一致**（严格档死区 1px、不限时、两轮制、
   车身矫正、冻结平移坐标、置 0 登记），只有**数据来源不同**：
     $SHOT  = 中间圆环圆心（STORE / ROUGH PLACE）；
     $SHOT2 = 三个横排物料里"中间那个"的颜色质心（ROUGH STACK）。 */
#define AL_IS_SHOT_SRC()   (((alSrc) == AL_SRC_SHOT) || ((alSrc) == AL_SRC_SHOT2))

static uint8_t  alSrc = AL_SRC_RCP;   /* 本次对位用哪个源（两个入口各自设置） */
static Al_Off_t alOff;                /* 当前源的偏移快照（每拍刷新） */
static uint8_t  alYawOn = 0U;         /* ★本次对位是否要做"车身/圆盘平行度"（$SHOT 或 $RP） */
static uint32_t alYawTick = 0U;       /* ★车身矫正段的起始时刻（安全阀 ALIGN_YAW_MAX_MS 用） */

/* 本数据源的名字（日志/页面用）：RCP / SHOT / SHOT2 */
static const char *Al_SrcName(void)
{
    if (alSrc == AL_SRC_SHOT)  { return "SHOT"; }
    if (alSrc == AL_SRC_SHOT2) { return "SHOT2"; }
    return "RCP";
}

/* ===== ★【2026-10-15】运行期 PID 参数读写（UART5 在线调参 / PC 自动调参） ===== */

static float Al_ClampF(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

uint8_t Align_SetPidCfg(uint8_t src, const Align_PidCfg_t *cfg)
{
    Align_PidCfg_t *dst;

    if ((src > 2U) || (cfg == NULL))
    {
        return 0U;
    }
    dst = &alPidCfg[src];

    /* 夹到安全范围：绝不接受负增益/0 上限（会把车轮锁死或反向飞车） */
    dst->kp       = Al_ClampF(cfg->kp,      0.0f, 2.0f);
    dst->ki       = Al_ClampF(cfg->ki,      0.0f, 2.0f);
    dst->kd       = Al_ClampF(cfg->kd,      0.0f, 0.50f);
    dst->rpmMax   = Al_ClampF(cfg->rpmMax,  2.0f, 60.0f);
    dst->wheelMax = (cfg->wheelMax < 10) ? 10 : ((cfg->wheelMax > 80) ? 80 : cfg->wheelMax);
    dst->ykp      = Al_ClampF(cfg->ykp,     0.0f, 2.0f);
    dst->yki      = Al_ClampF(cfg->yki,     0.0f, 2.0f);
    dst->ykd      = Al_ClampF(cfg->ykd,     0.0f, 0.50f);
    dst->yrpmMax  = Al_ClampF(cfg->yrpmMax, 1.0f, 40.0f);

#if (ALIGN_PID_ENABLE != 0U)
    /* 正在跑的就是这一套 -> 立刻换增益（保留积分/微分状态，便于边跑边微调） */
    if (src == alSrc)
    {
        alPidF.kp = dst->kp;  alPidF.ki = dst->ki;  alPidF.kd = dst->kd;
        alPidR.kp = dst->kp;  alPidR.ki = dst->ki;  alPidR.kd = dst->kd;
        alPidW.kp = dst->ykp; alPidW.ki = dst->yki; alPidW.kd = dst->ykd;
    }
#endif

    return 1U;
}

const Align_PidCfg_t *Align_GetPidCfg(uint8_t src)
{
    if (src > 2U)
    {
        return NULL;
    }
    return &alPidCfg[src];
}

/**
 * @brief  按当前数据源刷新偏移快照
 * @param  无
 * @retval 无
 * @note   $RCP 与 $SHOT 字段语义一致，这里只做搬运；**过期清零由 bsp_rpi 负责**
 *         （超 500ms 会把 valid 清 0 并把数值清 0），所以这里读到的就是权威值 ——
 *         绝不要在本文件里自己判断"过期"，否则会与 bsp_rpi 的判据打架。
 */
static void Al_RefreshOff(void)
{
    if (AL_IS_SHOT_SRC())
    {
        /* ★$SHOT 与 $SHOT2 是两套独立数据，按本次源取对应的那一份 */
        const RPi_Shot_t *s = (alSrc == AL_SRC_SHOT2) ? RPi_GetShot2() : RPi_GetShot();

        alOff.dxPx       = s->dxPx;
        alOff.dyPx       = s->dyPx;
        alOff.valid      = s->valid;
        alOff.lastRxTick = s->lastRxTick;

        /* ★车身矫正：平行度取值（见 bsp_align.h 的 ALIGN_YAW_Q_MODE）
             ★视觉组《打靶SHOT对接.md》：`$SHOT,q1,q2,q3,dx,dy` 里**只有 q1 有效**
               （q1 = 三环连线倾斜角，单位 0.1°，正 = 顺时针）；**q2/q3 是保留字段、恒 0**。
             所以默认 `ALIGN_YAW_Q_MODE = 1` **只取 q1**（早期按"三个平均"会把误差除以 3）。
             1/2/3 = 只取第 1/2/3 个；其它值(0) = 三个取平均（保留给以后视觉端真填了 q2/q3 时用）。 */
#if (ALIGN_YAW_ENABLE != 0U)
#if   (ALIGN_YAW_Q_MODE == 1U)
        alOff.qAvg = s->q[0];
#elif (ALIGN_YAW_Q_MODE == 2U)
        alOff.qAvg = s->q[1];
#elif (ALIGN_YAW_Q_MODE == 3U)
        alOff.qAvg = s->q[2];
#else
        alOff.qAvg = (int16_t)(((int32_t)s->q[0] + (int32_t)s->q[1] + (int32_t)s->q[2]) / 3);
#endif
#else
        alOff.qAvg = 0;
#endif
    }
    else
    {
        const RPi_Rcp_t *c = RPi_GetRcp();
        const RPi_Rp_t  *rp = RPi_GetRp();

        alOff.dxPx       = c->dxPx;
        alOff.dyPx       = c->dyPx;
        alOff.valid      = c->valid;
        alOff.lastRxTick = c->lastRxTick;

        /* ★转盘平行度（新增协议 `$RP`）：q1 单位 0.1°，与 $SHOT 的 q1 同口径。
           `$RP` 数据过期由 bsp_rpi 清零；这里只搬运。
           ⚠ q1=0 可能是"已对齐"也可能是"没检到红线" —— 0 时不产生转向（等效已对齐）。 */
#if (ALIGN_YAW_RCP != 0U)
        if (rp->valid != 0U)
        {
            alOff.qAvg = rp->q1;
        }
        else
        {
            alOff.qAvg = 0;
        }
#else
        alOff.qAvg       = 0;
#endif
    }
}

/* ===================== 对外接口 ===================== */

/**
 * @brief  开始一次对位（内部共用入口）
 * @param  src: AL_SRC_RCP（$RCP，转盘）/ AL_SRC_SHOT（$SHOT，打靶区）
 * @retval 1=已启动；0=拒绝启动（已在运行 / 该源当前无有效数据）
 */
static uint8_t Al_Begin(uint8_t src)
{
    const Al_Off_t *tt;

    if ((alState == AL_YAW) || (alState == AL_RUN) || (alState == AL_READ) ||
        (alState == AL_SETTLE))
    {
        return 0U;   /* 已在运行 */
    }

    alSrc = src;
    Al_RefreshOff();
    tt = &alOff;

    if (tt->valid == 0U)
    {
        Debug_Log("ALN", "start refused: %s offset invalid", Al_SrcName());
        return 0U;
    }

    alIter      = 0U;

    /* ★★【精度分档】按**数据源**选档（调用方不用管）：
         · $SHOT / $SHOT2（打靶/暂存区 STORE·ROUGH PLACE·ROUGH STACK）= **严格档**：死区 1px + **不限时**；
         · $RCP （转盘区 TRAY PICK）           = **常规档**：死区 6px。
       两档**都只在真正进死区时才判成功**（"一步到位"，末端放宽出口已于 2026-10-10 删除）。
       死区是 bsp_rpi 的运行期配置，判定(Al_EffDeadband)直接读它，所以在这里写就行。 */
    if (AL_IS_SHOT_SRC())
    {
        RPi_GetCfg()->deadbandPx = (int16_t)RPI_PX_DEADBAND_STRICT;
        /* ★【2026-10-10 用户要求】打靶放置（STORE / ROUGH PLACE）对位**不限时**：
           预算恒为 0 —— "不进 1px 死区永不收工"。
           即使调用方设过预算也不采纳（防止残留把打靶对位提前判失败）。 */
        alBudgetMs = 0U;
    }
    else
    {
        RPi_GetCfg()->deadbandPx = (int16_t)RPI_PX_DEADBAND;
        /* 转盘档沿用**调用方设的预算**（原行为：bsp_tt 会设 TT_ALIGN_BUDGET_MS，
           因为转盘只停 6~10s）。调用方没设过就是 0 = 不限时。 */
        alBudgetMs = alBudgetReq;
    }

    alLastPx    = tt->dxPx;
    alLastPy    = tt->dyPx;
    alPhaseTick = HAL_GetTick();
    alTick      = alPhaseTick;
    alRunLogTick = alPhaseTick;
    alLastSeen    = tt->lastRxTick;
    alNoDataTick  = alPhaseTick;
    alNoDataLogged = 0U;
    alRpmBase     = 0;              /* 限斜率从 0 起步 -> 软起步 */
    alProgTick    = alPhaseTick;
    alBudgetTick  = alPhaseTick;    /* 时间预算从这一刻开始算 */
    alProgPx      = 32767;          /* 首拍就算"有进步" */
    alBoost       = 0U;
    alBoostTick   = alPhaseTick;
    alTickCnt     = 0U;
    alHoldCnt     = 0U;
    alFrameTick   = alPhaseTick;   /* 帧间隔从本次开始计时 */
    alFramePeriod = 0U;
    alReqTick     = alPhaseTick;   /* 重发请求计时从本次开始（首帧刚拿到，不必立刻重发） */

    /* ★车身矫正：只有 **$SHOT（打靶/暂存 STORE·ROUGH PLACE）** 且开关打开时才做，
       且按用户 2026-10-10 要求**放在 XY 平移之前**（先转正、再平移）。
       转盘 $RCP 不受影响（该帧没有平行度字段）。 */
#if (ALIGN_YAW_ENABLE != 0U)
    alYawOn = (AL_IS_SHOT_SRC()) ? 1U : 0U;
#else
    alYawOn = 0U;
#endif
    /* ★【2026-10-15】转盘区：$RCP 本身没有 q，q 来自新增的 `$RP`（ALIGN_YAW_RCP） */
#if (ALIGN_YAW_RCP != 0U)
    if (alSrc == AL_SRC_RCP)
    {
        alYawOn = 1U;
    }
#endif
    alYawTick = alPhaseTick;       /* ★车身矫正段的计时起点（安全阀 ALIGN_YAW_MAX_MS 用） */
    alPass    = 1U;                /* ★两轮制【2026-10-10 晚】：每次启动都从第 1 轮（粗靠近）开始 */

    /* ★【2026-10-15】先按**数据源**选 PID 参数表（三套分开调），再清 PID 状态 */
#if (ALIGN_PID_ENABLE != 0U)
    alCfg = &alPidCfg[(alSrc < 3U) ? alSrc : 0U];
    Al_PidResetAll(alPhaseTick);
#endif

    /* ★【2026-10-15】`ALIGN_YAW_SIMUL=1`：不单独做"先转正"这一段，
       直接进 XY，由 `Align_RunPhase` 每帧同时算旋转轴（三轴一起收敛）。 */
#if (ALIGN_YAW_SIMUL != 0U)
    alState = AL_READ;
#else
    alState = (alYawOn != 0U) ? AL_YAW : AL_READ;
#endif

    /* ★【2026-10-11 用户要求】**任务点对位期间，平移坐标冻结、不再累加**（与原地旋转一样）：
       对位是"原地微调"，车身物理上会动一点，但用户要的是"坐标不动"（位置以视觉对位为准）。
       ★**只对打靶区（$SHOT）生效**：转盘区（$RCP）定位**必须继续计算绝对坐标**
         （用户："转盘区定位需要计算绝对坐标"）。
       冻结在 Al_FinishOk()/Al_FinishAbort()/Align_Abort() 里解除；
       下一次 GO 的 Mission_NavGoTo 开头还有一道"防御性解冻"，所以不会漏解冻。 */
    if (AL_IS_SHOT_SRC())
    {
        Chassis_OdomHoldXY(1U);
        Debug_Log("ALN", "odom holdXY ON (%s align in progress)", Al_SrcName());
    }

    Debug_Log("ALN", "start src=%s px=%d,%d db=%dpx tick=%u/%u px=%d..%d rpm=%d..%d",
              Al_SrcName(),
              (int)tt->dxPx, (int)tt->dyPx, (int)RPi_GetCfg()->deadbandPx,
              (unsigned)ALIGN_TICK_MS, (unsigned)ALIGN_TICK_FINE_MS,
              (int)ALIGN_XY_PX_MIN, (int)ALIGN_XY_PX_MAX,
              (int)ALIGN_XY_RPM_MIN, (int)ALIGN_XY_RPM_MAX);
    if (alYawOn != 0U)
    {
#if (ALIGN_YAW_SIMUL != 0U)
        Debug_Log("ALN", "yaw SIMUL with XY: q=%d mode=%u sign=%d db=%d rpm=%d",
                  (int)tt->qAvg, (unsigned)ALIGN_YAW_Q_MODE, (int)ALIGN_YAW_SIGN,
                  (int)ALIGN_YAW_DEADBAND, (int)alCfg->yrpmMax);
#else
        Debug_Log("ALN", "yaw first: q=%d mode=%u sign=%d db=%d rpm=%d..%d",
                  (int)tt->qAvg, (unsigned)ALIGN_YAW_Q_MODE, (int)ALIGN_YAW_SIGN,
                  (int)ALIGN_YAW_DEADBAND, (int)ALIGN_YAW_RPM_MIN, (int)ALIGN_YAW_RPM_MAX);
#endif
    }

#if (ALIGN_PID_ENABLE != 0U)
    /* ★【2026-10-15】把**本次实际生效的那套 PID**打出来（×1000 / ×1），
       这样抓到的每一段调参数据都能自证"用的哪组参数"，不会调串。 */
    Debug_Log("ALN", "pid cfg src=%s kp=%d ki=%d kd=%d rpm=%d wheel=%d yaw=%d,%d,%d,%d (x1000)",
              Al_SrcName(),
              (int)(alCfg->kp * 1000.0f), (int)(alCfg->ki * 1000.0f),
              (int)(alCfg->kd * 1000.0f), (int)alCfg->rpmMax, (int)alCfg->wheelMax,
              (int)(alCfg->ykp * 1000.0f), (int)(alCfg->yki * 1000.0f),
              (int)(alCfg->ykd * 1000.0f), (int)alCfg->yrpmMax);
#endif
    return 1U;
}

uint8_t Align_Start(void)
{
    return Al_Begin(AL_SRC_RCP);       /* 转盘区：数据源 $RCP（协议 §3.1） */
}

uint8_t Align_StartShot(void)
{
    return Al_Begin(AL_SRC_SHOT);      /* 打靶区：数据源 $SHOT（协议 §3.2） */
}

/**
 * @brief  开始闭环对位（★ROUGH STACK：数据源 `$SHOT2`）
 * @param  无
 * @retval 1=已开始；0=没开始（视觉数据无效/过期，或已在运行）
 * @note   ★【视觉组《SHOT2通讯对接.md》】`$SHOT2` 与 `$SHOT` 是**两个不同的测量**：
 *           `$SHOT`  瞄"中间**圆环**圆心"（色环靶位 -> STORE / ROUGH PLACE）；
 *           `$SHOT2` 瞄"三个横排物料里**中间那个的颜色质心**"（-> ROUGH STACK）。
 *         数据**各自独立**（`RPi_GetShot2()`）；本入口除数据源外的行为
 *         （严格档死区 1px、不限时、两轮制、车身矫正 q1、冻结平移坐标）与 `Align_StartShot()` **完全一致**。
 *         调本函数前应先发 `RPi_ReqShot2()` 并等 `RPi_GetShot2()->valid`。
 */
uint8_t Align_StartShot2(void)
{
    return Al_Begin(AL_SRC_SHOT2);     /* 打靶区(v2)：数据源 $SHOT2 */
}

/* ===================== 时间预算（"几秒内必须完成对位"） ===================== */

/**
 * @brief  当前生效的"死区"（像素）
 * @retval 已设预算且时间过半时逐级放宽（放宽量 = 2×ALIGN_BUDGET_RELAX_PX）
 * @note   ALIGN_BUDGET_RELAX_PX 现为 **0 = 不放宽**，所以有预算时死区也恒等于
 *         RPi_GetCfg()->deadbandPx（见 bsp_rpi.h 的 RPI_PX_DEADBAND）。
 *         此时时间预算只剩"提速 + 到点硬收工"（到点没进死区就判失败）。
 */
static int32_t Al_EffDeadband(void)
{
    uint32_t el;
    uint32_t b;

    if (alBudgetMs == 0U)
    {
        return (int32_t)(int32_t)(RPi_GetCfg()->deadbandPx);
    }

    el = HAL_GetTick() - alBudgetTick;
    b  = alBudgetMs;

    if (el >= ((b * 4U) / 5U))
    {
        return (int32_t)((int32_t)(RPi_GetCfg()->deadbandPx) + (2 * ALIGN_BUDGET_RELAX_PX));   /* 用掉 80% -> 再放宽 */
    }
    if (el >= ((b * 3U) / 5U))
    {
        return (int32_t)((int32_t)(RPi_GetCfg()->deadbandPx) + ALIGN_BUDGET_RELAX_PX);         /* 用掉 60% -> 先放宽 */
    }
    return (int32_t)(int32_t)(RPi_GetCfg()->deadbandPx);
}

/**
 * @brief  ★【2026-10-15】同步调试快照（状态类字段；每拍调一次，开销≈0）
 * @note   逐帧字段（mm 误差 / PID 输出 / dt）由 PID 更新点和 Align_VelMoveVel 填。
 */
static void Al_DbgSync(void)
{
    uint8_t running;

    running = ((alState == AL_YAW) || (alState == AL_RUN) ||
               (alState == AL_READ) || (alState == AL_SETTLE)) ? 1U : 0U;

    alDbg.active     = running;
    alDbg.state      = (uint8_t)alState;
    alDbg.src        = alSrc;
    alDbg.pass       = alPass;
    alDbg.iters      = alIter;
    alDbg.dxPx       = alOff.dxPx;
    alDbg.dyPx       = alOff.dyPx;
    alDbg.qRaw       = alOff.qAvg;
    alDbg.deadbandPx = (uint32_t)Al_EffDeadband();
    alDbg.elapsedMs  = (alState != AL_IDLE) ? (HAL_GetTick() - alBudgetTick) : 0U;
}

/**
 * @brief  是否已进"当前生效死区"
 * @param  tt: 当前源的偏移（$RCP 或 $SHOT，见 Al_RefreshOff）
 * @retval 1=已进死区；0=没进/数据无效
 */
static uint8_t Al_Aligned(const Al_Off_t *tt)
{
    int32_t tol = Al_EffDeadband();

    /* ★两轮制【2026-10-10 晚】：第 1 轮 = **粗靠近**（死区放大 ALIGN_PASS1_DB_MUL 倍，快点收工）；
       第 2 轮 = **精定位**（死区回到严格值，保证精准）。
       **只对 `$SHOT` 生效** —— `$RCP`（TRAY PICK）有 7s 时间预算、只能单轮，不能放宽。 */
    if ((ALIGN_PASS_COUNT > 1U) && (alPass <= 1U) && AL_IS_SHOT_SRC())
    {
        tol *= (int32_t)ALIGN_PASS1_DB_MUL;
    }

    if (tt->valid == 0U)
    {
        return 0U;
    }

    /* ★【2026-10-15】XY 与平行度**同时**闭环时：两者都进死区才算到位。
       否则会出现"平移到了、车身却是歪的"（旧行为：进 XY 段后 uW 恒 0）。 */
#if (ALIGN_YAW_SIMUL != 0U)
    if ((alYawOn != 0U) && (Align_Abs16(tt->qAvg) > (int16_t)ALIGN_YAW_DEADBAND))
    {
        return 0U;
    }
#endif

    return ((Align_Abs16(tt->dxPx) <= tol) && (Align_Abs16(tt->dyPx) <= tol)) ? 1U : 0U;
}

#if (ALIGN_PID_ENABLE != 0U)
/**
 * @brief  速度段一拍（★PID 版）：三轴 PID -> 四轮速度模式
 * @param  tt:  当前偏移快照（最新一帧）
 * @param  now: 当前时刻
 * @retval 无
 * @note   ★【2026-10-15 用户要求】"对位运用 PID 算法，对车轮进行速度模式控制，做到快速定位"。
 *         与原实现的区别：
 *           ① **X/Y 同时闭环**（`ALIGN_PID_XY_SIMUL=1`）：不再"先消 X 再消 Y"，两个方向一次到位
 *              —— 这是最大的提速来源（少走一趟）；
 *           ② **PID**：积分消"打滑/机构间隙"造成的稳态残差，微分抑制过冲，
 *              所以**允许更高的速度上限**（默认 25 RPM，旧版 15）；
 *           ③ 输出直接是**车轮 RPM**（`Align_VelMoveVel` 逆解），不再做"位移比例分配"。
 *         ⚠ **只在收到新视觉帧时更新一次 PID**（同一帧重复算 = 同一段误差被积分/微分多次 -> 振荡）；
 *            帧与帧之间保持上一次速度（速度模式下电机会自己保持），这也是"丝滑"的来源。
 *         进死区 -> 软停 + SETTLE（与旧版一致：仍要"停稳 + 复测确认"）。
 */
static void Align_RunPhase(const Al_Off_t *tt, uint32_t now)
{
    int16_t  mmFwd   = 0;
    int16_t  mmRight = 0;
    uint32_t dtMs;
    float    dt;
    float    uF = 0.0f;   /* 前进轴 PID 输出（RPM） */
    float    uR = 0.0f;   /* 横移轴 PID 输出（RPM） */
    float    uW = 0.0f;   /* 旋转轴：平移段保持 0（车身朝向已在 AL_YAW 段转正，沿用旧口径） */
    int16_t  em;

    alLastPx = tt->dxPx;
    alLastPy = tt->dyPx;

    /* ① 两轴都进死区 -> 软停，静置后复测确认 */
    if (Al_Aligned(tt) != 0U)
    {
        Align_StopWheels();
        alState     = AL_SETTLE;
        alPhaseTick = now;
        return;
    }

    /* ② 同一帧只算一次（关键：否则等于把同一段误差反复积分 -> 必然过冲/振荡） */
    if (tt->lastRxTick == alPidFrame)
    {
        return;
    }

    dtMs = now - alPidTick;
    if (dtMs < 10U)  { dtMs = 10U; }
    if (dtMs > 500U) { dtMs = 500U; }
    alPidFrame = tt->lastRxTick;
    alPidTick  = now;
    dt = (float)dtMs / 1000.0f;

    /* ③ 像素 -> **车体 mm**（唯一换算入口：含轴映射 + 方向符号 + ±maxPx 钳位） */
    RPi_PxToBodyMm(tt->dxPx, tt->dyPx, &mmFwd, &mmRight);

    /* ④ 平移两轴 PID */
#if (ALIGN_PID_XY_SIMUL != 0U)
    uF = Al_PidStep(&alPidF, (float)mmFwd,   dt,
                    alCfg->rpmMax, (float)ALIGN_PID_I_SEP_MM);
    uR = Al_PidStep(&alPidR, (float)mmRight, dt,
                    alCfg->rpmMax, (float)ALIGN_PID_I_SEP_MM);
#else
    /* 旧口径：先 X 后 Y（X 还在死区外就只动 X；另一轴 PID 复位，避免它偷偷积分） */
    if (Align_Abs16(tt->dxPx) > (int16_t)Al_EffDeadband())
    {
        uF = Al_PidStep(&alPidF, (float)mmFwd, dt,
                        alCfg->rpmMax, (float)ALIGN_PID_I_SEP_MM);
        Al_PidReset(&alPidR, alCfg->kp, alCfg->ki,
                    alCfg->kd, (float)ALIGN_PID_I_MAX);
    }
    else
    {
        uR = Al_PidStep(&alPidR, (float)mmRight, dt,
                        alCfg->rpmMax, (float)ALIGN_PID_I_SEP_MM);
        Al_PidReset(&alPidF, alCfg->kp, alCfg->ki,
                    alCfg->kd, (float)ALIGN_PID_I_MAX);
    }
#endif

    /* ⑤ ★【2026-10-15 用户要求】旋转轴**与 XY 同时**闭环（ALIGN_YAW_SIMUL）。
       原来这一段 uW 恒 0（只在开头的 AL_YAW 段转正），平移中打滑产生的偏航没人管 ->
       车身越跑越歪、XY 在旋转的坐标系里追，永远收敛不干净。
       现在每帧用同一个 `alPidW` 算 q 的 PID，和 uF/uR 一起逆解到四轮。
       $RCP 没有 q 字段（alYawOn=0），uW 保持 0，转盘行为不变。 */
#if (ALIGN_YAW_SIMUL != 0U)
#if (ALIGN_PID_ENABLE != 0U)
    if (alYawOn != 0U)
    {
        int32_t qErr = (int32_t)tt->qAvg * (int32_t)ALIGN_YAW_SIGN;

        uW = Al_PidStep(&alPidW, (float)qErr, dt,
                        alCfg->yrpmMax,
                        (float)((int32_t)ALIGN_YAW_DEADBAND * 10));
    }
#endif
#endif

    /* ⑥ ★【2026-10-15】静摩擦补偿：PID 想动（|u|>0）就至少给 ALIGN_PID_MIN_U。
       否则末端 0.1~0.5 RPM 时麦轮不转（实测 u=0.3 w=0 停 1.4s），只能等踢脚 ->
       走走停停、又慢又不准。方向不变，只在"太小"时抬到最低有效值。
       ⚠ 上一版曾改成 4.0 + 按帧自适应，现场反馈"更严重"，已退回 0.8。 */
    if ((uF > 0.0f) && (uF < (float)ALIGN_PID_MIN_U))   { uF = (float)ALIGN_PID_MIN_U; }
    if ((uF < 0.0f) && (uF > -(float)ALIGN_PID_MIN_U)) { uF = -(float)ALIGN_PID_MIN_U; }
    if ((uR > 0.0f) && (uR < (float)ALIGN_PID_MIN_U))   { uR = (float)ALIGN_PID_MIN_U; }
    if ((uR < 0.0f) && (uR > -(float)ALIGN_PID_MIN_U)) { uR = -(float)ALIGN_PID_MIN_U; }
    if ((uW > 0.0f) && (uW < (float)ALIGN_PID_MIN_U))   { uW = (float)ALIGN_PID_MIN_U; }
    if ((uW < 0.0f) && (uW > -(float)ALIGN_PID_MIN_U)) { uW = -(float)ALIGN_PID_MIN_U; }

    /* ★调参快照：本帧喂给 PID 的误差与帧间隔（对位测试页据此打印调参日志） */
    alDbg.mmFwd   = mmFwd;
    alDbg.mmRight = mmRight;
    alDbg.frameMs = (uint16_t)dtMs;
    alDbg.frames++;

    /* ⑤ 卡住保护：偏差长时间不缩小（静摩擦 / 轮子没转起来 / 打滑）-> **踢一脚**（保号抬到 KICK 档）。
       PID 的积分也能顶上去，但要好几个周期；这个检测 ALIGN_STALL_MS 就明确抬一次。
       偏差重新变小 -> 自动复位（回 PID 正常输出）。 */
    em = (Align_Abs16(tt->dxPx) > Align_Abs16(tt->dyPx)) ? Align_Abs16(tt->dxPx)
                                                         : Align_Abs16(tt->dyPx);

    /* ★【2026-10-15】XY 已在死区内、只剩平行度没到时**不要踢**：
       否则"踢一脚"会把已经对准的 XY 顶出去（uF/uR 原本 ~0，被抬到 2.5RPM）。 */
    if (em <= (int16_t)Al_EffDeadband())
    {
        alProgPx   = em;
        alProgTick = now;
        alBoost    = 0U;
    }

    if (em < alProgPx)
    {
        alProgPx   = em;
        alProgTick = now;
        alBoost    = 0U;
    }
    else if ((now - alProgTick) > (uint32_t)ALIGN_STALL_MS)
    {
        alProgTick = now;
        if (alBoost == 0U)
        {
            alBoost     = 1U;
            alBoostTick = now;          /* ★开始计时：最多踢 ALIGN_PID_KICK_MAX_MS */
            Debug_Log("ALN", "pid stall px=%d,%d -> kick %u rpm",
                      (int)tt->dxPx, (int)tt->dyPx, (unsigned)ALIGN_PID_KICK_RPM);
        }
    }
    if (alBoost != 0U)
    {
        if ((now - alBoostTick) > (uint32_t)ALIGN_PID_KICK_MAX_MS)
        {
            /* ★限时到 -> 撤回 PID。原实现这里没有出口，alBoost 会一直锁存 =>
               每拍都按 KICK 档输出，车在死区附近"满速来回摆"（实测 ±40mm）。 */
            alBoost = 0U;
            Debug_Log("ALN", "kick end after %u ms -> PID", (unsigned)ALIGN_PID_KICK_MAX_MS);
        }
        else
        {
            if (uF != 0.0f)
            {
                uF = (uF > 0.0f) ? (float)ALIGN_PID_KICK_RPM : -(float)ALIGN_PID_KICK_RPM;
            }
            if (uR != 0.0f)
            {
                uR = (uR > 0.0f) ? (float)ALIGN_PID_KICK_RPM : -(float)ALIGN_PID_KICK_RPM;
            }
        }
    }

    /* ⑥ 下发（速度模式：三轴合成 -> 四轮背靠背） */
    Align_VelMoveVel(uF, uR, uW, now);
}
#else
/**
 * @brief  速度段一拍：**X/Y 分轴 + 像素→速度连续映射**下发一次四轮速度
 * @param  tt:  转盘偏移（最新一帧）
 * @param  now: 当前时刻
 * @retval 无
 * @note   ★ 控制律（2026-10-09 二次改，替代原来的"两轴合成 P 控制"）：
 *           ① **一轴一轴来**：只要 |dx| 还在死区外就只动 X 轴（Y 轴置 0）；
 *              X 进死区后才开始动 Y。两轴分开动、互不牵扯，末端不会来回摆。
 *           ② **像素偏差 -> 速度线性插值**（越近越慢、速度连续，见 bsp_align.h）：
 *                 |偏差| ≤ ALIGN_XY_PX_MIN -> ALIGN_XY_RPM_MIN（保底）
 *                 |偏差| ≥ ALIGN_XY_PX_MAX -> ALIGN_XY_RPM_MAX（封顶）
 *                 中间                    -> 线性插值（逐级递减）
 *           ③ 偏差已进"保底速度区"（≤ ALIGN_XY_PX_MIN）后节拍换成更短的
 *              ALIGN_TICK_FINE_MS，每拍位移更小、末端更稳。
 *           ④ 两轴都进死区（Al_Aligned）-> 停轮 -> AL_SETTLE -> 复测确认
 *              （避免"边停边判"把滑行量算漏）。
 *           ⑤ 卡住保护：偏差长时间不缩小 -> 把速度抬到上限（轮子可能没转起来）。
 *         方向换算仍走 bsp_rpi 的唯一入口，**分轴时另一轴传 0**，
 *         所以"X>0 后退 / Y>0 左移"这条规则仍由 RPi_Cfg 的符号决定，本文件不含方向假设。
 */
static void Align_RunPhase(const Al_Off_t *tt, uint32_t now)
{
    int16_t  mmFwd   = 0;
    int16_t  mmRight = 0;
    int16_t  em;
    int16_t  axPx;                 /* 本拍要消的那一轴的偏差（像素，带符号） */
    int32_t  rpm;
    uint32_t tickMs;
    uint8_t  useY = 0U;            /* 0=本拍动 X 轴；1=本拍动 Y 轴 */
    uint8_t  near;

    alLastPx = tt->dxPx;
    alLastPy = tt->dyPx;

    /* ① 两轴都进死区 -> 软停，静置后复测确认 */
    if (Al_Aligned(tt) != 0U)
    {
        Align_StopWheels();
        alState     = AL_SETTLE;
        alPhaseTick = now;
        return;
    }

    /* ② 选轴：先 X（图像 dx），X 进死区后再 Y（图像 dy） */
    if (Align_Abs16(tt->dxPx) > (int16_t)Al_EffDeadband())
    {
        axPx = tt->dxPx;
    }
    else
    {
        axPx = tt->dyPx;
        useY = 1U;
    }

    /* ③ 选速 + 选节拍：**像素偏差 -> 速度线性插值**（越近越慢，见 bsp_align.h）：
             |偏差| ≤ ALIGN_XY_PX_MIN              -> ALIGN_XY_RPM_MIN（保底）
             |偏差| ≥ ALIGN_XY_PX_MAX              -> ALIGN_XY_RPM_MAX（封顶）
             PX_MIN < |偏差| < PX_MAX              -> 线性插值（逐级递减、速度连续） */
    {
        int32_t dev   = (int32_t)Align_Abs16(axPx);
        int32_t pxMin = (int32_t)ALIGN_XY_PX_MIN;
        int32_t pxMax = (int32_t)ALIGN_XY_PX_MAX;
        int32_t rMin  = (int32_t)ALIGN_XY_RPM_MIN;
        int32_t rMax  = (int32_t)ALIGN_XY_RPM_MAX;

        if ((pxMax <= pxMin) || (dev >= pxMax))
        {
            rpm = rMax;
        }
        else if (dev <= pxMin)
        {
            rpm = rMin;
        }
        else
        {
            rpm = rMin + (((rMax - rMin) * (dev - pxMin)) / (pxMax - pxMin));
        }
    }
    near   = (Align_Abs16(axPx) <= (int16_t)ALIGN_XY_PX_MIN) ? 1U : 0U;
    tickMs = (near != 0U) ? (uint32_t)ALIGN_TICK_FINE_MS : (uint32_t)ALIGN_TICK_MS;

    /* ④ 卡住检测：偏差一直没变小 -> 抬到速度上限（轮子可能根本没转起来） */
    em = (Align_Abs16(tt->dxPx) > Align_Abs16(tt->dyPx)) ? Align_Abs16(tt->dxPx)
                                                         : Align_Abs16(tt->dyPx);
    if (em < alProgPx)
    {
        alProgPx   = em;
        alProgTick = now;
        alBoost    = 0U;
    }
    else if ((now - alProgTick) > (uint32_t)ALIGN_STALL_MS)
    {
        alProgTick = now;
        if (alBoost == 0U)
        {
            alBoost = 1U;
            Debug_Log("ALN", "stall px=%d,%d -> max rpm", (int)tt->dxPx, (int)tt->dyPx);
        }
    }
    if (alBoost != 0U)
    {
        rpm = (int32_t)ALIGN_XY_RPM_MAX;
    }

    /* ⑤ 只把"本拍那一轴"的偏差送进换算入口（另一轴传 0） */
    if (useY != 0U)
    {
        RPi_PxToBodyMm(0, axPx, &mmFwd, &mmRight);
    }
    else
    {
        RPi_PxToBodyMm(axPx, 0, &mmFwd, &mmRight);
    }

    Align_VelMove(mmFwd, mmRight, 0, rpm, tickMs, now);   /* 平移段：不叠加旋转（rotW = 0） */
}
#endif   /* ALIGN_PID_ENABLE */

/* ===================== ★ 车身矫正（按 $SHOT 的"平行度 q"原地转正） =====================
   为什么要单独一段：原对位全程 dRot = 0（只平移），车身朝向不纠正；
   而暂存区/粗加工区（STORE/ROUGH PLACE）要求车与靶面"平行"。
   按用户 2026-10-10 指定：**先原地把 q 转进死区，再做 dx/dy 平移**（顺序固定在本段之前）。
   控制律与 XY 段同源：|q| 越近越慢（线性插值）、末端换短节拍、卡住抬到上限速度。
   参数/开关见 bsp_align.h 的 ALIGN_YAW_*；树莓派仍恒发 q=0 时本段会**立刻通过**（无副作用）。 */

/**
 * @brief  是否已"平行"（|q| 进 ALIGN_YAW_DEADBAND）
 * @param  tt: 当前偏移快照（qAvg 已按 ALIGN_YAW_Q_MODE 取好）
 * @retval 1=已平行（或数据无效时返回 0）
 */
static uint8_t Al_YawAligned(const Al_Off_t *tt)
{
    int32_t q;

    if (tt->valid == 0U)
    {
        return 0U;
    }
    q = (int32_t)tt->qAvg * (int32_t)ALIGN_YAW_SIGN;
    if (q < 0)
    {
        q = -q;
    }
    return (q <= (int32_t)ALIGN_YAW_DEADBAND) ? 1U : 0U;
}

/**
 * @brief  车身矫正一拍：按 |q| 算速度，原地旋转一次
 * @param  tt    : 当前偏移快照
 * @param  now   : 当前时刻
 * @param  tickMs: 本拍节拍（由调用方按 |q| 选长短，见 AL_YAW case）
 * @retval 无
 * @note   q 进死区 -> 停轮 + 转 AL_READ（开始 XY 平移）。
 */
#if (ALIGN_PID_ENABLE != 0U)
static void Align_YawPhase(const Al_Off_t *tt, uint32_t now, uint32_t tickMs)
{
    int32_t qErr = (int32_t)tt->qAvg * (int32_t)ALIGN_YAW_SIGN;   /* 带符号误差（0.1°） */
    int32_t qAbs;
    uint32_t dtMs;
    float    uW;

    /* ① 已平行 -> 停轮，交给 XY 平移段（AL_READ 会等新帧再评估 dx/dy） */
    if (Al_YawAligned(tt) != 0U)
    {
        Align_StopWheels();
        alRpmBase    = 0;
        alRunLogTick = now - 500U;
        alTick       = now;
        alProgTick   = now;
        alProgPx     = 32767;
        alBoost      = 0U;
        alPhaseTick  = now;
        alState      = AL_READ;
        Debug_Log("ALN", "yaw ok q=%d -> XY", (int)tt->qAvg);
        return;
    }

    /* ② 同一帧只算一次 PID（否则同一段误差被反复积分/微分 -> 振荡） */
    if (tt->lastRxTick == alPidFrame)
    {
        return;
    }

    dtMs = now - alPidTick;
    if (dtMs < 10U)  { dtMs = 10U; }
    if (dtMs > 500U) { dtMs = 500U; }
    alPidFrame = tt->lastRxTick;
    alPidTick  = now;

    /* ③ 旋转轴 PID：误差单位 0.1°，积分分离门限取死区的 10 倍（大角度阶段不积饱和） */
    uW = Al_PidStep(&alPidW, (float)qErr, (float)dtMs / 1000.0f,
                    alCfg->yrpmMax,
                    (float)((int32_t)ALIGN_YAW_DEADBAND * 10));

    /* ★调参快照：本帧的旋转误差与帧间隔（此时只有旋转轴在动） */
    alDbg.mmFwd   = 0;
    alDbg.mmRight = 0;
    alDbg.frameMs = (uint16_t)dtMs;
    alDbg.frames++;

    /* ④ 卡住保护：|q| 长时间不缩小 -> **踢一脚**（保号抬到 KICK 档） */
    qAbs = (qErr < 0) ? -qErr : qErr;
    if (qAbs < (int32_t)alProgPx)
    {
        alProgPx   = (int16_t)qAbs;
        alProgTick = now;
        alBoost    = 0U;
    }
    else if ((now - alProgTick) > (uint32_t)ALIGN_STALL_MS)
    {
        alProgTick = now;
        if (alBoost == 0U)
        {
            alBoost     = 1U;
            alBoostTick = now;          /* ★限时踢：最多 ALIGN_PID_KICK_MAX_MS */
            Debug_Log("ALN", "yaw stall q=%d -> kick %u rpm",
                      (int)tt->qAvg, (unsigned)ALIGN_PID_KICK_RPM);
        }
    }
    if (alBoost != 0U)
    {
        if ((now - alBoostTick) > (uint32_t)ALIGN_PID_KICK_MAX_MS)
        {
            alBoost = 0U;
            Debug_Log("ALN", "yaw kick end after %u ms -> PID", (unsigned)ALIGN_PID_KICK_MAX_MS);
        }
        else
        {
            uW = (uW > 0.0f) ? (float)ALIGN_PID_KICK_RPM : -(float)ALIGN_PID_KICK_RPM;
        }
    }

    (void)tickMs;   /* PID 版不再按节拍选速（节拍仍由 AL_YAW case 控制刷新频率） */

    /* ⑤ 只做原地旋转（前进/横移两轴为 0） */
    Align_VelMoveVel(0.0f, 0.0f, uW, now);
}
#else
/* ===== 旧版（`ALIGN_PID_ENABLE=0` 时用）：|q| 线性插值出转速 —— 保留以便回退 ===== */
static void Align_YawPhase(const Al_Off_t *tt, uint32_t now, uint32_t tickMs)
{
    int32_t qErr = (int32_t)tt->qAvg * (int32_t)ALIGN_YAW_SIGN;   /* 带符号误差 */
    int32_t qAbs;
    int32_t rpm;
    int16_t rotW;

    if (Al_YawAligned(tt) != 0U)
    {
        Align_StopWheels();
        alRpmBase    = 0;
        alRunLogTick = now - 500U;
        alTick       = now;
        alProgTick   = now;
        alProgPx     = 32767;
        alBoost      = 0U;
        alPhaseTick  = now;
        alState      = AL_READ;
        Debug_Log("ALN", "yaw ok q=%d -> XY", (int)tt->qAvg);
        return;
    }

    qAbs = (qErr < 0) ? -qErr : qErr;

    /* |q| -> 旋转速度：线性插值 */
    {
        int32_t qMin = (int32_t)ALIGN_YAW_PX_MIN;
        int32_t qMax = (int32_t)ALIGN_YAW_PX_MAX;
        int32_t rMin = (int32_t)ALIGN_YAW_RPM_MIN;
        int32_t rMax = (int32_t)ALIGN_YAW_RPM_MAX;

        if ((qMax <= qMin) || (qAbs >= qMax))
        {
            rpm = rMax;
        }
        else if (qAbs <= qMin)
        {
            rpm = rMin;
        }
        else
        {
            rpm = rMin + (((rMax - rMin) * (qAbs - qMin)) / (qMax - qMin));
        }
    }

    /* 卡住保护：|q| 长时间不缩小 -> 抬到上限速度 */
    if (qAbs < (int32_t)alProgPx)
    {
        alProgPx   = (int16_t)qAbs;
        alProgTick = now;
        alBoost    = 0U;
    }
    else if ((now - alProgTick) > (uint32_t)ALIGN_STALL_MS)
    {
        alProgTick = now;
        if (alBoost == 0U)
        {
            alBoost = 1U;
            Debug_Log("ALN", "yaw stall q=%d -> max rpm", (int)tt->qAvg);
        }
    }
    if (alBoost != 0U)
    {
        rpm = (int32_t)ALIGN_YAW_RPM_MAX;
    }

    rotW = (int16_t)((qErr >= 0) ? 1 : -1);
    Align_VelMove(0, 0, rotW, rpm, tickMs, now);
}
#endif   /* ALIGN_PID_ENABLE */

/**
 * @brief  设置对位时间预算（"几秒内必须完成对位"）
 * @param  budgetMs: 预算时长 ms；**0 = 不限时**（默认，一直磨到死区）
 * @note   开对位**之前**调用。★【2026-10-10 分档】只有**转盘档（$RCP）**会采纳本预算；
 *         **打靶档（$SHOT）恒为不限时**（见 Al_Begin），本设置对打靶无效。
 *         设了预算后：
 *           ① FAR 档速度上限 ×ALIGN_BUDGET_SPEED_PCT%（尽早把误差压下去）；
 *           ② 时间过半**逐级放宽死区**（ALIGN_BUDGET_RELAX_PX/级），把"就差一点"的收尾提前放行；
 *           ③ 到点仍未进最终死区 -> 判失败（不假装成功），日志打 `BUDGET ... used up`。
 *         用途：转盘只停 6~10s，不能让对位拖过这个窗口（见 bsp_tt 的 TT_ALIGN_BUDGET_MS）。
 */
void Align_SetBudget(uint32_t budgetMs)
{
    alBudgetReq = budgetMs;
}

/**
 * @brief  对位收尾：解冻平移坐标（成功 / 失败 / 中止都调）
 * @param  无
 * @retval 无
 * @note   对位期间平移坐标是**冻结**的（见 `Al_Begin`，与原地旋转一样）；这里解除。
 *         ⚠ 冻结/解冻**只有打靶区 `$SHOT` 才真正生效**：转盘区 `$RCP` 定位本来就没冻结
 *           （用户："转盘区定位需要计算绝对坐标"）。
 *         【已删除】原"对位成功后在任务点结束时把 IMU 清 0"的功能按用户要求
 *         （"将任务点置 0 删除"）**整体移除**。
 */
static void Al_ReleaseOdom(void)
{
    Chassis_OdomHoldXY(0U);
}

/* ===== 任务点"对位成功后静态置角"（由任务点在进入任务点时登记） =====
 * 与原来被删掉的"钉住 yaw"不同：这里只在**对位成功那一刻**做一次性置角
 * （IMU_StaticZeroTo），之后 yaw 照常积分，绝对坐标建立在新基准上。
 * ⚠ 对位刚结束时车可能还有残余转动（|gz| > 30 deg/s），IMU_StaticZeroTo 会拒绝；
 *   所以这里做成"挂起重试"：每拍试一次，静止了就立刻置角，最多等 AL_DONE_YAW_WAIT_MS。 */
#define AL_DONE_YAW_WAIT_MS   3000U   /* 置角挂起重试的最长时间（超过就放弃，绝不迟到乱置） */

static float    alDoneYawRef  = 0.0f;
static uint8_t  alDoneYawEn   = 0U;
static uint8_t  alZeroPend    = 0U;   /* 1=已登记、正等车停稳后置角 */
static uint32_t alZeroPendTick = 0U;

/** @brief 挂起的静态置角：车停稳就执行，最多等 AL_DONE_YAW_WAIT_MS */
static void Al_ZeroYawTick(uint32_t now)
{
    if (alZeroPend == 0U)
    {
        return;
    }

    /* ★必须用 Chassis_StaticZeroYaw：IMU 置角的同时平移里程计朝向基准，
       否则 Chassis_GetOdomYaw() 会跳变成 `ref − 原点yaw`（现场就是"置 0 后显示 180°"）。 */
    if (Chassis_StaticZeroYaw(alDoneYawRef) != 0U)
    {
        alZeroPend = 0U;
        return;
    }

    if ((now - alZeroPendTick) >= (uint32_t)AL_DONE_YAW_WAIT_MS)
    {
        alZeroPend = 0U;
        Debug_Log("ALN", "done-yaw %d give up (car still moving)", (int)(alDoneYawRef * 10.0f));
    }
}

void Align_SetDoneYawRef(float refDeg)
{
    alDoneYawRef  = refDeg;
    alDoneYawEn   = 1U;
    alZeroPend    = 0U;               /* 还没到"对位成功"，先不置角 */
    Debug_Log("ALN", "done-yaw ref=%d armed", (int)(refDeg * 10.0f));
}

void Align_ClearDoneYawRef(void)
{
    alDoneYawEn = 0U;
    alZeroPend  = 0U;
}

/**
 * @brief  对位**成功**收尾：解冻坐标 + 静态置角 + **响一下提示音**
 * @param  无
 * @retval 无
 * @note   ★静态置角：任务点在进入时用 Align_SetDoneYawRef() 登记目标角
 *         （TRAY PICK=0 / ROUGH PLACE=90 / ROUGH STACK=90），这里在对位成功
 *         那一刻调 IMU_StaticZeroTo() 一次性把零漂抹掉（未登记则不做）。
 *         ★另有用户要求："对位完成后蜂鸣器响一下做个提示"（见 bsp_align.h 的 ALIGN_DONE_BEEP_*）。
 */
static void Al_OnAligned(void)
{
    Al_ReleaseOdom();

    if (alDoneYawEn != 0U)
    {
        /* 只"挂起"：对位收尾这一拍车可能还在微动，由 Al_ZeroYawTick 等停稳后执行 */
        alZeroPend     = 1U;
        alZeroPendTick = HAL_GetTick();
        Al_ZeroYawTick(alZeroPendTick);
    }

    /* ★对位完成提示音：这里只**登记**（立即返回，不阻塞），真正发声在主循环的 BSP_Beep_Update()。
       只在对位**成功**时响（失败/中止路径走 Al_ReleaseOdom()，不响）。 */
#if (ALIGN_DONE_BEEP_TIMES > 0U)
    BSP_Beep((uint8_t)ALIGN_DONE_BEEP_TIMES,
             (uint16_t)ALIGN_DONE_BEEP_ON_MS,
             (uint16_t)ALIGN_DONE_BEEP_OFF_MS);
    Debug_Log("ALN", "aligned -> beep x%u (%ums)", (unsigned int)ALIGN_DONE_BEEP_TIMES,
              (unsigned int)ALIGN_DONE_BEEP_ON_MS);
#endif
}

void Align_Update(void)
{
    const Al_Off_t *tt;
    uint32_t now = HAL_GetTick();

    Al_RefreshOff();            /* 先按当前源（$RCP/$SHOT）刷新偏移快照 */
    tt = &alOff;

    Al_ZeroYawTick(now);        /* ★对位成功后的"静态置角"挂起重试（车停稳就执行） */

    Al_DbgSync();               /* ★调参快照（状态/源/死区/用时）：对位测试页会读它 */

    /* ---- 【协议 v2.0 必需】对位期间周期重发请求，保证一直有新帧 ----
       只在运行中（RUN/READ/SETTLE）重发：树莓派按请求回帧，不重发就没有新数据。 */
    if ((alState == AL_YAW) || (alState == AL_RUN) || (alState == AL_READ) ||
        (alState == AL_SETTLE))
    {
        if ((now - alReqTick) >= (uint32_t)ALIGN_REQ_POLL_MS)
        {
            alReqTick = now;
            if (alSrc == AL_SRC_SHOT)       { RPi_ReqShot();  }   /* 打靶区：$SHOT      */
            else if (alSrc == AL_SRC_SHOT2) { RPi_ReqShot2(); }   /* ★ROUGH STACK：$SHOT2 */
            else
            {
                RPi_ReqRcp();                                    /* 转盘区：$RCP（xy） */
#if (ALIGN_YAW_RCP != 0U)
                RPi_ReqRp();                                     /* ★转盘平行度：$RP（q1） */
#endif
            }
        }
    }

    /* ---- 时间预算兜底：设了预算就必须在预算内收工（转盘只停几秒时靠这个）----
       到点判据：误差已进"放宽后的死区" -> 判成功；否则**判失败**（绝不假装成功）。
       放宽是渐进的（见 Al_EffDeadband），所以多数情况会提前一点点收工。 */
    if ((alBudgetMs != 0U) && (alState != AL_IDLE) && (alState != AL_DONE) &&
        (alState != AL_FAILED))
    {
        uint32_t el = now - alBudgetTick;

        if (el >= alBudgetMs)
        {
            Align_StopWheels();

            if (Al_Aligned(tt) != 0U)
            {
                alState = AL_DONE;
                Al_OnAligned();     /* ★成功：解冻坐标 + 开始钉住本任务点的参考 yaw */
                Debug_Log("ALN", "done by BUDGET %u ms: px=%d,%d tol=%d iters=%u ticks=%u",
                          (unsigned)alBudgetMs, (int)tt->dxPx, (int)tt->dyPx,
                          (int)Al_EffDeadband(), (unsigned)alIter, (unsigned)alTickCnt);
            }
            else
            {
                alState = AL_FAILED;
                Al_ReleaseOdom();   /* ★解冻坐标 */
                Debug_Log("ALN", "fail: BUDGET %u ms used up, px=%d,%d still > tol %d",
                          (unsigned)alBudgetMs, (int)tt->dxPx, (int)tt->dyPx,
                          (int)Al_EffDeadband());
            }
            return;
        }
    }

    /* 【2026-10-10】原"末端放宽出口"（偏差不再变小就当到位）已删除 ——
       现在**只有** AL_READ 里"真正进死区"这一条成功判据（一步到位）。 */

    switch (alState)
    {
        case AL_YAW:
        {
            /* ★车身矫正（仅 $SHOT）：**原地**按 |q| 转到平行，再交给 XY 平移段。
               节拍与 XY 段一致：末端 |q| 小 -> 换短节拍，每拍转得更少、停得准。 */
            int32_t  qAbs   = (tt->qAvg < 0) ? -(int32_t)tt->qAvg : (int32_t)tt->qAvg;
            uint32_t tickMs = (qAbs <= (int32_t)ALIGN_YAW_PX_MIN)
                              ? (uint32_t)ALIGN_TICK_FINE_MS : (uint32_t)ALIGN_TICK_MS;

            /* ★安全阀：转正超时（可能 ALIGN_YAW_SIGN 标反 / q 收敛不了）-> 停轮 + 失败，
               否则"不限时 + 原地旋转"会让车一直打转（见 bsp_align.h 的 ALIGN_YAW_MAX_MS）。 */
#if (ALIGN_YAW_MAX_MS != 0U)
            if ((now - alYawTick) > (uint32_t)ALIGN_YAW_MAX_MS)
            {
                Align_StopWheels();
                Debug_Log("ALN", "yaw TIMEOUT %u ms, q=%d -> fail",
                          (unsigned)ALIGN_YAW_MAX_MS, (int)tt->qAvg);
                Align_Fail();
                return;
            }
#endif

            if ((now - alTick) < tickMs)
            {
                return;
            }
            alTick = now;

            /* 帧看门狗（与 AL_RUN 同一套：帧时间戳变了 = 收到新数据） */
            if (tt->lastRxTick != alLastSeen)
            {
                alLastSeen     = tt->lastRxTick;
                alNoDataTick   = now;
                alNoDataLogged = 0U;
                Align_MarkFrame(now);
            }
#if (ALIGN_NO_DATA_FAIL_MS != 0U)
            if ((now - alNoDataTick) > ALIGN_NO_DATA_FAIL_MS)
            {
                Align_StopWheels();
                Align_Fail();
                return;
            }
#endif
            /* 数据无效/过期 -> 软停 + 计数，留在本态（下一拍拿到新帧就继续转） */
            if ((tt->valid == 0U) || ((now - tt->lastRxTick) > Align_DataLimit()))
            {
                Align_StopWheels();
                alHoldCnt++;
                alProgTick = now;   /* 等数据的时间不算"卡住" */
                if (alNoDataLogged == 0U)
                {
                    alNoDataLogged = 1U;
                    Debug_Log("ALN", "yaw data stale (valid=%u), hold #%u",
                              (unsigned)tt->valid, (unsigned)alHoldCnt);
                }
                return;
            }

            Align_YawPhase(tt, now, tickMs);
            return;
        }

        case AL_RUN:
        {
            /* 节拍：末端自动切短节拍（每拍位移更小 -> 停得准、不一顿一顿） */
            uint32_t tickMs = (Align_InSlowZone(alLastPx, alLastPy) != 0U)
                              ? (uint32_t)ALIGN_TICK_FINE_MS : (uint32_t)ALIGN_TICK_MS;

            if ((now - alTick) < tickMs)
            {
                return;
            }
            alTick = now;

            /* 看门狗：帧时间戳变了 = 收到新数据 */
            if (tt->lastRxTick != alLastSeen)
            {
                alLastSeen     = tt->lastRxTick;
                alNoDataTick   = now;
                alNoDataLogged = 0U;
                Align_MarkFrame(now);
            }
#if (ALIGN_NO_DATA_FAIL_MS != 0U)
            if ((now - alNoDataTick) > ALIGN_NO_DATA_FAIL_MS)
            {
                Align_StopWheels();
                Align_Fail();
                return;
            }
#endif
            /* 数据无效/过期：**软停 + 计数**，但**留在 AL_RUN**（不切状态、不等停）——
               下一拍一旦拿到新偏移就立刻继续走。如果切到 AL_READ，就要多等
               "比本阶段起点更新的一帧"，视觉稍慢就变成走一段停一段。
               速度段用比通用过期判据更严的 ALIGN_VEL_DATA_MS（并按实测帧率自动放宽）：
               连续运动时按旧数据换算出来的速度会一路开过去，必须尽快停。 */
            if ((tt->valid == 0U) ||
                ((now - tt->lastRxTick) > Align_DataLimit()))
            {
                Align_StopWheels();
                alHoldCnt++;
                alProgTick = now;   /* 等数据的时间不算"卡住" */
                if (alNoDataLogged == 0U)
                {
                    alNoDataLogged = 1U;
                    Debug_Log("ALN", "data stale (valid=%u fper=%u limit=%u), hold #%u",
                              (unsigned)tt->valid, (unsigned)alFramePeriod,
                              (unsigned)Align_DataLimit(), (unsigned)alHoldCnt);
                }
                return;
            }

            Align_RunPhase(tt, now);
            return;
        }

        case AL_READ:
            /* 【无限迭代】不再有"迭代用尽"和"总超时"：一直纠到进死区为止。
               唯一兜底：连续 ALIGN_NO_DATA_FAIL_MS 一个**新**偏移帧都收不到
               （车跑出视野/视觉端停了），继续等没意义 -> 报错停下。
               ALIGN_NO_DATA_FAIL_MS 设 0 就是永不失败、死等。 */

            /* 看门狗：帧时间戳变了 = 收到新数据 */
            if (tt->lastRxTick != alLastSeen)
            {
                alLastSeen     = tt->lastRxTick;
                alNoDataTick   = now;
                alNoDataLogged = 0U;
                Align_MarkFrame(now);
            }
#if (ALIGN_NO_DATA_FAIL_MS != 0U)
            if ((now - alNoDataTick) > ALIGN_NO_DATA_FAIL_MS)
            {
                Align_Fail();
                return;
            }
#endif
            /* 数据还没到，或到的还是"本阶段开始之前"的旧帧 -> 继续等（只限流打一条提示） */
            if ((tt->valid == 0U) || (Align_IsNewer(tt->lastRxTick) == 0U))
            {
                if ((alNoDataLogged == 0U) && ((now - alPhaseTick) > ALIGN_FRAME_TIMEOUT_MS))
                {
                    alNoDataLogged = 1U;
                    Debug_Log("ALN", "waiting data (valid=%u) ...", (unsigned)tt->valid);
                }
                return;
            }

            alLastPx = tt->dxPx;
            alLastPy = tt->dyPx;

            /* 已达死区 -> 成功（误差在 RPi_GetCfg()->deadbandPx 以内，见 RPI_PX_DEADBAND） */
            if (Al_Aligned(tt) != 0U)
            {
                /* ★两轮制【2026-10-10 晚 用户要求"一次靠近、第二次精准"】：
                   第 1 轮（粗靠近）达成 -> **立刻再跑一轮做精定位**；
                   第 2 轮（或 ALIGN_PASS_COUNT=1 / $RCP 源）达成 -> 才真正判收敛。 */
                if ((ALIGN_PASS_COUNT > 1U) && AL_IS_SHOT_SRC() &&
                    (alPass < (uint8_t)ALIGN_PASS_COUNT))
                {
                    alPass++;
                    alIter    = 0U;
                    alTickCnt = 0U;
                    alHoldCnt = 0U;
                    alRpmBase = 0;
                    Align_StopWheels();
#if (ALIGN_PID_ENABLE != 0U)
                    Al_PidResetAll(now);    /* ★新一轮从干净的 PID 状态开始（旧的积分/微分不带到下一轮） */
#endif
                    alPhaseTick = now;
                    alYawTick   = now;      /* 第 2 轮车身矫正的安全阀重新计时 */
                    /* ★【2026-10-15】`ALIGN_YAW_SIMUL=1`：不单独做"先转正"这一段，
       直接进 XY，由 `Align_RunPhase` 每帧同时算旋转轴（三轴一起收敛）。 */
#if (ALIGN_YAW_SIMUL != 0U)
    alState = AL_READ;
#else
    alState = (alYawOn != 0U) ? AL_YAW : AL_READ;
#endif
                    Debug_Log("ALN", "pass%u close px=%d,%d -> pass%u fine (db=%dpx)",
                              (unsigned int)(alPass - 1U), (int)alLastPx, (int)alLastPy,
                              (unsigned int)alPass, (int)Al_EffDeadband());
                    return;
                }

                alState   = AL_DONE;
                alRpmBase = 0;
                /* ★对位成功收尾：解冻平移坐标（对位期间冻结，见 Al_Begin）
                   + 按本任务点登记的参考角**开始钉住 yaw**（见 Al_OnAligned）。 */
                Al_OnAligned();
                Debug_Log("ALN", "aligned px=%d,%d runs=%u ticks=%u hold=%u pass=%u",
                          (int)alLastPx, (int)alLastPy,
                          (unsigned)alIter, (unsigned)alTickCnt, (unsigned)alHoldCnt,
                          (unsigned int)alPass);
                return;
            }

            /* 还没进死区 -> **回速度段继续追**（全程速度模式，一轴一轴消）。
               从 0 起步 -> 软起步；把卡住检测的基准一并重置，否则会误判成"卡住"。 */
            alRpmBase    = 0;
            alRunLogTick = now - 500U;   /* 进速度段的第一拍就补一条日志 */
            alTick       = now;
            alProgTick   = now;
            alProgPx     = 32767;
            alBoost      = 0U;
            alIter++;
            alPhaseTick  = now;
            alState      = AL_RUN;
            return;

        case AL_SETTLE:
        {
            /* 静置：等"车确实停住 + 视觉拍到停住后的那一帧"。
               取实测帧间隔作为等待量（上限 ALIGN_SETTLE_MS，下限 40ms）：
               位置命令本身停得很准，无需固定等满 100ms，等太久就是可见的停顿。 */
            uint32_t settleMs = alFramePeriod;

            if (settleMs < 40U) { settleMs = 40U; }
            if (settleMs > (uint32_t)ALIGN_SETTLE_MS) { settleMs = (uint32_t)ALIGN_SETTLE_MS; }

            if ((now - alPhaseTick) >= settleMs)
            {
                /* 关键：把本阶段起点设为"现在"，下一轮 AL_READ 只认比现在更新的帧 */
                alPhaseTick = now;
                alState     = AL_READ;
            }
            return;
        }

        default:
            return;
    }
}

uint8_t Align_IsBusy(void)
{
    return ((alState == AL_YAW) || (alState == AL_RUN) || (alState == AL_READ) ||
            (alState == AL_SETTLE)) ? 1U : 0U;
}

/**
 * @brief  ★【2026-10-15】取调参用调试快照
 * @param  out: 输出（调用方提供；空指针直接返回）
 * @retval 无
 * @note   只读拷贝，不改任何控制状态。字段含义见 bsp_align.h 的 `Align_Dbg_t`。
 */
void Align_GetDebug(Align_Dbg_t *out)
{
    if (out == 0)
    {
        return;
    }
    *out = alDbg;
}

uint8_t Align_IsDone(void)
{
    return (alState == AL_DONE) ? 1U : 0U;
}

uint8_t Align_IsFailed(void)
{
    return (alState == AL_FAILED) ? 1U : 0U;
}

void Align_Abort(void)
{
    if (Align_IsBusy() != 0U)
    {
        /* 中止用**急停**（0xFE）：速度段可能正在动，必须立刻停住 */
        Mecanum_Stop();
        Debug_Log("ALN", "abort at iter%u tick=%u hold=%u",
                  (unsigned)alIter, (unsigned)alTickCnt, (unsigned)alHoldCnt);
    }
    alRpmBase = 0;
    alState = AL_IDLE;
    Al_ReleaseOdom();           /* ★中止：解冻平移坐标（对位期间是冻结的，见 Al_Begin） */
}

/**
 * @brief  获取对位状态文本（树莓派测试页第 4 行）
 * @param  无
 * @retval 静态缓冲指针，如 "V10 xy345,-267"
 * @note   【2026-10-09 改】第 4 行改显示**车体绝对坐标**（里程计 x,y），
 *         方便现场看"对位把车挪到哪了" —— 对位过程中坐标一直在算
 *         （main.c 每拍无条件调 Chassis_OdomUpdate，与谁在下发命令无关）。
 *         标签 `xy` 用来和**像素偏移**区分（像素在页面第 2 行 `TT:<dx>,<dy>`，
 *         对位的正是它，所以这里不再重复显示）。
 *         前缀：`V<rpm>` = 速度段（顺带看一眼当前速度档，1..ALIGN_XY_RPM_MAX）；
 *              `A<n>` = 停轮复测中（n = 已跑段数）；`AOK` / `AFAIL`。
 *         宽度：8x16 字体一行 16 字符，坐标到 4 位数时刚好占满，再长会被截断。
 */
const char *Align_GetStateText(void)
{
    static char buf[24];
    const char *tag;

    if (alState == AL_FAILED)
    {
        return "AFAIL nodata";
    }

    if ((alState != AL_YAW) && (alState != AL_RUN) && (alState != AL_READ) &&
        (alState != AL_SETTLE) && (alState != AL_DONE))
    {
        return "ALGN IDLE";
    }

    if (alState == AL_YAW)
    {
        tag = "Y";      /* ★车身矫正：原地按 q 转正 */
    }
    else if (alState == AL_RUN)
    {
        tag = "V";      /* 速度段 */
    }
    else if (alState == AL_DONE)
    {
        tag = "AOK";    /* 已收敛 */
    }
    else
    {
        tag = "A";      /* 停轮 / 复测中 */
    }

    (void)snprintf(buf, sizeof(buf), "%s%ld xy%d,%d",
                   tag,
                   ((alState == AL_RUN) || (alState == AL_YAW)) ? (long)alRpmBase : (long)alIter,
                   (int)Chassis_GetOdomX(), (int)Chassis_GetOdomY());
    return buf;
}
