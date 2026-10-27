/**
 ******************************************************************************
 * @file    bsp_arm.c
 * @brief   机械臂两个电机驱动实现（P8：电机5 手臂转向 / 电机6 升降丝杆）
 * @author  nx_car
 * @date    2026-09-18
 ******************************************************************************
 * @note   设计要点：
 *   1. 本模块只**读** bsp_Emm_V5.c 里的状态结构体（M5/M6 的位置/到位/回零），
 *      不改 CAN 回调；
 *   2. 这些状态必须靠 Arm_Update() 周期发 S_CPOS / S_FLAG / S_OFLAG 查询才会刷新，
 *      漏查询会导致"到位/回零完成"永远判不出来；
 *   3. 下发前统一等一个空闲发送邮箱（Arm_TxSlotWait），避免 CAN 帧被静默丢弃
 *      （原因见 bsp_motor.c 的 MOTOR_TX_SLOT_WAIT_MS）；
 *   4. 位置统一换算到"下发脉冲域"（上报值 ÷ 20），与绝对位置命令的 clk 同一量纲。
 *
 *   硬约束：全部非阻塞（只发帧不等应答），禁止 HAL_Delay；日志禁 %f。
 ******************************************************************************
 */
#include "bsp_arm.h"
#include "bsp_Emm_V5.h"    /* Emm_V5_Pos_Status / SUCCESSRECEIVE / FAILRECEIVE */
#include "can.h"           /* hcan：用于等空闲发送邮箱 */
#include "bsp_debug.h"
#include "LobotServoController.h"   /* 官方舵机库：moveServo（经调试板 TTL 口） */
#include "bsp_servo.h"              /* 回读模块：通知"刚发过运动指令"，错开读请求 */

/* ===================== 内部状态 ===================== */

static uint8_t  armQueryIdx  = 0U;      /* 查询轮转下标 0~5（见 Arm_Update） */
static uint32_t armQueryTick = 0U;      /* 上次查询时刻 */

/* "位置快查"目标地址：0 = 关闭（正常 6 拍轮转）；5/6 = 只快查该电机的状态。
 * 动作序列用它把"位置/到位标志的回读周期"从 120ms 降到 ARM_QUERY_PERIOD_MS(20ms)，
 * 见 bsp_arm.h 的 Arm_SetFastPoll 说明。
 * ★ 快查时 **位置(S_CPOS) 与 到位标志(S_FLAG) 交替发**（见 Arm_Update）：
 *   只发位置的话，"到位标志(0x3A)"永不刷新 -> `Arm_IsReachDone()` 恒为 0 ->
 *   动作步只剩"位置容差"一条判据，进不了容差就干等 ARM_MOVE_TIMEOUT_MS(15s) 才 ABORT
 *   —— 现象正是"**手臂卡住不动、过好久才动**"。 */
static uint8_t  armFastPollAddr = 0U;
static uint8_t  armFastPollIdx  = 0U;   /* 0=本拍发 S_CPOS(位置)；1=本拍发 S_FLAG(到位标志) */

static uint8_t  armHomeActive   = 0U;   /* 1=正在等回零完成（用于超时判定） */
static uint8_t  armHomeAddr     = 0U;   /* 正在回零的电机地址 */
static uint32_t armHomeStartTick = 0U;  /* 回零触发时刻 */

static uint32_t armTxBusyLogTick = 0U;  /* ★"位置命令凑不到 2 个空邮箱"日志限流（1 条/秒） */

/* 三个舵机最后一次下发的目标位置，0xFFFF = 本次上电还没下发过。
 * 下标 0 = SERVO_ID_CLAW(1)，下标 1 = SERVO_ID_GRIP(2)，下标 2 = SERVO_ID_TURNTABLE(3)。
 * 官方库没有位置回读，所以这是"我们让它去哪"，不是舵机实际角度。 */
static uint16_t armServoLastPos[3] = { 0xFFFFU, 0xFFFFU, 0xFFFFU };

/* ===================== 内部工具 ===================== */

/**
 * @brief  地址合法性检查
 * @param  addr: 电机地址
 * @retval 1=合法（5 或 6）；0=非法
 */
static uint8_t Arm_AddrValid(uint8_t addr)
{
    return ((addr == ARM_ADDR_TURN) || (addr == ARM_ADDR_LIFT)) ? 1U : 0U;
}

/**
 * @brief  取电机名字（日志用）
 * @param  addr: 电机地址
 * @retval "turn" / "lift"
 */
static const char *Arm_Name(uint8_t addr)
{
    return (addr == ARM_ADDR_TURN) ? "turn" : "lift";
}

/**
 * @brief  下发 CAN 命令前，确保发送邮箱可用数达到 need，带超时上限
 * @param  need     : 需要的空闲邮箱个数（短命令=1；0xFD 位置命令=**2**）
 * @param  timeoutMs: 等待上限，单位 ms
 * @retval 1=邮箱已够（或等待后够了）；0=超时仍不够（调用方**不要发**，否则会被静默丢弃）
 * @note   【为什么按个数等】STM32F1 只有 3 个发送邮箱，官方 `can_SendCmd` 对 >8 字节的命令
 *         会**拆成 2 帧**发送，而它调 `HAL_CAN_AddTxMessage` **不检查返回值** ——
 *         邮箱不够时后一包被**静默丢弃** ⇒ 电机只收到半条命令 ⇒ **整条作废、该关节不动**。
 *         ★【2026-10-15】手臂这边原来只等 **1** 个邮箱（`Arm_TxSlotWait`），而位置命令要 **2** 个
 *         —— 这正是"某个关节指令丢失"的一条硬根因（轮子那边 `bsp_motor` 早就按 2 修了）。
 *         一帧约 0.26ms @500kbps，正常几乎不等待。
 */
static uint8_t Arm_TxMailboxWait(uint8_t need, uint32_t timeoutMs)
{
    uint32_t t0;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) >= (uint32_t)need)
    {
        return 1U;
    }

    t0 = HAL_GetTick();
    while ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan) < (uint32_t)need) &&
           ((HAL_GetTick() - t0) < timeoutMs))
    {
        /* 忙等，有上限；此处不发日志（避免 UART 再拖时间） */
    }

    return (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) >= (uint32_t)need) ? 1U : 0U;
}

/**
 * @brief  短命令（≤8 字节，1 帧）下发前等一个空闲发送邮箱
 * @param  无
 * @retval 无
 * @note   读参数/使能/停止/回零这类命令都是 1 帧，等 1 个空邮箱即可。
 *         **位置模式命令（0xFD）必须用 `Arm_TxMailboxWait(2, ARM_TX_PAIR_WAIT_MS)`。**
 */
static void Arm_TxSlotWait(void)
{
    (void)Arm_TxMailboxWait(1U, (uint32_t)ARM_TX_SLOT_WAIT_MS);
}

/**
 * @brief  取某电机的到位应答状态字指针（0x3A 帧回传置位）
 * @param  addr: 电机地址 5/6
 * @retval 对应状态字指针；地址非法返回 NULL
 */
static __IO uint16_t *Arm_ReachStatusPtr(uint8_t addr)
{
    if (addr == ARM_ADDR_TURN)
    {
        return &Emm_V5_Pos_Status.M5_Reach_Status;
    }
    if (addr == ARM_ADDR_LIFT)
    {
        return &Emm_V5_Pos_Status.M6_Reach_Status;
    }
    return NULL;
}

/**
 * @brief  取某电机的回零完成状态字指针（0x3B 帧回传置位）
 * @param  addr: 电机地址 5/6
 * @retval 对应状态字指针；地址非法返回 NULL
 */
static __IO uint16_t *Arm_ZeroStatusPtr(uint8_t addr)
{
    if (addr == ARM_ADDR_TURN)
    {
        return &Emm_V5_Pos_Status.M5_Zero_Reach_Status;
    }
    if (addr == ARM_ADDR_LIFT)
    {
        return &Emm_V5_Pos_Status.M6_Zero_Reach_Status;
    }
    return NULL;
}

/**
 * @brief  解析某电机的位置应答（只读缓冲，不发 CAN 帧）
 * @param  addr: 电机地址 5/6
 * @param  out : 输出位置，单位下发脉冲域（已按 ARM_READ_SCALE_NUM/DEN 换算）
 * @retval 1=解析成功；0=无应答 / 帧头不符 / 地址非法
 * @note   0x36 位置帧 7 字节布局（与 bsp_motor.c 的解析一致）：
 *           buf[0]=0x36 帧头，buf[1]=符号(0=正,非0=负)，
 *           buf[2..5]=位置大端 32 位，buf[6]=0x6B 校验字节。
 *         上报的是编码器计数（约下发脉冲的 20 倍），所以必须换算。
 */
static uint8_t Arm_ParsePos(uint8_t addr, int32_t *out)
{
    __IO uint8_t  *pBuf;
    __IO uint16_t *pStatus;
    uint8_t  buf[7];
    uint8_t  i;
    uint32_t valU;
    int32_t  val;

    if (addr == ARM_ADDR_TURN)
    {
        pBuf    = Emm_V5_Pos_Status.M5_EncodedBuff;
        pStatus = &Emm_V5_Pos_Status.M5_EncodedStatus;
    }
    else if (addr == ARM_ADDR_LIFT)
    {
        pBuf    = Emm_V5_Pos_Status.M6_EncodedBuff;
        pStatus = &Emm_V5_Pos_Status.M6_EncodedStatus;
    }
    else
    {
        return 0U;
    }

    /* 必须成功收到过 0x36 位置应答 */
    if (*pStatus != SUCCESSRECEIVE)
    {
        return 0U;
    }

    /* 逐字节快照（避免读 volatile 数组被中断撕裂） */
    for (i = 0U; i < 7U; i++)
    {
        buf[i] = pBuf[i];
    }

    if (buf[0] != 0x36U)
    {
        return 0U;
    }

    /* 大端 32 位拼装 */
    valU  = ((uint32_t)buf[2] << 24);
    valU |= ((uint32_t)buf[3] << 16);
    valU |= ((uint32_t)buf[4] << 8);
    valU |=  (uint32_t)buf[5];
    val  = (int32_t)valU;

    if (buf[1] != 0U)
    {
        val = -val;
    }

    /* 上报域 -> 下发脉冲域 */
    *out = (val * (int32_t)ARM_READ_SCALE_NUM) / (int32_t)ARM_READ_SCALE_DEN;

    return 1U;
}

/**
 * @brief  取"位置增大方向"对应的 dir 值
 * @param  addr: 电机地址 5/6
 * @retval dir（0=CW / 1=CCW）
 * @note   电机6 的"坐标增大方向"就是 ARM_LIFT_DIR_UP（**实测标定结果**，
 *         它同时是回零方向）；反了就互换 bsp_arm.h 里那两个宏的值。
 */
static uint8_t Arm_DirInc(uint8_t addr)
{
    return (addr == ARM_ADDR_TURN) ? (uint8_t)ARM_TURN_DIR_A : (uint8_t)ARM_LIFT_DIR_UP;
}

/**
 * @brief  取"位置减小方向"对应的 dir 值
 * @param  addr: 电机地址 5/6
 * @retval dir（0=CW / 1=CCW）
 */
static uint8_t Arm_DirDec(uint8_t addr)
{
    return (addr == ARM_ADDR_TURN) ? (uint8_t)ARM_TURN_DIR_B : (uint8_t)ARM_LIFT_DIR_DOWN;
}

/**
 * @brief  速度限幅
 * @param  vel: 期望速度 RPM
 * @retval 限幅后的速度（0~ARM_VEL_MAX）
 */
static uint16_t Arm_ClampVel(uint16_t vel)
{
    return (vel > (uint16_t)ARM_VEL_MAX) ? (uint16_t)ARM_VEL_MAX : vel;
}

/**
 * @brief  加速度档限幅
 * @param  acc: 期望加减速档
 * @retval 限幅后的档位（0~ARM_ACC_MAX）
 */
static uint8_t Arm_ClampAcc(uint8_t acc)
{
    return (acc > (uint8_t)ARM_ACC_MAX) ? (uint8_t)ARM_ACC_MAX : acc;
}

/* ===================== 公开接口 ===================== */

/**
 * @brief  机械臂电机模块初始化（只复位状态，不发使能/动作指令）
 * @param  无
 * @retval 无
 * @note   CAN 状态结构体的清零由 Motor_Init 里的 Emm_V5_Pos_Status_Read_Init 完成
 *         （上电顺序：Motor_Init 在本函数之前），此处不重复清零，
 *         避免把已经收到的应答抹掉。
 */
void Arm_Init(void)
{
    armQueryIdx      = 0U;
    armQueryTick     = 0U;
    armFastPollAddr  = 0U;
    armFastPollIdx   = 0U;
    armHomeActive    = 0U;
    armHomeAddr      = 0U;
    armHomeStartTick = 0U;

    Debug_Log("ARM", "init, turn=%u lift=%u q=%ums",
              (unsigned int)ARM_ADDR_TURN, (unsigned int)ARM_ADDR_LIFT,
              (unsigned int)ARM_QUERY_PERIOD_MS);
}

/**
 * @brief  状态周期查询 + 回零完成判定（主循环调用，非阻塞）
 * @param  无
 * @retval 无
 * @note   六拍一轮，每拍只发一条查询，目的是把 CAN 负载压到最低：
 *           0: 地址5 读位置(S_CPOS  -> 0x36 帧)
 *           1: 地址6 读位置
 *           2: 地址5 读到位标志(S_FLAG -> 0x3A 帧)
 *           3: 地址6 读到位标志
 *           4: 地址5 读回零标志(S_OFLAG -> 0x3B 帧)
 *           5: 地址6 读回零标志
 *         故单个状态的刷新周期 = 6 × ARM_QUERY_PERIOD_MS。
 */
void Arm_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* ★【2026-10-23】舵机"命令发了但完全没动"的兜底：复查回读，真没动就补发 */
    Arm_ServoSvc();

    /* ---- 回零完成 / 超时判定（每拍都查，不依赖查询节拍） ---- */
    if (armHomeActive != 0U)
    {
        if (Arm_IsHomeDone(armHomeAddr) != 0U)
        {
            armHomeActive = 0U;
            Debug_Log("ARM", "%s home done, pos=%d",
                      Arm_Name(armHomeAddr), (int)Arm_GetPos(armHomeAddr));
        }
        else if ((now - armHomeStartTick) >= (uint32_t)ARM_HOME_TIMEOUT_MS)
        {
            armHomeActive = 0U;
            Debug_Log("ARM", "%s home TIMEOUT, check Response/SL_MA/OM_TM",
                      Arm_Name(armHomeAddr));
        }
        else
        {
            /* 还在回零中 */
        }
    }

    /* ---- 周期查询 ---- */
    if ((now - armQueryTick) < (uint32_t)ARM_QUERY_PERIOD_MS)
    {
        return;
    }
    armQueryTick = now;

    /* ---- 位置快查：正在做动作的那个电机占满全部查询拍 ----
     * 仍然**一拍只发一条 CAN 帧**（负载与轮转一样），但把该电机的状态刷新
     * 周期从 6×ARM_QUERY_PERIOD_MS(120ms) 压到 2×ARM_QUERY_PERIOD_MS(40ms)。
     * ★ **位置与到位标志交替发**：只发位置会让 0x3A 到位标志永不刷新，
     *   `Arm_IsReachDone()` 恒为 0，电机步就只剩"位置容差"判据 —— 容差进不去
     *   就干等 ARM_MOVE_TIMEOUT_MS(15s) 才 ABORT（现象：手臂卡住不动、过好久才动）。 */
    if (armFastPollAddr != 0U)
    {
        if (armFastPollIdx == 0U)
        {
            Emm_V5_Read_Sys_Params(armFastPollAddr, S_CPOS);   /* 位置（0x36） */
            armFastPollIdx = 1U;
        }
        else
        {
            Emm_V5_Read_Sys_Params(armFastPollAddr, S_FLAG);   /* 到位标志（0x3A） */
            armFastPollIdx = 0U;
        }
        return;
    }

    switch (armQueryIdx)
    {
        case 0U: Emm_V5_Read_Sys_Params(ARM_ADDR_TURN, S_CPOS);  break;
        case 1U: Emm_V5_Read_Sys_Params(ARM_ADDR_LIFT, S_CPOS);  break;
        case 2U: Emm_V5_Read_Sys_Params(ARM_ADDR_TURN, S_FLAG);  break;
        case 3U: Emm_V5_Read_Sys_Params(ARM_ADDR_LIFT, S_FLAG);  break;
        case 4U: Emm_V5_Read_Sys_Params(ARM_ADDR_TURN, S_OFLAG); break;
        default: Emm_V5_Read_Sys_Params(ARM_ADDR_LIFT, S_OFLAG); break;
    }

    armQueryIdx++;
    if (armQueryIdx >= 6U)
    {
        armQueryIdx = 0U;
    }
}

/**
 * @brief  使能 / 失能电机
 * @param  addr: 电机地址 5/6
 * @param  on  : 1=使能；0=失能
 * @retval 无
 */
void Arm_Enable(uint8_t addr, uint8_t on)
{
    if (Arm_AddrValid(addr) == 0U)
    {
        return;
    }

    Arm_TxSlotWait();
    Emm_V5_En_Control(addr, (on != 0U) ? true : false, false);

    Debug_Log("ARM", "%s en=%u", Arm_Name(addr), (unsigned int)((on != 0U) ? 1U : 0U));
}

/**
 * @brief  立即停止（0xFE 急停）
 * @param  addr: 电机地址 5/6
 * @retval 无
 */
void Arm_Stop(uint8_t addr)
{
    if (Arm_AddrValid(addr) == 0U)
    {
        return;
    }

    Arm_TxSlotWait();
    Emm_V5_Stop_Now(addr, false);

    Debug_Log("ARM", "%s stop", Arm_Name(addr));
}

/**
 * @brief  运动到**目标坐标**（0xFD 位置模式，`raF=1` **绝对位置**，两轴都用）
 * @param  addr     : 电机地址 5/6
 * @param  abs_pulse: 目标坐标，单位显示/标定域，**可为负**（M5 以 0 点为中心、M6 向下为负）
 * @param  vel      : 速度 RPM（内部限幅）
 * @param  acc      : 加减速档（内部限幅）
 * @param  dirMode  : 方向：ARM_DIR_AUTO 按坐标符号自动（**推荐**）；
 *                    ARM_DIR_A / ARM_DIR_B 强制方向，仅调试用
 * @retval 无
 * @note   ★【2026-10-24 用户要求】**电机6 也改成绝对位置**（原来只有 M5 是绝对，
 *         M6 还在"用回读位置算增量再走"）——不再靠算偏差，也就不再浪费一个回读周期。
 *         为什么绝对模式更稳：相对命令(raF=2)的 clk 是"目标 − 回读位置"的**增量**，
 *         而回读永远慢一拍；丢帧看门狗 / 严格闭环纠正 / 后台精修都会**重发同一条增量**，
 *         只要重发时回读没刷新，驱动器就在真实位置上**再叠加一次** → 越走越偏。
 *         绝对模式发的是**目标坐标**（幂等），发几次都落在同一坐标。
 *
 *         【坐标换算】命令 clk 是"驱动器脉冲域"（20.48 计数=1脉冲），
 *         而动作表/页面用的是"显示域"（上报值/20），两者差 2.4%：
 *         `drv = disp × 125/128`（带符号四舍五入），否则每一步都会多走 2.4%。
 *         **绝对目标按"幅值 + dir"下发**（clk 是无符号脉冲数，负数靠 dir 选方向，
 *         与相对模式同一约定；把负目标强转成 uint32 会变成 42 亿步）。
 *         **不再需要当前位置**（不需要算 delta）——位置读不到也能下发。
 */
void Arm_MoveAbsDir(uint8_t addr, int32_t abs_pulse, uint16_t vel, uint8_t acc, uint8_t dirMode)
{
    int32_t  drvTgt;
    uint32_t clkAbs;
    uint8_t  dir;
    uint16_t v;
    uint8_t  a;

    if (Arm_AddrValid(addr) == 0U)
    {
        Debug_Log("ARM", "moveabs bad addr=%u", (unsigned int)addr);
        return;
    }

    /* 显示域 -> 驱动器脉冲域（真比值 20.48，见 bsp_arm.h）：带符号四舍五入 */
    if (abs_pulse >= 0)
    {
        drvTgt = (abs_pulse * (int32_t)ARM_DISP_TO_PULSE_NUM +
                  ((int32_t)ARM_DISP_TO_PULSE_DEN / 2)) / (int32_t)ARM_DISP_TO_PULSE_DEN;
    }
    else
    {
        drvTgt = -(((-abs_pulse) * (int32_t)ARM_DISP_TO_PULSE_NUM +
                    ((int32_t)ARM_DISP_TO_PULSE_DEN / 2)) / (int32_t)ARM_DISP_TO_PULSE_DEN);
    }

    /* ★★【2026-10-24 修】绝对模式也必须按"**幅值 + dir**"下发（与相对模式同一约定）：
       0xFD 的 clk 是**无符号脉冲数**，方向由 dir 单独给。
       把负目标直接强转成 uint32（补码）会变成约 42 亿步 —— 驱动器会朝那个方向一直走
       （现场现象：**一执行任务 M6 就直接往下扎**，因为观察位/车载位都是负坐标）。
       → 所以这里取 |drvTgt| 作幅值，符号只用来选 dir。 */
    clkAbs = (drvTgt >= 0) ? (uint32_t)drvTgt : (uint32_t)(-drvTgt);

    /* dir：ARM_DIR_A/B 是调试用的强制方向；AUTO 时按目标坐标的符号选 */
    if (dirMode == (uint8_t)ARM_DIR_A)
    {
        dir = Arm_DirInc(addr);
    }
    else if (dirMode == (uint8_t)ARM_DIR_B)
    {
        dir = Arm_DirDec(addr);
    }
    else
    {
        dir = (drvTgt >= 0) ? Arm_DirInc(addr) : Arm_DirDec(addr);
    }

    v = Arm_ClampVel(vel);
    a = Arm_ClampAcc(acc);

    /* 关键顺序：先清到位标志，再下发（否则上一段的到位标志会被当成段到位） */
    Arm_ReachClear(addr);

    /* ★★【2026-10-15 严禁丢指令】0xFD 有 9 字节负载，官方库会**拆成 2 帧**发，
       必须**凑够 2 个空邮箱**再发：只剩 1 个时后一包会被 `HAL_CAN_AddTxMessage`
       **静默丢弃** ⇒ 电机只收到半条命令 ⇒ **该关节这一步完全不动**。
       凑不够就**本拍不发、直接返回**（绝不发半条）；动作层会按"位置没动"再次调用
       （严格闭环的停稳校验 / 容错档的丢帧看门狗都会重发）—— 所以**不会被永久丢掉**。 */
    if (Arm_TxMailboxWait(2U, (uint32_t)ARM_TX_PAIR_WAIT_MS) == 0U)
    {
        uint32_t nowMs = HAL_GetTick();

        if ((nowMs - armTxBusyLogTick) >= 1000U)   /* 限流：最多 1 条/秒 */
        {
            armTxBusyLogTick = nowMs;
            Debug_Log("ARM", "%s moveabs tx busy (need 2 mbx) -> deferred, will retry",
                      Arm_Name(addr));
        }
        return;
    }

    /* raF=1：**绝对位置**（目标坐标，不是增量）—— 重复下发幂等，不会再叠加走偏 */
    Emm_V5_Pos_Control(addr, dir, v, a, clkAbs, 1U, false);

    Debug_Log("ARM", "%s abs tgt=%ld clk=%lu dir=%u",
              Arm_Name(addr), (long)drvTgt, (unsigned long)clkAbs, (unsigned int)dir);
}

/**
 * @brief  读当前位置（只解析接收缓冲）
 * @param  addr: 电机地址 5/6
 * @retval 位置（下发脉冲域）；读失败返回 0
 */
int32_t Arm_GetPos(uint8_t addr)
{
    int32_t pos = 0;

    if (Arm_ParsePos(addr, &pos) == 0U)
    {
        return 0;
    }
    return pos;
}

/**
 * @brief  查询当前位置是否有效
 * @param  addr: 电机地址 5/6
 * @retval 1=有效；0=还没读到过
 */
uint8_t Arm_IsPosValid(uint8_t addr)
{
    int32_t pos = 0;

    return Arm_ParsePos(addr, &pos);
}

/**
 * @brief  手动触发一次位置查询
 * @param  addr: 电机地址 5/6
 * @retval 无
 */
void Arm_ReadPos(uint8_t addr)
{
    if (Arm_AddrValid(addr) == 0U)
    {
        return;
    }

    Arm_TxSlotWait();
    Emm_V5_Read_Sys_Params(addr, S_CPOS);
}

/**
 * @brief  开启/关闭"位置快查"（动作序列闭环检测提速）
 * @param  addr: 要快查的电机地址（5/6）；传 0 = 关闭，恢复 6 拍轮转
 * @retval 无
 * @note   只是记下目标地址，真正的查询由 Arm_Update() 每 ARM_QUERY_PERIOD_MS 发出
 *         （**位置与到位标志交替**，见 Arm_Update）。切换目标时清一下节拍，
 *         让它下一拍就生效，并**先读一次位置**（动作步开始时要的是"当前位置"）。
 */
void Arm_SetFastPoll(uint8_t addr)
{
    if (addr == 0U)
    {
        armFastPollAddr = 0U;
        armFastPollIdx  = 0U;
        return;
    }

    if (Arm_AddrValid(addr) == 0U)
    {
        return;
    }

    if (armFastPollAddr != addr)
    {
        armFastPollAddr = addr;
        armFastPollIdx  = 0U;   /* 切目标后**先读位置**（动作步要先拿"当前位置"） */
        /* 让快查下一拍立刻生效（否则最坏要等一个查询周期） */
        armQueryTick = HAL_GetTick() - (uint32_t)ARM_QUERY_PERIOD_MS;
    }
}

/**
 * @brief  电机5 手臂转向小幅度点动
 * @param  dirA: 1=向 A 方向；0=向 B 方向
 * @retval 无
 * @note   相对模式（raF=2）：相对当前实时位置走 ARM_TURN_TEST_PULSE 脉冲后自停，
 *         不需要当前位置，也便于把"KEY3 方向"直接标定成宏。
 */
void Arm_TurnTest(uint8_t dirA)
{
    uint8_t dir = (dirA != 0U) ? (uint8_t)ARM_TURN_DIR_A : (uint8_t)ARM_TURN_DIR_B;

    Arm_ReachClear(ARM_ADDR_TURN);
    Arm_TxSlotWait();
    Emm_V5_Pos_Control(ARM_ADDR_TURN, dir, ARM_TURN_VEL, ARM_TURN_ACC,
                       (uint32_t)ARM_TURN_TEST_PULSE, 2U, false);

    Debug_Log("ARM", "turn test pulse=%u dir=%u",
              (unsigned int)ARM_TURN_TEST_PULSE, (unsigned int)dir);
}

/**
 * @brief  电机6 升降小幅度点动
 * @param  dirUp: 1=向 A 方向；0=向 B 方向
 * @retval 无
 */
void Arm_LiftTest(uint8_t dirUp)
{
    uint8_t dir = (dirUp != 0U) ? (uint8_t)ARM_LIFT_DIR_UP : (uint8_t)ARM_LIFT_DIR_DOWN;

    Arm_ReachClear(ARM_ADDR_LIFT);
    Arm_TxSlotWait();
    Emm_V5_Pos_Control(ARM_ADDR_LIFT, dir, ARM_LIFT_VEL, ARM_LIFT_ACC,
                       (uint32_t)ARM_LIFT_TEST_PULSE, 2U, false);

    Debug_Log("ARM", "lift test pulse=%u dir=%u",
              (unsigned int)ARM_LIFT_TEST_PULSE, (unsigned int)dir);
}

/**
 * @brief  把回零参数写进电机（**svF = true，保存到电机 flash**）
 * @param  addr: 电机地址 5/6
 * @retval 无
 * @note   【为什么必须"保存"】`potF`（上电自动回零）是**驱动器级**参数：
 *         不保存就只在本次上电有效，下次上电驱动器又按它 flash 里的旧值走，
 *         于是又会出现"一上电自己就归 0"——这正是以前排查不到的根因。
 *         所以这里统一用 svF=true 写进去。
 *
 *         参数按轴取不同宏：
 *           电机5（单圈回零）：模式 / 回零方向 / 回零速度 / 超时 + potF = ARM_TURN_POTF
 *           电机6（碰撞回零）：模式2 / 方向 / 回零速度 / 超时 + 碰撞阈值 + potF = ARM_LIFT_POT_F
 *
 *         只在"标定 0 点 / 触发回零"时调用（不是周期调用），flash 写入次数可接受。
 */
static void Arm_WriteHomeParams(uint8_t addr)
{
    if (addr == ARM_ADDR_TURN)
    {
        Arm_TxSlotWait();
        Emm_V5_Origin_Modify_Params(addr, true,
                                    (uint8_t)ARM_TURN_HOME_MODE, (uint8_t)ARM_TURN_HOME_DIR,
                                    (uint16_t)ARM_TURN_OM_VEL, (uint32_t)ARM_TURN_OM_TM,
                                    0U, 0U, 0U, (bool)ARM_TURN_POTF);
    }
    else if (addr == ARM_ADDR_LIFT)
    {
        Arm_TxSlotWait();
        Emm_V5_Origin_Modify_Params(addr, true,
                                    (uint8_t)ARM_LIFT_HOME_MODE, (uint8_t)ARM_LIFT_DIR_UP,
                                    (uint16_t)ARM_LIFT_OM_VEL, (uint32_t)ARM_LIFT_OM_TM,
                                    (uint16_t)ARM_LIFT_SL_VEL, (uint16_t)ARM_LIFT_SL_MA,
                                    (uint16_t)ARM_LIFT_SL_MS, (bool)ARM_LIFT_POT_F);
    }
    else
    {
        /* 地址非法，不发 */
    }
}

/**
 * @brief  电机5 触发单圈回零
 * @param  无
 * @retval 无
 * @note   触发前先把回零参数（含"上电自动回零"potF）**保存**进电机，
 *         保证每次回零都停在同一个物理角度。
 */
void Arm_TurnHome(void)
{
    Arm_ReachClear(ARM_ADDR_TURN);

    /* 清回零完成标志：否则上一次的回零完成状态会被当成这次"立刻完成" */
    {
        __IO uint16_t *pz = Arm_ZeroStatusPtr(ARM_ADDR_TURN);
        if (pz != NULL)
        {
            *pz = FAILRECEIVE;
        }
    }

    /* 写回零参数（保存），再触发回零：CAN 帧按顺序进电机 FIFO，紧接发即可 */
    Arm_WriteHomeParams(ARM_ADDR_TURN);

    Arm_TxSlotWait();
    Emm_V5_Origin_Trigger_Return(ARM_ADDR_TURN, (uint8_t)ARM_TURN_HOME_MODE, false);

    armHomeActive    = 1U;
    armHomeAddr      = ARM_ADDR_TURN;
    armHomeStartTick = HAL_GetTick();

    Debug_Log("ARM", "turn home trigger mode=%u dir=%u potF=%u",
              (unsigned int)ARM_TURN_HOME_MODE, (unsigned int)ARM_TURN_HOME_DIR,
              (unsigned int)ARM_TURN_POTF);
}

/**
 * @brief  电机6 触发多圈无限位碰撞回零（向上撞停为 0 点）
 * @param  无
 * @retval 无
 * @note   触发前先把回零参数（含 potF）**保存**进电机；
 *         撞击动作有风险，调用前确认上方无遮挡、结构能承受。
 */
void Arm_LiftHome(void)
{
    Arm_ReachClear(ARM_ADDR_LIFT);

    {
        __IO uint16_t *pz = Arm_ZeroStatusPtr(ARM_ADDR_LIFT);
        if (pz != NULL)
        {
            *pz = FAILRECEIVE;
        }
    }

    /* 写回零参数（保存）：碰撞回零模式(2) + 方向 + 回零速度/超时 + 碰撞阈值 + potF */
    Arm_WriteHomeParams(ARM_ADDR_LIFT);

    /* 触发回零（CAN 帧按顺序进电机 FIFO，紧接参数帧后面发即可） */
    Arm_TxSlotWait();
    Emm_V5_Origin_Trigger_Return(ARM_ADDR_LIFT, (uint8_t)ARM_LIFT_HOME_MODE, false);

    armHomeActive    = 1U;
    armHomeAddr      = ARM_ADDR_LIFT;
    armHomeStartTick = HAL_GetTick();

    Debug_Log("ARM", "lift home trigger mode=%u sl_ma=%u sl_ms=%u potF=%u",
              (unsigned int)ARM_LIFT_HOME_MODE,
              (unsigned int)ARM_LIFT_SL_MA, (unsigned int)ARM_LIFT_SL_MS,
              (unsigned int)ARM_LIFT_POT_F);
}

/**
 * @brief  把当前位置设为单圈零点
 * @param  addr: 电机地址 5/6
 * @retval 无
 * @note   用法：先把机构摆到期望的 0 点（**等电机停稳**），再调用本函数。
 *         一次动作做三件事，保证"以后都停在这个角度"：
 *           1) 把当前位置标定为**单圈零点**（`addr 93 88 01 6B`，
 *              svF 由 ARM_SETZERO_SAVE_FLASH 决定，默认 1 = 掉电保持）；
 *           2) 再把**"当前位置"计数清零**（`addr 0A 6D 6B`，由
 *              ARM_SETZERO_RESET_CURPOS 控制）—— 固件里这两条是**两套坐标**，
 *              少了第 2 条页面 `p=` 不会立刻变 0；
 *           3) 把**回零参数（含"上电自动回零"potF）也保存**进电机。
 *         设完顺带清掉"回零完成"标志，避免页面 H 显示陈旧状态。
 */
void Arm_SetZero(uint8_t addr)
{
    __IO uint16_t *pStatus;

    if (Arm_AddrValid(addr) == 0U)
    {
        return;
    }

    Arm_TxSlotWait();

#if (ARM_SETZERO_SAVE_FLASH != 0U)
    /* svF = true：写入电机 flash，设置后**掉电保持**（标定机械零点的正常用法） */
    Emm_V5_Origin_Set_O(addr, true);
#else
    /* svF = false：不存储，只本次上电生效，掉电后回到旧零点 */
    Emm_V5_Origin_Set_O(addr, false);
#endif

#if (ARM_SETZERO_RESET_CURPOS != 0U)
    /* 再发一条"设置当前位置为零"（`addr 0A 6D 6B`）：
       上面那条"单圈零点"登记的是**回零终点角度**，而**"当前位置"是另一套计数** ——
       补这一条才能让页面 `p=` 立刻变成 0（否则会出现"按了没反应"的错觉）。 */
    Arm_ZeroCurPos(addr);
#endif

    /* 顺带把回零参数（含"上电自动回零"potF）也存进电机：
       这样"回零 / 上电自动回零"都指向刚标定的这个角度，不用再按第二次 */
    Arm_WriteHomeParams(addr);

    /* 清掉"回零完成"标志：设完零点后驱动内部零点已变，
       旧标志留着会让页面 H 显示 1、让人误以为"已经回过零" */
    pStatus = Arm_ZeroStatusPtr(addr);
    if (pStatus != NULL)
    {
        *pStatus = FAILRECEIVE;
    }

    Debug_Log("ARM", "%s set zero at cur pos (flash=%u resetPos=%u)",
              Arm_Name(addr), (unsigned int)ARM_SETZERO_SAVE_FLASH,
              (unsigned int)ARM_SETZERO_RESET_CURPOS);
}

/**
 * @brief  把电机"当前位置"计数清零（发送 `addr 0A 6D 6B`）
 * @param  addr: 电机地址 5/6
 * @retval 无
 * @note   只改"当前位置"这个计数，**不写电机 flash、不动回零参数**，
 *         所以可以频繁调用（区别于 Arm_SetZero 的"标定"）。
 *         典型用法：ARMSEQ 页每次进页让电机5 回零完成后，把基准对齐到 0。
 */
void Arm_ZeroCurPos(uint8_t addr)
{
    if (Arm_AddrValid(addr) == 0U)
    {
        return;
    }

    Arm_TxSlotWait();
    Emm_V5_Reset_CurPos_To_Zero(addr);

    Debug_Log("ARM", "%s cur pos -> 0", Arm_Name(addr));
}

/**
 * @brief  查询是否回零完成
 * @param  addr: 电机地址 5/6
 * @retval 1=完成；0=未完成/地址非法
 */
uint8_t Arm_IsHomeDone(uint8_t addr)
{
    __IO uint16_t *pStatus = Arm_ZeroStatusPtr(addr);

    if (pStatus == NULL)
    {
        return 0U;
    }
    return (*pStatus == SUCCESSRECEIVE) ? 1U : 0U;
}

/**
 * @brief  查询本次运动是否到位
 * @param  addr: 电机地址 5/6
 * @retval 1=到位；0=未到位/地址非法
 */
uint8_t Arm_IsReachDone(uint8_t addr)
{
    __IO uint16_t *pStatus = Arm_ReachStatusPtr(addr);

    if (pStatus == NULL)
    {
        return 0U;
    }
    return (*pStatus == SUCCESSRECEIVE) ? 1U : 0U;
}

/**
 * @brief  清除到位标志
 * @param  addr: 电机地址 5/6
 * @retval 无
 */
void Arm_ReachClear(uint8_t addr)
{
    __IO uint16_t *pStatus = Arm_ReachStatusPtr(addr);

    if (pStatus != NULL)
    {
        *pStatus = FAILRECEIVE;
    }
}

/**
 * @brief  ★【2026-10-23】舵机"没动"复查表（每 ID 一条）
 * @note   三个舵机**全部跟踪**（现场是三个都会偶发不动）。
 *         ★【2026-10-27 修"偶发丢动作"】销号条件**只剩"到目标容差内"**（S2 另有
 *         "夹到底"限幅判据）。原来还有一条"相对下发前动过 ≥ ARM_SERVO_MOVED_EPS 就销号"，
 *         但 `pos0` 是在**入队时刻**采样的：如果舵机此刻还在执行**上一条**命令（仍在动），
 *         那么它继续走 ≥20 就立刻销号 —— **新命令还没执行就被当成"已完成"**，
 *         补发链（`Act_SrvVerifyService` 之外唯一保底）就此失效 -> 现场"这一步有时没执行"。
 *         被挡住 / 走不到目标的命令，改由 `ARM_SERVO_RETX_MAX` 次补发后放弃（有日志），
 *         代价只是多几帧幂等指令，比"整步被吞掉"小得多。
 */
typedef struct {
    uint8_t  active;   /* 1=这条命令还没被回读确认 */
    uint8_t  id;       /* 舵机 ID */
    uint16_t tgt;      /* 目标位置 0~1000 */
    uint8_t  tries;    /* 已补发次数 */
    uint32_t tick;     /* 上次下发的时刻 */
} ArmSrvCmd_t;

static ArmSrvCmd_t armSrvCmd[3];   /* [0]=S1 [1]=S2 [2]=S3 */

/* 前置声明：复查服务里要补发（定义在本段末尾） */
static void Arm_ServoSendRaw(uint8_t servoId, uint16_t pos);

/**
 * @brief  把一条舵机命令登记到"没动复查表"（新命令覆盖旧命令）
 * @note   ★2026-10-23：三个舵机**全部跟踪**（现场是三个都会偶发不动）。
 *         销号只看"回读进入 ±ARM_SERVO_ARRIVE_TOL"（S2 另有夹到底限幅）。
 */
static void Arm_ServoCmdTrack(uint8_t id, uint16_t pos, uint32_t now)
{
    uint8_t i;

    if (id == (uint8_t)SERVO_ID_CLAW)
    {
        i = 0U;
    }
    else if (id == (uint8_t)SERVO_ID_GRIP)
    {
        i = 1U;
    }
    else if (id == (uint8_t)SERVO_ID_TURNTABLE)
    {
        i = 2U;
    }
    else
    {
        return;
    }

    armSrvCmd[i].active = 1U;
    armSrvCmd[i].id     = id;
    armSrvCmd[i].tgt    = pos;
    armSrvCmd[i].tries  = 0U;
    armSrvCmd[i].tick   = now;
}

/**
 * @brief  舵机"没动"复查服务（每拍调用）：到点还没动 -> 补发（幂等）
 * @note   三个舵机都生效；补发走 `Arm_ServoSendRaw()`（含总线静默等待）。
 *         销号条件：回读进入 ±ARM_SERVO_ARRIVE_TOL（S2 另有"夹到底"限幅）。
 *         ★【2026-10-27】新鲜度判断改用 `Servo_GetPosTickById()`（**本 ID 自己**的
 *         回读时刻）：原来用全局 `Servo_GetPosTick()`，别的舵机一答就"变新鲜"，
 *         本 ID 的旧值会被拿来当成"命令之后的新位置"。
 */
void Arm_ServoSvc(void)
{
    uint8_t  i;
    uint32_t now = HAL_GetTick();

    for (i = 0U; i < 3U; i++)
    {
        ArmSrvCmd_t *c = &armSrvCmd[i];
        uint16_t cur = 0U;
        int32_t  d;

        if (c->active == 0U)
        {
            continue;
        }

        /* 回读（必须是本次下发之后的）有效才做确认 */
        if ((Servo_GetPos(c->id, &cur) != 0U) &&
            ((int32_t)(Servo_GetPosTickById(c->id) - c->tick) >= 0))
        {
            d = (int32_t)cur - (int32_t)c->tgt;
            if (d < 0)
            {
                d = -d;
            }
            if (d <= (int32_t)ARM_SERVO_ARRIVE_TOL)
            {
                c->active = 0U;             /* ① 已到目标 */
                continue;
            }

            /* ★【2026-10-23 硬件限幅】S2 夹紧：命令值 <500、回读 ≤560 就是"夹到底"了 */
            if ((c->id == (uint8_t)SERVO_ID_GRIP) &&
                (c->tgt < (uint16_t)ARM_S2_CLOSE_CMD_MAX) &&
                (cur <= (uint16_t)ARM_S2_CLOSED_POS))
            {
                c->active = 0U;
                continue;
            }
        }

        if ((now - c->tick) < (uint32_t)ARM_SERVO_RETX_MS)
        {
            continue;
        }

        if (c->tries >= (uint8_t)ARM_SERVO_RETX_MAX)
        {
            c->active = 0U;
            Debug_Log("SRV", "id%u no move -> give up (tgt=%u cur=%u)",
                      (unsigned int)c->id, (unsigned int)c->tgt, (unsigned int)cur);
            continue;
        }

        c->tries++;
        c->tick = now;
        Debug_Log("SRV", "id%u no move -> resend %u/%u (tgt=%u cur=%u)",
                  (unsigned int)c->id, (unsigned int)c->tries,
                  (unsigned int)ARM_SERVO_RETX_MAX,
                  (unsigned int)c->tgt, (unsigned int)cur);
        Arm_ServoSendRaw(c->id, c->tgt);
    }
}

void Arm_ServoTrackClear(void)
{
    armSrvCmd[0].active = 0U;
    armSrvCmd[1].active = 0U;
    armSrvCmd[2].active = 0U;
}

/**
 * @brief  ★【2026-10-23】**一帧发多个舵机**（官方 `moveServosByArray`）
 * @param  n   : 个数 1~ARM_SRV_Q_MAX
 * @param  ids : 舵机 ID 数组
 * @param  poss: 目标位置数组（0~1000）
 * @retval 无
 * @note   同一拍要动两个关节（例如 S1 回缩 ‖ S3 转位）时，**合成一帧**下发：
 *           · 调试板只解析 1 帧（原来是 2 串、4 帧），丢帧概率大幅下降；
 *           · 官方库保证同帧内多个舵机**同时起转**。
 *         发之前仍然先等总线静默（`Servo_WaitBusIdle`），等不到才 Yield 强发。
 */
static void Arm_ServoSendList(uint8_t n, const uint8_t *ids, const uint16_t *poss)
{
    LobotServo srv[ARM_SRV_Q_MAX];
    uint8_t  i;
    uint8_t  rep;

    if ((n == 0U) || (n > (uint8_t)ARM_SRV_Q_MAX))
    {
        return;
    }

    /* ★【2026-10-23 修根因】等应答真正收完 + 静默窗；等不到才 Yield 强发 */
    if (Servo_WaitBusIdle((uint16_t)SERVO_QUIET_BEFORE_MOVE_MS,
                          (uint16_t)SERVO_WAIT_IDLE_HARD_MS) == 0U)
    {
        Servo_BusYieldForMotion();
    }

    for (i = 0U; i < n; i++)
    {
        srv[i].ID       = ids[i];
        srv[i].Position = poss[i];
    }

    for (rep = 0U; rep < (uint8_t)ARM_SERVO_MOVE_REPEAT; rep++)
    {
        moveServosByArray(srv, n, (uint16_t)SERVO_MOVE_TIME_MS);
    }

    /* 告诉回读模块"刚发过运动指令"：下一条读请求推迟 SERVO_MOVE_GAP_MS */
    Servo_NotifyMotionSent();
}

/**
 * @brief  立即下发单个舵机（补发用；正常路径走 `Arm_ServoMove` 的队列 + `Arm_ServoFlush`）
 */
static void Arm_ServoSendRaw(uint8_t servoId, uint16_t pos)
{
    Arm_ServoSendList(1U, &servoId, &pos);
}

/* ---- ★【2026-10-23】"同一拍合并成一帧"的待发队列 ---- */
static uint8_t  armSrvQN = 0U;
static uint8_t  armSrvQId[ARM_SRV_Q_MAX];
static uint16_t armSrvQPos[ARM_SRV_Q_MAX];

/**
 * @brief  把本拍积攒的舵机命令**合并成一帧**发出去
 * @param  无
 * @retval 无
 * @note   在 main 主循环里、所有"发舵机动作"的节拍之后调用（现在挂在 `Servo_Update` 之前）。
 *         同一 ID 重复命令会被**后者覆盖**（与原来的"后发先至"语义一致）。
 */
void Arm_ServoFlush(void)
{
    uint8_t ids[ARM_SRV_Q_MAX];
    uint16_t poss[ARM_SRV_Q_MAX];
    uint8_t n;
    uint8_t i;

    if (armSrvQN == 0U)
    {
        return;
    }

    n = armSrvQN;
    armSrvQN = 0U;
    for (i = 0U; i < n; i++)
    {
        ids[i]  = armSrvQId[i];
        poss[i] = armSrvQPos[i];
    }

    Debug_Log("SRV", "flush n=%u id=%u,%u,%u pos=%u,%u,%u",
              (unsigned int)n,
              (unsigned int)((n > 0U) ? ids[0] : 0U),
              (unsigned int)((n > 1U) ? ids[1] : 0U),
              (unsigned int)((n > 2U) ? ids[2] : 0U),
              (unsigned int)((n > 0U) ? poss[0] : 0U),
              (unsigned int)((n > 1U) ? poss[1] : 0U),
              (unsigned int)((n > 2U) ? poss[2] : 0U));

    Arm_ServoSendList(n, ids, poss);
}

/**
 * @brief  下发一条舵机运动指令，并记录"最后一次下发的目标位置"
 * @param  servoId: 舵机 ID（SERVO_ID_CLAW / SERVO_ID_GRIP）
 * @param  pos    : 目标位置 0~1000（内部钳位，越界不会打到机械限位）
 * @retval 无
 * @note   官方库 moveServo() 内部会校验 servoID ≤ 31 且 Time > 0，
 *         不满足时**直接 return 不发帧**；本函数传的 Time 固定为
 *         SERVO_MOVE_TIME_MS（必须 > 0）。
 *         ★2026-10-23：**不再立即发**，先入队，本拍结束由 `Arm_ServoFlush()` 合并成一帧；
 *         同时把 S1/S3 登记进"没动复查表"，由 `Arm_ServoSvc()` 复查回读、真没动就补发。
 */
void Arm_ServoMove(uint8_t servoId, uint16_t pos)
{
    uint8_t i;
    uint8_t found = 0U;

    if (pos > (uint16_t)ARM_SERVO_POS_MAX)
    {
        pos = (uint16_t)ARM_SERVO_POS_MAX;
    }

    /* ---- 入队（同 ID 覆盖） ---- */
    for (i = 0U; i < armSrvQN; i++)
    {
        if (armSrvQId[i] == servoId)
        {
            armSrvQPos[i] = pos;
            found = 1U;
            break;
        }
    }
    if (found == 0U)
    {
        if (armSrvQN >= (uint8_t)ARM_SRV_Q_MAX)
        {
            Arm_ServoFlush();               /* 队列满：先发一次腾地方 */
        }
        armSrvQId[armSrvQN]  = servoId;
        armSrvQPos[armSrvQN] = pos;
        armSrvQN++;
    }

    Arm_ServoCmdTrack(servoId, pos, HAL_GetTick());

    if (servoId == SERVO_ID_CLAW)
    {
        armServoLastPos[0] = pos;
    }
    else if (servoId == SERVO_ID_GRIP)
    {
        armServoLastPos[1] = pos;
    }
    else if (servoId == SERVO_ID_TURNTABLE)
    {
        armServoLastPos[2] = pos;
    }
    else
    {
        /* 其它 ID 不记录（本阶段只有 1/2/3 三个舵机） */
    }

    Debug_Log("SRV", "move id=%u pos=%u t=%u",
              (unsigned int)servoId, (unsigned int)pos, (unsigned int)SERVO_MOVE_TIME_MS);
}

/**
 * @brief  取某舵机最后一次下发的目标位置
 * @param  servoId: 舵机 ID（SERVO_ID_CLAW / SERVO_ID_GRIP）
 * @retval 0~1000；本次上电还没下发过（或 ID 不是两个已定义的）返回 0xFFFF
 */
uint16_t Arm_GetServoLastPos(uint8_t servoId)
{
    if (servoId == SERVO_ID_CLAW)
    {
        return armServoLastPos[0];
    }
    if (servoId == SERVO_ID_GRIP)
    {
        return armServoLastPos[1];
    }
    if (servoId == SERVO_ID_TURNTABLE)
    {
        return armServoLastPos[2];
    }
    return 0xFFFFU;
}
