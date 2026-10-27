/**
 ******************************************************************************
 * @file    bsp_motor.c
 * @brief   步进电机控制封装层实现（P3 电机单轮调试）
 * @author  nx_car
 * @date    2026-09-09
 ******************************************************************************
 * @note   封装官方库 Emm_V5.h：
 *           - 使能/失能 Emm_V5_En_Control（0xF3）
 *           - 速度模式 Emm_V5_Vel_Control（0xF6，8 字节帧）
 *           - 位置模式 Motor_PosMove -> Emm_V5_Pos_Control（0xFD，raF=2
 *             相对当前位置），加减速由电机固件内部规划，到位自停
 *           - 立即停止 Emm_V5_Stop_Now（0xFE，急停）
 *           - 缓停 Motor_SoftStop（同样走 0xF6 帧但速度发 0，由电机按
 *             acc 档减速到 0；acc=0 时等同瞬停）
 *           - 读实时位置 Emm_V5_Read_Sys_Params(addr, S_CPOS)（应答 0x36 帧
 *             由 bsp_Emm_V5.c 的 HAL_CAN_RxFifo0MsgPendingCallback 解析，
 *             置 Emm_V5_Pos_Status.Mx_EncodedStatus = SUCCESSRECEIVE）
 *         全部接口非阻塞，不含任何 HAL_Delay；日志统一 [MOT] 标签，禁 %f。
 ******************************************************************************
 */
#include "bsp_motor.h"
#include "bsp_Emm_V5.h"   /* 提供 Emm_V5_Pos_Status 接收状态结构体与复位函数 */
#include "bsp_debug.h"

/* ===================== 内部自检状态机 ===================== */
typedef enum {
    CHECK_PHASE_IDLE = 0,   /* 未开始（初始化前）         */
    CHECK_PHASE_QUERY,      /* 待发查询：本次调用发 S_CPOS */
    CHECK_PHASE_WAIT,       /* 等待应答：查状态/判超时     */
    CHECK_PHASE_DONE        /* 四轮全部查完               */
} CheckPhase_t;

static CheckPhase_t checkPhase = CHECK_PHASE_IDLE;  /* 自检状态机当前阶段     */
static uint8_t      checkAddr  = MOTOR_ADDR_MIN;    /* 当前正在自检的电机地址 */
static uint32_t     waitTick   = 0;                 /* 发出查询时的时间戳 ms  */
static uint8_t      onlineCnt  = 0;                 /* 在线电机计数           */

/* 周期查询状态（自检完成后主循环一直运行，供位置/转速读取） */
static uint8_t      motQueryIdx  = 0;               /* 查询槽位 0 ~ 2*MOTOR_NUM-1：
                                                       0~3 查位置(S_CPOS)，
                                                       4~7 查转速(S_VEL)        */
static uint32_t     motQueryLastTick = 0;           /* 上次查询时间戳 ms        */
static uint32_t     motRpmLastTick = 0;             /* 上次转速上报时间戳 ms    */
static uint32_t     motTxWaitCnt   = 0;             /* 因发送邮箱满而等待的次数 */
static uint32_t     motTxWaitLast  = 0;             /* 上次上报时的等待次数     */
static uint32_t canArbLostCnt  = 0;             /* CAN 仲裁丢失累计次数（只读诊断） */
static uint32_t canErrLastRec  = 0xFFFFFFFFU;   /* 上次上报的接收错误计数 */

/**
 * @brief  CAN 健康度只读诊断（仲裁丢失 / 错误计数）
 * @param  无
 * @retval 无
 * @note   读 CAN1 的 TSR / ESR 寄存器，不改任何功能配置。
 *         **仲裁丢失（ALST）**：本机发出的帧在仲裁阶段输给了别的节点（例如电机
 *         正好在回帧）。此时若 NART=1（`AutoRetransmission = DISABLE`），
 *         该帧会被**直接丢弃且不重传**——现象正是"某一路轮子偶发收不到命令、
 *         四个轮都有命令却不转"。本计数持续增长即可证实该机制在发生。
 *         接收错误计数（REC）异常增大则说明总线接线/终端电阻/干扰有问题。
 */
static void Motor_CanHealthReport(void)
{
    uint32_t tsr = CAN1->TSR;
    uint32_t alst = tsr & (CAN_TSR_ALST0 | CAN_TSR_ALST1 | CAN_TSR_ALST2);
    uint32_t rec  = (CAN1->ESR >> 8) & 0xFFU;

    if (alst != 0U)
    {
        /* 仲裁丢失标志是"写 1 清除" */
        canArbLostCnt++;
        CAN1->TSR = (CAN_TSR_ALST0 | CAN_TSR_ALST1 | CAN_TSR_ALST2);
    }

    if (canArbLostCnt != 0U)
    {
        Debug_Log("MOT", "can arbitration lost=%lu (NART: those frames are dropped)",
                  (unsigned long)canArbLostCnt);
        canArbLostCnt = 0U;    /* 报一次清零，避免刷屏 */
    }

    if (rec != canErrLastRec)
    {
        if (rec != 0U)
        {
            Debug_Log("MOT", "can rx err cnt=%lu (check wiring/terminator/noise)",
                      (unsigned long)rec);
        }
        canErrLastRec = rec;
    }
}

/**
 * @brief  打开 CAN 自动重传（NART 清零）——在自家文件里补，不改 can.c
 * @param  无
 * @retval 无
 * @note   **为什么需要**：can.c 里 `hcan.Init.AutoRetransmission = DISABLE`（NART=1），
 *         本机发出的帧一旦**仲裁失败**（电机正好在回帧时极易发生）或遇总线错误，
 *         就被**直接丢弃、不重传** → 那一路电机收不到命令（现象：页面 `w:` 四位
 *         都是 1、却有轮子不转，而且时好时坏）。
 *         这里在初始化后把 NART 关掉，让这类帧自动重发。
 *         **寄存器约束**：NART 只能在初始化模式（INRQ=1、等 INAK=1）下修改，
 *         所以流程是"进初始化模式 → 清 NART → 回正常模式"；滤波器配置不受影响。
 *         必须在 MX_CAN_Init() 之后调用（main.c 中 Motor_Init 在 CAN 初始化之后）。
 */
static void Motor_CanEnableRetransmission(void)
{
    uint32_t t0 = HAL_GetTick();

    /* 进入初始化模式 */
    hcan.Instance->MCR |= CAN_MCR_INRQ;
    while (((hcan.Instance->MSR & CAN_MSR_INAK) == 0U) &&
           ((HAL_GetTick() - t0) < 10U))
    {
        /* 等进入初始化模式，有上限，不会卡死 */
    }

    /* 关掉"发送失败不重传"（NART=0 = 自动重传） */
    hcan.Instance->MCR &= ~CAN_MCR_NART;

    /* 回到正常工作模式 */
    hcan.Instance->MCR &= ~CAN_MCR_INRQ;
    t0 = HAL_GetTick();
    while (((hcan.Instance->MSR & CAN_MSR_INAK) != 0U) &&
           ((HAL_GetTick() - t0) < 10U))
    {
        /* 等回到正常模式 */
    }

    Debug_Log("MOT", "can auto-retransmission enabled (NART cleared)");
}

/**
 * @brief  下发 CAN 命令前，确保发送邮箱可用数达到 need，带超时上限
 * @param  need:      需要的空闲邮箱个数（短命令=1；13 字节的 0xFD 位置命令=2）
 * @param  timeoutMs: 等待上限，单位 ms
 * @retval 无
 * @note   【为什么按"个数"等，而不是"有 1 个就行"】官方 `can_SendCmd` 对 **>8 字节**
 *         的命令会**拆成多包**发送（0xFD 位置命令 = `dir/vel/acc/clk` 一包 +
 *         `raF/snF/校验` 一包，共 2 帧），而它调 `HAL_CAN_AddTxMessage` 时
 *         **不检查返回值**：邮箱不够时后一包被**静默丢弃** → 电机只收到半条命令
 *         → 直接忽略不动（现象："四轮一起下发时只有个别轮子响应"）。
 *         所以位置命令必须一次凑够 2 个空邮箱；短命令 1 个即可。
 *         忙等有上限，不会阻塞主循环；一帧约 0.3ms @500kbps，正常几乎不等待。
 */
static void Motor_TxMailboxWait(uint8_t need, uint32_t timeoutMs)
{
    uint32_t t0;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) >= (uint32_t)need)
    {
        return;                 /* 邮箱够用：不等待、不计数 */
    }

    motTxWaitCnt++;
    t0 = HAL_GetTick();
    while ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan) < (uint32_t)need) &&
           ((HAL_GetTick() - t0) < timeoutMs))
    {
        /* 忙等，有上限；此处不发日志（避免 UART 再拖时间） */
    }
}

/**
 * @brief  短命令（≤8 字节，1 帧）下发前等一个空闲发送邮箱
 * @param  无
 * @retval 无
 * @note   使能/停止/速度/读参数这类命令都是 1 帧，等 1 个空邮箱即可。
 *         位置模式命令请用 Motor_TxMailboxWait(2, …)。
 */
static void Motor_TxSlotWait(void)
{
    Motor_TxMailboxWait(1U, (uint32_t)MOTOR_TX_SLOT_WAIT_MS);
}

/**
 * @brief  读取"因发送邮箱满而等待"的累计次数
 * @param  无
 * @retval 累计次数（>0 说明确实发生过邮箱占满，旧代码下这些帧会丢失）
 */
uint32_t Motor_GetTxWaitCount(void)
{
    return motTxWaitCnt;
}

/**
 * @brief  周期查询四轮实时位置/转速（主循环调用）
 * @param  无
 * @retval 无
 * @note   自检完成后主循环一直调度。每 MOTOR_QUERY_PERIOD_MS(10ms) 发 1 条查询，
 *         共 8 个查询槽位构成一轮：
 *           槽位 0~3 -> 依次查 1~4 号电机位置(S_CPOS)，应答 0x36 帧存 Mx_EncodedBuff；
 *           槽位 4~7 -> 依次查 1~4 号电机转速(S_VEL)， 应答 0x35 帧存 Mx_VelBuff。
 *         一轮 8 拍共 80ms，故单轮位置/转速的更新间隔均为 80ms。
 *         采用"前半轮查位置、后半轮查转速"而非逐拍交替，是为避免
 *         地址轮转（周期 4）与查询类型（周期 2）同相导致某轮永远只被查
 *         到同一种参数。
 *         位置供上层读位置/进度使用（Motor_GetAngle_x10deg），转速供
 *         Motor_RpmReport 上报（数据看板曲线）。
 *         位置更新间隔直接决定上层里程反馈的新鲜度：间隔越短，基于里程的
 *         速度/停位控制越准（停位过冲量 ≈ 末端速度 × 本间隔）。
 */
static void Motor_PeriodicQuery(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - motQueryLastTick) < MOTOR_QUERY_PERIOD_MS)
    {
        return;
    }
    motQueryLastTick = now;

    Motor_TxSlotWait();

    if (motQueryIdx < MOTOR_NUM)
    {
        /* 前半轮：查实时位置，应答 0x36 帧 -> Mx_EncodedBuff */
        Emm_V5_Read_Sys_Params(MOTOR_ADDR_MIN + motQueryIdx, S_CPOS);
    }
    else
    {
        /* 后半轮：查实时转速，应答 0x35 帧 -> Mx_VelBuff */
        Emm_V5_Read_Sys_Params(MOTOR_ADDR_MIN + (motQueryIdx - MOTOR_NUM), S_VEL);
    }

    motQueryIdx++;
    if (motQueryIdx >= (MOTOR_NUM * 2U))
    {
        motQueryIdx = 0U;
    }
}

/* 每轮自检结果（下标 0 对应地址 1） */
static MotorCheckState_t checkResult[MOTOR_NUM] = {
    MOTOR_CHECK_UNCHECKED, MOTOR_CHECK_UNCHECKED,
    MOTOR_CHECK_UNCHECKED, MOTOR_CHECK_UNCHECKED
};

/* ===================== 内部工具函数 ===================== */

/**
 * @brief  地址范围检查
 * @param  addr: 电机地址
 * @retval 1=合法（1~4），0=非法
 */
static uint8_t Motor_AddrValid(uint8_t addr)
{
    return ((addr >= MOTOR_ADDR_MIN) && (addr <= MOTOR_ADDR_MAX)) ? 1U : 0U;
}

/**
 * @brief  取某轮接收状态字的指针（回调按 ExtId 0x100~0x400 分别置位）
 * @param  addr: 电机地址 1~4
 * @retval 对应 Mx_EncodedStatus 指针；地址非法返回 NULL
 */
static __IO uint16_t *Motor_StatusPtr(uint8_t addr)
{
    switch (addr)
    {
        case MOTOR_ADDR_FL: return &Emm_V5_Pos_Status.M1_EncodedStatus;
        case MOTOR_ADDR_FR: return &Emm_V5_Pos_Status.M2_EncodedStatus;
        case MOTOR_ADDR_RL: return &Emm_V5_Pos_Status.M3_EncodedStatus;
        case MOTOR_ADDR_RR: return &Emm_V5_Pos_Status.M4_EncodedStatus;
        default:            return NULL;
    }
}

/**
 * @brief  取某轮"正转"对应的官方库 dir 值（方向标定宏）
 * @param  addr: 电机地址 1~4
 * @retval dir 值（0=CW / 1=CCW）；非法地址返回默认值
 */
static uint8_t Motor_DirFwd(uint8_t addr)
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
 * @brief  取某轮"反转"对应的官方库 dir 值（方向标定宏）
 * @param  addr: 电机地址 1~4
 * @retval dir 值（0=CW / 1=CCW）；非法地址返回默认值
 */
static uint8_t Motor_DirRev(uint8_t addr)
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

/* ===================== 公开接口 ===================== */

/**
 * @brief  电机模块初始化（启动 CAN 总线自检状态机）
 * @param  无
 * @retval 无
 * @note   只发读命令的准备工作：复位接收状态为 FAILRECEIVE，自检结果清零，
 *         状态机置 QUERY（首条查询在 Motor_Update 首次调用时发出）。
 *         禁止在此发使能/速度指令。
 */
void Motor_Init(void)
{
    /* 打开 CAN 自动重传（不改 can.c，在自家文件里补；必须在 MX_CAN_Init 之后调用） */
    Motor_CanEnableRetransmission();

    /* 复位官方库接收状态结构体（M1~M6_EncodedStatus = FAILRECEIVE） */
    Emm_V5_Pos_Status_Read_Init(&Emm_V5_Pos_Status);

    checkResult[0] = MOTOR_CHECK_UNCHECKED;
    checkResult[1] = MOTOR_CHECK_UNCHECKED;
    checkResult[2] = MOTOR_CHECK_UNCHECKED;
    checkResult[3] = MOTOR_CHECK_UNCHECKED;

    checkAddr  = MOTOR_ADDR_MIN;
    onlineCnt  = 0U;
    checkPhase = CHECK_PHASE_QUERY;

    Debug_Log("MOT", "bus check start");
}

/**
 * @brief  周期上报四轮实时转速（数据看板画转速曲线用）
 * @param  无
 * @retval 无
 * @note   按 MOTOR_RPM_REPORT_MS 周期读四轮转速缓冲（0x35 帧，由
 *          Motor_PeriodicQuery 发 S_VEL 查询触发），经 [MOT] addr=X rpm=Y
 *         日志上传。静止时电机返回 0，运动时返回实际转速；一直后台运行。
 */
static void Motor_RpmReport(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t  i;
    int32_t  rpm;

    if ((now - motRpmLastTick) < MOTOR_RPM_REPORT_MS)
    {
        return;
    }

    for (i = 0; i < MOTOR_NUM; i++)
    {
        if (Motor_GetRpm(MOTOR_ADDR_MIN + i, &rpm) == 0)
        {
            Debug_Log("MOT", "addr=%u rpm=%d",
                      (unsigned int)(MOTOR_ADDR_MIN + i), (int)rpm);
        }
    }

    /* 只在"因邮箱满而等待"的次数发生变化时打一行：>0 说明真的发生过邮箱占满
       （旧代码在这种情况下会静默丢弃该帧，表现为个别轮子偶发不动） */
    if (motTxWaitCnt != motTxWaitLast)
    {
        motTxWaitLast = motTxWaitCnt;
        Debug_Log("MOT", "can tx slot wait=%lu (mailbox was full)",
                  (unsigned long)motTxWaitCnt);
    }

    /* CAN 总线健康度：仲裁丢失（NART 下会丢帧）与接收错误计数 */
    Motor_CanHealthReport();

    motRpmLastTick = now;
}

/**
 * @brief  电机自检周期轮询（主循环调用，非阻塞）
 * @param  无
 * @retval 无
 * @note   状态机：QUERY 发 S_CPOS 读位置 -> WAIT 轮询应答/超时 ->
 *         记录 ok/no reply 并推进下一个地址 -> 四轮完成输出汇总。
 *         每次调用最多推进一步，禁止 HAL_Delay。
 */
void Motor_Update(void)
{
    __IO uint16_t *pStatus;

    switch (checkPhase)
    {
        case CHECK_PHASE_QUERY:
            /* 发"读实时位置"指令（必须 S_CPOS：应答 0x36 帧才会被回调解析） */
            Motor_TxSlotWait();
            Emm_V5_Read_Sys_Params(checkAddr, S_CPOS);
            waitTick   = HAL_GetTick();
            checkPhase = CHECK_PHASE_WAIT;
            break;

        case CHECK_PHASE_WAIT:
            pStatus = Motor_StatusPtr(checkAddr);
            if ((pStatus != NULL) && (*pStatus == SUCCESSRECEIVE))
            {
                /* 收到应答：记录在线 */
                checkResult[checkAddr - 1U] = MOTOR_CHECK_OK;
                onlineCnt++;
                Debug_Log("MOT", "addr=%u ok", (unsigned int)checkAddr);
            }
            else if ((HAL_GetTick() - waitTick) >= MOTOR_CHECK_TIMEOUT_MS)
            {
                /* 超时无应答：记录离线（属硬件链路问题，勿改代码） */
                checkResult[checkAddr - 1U] = MOTOR_CHECK_TIMEOUT;
                Debug_Log("MOT", "addr=%u no reply", (unsigned int)checkAddr);
            }
            else
            {
                /* 未收到且未超时：本次不推进，等下次调用再查 */
                break;
            }

            /* 推进下一个地址；四轮查完输出汇总 */
            if (checkAddr >= MOTOR_ADDR_MAX)
            {
                checkPhase = CHECK_PHASE_DONE;
                Debug_Log("MOT", "check done, %u/%u online",
                          (unsigned int)onlineCnt, (unsigned int)MOTOR_NUM);
            }
            else
            {
                checkAddr++;
                checkPhase = CHECK_PHASE_QUERY;
            }
            break;

        case CHECK_PHASE_IDLE:
        default:
            /* 自检未开始：无动作 */
            break;

        case CHECK_PHASE_DONE:
            /* 自检完成：周期查询位置/转速 + 转速上报（主循环一直运行） */
            Motor_PeriodicQuery();
            Motor_RpmReport();
            break;
    }
}

/**
 * @brief  使能/失能单个电机
 * @param  addr: 电机地址 1~4，超范围忽略
 * @param  on: 1=使能，0=失能
 * @retval 无
 */
void Motor_Enable(uint8_t addr, uint8_t on)
{
    if (!Motor_AddrValid(addr))
    {
        Debug_Log("MOT", "enable bad addr=%u", (unsigned int)addr);
        return;
    }
    Motor_TxSlotWait();
    Emm_V5_En_Control(addr, (on != 0U) ? true : false, false);
    Debug_Log("MOT", "addr=%u %s", (unsigned int)addr, (on != 0U) ? "enable" : "disable");
}

/**
 * @brief  失能单个电机
 * @param  addr: 电机地址 1~4，超范围忽略
 * @retval 无
 */
void Motor_Disable(uint8_t addr)
{
    Motor_Enable(addr, 0U);
}

/**
 * @brief  立即停止单个电机（0xFE 急停）
 * @param  addr: 电机地址 1~4，超范围忽略
 * @retval 无
 */
void Motor_Stop(uint8_t addr)
{
    if (!Motor_AddrValid(addr))
    {
        Debug_Log("MOT", "stop bad addr=%u", (unsigned int)addr);
        return;
    }
    Motor_TxSlotWait();
    Emm_V5_Stop_Now(addr, false);
    Debug_Log("MOT", "addr=%u stop", (unsigned int)addr);
}

/**
 * @brief  单个电机"缓停"：速度模式发 0 速度帧（电机走内部减速斜坡）
 * @param  addr: 电机地址 1~4，超范围忽略
 * @param  acc:  加速度档 0~255，限幅到 MOTOR_ACC_MAX；须非 0 才是缓停
 * @retval 无
 * @note   与 Motor_Stop 的 0xFE 瞬停相对：本函数发 0xF6 速度帧、速度=0，
 *         由电机按 acc 档减速到 0，用于正常到位收尾。
 *         方向位对本帧无实际意义（速度为 0），仍按该轮"正转"方向宏下发，
 *         保证报文内容确定、便于日志比对。
 */
void Motor_SoftStop(uint8_t addr, uint8_t acc)
{
    if (!Motor_AddrValid(addr))
    {
        Debug_Log("MOT", "softstop bad addr=%u", (unsigned int)addr);
        return;
    }

    /* 加速度限幅（uint8_t 天然 <=255，此处仅做语义化钳位） */
    if (acc > (uint8_t)MOTOR_ACC_MAX)
    {
        acc = (uint8_t)MOTOR_ACC_MAX;
    }

    /* 速度 0 的速度帧：acc!=0 时电机按斜坡减速到 0（缓停） */
    Motor_TxSlotWait();
    Emm_V5_Vel_Control(addr, Motor_DirFwd(addr), 0, acc, false);
    Debug_Log("MOT", "addr=%u softstop acc=%u", (unsigned int)addr, (unsigned int)acc);
}

/**
 * @brief  单轮速度控制（有符号转速）
 * @param  addr: 电机地址 1~4，超范围忽略
 * @param  rpm:  有符号转速 RPM，±MOTOR_RPM_MAX 限幅；0=立即停止；
 *               正/负按该轮方向标定宏映射 dir
 * @param  acc:  加速度档 0~255，限幅到 MOTOR_ACC_MAX，0=直接启动
 * @retval 无
 */
void Motor_SetVel(uint8_t addr, int16_t rpm, uint8_t acc)
{
    uint8_t  dir;
    uint16_t speed;

    if (!Motor_AddrValid(addr))
    {
        Debug_Log("MOT", "vel bad addr=%u", (unsigned int)addr);
        return;
    }

    /* rpm=0 等价于立即停止 */
    if (rpm == 0)
    {
        Motor_Stop(addr);
        return;
    }

    /* 转速软限幅（按绝对值） */
    if (rpm > (int16_t)MOTOR_RPM_MAX)
    {
        rpm = (int16_t)MOTOR_RPM_MAX;
    }
    else if (rpm < -(int16_t)MOTOR_RPM_MAX)
    {
        rpm = -(int16_t)MOTOR_RPM_MAX;
    }

    /* 加速度限幅（uint8_t 天然 <=255，此处仅做语义化钳位） */
    if (acc > (uint8_t)MOTOR_ACC_MAX)
    {
        acc = (uint8_t)MOTOR_ACC_MAX;
    }

    /* 正负转速映射到该轮方向宏（标定对调处） */
    if (rpm > 0)
    {
        dir   = Motor_DirFwd(addr);
        speed = (uint16_t)rpm;
    }
    else
    {
        dir   = Motor_DirRev(addr);
        speed = (uint16_t)(-rpm);
    }

    Motor_TxSlotWait();
    Emm_V5_Vel_Control(addr, dir, speed, acc, false);
    Debug_Log("MOT", "addr=%u vel=%d rpm acc=%u",
              (unsigned int)addr, (int)rpm, (unsigned int)acc);
}

/**
 * @brief  Same as Motor_SetVel but silent; rpm=0 sends speed 0 (soft stop, no 0xFE)
 * @param  addr: 1~4
 * @param  rpm:  signed RPM
 * @param  acc:  accel level
 * @retval None
 * @note   Route velocity mode calls this 4x per 20ms tick -> no UART logging.
 */
void Motor_SetVelQuiet(uint8_t addr, int16_t rpm, uint8_t acc)
{
    uint8_t  dir;
    uint16_t speed;

    if (!Motor_AddrValid(addr))
    {
        return;
    }

    if (rpm > (int16_t)MOTOR_RPM_MAX)
    {
        rpm = (int16_t)MOTOR_RPM_MAX;
    }
    else if (rpm < -(int16_t)MOTOR_RPM_MAX)
    {
        rpm = -(int16_t)MOTOR_RPM_MAX;
    }

    if (acc > (uint8_t)MOTOR_ACC_MAX)
    {
        acc = (uint8_t)MOTOR_ACC_MAX;
    }

    if (rpm >= 0)
    {
        dir   = Motor_DirFwd(addr);
        speed = (uint16_t)rpm;
    }
    else
    {
        dir   = Motor_DirRev(addr);
        speed = (uint16_t)(-rpm);
    }

    Motor_TxSlotWait();
    Emm_V5_Vel_Control(addr, dir, speed, acc, false);
}

/**
 * @brief  查询某轮上电自检结果
 * @param  addr: 电机地址 1~4，非法返回 MOTOR_CHECK_UNCHECKED
 * @retval MOTOR_CHECK_OK / MOTOR_CHECK_TIMEOUT / MOTOR_CHECK_UNCHECKED
 */
MotorCheckState_t Motor_IsOnline(uint8_t addr)
{
    if (!Motor_AddrValid(addr))
    {
        return MOTOR_CHECK_UNCHECKED;
    }
    return checkResult[addr - 1U];
}

/**
 * @brief  取某轮"到位回传"状态字指针（0xFD 到位帧置位）
 * @param  addr: 电机地址 1~4
 * @retval 对应 Mx_Reach_Status 指针；地址非法返回 NULL
 * @note   电机菜单 Response 必须设为 Reached / Other / Both 才会收到到位帧
 *         （出厂默认 Receive 只回"收到"命令，永远不会有到位帧）。
 *         到位帧：地址 + 0xFD + 0x9F + 0x6B；收到命令帧：地址 + 0xFD + 0x02 + 0x6B，
 *         两者帧头相同，只有 0x9F 才算到位（详见 bsp_Emm_V5.c 的解析）。
 *         本项目实测用 Other：位置模式命令只回到位帧，没有 0x02 确认帧干扰。
 */
static __IO uint16_t *Motor_ReachStatusPtr(uint8_t addr)
{
    switch (addr)
    {
        case MOTOR_ADDR_FL: return &Emm_V5_Pos_Status.M1_Reach_Status;
        case MOTOR_ADDR_FR: return &Emm_V5_Pos_Status.M2_Reach_Status;
        case MOTOR_ADDR_RL: return &Emm_V5_Pos_Status.M3_Reach_Status;
        case MOTOR_ADDR_RR: return &Emm_V5_Pos_Status.M4_Reach_Status;
        default:            return NULL;
    }
}

/**
 * @brief  清除某轮的到位标志
 * @param  addr: 电机地址 1~4
 * @retval 无
 * @note   **每下发一个新段之前必须清除**，否则上一段的到位标志会被当成
 *         本段已到位，导致"刚下发就判到位、实际没走"。
 */
void Motor_ReachClear(uint8_t addr)
{
    __IO uint16_t *pStatus = Motor_ReachStatusPtr(addr);

    if (pStatus != NULL)
    {
        *pStatus = FAILRECEIVE;
    }
}

/**
 * @brief  查询某轮是否已回传"到位"
 * @param  addr: 电机地址 1~4
 * @retval 1=已到位；0=未到位或地址非法
 */
uint8_t Motor_ReachOk(uint8_t addr)
{
    __IO uint16_t *pStatus = Motor_ReachStatusPtr(addr);

    if (pStatus == NULL)
    {
        return 0U;
    }
    return (*pStatus == SUCCESSRECEIVE) ? 1U : 0U;
}

/**
 * @brief  取某轮转速应答状态字指针（0x35 帧转速应答置位）
 * @param  addr: 电机地址 1~4
 * @retval 对应 Mx_VelStatus 指针；地址非法返回 NULL
 */
static __IO uint16_t *Motor_VelStatusPtr(uint8_t addr)
{
    switch (addr)
    {
        case MOTOR_ADDR_FL: return &Emm_V5_Pos_Status.M1_VelStatus;
        case MOTOR_ADDR_FR: return &Emm_V5_Pos_Status.M2_VelStatus;
        case MOTOR_ADDR_RL: return &Emm_V5_Pos_Status.M3_VelStatus;
        case MOTOR_ADDR_RR: return &Emm_V5_Pos_Status.M4_VelStatus;
        default:            return NULL;
    }
}

/**
 * @brief  取某轮转速应答帧缓冲指针（0x35 帧 7 字节由回调 memcpy 存入）
 * @param  addr: 电机地址 1~4
 * @retval 对应 Mx_VelBuff 指针；地址非法返回 NULL
 */
static __IO uint8_t *Motor_VelBufPtr(uint8_t addr)
{
    switch (addr)
    {
        case MOTOR_ADDR_FL: return Emm_V5_Pos_Status.M1_VelBuff;
        case MOTOR_ADDR_FR: return Emm_V5_Pos_Status.M2_VelBuff;
        case MOTOR_ADDR_RL: return Emm_V5_Pos_Status.M3_VelBuff;
        case MOTOR_ADDR_RR: return Emm_V5_Pos_Status.M4_VelBuff;
        default:            return NULL;
    }
}

/**
 * @brief  取某轮位置应答帧缓冲指针（0x36 帧 7 字节由回调 memcpy 存入）
 * @param  addr: 电机地址 1~4
 * @retval 对应 Mx_EncodedBuff 指针；地址非法返回 NULL
 */
static __IO uint8_t *Motor_EncodedBufPtr(uint8_t addr)
{
    switch (addr)
    {
        case MOTOR_ADDR_FL: return Emm_V5_Pos_Status.M1_EncodedBuff;
        case MOTOR_ADDR_FR: return Emm_V5_Pos_Status.M2_EncodedBuff;
        case MOTOR_ADDR_RL: return Emm_V5_Pos_Status.M3_EncodedBuff;
        case MOTOR_ADDR_RR: return Emm_V5_Pos_Status.M4_EncodedBuff;
        default:            return NULL;
    }
}

/**
 * @brief  读取某轮实时转速（解析 0x35 帧缓冲，不发 CAN 帧，非阻塞）
 * @param  addr: 电机地址 1~4
 * @param  out_rpm: 输出实时转速，单位 RPM，有符号；失败时不改变
 * @retval 0=成功；-1=失败（地址/指针非法、未收到应答、帧头校验不过）
 * @note   帧布局（DLC=5）：buf[0]=0x35、buf[1]=符号(0=正/非0=负)、
 *         buf[2..3]=转速大端 16 位（单位 RPM，1=1转/分）、buf[4]=0x6B。
 *         数据由 S_VEL(0x35) 查询触发，回调解析存入 Mx_VelBuff。
 *         转速符号已按方向标定宏取反（标定为反的 FR/RR 轮），使返回
 *         转速与"整车前进方向为正"一致。转速单位需实测标定。
 */
int16_t Motor_GetRpm(uint8_t addr, int32_t *out_rpm)
{
    __IO uint16_t *pStatus;
    __IO uint8_t  *pBuf;
    uint8_t  buf[5];
    uint8_t  i;
    uint16_t valU;
    int32_t  val;

    if ((out_rpm == NULL) || !Motor_AddrValid(addr))
    {
        return -1;
    }

    /* 必须成功收到过 0x35 转速应答（状态为 SUCCESSRECEIVE） */
    pStatus = Motor_VelStatusPtr(addr);
    pBuf    = Motor_VelBufPtr(addr);
    if ((pStatus == NULL) || (pBuf == NULL) || (*pStatus != SUCCESSRECEIVE))
    {
        return -1;
    }

    /* 逐字节快照（避免读 volatile 数组被中断撕裂） */
    for (i = 0; i < 5U; i++)
    {
        buf[i] = pBuf[i];
    }

    /* 帧头校验：0x35 转速帧 */
    if (buf[0] != 0x35U)
    {
        return -1;
    }

    /* 大端 16 位转速拼装（单位 RPM） */
    valU  = ((uint16_t)buf[2] << 8);
    valU |=  (uint16_t)buf[3];
    val   = (int32_t)valU;

    /* 符号位：buf[1]=0 为正，非 0 为负 */
    if (buf[1] != 0U)
    {
        val = -val;
    }

    /* 方向标定：标定为反的电机（正转 dir=REV，如 FR/RR），转速符号取反，
       使返回转速与"整车前进方向为正"一致，与 Motor_SetVel 的方向语义对齐 */
    if (Motor_DirFwd(addr) != MOTOR_DIR_FWD_DEFAULT)
    {
        val = -val;
    }

    *out_rpm = val;
    return 0;
}

/**
 * @brief  读取某轮实时位置（解析 0x36 帧缓冲，不发 CAN 帧，非阻塞）
 * @param  addr: 电机地址 1~4
 * @param  out_angle: 输出电机轴累计位置，有符号 32 位；失败时不改变
 * @retval 0=成功；-1=失败（地址/指针非法、未收到应答、帧头校验不过）
 * @note   帧布局（DLC=7）：buf[0]=0x36、buf[1]=符号(0=正/非0=负)、
 *         buf[2..5]=位置大端 32 位、buf[6]=0x6B（帧尾）。
 *         **该 32 位数值的物理单位待实测**：可能是电机轴脉冲（3200/转），
 *         也可能是 0.1°（3600/转），两者差 12.5%。
 *         本函数原样返回数值（不做单位换算），换算统一由上层
 *         bsp_mecanum 的 MEC_PULSE_SCALE_NUM/DEN 吸收。
 *         返回值为累计值，上层做位移时取两次采样的差值。
 *         符号已按方向标定宏取反（标定为反的 FR/RR 轮），使"整车前进"
 *         对应四轮增量同为正，与 Motor_SetVel / Motor_PosMove 语义对齐。
 */
int16_t Motor_GetAngle_x10deg(uint8_t addr, int32_t *out_angle)
{
    __IO uint16_t *pStatus;
    __IO uint8_t  *pBuf;
    uint8_t  buf[7];
    uint8_t  i;
    uint32_t valU;
    int32_t  val;

    if ((out_angle == NULL) || !Motor_AddrValid(addr))
    {
        return -1;
    }

    /* 必须成功收到过 0x36 位置应答（状态为 SUCCESSRECEIVE） */
    pStatus = Motor_StatusPtr(addr);
    pBuf    = Motor_EncodedBufPtr(addr);
    if ((pStatus == NULL) || (pBuf == NULL) || (*pStatus != SUCCESSRECEIVE))
    {
        return -1;
    }

    /* 逐字节快照（避免读 volatile 数组被中断撕裂） */
    for (i = 0; i < 7U; i++)
    {
        buf[i] = pBuf[i];
    }

    /* 帧头校验：0x36 位置帧 */
    if (buf[0] != 0x36U)
    {
        return -1;
    }

    /* 大端 32 位位置拼装 */
    valU  = ((uint32_t)buf[2] << 24);
    valU |= ((uint32_t)buf[3] << 16);
    valU |= ((uint32_t)buf[4] << 8);
    valU |=  (uint32_t)buf[5];
    val   = (int32_t)valU;

    /* 符号位：buf[1]=0 为正，非 0 为负 */
    if (buf[1] != 0U)
    {
        val = -val;
    }

    /* 方向标定：与 Motor_GetRpm 一致，标定为反的电机符号取反，
       使四轮增量与"整车前进为正"一致 */
    if (Motor_DirFwd(addr) != MOTOR_DIR_FWD_DEFAULT)
    {
        val = -val;
    }

    *out_angle = val;
    return 0;
}

/**
 * @brief  单轮位置模式（官方库 0xFD）：相对当前位置走指定脉冲后自停
 * @param  addr: 电机地址 1~4，超范围忽略并日志
 * @param  clk:  目标位移（有符号），正=正转方向、负=反转方向，0=忽略；
 *               单位与缩放**待实测**（见 Motor_GetAngle_x10deg 说明）
 * @param  vel:  运行速度，单位电机轴 RPM；**由调用方保证 ≤ MOTOR_RPM_MAX**
 * @param  acc:  加减速度档 0~255，限幅到 MOTOR_ACC_MAX；档位语义实测定
 * @retval 无
 * @note   内部调 Emm_V5_Pos_Control，运动标志 raF=2（相对当前实时位置做
 *         相对位置运动），多机同步 snF=false。加减速轨迹由电机固件内部
 *         规划（梯形/S 形），到位后电机自行停止，无 0xFE 急停冲击。
 *         正负脉冲映射到该轮的 MOTOR_xx_DIR_FWD/REV 方向宏
 *         （与 Motor_SetVel 同一套标定，整车方向标定不要在这里做）。
 *         **本函数不对 vel 单独钳位**：若在这里逐轮钳位，四轮比例会被
 *         削平（限幅前车之鉴）。限幅必须由上层 bsp_mecanum 按"四轮同比
 *         缩小"统一处理后再下发。
 */
void Motor_PosMove(uint8_t addr, int32_t clk, uint16_t vel, uint8_t acc)
{
    uint8_t  dir;
    uint32_t clkAbs;

    if (!Motor_AddrValid(addr))
    {
        Debug_Log("MOT", "posmove bad addr=%u", (unsigned int)addr);
        return;
    }

    /* clk=0 表示不动，直接忽略（避免发一条无意义的位置命令） */
    if (clk == 0)
    {
        Debug_Log("MOT", "addr=%u posmove clk=0 ignored", (unsigned int)addr);
        return;
    }

    /* 正负脉冲映射到该轮方向宏（标定对调处） */
    if (clk > 0)
    {
        dir    = Motor_DirFwd(addr);
        clkAbs = (uint32_t)clk;
    }
    else
    {
        dir    = Motor_DirRev(addr);
        clkAbs = (uint32_t)(-clk);
    }

    /* 加速度档语义化钳位（uint8_t 天然 <=255） */
    if (acc > (uint8_t)MOTOR_ACC_MAX)
    {
        acc = (uint8_t)MOTOR_ACC_MAX;
    }

    /* 等 **2 个**空邮箱：0xFD 是 13 字节，官方库会拆成 2 帧发，
       邮箱不够会静默丢后一包（电机只收到半条命令 → 不动） */
    Motor_TxMailboxWait(2U, (uint32_t)MOTOR_TX_PAIR_WAIT_MS);
    Emm_V5_Pos_Control(addr, dir, vel, acc, clkAbs, 2U, false);

    /* **本函数不打日志**：Debug_Log 走 UART5 逐字符阻塞（约 87µs/字符，一行 ~4ms），
       四轮成组下发时夹在四条命令之间会把第 4 轮拖后十几 ms（先发的先到先停）。
       日志统一由调用方（bsp_mecanum 的 Mecanum_MoveByPulse）在四条命令发完后打一行。 */
}
