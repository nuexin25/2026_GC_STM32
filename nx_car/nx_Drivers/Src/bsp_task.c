/**
 ******************************************************************************
 * @file    bsp_task.c
 * @brief   赛题流程引擎 + 运动原语 + 作业点（第1步：骨架完整可见，作业点留空桩）
 * @author  nx_car
 * @date    2026-10-07
 ******************************************************************************
 * @note   【本文件是后续所有赛题开发的主战场】
 *           第1步只做"骨架"：把整条流程（当前 47 步）按顺序写死在这里，
 *           做到**不接视觉、不接机械臂也能从第 1 步跑到最后一步**，
 *           并在赛题页上看到"跑到第几步"。
 *
 *         【现场点位改哪里】★
 *           下面 Task_Update() 的 switch 里，每个 GO(...) / Rotate(...) 的实参就是
 *           该作业点的相对坐标与朝向（单位 mm / deg），**直接改这些数字**。
 *           原点 (0,0) = 进赛题页那一刻的车身中心；+x = 出发点前方；+y = 出发点右侧；
 *           yaw 俯视逆时针为正。抽签后只改这些数字，不用动 bsp_task_cfg.h。
 *
 *         【运动怎么实现的（不重写）】
 *           GO()/Rotate() 只是**包装**：首次调用使能四轮并调
 *           Mission_NavGoTo() / Mission_NavRotate()（bsp_mission 里已验证的单段导航内核：
 *           转角闭环 + 同拍停稳 + 到位判定），之后每拍推进一次。
 *           超时（TASK_MOVE_TIMEOUT_MS）则打日志 + Task_Stop() + 返回 1 放行。
 *
 *         【流程引擎两个易错点（务必保留）】
 *           1) switch (g_step) 每拍都会走进同一个 case，必须用 g_stepTick 区分
 *              "刚进入本步（要下发命令）"与"本步进行中（只判完成）"；
 *           2) 每个动作都必须有超时：超时先 Task_Stop() 停车、再放行下一步，
 *              赛题只有一次启动机会，卡死 = 全场失败。
 *
 *         硬约束：全部非阻塞（禁 HAL_Delay / while 等待）；日志禁 %f（角度用整数度）；
 *                 不改 can / Emm_V5 / bsp_motor / bsp_mecanum / bsp_chassis / bsp_qr /
 *                 bsp_arm* 的现有函数签名。本文件 UTF-8 编码。
 ******************************************************************************
 */
#include "bsp_task.h"
#include "bsp_mission.h"     /* Mission_NavGoTo / Mission_NavRotate：复用已验证的导航内核 */
#include "bsp_motor.h"       /* Motor_Enable/Disable、MOTOR_ADDR_MIN/MAX（四轮使能/失能） */
#include "bsp_mecanum.h"     /* Mecanum_Stop()：四轮急停 */
#include "bsp_chassis.h"     /* Chassis_OdomReset/GetOdomX/GetOdomY/GetOdomYaw：相对坐标 */
#include "bsp_oled_ui.h"     /* Display_SetPage / Display_MissionRefresh / PAGE_MISSION */
#include "bsp_align.h"       /* Align_Start/IsDone/IsFailed：视觉闭环对位（第4步实做） */
#include "bsp_qr.h"          /* QR_IsNew / QR_GetCode / QR_ResetResult：扫码（作业点 QR 用） */
#include "bsp_rpi.h"         /* RPi_SendCode：把任务码发给树莓派；RPi_ReqRcp：进转盘区发 $RCP（协议 §3） */
#include "bsp_arm_action.h"  /* ArmAct_GotoTtReady/IsBusy/HasUnreached：机械臂摆到观察位 */
#include "bsp_tt.h"          /* TT_Start/IsBusy/IsDone/IsFailed/GetMapText：转盘取料序列 */
#include "bsp_imu.h"         /* IMU_StaticZeroTo：任务点静态置角（零漂治理） */
#include "bsp_debug.h"       /* Debug_Log */
#include <stdio.h>
#include <math.h>            /* cosf/sinf：GO_REL 的"车身系 -> 全局系"换算 */

/* ===================== 步骤名表（顺序必须与 bsp_task.h 的 TaskStep_t 完全一致） ===================== */
/* 缩写规则：去掉 T_ 前缀；RGH_ROT->RROT、STO_ROT->SROT、T_HOME_ROT->H_ROT；
 * 最长 9 个字符，保证 "45/45 B1_RGHPUT" 这种最长组合也不超过页面的 16 字符/行。
 * 名字排序跟枚举一致：每组都是"转向步(ROT) 在前、移动步(GO) 在后"。 */
static const char *const tskStepName[] = {
    /* S1 扫码 */
    "QR_ROT", "QR_GO", "QR_R1", "QR_T1", "QR_DO",
    /* 第一批 */
   "B1_TRAY" ,"B1_ROT" ,"B11_TRAY","B12_TRAY",
    "B1_PICK",
    "B1_RROT", "B1_RGH", "B11_RGH_GO", "B12_RGH",
    "B1_PLACE",
    "B1_PBACK",
     "B1_STO","B12_SROT", "B12_STO",
    "B1_RGHPUT",
    /* 第二批 */
    "B2_ROT", "B2_TRAY","B22_TRAY",
    "B2_PICK",
    "B2_RROT", "B2_RGH", "B22_RGH", "B2_RGHR",
    "B2_PLACE", "B2_PBACK",
    "B2_TOST", "B2_TSROT", "B2_TOST2",
    "B2_STACK",
    "B2_PBACK0",
    "B2_SROT", "B2_STO",
    /* 收尾 */
    "H_ROT", "HOME", "STAT", "END"
};

/* 步骤总数：由上面的表自动得出（增删步骤只需同步改 TaskStep_t 与本表） */
#define TSK_STEP_COUNT      ((uint16_t)(sizeof(tskStepName) / sizeof(tskStepName[0])))

/* ===================== 内部状态 ===================== */

/* ---- 流程引擎 ---- */
static uint8_t     tskActive   = 0U;                  /* 赛题页活动标志：1=本页活动 */
static TaskMode_t  g_mode      = TASK_MODE_STEP;      /* 运行模式（KEY3 切换） */
static TaskState_t g_state     = TASK_STATE_IDLE;     /* 总状态 */
static uint16_t    g_step      = 0U;                  /* 当前步骤下标（0 起） */
static uint32_t    g_stepTick  = 0U;                  /* 本步进入时刻；**0 = 本步刚进入、还没打时间戳** */
static uint32_t    g_runTick   = 0U;                  /* 启动/执行时刻（启动延迟计时用） */
static uint8_t     g_startHold = 0U;                  /* 1=一键启动后的等待中（AUTO 才有） */
static uint16_t    g_failCount = 0U;                  /* 超时/失败次数（统计用，日志输出） */

/* ---- 四轮使能状态：GO/Rotate 首次进入时使能一次，Task_Stop 后重新使能 ---- */
static uint8_t     tskWheelsOn = 0U;                  /* 1=四轮已使能 */

/* ---- 运动原语状态（区分"首次进入要下发"与"推进中只判完成"）---- */
static uint8_t     goActive    = 0U;                  /* GO 本段是否已开始 */
static uint32_t    goTick      = 0U;                  /* GO 本段开始时刻（超时计时） */
static uint8_t     rotActive   = 0U;                  /* Rotate 本段是否已开始 */
static uint32_t    rotTick     = 0U;                  /* Rotate 本段开始时刻 */
static uint8_t     relActive   = 0U;                  /* GO_REL 是否已锁存绝对目标 */
static int32_t     relX        = 0;                   /* GO_REL 锁存的绝对目标 X（mm） */
static int32_t     relY        = 0;                   /* GO_REL 锁存的绝对目标 Y（mm） */
static uint8_t     relRotActive = 0U;                 /* Rotate_REL 是否已锁存绝对目标 */
static float       relYawTarget = 0.0f;               /* Rotate_REL 锁存的绝对目标朝向（deg） */
static uint8_t     waitActive  = 0U;                  /* Wait 是否已开始计时 */
static uint32_t    waitTick    = 0U;                  /* Wait 开始时刻 */
/* ★【2026-10-21】1 = 本段 GO 快到任务点时提前把手臂摆到观察位（见 Tsk_OnStepEnter） */
static uint8_t     tskArmPre   = 0U;

/* ★【2026-10-21】任务点"开始执行即置角"（STORE BACK=180）的挂起重试：
   刚走到点位时车可能还有残余转动（|gz| > 30°/s），IMU 会拒绝置角；
   这里每拍重试，车停稳就立刻置角，最多等 TASK_YAW_ZERO_WAIT_MS（绝不迟到乱置）。 */
#define TASK_YAW_ZERO_WAIT_MS   3000U
static uint8_t     tskZeroPend = 0U;                  /* 1=有一段置角正挂起等车停稳 */
static float       tskZeroRef  = 0.0f;                /* 挂起的目标角（deg） */
static uint32_t    tskZeroTick = 0U;                  /* 挂起时刻（超时计时） */

/* ---- 页面第 2 行"本步目标"：由最近一次 GO/Rotate 锁存（供显示，不影响控制）---- */
static uint8_t     g_tgtValid  = 0U;                  /* 1=已有目标可显示 */
static int32_t     g_tgtX      = 0;                   /* 最近一次 GO 的目标 X（mm） */
static int32_t     g_tgtY      = 0;                   /* 最近一次 GO 的目标 Y（mm） */
static int32_t     g_tgtYaw    = 0;                   /* 最近一次 Rotate 的目标朝向（整数 deg） */

/* ---- 视觉对位（Align()）状态：第1步只有 ALIGN_TURNTABLE 接到 bsp_align ---- */
static uint8_t     alActive    = 0U;                  /* 1=对位已启动（超时由调用方/步骤超时兜底） */

/* ---- 作业点 QR：边往前匀速前进边扫码的状态 ---- */
static uint8_t     qrActive    = 0U;                  /* 1=正在"前进 + 扫码" */
static uint32_t    qrTick      = 0U;                  /* 起步时刻（扫不到的超时计时） */
static uint32_t    qrLogTick   = 0U;                  /* 绝对坐标日志上次打印时刻 */

/* ---- 作业点 转盘取料（PICK_TRAY_ALL）：发 $RCP -> 摆臂 -> 对位 + 连抓 3 件 ---- */
static uint8_t     trayPickActive  = 0U;              /* 1=本段已进入（$RCP 已发、摆臂已启动） */
static uint8_t     trayPickStarted = 0U;              /* 1=TT 序列已启动（臂已到位后才会置 1） */
static uint8_t     trayPickBatch   = 1U;              /* ★本批号 1/2：决定用任务码的第 1 组还是第 3 组颜色 */

/* ---- 作业点 打靶放置（PLACE_STORE_ALL）：驱动 bsp_tt 的 SP_ 序列（$SHOT 对位 + 搬 3 件） ---- */
static uint8_t     placeActive  = 0U;                 /* 1=本段已进入 */
static uint8_t     placeStarted = 0U;                 /* 1=SP 序列已启动 */

/* ---- 作业点 打靶取回（PLACE_BACK_ALL，STORE BACK）：驱动 bsp_tt 的 SB_ 序列 ---- */
static uint8_t     backActive   = 0U;                 /* 1=本段已进入 */
static uint8_t     backStarted  = 0U;                 /* 1=SB 序列已启动 */

/* ---- 收尾 STAT：是否已下发"手臂收回初始状态" ---- */
static uint8_t     statParked   = 0U;                 /* 1=ArmAct_Park() 已启动 */

/* ---- 任务码解析结果（协议 §5）：由 QR_GetCode() 搬运 + 颜色/靶位号范围复核 ---- */
static Task_Code_t taskCode;                          /* Task_GetCode() 返回其指针 */
static uint8_t     taskCodeLogged = 0xFFU;            /* 上次记日志的合法性（0xFF=尚未记过，避免刷屏） */

/* ---- 车载位↔颜色表（协议 §3.2）：下标 = 车载号位 1~3（0 不用）；值为颜色 1~4，0=空 ---- */
static uint8_t     taskSlotColor[4] = { 0U, 0U, 0U, 0U };

/* ===================== 内部工具 ===================== */

/**
 * @brief  浮点转 int32（四舍五入，避免正负都向 0 截断带来的 1mm 偏差）
 * @param  v: 输入值
 * @retval 最接近的整数
 */
static int32_t Tsk_RoundF(float v)
{
    return (int32_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

/**
 * @brief  把角度归一化到 (-180, 180]
 * @param  d: 输入角度，单位 deg
 * @retval 归一化后的角度
 */
static float Tsk_NormDeg(float d)
{
    while (d > 180.0f)  { d -= 360.0f; }
    while (d <= -180.0f) { d += 360.0f; }
    return d;
}

/**
 * @brief  使能四轮（幂等：已使能则直接返回）
 * @param  无
 * @retval 无
 * @note   位置模式必须先使能，否则电机不收位置命令。只在原语首次进入时调用一次，
 *         不每拍调用（避免高频 CAN 命令）。
 */
static void Tsk_EnsureWheels(void)
{
    uint8_t addr;

    if (tskWheelsOn != 0U)
    {
        return;
    }

    for (addr = MOTOR_ADDR_MIN; addr <= MOTOR_ADDR_MAX; addr++)
    {
        Motor_Enable(addr, 1U);
    }
    tskWheelsOn = 1U;
}

/**
 * @brief  四轮同时按同一有符号速度转：>0 沿**车身前方**前进、<0 后退、=0 停车
 * @param  rpm: 车轮转速，单位 RPM（正 = 沿**小车车身的前方**前进）
 * @retval 无
 * @note   ★ 这里的"前进"是**小车车身的前方**（车头朝哪就往哪走），
 *         **不是**绝对坐标系里的某个固定方向；绝对坐标系只负责"一直算位置"。
 *         麦轮**纯前进**时四轮同向同速，所以对四轮下同一个有符号 rpm 即可
 *         （每轮"正转"对应的实际 dir 由 bsp_motor 的方向宏映射，已在 P3 标定过）。
 *         **速度模式下发一次就一直保持**，不需要每拍重发 —— 所以只在"起步"和
 *         "停车"各调一次，不会给 CAN 总线加负担（也不违反 50ms 节拍约定）。
 */
static void Tsk_WheelsVel(int16_t rpm)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t)MOTOR_NUM; i++)
    {
        Motor_SetVel((uint8_t)(MOTOR_ADDR_MIN + i), rpm, (uint8_t)QR_DRIVE_ACC);
    }
}

/**
 * @brief  推进到下一步（STEP 停下等按键 / AUTO 继续；到 T_END 则停车 + ALLDONE）
 * @param  无
 * @retval 无
 * @note   **两种模式唯一的分叉点**：
 *           STEP -> 状态置 DONE，等人工再按 KEY5（步骤指针已经指向下一步）；
 *           AUTO -> 直接继续下一步，由主循环继续调度。
 *         超时降级也走本函数（先停车再放行）。
 */
static void Tsk_Advance(void)
{
    /* 下一步就是收尾：直接停车进 ALLDONE，不再走 switch 的 T_END 分支 */
    if ((uint16_t)(g_step + 1U) >= (uint16_t)T_END)
    {
        g_step     = (uint16_t)T_END;
        g_stepTick = 0U;
        Task_Stop();
        g_state = TASK_STATE_ALLDONE;
        Debug_Log("TASK", "all done, fail=%u", (unsigned int)g_failCount);
        Display_MissionRefresh();
        return;
    }

    g_step++;
    g_stepTick = 0U;            /* 让下一步"重新进入"（下一拍打时间戳） */

    if (g_mode == TASK_MODE_STEP)
    {
        uint8_t addr;

        g_state = TASK_STATE_DONE;

        /* 单步调试：每步做完就**失能四轮**，方便现场手动挪车 / 实测，为下一步取参数。
           下一步执行时 GO/Rotate 会重新使能（tskWheelsOn 已清 0），不需要手动恢复。
           连续（AUTO）模式不失能，否则步骤之间跑不连贯。 */
        for (addr = MOTOR_ADDR_MIN; addr <= MOTOR_ADDR_MAX; addr++)
        {
            Motor_Disable(addr);
        }
        tskWheelsOn = 0U;

        Debug_Log("TASK", "step done -> %u/%u %s (wheels disabled)",
                  (unsigned int)(g_step + 1U), (unsigned int)TSK_STEP_COUNT,
                  Task_GetStepName(g_step));
    }

    Display_MissionRefresh();
}

/**
 * @brief  开始执行/一键启动（KEY5 调用）
 * @param  无
 * @retval 无
 * @note   AUTO：置"启动延迟"标志，等 TASK_START_DELAY_MS 后再发第一步命令；
 *         STEP：不延迟，立刻执行当前步。
 *         已在 T_END（跑完）时再按 = 从头再来。
 */
static void Tsk_Run(void)
{
    if (g_state == TASK_STATE_RUN)
    {
        return;                 /* 执行中重复调用：忽略 */
    }

    if ((uint16_t)g_step >= (uint16_t)T_END)
    {
        g_step = 0U;            /* 已跑完再按：从头再来 */
    }

    g_stepTick  = 0U;
    g_runTick   = HAL_GetTick();
    g_startHold = (g_mode == TASK_MODE_AUTO) ? 1U : 0U;
    g_state     = TASK_STATE_RUN;

    /* 先使能四轮（读不到位置也不拒绝：由每步超时兜底放行） */
    Tsk_EnsureWheels();

    Debug_Log("TASK", "start %s step %u/%u %s",
              (g_mode == TASK_MODE_AUTO) ? "AUTO" : "STEP",
              (unsigned int)(g_step + 1U), (unsigned int)TSK_STEP_COUNT,
              Task_GetStepName(g_step));

    Display_MissionRefresh();
}

/* ===================== 任务码解析 + 车载位表（协议 v2.0 §5 / §3.2） ===================== */

/**
 * @brief  把 bsp_qr 的解析结果搬进 Task_Code_t，并复核颜色/靶位号范围
 * @param  无
 * @retval 无
 * @note   复用 bsp_qr 的成帧与 15 字符格式校验（QR_ValidateFrame 已保证 4 组×3 位数字、
 *         '+' 分隔、各数字 1~4），此处只做：① 字段搬运；② 靶位号范围复核
 *         （颜色位须 1~4、放置位须为靶位号 1~3，分别按 QR_COLOR_* / TASK_TGT_* 再判一次）。
 *         非法时各字段清零、valid=0，并（合法性发生变化时）打一条日志；**不崩溃**。
 */
static void Tsk_ParseCode(void)
{
    const QR_TaskCode_t *qr = QR_GetCode();
    uint8_t ok = qr->valid;
    uint8_t i;

    if (ok != 0U)
    {
        for (i = 0U; i < 3U; i++)
        {
            taskCode.color1[i] = qr->batch1Colors[i];
            taskCode.place1[i] = qr->batch1Pos[i];
            taskCode.color2[i] = qr->batch2Colors[i];
            taskCode.place2[i] = qr->batch2Pos[i];
        }

        /* 颜色位 1~4（协议 §5） */
        for (i = 0U; i < 3U; i++)
        {
            if ((taskCode.color1[i] < QR_COLOR_MIN) || (taskCode.color1[i] > QR_COLOR_MAX) ||
                (taskCode.color2[i] < QR_COLOR_MIN) || (taskCode.color2[i] > QR_COLOR_MAX))
            {
                ok = 0U;
            }
        }

        /* 放置位 = 靶位号 TASK_TGT_MIN~TASK_TGT_MAX（3 个靶位，1~3） */
        for (i = 0U; i < 3U; i++)
        {
            if ((taskCode.place1[i] < TASK_TGT_MIN) || (taskCode.place1[i] > TASK_TGT_MAX) ||
                (taskCode.place2[i] < TASK_TGT_MIN) || (taskCode.place2[i] > TASK_TGT_MAX))
            {
                ok = 0U;
            }
        }
    }

    if (ok == 0U)
    {
        for (i = 0U; i < 3U; i++)
        {
            taskCode.color1[i] = 0U;  taskCode.place1[i] = 0U;
            taskCode.color2[i] = 0U;  taskCode.place2[i] = 0U;
        }
    }
    taskCode.valid = ok;

    /* 合法性发生变化时才记日志（避免每拍刷屏） */
    if (ok != taskCodeLogged)
    {
        taskCodeLogged = ok;
        if (ok != 0U)
        {
            Debug_Log("TASK", "code ok: C%d%d%d P%d%d%d / C%d%d%d P%d%d%d",
                      (int)taskCode.color1[0], (int)taskCode.color1[1], (int)taskCode.color1[2],
                      (int)taskCode.place1[0], (int)taskCode.place1[1], (int)taskCode.place1[2],
                      (int)taskCode.color2[0], (int)taskCode.color2[1], (int)taskCode.color2[2],
                      (int)taskCode.place2[0], (int)taskCode.place2[1], (int)taskCode.place2[2]);
        }
        else
        {
            Debug_Log("TASK", "code invalid");
        }
    }
}

/**
 * @brief  清零"车载位↔颜色"表（进入第一批取料前调用）
 * @param  无
 * @retval 无
 */
static void Tsk_ClearCarSlots(void)
{
    taskSlotColor[1] = 0U;
    taskSlotColor[2] = 0U;
    taskSlotColor[3] = 0U;
}

/**
 * @brief  取当前任务码（解析结果）
 * @param  无
 * @retval 结构体常量指针
 */
const Task_Code_t *Task_GetCode(void)
{
    Tsk_ParseCode();
    return &taskCode;
}

/**
 * @brief  取某批第 idx 件的目标颜色
 * @param  batch: 批次 1/2；idx: 件号 0~2
 * @retval 颜色 1~4；越界/非法返回 0
 */
uint8_t Task_ColorOfBatch(uint8_t batch, uint8_t idx)
{
    const Task_Code_t *c;

    if (idx > 2U)
    {
        return 0U;
    }

    c = Task_GetCode();
    if (c->valid == 0U)
    {
        return 0U;
    }

    if (batch == 1U)
    {
        return c->color1[idx];
    }
    if (batch == 2U)
    {
        return c->color2[idx];
    }
    return 0U;
}

/**
 * @brief  取某批第 idx 件的放置靶位号
 * @param  batch: 批次 1/2；idx: 件号 0~2
 * @retval 靶位号 1~3；越界/非法返回 0
 */
uint8_t Task_PlaceOfBatch(uint8_t batch, uint8_t idx)
{
    const Task_Code_t *c;

    if (idx > 2U)
    {
        return 0U;
    }

    c = Task_GetCode();
    if (c->valid == 0U)
    {
        return 0U;
    }

    if (batch == 1U)
    {
        return c->place1[idx];
    }
    if (batch == 2U)
    {
        return c->place2[idx];
    }
    return 0U;
}

/**
 * @brief  记录车载某号位上物料的颜色
 * @param  slot : 车载号位 1~3
 * @param  color: 颜色 1~4（0=清空该位）
 * @retval 无
 */
void Task_SetCarSlotColor(uint8_t slot, uint8_t color)
{
    if ((slot < 1U) || (slot > 3U))
    {
        Debug_Log("TASK", "car slot %u invalid (1~3)", (unsigned int)slot);
        return;
    }
    taskSlotColor[slot] = color;
}

/**
 * @brief  取车载某号位上物料的颜色
 * @param  slot: 车载号位 1~3
 * @retval 颜色 1~4；未放料/越界返回 0
 */
uint8_t Task_GetCarSlotColor(uint8_t slot)
{
    if ((slot < 1U) || (slot > 3U))
    {
        return 0U;
    }
    return taskSlotColor[slot];
}

/**
 * @brief  按颜色反查它在车载哪个号位
 * @param  color: 颜色 1~4
 * @retval 车载号位 1~3；没找到返回 -1
 */
int8_t Task_FindSlotByColor(uint8_t color)
{
    uint8_t s;

    if (color == 0U)
    {
        return -1;
    }

    for (s = 1U; s <= 3U; s++)
    {
        if (taskSlotColor[s] == color)
        {
            return (int8_t)s;
        }
    }
    return -1;
}

/* ===================== 对外接口：生命周期 ===================== */

/**
 * @brief  赛题模块初始化（上电调一次）
 * @param  无
 * @retval 无
 */
void Task_Init(void)
{
    tskActive    = 0U;
    g_mode       = TASK_MODE_STEP;
    g_state      = TASK_STATE_IDLE;
    g_step       = 0U;
    g_stepTick   = 0U;
    g_runTick    = 0U;
    g_startHold  = 0U;
    g_failCount  = 0U;
    tskWheelsOn  = 0U;
    goActive     = 0U;
    rotActive    = 0U;
    relActive    = 0U;
    relRotActive = 0U;
    waitActive   = 0U;
    alActive     = 0U;
    qrActive     = 0U;
    trayPickActive  = 0U;
    trayPickStarted = 0U;
    g_tgtValid   = 0U;

    /* 任务码/车载位表：上电清零 */
    taskCode.valid = 0U;
    taskCode.color1[0] = 0U;  taskCode.color1[1] = 0U;  taskCode.color1[2] = 0U;
    taskCode.place1[0] = 0U;  taskCode.place1[1] = 0U;  taskCode.place1[2] = 0U;
    taskCode.color2[0] = 0U;  taskCode.color2[1] = 0U;  taskCode.color2[2] = 0U;
    taskCode.place2[0] = 0U;  taskCode.place2[1] = 0U;  taskCode.place2[2] = 0U;
    taskCodeLogged = 0xFFU;
    Tsk_ClearCarSlots();

    Debug_Log("TASK", "init, steps=%u", (unsigned int)TSK_STEP_COUNT);
}

/**
 * @brief  进入赛题页（主菜单 MISSION 项调用）
 * @param  无
 * @retval 无
 */
void Task_Enter(void)
{
    /* 上次残留运行态：先停掉，保证页面显示与电机状态一致 */
    if (g_state == TASK_STATE_RUN)
    {
        Task_Stop();
    }

    /* 复位状态，但**保留当前步号**（反复调同一步时不用重新选） */
    g_state     = TASK_STATE_IDLE;
    g_stepTick  = 0U;
    g_startHold = 0U;
    goActive    = 0U;
    rotActive   = 0U;
    relActive   = 0U;
    relRotActive = 0U;
    waitActive  = 0U;
    alActive    = 0U;

    /* 原点 = 出发点：把**当前位置/朝向**当作出发点；并失能四轮（使能只在 Tsk_Run） */
    Chassis_OdomReset();
    Task_Stop();                /* 内部含四轮急停 + 失能 + 清原语状态 */

    tskActive = 1U;
    Display_SetPage(PAGE_MISSION);

    Debug_Log("TASK", "enter, step=%u/%u odom zeroed",
              (unsigned int)(g_step + 1U), (unsigned int)TSK_STEP_COUNT);
}

/**
 * @brief  退出赛题页（KEY4 调用）
 * @param  无
 * @retval 无
 */
void Task_Exit(void)
{
    Task_Stop();
    tskActive = 0U;
    Debug_Log("TASK", "exit");
}

/**
 * @brief  串口测试用：显式设置运行模式（0=STEP 1=AUTO）
 * @note   KEY3 是"切换"，串口远程测试时没法知道当前是哪个，用这个直接定死。
 */
void Task_CmdSetMode(uint8_t autoMode)
{
    g_mode = (autoMode != 0U) ? TASK_MODE_AUTO : TASK_MODE_STEP;
    Debug_Log("TASK", "mode %s (cmd)",
              (g_mode == TASK_MODE_AUTO) ? "AUTO" : "STEP");
}

/**
 * @brief  立即中止：四轮急停 + 失能 + 清所有原语状态
 * @param  无
 * @retval 无
 */
void Task_Stop(void)
{
    uint8_t addr;

    Mecanum_Stop();             /* 四轮 0xFE 同拍急停 */

    for (addr = MOTOR_ADDR_MIN; addr <= MOTOR_ADDR_MAX; addr++)
    {
        Motor_Disable(addr);    /* 失能：避免长时间抱轴发热 */
    }
    tskWheelsOn = 0U;

    /* 解冻平移坐标：旋转步中途被急停/超时打断的话，冻结状态会一直残留下去，
       之后就成了"车在走、坐标不动"。 */
    Chassis_OdomHoldXY(0U);

    /* 中止可能还在后台跑的"视觉对位 / 机械臂动作 / 作业点序列（转盘取料 TT_ / 打靶放置 SP_）"：
       本页有整段作业点（会驱动对位 + 机械臂 + bsp_tt 序列），
       急停 / 单步超时 / 退出页面时若不中止它们，机械臂会继续动、序列会继续抓 ——
       与"停车 + 失能"的语义不符且危险。这些 Abort 都是幂等的（没在跑就什么都不做）。 */
    TT_Abort();
    SP_Abort();                 /* ★打靶放置序列也必须一并中止（漏了它 SP_ 会在后台继续跑） */
    SB_Abort();                 /* ★打靶取回序列（STORE BACK）同样要中止 */
    Align_Abort();
    ArmAct_Abort();

    goActive     = 0U;
    rotActive    = 0U;
    relActive    = 0U;
    relRotActive = 0U;
    waitActive   = 0U;
    alActive     = 0U;
    qrActive     = 0U;
    trayPickActive  = 0U;
    trayPickStarted = 0U;
    placeActive     = 0U;       /* ★同上：不清就会留着"已启动"的残留，下次直接跳过启动 */
    placeStarted    = 0U;
    backActive      = 0U;       /* ★STORE BACK 同理 */
    backStarted     = 0U;
    statParked      = 0U;       /* ★收尾收臂：重跑流程要重新下发 */
}

/**
 * @brief  赛题页是否活动
 * @param  无
 * @retval 1=本页活动
 */
uint8_t Task_IsActive(void)
{
    return tskActive;
}

/* ===================== 对外接口：运动原语 ===================== */

/**
 * @brief  ★规则1：快到任务点时把手臂**提前摆到观察位**
 * @param  x_mm/y_mm: 当前段的目标点（mm）
 * @retval 无
 * @note   只在"进任务点前最后一段运动"里生效（由 Tsk_OnStepEnter 置 tskArmPre）。
 *         GO / Rotate **两种步都要调**：有些任务点前最后一步是原地转向（如 ROUGH PLACE
 *         前的 `Rotate(180)`），只挂在 GO 里会出现"手臂一直没提前摆"。
 *         臂忙（上一段还没跑完）就下一拍再试，不阻塞底盘。
 */
static void Tsk_ArmPreTry(int32_t x_mm, int32_t y_mm)
{
#if (TASK_ARM_PRE_OBSERVE_MM > 0)
    float dx;
    float dy;
    float d;

    if (tskArmPre == 0U)
    {
        return;
    }

    dx = (float)x_mm - Chassis_GetOdomX();
    dy = (float)y_mm - Chassis_GetOdomY();
    d  = sqrtf((dx * dx) + (dy * dy));

    if (d > (float)TASK_ARM_PRE_OBSERVE_MM)
    {
        return;
    }

    if (ArmAct_GotoTtReady() != 0U)     /* 臂忙就下一拍再试 */
    {
        tskArmPre = 0U;
        Debug_Log("TASK", "arm -> observe early (dist=%dmm)", (int)d);
    }
#else
    (void)x_mm;
    (void)y_mm;
#endif
}

/**
 * @brief  走到相对坐标 (x,y)（平移到点，不含转向）
 * @param  x_mm: 目标 X，单位 mm
 * @param  y_mm: 目标 Y，单位 mm
 * @retval 1=已到达；0=进行中
 */
uint8_t GO(int32_t x_mm, int32_t y_mm)
{
    uint32_t now = HAL_GetTick();

    if (goActive == 0U)
    {
        goActive = 1U;
        goTick   = now;
        Tsk_EnsureWheels();
        g_tgtX     = x_mm;      /* 页面第 2 行显示用 */
        g_tgtY     = y_mm;
        g_tgtValid = 1U;
        Debug_Log("TASK", "step%u GO %d,%d (now %d,%d h%d)",
                  (unsigned int)g_step, (int)x_mm, (int)y_mm,
                  (int)Chassis_GetOdomX(), (int)Chassis_GetOdomY(),
                  (int)Chassis_GetOdomYaw());
    }

    /* ★【2026-10-21 规则1】快到任务点时，手臂**提前去观察位**，省掉到点后再等手臂的时间。
       只在"进任务点前的最后一段运动"里生效（由 Tsk_OnStepEnter 置 tskArmPre）。 */
    Tsk_ArmPreTry(x_mm, y_mm);

    if (Mission_NavGoTo(x_mm, y_mm) != 0U)
    {
        goActive = 0U;
        return 1U;
    }

    /* 超时降级：先停车再放行，绝不卡死（框架指南 C3/C4） */
    if ((now - goTick) >= TASK_MOVE_TIMEOUT_MS)
    {
        Debug_Log("TASK", "step%u GO %d,%d timeout", (unsigned int)g_step, (int)x_mm, (int)y_mm);
        Task_Stop();
        goActive = 0U;
        g_failCount++;
        return 1U;
    }

    return 0U;
}

/**
 * @brief  原地转到相对朝向 deg（逆时针为正）
 * @param  deg: 目标朝向，单位 deg
 * @retval 1=已到位；0=进行中
 */
uint8_t Rotate(float deg)
{
    uint32_t now = HAL_GetTick();

    if (rotActive == 0U)
    {
        rotActive  = 1U;
        rotTick    = now;
        Tsk_EnsureWheels();
        g_tgtYaw   = (int32_t)deg;   /* 页面第 2 行显示用（整数度） */
        g_tgtValid = 1U;
        Debug_Log("TASK", "step%u ROT %d (now h%d)",
                  (unsigned int)g_step, (int)deg, (int)Chassis_GetOdomYaw());
    }

    /* ★【2026-10-21 规则1】任务点前的最后一步若只是**原地转向**，也要在这里提前摆臂：
       此刻车已经在目标点上（d≈0），直接判定"已到达"，不再看距离。 */
    Tsk_ArmPreTry((int32_t)Chassis_GetOdomX(), (int32_t)Chassis_GetOdomY());

    if (Mission_NavRotate(deg) != 0U)
    {
        rotActive = 0U;
        return 1U;
    }

    if ((now - rotTick) >= TASK_MOVE_TIMEOUT_MS)
    {
        Debug_Log("TASK", "step%u ROT %d timeout", (unsigned int)g_step, (int)deg);
        Task_Stop();
        rotActive = 0U;
        g_failCount++;
        return 1U;
    }

    return 0U;
}

/**
 * @brief  原地转到 rotateDeg，同时把**绝对坐标计算强制按 coordDeg**（★两个域分开给角度）
 * @param  rotateDeg: **要转到的角度**（照现场写法填，含零漂补偿；例 98 = 真 90 + 补 8）
 * @param  coordDeg : **绝对坐标计算要用的角度**（强制设定；例 90）
 * @retval 1=已到位；0=进行中
 * @note   ★【2026-10-11 用户要求（最终口径）】"**转向转 98°，绝对值坐标按 90°（强制设定）计算**"：
 *           · **旋转域**：照 `rotateDeg`(98) 下发 ⇒ 车身物理朝向 = 真 90°；
 *           · **坐标域**：里程计位置与"全局→车身"变换按 `coordDeg`(90) 算
 *             （内部即 `Chassis_SetYawCompDeg(rotateDeg − coordDeg)`：
 *              补偿量 = 你写的角 − 坐标要用的角，= 8；此后 θ_true = θ_imu − 8）。
 *         ⇒ 车头正、落点也对，不会被"漂移后的 98°"整体带偏。
 *         ⚠ 坐标补偿量**跨步保持**，直到下一次 `Rotate_Comp()`；不想再补就写 `Rotate_Comp(deg, deg)`（差=0）。
 *         非阻塞；首次调用锁存，之后每拍推进，与 `Rotate()` 同一套。
 *         用法：`if (Rotate_Comp(98.0f, 90.0f) != 0U) { Tsk_Advance(); } break;`
 */
uint8_t Rotate_Comp(float rotateDeg, float coordDeg)
{
    /* 坐标域：把"补偿量"设成 (要转的角 − 坐标要用的角)，坐标变换即按 coordDeg 走 */
    Chassis_SetYawCompDeg(rotateDeg - coordDeg);
    /* 旋转域：照现场写法的角度下发（98），保证车身物理朝向 = 真 90° */
    return Rotate(rotateDeg);
}

/**
 * @brief  相对**当前车身**增量平移 (dx,dy)：沿车头前进 dx、沿车身右侧横移 dy
 * @param  dx_mm: 沿**车头方向**的位移（前进为正），单位 mm
 * @param  dy_mm: 沿**车身右侧**的位移（右移为正），单位 mm
 * @retval 1=已到达；0=进行中
 * @note   ★ 现场路线里 dy 一律填 0（"直着开过去"），横移靠"先转到位"来实现。
 *         换算用的是 bsp_chassis 里程计的"正变换"（见 Chassis_OdomUpdate），保证自洽：
 *           全局增量 X = dx·cosθ + dy·sinθ
 *           全局增量 Y = −dx·sinθ + dy·cosθ        （θ = 当前实测 yaw）
 *         再加到当前 ODOM 坐标上得到**绝对目标点**，交给 GO() 去闭环。
 *         **首次调用锁存**：不锁存的话目标会跟着车走，永远到不了。
 *         因为目标是"按当前实测位置现算"的，所以**位置误差不会逐段累积**。
 */
uint8_t GO_REL(int32_t dx_mm, int32_t dy_mm)
{
    /* 没有位移（数字还留着 0 占位）：什么都不做、立刻放行，避免下发无意义的微动 */
    if ((dx_mm == 0) && (dy_mm == 0))
    {
        return 1U;
    }

    if (relActive == 0U)
    {
        float th = Chassis_GetOdomYawTrue() * MEC_RAD_PER_DEG;   /* θ：真实坐标系（已扣坐标补偿） */
        float c  = cosf(th);
        float s  = sinf(th);

        /* 车身系 -> 全局系（正变换），再加到当前 ODOM 坐标上得到绝对目标 */
        relX = (int32_t)Chassis_GetOdomX()
             + Tsk_RoundF(((float)dx_mm * c) + ((float)dy_mm * s));
        relY = (int32_t)Chassis_GetOdomY()
             + Tsk_RoundF((-(float)dx_mm * s) + ((float)dy_mm * c));

        relActive = 1U;
        Debug_Log("TASK", "step%u GO_REL fwd=%d right=%d h=%d -> abs %d,%d",
                  (unsigned int)g_step, (int)dx_mm, (int)dy_mm,
                  (int)Chassis_GetOdomYaw(), (int)relX, (int)relY);
    }

    if (GO(relX, relY) != 0U)
    {
        relActive = 0U;
        return 1U;
    }

    return 0U;
}

/**
 * @brief  相对当前朝向增量旋转 ddeg
 * @param  ddeg: 增量角度，单位 deg（逆时针为正）
 * @retval 1=已到位；0=进行中
 * @note   换算：绝对目标朝向 = 归一化(当前 yaw + 增量)。**首次调用锁存**。
 */
uint8_t Rotate_REL(float ddeg)
{
    if (relRotActive == 0U)
    {
        relYawTarget = Tsk_NormDeg(Chassis_GetOdomYaw() + ddeg);
        relRotActive = 1U;
        Debug_Log("TASK", "step%u ROT_REL %d -> h%d",
                  (unsigned int)g_step, (int)ddeg, (int)relYawTarget);
    }

    if (Rotate(relYawTarget) != 0U)
    {
        relRotActive = 0U;
        return 1U;
    }

    return 0U;
}

/**
 * @brief  非阻塞等待
 * @param  ms: 等待时长，单位 ms（0 = 立即完成）
 * @retval 1=时间到；0=未到
 */
uint8_t Wait(uint16_t ms)
{
    if (ms == 0U)
    {
        return 1U;
    }

    if (waitActive == 0U)
    {
        waitActive = 1U;
        waitTick   = HAL_GetTick();
        return 0U;
    }

    if ((HAL_GetTick() - waitTick) >= (uint32_t)ms)
    {
        waitActive = 0U;
        return 1U;
    }

    return 0U;
}

/* ===================== 流程引擎（本次核心） ===================== */

/**
 * @brief  取某一步的**单步超时**（按步骤区分，默认用 TASK_STEP_TIMEOUT_MS）
 * @param  step: 步骤下标（TaskStep_t）
 * @retval 超时，单位 ms
 * @note   只有"整段做完"的两步需要更长的超时（它们内部串了"对位 + 搬 3 件"整条链，
 *         通用的单步超时必然不够、会把正常流程误判成超时）：
 *           · `T_B1_TRAY_PICK`（转盘取料）-> `TASK_TRAY_PICK_TIMEOUT_MS`
 *           · `T_B1_PLACE`（打靶放置）  -> `TASK_STORE_PLACE_TIMEOUT_MS`
 *           · `T_B1_PBACK`（打靶取回）  -> `TASK_STORE_BACK_TIMEOUT_MS`
 *         其余步骤一律用通用值，保持"卡住就尽快放行"的兜底强度。
 */
static uint32_t Tsk_StepTimeoutMs(uint16_t step)
{
    if ((step == (uint16_t)T_B1_TRAY_PICK) || (step == (uint16_t)T_B2_TRAY_PICK))
    {
        /* ★两批 TRAY PICK 都跑"整段（3 件）"（`PICK_TRAY_ALL()`），必须给整段超时。 */
        return (uint32_t)TASK_TRAY_PICK_TIMEOUT_MS;
    }
    if ((step == (uint16_t)T_B1_PLACE) || (step == (uint16_t)T_B2_PLACE))
    {
        return (uint32_t)TASK_STORE_PLACE_TIMEOUT_MS;
    }
    if ((step == (uint16_t)T_B1_PBACK) || (step == (uint16_t)T_B2_PBACK))
    {
        return (uint32_t)TASK_STORE_BACK_TIMEOUT_MS;
    }
    if (step == (uint16_t)T_B2_STACK)
    {
        /* ★【2026-10-11】**任务点 ROUGH STACK**（原 T_B2_PLACE0/1/2 + T_B2_STACK0/1/2 共 6 步合并而来）：
           整段（3 件）都跑在这一步里 -> 必须给**整段**超时（沿用"放置类整段"的口径）。
           ⚠ 该作业函数 `STACK_STORE()` 目前仍是**空桩**（立即返回），等实做后这个超时才真正起作用。 */
        return (uint32_t)TASK_STORE_PLACE_TIMEOUT_MS;
    }
    if (step == (uint16_t)T_B1_RGH_PLACE)
    {
        /* ★同理：这一步跑"任务点 ROUGH PLACE"（同一份 SP_ 序列，3 件一次做完），
           整段都在它里面 -> 必须给整段超时。 */
        return (uint32_t)TASK_STORE_PLACE_TIMEOUT_MS;
    }
    return (uint32_t)TASK_STEP_TIMEOUT_MS;
}

/**
 * @brief  刚进入某一步时的"任务点设置"（每步只调用一次）
 * @param  step: 当前步骤号
 * @retval 无
 * @note   ★陀螺仪零漂治理（用户要求）：
 *           - TRAY PICK 对位成功后 -> 静态置 0°（登记给 bsp_align，对位成功那刻执行）
 *           - ROUGH PLACE / ROUGH STACK 对位成功后 -> 静态置 90°（同上）
 *           - STORE BACK **开始执行时** -> 直接静态置 180°
 *         其余步骤取消登记（避免影响别的任务点的对位）。
 *         置角后 yaw 照常积分，后续绝对坐标全部基于新基准。
 */
static void Tsk_OnStepEnter(uint16_t step)
{
    Align_ClearDoneYawRef();
    tskArmPre   = 0U;
    tskZeroPend = 0U;                 /* 新步骤：清掉上一段的挂起置角 */

    switch (step)
    {
        case T_B1_TRAY_PICK:
        case T_B2_TRAY_PICK:
            Align_SetDoneYawRef(0.0f);
            break;

        case T_B1_RGH_PLACE:
        case T_B2_STACK:
            Align_SetDoneYawRef(90.0f);
            break;

        case T_B1_PBACK:
        case T_B2_PBACK:
            /* STORE BACK 开始执行即置 180；车还没停稳被拒时挂起，由 Tsk_ZeroYawTick 重试 */
            if (Chassis_StaticZeroYaw(180.0f) == 0U)
            {
                tskZeroRef  = 180.0f;
                tskZeroPend = 1U;
                tskZeroTick = HAL_GetTick();
            }
            break;

        /* ★【2026-10-21 用户要求】去任务点的**最后一段 GO**：快到点时提前把手臂摆到观察位，
           省掉"到点后再等手臂"的时间（触发距离见 TASK_ARM_PRE_OBSERVE_MM）。 */
        case T_B12_TRAY_GO:
        case T_B12_RGH_GO:
        case T_B12_STO_GO:
        case T_B22_TRAY_GO:
        case T_B22_RGH_GO:
        case T_B2_TOST_GO2:
            tskArmPre = 1U;
            break;

        default:
            break;
    }
}

/**
 * @brief  挂起的"任务点开始即置角"重试（每拍调用，非阻塞）
 * @param  now: 当前时刻 ms
 * @retval 无
 * @note   只为 STORE BACK 的 180° 服务：进点那拍车若还在微动会被 IMU 拒绝，
 *         这里等它停稳再置；超过 TASK_YAW_ZERO_WAIT_MS 放弃并打日志。
 */
static void Tsk_ZeroYawTick(uint32_t now)
{
    if (tskZeroPend == 0U)
    {
        return;
    }

    if (Chassis_StaticZeroYaw(tskZeroRef) != 0U)
    {
        tskZeroPend = 0U;
        return;
    }

    if ((now - tskZeroTick) >= (uint32_t)TASK_YAW_ZERO_WAIT_MS)
    {
        tskZeroPend = 0U;
        Debug_Log("TASK", "static zero %d give up (car still moving)", (int)(tskZeroRef * 10.0f));
    }
}

/**
 * @brief  流程引擎推进（主循环调度，非阻塞）
 * @param  无
 * @retval 无
 * @note   结构（照《赛题开发框架指南》第 8 节）：
 *           ① 不在 RUN 直接返回；
 *           ② AUTO 的启动延迟；
 *           ③ 识别"刚进入本步"（g_stepTick==0）并判单步超时；
 *           ④ switch (g_step) 调本步动作函数，返回 1 就 Tsk_Advance()。
 */
void Task_Update(void)
{
    uint32_t now;

    if (g_state != TASK_STATE_RUN)
    {
        return;
    }

    /* 执行中持续置脏，让页面按 REFRESH_PERIOD_MS 节流刷新（步骤号/状态在变） */
    Display_MissionRefresh();

    now = HAL_GetTick();

    /* ---- AUTO 一键启动后的等待（赛题要求 5s 内出发）---- */
    if (g_startHold != 0U)
    {
        if ((now - g_runTick) < TASK_START_DELAY_MS)
        {
            return;
        }
        g_startHold = 0U;
        g_stepTick  = 0U;       /* 让第一步重新打时间戳 */
    }

    /* ---- 识别"刚进入本步" + 单步总超时（超时先停车再放行）---- */
    if (g_stepTick == 0U)
    {
        g_stepTick = now;
        Tsk_OnStepEnter(g_step);      /* ★任务点进入时：登记/执行陀螺仪静态置角 */
    }
    else if ((now - g_stepTick) >= Tsk_StepTimeoutMs(g_step))
    {
        Debug_Log("TASK", "step %u %s timeout", (unsigned int)g_step, Task_GetStepName(g_step));
        Task_Stop();
        g_failCount++;
        Tsk_Advance();
        return;
    }

    Tsk_ZeroYawTick(now);             /* ★挂起的任务点置角：车停稳就补做（STORE BACK=180） */

    /* =====================================================================
     * ★ 现场路线：**绝对坐标系**（相对出发点；抽签后直接改下面的数字）
     *     Rotate(deg)        转到目标朝向：deg = 相对**出发点朝向**的绝对角，
     *                        俯视逆时针为正（0 = 与出发时车头同向）
     *     GO(x_mm, y_mm)     走到目标点：x = 出发点前方(mm)，y = 出发点右侧(mm)
     *   每个作业点两步：T_xx_ROT(转) -> T_xx_GO(走)；要拐弯就成对插
     *   T_xx_R*(转到拐角方向) / T_xx_T*(走到拐角点)。
     *   ★ 每步都朝同一个世界基准回纠，**朝向/位置误差都不会逐段累积**。
     * ⚠ 空操作怎么写：想让某一步不动，就把它填成"和上一步相同的点/朝向"
     *   （GO 算出的增量为 0 -> 立刻通过）。**绝对写法下 GO(0,0) 是"回出发点"、
     *   不是空操作**，别拿它当占位。
     * 下面 QR 段 = 原相对写法按当前位姿**逐段换算**的结果（行为与之前一致）；
     * 其余段 = 原理论点位表的**理论值**，现场按实测替换。
     * 注：下面 case 的**书写顺序不影响执行**，执行顺序由 bsp_task.h 的 TaskStep_t 决定。
     * ===================================================================== */
    switch (g_step)
    {
        /* ============ S1 扫码（转向 -> 走到扫码起点 -> 拐角转向 -> 走到扫码位置 -> 扫码作业）============ */
        case T_QR_ROT:      if (Rotate(0.0f) != 0U)       { Tsk_Advance(); } break;
        case T_QR_GO:       if (GO(150, -150) != 0U)      { Tsk_Advance(); } break;
        case T_QR_R1:       if (Rotate(-90.0f) != 0U)      { Tsk_Advance(); } break;
        case T_QR_T1:       if (GO(150, -1300) != 0U)     { Tsk_Advance(); } break;
        case T_QR_DO:       if (QR() != 0U)               { Tsk_Advance(); } break;

        /* ============ 第一批：原料区取 3 件 ============ */
        case T_B1_TRAY_GO:  if (GO(150, -1060) != 0U)     { Tsk_Advance(); } break;
				case T_B1_TRAY_ROT: if (Rotate(0.0f) != 0U)       { Tsk_Advance(); } break;
				case T_B11_TRAY_GO:  if (GO(1042, -1080) != 0U)     { Tsk_Advance(); } break;
				case T_B12_TRAY_GO:  if (GO(1042, -109) != 0U)     { Tsk_Advance(); } break;

        /* ★ 到转盘区后**一整段做完**：发 $RCP 起头 -> 摆臂到观察位 ->
           等臂到位启动 $RCP 对位 + $RZ/$CCG 连抓 3 件上车（bsp_tt），详见 PICK_TRAY_ALL()。 */
        /* ★任务点 TRAY PICK（第一批）：按**任务码第 1 组**颜色取料 */
        case T_B1_TRAY_PICK: if (PICK_TRAY_ALL(1U) != 0U)   { Tsk_Advance(); } break;

        /* ============ 第一批：暂存区放 3 件 ============ */
        case T_B1_RGH_ROT:  if (GO(1042, -197) != 0U)      { Tsk_Advance(); } break;
        case T_B1_RGH_GO:   if (Rotate(90.0f)  != 0U)      { Tsk_Advance(); } break;
				case T_B11_RGH_GO:  if (GO(1042, -1820)  != 0U)      { Tsk_Advance(); } break;
				case T_B12_RGH_GO:   if (Rotate(180.0f)  != 0U)      { Tsk_Advance(); } break;
        /* ★ 打靶放置**一整段做完**：bsp_tt 的 SP_ 序列
           （臂进观察位 -> $SHOT 对位(严格档/不限时) -> 车载 1/2/3 取件 -> 按绑定色查靶位 -> 放靶位），
           与任务调试页 TDBG_STORE_PLACE / TDBG_ROUGH_PLACE 共用一份实现，详见 PLACE_STORE_ALL()。 */
        /* ★任务点 STORE PLACE；
           `SP_SetRoughMode(0)` = **不做**"回车载位不抓"收尾（要"只松抓"，好衔接 STORE BACK）。 */
        case T_B1_PLACE:    SP_SetBatch(1U); SP_SetRoughMode(0U);
                            if (PLACE_STORE_ALL() != 0U)  { Tsk_Advance(); } break;

        /* ============ 第一批：打靶取回（STORE BACK，按放置逆序放回车载） ============ */
        case T_B1_PBACK:
                            SB_SetBatch(1U);
                            if (PLACE_BACK_ALL() != 0U)   { Tsk_Advance(); } break;

        /* ============ 第一批：暂存区放 3 件 ============ */
        
        case T_B1_STO_GO:   if (GO(1887, -1820) != 0U)     { Tsk_Advance(); } break;
				case T_B12_STO_ROT:  if (Rotate(90.0f) != 0U)       { Tsk_Advance(); } break;
				case T_B12_STO_GO:   if (GO(1887, -970) != 0U)     { Tsk_Advance(); } break;

        /* ★任务点 ROUGH PLACE：**整段（3 件）一次做完** —— 与 STORE PLACE 共用同一份 SP_ 实现，
           所以只占**一步**（原 T_B1_STORE0/1/2 三步已合并为这一步）。
           `SP_SetRoughMode(1)` = 放完 3 件后手臂**回车载 3 号位、不抓**
           （顺序：电机6 抬高 -> 舵机1 收回 -> 电机5 转向 -> 舵机2 张开）。 */
        case T_B1_RGH_PLACE: SP_SetBatch(1U); SP_SetRoughMode(1U);
                             if (PLACE_ROUGH(0) != 0U)  { Tsk_Advance(); } break;

        /* ============ 第二批：先在粗加工区按第 4 组放置 -> 取回 -> 去暂存区同色码垛 ============ */
        case T_B2_TRAY_ROT: if (GO(1900, -109) != 0U)    { Tsk_Advance(); } break;
        case T_B2_TRAY_GO:  if (Rotate(0.0f) != 0U)    { Tsk_Advance(); } break;
				case T_B22_TRAY_GO:  if (GO(1024, -109) != 0U)  { Tsk_Advance(); } break;
        /* ★任务点 TRAY PICK（第二批）：**整段（3 件）一次做完** —— 与第一批共用同一份实现
           `PICK_TRAY_ALL()`，但★**批号传 2**（= 按任务码**第 3 组**颜色取料，2026-10-11 用户要求）。 */
        case T_B2_TRAY_PICK: if (PICK_TRAY_ALL(2U) != 0U) { Tsk_Advance(); } break;

        /* ★【2026-10-23 用户要求】第二批不再"直接去码垛"，而是与第一批同构：
             按任务码第 4 组放到粗加工区 -> 取回车上 -> 搬到暂存区 -> 叠到**第一批同色物料**之上。
           行驶路线沿用第一批的那组坐标（1042,-197 / 1042,-1820 / 1887,-1820 / 1887,-970）。 */
        case T_B2_RGH_ROT:  if (GO(1042, -197) != 0U)       { Tsk_Advance(); } break;
        case T_B2_RGH_GO:   if (Rotate(90.0f) != 0U)        { Tsk_Advance(); } break;
				case T_B22_RGH_GO:  if (GO(1042, -1820) != 0U)      { Tsk_Advance(); } break;
        case T_B2_RGH_TURN: if (Rotate(180.0f) != 0U)       { Tsk_Advance(); } break;

        /* ★第二批"按任务码第 4 组放置"（SP_ batch=2；rough=0 -> 只松抓，好接下面的取回） */
        case T_B2_PLACE:    SP_SetBatch(2U); SP_SetRoughMode(0U);
                            if (PLACE_STORE_ALL() != 0U)    { Tsk_Advance(); } break;

        /* ★第二批"取回"（SB_ batch=2；按放置的逆序 车载3->2->1 搬回车上） */
        case T_B2_PBACK:    SB_SetBatch(2U);
                            if (PLACE_BACK_ALL() != 0U)     { Tsk_Advance(); } break;

        /* ★搬到暂存区（码垛区）：与第一批 PBACK 之后那条路线完全一致 */
        case T_B2_TOST_GO:  if (GO(1887, -1820) != 0U)      { Tsk_Advance(); } break;
        case T_B2_TOST_ROT: if (Rotate(90.0f) != 0U)        { Tsk_Advance(); } break;
        case T_B2_TOST_GO2: if (GO(1887, -970) != 0U)       { Tsk_Advance(); } break;

        /* ★码垛：每件按"第二批颜色"找**第一批同色物料**所在的靶位，叠在它上面
           （原来这里是按第 4 组 place2 直接叠，2026-10-23 改） */
        case T_B2_STACK:
                            if (STACK_STORE(0) != 0U)     { Tsk_Advance(); } break;

        case T_B2_PBACK0:   if (GO(1800, -950) != 0U)       { Tsk_Advance(); } break;

        case T_B2_STO_ROT:  if (Rotate(0.0f) != 0U)       { Tsk_Advance(); } break;
        case T_B2_STO_GO:   if (GO(130, -950) != 0U)     { Tsk_Advance(); } break;

        /* ============ 收尾 ============ */
        case T_HOME_ROT:    if (Rotate(0.0f) != 0U)       { Tsk_Advance(); } break;
        case T_HOME_GO:     if (GO(-16, 0) != 0U)           { Tsk_Advance(); } break;
        case T_STAT:        if (STAT() != 0U)             { Tsk_Advance(); } break;

        /* T_END 是安全兜底：正常情况下 Tsk_Advance 已直接置 ALLDONE，走不到这里 */
        case T_END:
            Task_Stop();
            g_state = TASK_STATE_ALLDONE;
            Debug_Log("TASK", "all done, fail=%u", (unsigned int)g_failCount);
            Display_MissionRefresh();
            break;

        default:
            /* ★ 走到这里说明"**三处不同步**"：枚举/名字表里有这一步，但 switch 里没写它的 case。
               后果是这一步什么都不做、要一直等到单步超时才放行（现象：车不动、步骤号不跳）。
               打一条明确日志，避免这种"静默卡住"被误判成"动作没实现"。 */
            Debug_Log("TASK", "step %u %s HAS NO CASE! (sync TaskStep_t/tskStepName/switch)",
                      (unsigned int)g_step, Task_GetStepName(g_step));
            break;
    }
}

/* ===================== 按键处理 ===================== */

/**
 * @brief  赛题页短按按键处理
 * @param  key: 按键 ID（KEY1~KEY5）
 * @retval 无
 */
void Task_HandleKey(KeyID_t key)
{
    if (key == KEY_NONE)
    {
        return;
    }

    /* 运行中只允许"急停(KEY5)"和"退出(KEY4)"，其余一律忽略，防状态错乱 */
    if (g_state == TASK_STATE_RUN)
    {
        if (key == KEY5)
        {
            Task_Stop();
            g_state = TASK_STATE_ABORT;
            Debug_Log("TASK", "abort by KEY5 at step %u", (unsigned int)g_step);
            Display_MissionRefresh();
        }
        else if (key == KEY4)
        {
            Task_Exit();
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
        case KEY1:      /* 上一步（只移指针，不执行） */
            if (g_step > 0U)
            {
                g_step--;
                g_stepTick = 0U;
            }
            g_state = TASK_STATE_IDLE;
            Display_MissionRefresh();
            break;

        case KEY2:      /* 下一步（只移指针，不执行） */
            if ((uint16_t)(g_step + 1U) < (uint16_t)T_END)
            {
                g_step++;
                g_stepTick = 0U;
            }
            g_state = TASK_STATE_IDLE;
            Display_MissionRefresh();
            break;

        case KEY3:      /* 切换模式 STEP <-> AUTO */
            g_mode = (g_mode == TASK_MODE_STEP) ? TASK_MODE_AUTO : TASK_MODE_STEP;
            Debug_Log("TASK", "mode %s", (g_mode == TASK_MODE_AUTO) ? "AUTO" : "STEP");
            Display_MissionRefresh();
            break;

        case KEY4:      /* 中止并退出回主菜单 */
            Task_Exit();
            Display_MenuEnter();
            break;

        case KEY5:      /* STEP：执行当前步 / AUTO：一键启动 */
            Tsk_Run();
            break;

        default:
            break;
    }
}

/**
 * @brief  赛题页长按按键处理
 * @param  key: 按键 ID（KEY1~KEY5）
 * @retval 无
 * @note   只有一条：长按 KEY5 = ODOM 清零（原点 = 当前位置，朝向基准 = 当前 yaw）。
 */
void Task_HandleLongKey(KeyID_t key)
{
    if ((key == KEY5) && (g_state != TASK_STATE_RUN))
    {
        Chassis_OdomReset();
        Debug_Log("TASK", "odom zeroed (origin = now)");
        Display_MissionRefresh();
    }
}

/* ===================== 作业点函数 ===================== */
/* 【已实做】`QR()`：边"沿车身前方匀速前进"边扫码，扫到合法码就停车并把任务码发给树莓派
 *           （速度/超时/日志节拍等参数见 bsp_task_cfg.h 的 `[2] QR` 节）。
 * 【仍是空桩】PICK_TRAY / PLACE_ROUGH / PICK_ROUGH / PLACE_STORE / STACK_STORE / STAT：
 *   空桩三原则（框架指南"提醒1"）：
 *     ① 打一条 `[TASK] xxx not implemented` 日志；
 *     ② **返回 1 放行**（1 = 本作业已完成），让整条流程能跑完；
 *     ③ **不发任何电机/机械臂/视觉命令**（保证车不动也能跑流程）。
 *   注释里的 TODO 写清"真实实现该调哪个现有模块"，后续按第 5~8 步逐个填实。 */

/**
 * @brief  二维码作业：边**沿车身前方匀速前进**边扫码，扫到就停车并把任务码发给树莓派
 * @retval 1=已完成（扫到合法码并已下发）/ 超时放行；0=进行中
 * @note   ★ 全非阻塞，主循环每拍推进一次：
 *           ① 首次进入：丢掉上一轮残留结果 -> 使能四轮 -> 四轮按 QR_DRIVE_RPM **正转**；
 *           ② 行进中每拍查 `QR_IsNew()`（**读后自清零**，只对**合法帧**返回 1）：
 *              扫到合法码 -> 四轮发 0 速度**停车** -> `RPi_SendCode()` 把任务码发给树莓派 -> 返回 1；
 *           ③ 行进中按 `QR_ODOM_LOG_MS` 周期输出**绝对坐标与朝向** —— 里程计一直在算，
 *              既能看到"走到哪了"，也能判断车有没有越走越偏；
 *           ④ 超过 `QR_SCAN_TIMEOUT_MS` 仍没扫到 -> 停车 + 记失败 + 放行（绝不卡死）。
 *         ★ 【前进方向】正 rpm 就是"**小车车身的前方**"（车头朝哪就往哪走），
 *           不是绝对坐标系里的某个固定方向；绝对坐标系只负责"一直算位置"。
 *         停车用速度模式发 0（按加减速档平顺停），不用 0xFE 急停，收尾更稳。
 *         非法码由 bsp_qr 内部记 bad 且**不置"新帧"标志**，所以这里天然忽略它、继续走。
 */
uint8_t QR(void)
{
    const QR_TaskCode_t *code;

    /* ---- ① 首次进入：复位结果 + 使能四轮 + 沿车身前方匀速起步 ---- */
    if (qrActive == 0U)
    {
        qrActive  = 1U;
        qrTick    = HAL_GetTick();
        qrLogTick = 0U;

        QR_ResetResult();                       /* 丢掉上一轮残留，避免"刚进就判扫到" */
        Tsk_EnsureWheels();
        Tsk_WheelsVel((int16_t)QR_DRIVE_RPM);   /* 正 rpm = 整车沿车身前方前进 */

        Debug_Log("TASK", "QR: drive fwd(body) %u rpm, scan timeout %u ms",
                  (unsigned int)QR_DRIVE_RPM, (unsigned int)QR_SCAN_TIMEOUT_MS);
        return 0U;
    }

    /* ---- ② 扫到新合法帧 -> 停车 + 发给树莓派 ---- */
    if (QR_IsNew() != 0U)
    {
        code = QR_GetCode();

        Tsk_WheelsVel(0);                       /* 先停车 */

        if ((code != NULL) && (code->valid != 0U))
        {
            Debug_Log("TASK", "QR ok: %s (stopped at x=%d y=%d h=%d)",
                      code->raw, (int)Chassis_GetOdomX(), (int)Chassis_GetOdomY(),
                      (int)Chassis_GetOdomYaw());

            RPi_SendCode(code->raw);            /* 把任务码发给树莓派 */
            /* ★【2026-10-31 新赛规】新一轮开始：抓/放计数清零（屏幕下 1/3 变"抓 0 放 0"） */
            RPi_ReqCnt(0U);
            qrActive = 0U;
            return 1U;                          /* 本作业完成 */
        }

        Debug_Log("TASK", "QR frame invalid, keep driving");
        Tsk_WheelsVel((int16_t)QR_DRIVE_RPM);   /* 非法帧：继续走继续扫 */
        return 0U;
    }

    /* ---- ④ 超时兜底：走太久还没扫到 -> 停车放行 ---- */
    if ((HAL_GetTick() - qrTick) >= QR_SCAN_TIMEOUT_MS)
    {
        Tsk_WheelsVel(0);
        g_failCount++;
        qrActive = 0U;
        Debug_Log("TASK", "QR scan timeout -> stop & give up");
        return 1U;
    }

    /* ---- ③ 绝对坐标系一直在算：周期输出当前位置与朝向 ---- */
#if (QR_ODOM_LOG_MS > 0U)
    if ((HAL_GetTick() - qrLogTick) >= (uint32_t)QR_ODOM_LOG_MS)
    {
        qrLogTick = HAL_GetTick();
        Debug_Log("TASK", "QR driving: x=%d y=%d h=%d",
                  (int)Chassis_GetOdomX(), (int)Chassis_GetOdomY(),
                  (int)Chassis_GetOdomYaw());
    }
#endif

    return 0U;
}

/**
 * @brief  原料区取第 i 件（空桩）
 * @param  i: 件号（0 起）
 * @retval 1=放行
 */
uint8_t PICK_TRAY(uint8_t i)
{
    /* TODO(第5步)：视觉侧 Align(ALIGN_TURNTABLE)（bsp_align）；
     *             机械臂侧调 bsp_tt（转盘取料序列：丢弃第一个颜色 -> 等颜色变化
     *             -> 抓转盘 1 号位 -> 放上车确认）；参数见 bsp_task_cfg.h [3] TRAY 节。
     *             —— 两批 TRAY PICK 现在都改用 PICK_TRAY_ALL()（一整段做完），
     *                本函数**已无调用者**（保留仅为兼容旧签名）。 */
    Debug_Log("TASK", "PICK_TRAY not implemented (i=%u)", (unsigned int)i);
    return 1U;
}

/**
 * @brief  转盘取料（一批 3 件**一整段做完**）：直接驱动 `bsp_tt` 的 TRAY PICK 序列
 * @param  batch: 批号 **1 / 2**（★2026-10-11 新增）—— 决定用任务码的哪一组颜色：
 *                **第 1 批 -> 第 1 组（color1）；第 2 批 -> 第 3 组（color2）**
 * @retval 1=整段完成（或失败放行）；0=进行中
 * @note   ★【2026-10-10 统一实现】本作业点不再自己写流程，改为调用 `bsp_tt`
 *         （与任务调试页 `TDBG_TRAY_PICK` 共用**同一份代码**，不会再出现两套流程不一致）。
 *         `bsp_tt` 内部按 v2.0 §6.1 做：
 *           **先进观察位** -> `$RCP` 对位 -> （周期）`$RZ` 找色位 -> 准备位 Q ->
 *           （周期）`$CCG` 轮询 -> 抓住 + 颜色硬校验 -> 放车载 1/2/3 -> 连抓 3 件。
 *         ① 首次进入只做"清车载位表 + 记住批号 + 打日志"；
 *         ② ★按 `batch` 取**任务码对应那一组颜色**后用 `TT_StartWithColors()` 启动
 *            （`TT_Start()` 写死了第 1 组，**第二批不能用它**）；
 *            被拒（颜色非法 / 对位或机械臂正忙 / 任务码未扫到）时下一拍重试，
 *            由**单步超时** `TASK_TRAY_PICK_TIMEOUT_MS`（比通用单步超时长得多）兜底；
 *         ③ 序列结束后（**成功/失败都做**）**不再发送任何帧**（v2.0 取消了推送式 $CMD，
 *            数据帧按 500ms 超时自动无效，打靶阶段由打靶作业点自行发 `$SHOT`）。
 *         推进靠主循环的 `ArmAct_Update()` / `TT_Update()`（main.c 已调度，漏调度会卡住）。
 */
uint8_t PICK_TRAY_ALL(uint8_t batch)
{
    /* ---- ① 首次进入：清车载位表 + 记住批号（序列本身在 ② 启动）---- */
    if (trayPickActive == 0U)
    {
        trayPickActive  = 1U;
        trayPickStarted = 0U;
        trayPickBatch   = (batch == 2U) ? 2U : 1U;   /* ★只允许 1/2 */

        Tsk_ClearCarSlots();                    /* 车载位↔颜色表清零（TT_ 启动时也会清） */
        Debug_Log("TASK", "tray pick: start (bsp_tt), batch=%u items=%u",
                  (unsigned int)trayPickBatch, (unsigned int)TT_ITEM_COUNT);
        return 0U;
    }

    /* ---- ② 启动序列（被拒 -> 下一拍重试，单步超时兜底）----
       ★【2026-10-11 用户要求】**按本批的任务码颜色组取料**：
         第 1 批 -> 任务码**第 1 组**（color1）；第 2 批 -> 任务码**第 3 组**（color2）。
       所以这里**不能用 `TT_Start()`**（它写死第 1 组），改用 `TT_StartWithColors()`。 */
    if (trayPickStarted == 0U)
    {
        uint8_t colors[3];
        uint8_t i;

        if (Task_GetCode()->valid == 0U)
        {
            return 0U;                          /* 任务码未扫到 -> 下一拍重试 */
        }
        for (i = 0U; i < 3U; i++)
        {
            colors[i] = Task_ColorOfBatch(trayPickBatch, i);
        }
        if (TT_StartWithColors(colors) == 0U)
        {
            return 0U;                          /* 颜色非法 / 对位或机械臂正忙 -> 下一拍重试 */
        }

        trayPickStarted = 1U;
        return 0U;
    }

    /* ---- ③ 等序列跑完 ----
       ★【2026-10-15 用户要求】"任务点已完成但机械臂还有多余动作时，先驱动底盘、
         手臂同时回到该到的位置"：最后一件已到手且 **S1 已回 0** 时 `TT_TailOnly()=1`，
         这里就**提前放行** -> 下一段的底盘 GO 与"手臂放回车载位"并行跑（省时间）。
         判据里带上 S1=0，正是用户强调的"**S1 没回 0 不能动车，否则电机5 会晃**"。
         `TT_Update()` 照常把收尾动作跑完，不会被打断。 */
    if ((TT_IsDone() != 0U) || (TT_TailOnly() != 0U) || (TT_IsFailed() != 0U))
    {
        uint8_t ok = ((TT_IsDone() != 0U) || (TT_TailOnly() != 0U)) ? 1U : 0U;
        uint8_t tail = (TT_IsDone() == 0U) ? 1U : 0U;

        trayPickActive  = 0U;
        trayPickStarted = 0U;

        /* 【v2.0 变更】原 v1.0 在此补发 `$CMD,COLOR_STOP` + `$CMD,TARGET_START`
           （停颜色流 + 开靶位识别）。v2.0 取消了推送式 $CMD：数据帧按 500ms 超时自动判无效，
           打靶阶段由打靶作业点自行发 `$SHOT`（点位表 MS_REQ_SHOT / 后续作业点实现）。
           故本步结束后**不再发送任何帧**。 */
        if (ok != 0U)
        {
            Debug_Log("TASK", "tray pick %s: %s",
                      (tail != 0U) ? "release (arm continues)" : "done", TT_GetMapText());
            return 1U;
        }

        g_failCount++;
        Debug_Log("TASK", "tray pick FAIL (see TT logs)");
        return 1U;                              /* 失败也放行（框架指南 C3/C4） */
    }

    return 0U;
}

/**
 * @brief  打靶放置（一批 3 件**一整段做完**）：直接驱动 `bsp_tt` 的 SP_ 打靶放置序列
 * @param  无
 * @retval 1=整段完成（或失败放行）；0=进行中
 * @note   ★【2026-10-10 统一实现】本作业点不再自己写流程，改为调用 `bsp_tt` 的 `SP_` 序列
 *         （与任务调试页 `TDBG_STORE_PLACE` / `TDBG_ROUGH_PLACE` 共用**同一份代码**）。
 *         `SP_` 内部：**臂进观察位** -> `$SHOT` -> 等有效 -> 精确对位（严格档 1px、不限时）
 *         -> **车载位 1→2→3**：读绑定色 -> 任务码第 1 组查靶位 -> `ARM_OP_CAR_PICK` 取件
 *         -> `ARM_OP_TGT_PLACE` 放靶位。
 *         ⚠ 前提：**先跑过 PICK_TRAY_ALL()**（颜色↔车载位号绑定），否则 `SP_` 会 `No binding` 失败。
 *         单步超时用 `TASK_STORE_PLACE_TIMEOUT_MS`（内部串了"对位 + 搬 3 件"，通用值不够）。
 *         推进靠主循环的 `ArmAct_Update()` / `SP_Update()`（main.c 已调度，漏调度会卡住）。
 */
uint8_t PLACE_STORE_ALL(void)
{
    /* ---- ① 首次进入：前置条件做成**与调试页 Tdbg_Start 完全一致** ----
       调试页按 KEY5 时会先：清残留对位 + **使能四轮** + 中止残留序列；
       赛题步这边必须照做，否则会出现"命令发了轮子不动 / $SHOT 对位动不了 /
       残留序列打架"——现象就和调试页的 STORE PLACE 不一样。 */
    if (placeActive == 0U)
    {
        Align_Abort();              /* 清掉可能残留的对位 */
        Tsk_EnsureWheels();         /* ★使能四轮（幂等）：$SHOT 对位要用速度模式动轮子 */
        TT_Abort();                 /* 幂等：没有在跑就什么都不做 */
        SP_Abort();

        placeActive  = 1U;
        placeStarted = 0U;

        Debug_Log("TASK", "place: start (bsp_tt SP_), items=%u, wheels on",
                  (unsigned int)SP_ITEM_COUNT);
        return 0U;
    }

    /* ---- ② 启动序列（被拒：对位/机械臂正忙 -> 下一拍重试，单步超时兜底）---- */
    if (placeStarted == 0U)
    {
        if (SP_Start() == 0U)
        {
            return 0U;
        }

        placeStarted = 1U;
        return 0U;
    }

    /* ---- ③ 等序列跑完 ----
       ★【2026-10-15 用户要求】ROUGH PLACE（`SP_SetRoughMode(1)`）放完 3 件后还有一段
         "回车载位不抓"的收尾（M6 抬高 -> S1 收回 -> M5 转向 -> S2 张开）——这就是
         "任务点已完成但机械臂还有多余动作"：`SP_TailOnly()=1`（3 件放完 **且 S1 已回 0**）
         时提前放行，让**下一段的底盘 GO 与这段收尾并行**。
         STORE PLACE（rough=0）没有这段收尾，`SP_TailOnly()` 恒 0 -> 行为与原来完全一致。 */
    if ((SP_IsDone() != 0U) || (SP_TailOnly() != 0U) || (SP_IsFailed() != 0U))
    {
        uint8_t ok = ((SP_IsDone() != 0U) || (SP_TailOnly() != 0U)) ? 1U : 0U;
        uint8_t tail = (SP_IsDone() == 0U) ? 1U : 0U;

        placeActive  = 0U;
        placeStarted = 0U;

        if (ok != 0U)
        {
            Debug_Log("TASK", "place %s: 3/3, car map %s",
                      (tail != 0U) ? "release (arm continues)" : "done", TT_GetMapText());
            return 1U;
        }

        g_failCount++;
        Debug_Log("TASK", "place FAIL (see SP logs)");
        return 1U;                              /* 失败也放行（框架指南 C3/C4） */
    }

    return 0U;
}

/**
 * @brief  STORE BACK 整段（**按放置的逆序，把 3 件从靶位搬回车载 3 个号位**）
 * @param  无
 * @retval 1=本步已完成（可 Tsk_Advance）；0=进行中
 * @note   与任务调试页 `TDBG_STORE_BACK` **共用同一份代码**（`bsp_tt` 的 `SB_` 序列）。
 *         `SB_` 内部按 **车载位 3→2→1**（STORE PLACE 是 1→2→3）逐件搬：
 *           读该位绑定色 -> 任务码**第 1 组**反查靶位 -> `ArmAct_SbPick` 从靶位取件
 *           -> `ArmAct_SbPlace` 放回车载该号位。
 *         **第一件**（= 最后放上去的那件）按用户要求"**不上抬、不转向**，等 1s 直接抓"
 *         （`SB_Start(chained=1)`）。
 *         ⚠ 前提：**先跑过 `PICK_TRAY_ALL()` + `PLACE_STORE_ALL()`**（颜色绑定 + 料已在靶位上），
 *           否则会 `no binding` 失败。
 *         单步超时用 `TASK_STORE_BACK_TIMEOUT_MS`（内部串了 3 × (取件+放料)）。
 *         推进靠主循环的 `ArmAct_Update()` / `SB_Update()`（main.c 已调度）。
 */
uint8_t PLACE_BACK_ALL(void)
{
    /* ---- ① 首次进入：清残留 + 使能四轮（与调试页 Tdbg_Start 保持一致的前置状态）---- */
    if (backActive == 0U)
    {
        Align_Abort();              /* 清掉可能残留的对位 */
        Tsk_EnsureWheels();         /* 幂等：本段本身不动轮子，但保持与调试页一致 */
        TT_Abort();                 /* 幂等：没有在跑就什么都不做 */
        SP_Abort();
        SB_Abort();

        backActive  = 1U;
        backStarted = 0U;

        Debug_Log("TASK", "back: start (bsp_tt SB_), items=%u", (unsigned int)SB_ITEM_COUNT);
        return 0U;
    }

    /* ---- ② 启动序列（被拒：机械臂正忙 -> 下一拍重试，单步超时兜底）---- */
    if (backStarted == 0U)
    {
        /* ★chained=1：本段紧跟 STORE PLACE，臂正停在"最后一次放置的那个靶位"姿态上，
           所以第一件走"不上抬、不转向"的短流程 + 先等 1s（用户要求）。 */
        if (SB_Start(1U) == 0U)
        {
            return 0U;
        }

        backStarted = 1U;
        return 0U;
    }

    /* ---- ③ 等序列跑完 ----
       ★【2026-10-15 用户要求·就是用户举的例子】"抓到第三个物料时，可在**舵机1 回退到 0 时**
         先让底盘动起来"：`SB_TailOnly()=1`（最后一件已到手 **且 S1 已回 0**）时这里提前放行，
        -> 下一段 `T_B1_STO_GO` 的底盘 GO 与"手臂放回车载 3 号位"并行跑。
       `SB_Update()` 照常把收尾动作跑完，不会被打断。 */
    if ((SB_IsDone() != 0U) || (SB_TailOnly() != 0U) || (SB_IsFailed() != 0U))
    {
        uint8_t ok = ((SB_IsDone() != 0U) || (SB_TailOnly() != 0U)) ? 1U : 0U;
        uint8_t tail = (SB_IsDone() == 0U) ? 1U : 0U;

        backActive  = 0U;
        backStarted = 0U;

        if (ok != 0U)
        {
            Debug_Log("TASK", "back %s: 3/3 back to car, map %s",
                      (tail != 0U) ? "release (arm continues)" : "done", TT_GetMapText());
            return 1U;
        }

        g_failCount++;
        Debug_Log("TASK", "back FAIL (see SB logs)");
        return 1U;                              /* 失败也放行（框架指南 C3/C4） */
    }

    return 0U;
}

/**
 * @brief  粗加工区放置（★【2026-10-10 晚 用户要求】功能与 STORE PLACE **完全一样**）
 * @param  i: 件号（0 起；**现在只有 `T_B1_RGH_PLACE` 一步在调它（i=0）**）
 * @retval 1=放行
 * @note   **共用同一份实现**：直接驱动 bsp_tt 的 `SP_` 打靶放置序列（与任务调试页的
 *         `TDBG_ROUGH_PLACE`、以及赛题 `T_B1_PLACE` 走的是**同一段代码**）：
 *         观察位 -> `$SHOT` + 车身矫正/对位 -> 按任务码取件 -> 放到靶位，共 3 件。
 *
 *         为什么这里要"借用 i"（历史原因，现已只用 i=0）：
 *           `PLACE_STORE_ALL()` 是"**启动一次 -> 等整段跑完**"的语义（内部有 started 标志）。
 *           早年 `T_B2_PLACE0/1/2` 三步都调本函数：**i=0 启动并等整段跑完**、i=1 沿用结果放行、
 *           i=2 复位标志。**2026-10-11 起那三步已合并**（第二批改为单步 `T_B2_STACK`），
 *           所以 i=1/2 两个分支现在只是保留的兼容路径、不再有人调用。 */
uint8_t PLACE_ROUGH(uint8_t i)
{
    static uint8_t roughDone = 0U;   /* i=0 那一整段的结果（含失败：失败也放行，见框架 C3/C4） */

    if (i == 0U)
    {
        roughDone = PLACE_STORE_ALL();      /* ★同一份实现（bsp_tt 的 SP_ 序列） */
        return roughDone;
    }

    if (i == 1U)
    {
        return roughDone;                   /* 整段已在 i=0 跑完，这里只是放行 */
    }

    roughDone = 0U;                         /* i=2：本轮结束，复位给下一次 */
    return 1U;
}

/**
 * @brief  粗加工区取回第 i 件（空桩）
 * @param  i: 件号（0 起）
 * @retval 1=放行
 */
uint8_t PICK_ROUGH(uint8_t i)
{
    /* TODO(第6步)：Align(ALIGN_ROUGH) -> 抓取 -> 放上车确认（bsp_arm_action）。 */
    Debug_Log("TASK", "PICK_ROUGH not implemented (i=%u)", (unsigned int)i);
    return 1U;
}

/**
 * @brief  暂存区放第 i 件（空桩）
 * @param  i: 件号（0 起）
 * @retval 1=放行
 */
uint8_t PLACE_STORE(uint8_t i)
{
    /* TODO(第7步)：Align(ALIGN_FINE) -> 放置到任务码指定的靶位（对 3 个靶位之一）。 */
    Debug_Log("TASK", "PLACE_STORE not implemented (i=%u)", (unsigned int)i);
    return 1U;
}

/* ---- 作业点 粗加工区码垛（★ROUGH STACK，**第二批**）：$SHOT 对位 + 按"第 2 批任务码"堆叠 3 件 ----
   ★【2026-10-11 用户要求】三步口径：
     ① 本步**最先做的是"手臂到达观察位"**（臂不摆过去，视觉看不到物料、给不出偏移）；
     ② 视觉用 **`$SHOT`**（与 ROUGH PLACE / STORE PLACE **同一套数据源**，瞄中间圆环圆心），
        对位入口 `Align_StartShot()`（严格档死区 1px、不限时，与 `$SHOT` 同口径）；
     ③ 堆叠顺序**按第二批任务码**：颜色取**第 3 组**（color2）、靶位取**第 4 组**（place2）；
     ④ **件序 = 第 3 组颜色的顺序**（用户要求"与车载 3 个位置绑定的颜色一定按第二轮任务码顺序抓取"）：
        逐件先按该颜色**反查它在车载哪个位**（`Task_FindSlotByColor`）→ `ARM_OP_CAR_PICK` **取件**
        → `ARM_OP_STACK` 堆到第 4 组对应靶位（**与 STORE PLACE 的 `SP_` 同一套路，只把"放靶位"换成"堆叠"**）。
   期望日志：`stack: start` → `arm->observe` → `SHOT ok ..` → `aligned` → `pick car slot3 (tgt1)` → `stack -> tgt1` → `DONE 3/3`。 */
static uint8_t  stackActive    = 0U;   /* 1=本段已进入 */
static uint8_t  stackStage     = 0U;   /* 段内阶段 1~6 */
static uint8_t  stackIdx       = 0U;   /* 本件 0~2 */
static uint8_t  stackSlot      = 0U;   /* 本件从**车载几号位**取件（按第 2 批任务码颜色反查得到） */
static uint8_t  stackTgt       = 0U;   /* 本件堆到**第 4 组**几号靶位 */
static uint8_t  stackColor     = 0U;   /* 本件颜色（第 3 组） */
static uint8_t  stackArmOn     = 0U;   /* 1=当前机械臂动作已启动 */
static uint32_t stackStageTick = 0U;   /* 当前阶段起点（超时/重发计时） */
static uint32_t stackShotTick  = 0U;   /* 上次发 $SHOT 的时刻 */

/**
 * @brief  本段失败收尾：停对位/机械臂 + 复位状态 + **放行**（框架 C3/C4：失败也放行、绝不卡死）
 */
static uint8_t Stack_Fail(const char *why)
{
    Align_Abort();
    ArmAct_Abort();
    stackActive = 0U;
    stackStage  = 0U;
    stackArmOn  = 0U;
    g_failCount++;
    Debug_Log("TASK", "stack FAIL: %s", why);
    return 1U;
}

/**
 * @brief  找"第一批同色物料"在暂存区的靶位号（第二批码垛用）
 * @param  color: 第二批某件的颜色 1~4
 * @retval 1~3 = 该颜色第一批物料所在的靶位号（第 2 组）；0 = 第一批里没有同色物料
 * @note   ★【2026-10-23 用户要求】"第二批在暂存区码垛放置在第一批同色物料之上"
 *         （赛题规则：暂存区码垛位置由**第一批物料颜色**决定，第 4 组只管粗加工区）。
 */
static uint8_t Tsk_Place1OfColor(uint8_t color)
{
    uint8_t k;

    if (color == 0U)
    {
        return 0U;
    }
    for (k = 0U; k < 3U; k++)
    {
        if (Task_ColorOfBatch(1U, k) == color)
        {
            return Task_PlaceOfBatch(1U, k);
        }
    }
    return 0U;
}

/**
 * @brief  粗加工区码垛（★ROUGH STACK 整段）：**观察位 → `$SHOT` → 对位 → 按第 2 批任务码：取车载件 → 堆叠 3 件**
 * @param  i: 件号（**只有 0 会传进来**，见 `T_B2_STACK`；其它值直接放行）
 * @retval 1=整段完成（或失败放行）；0=进行中
 * @note   非阻塞：状态机在 `Task_Update()` 里每拍推进一步。
 *         视觉/对位与任务调试页 `TDBG_ROUGH_STACK` **同一套口径**（都是 `$SHOT` + `Align_StartShot`）。
 *         逐件动作 = **`ARM_OP_CAR_PICK`（从车载该位取件）→ `ARM_OP_STACK`（堆到靶位）**；
 *         ★靶位 = **第一批同色物料所在靶位**（`Tsk_Place1OfColor`，2026-10-23 用户要求）；
 *         件序 = **第 3 组颜色顺序**（先按颜色反查车载位，再取）。
 *         ⚠ `ARM_CAL_LIST` 里 `ARM_OP_STACK` 三行**当前还没标定**（全 0）→ 堆叠会被拒并**跳过该件**（打日志说明），
 *           等现场标定后就自动生效、不用改代码。
 */
uint8_t STACK_STORE(uint8_t i)
{
    if (i != 0U)
    {
        return 1U;                      /* 兼容旧签名：非 0 值直接放行 */
    }

    /* ---- 首次进入：清残留 + 使能四轮（与其它作业点一致的前置状态）---- */
    if (stackActive == 0U)
    {
        Align_Abort();
        Tsk_EnsureWheels();             /* ★$SHOT 对位要用速度模式动轮子 */
        TT_Abort();
        SP_Abort();
        SB_Abort();

        stackActive    = 1U;
        stackStage     = 1U;
        stackIdx       = 0U;
        stackArmOn     = 0U;
        stackStageTick = HAL_GetTick();
        Debug_Log("TASK", "stack: start (observe -> $SHOT -> align -> batch-2 code x3)");
        return 0U;
    }

    switch (stackStage)
    {
        /* ---- ① ★最先：手臂摆到【观察位】---- */
        case 1U:
            /* ★【2026-10-21 用户要求】路上已提前摆到观察位 -> 本步直接跳过 */
            if (ArmAct_IsAtTtReady() != 0U)
            {
                stackArmOn = 0U;
                Debug_Log("TASK", "stack: already at observe -> skip");
            }
            else if (stackArmOn == 0U)
            {
                if (ArmAct_GotoTtReady() != 0U)
                {
                    stackArmOn = 1U;
                    Debug_Log("TASK", "stack: arm->observe");
                }
                break;                  /* 起不来（已有动作在跑）-> 下一拍重试 */
            }
            if (ArmAct_IsBusy() != 0U)
            {
                break;
            }
            stackArmOn = 0U;
            if (ArmAct_IsAborted() != 0U)
            {
                return Stack_Fail("arm->observe aborted");
            }
            RPi_ReqShot();             /* 先发一帧 $SHOT（与 ROUGH PLACE 同一数据源） */
            stackShotTick  = HAL_GetTick();
            stackStage     = 2U;
            stackStageTick = HAL_GetTick();
            break;

        /* ---- ② 等 `$SHOT` 有效（**周期重发**：协议一问一答、无 ACK）---- */
        case 2U:
            if (RPi_GetShot()->valid != 0U)
            {
                Debug_Log("TASK", "stack: SHOT ok dx=%d dy=%d q=%d",
                          (int)RPi_GetShot()->dxPx, (int)RPi_GetShot()->dyPx,
                          (int)RPi_GetShot()->q[0]);
                stackStage     = 3U;
                stackStageTick = HAL_GetTick();
                break;
            }
            if ((HAL_GetTick() - stackShotTick) >= (uint32_t)SP_SHOT_REQ_MS)
            {
                RPi_ReqShot();         /* 与 SP_ 同一重发口径 */
                stackShotTick = HAL_GetTick();
            }
            if ((HAL_GetTick() - stackStageTick) >= (uint32_t)SP_SHOT_TIMEOUT_MS)
            {
                return Stack_Fail("$SHOT timeout");
            }
            break;

        /* ---- ③ 启动 `$SHOT` 对位（严格档 1px、**不限时**）---- */
        case 3U:
            Align_SetBudget(0U);        /* 0 = 不限时（不到 (0,0) 不放过） */
            if (Align_StartShot() != 0U)
            {
                Debug_Log("TASK", "stack: align start (SHOT, no time limit)");
                stackStage     = 4U;
                stackStageTick = HAL_GetTick();
            }
            else
            {
                RPi_ReqShot();         /* 帧失效 -> 补一帧，下一拍再试（不是失败） */
            }
            break;

        /* ---- ④ 等对位结束 ---- */
        case 4U:
            if (Align_IsBusy() != 0U)
            {
                break;
            }
            if (Align_IsDone() == 0U)
            {
                return Stack_Fail("align fail");
            }
            stackIdx       = 0U;
            stackArmOn     = 0U;
            stackStage     = 5U;
            stackStageTick = HAL_GetTick();
            Debug_Log("TASK", "stack: aligned -> pick car + stack x3 (batch-2 code)");
            break;

        /* ---- ⑤ 逐件：按**第二批任务码的颜色顺序**找车载位 -> 取件 ---- */
        case 5U:
        {
            int8_t slot;

            if (stackIdx >= 3U)         /* 3 件全部堆完 */
            {
                stackActive = 0U;
                stackStage  = 0U;
                Debug_Log("TASK", "stack: DONE 3/3");
                return 1U;
            }

            /* ★【2026-10-11 用户要求】"与车载 3 个位置绑定的颜色，**一定按第二轮任务码顺序抓取**"：
                 先取任务码**第 3 组（color2）第 i 个颜色**，再反查"这个颜色绑在车载哪个位"
                 （TRAY PICK 第 2 批时用 `Task_SetCarSlotColor()` 绑的）。 */
            /* ★【2026-10-23 用户要求】码垛位置 = **第一批同色物料**所在靶位（第 2 组），
               不再用第 4 组（第 4 组只用于刚在粗加工区的那次放置）。 */
            stackColor = Task_ColorOfBatch(2U, stackIdx);
            slot       = Task_FindSlotByColor(stackColor);
            stackTgt   = Tsk_Place1OfColor(stackColor);
            if (stackTgt == 0U)
            {
                /* 第一批没有同色物料（理论上不会抽到）：退回第 4 组，保证不跳过该件 */
                stackTgt = Task_PlaceOfBatch(2U, stackIdx);
                Debug_Log("TASK", "stack: color%u has no batch-1 mate -> fallback place2=%u",
                          (unsigned int)stackColor, (unsigned int)stackTgt);
            }

            if ((stackColor == 0U) || (slot < 1) ||
                (stackTgt < (uint8_t)TASK_TGT_MIN) || (stackTgt > (uint8_t)TASK_TGT_MAX))
            {
                Debug_Log("TASK", "stack: %u/3 no binding (color%u slot%d tgt%u) -> skip",
                          (unsigned int)(stackIdx + 1U), (unsigned int)stackColor,
                          (int)slot, (unsigned int)stackTgt);
                stackIdx++;
                break;
            }
            stackSlot = (uint8_t)slot;

            if (stackArmOn == 0U)
            {
                if (ArmAct_Run(ARM_OP_CAR_PICK, stackSlot) != 0U)
                {
                    stackArmOn     = 1U;
                    stackStageTick = HAL_GetTick();
                    Debug_Log("TASK", "stack: %u/3 color%u -> pick car slot%u (tgt%u)",
                              (unsigned int)(stackIdx + 1U), (unsigned int)stackColor,
                              (unsigned int)stackSlot, (unsigned int)stackTgt);
                }
                break;                  /* 起不来（已有动作在跑）-> 下一拍重试（整段超时兜底） */
            }
            if (ArmAct_IsBusy() != 0U)
            {
                break;
            }
            stackArmOn = 0U;
            /* ★★【2026-10-24 用户要求】取件没夹到物料 -> **停下报 FAIL**（不再跳过本件继续跑） */
            if (ArmAct_GripFailed() != 0U)
            {
                return Stack_Fail("grip failed");
            }
            if (ArmAct_IsAborted() != 0U)
            {
                /* ★【2026-10-23 用户要求】"不要 FAIL，没达成就下一步"：
                   取件被中止 -> 告警并**跳过本件**（不再整段失败）。 */
                Debug_Log("TASK", "stack: %u/3 WARN car pick aborted -> skip item",
                          (unsigned int)(stackIdx + 1U));
                stackIdx++;
                stackStage = 5U;
                break;
            }
            stackStage = 6U;            /* 取件完成 -> 去堆叠 */
            break;
        }

        /* ---- ⑥ 堆叠到任务码**第 4 组**靶位（`ARM_OP_STACK`，流程 `flowStack`）---- */
        case 6U:
            if (stackArmOn == 0U)
            {
                if (ArmAct_Run(ARM_OP_STACK, stackTgt) != 0U)
                {
                    stackArmOn     = 1U;
                    stackStageTick = HAL_GetTick();
                    Debug_Log("TASK", "stack: %u/3 stack -> tgt%u",
                              (unsigned int)(stackIdx + 1U), (unsigned int)stackTgt);
                }
                else
                {
                    /* 该行还没标定（`ARM_CAL_LIST` 的 `ARM_OP_STACK` 三行全 0）-> 明确报出来并跳过本件 */
                    Debug_Log("TASK", "stack: %u/3 tgt%u refused (ARM_OP_STACK not calibrated?) -> skip",
                              (unsigned int)(stackIdx + 1U), (unsigned int)stackTgt);
                    stackIdx++;
                    stackStage = 5U;
                }
                break;
            }
            if (ArmAct_IsBusy() != 0U)
            {
                break;
            }
            stackArmOn = 0U;
            if (ArmAct_IsAborted() != 0U)
            {
                /* ★"不要 FAIL"：堆叠被中止 -> 告警后照常进入下一件 */
                Debug_Log("TASK", "stack: %u/3 WARN stack aborted -> continue",
                          (unsigned int)(stackIdx + 1U));
            }
            stackIdx++;                 /* 本件完成 -> 下一件 */
            stackStage = 5U;
            break;

        default:
            break;
    }

    return 0U;
}

/**
 * @brief  统计显示 + 收尾收臂
 * @retval 1=放行（手臂已收回初始状态）
 * @note   ★【2026-10-26 用户要求】"收尾 VR_TASK_STAT 需要把手臂收回初始状态"：
 *         车已回原点，本步先把手臂收回去（`ArmAct_Park()`：张爪 -> 抬安全高 ‖ 收爪
 *         -> 舵盘回 0 -> 电机5 回 0），**等它跑完**才放行。
 *         非阻塞：臂正忙时启动会被拒，下一拍重试（单步超时兜底放行）。
 */
uint8_t STAT(void)
{
    if (statParked == 0U)
    {
        if (ArmAct_Park() == 0U)
        {
            return 0U;              /* 臂还在跑别的动作：下一拍再试 */
        }
        statParked = 1U;
        Debug_Log("TASK", "STAT: park arm to init pose");
        return 0U;
    }
    if (ArmAct_IsBusy() != 0U)
    {
        return 0U;
    }

    /* TODO(第8步)：汇总抓取/放置正确次数 -> 车顶显示装置（字高 ≥12mm）。 */
    Debug_Log("TASK", "STAT done, fail=%u", (unsigned int)g_failCount);
    return 1U;
}

/**
 * @brief  视觉 PID 闭环对位（第1步：只有转盘 1 号位接到现有 bsp_align）
 * @param  target: 用哪路视觉帧，AlignTarget_t
 * @retval 1=已收敛（含失败放行）；0=进行中
 * @note   本函数不在第 1 步的流程里被调用（流程里的作业点都是空桩），
 *         它只是把"对位"这个动作按用户要求接到现有的 bsp_align 上，
 *         供第 4 步作业点填实时直接调用。依赖主循环的 Align_Update()（main.c 已调度）。
 */
uint8_t Align(uint8_t target)
{
    if (target != (uint8_t)ALIGN_TURNTABLE)
    {
        /* 粗加工靶位 / 精加工靶位两路帧：第 4 步实做 */
        Debug_Log("TASK", "Align(%u) not implemented", (unsigned int)target);
        return 1U;
    }

    if (alActive == 0U)
    {
        if (Align_Start() == 0U)
        {
            /* 视觉数据无效/过期：本拍没启动起来，下一拍再试（由调用方超时兜底） */
            return 0U;
        }
        alActive = 1U;
        Debug_Log("TASK", "align turntable started");
        return 0U;
    }

    if (Align_IsDone() != 0U)
    {
        alActive = 0U;
        Debug_Log("TASK", "align turntable done");
        return 1U;
    }

    if (Align_IsFailed() != 0U)
    {
        alActive = 0U;
        g_failCount++;
        Debug_Log("TASK", "align turntable FAIL");
        return 1U;              /* 失败也放行（框架指南 C3/C4） */
    }

    return 0U;
}

/* ===================== 页面显示 ===================== */

/**
 * @brief  取步骤总数
 * @retval 步骤个数（由步骤名表自动得出）
 */
uint16_t Task_GetStepCount(void)
{
    return TSK_STEP_COUNT;
}

/**
 * @brief  取当前步骤下标
 * @retval 当前步骤下标（0 起）
 */
uint16_t Task_GetStepIndex(void)
{
    return g_step;
}

/**
 * @brief  取当前运行模式
 * @retval TASK_MODE_STEP / TASK_MODE_AUTO
 */
TaskMode_t Task_GetMode(void)
{
    return g_mode;
}

/**
 * @brief  取当前总状态
 * @retval TaskState_t
 */
TaskState_t Task_GetState(void)
{
    return g_state;
}

/**
 * @brief  取步骤名（缩写）
 * @param  idx: 步骤下标
 * @retval 静态字符串；越界返回 "?"
 */
const char *Task_GetStepName(uint16_t idx)
{
    if (idx >= TSK_STEP_COUNT)
    {
        return "?";
    }
    return tskStepName[idx];
}

/**
 * @brief  取页面第 1 行：步骤序号/总数 + 当前步骤名
 * @retval 形如 "12/47 B1_RROT"（≤16 字符）
 */
const char *Task_GetTitleText(void)
{
    static char buf[24];

    snprintf(buf, sizeof(buf), "%u/%u %s",
             (unsigned int)(g_step + 1U), (unsigned int)TSK_STEP_COUNT,
             Task_GetStepName(g_step));

    return buf;
}

/**
 * @brief  取页面第 2 行：本步目标（最近一次 GO/Rotate 的目标）
 * @retval 形如 "T1200,300h90"；尚无目标时 "T---"
 */
const char *Task_GetTargetText(void)
{
    static char buf[24];

    if (g_tgtValid == 0U)
    {
        snprintf(buf, sizeof(buf), "T---");
    }
    else
    {
        snprintf(buf, sizeof(buf), "T%d,%dh%d", (int)g_tgtX, (int)g_tgtY, (int)g_tgtYaw);
    }

    return buf;
}

/**
 * @brief  取页面第 3 行：实测 ODOM 坐标与朝向
 * @retval 形如 "A1180,295h89"
 */
const char *Task_GetOdomText(void)
{
    static char buf[24];

    snprintf(buf, sizeof(buf), "A%d,%dh%d",
             (int)Chassis_GetOdomX(), (int)Chassis_GetOdomY(),
             (int)Chassis_GetOdomYaw());

    return buf;
}

/**
 * @brief  取页面第 4 行：状态 + 模式 + 按键提示
 * @retval "IDLE STEP K5=GO" / "RUN AUTO" / "DONE STEP" / "ALLDONE AUTO" / "ABORT"
 */
const char *Task_GetStateText(void)
{
    static char buf[24];
    const char *md = (g_mode == TASK_MODE_AUTO) ? "AUTO" : "STEP";

    switch (g_state)
    {
        case TASK_STATE_RUN:
            snprintf(buf, sizeof(buf), "RUN %s", md);
            break;

        case TASK_STATE_DONE:
            snprintf(buf, sizeof(buf), "DONE %s", md);
            break;

        case TASK_STATE_ALLDONE:
            snprintf(buf, sizeof(buf), "ALLDONE %s", md);
            break;

        case TASK_STATE_ABORT:
            snprintf(buf, sizeof(buf), "ABORT %s", md);
            break;

        default:
            snprintf(buf, sizeof(buf), "IDLE %s K5=GO", md);
            break;
    }

    return buf;
}
