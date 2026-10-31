/**
 ******************************************************************************
 * @file    bsp_vel_route.c
 * @brief   速度模式跑图页实现（VELROUTE）：赛题路段表 + 纯速度模式
 * @author  nx_car
 * @date    2026-10-24
 ******************************************************************************
 * @note   控制律**不在这里**：ROT/GO 都调 bsp_mission 的速度模式内核
 *         （Mission_VelRot / Mission_VelGo —— 与 VELTEST 正方形路线同一套），
 *         本文件只负责：路段表、步序推进、STEP/AUTO、超时、页面文字。
 *
 *         硬约束：禁止 HAL_Delay；日志禁 %f；全部非阻塞。
 ******************************************************************************
 */
#include "bsp_vel_route.h"
#include "bsp_mission.h"      /* Mission_VelGo / Mission_VelRot / Mission_VelStop */
#include "bsp_motor.h"        /* Motor_Enable / Motor_Disable */
#include "bsp_chassis.h"      /* Chassis_OdomReset / GetOdomX/Y/Yaw */
#include "bsp_oled_ui.h"      /* Display_SetPage / Display_MenuEnter / Display_VelRouteRefresh */
#include "bsp_task.h"         /* ★任务点复用：QR / PICK_TRAY_ALL / PLACE_STORE_ALL / PLACE_BACK_ALL / PLACE_ROUGH / STACK_STORE / STAT */
#include "bsp_tt.h"           /* SP_SetBatch / SP_SetRoughMode / SB_SetBatch / TT_Abort / SP_Abort / SB_Abort */
#include "bsp_debug.h"
#include <stdio.h>
#include <math.h>

/* ===================== 参数 ===================== */

#define VR_TIMEOUT_MS     20000U   /* 移动段单步超时（含 4m 段 + 停车 150ms，够用） */
#define VR_TASK_TIMEOUT_MS 60000U  /* 任务点整段超时（取 3 件/放 3 件 + 视觉对位，耗时长） */
#define VR_LINE_LEN       20U      /* 每行缓冲（8x16 一屏 16 字符） */

/* ===================== 路段表 ===================== */
/* ★ 与位置模式赛题页（bsp_task.c 的 Task_Update() switch）**同一套绝对坐标**，
 *   路段顺序 1:1 对应；区别只在移动内核：这里走速度模式（纯时间开环 + 航向闭环）。
 *   ROT  : 原地转到绝对朝向 deg（相对出发点朝向，俯视逆时针为正）
 *   GO   : 走到绝对坐标 (x,y)（mm，x=出发点前方，y=出发点右侧）
 *          deg 字段复用为"先走腿"：0=先前后 1=先左右
 *   TASK : 任务点，原地执行真实作业（task 字段选 VrTask_t）
 * 抽签换场地时，**只改这张表的 x/y/deg 绝对坐标**（与位置模式赛题页改法一致）。 */
typedef enum {
    VR_ROT = 0,
    VR_GO,
    VR_TASK
} VrKind_t;

typedef enum {
    VR_TASK_QR = 0,
    VR_TASK_PICK_B1,
    VR_TASK_PLACE_B1,
    VR_TASK_PBACK_B1,
    VR_TASK_RGHPLACE_B1,
    VR_TASK_PICK_B2,
    VR_TASK_PLACE_B2,
    VR_TASK_PBACK_B2,
    VR_TASK_STACK_B2,
    VR_TASK_STAT
} VrTask_t;

typedef struct {
    uint8_t     kind;    /* VR_ROT / VR_GO / VR_TASK */
    int32_t     x;       /* GO：绝对 X（mm，前正） */
    int32_t     y;       /* GO：绝对 Y（mm，右正） */
    int16_t     deg;     /* ROT：绝对朝向（deg）；GO：不用（保留，老版本当"先走哪条腿"） */
    uint8_t     task;    /* VR_TASK：VrTask_t 作业类型 */
    const char *name;    /* 页面显示用（≤8 字符） */
} VrStep_t;

static const VrStep_t vrSteps[] =
{
    /* S1 扫码 */
    { VR_GO,    0,    0,    0, 0, "QR ROT"  },
    { VR_GO,   1060,  -200,    0, 0, "QR GO"   },
    { VR_TASK,   0,     0,    0, VR_TASK_QR, "QR DO" },

    /* 第一批：原料区取 3 件 */
    { VR_GO,   1920,  -80,    0, 0, "B1 T-GO" },
    { VR_ROT,    0,     0,    90, 0, "B1 T-R0" },
    { VR_GO,  1900, -1042,    0, 0, "B1 T-IN" },
    { VR_TASK,   0,     0,    0, VR_TASK_PICK_B1, "B1 PICK" },

    /* 第一批：暂存区放 3 件（原来这里有一段零距离 GO，已删） */
		 { VR_GO,  1900, -1092,    0, 0, "B1 R-IN" },
    { VR_ROT,    0,      0,  180, 0, "B1 R-90" },
    { VR_GO,   250,  -1122,    0, 0, "B1 RGH"  },
    { VR_ROT,    0,     0,    -90,   0, "B1 R-180" },
    { VR_TASK,   0,     0,    0, VR_TASK_PLACE_B1, "B1 PLACE" },
    { VR_TASK,   0,     0,    0, VR_TASK_PBACK_B1, "B1 PBACK" },

    /* 第一批：暂存区再放（ROUGH PLACE） */
    { VR_GO,   130, -1857,    0, 0, "B1 STO"  },
    { VR_ROT,    0,     0,   180, 0, "B1 S-90" },
    { VR_GO,   1042, -1847,  0, 0, "B1 S-GO" },
    { VR_TASK,   0,     0,    0, VR_TASK_RGHPLACE_B1, "B1 RHGP" },

    /* 第二批：原料区取 3 件 */
    { VR_GO,  1990,  -2007,    0, 0, "B2 T-GO" },
    { VR_ROT,    0,     0,  90, 0, "B2 T-R0" },
    { VR_GO,  1940,  -1012,    0, 0, "B2 T-IN" },
    { VR_TASK,   0,     0,    0, VR_TASK_PICK_B2, "B2 PICK" },

    /* 第二批：粗加工区放置 -> 取回 -> 暂存区码垛 */
    { VR_GO,  1850,  -992,    0, 0, "B2 R-BK" },
    { VR_ROT,    0,      0,  180, 0, "B2 R-90" },
    { VR_GO,   250,  -1022,    0, 0, "B2 RGH"  },
    { VR_ROT,    0,     0,   -90, 0, "B2 R-180" },
    { VR_TASK,   0,     0,    0, VR_TASK_PLACE_B2, "B2 PLACE" },
    { VR_TASK,   0,     0,    0, VR_TASK_PBACK_B2, "B2 PBACK" },
    { VR_GO,   130, -1817,    0, 0, "B2 TOST" },
    { VR_ROT,    0,     0,   180, 0, "B2 T-R90" },
    { VR_GO,  1042, -1817,    0, 0, "B2 TST2" },
    { VR_TASK,   0,     0,    0, VR_TASK_STACK_B2, "B2 STACK" },

    /* 返程（原来这里有一段零距离 GO，已删） */
    { VR_GO,   250,  -1857,    0, 0, "B2 STO"  },
		{ VR_ROT,    0,     0,    -90, 0, "B2 S-R0" },

    /* 收尾 */
    { VR_GO,    250,    -100,    0, 0, "HOME R"  },
    { VR_GO,     0,     -100,    0, 0, "HOME GO" },
    { VR_TASK,   0,     0,    0, VR_TASK_STAT, "STAT" },
};

#define VR_COUNT   ((uint8_t)(sizeof(vrSteps) / sizeof(vrSteps[0])))

/* ===================== 段间衔接规划（AUTO 连跑：不停车 + 进弯降速） ===================== */
/* ★原来每段结束都要 Mnv_Stop 补发 0 速度 150ms，段内两条腿之间还要再来一次，
   连跑起来就是"一顿一顿"。这里按路段表**前瞻一步**给出本段怎么收尾：
     - 下一步 ROT：**边走边转**（本段行进中就把朝向转过去，到点已对准，ROT 只做末端精修）
     - 下一步 TASK：必须停稳（机械臂作业），且进点前降速
     - 下一步 GO：方向基本一致 -> 全速直连；拐角明显 -> 降速直连（中途不停车）
                  原路折返 -> 必须停稳（四轮瞬间反向会打滑）
   百分比是"相对运行期巡航速度"（VRCFG 的 V），改了 V 这里自动跟着变。 */
#define VR_APPROACH_PCT   92      /* 进点（下一步 ROT/TASK）前一段的巡航速度百分比
                                     ★2026-10-28 二轮提速：80 -> 92（段末本来就要停稳，
                                       降速只影响"到点前的最后一段"，全速收尾更快） */
#define VR_CORNER_PCT     88      /* 拐弯段（下一步 GO 且转角明显）的巡航速度百分比
                                     ★2026-10-28 二轮提速：80 -> 88（拐角只是降速直连，不停车） */
#define VR_CORNER_DEG     35.0f   /* 转角小于它 = 直行，全速通过 */
#define VR_REVERSE_DEG    150.0f  /* 转角大于它 = 原路折返，必须停稳 */

/** 上一个 GO 段的落点（沿途 ROT/TASK 不改变位置）；表首 = 里程计原点 */
static void Vr_PrevGoPoint(uint8_t idx, int32_t *x, int32_t *y)
{
    while (idx > 0U)
    {
        idx--;
        if (vrSteps[idx].kind == (uint8_t)VR_GO)
        {
            *x = vrSteps[idx].x;
            *y = vrSteps[idx].y;
            return;
        }
    }
    *x = 0;
    *y = 0;
}

/** 规划当前 GO 段的收尾：
 *  cap = 本段巡航上限 mm/s（0=用巡航速度）；chain = 1 表示段末不停车；
 *  turnDeg/turnOn = 1 表示本段行进中把车头转到 turnDeg（下一步 ROT 的朝向）。 */
static void Vr_PlanApproach(uint8_t idx, int32_t *cap, uint8_t *chain,
                            int32_t *turnDeg, uint8_t *turnOn)
{
    int32_t cruise = Mission_VelGetCruise();
    int32_t px;
    int32_t py;
    float   ax, ay, bx, by, la, lb, cosA, ang;

    *cap     = 0;
    *chain   = 1U;                      /* 默认直连（段末不停车） */
    *turnDeg = 0;
    *turnOn  = 0U;

    if ((uint16_t)(idx + 1U) >= (uint16_t)VR_COUNT)
    {
        *chain = 0U;                    /* 最后一段：照常停稳 */
        return;
    }

    if (vrSteps[idx + 1U].kind == (uint8_t)VR_ROT)
    {
        /* 下一步原地转：本段行进中就转过去（进点前降速），到点朝向已对准。
           ★段末**不停车**（chain=1）：弧线已经把朝向收口，ROT 只是过一下，
           随后直接带速跑下一路段 —— 这段原来是两个 150ms 补发 0 的停顿。 */
        *cap     = (int32_t)((cruise * VR_APPROACH_PCT) / 100);
        *chain   = 1U;
        *turnDeg = (int32_t)vrSteps[idx + 1U].deg;
        *turnOn  = 1U;
        return;
    }

    if (vrSteps[idx + 1U].kind != (uint8_t)VR_GO)
    {
        /* 下一步是 TASK：进点前降速 + 停稳 */
        *cap   = (int32_t)((cruise * VR_APPROACH_PCT) / 100);
        *chain = 0U;
        return;
    }

    /* 下一步也是 GO：比"进本段的方向"和"出本段的方向"的夹角 */
    Vr_PrevGoPoint(idx, &px, &py);
    ax = (float)(vrSteps[idx].x - px);
    ay = (float)(vrSteps[idx].y - py);
    bx = (float)(vrSteps[idx + 1U].x - vrSteps[idx].x);
    by = (float)(vrSteps[idx + 1U].y - vrSteps[idx].y);
    la = sqrtf((ax * ax) + (ay * ay));
    lb = sqrtf((bx * bx) + (by * by));

    if ((la < 1.0f) || (lb < 1.0f))
    {
        return;                         /* 有一边没位移：没有转角可言，直接直连 */
    }

    cosA = ((ax * bx) + (ay * by)) / (la * lb);
    if (cosA >  1.0f) { cosA =  1.0f; }
    if (cosA < -1.0f) { cosA = -1.0f; }
    ang = acosf(cosA) * 57.29578f;      /* rad -> deg */

    if (ang <= VR_CORNER_DEG)
    {
        *cap = 0;                       /* 直行：全速直连 */
    }
    else if (ang < VR_REVERSE_DEG)
    {
        *cap = (int32_t)((cruise * VR_CORNER_PCT) / 100);   /* 拐弯：降速、不停车 */
    }
    else
    {
        *cap   = (int32_t)((cruise * VR_APPROACH_PCT) / 100);
        *chain = 0U;                    /* 折返：停稳再走 */
    }
}

/* ===================== 状态 ===================== */

typedef enum {
    VR_IDLE = 0,     /* 空闲：未启动 / 已停止（可按键执行当前步） */
    VR_RUN,          /* 正在跑当前段 */
    VR_STEPDONE,     /* STEP 模式：当前步跑完，等 KEY5 跑下一步 */
    VR_ALLDONE,      /* 全部路段跑完 */
    VR_ERR           /* 超时 / 异常中止 */
} VrState_t;

static uint8_t   vrActive = 0U;
static VrState_t vrState  = VR_IDLE;
static uint8_t   vrIdx    = 0U;      /* 当前步 0..VR_COUNT-1 */
static uint8_t   vrAuto   = 1U;      /* 1=AUTO 连跑，0=STEP 单步 */
static uint32_t  vrTick   = 0U;      /* 本步开始时刻（超时用） */
static uint32_t  vrTotalMs = 0U;     /* 本轮整圈累计耗时（步耗时日志用） */
static uint16_t  vrTeleMs = 0U;      /* 遥测周期 ms（0=关闭） */
static uint32_t  vrTeleTick = 0U;
static uint8_t   vrTestRot = 0U;     /* 1 = 当前在跑"单次转向整定" */
static int32_t   vrTestDeg = 90;
static char      vrLine[4][VR_LINE_LEN];

static void Vr_Refresh(void)
{
    Display_VelRouteRefresh();
}

static const char *Vr_StateName(void)
{
    switch (vrState)
    {
        case VR_RUN:      return "RUN";
        case VR_STEPDONE: return "STEP-OK";
        case VR_ALLDONE:  return "ALLDONE";
        case VR_ERR:      return "ERR";
        default:          return "IDLE";
    }
}

/** 启动当前步（STEP/AUTO 共用） */
static void Vr_Start(void)
{
    if (vrIdx >= VR_COUNT)
    {
        vrIdx = 0U;
    }

    Mission_VelStop();                  /* 清内核状态，保证从"新的一段"开始 */
    vrState = VR_RUN;
    vrTick  = HAL_GetTick();
    vrTotalMs = 0U;

    Debug_Log("VR", "start %u/%u %s mode=%s",
              (unsigned int)(vrIdx + 1U), (unsigned int)VR_COUNT,
              vrSteps[vrIdx].name, (vrAuto != 0U) ? "AUTO" : "STEP");
    Vr_Refresh();
}

/** 任务点分派：每拍调用，返回 1=作业完成（推进下一步）。作业函数均非阻塞（复用 bsp_task 的真实作业） */
static uint8_t Vr_TaskUpdate(uint8_t task)
{
    switch (task)
    {
        case VR_TASK_QR:          return QR();
        case VR_TASK_PICK_B1:     return PICK_TRAY_ALL(1U);
        case VR_TASK_PLACE_B1:    SP_SetBatch(1U); SP_SetRoughMode(0U); return PLACE_STORE_ALL();
        case VR_TASK_PBACK_B1:    SB_SetBatch(1U);                        return PLACE_BACK_ALL();
        case VR_TASK_RGHPLACE_B1: SP_SetBatch(1U); SP_SetRoughMode(1U); return PLACE_ROUGH(0);
        case VR_TASK_PICK_B2:     return PICK_TRAY_ALL(2U);
        case VR_TASK_PLACE_B2:    SP_SetBatch(2U); SP_SetRoughMode(0U); return PLACE_STORE_ALL();
        case VR_TASK_PBACK_B2:    SB_SetBatch(2U);                        return PLACE_BACK_ALL();
        case VR_TASK_STACK_B2:    return STACK_STORE(0);
        case VR_TASK_STAT:        return STAT();
        default:                  return 1U;
    }
}

/** 一帧推进 */
void VelRoute_Update(void)
{
    uint32_t now;
    uint8_t  done;

    now = HAL_GetTick();

    /* ★串口遥测（VRTELE）：**不依赖本页是否活动** —— 赛题页（位置模式平移 + 速度模式转向）
       用的也是这套内核，抓它的转向曲线就靠这里。
       t=段内ms st=状态 i=段号 d=剩余mm(转向时为角度0.1°)
       e=航向误差0.1° h=航向0.1° v=速度mm/s w=下发角速度0.1dps g=允许速度 b=横偏mm */
    if ((vrTeleMs > 0U) && ((now - vrTeleTick) >= (uint32_t)vrTeleMs))
    {
        int32_t tD;
        int32_t tE;
        int32_t tH;
        int32_t tV;
        int32_t tW;

        vrTeleTick = now;
        Mission_VelGetTele(&tD, &tE, &tH, &tV, &tW);
        Debug_Log("VR", "T t=%u st=%u i=%u d=%d e=%d h=%d v=%d w=%d g=%d b=%d",
                  (unsigned int)(now - vrTick), (unsigned int)vrState,
                  (unsigned int)(vrIdx + 1U), (int)tD, (int)tE, (int)tH,
                  (int)tV, (int)tW, (int)Mission_VelGetDecel(),
                  (int)Mission_VelGetLat());
    }

    if (vrActive == 0U)
    {
        return;
    }

    if (vrState != VR_RUN)
    {
        return;
    }

    /* ★转向整定：只转一次，跑完停车回 IDLE（不走路线、不推进步号） */
    if (vrTestRot != 0U)
    {
        if (Mission_VelRot((float)vrTestDeg) != 0U)
        {
            vrTestRot = 0U;
            Mission_VelStop();
            vrState = VR_IDLE;
            Debug_Log("VR", "rot test done %d", (int)vrTestDeg);
            Vr_Refresh();
        }
        return;
    }

    if (vrIdx >= VR_COUNT)
    {
        vrState = VR_ALLDONE;
        Mission_VelStop();
        Vr_Refresh();
        return;
    }

    now = HAL_GetTick();

    if (vrSteps[vrIdx].kind == (uint8_t)VR_ROT)
    {
        /* ★下一步是 GO -> 到位不停车，直接带速接上；否则（TASK / 表尾）必须停稳 */
        uint8_t chain = 0U;

        if ((vrAuto != 0U) && ((uint16_t)(vrIdx + 1U) < (uint16_t)VR_COUNT) &&
            (vrSteps[vrIdx + 1U].kind == (uint8_t)VR_GO))
        {
            chain = 1U;
        }

        done = Mission_VelRotEx((float)vrSteps[vrIdx].deg, chain);
    }
    else if (vrSteps[vrIdx].kind == (uint8_t)VR_GO)
    {
        int32_t cap;
        int32_t turnDeg = 0;
        uint8_t chain;
        uint8_t turnOn;

        /* ★段间衔接：前瞻一步给出本段是否降速、段末是否停车、行进中转到哪个朝向 */
        Vr_PlanApproach(vrIdx, &cap, &chain, &turnDeg, &turnOn);

        if (vrAuto == 0U)
        {
            chain  = 0U;                /* STEP 单步：每一步都要停住等按键 */
            turnOn = 0U;                /* 单步不做行进中转向，朝向交给 ROT 步 */
        }

        done = Mission_VelGoEx(vrSteps[vrIdx].x, vrSteps[vrIdx].y,
                               cap, chain, turnDeg, turnOn);
    }
    else
    {
        done = Vr_TaskUpdate(vrSteps[vrIdx].task);
    }

    if (done != 0U)
    {
        uint32_t dt = now - vrTick;     /* ★本段耗时（整圈提速定量用） */

        vrTotalMs += dt;
        Debug_Log("VR", "step%u %s ok t=%ums tot=%ums",
                  (unsigned int)(vrIdx + 1U), vrSteps[vrIdx].name,
                  (unsigned int)dt, (unsigned int)vrTotalMs);

        vrIdx++;
        vrTick = now;

        if ((vrAuto == 0U) && (vrIdx < VR_COUNT))
        {
            Mission_VelStop();
            vrState = VR_STEPDONE;
            Vr_Refresh();
            return;
        }

        if (vrIdx >= VR_COUNT)
        {
            Mission_VelStop();
            vrState = VR_ALLDONE;
            Debug_Log("VR", "all done (%u steps) total=%ums",
                      (unsigned int)VR_COUNT, (unsigned int)vrTotalMs);
            Vr_Refresh();
            return;
        }

        Vr_Refresh();                   /* 自动进入下一段（刷新只是显示步号） */
        return;
    }

    /* 超时兜底：移动段里程计/IMU 异常、任务点卡住时绝不一直等 */
    {
        uint32_t tmo = (vrSteps[vrIdx].kind == (uint8_t)VR_TASK)
                           ? (uint32_t)VR_TASK_TIMEOUT_MS
                           : (uint32_t)VR_TIMEOUT_MS;

        if ((now - vrTick) >= tmo)
        {
            Mission_VelStop();
            TT_Abort(); SP_Abort(); SB_Abort();
            vrState = VR_ERR;
            Debug_Log("VR", "step%u %s TIMEOUT", (unsigned int)(vrIdx + 1U), vrSteps[vrIdx].name);
            Vr_Refresh();
        }
    }
}

/* ===================== 生命周期 ===================== */

void VelRoute_Enter(void)
{
    uint8_t addr;

    for (addr = (uint8_t)MOTOR_ADDR_MIN; addr <= (uint8_t)MOTOR_ADDR_MAX; addr++)
    {
        Motor_Enable(addr, 1U);         /* 抱轴：进页不跑，但车不会被推动 */
    }

    TT_Abort(); SP_Abort(); SB_Abort(); /* 清掉上次可能残留的作业序列，防跨页污染 */

    /* ROT/GO 都是绝对坐标：进页把里程计清零（原点=当前位置，航向基准=当前朝向） */
    Chassis_OdomReset();

    vrActive = 1U;
    vrState  = VR_IDLE;
    vrIdx    = 0U;
    vrAuto   = 1U;

    Display_SetPage(PAGE_VELROUTE);
    Vr_Refresh();

    Debug_Log("VR", "enter: %u steps, odom zeroed. K1/K2=step K3=AUTO/STEP K5=run",
              (unsigned int)VR_COUNT);
}

void VelRoute_Exit(void)
{
    Mission_VelStop();
    TT_Abort(); SP_Abort(); SB_Abort(); /* 停车 + 中止残留作业序列 */
    vrState = VR_IDLE;

    {
        uint8_t addr;

        for (addr = (uint8_t)MOTOR_ADDR_MIN; addr <= (uint8_t)MOTOR_ADDR_MAX; addr++)
        {
            Motor_Disable(addr);
        }
    }

    vrActive = 0U;
    Debug_Log("VR", "exit");
    Display_MenuEnter();
}

uint8_t VelRoute_IsActive(void)
{
    return vrActive;
}

/* ===================== 按键 ===================== */

void VelRoute_HandleKey(KeyID_t key)
{
    if (vrActive == 0U)
    {
        return;
    }

    /* 运行中只认"停止(KEY5)"和"退出(KEY4)"，其余忽略，防状态错乱 */
    if (vrState == VR_RUN)
    {
        if (key == KEY5)
        {
            Mission_VelStop();
            TT_Abort(); SP_Abort(); SB_Abort(); /* 停机械臂作业序列 */
            vrState = VR_IDLE;
            Debug_Log("VR", "stop by KEY5 at %u/%u",
                      (unsigned int)(vrIdx + 1U), (unsigned int)VR_COUNT);
            Vr_Refresh();
        }
        else if (key == KEY4)
        {
            VelRoute_Exit();
        }
        return;
    }

    switch (key)
    {
        case KEY1:      /* 上一步（只移指针，不执行） */
            if (vrIdx > 0U)
            {
                vrIdx--;
            }
            vrState = VR_IDLE;
            Vr_Refresh();
            break;

        case KEY2:      /* 下一步（只移指针，不执行） */
            if ((uint16_t)(vrIdx + 1U) < (uint16_t)VR_COUNT)
            {
                vrIdx++;
            }
            vrState = VR_IDLE;
            Vr_Refresh();
            break;

        case KEY3:      /* AUTO <-> STEP */
            vrAuto = (vrAuto != 0U) ? 0U : 1U;
            Debug_Log("VR", "mode %s", (vrAuto != 0U) ? "AUTO" : "STEP");
            Vr_Refresh();
            break;

        case KEY4:
            VelRoute_Exit();
            break;

        case KEY5:
            Vr_Start();
            break;

        default:
            break;
    }
}

void VelRoute_HandleLongKey(KeyID_t key)
{
    if ((vrActive != 0U) && ((key == KEY4) || (key == KEY5)))
    {
        VelRoute_Exit();
    }
}

/* ===================== 页面文字 ===================== */

const char *VelRoute_GetLine(uint8_t idx)
{
    if (idx >= 4U)
    {
        return "";
    }

    switch (idx)
    {
        case 0:
            (void)snprintf(vrLine[0], VR_LINE_LEN, "VELRT %s %s",
                           Vr_StateName(), (vrAuto != 0U) ? "A" : "S");
            break;

        case 1:
            if (vrIdx < VR_COUNT)
            {
                (void)snprintf(vrLine[1], VR_LINE_LEN, "%u/%u %s",
                               (unsigned int)(vrIdx + 1U), (unsigned int)VR_COUNT,
                               vrSteps[vrIdx].name);
            }
            else
            {
                (void)snprintf(vrLine[1], VR_LINE_LEN, "%u/%u END",
                               (unsigned int)(vrIdx + 1U), (unsigned int)VR_COUNT);
            }
            break;

        case 2:
            if (vrIdx < VR_COUNT)
            {
                if (vrSteps[vrIdx].kind == (uint8_t)VR_ROT)
                {
                    (void)snprintf(vrLine[2], VR_LINE_LEN, "TGT ROT %d",
                                   (int)vrSteps[vrIdx].deg);
                }
                else if (vrSteps[vrIdx].kind == (uint8_t)VR_TASK)
                {
                    (void)snprintf(vrLine[2], VR_LINE_LEN, "TASK");
                }
                else
                {
                    (void)snprintf(vrLine[2], VR_LINE_LEN, "TGT %d,%d",
                                   (int)vrSteps[vrIdx].x, (int)vrSteps[vrIdx].y);
                }
            }
            else
            {
                (void)snprintf(vrLine[2], VR_LINE_LEN, "TGT ---");
            }
            break;

        default:
            (void)snprintf(vrLine[3], VR_LINE_LEN, "N %d,%d %d",
                           (int)Chassis_GetOdomX(), (int)Chassis_GetOdomY(),
                           (int)Chassis_GetOdomYaw());
            break;
    }

    return vrLine[idx];
}

/* ===================== 串口在线调参 ===================== */

uint8_t VelRoute_CmdStart(uint8_t fromStep)
{
    uint8_t idx;

    if (vrActive == 0U)
    {
        VelRoute_Enter();               /* 会自动使能四轮 + 里程计清零 + 切页 */
    }

    idx = (fromStep == 0U) ? 0U : (uint8_t)(fromStep - 1U);
    if (idx >= VR_COUNT)
    {
        idx = 0U;
    }

    vrIdx  = idx;
    vrAuto = 1U;
    Vr_Start();
    return 1U;
}

void VelRoute_CmdStop(void)
{
    Mission_VelStop();
    vrTestRot = 0U;
    if (vrState == VR_RUN)
    {
        vrState = VR_IDLE;
    }
    Debug_Log("VR", "cmd stop at %u/%u",
              (unsigned int)(vrIdx + 1U), (unsigned int)VR_COUNT);
    Vr_Refresh();
}

void VelRoute_CmdDump(void)
{
    Debug_Log("VR", "cfg idx=%u/%u st=%u auto=%u tele=%u",
              (unsigned int)(vrIdx + 1U), (unsigned int)VR_COUNT,
              (unsigned int)vrState, (unsigned int)vrAuto,
              (unsigned int)vrTeleMs);
    Mission_VelDump();
}

uint8_t VelRoute_CmdRot(int32_t deg)
{
    if (vrActive == 0U)
    {
        VelRoute_Enter();
    }

    Mission_VelStop();
    vrTestDeg = deg;
    vrTestRot = 1U;
    vrState   = VR_RUN;
    vrTick    = HAL_GetTick();

    Debug_Log("VR", "rot test start %d", (int)deg);
    return 1U;
}

void VelRoute_SetTeleMs(uint16_t ms)
{
    vrTeleMs   = ms;
    vrTeleTick = HAL_GetTick();
}

uint16_t VelRoute_GetTeleMs(void)
{
    return vrTeleMs;
}
