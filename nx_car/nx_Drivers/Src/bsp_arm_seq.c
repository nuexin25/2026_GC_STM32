/**
 ******************************************************************************
 * @file    bsp_arm_seq.c
 * @brief   手臂调试页实现（P8 联调）：串行回原点 + 多关节动作序列状态机
 * @author  nx_car
 * @date    2026-09-18
 ******************************************************************************
 * @note   两个状态机都在 ArmSeq_Update() 里推进（主循环调度），**完全非阻塞**：
 *           回原点：INIT_LIFT_HOME -> INIT_TURN_HOME -> INIT_CLAW -> INIT_GRIP -> READY
 *                   （电机6 碰撞回零；电机5 **单向回零后把该点置为 0 点**，
 *                     让动作表里的绝对位置每次都有同一个基准；
 *                     电机5 的"回零零点角度"由参数页标定一次、存在电机 flash 里）
 *           动作  ：按 armSteps_xxx[] 的**书写顺序**（下标 0 起）逐步执行
 *
 *         每步都是"先下发一次、之后只判完成"两拍结构（seqStepSent 标志），
 *         避免同一拍重复下发命令。
 *
 *         完成判据：
 *           电机：Arm_IsReachDone()（0x3A 到位帧）为主；
 *                 若"当前位置已等于目标位置"也直接放行（否则像 HOME 里
 *                 "刚回完零又命令去 0"会因收不到到位帧而误判超时）；
 *                 兜底超时 ARM_MOVE_TIMEOUT_MS -> ABORT + 急停。
 *           舵机：先用 bsp_servo 的**回读位置**判断"是否已到目标"
 *                 （差值 ≤ ARM_SEQ_SERVO_TOL 就提前进入下一步）；
 *                 回读不可用（未接线/超时/数据过期）时退回
 *                 "下发后等 SERVO_MOVE_TIME_MS + ARM_SEQ_SERVO_MARGIN_MS"计时。
 *
 *         硬约束：禁止 HAL_Delay；日志禁 %f（位置/参数用整数打印）。
 ******************************************************************************
 */
#include "bsp_arm_seq.h"
#include "bsp_servo.h"          /* 舵机回读：Servo_GetPos 判断舵机是否到位 */
#include "bsp_oled_ui.h"
#include "bsp_debug.h"
#include <stdio.h>

/* ===================== 动作表（增删动作只改这里） ===================== */
/* 【怎么加动作】三步（详见《调参手册》P8 联调章 3.a）：
 *   1) 下面照抄一组关节数组，改名字和参数 —— **行顺序 = 执行顺序**；
 *   2) 在 ARM_ACTION_TABLE 里加一行 ARM_ROW(编号, "显示名", 数组名)；
 *   3) 重新编译。动作个数自动 +1，调试页 KEY1/KEY2 就能选到。
 * 【怎么删动作】删掉该数组 + ARM_ACTION_TABLE 里对应的 ARM_ROW 行（共两处）。
 * 【注意】ARM_ACTION_TABLE 里写了哪个数组名，就必须有对应的数组定义，
 *         否则编译会报 "identifier xxx is undefined"。
 */

/* ---- 关节类型：决定该步参数怎么下发 ---- */
typedef enum {
    AJ_TURN = 0,   /* 电机5 手臂转向：参数 = 绝对位置（下发脉冲域，可为负） */
    AJ_LIFT,       /* 电机6 升降    ：参数 = 绝对位置（下发脉冲域，可为负） */
    AJ_CLAW,       /* 舵机ID1 爪子伸缩：参数 = 位置 0~1000 */
    AJ_GRIP        /* 舵机ID2 爪子抓松：参数 = 位置 0~1000 */
} ArmJoint_t;

/* ---- 一个关节步骤：类型 + 参数 + 方向（舵机忽略方向） ---- */
typedef struct {
    uint8_t joint;    /* ArmJoint_t：关节类型 */
    int32_t param;    /* 电机 = 绝对位置（下发脉冲域，可为负）；舵机 = 位置 0~1000 */
    uint8_t dir;      /* **电机方向**：ARM_DIR_AUTO(0，默认)按"目标−当前"符号自动；
                         ARM_DIR_A(1) 强制 A 方向；ARM_DIR_B(2) 强制 B 方向；
                         舵机步骤（AJ_CLAW/AJ_GRIP）忽略本字段 */
} ArmJointStep_t;

/* 书写一步的简写宏（**故意不做成一条宏串，便于逐行加注释**）
 *   ARM_J (关节, 参数)         ：方向**自动**（按"目标 − 当前位置"的符号）—— 常用
 *   ARM_JD(关节, 参数, 方向)   ：**显式指定方向**，方向填 ARM_DIR_A / ARM_DIR_B
 *
 * 什么时候需要显式方向：
 *   ① 自动方向算错（位置没读到/不准）时，表现为"该往这边走的却往那边走"；
 *   ② 想"每次都从同一侧接近目标"（消除齿轮回程差、提高重复精度）。 */
#define ARM_J(joint_type, value)             { (uint8_t)(joint_type), (int32_t)(value), (uint8_t)ARM_DIR_AUTO }
#define ARM_JD(joint_type, value, dirmode)   { (uint8_t)(joint_type), (int32_t)(value), (uint8_t)(dirmode) }

/* ---- 每个动作的关节序列（顺序 = 数组书写顺序 = 执行顺序） ---- */
/* 【坐标怎么填】见 bsp_arm.h 的"坐标约定"：
 *   电机5 AJ_TURN：以 0 为中心**可正可负**（正 = 顺时针，负 = 逆时针）；
 *   电机6 AJ_LIFT：以最高点为 0、**往下全为正**（填下降的脉冲数即可，不用负数）；
 *   舵机 AJ_CLAW / AJ_GRIP：位置 0~1000。
 * 【怎么写"指定方向"的一步】用 ARM_JD，第三个参数填 ARM_DIR_A / ARM_DIR_B；
 *   不写方向（用 ARM_J）就是**按坐标增减自动决定方向**（推荐）：
 *       ARM_J (AJ_TURN, 500),              // 自动：走到坐标 500
 *       ARM_J (AJ_TURN, -100),             // 自动：走到坐标 -100（反向走）
 *       ARM_JD(AJ_TURN, 500, ARM_DIR_B),   // 强制走 B 方向（调试用，方向与目标相反会走岔）
 *   舵机步骤（AJ_CLAW / AJ_GRIP）没有方向概念，第三个字段会被忽略。 */

/* 动作 HOME：高台归位 */
static const ArmJointStep_t armSteps_HOME[] = {
	ARM_J(AJ_GRIP, 600),      /* 1：舵机2 松开 */
	ARM_J(AJ_TURN, 642),			/* 2：电机5 转到1号位 */
	ARM_J(AJ_CLAW, 249),      /* 3：舵机1 伸到1号位 */
	ARM_J(AJ_LIFT, -17000),   /* 4：电机6 降到1号位 */
	ARM_J(AJ_GRIP, 500),      /* 5：舵机2 抓紧（机械限位 = 500） */
 	ARM_J(AJ_LIFT, -3000),    /* 6：电机6 抬高 */
	ARM_J(AJ_CLAW, 70),      /* 3：舵机1 伸到1号位 */
	ARM_J(AJ_TURN, 1940),			/* 7：电机5 转到放置2 */
	ARM_J(AJ_CLAW, 67),      /* 8：舵机1 伸出放置2 */
	ARM_J(AJ_LIFT, -28446),   /* 9：电机6 下降 */
	ARM_J(AJ_GRIP, 600),      /* 10：舵机2 松开（>500 才算张开） */
	ARM_J(AJ_LIFT, -3000),    /* 6：电机6 抬高 */
	ARM_J(AJ_TURN, 0),			/* 2：电机5 转到1号位 */
};

/* 动作 PICK：取料示例（参数待实测填写，先用小值试） */
static const ArmJointStep_t armSteps_PICK[] = {
  ARM_J(AJ_GRIP, 600),      /* 1：舵机2 松开（>500 才算张开） */
	ARM_J(AJ_TURN, -551),			/* 2：电机5 转到2号位 */
	ARM_J(AJ_CLAW, 573),      /* 3：舵机1 伸到2号位 */
	ARM_J(AJ_LIFT, -16100),   /* 4：电机6 降到2号位 */
	ARM_J(AJ_GRIP, 500),      /* 5：舵机2 抓紧（机械限位 = 500） */
 	ARM_J(AJ_LIFT, -3000),    /* 6：电机6 抬高 */
	ARM_J(AJ_TURN, 595),			/* 7：电机5 转到放置1 */
	ARM_J(AJ_CLAW, 238),      /* 8：舵机1 伸出放置1 */
	ARM_J(AJ_LIFT, -62646),   /* 9：电机6 下降 */
	ARM_J(AJ_GRIP, 600),      /* 10：舵机2 松开（>500 才算张开） */
	ARM_J(AJ_LIFT, -3000),    /* 6：电机6 抬高 */
	ARM_J(AJ_TURN, 0),			/* 2：电机5 转到1号位 */
};

/* 动作 PLACE：放置示例 */
static const ArmJointStep_t armSteps_PLACE[] = {
  ARM_J(AJ_GRIP, 600),      /* 1：舵机2 松开（>500 才算张开） */
	ARM_J(AJ_TURN, 2405),			/* 2：电机5 转到3号位 */
	ARM_J(AJ_CLAW, 398),      /* 3：舵机1 伸到3位 */
	ARM_J(AJ_LIFT, -16100),   /* 4：电机6 降到3号位 */
	ARM_J(AJ_GRIP, 500),      /* 5：舵机2 抓紧（机械限位 = 500） */
 	ARM_J(AJ_LIFT, -3000),    /* 6：电机6 抬高 */
	ARM_J(AJ_TURN, 1482),			/* 7：电机5 转到放置3 */
	ARM_J(AJ_CLAW, 215),      /* 8：舵机1 伸出放置3 */
	ARM_J(AJ_LIFT, -62646),   /* 9：电机6 下降 */
	ARM_J(AJ_GRIP, 600),      /* 10：舵机2 松开（>500 才算张开） */
	ARM_J(AJ_LIFT, -3000),    /* 6：电机6 抬高 */
	ARM_J(AJ_TURN, 0),			/* 2：电机5 转到1号位 */
};

/* 动作 MYACT：**模板**（先给了一组和 HOME 一样的安全参数，请按你的实际动作改）
   之所以必须有这个数组：ARM_ACTION_TABLE 里已经写了 ARM_ROW(MYACT, ...)，
   没有对应数组会编译报错 "identifier armSteps_MYACT is undefined"。
   不需要这个动作时：把本数组 + ARM_ACTION_TABLE 里那行一起删掉即可。 */
static const ArmJointStep_t armSteps_MYACT[] = {
   ARM_J(AJ_GRIP, 600),      /* 1：舵机2 松开（>500 才算张开） */
	ARM_J(AJ_CLAW, 339),      /* 2：舵机1 伸出放置2 */
	ARM_J(AJ_TURN, 836),			/* 3：电机5 转到放置2 */
	ARM_J(AJ_LIFT, -28883),   /* 4：电机6 下降 */
	ARM_J(AJ_GRIP, 500),      /* 5：舵机2 抓紧（机械限位 = 500） */
	ARM_J(AJ_LIFT, -3000),    /* 6：电机6 抬高 */
	ARM_J(AJ_TURN, -295),			/* 7：电机5 转到1号位 */
	ARM_J(AJ_CLAW, 164),      /* 8：舵机1 伸到1号位 */
	ARM_J(AJ_LIFT, -16100),   /* 9：电机6 降到1号位 */
	ARM_J(AJ_GRIP, 600),      /* 10：舵机2 松开（>500 才算张开） */
	ARM_J(AJ_LIFT, -3000),    /* 11：电机6 抬高 */
	ARM_J(AJ_TURN, 0),			/*12：电机5 转到1号位 */
};
/* 【2026-10-24 已删除】原来的"车载 1/2/3 号位"三组调试动作
   （`Car_Location1/2/3`：M5=-295/-551/836、M6=-16100/-3000、S1=395/573/339）
   是**旧三位置方案**的数据；车载现在靠舵机3 选位、手臂只有一组统一姿态
   （见 `bsp_arm_action.h` 的 `ARM_CAR_M5/M6/S1` + `ARM_CAR_S3_POS1/2/3`），
   留着只会在调试页里把 M5 开回老位置，故整组删除。 */

/* ---- 动作表（**新增动作 = 在这里加一行**） ---- */
#define ARM_ACTION_TABLE \
    ARM_ROW(HOME,  "HOME",  armSteps_HOME)   \
    ARM_ROW(PICK,  "PICK",  armSteps_PICK)   \
    ARM_ROW(PLACE, "PLACE", armSteps_PLACE)  \
    ARM_ROW(MYACT, "MYACT", armSteps_MYACT)
    /* ↑ 新增动作：照抄一行，写 (编号, 显示名, 关节数组名) 即可
       注意最后一行**不要**以反斜杠结尾（反斜杠是"续行"，最后一行后面不能再接内容） */

/* 自动生成动作编号枚举 ARM_ACT_HOME / ARM_ACT_PICK / ... */
#define ARM_ROW(id, name, steps) ARM_ACT_##id,
typedef enum { ARM_ACTION_TABLE ARM_ACT_COUNT } ArmActionId_t;
#undef ARM_ROW

/* 动作表结构（显示名 + 关节序列 + 步数，步数自动算） */
typedef struct {
    const char *name;
    const ArmJointStep_t *steps;
    uint8_t stepCount;
} ArmAction_t;

#define ARM_ROW(id, name, steps) \
    { (name), (steps), (uint8_t)(sizeof(steps) / sizeof((steps)[0])) },
static const ArmAction_t armActions[] = { ARM_ACTION_TABLE };
#undef ARM_ROW

/* 动作个数（自动计算，增删动作不用改这里） */
#define ARM_ACTION_COUNT ((uint8_t)(sizeof(armActions) / sizeof(armActions[0])))

/* ===================== 内部状态 ===================== */

/* 本页状态机状态 */
typedef enum {
    SEQ_IDLE = 0,   /* 空闲（未进页面 / 已退出） */
    SEQ_INIT,       /* 回原点中（子步见 SeqInitStep_t） */
    SEQ_READY,      /* 回原点完成，等待 KEY5 启动动作 */
    SEQ_RUN,        /* 动作执行中 */
    SEQ_DONE,       /* 动作执行完成 */
    SEQ_ABORT       /* 被中止或超时 */
} SeqState_t;

/* 回原点的子步（**严格串行，顺序固定**）
 * 【电机5 的基准怎么来的】分两步，配合起来就"每次都到同一角度"：
 *   ① 参数页（ARMPARA）**标定一次**：长按 KEY3 把当时的角度登记为"回零零点"并写进电机 flash；
 *   ② 本页**每次进页**：让电机5 **单向回零**（`ARM_TURN_HOME_MODE = 1`，同一侧接近零点，
 *      消除齿隙）→ 回零完成后再发 `0A 6D`（`Arm_ZeroCurPos`）把"当前位置"算作 0。
 * 这样位置基准每次都被重新对齐，不依赖"位置能否跨掉电保持"。
 * 电机6（丝杆）没有绝对零点，只能靠每次向上撞停标定，所以也必须自动做。 */
typedef enum {
    SEQ_INIT_LIFT_HOME = 0,   /* ① 电机6 碰撞回零（向上撞停 = 0 点） */
    SEQ_INIT_TURN_HOME,       /* ② 电机5 单向回零 -> 回零后把该点置为 0 点 */
    SEQ_INIT_CLAW,            /* ③ 舵机1 转到 ARM_SEQ_HOME_CLAW_POS */
    SEQ_INIT_GRIP,            /* ④ 舵机2 转到 ARM_SEQ_HOME_GRIP_POS */
    SEQ_INIT_STEP_COUNT       /* 子步总数（显示 "INIT n/4" 用，自动算） */
} SeqInitStep_t;

static uint8_t  seqActive   = 0U;                    /* 1=本页活动 */
static uint8_t  seqState    = SEQ_IDLE;              /* 当前状态 */
static uint8_t  seqInitStep = SEQ_INIT_LIFT_HOME;    /* 回原点当前子步 */
static uint32_t seqStepTick = 0U;                    /* 当前步开始时刻（计时/超时用） */

static uint8_t  seqAction   = 0U;                    /* 当前选中的动作号 */
static uint8_t  seqStep     = 0U;                    /* 动作内当前步号（0 起） */
static uint8_t  seqStepSent = 0U;                    /* 本步命令是否已下发 */

/* ---- 电机步的闭环检测状态（见 bsp_arm_seq.h 的"闭环检测"说明） ---- */
static uint8_t  seqMotorRetry        = 0U;   /* 本步已"补发剩余距离"的次数 */
static uint8_t  seqMotorStallOn      = 0U;   /* 1=本步已建立位置基准，正在计时"位置不变" */
static int32_t  seqMotorLastPos      = 0;    /* 上一次采样的位置（下发脉冲域） */
static uint32_t seqMotorStallTick    = 0U;   /* 位置最后一次发生变化的时刻 */
static uint8_t  seqMotorReachIgnored = 0U;   /* 1=本步已打印过"到位帧被位置校验挡下"，防刷屏 */

/* ===================== 内部工具 ===================== */

/**
 * @brief  取关节名称（日志与页面显示用）
 * @param  joint: ArmJoint_t
 * @retval "TURN" / "LIFT" / "CLAW" / "GRIP"
 */
static const char *ArmSeq_JointName(uint8_t joint)
{
    switch (joint)
    {
        case AJ_TURN: return "TURN";
        case AJ_LIFT: return "LIFT";
        case AJ_CLAW: return "CLAW";
        default:      return "GRIP";
    }
}

/**
 * @brief  把动作表里的舵机参数钳位到合法范围
 * @param  p: 动作表里写的参数（可能是负数或超范围）
 * @retval 钳位后的 0~ARM_SERVO_POS_MAX
 * @note   动作表是手写的，加这层保护避免写错数字把爪子顶到限位。
 */
static uint16_t ArmSeq_ServoParam(int32_t p)
{
    if (p < (int32_t)ARM_SERVO_POS_MIN)
    {
        return (uint16_t)ARM_SERVO_POS_MIN;
    }
    if (p > (int32_t)ARM_SERVO_POS_MAX)
    {
        return (uint16_t)ARM_SERVO_POS_MAX;
    }
    return (uint16_t)p;
}

/**
 * @brief  判断某电机是否在"目标位置 ± tol"范围内
 * @param  addr  : 电机地址 5/6
 * @param  target: 目标绝对位置（下发脉冲域）
 * @param  tol   : 容差（下发脉冲域，恒为正）
 * @retval 1=在容差内；0=不在容差内或位置还没读到
 * @note   容差由调用方给（动作序列里用该轴的 ARM_SEQ_TURN/LIFT_POS_TOL）。
 */
static uint8_t ArmSeq_NearTarget(uint8_t addr, int32_t target, int32_t tol)
{
    int32_t d;

    if (Arm_IsPosValid(addr) == 0U)
    {
        return 0U;   /* 位置还没读到，不能判定 */
    }

    d = Arm_GetPos(addr) - target;
    if (d < 0)
    {
        d = -d;
    }

    return (d <= tol) ? 1U : 0U;
}

/**
 * @brief  中止当前动作：两个电机急停 + 状态置 ABORT
 * @param  why: 中止原因（字符串常量，写日志用）
 * @retval 无
 * @note   舵机没有停止指令，只能靠它自己走完 SERVO_MOVE_TIME_MS。
 */
static void ArmSeq_Abort(const char *why)
{
    Arm_Stop(ARM_ADDR_TURN);
    Arm_Stop(ARM_ADDR_LIFT);

    Arm_SetFastPoll(0U);   /* 中止：关掉位置快查，恢复正常 6 拍轮转 */
    seqState = SEQ_ABORT;

    Debug_Log("SEQ", "abort (%s)", why);
    Display_ArmSeqRefresh();
}

/**
 * @brief  启动（或重新启动）回原点流程
 * @param  无
 * @retval 无
 * @note   只触发第①步，后续步骤由 ArmSeq_InitUpdate() 串行推进。
 */
static void ArmSeq_StartInit(void)
{
    seqInitStep = SEQ_INIT_LIFT_HOME;
    seqStepTick = HAL_GetTick();
    seqState    = SEQ_INIT;

    /* 回原点靠"回零完成帧(0x3B)"判完成，不用位置快查 —— 关掉恢复正常轮转 */
    Arm_SetFastPoll(0U);

    /* 第①步：电机6 碰撞回零（向上撞到最高点，把那里当 0 点） */
    Arm_LiftHome();

    Debug_Log("SEQ", "init step=1 lift home");
    Display_ArmSeqRefresh();
}

/**
 * @brief  回原点状态机推进（每拍调用）
 * @param  无
 * @retval 无
 * @note   严格串行：前一步完成才做下一步；回零步有超时兜底（进 ABORT）。
 *         舵机步没有回读，等待时长到即算完成。
 */
static void ArmSeq_InitUpdate(void)
{
    uint32_t now     = HAL_GetTick();
    uint32_t elapsed = now - seqStepTick;

    switch (seqInitStep)
    {
        case SEQ_INIT_LIFT_HOME:   /* ① 等电机6 回零完成 */
            if (Arm_IsHomeDone(ARM_ADDR_LIFT) != 0U)
            {
                seqInitStep = SEQ_INIT_TURN_HOME;
                seqStepTick = now;
                /* ② 电机5 单向回零（模式由 ARM_TURN_HOME_MODE 决定，
                   里面会先把回零参数写进电机，保证每次都朝同一个零点角走） */
                Arm_TurnHome();
                Debug_Log("SEQ", "init step=2 turn home");
                Display_ArmSeqRefresh();
            }
            else if (elapsed >= (uint32_t)ARM_HOME_TIMEOUT_MS)
            {
                ArmSeq_Abort("lift home timeout");
            }
            else
            {
                /* 继续等回零完成帧 */
            }
            break;

        case SEQ_INIT_TURN_HOME:   /* ② 等电机5 回零完成 -> 把该点置为 0 点 */
            if (Arm_IsHomeDone(ARM_ADDR_TURN) != 0U)
            {
                /* 回零只保证"停在零点角度"，并不保证位置计数是 0 ——
                   这里把"当前位置"算作 0（只发 0A 6D，不写电机 flash），
                   之后动作表里 AJ_TURN 的绝对位置就有同一个基准了 */
                Arm_ZeroCurPos(ARM_ADDR_TURN);
                Arm_ReadPos(ARM_ADDR_TURN);   /* 插一次查询，页面 p= 尽快显示 0 */

                seqInitStep = SEQ_INIT_CLAW;
                seqStepTick = now;
                Arm_ServoMove(SERVO_ID_CLAW, ArmSeq_ServoParam(ARM_SEQ_HOME_CLAW_POS));
                Debug_Log("SEQ", "init step=3 claw");
                Display_ArmSeqRefresh();
            }
            else if (elapsed >= (uint32_t)ARM_HOME_TIMEOUT_MS)
            {
                ArmSeq_Abort("turn home timeout");
            }
            else
            {
                /* 继续等回零完成帧 */
            }
            break;

        case SEQ_INIT_CLAW:        /* ③ 等舵机1 走完（回读不可用时按时间） */
            if (elapsed >= ARM_SEQ_SERVO_WAIT_MS)
            {
                seqInitStep = SEQ_INIT_GRIP;
                seqStepTick = now;
                Arm_ServoMove(SERVO_ID_GRIP, ArmSeq_ServoParam(ARM_SEQ_HOME_GRIP_POS));
                Debug_Log("SEQ", "init step=4 grip");
                Display_ArmSeqRefresh();
            }
            break;

        default:                   /* ④ 等舵机2 走完 -> 回原点完成 */
            if (elapsed >= ARM_SEQ_SERVO_WAIT_MS)
            {
                seqState = SEQ_READY;
                Debug_Log("SEQ", "init done, ready");
                Display_ArmSeqRefresh();
            }
            break;
    }
}

/**
 * @brief  动作执行状态机推进（每拍调用）
 * @param  无
 * @retval 无
 * @note   **执行顺序严格等于动作表数组的书写顺序**（下标 0 -> stepCount-1），
 *         不做任何排序或按关节类型分组。
 */
static void ArmSeq_RunUpdate(void)
{
    const ArmAction_t *act = &armActions[seqAction];
    const ArmJointStep_t *st;
    uint32_t now;
    uint32_t elapsed;
    uint8_t  done = 0U;

    /* 全部步走完 -> 动作完成 */
    if (seqStep >= act->stepCount)
    {
        seqState = SEQ_DONE;
        Debug_Log("SEQ", "done");
        Display_ArmSeqRefresh();
        return;
    }

    st      = &act->steps[seqStep];
    now     = HAL_GetTick();
    elapsed = now - seqStepTick;

    /* ---- 第一拍：下发本步命令 ---- */
    if (seqStepSent == 0U)
    {
        seqStepSent = 1U;
        seqStepTick = now;

        /* 闭环检测状态复位：以"下发前的位置"为基准开始计时"位置有没有变" */
        seqMotorRetry        = 0U;
        seqMotorStallOn      = 1U;
        seqMotorStallTick    = now;
        seqMotorReachIgnored = 0U;
        if (st->joint == AJ_TURN)
        {
            seqMotorLastPos = Arm_GetPos(ARM_ADDR_TURN);
        }
        else if (st->joint == AJ_LIFT)
        {
            seqMotorLastPos = Arm_GetPos(ARM_ADDR_LIFT);
        }
        else
        {
            /* 舵机步不用位置基准 */
        }

        switch (st->joint)
        {
            case AJ_TURN:
                /* 按本步的方向字段下发（ARM_DIR_AUTO=自动 / A / B）；
                   Arm_MoveAbsDir 内部会先清到位标志，防上一段被误判 */
                Arm_MoveAbsDir(ARM_ADDR_TURN, st->param,
                               ARM_TURN_VEL, ARM_TURN_ACC, st->dir);
                break;

            case AJ_LIFT:
                Arm_MoveAbsDir(ARM_ADDR_LIFT, st->param,
                               ARM_LIFT_VEL, ARM_LIFT_ACC, st->dir);
                break;

            case AJ_CLAW:
                Arm_ServoMove(SERVO_ID_CLAW, ArmSeq_ServoParam(st->param));
                break;

            default:   /* AJ_GRIP */
                Arm_ServoMove(SERVO_ID_GRIP, ArmSeq_ServoParam(st->param));
                break;
        }

        /* 日志：电机步骤带上方向字段（0=自动 / 1=A / 2=B），舵机步骤不带。
           电机步还要**打开位置快查**（位置回读 120ms -> 20ms），
           这样闭环判据②（位置进容差）能在电机到位后几十毫秒内就发现，
           不用等下一次轮到它的那个查询槽（见 bsp_arm.h 的 Arm_SetFastPoll）。 */
        if ((st->joint == AJ_TURN) || (st->joint == AJ_LIFT))
        {
            Arm_SetFastPoll((st->joint == AJ_TURN) ? (uint8_t)ARM_ADDR_TURN
                                                   : (uint8_t)ARM_ADDR_LIFT);
            Debug_Log("SEQ", "step%u %s %d dir=%u",
                      (unsigned int)(seqStep + 1U), ArmSeq_JointName(st->joint),
                      (int)st->param, (unsigned int)st->dir);
        }
        else
        {
            Debug_Log("SEQ", "step%u %s %d",
                      (unsigned int)(seqStep + 1U), ArmSeq_JointName(st->joint),
                      (int)st->param);
        }
        Display_ArmSeqRefresh();
        return;
    }

    /* ---- 后续拍：判本步是否完成 ----
     * 两个轴**故意分开判**：
     *   电机5（严格位置闭环 STRICT=1）：
     *       跑完（位置停稳）-> 校验 |当前 − 目标| ≤ 到位容差 -> 不合格就**纠正**再来，
     *       直到合格才允许进下一步（见 bsp_arm_seq.h 的严格闭环说明）。
     *   电机6（容错 STRICT=0）：到位帧 / 位置容差 / 停住补发 / 补发用尽告警继续，
     *       避免被丝杆负载顶住时把整段动作卡死。
     */
    if ((st->joint == AJ_TURN) || (st->joint == AJ_LIFT))
    {
        uint8_t addr   = (st->joint == AJ_TURN) ? (uint8_t)ARM_ADDR_TURN
                                               : (uint8_t)ARM_ADDR_LIFT;
        uint16_t mvel  = (st->joint == AJ_TURN) ? (uint16_t)ARM_TURN_VEL
                                               : (uint16_t)ARM_LIFT_VEL;
        uint8_t  macc  = (st->joint == AJ_TURN) ? (uint8_t)ARM_TURN_ACC
                                               : (uint8_t)ARM_LIFT_ACC;
        /* 本轴是否"严格位置闭环"（见 bsp_arm_seq.h 的 ARM_SEQ_TURN/LIFT_STRICT_POS） */
        uint8_t  strictPos = (st->joint == AJ_TURN) ? (uint8_t)ARM_SEQ_TURN_STRICT_POS
                                                   : (uint8_t)ARM_SEQ_LIFT_STRICT_POS;
        /* 本轴到位容差（下发脉冲域，两轴各自一套）：电机5 = ARM_SEQ_TURN_POS_TOL，
           电机6 = ARM_SEQ_LIFT_POS_TOL。容差越小停得越准 */
        int32_t  posTol = (st->joint == AJ_TURN) ? (int32_t)ARM_SEQ_TURN_POS_TOL
                                                 : (int32_t)ARM_SEQ_LIFT_POS_TOL;

        if (strictPos != 0U)
        {
            /* ================= 电机5：跑完 -> 校验 -> 纠正 ================= */
            if (Arm_IsPosValid(addr) == 0U)
            {
                /* 位置读不到（接线/电机 Response 没配）：只能退回采信到位帧 */
                if (Arm_IsReachDone(addr) != 0U)
                {
                    done = 1U;
                }
            }
            else
            {
                int32_t cur = Arm_GetPos(addr);

                if (cur != seqMotorLastPos)
                {
                    /* 位置还在变 => 电机还在跑：刷新基准、重新开始"停稳"计时 */
                    seqMotorLastPos   = cur;
                    seqMotorStallTick = now;
                }
                else if ((now - seqMotorStallTick) < (uint32_t)ARM_SEQ_MOVE_STALL_MS)
                {
                    /* 位置不变但还没到"停稳"时长，继续观察 */
                }
                else
                {
                    /* 已经停稳 -> **校验到位误差** */
                    int32_t err = cur - st->param;

                    if (err < 0)
                    {
                        err = -err;
                    }

                    if (err <= posTol)
                    {
                        done = 1U;   /* 校验通过：这一步才算完成，才允许进下一步 */
                    }
                    else
                    {
                        /* 误差超容差 -> **闭环纠正**：按"目标 − 当前"再走一次剩余距离 */
                        if (seqMotorRetry < 250U)
                        {
                            seqMotorRetry++;   /* 防 uint8 溢出 */
                        }
                        seqMotorStallTick = now;
                        Debug_Log("SEQ", "step%u %s fix%u cur=%d tgt=%d err=%d tol=%d",
                                  (unsigned int)(seqStep + 1U), ArmSeq_JointName(st->joint),
                                  (unsigned int)seqMotorRetry, (int)cur, (int)st->param,
                                  (int)err, (int)posTol);
                        Arm_MoveAbsDir(addr, st->param, mvel, macc, st->dir);
                    }
                }
            }

            /* 硬上限：反复纠正仍到不了 -> 报警中止，**绝不带着误差往下走** */
            if ((done == 0U) && (elapsed >= (uint32_t)ARM_MOVE_TIMEOUT_MS))
            {
                Debug_Log("SEQ", "step%u %s TIMEOUT cur=%d tgt=%d valid=%u fixed=%u",
                          (unsigned int)(seqStep + 1U), ArmSeq_JointName(st->joint),
                          (int)Arm_GetPos(addr), (int)st->param,
                          (unsigned int)Arm_IsPosValid(addr),
                          (unsigned int)seqMotorRetry);
                ArmSeq_Abort("turn not in position");
                return;
            }
        }
        else
        {
            /* ================= 电机6：容错（到位帧 / 位置 / 停住补发） ================= */
            /* 判据① 的"位置校验窗"：到位帧离目标超过这个距离就认为它是**上一步的迟到应答**。
               ARM_SEQ_REACH_GUARD_MUL = 0 时取 int32 最大值 = 关闭校验（老行为）。 */
            int32_t guardTol = (ARM_SEQ_REACH_GUARD_MUL == 0U)
                             ? (int32_t)0x7FFFFFFF
                             : (posTol * (int32_t)ARM_SEQ_REACH_GUARD_MUL);

            if (Arm_IsReachDone(addr) != 0U)
            {
                /* 判据①：驱动器到位帧 —— **带位置校验**（见 ARM_SEQ_REACH_GUARD_MUL 说明） */
                if ((Arm_IsPosValid(addr) == 0U) ||
                    (ArmSeq_NearTarget(addr, st->param, guardTol) != 0U))
                {
                    done = 1U;
                }
                else if (seqMotorReachIgnored == 0U)
                {
                    /* 位置离目标还很远 -> 这条到位帧是"清标志后才到的旧应答"，忽略它，
                       继续往下走位置闭环（判据②③），别让这一步被提前判完成 */
                    seqMotorReachIgnored = 1U;
                    Debug_Log("SEQ", "step%u %s reach ignored cur=%d tgt=%d (stale?)",
                              (unsigned int)(seqStep + 1U), ArmSeq_JointName(st->joint),
                              (int)Arm_GetPos(addr), (int)st->param);
                }
                else
                {
                    /* 已提示过，继续忽略这一条 */
                }
            }
            else if (ArmSeq_NearTarget(addr, st->param, posTol) != 0U)
            {
                /* 判据②：位置已进**本轴**到位容差（目标就是当前位置、或到位置了但没回到位帧） */
                done = 1U;
            }
            else if (Arm_IsPosValid(addr) != 0U)
            {
                /* 判据③（闭环）：看位置有没有在变 —— 停住且没到目标就补发剩余距离 */
                int32_t cur = Arm_GetPos(addr);

                if (cur != seqMotorLastPos)
                {
                    /* 位置在变 => 电机还在朝目标走，刷新基准与计时 */
                    seqMotorLastPos   = cur;
                    seqMotorStallTick = now;
                    seqMotorStallOn   = 1U;
                }
                else if (seqMotorStallOn != 0U)
                {
                    if ((now - seqMotorStallTick) < (uint32_t)ARM_SEQ_MOVE_STALL_MS)
                    {
                        /* 位置不变但还没到 STALL_MS，先继续观察 */
                    }
                    else if (seqMotorRetry < (uint8_t)ARM_SEQ_MOVE_RETRY_MAX)
                    {
                        /* 【闭环核心】电机停住但离目标还差 delta：按"目标 − 当前"补发一次 */
                        seqMotorRetry++;
                        seqMotorStallTick = now;
                        Debug_Log("SEQ", "step%u %s retry %u cur=%d tgt=%d",
                                  (unsigned int)(seqStep + 1U), ArmSeq_JointName(st->joint),
                                  (unsigned int)seqMotorRetry, (int)cur, (int)st->param);
                        Arm_MoveAbsDir(addr, st->param, mvel, macc, st->dir);
                    }
#if (ARM_SEQ_MOVE_LENIENT != 0U)
                    else
                    {
                        /* 补发用完仍未到：**不卡死流程**，打警告日志后继续下一步 */
                        Debug_Log("SEQ", "step%u %s UNREACHED cur=%d tgt=%d, continue",
                                  (unsigned int)(seqStep + 1U), ArmSeq_JointName(st->joint),
                                  (int)cur, (int)st->param);
                        done = 1U;
                    }
#else
                    else
                    {
                        /* 严格模式：补发用完仍未到就中止（ARM_SEQ_MOVE_LENIENT = 0） */
                        Debug_Log("SEQ", "step%u %s UNREACHED cur=%d tgt=%d, abort",
                                  (unsigned int)(seqStep + 1U), ArmSeq_JointName(st->joint),
                                  (int)cur, (int)st->param);
                        ArmSeq_Abort("motor not reachable");
                        return;
                    }
#endif
                }
                else
                {
                    /* 还没建立位置基准（这一拍刚下发），下一拍再判 */
                }
            }
            else
            {
                /* 读不到位置：只能靠到位帧，兜底就是下面的总超时 */
            }

            /* 总超时（硬上限）：即使电机一直在慢慢动，也不会无限等下去 */
            if ((done == 0U) && (elapsed >= (uint32_t)ARM_MOVE_TIMEOUT_MS))
            {
                /* 【排查用】超时前把这一步的账打清楚（三种情况一眼区分）：
                     tgt   = 动作表里写的目标坐标；
                     cur   = 当前读回的坐标；valid = 坐标是否读到过；
                     reach = 到位标志（0x3A 查询 / 0xFD 的 0x9F 完成帧）。
                   valid=0            -> 读不到位置，这一步的命令根本没下发（查电机 Response / Arm_Update 调度）
                   valid=1 reach=0 且 cur≈tgt -> 电机到位了但没回到位帧（查电机 Response 是否含 0x3A / 是否 None）
                   valid=1 reach=0 且 cur≠tgt -> 电机没走到目标（被结构挡住 / 力矩不够 / 速度太慢） */
                Debug_Log("SEQ", "step%u %s TIMEOUT tgt=%d cur=%d valid=%u reach=%u",
                          (unsigned int)(seqStep + 1U), ArmSeq_JointName(st->joint),
                          (int)st->param, (int)Arm_GetPos(addr),
                          (unsigned int)Arm_IsPosValid(addr),
                          (unsigned int)Arm_IsReachDone(addr));
                ArmSeq_Abort("move timeout");
                return;
            }
        }
    }
    else
    {
        /* 舵机步：优先用回读值判断是否已到目标（提前放行），
           回读不可用时退回按 SERVO_MOVE_TIME_MS + 余量 计时 */
        uint8_t  sid    = (st->joint == AJ_CLAW) ? (uint8_t)SERVO_ID_CLAW
                                                 : (uint8_t)SERVO_ID_GRIP;
        uint16_t target = ArmSeq_ServoParam(st->param);
        uint16_t rd     = 0U;

        if ((Servo_GetPos(sid, &rd) != 0U) &&
            ((int32_t)(Servo_GetPosTickById(sid) - seqStepTick) >= 0))
        {
            /* 必须用"本步下发之后"才拿到的回读值（且是**本 ID** 的），否则会拿上一拍的旧值提前放行 */
            int32_t d = (int32_t)rd - (int32_t)target;

            if (d < 0)
            {
                d = -d;
            }

            if (d <= (int32_t)ARM_SEQ_SERVO_TOL)
            {
                done = 1U;   /* 回读确认到位 */
            }
        }

        if ((done == 0U) && (elapsed >= ARM_SEQ_SERVO_WAIT_MS))
        {
            /* 超时兜底：回读没确认到位也往下走，但打日志提示
               （舵机堵转/力矩不够/舵机 ID 不对都可能表现在这里） */
            done = 1U;
            Debug_Log("SEQ", "step%u %s time-fallback (readback not reached)",
                      (unsigned int)(seqStep + 1U), ArmSeq_JointName(st->joint));
        }
    }

    if (done != 0U)
    {
        seqStep++;
        seqStepSent = 0U;
        seqStepTick = now;
        Arm_SetFastPoll(0U);   /* 本步结束：关掉位置快查，恢复正常 6 拍轮转 */
        Display_ArmSeqRefresh();
    }
}

/**
 * @brief  切换当前选中的动作（循环）
 * @param  next: 1=下一个；0=上一个
 * @retval 无
 */
static void ArmSeq_SelAction(uint8_t next)
{
    if (next != 0U)
    {
        seqAction++;
        if (seqAction >= ARM_ACTION_COUNT)
        {
            seqAction = 0U;
        }
    }
    else
    {
        if (seqAction == 0U)
        {
            seqAction = (uint8_t)(ARM_ACTION_COUNT - 1U);
        }
        else
        {
            seqAction--;
        }
    }

    /* 【换动作 = 重新准备】把 DONE / ABORT 复位成 READY：
       否则状态一直停在上一个动作的完成态，页面第2行显示的还是旧动作名，
       而且按 KEY5 会"没反应"（KEY5 在 DONE/ABORT 下也要能启动，见 HandleKey）。
       READY 状态下换动作不需要复位（本来就处于待启动）。 */
    if ((seqState == SEQ_DONE) || (seqState == SEQ_ABORT))
    {
        seqState    = SEQ_READY;
        seqStep     = 0U;
        seqStepSent = 0U;
    }

    Debug_Log("SEQ", "action=%u(%s)",
              (unsigned int)seqAction, armActions[seqAction].name);
    Display_ArmSeqRefresh();
}

/* ===================== 公开接口 ===================== */

/**
 * @brief  进入手臂调试页
 * @param  无
 * @retval 无
 * @note   使能电机 5/6（位置模式必须先使能），随后立刻启动回原点流程。
 */
void ArmSeq_Enter(void)
{
    seqActive   = 1U;
    seqAction   = 0U;
    seqStep     = 0U;
    seqStepSent = 0U;

    Display_SetPage(PAGE_ARM_SEQ);

    /* 使能两个电机（只在这里发生，初始化阶段不发） */
    Arm_Enable(ARM_ADDR_LIFT, 1U);
    Arm_Enable(ARM_ADDR_TURN, 1U);

    Debug_Log("SEQ", "enter");

    ArmSeq_StartInit();
}

/**
 * @brief  退出手臂调试页（中止 + 停电机 + 回主菜单）
 * @param  无
 * @retval 无
 */
void ArmSeq_Exit(void)
{
    Arm_Stop(ARM_ADDR_TURN);
    Arm_Stop(ARM_ADDR_LIFT);

    Arm_SetFastPoll(0U);   /* 退出：务必关掉位置快查，否则其它电机的状态不再刷新 */
    seqActive = 0U;
    seqState  = SEQ_IDLE;

    Debug_Log("SEQ", "exit");

    Display_MenuEnter();
}

/**
 * @brief  本页主循环节拍（非阻塞）
 * @param  无
 * @retval 无
 */
void ArmSeq_Update(void)
{
    if (seqActive == 0U)
    {
        return;
    }

    switch (seqState)
    {
        case SEQ_INIT:
            ArmSeq_InitUpdate();
            break;

        case SEQ_RUN:
            ArmSeq_RunUpdate();
            break;

        default:
            /* IDLE / READY / DONE / ABORT：无状态机要推进 */
            break;
    }
}

/**
 * @brief  本页按键处理（短按）
 * @param  key: 按键 ID
 * @retval 无
 */
void ArmSeq_HandleKey(KeyID_t key)
{
    if (key == KEY_NONE)
    {
        return;
    }

    switch (key)
    {
        case KEY1:   /* 上一个动作（运行中忽略，防跑飞） */
            if (seqState != SEQ_RUN)
            {
                ArmSeq_SelAction(0U);
            }
            break;

        case KEY2:   /* 下一个动作（运行中忽略） */
            if (seqState != SEQ_RUN)
            {
                ArmSeq_SelAction(1U);
            }
            break;

        case KEY3:   /* 重新执行回原点流程（运行中忽略） */
            if (seqState != SEQ_RUN)
            {
                ArmSeq_StartInit();
            }
            break;

        case KEY4:   /* 中止 + 退出回主菜单 */
            ArmSeq_Exit();
            break;

        case KEY5:   /* READY / DONE / ABORT 都能启动当前动作；RUN 中止；INIT 中不响应 */
            if ((seqState == SEQ_READY) ||
                (seqState == SEQ_DONE)  ||
                (seqState == SEQ_ABORT))
            {
                seqStep     = 0U;
                seqStepSent = 0U;
                seqStepTick = HAL_GetTick();
                seqState    = SEQ_RUN;

                Debug_Log("SEQ", "run action=%u(%s) steps=%u",
                          (unsigned int)seqAction, armActions[seqAction].name,
                          (unsigned int)armActions[seqAction].stepCount);
                Display_ArmSeqRefresh();
            }
            else if (seqState == SEQ_RUN)
            {
                ArmSeq_Abort("key5 stop");
            }
            else
            {
                /* 回原点中（INIT）：不响应，避免在回原点未完成时插动作 */
            }
            break;

        default:
            break;
    }
}

/**
 * @brief  查询本页是否活动
 * @param  无
 * @retval 1=活动，0=非本页
 */
uint8_t ArmSeq_IsActive(void)
{
    return seqActive;
}

/**
 * @brief  取页面第2行文本（状态 + 动作名 + 序号）
 * @param  无
 * @retval 静态缓冲指针
 * @note   长度控制在 16 字符内（8x16 大字体一屏宽度）。
 */
const char *ArmSeq_GetStateText(void)
{
    static char buf[24];

    switch (seqState)
    {
        case SEQ_INIT:
            /* 回原点进度：子步 1~4 */
            snprintf(buf, sizeof(buf), "INIT %u/%u",
                     (unsigned int)(seqInitStep + 1U), (unsigned int)SEQ_INIT_STEP_COUNT);
            break;

        case SEQ_READY:
            /* 就绪：动作名 + 动作序号/总数 */
            snprintf(buf, sizeof(buf), "READY %s %u/%u",
                     armActions[seqAction].name,
                     (unsigned int)(seqAction + 1U), (unsigned int)ARM_ACTION_COUNT);
            break;

        case SEQ_RUN:
            /* 运行中：动作名 + 步号/总步数 */
            snprintf(buf, sizeof(buf), "RUN %s %u/%u",
                     armActions[seqAction].name,
                     (unsigned int)(seqStep + 1U),
                     (unsigned int)armActions[seqAction].stepCount);
            break;

        case SEQ_DONE:
            snprintf(buf, sizeof(buf), "DONE %s", armActions[seqAction].name);
            break;

        case SEQ_ABORT:
            snprintf(buf, sizeof(buf), "ABORT %s", armActions[seqAction].name);
            break;

        default:
            snprintf(buf, sizeof(buf), "IDLE");
            break;
    }

    return buf;
}

/**
 * @brief  取页面第3行文本（当前关节与参数）
 * @param  无
 * @retval 静态缓冲指针
 * @note   运行中显示"关节名 + 参数"（如 "LIFT -3000"）；
 *         未运行时显示选中动作的步数与选动作提示（如 "K12SEL steps=4"）。
 */
const char *ArmSeq_GetStepText(void)
{
    static char buf[24];

    if ((seqState == SEQ_RUN) && (seqStep < armActions[seqAction].stepCount))
    {
        const ArmJointStep_t *st = &armActions[seqAction].steps[seqStep];

        /* 电机步骤：若显式指定了方向，参数后面跟 (A) / (B)，方便一眼看出往哪边走
           （自动方向不加后缀） */
        if ((st->joint == AJ_TURN) || (st->joint == AJ_LIFT))
        {
            if (st->dir == (uint8_t)ARM_DIR_A)
            {
                snprintf(buf, sizeof(buf), "%s %d (A)",
                         ArmSeq_JointName(st->joint), (int)st->param);
            }
            else if (st->dir == (uint8_t)ARM_DIR_B)
            {
                snprintf(buf, sizeof(buf), "%s %d (B)",
                         ArmSeq_JointName(st->joint), (int)st->param);
            }
            else
            {
                snprintf(buf, sizeof(buf), "%s %d",
                         ArmSeq_JointName(st->joint), (int)st->param);
            }
        }
        else
        {
            snprintf(buf, sizeof(buf), "%s %d",
                     ArmSeq_JointName(st->joint), (int)st->param);
        }
        return buf;
    }

    snprintf(buf, sizeof(buf), "K12SEL steps=%u",
             (unsigned int)armActions[seqAction].stepCount);
    return buf;
}

/**
 * @brief  取页面第4行文本（按键提示）
 * @param  无
 * @retval 静态字符串
 */
const char *ArmSeq_GetHintText(void)
{
    return (seqState == SEQ_RUN) ? "K5=STOP K4=EXIT" : "K5=GO K4=EXIT";
}
