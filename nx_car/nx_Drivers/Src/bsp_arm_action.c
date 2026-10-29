/**
 ******************************************************************************
 * @file    bsp_arm_action.c
 * @brief   赛题抓取动作库实现（P9 用）：流程模板 + 标定表 + 非阻塞执行器
 * @author  nx_car
 * @date    2026-09-28
 ******************************************************************************
 * @note   【结构】本文件分三段，**改标定只需要动头文件的 ARM_CAL_LIST**：
 *           ① 流程模板（下面 6 个 flowXxx[]）—— 顺序写死，是"动作怎么走"；
 *           ② 标定表（头文件 ARM_CAL_LIST）—— 是"该位置走到哪"，18 行 × 4 值；
 *           ③ 执行器 —— 把 ①+② 展开成一条步骤数组，逐步下发并等完成。
 *
 *         【为什么不用 bsp_arm_seq 的执行器】那边的执行器与"手臂调试页"的状态机
 *         （回原点 4 子步、位置快查、页面显示）耦合在一起，抽出来要动已验证代码。
 *         这里按同样的原则做一个**精简版**：到位帧为主 + 位置容差兜底 + 硬超时 ABORT，
 *         复用 bsp_arm_seq.h 里的容差/超时/舵机等待参数（口径一致，不另立一套）。
 *         **两者不要同时跑**（都会操作电机5/6 与位置快查）。
 *
 *         硬约束：禁止 HAL_Delay；日志禁 %f；全部非阻塞。
 ******************************************************************************
 */
#include "bsp_arm_action.h"
#include "bsp_arm.h"        /* Arm_Enable / Arm_MoveAbsDir / Arm_IsReachDone / Arm_ServoMove ... */
#include "bsp_servo.h"      /* Servo_GetPos：舵机到位回读 */
#include "bsp_arm_seq.h"    /* 复用 ARM_SEQ_* 容差/超时/舵机等待（口径与手臂调试页一致） */
#include "bsp_debug.h"
#include <stdio.h>

/* ===================== ② 标定表（数据来自头文件 ARM_CAL_LIST） ===================== */

typedef struct {
    int32_t m5;   /* 电机5 转向位置（下发脉冲域，可负） */
    int32_t m6;   /* 电机6 升降位置（向下为负） */
    int32_t s1;   /* 舵机1 伸爪位置 0~1000 */
    int32_t s2;   /* 舵机2 夹紧/松开位置 0~1000 */
    int32_t s3;   /* ★舵机3 车载转盘位置 0~1000（只有 CAR_PLACE / CAR_PICK 用） */
} ArmCal_t;

typedef struct {
    uint8_t  op;    /* ArmOp_t */
    uint8_t  pos;   /* 1~3 */
    ArmCal_t cal;
} ArmCalRow_t;

#define ARM_CAL(op, pos, m5, m6, s1, s2, s3) \
    { (uint8_t)(op), (uint8_t)(pos), { (m5), (m6), (s1), (s2), (s3) } },
static const ArmCalRow_t actCalTable[] = { ARM_CAL_LIST };
#undef ARM_CAL

#define ACT_CAL_COUNT ((uint8_t)(sizeof(actCalTable) / sizeof(actCalTable[0])))

/* ★【2026-10-23 用户要求】车载转盘（舵机3）的 3 个角度，**只这一处**：
 *   下标 0/1/2 = 号位 1/2/3；运行期"要抓/放的颜色"先查到绑定号位，再取这里的角度。
 *   标定方法见 bsp_arm_action.h 的 ARM_CAR_S3_POS1/2/3 注释（ARMPARA 页手扳读数）。 */
static const int32_t actCarS3[3] = {
    (int32_t)ARM_CAR_S3_POS1,
    (int32_t)ARM_CAR_S3_POS2,
    (int32_t)ARM_CAR_S3_POS3
};

/* ===================== ① 流程模板 ===================== */
/* 每步 =（关节，取值来源）。取值来源见 ActValSrc_t：
 *   AS_M5/AS_M6/AS_S1/AS_S2/AS_S3 = 取**本行标定值**（每个位置不同）
 *   AS_FIX_*               = 取固定值（所有位置相同，见 bsp_arm_action.h）
 * **数组书写顺序 = 执行顺序**，要调整动作顺序就改这里（但一般不需要动）。 */

typedef enum { AJT_TURN = 0, AJT_LIFT, AJT_CLAW, AJT_GRIP, AJT_CAR } ActJoint_t;
typedef enum {
    AS_M5 = 0, AS_M6, AS_S1, AS_S2, AS_S3,
    AS_CAR_M5, AS_CAR_M6, AS_CAR_S1,          /* ★车载统一姿态（3 个位置 + 抓放共用） */
    AS_CAR_PLACE_M6,                          /* ★车载**放料**高度（比抓取高 2000，见头文件） */
    AS_TT_READY_M5, AS_TT_READY_M6, AS_TT_READY_S1,   /* ★观察位（全局，TRAY PICK 用） */
    AS_FIX_S2_OPEN, AS_FIX_S1_RET, AS_FIX_M6_SAFE,
    AS_FIX_SB_SAFE, AS_FIX_S2_CLOSE, AS_FIX_S3_HOME
} ActValSrc_t;

/* ★★【2026-10-24 用户要求】"转盘区手臂联调再丝滑一些"——
   用**每步一个标志位**精确控制"哪两步可以同拍下发 / 哪一步发完就走"，
   不再靠全局猜"相邻两步能不能并行"（以前那样会把不该并行的也并进去）。
   · `ACT_STEP_PAR`      ：本步与**下一步**同拍下发，两个都到位才跨两步（并行对）；
   · `ACT_STEP_NOWAIT`   ：舵机步**命令发出即放行**（用户："松抓指令下发就行"）；
   · `ACT_STEP_EARLY`    ：电机步**进入 ±`ACT_TURN_EARLY_REL` 就提前放行**
                            （用户 TRAY PICK："电机5 转到目标 ±100 内电机6下降"）。 */
#define ACT_STEP_PAR            0x01U
#define ACT_STEP_NOWAIT         0x02U
#define ACT_STEP_EARLY          0x04U
/* ★【2026-10-23 用户要求 · STORE BACK 专用】转向步**必须等舵机1 收 < 500 才发**。
   本组里 S1 缩回与舵盘（S3）转位是**同拍**的，转向步前面隔着一步 CAR，
   全局"CLAW 紧邻 TURN 才拦"的闸门打不到，所以给 TURN 步单独标本标志。 */
#define ACT_STEP_S1_GATE        0x08U
/* ★【2026-10-23 用户要求 · STORE BACK】**电机6 与舵机1 错开 200ms 再动**
   （同时动时物料会撞到车体）。标在**要延后的那一步**上：
   该步不与上一步同拍下发，而是等 `ACT_LATE_START_MS` 后再发；
   期间上一步（电机6）继续上升，两者**并行、只是错开启动**。 */
#define ACT_STEP_LATE_START     0x10U
#define ACT_LATE_START_MS       200U
/* ★【2026-10-24 用户要求】"放料时物料还没放稳，舵机3 就开始转"：
   舵机步命令发出后**强制静置 `ACT_PLACE_SETTLE_MS`** 才放行下一步
   （用于放料松爪步）。与 `ACT_STEP_NOWAIT` 互斥——本标志优先。 */
#define ACT_STEP_SETTLE         0x20U
/* ★★【2026-10-29 用户要求】"抓取/放置车载时爪子要真的伸出去"：
   车载伸缩位 `ARM_CAR_S1`(66) 与缩回位 0 只差 66，落在 `ACT_SERVO_LOOSE_TOL`(80) 里，
   于是每个 `CLAW -> 66` 的步都被 `Act_StepAlreadyThere()` 判成"已到位"**直接跳过**
   （日志实证：`step2/13 CLAW already at 66 -> next` ×4），命令一帧都没发 -> 现场"手不伸"。
   本标志 = **该步必须真正下发**，不参与"已经在那儿"的跳过。 */
#define ACT_STEP_FORCE          0x40U

/* ★【2026-10-24 用户要求】"电机5 转向到达目标参数 **±100** 范围内，电机6 下降" */
#define ACT_TURN_EARLY_REL      100
/* ★【2026-10-23 用户要求 · STORE BACK】"电机6 上升，**离目标 ±10000** 就开始提前回缩舵机1"
   （单位：脉冲；M6 从取件高度升到安全高约 54000 脉冲，等于最后 ~1/5 段开始收爪） */
#define ACT_LIFT_EARLY_REL      10000

typedef struct { uint8_t joint; uint8_t src; uint8_t flags; } ActFlowStep_t;

/* ---- A. 抓取转盘 1~3 号位 ----
   【2026-10-06 用户要求】手臂停在**观察位**，一收到"可以抓"的指令就**直接夹**：
   "观察位 -> 抓"之间**不留任何多余动作**。

   前提（现场标定 + 调用顺序都要满足）：
     ① **观察位本身就是抓取姿态** —— S2 = 松开(1000)、S1 = 取料位、
        M5 = 转盘1号位、M6 = 取料高度；
     ② 调用前机械臂**已经在观察位**（bsp_tt / 调试页都是先摆好臂再看料、对位）。
   注意：**观察位 M6 现在是取料高度**，改 ARM_TT_READY_M6 时不能只看"能不能看见圆环"，
   还要保证爪子在这个高度够得到料。抓完仍要抬到安全高，否则车一动会撞转盘。

   【2026-10-10 加回"舵机1 缩回"，修"放料时手臂抖动"】
   观察位 S1 = 244（伸爪）：抓完若直接大幅转头，就是**带着伸出去的爪子转**
   → 手臂抖动、有扫撞风险。所以抓完必须"抬到安全高 + 收爪"。
   【2026-10-24 用户要求】这两件 + 转向**同时做**（见下面三路并行组），
   靠 `ACT_PAIR_GRACE_MS` 兜底；若现场发现"边转边收"仍会扫到转盘，
   把第 3 步的 `ACT_STEP_PAR` 去掉即可退回"先抬+收，再转"。
   （★2026-10-21 用户说的"删掉缩回"指的是**观察区去抓取**那段 `ArmAct_GotoTtPos`
     的规则3 插步，不是这里；见 `actSkipRule3`。） */
static const ActFlowStep_t flowTtPick[] = {
    { AJT_GRIP, AS_S2,          0U           },   /* 1 舵机2 夹紧（= 直接抓，观察位就在取料位） */
    { AJT_LIFT, AS_FIX_M6_SAFE, ACT_STEP_PAR },   /* 2 ★电机6 抬到安全高（与 3 步同拍） */
    { AJT_CLAW, AS_FIX_S1_RET,  ACT_STEP_PAR },   /* 3 ★缩回；**转向要等它 <500（全局闸门）** */
    { AJT_TURN, AS_CAR_M5,      0U           },   /* 4 ★电机5 转向车载放置位（S1<500 后才发） */
};
/* ★【2026-10-24 用户要求】"抓取到物料后：**抬升 M6 + 缩回 S1 同时进行**；
   **转向 M5 等 S1 收到 500 以下再发**（S1 伸太长时转身向心力太大）；
   如果发出这一刻 S1 本来就 < 500，则**三者仍然同时**下发。"
   步骤 2/3 是并行组；第 4 步转向被**全局闸门**挂起（`Act_TurnGateKick`）：
   · 只要"当前步是舵机1、下一步是 TURN"，就先看 S1 回读 —— 已 < 500 才同拍发；
   · 否则挂起，**一直等到 S1 < 500 才发**（回读可用时上限 2000ms，读不到才 400ms 兜底）；
   · 转向发出去后，组变成 3 步：三个关节**都到位**才结束本动作。
   · 转向目标直接取车载统一姿态 `ARM_CAR_M5`，因为 TT_PICK 之后必然接"放车载"，
     省掉一次中间转向；之后 `CAR_PLACE` 里那一步 M5 就不会再动（绝对定位：已在目标）。 */

/* ===================== ★ TRAY PICK 专用动作组（2026-10-24 用户重新定义） =====================
   【用户口径 · 每个物料一遍，共 3 件】
     ① 电机5 和舵机1 同时动作到观察区对位（同时舵机3 转到相应车载位号，第 1 件=1 号位）
     ② 识别到目标物料：舵机1（伸出或缩回）和电机6 下降同时运行到等待位
     ③ 看到目标物料：舵机2 抓取
     ④ 电机6 抬升和舵机1 缩回同时运行；舵机1 < 500 时电机5 开始转向
        （如果本来就 < 500，则电机5 / 电机6 / 舵机1 三者同拍）
     ⑤ 电机5 转到目标参数 ±100 内 -> 电机6 下降；到位直接松手舵机2
     ⑥ 电机5 和舵机1 同时动作回观察区，进行下一个物料（同时舵机3 转到下一个号位）
   【与其它任务点的关系】**完全独立**：这两个数组只被 `ArmAct_TrayReady()` /
     `ArmAct_TrayPickPlace()` 使用，改 STORE PLACE / STORE BACK 不影响这里。 */

/* A2-1. 回观察区对位（①）+ 舵盘选号位（S3 由 `ArmAct_PreSelectSlot` 同时启动） */
static const ActFlowStep_t flowTrayReady[] = {
    { AJT_GRIP, AS_FIX_S2_OPEN, ACT_STEP_NOWAIT },  /* 0 舵机2 松开（命令即走，空爪） */
    { AJT_LIFT, AS_TT_READY_M6, 0U              },  /* 1 电机6 -> 观察高 */
    { AJT_CLAW, AS_TT_READY_S1, ACT_STEP_PAR    },  /* 2 舵机1 -> 观察位 ‖ */
    { AJT_TURN, AS_TT_READY_M5, 0U              },  /* 3 电机5 -> 观察位 ‖（与 2 同拍） */
};

/* A2-2. 抓取 + 放车载（③④⑤，一整套）
   ★【2026-10-24 转盘新抓取逻辑】末尾补第 7 步"电机6 抬回安全高"：
     新逻辑第 2/3 件**不再回观察位**（不调 `ArmAct_TrayReady()`），放完料后直接
     `ArmAct_GotoTtPos()` 转 M5 去下一个位号。而 `flowTtPrep` 第 1 步就是转 M5，
     若此时 M6 还停在车载高度就会扫到转盘/物料 —— 所以本动作收尾必须先把爪子
     抬离车载高度。顺带让"最后一件跑完"的手臂姿态也是安全高，便于接后续动作。 */
static const ActFlowStep_t flowTrayPickPlace[] = {
    { AJT_GRIP, AS_FIX_S2_CLOSE, 0U              },  /* 1 舵机2 夹紧 = 抓 */
    { AJT_LIFT, AS_FIX_M6_SAFE,  ACT_STEP_PAR    },  /* 2 电机6 抬安全高 ‖ */
    { AJT_CLAW, AS_FIX_S1_RET,   ACT_STEP_PAR    },  /* 3 舵机1 缩回 ‖（S1<500 才发转向） */
    { AJT_TURN, AS_CAR_M5,       ACT_STEP_EARLY | ACT_STEP_S1_GATE },  /* 4 电机5 转向车载位（±100 提前放行，且 S1<500 才发） */
    { AJT_CLAW, AS_CAR_S1,       ACT_STEP_FORCE  },  /* 5 ★伸爪到车载位（★2026-10-29：先伸爪再下降；强制下发，否则被"已到位"跳过） */
    { AJT_LIFT, AS_CAR_PLACE_M6, 0U              },  /* 6 ★电机6 降到车载**放料**高度（比抓取高 2000，防撞翻） */
    { AJT_GRIP, AS_FIX_S2_OPEN,  ACT_STEP_SETTLE },  /* 7 舵机2 松手（静置落料后才走） */
    { AJT_LIFT, AS_FIX_M6_SAFE,  0U              },  /* 8 ★抬回安全高（空爪，为下一次转 M5 让路） */
};

/* ---- B. 放置车载 1~3 号盘 ----
   ★【2026-10-24 用户要求】放车载（三个位置同一套动作）要更丝滑：
     "抓完料 -> 到车载位上方 -> **直接下降 -> 松抓**，不要犹豫"。
     所以本动作只剩 **降 M6 -> 松抓(不等待) -> 抬高**。
   ★★【2026-10-24 用户要求】**删掉开头的"S1 缩回 + M5 转向"两步**：
     这两步与**前一段抓取动作组 `flowTtPick` 末尾完全重合**
     （TTPICK 第 3 步 S1 -> 0、第 4 步 M5 -> `ARM_CAR_M5`，目标值一模一样）——
     重发一遍只是多占一次舵机/电机总线，还把对位时刚预转的 **S3 挤丢步**。
     现在保证"缩回 / 转向"**只在 TTPICK 执行一次**。
   · 进入本动作时 M6 已在 TT_PICK 抬到的安全高（-1000），所以先转后降的"安全高转向"
     已由 TTPICK 完成；这里直接下降。
   · 第 2 步带 `ACT_STEP_NOWAIT`：用户明确"松抓指令下发就行"，不等回读确认。
   ★【2026-10-24 用户要求】舵盘（S3）由**对位成功那拍预转**（`ArmAct_PreSelectSlot`），
     本动作不再单独走"选号位"这一步。
   ⚠ 本动作**只能在 TTPICK 之后调用**（依赖"M5 已在车载位"）。
   ★★【2026-10-29 用户要求】"放置车载时手也要伸出去"：开头补一步 `S1 -> ARM_CAR_S1`(66)，
     且带 `ACT_STEP_FORCE`（66 与缩回位 0 只差 66 < 容差 80，不带本标志会被判"已到位"跳过）。 */

   /* 动序：伸爪 -> 降 M6 -> 松抓(不等待) -> 抬高 */
static const ActFlowStep_t flowCarPlace[] = {
    { AJT_CLAW, AS_CAR_S1,       ACT_STEP_FORCE },  /* 1 ★伸爪到车载位（★2026-10-29 用户要求：放料也要伸出去；强制下发） */
    { AJT_LIFT, AS_CAR_PLACE_M6, 0U                    },  /* 2 ★降到车载**放料**高度（比抓取高 2000，防撞翻） */
    { AJT_GRIP, AS_FIX_S2_OPEN, ACT_STEP_NOWAIT       },  /* 3 ★松开=放料（命令发出即走，不等回读） */
    { AJT_LIFT, AS_FIX_M6_SAFE, 0U                    },  /* 4 电机6 抬高（空爪） */
};
/* 【2026-10-24 已清掉旧的三位置数据】原来这里记着 1/2/3 号盘各自的
   M5=642/347/49、S1=235/82/233、M6=-15415 —— 那套随"车载改用舵机3 选位 +
   手臂统一姿态"一起删除：现在三个号位**只差 S3**，M5/M6/S1 都是 `ARM_CAR_*` 同一组值。
   注意：**"放料"完全由第 4 步的 `AS_FIX_S2_OPEN` 决定**（必须是"松开"值）。
   另外第 4 步的 S2 目标 == 原第 6 步（旧版"舵机2 松开"）——**空跑步，已删**。 */

/* ---- C. 抓取车载 1~3 号盘 ----
   【2026-10-10 ★删掉"舵机2 松开"这一步（空跑）】
   本动作在 **SP_ 打靶放置**里被调用的**唯一**场合，进入时舵机2 一定是"松开(1000)"：
     · 第 1 件：上一段是 `ArmAct_GotoTtReady()`（观察位 S2=1000）；
     · 第 2/3 件：上一段是 `ARM_OP_TGT_PLACE`（放料后 S2=1000）。
   所以"松开"是**空跑步**（目标 == 当前，只是白等一个回读周期），已删除。
   ⚠ 新增调用者必须保证**进入时舵机2 已松开**，否则下压会压到爪子/料。
   ⚠ `ARM_OP_CAR_PICK` 目前**只被 bsp_tt 的 SP_ 调用**（已确认全工程无其它调用点）。 */
static const ActFlowStep_t flowCarPick[] = {
    { AJT_LIFT, AS_FIX_M6_SAFE },                   /* 1 电机6 抬到安全高（先把爪子抬离转盘） */
    { AJT_CAR,  AS_S3          },                   /* 2 ★车载转盘（舵机3）转到该号位 */
    { AJT_TURN, AS_CAR_M5      },                   /* 3 统一转向角（ARM_CAR_M5） */
    { AJT_CLAW, AS_CAR_S1, ACT_STEP_FORCE },        /* 4 统一伸爪位置（ARM_CAR_S1；★2026-10-29 强制下发，真伸出去） */
    { AJT_LIFT, AS_CAR_M6      },                   /* 5 统一高度（ARM_CAR_M6） */
    { AJT_GRIP, AS_FIX_S2_CLOSE},                   /* 6 舵机2 夹紧（固定"夹紧"） */
    { AJT_LIFT, AS_FIX_M6_SAFE },                   /* 7 电机6 抬高（带料） */
    { AJT_CLAW, AS_FIX_S1_RET  },                   /* 8 舵机1 缩回 */
};
/* ★【2026-10-24 用户要求】**舵盘回 0 点不能放在这里**：
   抓完料时手臂还在车载位（M5 朝舵盘），这时转 S3 会和手臂/物料打架。
   正确时机 = "手臂**已经转向靶位之后**、下降之前"，见 `flowTgtPlace` / `flowStack` 第 2 步。 */

/* ---- D. 放置靶子 1~3 号位 ----
   【2026-10-10 按现场要求调整顺序】**先伸舵机1，再降电机6**：
   原来顺序是"下降后伸爪"，爪子前伸时已经贴着靶面，容易顶到靶环/已有料；
   改成"先伸爪 -> 再垂直下降"，落料是纯垂直动作，稳且不刮碰。
   【2026-10-10 晚 用户要求 ★改收尾：**放置后"只松抓"就结束**，不再抬 M6、不再缩爪1】
   目的 = **和 STORE BACK 无缝衔接**：STORE PLACE 最后一件放在哪个靶位，
   臂就**停在那里的靶位姿态**（S1 伸着、M6 在取件高度、S2 已松开），
   STORE BACK 的第一件正好从同一姿态接着做（只需"夹紧 -> 抬安全高 -> 缩爪"，见 `flowSbPickFirst`）。
   ⚠ 因此**本动作结束时臂是"低位 + 伸爪"状态**：
     · 赛题链里它后面紧跟 `T_B1_PBACK`（STORE BACK），而 SB_ 每个动作都以**安全高**收尾
       （`flowSbPick` 第 7 步 / `flowSbPlace` 第 6 步），所以"之后动车"是安全的；
     · **调试页单独跑 `ROUGH PLACE`**（后面没有 STORE BACK）时臂会停在靶位低位，
       之后若要转 M5 或动车，请先摆到安全高（或先跑一次 STORE BACK）。
   ⚠ 另：本动作进入时仍要求**舵机1 已缩回**（否则会带着伸出的爪子转 M5）。 */
static const ActFlowStep_t flowTgtPlace[] = {
    { AJT_TURN, AS_M5,          0U           },   /* 1 电机5 转到该号靶（爪已缩回） */
    { AJT_CAR,  AS_FIX_S3_HOME, 0U           },   /* 2 ★手臂转走之后再让舵盘回 0（防齿条打料） */
    { AJT_CLAW, AS_S1,          0U           },   /* 3 ★舵机1 伸到靶位（先伸爪） */
    { AJT_LIFT, AS_M6,          0U           },   /* 4 ★电机6 垂直降到位（后下降） */
    { AJT_GRIP, AS_S2,          0U           },   /* 5 舵机2 松开 = 放料 —— ★**最后一步就是"松抓"** */
};
/* ★【2026-10-24 用户要求】第 2 步（舵盘回 0）的时机：
   必须**等电机5 已经转到靶位之后**再转 S3 —— 抓完料时手臂还对着舵盘，
   先转 S3 会和手臂/物料打架；反过来，S3 不回到 0，下降时**升降齿条会打到舵盘上的物料**。
   顺序固定为：**转 M5（离开舵盘） -> 舵盘回 0 -> 伸爪 -> 下降 -> 松抓**。
   ⚠ 本步按回读确认到位（`ACT_S3_ARRIVE_TOL`），转不到位会自动重发一次。 */

/* ---- E. 抓取靶子 1~3 号位 ---- */
static const ActFlowStep_t flowTgtPick[] = {
    { AJT_GRIP, AS_FIX_S2_OPEN },               /* 1 舵机2 松开 */
    { AJT_TURN, AS_M5          },               /* 2 电机5 转到该号靶 */
    { AJT_CLAW, AS_S1          },               /* 3 舵机1 伸到靶位 */
    { AJT_LIFT, AS_M6          },               /* 4 电机6 降到位 */
    { AJT_GRIP, AS_S2          },               /* 5 舵机2 夹紧 */
    { AJT_LIFT, AS_FIX_M6_SAFE },               /* 6 电机6 抬高 */
    { AJT_CLAW, AS_FIX_S1_RET  },               /* 7 舵机1 缩回 */
};

/* ===================== ★ STORE BACK 专用动作组（2026-10-23 用户要求重写） =====================
   【旧的 `flowSbPickFirst` / `flowSbPick` / `flowSbPlace` 三块流程已**整段删除**】

   【用户给的完整动序（一件 = 靶位抓 -> 送回车载 -> 顺手摆好下一件）】
     ① PLACE 结束手臂是"仅松手"的姿态 -> **直接抓**（S2 夹紧）
     ② 抓到后 **手臂先上抬（M6）**；**200ms 后**舵机1 回缩 + 舵机3 转本件号位 一起动
        （★2026-10-23 用户要求：M6 与 S1 同时动时**物料会撞到车体**，所以错开 200ms）
     ③ **舵机1 < 500 时**才发转向（M5 -> 车载位）；**本来就 <500** -> 立刻发
     ④ 当 M5 到目标 **±100** 内 -> M6 下降（-> 车载高度）
     ⑤ 到位**直接松手**（S2 松开，命令发完就走）
     ⑥ 手臂转向下一件物料（M5）同时舵盘回 0；然后伸出（S1）同时下降（M6）
        -> 下一件正好从"爪已伸到位、在取件高度、爪子张开"开始（接 ①）

   所以本组按 SB_ 的两段拆成三块数组：
     · `flowSbPick`  = ①抓 -> ②M6 上抬（200ms 后 S1 回缩并入）-> ③M6 到位后 S3 转本件号位
                       （★2026-10-24 用户要求：舵盘必须等物料拿起才转）-> ④转向车载（±100 提前放行）
     · `flowSbPlace` = ④下降到车载高度 -> ⑤松手（静置落稳）-> ⑥抬回安全高
     · `flowSbNext`  = ⑥转向下一件 + 舵盘回 0 -> 伸出 + 下降
       （最后一件没有"下一件"，改用 `flowSbLast`：抬安全高 + 收爪，之后底盘才能动车）
     · `flowSbPrep`  = **独立跑**（没紧跟 PLACE）时，先把手臂摆到本件靶位姿态

   标定值：抓取 / 下一件的 M5/M6/S1/S2 全部复用 `ARM_OP_TGT_PICK` 三行；
           车载姿态统一 `ARM_CAR_*`，舵盘角度按号位取 `actCarS3[]`。
   ⚠ 只用于 STORE BACK，改这里**不影响** TRAY PICK / STORE PLACE（各有自己的数组）。 */
static const ActFlowStep_t flowSbPick[] = {
    { AJT_GRIP, AS_S2,           0U },                        /* 1 直接抓（S2 夹紧，单独运行） */
    { AJT_LIFT, AS_FIX_M6_SAFE,  ACT_STEP_PAR },              /* 2 上抬 M6（先动）‖ 3
                                                                 ★2026-10-28 用户要求：抬升高度
                                                                 与转盘区 `flowTrayPickPlace`
                                                                 第 2 步一致（-1000），不再用
                                                                 SB 专用的 -6576 */
    { AJT_CLAW, AS_FIX_S1_RET,   ACT_STEP_LATE_START },       /* 3 ★晚 200ms 再回缩（防物料撞车体）；
                                                                 ★不带 PAR：并行链看"前一步"的 PAR，
                                                                 本步不标 -> 第 4 步不进本组 */
    { AJT_CAR,  AS_S3,           0U },                        /* 4 ★【2026-10-24 用户要求】舵盘等
                                                                 M6 抬升到位才转（原与 3 同拍，
                                                                 物料没拿起舵盘就转，已改） */
    { AJT_TURN, AS_CAR_M5,       ACT_STEP_EARLY | ACT_STEP_S1_GATE },  /* 5 转向车载；S1<500 才发，±100 提前放行 */
};

/* ④⑤：降到车载高度 -> 松手（静置落料）-> 抬回安全高
   ★【2026-10-26 用户要求】"粗加工区抓取回来放置到车上，转盘舵机3 老是转太早，
     物料还没放稳就转动、物料掉落，可以延用转盘区的放置逻辑"：
     照 `flowTrayPickPlace` 的放料段处理 —— 松爪命令发出后**静置 `ACT_PLACE_SETTLE_MS`**
     （等爪子张开、料落进槽坐稳），再**抬 M6 到安全高**；舵盘回 0（`flowSbNext` 第 2 步）
     自然就排在"抬臂之后"，不会在料还在爪口时就回转刮落。 */
static const ActFlowStep_t flowSbPlace[] = {
    { AJT_CLAW, AS_CAR_S1,       ACT_STEP_FORCE },                     /* 1 ★伸爪到车载位（★2026-10-29；强制下发，否则被"已到位"跳过） */
    { AJT_LIFT, AS_CAR_PLACE_M6, 0U },                                 /* 2 ★降到车载**放料**高度（比抓取高 2000，防撞翻） */
    { AJT_GRIP, AS_FIX_S2_OPEN,  ACT_STEP_SETTLE },                    /* 3 松手 = 放料（静置落稳后才走） */
    { AJT_LIFT, AS_FIX_M6_SAFE,  0U },                                 /* 4 ★抬到安全高（爪子离料后舵盘才转） */
};

/* ⑥：转向下一件 + 舵盘回 0（同拍）-> 伸出 + 下降（同拍） */
static const ActFlowStep_t flowSbNext[] = {
    { AJT_TURN, AS_M5,           ACT_STEP_PAR },                       /* 1 转向下一件物料（‖ 下一步 S3） */
    { AJT_CAR,  AS_FIX_S3_HOME,  0U },                                 /* 2 舵盘回 0（与 1 同拍） */
    { AJT_CLAW, AS_S1,           ACT_STEP_PAR },                       /* 3 伸出（‖ 下一步下降） */
    { AJT_LIFT, AS_M6,           0U },                                 /* 4 下降到下一件取件高度 */
};

/* 最后一件收尾：抬到安全高 + 收爪（空爪离作业面，之后底盘才能动车） */
static const ActFlowStep_t flowSbLast[] = {
    { AJT_LIFT, AS_FIX_M6_SAFE,  0U },                                 /* 1 抬到安全高（★与转盘区同一值 -1000） */
    { AJT_CLAW, AS_FIX_S1_RET,   0U },                                 /* 2 舵机1 缩回 */
};

/* 独立跑（chained=0）时的"摆到本件靶位姿态"：转向靶位 -> 舵盘回 0 -> 伸出 -> 下降 */
static const ActFlowStep_t flowSbPrep[] = {
    { AJT_TURN, AS_M5,           ACT_STEP_PAR },                       /* 1 转向本件靶位（‖ 下一步 S3） */
    { AJT_CAR,  AS_FIX_S3_HOME,  0U },                                 /* 2 舵盘回 0（与 1 同拍） */
    { AJT_CLAW, AS_S1,           ACT_STEP_PAR },                       /* 3 伸到取件位（‖ 下一步下降） */
    { AJT_LIFT, AS_M6,           0U },                                 /* 4 降到取件高度 */
};

/* ---- F. ★堆叠放置到粗加工区 1~3 号圆环（第二批用）----
   堆叠 = 把料放到"**已有物料之上**"：
     · 流程与 TGT_PLACE **完全相同**
       （缩爪 -> 转向 -> 伸爪 -> 下降 -> 松开 -> 升到车载高度 -> 缩爪）；
     · 差别**只在标定值** —— M6 必须比普通放置**高一层**（叠在已有料上，不抬高会撞料）。
   故这里**单独写一份数组**（不直接复用 flowTgtPlace），
   好处：将来若堆叠需要单独调整动作顺序（例如多一段"下压确认"），改这里不影响靶位放置。
   【2026-10-10】与 TGT_PLACE 同步：**先伸舵机1、再降电机6**；收尾**先升到车载高度、再缩爪**。 */
static const ActFlowStep_t flowStack[] = {
    { AJT_CLAW, AS_FIX_S1_RET  },   /* 1 舵机1 缩回（先收爪再转，防扫撞） */
    { AJT_TURN, AS_M5          },   /* 2 电机5 转到该号圆环 */
    { AJT_CAR,  AS_FIX_S3_HOME },   /* 3 ★手臂转走之后再让舵盘回 0（防齿条打料，与 TGT_PLACE 同一口径） */
    { AJT_CLAW, AS_S1          },   /* 4 ★舵机1 伸到位置（先伸爪） */
    { AJT_LIFT, AS_M6          },   /* 5 ★电机6 垂直降到堆叠高度（后下降；标定值已含"高一层"） */
    { AJT_GRIP, AS_S2          },   /* 6 舵机2 松开（放料；本动作的 S2 填"松开"值） */
    { AJT_LIFT, AS_CAR_M6      },   /* 7 ★电机6 升到车载高度(-11449)：下一次抓车载料就在这个高度 */
    { AJT_CLAW, AS_FIX_S1_RET  },   /* 8 ★舵机1 缩回（升到位后再收爪） */
};

/* ---- G. ★【转盘准备位】摆到"转盘 pos 号位取料姿态"（**不含舵机2 抓紧**）----
   【为什么要有这条】观察位（`ArmAct_GotoTtReady()`，高位看料）**不等于**取料高度，
   所以从观察位下来必须先"摆到取料姿态"，等目标色转过来再单独执行"抓紧"。
   【参数从哪来】本流程的 M5/M6/S1 **取 `ARM_OP_TT_PICK`（转盘 3 个位号动作组）对应行的值**，
   不单独开标定行 —— 也就是"准备位的参数由转盘 3 个位号动作组提供"。
   【为什么不含 S2】抓紧要等 `$CCG` 确认目标色出现后才做（见流程 §6.1），
   所以这一步**只摆姿态、不动舵机2**；真正的"抓紧" = `ArmAct_Run(ARM_OP_TT_PICK, pos)`。 */
static const ActFlowStep_t flowTtPrep[] = {
    { AJT_TURN, AS_M5,  0U              },   /* 1 电机5 转到该位 */
    { AJT_CLAW, AS_S1,  ACT_STEP_PAR    },   /* 2 ★舵机1 伸到取料位（与下一步"下降"同拍下发） */
    { AJT_LIFT, AS_M6,  0U              },   /* 3 ★电机6 降到取料高度（与"伸爪"同时动；此处**不夹紧**） */
};

/* ---- H. ★【2026-10-11 用户要求】"放置完成收尾"：手臂回到**车载号位、不抓**的状态 ----
   用户口径："粗加工区三个放置完后，手臂回到车载 3 号位**不抓**的状态，
             顺序是**先电机6 抬高，舵机1 收回，电机5 转向，舵机2 张开**"。
   参数：★2026-10-22 起车载姿态统一用 `ARM_CAR_M5`（`AS_CAR_M5`），
        M6 用固定"安全高"、S1 用固定"收回"、S2 用固定"张开"，**不取行里的值**
        （所以最后一定是"空爪 + 爪张开"的待命姿态，不会把爪子伸在盘位里）。 */
static const ActFlowStep_t flowCarIdle[] = {
    { AJT_LIFT, AS_FIX_M6_SAFE },   /* 1 ★电机6 抬高（到安全高） */
    { AJT_CLAW, AS_FIX_S1_RET  },   /* 2 ★舵机1 收回 */
    { AJT_CAR,  AS_S3          },   /* 3 ★车载转盘（舵机3）转到该号位 */
    { AJT_TURN, AS_CAR_M5      },   /* 4 ★统一转向角（ARM_CAR_M5） */
    { AJT_GRIP, AS_FIX_S2_OPEN },   /* 5 ★舵机2 张开（不抓） */
};

/* 动作 -> 流程模板 的对照表（顺序与 ArmOp_t 一致，增删动作要同步改两处） */
typedef struct {
    const ActFlowStep_t *flow;
    uint8_t              count;
    const char          *name;   /* 日志/摘要用（≤6 字符） */
} ActOpInfo_t;

#define ACT_FLOW_ROW(flowArr, dispName) \
    { (flowArr), (uint8_t)(sizeof(flowArr) / sizeof((flowArr)[0])), (dispName) }

static const ActOpInfo_t actOps[ARM_OP_COUNT] = {
    ACT_FLOW_ROW(flowTtPick,   "TTPICK"),
    ACT_FLOW_ROW(flowCarPlace, "CARPUT"),
    ACT_FLOW_ROW(flowCarPick,  "CARGET"),
    ACT_FLOW_ROW(flowTgtPlace, "TGTPUT"),
    ACT_FLOW_ROW(flowTgtPick,  "TGTGET"),
    ACT_FLOW_ROW(flowStack,    "STACK"),   /* ★必须与 ARM_OP_LIST 的顺序一致（追加在末尾） */
};

/* ===================== ③ 执行器 ===================== */

/* 展开后的实际步骤（**必须 static**：执行期间要一直有效） */
#define ACT_MAX_STEPS   16U

typedef struct {
    uint8_t joint;   /* ActJoint_t */
    uint8_t flags;   /* ACT_STEP_* 位掩码（并行 / 不等待 / 提前转向） */
    int32_t param;   /* 电机 = 绝对位置；舵机 = 0~1000 */
} ActStep_t;

typedef enum {
    ACT_IDLE = 0,   /* 空闲 */
    ACT_RUN,        /* 执行中 */
    ACT_DONE,       /* 已跑完 */
    ACT_ABORT       /* 被中止 / 超时 */
} ActState_t;

static ActStep_t actSteps[ACT_MAX_STEPS];
static uint8_t   actStepCount = 0U;
static uint8_t   actIdx       = 0U;   /* 当前步号（0 起） */
static uint8_t   actStepSent  = 0U;   /* 本步命令是否已下发 */
static uint32_t  actStepTick  = 0U;   /* 本步开始时刻（超时/舵机计时用） */
static uint32_t  actRunTick   = 0U;   /* 本段动作开始时刻（统计整段时间用） */
static uint8_t   actState     = ACT_IDLE;
static char      actTitle[12];        /* 形如 "CARGET2" / "TGTPUT1" */
static uint8_t   actPosWaitOn   = 0U; /* 1=本步正在等"当前位置可读"（防重复打日志） */
static uint32_t  actPosWaitTick = 0U; /* 开始等位置的时刻 */
/* 电机步位置哨兵：表示"当前位置不可用"（丢帧看门狗判断"有没有动"时用）。 */
#define ACT_POS_NONE    (0x7FFFFFFF)

static uint32_t  actSrvBusWaitTick = 0U;  /* ★0=没在等总线；否则=开始等 9600 总线静默的时刻 */
/* ★【2026-10-24】车载转盘（舵机3）"对位成功就预选号位"的状态：
   actPos        = 本次动作的车载号位（1~3；非车载动作 / 无号位 = 0）
   actCarS3Pre   = 已被预选的号位（0 = 没预选）
   actCarS3PreTick = 预选命令发出时刻（用于判断"是否已转过 ACT_S3_MOVE_MS"） */
static uint8_t   actPos         = 0U;
static uint8_t   actCarS3Pre    = 0U;
static uint32_t  actCarS3PreTick = 0U;
static uint8_t   actCarS3Resent = 0U;   /* 本步 S3 未到位时已重发过一次 */
/* ★【2026-10-24】"对位后预转 S3"的状态：登记后由 `Act_PreSlotService()` 按回读核对 + 补发 */
static int32_t   actPreSlotVal  = 0;
/* ★【2026-10-24 修"预转没动"】预选目标的有效 + 重试：
   `ArmAct_PreSelectSlot()` 只登记目标，`Act_PreSlotService()` 每拍核对 S3 回读，
   **没到位就每 300ms 补发一次**（最多 6 次 / 2.5s），到位才结束 ——
   这样"命令被调试板丢掉"也不会表现为"对位后舵盘完全没动"。 */
static uint8_t   actPreSlotOn    = 0U;   /* 1=预选目标有效（还没确认到位） */
static uint8_t   actPreSlotTries = 0U;   /* 已重发次数 */
static uint32_t  actPreSlotTick  = 0U;   /* 上一次下发时刻 */

/* 舵机步的"到位 / 顶住"判定状态（见 Act_ServoDone）。
   为什么要单独一套：舵机**没有到位帧**，只能看回读位置；
   以前"回读没到位就到时间直接放行"会把还没夹紧的爪子当成夹好了 —— 现象就是
   "物料还没抓到，手臂就抬起来了"。 */
/* ---- 舵机步"回读/重发"的内部状态 ----
   ★【2026-10-15】S1（爪子前伸/缩回）已改为"到点放行 + 后台校验"（`actSrvVerify[]`），不在这里；
   这组状态给 **S2（夹紧/松开）的"必须确认到位"判定**（`Act_ServoDoneGrip`）用 ——
   用户明确："**夹紧和松抓，还是保证松开和抓到再进行下一步**"。 */
static uint32_t  actSrvStepTick = 0U;   /* 上次判过的那一步的起点（变了 = 新的一步） */
static uint16_t  actSrvLastPos  = 0U;   /* 上一次回读到的位置 */
static uint32_t  actSrvStallTick = 0U;  /* 回读位置最后一次"变化"的时刻 */
static uint8_t   actSrvBlocked  = 0U;   /* 1=本步已按"顶住不动"放行过（日志限流用） */
/* ★【2026-10-23】夹紧确认后的"夹稳静置"（防止下一动作提前开始把料带空） */
static uint8_t   actGripSettleOn = 0U;   /* 1=已确认、正在静置 */
static uint32_t  actGripSettleTick = 0U; /* 静置起点 */
/* ★【2026-10-23】夹紧命令**下发前**的回读位置：判"确实动过"要和它比。
   动作时间 100ms 时，第一条"命令之后"的回读可能已经是最终位置，
   只比"相邻两帧不同"会误判成没动 -> 误重发/误中止。 */
static uint16_t  actGripPos0   = 0U;
static uint8_t   actGripPos0Ok = 0U;
static uint8_t   actSrvMoved    = 0U;   /* 1=本步回读出现过变化（= 命令确实生效 / 丢帧防护用） */
static uint8_t   actSrvRetx     = 0U;   /* 本步舵机命令已重发次数（丢帧防护，见 ACT_SRV_RETX_MAX） */
static uint8_t   actSrvSeen     = 0U;   /* 1=本步已收到过"命令之后"的第一条回读（作基准用） */
/* ★★【2026-10-24】"夹不到就张开重夹"的现场状态（见 bsp_arm_action.h 的 ACT_GRIP_RETRY_*） */
static uint8_t   actGripRetry   = 0U;   /* 本步夹紧已"张开->重夹"次数（换步/新动作清零） */
static uint8_t   actGripReopenOn = 0U;  /* 1=正在执行"张开"（重夹的准备阶段） */
static uint32_t  actGripReopenTick = 0U;/* "张开"开始时刻 */
static uint8_t   actGripFail    = 0U;   /* 1=夹紧彻底失败（调用方据此停下报 FAIL） */

/* 丢帧看门狗状态（见 bsp_arm_action.h 的 ACT_RETX_* 说明）。
   为什么单独一组：它**不改变完成判据**，只负责"命令丢了就再发一次"，
   保证"每一步都真的执行"，同时因为窗口只有 150ms 而不拖慢节奏。 */
static int32_t   actStepPos0 = ACT_POS_NONE; /* 本步命令下发前的位置（"有没有动"的基准 / 闭环停稳初值） */
#if (ACT_MOTOR_SW_LOOP != 0)
static uint8_t   actRetx     = 0U;           /* 本步电机命令已重发 / 补发 / 纠正次数 */

/* 电机关节的"软件闭环"状态（口径同 bsp_arm_seq，见 bsp_arm_action.h 的开关说明）：
   电机5 严格（跑完 -> 校验 -> 超差纠正）；电机6 容错（到位帧 / 位置容差 / 停住补发）。
   ★【2026-10-24】软件闭环已按用户要求关闭（`ACT_MOTOR_SW_LOOP=0`），
     这组状态只属于旧闭环路径 —— 开关为 0 时连同复位一起不编译，避免"设了没人读"。 */
static int32_t   actMotorLastPos      = 0;   /* 上一次看到的位置（判断"停稳"） */
static uint32_t  actMotorStallTick    = 0U;  /* 位置最后一次"变化"的时刻 */
static uint8_t   actMotorStallOn      = 0U;  /* 1=本步电机已动过（"动过又停住"才算停住补发） */
static uint8_t   actMotorReachIgnored = 0U;  /* 1=本步已忽略过一次"迟到到位帧"（日志限流） */
#endif

/* 前置声明（Act_ServoDone 的日志要用，定义在本文件靠后） */
static const char *Act_JointName(uint8_t joint);
/* 前置声明：抓取未确认时要中止本段（定义在本文件靠后） */
static void Act_AbortInternal(const char *why);

/* 【保障层】本次动作里是否出现过"补发用完仍未到目标"的步骤。
 * 为什么需要它：软件闭环只是**保障**（精度由驱动器自带的位置模式保证），
 * 保障层最重要的职责是"出事要说出来" —— 电机6 补发用完时为了不卡死整段流程，
 * 选择"告警后继续"，但如果最后还显示 DONE，操作者会以为姿态是对的（可能已经抓偏）。
 * 所以记下这个标志，动作结束时明确告警、页面状态也标出来。 */
static uint8_t   actUnreached = 0U;

/* 两段动作衔接（ArmAct_RunPair）
   0 = 单段（或空闲）；1 = 段A 运行中；2 = 段A 已完成、待启动段B；3 = 段B 运行中 */
static uint8_t actPairStage = 0U;

/* ★【规则2】并行下发标志：1 = 本步与下一步（**舵机1 + 电机5**）已经**一起下发**，
 *   必须"**两个都到位**"才允许前进，且前进时**一次跨两步**（被并行的那步不再单独下发）。
 *   见 Act_PairOk（判定能否并行）/ Act_Advance（跨步）。 */
static uint8_t actPairOn = 0U;

/* ★【2026-10-24】**并行组的步数**（1=没有并行；2=两步；3=三步，如"抬升+缩回+转向"）。
   `actPairOn=1` 时，本组 = `actSteps[actIdx .. actIdx+actParN-1]`（都在同一拍下发过）。
   见 `Act_PairOk`（能否并入下一组）/ `Act_Advance`（组内全部到位才跨步）。 */
static uint8_t actParN = 1U;
/* 并行组内**每一步**"命令下发前该轴的位置"（步号索引；到位帧可信判据用） */
static int32_t actParPos0[ACT_MAX_STEPS];

/* ★【2026-10-24】"转向等舵机1 收到 500 以下再发"的挂起状态：
   `actTurnPendIdx` = 0xFF 表示没有挂起；否则 = 待发的 TURN 步号。
   `actTurnPendTick` = 挂起时刻（兜底：`ACT_TURN_GATE_MS` 到点强发）。 */
#define ACT_TURN_PEND_NONE      0xFFU
static uint8_t  actTurnPendIdx  = ACT_TURN_PEND_NONE;
static uint32_t actTurnPendTick = 0U;

/* ★【2026-10-23】带 `ACT_STEP_S1_GATE` 的**当前**转向步"等 S1 < 500"的起始时刻
   （0 = 没在等）。与上面的 `actTurnPendIdx` 是两套：那套用于**并行组内**待发的 TURN，
   这套用于**单独一步**的 TURN（本组里 TURN 前隔着一步 CAR，组机制打不到）。 */
static uint32_t actS1GateTick = 0U;

/* ★【2026-10-28 修"抓空"】夹紧步"抓取闸门"的起始时刻（0 = 没在等）：
   夹紧命令下发前等舵机1（伸缩）后台校验收口，最多 `ACT_GRAB_GATE_MS`。 */
static uint32_t actGrabGateTick = 0U;

/* ★★【2026-10-29 修"转盘第 2 件转向时 S1 还没缩到位"】舵机1 最近一次**命令下发**时刻
   （0 = 本次动作还没给 S1 发过命令）。
   现场现象：转盘区放第 2/3 件时，`flowTtPrep` 先把 S1 伸到 898/924，紧接着
   `flowTrayPickPlace` 第 3 步命令 S1 -> 0；但**命令下发那一拍**舵机回读缓存还是
   "伸出之前"的旧值（如 244 < 500），`Act_S1BelowGate()` 误判"已缩到位"，于是
   电机5 与 S1 同拍下发 -> "电机5 已经转了、S1 还没缩回来"。
   本时刻专用于闸门：要求**回读必须晚于这条缩回命令**，否则一律按"还没缩到位"处理。 */
static uint32_t actClawCmdTick = 0U;

/* ★【2026-10-23】"错开启动"的待发步（`ACT_LATE_NONE` = 没有）：
   `ACT_STEP_LATE_START` 标出的那一步先不发，由 `Act_LateStartKick()` 到点发出。 */
#define ACT_LATE_NONE           0xFFU
static uint8_t  actLateIdx  = ACT_LATE_NONE;
static uint32_t actLateTick = 0U;

/* ★【2026-10-11 用户要求】"舵机1 缩回 + 电机5 转向"并行时，**先只发缩回、把转向挂起**：
 *   本标志 = 1 表示"转向已挂起、在等舵机1 的值回落到 ≤ ACT_PAIR_TURN_S1_HI（默认 20，即 0~20）"；
 *   放行条件与"一定兜底"见 Act_PairTurnKick（旧版 `actPairStartPos`/`ACT_PAIR_TURN_S1_MAX` 已删除）。 */
static uint8_t  actPairTurnPend = 0U;
/* ★【2026-10-21 用户要求】"观察区 -> 抓取姿态"（ArmAct_GotoTtPos）这一段**不要**在转向
   前插"舵机1 缩回"：用户现场要的动序是 **直接转向 -> 再伸出**。
   本标志 = 1 时，Act_EnsureS1RetBeforeTurn() 本次不插步（消费一次后自动清零）。 */
static uint8_t  actSkipRule3 = 0U;

/* ★【2026-10-21 用户要求】"最后下发的动作是**观察位**"标记：
   任务点出发前会提前摆臂到观察位（TASK_ARM_PRE_OBSERVE_MM），到场后就不该
   再重复执行"到观察位"。本标志由 ArmAct_GotoTtReady() 置 1，任何别的动作启动
   （Act_Begin 会先清 0）或中止（ArmAct_Abort）都会清掉。
   查询接口见 ArmAct_IsAtTtReady()。 */
static uint8_t  actAtTtReady = 0U;
static uint8_t actPendOp    = 0U;
static uint8_t actPendPos   = 0U;

/* ---- ★ 后台精修（"提前放行"配套，见 bsp_arm_action.h 的 ACT_RELEASE_TOL_MUL）----
   某轴被"提前放行"后，它的严格闭环不再阻塞流程，转为这里的**后台服务**：
   等该轴停稳 -> 校验 |当前−目标| > 严格容差就纠正，最多 ACT_TRIM_RETRY_MAX 次。
   下标 0 = 电机5（TURN），下标 1 = 电机6（LIFT）。 */
typedef struct {
    uint8_t  pending;     /* 1 = 该轴已被提前放行、等待精修到位 */
    int32_t  target;      /* 严格目标（下发脉冲域） */
    uint8_t  retry;       /* 已精修纠正次数 */
    int32_t  lastPos;     /* 上一次看到的位置（停稳判定用；ACT_POS_NONE = 还没读到过） */
    uint32_t stallTick;   /* 位置最后一次变化的时刻 */
    uint32_t startTick;   /* 登记时刻（★挂起上限 ACT_TRIM_MAX_MS 计时用） */
} ActTrim_t;

static ActTrim_t actTrim[2];

/**
 * @brief  电机地址 -> 后台精修表下标（0=电机5 / 1=电机6）
 */
static uint8_t Act_TrimIdx(uint8_t addr)
{
    return (addr == ARM_ADDR_TURN) ? 0U : 1U;
}

/**
 * @brief  取某轴的名字（精修日志用）
 */
static const char *Act_TrimName(uint8_t idx)
{
    return (idx == 0U) ? "TURN" : "LIFT";
}

/**
 * @brief  取消某轴的后台精修
 * @note   新的一步要直接驱动该轴时必须调用（由新步自己负责到位）。
 */
static void Act_TrimClear(uint8_t addr)
{
    actTrim[Act_TrimIdx(addr)].pending = 0U;
}

/**
 * @brief  把某轴交给后台精修（提前放行时调用）
 * @param  addr  : 电机地址
 * @param  target: 严格目标
 * @param  cur   : 放行那一刻的位置（停稳判定的初值）；**读不到位置时传 ACT_POS_NONE**，
 *                后台会在第一次读到位置时重建基准（不再"读不到就不登记"，否则闭环真丢了）
 * @param  now   : 当前时刻
 * @note   ★【2026-10-24】只有旧软件闭环路径会调用；`ACT_MOTOR_SW_LOOP=0` 时不编译。
 */
#if (ACT_MOTOR_SW_LOOP != 0)
static void Act_TrimRelease(uint8_t addr, int32_t target, int32_t cur, uint32_t now)
{
    ActTrim_t *t = &actTrim[Act_TrimIdx(addr)];

    t->pending   = 1U;
    t->target    = target;
    t->retry     = 0U;
    t->lastPos   = cur;
    t->stallTick = now;
    t->startTick = now;
}
#endif

/**
 * @brief  后台精修服务（每拍调用，**非阻塞**）
 * @param  now: 当前时刻
 * @retval 无
 * @note   只处理"已被提前放行"的轴；**当前步正在直接驱动的轴由该步自己管，这里不插手**。
 */
static void Act_TrimService(uint32_t now)
{
    uint8_t i;
    uint8_t anyPending = 0U;

    for (i = 0U; i < 2U; i++)
    {
        ActTrim_t *t    = &actTrim[i];
        uint8_t    addr = (i == 0U) ? (uint8_t)ARM_ADDR_TURN : (uint8_t)ARM_ADDR_LIFT;
        int32_t    tol  = (i == 0U) ? (int32_t)ARM_SEQ_TURN_POS_TOL
                                    : (int32_t)ARM_SEQ_LIFT_POS_TOL;
        int32_t    cur;
        int32_t    err;

        if (t->pending == 0U)
        {
            continue;
        }

        anyPending = 1U;

        /* ★【2026-10-15】挂起上限：长时间校正不完（读不到位置 / 电机掉线 / 一直纠不过）
           -> 放弃该轴 + 置 UNREACHED + 告警。**不留僵尸 pending**（否则会一直占着位置快查）。 */
#if (ACT_TRIM_MAX_MS > 0U)
        if ((now - t->startTick) > (uint32_t)ACT_TRIM_MAX_MS)
        {
            t->pending   = 0U;
            actUnreached = 1U;
            Debug_Log("ACT", "trim %s give up (%ums timeout) retry=%u",
                      Act_TrimName(i), (unsigned int)(now - t->startTick),
                      (unsigned int)t->retry);
            continue;
        }
#endif

        /* 当前步正在直接驱动这个轴：让该步自己管，后台不插手（避免两条命令打架） */
        if ((actState == ACT_RUN) && (actIdx < actStepCount))
        {
            uint8_t j = actSteps[actIdx].joint;

            if (((j == AJT_TURN) && (i == 0U)) || ((j == AJT_LIFT) && (i == 1U)))
            {
                continue;
            }
        }

        if (Arm_IsPosValid(addr) == 0U)
        {
            continue;                 /* 读不到位置：下一拍再试（到 ACT_TRIM_MAX_MS 会放弃） */
        }

        cur = Arm_GetPos(addr);
        err = cur - t->target;
        if (err < 0)
        {
            err = -err;
        }
        if (err <= tol)
        {
            /* ★已进严格容差：后台闭环收口（真纠过才打日志，避免刷屏） */
            if (t->retry > 0U)
            {
                Debug_Log("ACT", "trim %s ok cur=%d tgt=%d tol=%d (fixed x%u)",
                          Act_TrimName(i), (int)cur, (long)t->target,
                          (int)tol, (unsigned int)t->retry);
            }
            t->pending = 0U;
            continue;
        }

        /* 停稳判定：位置连续 ARM_SEQ_MOVE_STALL_MS 没变才纠正（别在电机还在跑时插命令） */
        if (cur != t->lastPos)
        {
            t->lastPos   = cur;
            t->stallTick = now;
            continue;
        }
        if ((now - t->stallTick) < (uint32_t)ARM_SEQ_MOVE_STALL_MS)
        {
            continue;
        }

        if (t->retry >= (uint8_t)ACT_TRIM_RETRY_MAX)
        {
            /* 纠不过来：放弃后台精修（保留误差，打日志 + 置 UNREACHED 供页面/结束时告警） */
            t->pending   = 0U;
            actUnreached = 1U;
            Debug_Log("ACT", "trim %s give up cur=%d tgt=%d",
                      Act_TrimName(i), (int)cur, (long)t->target);
            continue;
        }

        t->retry++;
        t->stallTick = now;
        Debug_Log("ACT", "trim %s fix%u cur=%d tgt=%d",
                  Act_TrimName(i), (unsigned int)t->retry, (int)cur, (long)t->target);
        Arm_ReachClear(addr);
        Arm_MoveAbsDir(addr, t->target,
                       (i == 0U) ? (uint16_t)ARM_TURN_VEL : (uint16_t)ARM_LIFT_VEL,
                       (i == 0U) ? (uint8_t)ARM_TURN_ACC : (uint8_t)ARM_LIFT_ACC,
                       (uint8_t)ARM_DIR_AUTO);
    }

    /* 有待精修的轴、且当前没有电机步在跑 -> 给它开"位置快查"，
       否则位置 120ms 才刷新一次，停稳判定会明显迟钝。有活动电机步时快查归该步管。 */
    if (anyPending != 0U)
    {
        uint8_t activeMotor = 0U;

        if ((actState == ACT_RUN) && (actIdx < actStepCount) &&
            ((actSteps[actIdx].joint == AJT_TURN) || (actSteps[actIdx].joint == AJT_LIFT)))
        {
            activeMotor = 1U;
        }
        if (activeMotor == 0U)
        {
            Arm_SetFastPoll((actTrim[0].pending != 0U) ? (uint8_t)ARM_ADDR_TURN
                                                       : (uint8_t)ARM_ADDR_LIFT);
        }
    }
}

/* ============================================================================
 *            ★【2026-10-15 用户要求】后台"**舵机校验 / 补发**"
 *   背景：用户要求"**抓放优先速度，精度都在下一个关节运行的时候闭环**"。
 *   事实：舵机运动时间 `SERVO_MOVE_TIME_MS` = 1ms，物理上瞬间到位；
 *         流程里原来等的 100~400ms（还要乘重发次数）**只是在等回读确认**，不是物理需要。
 *   做法：
 *     · `Act_ServoDone` 最快路径仍"回读进容差立刻放行"；否则最多等
 *       `ACT_SRV_FAST_MS`(120ms) 就放行，让下一个关节马上动；
 *     · 本表在**后台**继续核对：没到目标 -> **重发**（幂等，走 `Arm_ServoMove` 的总线仲裁）；
 *       进了 `ACT_SERVO_LOOSE_TOL` 就收口；超过 `ACT_SRV_TRIM_MAX_MS` 才放弃并记 UNREACHED。
 *   新命令打向同一舵机时**覆盖**旧表项（通过 `Act_SendStep` -> `Act_SrvVerifyArm`），
 *   所以后台不会把舵机往旧目标拽。
 *   下标 0 = S1(CLAW)，1 = S2(GRIP)。
 * ==========================================================================*/
#if (ACT_SERVO_OPEN_LOOP != 0U)
typedef struct {
    uint8_t  pending;     /* 1 = 已下发新目标、等后台核对 */
    uint16_t target;      /* 目标（舵机位置域 0~1000） */
    uint16_t lastPos;     /* 上次回读（停住判定） */
    uint8_t  seen;        /* 1 = 已收到过"本次下发之后"的第一条回读 */
    uint8_t  retry;       /* 已重发次数 */
    uint32_t startTick;   /* 本次下发时刻（挂起上限 / 重发窗口计时） */
    uint32_t stallTick;   /* 回读位置最后一次"变化"的时刻 */
} ActSrvVerify_t;

static ActSrvVerify_t actSrvVerify[2];

static uint8_t Act_SrvVerifyIdx(uint8_t id)
{
    return (id == (uint8_t)SERVO_ID_CLAW) ? 0U : 1U;
}

/**
 * @brief  登记"后台舵机校验"（每次下发舵机命令时调用）
 */
static void Act_SrvVerifyArm(uint8_t id, uint16_t target, uint32_t now)
{
    ActSrvVerify_t *v = &actSrvVerify[Act_SrvVerifyIdx(id)];

    v->pending   = 1U;
    v->target    = target;
    v->lastPos   = 0U;
    v->seen      = 0U;
    v->retry     = 0U;
    v->startTick = now;
    v->stallTick = now;
}

/**
 * @brief  取消某舵机的后台校验（已确认到位 / 外部接管时调用）
 */
static void Act_SrvVerifyClear(uint8_t id)
{
    actSrvVerify[Act_SrvVerifyIdx(id)].pending = 0U;
}

/**
 * @brief  后台舵机校验服务（每拍调用，**非阻塞**）
 * @note   只在下发后的"回读窗口"里工作；重发是幂等的（绝对目标）。
 */
static void Act_SrvVerifyService(uint32_t now)
{
    uint8_t i;

    for (i = 0U; i < 2U; i++)
    {
        ActSrvVerify_t *v = &actSrvVerify[i];
        uint8_t  id  = (i == 0U) ? (uint8_t)SERVO_ID_CLAW : (uint8_t)SERVO_ID_GRIP;
        uint16_t cur = 0U;
        int32_t  d;

        if (v->pending == 0U)
        {
            continue;
        }

        /* 挂起上限：一直确认不了（读不到 / 舵机堵转）-> 放弃并告警，**不留僵尸 pending** */
#if (ACT_SRV_TRIM_MAX_MS > 0U)
        if ((now - v->startTick) > (uint32_t)ACT_SRV_TRIM_MAX_MS)
        {
            v->pending   = 0U;
            actUnreached = 1U;
            Debug_Log("ACT", "srv verify %s give up (%ums) retry=%u",
                      (i == 0U) ? "S1" : "S2",
                      (unsigned int)(now - v->startTick), (unsigned int)v->retry);
            continue;
        }
#endif

        if (Servo_GetPos(id, &cur) == 0U)
        {
            continue;                 /* 回读暂时无效：下一拍再试 */
        }
        if ((int32_t)(Servo_GetPosTickById(id) - v->startTick) < 0)
        {
            continue;                 /* 只认"本次下发之后"**本 ID** 的回读 */
        }

        d = (int32_t)cur - (int32_t)v->target;
        if (d < 0)
        {
            d = -d;
        }
        if (d <= (int32_t)ACT_SERVO_LOOSE_TOL)
        {
            if (v->retry > 0U)
            {
                Debug_Log("ACT", "srv verify %s ok pos=%u tgt=%u (resent x%u)",
                          (i == 0U) ? "S1" : "S2", (unsigned int)cur,
                          (unsigned int)v->target, (unsigned int)v->retry);
            }
            v->pending = 0U;
            continue;
        }

        /* 位置还在变 -> 命令确实送到了，让它继续走完（不要中途插命令）
           ★【2026-10-27】用 `ACT_SRV_STALL_EPS` 死区，不能精确比较：回读 ±1 抖动
              会一直刷新 stallTick -> 永远判不出"停住" -> 不补发 -> UNREACHED。 */
        if (v->seen == 0U)
        {
            v->seen      = 1U;
            v->lastPos   = cur;
            v->stallTick = now;
            continue;
        }
        d = (int32_t)cur - (int32_t)v->lastPos;
        if (d < 0)
        {
            d = -d;
        }
        if (d > (int32_t)ACT_SRV_STALL_EPS)
        {
            v->lastPos   = cur;
            v->stallTick = now;
            continue;
        }

        /* 停住但没到目标 -> 命令多半丢了 -> **重发同一条**（绝对目标，幂等） */
        if ((now - v->stallTick) < (uint32_t)ACT_SRV_STALL_MS)
        {
            continue;
        }

        if (v->retry >= (uint8_t)ACT_SRV_RETX_MAX)
        {
            v->pending   = 0U;
            actUnreached = 1U;
            Debug_Log("ACT", "srv verify %s give up (retx) pos=%u tgt=%u",
                      (i == 0U) ? "S1" : "S2", (unsigned int)cur,
                      (unsigned int)v->target);
            continue;
        }

        /* ★【2026-10-24】后台重发也**必须等总线静默**：它每拍都会跑，
           若正赶上有读应答在途，重发会和应答撞车被丢（"重发也丢步"的来源之一）。
           总线忙就下一拍再发，不消耗重试次数。 */
        if (Servo_BusQuiet((uint16_t)ACT_SRV_BUS_QUIET_MS) == 0U)
        {
            continue;
        }

        v->retry++;
        v->startTick = now;
        v->stallTick = now;
        v->seen      = 0U;
        v->lastPos   = cur;
        Arm_ServoMove(id, v->target);   /* 内部含总线仲裁（yield + notify） */
        Debug_Log("ACT", "srv verify %s resend %u/%u pos=%u tgt=%u",
                  (i == 0U) ? "S1" : "S2", (unsigned int)v->retry,
                  (unsigned int)ACT_SRV_RETX_MAX, (unsigned int)cur,
                  (unsigned int)v->target);
    }
}
#endif   /* ACT_SERVO_OPEN_LOOP != 0U */

/**
 * @brief  查标定表某一行
 * @param  op : 动作
 * @param  pos: 位置号
 * @retval 该行指针；找不到返回 NULL
 */
static const ArmCalRow_t *Act_FindCal(uint8_t op, uint8_t pos)
{
    uint8_t i;

    for (i = 0U; i < ACT_CAL_COUNT; i++)
    {
        if ((actCalTable[i].op == op) && (actCalTable[i].pos == pos))
        {
            return &actCalTable[i];
        }
    }
    return NULL;
}

/**
 * @brief  判断该行是否四项都已标定
 * @param  row: 标定行
 * @retval 1=已标定；0=还有 ARM_UNSET
 */
static uint8_t Act_IsCalibrated(const ArmCalRow_t *row)
{
    if (row == NULL) { return 0U; }

    /* ★【2026-10-22 用户要求】车载抓/放的手臂姿态已统一到 ARM_CAR_*，
       这两组的三行**只需要 S3 已标定**（M5/M6/S1/S2 填 ARM_UNSET 也允许）。 */
    if ((row->op == (uint8_t)ARM_OP_CAR_PLACE) || (row->op == (uint8_t)ARM_OP_CAR_PICK))
    {
        return (row->cal.s3 != ARM_UNSET) ? 1U : 0U;
    }

    return ((row->cal.m5 != ARM_UNSET) && (row->cal.m6 != ARM_UNSET) &&
            (row->cal.s1 != ARM_UNSET) && (row->cal.s2 != ARM_UNSET)) ? 1U : 0U;
}

/**
 * @brief  舵机参数钳位到合法范围
 * @param  p: 表里写的参数
 * @retval 0~ARM_SERVO_POS_MAX
 */
static uint16_t Act_ServoParam(int32_t p)
{
    if (p < (int32_t)ARM_SERVO_POS_MIN) { return (uint16_t)ARM_SERVO_POS_MIN; }
    if (p > (int32_t)ARM_SERVO_POS_MAX) { return (uint16_t)ARM_SERVO_POS_MAX; }
    return (uint16_t)p;
}

/**
 * @brief  S1（爪子伸缩）回读是否已经 **小于 `ACT_TURN_S1_GATE`(500)** 且**可信**
 * @param  无
 * @retval 1=已小于门限（可以转向）；0=还伸着 / 读不到 / **回读是命令之前的老值**
 * @note   ★【2026-10-24 用户要求】"严格要求：**舵机1 未小于 500，电机5 不能转向**"。
 *         读不到时按"还伸着"处理（宁可不转），由 `Act_TurnGateKick` 的兜底超时处理，
 *         避免因为一个回读故障把整段动作永久卡住。
 *         ★★【2026-10-29 修"转盘第 2 件转向时 S1 还没缩到位"】增加**回读新鲜度**判据：
 *         本次动作已经给 S1 发过命令（`actClawCmdTick != 0`）时，要求该 ID 的最后一次
 *         回读**晚于**这条命令。否则读到的是"伸出之前"的老值（< 500），会把
 *         "还没缩回"误判成"已缩到位"，导致 M5 与 S1 同拍下发。
 *         · 判"命令之后有没有新回读"必须用 `Servo_GetPosTickById()`（全局时刻会被
 *           别的舵机刷新成新鲜）。
 */
static uint8_t Act_S1BelowGate(void)
{
    uint16_t cur = 0U;

    if (Servo_GetPos((uint8_t)SERVO_ID_CLAW, &cur) == 0U) { return 0U; }

    /* 本动作给 S1 发过命令 -> 回读必须晚于那条命令，否则是陈旧值，不能作为放行依据 */
    if ((actClawCmdTick != 0U) &&
        ((int32_t)(Servo_GetPosTickById((uint8_t)SERVO_ID_CLAW) - actClawCmdTick) < 0))
    {
        return 0U;
    }
    return (cur < (uint16_t)ACT_TURN_S1_GATE) ? 1U : 0U;
}

/**
 * @brief  该车载号位是否已在"对位成功"时预选过、且已转过 ACT_S3_MOVE_MS
 * @param  无
 * @retval 1=已在该号位（S3 不用再发/不用再等）；0=没有
 * @note   预选入口是 `ArmAct_PreSelectSlot()`（`TT_ALIGN` 对位成功时由 bsp_tt 调用）。
 *         这样 S3 的换位与手臂后续动作**并行**，不再在 CAR_PLACE 开头白等 700ms。
 */
static uint8_t Act_CarS3AlreadyDone(void)
{
    if (actPos == 0U)                                                   { return 0U; }
    if (actCarS3Pre != actPos)                                          { return 0U; }
    if ((HAL_GetTick() - actCarS3PreTick) < (uint32_t)ACT_S3_MOVE_MS)   { return 0U; }
    return 1U;
}

/**
 * @brief  S3（舵盘）回读是否已经在某个目标角度附近
 * @param  tgt: 目标角度 0~1000
 * @retval 1=已到位；0=没到 / 读不到
 */
static uint8_t Act_S3At(int32_t tgt)
{
    uint16_t cur = 0U;
    int32_t  d;

    if (Servo_GetPos((uint8_t)SERVO_ID_TURNTABLE, &cur) == 0U) { return 0U; }

    d = (int32_t)cur - tgt;
    if (d < 0) { d = -d; }
    return (d <= (int32_t)ACT_S3_ARRIVE_TOL) ? 1U : 0U;
}

/**
 * @brief  取某个"取值来源"对应的实际数值
 * @param  src: ActValSrc_t
 * @param  cal: 该行的标定值
 * @retval 实际参数值
 */
static int32_t Act_Resolve(uint8_t src, const ArmCal_t *cal)
{
    switch (src)
    {
        case AS_M5: return cal->m5;
        case AS_M6: return cal->m6;
        case AS_S1: return cal->s1;
        case AS_S2: return cal->s2;
        case AS_S3: return cal->s3;      /* ★车载转盘（舵机3） */
        case AS_CAR_M5: return ARM_CAR_M5;   /* ★车载统一姿态（与位置无关） */
        case AS_CAR_M6: return ARM_CAR_M6;
        case AS_CAR_PLACE_M6: return ARM_CAR_PLACE_M6;   /* ★放料高度（比抓取高 2000） */
        case AS_CAR_S1: return ARM_CAR_S1;
        case AS_TT_READY_M5: return ARM_TT_READY_M5;   /* ★观察位（TRAY PICK 用） */
        case AS_TT_READY_M6: return ARM_TT_READY_M6;
        case AS_TT_READY_S1: return ARM_TT_READY_S1;
        case AS_FIX_S2_OPEN: return ARM_FIX_S2_OPEN;
        case AS_FIX_S2_CLOSE: return ARM_FIX_S2_CLOSE;
        case AS_FIX_S3_HOME:  return ARM_CAR_S3_HOME;    /* ★舵盘回 0 点（抓完料让位） */
        case AS_FIX_S1_RET:  return ARM_FIX_S1_RET;
        case AS_FIX_SB_SAFE:   return ARM_SB_M6_SAFE;    /* ★STORE BACK 专用安全高(-6576) */
        default:             return ARM_FIX_M6_SAFE;
    }
}

/**
 * @brief  取某步的实际参数（在 Act_Resolve 之上再处理"车载转盘"）
 * @param  src: ActValSrc_t
 * @param  cal: 该动作行的标定值（车载动作传全 0 的占位即可）
 * @param  pos: 位置号/车载号位 1~3
 * @retval 实际参数
 * @note   ★【2026-10-23】`AS_S3` 不再来自动作表，而是按**号位**取 `actCarS3[]`
 *         （号位由"颜色↔号位"绑定得到），所以任何动作表都不用再填 S3。
 */
static int32_t Act_SrcValue(uint8_t src, const ArmCal_t *cal, uint8_t pos)
{
    if (src == (uint8_t)AS_S3)
    {
        return actCarS3[(pos >= 1U) && (pos <= 3U) ? (pos - 1U) : 0U];
    }
    return Act_Resolve(src, cal);
}

/**
 * @brief  把"流程模板 + 标定值"展开成实际步骤数组
 * @param  op : 动作
 * @param  pos: 位置号
 * @retval 1=展开成功；0=动作非法 / 标定行缺失 / 未标定
 */
static uint8_t Act_Build(ArmOp_t op, uint8_t pos)
{
    const ActOpInfo_t *info;
    const ArmCalRow_t *row;
    ArmCal_t calNone = { 0, 0, 0, 0, 0 };
    uint8_t i;

    if ((uint8_t)op >= (uint8_t)ARM_OP_COUNT) { return 0U; }
    if ((pos < 1U) || (pos > 3U))             { return 0U; }

    /* ★车载号位（用于"舵机3 已在对位成功时预选过"->跳过 S3 等待）；非车载动作记 0 */
    actPos = (((uint8_t)op == (uint8_t)ARM_OP_CAR_PLACE) ||
              ((uint8_t)op == (uint8_t)ARM_OP_CAR_PICK)) ? pos : 0U;

    info = &actOps[(uint8_t)op];
    if (info->count > ACT_MAX_STEPS)          { return 0U; }

    /* ★【2026-10-23 用户要求】车载（放/取）**不需要标定行**：
       姿态来自 ARM_CAR_*，转盘角度按"号位"从 actCarS3[] 取（号位由颜色绑定得到）。 */
    if (((uint8_t)op == (uint8_t)ARM_OP_CAR_PLACE) || ((uint8_t)op == (uint8_t)ARM_OP_CAR_PICK))
    {
        for (i = 0U; i < info->count; i++)
        {
            actSteps[i].joint = info->flow[i].joint;
            actSteps[i].flags = info->flow[i].flags;
            actSteps[i].param = Act_SrcValue(info->flow[i].src, &calNone, pos);
        }
        actStepCount = info->count;
        (void)snprintf(actTitle, sizeof(actTitle), "%s%u", info->name, (unsigned int)pos);
        return 1U;
    }

    row = Act_FindCal((uint8_t)op, pos);
    if (Act_IsCalibrated(row) == 0U)
    {
        /* 未标定：**拒绝执行**（否则会按 0 位乱动，可能撞机构） */
        Debug_Log("ACT", "%s%u NOT calibrated -> refused",
                  info->name, (unsigned int)pos);
        return 0U;
    }

    for (i = 0U; i < info->count; i++)
    {
        actSteps[i].joint = info->flow[i].joint;
        actSteps[i].flags = info->flow[i].flags;
        actSteps[i].param = Act_SrcValue(info->flow[i].src, &row->cal, pos);
    }
    actStepCount = info->count;

    (void)snprintf(actTitle, sizeof(actTitle), "%s%u", info->name, (unsigned int)pos);
    return 1U;
}

/**
 * @brief  下发一步命令
 * @param  st: 步骤
 * @retval 无
 * @note   电机步：先清到位标志再下发（避免上一步的迟到应答把本步立刻判成完成）。
 */
static void Act_SendStep(const ActStep_t *st)
{
    switch (st->joint)
    {
        case AJT_TURN:
            /* ★【2026-10-15】本步要**直接驱动该轴** -> 取消它残留的"后台精修"
               （由本步自己负责到位；否则后台会朝旧目标纠、和新目标打架）。
               放在这里而不是"动作开始时"：这是**唯一**的电机命令下发口
               （主分支 / 并行对 / 两段一起发 三个路径都经过本函数），
               覆盖最全；而且精修能一直工作到**真正下发**那一刻为止（多赚一点校正时间）。 */
            Act_TrimClear(ARM_ADDR_TURN);
            Arm_ReachClear(ARM_ADDR_TURN);
            Arm_MoveAbsDir(ARM_ADDR_TURN, st->param,
                           ARM_TURN_VEL, ARM_TURN_ACC, ARM_DIR_AUTO);
            /* 打开"位置快查"：位置回读周期由 6 拍轮转(120ms) 降到 20ms，
               闭环才能在电机停下后几十毫秒内判出"停住/误差"（与 bsp_arm_seq 同一机制）。 */
            Arm_SetFastPoll(ARM_ADDR_TURN);
            break;

        case AJT_LIFT:
            Act_TrimClear(ARM_ADDR_LIFT);   /* 同上：本步直接驱动电机6 */
            Arm_ReachClear(ARM_ADDR_LIFT);
            Arm_MoveAbsDir(ARM_ADDR_LIFT, st->param,
                           ARM_LIFT_VEL, ARM_LIFT_ACC, ARM_DIR_AUTO);
            Arm_SetFastPoll(ARM_ADDR_LIFT);
            break;

        case AJT_CLAW:
        {
            uint16_t tgt = Act_ServoParam(st->param);

            /* ★【2026-10-29】记录本命令下发时刻：`Act_S1BelowGate()` 用它要求
               "回读必须晚于这条命令"，避免拿伸出之前的旧回读提前放行转向。 */
            actClawCmdTick = HAL_GetTick();
            Arm_ServoMove(SERVO_ID_CLAW, tgt);
#if (ACT_SERVO_OPEN_LOOP != 0U)
            /* ★【2026-10-15】登记"后台舵机校验/补发"：本步放行后由它继续确认到位，
               没到就重发 —— **精度不在流程里等**（用户要求"精度下一个关节运行时闭环"）。 */
            Act_SrvVerifyArm((uint8_t)SERVO_ID_CLAW, tgt, HAL_GetTick());
#endif
            break;
        }

        case AJT_CAR:
        {
            /* ★【2026-10-21 硬件改动】车载转盘（舵机3）：**开环**执行，
               不做回读校验/补发（本模块的回读表只覆盖 S1/S2），按 ACT_S3_MOVE_MS 定时放行。 */
            /* ★★【2026-10-24 用户要求】"对位后预转可以，但**预转完到放料结束之前不能再转**"：
               如果本步角度正是对位时预选的那个号位、而且回读确认**已经在位** ->
               **不再下发**，直接让完成判定放行（不会再产生任何可见转动）。
               只有"没到位（预选那条被挤掉/还在路上）"时才补发，属于必要纠正。 */
            if ((actCarS3Pre != 0U) &&
                (st->param == actCarS3[(uint8_t)(actCarS3Pre - 1U)]) &&
                (Act_S3At(st->param) != 0U))
            {
                Debug_Log("ACT", "step%u S3=%ld already in place (preselected) -> no resend",
                          (unsigned int)(actIdx + 1U), (long)st->param);
                break;
            }

            /* 目标不是"已预选的那个号位角度"（例如放靶位前回 0 点）-> 预选状态作废，
               后面再放/取同一号位时必须重新等到位。 */
            if ((actCarS3Pre != 0U) &&
                (st->param != actCarS3[(uint8_t)(actCarS3Pre - 1U)]))
            {
                actCarS3Pre = 0U;
            }
            Arm_ServoMove(SERVO_ID_TURNTABLE, Act_ServoParam(st->param));
            break;
        }

        default:   /* AJT_GRIP */
        {
            uint16_t tgt = Act_ServoParam(st->param);
            uint16_t p0  = 0U;

            /* ★记录"下发前"的回读：抓取确认时用它判断"确实动过"
               （动作时间只有 100ms，第一帧回读就可能已是终点） */
            actGripPos0Ok = Servo_GetPos((uint8_t)SERVO_ID_GRIP, &p0);
            actGripPos0   = p0;

            /* ★【2026-10-15 用户要求】S2（夹紧/松开）**必须确认到位再走下一步**
               -> 由 `Act_ServoDoneGrip()` 在流程里确认，**不登记后台校验**
               （否则后台会在"被物料挡住、位置没到目标"时误判为没到位而反复重发）。 */
            Arm_ServoMove(SERVO_ID_GRIP, tgt);
            break;
        }
    }
}

/**
 * @brief  夹紧确认后的"夹稳静置"（★2026-10-23 用户要求）
 * @param  now: 当前时刻
 * @retval 1=静置够了，可以走下一步；0=还在静置
 * @note   现场现象："S2 已经在夹，但**下一个动作提前开始**，把料带空/抓空"。
 *         所以在"确认夹到位 / 顶住"之后再压 `ACT_GRIP_SETTLE_MS`(150ms)，
 *         保证爪子真正夹稳了才允许抬臂或转向。
 */
static uint8_t Act_GripSettleDone(uint32_t now)
{
    if (actGripSettleOn == 0U)
    {
        actGripSettleOn   = 1U;
        actGripSettleTick = now;
        return 0U;
    }
    return ((now - actGripSettleTick) >= (uint32_t)ACT_GRIP_SETTLE_MS) ? 1U : 0U;
}

/**
 * @brief  ★★【2026-10-24】夹紧没成功时的统一处理：**先张开重夹（可小幅下降找料）**，用尽才判失败
 * @param  now : 当前时刻
 * @param  why : 失败原因（日志用）
 * @param  cur : 当前 S2 回读（日志用）
 * @retval 1=已开始重夹（本拍继续等）；0=重夹用尽（已置 `actGripFail` 并中止本段）
 * @note   现场：爪子没夹到料，手臂就上抬 —— 除了"判据太快"，还必须有**补救**：
 *         张开 -> 等它真张开 -> （M6 小幅下降找料）-> 重新夹紧。
 *         用户明确："失败就停下报 FAIL，绝不动别的关节"，但**允许重夹时小幅下降**。
 */
static uint8_t Act_GripRetryOrFail(uint32_t now, const char *why, uint16_t cur)
{
    int32_t lift;

    if (actGripRetry < (uint8_t)ACT_GRIP_RETRY_MAX)
    {
        actGripRetry++;
        actGripReopenOn   = 1U;
        actGripReopenTick = now;
        actGripSettleOn   = 0U;

        /* ① 爪子张开：重夹前必须先松口，否则等于"没放手又夹一次" */
        Arm_ServoMove((uint8_t)SERVO_ID_GRIP, Act_ServoParam((int32_t)ARM_FIX_S2_OPEN));

        /* ② 电机6 小幅**下降**找料（脉冲更负 = 更低） */
        if (Arm_IsPosValid((uint8_t)ARM_ADDR_LIFT) != 0U)
        {
            lift = Arm_GetPos((uint8_t)ARM_ADDR_LIFT) - (int32_t)ACT_GRIP_RETRY_DROP;

            Arm_ReachClear((uint8_t)ARM_ADDR_LIFT);
            Arm_MoveAbsDir((uint8_t)ARM_ADDR_LIFT, lift,
                           ARM_LIFT_VEL, ARM_LIFT_ACC, ARM_DIR_AUTO);
            Arm_SetFastPoll((uint8_t)ARM_ADDR_LIFT);
        }
        Debug_Log("ACT", "step%u S2 %s (pos=%u) -> REOPEN + M6 drop %d, retry %u/%u",
                  (unsigned int)(actIdx + 1U), why, (unsigned int)cur,
                  (int)ACT_GRIP_RETRY_DROP,
                  (unsigned int)actGripRetry, (unsigned int)ACT_GRIP_RETRY_MAX);
        return 1U;
    }

    /* 重夹用尽：**如实判失败**，绝不假装抓到了继续抬臂 */
    actGripFail  = 1U;
    actUnreached = 1U;
    Debug_Log("ACT", "step%u S2 GRIP FAIL %s (pos=%u) after %u retries -> STOP",
              (unsigned int)(actIdx + 1U), why, (unsigned int)cur,
              (unsigned int)ACT_GRIP_RETRY_MAX);
    Act_AbortInternal("grip failed");
    return 0U;
}

/**
 * @brief  ★【2026-10-15 用户要求】**S2（夹紧/松开）必须确认到位才允许下一步**
 * @param  st : 当前步骤（AJT_GRIP）
 * @param  now: 当前时刻
 * @retval 1=确认完成；0=继续等（若抓取确认彻底失败会 `Act_AbortInternal`，动作直接中止）
 * @note   ★【2026-10-23 用户要求】"**抓取物料时一定要确保抓到了再走下一步**"：
 *           ① **夹紧步**：回读 ≤ `ACT_GRIP_CLOSED_POS`(520，硬件限幅：夹到底 ≈500)
 *              + **停稳 `ACT_GRIP_STALL_MS`** -> 再区分"夹到料 / 空抓"；
 *              **松开步**：回读与目标差 ≤ `ACT_SERVO_LOOSE_TOL` -> 完成；
 *           ② 回读**动过**、之后停住 `ACT_GRIP_STALL_MS`(500ms) 仍未到目标
 *              -> 被物料挡住 = **确实夹到东西了** -> 完成
 *              （★**还要求停住位置 ≤ `ACT_GRIP_GRAB_MAX`**：停在半路不算夹到，
 *                这条是 2026-10-24"爪子没夹到料就抬臂"的直接修正）；
 *           ③ **一次都没动过 / 读不到回读** -> 判定命令丢了 -> 重发
 *              （夹紧步最多 `ACT_GRIP_RETX_MAX`(3) 次，松开步 1 次）；
 *           ④ **夹紧步**空抓 / 超时 / 用尽 -> 先**张开重夹**（`Act_GripRetryOrFail`，
 *              允许 M6 小幅下降找料），重夹用尽 -> 置 `actGripFail` + **中止本段**；
 *              **松开步**用尽 -> 放行（松手不会因为被挡住而失败）。
 *         ⚠ 旧版把"停住但**一次都没动过**"也当成"被物料挡住 -> 完成"，
 *           于是命令丢了照样放行 —— 现场就是"没抓到就抬臂/转走"。本次已修。
 */
static uint8_t Act_ServoDoneGrip(const ActStep_t *st, uint32_t now)
{
    uint8_t  id  = (uint8_t)SERVO_ID_GRIP;
    uint16_t cur = 0U;
    int32_t  d;
    uint8_t  havePos = 0U;
    uint8_t  isClose;
    uint8_t  retxMax;

    isClose = (st->param < (int32_t)ACT_S2_OPEN_MIN) ? 1U : 0U;
    retxMax = (isClose != 0U) ? (uint8_t)ACT_GRIP_RETX_MAX : 1U;

    /* 换了一步：清本步的回读/重发/重夹状态 */
    if (actSrvStepTick != actStepTick)
    {
        actSrvStepTick  = actStepTick;
        actSrvSeen      = 0U;
        actSrvMoved     = 0U;
        actSrvRetx      = 0U;
        actSrvLastPos   = 0U;
        actSrvStallTick = now;
        actSrvBlocked   = 0U;
        actGripSettleOn   = 0U;         /* 新的一步：夹稳静置重新计时 */
        actGripSettleTick = now;
        actGripRetry      = 0U;         /* 新的一步：重夹次数重新计 */
        actGripReopenOn   = 0U;
    }

    /* ★★【2026-10-24】"张开 -> 再夹"的准备阶段：等爪子**真的张开**（或等够时间）再发夹紧 */
    if ((isClose != 0U) && (actGripReopenOn != 0U))
    {
        uint16_t p      = 0U;
        uint8_t  openOk = 0U;

        if ((Servo_GetPos(id, &p) != 0U) && (p >= (uint16_t)ACT_GRIP_REOPEN_DONE_POS))
        {
            openOk = 1U;
        }
        if ((openOk == 0U) && ((now - actGripReopenTick) < (uint32_t)ACT_GRIP_REOPEN_MS))
        {
            return 0U;                  /* 还在张开 */
        }

        actGripReopenOn = 0U;
        actStepTick     = now;          /* 重新计时：本步的超时/重发窗口都从这里算 */
        actSrvStepTick  = actStepTick;  /* 同步，下一拍不会再把状态清一遍 */
        actSrvSeen      = 0U;
        actSrvMoved     = 0U;
        actSrvRetx      = 0U;
        actSrvLastPos   = 0U;
        actSrvStallTick = now;
        actSrvBlocked   = 0U;
        actGripSettleOn = 0U;
        actGripPos0Ok   = Servo_GetPos(id, &p);
        actGripPos0     = p;
        Arm_ServoMove(id, Act_ServoParam(st->param));
        Debug_Log("ACT", "step%u S2 re-close (open=%u, retry %u/%u)",
                  (unsigned int)(actIdx + 1U), (unsigned int)p,
                  (unsigned int)actGripRetry, (unsigned int)ACT_GRIP_RETRY_MAX);
        return 0U;
    }

    if (Servo_GetPos(id, &cur) != 0U)
    {
        /* 只认"本次命令下发之后"**本 ID** 拿到的回读：否则会用命令之前的旧值提前放行 */
        if ((int32_t)(Servo_GetPosTickById(id) - actStepTick) >= 0)
        {
            havePos = 1U;

            /* ★"命令确实被执行"：回读与**下发前**（actGripPos0）比，差值够大就算动过。
               不能只比"相邻两帧是否不同"——动作时间 100ms 时第一帧就可能已是终点。 */
            if ((actGripPos0Ok != 0U) && (actSrvMoved == 0U))
            {
                int32_t dv = (int32_t)cur - (int32_t)actGripPos0;

                if (dv < 0)
                {
                    dv = -dv;
                }
                if (dv >= (int32_t)ACT_GRIP_MOVED_EPS)
                {
                    actSrvMoved = 1U;
                }
            }

            /* ① 已到位：**夹紧步按"回读 ≤ ACT_GRIP_CLOSED_POS"判**（硬件限幅：夹到底 ≈500，
                  永远到不了 0）；松开步按通用宽容差判。 */
            d = (int32_t)cur - st->param;
            if (d < 0)
            {
                d = -d;
            }
            if (isClose != 0U)
            {
                if (cur <= (uint16_t)ACT_GRIP_CLOSED_POS)
                {
                    /* ★【2026-10-24 用户要求】"优先保证抓住物料，再进行别的动作"：
                       进阈值区 ≠ 夹稳——必须先等**停稳**（防止"爪子还在往下夹、回读
                       经过 520 的途中帧"就放行抬臂），停稳后再区分"夹到料 / 空抓"：
                       · 停稳后 cur ≤ ACT_GRIP_EMPTY_POS = 闭到机械限位、中间没东西挡
                         = **空抓** -> 张开重夹（用尽才判失败）；
                       · 停稳且 > EMPTY_POS = 被物料挡住 = 真夹到了 -> 静置后放行。 */
                    if (actSrvSeen == 0U)
                    {
                        actSrvSeen      = 1U;
                        actSrvLastPos   = cur;
                        actSrvStallTick = now;
                        return 0U;
                    }
                    /* ★【2026-10-28】用 `ACT_SRV_STALL_EPS` 死区判"还在动"：
                       回读 ±1 抖动会一直刷新 stallTick -> 永远判不出停稳 -> 空抓检测失效。 */
                    d = (int32_t)cur - (int32_t)actSrvLastPos;
                    if (d < 0) { d = -d; }
                    if (d > (int32_t)ACT_SRV_STALL_EPS)
                    {
                        actSrvLastPos   = cur;
                        actSrvStallTick = now;
                        actSrvMoved     = 1U;
                        return 0U;
                    }
                    if ((now - actSrvStallTick) < (uint32_t)ACT_GRIP_STALL_MS)
                    {
                        return 0U;                  /* 等停稳 */
                    }
                    if (cur <= (uint16_t)ACT_GRIP_EMPTY_POS)
                    {
                        (void)Act_GripRetryOrFail(now, "EMPTY (closed on air)", cur);
                        return 0U;
                    }
                    return Act_GripSettleDone(now);   /* 停稳且被物料挡住 = 真夹到了 */
                }
            }
            else if (d <= (int32_t)ACT_SERVO_LOOSE_TOL)
            {
                return Act_GripSettleDone(now);
            }

            /* 本步"命令之后"的第一条回读：只作基准 */
            if (actSrvSeen == 0U)
            {
                actSrvSeen      = 1U;
                actSrvLastPos   = cur;
                actSrvStallTick = now;
                return 0U;
            }

            /* ② 还在动：刷新基准，继续等（±`ACT_SRV_STALL_EPS` 内算没动，防回读抖动） */
            d = (int32_t)cur - (int32_t)actSrvLastPos;
            if (d < 0) { d = -d; }
            if (d > (int32_t)ACT_SRV_STALL_EPS)
            {
                actSrvLastPos   = cur;
                actSrvStallTick = now;
                actSrvMoved     = 1U;       /* 动过 = 命令确实送到了 */
                return 0U;
            }

            /* 停住不动但没到目标：**只有"动过"才算被物料挡住（= 确实夹到东西了）**
               ★【2026-10-23】没动过 = 命令丢了，不能当"夹住"放行，往下走重发/中止。
               ★★【2026-10-24 修"没夹到料就抬臂"】夹紧步**还要求停住位置在夹紧末段**
                  （≤ `ACT_GRIP_GRAB_MAX`）：停在半路（比如 800）只是"一次回读没刷新"，
                  绝不能当成夹到料。 */
            if ((actSrvMoved != 0U) &&
                ((now - actSrvStallTick) >= (uint32_t)ACT_GRIP_STALL_MS))
            {
                if ((isClose != 0U) && (cur > (uint16_t)ACT_GRIP_GRAB_MAX))
                {
                    if (actSrvBlocked == 0U)
                    {
                        actSrvBlocked = 1U;
                        Debug_Log("ACT", "step%u S2 stall at %u (> GRAB_MAX %u) -> still closing, wait",
                                  (unsigned int)(actIdx + 1U), (unsigned int)cur,
                                  (unsigned int)ACT_GRIP_GRAB_MAX);
                    }
                    return 0U;
                }
                if (actSrvBlocked == 0U)
                {
                    actSrvBlocked = 1U;
                    Debug_Log("ACT", "step%u S2 held at %u (target %ld) -> grabbed, next",
                              (unsigned int)(actIdx + 1U), (unsigned int)cur,
                              (long)st->param);
                }
                return Act_GripSettleDone(now);   /* 顶住了也要先夹稳，再走下一步 */
            }
        }
    }

    /* ★★【2026-10-24】夹紧硬超时：等太久（丢帧卡半路 / 物料一直滑）-> **张开重夹**，
       绝不无限等，也绝不假装夹到。放在"重发"之前判：重发救不了"卡住"的情况。 */
    if ((isClose != 0U) &&
        ((now - actStepTick) >= (uint32_t)ACT_GRIP_CLOSE_MAX_MS))
    {
        (void)Act_GripRetryOrFail(now, "close timeout", cur);
        return 0U;
    }

    /* ③/④ 到点仍未确认：没动过（命令丢了）-> 重发；夹紧步用尽 -> **重夹 / 中止** */
    if ((now - actStepTick) >= (uint32_t)ACT_GRIP_WAIT_MS)
    {
        if (((havePos == 0U) || (actSrvMoved == 0U)) && (actSrvRetx < retxMax))
        {
            /* 重发前等总线静默（否则会和读应答撞车被丢） */
            if (Servo_BusQuiet((uint16_t)ACT_SRV_BUS_QUIET_MS) == 0U)
            {
                return 0U;                  /* 下一拍再重发（不消耗重试次数、不重置计时） */
            }
            actSrvRetx++;
            actStepTick     = now;          /* 重新计时（重发次数有上限，总时长有界） */
            actSrvStepTick  = actStepTick;  /* 必须同步，否则下一拍会把重发次数清零 */
            actSrvSeen      = 0U;
            actSrvMoved     = 0U;
            actSrvStallTick = now;
            actSrvBlocked   = 0U;
            Arm_ServoMove(id, Act_ServoParam(st->param));
            Debug_Log("ACT", "step%u S2 no move (pos=%u target %ld, hp=%u) -> resend %u/%u",
                      (unsigned int)(actIdx + 1U), (unsigned int)cur,
                      (long)st->param, (unsigned int)havePos,
                      (unsigned int)actSrvRetx, (unsigned int)retxMax);
            return 0U;
        }

        /* ★★【2026-10-28 修"抓空"】**夹紧步动过但还没停稳 -> 绝不放行**。
           以前这里是 `return 1U`（日志就是 `step1 GRIP ok +400 ms`）：夹爪从 950 往 500
           走要 600ms 以上，400ms 时回读还在 727/868 就被放行，M6 立刻抬臂 -> 夹空。
           现在继续等：真停稳 -> 上面判"夹到 / 空抓"；等够 `ACT_GRIP_CLOSE_MAX_MS`
           -> 张开重夹（带 M6 下降找料）。 */
        if ((isClose != 0U) && (havePos != 0U) && (actSrvMoved != 0U))
        {
            return 0U;
        }

        if ((actSrvMoved == 0U) || (havePos == 0U))
        {
            /* ★【2026-10-24 用户要求】"优先保证抓住物料，再进行别的动作"：
               夹紧步重试用尽仍未确认 -> **张开重夹**；重夹用尽 -> 判失败 + 中止本段。
               （覆盖 2026-10-23"不要 FAIL、放行"的旧要求——用户现已明确抓住优先。） */
            if (isClose != 0U)
            {
                (void)Act_GripRetryOrFail(now, "UNCONFIRMED", cur);
                return 0U;
            }

            actUnreached = 1U;              /* 如实告警 */
            Debug_Log("ACT", "step%u S2 UNCONFIRMED (pos=%u target=%ld, hp=%u) -> abort",
                      (unsigned int)(actIdx + 1U), (unsigned int)cur,
                      (long)st->param, (unsigned int)havePos);
            Act_AbortInternal("grip unconfirmed");
            return 0U;
        }
        return 1U;
    }

    return 0U;
}

/**
 * @brief  判断舵机步是否已完成
 * @param  st : 当前步骤（AJT_CLAW / AJT_GRIP）
 * @param  now: 当前时刻
 * @retval 1=完成
 * @note   舵机没有"到位帧"，只能靠 0x15 回读位置判断。★【2026-10-15 用户要求】
 *         "**抓放优先速度，精度都在下一个关节运行的时候闭环**"：
 *          ① 回读已进 `ACT_SERVO_LOOSE_TOL` -> **立刻放行**（最快路径）；
 *          ② 否则最多等 `ACT_SRV_FAST_MS`(120ms，≈ 一个回读周期) 就放行；
 *             "到底到没到 / 命令丢没丢"**全部交给后台** `Act_SrvVerifyService`
 *             （没到就重发，最多 `ACT_SRV_RETX_MAX` 次；到点还没到才记 UNREACHED）。
 *          ③ 安全例外：`actPairTurnPend`（S1 缩回 + M5 转向的挂起对）**不许提前放行**，
 *             必须等 S1 ≤ `ACT_PAIR_TURN_S1_HI` 才让转向发出（否则会带着伸出的爪子转头）。
 *         `ACT_SERVO_OPEN_LOOP == 0` 时仍走下面的旧回读闭环档（保持原行为）。
 */
static uint8_t Act_ServoDone(const ActStep_t *st, uint32_t now)
{
#if (ACT_SERVO_OPEN_LOOP != 0U)
    /* ★★【2026-10-15 用户要求】"**夹紧和松抓，还是保证松开和抓到再进行下一步**"
       -> S2（AJT_GRIP）**必须确认**（到位置 / 被物料挡住 / 重发后仍读不到才放行）；
          S1（AJT_CLAW，前伸/缩回）才走"到点放行 + 后台校验"的快档。 */
    if (st->joint == AJT_GRIP)
    {
        return Act_ServoDoneGrip(st, now);
    }

    /* ===== ★规则1 + 2026-10-15 提速：到位置附近 -> 立刻放行；否则到点放行，校验挪后台 =====
       ⚠ 原来的"内联重发"已挪到后台（见 `Act_SrvVerifyService`）：
         它负责"命令丢了就补发"，而且是在**下一个关节已经在动**的时候补，不占流程时间。
         这样既满足"抓放优先速度"，又保留"命令不会永久丢"的保证。 */
    {
        uint8_t  id  = (st->joint == AJT_CLAW) ? SERVO_ID_CLAW : SERVO_ID_GRIP;
        uint16_t cur = 0U;
        int32_t  d;

        /* ---- ① 最快路径：回读（必须是"本次命令之后"、且是**本 ID** 的）已在目标附近 -> 立刻放行 ---- */
        if (Servo_GetPos(id, &cur) != 0U)
        {
            if ((int32_t)(Servo_GetPosTickById(id) - actStepTick) >= 0)
            {
                d = (int32_t)cur - st->param;
                if (d < 0)
                {
                    d = -d;
                }
                if (d <= (int32_t)ACT_SERVO_LOOSE_TOL)
                {
                    Act_SrvVerifyClear(id);   /* 已到位：后台不用再管这个舵机 */
                    return 1U;
                }
            }
        }

        /* ---- ② 到点放行：**抓放优先速度** ----
           ★【2026-10-15 用户要求】"精度都在下一个关节运行的时候闭环"：
           最多等 `ACT_SRV_FAST_MS`(120ms，≈ 一个回读周期) 就放行下一步；
           "到底到没到 / 命令是不是丢了"交给**后台** `Act_SrvVerifyService`
           （没到就重发，最多 `ACT_SRV_RETX_MAX` 次；到点还没到才记 UNREACHED）。
           ⚠ 安全例外：本步若是"**舵机1 缩回 + 电机5 转向**"的**挂起对**（`actPairTurnPend`），
             必须按规则2/3 等 S1 ≤ `ACT_PAIR_TURN_S1_HI` 才发转向，**这里不许提前放行** ——
             否则会带着伸出的爪子转头（用户明确要求）。 */
        if (actPairTurnPend != 0U)
        {
            return 0U;                  /* 交给 Act_PairTurnKick：S1 到位或 400ms 兜底后才发转向 */
        }

        if ((now - actStepTick) >= (uint32_t)ACT_SRV_FAST_MS)
        {
            return 1U;
        }
    }
    return 0U;
#else
    uint8_t  id  = (st->joint == AJT_CLAW) ? SERVO_ID_CLAW : SERVO_ID_GRIP;
    uint16_t cur = 0U;
    int32_t  d;

    /* 换了一步：把"顶住判定"的基准清掉 */
    if (actSrvStepTick != actStepTick)
    {
        actSrvStepTick  = actStepTick;
        actSrvLastPos   = 0U;
        actSrvStallTick = now;
        actSrvBlocked   = 0U;
        actSrvMoved     = 0U;    /* 本步"动过没有"重新计（丢帧防护要用） */
        actSrvRetx      = 0U;
        actSrvSeen      = 0U;
    }

    /* ---- ① 回读可用：**只认回读**（不到位就继续等，不再用计时放行）----
       这是"物料还没抓到手臂就抬起来"的根因修复：
       以前超过 ARM_SEQ_SERVO_WAIT_MS 就无条件放行，爪子还没夹紧就被当成夹好了。 */
    if (Servo_GetPos(id, &cur) != 0U)
    {
        /* 只认"本次命令下发之后"**本 ID** 拿到的回读：否则会用命令之前的旧值提前放行 */
        if ((int32_t)(Servo_GetPosTickById(id) - actStepTick) < 0)
        {
            return 0U;
        }

        d = (int32_t)cur - st->param;
        if (d < 0) { d = -d; }
        if (d <= (int32_t)ARM_SEQ_SERVO_TOL)
        {
            return 1U;                      /* 已到目标 */
        }

        /* 本步"命令之后"的第一条回读：只当**基准**，不算"动过"
           （否则基准 0 会让第一条回读永远看起来"动过"，丢帧重发就永远触发不了）。 */
        if (actSrvSeen == 0U)
        {
            actSrvSeen      = 1U;
            actSrvLastPos   = cur;
            actSrvStallTick = now;
            return 0U;
        }

        /* 没到目标：看它是不是已经**顶住不动**了（被物料挡住 -> 夹到位；
           或行程到头 -> 也只能这样了）。变化幅度超过容差才算"还在动"。 */
        d = (int32_t)cur - (int32_t)actSrvLastPos;
        if (d < 0) { d = -d; }
        if (d > (int32_t)ARM_SEQ_SERVO_TOL)
        {
            actSrvLastPos   = cur;          /* 还在动：刷新基准 */
            actSrvStallTick = now;
            actSrvMoved     = 1U;           /* 动过 = 命令确实送到了（丢帧防护的关键证据） */
            return 0U;
        }
        if ((now - actSrvStallTick) >= (uint32_t)ACT_STALL_MS)
        {
            /* 【丢动作防护】回读从命令发出到现在**一次都没变过** + 离目标还远
               -> 大概率是舵机命令在 9600 串口上丢了（官方库只发不校验）
               -> 重发同一条命令，而不是直接当成"夹到位"放行（那会让爪子没夹/没松就继续）。 */
            if ((actSrvMoved == 0U) && (actSrvRetx < (uint8_t)ACT_SRV_RETX_MAX))
            {
                /* ★【2026-10-24】重发前必须等总线静默，否则会和读应答撞车被丢 */
                if (Servo_BusQuiet((uint16_t)ACT_SRV_BUS_QUIET_MS) == 0U)
                {
                    return 0U;          /* 下一拍再重发（不消耗重试次数） */
                }
                actSrvRetx++;
                actSrvStallTick = now;
                actSrvBlocked   = 0U;
                Arm_ServoMove(id, Act_ServoParam(st->param));
                Debug_Log("ACT", "step%u %s no move at pos=%u (target %ld) -> resend %u/%u",
                          (unsigned int)(actIdx + 1U), Act_JointName(st->joint),
                          (unsigned int)cur, (long)st->param,
                          (unsigned int)actSrvRetx, (unsigned int)ACT_SRV_RETX_MAX);
                return 0U;
            }

            if (actSrvBlocked == 0U)
            {
                actSrvBlocked = 1U;
                Debug_Log("ACT", "step%u %s held at %u (target %ld) -> treat as done",
                          (unsigned int)(actIdx + 1U), Act_JointName(st->joint),
                          (unsigned int)cur, (long)st->param);
            }
            return 1U;
        }
        return 0U;
    }

    /* ---- ② 完全读不到回读（没接线/调试板不应答）：只能按时间兜底 ----
       这种情况没法知道爪子到没到，只能等够 SERVO_MOVE_TIME_MS+余量 再走。 */
    return (((now - actStepTick) >= (uint32_t)ARM_SEQ_SERVO_WAIT_MS) ? 1U : 0U);
#endif   /* ACT_SERVO_OPEN_LOOP */
}

/**
 * @brief  判断某电机当前位置是否已在"目标 ± 容差"内
 * @param  addr  : ARM_ADDR_TURN / ARM_ADDR_LIFT
 * @param  target: 目标（下发脉冲域）
 * @param  tol   : 容差（下发脉冲域，恒正）
 * @retval 1=在容差内；0=不在或位置读不到
 */
static uint8_t Act_NearTarget(uint8_t addr, int32_t target, int32_t tol)
{
    int32_t d;

    if (Arm_IsPosValid(addr) == 0U) { return 0U; }

    d = Arm_GetPos(addr) - target;
    if (d < 0) { d = -d; }
    return (d <= tol) ? 1U : 0U;
}

/**
 * @brief  ★【2026-10-24 用户要求】"**这里不要犹豫**"：本步目标**现在就已经满足**
 * @param  st: 本步
 * @retval 1=已经到位（可以直接跨步，不用下发、不用等回读/到位帧）；0=还要动
 * @note   典型场景（TRAY PICK 抓完料 -> 放车载）：
 *           `flowTtPick` 末步已经把 **S1 缩回 0、M5 转到车载位**，
 *           `flowCarPlace` 开头那两步（S1 -> `ARM_CAR_S1`、M5 -> `ARM_CAR_M5`）
 *           目标与当前**完全相同**，是空跑；S3 也通常已由"对位成功预选"转到位。
 *         原来每步仍要走"等总线静默 -> 下发 -> 等回读/到位帧"一圈，
 *         现场就是"**到车载位上方犹豫一下才下降**"。
 *         现在：目标已满足就**直接跨步** —— 紧接着就是 M6 下降 + 松抓。
 *         ⚠ 容差用"放宽"档（舵机 `ACT_SERVO_LOOSE_TOL` / S3 `ACT_S3_ARRIVE_TOL` /
 *           电机 严格容差 × `ACT_MOVE_LOOSE_MULT`），与"差不多就放行"的口径一致；
 *           读不到回读时一律返回 0（该发还是发，绝不靠猜）。
 */
static uint8_t Act_StepAlreadyThere(const ActStep_t *st)
{
    int32_t d;

    /* ★【2026-10-29】带 `ACT_STEP_FORCE` 的步**必须真正下发**，不做"已经在那儿"的跳过
       （车载伸爪位 66 与缩回位 0 只差 66 < 容差 80，否则永远被跳过 -> 手不伸）。 */
    if ((st->flags & ACT_STEP_FORCE) != 0U)
    {
        return 0U;
    }

    if (st->joint == AJT_CLAW)
    {
        uint16_t cur = 0U;

        if (Servo_GetPos((uint8_t)SERVO_ID_CLAW, &cur) == 0U) { return 0U; }
        /* ★【2026-10-27】只认"很新"的回读（≤ ACT_SKIP_POS_FRESH_MS）：
           `Servo_GetPos` 的有效期是 1000ms，上一步留下的旧值也能让**整步被跳过**，
           于是这条命令**根本没下发** -> 现场"这一步有时执行、有时不执行"。 */
        if ((HAL_GetTick() - Servo_GetPosTickById((uint8_t)SERVO_ID_CLAW))
            > (uint32_t)ACT_SKIP_POS_FRESH_MS)
        {
            return 0U;
        }
        d = (int32_t)cur - st->param;
        if (d < 0) { d = -d; }
        return (d <= (int32_t)ACT_SERVO_LOOSE_TOL) ? 1U : 0U;
    }

    if (st->joint == AJT_GRIP)
    {
        /* ★★【2026-10-23】**夹爪步一律真正下发，不做"已经在那儿就跳过"**：
           现场出现过"S2 中途完全不工作" —— 根因是上一条"张开"命令丢了/没执行，
           夹爪停在闭合位(≈500)，之后每个"夹紧"步都被这里当成"已到位"跳过，
           于是再也不发 S2 命令（人看着就是"舵机2 完全不工作"）。
           夹爪命令是幂等的，多发一帧代价极小，**可靠性优先**。 */
        return 0U;
    }

    if (st->joint == AJT_CAR)
    {
        uint16_t cur = 0U;

        if (Servo_GetPos((uint8_t)SERVO_ID_TURNTABLE, &cur) == 0U) { return 0U; }
        /* ★【2026-10-27】同 AJT_CLAW：只认"很新"的回读，避免用旧值把整步跳过（不下发） */
        if ((HAL_GetTick() - Servo_GetPosTickById((uint8_t)SERVO_ID_TURNTABLE))
            > (uint32_t)ACT_SKIP_POS_FRESH_MS)
        {
            return 0U;
        }
        d = (int32_t)cur - st->param;
        if (d < 0) { d = -d; }
        return (d <= (int32_t)ACT_S3_ARRIVE_TOL) ? 1U : 0U;
    }

    if (st->joint == AJT_TURN)
    {
        return Act_NearTarget((uint8_t)ARM_ADDR_TURN, st->param,
                              (int32_t)ARM_SEQ_TURN_POS_TOL * (int32_t)ACT_MOVE_LOOSE_MULT);
    }

    if (st->joint == AJT_LIFT)
    {
        return Act_NearTarget((uint8_t)ARM_ADDR_LIFT, st->param,
                              (int32_t)ARM_SEQ_LIFT_POS_TOL * (int32_t)ACT_MOVE_LOOSE_MULT);
    }

    return 0U;
}

/**
 * @brief  ★【2026-10-11】驱动器"到位帧"是否**可信**（防"没动就放行"）
 * @param  addr  : 电机地址
 * @param  target: 本步目标（下发脉冲域）
 * @param  tol   : 本轴容差
 * @param  pos0  : **本步命令下发前**该轴的位置（ACT_POS_NONE = 不知道）
 * @retval 1=可信；0=视为"上一步的迟到帧"，不要据此放行
 * @note   【为什么必须有】`bsp_Emm_V5.c` 的 0x3A 到位标志位测试写错
 *         （`(pdata[1] & 0x02) == 0x01` **恒假**）导致 `Arm_IsReachDone()` **一直是 0**，
 *         于是"到位即放行"的①号判据从未生效 —— 现场现象就是**每步都要等软件的
 *         位置窗/闭环预算，电机5 卡好久**。0x3A 修好后这个判据才真正可用。
 *         但修好后多了一个新风险：**上一步的到位帧可能"迟到"到本步**（查询早于命令发出，
 *         应答晚于命令到达），会让"电机还没动"就误判到位。所以这里加一道可信校验：
 *           · 位置读不到        -> 只能信帧（否则永远没法放行）；
 *           · 已在容差内        -> 信（本步目标已达成，无所谓帧是不是旧的）；
 *           · 相对命令前**明显动过**（> ACT_REACH_TRUST_MOVE）-> 信（说明本步命令生效了）；
 *           · 其余（一点没动）  -> **不信**，继续等下面的位置/闭环判据。
 */
static uint8_t Act_ReachTrusted(uint8_t addr, int32_t target, int32_t tol, int32_t pos0)
{
    int32_t cur;
    int32_t d;

    if (Arm_IsPosValid(addr) == 0U)
    {
        return 1U;              /* 位置不可用：只能采信到位帧（否则完全没法推进） */
    }

    if (Act_NearTarget(addr, target, tol) != 0U)
    {
        return 1U;              /* 已经进容差：本步目标已达成 */
    }

    /* ★【2026-10-15 速度优先】到位帧 + 位置已在"**宽容差窗**"（tol × ARM_SEQ_REACH_GUARD_MUL）
       之内 -> **直接采信**，不再等"停稳 200ms + 严格容差校验"。
       【为什么可以信】到位帧是**驱动器自己的"已停稳"信号**（0.1~0.3° 窗口）；
       配"宽容差窗"足以排除"上一步的迟到帧"（迟到帧对应的位置差通常远大于 5×tol）。
       没进严格容差的那点残差交给**后台精修**（本函数返回后由调用方登记），符合
       "**精度闭环全部后台做、流程优先速度**"的口径。 */
    cur = Arm_GetPos(addr);
    d   = cur - target;
    if (d < 0) { d = -d; }
    if (d <= (tol * (int32_t)ARM_SEQ_REACH_GUARD_MUL))
    {
        return 1U;
    }

    if (pos0 == ACT_POS_NONE)
    {
        return 1U;              /* 不知道命令前的位置：无法判别，采信 */
    }

    d = cur - pos0;
    if (d < 0) { d = -d; }

    return (d > (int32_t)ACT_REACH_TRUST_MOVE) ? 1U : 0U;
}

/* 前置声明：Act_Advance 在其定义之前就要用（避免把函数整体搬来搬去） */
static const char *Act_JointName(uint8_t joint);

/**
 * @brief  当前步完成，进入下一步
 * @param  无
 * @retval 无
 * @note   打日志时用的是**已完成的那一步**的关节名（所以必须在 actIdx++ 之前取）。
 */
/* ===================== ★ 规则2（舵机1 + 电机5 并行）用的小工具 ===================== */

/**
 * @brief  电机6 当前是否在"**安全高度以上**"
 * @retval 1=是（可并行）；0=不是 / 位置读不到（保守：不并行）
 * @note   判据 `Arm_GetPos(ARM_ADDR_LIFT) >= ACT_PAIR_MIN_M6`（值越大越高，见 bsp_arm_action.h）。
 */
static uint8_t Act_LiftAboveSafe(void)
{
    if (Arm_IsPosValid(ARM_ADDR_LIFT) == 0U)
    {
        return 0U;
    }
    return (Arm_GetPos(ARM_ADDR_LIFT) >= (int32_t)ACT_PAIR_MIN_M6) ? 1U : 0U;
}

/**
 * @brief  ★【2026-10-15 用户要求】电机6 **上升步**能否"**提前放行**"
 * @param  st: 当前步（应为 AJT_LIFT）
 * @retval 1=可以（M6 已升过安全高度，且后面紧跟"舵机1 缩回 + 电机5 转向"）；0=不行
 * @note   背景：`rule3` 会在**每个 TURN 前插一步"舵机1 缩回"**，所以实际步骤是
 *           **[M6 上升] -> [S1 缩回] -> [M5 转向]**，`(M6 上升, TURN)` **永远不相邻**，
 *           靠"并行对"判据打不到。
 *         用户要求"**电机6 在安全高度以上的时候，上升可以和电机5 转向同时进行**"，
 *         做法 = **M6 一升过 `ACT_PAIR_MIN_M6` 就提前放行这一步**（M6 继续升到目标，
 *         由后台精修校验），让后面的"缩回 + 转向"立刻开始 ⇒ **上升与转向真正同时进行**。
 *         ⚠ 只认"上升 + 终点在安全高以上 + 后面正好是 (缩回, 转向)"这一种，别的步一律不动。
 */
static uint8_t Act_M6EarlyReleaseOk(const ActStep_t *st)
{
    if ((uint8_t)ACT_PAIR_M6_UP_TURN == 0U)                     { return 0U; }
    if (st->joint != AJT_LIFT)                                  { return 0U; }
    if ((actIdx + 2U) >= actStepCount)                          { return 0U; }
    if (actSteps[actIdx + 1U].joint != AJT_CLAW)                { return 0U; }
    if (actSteps[actIdx + 1U].param > (int32_t)ARM_FIX_S1_RET)  { return 0U; }  /* 必须是"缩回" */
    if (actSteps[actIdx + 2U].joint != AJT_TURN)                { return 0U; }
    if (actStepPos0 == ACT_POS_NONE)                            { return 0U; }
    if (st->param <= actStepPos0)                               { return 0U; }  /* 必须"上升" */
    if (st->param < (int32_t)ACT_PAIR_MIN_M6)                   { return 0U; }  /* 终点在安全高以上 */
    return Act_LiftAboveSafe();                                                 /* 已升过安全高才放行 */
}

/**
 * @brief  这两个关节是不是"舵机1 + 电机5"（允许并行的唯一一对）
 */
static uint8_t Act_IsS1M5Pair(uint8_t a, uint8_t b)
{
    return (((a == AJT_CLAW) && (b == AJT_TURN)) ||
            ((a == AJT_TURN) && (b == AJT_CLAW))) ? 1U : 0U;
}

/**
 * @brief  该步是不是"舵机1 且目标是**缩回**"（= 0）
 * @note   ★用户规则2：**只有"缩回"允许和电机5 并行**；
 *         "伸出"必须先等电机5 到位再伸（伸出时爪子在盘位附近，跟着大轴转会扫撞）。
 */
static uint8_t Act_S1StepIsRetract(uint8_t idx)
{
    return ((actSteps[idx].joint == AJT_CLAW) &&
            (actSteps[idx].param <= (int32_t)ARM_FIX_S1_RET)) ? 1U : 0U;
}

/**
 * @brief  当前步能否与下一步"**现在一起下发**"（★规则2）
 * @param  curIdx: 当前步下标
 * @retval 1=并行；0=不并行（照原样一步步走）
 * @note   条件：① 开关开；② 本步与下一步是 (舵机1, 电机5) 那一对；
 *         ③ **舵机1 那一步必须是"缩回"**（"伸出"不许并行，必须先等电机5 到位）；
 *         ④ **电机6 在安全高度以上**；
 *         ⑤ 若下一步是电机步，其位置必须**现在可读** —— 否则 `Arm_MoveAbsDir` 会拒发，
 *            那一步永远等不到"到位"，会把整步拖到超时。
 */
static uint8_t Act_PairOk(uint8_t curIdx)
{
    const ActStep_t *nxt;
    uint8_t addr;
    uint8_t s1Idx;

    if ((curIdx + 1U) >= actStepCount)                       { return 0U; }

    /* ★★【2026-10-24】模板里**显式写了 `ACT_STEP_PAR`** 的步：直接允许同拍下发。
       （转盘取料"伸爪 ‖ 下降"、放车载"缩爪 ‖ 降 M6"、观察位"转向 ‖ 伸爪"）
       不受"必须是 S1+M5 / 必须缩回 / 必须安全高"这些旧限制 —— 由模板逐条把关。 */
    if ((actSteps[curIdx].flags & ACT_STEP_PAR) != 0U)
    {
        nxt = &actSteps[curIdx + 1U];
        if ((nxt->joint == AJT_TURN) || (nxt->joint == AJT_LIFT))
        {
            addr = (nxt->joint == AJT_TURN) ? (uint8_t)ARM_ADDR_TURN
                                            : (uint8_t)ARM_ADDR_LIFT;
            if (Arm_IsPosValid(addr) == 0U)                  { return 0U; }
        }
        return 1U;
    }

    if ((uint8_t)ACT_PAIR_S1_M5 == 0U)                       { return 0U; }
    if (Act_IsS1M5Pair(actSteps[curIdx].joint,
                       actSteps[curIdx + 1U].joint) == 0U)   { return 0U; }

    /* ★规则2：找出这一对里"舵机1"在哪一步，**只允许缩回**并行 */
    s1Idx = (actSteps[curIdx].joint == AJT_CLAW) ? curIdx : (uint8_t)(curIdx + 1U);
    if (Act_S1StepIsRetract(s1Idx) == 0U)                    { return 0U; }

    if (Act_LiftAboveSafe() == 0U)                           { return 0U; }

    nxt = &actSteps[curIdx + 1U];
    if ((nxt->joint == AJT_TURN) || (nxt->joint == AJT_LIFT))
    {
        addr = (nxt->joint == AJT_TURN) ? (uint8_t)ARM_ADDR_TURN : (uint8_t)ARM_ADDR_LIFT;
        if (Arm_IsPosValid(addr) == 0U)                      { return 0U; }
    }
    return 1U;
}

/**
 * @brief  `ACT_STEP_EARLY` 的"提前放行窗口"（按关节区分）
 * @param  joint: ActJoint_t
 * @retval 电机5 = ±100（用户："转到目标参数 ±100 内电机6 下降"）；
 *         电机6 = ±10000（用户 STORE BACK："上升离目标 ±10000 就开始提前回缩舵机1"）
 */
static int32_t Act_EarlyMargin(uint8_t joint)
{
    return (joint == AJT_TURN) ? (int32_t)ACT_TURN_EARLY_REL
                               : (int32_t)ACT_LIFT_EARLY_REL;
}

/**
 * @brief  某一步"是否到位"（**并行判定专用**）
 * @param  idx: 步骤下标
 * @param  now: 当前时刻
 * @retval 1=到位
 * @note   ⚠ 只给"被并行的那一步"用；**当前步**仍走主循环里的分支（那里还有闭环/精修/日志）。
 *         电机步只按"到位帧 或 位置进本轴容差"判（不参与闭环纠正），避免和主分支抢状态。
 */
static uint8_t Act_StepDoneAt(uint8_t idx, uint32_t now)
{
    const ActStep_t *s = &actSteps[idx];
    uint8_t addr;
    int32_t tol;

    if ((s->joint == AJT_CLAW) || (s->joint == AJT_GRIP))
    {
        return Act_ServoDone(s, now);
    }

    /* ★【2026-10-24】车载转盘（舵机3）也能进并行组：
       能读到回读并且已到 ±容差 -> 立刻算到位；
       ★【2026-10-24 修"手臂悬在车载位上方停很久"】回读不可靠时**按 `ACT_S3_MOVE_MS` 开环放行**，
         不再死等回读 —— 否则整组要耗到 `ACT_PAIR_GRACE_MS`(1.5s) 才被强放。 */
    if (s->joint == AJT_CAR)
    {
        if (Act_S3At(s->param) != 0U) { return 1U; }
        return ((HAL_GetTick() - actStepTick) >= (uint32_t)ACT_S3_MOVE_MS) ? 1U : 0U;
    }

    addr = (s->joint == AJT_TURN) ? (uint8_t)ARM_ADDR_TURN : (uint8_t)ARM_ADDR_LIFT;
    tol  = (s->joint == AJT_TURN) ? (int32_t)ARM_SEQ_TURN_POS_TOL
                                  : (int32_t)ARM_SEQ_LIFT_POS_TOL;

    /* ★【2026-10-24 用户要求】带 `ACT_STEP_EARLY` 的步：进 **±`ACT_TURN_EARLY_REL`(100)**
       就算到位（"电机5 转到目标 ±100 内 -> 电机6 下降"）。
       并行组里"转向"是**组员**，由本函数判定，所以提前放行必须写在这里。 */
    if (((s->flags & ACT_STEP_EARLY) != 0U) &&
        (Act_NearTarget(addr, s->param, Act_EarlyMargin(s->joint)) != 0U))
    {
        return 1U;
    }

    if ((Arm_IsReachDone(addr) != 0U) &&
        (Act_ReachTrusted(addr, s->param, tol, actParPos0[idx]) != 0U))
    {
        return 1U;
    }
    if ((Arm_IsPosValid(addr) != 0U) &&
        (Act_NearTarget(addr, s->param, tol * (int32_t)ACT_MOVE_LOOSE_MULT) != 0U))
    {
        return 1U;      /* ★宽容差（× ACT_MOVE_LOOSE_MULT）即算组内到位，别让整组等闭环 */
    }
    return 0U;
}

/**
 * @brief  "缩回 + 转电机5"并行时：**舵机1 的值在 0~20 就发转向**
 * @param  now: 当前时刻
 * @retval 无
 * @note   下发"舵机1 缩回"那一步时若发现下一步是**可以并行的 TURN**，就先**只发缩回**、
 *         把转向**挂起**（`actPairTurnPend`）；本函数每拍检查：
 *           ① 回读可用且 `值 ≤ ACT_PAIR_TURN_S1_HI`(20) -> 发转向；
 *           ② **兜底**：回读异常 / 一直没进区间 -> 到 `ACT_SERVO_MOVE_MS` 也**无条件**发转向。
 *         ⚠ 仅当 `ACT_PAIR_TURN_HALF = 1` 时才会挂起；HALF=0（**当前**）时这类对是"立即一起发"，
 *           所以本函数实际为空（保留是为了随时能改回旧行为）。
 *         发出后把 `actPairOn` 置 1：完成判据随之变成"**两个都到位**"（见 `Act_Advance`）。
 */
static void Act_PairTurnKick(uint32_t now)
{
#if (ACT_PAIR_TURN_HALF != 0U)
    uint16_t cur = 0U;
    uint8_t  go  = 0U;

    if (actPairTurnPend == 0U)
    {
        return;
    }
    if ((actIdx + 1U) >= actStepCount)
    {
        actPairTurnPend = 0U;       /* 兜底：没有下一步（正常不会发生） */
        return;
    }

    if (Servo_GetPos((uint8_t)SERVO_ID_CLAW, &cur) != 0U)
    {
        /* 舵机1 的值 ≤ 20（位置域 0~1000 ⇒ 即 0~20）就允许转（**不要求回到 0**） */
        if (cur <= (uint16_t)ACT_PAIR_TURN_S1_HI)
        {
            go = 1U;
        }
    }

    /* ★兜底：**一定放行**（读不到 / 一直没进区间都不再卡死） */
    if ((go == 0U) && ((now - actStepTick) >= (uint32_t)ACT_SERVO_MOVE_MS))
    {
        go = 1U;
    }

    if (go == 0U)
    {
        return;
    }

    actPairTurnPend = 0U;
    Act_SendStep(&actSteps[actIdx + 1U]);   /* 现在才把转向发出去 */
    actPairOn = 1U;
    Debug_Log("ACT", "pair: S1=%u <= %u -> TURN tgt=%ld",
              (unsigned int)cur, (unsigned int)ACT_PAIR_TURN_S1_HI,
              (long)actSteps[actIdx + 1U].param);
#else
    (void)now;
#endif
}

/**
 * @brief  真正前进到下一步（**原逻辑**；并行跨步由外层 `Act_Advance` 处理）
 */
static void Act_AdvanceRaw(void)
{
    /* 带本步耗时：现场一眼看出**时间花在哪一步**（调抓取节奏主要看这行）。
       电机步 ≈ ACT_STALL_MS 起；舵机步 ≈ SERVO_READ_PERIOD_MS 起。 */
    Debug_Log("ACT", "step%u %s ok +%u ms", (unsigned int)(actIdx + 1U),
              Act_JointName(actSteps[actIdx].joint),
              (unsigned int)(HAL_GetTick() - actStepTick));

    /* 本步结束：关掉位置快查，恢复正常 6 拍轮转（否则其它电机的状态不再刷新） */
    Arm_SetFastPoll(0U);

    actIdx++;
    actStepSent  = 0U;
    actPosWaitOn = 0U;
    actTurnPendIdx = ACT_TURN_PEND_NONE;   /* 跨步：清掉"等 S1 再转向"的挂起 */
    actLateIdx     = ACT_LATE_NONE;        /* 跨步：清掉"错开启动"挂起 */
    actS1GateTick  = 0U;                   /* 跨步：清掉 STORE BACK 的"等 S1<500 再转"计时 */
    actCarS3Resent = 0U;                   /* 跨步：S3 的"已重发"标志复位 */

    if (actIdx >= actStepCount)
    {
        actState = ACT_DONE;

        /* 【保障层】有步骤是"补发用完仍未到目标"就明确告警 —— 不能让它看着像成功 */
        if (actUnreached != 0U)
        {
            Debug_Log("ACT", "%s done in %u ms **UNREACHED** (some step stopped short)",
                      actTitle, (unsigned int)(HAL_GetTick() - actRunTick));
        }
        else
        {
            Debug_Log("ACT", "%s done in %u ms", actTitle,
                      (unsigned int)(HAL_GetTick() - actRunTick));
        }
    }
}

/**
 * @brief  本步完成 -> 前进（**对外唯一的"前进"入口**）
 * @param  无
 * @retval 无
 * @note   ★【规则2】**并行时**：先确认"**组内每个关节都到位**"（有一个没到就原地等，不抢跑），
 *         再**一次跨过整组** —— 被并行的步已经在开头一起下发过，不能再单独下发。
 *         并行组可以是 2 步（S1+M5）或 3 步（如"抬升 M6 + 缩回 S1 + 转向 M5"）。
 *         非并行时就是原来的"前进一步"。
 */
static void Act_Advance(void)
{
    uint8_t k;

    if (actPairOn == 0U)
    {
        Act_AdvanceRaw();
        return;
    }

    /* ★【2026-10-23】组里还有一步"错开启动"没发（actLateIdx）：先别跨步，
       否则当前步（如 M6 上升）一到点就把整组带过去、延后那步根本没发。 */
    if (actLateIdx != ACT_LATE_NONE)
    {
        return;
    }

    /* ★【2026-10-24】组里还有一步"等 S1 < 500 才发"的 TURN（`Act_TurnGateKick`）：
       先别跨步 —— 否则舵机1 一被"到点放行"(120ms) 就会把整组带过去，转向没等就发了。
       闸门有 `ACT_TURN_GATE_MS`(400ms) 兜底，不会卡死。 */
    if (actTurnPendIdx != ACT_TURN_PEND_NONE)
    {
        return;
    }

    if (actParN < 2U) { actParN = 2U; }     /* 防御：并行组至少 2 步 */

    /* ★【2026-10-24】并行组可能是 **2 步或 3 步**（如"抬升+缩回+转向"）：
       组内**每个**关节都到位才允许跨步。 */
    for (k = 1U; k < actParN; k++)
    {
        if ((uint8_t)(actIdx + k) >= actStepCount)
        {
            break;                          /* 组尾越界（正常不会发生）：按已存在的算 */
        }
        if (Act_StepDoneAt((uint8_t)(actIdx + k), HAL_GetTick()) == 0U)
        {
#if (ACT_PAIR_GRACE_MS != 0U)
            /* ★【2026-10-10 晚】**并行"卡死"兜底**：当前步已完成，但组里还有关节在
               `ACT_PAIR_GRACE_MS` 内仍没到位 -> **放行 + 记 UNREACHED**。
               剩余偏差交给该轴自身的"后台精修"或后续步骤继续处理。 */
            if ((HAL_GetTick() - actStepTick) >= (uint32_t)ACT_PAIR_GRACE_MS)
            {
                uint8_t n = actParN;

                actUnreached = 1U;
                actPairOn    = 0U;
                actParN      = 1U;
                Debug_Log("ACT", "pair: step%u %s not reached in %ums -> release anyway",
                          (unsigned int)(actIdx + k + 1U),
                          Act_JointName(actSteps[actIdx + k].joint),
                          (unsigned int)ACT_PAIR_GRACE_MS);
                while ((n > 0U) && (actIdx < actStepCount))
                {
                    Act_AdvanceRaw();
                    n--;
                }
                return;
            }
#endif
            return;                         /* 组里还有关节没到位 -> 继续等 */
        }
    }

    {
        uint8_t n = actParN;

        actPairOn = 0U;
        actParN   = 1U;
        while ((n > 0U) && (actIdx < actStepCount))
        {
            Act_AdvanceRaw();               /* 一次跨过整组（都已在下发时发过） */
            n--;
        }
    }
}

/**
 * @brief  中止当前动作（电机急停 + 状态置 ABORT）
 * @param  why: 原因（日志用）
 * @retval 无
 */
static void Act_AbortInternal(const char *why)
{
    Arm_Stop(ARM_ADDR_TURN);
    Arm_Stop(ARM_ADDR_LIFT);
    Arm_SetFastPoll(0U);      /* 中止也必须关掉位置快查 */
    Act_TrimClear(ARM_ADDR_TURN);   /* 中止：两轴的"后台精修"一并取消（电机已急停） */
    Act_TrimClear(ARM_ADDR_LIFT);
    actState = ACT_ABORT;
    Debug_Log("ACT", "%s abort (%s)", actTitle, why);
}

/* ===================== ★ 规则3：所有转向前，舵机1 先回 0 =====================
   用户 2026-10-10 晚："在定义一个规则，是所有转向前舵机1都要先回到0。"
   【做法】在**步骤展开好之后、开始执行之前**做一次插入扫描：
     凡遇到 `AJT_TURN`（电机5 转向）那一步，就在它**前面插一步"舵机1 -> 0"（缩回）**；
     若前一步**本来就是**"舵机1 缩回"，就不再重复插。
   【与规则2 天然配合】插进来的是"缩回"，正好满足规则2 的并行条件
     ⇒ 这两步会变成**可并行对**：S1 一边收、M5 一边转（用户已明确"缩回可以和电机5 一起动"）。
   【保底】步骤位置不够（`ACT_MAX_STEPS`）就**放弃插入并告警**，绝不越界。 */
static void Act_EnsureS1RetBeforeTurn(void)
{
    /* ★【2026-10-21】本动作明确要求"转向前不收爪"（观察区 -> 抓取姿态），直接跳过 */
    if (actSkipRule3 != 0U)
    {
        actSkipRule3 = 0U;
        Debug_Log("ACT", "rule3 skipped for this action (no S1 ret before TURN)");
        return;
    }

#if (ACT_RULE3_S1_RET_BEFORE_TURN != 0U)
    ActStep_t out[ACT_MAX_STEPS];
    uint8_t   n = 0U;
    uint8_t   i;

    for (i = 0U; i < actStepCount; i++)
    {
        if (actSteps[i].joint == AJT_TURN)
        {
            uint8_t already = 0U;

            /* 前一步（按**原始数组**判断）本来就是"舵机1 缩回" -> 不用插 */
            if ((i > 0U) && (actSteps[i - 1U].joint == AJT_CLAW) &&
                (actSteps[i - 1U].param <= (int32_t)ARM_FIX_S1_RET))
            {
                already = 1U;
            }

            if (already == 0U)
            {
                if (n >= (uint8_t)ACT_MAX_STEPS)
                {
                    Debug_Log("ACT", "rule3 S1->0 insert SKIPPED (steps full)");
                    return;                 /* 不插入，保持原步骤（安全第一） */
                }
                out[n].joint = AJT_CLAW;
                out[n].flags = 0U;
                out[n].param = (int32_t)ARM_FIX_S1_RET;
                n++;
            }
        }

        if (n >= (uint8_t)ACT_MAX_STEPS)
        {
            Debug_Log("ACT", "rule3 S1->0 insert SKIPPED (steps full)");
            return;
        }
        out[n].joint = actSteps[i].joint;
        out[n].flags = actSteps[i].flags;
        out[n].param = actSteps[i].param;
        n++;
    }

    for (i = 0U; i < n; i++)
    {
        actSteps[i] = out[i];
    }

    if (n != actStepCount)
    {
        Debug_Log("ACT", "rule3: S1->0 inserted before TURN (%u -> %u steps)",
                  (unsigned int)actStepCount, (unsigned int)n);
    }
    actStepCount = n;
#endif   /* ACT_RULE3_S1_RET_BEFORE_TURN */
}

/**
 * @brief  启动"已展开好的"步骤数组（使能电机 + 复位步号 + 置 RUN）
 * @param  无
 * @retval 1=已启动
 * @note   调用前必须已填好 actSteps[] / actStepCount / actTitle。
 */
static uint8_t Act_Begin(void)
{
    /* ★规则3：**所有转向前，舵机1 先回 0** —— 统一在这里插步，覆盖所有动作组/所有调用者
       （`ArmAct_Run` / `RunPair` / `GotoTtReady` / `GotoTtPos` / `SbPick` / `SbPlace` 都走这里） */
    Act_EnsureS1RetBeforeTurn();

    /* 参数化动作是**明确的动作请求**，这里使能电机（初始化阶段不会走到本函数） */
    Arm_Enable(ARM_ADDR_TURN, 1U);
    Arm_Enable(ARM_ADDR_LIFT, 1U);

    actIdx      = 0U;
    actStepSent = 0U;
    actPairOn   = 0U;             /* ★规则2 的并行标志：新动作从头开始，不许残留 */
    actParN     = 1U;             /* ★并行组步数：同上 */
    actTurnPendIdx = ACT_TURN_PEND_NONE;  /* ★"等 S1 再转向"挂起：不许残留 */
    actLateIdx     = ACT_LATE_NONE;       /* ★"错开启动"挂起：不许残留 */
    actS1GateTick  = 0U;                  /* ★STORE BACK 的"等 S1<500 再转"计时：不许残留 */
    actClawCmdTick = 0U;                  /* ★"S1 最近一次命令时刻"：新动作重新计（不许残留） */
    actCarS3Resent = 0U;                  /* ★S3 的"已重发"标志：不许残留 */
    {
        uint8_t i;
        for (i = 0U; i < (uint8_t)ACT_MAX_STEPS; i++)
        {
            actParPos0[i] = (int32_t)ACT_POS_NONE;
        }
    }
    actSrvBusWaitTick = 0U;       /* ★不继承上一段的"等总线"计时 */
    actStepTick = HAL_GetTick();
    actRunTick  = actStepTick;    /* 统计整段耗时（结束日志用） */
    actUnreached = 0U;            /* 保障层标志：本次动作重新计 */
    actGripFail  = 0U;            /* ★夹爪失败标志：每段动作重新计（调用方在结束时查它） */
    actGripRetry = 0U;
    actGripReopenOn = 0U;
    actAtTtReady = 0U;            /* ★新动作一开始就**不再**处于观察位（GotoTtReady 会重新置 1） */
#if (ACT_MOTOR_SW_LOOP != 0)
    actMotorStallOn      = 0U;    /* 电机闭环状态复位（每步下发时会再初始化一次） */
    actMotorReachIgnored = 0U;
#endif
    /* ★【2026-10-15 修】这里**不再**清两轴的"后台精修"：
       原来"每个新动作开始就取消所有精修"，而任务点里动作是一条接一条下发的
       （TT_/SP_/SB_ 每段一个 ArmAct_Run），于是被提前放行的轴**还没来得及校正就被清掉**
       —— 现场现象就是用户报的"**两个电机的闭环好像都没有了**"。
       现在精修**跨动作存活**；只有"**某一步真的命令了该轴**"时才取消它
       （见 `Act_SendStep()` 里的 `Act_TrimClear`）——
       那一刻该步会自己把轴校到自己的目标，精修没必要、也不该插手。 */
    actState    = ACT_RUN;

    Debug_Log("ACT", "run %s steps=%u", actTitle, (unsigned int)actStepCount);
    return 1U;
}

/**
 * @brief  启动一段动作（内部；已完成标定检查与展开）
 * @param  op : 动作
 * @param  pos: 位置号
 * @retval 1=已启动；0=未启动
 */
static uint8_t Act_StartInternal(ArmOp_t op, uint8_t pos)
{
    if (Act_Build(op, pos) == 0U)
    {
        actState = ACT_ABORT;
        return 0U;
    }

    /* ★【2026-10-21 用户要求】放车载（CAR_PLACE）时**不要先缩到 0**：
       直接把舵机1 缩到"该车载位的标定值"就可以转向（见 flowCarPlace 第 1 步）。
       所以本动作跳过规则3 的"转向前插 S1->0"。 */
    if (op == ARM_OP_CAR_PLACE)
    {
        actSkipRule3 = 1U;
    }

    return Act_Begin();
}

/* ===================== 对外接口 ===================== */

uint8_t ArmAct_Run(ArmOp_t op, uint8_t pos)
{
    if (actState == ACT_RUN) { return 0U; }   /* 已有动作在跑 */
    actPairStage = 0U;
    return Act_StartInternal(op, pos);
}

uint8_t ArmAct_RunPair(ArmOp_t opA, uint8_t posA, ArmOp_t opB, uint8_t posB)
{
    if (actState == ACT_RUN) { return 0U; }
    if ((uint8_t)opB >= (uint8_t)ARM_OP_COUNT) { return 0U; }
    if ((posB < 1U) || (posB > 3U))            { return 0U; }

    /* 先把第二段的目的记下来；第一段正常跑完才启动它 */
    actPendOp  = (uint8_t)opB;
    actPendPos = posB;
    actPairStage = 1U;

    if (Act_StartInternal(opA, posA) == 0U)
    {
        actPairStage = 0U;
        return 0U;
    }
    return 1U;
}

uint8_t ArmAct_GotoTtReady(void)
{
    if (actState == ACT_RUN) { return 0U; }
    actPairStage = 0U;

    /* ★ 这是**全局观察位**（看圆环的姿态）：转盘 / 粗加工 / 精加工 三处定位闭环
       开始前都要先把臂摆到这里，视觉才在同一个观察基准下给偏移量。
       ★【2026-10-10 晚 用户要求】**舵机1 必须在电机5 到位之后再伸出去**
         （原来顺序是 S1(伸) -> S2 -> M5 -> M6：**带着伸出的爪子转大轴**，会扫撞，是错的）。
         当时改成 **S2 -> M5(转) -> M6(抬) -> S1(伸)**，靠规则3 的"转向前插 S1->0"做双保险。 */
    /* ★【2026-10-24 用户要求】"放置完后手臂立马抬升回到观察位，**舵机1 伸出和电机5 转向同时进行**"：
         顺序改为 **S2(松开) -> M6(到观察高) -> [M5 转向 ‖ S1 伸出]**。
         为什么现在敢"边转边伸"：M6 已经先抬到观察位高度（最高处），爪子在空旷区，
         不会像低位时那样扫撞；而且这两步是**并行对**（两个都到位才结束本动作）。
       ★【2026-10-24】本动作**跳过规则3**（不插"转向前的 S1->0"）：否则会把
         "伸出"拆成"先缩回再伸出"，正好破坏用户要的"伸出与转向同时"。 */
    actSteps[0].joint = AJT_GRIP;  actSteps[0].flags = ACT_STEP_NOWAIT; actSteps[0].param = ARM_TT_READY_S2;
    actSteps[1].joint = AJT_LIFT;  actSteps[1].flags = 0U;             actSteps[1].param = ARM_TT_READY_M6;
    actSteps[2].joint = AJT_TURN;  actSteps[2].flags = ACT_STEP_PAR;   actSteps[2].param = ARM_TT_READY_M5;
    actSteps[3].joint = AJT_CLAW;  actSteps[3].flags = 0U;             actSteps[3].param = ARM_TT_READY_S1;
    actStepCount = 4U;
    (void)snprintf(actTitle, sizeof(actTitle), "TTREADY");
    actPos = 0U;                  /* 没有车载号位 */

    actSkipRule3 = 1U;            /* ★不要插"转向前的 S1 缩回"（见上） */

    if (Act_Begin() == 0U)
    {
        return 0U;
    }
    actAtTtReady = 1U;            /* ★本段跑完（且没被别的动作覆盖）就算"在观察位" */
    return 1U;
}

/* "已在观察位"的位置容差（下发脉冲域）：观察位只用于"看"，不需要严格到位，
   所以放宽到严格容差的 3 倍 / 2 倍，避免刚跑完就差几脉冲被判"不在观察位"。 */
#define ACT_READY_TOL_TURN   (ARM_SEQ_TURN_POS_TOL * 3)
#define ACT_READY_TOL_LIFT   (ARM_SEQ_LIFT_POS_TOL * 2)

uint8_t ArmAct_IsAtTtReady(void)
{
    if (actAtTtReady == 0U)                        { return 0U; }  /* 最后下发的不是观察位 */
    if (actState == ACT_RUN)                       { return 0U; }  /* 还在动 */
    if (actUnreached != 0U)                        { return 0U; }  /* 上一段有步骤没确认到 */

    if (Act_NearTarget(ARM_ADDR_TURN, ARM_TT_READY_M5, ACT_READY_TOL_TURN) == 0U) { return 0U; }
    if (Act_NearTarget(ARM_ADDR_LIFT, ARM_TT_READY_M6, ACT_READY_TOL_LIFT) == 0U) { return 0U; }

    return 1U;
}

uint8_t ArmAct_IsAtTtPos(uint8_t pos)
{
    const ArmCalRow_t *row;

    if ((pos < 1U) || (pos > 3U))    { return 0U; }
    if (actState == ACT_RUN)         { return 0U; }  /* 还在动 */

    row = Act_FindCal((uint8_t)ARM_OP_TT_PICK, pos);
    if (Act_IsCalibrated(row) == 0U) { return 0U; }

    /* ★【2026-10-26 用户要求】"请在**手臂到达等待位**再开启等待颜色"：
       判据必须是**回读到位**，不能只看"动作跑完"—— 并行对靠 `ACT_PAIR_GRACE_MS`
       兜底、电机步没到位时也会带着 `actUnreached` 收尾，那时 `actState` 已是 DONE
       而电机5/6 还在走最后一段，正好就是"手臂还在去等待位途中就抓"的现场现象。 */
    if (Act_NearTarget((uint8_t)ARM_ADDR_TURN, row->cal.m5, ACT_READY_TOL_TURN) == 0U) { return 0U; }
    if (Act_NearTarget((uint8_t)ARM_ADDR_LIFT, row->cal.m6, ACT_READY_TOL_LIFT) == 0U) { return 0U; }

    return 1U;
}

uint8_t ArmAct_IsTurning(void)
{
    if (actState != ACT_RUN)          { return 0U; }
    if (actIdx >= actStepCount)       { return 0U; }
    if (actSteps[actIdx].joint != AJT_TURN) { return 0U; }

    return 1U;
}

uint8_t ArmAct_GotoTtPos(uint8_t pos)
{
    const ArmCalRow_t *row;
    uint8_t i;

    if (actState == ACT_RUN) { return 0U; }
    actPairStage = 0U;

    /* 参数取"转盘 3 个位号动作组"（ARM_OP_TT_PICK）对应行：
       M5 = 转向、S1 = 伸爪、M6 = 取料高度；**不取 S2（不夹紧）**。 */
    row = Act_FindCal((uint8_t)ARM_OP_TT_PICK, pos);
    if (Act_IsCalibrated(row) == 0U)
    {
        Debug_Log("ACT", "TTPREP%u NOT calibrated -> refused", (unsigned int)pos);
        actState = ACT_ABORT;
        return 0U;
    }

    for (i = 0U; i < (uint8_t)(sizeof(flowTtPrep) / sizeof(flowTtPrep[0])); i++)
    {
        actSteps[i].joint = flowTtPrep[i].joint;
        actSteps[i].flags = flowTtPrep[i].flags;
        actSteps[i].param = Act_Resolve(flowTtPrep[i].src, &row->cal);
    }
    actStepCount = (uint8_t)(sizeof(flowTtPrep) / sizeof(flowTtPrep[0]));
    (void)snprintf(actTitle, sizeof(actTitle), "TTPREP%u", (unsigned int)pos);
    actPos = 0U;                  /* 没有车载号位 */

    /* ★【2026-10-21 用户要求】观察区 -> 抓取姿态这一段**不要**"先缩回舵机1"：
       现场要的动序是 **直接转向 M5 -> 再伸出 S1**。
       这里让规则3 本次不插步（标志由 Act_EnsureS1RetBeforeTurn 消费一次）。 */
    actSkipRule3 = 1U;

    return Act_Begin();
}

/**
 * @brief  ★放置完成收尾：手臂回到"**车载 slot 号位、不抓**"的待命姿态
 * @param  slot: 车载号位 1~3
 * @retval 1=已启动；0=未启动（号位非法 / 该行未标定 / 已有动作在跑）
 * @note   ★【2026-10-11 用户要求】"粗加工区三个放置完后，手臂回到车载 3 号位不抓的状态"，
 *         顺序固定 **电机6 抬高 -> 舵机1 收回 -> 电机5 转向 -> 舵机2 张开**（见 `flowCarIdle`）。
 *         参数：M5 用统一值 `ARM_CAR_M5`；M6/S1/S2 用固定值；S3 按号位取 `actCarS3[]`。
 *         非阻塞：启动后由 `ArmAct_Update()` 推进，用 `ArmAct_IsBusy()` 判断是否还在动。
 */
uint8_t ArmAct_PreSelectSlot(uint8_t slot)
{
    if ((slot < 1U) || (slot > 3U)) { return 0U; }

    actCarS3Pre     = slot;
    actCarS3PreTick = HAL_GetTick();
    actPreSlotVal   = actCarS3[(uint8_t)(slot - 1U)];

    /* ★【2026-10-24 修"预转没动"】登记目标 + 清零重试计数：
       总线空闲就在这里立刻发一次（多数情况），
       其余由 `Act_PreSlotService()` **按回读核对、没到位就补发**。 */
    actPreSlotOn    = 1U;
    actPreSlotTries = 0U;
    actPreSlotTick  = HAL_GetTick();

    if (Servo_ReqPending() == 0U)
    {
        Servo_BusYieldForMotion();      /* 让出在途读请求 + 把读节拍推到指令之后 */
        Arm_ServoMove(SERVO_ID_TURNTABLE, Act_ServoParam(actPreSlotVal));
        actPreSlotTries = 1U;
        Debug_Log("ACT", "preselect car slot%u S3=%ld sent",
                  (unsigned int)slot, (long)actPreSlotVal);
    }
    else
    {
        Debug_Log("ACT", "preselect car slot%u S3=%ld queued (bus busy)",
                  (unsigned int)slot, (long)actPreSlotVal);
    }
    return 1U;
}

/* ===================== ★ TRAY PICK 专用：两个动作组入口 ===================== */

uint8_t ArmAct_TrayReady(uint8_t slot)
{
    ArmCal_t calNone = { 0, 0, 0, 0, 0 };
    uint8_t  i;

    if (actState == ACT_RUN)                        { return 0U; }
    if ((slot < 1U) || (slot > 3U))                 { return 0U; }
    actPairStage = 0U;

    /* ★"同时舵机3 转到相应车载位号"：走预选机制（回读确认 + 没到位自动补发），
       和手臂动作**同一刻**开始，不用等放料那一步才转。 */
    (void)ArmAct_PreSelectSlot(slot);

    for (i = 0U; i < (uint8_t)(sizeof(flowTrayReady) / sizeof(flowTrayReady[0])); i++)
    {
        actSteps[i].joint = flowTrayReady[i].joint;
        actSteps[i].flags = flowTrayReady[i].flags;
        actSteps[i].param = Act_SrcValue(flowTrayReady[i].src, &calNone, slot);
    }
    actStepCount = (uint8_t)(sizeof(flowTrayReady) / sizeof(flowTrayReady[0]));
    (void)snprintf(actTitle, sizeof(actTitle), "TRAYRDY%u", (unsigned int)slot);
    actPos = slot;

    /* 本流程已显式排序（S1 伸出与 M5 同拍），不要规则3 插"S1->0" */
    actSkipRule3 = 1U;

    if (Act_Begin() == 0U) { return 0U; }

    actAtTtReady = 1U;          /* 跑完（且没被别的动作覆盖）就算"在观察位" */
    return 1U;
}

uint8_t ArmAct_TrayPickPlace(uint8_t slot)
{
    ArmCal_t calNone = { 0, 0, 0, 0, 0 };
    uint8_t  i;

    if (actState == ACT_RUN)                        { return 0U; }
    if ((slot < 1U) || (slot > 3U))                 { return 0U; }
    actPairStage = 0U;

    for (i = 0U; i < (uint8_t)(sizeof(flowTrayPickPlace) / sizeof(flowTrayPickPlace[0])); i++)
    {
        actSteps[i].joint = flowTrayPickPlace[i].joint;
        actSteps[i].flags = flowTrayPickPlace[i].flags;
        actSteps[i].param = Act_SrcValue(flowTrayPickPlace[i].src, &calNone, slot);
    }
    actStepCount = (uint8_t)(sizeof(flowTrayPickPlace) / sizeof(flowTrayPickPlace[0]));
    (void)snprintf(actTitle, sizeof(actTitle), "TRAYPICK%u", (unsigned int)slot);
    actPos = slot;

    return Act_Begin();
}

/**
 * @brief  预选 S3 的"下发 + 核对 + 补发"服务（每拍调用）
 * @param  无
 * @retval 无
 * @note   ★【2026-10-24 修"对位后舵盘不预转"】
 *         原来只发一次，被调试板丢掉就再也不动了。现在：
 *           ① 先等总线静默（没应答在途）才发，避免撞车；
 *           ② 每拍核对 S3 回读，**已在目标 ±`ACT_S3_ARRIVE_TOL`** 就结束；
 *           ③ 没到位就每 `ACT_S3_RESEND_MS`(300ms) **补发一次**，最多 6 次（约 2.5s），
 *              到点仍不到 -> 打 `S3 preset give up` 并交给动作里那一步继续纠。
 *         动作层一旦改发别的角度（`actCarS3Pre` 被清 0），本服务立刻退出，不会互相打架。
 */
static void Act_PreSlotService(void)
{
    uint32_t now = HAL_GetTick();

    if (actPreSlotOn == 0U) { return; }

    /* ★【2026-10-24 用户要求】"务必单独运行舵机2，不要和别的关节同时进行"：
       动作层正在执行**舵机2（抓/放）**步时，本服务一律不发 S3（预选补发也暂停），
       保证夹爪动作期间没有任何别的关节在动；抓取步一过自动恢复核对。 */
    if ((actState == ACT_RUN) &&
        (actIdx < actStepCount) &&
        (actSteps[actIdx].joint == AJT_GRIP))
    {
        return;
    }

    if (actCarS3Pre == 0U)
    {
        actPreSlotOn = 0U;                  /* 动作层已经改发别的角度：预选作废 */
        return;
    }

    if (Act_S3At(actPreSlotVal) != 0U)
    {
        actPreSlotOn = 0U;                  /* 已到位（含"本来就在"）：结束 */
        return;
    }

    if (actPreSlotTries >= 6U)
    {
        actPreSlotOn = 0U;
        Debug_Log("ACT", "S3 preset give up (tgt=%ld)", (long)actPreSlotVal);
        return;
    }
    if ((actPreSlotTries != 0U) && ((now - actPreSlotTick) < (uint32_t)ACT_S3_RESEND_MS))
    {
        return;
    }
    if (Servo_BusQuiet((uint16_t)ACT_SRV_BUS_QUIET_MS) == 0U)
    {
        Servo_BusYieldForMotion();          /* 让出在途读请求，下一拍再看 */
        return;
    }

    Servo_BusYieldForMotion();
    Arm_ServoMove(SERVO_ID_TURNTABLE, Act_ServoParam(actPreSlotVal));
    actPreSlotTick = now;
    actPreSlotTries++;

    Debug_Log("ACT", "S3 preset resend %u/6 (tgt=%ld)",
              (unsigned int)actPreSlotTries, (long)actPreSlotVal);
}

uint8_t ArmAct_GotoCarIdle(uint8_t slot)
{
    ArmCal_t calNone = { 0, 0, 0, 0, 0 };
    uint8_t i;

    if (actState == ACT_RUN) { return 0U; }
    if ((slot < 1U) || (slot > 3U)) { return 0U; }
    actPairStage = 0U;

    for (i = 0U; i < (uint8_t)(sizeof(flowCarIdle) / sizeof(flowCarIdle[0])); i++)
    {
        actSteps[i].joint = flowCarIdle[i].joint;
        actSteps[i].flags = flowCarIdle[i].flags;
        actSteps[i].param = Act_SrcValue(flowCarIdle[i].src, &calNone, slot);
    }
    actStepCount = (uint8_t)(sizeof(flowCarIdle) / sizeof(flowCarIdle[0]));
    (void)snprintf(actTitle, sizeof(actTitle), "CARIDLE%u", (unsigned int)slot);
    actPos = slot;

    return Act_Begin();
}

/**
 * @brief  ★【2026-10-26 用户要求】**收尾把手臂收回初始（待命）状态**
 * @param  无
 * @retval 1=已启动；0=未启动（已有动作在跑）
 * @note   "初始状态" = 上电/出发时的姿态：
 *           **舵机2 张开（空爪） -> 电机6 抬到安全高 ‖ 舵机1 收回 -> 舵盘回 0 -> 电机5 回 0**。
 *         顺序沿用现场已验证的口径：先抬/收（爪子离开作业面），再转舵盘、再摆大臂，
 *         避免带爪旋转扫撞；末段 M5 回 0 = 车头正前，与上电零点一致。
 *         非阻塞：启动后由 `ArmAct_Update()` 推进，用 `ArmAct_IsBusy()` 判断是否还在动。
 */
uint8_t ArmAct_Park(void)
{
    if (actState == ACT_RUN) { return 0U; }
    actPairStage = 0U;

    actSteps[0].joint = AJT_GRIP; actSteps[0].flags = ACT_STEP_NOWAIT; actSteps[0].param = ARM_FIX_S2_OPEN;
    actSteps[1].joint = AJT_LIFT; actSteps[1].flags = ACT_STEP_PAR;    actSteps[1].param = ARM_FIX_M6_SAFE;
    actSteps[2].joint = AJT_CLAW; actSteps[2].flags = 0U;               actSteps[2].param = ARM_FIX_S1_RET;
    actSteps[3].joint = AJT_CAR;  actSteps[3].flags = 0U;               actSteps[3].param = ARM_CAR_S3_HOME;
    actSteps[4].joint = AJT_TURN; actSteps[4].flags = 0U;               actSteps[4].param = 0;  /* 电机5 零点 = 初始朝向 */
    actStepCount = 5U;
    (void)snprintf(actTitle, sizeof(actTitle), "PARK");
    actPos = 0U;                 /* 没有车载号位 */

    actSkipRule3 = 1U;           /* 第 2 步已经是"舵机1 收回"，别再插一步 */

    return Act_Begin();
}

/**
 * @brief  ★【2026-10-24 用户要求】**STORE PLACE 单件连续动作**：车载取件 -> 靶位放置
 * @param  slot    : 车载号位 1~3
 * @param  tgtPos  : 靶位号 1~3
 * @param  keepPose: 1=**最后一个物料**：放完**只松抓就结束**，不收爪、不抬升
 *                   （手臂留在靶位取件姿态，正好接 STORE BACK 的"直接抓"）；
 *                   0=放完 [M6 抬升 ‖ S1 缩回] 同拍，便于回车载抓下一件
 * @retval 1=已启动；0=未启动
 * @note   把原来的 `CAR_PICK` + `TGT_PLACE` **两个动作合并成一条**，才能做到：
 *           · 抓完：**M6 单独抬升到位**（物料离盘，★2026-10-24 用户要求），
 *             然后 **[M5 转向靶位 ‖ S3 回 0]** 同拍；
 *           · 放料：**[S1 伸出 ‖ M6 下降]** 同拍（先到位再落料）；
 *           · 放完：**[M6 抬升 ‖ S1 缩回]** 同拍，随后由下一件的开头
 *             [S1 到车载位 ‖ M5 转车载 ‖ M6 降车载] 接着"边转边降"去抓下一件。
 *         ★【2026-10-23 用户要求】STORE PLACE **最后一个物料松抓就行、手臂不用上台**：
 *           `keepPose=1` 时**不追加**最后两步（抬 M6 + 收 S1），动作到"松手"即结束。
 *         车载姿态用 `ARM_CAR_*`；靶位姿态取 `ARM_OP_TGT_PLACE` 对应行。
 *         规则3（转向前先收爪）本动作**跳过**：S1 由本流程显式安排，
 *         且转向时 S1 已在车载爪位（`ARM_CAR_S1`，接近缩回）/ 缩回 0。
 */
uint8_t ArmAct_RunCar2Tgt(uint8_t slot, uint8_t tgtPos, uint8_t keepPose)
{
    const ArmCalRow_t *row;
    uint8_t n = 0U;

    if (actState == ACT_RUN)                 { return 0U; }
    if ((slot < 1U) || (slot > 3U))          { return 0U; }
    if ((tgtPos < 1U) || (tgtPos > 3U))      { return 0U; }

    row = Act_FindCal((uint8_t)ARM_OP_TGT_PLACE, tgtPos);
    if (Act_IsCalibrated(row) == 0U)
    {
        Debug_Log("ACT", "C2T%u NOT calibrated -> refused", (unsigned int)tgtPos);
        actState = ACT_ABORT;
        return 0U;
    }

    actPairStage = 0U;

    actSteps[n].joint = AJT_CAR;  actSteps[n].flags = 0U;              actSteps[n].param = actCarS3[(uint8_t)(slot - 1U)]; n++;
    actSteps[n].joint = AJT_CLAW; actSteps[n].flags = ACT_STEP_PAR | ACT_STEP_FORCE;
                                                                       actSteps[n].param = ARM_CAR_S1;      n++;
    actSteps[n].joint = AJT_TURN; actSteps[n].flags = ACT_STEP_PAR;    actSteps[n].param = ARM_CAR_M5;      n++;
    actSteps[n].joint = AJT_LIFT; actSteps[n].flags = 0U;              actSteps[n].param = ARM_CAR_M6;      n++;
    actSteps[n].joint = AJT_GRIP; actSteps[n].flags = 0U;              actSteps[n].param = ARM_FIX_S2_CLOSE; n++;
    /* ★★【2026-10-24 用户要求】"优先保证抓住物料，再进行别的动作"：
       抓到后 **M6 单独抬升**（不并任何关节）——物料还躺在舵盘号位里时
       转向/转舵盘会把它从爪里拽掉（现场"没抓住就动下一个关节"）。
       必须等 M6 **抬升到位**（物料彻底离盘）后才允许别的关节动。 */
    actSteps[n].joint = AJT_LIFT; actSteps[n].flags = 0U;              actSteps[n].param = ARM_FIX_M6_SAFE; n++;
    /* ★【2026-10-24 用户要求】"等我手臂拿起物料了（电机6 抬升到位）再转舵机3"：
       S3 回 0 从原来的"与 M6 同拍"改为**排在 M6 到位之后**，与 M5 转向同拍。 */
    actSteps[n].joint = AJT_TURN; actSteps[n].flags = ACT_STEP_PAR;    actSteps[n].param = row->cal.m5;     n++;
    actSteps[n].joint = AJT_CAR;  actSteps[n].flags = 0U;              actSteps[n].param = ARM_CAR_S3_HOME; n++;
    actSteps[n].joint = AJT_CLAW; actSteps[n].flags = ACT_STEP_PAR;    actSteps[n].param = row->cal.s1;     n++;
    actSteps[n].joint = AJT_LIFT; actSteps[n].flags = 0U;              actSteps[n].param = row->cal.m6;     n++;
    actSteps[n].joint = AJT_GRIP; actSteps[n].flags = ACT_STEP_NOWAIT; actSteps[n].param = row->cal.s2;     n++;

    /* ★最后一个物料：松抓即结束（手臂留在靶位姿态，接 STORE BACK 的"直接抓"） */
    if (keepPose == 0U)
    {
        actSteps[n].joint = AJT_LIFT; actSteps[n].flags = ACT_STEP_PAR; actSteps[n].param = ARM_FIX_M6_SAFE; n++;
        actSteps[n].joint = AJT_CLAW; actSteps[n].flags = 0U;           actSteps[n].param = ARM_FIX_S1_RET;  n++;
    }

    actStepCount = n;
    (void)snprintf(actTitle, sizeof(actTitle), "C2T%u-%u%s",
                   (unsigned int)slot, (unsigned int)tgtPos,
                   (keepPose != 0U) ? "-L" : "");
    actPos = slot;                       /* 车载号位（S3 预选/到位判定用） */

    actSkipRule3 = 1U;                   /* S1 由本流程显式安排，不插"转向前收爪" */

    return Act_Begin();
}

/**
 * @brief  ★STORE BACK 第①段：靶位**直接抓** -> 上升（提前回缩 + 舵盘转位）-> 转向车载
 * @param  tgtPos: 靶位号 1~3（参数取 `ARM_OP_TGT_PICK` 行）
 * @param  slot  : 车载号位 1~3（舵机3 转到该号位）
 * @param  atPose: 1=手臂已经在本靶位取件姿态（紧跟 STORE PLACE / 上一件的第⑥步）-> 直接抓；
 *                 0=**独立跑**，先用 `flowSbPrep` 把手臂摆到靶位姿态
 * @retval 1=已启动；0=未启动
 * @note   动序见 `flowSbPick`：抓 -> 上升(±10000 提前放行) -> 回缩+S3(同拍) -> 转向车载
 *         (S1<500 才发，±100 提前放行)。
 *         结尾停在"朝向车载、高度已在安全高附近"，由 `ArmAct_SbPlace` 接着下降放料。
 */
uint8_t ArmAct_SbPick(uint8_t tgtPos, uint8_t slot, uint8_t atPose)
{
    const ArmCalRow_t *row;
    uint8_t n = 0U;
    uint8_t i;

    if (actState == ACT_RUN) { return 0U; }
    if ((tgtPos < 1U) || (tgtPos > 3U)) { return 0U; }
    if ((slot   < 1U) || (slot   > 3U)) { return 0U; }
    actPairStage = 0U;

    /* 标定值取"靶位取件"那一组（与 TGT_PICK 完全一样，不用另标） */
    row = Act_FindCal((uint8_t)ARM_OP_TGT_PICK, tgtPos);
    if (Act_IsCalibrated(row) == 0U)
    {
        Debug_Log("ACT", "SBPICK%u NOT calibrated -> refused", (unsigned int)tgtPos);
        actState = ACT_ABORT;
        return 0U;
    }

    /* 独立跑：先把手臂摆到本靶位取件姿态（转向 -> 舵盘回0 -> 伸出 -> 下降） */
    if (atPose == 0U)
    {
        for (i = 0U; i < (uint8_t)(sizeof(flowSbPrep) / sizeof(flowSbPrep[0])); i++)
        {
            actSteps[n].joint = flowSbPrep[i].joint;
            actSteps[n].flags = flowSbPrep[i].flags;
            actSteps[n].param = Act_SrcValue(flowSbPrep[i].src, &row->cal, slot);
            n++;
        }
    }

    for (i = 0U; i < (uint8_t)(sizeof(flowSbPick) / sizeof(flowSbPick[0])); i++)
    {
        actSteps[n].joint = flowSbPick[i].joint;
        actSteps[n].flags = flowSbPick[i].flags;
        actSteps[n].param = Act_SrcValue(flowSbPick[i].src, &row->cal, slot);
        n++;
    }
    actStepCount = n;
    (void)snprintf(actTitle, sizeof(actTitle), "SBPICK%u-%u",
                   (unsigned int)(tgtPos), (unsigned int)slot);
    actPos = slot;                /* 车载号位：S3 取角 / 预选到位判定用 */

    actSkipRule3 = 1U;            /* S1 由本流程显式安排，不再插"转向前收爪" */

    return Act_Begin();
}

/**
 * @brief  ★STORE BACK 第②段：下降 -> 松手 -> 顺手摆好下一件（或收尾）
 * @param  slot   : 车载号位 1~3（本次放料的位置）
 * @param  nextTgt: 下一件的靶位号 1~3；**0 = 这是最后一件**
 * @retval 1=已启动；0=未启动
 * @note   `flowSbPlace`：M6 降到车载高度 -> 松手（★静置 `ACT_PLACE_SETTLE_MS` 等料落稳）
 *         -> M6 抬回安全高（★抬臂之后才允许舵盘回 0）。
 *         之后按 `nextTgt` 二选一：
 *           · 有下一件 -> `flowSbNext`：转向下一件 ‖ 舵盘回 0，然后伸出 ‖ 下降
 *             （下一件因此从"伸到位、取件高度、爪子张开"开始，接 `ArmAct_SbPick` 的第①步）；
 *           · 最后一件 -> `flowSbLast`：抬到安全高 + 收爪（空爪离作业面，之后底盘才能动车）。
 */
uint8_t ArmAct_SbPlace(uint8_t slot, uint8_t nextTgt)
{
    const ArmCalRow_t *nrow = 0;
    ArmCal_t calNone = { 0, 0, 0, 0, 0 };
    uint8_t n = 0U;
    uint8_t i;

    if (actState == ACT_RUN) { return 0U; }
    if ((slot < 1U) || (slot > 3U)) { return 0U; }
    if (nextTgt > 3U)               { return 0U; }
    actPairStage = 0U;

    if (nextTgt != 0U)
    {
        nrow = Act_FindCal((uint8_t)ARM_OP_TGT_PICK, nextTgt);
        if (Act_IsCalibrated(nrow) == 0U)
        {
            Debug_Log("ACT", "SBNEXT%u NOT calibrated -> refused", (unsigned int)nextTgt);
            actState = ACT_ABORT;
            return 0U;
        }
    }

    /* ④⑤ 降到车载高度 -> 松手 */
    for (i = 0U; i < (uint8_t)(sizeof(flowSbPlace) / sizeof(flowSbPlace[0])); i++)
    {
        actSteps[n].joint = flowSbPlace[i].joint;
        actSteps[n].flags = flowSbPlace[i].flags;
        actSteps[n].param = Act_SrcValue(flowSbPlace[i].src, &calNone, slot);
        n++;
    }

    if (nextTgt != 0U)
    {
        /* ⑥ 转向下一件 + 舵盘回 0 -> 伸出 + 下降（参数取**下一件**的标定行） */
        for (i = 0U; i < (uint8_t)(sizeof(flowSbNext) / sizeof(flowSbNext[0])); i++)
        {
            actSteps[n].joint = flowSbNext[i].joint;
            actSteps[n].flags = flowSbNext[i].flags;
            actSteps[n].param = Act_SrcValue(flowSbNext[i].src, &nrow->cal, slot);
            n++;
        }
    }
    else
    {
        /* 最后一件：抬安全高 + 收爪 */
        for (i = 0U; i < (uint8_t)(sizeof(flowSbLast) / sizeof(flowSbLast[0])); i++)
        {
            actSteps[n].joint = flowSbLast[i].joint;
            actSteps[n].flags = flowSbLast[i].flags;
            actSteps[n].param = Act_SrcValue(flowSbLast[i].src, &calNone, slot);
            n++;
        }
    }
    actStepCount = n;
    (void)snprintf(actTitle, sizeof(actTitle), "SBPUT%u>%u",
                   (unsigned int)slot, (unsigned int)nextTgt);
    actPos = slot;

    actSkipRule3 = 1U;            /* S1 由本流程显式安排 */

    return Act_Begin();
}

/**
 * @brief  取关节名（日志用）
 * @param  joint: ActJoint_t
 * @retval "TURN" / "LIFT" / "CLAW" / "GRIP"
 */
static const char *Act_JointName(uint8_t joint)
{
    switch (joint)
    {
        case AJT_TURN: return "TURN";
        case AJT_LIFT: return "LIFT";
        case AJT_CLAW: return "CLAW";
        case AJT_CAR:  return "S3";
        default:       return "GRIP";
    }
}

/**
 * @brief  电机步下发前，确保"当前位置"可读
 * @param  st: 当前步骤
 * @retval 1=可以下发；0=还没就绪（继续等，或已因超时中止）
 * @note   **不这样做就会"命令发了电机不动"**：Arm_MoveAbsDir 在没有当前位置时
 *         直接 return（且拒绝日志只打一次）。这里主动 Arm_ReadPos() 催一次，
 *         超过 ACT_POS_WAIT_MS 仍未读到就中止并明确报错。
 */
static uint8_t Act_PosReady(const ActStep_t *st)
{
    uint8_t addr = (st->joint == AJT_TURN) ? ARM_ADDR_TURN : ARM_ADDR_LIFT;

    if (Arm_IsPosValid(addr) != 0U)
    {
        return 1U;
    }

    if (actPosWaitOn == 0U)
    {
        actPosWaitOn   = 1U;
        actPosWaitTick = HAL_GetTick();
        Debug_Log("ACT", "step%u %s waiting pos ...",
                  (unsigned int)(actIdx + 1U), Act_JointName(st->joint));
    }
    else if ((HAL_GetTick() - actPosWaitTick) > ACT_POS_WAIT_MS)
    {
        /* 位置一直读不到：多半是电机没使能 / 驱动器 Response 未设成 Reached|Both /
           或电机的状态轮询被别的模块占满。
           ★【2026-10-23 用户要求】"不要 FAIL，没达成就下一步" -> **跳过本步**（告警），
           不再中止整段动作。 */
        actUnreached = 1U;
        Debug_Log("ACT", "step%u %s no pos %ums -> skip",
                  (unsigned int)(actIdx + 1U), Act_JointName(st->joint),
                  (unsigned int)ACT_POS_WAIT_MS);
        actPosWaitOn = 0U;
        Act_Advance();
        return 0U;
    }

    Arm_ReadPos(addr);   /* 主动催一次位置（周期调用，别干等） */
    return 0U;
}

/**
 * @brief  ★【2026-10-23】"错开启动"到点补发（每拍调用）
 * @param  无
 * @retval 无
 * @note   `ACT_STEP_LATE_START` 标出的那一步在下发时被挂起（`actLateIdx`），
 *         过了 `ACT_LATE_START_MS` 在这里发出去；发出去后**继续把紧邻、且上一步标了
 *         `ACT_STEP_PAR` 的步一起发**（★2026-10-24 起 SB_ 里 S3 已改为"M6 到位后单独转"，
 *         不再进本链），遇到 TURN 就停（转向还要走 S1<500 闸门，交给主路径）。 */
static void Act_LateStartKick(void)
{
    uint8_t  idx;
    uint8_t  n;
    uint32_t now = HAL_GetTick();

    if (actLateIdx == ACT_LATE_NONE)
    {
        return;
    }
    if ((actState != ACT_RUN) || (actLateIdx >= actStepCount) || (actLateIdx <= actIdx))
    {
        actLateIdx = ACT_LATE_NONE;      /* 已经跨过 / 中止：作废 */
        return;
    }
    if ((now - actLateTick) < (uint32_t)ACT_LATE_START_MS)
    {
        return;
    }

    idx = actLateIdx;
    actLateIdx = ACT_LATE_NONE;
    n = (uint8_t)((idx - actIdx) + 1U);

    /* 记下发前轴位置（电机步的到位帧可信校验要用） */
    if (actSteps[idx].joint == AJT_TURN)
    {
        actParPos0[idx] = Arm_GetPos((uint8_t)ARM_ADDR_TURN);
    }
    else if (actSteps[idx].joint == AJT_LIFT)
    {
        actParPos0[idx] = Arm_GetPos((uint8_t)ARM_ADDR_LIFT);
    }

    Act_SendStep(&actSteps[idx]);
    Debug_Log("ACT", "step%u %s late-start %ums -> sent",
              (unsigned int)(idx + 1U), Act_JointName(actSteps[idx].joint),
              (unsigned int)ACT_LATE_START_MS);

    /* 继续串"上一步标了 PAR"的后续步（不串 TURN：它有自己的闸门） */
    while (((uint8_t)(actIdx + n) < actStepCount) &&
           ((actSteps[actIdx + n - 1U].flags & ACT_STEP_PAR) != 0U))
    {
        uint8_t jn = actSteps[actIdx + n].joint;

        if (jn == AJT_TURN)
        {
            break;
        }
        if (jn == AJT_LIFT)
        {
            if (Arm_IsPosValid((uint8_t)ARM_ADDR_LIFT) == 0U)
            {
                break;
            }
            actParPos0[actIdx + n] = Arm_GetPos((uint8_t)ARM_ADDR_LIFT);
        }
        Act_SendStep(&actSteps[actIdx + n]);
        n++;
    }

    actParN   = n;
    actPairOn = 1U;
}

/**
 * @brief  ★【2026-10-24 用户要求】"等舵机1 收到 500 以下再发转向"
 * @param  无
 * @retval 无
 * @note   每拍调用（`ArmAct_Update` 里紧跟后台精修）。只在并行组被"闸门挂起"
 *         （`actTurnPendIdx != 0xFF`）时工作：
 *           · S1 回读 < `ACT_TURN_S1_GATE`(500) -> **现在发** TURN，组扩成 3 步；
 *           · 读不到 / 到 `ACT_TURN_GATE_MS`(400ms) 还没降下来 -> **兜底强发**（绝不卡死）。
 *         发出后把"位置快查"还给当前步的电机（否则 M6 的位置又回到 120ms 轮询，
 *         严格闭环会误判没到位、白等 3s）。
 */
static void Act_TurnGateKick(void)
{
    uint8_t  idx = actTurnPendIdx;
    uint16_t curS1 = 0U;
    uint8_t  posOk = 0U;
    uint8_t  send = 0U;

    if (idx == ACT_TURN_PEND_NONE) { return; }
    if ((actState != ACT_RUN) || (idx >= actStepCount) || (idx <= actIdx))
    {
        actTurnPendIdx = ACT_TURN_PEND_NONE;   /* 已经跨过 / 中止：作废 */
        return;
    }

    /* ★【2026-10-29】统一用 `Act_S1BelowGate()`（含"回读必须晚于缩回命令"的新鲜度判据），
       posOk 只用于选择兜底超时（有回读 2000ms / 读不到 400ms）。 */
    posOk = Servo_GetPos((uint8_t)SERVO_ID_CLAW, &curS1);
    if (Act_S1BelowGate() != 0U)
    {
        send = 1U;
    }
    else if (posOk != 0U)
    {
        if ((HAL_GetTick() - actTurnPendTick) >= (uint32_t)ACT_TURN_GATE_MS)
        {
            /* 回读可用但 S1 一直降不下来（真的顶住了）：2000ms 上限，宁可慢也不无限等 */
            send = 1U;
        }
    }
    else if ((HAL_GetTick() - actTurnPendTick) >= (uint32_t)ACT_TURN_GATE_NOPOS_MS)
    {
        send = 1U;                             /* 读不到回读：400ms 兜底，不能卡死 */
    }
    if (send == 0U) { return; }

    actTurnPendIdx = ACT_TURN_PEND_NONE;
    actParPos0[idx] = Arm_GetPos((uint8_t)ARM_ADDR_TURN);
    Act_SendStep(&actSteps[idx]);
    if ((uint8_t)((idx - actIdx) + 1U) > actParN)
    {
        actParN = (uint8_t)((idx - actIdx) + 1U);   /* 组扩成含 TURN 的 3 步 */
    }

    /* 把位置快查还给当前步的电机（TURN 的 SetFastPoll 会把它抢走） */
    if ((actSteps[actIdx].joint == AJT_TURN) || (actSteps[actIdx].joint == AJT_LIFT))
    {
        Arm_SetFastPoll((actSteps[actIdx].joint == AJT_TURN) ? (uint8_t)ARM_ADDR_TURN
                                                             : (uint8_t)ARM_ADDR_LIFT);
    }

    Debug_Log("ACT", "S1=%u < %u -> TURN %ld sent, group %u steps",
              (unsigned int)curS1, (unsigned int)ACT_TURN_S1_GATE,
              (long)actSteps[idx].param, (unsigned int)actParN);
}

void ArmAct_Update(void)
{
    const ActStep_t *st;
    uint32_t now;

    /* ★【2026-10-24】"对位后预转 S3"：等总线静默再发（避免被 9600 一问一答丢掉） */
    Act_PreSlotService();

    /* ★ 后台精修（"提前放行"配套）：**必须最先跑**，且不受下面 actState 提前 return 的影响 ——
       被提前放行的轴在动作 DONE 之后仍要把自己校到严格容差（见 Act_TrimService）。 */
    Act_TrimService(HAL_GetTick());

    /* ★【2026-10-24】"S1 收到 500 以下再转向"的闸门：每拍检查 */
    Act_TurnGateKick();

    /* ★【2026-10-23】"错开启动"（M6 先动、舵机1 晚 500ms）到点补发 */
    Act_LateStartKick();

#if (ACT_SERVO_OPEN_LOOP != 0U)
    /* ★【2026-10-15】后台"舵机校验/补发"同样**最先跑**、不受下面 actState 提前 return 影响：
       舵机步一放行、下一个关节一动，就在这里继续核对"到底到没到 / 命令丢没丢"。 */
    Act_SrvVerifyService(HAL_GetTick());
#endif

    /* ---- 两段动作的衔接：段A 结束就启动段B ---- */
    if ((actPairStage == 1U) && (actState != ACT_RUN))
    {
        if (actState == ACT_ABORT)
        {
            actPairStage = 0U;               /* 第一段就失败了，不再启动第二段 */
        }
        else
        {
            actPairStage = 2U;
        }
    }
    else if (actPairStage == 2U)
    {
        actPairStage = 0U;
        if (Act_StartInternal((ArmOp_t)actPendOp, actPendPos) != 0U)
        {
            actPairStage = 3U;               /* 进入段B */
        }
    }
    else if ((actPairStage == 3U) && (actState != ACT_RUN))
    {
        actPairStage = 0U;                   /* 段B 结束（成功或中止） */
    }

    /* ---- 单段执行状态机 ---- */
    if (actState != ACT_RUN) { return; }
    if (actIdx >= actStepCount) { actState = ACT_DONE; return; }

    now = HAL_GetTick();
    st  = &actSteps[actIdx];

    /* ★★【2026-10-24 用户要求】"**这里不要犹豫**"：
       本步目标**现在就已经满足** -> 直接跨步，不再走"等总线静默 -> 下发 -> 等到位帧/回读"。
       典型：TRAY PICK 抓完料时 S1 已缩到 0、M5 已在车载位、S3 已预选到位，
       紧接着的 CAR_PLACE 开头几步都是空跑 —— 去掉它们，**下一步立刻就是 M6 下降**。 */
    if ((actStepSent == 0U) && (Act_StepAlreadyThere(st) != 0U))
    {
        Debug_Log("ACT", "step%u/%u %s already at %ld -> next",
                  (unsigned int)(actIdx + 1U), (unsigned int)actStepCount,
                  Act_JointName(st->joint), (long)st->param);
        Act_Advance();
        return;
    }

    if (actStepSent == 0U)
    {
        /* ★【2026-10-23 · STORE BACK 用户要求】"舵机1 的值小于 500 才开始提前转向"
           （本来就 < 500 -> **直接一起发**，不额外等）。
           本组的 TURN 前面隔着一步 CAR（舵盘转位），走不到全局"CLAW 紧邻 TURN"的闸门，
           所以由模板给 TURN 步标 `ACT_STEP_S1_GATE`，在这里判。
           兜底：S1 回读可用等 `ACT_TURN_GATE_MS`(2000ms)，读不到等 `ACT_TURN_GATE_NOPOS_MS`(400ms)。 */
        if (((st->flags & ACT_STEP_S1_GATE) != 0U) && (st->joint == AJT_TURN))
        {
            uint16_t curS1 = 0U;
            uint8_t  posOk = Servo_GetPos((uint8_t)SERVO_ID_CLAW, &curS1);
            /* ★【2026-10-29】统一用 `Act_S1BelowGate()`：它额外要求"回读必须晚于本次
               动作给 S1 下发的缩回命令"，否则拿伸出之前的旧值（<500）提前放行转向。 */
            uint8_t  ready = Act_S1BelowGate();

            if (ready == 0U)
            {
                if (actS1GateTick == 0U)
                {
                    actS1GateTick = now;
                    Debug_Log("ACT", "step%u TURN gated: wait S1 < %u (now %u)",
                              (unsigned int)(actIdx + 1U),
                              (unsigned int)ACT_TURN_S1_GATE, (unsigned int)curS1);
                }
                if ((now - actS1GateTick) <
                    (uint32_t)((posOk != 0U) ? ACT_TURN_GATE_MS : ACT_TURN_GATE_NOPOS_MS))
                {
                    return;
                }
            }
            actS1GateTick = 0U;
        }

#if (ACT_SERVO_OPEN_LOOP != 0U)
        /* ★★【2026-10-28 修"抓空"】**抓取闸门**：夹紧步下发前，先等舵机1（伸缩）
           的**上一条命令真正到位**（后台校验 `pending` 收口 = 回读已进 LOOSE_TOL 或已放弃）。
           现场：`TTPREP2` 的 S1 243→898 被 120ms 提前放行，紧接着 `TRAYPICK` 第 1 步
           就发夹紧，此时爪子只伸到 797 —— 夹爪在物料**前面**闭合 = 抓空。
           兜底 `ACT_GRAB_GATE_MS`(900ms)：读不到 / 校验放弃就照发，绝不卡死。 */
        if ((st->joint == AJT_GRIP) && (st->param < (int32_t)ACT_S2_OPEN_MIN) &&
            (actSrvVerify[Act_SrvVerifyIdx((uint8_t)SERVO_ID_CLAW)].pending != 0U))
        {
            if (actGrabGateTick == 0U)
            {
                actGrabGateTick = now;
                Debug_Log("ACT", "step%u GRIP gated: wait S1 arrive before close",
                          (unsigned int)(actIdx + 1U));
            }
            if ((now - actGrabGateTick) < (uint32_t)ACT_GRAB_GATE_MS)
            {
                return;
            }
            Debug_Log("ACT", "step%u GRIP gate timeout %ums -> close anyway",
                      (unsigned int)(actIdx + 1U),
                      (unsigned int)(now - actGrabGateTick));
        }
        actGrabGateTick = 0U;
#endif
        /* 电机步：先确保"当前位置"可读，否则 Arm_MoveAbsDir 会拒绝下发
           （表现就是"命令发了、电机一点都不动"）。舵机不需要位置。 */
        if (((st->joint == AJT_TURN) || (st->joint == AJT_LIFT)) &&
            (Act_PosReady(st) == 0U))
        {
            return;
        }

        /* 记下"命令发出前的位置"：丢帧看门狗 + 闭环"停稳"判断都以它为初值。
           舵机步没有位置，填哨兵值（舵机有自己的"回读有没有变过"判据）。 */
        if ((st->joint == AJT_TURN) || (st->joint == AJT_LIFT))
        {
            uint8_t maddr = (st->joint == AJT_TURN) ? (uint8_t)ARM_ADDR_TURN
                                                    : (uint8_t)ARM_ADDR_LIFT;

            actStepPos0 = Arm_GetPos(maddr);
#if (ACT_MOTOR_SW_LOOP != 0)
            /* 闭环状态复位：以"命令前位置"作停稳观察初值，计时从下发时刻起算 */
            actMotorLastPos      = actStepPos0;
            actMotorStallTick    = now;
            actMotorStallOn      = 0U;
            actMotorReachIgnored = 0U;
#endif
            /* ★"本步要直接驱动该轴 -> 取消后台精修"这一动作**已移到 `Act_SendStep()`**
               （唯一下发口，覆盖 主分支/并行对/两段一起发 三条路，见那里说明）。 */
        }
        else
        {
            actStepPos0 = ACT_POS_NONE;
        }

        /* ===== ★【2026-10-11 根治"舵机丢步"】下发舵机命令前，先等 9600 总线静默 =====
           舵机运动指令(调试板 Cmd 0x03)与位置回读请求(Cmd 0x15)**共用同一条** USART3，
           调试板是"一问一答"设备 —— 应答没结束时插进去的运动指令**会被丢掉**（= 该关节整步没执行）。
           ★【2026-10-24 修】等待期间**不再 `Servo_BusYieldForMotion()`**：
             一 yield 就把 `servoReqPending` 清成 0，"静默"是假的 —— 调试板其实还在回那一帧，
             40ms 后发出的运动指令正好撞上应答尾部，现场就是"**舵机偶尔不执行**"。
             现在改成：**等读请求自然结束**（它自己会超时/重试，最长 3×50ms），
             真正等不到（板子异常）才在硬上限 `ACT_SRV_BUS_WAIT_HARD_MS` 后 yield + 强发。 */
        if ((st->joint == AJT_CLAW) || (st->joint == AJT_GRIP) || (st->joint == AJT_CAR))
        {
            if (Servo_BusQuiet((uint16_t)ACT_SRV_BUS_QUIET_MS) == 0U)
            {
                if (actSrvBusWaitTick == 0U)
                {
                    actSrvBusWaitTick = now;
                }
                if ((now - actSrvBusWaitTick) < (uint32_t)ACT_SRV_BUS_WAIT_HARD_MS)
                {
                    return;                 /* 还在发/收：下一拍再看（绝不插队） */
                }
                Servo_BusYieldForMotion();  /* 板子异常（一直不结束）：让出并强发，避免卡死 */
                Debug_Log("ACT", "step%u %s bus stuck %ums -> yield + send anyway",
                          (unsigned int)(actIdx + 1U), Act_JointName(st->joint),
                          (unsigned int)(now - actSrvBusWaitTick));
            }
        }
        actSrvBusWaitTick = 0U;             /* 本步不再等总线 */

        Act_SendStep(st);
        actStepSent  = 1U;
        actStepTick  = now;
        actPosWaitOn = 0U;
#if (ACT_MOTOR_SW_LOOP != 0)
        actRetx      = 0U;      /* 本步重发 / 补发 / 纠正次数清零 */
#endif

        /* 每一步都打日志：现场直接看出**真实执行顺序**（排查"没按顺序到位"用） */
        Debug_Log("ACT", "step%u/%u %s %ld",
                  (unsigned int)(actIdx + 1U), (unsigned int)actStepCount,
                  Act_JointName(st->joint), (long)st->param);

        /* ===== ★【规则2】安全高度以上：舵机1(缩回) 与 电机5 并行 =====
           完成判定由 Act_Advance 保证"**两个都到位**"才跨步。
           ★【2026-10-11 用户要求】若这一步是"**舵机1 缩回 -> 电机5 转向**"（规则3 插步后必然如此），
             就**先只发缩回**，把转向**挂起**到"**舵机1 的值回落到 ≤ ACT_PAIR_TURN_S1_HI（默认 20，即 0~20）**"
             再发（见 Act_PairTurnKick；回读异常/超时也会兜底发出，绝不卡死）。
           不能并行（高度不够 / 位置不可读 / 不是那一对 / 那步是"伸出"）就照原样一步步走。 */
        actPairOn       = 0U;
        actParN         = 1U;       /* 每一步下发时先按"没有并行组"处理 */
        actPairTurnPend = 0U;       /* 每一步下发时先清"转向挂起"，防上一段残留 */
        if (Act_PairOk(actIdx) != 0U)
        {
            if (((uint8_t)ACT_PAIR_TURN_HALF != 0U) &&
                (st->joint == AJT_CLAW) && (actSteps[actIdx + 1U].joint == AJT_TURN))
            {
                actPairTurnPend = 1U;
                Debug_Log("ACT", "pair: wait S1 <= %u before TURN tgt=%ld",
                          (unsigned int)ACT_PAIR_TURN_S1_HI,
                          (long)st->param);
            }
            else
            {
                /* ★【2026-10-24】把**后续相邻、且前一步标了 `ACT_STEP_PAR`** 的步
                   全部在同一拍发下去 —— 支持 **2 步或 3 步**并行组
                   （例：抓取后的"抬升 M6 + 缩回 S1 + 转向 M5"三步同拍）。 */
                uint8_t n = 2U;     /* 第一对：第二步一定发（含旧的 S1+M5 规则） */
                uint8_t j = actSteps[actIdx + 1U].joint;

                /* ★【2026-10-23 用户要求 · STORE BACK】"电机6 与舵机1 错开 500ms"：
                   下一步标了 `ACT_STEP_LATE_START` -> **本拍只发当前步**（M6 上升），
                   那一步挂起，由 `Act_LateStartKick()` 到点再发（两者仍然并行）。 */
                if ((actSteps[actIdx + 1U].flags & ACT_STEP_LATE_START) != 0U)
                {
                    actLateIdx  = (uint8_t)(actIdx + 1U);
                    actLateTick = now;
                    actPairOn   = 1U;   /* 组里还有一步待发：Act_Advance 会被挡住 */
                    actParN     = 1U;
                    Debug_Log("ACT", "step%u %s late-start %ums (M6 first)",
                              (unsigned int)(actIdx + 2U), Act_JointName(j),
                              (unsigned int)ACT_LATE_START_MS);
                    return;
                }

                /* ★★【2026-10-24 用户要求 · 全局闸门】
                   "**舵机1 未 < 500，电机5 不能转向**" —— 对**所有**"先发 S1、紧接 TURN"的对生效
                   （规则3 插出来的 S1 缩回、放车载的 S1 缩、抓完料的 S1 缩…）。
                   此刻 S1 回读还是**命令前**的值：本来就 <500 才允许同拍，否则只发 S1，
                   TURN 交给 `Act_TurnGateKick` 等它降下来再发。 */
                if ((st->joint == AJT_CLAW) && (j == AJT_TURN) &&
                    (Act_S1BelowGate() == 0U))
                {
                    uint16_t curS1 = 0U;

                    (void)Servo_GetPos((uint8_t)SERVO_ID_CLAW, &curS1);

                    actTurnPendIdx  = (uint8_t)(actIdx + 1U);
                    actTurnPendTick = HAL_GetTick();
                    actPairOn       = 1U;   /* 组里还有一步待发（`Act_Advance` 会被 pend 挡住） */
                    actParN         = 1U;
                    Debug_Log("ACT", "TURN deferred: wait S1 < %u (now %u)",
                              (unsigned int)ACT_TURN_S1_GATE, (unsigned int)curS1);
                    return;
                }

                if ((j == AJT_TURN) || (j == AJT_LIFT))
                {
                    /* 记下该步命令前的轴位置：给它自己的到位帧做可信校验 */
                    actParPos0[actIdx + 1U] = Arm_GetPos((j == AJT_TURN)
                                                         ? (uint8_t)ARM_ADDR_TURN
                                                         : (uint8_t)ARM_ADDR_LIFT);
                }
                Act_SendStep(&actSteps[actIdx + 1U]);

                /* 继续往后串：只有"上一步显式标了 PAR"才允许再并一步 */
                while (((uint8_t)(actIdx + n) < actStepCount) &&
                       ((actSteps[actIdx + n - 1U].flags & ACT_STEP_PAR) != 0U))
                {
                    uint8_t jn = actSteps[actIdx + n].joint;

                    /* ★同上：串到 TURN 且（前一步是舵机1 **或** 该 TURN 步标了 S1 闸门）
                       -> 都走闸门（STORE BACK 的 TURN 前面隔着一步 S3，靠步上的标志命中） */
                    if ((jn == AJT_TURN) &&
                        (((actSteps[actIdx + n - 1U].joint == AJT_CLAW) ||
                          ((actSteps[actIdx + n].flags & ACT_STEP_S1_GATE) != 0U))) &&
                        (Act_S1BelowGate() == 0U))
                    {
                        uint16_t curS1 = 0U;

                        (void)Servo_GetPos((uint8_t)SERVO_ID_CLAW, &curS1);

                        actTurnPendIdx  = (uint8_t)(actIdx + n);
                        actTurnPendTick = HAL_GetTick();
                        Debug_Log("ACT", "TURN deferred: wait S1 < %u (now %u)",
                                  (unsigned int)ACT_TURN_S1_GATE, (unsigned int)curS1);
                        break;
                    }

                    if ((jn == AJT_TURN) || (jn == AJT_LIFT))
                    {
                        uint8_t a = (jn == AJT_TURN) ? (uint8_t)ARM_ADDR_TURN
                                                     : (uint8_t)ARM_ADDR_LIFT;

                        if (Arm_IsPosValid(a) == 0U) { break; }
                        actParPos0[actIdx + n] = Arm_GetPos(a);
                    }
                    Act_SendStep(&actSteps[actIdx + n]);
                    n++;
                }

                actParN   = n;              /* 组内总步数（含当前步） */
                actPairOn = 1U;

                /* ★★【2026-10-24 修】`Act_SendStep` 每发一个电机步都会 `Arm_SetFastPoll(该轴)`，
                   组里**最后发的那个**会把快查抢走（三步组里是 M5）。
                   若当前步是 M6 而快查给了 M5，**M6 的位置就回到 120ms 一轮** →
                   M6 的严格闭环会把"还没刷新"误判成"没到位"，一直纠到
                   `ACT_STRICT_BUDGET_MS`(3s) 才跳步 —— 现场就是
                   "抓完料回缩+转向到位后**静止好久**"。
                   所以这里把快查**还给当前步**（当前步是舵机时，给组里第一个电机步）。 */
                if ((st->joint == AJT_TURN) || (st->joint == AJT_LIFT))
                {
                    Arm_SetFastPoll((st->joint == AJT_TURN) ? (uint8_t)ARM_ADDR_TURN
                                                            : (uint8_t)ARM_ADDR_LIFT);
                }
                else
                {
                    uint8_t q;

                    for (q = 1U; q < n; q++)
                    {
                        uint8_t jq = actSteps[actIdx + q].joint;

                        if ((jq == AJT_TURN) || (jq == AJT_LIFT))
                        {
                            Arm_SetFastPoll((jq == AJT_TURN) ? (uint8_t)ARM_ADDR_TURN
                                                             : (uint8_t)ARM_ADDR_LIFT);
                            break;
                        }
                    }
                }

                Debug_Log("ACT", "pair group %u steps from step%u (%s .. %s) parallel",
                          (unsigned int)actParN, (unsigned int)(actIdx + 1U),
                          Act_JointName(st->joint),
                          Act_JointName(actSteps[actIdx + actParN - 1U].joint));
            }
        }
        return;
    }

    /* 硬超时：防止某一步永远等不到到位（堵转/被挡/行程不够） */
    if ((now - actStepTick) > ARM_MOVE_TIMEOUT_MS)
    {
#if (ACT_STEP_TIMEOUT_SKIP != 0U)
        /* ★【2026-10-10 晚 用户报"STORE PLACE 放到靶位后不松爪"的根本修法】
           原来这里直接 `Act_AbortInternal("timeout")` ⇒ **整段动作中止**，
           于是**排在它后面的所有步都不执行** —— 典型现象就是
           "料已经放到靶位了，但**松手那一步没跑，爪子一直不松**"
           （前面 "M6 下降" 那一步进不了严格容差 -> 卡满 ARM_MOVE_TIMEOUT_MS）。
           改成：**记 UNREACHED + 跳到下一步**（流程一定跑到最后一步，松手一定执行）。
           ⚠ 需要"整段中止"语义时把 ACT_STEP_TIMEOUT_SKIP 置 0 退回原行为。 */
        actUnreached = 1U;              /* 保障层：这段动作会带 **UNREACHED** 告警收尾 */
        actPairOn    = 0U;              /* 清并行标志，确保下面这一下能真的往前走 */
        Debug_Log("ACT", "step%u/%u %s TIMEOUT %ums -> skip (UNREACHED)",
                  (unsigned int)(actIdx + 1U), (unsigned int)actStepCount,
                  Act_JointName(st->joint), (unsigned int)ARM_MOVE_TIMEOUT_MS);
        Act_Advance();
        return;
#else
        Act_AbortInternal("timeout");
        return;
#endif
    }

    /* ---- ★车载转盘（舵机3）：**按回读确认到位**（S3 有回读，参数页就是读它标定的）---- */
    if (st->joint == AJT_CAR)
    {
        uint16_t curS3  = 0U;
        uint8_t  haveS3 = Servo_GetPos((uint8_t)SERVO_ID_TURNTABLE, &curS3);
        uint32_t el     = now - actStepTick;

        /* ① 回读已到目标 ±`ACT_S3_ARRIVE_TOL` -> 立刻放行（预选过的话几乎是秒过） */
        if (haveS3 != 0U)
        {
            int32_t d = (int32_t)curS3 - st->param;

            if (d < 0) { d = -d; }
            if (d <= (int32_t)ACT_S3_ARRIVE_TOL)
            {
                Act_Advance();
                return;
            }
        }
        else if (Act_CarS3AlreadyDone() != 0U)
        {
            /* 读不到回读：只能按"对位成功时预选过且已转过 ACT_S3_MOVE_MS"放行 */
            Act_Advance();
            return;
        }

        if (el < (uint32_t)ACT_S3_MOVE_MS) { return; }

        /* ② 到点还没到位 = **丢步** -> 重发一次，再给一个窗口（这是本次修的重点） */
        if (actCarS3Resent == 0U)
        {
            actCarS3Resent = 1U;
            actStepTick    = now;
            Arm_ServoMove(SERVO_ID_TURNTABLE, Act_ServoParam(st->param));
            Debug_Log("ACT", "step%u S3 not arrived (cur=%u tgt=%ld) -> resend",
                      (unsigned int)(actIdx + 1U), (unsigned int)curS3, (long)st->param);
            return;
        }

        /* ③ 重发后仍不到位：放行 + 告警（绝不卡死整段） */
        actUnreached = 1U;
        Debug_Log("ACT", "step%u S3 UNREACHED (cur=%u tgt=%ld) -> skip",
                  (unsigned int)(actIdx + 1U), (unsigned int)curS3, (long)st->param);
        Act_Advance();
        return;
    }

    /* ---- 舵机步：回读到目标附近即完成；回读不可用则按 ARM_SEQ_SERVO_WAIT_MS 兜底 ---- */
    if ((st->joint == AJT_CLAW) || (st->joint == AJT_GRIP))
    {
        /* ★【2026-10-24 用户要求】"放料时物料还没放稳，舵机3 就开始转"：
           松爪步命令发出后先**强制静置** `ACT_PLACE_SETTLE_MS`（等爪子张开、料落稳），
           静置期结束前**不走**下面任何"早就到位/NOWAIT 立刻放行"的捷径。 */
        if ((st->flags & ACT_STEP_SETTLE) != 0U)
        {
            if ((uint32_t)(now - actStepTick) < (uint32_t)ACT_PLACE_SETTLE_MS)
            {
                return;
            }
        }

        /* ★★【2026-10-24 用户要求】"一气呵成"：**已经在目标附近就直接过**，不等新回读。
           典型场景：抓完料已经转到车载位，`CAR_PLACE` 开头又把 S1 命令到车载值——
           其实它本来就在那儿；原来要等一个回读周期(100ms)+确认，白停一下。
           ★【2026-10-23】**夹紧步除外**：必须走 `Act_ServoDoneGrip`（确认 + 夹稳静置），
           否则会"还没夹稳就抬臂" -> 抓空。 */
        if (!((st->joint == AJT_GRIP) && (st->param < (int32_t)ACT_S2_OPEN_MIN)))
        {
            uint8_t  id  = (st->joint == AJT_CLAW) ? (uint8_t)SERVO_ID_CLAW
                                                   : (uint8_t)SERVO_ID_GRIP;
            uint16_t cur = 0U;
            int32_t  d;

            /* ★【2026-10-27】只认"很新"的回读（≤ ACT_SKIP_POS_FRESH_MS）：
               `Servo_GetPos` 的有效期是 1000ms，用它做"已在目标附近就过"的判据，
               可能拿到上一步留下的陈旧值提前放行。 */
            if ((Servo_GetPos(id, &cur) != 0U) &&
                ((HAL_GetTick() - Servo_GetPosTickById(id)) <= (uint32_t)ACT_SKIP_POS_FRESH_MS))
            {
                d = (int32_t)cur - st->param;
                if (d < 0) { d = -d; }
                if (d <= (int32_t)ACT_SERVO_LOOSE_TOL)
                {
                    Act_Advance();
                    return;
                }
            }
        }

        /* ★【2026-10-24 用户要求】"松抓指令下发就行"：
           · 模板里显式标了 `ACT_STEP_NOWAIT` 的动作（如放车载的松爪）；或
           · 这是**舵机2 的"松开"**（S2 ≥ `ACT_S2_OPEN_MIN`，本项目约定 大 = 松开）
           → 命令本拍已经发出，**不等回读确认**，立刻放行下一步。
           只有"夹紧"（S2 小）才需要确认"确实夹到了"（由 `Act_ServoDoneGrip` 用 120ms 窗口快速判）。 */
        if (((st->flags & ACT_STEP_NOWAIT) != 0U) ||
            ((st->joint == AJT_GRIP) && (st->param >= (int32_t)ACT_S2_OPEN_MIN)))
        {
            Debug_Log("ACT", "step%u %s nowait -> advance",
                      (unsigned int)(actIdx + 1U), Act_JointName(st->joint));
            Act_Advance();
            return;
        }

        /* ★【用户 2026-10-10 晚】"缩回 + 转电机5"并行时：**等舵机1 缩够一半再发转向** */
        Act_PairTurnKick(now);

        if (Act_ServoDone(st, now) != 0U)
        {
            Act_Advance();
        }
        return;
    }

    /* ---- 电机步完成判定：驱动器纯位置模式 + **软件闭环兜底** ----
       【2026-10-10 按要求把软件闭环加回来】驱动器那层仍是"发增量 -> 编码器到位就停"，
       软件再兜一层，处理它兜不住的情况（丢帧 / 背隙 / 被负载顶住停短）：
         电机5（ACT_TURN_STRICT_POS=1，严格）：**跑完(停稳) -> 校验 |当前−目标| ≤ 容差
           -> 不合格就按"目标−当前"纠正**，直到合格才允许下一步；
         电机6（ACT_LIFT_STRICT_POS=0，容错）：① 到位帧(带"迟到应答"校验)
           ② 位置进容差 ③ 停住补发（≤ ACT_MOVE_RETRY_MAX 次），补发用尽按 ACT_MOVE_LENIENT
           决定"告警后继续"或"中止"；
         两者都保留**丢帧看门狗**（ACT_RETX_*）：位置毫无反应 -> 重发同一条命令。
       与 bsp_arm_seq（手臂调试页）**同一口径**，判据参数复用 bsp_arm_seq.h。 */
#if (ACT_MOTOR_SW_LOOP == 0)
    /* ★★【2026-10-24 用户要求】"**闭环全部删除，完全靠张大头的绝对位置来跑**"：
       本层不再做软件闭环（不精修、不停稳校验、不补发、不看看门狗），
       只等驱动器自己的**到位帧**；或位置已进"差不多"的宽容差也直接放行。
       唯一保留：M6"升过安全高就提前放行"（这不是闭环，是让上升与转向并行）。
       超时兜底由上面的 `ARM_MOVE_TIMEOUT_MS` 统一切步（记 UNREACHED）。 */
    {
        uint8_t addr = (st->joint == AJT_TURN) ? (uint8_t)ARM_ADDR_TURN
                                               : (uint8_t)ARM_ADDR_LIFT;
        int32_t tol  = (st->joint == AJT_TURN) ? (int32_t)ARM_SEQ_TURN_POS_TOL
                                               : (int32_t)ARM_SEQ_LIFT_POS_TOL;

        if (Act_M6EarlyReleaseOk(st) != 0U)
        {
            Debug_Log("ACT", "step%u LIFT early-release (M6=%d) -> next joints start",
                      (unsigned int)(actIdx + 1U), (int)Arm_GetPos(addr));
            Act_Advance();
            return;
        }

        /* ★【2026-10-24 用户要求】"电机5 转到目标 **±100** 内 -> 电机6 下降"：
           本步带 `ACT_STEP_EARLY` 才生效（目前只有 TRAY PICK 的转向步）。
           提前放行后 M5 仍按绝对目标继续转到位，不需要再管。 */
        if (((st->flags & ACT_STEP_EARLY) != 0U) &&
            (Act_NearTarget(addr, st->param, Act_EarlyMargin(st->joint)) != 0U))
        {
            Debug_Log("ACT", "step%u %s within %d (cur=%d tgt=%ld) -> next (early)",
                      (unsigned int)(actIdx + 1U), Act_JointName(st->joint),
                      (int)Act_EarlyMargin(st->joint), (int)Arm_GetPos(addr), (long)st->param);
            Act_Advance();
            return;
        }

        if ((Arm_IsReachDone(addr) != 0U) ||
            (Act_NearTarget(addr, st->param,
                            tol * (int32_t)ACT_MOVE_LOOSE_MULT) != 0U))
        {
            Act_Advance();
        }
        return;
    }
#else
    {
        uint8_t  addr   = (st->joint == AJT_TURN) ? ARM_ADDR_TURN : ARM_ADDR_LIFT;
        uint16_t mvel   = (st->joint == AJT_TURN) ? ARM_TURN_VEL : ARM_LIFT_VEL;
        uint8_t  macc   = (st->joint == AJT_TURN) ? (uint8_t)ARM_TURN_ACC : (uint8_t)ARM_LIFT_ACC;
        uint8_t  strict = (st->joint == AJT_TURN) ? (uint8_t)ACT_TURN_STRICT_POS
                                                  : (uint8_t)ACT_LIFT_STRICT_POS;
        int32_t  tol    = (st->joint == AJT_TURN) ? (int32_t)ARM_SEQ_TURN_POS_TOL
                                                  : (int32_t)ARM_SEQ_LIFT_POS_TOL;
        uint8_t  done   = 0U;

        /* ★【2026-10-24】`ACT_STEP_EARLY`：进 ±`Act_EarlyMargin()` 就放行下一步 */
        if (((st->flags & ACT_STEP_EARLY) != 0U) &&
            (Act_NearTarget(addr, st->param, Act_EarlyMargin(st->joint)) != 0U))
        {
            if ((Arm_IsPosValid(addr) != 0U) &&
                (Act_NearTarget(addr, st->param, tol) == 0U))
            {
                Act_TrimRelease(addr, st->param, Arm_GetPos(addr), now);
            }
            Debug_Log("ACT", "step%u %s within %d (cur=%d tgt=%ld) -> next (early)",
                      (unsigned int)(actIdx + 1U), Act_JointName(st->joint),
                      (int)Act_EarlyMargin(st->joint), (int)Arm_GetPos(addr), (long)st->param);
            Act_Advance();
            return;
        }

        if (strict != 0U)
        {
            /* ★★【2026-10-15 用户要求】"电机6 在安全高度以上的时候，**上升可以和电机5 转向同时进行**"：
               M6 一升过安全高度就**提前放行本步**（M6 继续升到目标，由后台精修校验），
               后面的"舵机1 缩回 + 电机5 转向"立刻开始 ⇒ 上升与转向**同时跑**。
               （只对"上升 + 后面紧跟(缩回,转向)"的步生效，见 `Act_M6EarlyReleaseOk`） */
            if (Act_M6EarlyReleaseOk(st) != 0U)
            {
                if ((Arm_IsPosValid(addr) != 0U) &&
                    (Act_NearTarget(addr, st->param, tol) == 0U))
                {
                    Act_TrimRelease(addr, st->param, Arm_GetPos(addr), now);
                }
                Debug_Log("ACT", "step%u LIFT early-release (M6=%d >= %d) -> S1 ret + TURN start now",
                          (unsigned int)(actIdx + 1U), (int)Arm_GetPos(addr),
                          (int)ACT_PAIR_MIN_M6);
                Act_Advance();
                return;
            }

            /* ===== ★ 到位即放行：把闭环挪到后台，**先让下一个关节动** =====
               "到位" = ① 驱动器到位帧（0xFD 的 0x9F）到达；或 ② 位置已进本轴**严格容差**。
               一到"到位"就放行下一步（别的关节马上开始动）；若此刻**还没进严格容差**，
               把该轴交给**后台精修**（Act_TrimService），等它停稳后再校到严格容差。
               ⚠ **不放宽到位判据**（不是"差不多就放"），只是不再等闭环跑完。
               （开关见 bsp_arm_action.h 的 ACT_RELEASE_ON_REACH；=0 即退回原严格闭环） */
            if ((uint8_t)ACT_RELEASE_ON_REACH != 0U)
            {
                uint8_t released = 0U;

                if ((Arm_IsReachDone(addr) != 0U) &&
                    (Act_ReachTrusted(addr, st->param, tol, actStepPos0) != 0U))
                {
                    /* 到位帧 + **可信校验**：不可信的"迟到旧帧"不放行（见 Act_ReachTrusted） */
                    released = 1U;
                }
                else if (Act_NearTarget(addr, st->param,
                                        tol * (int32_t)ACT_MOVE_LOOSE_MULT) != 0U)
                {
                    /* ★【2026-10-24 用户要求】"偏差不是很大就直接放置"：
                       位置进**宽容差**（严格容差 × ACT_MOVE_LOOSE_MULT）也放行，
                       剩下的偏差由后台精修继续收敛，**不再让整条动作等闭环**。 */
                    released = 1U;
                }
                else
                {
                    /* 还没到位：继续等（由下面的严格闭环兜底纠正） */
                }

                if (released != 0U)
                {
                    /* 还没进严格容差 -> 登记后台精修（进了就不必再管）。
                       ★【2026-10-15】位置**读不到也登记**（传 ACT_POS_NONE）：后台会在第一次
                       读到位置时重建基准 —— 否则"读不到位置"这条路上闭环就真丢了。 */
                    if (Act_NearTarget(addr, st->param, tol) == 0U)
                    {
                        int32_t p0 = (Arm_IsPosValid(addr) != 0U) ? Arm_GetPos(addr)
                                                                  : (int32_t)ACT_POS_NONE;

                        Act_TrimRelease(addr, st->param, p0, now);
                    }
                    Act_Advance();
                    return;
                }
            }

            /* ============ 兜底：严格闭环（跑完 -> 校验 -> 纠正） ============ */
            if (Arm_IsPosValid(addr) == 0U)
            {
                /* 位置读不到（接线 / Response 没配）：只能退回采信到位帧，否则完全没法动 */
                if (Arm_IsReachDone(addr) != 0U)
                {
                    done = 1U;
                }
            }
            else
            {
                int32_t cur = Arm_GetPos(addr);

                if (cur != actMotorLastPos)
                {
                    /* 位置还在变 => 电机还在跑：刷新基准、重新开始"停稳"计时 */
                    actMotorLastPos   = cur;
                    actMotorStallTick = now;
                    actMotorStallOn   = 1U;
                }
                else if ((now - actMotorStallTick) < (uint32_t)ARM_SEQ_MOVE_STALL_MS)
                {
                    /* 位置不变但还没到"停稳"时长：继续观察 */
                }
                else
                {
                    /* 已停稳 -> **校验到位误差** */
                    int32_t err = cur - st->param;

                    if (err < 0)
                    {
                        err = -err;
                    }

                    if (err <= tol)
                    {
                        done = 1U;      /* 校验通过：这一步才算完成，才允许进下一步 */
                    }
                    else
                    {
                        /* 误差超容差 -> **闭环纠正**：按"目标 − 当前"再走一次剩余距离 */
                        if (actRetx < 250U)
                        {
                            actRetx++;
                        }
                        actMotorStallTick = now;
                        Debug_Log("ACT", "step%u %s fix%u cur=%d tgt=%d err=%d tol=%d",
                                  (unsigned int)(actIdx + 1U), Act_JointName(st->joint),
                                  (unsigned int)actRetx, (int)cur, (long)st->param,
                                  (int)err, (int)tol);
                        Arm_ReachClear(addr);
                        Arm_MoveAbsDir(addr, st->param, mvel, macc, ARM_DIR_AUTO);
                    }
                }
            }

            /* ===== 硬上限（两级）=====
               ★【2026-10-10 晚 用户报"有时停下很久才走下一步"的根因修法】
               改前这里只有一条：`(now-actStepTick) > ARM_MOVE_TIMEOUT_MS(15s)` -> `Act_AbortInternal`
               ⇒ ① 该步要**卡满 15s** 才往下走（现场就是"停下很久"）；
                  ② 而且是**中止整段**（不是跳步）⇒ 排在后面的步全不跑
                     （典型现象：料放到靶位了，**松手那步没执行、爪子一直不松**）。
               改成两级，都**只跳这一步、绝不中止整段**：
                 ① `ACT_STRICT_BUDGET_MS`（默认 3000ms）：纠正在预算内仍进不了容差 -> 跳步；
                 ② `ARM_MOVE_TIMEOUT_MS`（15s）绝对看门狗（兜"位置完全读不到"）-> 同样跳步。
               偏差不会丢：该轴已登记的**后台精修**会继续把它校到位（见 Act_TrimService），
               本段动作也会带 **UNREACHED** 告警收尾（页面可见）。 */
            if (done == 0U)
            {
                uint32_t el = now - actStepTick;
                uint32_t budget = (uint32_t)ACT_STRICT_BUDGET_MS;

                if ((budget != 0U) && (el > budget))
                {
                    actUnreached = 1U;
                    actPairOn    = 0U;
                    Debug_Log("ACT", "step%u/%u %s not in pos %ums (fix%u cur=%d tgt=%d tol=%d) -> skip (UNREACHED)",
                              (unsigned int)(actIdx + 1U), (unsigned int)actStepCount,
                              Act_JointName(st->joint), (unsigned int)el,
                              (unsigned int)actRetx, (int)Arm_GetPos(addr),
                              (long)st->param, (int)tol);
                    /* ★【2026-10-11】跳步也**不丢偏差**：登记后台精修，让它在本步之后校回目标
                       （原来只有"提前放行"那条路才登记精修 —— 跳步的偏差没人管 → 抓偏/放歪）。 */
                    if (Arm_IsPosValid(addr) != 0U)
                    {
                        Act_TrimRelease(addr, st->param, Arm_GetPos(addr), now);
                    }
                    Act_Advance();
                    return;
                }

                if (el > (uint32_t)ARM_MOVE_TIMEOUT_MS)
                {
                    actUnreached = 1U;
                    actPairOn    = 0U;
                    Debug_Log("ACT", "step%u/%u %s WATCHDOG %ums -> skip (UNREACHED)",
                              (unsigned int)(actIdx + 1U), (unsigned int)actStepCount,
                              Act_JointName(st->joint), (unsigned int)el);
                    /* ★同上：看门狗跳步也登记后台精修，偏差不丢 */
                    if (Arm_IsPosValid(addr) != 0U)
                    {
                        Act_TrimRelease(addr, st->param, Arm_GetPos(addr), now);
                    }
                    Act_Advance();
                    return;
                }
            }
            if (done != 0U)
            {
                Act_Advance();
            }
            return;
        }

        /* ================= 电机6：容错闭环（到位帧 / 位置容差 / 停住补发） ================= */
        {
            /* 到位帧的"位置校验窗"：离目标超过 容差×倍数 就认为它是**上一步的迟到应答** */
            int32_t guardTol = (ARM_SEQ_REACH_GUARD_MUL == 0U)
                             ? (int32_t)0x7FFFFFFF
                             : (tol * (int32_t)ARM_SEQ_REACH_GUARD_MUL);

            if (Arm_IsReachDone(addr) != 0U)
            {
                /* 判据①：驱动器到位帧（带位置校验） */
                if ((Arm_IsPosValid(addr) == 0U) ||
                    (Act_NearTarget(addr, st->param, guardTol) != 0U))
                {
                    done = 1U;
                }
                else if (actMotorReachIgnored == 0U)
                {
                    actMotorReachIgnored = 1U;
                    Debug_Log("ACT", "step%u %s reach ignored cur=%d tgt=%d (stale?)",
                              (unsigned int)(actIdx + 1U), Act_JointName(st->joint),
                              (int)Arm_GetPos(addr), (long)st->param);
                }
                else
                {
                    /* 已提示过，继续忽略这一条 */
                }
            }
            else if (Act_NearTarget(addr, st->param, tol) != 0U)
            {
                /* 判据②：位置已进本轴到位容差（目标就是当前位置、或到位置了但没回到位帧） */
                done = 1U;
            }
            else if (Arm_IsPosValid(addr) != 0U)
            {
                /* 判据③：看位置有没有在变 —— "动过又停住"且没到目标 -> 补发剩余距离 */
                int32_t cur = Arm_GetPos(addr);

                if (cur != actMotorLastPos)
                {
                    actMotorLastPos   = cur;
                    actMotorStallTick = now;
                    actMotorStallOn   = 1U;
                }
                else if (actMotorStallOn != 0U)
                {
                    if ((now - actMotorStallTick) < (uint32_t)ARM_SEQ_MOVE_STALL_MS)
                    {
                        /* 位置不变但还没到 STALL_MS，先继续观察 */
                    }
                    else if (actRetx < (uint8_t)ACT_MOVE_RETRY_MAX)
                    {
                        actRetx++;
                        actMotorStallTick = now;
                        Debug_Log("ACT", "step%u %s retry %u cur=%d tgt=%d",
                                  (unsigned int)(actIdx + 1U), Act_JointName(st->joint),
                                  (unsigned int)actRetx, (int)cur, (long)st->param);
                        Arm_ReachClear(addr);
                        Arm_MoveAbsDir(addr, st->param, mvel, macc, ARM_DIR_AUTO);
                    }
#if (ACT_MOVE_LENIENT != 0U)
                    else
                    {
                        /* 补发用完仍未到：**不卡死流程**，打 UNREACHED 告警后继续下一步。
                           ★【2026-10-15】同时登记**后台精修**：偏差交给后台继续校到位
                           （"所有闭环后台做"），本步只负责把流程放行。 */
                        actUnreached = 1U;
                        Debug_Log("ACT", "step%u %s UNREACHED cur=%d tgt=%d, continue + bg trim",
                                  (unsigned int)(actIdx + 1U), Act_JointName(st->joint),
                                  (int)cur, (long)st->param);
                        Act_TrimRelease(addr, st->param, cur, now);
                        done = 1U;
                    }
#else
                    else
                    {
                        /* 严格模式：补发用完仍未到就中止 */
                        Debug_Log("ACT", "step%u %s UNREACHED cur=%d tgt=%d, abort",
                                  (unsigned int)(actIdx + 1U), Act_JointName(st->joint),
                                  (int)cur, (long)st->param);
                        Act_AbortInternal("motor not reachable");
                        return;
                    }
#endif
                }
                else
                {
                    /* 本步电机一直没动（stallOn 仍为 0）-> 交给下面的丢帧看门狗 */
                }
            }
            else
            {
                /* 读不到位置：只能靠到位帧，兜底就是下面的总超时 */
            }

            /* ---- 丢帧看门狗：发出后**毫无反应**（电机压根没动）-> 重发同一条命令 ----
               【判据】位置相对"命令下发前"一直没有明显变化（≤ ACT_RETX_MOVE_POS）
               且已过 ACT_RETX_MS；只在"本次目标离当前位置本来就很远"时才判，避免小距离误判。
               【为什么要它】CAN 下发不检查发送结果，整帧丢了没有任何反馈 ——
               不补发就会一直等到 ARM_MOVE_TIMEOUT_MS 硬超时中止（= 这一步没执行）。
               【与判据③的分工】③ 管"动过又停住"；本看门狗管"压根没动"。 */
            if ((done == 0U) && (actMotorStallOn == 0U) &&
                (actRetx < (uint8_t)ACT_RETX_MAX) && (actStepPos0 != ACT_POS_NONE) &&
                (Arm_IsPosValid(addr) != 0U))
            {
                int32_t need  = st->param - actStepPos0;
                int32_t moved = Arm_GetPos(addr) - actStepPos0;

                if (need < 0)  { need = -need; }
                if (moved < 0) { moved = -moved; }

                if ((need > (int32_t)ACT_RETX_MOVE_POS) &&
                    (moved <= (int32_t)ACT_RETX_MOVE_POS) &&
                    ((now - actStepTick) >= (uint32_t)ACT_RETX_MS))
                {
                    actRetx++;
                    actStepTick = now;      /* 重新计时，给重发留出反应时间 */
                    Arm_ReachClear(addr);
                    Arm_MoveAbsDir(addr, st->param, mvel, macc, ARM_DIR_AUTO);
                    Debug_Log("ACT", "step%u %s no reaction -> resend %u/%u",
                              (unsigned int)(actIdx + 1U), Act_JointName(st->joint),
                              (unsigned int)actRetx, (unsigned int)ACT_RETX_MAX);
                    return;
                }
            }

            /* 总超时（硬上限）：即使电机一直在慢慢动，也不会无限等下去 */
            if ((done == 0U) && ((now - actStepTick) > ARM_MOVE_TIMEOUT_MS))
            {
                Act_AbortInternal("move timeout");
                return;
            }
            if (done != 0U)
            {
                Act_Advance();
            }
            return;
        }
    }
#endif
}

uint8_t ArmAct_IsBusy(void)
{
    return ((actState == ACT_RUN) || (actPairStage != 0U)) ? 1U : 0U;
}

/* ★【2026-10-15 用户要求】S1（爪子前伸/缩回）是否已收回到 0 附近。
   用途：任务点"关键作业已完成、只剩回位类多余动作"时，允许底盘先动、手臂并行；
   前提 S1 必须回 0，否则车一动电机5 会晃（见头文件说明）。
   ★统一严格档：回读不可用 -> 0（**不放行**），宁可退回"等整段跑完"也不冒险。 */
uint8_t ArmAct_S1Retracted(void)
{
    uint16_t p = 0U;

    if (Servo_GetPos((uint8_t)SERVO_ID_CLAW, &p) == 0U)
    {
        return 0U;
    }

    return (p <= (uint16_t)ACT_PAIR_TURN_S1_HI) ? 1U : 0U;
}

uint8_t ArmAct_IsDone(void)
{
    return (actState == ACT_DONE) ? 1U : 0U;
}

uint8_t ArmAct_IsAborted(void)
{
    return (actState == ACT_ABORT) ? 1U : 0U;
}

/* ★【2026-10-24 用户要求】"夹不到就停下报 FAIL，绝不动别的关节"：
   本段动作是因为**夹爪确认失败**（空抓/超时/重发用尽，且重夹也用尽）而中止的。
   调用方用它来决定"停下报 FAIL"还是走原来的告警继续。 */
uint8_t ArmAct_GripFailed(void)
{
    return actGripFail;
}

/* ★★【2026-10-29 用户要求】"抓最后一个物料时，一定要保证**物料已经抓在爪子上**。
   用途：任务点想"提前跑路段"（底盘先动、手臂并行）时，光看 S1 回 0 不够 ——
   S1 的回读可能是**上一件留下的 0**（缓存有效期 1000ms），那时爪子还没夹紧。
   判据 = ①本段没被判夹爪失败；②舵机2 回读有效且 ≤ `ACT_GRIP_GRAB_MAX`(700)
          （夹到料一定会被顶在这个位置以内；空爪全闭是 520，也算"闭合"，
            所以必须配合 `ArmAct_GripFailed()` 一起看，失败时本函数返回 0）。
   注意：回读不可用 -> 0（**不放行**），宁可退回"等整段跑完"也不带着没夹住的料动车。 */
uint8_t ArmAct_GripHeld(void)
{
    uint16_t p = 0U;

    if (actGripFail != 0U)
    {
        return 0U;
    }
    if (Servo_GetPos((uint8_t)SERVO_ID_GRIP, &p) == 0U)
    {
        return 0U;
    }
    return (p <= (uint16_t)ACT_GRIP_GRAB_MAX) ? 1U : 0U;
}

/* ★★【2026-10-29 用户要求】"抓最后一个物料时，一定要保证**手臂抬高了**"。
   直接转调内部的 `Act_LiftAboveSafe()`（M6 ≥ `ACT_PAIR_MIN_M6`，值越大越高）。 */
uint8_t ArmAct_LiftAboveSafe(void)
{
    return Act_LiftAboveSafe();
}

uint8_t ArmAct_HasUnreached(void)
{
    return actUnreached;
}

void ArmAct_Abort(void)
{
    actPairStage = 0U;
    actPairOn    = 0U;            /* ★规则2 的并行标志：中止时一并清掉，防残留 */
    actAtTtReady = 0U;            /* ★中止后姿态未知，不能再认为"在观察位" */
    Arm_SetFastPoll(0U);          /* 兜底：确保位置快查不会残留 */
    Act_TrimClear(ARM_ADDR_TURN); /* 取消后台精修（即便当前不在 RUN 态也要清） */
    Act_TrimClear(ARM_ADDR_LIFT);
#if (ACT_SERVO_OPEN_LOOP != 0U)
    /* ★【2026-10-15】中止时把"后台舵机校验/补发"也停掉，避免它继续发舵机命令 */
    Act_SrvVerifyClear((uint8_t)SERVO_ID_CLAW);
    Act_SrvVerifyClear((uint8_t)SERVO_ID_GRIP);
#endif
    if (actState == ACT_RUN)
    {
        Act_AbortInternal("by caller");
    }
}

/* ★【2026-10-15】外部"要接管电机5/6 / 舵机"时调：只丢后台目标，**不动电机/舵机**。
   典型场景：进机械臂测试页手动点动（`Arm_TurnTest`/`Arm_LiftTest` 绕过 `Act_SendStep`，
   不会自动清精修；舵机点动也绕过 `Arm_ServoMove` 的后台登记）——
   不清的话后台会把用户点动过的关节拽回旧目标。 */
void ArmAct_TrimCancelAll(void)
{
    Act_TrimClear(ARM_ADDR_TURN);
    Act_TrimClear(ARM_ADDR_LIFT);
#if (ACT_SERVO_OPEN_LOOP != 0U)
    Act_SrvVerifyClear((uint8_t)SERVO_ID_CLAW);
    Act_SrvVerifyClear((uint8_t)SERVO_ID_GRIP);
#endif
}

const char *ArmAct_GetStateText(void)
{
    static char buf[20];

    switch (actState)
    {
        case ACT_RUN:
            (void)snprintf(buf, sizeof(buf), "RUN %s %u/%u", actTitle,
                           (unsigned int)(actIdx + 1U), (unsigned int)actStepCount);
            break;

        case ACT_DONE:
            /* 保障层：有步骤没走到目标就加个 `!` 标记（日志里还有一条 **UNREACHED** 告警） */
            (void)snprintf(buf, sizeof(buf), (actUnreached != 0U) ? "DONE! %s" : "DONE %s",
                           actTitle);
            break;

        case ACT_ABORT:
            (void)snprintf(buf, sizeof(buf), "ABORT %s", actTitle);
            break;

        default:
            (void)snprintf(buf, sizeof(buf), "IDLE");
            break;
    }
    return buf;
}
