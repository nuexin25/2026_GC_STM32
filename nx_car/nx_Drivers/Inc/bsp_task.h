/**
 ******************************************************************************
 * @file    bsp_task.h
 * @brief   赛题流程层对外接口（流程引擎 + 运动原语 + 作业点）
 * @author  nx_car
 * @date    2026-10-07
 ******************************************************************************
 * @note   【本模块是什么】
 *           把整条赛题流程按《赛题开发框架指南》第 8 节的骨架写成**一条顺序流程**：
 *           每一步调用一个动作函数，返回 1 就进下一步、返回 0 就等下一拍。
 *           与 bsp_mission 的关系：**导航内核复用 bsp_mission**（Mission_NavGoTo /
 *           Mission_NavRotate 包装了它已验证的转角闭环/同拍停稳/到位判定），
 *           本模块只负责"步骤推进 + 超时降级 + 作业点调度"，不重写运动逻辑。
 *
 *         【两种模式】KEY3 切换（运行中不允许切）：
 *           - TASK_MODE_STEP 单步调试：按 KEY5 执行当前步，执行完停在 DONE 等按键；
 *           - TASK_MODE_AUTO 任务模式：按 KEY5 一键启动，从当前步连续跑到 T_END。
 *
 *         【返回值约定（框架指南 4.4）】
 *           1 = 本动作已完成（可进下一步）；0 = 进行中。
 *           **失败不靠返回值表达**：超时后由本层打日志 + 先停车 + 返回 1 放行，
 *           失败次数记入统计 —— 流程永远不会因为一个动作失败而停住。
 *
 *         【每一步的目标/朝向都写在 bsp_task.c 的 Task_Update() 里】
 *           现场抽签后**只改那里的数字**（如 GO(1200, -300)、Rotate(90)），
 *           本文件与 bsp_task_cfg.h 都不用动。
 *
 *         硬约束：全部非阻塞（禁止 HAL_Delay / while 等待）；日志禁 %f；
 *                 坐标直接写字面量（不设点位宏）。
 ******************************************************************************
 */
#ifndef __BSP_TASK_H
#define __BSP_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "bsp_key.h"
#include "bsp_task_cfg.h"

/* ===================== 模式与状态枚举 ===================== */

/* 运行模式（KEY3 切换；运行中不允许切） */
typedef enum {
    TASK_MODE_STEP = 0,   /* 单步调试：KEY5 执行当前步，执行完停在 DONE 等按键 */
    TASK_MODE_AUTO        /* 任务模式：KEY5 一键启动，连续跑到 T_END */
} TaskMode_t;

/* 总状态（赛题页第 4 行显示用） */
typedef enum {
    TASK_STATE_IDLE = 0,   /* 空闲：未启动 / 已退出（可按键执行/启动） */
    TASK_STATE_RUN,        /* 执行中 */
    TASK_STATE_DONE,       /* STEP：当前步执行完成，等按键 */
    TASK_STATE_ALLDONE,    /* AUTO：全部步骤跑完 */
    TASK_STATE_ABORT       /* 已中止 / 超时 */
} TaskState_t;

/* ===================== 任务码与车载位表（协议 v2.0 §5 / §3.2） ===================== */
/* 【任务码】格式 DDD+DDD+DDD+DDD（15 字符，如 "134+123+314+231"），解析结果如下：
 *   第1组 color1[3]：第一批 3 件的搬运颜色顺序（1红 2黄 3蓝 4绿）
 *   第2组 place1[3]：第一批 3 件的放置**靶位号**（与 color1 一一对应）
 *   第3组 color2[3]：第二批 3 件的搬运颜色顺序
 *   第4组 place2[3]：第二批 3 件在粗加工区的放置**靶位号**
 * 【靶位 vs 环 —— 别混】暂存区/粗加工区各有 **3 个靶位**（编号 1/2/3，赛题图3 的
 *   "数字1、2、3"）；**一个靶位由 6 个「环」组成，6 环是同心圆**（赛题表3 的 1环~6环，
 *   用于摆放精度评分 15/10/7/5/3/1，6环外 0）。**环不进任务码**，任务码里的是靶位号(1~3)。
 * 解析复用 bsp_qr 的成帧/校验结果（QR_GetCode），此处只做字段搬运与"靶位号范围"复核。 */
typedef struct {
    uint8_t color1[3];   /* 第1组：第一批颜色顺序，取值 1~4 */
    uint8_t place1[3];   /* 第2组：第一批放置靶位号，取值 TASK_TGT_MIN~TASK_TGT_MAX */
    uint8_t color2[3];   /* 第3组：第二批颜色顺序，取值 1~4 */
    uint8_t place2[3];   /* 第4组：第二批粗加工区放置靶位号 */
    uint8_t valid;       /* 1=合法（已扫到且四组均在范围内）；0=未扫到/非法 */
} Task_Code_t;

/* 任务码"放置位"（靶位号）的取值范围 = 1~3。
 * 依据：暂存区/粗加工区各有 **3 个靶位**（赛题图3 的"数字1、2、3 面向赛场中心"）。
 * 注意：赛题第 6 页"表3"列的 1环~6环是**一个靶位内部的 6 个同心圆环**（摆放精度评分），
 *       不是任务码里的编号，所以上界是 3 不是 6。 */
#define TASK_TGT_MIN    1U
#define TASK_TGT_MAX    3U

/**
 * @brief  取当前任务码（解析结果）
 * @retval 结构体常量指针（valid=0 表示未扫到/非法，各字段已清零）
 * @note   每次调用都按 bsp_qr 的最新成帧结果刷新一次（合法→非法变化时打一条日志）。
 */
const Task_Code_t *Task_GetCode(void);

/**
 * @brief  取某批第 idx 件（idx 0~2）的目标颜色
 * @param  batch: 批次 1/2
 * @param  idx  : 件号 0~2
 * @retval 颜色 1~4；参数越界/任务码非法返回 0
 */
uint8_t Task_ColorOfBatch(uint8_t batch, uint8_t idx);

/**
 * @brief  取某批第 idx 件（idx 0~2）的放置靶位号
 * @param  batch: 批次 1/2
 * @param  idx  : 件号 0~2
 * @retval 靶位号 1~3；参数越界/任务码非法返回 0
 */
uint8_t Task_PlaceOfBatch(uint8_t batch, uint8_t idx);

/**
 * @brief  记录车载某号位上物料的颜色（颜色↔位号绑定，供后续放到加工区取用）
 * @param  slot : 车载号位 1~3
 * @param  color: 颜色 1~4（0=清空该位）
 * @retval 无
 */
void Task_SetCarSlotColor(uint8_t slot, uint8_t color);

/**
 * @brief  取车载某号位上物料的颜色
 * @param  slot: 车载号位 1~3
 * @retval 颜色 1~4；该位未放料 / 参数越界返回 0
 */
uint8_t Task_GetCarSlotColor(uint8_t slot);

/**
 * @brief  按颜色反查它在车载哪个号位
 * @param  color: 颜色 1~4
 * @retval 车载号位 1~3；没找到（或 color=0）返回 -1
 */
int8_t Task_FindSlotByColor(uint8_t color);

/* ===================== 步骤枚举（顺序 = 执行顺序，不能改） ===================== */
/* 【写法约定：★ 绝对坐标系（相对出发点）】
 *   每个作业点固定两步：
 *       T_xx_ROT  用 Rotate(deg)     转到**相对出发点朝向**的绝对角（俯视逆时针为正）；
 *       T_xx_GO   用 GO(x_mm, y_mm)  走到**相对出发点**的绝对坐标点。
 *   按这个顺序排（ROT 在前、GO 在后），所以枚举里**每组都是 xxx_ROT 在 xxx_GO 前面**。
 *   好处：每步都朝**同一个世界基准**回纠，所以朝向/位置误差都**不会逐段累积**。
 *   两点之间若不能一条直线到达（场地只有十字车道），就在中间插**一对**：
 *       T_xx_R*（转到拐角方向）/ T_xx_T*（走到拐角点）—— 见下面的 T_QR_R1 / T_QR_T1 示例。
 * ⚠ 增删步骤必须**三处同步**（顺序一一对应）：
 *       ① 本枚举 ② bsp_task.c 的 tskStepName[] ③ bsp_task.c 的 Task_Update() switch。 */
typedef enum {
    /* ---- S1 扫码（R1/T1 是"拐角再加一对：转 + 走"的写法示例）---- */
    T_QR_ROT, T_QR_GO, T_QR_R1, T_QR_T1, T_QR_DO,

    /* ---- 第一批 ---- */
    T_B1_TRAY_GO, T_B1_TRAY_ROT,T_B11_TRAY_GO,T_B12_TRAY_GO,
    T_B1_TRAY_PICK,
    T_B1_RGH_ROT,  T_B1_RGH_GO,T_B11_RGH_GO,T_B12_RGH_GO,
    T_B1_PLACE,                              /* 打靶放置整段（SP_ 序列：$SHOT 对位 + 车载1/2/3 取件放靶位） */
    T_B1_PBACK,                              /* STORE BACK 整段（SB_ 序列：按放置逆序 车载3→2→1 取回） */
    T_B1_STO_GO,T_B12_STO_ROT, T_B12_STO_GO,
    T_B1_RGH_PLACE,                          /* ★ROUGH PLACE 整段（3 件一次做完；与 STORE PLACE 共用 SP_ 实现） */

    /* ---- 第二批（与第一批同构：先在粗加工区按第 4 组放置 -> 取回 -> 去暂存区同色码垛）---- */
    T_B2_TRAY_ROT, T_B2_TRAY_GO, T_B22_TRAY_GO,
    T_B2_TRAY_PICK,                          /* ★TRAY PICK 整段（3 件一次做完；与第一批共用 PICK_TRAY_ALL） */
    T_B2_RGH_ROT,  T_B2_RGH_GO,T_B22_RGH_GO,
    T_B2_RGH_TURN,                           /* ★Rotate(180)：与第一批去粗加工区的最后一步同构 */
    T_B2_PLACE,                              /* ★第二批"按任务码第 4 组放置"整段（SP_ batch=2） */
    T_B2_PBACK,                              /* ★第二批"取回"整段（SB_ batch=2：按放置逆序搬回车上） */
    T_B2_TOST_GO, T_B2_TOST_ROT, T_B2_TOST_GO2,  /* ★搬去暂存区（沿第一批的路线坐标） */
    T_B2_STACK,                              /* ★码垛：叠到"第一批同色物料"所在靶位 */
    T_B2_PBACK0, 
    T_B2_STO_ROT, T_B2_STO_GO,

    /* ---- 收尾 ---- */
    T_HOME_ROT, T_HOME_GO, T_STAT, T_END
} TaskStep_t;

/* ===================== 对外接口：生命周期 ===================== */

/**
 * @brief  赛题模块初始化（上电调一次）
 * @param  无
 * @retval 无
 * @note   只复位状态机，**不发电机命令、不清零里程计、不使能四轮**。
 */
void Task_Init(void);

/**
 * @brief  进入赛题页（主菜单 MISSION 项调用）
 * @param  无
 * @retval 无
 * @note   动作：若上次残留运行态先中止 -> 状态复位 IDLE（保留当前步号，方便反复调同一步）
 *         -> Chassis_OdomReset()（原点 = 当前位置，朝向基准 = 当前 yaw）-> 四轮失能
 *         -> 切 PAGE_MISSION。**不自动执行**（安全，必须按 KEY5 才动）。
 */
void Task_Enter(void);

/**
 * @brief  退出赛题页（KEY4 调用）
 * @param  无
 * @retval 无
 * @note   先 Task_Stop()（四轮急停 + 失能），再清活动标志。
 */
void Task_Exit(void);

/**
 * @brief  流程引擎推进（主循环调度，非阻塞）
 * @param  无
 * @retval 无
 * @note   每拍只做一件小事：判本步超时 / 调一次当前步的动作函数 / 推进步骤指针。
 *         **漏调度会一直停在本步**（位置模式电机自己能走完，软件不会推进）。
 */
void Task_Update(void);

/**
 * @brief  赛题页短按按键处理（本页活动时由 main 路由调用）
 * @param  key: 按键 ID（KEY1~KEY5）
 * @retval 无
 * @note   KEY1/KEY2 = 上一步/下一步（只移步骤指针，不执行；运行中忽略）
 *         KEY3      = 切换模式 STEP <-> AUTO（运行中忽略）
 *         KEY4      = 中止并退出回主菜单
 *         KEY5      = STEP：执行当前步；AUTO：一键启动；运行中 = 急停
 */
void Task_HandleKey(KeyID_t key);

/**
 * @brief  赛题页长按按键处理（本页活动时由 main 路由调用）
 * @param  key: 按键 ID（KEY1~KEY5）
 * @retval 无
 * @note   目前只有一条：**长按 KEY5 = ODOM 清零**（原点 = 当前位置，朝向基准 = 当前 yaw）。
 *         运行中忽略。为什么放长按：短按 KEY5 已被"执行/一键启动"占用。
 */
void Task_HandleLongKey(KeyID_t key);

/**
 * @brief  查询赛题页是否活动
 * @param  无
 * @retval 1=本页活动（按键交本页处理），0=非本页
 */
uint8_t Task_IsActive(void);

/* ===================== 对外接口：运动原语 ===================== */
/* 全部非阻塞：1 = 已完成；0 = 进行中（下一拍继续调）。
 * 内部统一：首次调用负责"使能四轮 + 下发命令"，之后只推进；
 * 超时（TASK_MOVE_TIMEOUT_MS）则打日志 + Task_Stop() + 返回 1 放行。
 *
 * 【流程里用哪两个】★ 现场路线用下面这对**绝对坐标系**原语（相对出发点）：
 *     Rotate(deg)          先转到目标朝向：deg 是**相对出发点朝向**的绝对角（逆时针为正）
 *     GO(x_mm, y_mm)       再走到目标点：x/y 是**相对出发点**的绝对坐标
 *   所以每个作业点都是 "T_xx_ROT(转) -> T_xx_GO(走)" 两步；
 *   拐弯就在中间再插一对 "T_xx_R*(转到拐角方向) -> T_xx_T*(走到拐角点)"。
 *   好处：每步都朝同一个世界基准回纠，**朝向与位置误差都不逐段累积**。
 *
 * 【GO_REL / Rotate_REL 为什么还留着】它们是"车身坐标系增量"写法
 *   （dx 沿车头、dy 沿车身右侧；ddeg 在当前朝向上再转多少）。现场若想
 *   "只管走一段、不用管绝对点"也能用，但**朝向残差会逐段累积**；当前流程没有用它俩。 */

/**
 * @brief  走到相对坐标 (x,y)：**绝对坐标写法**（相对出发点，现场路线用这个）
 * @param  x_mm: 目标 X，单位 mm，正 = 出发点前方
 * @param  y_mm: 目标 Y，单位 mm，正 = 出发点右侧
 * @retval 1=已到达（含停稳）；0=进行中
 * @note   只做平移、不含转向。**每步都朝同一个世界点回纠，落点误差不逐段累积。**
 */
uint8_t GO(int32_t x_mm, int32_t y_mm);

/**
 * @brief  原地转到相对朝向 deg：**绝对朝向写法**（相对出发点朝向，现场路线用这个）
 * @param  deg: 目标朝向，单位 deg，范围 -180~+180，俯视逆时针为正
 * @retval 1=已到位（含转角闭环 + 朝向稳定等待）；0=进行中
 * @note   会把朝向**拉回绝对基准**（每步都朝同一个基准回纠，**不累积朝向误差**）——
 *         这正是绝对写法相对 `Rotate_REL`（相对写法）的最大好处。
 */
uint8_t Rotate(float deg);

/**
 * @brief  原地转到 rotateDeg，同时把**绝对坐标计算强制按 coordDeg**（★两个域分别给角度）
 * @param  rotateDeg: **要转到的角度**（照现场写法填，含零漂补偿；例 98 = 真 90 + 补 8）
 * @param  coordDeg : **绝对坐标计算要用的角度**（强制设定；例 90）
 * @retval 1=已到位（含转角闭环 + 朝向稳定等待）；0=进行中
 * @note   ★【2026-10-11 用户要求（最终口径）】**"转向转 98°，绝对值坐标按 90°（强制设定）计算"**：
 *           · 旋转域：照 `rotateDeg`(98) 下发 ⇒ 车身物理朝向 = 真 90°；
 *           · 坐标域：里程计位置与"全局→车身"变换按 `coordDeg`(90) 算
 *             （内部 = `Chassis_SetYawCompDeg(rotateDeg − coordDeg)`）。
 *         ⚠ 坐标补偿量**跨步保持**到下一次 `Rotate_Comp()`；不想再补写 `Rotate_Comp(deg, deg)`。
 *         非阻塞，与 `Rotate()` 同一套。
 *         用法：`case T_B1_RGH_GO: if (Rotate_Comp(98.0f, 90.0f) != 0U) { Tsk_Advance(); } break;`
 */
uint8_t Rotate_Comp(float rotateDeg, float coordDeg);

/**
 * @brief  相对**当前车身**增量平移 (dx,dy)：沿车头前进 dx、沿车身右侧横移 dy
 * @param  dx_mm: 沿**车头方向**的位移（前进为正），单位 mm
 * @param  dy_mm: 沿**车身右侧**的位移（右移为正），单位 mm
 * @retval 1=已到达；0=进行中
 * @note   ★ 这是**车身系增量**写法，**当前流程没有使用**（现场路线用绝对的 GO）。
 *         换算用的是 bsp_chassis 里程计的"正变换"（见 Chassis_OdomUpdate），保证自洽：
 *           全局增量 X = dx·cosθ + dy·sinθ
 *           全局增量 Y = −dx·sinθ + dy·cosθ        （θ = 当前实测 yaw）
 *         再加到当前 ODOM 坐标上得到**绝对目标点**，交给 GO() 去闭环。
 *         **首次调用锁存**：不锁存的话目标会跟着车走，永远到不了。
 *         因为目标是"按当前实测位置现算"的，所以**位置误差不会逐段累积**。
 */
uint8_t GO_REL(int32_t dx_mm, int32_t dy_mm);

/**
 * @brief  相对**当前朝向**增量旋转 ddeg（**车身系增量写法，当前流程没有使用**）
 * @param  ddeg: 增量角度，单位 deg（逆时针为正；例如右转 90° 就写 -90）
 * @retval 1=已到位；0=进行中
 * @note   首次调用锁存绝对目标朝向 = 归一化(当前 yaw + ddeg)。
 *         ⚠ 因为是相对的，**朝向残差会逐段累积**；所以现场路线改用绝对写法 `Rotate(deg)`。
 */
uint8_t Rotate_REL(float ddeg);

/**
 * @brief  非阻塞等待
 * @param  ms: 等待时长，单位 ms（0 = 立即返回 1）
 * @retval 1=时间到；0=未到
 */
uint8_t Wait(uint16_t ms);

/**
 * @brief  立即中止：四轮急停 + 失能 + 清所有原语状态
 * @param  无
 * @retval 无
 * @note   用于 KEY5 急停 / KEY4 退出 / 任一动作超时降级。
 */
void Task_Stop(void);

/** @brief 串口测试用：显式设置运行模式（0=STEP 1=AUTO）；KEY3 是切换，远程用这个 */
void Task_CmdSetMode(uint8_t autoMode);

/* ===================== 对外接口：作业点（本次全部空桩） ===================== */
/* ⚠ 本次（第1步）全部是空桩：打一条 `[TASK] xxx not implemented` 日志后**返回 1 放行**，
 *   不发任何电机/机械臂/视觉命令 —— 目的是让整条流程"不接视觉、不接机械臂也能跑完"。
 *   注释里写清了真实实现该调哪个现有模块（bsp_qr / bsp_align / bsp_tt / bsp_arm_action），
 *   后续第 3~8 步逐个把桩换成真实实现，**不用改流程结构**。 */

/**
 * @brief  二维码作业：读任务码 -> 解析 -> 显示 -> 下发视觉端
 * @retval 1=完成；0=进行中
 * @note   TODO(第3步)：等 QR_SETTLE_MS -> 用 bsp_qr 的 QR_IsNew()/QR_GetCode() 取码
 *         -> 解析 4 组（3 位数字/+ 分隔，1~4）-> Display_SetTaskCode() -> 下发视觉端。
 */
uint8_t QR(void);

/**
 * @brief  原料区取第 i 件（i=0/1/2）
 * @param  i: 件号（0 起）
 * @retval 1=完成；0=进行中
 * @note   TODO(第5步)：复用现有两套 —— 视觉侧 Align(ALIGN_TURNTABLE)（bsp_align），
 *         机械臂侧走 bsp_tt（转盘取料序列：丢弃第一个颜色 -> 等颜色变化 -> 抓 -> 上车确认）。
 *         参数在 bsp_task_cfg.h 的 [3] TRAY 节（含机械臂动作编号 TRAY_PICK_ARM_ACT*）。
 */
uint8_t PICK_TRAY(uint8_t i);

/**
 * @brief  转盘取料（**一批 3 件一次做完**）
 * @param  batch: 批号 **1 / 2**（★2026-10-11 新增）
 *                —— 决定用任务码的哪一组颜色：**第 1 批 -> 第 1 组（color1）；第 2 批 -> 第 3 组（color2）**
 * @retval 1=整段完成（或失败放行）；0=进行中
 * @note   自动按顺序做完（全非阻塞，主循环每拍推进一次）：
 *           ① 进转盘区发 `$RCP` 起头（协议 v2.0 §3.1/§6.1），并清零"车载位↔颜色"表；
 *              同时 `ArmAct_GotoTtReady()` 把机械臂摆到**观察位**（看得到物料上的圆环）；
 *           ② 等臂**真正到位**后按 `batch` 取颜色 → `TT_StartWithColors()` —— 内部按 v2.0 §6.1 做
 *              "$RCP 对位 -> 观察位 -> $RZ 找色位 -> 准备位 Q -> $CCG 轮询 -> 抓 ->
 *               放车载 1/2/3"（bsp_tt）；
 *           ③ 结束后（成功/失败都做）**不再发送任何帧**：v2.0 无推送式 $CMD，
 *              打靶阶段由打靶作业点自行发 `$SHOT`。
 *         依赖主循环的 `ArmAct_Update()` / `TT_Update()`（main.c 已调度，漏调度会卡住）。
 */
uint8_t PICK_TRAY_ALL(uint8_t batch);

/**
 * @brief  打靶放置（**一批 3 件一次做完**，STORE / ROUGH PLACE 通用）
 * @param  无
 * @retval 1=整段完成（或失败放行）；0=进行中
 * @note   ★【2026-10-10】直接驱动 `bsp_tt` 的 **SP_ 打靶放置序列**（与任务调试页
 *         `TDBG_STORE_PLACE` / `TDBG_ROUGH_PLACE` **共用同一份代码**）：
 *           **臂进观察位** -> `$SHOT` -> 等有效 -> 精确对位（严格档 1px、不限时） ->
 *           **车载位 1→2→3**：**按任务码件号**取靶位（车载位 k = 第 k-1 件 -> `place1[k-1]`；
 *           `ArmAct_Run(ARM_OP_CAR_PICK, slot)` 取件 -> `ArmAct_Run(ARM_OP_TGT_PLACE, tgt)` 放靶位。
 *         ⚠ 前提：先跑过 PICK_TRAY_ALL()（颜色↔车载位号绑定；**现在只用于核对/告警**，
 *           靶位号按任务码件号取，不再依赖绑定色反查）。
 *         单步超时用 `TASK_STORE_PLACE_TIMEOUT_MS`（内部串了"对位 + 搬 3 件"，通用值不够）。
 *         推进靠主循环的 `ArmAct_Update()` / `SP_Update()`（main.c 已调度，漏调度会卡住）。
 */
uint8_t PLACE_STORE_ALL(void);

/**
 * @brief  打靶取回（**STORE BACK**：按放置的逆序把 3 件从靶位搬回车载 3 个号位）
 * @param  无
 * @retval 1=整段完成（或失败放行）；0=进行中
 * @note   ★【2026-10-10 实做】直接驱动 `bsp_tt` 的 **SB_ 打靶取回序列**（与任务调试页
 *         `TDBG_STORE_BACK` **共用同一份代码**）：
 *           按 **车载位 3→2→1**（STORE PLACE 是 1→2→3）：读该位绑定色（仅核对/告警） ->
 *           任务码第 1 组**按件号**取靶位（slot k -> `place1[k-1]`）-> `ArmAct_SbPick(tgt, ...)` 从靶位取件 ->
 *           `ArmAct_SbPlace(slot)` 放回车载该号位。
 *         **第一件**（= 最后放上去的那件）：按用户要求"**不上抬、不转向**，等 1s 直接抓"。
 *         **不需要视觉**（车已在打靶位对好，靶位号由绑定色 + 任务码算出）。
 *         ⚠ 前提：先跑过 `PICK_TRAY_ALL()` + `PLACE_STORE_ALL()`；绑定色只用于核对/告警。
 *         单步超时用 `TASK_STORE_BACK_TIMEOUT_MS`。
 *         推进靠主循环的 `ArmAct_Update()` / `SB_Update()`（main.c 已调度）。
 */
uint8_t PLACE_BACK_ALL(void);

/**
 * @brief  粗加工区放第 i 件
 * @param  i: 件号（0 起）
 * @retval 1=完成；0=进行中
 * @note   TODO(第6步)：Align(ALIGN_ROUGH) -> 按任务码第 2 组第 i 件的靶位号放置。
 *         机械臂动作调 bsp_arm_action（编号见 bsp_task_cfg.h [4] RGH 节）。
 */
uint8_t PLACE_ROUGH(uint8_t i);

/**
 * @brief  粗加工区取回第 i 件
 * @param  i: 件号（0 起）
 * @retval 1=完成；0=进行中
 * @note   TODO(第6步)：Align(ALIGN_ROUGH) -> 抓取 -> 上车确认。
 */
uint8_t PICK_ROUGH(uint8_t i);

/**
 * @brief  暂存区放第 i 件
 * @param  i: 件号（0 起）
 * @retval 1=完成；0=进行中
 * @note   TODO(第7步)：Align(ALIGN_FINE) -> 放置到对应靶位。
 */
uint8_t PLACE_STORE(uint8_t i);

/**
 * @brief  暂存区码垛第 i 件
 * @param  i: 件号（0 起）
 * @retval 1=完成；0=进行中
 * @note   TODO(第7步)：定位到**第一批同色物料**所在环 -> 抬高 STO_STACK_LIFT_MM
 *         -> 平稳叠放（叠在同色之上）。
 */
uint8_t STACK_STORE(uint8_t i);

/**
 * @brief  统计显示：汇总抓取/放置正确次数并显示到车上装置
 * @retval 1=完成；0=进行中
 * @note   TODO(第8步)：汇总 -> 车顶显示装置（字高 ≥12mm）。
 */
uint8_t STAT(void);

/**
 * @brief  视觉 PID 闭环对位（作业函数内部调用；第 4 步实做）
 * @param  target: 用哪路视觉帧，AlignTarget_t（ALIGN_TURNTABLE/ROUGH/FINE）
 * @retval 1=已收敛（含失败放行）；0=进行中
 * @note   本次已把 ALIGN_TURNTABLE 接到现有 bsp_align（Align_Start/IsDone/IsFailed）；
 *         ALIGN_ROUGH / ALIGN_FINE 两路靶位帧留桩（第 4 步实做）。
 *         依赖主循环的 Align_Update()（main.c 已在调度）。
 */
uint8_t Align(uint8_t target);

/* ===================== 对外接口：页面显示 ===================== */
/* 赛题页 4 行布局（8x16 大字体，每行最多 16 个 ASCII 字符）：
 *   第1行  12/47 B1_RROT      ← 步骤序号/总数 + 当前步骤名
 *   第2行  T1200,300h90       ← 本步目标（T=目标点坐标 + 朝向；从未走过 GO/Rotate 时显示 "T---"）
 *   第3行  A1180,295h89       ← 实测 ODOM（A=实际坐标 + 朝向，标定用）
 *   第4行  IDLE STEP K5=GO    ← 状态 + 模式 + 按键提示
 * 说明：第 1 行不放 "TASK" 前缀、步骤名用缩写（如 T_B1_RGH_ROT -> "B1_RROT"），
 *       否则最长的组合会超过 16 个字符被截断。 */

/**
 * @brief  取页面第 1 行文本：步骤序号/总数 + 当前步骤名
 * @retval 形如 "12/47 B1_RROT" / "1/47 QR_ROT"
 */
const char *Task_GetTitleText(void);

/**
 * @brief  取页面第 2 行文本：本步目标（最近一次 GO/Rotate 的目标）
 * @retval 形如 "T1200,300h90"；尚无目标时为 "T---"
 * @note   朝向按整数度显示（日志禁 %f，页面同样只显示整数度）。
 */
const char *Task_GetTargetText(void);

/**
 * @brief  取页面第 3 行文本：实测 ODOM 坐标与朝向（现场标定用）
 * @retval 形如 "A1180,295h89"（单位 mm/deg，取整；负值带 - 号）
 */
const char *Task_GetOdomText(void);

/**
 * @brief  取页面第 4 行文本：状态 + 模式 + 按键提示
 * @retval "IDLE STEP K5=GO" / "RUN AUTO" / "DONE STEP" / "ALLDONE AUTO" / "ABORT"
 */
const char *Task_GetStateText(void);

/**
 * @brief  取步骤名（缩写，≤9 字符；越界返回 "?"）
 * @param  idx: 步骤下标（0 ~ Task_GetStepCount()-1）
 * @retval 静态字符串指针
 */
const char *Task_GetStepName(uint16_t idx);

/**
 * @brief  取步骤总数（由 TaskStep_t 枚举自动得出）
 * @retval 步骤个数
 */
uint16_t Task_GetStepCount(void);

/**
 * @brief  取当前步骤下标（0 起）
 * @retval 当前步骤下标
 */
uint16_t Task_GetStepIndex(void);

/**
 * @brief  取当前运行模式
 * @retval TASK_MODE_STEP / TASK_MODE_AUTO
 */
TaskMode_t Task_GetMode(void);

/**
 * @brief  取当前总状态
 * @retval TaskState_t
 */
TaskState_t Task_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_TASK_H */
