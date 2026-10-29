/**
 ******************************************************************************
 * @file    bsp_mission.h
 * @brief   赛题任务执行器（P9 步骤1：跑点位 —— 单步 STEP + 连续 AUTO 双模式）
 * @author  nx_car
 * @date    2026-09-14
 ******************************************************************************
 * @note   本模块是**后续所有赛题开发的落点**（点位导航 + 后续工艺动作）。
 *
 *         【本次范围（步骤1）】只做"跑点位"：
 *           按 MS_POINT_LIST 的点位顺序，逐个导航到位（先转到目标朝向 -> 再到
 *           目标坐标点）；**作业点**（MS_PT_WORK）到点停
 *           MS_ARRIVE_HOLD_MS(默认 2s)，**中转点**（MS_PT_VIA）只路过不停；
 *           全程只做移动，**不做扫码/抓取/放置/视觉对位**。
 *
 *         【两种模式（《赛题开发指南》3.2）】按 KEY3 切换：
 *           - STEP 单步调试：按 KEY5 执行当前点位，执行完停在 DONE 等下一次按键；
 *           - AUTO 任务模式：按 KEY5 一键启动，从当前点位**连续跑到底**（自动
 *             msIdx++），跑完进 ALLDONE。
 *
 *         【怎么让车沿十字车道走、不进禁入区】
 *           相邻两点之间车走的是**直线**，所以要在两点之间插**中转点**把斜线拆成
 *           正交折线（场地只有中间十字灰色车道可行驶）。中转点写法与示例见
 *           bsp_mission_cfg.h 的 MS_POINT_LIST 说明。
 *
 *         【本次明确不做】
 *           扫码(MS_ACT_SCAN)/取料(MS_ACT_PICK)/放置(MS_ACT_PLACE)/视觉(MS_ACT_VISION)：
 *             只在 bsp_mission_cfg.h 里定义了枚举占位，实现体留空并打
 *             `not implemented` 日志，防止误调用。
 *           障碍物避障、外部视觉数据接入、统计显示。
 *
 *         【运动策略 = 与 P7 跑图完全一致（"丝滑"要求）】
 *           复用 P4 已验证的位置模式 + P7 的到位闭环三件套，参数也复用
 *           `Route_GetCfg()` 那一套运行期参数（见 bsp_mission_cfg.h 文件头）：
 *             ① 到位优先等电机主动回传(Motor_ReachOk)，超时退脉冲容差兜底；
 *             ② 到位那一刻**四轮同拍 0xFE 停 + 静置 holdMs**，治减速段末端横移；
 *             ③ **转角闭环**：按 HWT101CT 回传的 θ 补转（P + 死区 + 限幅），
 *                治"开环转角误差大 -> 下一段方向歪 -> 横偏"；
 *             ④ 转弯后**朝向稳定等待**，保证用补转后的新 IMU 样本规划平移方向。
 *
 *         硬约束：全部函数非阻塞，禁止 HAL_Delay；日志禁 %f（整数打印）。
 ******************************************************************************
 */
#ifndef __BSP_MISSION_H
#define __BSP_MISSION_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "bsp_key.h"
#include "bsp_mission_cfg.h"

/* ===================== 运行模式 ===================== */

/* 两种模式（《赛题开发指南》3.2）：STEP 用于逐步验证/定位问题，AUTO 用于整场演示。
 * 赛题页按 KEY3 切换，运行中不允许切。 */
typedef enum {
    MS_MODE_STEP = 0,   /* 单步调试：按 KEY5 执行当前点位，执行完停下来等下一次按键 */
    MS_MODE_AUTO        /* 任务模式：按 KEY5 一键启动，从当前点位**连续跑到底** */
} MsMode_t;

/* ===================== 状态/阶段枚举 ===================== */

/* 总状态（页面第 4 行显示用） */
typedef enum {
    MS_STATE_IDLE = 0,   /* 空闲：未启动 / 已退出（可按键执行当前点位） */
    MS_STATE_RUN,        /* 执行中：旋转 / 平移 / 到点停留            */
    MS_STATE_DONE,       /* **STEP** 当前点位执行完成（等用户上/下切换或再执行） */
    MS_STATE_ALLDONE,    /* **AUTO** 全部点位跑完                    */
    MS_STATE_ABORT       /* 已中止 / 超时                            */
} MsState_t;

/* 当前点位所处阶段 */
typedef enum {
    MS_PHASE_TURN = 0,   /* 旋转阶段：转到目标朝向 + 转角闭环 */
    MS_PHASE_MOVE,       /* 平移阶段：走到目标坐标点          */
    MS_PHASE_HOLD,       /* 到点停留阶段：停 MS_ARRIVE_HOLD_MS */
    MS_PHASE_WAIT        /* 空闲/未推进（IDLE/DONE/ABORT）     */
} MsPhase_t;

/* ===================== 到点视觉请求（协议 v2.0） ===================== */
/* 点位表第 8 个字段（vcmd）的取值：车到达作业点**停稳后**向树莓派发出的 v2.0 请求。
 * v2.0 取消了 v1.0 的"推送式 $CMD"，改为"到达作业点发一次请求"（《树莓派通讯协议》§3）。
 * 取值含义：
 *   MS_REQ_NONE  = 0  不发任何请求（普通导航点 / 中转点）
 *   MS_REQ_RCP   = 1  发 $RCP：请求转盘中心 xy 偏移（转盘区取料，用于车身定位）
 *   MS_REQ_RZ    = 2  发 $RZ ：请求转盘三个位号的位号/颜色（手臂进入观察位后）
 *   MS_REQ_SHOT  = 3  发 $SHOT：请求打靶区圆环数据（进入打靶作业区）
 *   MS_REQ_CCG3  = 4  发 $CCG,3：请求 3 号位颜色（用于单点核对） */
typedef enum {
    MS_REQ_NONE = 0,   /* 不发（默认） */
    MS_REQ_RCP  = 1,   /* $RCP：转盘中心偏移 */
    MS_REQ_RZ   = 2,   /* $RZ ：转盘三位号/颜色 */
    MS_REQ_SHOT = 3,   /* $SHOT：打靶区圆环数据 */
    MS_REQ_CCG3 = 4    /* $CCG,3：3 号位颜色 */
} MsReq_t;

/* ===================== 对外接口（状态机 / 控制） ===================== */
/* ⚠ 本节接口属于**旧点位流程**，已被 bsp_task 取代（main.c 不再调用它们）；
 *   保留函数体与签名只为"不断编译 / 不被牵连"，导航内核 Mission_Nav* 仍在用。
 *   待后续步骤确认无引用后可整体清理。 */

/**
 * @brief  赛题模块初始化（只复位状态机，不发电机命令）
 * @param  无
 * @retval 无
 * @note   上电初始化阶段禁止使能/转动车轮；四轮使能只发生在 Mission_StartPoint()。
 *         本函数不清零里程计（清零只发生在进入赛题页或按 KEY5 时）。
 */
void Mission_Init(void);

/**
 * @brief  进入赛题页（主菜单 MISSION 项调用）
 * @param  无
 * @retval 无
 * @note   动作顺序：
 *           1) 若上次残留运行态，先中止（四轮急停 + 失能）；
 *           2) 状态复位为 IDLE、阶段 WAIT（**保留上次选中的点位号**，方便反复调同一点）；
 *           3) `Chassis_OdomReset()`：把**当前位置/朝向当作出发点**（原点 (0,0)，
 *              yaw 基准 = 当前朝向）—— 与"原点 = 出发点"的约定一致，省掉一次忘按清零；
 *              若车已在别处、想换原点，进页后按 KEY5 重新清零即可；
 *           4) 切到 PAGE_MISSION。**不自动执行点位**（安全，必须按 KEY3 启动）。
 */
void Mission_Enter(void);

/**
 * @brief  退出赛题页（KEY4 调用）
 * @param  无
 * @retval 无
 * @note   先中止（四轮急停 + 失能），再清活动标志。
 */
void Mission_Exit(void);

/**
 * @brief  赛题状态机推进（主循环调用，非阻塞）
 * @param  无
 * @retval 无
 * @note   每拍只推进"一个动作"：要么下发当前阶段的命令，要么判一次是否到位，
 *         保证任何一次调用都很快返回（OLED 刷新/按键响应不受影响）。
 *         **漏调度会导致永远不判到位**（位置模式电机自己会走完，软件停在 RUN）。
 */
void Mission_Update(void);

/**
 * @brief  赛题页按键处理（本页活动时由 main 路由调用）
 * @param  key: 按键 ID（KEY1~KEY5）
 * @retval 无
 * @note   按键布局按《赛题开发指南》3.4：
 *           KEY1 = 上一点位（只移光标，不执行；执行中忽略）
 *           KEY2 = 下一点位（只移光标，不执行；到底循环）
 *           KEY3 = **切换模式 STEP ↔ AUTO**（执行中忽略）
 *           KEY4 = 中止并退出回主菜单（KEY4 统一退出规范）
 *           KEY5 = **STEP：执行当前点位 / 运行中=中止**
 *                  **AUTO：一键启动（从当前点位连续跑到底）/ 运行中=急停**
 *         里程计清零走**长按 KEY5**，见 Mission_HandleLongKey()。
 */
void Mission_HandleKey(KeyID_t key);

/**
 * @brief  赛题页**长按**按键处理（本页活动时由 main 路由调用）
 * @param  key: 按键 ID（KEY1~KEY5）
 * @retval 无
 * @note   目前只用到一条：**长按 KEY5 = 里程计清零**
 *         （原点 = 当前位置，朝向基准 = 当前 yaw；执行中忽略）。
 *         为什么放到长按：短按 KEY5 已经被"执行/一键启动"占了，
 *         而清零在标定时才用（见《赛题调参手册》3.b），不该和常用键抢短按。
 *         注意 bsp_key 的长按**不会**再产生短按事件，所以两个处理函数不冲突。
 */
void Mission_HandleLongKey(KeyID_t key);

/**
 * @brief  查询赛题页是否活动
 * @param  无
 * @retval 1=本页活动（按键交本页处理），0=非本页
 */
uint8_t Mission_IsActive(void);

/* ===================== 对外接口（运行控制） ===================== */

/**
 * @brief  启动当前点位（**两种模式都调它**）
 * @param  无
 * @retval 无
 * @note   **STEP**：执行当前点位，到位停 2s 后**停在 DONE**，等人工再按键；
 *         **AUTO**：以当前点位为起点，**连续执行到底**（到位后自动 msIdx++，
 *                  中转点不停留，作业点停 2s + 执行动作），全部跑完进 ALLDONE。
 *         命令顺序（顺序不能反）：
 *           1) 先确认能读到四轮位置，读不到就拒绝启动并打 `no pos data` 日志
 *              （里程计坐标与目标增量都依赖位置数据，读不到会朝错误坐标走）；
 *           2) 使能四轮（位置模式必须先使能，否则电机不收位置命令）；
 *           3) 状态置 RUN、阶段置 TURN，按状态机推进；
 *              姿态用**绝对目标 yaw**（点位表里的 yaw_deg），
 *              位移用"目标点 − 当前 ODOM 坐标"算车身系增量 -> 每步都朝同一个
 *              世界点回纠，单点误差不累积。
 *         执行中重复调用被忽略。
 */
void Mission_StartPoint(void);

/**
 * @brief  中止当前点位执行（四轮急停 + 失能 + 状态置 ABORT）
 * @param  无
 * @retval 无
 * @note   用于人工中止、退出页面或超时兜底。正常到位由电机固件平顺停车。
 */
void Mission_Abort(void);

/**
 * @brief  里程计清零：原点 = 当前位置，朝向基准 = 当前 yaw
 * @param  无
 * @retval 无
 * @note   执行中（RUN）忽略，避免把正在规划的目标点算乱。
 *         用途：① 把小车摆到出发点后建立原点；② 换一个参考点重新标定。
 */
void Mission_OdomZero(void);

/* ===================== 对外接口（单段导航，供 bsp_task 复用） ===================== */
/* ★ 下面两个函数是"把本文件里已验证的静态导航内核对外暴露"的**唯一入口**：
 *   bsp_task 的 GO()/Rotate() 通过它们复用 Ms_IssueMoveAbs / Ms_IssueRot /
 *   Ms_PhaseArrived / Ms_StopAndSettle / Ms_YawLoop / Ms_YawSettleWait / Ms_CheckPosErr，
 *   **不另写一套移动/转向逻辑**（见《赛题开发框架指南》4.1 / 提醒2）。
 *   两者都非阻塞：首次调用负责"下发命令"，之后每拍调用只推进一次；
 *   返回 1 时内部自复位，因此下一次调用会重新启动（幂等，可反复调用）。
 *   注意：本文件原有的点位流程（Mission_Enter/Update/HandleKey…）已被 bsp_task 取代，
 *   但这些静态内核仍在被下面的接口使用，**不要删除**。 */

/* ===================== ★赛题路段：速度模式（2026-10-24） =====================
 * MISSION_NAV_VEL = 1：`Mission_NavGoTo / Mission_NavRotate` 改成**速度模式**跑：
 *   · 平移：0xF6 速度闭环 + IMU 航向保持 + **里程计距离**判到点（不再发位置脉冲）；
 *   · 转向：原地差速旋转 + IMU 航向闭环（连续航向，跨 ±180 不翻号）。
 *   与 TEST 页（VELTEST 正方形）用的是同一套控制律，参数在这里统一调。
 * = 0：退回原来的位置模式（脉冲 + 到位帧 + 同拍停稳），行为与以前完全一致。
 * ⚠ 速度模式下**没有"到点后抖动收敛"**：到点即停，落点精度靠里程计 + 航向环，
 *   任务点的视觉对位仍照旧把它拉正。 */
#define MISSION_NAV_VEL          0
#define MISSION_ROT_VEL          0       /* 转向也回退到**位置模式**（2026-10-23）：
                                            速度模式转向在 VELTEST/VELROUTE 页很准（过冲 ≤1°），
                                            但赛题页主循环一拍 75~100ms，刹不住、实测过冲 14°，
                                            故赛题页统一用位置模式转向。 */

/* 速度模式内核：赛题点位表用不用（MISSION_NAV_VEL）与它编译不编译**分开**。
   现在赛题切回位置模式，内核仍保留给 VELROUTE 页（速度模式跑图）直接调用。 */
#define MISSION_VEL_API          1

#define MNV_VEL_MMPS             5000.0f  /* 巡航速度 mm/s（★实测 >1100 高速段开始左右画龙） */
#define MNV_ACC_MMPS2            1600.0f   /* 加减速度 mm/s²（★2026-10-28 二轮提速：1200 -> 1600；
                                              ★现场串口 VRCFG 的 A= 会覆盖它，要生效就别再发 A= 或发 A=1600） */
#define MNV_CREEP_MMPS           220.0f   /* 减速段最低爬行速度（★2026-10-28：160 -> 220；
                                              太低会"轮子转车不走"，太高会冲过点） */
#define MNV_LAT_MAX_MMPS         300.0f   /* ★横向（横移修正）速度上限：高速全速横移会把车带成蛇形 */
#define MNV_LAT_DEAD_MM          12.0f    /* ★横向死区（全程生效）：小于它就不横移，别追着微差画龙 */
#define MNV_LAT_FILT             0.15f    /* ★横向误差低通系数（每 20ms 一拍，≈200ms 时间常数）：
                                             里程计的横向分量在 CAN 拥堵时是噪声（实测 100ms 内
                                             跳 100mm），直接拿去控制就是左右画龙 */
#define MNV_FINAL_MM             45.0f    /* 末端精定位区长度：区内改用更保守的制动包络
                                             （★2026-10-28 二轮提速：70 -> 45，缩短爬行段） */
#define MNV_FINAL_ACC_MMPS2      350.0f   /* 末端制动包络 mm/s²（斜走只有两轮出力，
                                             实际减速能力远小于主包络，用主包络必然冲过点）
                                             （★2026-10-28：250 -> 350） */
#define MNV_CAPTURE_MM           90.0f    /* ★末端捕获区：区内禁止自转 */
#define MNV_CAPTURE_MMPS         220.0f   /* ★捕获区限速上限（★2026-10-28：160 -> 220，随爬行速度一起抬） */
#define MNV_TAPER_MM             1000.0f   /* ★末端线性减速起点：从 320mm 开始平滑收速 */
#define MNV_TAPER_LOW            100.0f   /* ★线性段终点：100mm 内固定爬行速度 */
#define MNV_TAPER_MMPS           550.0f   /* ★线性段起点速度（硬切会顿一下 -> 横偏突然跳 60mm） */
#define MNV_CAPTURE_WZ           0.0f     /* ★捕获区**禁止自转**（爬行时再扭 = 四轮对拽；
                                             航向留给下一段 ROT 修，差 1~2° 不影响落点） */
#define MNV_CAPTURE_LAT_MM       20.0f    /* ★捕获区横向死区（横向很小就别横移，横移靠不住） */
#define MNV_STALL_MM             0.5f     /* ★失速判定：距离进步小于它就不算"在靠近" */
#define MNV_STALL_MS             400U     /* ★连续这么久没靠近 = 卡住/被挡 -> 直接判到点 */
#define MNV_SETTLE_MM            25.0f    /* 停稳后残差超过它 = 冲过点，再贴一次 */
#define MNV_SETTLE_MAX           2U       /* 最多补贴几次（防来回蹭） */
#define MNV_SPLIT_MIN_MM         40.0f    /* 正交化：两个分量都超过它才考虑拆成两段 */
#define MNV_SPLIT_RATIO          0.5f     /* 且短分量 ≥ 长分量×它（≈45°±）；略偏的不拆，
                                             免得把"几乎直走 + 一点横移"掰成 L 形绕路 */
#define MNV_STOP_TOL_MM          12.0f    /* 到点容差 mm */
#define MNV_TICK_MS              20U      /* 控制周期 ms */
#define MNV_STOP_RESEND_MS       150U     /* 到点后连续补发 0 速度的时长（0xF6 无应答，丢一帧
                                             轮子就保持原速 -> 车原地打圈；7~8 帧足够） */

#define MNV_YAW_HOLD             1        /* ★直行航向角闭环开关（2026-10-23）：1=开（IMU 航向保持，直行时左右轮发差速修正）；
                                             0=关（直行不开航向角闭环，四轮只发平移速度、始终同速 同启停，
                                                 航向漂移留给下一段 ROT 的 IMU 航向闭环统一修）。
                                             关掉后下面 MNV_YAW_KP/KD/MAX_DPS/DEAD/LOW 全部失效（不影响原地转向）。 */
#define MNV_YAW_KP               0.9f     /* 直行航向保持 KP（★1.2 高速段修正太硬 -> 画龙；仅 MNV_YAW_HOLD=1 生效） */
#define MNV_YAW_KD               1.2f     /* 直行航向保持 KD */
#define MNV_YAW_MAX_DPS          40.0f    /* 直行修正上限 */
#define MNV_YAW_DEAD_DEG         0.8f     /* ★0.5 太紧：陀螺 0.1° 量化会被当成误差追，抖成左右晃 */
#define MNV_YAW_DEAD_DPS         6.0f
#define MNV_YAW_LOW_MMPS         250.0f   /* ★低于这个速度才按比例收航向修正（刹车段必须给满） */
#define MNV_YAW_LOW_MIN_DPS      8.0f     /* 低速时至少留这么多 */
#define MNV_LAT_ZONE_MM          100000.0f /* （保留定义，横向死区现在全程生效） */

#define MNV_TURN_KP              3.5f     /* 原地转：位置 P（dps per deg） */
#define MNV_TURN_KD              0.0f     /* 原地转：角速度阻尼（dps per dps），0=纯 P */
#define MNV_TURN_MAX_DPS         240.0f   /* 原地转：角速度上限（★2026-10-28 二轮提速：200 -> 240） */
#define MNV_TURN_ACC_DPS2        1500.0f  /* 原地转：制动包络 */
#define MNV_TURN_MIN_DPS         10.0f
#define MNV_TURN_NEAR_DEG        10.0f    /* ★末端减速带：剩余小于它…（★2026-10-28：14 -> 10） */
#define MNV_TURN_NEAR_DPS        25.0f    /* ★…角速度封顶（正/反转对称留余量，实测 -90° 会多冲 3~4°） */
#define MNV_TURN_LEAD_S          0.0f     /* ⚠延迟补偿已停用：预测到 0 会让指令归零、车卡住不动 */
#define MNV_ROT_PRE_MS           0U       /* 转向前空转时长（0=不用）。试过 120ms 没改善，保留接口 */
#define MNV_TURN_ARRIVE_DPS      25.0f    /* ★15 太严：0.1° 量化的实测角速度在 15 附近抖，
                                             到位判定被反复打断，尾段白等 0.6s */
#define MNV_TURN_TOL_DEG         1.0f
#define MNV_ROT_RECHECK_DEG      3.0f     /* ★兜底补转阈值：小于它就不补转（正常情况下末端混合已把它压到 1° 内） */
#define MNV_ROT_CHAIN_SKIP_DEG   2.0f     /* ★chain=1（段间不停顿）时的"免转"阈值：已经在这以内就直接过，
                                              不为 1° 去停一次车（残余由下一段弧线/航向闭环收掉） */
#define MNV_ROT_FIX_BLEND        0.25f    /* ★末端（最后 MNV_TURN_NEAR_DEG 内）用**里程计残差**修正
                                             IMU 目标的每拍混合系数：一次扫到位，不再"落地后再补转一小段"
                                             （IMU 零漂 -> "IMU 转够了、物理少 2~3°"） */
#define MNV_TURN_HOLD_MS         40U
#define MNV_WZ_SLEW_DPS2         800.0f   /* 转向指令斜率限制 */

#define MNV_ACC_LVL              0U       /* 0xF6 加减速档（0=立即跟随） */

/* ★斜向直连（2026-10-26）：GO 不再拆两条正交腿（L 形），段首把目标换算到车身系后
   **一条直线**走完（f,l 合成方向），到段末速度方向不再突变成 90°。 */

/* ★边走边转（2026-10-26，★2026-10-29 改成"只转段末最后一段"）：
   GO 段**最后 MNV_ARC_WIN_MM 内**才把车头转到下一步 ROT 的朝向，到点时朝向已对准，
   ROT 只剩末端几度精修；段末不再停车、不再补发 0 速度、不再留 20ms 空档
   -> 直角弯"带速过弯 + 一步到位 + 直接跑下一段"。
   ★窗口之外的整段仍然笔直：Mnv_Go 的世界系 phi 补偿保证车身转动时轨迹不变，
     转动量被压缩到最后一点点，跑偏累积比"全程慢慢扭"小一个量级。
   转角小于 MNV_ARC_MIN_DEG 或段长小于 MNV_ARC_MIN_MM 时不走弧线，留给 ROT。 */
#define MNV_ARC_TURN_EN          1        /* 1=开 0=关（关掉退回"先直行、再原地转"的老行为） */
#define MNV_ARC_KP               2.2f     /* 弧线航向 P（dps per deg）：偏了就多转一点 */
#define MNV_ARC_KD               0.6f     /* 弧线航向 D（阻尼）：压住甩头 */
#define MNV_ARC_YMX_DPS          150.0f   /* ★弧线角速度上限 dps：窗口内要转完 90°，太小会转不完 */
#define MNV_ARC_MIN_DEG          5.0f     /* 转角小于它不走弧线（留给 ROT，免得来回修） */
#define MNV_ARC_MIN_MM           400.0f   /* ★段长小于它不走弧线（太短转不过来，前馈会爆） */
#define MNV_ARC_WIN_MM           700.0f   /* ★弧线窗口 mm：只在最后这么长内转（前段全速走直线）
                                             （改大 = 转得更平缓、但跑偏累积更久；改小 = 转得更急） */
#define MNV_ARC_FIX_MM           250.0f   /* ★窗口末段 mm：把航向参考按里程计绝对朝向收口，
                                             补掉 IMU 零漂造成的"少转 2~3°"，到点即真值（ROT 不用再补） */

/**
 * @brief  单段导航：走到**绝对里程计坐标** (x, y)（与 ODOM 清零后的坐标系一致）
 * @param  x_mm: 目标 X，单位 mm，正 = 出发点前方
 * @param  y_mm: 目标 Y，单位 mm，正 = 出发点右侧
 * @retval 1=已到达；0=进行中
 * @note   MISSION_NAV_VEL=1 时走**速度模式**（航向保持 + 里程计距离闭环）；
 *         =0 时是原来的位置模式（脉冲 + 到位帧 + 同拍停稳）。
 *         要求：调用前需已使能四轮（由 bsp_task 的 GO() 包装负责）。
 */
uint8_t Mission_NavGoTo(int32_t x_mm, int32_t y_mm);


/**
 * @brief  单段导航：原地转到相对坐标系中的目标朝向 deg（相对出发点朝向）
 * @param  deg: 目标朝向，单位 deg，俯视逆时针为正，范围 -180~+180
 * @retval 1=已到位（含转角闭环 + 朝向稳定等待）；0=进行中
 * @note   全程复用 Ms_IssueRot / Ms_PhaseArrived / Ms_StopAndSettle / Ms_YawLoop /
 *         Ms_YawSettleWait；已在目标朝向（差 < MS_TURN_SKIP_DEG）时直接返回 1。
 */
uint8_t Mission_NavRotate(float deg);

/* ===================== 速度模式内核（供 VELROUTE 页直接调用） =====================
 * 这一组**不受 MISSION_NAV_VEL 影响**（由 MISSION_VEL_API 控制编译）：
 * 赛题点位表走位置模式时，/VELROUTE/ 页面仍可用速度模式跑同一套路段表。
 * 三个函数都是"首次调用自动启动本段，之后每拍推进，返回 1 时自复位"，
 * 与 Mission_NavGoTo / Mission_NavRotate 的调用约定完全一致。 */

/** @brief 速度模式：走到**绝对坐标** (x_mm,y_mm)（里程计坐标系）；一条直线开环走完；1=已到点 */
uint8_t Mission_VelGo(int32_t x_mm, int32_t y_mm);

/**
 * @brief  速度模式：带"本段降速上限 + 段末不停车 + 行进中转向"的 GO（赛题连跑衔接用）
 * @param  cap_mmps: 本段巡航速度上限 mm/s（0 = 用运行期巡航速度）
 * @param  chain:    1 = 段末**不停车**，爬行速度直接交给下一段（拐弯/直行衔接）
 *                   0 = 段末补发 0 速度停稳（转向 / 机械臂作业前必须用 0）
 * @param  turn_deg: 下一步 ROT 的目标朝向 deg（turn_on=1 时才有意义）
 * @param  turn_on:  1 = 行进中把车头转到 turn_deg（边走边转，到点已对准，ROT 只做精修）
 *                   0 = 不转（车头守住段首朝向）
 * @retval 1=已到点；0=进行中
 */
uint8_t Mission_VelGoEx(int32_t x_mm, int32_t y_mm, int32_t cap_mmps, uint8_t chain,
                        int32_t turn_deg, uint8_t turn_on);

/** @brief 取运行期巡航速度 mm/s（路段表规划"进弯/进点降速"用） */
int32_t Mission_VelGetCruise(void);

/** @brief 速度模式：原地转到绝对朝向 deg（相对出发点，逆时针正）；1=已到位 */
uint8_t Mission_VelRot(float deg);

/**
 * @brief  速度模式：原地转到绝对朝向 deg，并指定"到位后是否停车"
 * @param  chain: 1 = 到位后**不补发 0 速度**，直接返回（下一段 GO 马上接手，段间不停顿）
 *                0 = 到位后补发 0 速度停稳（下一段是机械臂作业时必须用 0）
 * @retval 1=已到位；0=进行中
 */
uint8_t Mission_VelRotEx(float deg, uint8_t chain);

/** @brief 速度模式：立即停车（0 速度 + 复位内核状态），用于急停/退出/超时 */
void Mission_VelStop(void);

/* ---- 运行期参数（串口 VRCFG 在线调；掉电回默认，默认值见上面的 MNV_*） ---- */
typedef enum {
    MV_VEL = 0,   /* 巡航速度 mm/s            （200~1500） */
    MV_ACC,       /* 加减速度 mm/s²           （100~2000，打滑就降） */
    MV_FACC,      /* 末端制动包络 mm/s²       （100~2000） */
    MV_CREEP,     /* 末端爬行速度 mm/s        （30~300） */
    MV_TOL,       /* 到点容差 mm              （5~60） */
    MV_KP,        /* 直行航向 KP ×10          （2~50 -> 0.2~5.0） */
    MV_KD,        /* 直行航向 KD ×10          （0~50） */
    MV_YMAX,      /* 直行航向修正上限 dps     （5~120） */
    MV_TMAX,      /* 原地转角速度上限 dps     （40~300） */
    MV_TKP,       /* 原地转位置环 KP ×10      （10~80） */
    MV_TKD,       /* 原地转角速度阻尼 KD ×10  （0~50，抑制冲过头/来回摆） */
    MV_TACC,      /* 原地转制动包络 dps²/10   （30~300 -> 300~3000） */
    MV_COUNT
} MissionVelParam_t;

/** @brief 改一个运行期参数（越界自动夹取）；返回 1=已生效 */
uint8_t Mission_VelSetParam(uint8_t idx, int32_t v);

/** @brief 打印当前全部运行期参数 */
void Mission_VelDump(void);

/** @brief 取遥测：dist=剩余距离(mm) err=航向误差(0.1°) yaw=航向(0.1°) vel=mm/s wz=0.1dps */
void Mission_VelGetTele(int32_t *dist, int32_t *err, int32_t *yaw,
                        int32_t *vel, int32_t *wz);

/** @brief 取本拍"允许速度"（制动包络）mm/s，诊断 v 与包络是否一致 */
int32_t Mission_VelGetDecel(void);

/** @brief 取本拍横向偏差 mm（死区处理前；-9999 = 本段不是 GO），看蛇形用 */
int32_t Mission_VelGetLat(void);

/* ===================== 对外接口（页面显示） ===================== */

/**
 * @brief  取点位总数
 * @param  无
 * @retval 点位个数（由 MS_POINT_LIST 自动计算）
 */
uint8_t Mission_GetPointCount(void);

/**
 * @brief  取当前选中的点位号（0 起，页面显示用）
 * @param  无
 * @retval 当前点位下标
 */
uint8_t Mission_GetPointIdx(void);

/**
 * @brief  取点位显示名
 * @param  idx: 点位下标（0 ~ 总数-1），越界返回 "??"
 * @retval 静态字符串指针（如 "QR"）
 */
const char *Mission_GetPointName(uint8_t idx);

/**
 * @brief  取当前点位的动作名（页面/日志用）
 * @param  idx: 点位下标，越界返回 "?"
 * @retval "NONE" / "SCAN" / "PICK" / "PLACE" / "VIS"
 */
const char *Mission_GetActName(uint8_t idx);

/**
 * @brief  取当前运行模式文字（页面显示用）
 * @param  无
 * @retval "STEP" / "AUTO"
 */
const char *Mission_GetModeText(void);

/* 赛题页 4 行布局（8x16 大字体，每行最多 16 个 ASCII 字符）：
 *   第1行  MS STEP 3/7 ROU   ← 模式 + 点位序号/总数 + 当前点位名
 *   第2行  T1200,1600h90     ← 目标：T = 目标点，x,y 单位 mm，h 单位 deg
 *   第3行  A800,0h0          ← 实测：A = 实际坐标（里程计），标定/读数用
 *   第4行  IDLE K5=GO        ← 状态 / 到点停留倒计时 / 按键提示
 * 说明：第1~3行都用紧凑写法，保证最宽的情况（含负号）也能完整落在一行内。 */

/**
 * @brief  取页面第 1 行文本：模式 + 点位序号/总数 + 当前点位名
 * @param  无
 * @retval 形如 "MS STEP 3/7 ROU" / "MS AUTO 1/7 QR" / "MS AUTO 7/7 BACK"
 */
const char *Mission_GetTitleText(void);

/**
 * @brief  取页面第 2 行文本：**目标**坐标与朝向
 * @param  无
 * @retval 形如 "T1200,1600h90" / "T0,0h180"
 */
const char *Mission_GetTargetText(void);

/**
 * @brief  取页面第 3 行文本：**实测**坐标与朝向（实测替换点位用）
 * @param  无
 * @retval 形如 "A800,0h0"（单位 mm/deg，取整；负值带 - 号）
 * @note   4.3 节实测替换方法：把车物理摆到目标点，读本行的三个值，
 *         填回 bsp_mission_cfg.h 的 MS_POINT_LIST 对应行。
 */
const char *Mission_GetOdomText(void);

/**
 * @brief  取页面第 4 行文本：阶段 / 到点停留倒计时 / 按键提示
 * @param  无
 * @retval 空闲：STEP "IDLE K5=GO"；AUTO "READY K5=RUN"
 *         执行中：    "RUN TURN" / "RUN MOVE"
 *         到点停留：  "HOLD 1.2s"（剩余秒数，0.1s 分辨率）
 *         STEP 完成： "DONE K5=GO"；AUTO 跑完："ALL DONE"
 *         已中止：    "ABORT"
 */
const char *Mission_GetStateText(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MISSION_H */
