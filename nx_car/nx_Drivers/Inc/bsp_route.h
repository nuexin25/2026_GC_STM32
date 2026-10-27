/**
 ******************************************************************************
 * @file    bsp_route.h
 * @brief   跑图执行器（P7：按"路段表"顺序自动行驶，无障碍物纯跑图测试）
 * @author  nx_car
 * @date    2026-09-17
 ******************************************************************************
 * @note   设计要点：
 *   1. 路段表用 **X-macro** 定义（见下方 ROUTE_SEG_LIST）：增删路段只改这一处，
 *      段数由 sizeof 自动计算，不需要同步改其它文件；
 *   2. 每个路段的语义是"**先转角度 -> 再走位移**"，两个阶段都可缺省：
 *        只转不走：转角填目标角度，x/y 都填 ROUTE_NO_MOVE；
 *        只走不转：转角填 0，x/y 填本段位移；
 *        先转后走：转角与位移都填，执行时自动先转再走；
 *   3. 默认用**绝对坐标点**语义（ROUTE_SEG_RELATIVE = 0）：
 *        x/y = "相对**跑图起点**的目标坐标点"（x 正 = 起点前方、y 正 = 起点右侧）；
 *        执行时用"目标点 − 当前 ODOM 坐标"算车身系增量 → **每段都朝同一个世界点回纠，
 *        单段误差不累积**；代价是**依赖 IMU 朝向 θ 正确**（朝向不对会走偏方向）。
 *        想换成"每段只走一段车身位移（不依赖朝向、但误差逐段累积）"，
 *        把 ROUTE_SEG_RELATIVE 改成 1 并按"前进/右移"重写路段表第 2/3 字段；
 *   4. 运动全部复用 P4 已验证的**位置模式**（Mecanum_MoveBy / Mecanum_MoveByRot），
 *      到位判定复用电机**到位回传** Motor_ReachOk，超时退回脉冲容差兜底；
 *   5. 全部非阻塞，禁 HAL_Delay；日志统一 [ROUTE] 标签，禁 %f（整数打印）。
 *
 *   【怎么改路段】只改下方 ROUTE_SEG_LIST 一处即可，改法见《调参手册》P7 章。
 ******************************************************************************
 */
#ifndef __BSP_ROUTE_H
#define __BSP_ROUTE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* ===================== 路段表（顺序 = 运动顺序） ===================== */
/* 每个路段一行 ROUTE_SEG(转角deg, 目标点x_mm, 目标点y_mm)：
 *   转角 : int16_t，单位 deg，**正 = 逆时针（左转），负 = 顺时针（右转）**，
 *          0 = 本段不旋转；**与页面/日志的 θ（H）同号**，三者一致无需换算；
 *   目标点 : int32_t，单位 mm，**相对"跑图起点"（= 按 KEY3 那一瞬间的位置）的坐标点**：
 *          x 正 = 起点**前方**、x 负 = 起点后方；
 *          y 正 = 起点**右侧**、y 负 = 起点**左侧**；
 *          两个都填 ROUTE_NO_MOVE = 本段不移动（只旋转）。
 *   每段执行时：先转 turn_deg，再用"目标点 − 当前 ODOM 坐标"算车身系增量走过去
 *   （见 ROUTE_SEG_RELATIVE = 0 的说明）—— **每段都朝同一个世界坐标点回纠**。
 *
 * 【新增路段】在下方列表里加一行 ROUTE_SEG(...) 并写一行注释说明该路段；
 * 【删除路段】删掉对应那一行；
 * 【改顺序】调整行的上下顺序（从上到下 = 执行顺序）；
 * 段数自动计算，无需改 bsp_route.c 或其它文件。 */
#define ROUTE_NO_MOVE       (-9999)     /* 位移占位：表示本段无位移 */

/* 路段表语义开关，**当前 = 0（绝对坐标点）**。两种模式下"转角"的含义完全相同。
 *   0 = **绝对坐标点 —— 当前使用**：
 *         x/y 填"相对**跑图起点**的坐标点"；执行时用"目标点 − 当前 ODOM 坐标"
 *         算车身系增量 → **依赖 IMU 朝向 θ 正确**（朝向不对就会走偏方向）；
 *         每段都朝同一个世界点回纠，**单段误差不累积**。
 *   1 = **车体系（相对）**：
 *         x/y 填"本段要走的**车身位移**"：x = 前进 mm、y = 右移 mm
 *         （在"本段旋转完成后"的车头方向上量）；
 *         例如 `ROUTE_SEG(90, 1000, 0)` = 先左转 90°，再**沿车头前进 1000mm**。
 *         **运动不依赖 IMU 朝向**，转弯后一定朝车头走（IMU 只用于坐标显示/漂移补偿）；
 *         代价：各段位移独立执行，机械误差会逐段累积。
 * 选择建议：路径按"每段转向 + 前进/横移多少"来描述时用 1（最直观、最稳）；
 *           需要"无论怎么走都回到同一个世界坐标点"时用 0（依赖朝向调准）。 */
#define ROUTE_SEG_RELATIVE      0

/* ===== Velocity-mode route (pure gyro) ======================================
 * ROUTE_VEL_MODE = 1: each segment is executed in WHEEL VELOCITY mode instead of
 *   position mode. Translation distance comes from the route table and is turned
 *   into a trapezoid speed profile (distance / speed = time); heading is held by
 *   a gyro PID (wz); an in-place turn (differential: all four wheels spin about
 *   the BODY CENTER, no pivot about a wheel) ends when the gyro reaches the target.
 *   No wheel-position feedback is used for the trajectory (pure gyro scheme).
 *   0 = old position-mode executor (kept, still compiled, selectable here).
 *
 * Units: MMPS = mm/s body speed, ACC = mm/s^2, DPS = deg/s body yaw rate.
 * Tuning tips:
 *   - wheels buzz / car does not move at start -> lower ROUTE_VEL_MMPS or
 *     ROUTE_VEL_ACC_MMPS2 (the mecanum slips when accelerating too hard).
 *   - stops short / long -> ROUTE_VEL_DIST_PCT (90..110).
 *   - heading wanders -> raise ROUTE_VEL_YAW_KP (1.0..3.0), add KD for damping.
 *   - turn overshoots -> raise ROUTE_VEL_TURN_KD or lower ROUTE_VEL_TURN_MAX_DPS.
 * ========================================================================== */
#define ROUTE_VEL_MODE            0
#define ROUTE_VEL_MMPS            700.0f   /* cruise speed (mm/s), ~134 wheel RPM */
#define ROUTE_VEL_ACC_MMPS2       380.0f   /* accel / decel (mm/s^2) for the LINEAR
                                              time solver; the profile itself is a
                                              SINE ease, whose peak accel is PI/2
                                              times higher -> 380*1.57 ~= 600. */
#define ROUTE_VEL_CORNER_MMPS     300.0f   /* speed kept THROUGH a corner (blend mode only) */
#define ROUTE_VEL_BLEND_MS        1200U    /* blend mode: start turning this long before end */
#define ROUTE_VEL_BLEND_DPS       100.0f   /* yaw-rate cap during corner blending */
#define ROUTE_VEL_TICK_MS         20U      /* velocity loop period (ms) */
#define ROUTE_VEL_ACC             40U      /* Emm_V5 acc level for 0xF6 frames */
#define ROUTE_VEL_DIST_PCT        100U     /* distance scale (%), 90..110 */
#define ROUTE_VEL_STOP_MS         80U      /* settle after the LAST segment (ms) */

/* 0 = CONTINUOUS (current): slow down to ROUTE_VEL_CORNER_MMPS before the corner
 *     and blend the turn into the last ROUTE_VEL_BLEND_MS of the straight, then
 *     carry on into the next leg WITHOUT stopping (the last leg still stops).
 * 1 = SEGMENTED (test mode): full stop after EVERY segment, then turn in place.
 * When 0, the straight legs still hold their absolute planned heading; only the
 * decel-to-zero cut is disabled (it exists only for the segmented stop). */
#define ROUTE_VEL_SEG_STOP        0U
#define ROUTE_VEL_SEG_STOP_MS     200U     /* wheels-halted wait between segments (ms) */

/* Segmented mode: before turning, wait until the wheels have REALLY stopped
 * (position deltas <= ROUTE_VEL_STOP_TOL_PULSE) for ROUTE_VEL_STOP_HOLD_MS.
 * ROUTE_VEL_STOP_MAX_MS is the hard cap so a missing position frame can never
 * hang the run. 8 pulses ~= 0.8 mm (MEC_PULSE_PER_MM ~= 10.2). */
#define ROUTE_VEL_STOP_TOL_PULSE  8
#define ROUTE_VEL_STOP_HOLD_MS    120U
#define ROUTE_VEL_STOP_MAX_MS     1200U

#define ROUTE_VEL_TURN_TOL_DEG    0.5f     /* arrive deadband (deg) */
#define ROUTE_VEL_TURN_MAX_DPS    100.0f   /* yaw-rate cap (also the braking authority) */
#define ROUTE_VEL_TURN_TIMEOUT_MS 6000U

/* ---- In-place turn: position P -> rate reference -> rate-loop P ------------
 *   rate_ref = min(KP_POS*|err|, sqrt(2*ACC*|err|), MAX), not below MIN while
 *              outside the deadband (overcomes mecanum stiction)
 *   wz       = rate_ref + KP_RATE*(rate_ref - measured rate), capped at +-MAX
 *   arrive   : |err|<=TOL && |rate|<=ARRIVE held for HOLD_MS
 * The rate loop is what removes the overshoot: when the car is faster than the
 * reference the command goes negative (real braking). A big near-target P would
 * shoot through the 0.5 deg deadband at 20ms and oscillate. */
#define ROUTE_VEL_TURN_ACC_DPS2   700.0f   /* deceleration envelope deg/s^2 */
#define ROUTE_VEL_TURN_MIN_DPS    6.0f     /* crawl floor outside the deadband */
#define ROUTE_VEL_TURN_ARRIVE_DPS 10.0f    /* below this rate we may finish */
#define ROUTE_VEL_TURN_KP_POS     4.0f     /* position P: (deg/s) per deg */
#define ROUTE_VEL_TURN_KP_RATE    1.8f     /* rate-loop P (brakes when too fast) */
#define ROUTE_VEL_TURN_HOLD_MS    80U      /* deadband must hold this long */

#define ROUTE_VEL_YAW_KP          1.6f     /* heading hold while translating */
#define ROUTE_VEL_YAW_KD          0.30f    /* damping on measured yaw rate */
#define ROUTE_VEL_YAW_MAX_DPS     30.0f

/* ===== Position closed loop on the ENCODER odometry =========================
 * The velocity-mode legs use Chassis_GetOdomX/Y (wheel-pulse odometry, origin =
 * route start) as their position feedback: each leg heads for its table target
 * from the MEASURED position, and after the last segment a "home" approach
 * drives the car to (0,0) using the same odometry, so every lap ends at 0,0.
 *   ROUTE_VEL_HOME_ENABLE  : 1 = do the final (0,0) approach, 0 = finish as-is
 *   ROUTE_VEL_POS_TOL_MM   : endpoint tolerance, mm
 *   ROUTE_VEL_HOME_KP      : approach speed gain (mm/s per mm)
 *   ROUTE_VEL_HOME_MIN_MMPS: minimum crawl speed (overcomes mecanum stiction)
 *   ROUTE_VEL_HOME_MAX_MS  : hard timeout for the final approach
 * ========================================================================== */
#define ROUTE_VEL_HOME_ENABLE     1U
#define ROUTE_VEL_POS_TOL_MM      10.0f
#define ROUTE_VEL_HOME_KP         2.0f
#define ROUTE_VEL_HOME_MIN_MMPS   40.0f
#define ROUTE_VEL_HOME_MAX_MS     5000U

/* ---- 本表的几何参数：**正方形测试圈（4 段直行 + 4 个右转 90°）** ----
 * 只改 RT_SQ_S 就能整体缩放边长，路段表本身不用动。
 * 路径（起点在 (0,0)，车头朝 +x = 0°）：
 *   (0,0) --沿 0° 直行S--> (S,0)
 *         --右转90°--> 沿 -90° 直行S --> (S,S)
 *         --右转90°--> 沿 180° 直行S --> (0,S)
 *         --右转90°--> 沿 90° 直行S  --> (0,0)
 *         --停稳后原地右转90°--> 车头回到 0°，结束 */
#define RT_SQ_S         1500      /* 正方形边长，单位 mm */

/* 当前路段表（**绝对坐标点**）—— 每段"先转 turn_deg，再走到目标点"。
 * 转角：负 = 顺时针（右转）；目标点相对"跑图起点"，x 正 = 前方、y 正 = 右侧。
 * 配合 ROUTE_VEL_SEG_STOP=1：每段先停稳，再执行下一段的转角。 */
#define ROUTE_SEG_LIST                                                              \
    ROUTE_SEG(0,      RT_SQ_S,  0)          /* ① 沿 0° 直行（边长 S） */             \
    ROUTE_SEG(-90,    RT_SQ_S,  RT_SQ_S)    /* ② 右转90°，沿 -90° 直行 */           \
    ROUTE_SEG(-90,    0,        RT_SQ_S)    /* ③ 右转90°，沿 180° 直行 */           \
    ROUTE_SEG(-90,    0,        0)          /* ④ 右转90°，沿 90° 直行回原点 */      \
    ROUTE_SEG(-90,    ROUTE_NO_MOVE, ROUTE_NO_MOVE)  /* ⑤ 停稳后原地右转90°，车头回 0° */ \
/* ===================== 可调参数宏（集中调参区） ===================== */

/* 单个路段的总超时，单位 ms，默认 15000，可调范围 5000~60000。
 * 从"本段开始"计时，覆盖本段的旋转阶段 + 平移阶段；
 * 超时立即急停并把状态置 ABORT，防止因位置数据异常导致车轮一直转。 */
#define ROUTE_SEG_TIMEOUT_MS      30000U

/* 到位回传等待上限，单位 ms，默认 2000，可调范围 200~10000。
 * 【2026-10-15 提速】5000 -> 2000：本值只在"某轮一直没收到到位回传"时才生效
 * （正常收到回传立刻放行，与本值无关）。原来 5s 太长，一旦漏一帧就会让整车在
 * 原地干等几秒；降到 2s 后仍会走"脉冲容差"兜底，**不会因为等不到回传而丢命令或跑丢**。
 * 本阶段（旋转或平移）下发后超过本值仍未收齐四轮到位回传，
 * 就退回"脉冲容差"判定并打印 `reach timeout, fallback to pulse`。
 * 前提：4 个电机菜单 Response 已设为 Reached / Other / Both（本项目为 Other）。 */
#define ROUTE_REACH_FALLBACK_MS   2000U

/* 脉冲容差兜底判据，单位 脉冲，默认 50，可调范围 10~200。
 * 判据：四轮"已走脉冲 − 目标脉冲"的绝对值都小于本值即认为到位。
 * 参考量级：50 脉冲 ≈ 50 / MEC_PULSE_PER_MM ≈ 5mm（几何参数默认值下）。 */
#define ROUTE_PULSE_TOL           50

/* 到坐标点的允许误差，单位 mm，默认 5，可调范围 2~20。
 * 仅用于**日志提示**：某段到位后若 |目标 − 实测 ODOM| 超过本值，
 * 打印 `pos err x=… y=…` 提醒（不影响状态机推进）。 */
#define ROUTE_DIST_TOL_MM         2

/* 路段"转角达成度"的**提示**阈值，单位 deg，默认 30，可调范围 10~90。
 * 旋转到位后把"实测转角"与"命令转角"比较，超出本值就打印一行提示
 * （含建议的转角刻度值），**不会中止跑图**——后续路段用**实测 θ** 算目标增量，
 * 所以"转多/转少"不会把车带离目标点，只影响"是否正好朝车头方向走"。
 * 要转准请按 3.10 节用日志给出的建议值标定 `MEC_ROT_CMD_GAIN_NUM/DEN`。 */
#define ROUTE_TURN_TOL_DEG        1

/* "转角是否真的发生了"的判据下限，单位 deg，默认 15，可调范围 5~45。
 * 命令转角足够大（≥ ROUTE_TURN_TOL_DEG）时，若实测 |转角| 小于本值，
 * 说明朝向数据可疑（IMU 无数据/冻结），日志会打印 `hint: yaw not updating`
 * 供排查；**仅为提示**，不影响状态机推进。 */
#define ROUTE_TURN_ALIVE_DEG      15

/* 转弯到位后、下发本段平移前的**朝向等待**上限，单位 ms，默认 120，可调范围 0~2000。
 * 【2026-10-15 提速】300 -> 120：见 bsp_route.c Route_YawSettleWait 说明。
 *   本值只在"朝向还没到期望值、且确实在缓慢变化"时才起收尾作用；
 *   绝大多数情况（朝向已到 / 已被零漂窗口覆盖）都走"立刻放行"，与本值无关。 */
#define ROUTE_YAW_SETTLE_MS       120U

/* 朝向等待的硬上限，单位 ms，默认 2000，可调范围 300~5000。
 * 转弯后最多等这么久：到点仍未达到期望朝向就按当前值规划，并打印
 * `yaw NOT ready ... -> check IMU refresh` 报警（说明 IMU 刷新太慢/异常）。
 * 若日志频繁出现该报警，说明模块输出周期偏长，可提高它的输出速率
 * （HWT101CT 的 RRATE 寄存器，出厂默认 10Hz；同时需相应提高波特率）。 */
#define ROUTE_YAW_SETTLE_MAX_MS   2000U

/* [2026-10-07] Landing-point closed loop: max extra "walk a short leg" rounds allowed
 * per segment after arrival + settle (default 3, range 0~5). 0 = off (old behaviour).
 * Only effective in ABSOLUTE mode (ROUTE_SEG_RELATIVE == 0, i.e. seg->x/y are target
 * coordinates): Route_IssueMoveAbs is re-issued with the SAME target so that its
 * body-frame delta is recomputed from the MEASURED position -- slip / scale error
 * gets corrected. Loops stop as soon as the rounded delta is 0 (pulse resolution).
 * Cost: 1~3 extra "walk + settle" cycles per waypoint (a few hundred ms each). */
#define ROUTE_POS_LOOP_MAX        3U

/* 转弯到位后的**静止零漂测量窗口**，单位 ms，默认 320，可调范围 0~2000；0=关闭。
 * 这段静止时间里车不动，IMU 的 yaw 若还在缓慢变化，那就是陀螺零漂；测出速率后
 * 由 bsp_chassis 按时间线性扣除（每次转弯校准一次），抑制长路径上"世界坐标系
 * 缓慢旋转"造成的累计误差。代价：每次转弯多停这么久（与朝向等待取较大者）。
 * 只想测不想补时，把 bsp_chassis.h 的 CHS_YAW_DRIFT_COMP_ENABLE 置 0。
 * 【2026-10-15 提速】350 -> 320，并**改为每圈只在第一次转弯时强制等待**：
 *   零漂是缓变量，一圈测一次足够；从第二次转弯起不再强制停，衔接时间只留 holdMs。
 *   必须 ≥ bsp_chassis.h 的 CHS_YAW_DRIFT_CAL_MS(300)，否则 bsp_chassis 会认为
 *   窗口太短而丢弃本次估计（"只测不补"）——故下限按下 320 留余量。 */
#define ROUTE_DRIFT_CAL_MS        320U

/* ===================== 运行期可调参数（在线调参 / 自动调车用） ===================== */
/* 【为什么要有这个结构体】上面的 ROUTE_* / MEC_* 都是编译期宏，现场调一次就要
 *   重编译+烧录一次，PC 端没法自动调。这里把最影响"转 90° + 直行"精度的量
 *   做成**运行期变量**（初值 = 上面那些宏），通过 bsp_tune 的串口协议在线改，
 *   PC 脚本就能"跑一圈 → 读数据 → 改参数 → 再跑"自动收敛。
 * 【注意】这里的值**掉电不保存**（只在本次上电有效）；调好后要把最终值抄回宏里。 */
typedef struct {
    /* --- 旋转 --- */
    uint16_t rotScaleNum;    /* 旋转弧长缩放分子（默认 100）：90° 转不够就加大 */
    uint16_t rotScaleDen;    /* 旋转弧长缩放分母（默认 100） */
    uint16_t rotVel;         /* 原地旋转速度（车轮 RPM，默认 MEC_ROT_VEL=120）：打滑就继续降低 */
    /* --- 平移 --- */
    uint16_t distScaleNum;   /* 平移位移缩放分子（默认 100）：走短了加大 */
    uint16_t distScaleDen;   /* 平移位移缩放分母（默认 100） */
    /* --- 跑圈 --- */
    uint16_t tableScalePct;  /* 整张路段表缩放（%，默认 100）：调车时缩成小圈，如 40 */
    uint8_t  lapRepeat;      /* 连跑圈数：0=只跑一次；1~254；255=无限（PC 自动调参用） */
    /* --- 到位后的"统一停轮 + 停稳"--- */
    uint16_t holdMs;         /* 判定到位后：四轮同拍瞬停 + 静置多久再进下一段（ms，默认 30）。
                                0 = 仍然同拍停四轮，但不额外等待（立刻进下一段）。
                                作用：消除四轮独立闭环"先停的被后停的推着走"造成的
                                减速段末端横移/走偏，并让残余运动彻底停下再下发下一段。
                                【2026-10-15 提速】100 -> 30：本值只压缩"死等"，
                                0xFE 同拍停照旧；CAN 下发前有"等空闲发送邮箱"保护，
                                压缩本值不会丢命令。 */
    /* --- 转角闭环（按 HWT101CT 回传的 θ 补转，只作用于"旋转阶段"） --- */
    uint8_t  yawEnable;      /* 转角闭环总开关：1=启用（默认）；0=关闭，退回纯开环转角 */
    uint16_t yawTolDeg;      /* 转角容差（deg，默认 2）：|目标θ − 实测θ| ≤ 本值即达标放行 */
    uint16_t yawCorrPct;     /* 单次**只补误差的百分之几**（默认 70，范围 20~100）：
                                留余量防"补过头 → 反向偏 → 再补 → 来回摆" */
    uint16_t yawMaxCorrX10;  /* **单次补转上限**（0.1°，默认 150 = 15°）：超过就限幅并打
                                `yaw corr clamp` —— 误差十几度以上通常是 yaw 跳变/回绕/刚归零，
                                不该按异常值硬转那么多 */
    uint8_t  yawMaxRound;    /* 单段最多补转轮次（默认 2，范围 1~5） */
    uint16_t yawBudgetMs;    /* 单段转角闭环时间预算（ms，默认 1200，范围 200~3000）：
                                超了打 `yaw giveup` 放行，**绝不中止跑图** */
} RouteCfg_t;

/**
 * @brief  取运行期可调参数的指针（在线调参直接改这个结构体）
 * @param  无
 * @retval 指向全局配置的指针（恒非空）
 * @note   初值在 Route_Init 里由对应宏赋一次；改完**立刻生效**（下一段就按新值走）。
 *         画页/日志不需要读它，调参协议（bsp_tune）与自动化脚本才会用。
 */
RouteCfg_t *Route_GetCfg(void);

/**
 * @brief  把运行期可调参数恢复成 bsp_route.h 里的默认值
 * @param  无
 * @retval 无
 * @note   在线调参命令 `DEFAULT` 用它；跑图进行中也可以调用（下一段生效）。
 */
void Route_CfgReset(void);

/* ===================== 状态/阶段枚举 ===================== */

/* 跑图总状态 */
typedef enum {
    ROUTE_STATE_IDLE = 0,   /* 空闲（未启动/已退出） */
    ROUTE_STATE_RUN,        /* 跑图中             */
    ROUTE_STATE_DONE,       /* 全部路段跑完       */
    ROUTE_STATE_ABORT       /* 已中止/超时        */
} RouteState_t;

/* 当前段所处阶段（页面第3行显示用） */
typedef enum {
    ROUTE_PHASE_TURN = 0,   /* 本段旋转阶段（下发 + 等到位） */
    ROUTE_PHASE_MOVE,       /* 本段平移阶段（下发 + 等到位） */
    ROUTE_PHASE_WAIT        /* 空闲/未推进（IDLE/DONE/ABORT） */
} RoutePhase_t;

/* ===================== 函数声明 ===================== */

/**
 * @brief  跑图模块初始化（只复位状态机，不发电机命令）
 * @param  无
 * @retval 无
 * @note   上电初始化阶段禁止使能/转动车轮；四轮使能只发生在 Route_Start。
 */
void Route_Init(void);

/**
 * @brief  启动跑图（清零相对坐标 + 段索引归 0 + 置 RUN）
 * @param  无
 * @retval 无
 * @note   内部动作顺序：
 *           1) 先确认能读到四轮位置，读不到则拒绝启动并日志 `no pos data, start rejected`
 *              （里程计坐标与目标增量都依赖位置数据，读不到会朝错误坐标走）；
 *           2) 使能四轮（位置模式必须先使能，否则电机不收位置命令）；
 *           3) Chassis_OdomReset()：原点 = 当前位置、朝向基准 = 当前 yaw；
 *           4) 段索引归 0、阶段置 TURN、状态置 RUN，按 ROUTE_SEG_LIST 逐段执行。
 *         车体系模式下各段位移独立执行，误差会逐段累积；绝对点模式每段朝
 *         同一个世界点回纠，单段误差不累积（见 ROUTE_SEG_RELATIVE）。
 */
void Route_Start(void);

/**
 * @brief  中止跑图（四轮急停 + 失能 + 状态置 ABORT）
 * @param  无
 * @retval 无
 * @note   用于人工中止、退出页面或单段超时兜底。正常跑完不需要本函数
 *         （位置模式电机到点自停）。
 */
void Route_Abort(void);

/**
 * @brief  跑图状态机推进（主循环调用，非阻塞）
 * @param  无
 * @retval 无
 * @note   每拍按当前阶段推进：
 *           ROUTE_PHASE_TURN：首次进入 -> 下发原地旋转（转角的弧长）；
 *                             等待到位 -> 换 ROUTE_PHASE_MOVE；
 *                             本段转角为 0 时直接跳到 MOVE；
 *           ROUTE_PHASE_MOVE：首次进入 -> 按路段表语义算出车身系增量并下发位移
 *                             （车体系模式直接用表里数值，绝对点模式先做坐标变换）；
 *                             等待到位 -> 本段完成，segIdx++ 并回到 TURN；
 *                             本段位移为 ROUTE_NO_MOVE 时直接完成本段；
 *           段索引走完 ROUTE_SEG_COUNT -> DONE。
 *         到位优先等电机回传（Motor_ReachOk），超时退脉冲容差；两个判据都不中止跑图；
 *         **漏调度会导致永远不判到位**（电机自己会走完，软件停在 RUN）。
 */
void Route_Update(void);

/**
 * @brief  查询是否正在跑图
 * @param  无
 * @retval 1=跑图中（RUN），0=其它状态
 */
uint8_t Route_IsActive(void);

/**
 * @brief  查询是否跑图完成
 * @param  无
 * @retval 1=全部路段跑完（DONE），0=其它状态
 */
uint8_t Route_IsDone(void);

/**
 * @brief  取当前路段号（0 起，显示用）
 * @param  无
 * @retval 当前段索引；DONE 后等于总段数
 */
uint8_t Route_GetSegIdx(void);

/**
 * @brief  取总路段数
 * @param  无
 * @retval 路段数（由 ROUTE_SEG_LIST 自动计算）
 */
uint8_t Route_GetSegCount(void);

/**
 * @brief  取本段/本阶段"哪些轮被下过命令"的位掩码
 * @param  无
 * @retval bit0~bit3 依次对应 FL/FR/RL/RR：1=该轮有命令（应转动），0=位移为 0（保持不动）
 * @note   用于区分"轮子不动"的原因：
 *         **位为 0 是正常的**（如 45° 斜行只需两个轮）；位为 1 却没动才是异常。
 *         页面第 2 行显示为 `w:1001`（依次 FL/FR/RL/RR）。
 */
uint8_t Route_GetWheelMask(void);

/**
 * @brief  取跑图状态文字（页面显示用）
 * @param  无
 * @retval "IDLE" / "RUN" / "DONE" / "ABORT"
 */
const char *Route_GetStateText(void);

/**
 * @brief  取当前段阶段文字（页面显示用）
 * @param  无
 * @retval "TURN" / "MOVE" / "WAIT"（空闲/未推进）
 */
const char *Route_GetPhaseText(void);

/**
 * @brief  取当前段目标文字（页面第4行显示用）
 * @param  无
 * @retval 静态缓冲指针，形如：
 *         车体系模式（当前）："T90 f1890 r0" / "f1890 r0" / "turn90" / "-"
 *         绝对点模式：        "T90 (500,500)" / "to(0,500)" / "turn90" / "-"
 * @note   转角非 0 时前缀 "T<角度>"，位移/坐标非 ROUTE_NO_MOVE 时附具体数值
 *         （车体系是 "f<前进> r<右移>"，绝对点是 "to(x,y)"）；
 *         两者都没有（占位段）显示 "-"。
 */
const char *Route_GetSegText(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ROUTE_H */
