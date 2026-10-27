/**
 ******************************************************************************
 * @file    bsp_chassis.h
 * @brief   车机运动序列（P4：整车位置模式运动 + 到位判定 + 航向纠偏）
 * @author  nx_car
 * @date    2026-09-16
 ******************************************************************************
 * @note   本模块把 bsp_mecanum 的整车位置模式运动封装成**非阻塞状态机**，
 *         供车机测试页人工触发。
 *
 *         为什么用位置模式：电机固件内部用编码器闭环数脉冲，位置精度不受
 *         CAN 往返延迟影响；速度模式靠上层每 80ms 轮询里程判到位，
 *         末端必然过冲几 mm~几十 mm。
 *
 *         位置模式没有实时速度环，所以**航向漂移不能实时修正**，改为
 *         "分段运动 + 每段到位后纠偏"：平移类动作到位后读 IMU yaw，
 *         偏差超过 CHS_YAW_TOL_DEG 就自动补一次原地旋转纠偏。
 *
 *         注意 ARC 与 SPIN 是**两种不同的运动**，不要混淆：
 *           - ARC ：沿圆弧"拐弯"——车体始终朝自己的前方走，世界系轨迹是弧线，
 *                   终态是"沿弧线走 CHS_ARC_DIST_MM + 车头方向转了 CHS_ARC_DEG"；
 *                   用**位置模式**一次下发（等时长法保证四轮同时到位）。
 *           - SPIN：**先纯直行 CHS_SPIN_DIST_MM，到位后再原地自转 CHS_SPIN_DEG**
 *                   （分两段顺序执行，段内都是纯运动，不存在四轮比例失真）。
 *                   终态是"前进 CHS_SPIN_DIST_MM + 车身朝向转了 CHS_SPIN_DEG"。
 *
 *         硬约束：全部函数非阻塞，禁止 HAL_Delay；日志禁 %f。
 ******************************************************************************
 */
#ifndef __BSP_CHASSIS_H
#define __BSP_CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* ===================== 可调参数宏（集中调参区） ===================== */

/* 直行/后退/横移目标距离，单位 mm，默认 500，可调范围 100~2000 */
#define CHS_DIST_MM             500

/* 原地旋转目标角度，单位 deg，默认 90，可调范围 90~720；正=逆时针 */
#define CHS_ROT_DEG             90

/* 斜 45° 行进的"前进分量"距离，单位 mm，默认 300，可调范围 100~1000；
   dx = dy = 本值，合成方向为斜前方 45° */
#define CHS_DIAG_DIST_MM        300

/* ARC 动作的前进距离，单位 mm，默认 500，可调范围 100~2000 */
#define CHS_ARC_DIST_MM         500

/* ARC 动作的目标转角，单位 deg，默认 90，可调范围 30~360 */
#define CHS_ARC_DEG             90

/* ===================== 直行 + 自转（SPIN 动作） ===================== */
/* 动作含义：**先沿直线前进 CHS_SPIN_DIST_MM，到位后再原地自转 CHS_SPIN_DEG**。
 *           终态是"前进 500mm + 车身朝向转了 90°"（相对动作起点）。
 *
 * 【实现方式：位置模式 . 分两段顺序执行】
 *   第一段：纯直行 CHS_SPIN_DIST_MM（等价于 FWD，四轮位移等长）
 *   第二段：纯原地自转 CHS_SPIN_DEG（等价于 ROT，四轮两两反向）
 *   两段都是纯运动、四轮位移成对相等，**不存在四轮比例失真问题**，
 *   所以用已验证可靠的**位置模式**即可，精度最好。
 *   代价：不是"边走边转"，而是"先走完再转"（顺序完成）。
 *
 *   > 曾经试过两种"边走边转"的实现，都已放弃，原因记录在此避免重复踩坑：
 *   >   1) 位置模式分段逼近：把转角拆成 N 段逐段下发。每段只有几十 ms，
 *   >      电机固件的加减速段占满整段、四轮速度比例失效、麦轮互相刮擦，
 *   >      **车身几乎走不动**；
 *   >   2) 速度模式实时解算（每拍把速度按当前航向反向旋转）：实测同样不行。
 *   > 所以本动作最终采用"分两段顺序执行"。 */

/* 直线行程，单位 mm，默认 500，可调范围 100~2000（第一段） */
#define CHS_SPIN_DIST_MM        500

/* 自转角度，单位 deg，默认 90，可调范围 15~360（第二段）；正=逆时针 */
#define CHS_SPIN_DEG            90

/* 第二段航向漂移补偿的**有效范围**，单位 deg，默认 30，可调范围 5~90。
 *
 * 背景：第二段实际下发角度 = CHS_SPIN_DEG − 第一段造成的航向漂移，
 *       这样终态朝向正好是"起始朝向 + CHS_SPIN_DEG"。
 *       正常直行的漂移只有几度，所以只在 |漂移| < 本值时才补偿。
 *
 * **为什么必须有这个范围限制**：若陀螺仪未初始化/数据异常，漂移值可能离谱地大
 * （实测出现过接近 90° 的情况），此时 `CHS_SPIN_DEG − 漂移` 会算成 0，
 * 导致第二段**完全不下发、直接判到位**（现象：直行正常但完全不转，页面 `a=` 停在 0）。
 * 超出本范围就忽略补偿、直接转 CHS_SPIN_DEG，并在串口打
 * `spin drift=… out of range, ignore` 提示。
 * 若确认漂移值可靠且需要更大补偿范围，可调大到 60。 */
#define CHS_SPIN_DRIFT_MAX_DEG  30.0f

/* 段到位判据开关，默认 **1（电机到位回传优先）**：
     1 = 优先用电机**主动回传的到位帧**（`地址 + FD + 9F + 6B`）判断本段是否走完。
         这是电机自己的到位判据（位置进入"位置到达窗口"才算，默认窗口仅 0.1~0.3°，
         比脉冲容差严格得多），不依赖 CAN 往返延迟，最可靠。
         **前提：4 个电机的菜单 `Response` 必须设为 `Reached` / `Other` / `Both`**
         （出厂默认 `Receive` 只回"收到"命令、不回到位帧）。
         本项目实测设为 **`Other`**（位置模式命令只回到位帧、不返"收到"确认），
         对判据最干净：不会出现 `FD 02` 确认帧干扰。
         若回传迟迟不来（漏帧或没配菜单），超过 CHS_REACH_FALLBACK_MS 会自动退回
         脉冲容差判定并打印 `reach timeout, fallback to pulse`，不会挂死。
     0 = 只用四轮脉冲容差判定（旧行为，不依赖电机菜单设置）。
   要一键退回旧行为就改成 0。 */
#define CHS_USE_MOTOR_REACH     1

/* 到位回传等待上限，单位 ms，默认 2000，可调范围 200~10000。
   段开始后超过本值仍未收齐四轮到位回传，就退回脉冲容差判定并打印提示。
   调小会让"回传优先"退化成"谁先到算谁"；调大则漏帧时本段结束更慢。 */
#define CHS_REACH_FALLBACK_MS   2000U

/* 到位判定的脉冲容差，默认 50，可调范围 10~200。
   判据：四轮"已走脉冲 − 目标脉冲"的绝对值都小于本值即认为到位。
   参考量级：50 脉冲 ≈ 50 / MEC_PULSE_PER_MM ≈ 1.6mm（默认几何参数下）。
   调法：精度要求高就调小；若因丢帧/抖动迟迟不判到位可适当调大。 */
#define CHS_PULSE_TOL           50

/* 单个运动段的总超时，单位 ms，默认 15000，可调范围 5000~60000。
   每段（主运动段 / 纠偏段 / ARC 第二段）各自计时；
   超时立即急停并把状态置 ABORT，防止因位置数据异常导致车轮一直转。 */
#define CHS_TIMEOUT_MS          30000U

/* 航向纠偏触发阈值，单位 deg，默认 2.0f，可调范围 1~10；
   平移类动作到位后 |当前yaw − 起始yaw| 超过本值就补一次原地旋转纠偏。
   直行跑偏明显就调小（代价是纠偏动作变多）。 */
#define CHS_YAW_TOL_DEG         3.0f

/* ============ 坐标系与旋转方向的符号基准（P7 约定的唯一出口） ============ */
/* 相对坐标系（原点 = 启动点，俯视）：
 *    +X = 启动时车头方向（前）     -X = 车尾
 *    +Y = 启动时车右方向（右）     -Y = 车左
 * θ（航向角）正方向约定：**俯视逆时针为正**（车头由 +X 转向 -Y）。
 * 车身方向单位向量（全局系）：车头 = (cosθ, -sinθ)，车右 = (sinθ, cosθ)。
 * 正变换（车身位移 -> 全局，里程计用）：X += dx·cosθ + dy·sinθ
 *                                      Y += -dx·sinθ + dy·cosθ
 * 逆变换（全局位移 -> 车身，跑图"到坐标"用）：
 *                                      dx =  X·cosθ - Y·sinθ
 *                                      dy =  X·sinθ + Y·cosθ
 * 两式互为转置（严格互逆），**里程计与跑图必须共用同一个 θ**。
 *
 * 下面这个宏定义 θ 与 IMU 原始偏航角的换算关系，**必须实测确认一次**：
 *   1 = 俯视**逆时针**旋转 90° 时 IMU_GetYaw() 增大（≈+90）——与上述 θ 同向（标准）；
 *   0 = 俯视逆时针旋转 90° 时 IMU_GetYaw() 减小（≈-90）——与上述 θ 反向，需取反后用。
 * 实测步骤（《调参手册》P7 章 5 节）：
 *   1) 上电后按 KEY3 让 IMU 归零（IMU_ResetYaw）；
 *   2) 手动把整车**俯视逆时针**转 90°（车头转到初始的车左方向）；
 *   3) 读串口 `[IMU] yaw=` 或数据页 `yaw=`：≈+90 则置 1，≈-90 则置 0。
 * 【作用范围】本宏一经确定，里程计正变换、跑图逆变换、页面/日志的 H、
 *  P4 航向纠偏方向、SPIN 漂移补偿全部随之统一，不需再改别处。
 *  实测结论（2026-09-17）：**1**（已实测确认）。
 *   依据：给"左转 90°"（路段表里写 +90）时，`[IMU] yaw=` 反馈的值是 **+90**，
 *         反向同理（右转 90° 反馈 -90）——即 IMU 偏航角**逆时针为正**，
 *         与上面 θ 的定义同向，故直接用，本宏 = 1。
 *         另外：路段表转角的约定已统一为"正=逆时针"，与 θ 同号（见 bsp_route.h）；
 *         电机侧的符号转换被封装在 bsp_route 的 Route_IssueRot() 里。 */
#define IMU_YAW_CCW_POSITIVE    1

/* 里程计原始数据日志周期，单位 ms，默认 500，可调范围 0~5000；0=关闭。
   >0 时在**运动中**按本周期打印一行 `odom p=… x=… y=… h=…`：
   `p=` 是换算到**下发脉冲域**的四轮位置（跑 500mm 时单轮增量应约 5093），
   `h=` 是相对坐标原点朝向的转角 ×10（单位 0.1°，便于判断 IMU 朝向有没有跟着转）。
   用于标定刻度/排查坐标异常。日常运行建议改 0 避免刷屏。 */
#define CHS_ODOM_LOG_MS         500U

/* ★里程计读位置的节拍（ms）。0=每拍都读（旧行为）。
   旧行为下每拍要发 4 帧 CAN 位置查询，主循环 1kHz 时就是 4000 帧/s，
   把 CAN 邮箱挤满（日志 `can tx slot wait=NNN` 一直涨）——**速度指令被推迟**，
   四个轮子收到指令的时刻不同，表现为车头左右甩、刹车时被甩 10°+。
   100Hz 对里程计完全够用（积分用的是脉冲增量，与采样率无关）。 */
#define CHS_ODOM_READ_MS        10U
/* ================= 静止零漂补偿（每次转弯后校准角度） =================
 * 原理：车**静止**时 yaw 的缓慢变化就是陀螺零漂；在每次转弯到位后的静止窗口里
 *       测出漂移速率，之后按时间从 yaw 中线性扣除（里程计与跑图规划共用同一个 θ，
 *       所以两边同时受益），抑制长路径上"世界坐标系缓慢旋转"造成的累计误差。
 * 安全措施：只在确认静止的窗口测量；估计值限幅；一阶平滑；日志可核对；
 *           置 CHS_YAW_DRIFT_COMP_ENABLE=0 即完全退回原行为（只测不补）。 */
#define CHS_YAW_DRIFT_COMP_ENABLE  1        /* 1=启用零漂补偿；0=关闭（只测不补） */
#define CHS_YAW_DRIFT_CAL_MS       300U     /* 静止测量窗口，单位 ms，范围 100~2000 */
#define CHS_YAW_DRIFT_MAX_DPS      (2.0f)   /* 零偏估计限幅，单位 °/s；超出视为未停稳/干扰，丢弃本次 */
#define CHS_YAW_DRIFT_SMOOTH       (0.3f)   /* 一阶平滑权重，范围 0.05~1.0；越小越稳、跟随越慢 */

/* ARC 实现方式开关，默认 0：
     0 = **等时长法**（四轮速度按位移比例缩放，边走边转中途保持正确弧线、
         四轮同时到位；由 bsp_mecanum 的 Mecanum_MoveBy 实现）
     1 = **拆两段**（先直行到位，再原地旋转；四轮位移等长、无横向刮擦，
         精度最好，但不满足"同时"要求）
   若实测发现方式 0 的弧线形状或同时性不达标，把本宏改成 1 即切换，
   无需改其它代码。 */
#define CHS_ARC_SPLIT           0

/* ===================== 速度模式单步调试（P4 车机测试页） =====================
 * CHS_VEL_MODE = 1：测试页每个动作改用**轮速模式 + 陀螺仪闭环**执行，
 *   与 P7 跑图速度模式同一套做法，方便逐个动作单独验证（不依赖位置数据）：
 *     FWD/BACK/LEFT/RIGHT/DIAG : 梯形速度曲线 + 陀螺仪锁航向（走直线自纠角）；
 *     ROT                      : 绕车体中心**差速原地转**（陀螺仪到位，可 >180°）；
 *     ARC                      : 边走边转（前进 + 按距离线性给航向斜坡），
 *                                走完再补一次原地转把残差收到位；
 *     SPIN                     : 先直行、停稳后再原地转（两段）。
 *   0 = 原来的位置模式执行器（完整保留，可随时切回）。
 * 注意：速度模式**不需要四轮位置数据**，启动时不再要求 0x36 位置帧；
 *       进度页的 `D=` 仍按编码器脉冲统计，`a=` 改为陀螺仪实际转角。
 * ========================================================================== */
#define CHS_VEL_MODE            0
#define CHS_VEL_MMPS            700.0f   /* 巡航速度 mm/s */
#define CHS_VEL_ACC_MMPS2       380.0f   /* 线性求解用的加减速 mm/s^2；实际曲线是正弦，
                                             峰值加速度 = 本值 * PI/2 ≈ 600 */
#define CHS_VEL_DIST_PCT        100U     /* 距离补偿 %：走短了加到 102~108 */
#define CHS_VEL_TICK_MS         20U      /* 速度环节拍 ms */
#define CHS_VEL_ACC             40U      /* 0xF6 帧的加减速档 */
#define CHS_VEL_STOP_MS         200U     /* 段末停稳等待 ms */

/* ---- 原地转：位置 P -> 速度参考 -> 速度环 P（一条连续律，无分段切换）----
 *   rate_ref = min(KP_POS*|err|, √(2*ACC*|err|), MAX)，|err|>TOL 时不低于 MIN
 *   wz       = rate_ref + KP_RATE*(rate_ref − 实测角速度)，限幅 ±MAX
 *   |err|≤TOL 且 |rate|≤ARRIVE 且保持 HOLD_MS -> 完成
 * 为什么不再用"近段大 Kp"：20ms 采样 + 麦轮打滑时，大 Kp 会一次冲过死区再反向，
 * 表现就是来回振荡；速度环在末端会自动给出反向刹车量。 */
#define CHS_VEL_ROT_TOL_DEG     0.5f     /* 到位死区 deg */
#define CHS_VEL_ROT_ACC_DPS2    700.0f   /* 减速包络 deg/s²（越大越晚减速、越快） */
#define CHS_VEL_ROT_MAX_DPS     100.0f   /* 转角速度上限 deg/s（也是刹车能力上限） */
#define CHS_VEL_ROT_MIN_DPS     6.0f     /* 死区外的最低爬行速度 deg/s（克服静摩擦） */
#define CHS_VEL_ROT_ARRIVE_DPS  10.0f    /* 低于此角速度才允许判定到位 */
#define CHS_VEL_ROT_KP_POS      4.0f     /* 位置 P：deg/s per deg */
#define CHS_VEL_ROT_KP_RATE     1.8f     /* 速度环 P：实际偏快时输出转负 -> 刹车 */
#define CHS_VEL_ROT_HOLD_MS     80U      /* 死区需保持多久才算完成 */

#define CHS_VEL_YAW_KP          1.6f     /* 直行航向保持 P */
#define CHS_VEL_YAW_KD          0.30f    /* 直行航向保持 D（实测角速度） */
#define CHS_VEL_YAW_MAX_DPS     30.0f    /* 航向修正上限 deg/s */
#define CHS_VEL_ARC_DPS_CAP     110.0f   /* ARC 边走边转时的角速度上限 deg/s */

/* ===================== 动作枚举 ===================== */
typedef enum {
    CHS_ACT_FWD = 0,   /* 直行前进 CHS_DIST_MM      */
    CHS_ACT_BACK,      /* 直行后退 CHS_DIST_MM      */
    CHS_ACT_LEFT,      /* 左横移 CHS_DIST_MM        */
    CHS_ACT_RIGHT,     /* 右横移 CHS_DIST_MM        */
    CHS_ACT_ROT,       /* 原地旋转 CHS_ROT_DEG      */
    CHS_ACT_DIAG,      /* 斜 45° 行进（前进+右移等分量） */
    CHS_ACT_ARC,       /* 直行同时旋转（前进 CHS_ARC_DIST_MM + 转 CHS_ARC_DEG） */
    CHS_ACT_SPIN,      /* 直行 + 自转：先直行 CHS_SPIN_DIST_MM，到位后原地自转 CHS_SPIN_DEG（两段顺序执行） */
    CHS_ACT_COUNT      /* 动作总数（仅用于边界判断，不是有效动作） */
} ChassisAct_t;

/* ===================== 状态枚举 ===================== */
typedef enum {
    CHS_STATE_IDLE = 0,   /* 空闲（未启动/已退出） */
    CHS_STATE_RUN,        /* 运动中               */
    CHS_STATE_DONE,       /* 已到位               */
    CHS_STATE_ABORT       /* 已中止/超时          */
} ChassisState_t;

/* ===================== 函数声明 ===================== */

/**
 * @brief  车机运动模块初始化（只复位状态，不发电机命令）
 * @param  无
 * @retval 无
 * @note   上电初始化阶段禁止使能/转动车轮；使能只发生在 Chassis_Start。
 */
void Chassis_Init(void);

/**
 * @brief  启动一个动作（使能四轮 + 下发位置模式目标）
 * @param  act: 动作编号（ChassisAct_t 值 0 ~ CHS_ACT_COUNT-1），超范围忽略
 * @retval 无
 * @note   启动前必须能读到四轮位置（0x36 帧），否则拒绝启动并日志
 *         `no pos data` —— 位置模式的到位判定完全依赖位置数据。
 *         四轮使能只在本函数发生；中止/退出时由 Chassis_Abort 失能。
 */
void Chassis_Start(uint8_t act);

/**
 * @brief  中止当前动作（四轮急停 + 失能 + 状态复位）
 * @param  无
 * @retval 无
 * @note   正常到位由电机固件平顺停车，不需要本函数；本函数用于人工中止、
 *         退出页面或超时兜底，属紧急停车（0xFE）。
 */
void Chassis_Abort(void);

/**
 * @brief  车机运动状态机推进（主循环调用，非阻塞）
 * @param  无
 * @retval 无
 * @note   每拍读四轮位置 -> 算与目标脉冲的误差 -> 判到位 ->
 *         按阶段推进（主运动段 -> 纠偏段/ARC 第二段 -> DONE）。
 *         同时更新进度（已走 mm / 已转 deg）并刷新测试页。
 *         **漏调度会导致永远不判到位**（位置模式电机自己会走完，
 *         但软件状态机会一直停在 RUN 直到超时）。
 */
void Chassis_Update(void);

/**
 * @brief  查询是否正在运动
 * @param  无
 * @retval 1=运动中，0=非运动
 */
uint8_t Chassis_IsActive(void);

/**
 * @brief  查询是否已到位完成
 * @param  无
 * @retval 1=已到位（DONE），0=其它状态
 */
uint8_t Chassis_IsDone(void);

/**
 * @brief  取当前/上次动作编号
 * @param  无
 * @retval ChassisAct_t 值
 */
uint8_t Chassis_GetAct(void);

/**
 * @brief  取当前状态文字（页面显示用）
 * @param  无
 * @retval "IDLE" / "RUN" / "DONE" / "ABORT"
 */
const char *Chassis_GetStateText(void);

/**
 * @brief  取动作名称文字（页面显示用）
 * @param  act: 动作编号
 * @retval "FWD"/"BACK"/"LEFT"/"RIGHT"/"ROT"/"DIAG"/"ARC"/"SPIN"；非法返回 "???"
 */
const char *Chassis_GetActName(uint8_t act);

/**
 * @brief  取本次动作已走的平移距离（进度显示）
 * @param  无
 * @retval 已完成平移距离，单位 mm（水平位移矢量模长）
 */
int16_t Chassis_GetDistMm(void);

/**
 * @brief  取本次动作已转的角度（进度显示）
 * @param  无
 * @retval 已完成转角，单位 deg（正=逆时针）
 */
int16_t Chassis_GetAngleDeg(void);

/* ===================== 相对坐标（里程计） ===================== */
/* 以"进入车机测试页那一刻的位置"为原点(0,0)，坐标轴取当时的车身方向：
 *     +x = 当时车头朝向（前方）
 *     +y = 当时车身右侧（右方）
 * 朝向角 = 相对当时朝向的转角，单位 deg。
 *
 * 原理（每拍在主循环里累加）：
 *   1) 由四轮脉冲增量做麦轮正解，得到**本体坐标系**下的位移增量 (dx_body, dy_body)；
 *   2) 用当时实测的全局朝向 theta 把它旋转回原点坐标系后累加：
 *        x += dx_body·cos(theta) − dy_body·sin(theta)
 *        y += dx_body·sin(theta) + dy_body·cos(theta)
 *
 * **注意**：
 *   - theta 来自陀螺仪，所以**坐标正确性依赖 IMU**；IMU 离线时 theta 恒为 0，
 *     转弯后再直行会算错（报"一直往前"）；
 *   - 这是**轮式里程**，车轮打滑会累积误差，长时间/多次动作后需要重新进入页面清零；
 *   - 进入车机测试页会自动清零（原点重置），退出页面不保留。 */

/**
 * @brief  相对坐标清零：原点 = 当前位置，朝向基准 = 当前陀螺仪角度
 * @param  无
 * @retval 无
 * @note   进入车机测试页时自动调用；清零后坐标立即从 (0,0)/0° 开始累加。
 */
void Chassis_OdomReset(void);

/**
 * @brief  原地旋转期间"冻结 / 解冻"平移坐标累加
 * @param  on: 1=冻结（转多少度 x/y 都不动）；0=恢复正常累加
 * @retval 无
 * @note   旋转步进入时发 1、退出时发 0；**用完必须解冻**，否则会"车在走、坐标不动"。
 *         冻结期间只冻结 x/y —— 朝向 yaw 仍正常更新（它来自 IMU，不由编码器积分）。
 *         日志：`[CHS] odom holdXY ON (freeze x/y)` / `OFF`。
 */
void Chassis_OdomHoldXY(uint8_t on);

/**
 * @brief  ★任务点"静态置角"：IMU 置 refDeg **并同步里程计朝向基准**
 * @param  refDeg: 期望当前朝向（TRAY PICK=0 / ROUGH PLACE/STACK=90 / STORE BACK=180）
 * @retval 1=已置角；0=车还在转被 IMU 拒绝（本拍不改任何基准）
 * @note   【为什么必须成对做】`Chassis_GetOdomYaw()` = IMU yaw − 原点基准(`chsOdomYaw0`)。
 *         只改 IMU 会连带让里程计朝向**跳变**（现场现象：TRAY PICK 置 0 后
 *         页面航向角变成 180°），后续 GO/Rotate 的变换也跟着错。
 *         这里在置角成功后把 `chsOdomYaw0` 一起平移，使
 *         `Chassis_GetOdomYaw()` == refDeg（= 后续绝对坐标使用的新角度）。
 *         只改"朝向基准"，**不动原点坐标 x/y**（位置由轮式里程累积，与 yaw 基准无关）。
 */
uint8_t Chassis_StaticZeroYaw(float refDeg);

/**
 * @brief  开始一次"静止零漂"测量（车**已停稳**时调用）
 * @param  无
 * @retval 无
 * @note   与 Chassis_YawDriftEnd() 成对使用；跑图里在"转弯到位后"的静止窗口调用。
 *         窗口短于 CHS_YAW_DRIFT_CAL_MS 时不更新估计。
 */
void Chassis_YawDriftBegin(void);

/**
 * @brief  结束"静止零漂"测量并更新零偏估计
 * @param  无
 * @retval 无
 * @note   估计值经限幅（CHS_YAW_DRIFT_MAX_DPS）与一阶平滑（CHS_YAW_DRIFT_SMOOTH）
 *         后，由 Chassis_OdomUpdate 按时间线性扣除；启用开关 CHS_YAW_DRIFT_COMP_ENABLE。
 *         日志：`[CHS] yaw drift cal <窗口> ms: rate=… bias=… mdeg/s`。
 */
void Chassis_YawDriftEnd(void);

/**
 * @brief  取当前零偏估计（整数打印用，禁 %f）
 * @param  无
 * @retval 零偏估计 ×1000，单位 0.001°/s
 */
int32_t Chassis_GetYawDrift_mdps(void);

/**
 * @brief  相对坐标每拍累加（主循环周期调用，与运动状态无关）
 * @param  无
 * @retval 无
 * @note   非阻塞；任一轮位置读不到就整拍跳过（不污染累计）。
 *         手动推车也会计入（只要轮子转了）。
 */
void Chassis_OdomUpdate(void);

/**
 * @brief  取相对原点的 x 坐标
 * @param  无
 * @retval x，单位 mm，正=沿原点方向向前
 */
float Chassis_GetOdomX(void);

/**
 * @brief  取相对原点的 y 坐标
 * @param  无
 * @retval y，单位 mm，正=沿原点方向的右侧
 */
float Chassis_GetOdomY(void);

/**
 * @brief  取相对原点朝向的转角
 * @param  无
 * @retval 角度 deg，范围 ±180（正=车头朝原点右侧转）
 */
float Chassis_GetOdomYaw(void);

/* ===== Yaw compensation for the COORDINATE frame (English comments: GBK file) ====
 * The mission compensates IMU zero-drift by writing a rotated target angle, e.g.
 * Rotate_Comp(90, 8) drives the IMU-frame heading to 98 so the body physically
 * faces 90. But every COORDINATE computation (odometry position + the
 * global<->body transform used by GO) must then use the TRUE heading
 * (IMU heading - comp), otherwise the reached absolute point is rotated by the
 * drift amount (the exact symptom the user reported).
 * chsYawCompDeg = the compensation currently in force (set by Chassis_SetYawCompDeg,
 * which bsp_task's Rotate_Comp calls). 0 = no compensation = previous behaviour.
 * Chassis_GetOdomX/Y return the position in the TRUE frame (raw accumulation is
 * internally rotated by -comp), and Chassis_GetOdomYawTrue() gives the TRUE yaw.
 * Chassis_GetOdomYaw() itself stays in the IMU frame (rotation control uses it). */
void  Chassis_SetYawCompDeg(float deg);   /* set current compensation (deg); 0 = off */
float Chassis_GetYawCompDeg(void);        /* get current compensation (deg) */
float Chassis_GetOdomYawTrue(void);       /* TRUE heading = Chassis_GetOdomYaw() - comp */

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CHASSIS_H */
