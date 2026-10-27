/**
 ******************************************************************************
 * @file    bsp_mecanum.h
 * @brief   麦轮底盘运动学（P4：整车位移 -> 四轮目标脉冲，位置模式）
 * @author  nx_car
 * @date    2026-09-16
 ******************************************************************************
 * @note   本模块只做"整车位移 -> 四轮脉冲/速度"的数学换算与下发，不含运动
 *         状态机（见 bsp_chassis）。
 *         运动全部走**电机位置模式**（Motor_PosMove -> 0xFD）：
 *         一次下发目标脉冲，加减速轨迹由电机固件内部规划，到位后电机自停，
 *         停车平滑无 0xFE 急停冲击。
 *
 *         为什么位置模式能解决"停位误差大"：
 *           电机固件内部用编码器闭环数脉冲，位置精度不受 CAN 往返延迟影响；
 *           而速度模式靠上层每 80ms 轮询里程判断到位，末端必然过冲几 mm~几十 mm。
 *
 *         坐标系与单位约定（整车右手坐标系）：
 *           x 轴：车体正前方，向前为正，单位 mm
 *           y 轴：车体正右方，向右为正，单位 mm
 *           dRot: equivalent arc length of the rotation, mm
 *                 (dRot = angle_rad * MEC_ROT_RADIUS_MM).
 *                 Positive dRot = **CLOCKWISE** on this car (left wheels move
 *                 forward, right wheels backward). Callers whose angle is
 *                 CCW-positive (route table / IMU yaw / mission points) must
 *                 negate it -- see Route_SendRotDeg and bsp_mission.
 *
 *         麦轮逆运动学（位移版，X 形安装）：
 *           dFL = +dx + dy + dRot
 *           dFR = +dx - dy - dRot
 *           dRL = +dx - dy + dRot
 *           dRR = +dx + dy - dRot
 *
 *         硬约束：全部函数非阻塞，禁止 HAL_Delay；日志禁 %f。
 ******************************************************************************
 */
#ifndef __BSP_MECANUM_H
#define __BSP_MECANUM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "bsp_motor.h"   /* 提供 MOTOR_NUM / MOTOR_ADDR_xx / MOTOR_PULSE_PER_REV /
                            MOTOR_RPM_MAX / Motor_PosMove / Motor_Stop */

/* ===================== 几何参数宏（装机实测后修改） ===================== */

/* 车轮（麦轮滚轮有效）外径，单位 mm，默认 100.0f，可调范围 40~200；
   调法：卷尺量滚轮外径（含滚轮），或跑已知距离反推（见《调参手册》P4 章） */
#define WHEEL_DIAMETER_MM       100.0f

/* 轴距：前后轮轴中心距，单位 mm，默认 170.0f，可调范围 100~400；
   调法：装车后直接量前轮轴心到后轮轴心的距离 */
#define WHEELBASE_MM            170.0f

/* 轮距：左右轮中心距，单位 mm，默认 253.0f，可调范围 100~500；
   调法：装车后直接量左轮轴心到右轮轴心的距离 */
#define TRACKWIDTH_MM           245.0f

/* 减速比 = 电机转数 / 车轮转数，可调范围 1~20；
   调法：手转车轮整一圈，数电机转过的圈数（或查电机铭牌/驱动器减速比）。
   **本项目实测为 1.0**（电机与车轮 1:1 直连），结论已登记《调参手册》P4 章 3.8。
   注意：本值同时决定"位移→脉冲"换算，改动后必须重跑 500mm 复核落点。 */
#define MOTOR_TO_WHEEL_RATIO    1.0f

/* ===================== 速度与加减速档 ===================== */

/* 整车默认运动速度，单位**车轮 RPM**，可调范围 30~470；
   内部换算成电机轴 RPM = 本值 × 减速比，换算关系见 MEC_MOTOR_RPM_PER_WHEEL_RPM。
   调法：太快导致过冲/打滑就调小，太慢调大。
   限幅：判据是**电机轴 RPM 不得超过 MOTOR_RPM_MAX(500)**，
   即本值上限 = 500 / MOTOR_TO_WHEEL_RATIO。本项目减速比 1.0，
   故上限 500 车轮 RPM。
   【2026-09-23 由 300 降到 200】现场反馈"直行到点位的**减速段**出现微小横移"：
   轮径 100mm 时 300 车轮 RPM ≈ **1.57 m/s**，对麦轮偏快，减速段滚子打滑与
   "四轮不同时到位"都更明显；降到 200（≈1.05 m/s）可明显缓解。
   想更准可继续降到 150；想更快回到 250~300。 */
#define MEC_DEFAULT_VEL         400U

/* Velocity-mode wheel RPM cap (route velocity mode). Any inverse-solved wheel
   speed above this is scaled down proportionally so the ratio (straight line)
   is preserved. Range 100~500; 0 uses MOTOR_RPM_MAX. */
#define MEC_VEL_RPM_MAX         500U

/* 原地旋转专用速度，单位**车轮 RPM**，默认 120，可调范围 30~300。
   为什么单独降速：电机的**原地旋转**是靠四个麦轮横向刮擦实现的，转速越高打滑越严重，
   而打滑会直接变成"转角不准 + 未测量的位移丢失"（轮子量不到，里程计就看不见），
   是长路径累计误差的主要来源之一。降到一半速度可明显减少打滑（代价是转弯变慢）。
   本值同时作用于跑图的"原地转"与 P4 车机页的 ROT/SPIN；改宏后 Rebuild 即可。
   【2026-09-23 由 150 降到 120】现场要求"转弯速度慢一些（不要太慢）、保证转角精度"：
   120 比 150 打滑更少、转角更准，又不至于明显拖慢跑圈。
   **跑图侧还可通过在线调参 `rotVel` 现场微调**（默认就是本值）。 */
#define MEC_ROT_VEL             300U

/* 位置模式加减速度档（0~255），可调范围 0~255。
   含义：数值越大，电机固件规划的加减速**越缓**（0 = 无曲线、直接启动）。
   调法：起步顿挫/打滑调大，加速太肉调小。
   【2026-09-23 由 150 提到 200】现场要求"跑图加速度小一点、优先保证位置模式到位精度"：
   加减速更缓 → 减速段的惯性冲击与滚子打滑更小，且"等时长法"分配的四轮速度
   更接近理想比例 → 到位更准（代价：每个动作多花一点时间）。
   仍嫌顿挫可继续 220~255；想更快回到 150 及以下。
   注意：本值现在**只管平移**（直行 / 横移 / 边走边转），**原地旋转改用下面的
   `MEC_ROT_ACC`** —— 两者在 `Mecanum_MoveByPulse` 里由调用方传入的 acc 参数区分。
   P4 车机页的位置模式动作也走平移路径，会一并变缓。
   **改宏后需 Rebuild**（不支持在线调）。 */
#define MEC_ACC                 120U

/* 原地旋转加减速度档（0~255），可调范围 0~255。
   含义与 MEC_ACC 相同（**数值越大越缓**），但**只作用于原地旋转**。
   为什么要和 MEC_ACC 拆开：
     原地旋转靠四个麦轮**横向刮擦**，与直行的滚动摩擦完全不同 ——
     旋转时轮子是被"拖着走"的，加减速越猛，起步那一下的瞬间冲击越大，
     滚子越容易打滑，而打滑直接变成"转角不准 + 里程计看不见的位移丢失"；
     直行要解决的却是"末端减速更柔"（MEC_ACC 已偏大到 200）。
     两者最优值通常不同（旋转一般要**更缓**），所以拆成独立宏单独调。
   调法：
     · 旋转起步/收尾"咯噔"、转完总是多转一点 -> 调大（220~255）更缓；
     · 旋转太肉、每个点位等太久           -> 调小（150~180）。
   **默认 200 = 拆分前 MEC_ACC 的值**，所以不动的就是老行为，现场按现象单独试。
   改宏后需 Rebuild（不支持在线调）。 */
#define MEC_ROT_ACC             120U

/* 单轮最低下发速度，单位**电机轴 RPM**，默认 20，可调范围 5~60。
   只在"该轮需要运动（脉冲数不为 0）但按位移比例算出的速度过低"时生效，
   防止位置模式收到 0 速度导致不动或不响应。
   注意：这是保底值，会牺牲一点"同时到位"的精度（位移极小的轮会提前走完），
   但绝对偏差很小（毫秒级），可接受。 */
#define MEC_VEL_MIN             20U

/* ===================== 三个轴符号宏（整车方向标定唯一入口） ===================== */
/* 用途：整车层面运动方向与期望相反时，只改这三个宏（+1 / -1），
 *       禁止改单轮系数、禁止改麦轮公式、禁止交换轮序。
 * 判据：整车三轴中哪一轴方向反了，就把该轴宏改成 -1。
 *   - 前进变后退        -> 改 MEC_SIGN_VX
 *   - 右移变左移        -> 改 MEC_SIGN_VY
 *   - 逆时针变顺时针    -> 改 MEC_SIGN_WZ
 * 单轮方向反（某个轮单独转反）请改 bsp_motor.h 的 MOTOR_xx_DIR_FWD/REV。
 * 本三宏同时作用于逆解（下发）与正解（进度反馈），保证两向一致。 */
#define MEC_SIGN_VX             (+1)    /* 前进轴符号：+1=前进为正，-1=取反 */
#define MEC_SIGN_VY             (+1)    /* 横移轴符号：+1=右移为正，-1=取反 */
#define MEC_SIGN_WZ             (+1)    /* 旋转轴符号：+1=逆时针为正，-1=取反 */

/* ===================== 位置刻度标定（×10 / 0.1° 疑点） ===================== */
/* 最终下发的脉冲数 = 理论脉冲数 × MEC_PULSE_SCALE_NUM / MEC_PULSE_SCALE_DEN。
 *
 * 为什么需要这个宏（**P4 第一件要标定的事**）：
 *   官方 0x36 位置帧 / 0xFD 位置命令的数值单位有两种常见可能：
 *     - 电机轴脉冲：每转 = MOTOR_PULSE_PER_REV = 3200（16 细分 × 200 步）
 *     - 0.1°     ：每转 = 3600
 *   两者相差 3600/3200 = 1.125，会让**走距直接差 12.5%**（500mm 差 62mm），
 *   而且这是"刻度错"，靠调加速度/容差都修不掉。
 *
 * 标定方法（见《调参手册》P4 章步骤 a）：
 *   1) 保持 1/1，地面贴标记跑一次 CHS 页 FWD（500mm），用卷尺量实际距离；
 *   2) 实际 ≈500mm              -> 保持 1/1（单位是电机轴脉冲）；
 *   3) 实际 ≈444mm（约 89%）    -> 单位是 0.1°，改成 3600 / 3200；
 *   4) 其他比例按 实测/设定 的倒数调整分子分母。
 * 改这一处即对"下发目标脉冲"与"进度/到位判定换算"同时生效。 */
#define MEC_PULSE_SCALE_NUM     1
#define MEC_PULSE_SCALE_DEN     1

/* ===================== 派生换算宏（公式写在注释里，不要手算） ===================== */

/* 圆周率 */
#define MEC_PI                  3.14159265f

/* 角度 <-> 弧度换算系数（度 × 本值 = 弧度） */
#define MEC_RAD_PER_DEG         (MEC_PI / 180.0f)

/* 原地旋转等效半径 R，单位 mm（旋转时四轮合成力臂，取轴距与轮距之和的一半） */
#define MEC_ROT_RADIUS_MM       ((WHEELBASE_MM + TRACKWIDTH_MM) * 0.5f)

/* 车轮每转对应的**电机轴**脉冲数 = 电机每转脉冲数 × 减速比 */
#define MEC_PULSE_PER_WHEEL_REV ((float)MOTOR_PULSE_PER_REV * MOTOR_TO_WHEEL_RATIO)

/* 车轮周长，单位 mm */
#define MEC_WHEEL_CIRC_MM       (MEC_PI * WHEEL_DIAMETER_MM)

/* 位移 mm -> 电机轴脉冲：脉冲 = mm × MEC_PULSE_PER_WHEEL_REV / 周长
   （等价于任务书写的 mm × MOTOR_PULSE_PER_REV × 减速比 / (π × 轮径)） */
#define MEC_PULSE_PER_MM        (MEC_PULSE_PER_WHEEL_REV / MEC_WHEEL_CIRC_MM)

/* 电机轴脉冲 -> 位移 mm（正解用，MEC_PULSE_PER_MM 的倒数，不引入额外除法误差） */
#define MEC_MM_PER_PULSE        (MEC_WHEEL_CIRC_MM / MEC_PULSE_PER_WHEEL_REV)

/* 原地旋转"角度 <-> 弧长"的**指令刻度修正**，默认 1/1（不修正）：
 *     弧长 = 角度 × π/180 × MEC_ROT_RADIUS_MM × NUM / DEN
 * 用途：实测发现"命令转 90°，车实际转了 180°"（转多了）时，说明下发的弧长偏大，
 *       把本值改成 1/2（NUM=1、DEN=2）即可；反过来"转少了"就用大于 1 的比例
 *       （例：命令 90° 只转了 76° -> ×100/76 ≈ 1.32，取 NUM=132/DEN=100）。
 * 标定方法：跑图页跑一次"只旋转"的路段，看日志
 *      `[ROUTE] seg0 turn done, yaw d=… (cmd=…)` + `turn angle off by …`
 *   按"实测/命令"的比例设置本值；结论写进《调参手册》P7 章。
 * 注意：本值只影响**下发的旋转弧长**（及 P4 的进度换算），不影响里程计朝向
 *       （朝向来自 IMU，是实测值）。 */
#define MEC_ROT_CMD_GAIN_NUM    1
#define MEC_ROT_CMD_GAIN_DEN    1

/* 车轮 RPM -> 电机轴 RPM 的系数（= 减速比） */
#define MEC_MOTOR_RPM_PER_WHEEL_RPM   (MOTOR_TO_WHEEL_RATIO)

/* ===================== 函数声明 ===================== */

/**
 * @brief  麦轮运动学模块初始化（只打印几何参数，不发电机命令）
 * @param  无
 * @retval 无
 * @note   上电初始化阶段禁止使能/转动车轮，故本函数不下发任何 CAN 命令。
 */
void Mecanum_Init(void);

/**
 * @brief  整车位移 -> 四轮目标脉冲并下发（位置模式）
 * @param  dx_mm:  前进方向位移，单位 mm（正=前进，负=后退）
 * @param  dy_mm:  右移方向位移，单位 mm（正=右移，负=左移）
 * @param  dRot_mm: 旋转等效弧长，单位 mm（正=逆时针，负=顺时针）；
 *                  角度转弧长用 Mecanum_DegToArcMm()
 * @param  out_clk: 输出四轮本次下发的**有符号脉冲数**，下标 0~3 依次对应
 *                  FL/FR/RL/RR。供上层记录目标脉冲做到位判定。
 *                  **必须传入有效的 4 元素数组**（不允许 NULL）。
 * @retval 无
 * @note   流程：
 *           1) 按 MEC_SIGN_VX/VY/WZ 做整车三轴方向标定；
 *           2) 麦轮逆解（位移版）算四轮位移；
 *           3) 位移 mm -> 电机轴脉冲（含 MEC_PULSE_SCALE 刻度标定）；
 *           4) **等时长法**分配各轮速度：v_i = v_base × |d_i| / d_max，
 *              使四轮用时相同 -> 边走边转的中途也保持正确比例、同时到位；
 *           5) 逐轮 Motor_PosMove 下发。
 *         位移为 0 的轮（如 45° 斜行的 FR/RL）**不下发命令**，自然不动。
 */
void Mecanum_MoveBy(int32_t dx_mm, int32_t dy_mm, int32_t dRot_mm,
                    int32_t out_clk[MOTOR_NUM]);

/**
 * @brief  原地旋转便捷封装（= Mecanum_MoveBy(0, 0, dRot_mm, out_clk)）
 * @param  dRot_mm: 旋转等效弧长，单位 mm（正=逆时针），用 Mecanum_DegToArcMm() 换算
 * @param  out_clk: 输出四轮脉冲数，必须非 NULL
 * @retval 无
 * @note   与 Mecanum_MoveBy 的唯一差别：速度用 `MEC_ROT_VEL`、
 *         加减速档用 `MEC_ROT_ACC`（都比平移更保守，减少麦轮横向刮擦打滑）。
 */
void Mecanum_MoveByRot(int32_t dRot_mm, int32_t out_clk[MOTOR_NUM]);

/**
 * @brief  按已算好的四轮脉冲下发电机位置模式（速度按等时长法自动分配）
 * @param  clk: 四轮目标脉冲（有符号），下标 0~3 依次对应 FL/FR/RL/RR；
 *              为 0 的轮不下发（该轮保持不动）
 * @param  velWheelRpm: 基准速度，单位**车轮 RPM**（内部 × 减速比 得电机轴 RPM）；
 *              传 0 表示用默认 MEC_DEFAULT_VEL
 * @param  acc: 加减速度档（0~255，**数值越大越缓**），直接透传给 Motor_PosMove；
 *              **平移传 `MEC_ACC`、原地旋转传 `MEC_ROT_ACC`** —— 两个用途的
 *              最优值不同（旋转靠横向刮擦，通常要更缓）。
 * @retval 无
 * @note   与 Mecanum_MoveBy 的分工：位移→脉冲的换算由调用方完成，本函数只负责
 *         "等时长法分配各轮速度 + 逐轮 Motor_PosMove 下发"。
 *         用于需要上层自己算脉冲的场合（例如"直行 + 自转"的分段逼近，
 *         它的车轮位移是角度的非线性函数，不能走一次性的位移逆解）；
 *         也用于"旋转刻度要在线可调"的场合（bsp_route / bsp_mission 自己算脉冲后
 *         调本函数，acc 仍取编译期的 `MEC_ROT_ACC`）。
 *         速度分配：v_i = 基准速度 × |clk_i| / max|clk|，使四轮用时相同。
 *         **分段逼近类动作建议传较小的基准速度**：分段短、如果还跑得快，
 *         电机固件的加减速段会占满整段，四轮速度比例来不及保持，
 *         麦轮会互相刮擦、车身走不动（这是"只动一小下"的常见原因）。
 */
void Mecanum_MoveByPulse(const int32_t clk[MOTOR_NUM], uint16_t velWheelRpm, uint8_t acc);

/**
 * @brief  四轮立即停止（0xFE 急停）
 * @param  无
 * @retval 无
 * @note   用于中止/退出/超时兜底。正常到位由电机固件自行平顺停车，
 *         不需要调本函数。
 */
void Mecanum_Stop(void);

/**
 * @brief  Body-frame velocity command (translation + rotation), continuous
 * @param  rpmFwd:   forward component, wheel RPM (positive = forward)
 * @param  rpmRight: right component, wheel RPM (positive = right)
 * @param  rpmRot:   rotation component, wheel RPM. NOTE: with the mixing used
 *                   here (same as Mecanum_MoveBy), **positive = CLOCKWISE**
 *                   (left wheels forward, right wheels backward). Callers that
 *                   work with a CCW-positive yaw rate must negate it.
 *                   (Measured on the car; see Route_SendRotDeg, which negates.)
 * @param  acc:      accel level 0~255 (same semantic as Motor_SetVel)
 * @retval None
 * @note   Same X-shape inverse kinematics and MEC_SIGN_VX/VY/WZ as Mecanum_MoveBy.
 *         Speeds are scaled down if any wheel exceeds MEC_VEL_RPM_MAX.
 *         rpm=0 on all three axes -> four speed-0 frames (soft stop, not 0xFE).
 *         Silent: safe for a 20ms control loop.
 */
void Mecanum_VelBodyRpm(float rpmFwd, float rpmRight, float rpmRot, uint8_t acc);

/**
 * @brief  麦轮正解：四轮脉冲增量 -> 整车位移增量（进度显示用）
 * @param  dClk: 四轮脉冲增量，下标 0~3 依次对应 FL/FR/RL/RR（有符号）
 * @param  out_dx_mm:   输出前进方向位移增量 mm，可传 NULL
 * @param  out_dy_mm:   输出右移方向位移增量 mm，可传 NULL
 * @param  out_dRot_mm: 输出旋转等效弧长增量 mm，可传 NULL
 * @retval 无
 * @note   按 MEC_MM_PER_PULSE 换算后套正解公式，再按 MEC_SIGN_* 做整车标定，
 *         与 Mecanum_MoveBy 严格互逆。
 *         用途：测试页的"已走 mm / 已转 deg"进度显示。
 */
void Mecanum_ForwardPulse(const int32_t dClk[MOTOR_NUM],
                          float *out_dx_mm, float *out_dy_mm, float *out_dRot_mm);

/**
 * @brief  旋转角度 -> 等效弧长
 * @param  deg: 旋转角度，单位 deg（正=逆时针）
 * @retval 等效弧长，单位 mm（= deg × π/180 × MEC_ROT_RADIUS_MM）
 */
float Mecanum_DegToArcMm(float deg);

/**
 * @brief  等效弧长 -> 旋转角度
 * @param  arc_mm: 等效弧长，单位 mm
 * @retval 旋转角度，单位 deg
 */
float Mecanum_ArcMmToDeg(float arc_mm);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MECANUM_H */
