/**
 ******************************************************************************
 * @file    bsp_mecanum.c
 * @brief   麦轮底盘运动学实现（P4：整车位移 -> 四轮目标脉冲，位置模式）
 * @author  nx_car
 * @date    2026-09-16
 ******************************************************************************
 * @note   纯数学换算 + 逐轮位置模式下发，非阻塞，无 HAL_Delay。
 *         下发统一走 bsp_motor 的 Motor_PosMove（官方 0xFD 位置模式），
 *         不使用速度模式。
 *         日志统一 [MEC] 标签，禁 %f。
 ******************************************************************************
 */
#include "bsp_mecanum.h"
#include "bsp_debug.h"

/* ===================== 公开接口 ===================== */

/**
 * @brief  麦轮运动学模块初始化（只打印几何参数，不发电机命令）
 * @param  无
 * @retval 无
 * @note   上电初始化阶段禁止使能/转动车轮，故此处只输出参数便于核对标定值。
 */
void Mecanum_Init(void)
{
    Debug_Log("MEC", "init d=%d wb=%d tw=%d ratio=%d vel=%u acc=%u",
              (int)WHEEL_DIAMETER_MM, (int)WHEELBASE_MM,
              (int)TRACKWIDTH_MM, (int)(MOTOR_TO_WHEEL_RATIO * 100.0f),
              (unsigned int)MEC_DEFAULT_VEL, (unsigned int)MEC_ACC);
    Debug_Log("MEC", "pulse/mm=%d scale=%d/%d R=%d mm",
              (int)(MEC_PULSE_PER_MM * 100.0f),
              (int)MEC_PULSE_SCALE_NUM, (int)MEC_PULSE_SCALE_DEN,
              (int)MEC_ROT_RADIUS_MM);
}

/**
 * @brief  整车位移 -> 四轮目标脉冲并下发（位置模式）
 * @param  dx_mm:  前进方向位移，单位 mm
 * @param  dy_mm:  右移方向位移，单位 mm
 * @param  dRot_mm: 旋转等效弧长，单位 mm
 * @param  out_clk: 输出四轮有符号脉冲数（FL/FR/RL/RR），必须非 NULL
 * @retval 无
 * @note   速度分配用**等时长法**：v_i = v_base × |d_i| / |d|_max。
 *         因为位置模式下四轮各自独立规划，位移不同则用时不同；按位移比例
 *         缩放速度后四轮用时相同，边走边转的中途才能保持正确比例并同时到位。
 *         位移为 0 的轮不下发（如 45° 斜行的 FR/RL）。
 */
void Mecanum_MoveBy(int32_t dx_mm, int32_t dy_mm, int32_t dRot_mm,
                    int32_t out_clk[MOTOR_NUM])
{
    float   dx;
    float   dy;
    float   dr;
    float   dWheel[MOTOR_NUM];
    float   maxAbs = 0.0f;
    float   scale;
    uint8_t i;

    /* 1) 整车三轴方向标定（整车层面方向反只改这三个宏） */
    dx = (float)MEC_SIGN_VX * (float)dx_mm;
    dy = (float)MEC_SIGN_VY * (float)dy_mm;
    dr = (float)MEC_SIGN_WZ * (float)dRot_mm;

    /* 2) 麦轮逆解（位移版，X 形安装） */
    dWheel[0] = dx + dy + dr;   /* FL 前左 */
    dWheel[1] = dx - dy - dr;   /* FR 前右 */
    dWheel[2] = dx - dy + dr;   /* RL 后左 */
    dWheel[3] = dx + dy - dr;   /* RR 后右 */

    /* 3) 位移 mm -> 电机轴脉冲（含刻度标定），四舍五入 */
    scale = MEC_PULSE_PER_MM * (float)MEC_PULSE_SCALE_NUM / (float)MEC_PULSE_SCALE_DEN;

    for (i = 0; i < MOTOR_NUM; i++)
    {
        float p = dWheel[i] * scale;
        out_clk[i] = (int32_t)((p >= 0.0f) ? (p + 0.5f) : (p - 0.5f));
    }

    /* 4) 求最大脉冲绝对值（仅用于判断"是否全为 0"的日志分支） */
    for (i = 0; i < MOTOR_NUM; i++)
    {
        float a = (out_clk[i] < 0) ? -(float)out_clk[i] : (float)out_clk[i];
        if (a > maxAbs)
        {
            maxAbs = a;
        }
    }
    if (maxAbs < 0.5f)
    {
        Debug_Log("MEC", "moveby zero, ignored");
        return;
    }

    /* 5) hand over to the common "equal-duration + per-wheel issue" path.
       Base speed = default MEC_DEFAULT_VEL; landing-point fine tuning uses its own
       dedicated low speed via Mecanum_MoveByPulse (see bsp_mission / bsp_align). */
    Mecanum_MoveByPulse(out_clk, 0U, (uint8_t)MEC_ACC);

    Debug_Log("MEC", "moveby dx=%d dy=%d rot=%d clk=%d,%d,%d,%d",
              (int)dx_mm, (int)dy_mm, (int)dRot_mm,
              (int)out_clk[0], (int)out_clk[1], (int)out_clk[2], (int)out_clk[3]);
}

/**
 * @brief  按已算好的四轮脉冲下发电机位置模式（等时长法分配速度）
 * @param  clk: 四轮目标脉冲（FL/FR/RL/RR），0 表示该轮不动
 * @param  velWheelRpm: 基准速度，单位车轮 RPM；0 = 用 MEC_DEFAULT_VEL
 * @param  acc: 加减速度档（0~255，**数值越大越缓**），透传给 Motor_PosMove。
 *              **平移传 MEC_ACC、原地旋转传 MEC_ROT_ACC**（旋转靠横向刮擦，
 *              与直行的滚动摩擦机理不同，最优加减速通常不同，故拆成两个宏）。
 * @retval 无
 * @note   等时长法：v_i = v_base × |clk_i| / max|clk|。
 *         因为位置模式下四轮各自独立规划，脉冲数不同则用时不同；按脉冲比例
 *         缩放速度后四轮用时相同，多轴合成的运动中途才能保持正确比例。
 *         基准速度由 MEC_DEFAULT_VEL（车轮 RPM）× 减速比 得到电机轴 RPM，
 *         超 MOTOR_RPM_MAX 时对**基准值**统一钳位（四轮同比缩小，不破坏比例）。
 *         四轮命令**背靠背连发、中间不插日志**（见下方注释），整段下发跨度 ≈2~3ms；
 *         日志挪到全部发完之后打一行汇总。
 */
void Mecanum_MoveByPulse(const int32_t clk[MOTOR_NUM], uint16_t velWheelRpm, uint8_t acc)
{
    float    rpmBase;
    float    maxAbs = 0.0f;
    uint16_t velArr[MOTOR_NUM];
    uint32_t t0;
    uint8_t  i;

    /* 求最大脉冲绝对值作为等时长法的比例基准 */
    for (i = 0; i < MOTOR_NUM; i++)
    {
        float a = (clk[i] < 0) ? -(float)clk[i] : (float)clk[i];
        if (a > maxAbs)
        {
            maxAbs = a;
        }
    }

    /* 四轮都不动：不发命令 */
    if (maxAbs < 0.5f)
    {
        return;
    }

    /* 基准速度：入参是车轮 RPM（0 表示用默认值），换成电机轴 RPM 后统一限幅 */
    if (velWheelRpm == 0U)
    {
        velWheelRpm = (uint16_t)MEC_DEFAULT_VEL;
    }
    rpmBase = (float)velWheelRpm * MEC_MOTOR_RPM_PER_WHEEL_RPM;
    if (rpmBase > (float)MOTOR_RPM_MAX)
    {
        Debug_Log("MEC", "base rpm %d > max, clamped to %u",
                  (int)rpmBase, (unsigned int)MOTOR_RPM_MAX);
        rpmBase = (float)MOTOR_RPM_MAX;
    }

    /* ---- 四轮命令**背靠背连发**（中间不插任何日志）----
       Debug_Log 走 UART5 **逐字符阻塞**发送（115200 → 约 87µs/字符，一行 ~4ms）：
       以前每轮命令后面跟一行日志，把第 4 条命令拖到第 1 条之后 **~13ms** 才发出
       —— 先启动的先到先停、后停的推着整车走，正是"四轮不同时停 / 越过点位"的来源。
       现在四条连发：4 条 × 2 帧 × 0.3ms ≈ **2.4ms**（单条命令的邮箱等待上限见
       bsp_motor.h 的 MOTOR_TX_PAIR_WAIT_MS），日志挪到全部发完之后。
       ⚠ 正因为不再有日志"垫时间"，位置命令下发前必须等 **2 个**空邮箱
       （0xFD 会被官方库拆成 2 帧，邮箱不够会静默丢后一包 → 只有个别轮子响应）。 */
    t0 = HAL_GetTick();
    for (i = 0; i < MOTOR_NUM; i++)
    {
        float    a   = (clk[i] < 0) ? -(float)clk[i] : (float)clk[i];
        uint16_t vel;

        /* 位移为 0 的轮不下发，自然保持不动 */
        if (clk[i] == 0)
        {
            velArr[i] = 0U;
            continue;
        }

        vel = (uint16_t)(rpmBase * (a / maxAbs));
        if (vel < (uint16_t)MEC_VEL_MIN)
        {
            vel = (uint16_t)MEC_VEL_MIN;
        }
        velArr[i] = vel;

        Motor_PosMove((uint8_t)(MOTOR_ADDR_MIN + i), clk[i], vel, acc);
    }

    /* 四条命令全部发完后再打**一行汇总**：四轮 clk/vel 一眼可比（用来核对
       "速度/加速度是否统一"），dt = 四条命令的总下发耗时（ms），正常应 ≈2~3ms
       （日志夹在中间时 ≈13ms）。 */
    Debug_Log("MEC", "pos dt=%u clk=%d,%d,%d,%d v=%u,%u,%u,%u acc=%u",
              (unsigned int)(HAL_GetTick() - t0),
              (int)clk[0], (int)clk[1], (int)clk[2], (int)clk[3],
              (unsigned int)velArr[0], (unsigned int)velArr[1],
              (unsigned int)velArr[2], (unsigned int)velArr[3],
              (unsigned int)acc);
}

/**
 * @brief  原地旋转便捷封装
 * @param  dRot_mm: 旋转等效弧长，单位 mm（正=逆时针）
 * @param  out_clk: 输出四轮脉冲数，必须非 NULL
 * @retval 无
 */
void Mecanum_MoveByRot(int32_t dRot_mm, int32_t out_clk[MOTOR_NUM])
{
    float   dr;
    float   scale;
    float   p;
    uint8_t i;

    /* 原地旋转单独用**更低的速度** MEC_ROT_VEL + **独立的加减速档** MEC_ROT_ACC 下发：
       麦轮原地旋转靠四轮横向刮擦，转速/加速度越高打滑越严重，而打滑会直接变成
       "转角不准 + 未测量的位移丢失"，是长路径累计误差的主要来源之一。
       逆解与 Mecanum_MoveBy 完全一致（dx=dy=0），只是速度与加减速档不同：
           FL = +dr、FR = −dr、RL = +dr、RR = −dr  */
    dr    = (float)MEC_SIGN_WZ * (float)dRot_mm;
    scale = MEC_PULSE_PER_MM * (float)MEC_PULSE_SCALE_NUM / (float)MEC_PULSE_SCALE_DEN;

    for (i = 0U; i < (uint8_t)MOTOR_NUM; i++)
    {
        float sign = ((i == 0U) || (i == 2U)) ? 1.0f : -1.0f;   /* FL/RL 同向，FR/RR 反向 */

        p = sign * dr * scale;
        out_clk[i] = (int32_t)((p >= 0.0f) ? (p + 0.5f) : (p - 0.5f));
    }

    Mecanum_MoveByPulse(out_clk, (uint16_t)MEC_ROT_VEL, (uint8_t)MEC_ROT_ACC);

    Debug_Log("MEC", "rot arc=%d vel=%u acc=%u clk=%d,%d,%d,%d",
              (int)dRot_mm, (unsigned int)MEC_ROT_VEL, (unsigned int)MEC_ROT_ACC,
              (int)out_clk[0], (int)out_clk[1], (int)out_clk[2], (int)out_clk[3]);
}

/**
 * @brief  四轮立即停止（0xFE 急停）
 * @param  无
 * @retval 无
 * @note   用于中止/退出/超时兜底；正常到位由电机固件自行平顺停车。
 */
void Mecanum_Stop(void)
{
    uint8_t i;

    for (i = 0; i < MOTOR_NUM; i++)
    {
        Motor_Stop((uint8_t)(MOTOR_ADDR_MIN + i));
    }
}

/**
 * @brief  Body-frame velocity command (translation + rotation), continuous
 * @param  rpmFwd:   forward component, wheel RPM
 * @param  rpmRight: right component, wheel RPM
 * @param  rpmRot:   rotation component, wheel RPM (positive = CCW)
 * @param  acc:      accel level 0~255
 * @retval None
 * @note   Same inverse kinematics as Mecanum_MoveBy:
 *           FL = F + R + W, FR = F - R - W, RL = F - R + W, RR = F + R - W
 *         All four are sent back-to-back and silently (no UART log), so this is
 *         safe inside a 20ms control loop. If any wheel exceeds MEC_VEL_RPM_MAX
 *         all four are scaled by the same factor (keeps the straight-line ratio).
 */
void Mecanum_VelBodyRpm(float rpmFwd, float rpmRight, float rpmRot, uint8_t acc)
{
    static const uint8_t addr[MOTOR_NUM] = {
        (uint8_t)MOTOR_ADDR_FL, (uint8_t)MOTOR_ADDR_FR,
        (uint8_t)MOTOR_ADDR_RL, (uint8_t)MOTOR_ADDR_RR
    };
    float   axF = (float)MEC_SIGN_VX * rpmFwd;
    float   axR = (float)MEC_SIGN_VY * rpmRight;
    float   axW = (float)MEC_SIGN_WZ * rpmRot;
    float   c[MOTOR_NUM];
    float   mx = 0.0f;
    float   k  = 1.0f;
    uint8_t i;

    c[0] = axF + axR + axW;     /* FL */
    c[1] = axF - axR - axW;     /* FR */
    c[2] = axF - axR + axW;     /* RL */
    c[3] = axF + axR - axW;     /* RR */

    for (i = 0U; i < (uint8_t)MOTOR_NUM; i++)
    {
        float a = (c[i] < 0.0f) ? -c[i] : c[i];
        if (a > mx)
        {
            mx = a;
        }
    }

    if (mx > (float)MEC_VEL_RPM_MAX)
    {
        k = (float)MEC_VEL_RPM_MAX / mx;
    }

    for (i = 0U; i < (uint8_t)MOTOR_NUM; i++)
    {
        float   v = c[i] * k;
        int16_t r = (v >= 0.0f) ? (int16_t)(v + 0.5f) : (int16_t)(v - 0.5f);

        Motor_SetVelQuiet(addr[i], r, acc);
    }
}

/**
 * @brief  麦轮正解：四轮脉冲增量 -> 整车位移增量（进度显示用）
 * @param  dClk: 四轮脉冲增量（FL/FR/RL/RR）
 * @param  out_dx_mm:   输出前进位移增量 mm，可传 NULL
 * @param  out_dy_mm:   输出右移位移增量 mm，可传 NULL
 * @param  out_dRot_mm: 输出旋转等效弧长增量 mm，可传 NULL
 * @retval 无
 * @note   与 Mecanum_MoveBy 严格互逆：
 *         脉冲 -> mm 时要用刻度标定的**反变换**（× DEN/NUM），
 *         否则改过 MEC_PULSE_SCALE 后进度显示会与实际不符。
 */
void Mecanum_ForwardPulse(const int32_t dClk[MOTOR_NUM],
                          float *out_dx_mm, float *out_dy_mm, float *out_dRot_mm)
{
    float   d[MOTOR_NUM];
    float   dx;
    float   dy;
    float   dr;
    float   invScale = (float)MEC_PULSE_SCALE_DEN / (float)MEC_PULSE_SCALE_NUM;
    uint8_t i;

    for (i = 0; i < MOTOR_NUM; i++)
    {
        d[i] = (float)dClk[i] * MEC_MM_PER_PULSE * invScale;
    }

    dx = ( d[0] + d[1] + d[2] + d[3]) * 0.25f;
    dy = ( d[0] - d[1] - d[2] + d[3]) * 0.25f;
    dr = ( d[0] - d[1] + d[2] - d[3]) * 0.25f;

    if (out_dx_mm != NULL)
    {
        *out_dx_mm = (float)MEC_SIGN_VX * dx;
    }
    if (out_dy_mm != NULL)
    {
        *out_dy_mm = (float)MEC_SIGN_VY * dy;
    }
    if (out_dRot_mm != NULL)
    {
        *out_dRot_mm = (float)MEC_SIGN_WZ * dr;
    }
}

/**
 * @brief  旋转角度 -> 等效弧长
 * @param  deg: 旋转角度，单位 deg（正=逆时针）
 * @retval 等效弧长，单位 mm
 * @note   含指令刻度修正 MEC_ROT_CMD_GAIN_NUM/DEN（实测"转多/转少"时按比例修正），
 *         默认 1/1 即不修正；与 Mecanum_ArcMmToDeg 严格互逆。
 */
float Mecanum_DegToArcMm(float deg)
{
    return deg * MEC_RAD_PER_DEG * MEC_ROT_RADIUS_MM
           * (float)MEC_ROT_CMD_GAIN_NUM / (float)MEC_ROT_CMD_GAIN_DEN;
}

/**
 * @brief  等效弧长 -> 旋转角度
 * @param  arc_mm: 等效弧长，单位 mm
 * @retval 旋转角度，单位 deg
 * @note   与 Mecanum_DegToArcMm 互逆（含 MEC_ROT_CMD_GAIN 的倒数）。
 */
float Mecanum_ArcMmToDeg(float arc_mm)
{
    return arc_mm / MEC_ROT_RADIUS_MM / MEC_RAD_PER_DEG
           * (float)MEC_ROT_CMD_GAIN_DEN / (float)MEC_ROT_CMD_GAIN_NUM;
}
