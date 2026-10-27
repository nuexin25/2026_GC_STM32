/**
 ******************************************************************************
 * @file    bsp_motor.h
 * @brief   步进电机控制封装层（P3 电机单轮调试）
 * @author  nx_car
 * @date    2026-09-09
 ******************************************************************************
 * @note   本模块对上层隐藏 CAN 命令细节，仅封装官方库 Emm_V5.h 已有接口：
 *           - 使能/失能（0xF3）、速度模式（0xF6，方向+转速 RPM+加速度档）、
 *             立即停止（0xFE）；
 *           - 上电 CAN 总线自检：只读实时位置（S_CPOS，应答 0x36 帧由
 *             bsp_Emm_V5.c 接收回调解析），初始化阶段禁止发使能/速度指令。
 *         硬约束：全部函数非阻塞，禁止 HAL_Delay；日志禁 %f（整数打印）。
 ******************************************************************************
 */
#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* ===================== 电机地址宏 ===================== */
/* 电机地址与车轮位置映射（与电机菜单 ID_Addr 一致） */
#define MOTOR_ADDR_FL      1U    /* 前左轮（Front-Left）  */
#define MOTOR_ADDR_FR      2U    /* 前右轮（Front-Right） */
#define MOTOR_ADDR_RL      3U    /* 后左轮（Rear-Left）   */
#define MOTOR_ADDR_RR      4U    /* 后右轮（Rear-Right）  */
#define MOTOR_ADDR_MIN     1U    /* 地址下限，范围检查用   */
#define MOTOR_ADDR_MAX     4U    /* 地址上限，范围检查用   */
#define MOTOR_NUM          4U    /* 电机总数               */

/* ===================== 方向宏（标定用） ===================== */
/* 官方库 Emm_V5_Vel_Control 的 dir 参数：0=CW，1=CCW。
 * 下方每轮 FWD/REV 两个宏决定该轮"正转/反转"实际下发的 dir 值。
 * 出厂默认 FWD=0 / REV=1；若某轮点动"正转"时转向与整车前进方向相反，
 * 把该轮的两个宏值对调（FWD=1 / REV=0）即可，其余轮不受影响。
 * 标定完成后复验四轮，并在《调参手册》方向标定记录表登记。 */
#define MOTOR_DIR_FWD_DEFAULT   0U   /* 正转 dir 默认值（CW）  */
#define MOTOR_DIR_REV_DEFAULT   1U   /* 反转 dir 默认值（CCW） */

#define MOTOR_FL_DIR_FWD    MOTOR_DIR_FWD_DEFAULT   /* 前左正转 dir，标定对调用 */
#define MOTOR_FL_DIR_REV    MOTOR_DIR_REV_DEFAULT   /* 前左反转 dir，标定对调用 */
#define MOTOR_FR_DIR_FWD    MOTOR_DIR_REV_DEFAULT   /* 前右正转 dir，标定对调用 */
#define MOTOR_FR_DIR_REV    MOTOR_DIR_FWD_DEFAULT   /* 前右反转 dir，标定对调用 */
#define MOTOR_RL_DIR_FWD    MOTOR_DIR_FWD_DEFAULT   /* 后左正转 dir，标定对调用 */
#define MOTOR_RL_DIR_REV    MOTOR_DIR_REV_DEFAULT   /* 后左反转 dir，标定对调用 */
#define MOTOR_RR_DIR_FWD    MOTOR_DIR_REV_DEFAULT   /* 后右正转 dir，标定对调用 */
#define MOTOR_RR_DIR_REV    MOTOR_DIR_FWD_DEFAULT   /* 后右反转 dir，标定对调用 */

/* ===================== 可调参数宏（集中调参区） ===================== */
#define MOTOR_JOG_RPM          80U     /* 点动转速，单位 RPM，默认 80，推荐范围 20~200；
                                          禁止 1000RPM 起步（高速裸转仅用于底层演示） */
#define MOTOR_JOG_ACC          30U     /* 点动加速度档，默认 30，范围 0~255；
                                          0=直接启动（无加减速斜坡），值越大斜坡越缓 */
#define MOTOR_RPM_MAX          500U    /* 转速软限幅，单位 RPM，范围 100~1000；
                                          Motor_SetVel 内部对转速绝对值限幅 */
#define MOTOR_ACC_MAX          255U    /* 加速度档上限（官方库 acc 为单字节） */
#define MOTOR_PULSE_PER_REV    3200U   /* 电机轴每转脉冲数（1.8°步进 200 步/转 × 16 细分）。
                                          整车运动距离/角度换算用：
                                          脉冲/mm = 本值 × 减速比 / (π × 轮径)；
                                          细分改动时同步改此宏（见 Emm_V5_Modify_MicroStep） */

/* 驱动器 0x36 上报位置 -> 下发脉冲 的**单位换算系数**：
      下发脉冲域的值 = 上报值 × MOTOR_READ_SCALE_NUM / MOTOR_READ_SCALE_DEN
   实测（P4 里程计标定，2026-09-17 / 复核 2026-09-30）：
     - 下发 5093 脉冲走 500mm（= MEC_PULSE_PER_MM × 500），落点准确；
     - 但 0x36 上报的位置变化量实测为下发脉冲的 **20.48 倍**
       （命令 5093 脉冲，上报域读数 5215，5215/5093 = 1.024 = 20.48/20）；
     - 故 NUM/DEN = **25/512 = 1/20.48**（原为 1/20，偏大 2.4%）。
   【2026-09-30】按本文件下面"标定方法"里给出的算法把 1/20 修正为 25/512：
   上报的是编码器计数（65536/转），命令 clk 是 16 细分脉冲（3200/转），
   65536/3200 = 20.48 是**结构比值**，不是约数。
   影响范围：所有"用上报值算位移 / 判到位"的地方（Chassis_ReadPos / Route / Mission 统一换算）。
   用 1/20 时里程计会**多读 2.4%**（以为走得比实际远），导致跑图/赛题的点位**少走 2.4%**。
   改后必须重跑 500mm 直行复核（`[CHS] odom p=` 单轮增量应约 5093）。
 */
#define MOTOR_READ_SCALE_NUM   25
#define MOTOR_READ_SCALE_DEN   512

#define MOTOR_CHECK_TIMEOUT_MS 2000U   /* 自检单地址应答超时，单位 ms，默认 2000，
                                          可调范围 500~10000；超时记 no reply */
#define MOTOR_QUERY_PERIOD_MS   10U     /* 周期查询节拍，单位 ms，范围 10~100；自检完成后
                                          主循环按此节拍 round-robin 交替查询位置(S_CPOS)
                                          与转速(S_VEL)：前半轮 4 拍查位置、后半轮 4 拍查
                                          转速，共 8 拍，故单轮位置/转速更新间隔均为
                                          8 × 10ms = 80ms。
                                          取值说明（P4 精度相关）：位置更新间隔直接决定
                                          里程定位的"反馈延迟"，而延迟会在停车时变成过冲
                                          （过冲量 ≈ 末端速度 × 更新间隔）。默认 10ms
                                          把间隔压到 80ms，配合末端爬行段把停位误差控制在
                                          几 mm；CAN 负载约 100 帧/s ≈ 27kbps，占 500kbps
                                          总线约 5%，安全。改动前请先确认总线无误码。 */
#define MOTOR_RPM_REPORT_MS     200U    /* 转速上报周期，单位 ms，范围 50~1000；按此周期
                                          读四轮实时转速经 [MOT] 日志上传，静止返回 0 */
#define MOTOR_TX_SLOT_WAIT_MS   2U      /* 下发 CAN 命令前等"空闲发送邮箱"的上限，单位 ms，
                                          默认 2，可调范围 0~20；0=不等（不推荐）。
                                          STM32F1 只有 3 个发送邮箱，四轮位置命令连发 4 帧
                                          可能占满，而官方 can_SendCmd 不检查
                                          HAL_CAN_AddTxMessage 返回值 → 该帧被**静默丢弃**，
                                          现象是"个别轮子偶发不动"。等一个空邮箱即可根除；
                                          一帧约 0.26ms @500kbps，正常几乎不等待。 */
#define MOTOR_TX_PAIR_WAIT_MS   5U      /* 下发**位置模式**命令前，等"**2 个**空闲发送邮箱"的
                                          上限，单位 ms，默认 5，可调范围 1~20。
                                          【为什么要 2 个】官方 can_SendCmd 对 >8 字节命令会
                                          **拆包**发送（0xFD 位置命令 = 2 帧），而它不检查
                                          HAL_CAN_AddTxMessage 返回值：邮箱不够时后一包被
                                          **静默丢弃** → 电机只收到半条命令，直接忽略不动
                                          （现象："四轮一起下发时只有个别轮子响应"）。
                                          所以一次必须凑够整条命令的帧数再发。
                                          【为什么是 5ms】四条命令现在已经背靠背连发（帧间不再
                                          插日志），整段跨度应 ≈ 4×0.6ms ≈ 2.4ms；本上限只在
                                          总线异常/重传时兜底，取 5ms 能覆盖又不会拖慢主循环。 */

/* ===================== 自检结果枚举 ===================== */
typedef enum {
    MOTOR_CHECK_UNCHECKED = 0,   /* 未检（自检未开始或未轮到） */
    MOTOR_CHECK_OK,              /* 成功收到应答帧（在线）     */
    MOTOR_CHECK_TIMEOUT          /* 超时无应答（离线/链路故障） */
} MotorCheckState_t;

/* ===================== 函数声明 ===================== */

/**
 * @brief  电机模块初始化（启动 CAN 总线自检状态机）
 * @param  无
 * @retval 无
 * @note   硬约束：初始化只读不写——仅复位接收状态并依次准备向 1~4 号电机
 *         发"读实时位置 S_CPOS"指令，禁止发使能/速度指令（防上电车轮突转）。
 *         实际查询由 Motor_Update() 在主循环中非阻塞推进。
 */
void Motor_Init(void);

/**
 * @brief  电机自检周期轮询（主循环调用，非阻塞）
 * @param  无
 * @retval 无
 * @note   每次调用只推进一个状态：发查询 -> 等应答/超时 -> 记录并查下一个；
 *         全部查完输出汇总（几/4 在线）。禁止 HAL_Delay。
 */
void Motor_Update(void);

/**
 * @brief  使能/失能单个电机
 * @param  addr: 电机地址 1~4（MOTOR_ADDR_FL~MOTOR_ADDR_RR），超范围忽略
 * @param  on: 1=使能（抱轴/可转），0=失能（自由轴）
 * @retval 无
 */
void Motor_Enable(uint8_t addr, uint8_t on);

/**
 * @brief  失能单个电机（Motor_Enable(addr, 0) 的语义化封装）
 * @param  addr: 电机地址 1~4，超范围忽略
 * @retval 无
 */
void Motor_Disable(uint8_t addr);

/**
 * @brief  立即停止单个电机（官方库 0xFE 急停）
 * @param  addr: 电机地址 1~4，超范围忽略
 * @retval 无
 */
void Motor_Stop(uint8_t addr);

/**
 * @brief  单个电机"缓停"：速度模式发 0 速度帧，让电机按自身加速度档减速到 0
 * @param  addr: 电机地址 1~4，超范围忽略
 * @param  acc:  加速度档 0~255；**必须非 0 才是缓停**，acc=0（无曲线）
 *               时发 0 速度帧等同瞬停，此时请直接用 Motor_Stop
 * @retval 无
 * @note   与 Motor_Stop（0xFE 瞬停）的区别：
 *           - Motor_Stop    ：0xFE 立即停止，用于中止/紧急停车；
 *           - Motor_SoftStop：0xF6 速度帧 + 速度 0，电机走内部减速斜坡，
 *                             用于正常到位收尾，避免硬停冲击。
 *         注意 Motor_SetVel 对 rpm=0 的内部实现是 Motor_Stop（0xFE），
 *         所以需要缓停时必须调用本函数，不能靠 Motor_SetVel(addr, 0, acc)。
 */
void Motor_SoftStop(uint8_t addr, uint8_t acc);

/**
 * @brief  单轮速度控制（有符号转速）
 * @param  addr: 电机地址 1~4，超范围忽略
 * @param  rpm:  有符号转速，单位 RPM，范围 ±MOTOR_RPM_MAX；
 *               正数=正转、负数=反转，内部按该轮方向宏映射 dir；
 *               rpm=0 等价于 Motor_Stop 立即停止
 * @param  acc:  加速度档 0~255，0=直接启动，内部限幅到 MOTOR_ACC_MAX
 * @retval 无
 * @note   转发为官方库 Emm_V5_Vel_Control 速度命令（0xF6 帧）。
 */
void Motor_SetVel(uint8_t addr, int16_t rpm, uint8_t acc);

/**
 * @brief  Same as Motor_SetVel but SILENT (no UART log), rpm=0 -> speed 0 (soft stop)
 * @param  addr: 1~4
 * @param  rpm:  signed RPM, clamped to +-MOTOR_RPM_MAX
 * @param  acc:  accel level 0~255
 * @retval None
 * @note   For high-rate velocity loops (route velocity mode sends 4 frames / 20ms).
 *         rpm=0 here sends a 0xF6 speed-0 frame (decelerate by acc), NOT 0xFE.
 */
void Motor_SetVelQuiet(uint8_t addr, int16_t rpm, uint8_t acc);

/**
 * @brief  读取某轮实时转速（只解析接收缓冲，不发 CAN 帧）
 * @param  addr: 电机地址 1~4
 * @param  out_rpm: 输出实时转速，单位 RPM（有符号 32 位），失败时不改变
 * @retval 0=成功；-1=失败（地址非法 / 指针 NULL / 该轮转速应答状态
 *         非 SUCCESSRECEIVE / 缓冲帧头校验不过）
 * @note   数据来源：bsp_Emm_V5.c 收到 0x35 转速帧（DLC=7）后存入
 *         Emm_V5_Pos_Status.Mx_VelBuff 的 7 字节，布局同位置帧：
 *         buf[0]=0x35、buf[1]=符号、buf[2..5]=转速大端 32 位（RPM）、
 *         buf[6]=0x6B。转速单位需实测标定。
 */
int16_t Motor_GetRpm(uint8_t addr, int32_t *out_rpm);

/**
 * @brief  读取"因 CAN 发送邮箱满而等待"的累计次数
 * @param  无
 * @retval 累计次数；>0 说明确实发生过邮箱占满（旧代码在这种情况下会静默丢帧，
 *         表现为"个别轮子偶发不动"），现已通过下发前等空邮箱根除
 */
uint32_t Motor_GetTxWaitCount(void);

/**
 * @brief  读取某轮实时位置（只解析接收缓冲，不发 CAN 帧）
 * @param  addr: 电机地址 1~4
 * @param  out_angle: 输出电机轴累计位置（有符号 32 位），失败时不改变
 * @retval 0=成功；-1=失败（地址非法 / 指针 NULL / 该轮位置应答状态
 *         非 SUCCESSRECEIVE / 缓冲帧头校验不过）
 * @note   **单位待实测标定**：数据来源是 bsp_Emm_V5.c 收到的 0x36 位置帧
 *         （DLC=7），布局为 buf[0]=0x36、buf[1]=符号（0=正/非0=负）、
 *         buf[2..5]=位置大端 32 位、buf[6]=0x6B（帧尾）。该 32 位数值的
 *         物理单位由电机固件决定，存在两种可能，**必须实测确认**：
 *           - 电机轴脉冲（若如此：每转 = MOTOR_PULSE_PER_REV = 3200）；
 *           - 0.1°（若如此：每转 = 3600）。
 *         两者差 3600/3200 = 1.125，会让走距差 12.5%，所以不能想当然。
 *         实测与换算不放在本函数，统一由上层 bsp_mecanum 的
 *         MEC_PULSE_SCALE_NUM/DEN 宏吸收（改一处即全局生效），
 *         标定步骤见《调参手册》P4 章。
 *         返回值为电机侧累计值（绝对值较大），上层做位移时取两次采样的
 *         差值，不要直接当位移用。
 *         符号已按方向标定宏取反（标定为反的 FR/RR 轮），使"整车前进"
 *         对应四轮增量同为正，与 Motor_SetVel / Motor_PosMove 语义对齐。
 *         更新由 Motor_PeriodicQuery 周期发 S_CPOS 查询触发，间隔
 *         约 8 × MOTOR_QUERY_PERIOD_MS(10ms) = 80ms。
 *         只读缓冲、非阻塞。
 */
int16_t Motor_GetAngle_x10deg(uint8_t addr, int32_t *out_angle);

/**
 * @brief  单轮位置模式（官方库 0xFD）：相对当前位置走指定脉冲后自停
 * @param  addr: 电机地址 1~4，超范围忽略并日志
 * @param  clk:  目标位移（有符号），正=正转方向、负=反转方向，0=忽略；
 *               单位与缩放**待实测**（见 Motor_GetAngle_x10deg 说明）
 * @param  vel:  运行速度，单位电机轴 RPM；**由调用方保证 ≤ MOTOR_RPM_MAX**
 * @param  acc:  加减速度档 0~255，限幅到 MOTOR_ACC_MAX；档位语义实测定
 * @retval 无
 * @note   内部调 Emm_V5_Pos_Control，运动标志 raF=2（相对当前实时位置做
 *         相对位置运动），多机同步 snF=false。加减速轨迹由电机固件内部
 *         规划（梯形/S 形），到位后电机自行停止，无 0xFE 急停冲击。
 *         正负脉冲映射到该轮的 MOTOR_xx_DIR_FWD/REV 方向宏
 *         （与 Motor_SetVel 同一套标定，整车方向标定不要在这里做）。
 *         **本函数不对 vel 单独钳位**：若在这里逐轮钳位，四轮比例会被
 *         削平（限幅前车之鉴）。限幅必须由上层 bsp_mecanum 按"四轮同比
 *         缩小"统一处理后再下发。
 */
void Motor_PosMove(uint8_t addr, int32_t clk, uint16_t vel, uint8_t acc);

/**
 * @brief  查询某轮上电自检结果
 * @param  addr: 电机地址 1~4，超范围返回 MOTOR_CHECK_UNCHECKED
 * @retval MOTOR_CHECK_OK=在线 / MOTOR_CHECK_TIMEOUT=超时无应答 /
 *         MOTOR_CHECK_UNCHECKED=未检
 */
MotorCheckState_t Motor_IsOnline(uint8_t addr);

/**
 * @brief  清除某轮"到位回传"标志
 * @param  addr: 电机地址 1~4
 * @retval 无
 * @note   每下发一个新动作段前必须清除，否则上一段的到位标志会被误当本段已到位。
 *         电机菜单 Response 需为 Reached/Both/Other 才会收到到位帧。
 */
void Motor_ReachClear(uint8_t addr);

/**
 * @brief  查询某轮是否已回传"到位"
 * @param  addr: 电机地址 1~4
 * @retval 1=已到位（收到 0xFD 0x9F 帧）；0=未到位或地址非法
 * @note   这是电机自己的到位判据（位置到达窗口内，比脉冲容差严格得多），
 *         来自电机主动推送的 `地址 + FD + 9F + 6B` 帧。
 */
uint8_t Motor_ReachOk(uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MOTOR_H */
