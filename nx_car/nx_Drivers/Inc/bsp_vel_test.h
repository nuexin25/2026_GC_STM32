/**
 ******************************************************************************
 * @file    bsp_vel_test.h
 * @brief   OLED 速度模式测试页（P25）：**纯陀螺仪 + 时间控制**，不用轮编码器测距
 * @author  nx_car
 * @date    2026-10-24
 ******************************************************************************
 * @note   【要验证什么】
 *           用"速度模式(0xF6)"跑图时，距离**不用轮编码器**，只靠
 *           "速度 × 时间"开环；方向靠 IMU 陀螺仪闭环。用来现场测：
 *             · 同样时间/速度下，实际走了多远（皮尺量）；
 *             · 陀螺仪直线保持 + 原地转 90° 的精度；
 *             · 打滑/加减速对"时间-距离"的影响。
 *
 *         【一键流程（KEY5 启动）】★过弯**不停车**（低速转直角弯）：
 *           直行 T1（加速到 V）
 *           -> 按 A 降到过弯速度 VC（不降到 0）
 *           -> **边低速前进边转 G 度**（陀螺仪闭环，转弯半径 ≈ VC / 角速度）
 *           -> 直行 T2（从 VC 加速到 V）
 *           -> 最后减速停 -> DONE（四轮停住，等下次 KEY5）
 *
 *         【参数（页面上可调，掉电不保存）】
 *           V  : 巡航速度 mm/s      （步进 50，100~1500）
 *           T1 : 第一段直行**总时间** ms（从启动算到开始过弯，步进 100，200~9999）
 *           T2 : 第二段直行总时间 ms（从过弯结束算起，步进 100，200~9999）
 *           A  : 加减速度 mm/s²     （步进 50，100~1000）
 *           G  : 过弯转角 deg       （步进 5，-180~180；正=左转）
 *           VC : 过弯速度 mm/s      （步进 50，0~600，且不超过 V；**0 = 原地慢转**）
 *           K  : 直行航向保持 KP    （×0.1，步进 0.1，0.2~2.0，默认 0.8）
 *           D  : 直行航向保持 KD    （×0.1，步进 0.1，0~3.0，默认 1.5）
 *         ⚠ 参数共 8 个，页面一次显示 6 个；选中 K/D 时窗口会自动下滚。
 *
 *         【横向摆动（蛇形）怎么调】
 *           · 摆得**快而大** -> KD 太小：先把 D 加大（1.5 -> 1.8 -> 2.2）；
 *           · 摆得**慢而软**、回到直线要很久 -> KP 太小：把 K 加大（0.8 -> 1.0）；
 *           · 小幅高频抖 -> KD 过大或地面滑：把 D 降到 1.2，检查胎面/地面；
 *           · 判据：出弯后 1~2 s 内应贴着直线走，`VT` 日志里 yaw 不应持续左右摆。
 *
 *         【按键（本页语义独立）】
 *           KEY1 / KEY2 : 选上一个 / 下一个参数
 *           KEY3        : 选中参数 +1 档
 *           KEY4        : 选中参数 -1 档
 *           KEY5        : 启动 / 运行中=停止
 *           长按 KEY4 或 KEY5 : 停止并退出回主菜单
 *
 *         【安全】
 *           · 进入本页不自动跑，必须按 KEY5；
 *           · IMU 离线时拒绝启动（没有航向就没法闭环）；
 *           · 任意时刻按 KEY5 立即停（速度 0，由驱动器自身减速）；
 *           · 退出本页 = 停止 + 四轮失能。
 ******************************************************************************
 */
#ifndef __BSP_VEL_TEST_H
#define __BSP_VEL_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "bsp_key.h"

/* ===================== 函数声明 ===================== */

/**
 * @brief  进入速度模式测试页（主菜单 VELTEST 项）
 * @param  无
 * @retval 无
 * @note   切到 PAGE_VELTEST；四轮使能抱轴；状态复位 READY，**不自动启动**。
 */
void VelTest_Enter(void);

/**
 * @brief  退出本页：停止 + 四轮失能 + 回主菜单
 * @param  无
 * @retval 无
 */
void VelTest_Exit(void);

/**
 * @brief  查询本页是否活动（main 按键路由用）
 * @retval 1=活动；0=非本页
 */
uint8_t VelTest_IsActive(void);

/**
 * @brief  本页主循环节拍（非阻塞）：速度曲线 / 转弯闭环 / 状态推进
 * @param  无
 * @retval 无
 * @note   **漏调度会停在某个状态不动**（速度不再更新）。
 */
void VelTest_Update(void);

/**
 * @brief  短按处理（本页活动时由 main 路由）
 * @param  key: KEY1~KEY5
 * @retval 无
 */
void VelTest_HandleKey(KeyID_t key);

/**
 * @brief  长按处理：KEY4 / KEY5 = 停止并退出
 * @param  key: 长按的按键 ID
 * @retval 无
 */
void VelTest_HandleLongKey(KeyID_t key);

/**
 * @brief  取页面某一行文本（供 bsp_oled_ui 绘制 8x16 四行）
 * @param  idx: 0=标题+状态，1=参数行1(V,T1)，2=参数行2(T2,A)，3=参数行3(G)或运行中的航向
 * @retval 静态缓冲指针（每行 ≤16 字符）
 */
const char *VelTest_GetLine(uint8_t idx);

/* ===================== 串口调参接口（COM8 / UART5） ===================== */
/* 命令（在 COM8 上发一行，115200 8N1）：
 *   VTCFG V=600 T1=1500 T2=1500 A=300 G=90 C=250 K=8 D=15   // 只改写到的项
 *   VTRUN            // 启动（未进页会自动进页）
 *   VTSTOP           // 停止
 *   VTEXIT           // 停止并退出本页
 *   VTDUMP           // 打印当前参数
 *   VTTELE=50        // 实时数据周期 ms（0=关闭；默认 50ms=20Hz）
 * 关掉数据后仍会保留最近一次参数。K/D 是 ×10 的整数（K=8 -> KP 0.8）。 */

typedef enum {
    VT_CMD_V = 0,   /* mm/s */
    VT_CMD_T1,      /* ms */
    VT_CMD_T2,      /* ms */
    VT_CMD_A,       /* mm/s^2 */
    VT_CMD_G,       /* deg */
    VT_CMD_VC,      /* mm/s */
    VT_CMD_KP,      /* ×10 */
    VT_CMD_KD,      /* ×10 */
    VT_CMD_TRK,     /* 过弯位置环 KP ×10 */
    VT_CMD_TRR,     /* 过弯角速度环 KP ×10 */
    VT_CMD_TRM,     /* 过弯角速度上限 dps */
    VT_CMD_TRA,     /* 过弯制动包络 dps^2 / 10 */
    VT_CMD_TRD,     /* 过弯前减速率 mm/s^2 / 10 */
    VT_CMD_SQ,      /* 0=单段 1=正方形路线 */
    VT_CMD_LEG,     /* 正方形边长 mm */
    VT_CMD_COUNT
} VelCmdParam_t;

/** @brief 串口改一个参数（范围自动夹取）；返回 1=成功 */
uint8_t VelTest_CmdSet(uint8_t idx, int32_t v);

/** @brief 串口启动（必要时自动进页 + 使能四轮）；返回 1=已开跑 */
uint8_t VelTest_CmdStart(void);

/** @brief 串口停止 */
void VelTest_CmdStop(void);

/** @brief 串口退出本页 */
void VelTest_CmdExit(void);

/** @brief 打印当前全部参数 */
void VelTest_CmdDump(void);

/** @brief 设置实时数据周期（0=关闭） */
void VelTest_SetTeleMs(uint16_t ms);

/** @brief 设置 0xF6 加减速档（0~255，越大越缓；串口 AC= 用） */
void VelTest_CmdSetAcc(uint8_t acc);

/** @brief 读实时数据周期 */
uint16_t VelTest_GetTeleMs(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_VEL_TEST_H */
