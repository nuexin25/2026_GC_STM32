/**
 ******************************************************************************
 * @file    bsp_vel_route.h
 * @brief   速度模式跑图页（VELROUTE）：按赛题**路段表**跑，全程 0xF6 速度模式
 * @author  nx_car
 * @date    2026-10-24
 ******************************************************************************
 * @note   【和 VELTEST 页的区别】
 *           VELTEST 是"时间开环"跑一条固定形状（直行/过弯），用来整定参数；
 *           本页是**按绝对坐标的路段表**跑（和赛题点位表同一套坐标），
 *           移动段纯时间开环 + IMU 航向闭环；路段表换成赛题的**全部移动段 + 任务点**。
 *
 *         【为什么不复用赛题页】赛题页仍走**位置模式**（脉冲 + 到位帧 + 停稳），
 *           本页独立验证"纯速度模式能不能跑完整场"，互不影响，随时可回退。
 *
 *         【路段表】含**移动段（ROT/GO，绝对坐标）+ 任务点（VR_TASK，真实作业）**，
 *           顺序与坐标 1:1 照抄 bsp_task.c 的赛题路线，所以两边跑出来的是同一条完整赛题。
 *
 *         【按键】
 *           KEY1 / KEY2 : 上一步 / 下一步（运行中无效）
 *           KEY3        : 切换 AUTO（连跑到底）/ STEP（跑一步停一步）
 *           KEY4        : 停止并退出回主菜单
 *           KEY5        : 启动 / 运行中 = 停止
 *           长按 KEY4 或 KEY5 : 停止并退出
 *
 *         【安全】
 *           · 进页不自动跑，必须按 KEY5；
 *           · 进页会把**里程计清零**（路段表是绝对坐标，原点必须就是当前位置）；
 *           · 移动段超 20s / 任务点超 60s 未完成 -> 停车 + ERR，绝不卡死；
 *           · 退出 = 停车 + 四轮失能。
 ******************************************************************************
 */
#ifndef __BSP_VEL_ROUTE_H
#define __BSP_VEL_ROUTE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "bsp_key.h"

/**
 * @brief  进入速度模式跑图页（主菜单 VELROUTE 项）
 * @param  无
 * @retval 无
 * @note   四轮使能抱轴 + 里程计清零 + 切 PAGE_VELROUTE，**不自动跑**。
 */
void VelRoute_Enter(void);

/**
 * @brief  退出本页：停车 + 四轮失能 + 回主菜单
 * @param  无
 * @retval 无
 */
void VelRoute_Exit(void);

/**
 * @brief  查询本页是否活动（main 按键路由用）
 * @retval 1=活动；0=非本页
 */
uint8_t VelRoute_IsActive(void);

/**
 * @brief  主循环节拍：推进当前路段（非阻塞）
 * @param  无
 * @retval 无
 * @note   **漏调度会停在当前段不动**。
 */
void VelRoute_Update(void);

/**
 * @brief  短按处理
 * @param  key: KEY1~KEY5
 * @retval 无
 */
void VelRoute_HandleKey(KeyID_t key);

/**
 * @brief  长按处理：KEY4 / KEY5 = 停止并退出
 * @param  key: 长按的按键 ID
 * @retval 无
 */
void VelRoute_HandleLongKey(KeyID_t key);

/**
 * @brief  取页面某一行文本（8x16 四行）
 * @param  idx: 0=标题+状态，1=步号/步名，2=本段目标，3=当前里程计
 * @retval 静态缓冲指针（每行 ≤16 字符）
 */
const char *VelRoute_GetLine(uint8_t idx);

/* ===================== 串口在线调参（COM8 / UART5） ===================== */
/* 命令（一行，115200 8N1）：
 *   VRCFG V=1000 A=600 FA=300 C=80 T=12 K=12 D=15 Y=40 M=200   // 只改写到的项
 *   VRRUN=1     // 从第 1 段开始 AUTO 跑（未进页会自动进页 + 里程计清零）
 *   VRSTOP      // 停车
 *   VRDUMP      // 打印当前参数 + 当前段号
 *   VRTELE=20   // 遥测周期 ms（0=关闭）
 * 参数含义见 bsp_mission.h 的 MissionVelParam_t。 */

/** @brief 串口启动：fromStep 从 1 开始计数（0 或越界按 1） */
uint8_t VelRoute_CmdStart(uint8_t fromStep);

/** @brief 串口停车 */
void VelRoute_CmdStop(void);

/** @brief ★转向整定专用：原地转到 deg 后自动停车（只转不走，跑完回 IDLE） */
uint8_t VelRoute_CmdRot(int32_t deg);

/** @brief 打印参数与当前段号 */
void VelRoute_CmdDump(void);

/** @brief 设置遥测周期 ms（0=关闭） */
void VelRoute_SetTeleMs(uint16_t ms);

/** @brief 读遥测周期 */
uint16_t VelRoute_GetTeleMs(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_VEL_ROUTE_H */
