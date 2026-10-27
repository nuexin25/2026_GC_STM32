/**
 ******************************************************************************
 * @file    bsp_task_dbg.h
 * @brief   任务点调试页（P9）：把赛题每个任务点做成可单点执行/反复跑的调试入口
 * @author  nx_car
 * @date    2026-10-10
 ******************************************************************************
 * @note   【本页是什么】
 *           主菜单新增 "TDBG" 项进入（顶层页，不属于模块测试子页）。
 *           进入后是一个**6 个任务点的列表菜单**，KEY1/KEY2 选择、**KEY5（或 KEY3）启动**、
 *           KEY4 退出（运行中按 KEY5/KEY3 = 中止）：
 *             1 TRAY PICK    转盘取料：进观察位 -> $RCP 对位(0,0) -> $RZ 定色位
 *                            -> 摆准备位(转盘 Q 号位动作) -> $CCG 轮询 -> 舵机2抓紧 -> 放车载
 *             2 STORE PLACE  暂存区放置：$SHOT 对位 -> 按任务码第2组靶位号放 3 件
 *             3 STORE BACK   暂存区取回：按车载顺序取回 3 件
 *             4 ROUGH PLACE  粗加工区放置：$SHOT 对位 -> 按任务码第2组靶位号放 3 件
 *             5 ROUGH STACK  第二批粗加工区堆叠：$SHOT 对位 -> 按第4组靶位号堆叠 3 件
 *             6 HOME         回启停区：Mission_NavGoTo(0,0)
 *           目的：现场能**单独验证每个任务点**（能发指令、能收数据、能对位、能打日志），
 *           不必跑完整流程。
 *
 *         硬约束：全部非阻塞（发送除外：帧短 ≤32 字节 + 超时保护）；禁止 HAL_Delay；
 *                 日志前缀 [TDBG]；日志禁 %f。
 ******************************************************************************
 */
#ifndef __BSP_TASK_DBG_H
#define __BSP_TASK_DBG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "bsp_key.h"

/* ===================== 任务点枚举 ===================== */

/* 6 个任务点（顺序 = 页面 KEY1/KEY2 选择顺序）。
 * TDBG_NONE(=0) 仅作占位，不参与选择；TDBG_COUNT 为计数哨兵。 */
typedef enum {
    TDBG_NONE = 0,       /* 占位：无选中 */
    TDBG_TRAY_PICK,      /* 转盘取料（单件）：进观察位 -> $RCP 对位(0,0) -> $RZ 定本轮目标色
                            (任务码第1组第 piece 件) -> 摆准备位(转盘 Q 号位动作，不含抓紧)
                            -> $CCG,Q 轮询 -> 命中 -> 舵机2抓紧 -> 放车载(1/2/3) + 颜色绑定 */
    TDBG_STORE_PLACE,    /* 暂存区放置：SHOT 对位 -> 按任务码第2组靶位号放 3 件 */
    TDBG_STORE_BACK,     /* 暂存区取回：按原车载顺序取回 3 件 */
    TDBG_ROUGH_PLACE,    /* 粗加工区放置：SHOT 对位 -> 按任务码第2组靶位号放 3 件 */
    TDBG_ROUGH_STACK,    /* 第二批粗加工区堆叠：SHOT 对位 -> 按第4组靶位号堆叠 3 件 */
    TDBG_HOME,           /* 回启停区：Mission_NavGoTo(0,0) */
    TDBG_COUNT
} TaskDbgItem_t;

/* ===================== 对外接口 ===================== */

/**
 * @brief  任务调试模块初始化（上电调一次）
 * @param  无
 * @retval 无
 * @note   只复位状态，不发电机/机械臂命令、不切页。
 */
void TaskDbg_Init(void);

/**
 * @brief  进入任务调试页（主菜单 TDBG 项调用）
 * @param  无
 * @retval 无
 * @note   中止可能残留的对位 -> 复位状态 IDLE -> 切到 PAGE_TASK_DBG。
 *         **不自动执行**（安全，必须按 KEY3 才动）。
 */
void TaskDbg_Enter(void);

/**
 * @brief  退出任务调试页（KEY4/KEY5 调用）
 * @param  无
 * @retval 无
 * @note   运行中先中止（对位 Abort + 四轮急停），再清活动标志并回主菜单。
 */
void TaskDbg_Exit(void);

/**
 * @brief  本页主循环节拍（非阻塞）
 * @param  无
 * @retval 无
 * @note   在 main 主循环里调度，推进当前任务点的三段式状态机。
 *         **漏调度会卡在某一阶段**。
 */
void TaskDbg_Update(void);

/**
 * @brief  本页按键处理（短按）
 * @param  key: 按键 ID（KEY1~KEY5）
 * @retval 无
 * @note   KEY1/KEY2 = 上一项 / 下一项（运行中忽略）；
 *         KEY3      = 执行当前任务点；运行中 = 中止；
 *         KEY4      = 退出回主菜单；
 *         KEY5      = 中止 + 退出。
 */
void TaskDbg_HandleKey(KeyID_t key);

/**
 * @brief  查询任务调试页是否活动
 * @param  无
 * @retval 1=本页活动（按键交本页处理），0=非本页
 */
uint8_t TaskDbg_IsActive(void);

/* ===================== 页面 4 行文本（DrawPageTaskDbg 用） ===================== */

/**
 * @brief  取页面第 1 行文本：当前项序号 / 总数
 * @retval 形如 "TASK DBG 2/6"
 */
const char *TaskDbg_GetTitleText(void);

/**
 * @brief  取页面第 2 行文本：当前任务点名
 * @retval 形如 "TRAY PICK" / "ROUGH STACK"
 */
const char *TaskDbg_GetNameText(void);

/**
 * @brief  取页面第 3 行文本：状态 + 阶段说明
 * @retval 形如 "IDLE" / "RUN ccg P2" / "DONE" / "FAIL ccg to"
 */
const char *TaskDbg_GetStateText(void);

/**
 * @brief  取页面第 4 行文本：关键数据
 * @retval 形如 "c1 P2 Q3" / "RZ 1:1 2:3 3:4" / "c1->S3"；无数据 "---"
 */
const char *TaskDbg_GetDataText(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_TASK_DBG_H */
