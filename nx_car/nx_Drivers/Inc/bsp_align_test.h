/**
 ******************************************************************************
 * @file    bsp_align_test.h
 * @brief   对位调参测试页（P24）：单独跑一次对位 + 逐帧输出调参数据
 * @author  nx_car
 * @date    2026-10-15
 ******************************************************************************
 * @note   【做什么】
 *           在主菜单选 "ALNT" 进入本页，可以**单独**触发一次视觉对位
 *           （$RCP / $SHOT / $SHOT2 三选一），不必跑整条赛题流程；
 *           对位进行中按 `ALNT_LOG_MS` 周期打印一行**调参数据**：
 *
 *             [ATN] t=1230 st=2 f=12 dt=100 px=-12,34 mm=-24,68 q=-8 u=1820,-410,0 w=22 db=1
 *
 *           字段：t=已用时ms  st=状态(1=YAW 2=RUN 3=READ 4=SETTLE)  f=已用视觉帧数
 *                 dt=本帧间隔ms  px=图像偏移  mm=车体误差(mm)  q=平行度(0.1°)
 *                 u=三轴PID输出×100(前进,横移,旋转 RPM)  w=本拍最大单轮RPM  db=生效死区
 *           把这串数据发给我，就能判断"走不到位 / 冲过头 / 稳态残差 / 帧率太低"，
 *           从而给出下一组 `ALIGN_PID_*`。
 *
 *         【按键】
 *           KEY1 / KEY2 : 切换数据源（RCP -> SHOT -> SHOT2 循环）
 *           KEY3        : 启动对位 / 运行中=停止
 *           KEY4        : 退出（先停止）
 *           KEY5        : 停止（不退出）
 *
 *         【安全】进页时只"使能抱轴"四轮，**不发任何速度**；只有按 KEY3 且视觉数据有效
 *           才会真正动。退出/停止走 `Align_Abort()`（四轮软停）。
 *
 *         硬约束：非阻塞；禁止 HAL_Delay；日志禁 %f。
 ******************************************************************************
 */
#ifndef __BSP_ALIGN_TEST_H
#define __BSP_ALIGN_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "bsp_key.h"     /* KeyID_t（按键处理接口用） */

/* ===================== 可调参数 ===================== */

/* 调参日志周期（ms）：对位运行中每隔这么久打一行 `[ATN] ...`。
 * 默认 100（≈ 视觉帧周期）：一帧一条，正好能看出"每帧误差 -> 每帧输出"的对应关系。
 * 调小（50）会更密、更利于看细节，但串口占用变高；调大（200）只适合看趋势。 */
#ifndef ALNT_LOG_MS
#define ALNT_LOG_MS             100U
#endif

/* 等视觉有效数据的上限（ms）：按 KEY3 后先让**手臂去观察位**，摆好后再发请求等数据；
 * 超过本值还没数据 -> 判失败（日志 `no data`）。默认 5000。 */
#ifndef ALNT_WAIT_MS
#define ALNT_WAIT_MS            5000U
#endif

/* 手臂摆到观察位的上限（ms）：超过还没完成 -> 判失败（不带着没摆好的手臂开跑）。
 * 默认 12000（回观察位是 S1 -> S2 -> M5 -> M6 四步）。 */
#ifndef ALNT_ARM_MS
#define ALNT_ARM_MS             12000U
#endif

/* 等待期间重发请求的周期（ms），默认 200（比树莓派"识别+回帧"略大）。 */
#ifndef ALNT_REQ_MS
#define ALNT_REQ_MS             200U
#endif

/* 页面刷新节流（ms）：OLED 重绘比日志慢，默认 200。 */
#ifndef ALNT_UI_MS
#define ALNT_UI_MS              200U
#endif

/* ===================== 对外接口 ===================== */

/**
 * @brief  进入对位调参页（四轮使能抱轴 + 切页，**不自动开跑**）
 */
void AlnTest_Enter(void);

/**
 * @brief  退出对位调参页（先中止对位，再回主菜单）
 */
void AlnTest_Exit(void);

/**
 * @brief  本页是否活动
 * @retval 1=活动
 */
uint8_t AlnTest_IsActive(void);

/**
 * @brief  本页主循环节拍（非阻塞）：推进"等数据 / 运行 / 结束"并打印调参日志
 */
void AlnTest_Update(void);

/**
 * @brief  本页按键处理（短按）
 * @param  key: 按键 ID
 */
void AlnTest_HandleKey(KeyID_t key);

/* ===== ★【2026-10-15】命令行入口（UART5 在线调参 / PC 自动调参用） ===== */

/**
 * @brief  用指令启动一次对位（等价于"进页 + 选源 + 按 KEY3"）
 * @param  src: 0=$RCP 1=$SHOT 2=$SHOT2
 * @retval 无
 * @note   本页没活动时**自动进页**（四轮抱轴、切页）；若上次还在跑，先停止再重开。
 *         随后走与 KEY3 完全相同的流程：手臂先去观察位 -> 等视觉 -> 开跑。
 */
void AlnTest_StartSrc(uint8_t src);

/**
 * @brief  用指令停止当前对位（不退出本页，等价 KEY5）
 */
void AlnTest_StopCmd(void);

/**
 * @brief  用指令退出本页（先停止，等价 KEY4）
 */
void AlnTest_ExitCmd(void);

/**
 * @brief  当前测试页状态码（命令行调参用）
 * @retval 0=IDLE 1=ARM 2=WAIT 3=RUN 4=DONE 5=FAIL
 */
uint8_t AlnTest_GetStateCode(void);

/**
 * @brief  取页面第 1 行文本（标题 + 数据源）
 * @retval 形如 "ALN TEST 1/3 RCP"
 */
const char *AlnTest_GetTitleText(void);

/**
 * @brief  取页面第 2 行文本（图像偏移 + 平行度）
 * @retval 形如 "px=-12,34 q=-8" / "px=-- (wait data)"
 */
const char *AlnTest_GetErrText(void);

/**
 * @brief  取页面第 3 行文本（三轴 PID 输出）
 * @retval 形如 "u=18.2,-4.1 w=22"
 */
const char *AlnTest_GetOutText(void);

/**
 * @brief  取页面第 4 行文本（状态 + 用时 + 按键提示）
 * @retval 形如 "RUN f=12 1.8s K3=STOP"
 */
const char *AlnTest_GetStateText(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ALIGN_TEST_H */
