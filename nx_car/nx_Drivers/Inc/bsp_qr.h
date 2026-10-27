/**
 ******************************************************************************
 * @file    bsp_qr.h
 * @brief   GM65 二维码扫码模块应用层头文件（P5，UART4 中断接收）
 * @author  nx_car
 * @date    2026-09-10
 ******************************************************************************
 * @note   硬件：GM65 扫码模块接 UART4（PC10-TX / PC11-RX，9600-8-N-1），
 *         usart.c 已完成初始化，UART4_IRQn 已在 MspInit 中使能。
 *         本模块只做"接收定界 + 格式校验 + 结构化解析"，不做任务调度
 *         （任务调度属 P9 状态机范畴，本次不实现）。
 *
 *         任务码语义（赛题）：
 *           格式 DDD+DDD+DDD+DDD，4 组 3 位数，分隔符 '+'。
 *           每组 3 位 = 颜色值(1 位) + 放置位(1 位) + 粗加工位(1 位)，
 *           颜色值 1~4（**1=红 2=黄 3=蓝 4=绿**，见《树莓派通讯协议.md》v2.0 §5；旧文档"1红2蓝3绿4黄"作废）。
 *           示例 "134+123+314+231"。
 *           解析后按结构体字段顺序存入 4 个数组（每组 3 位分别填入
 *           batch1Colors / batch1Pos / batch2Colors / batch2Pos）。
 *
 *         硬约束：中断只搬字节；解析在主循环 QR_Update 中做；
 *         Display_SetTaskCode 只在主循环上下文 QR_ParseFrame 中调用。
 ******************************************************************************
 */
#ifndef __BSP_QR_H
#define __BSP_QR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* ===================== 可调参数宏 ===================== */
/* 帧间空闲超时，单位 ms。GM65 9600bps 下每字节约 1.04ms，一帧 15 字节约 16ms，
   50ms 空闲足够定界；帧截断不完整时可调大。可调范围 20~200。 */
#define QR_FRAME_TIMEOUT_MS     50U

/* 单帧最大长度（含可能的尾缀）。"DDD+DDD+DDD+DDD\r\n" = 19，留余量取 32。 */
#define QR_MAX_LEN              32U

/* 合法任务码长度（不含尾缀 CR/LF）：4 组 3 位 + 3 个 '+' = 15。固定值。 */
#define QR_VALID_LEN            15U

/* 颜色值范围（**1=红 2=黄 3=蓝 4=绿**）。
 * 依据：赛题原文《附件2-第十三届浙江省工创大赛…命题与运行》第 7 页
 *       「颜色的编号如下：红色1、黄色2、蓝色3、绿色4」。
 * （旧文档写的"1红2蓝3绿4黄"作废。） */
#define QR_COLOR_MIN            1U
#define QR_COLOR_MAX            4U

/* 任务码里**单个数字位**的合法范围（**1~4**）—— 成帧校验用。
 * 任务码数字只有两类含义，取并集 = 1~4：
 *   ① 颜色位：1~4（红1黄2蓝3绿4，赛题原文第 7 页）；
 *   ② 放置位：**靶位号 1~3** —— 暂存区/粗加工区各有 **3 个靶位**（编号 1/2/3），
 *      不是 1~6。赛题第 6 页"表3"列的 1环~6环是**一个靶位内部的 6 个同心圆环**
 *      （用于摆放精度评分 15/10/7/5/3/1），**不进入任务码**。
 * 颜色是否 ≤4 由任务层再判一次（`bsp_task.c` 的 `Tsk_ParseCode` 用 QR_COLOR_MIN/MAX），
 * 放置位是否 ≤3 由 `TASK_TGT_MIN/MAX` 再判一次。 */
#define QR_DIGIT_MIN            1U
#define QR_DIGIT_MAX            4U

/* ===================== 任务码结构体 ===================== */
/* 供上层 P9 任务调度状态机使用；本次只解析存储，不调度。 */
typedef struct {
    char raw[QR_MAX_LEN];           /* 原始 ASCII 字符串（以 '\0' 结尾） */
    uint8_t valid;                  /* 1=合法 0=非法/未扫到 */
    uint8_t batch1Colors[3];       /* 第一批颜色顺序（颜色值 1~4） */
    uint8_t batch1Pos[3];          /* 第一批放置位 1~3 */
    uint8_t batch2Colors[3];       /* 第二批颜色顺序（颜色值 1~4） */
    uint8_t batch2Pos[3];          /* 第二批粗加工位 1~3 */
} QR_TaskCode_t;

/* ===================== 扫码状态枚举（供测试页显示） ===================== */
typedef enum {
    QR_STAT_OFFLINE = 0,   /* UART4 从未收到字节（模块离线/接线问题） */
    QR_STAT_WAIT,         /* 在线，但尚未成帧（等待帧超时或数据） */
    QR_STAT_OK,           /* 上一帧合法 */
    QR_STAT_BAD           /* 上一帧非法 */
} QR_Status_t;

/* ===================== 对外函数声明 ===================== */

/**
 * @brief  扫码模块初始化
 * @param  无
 * @retval 无
 * @note   复位接收状态，启动 UART4 中断接收（单字节模式，仿 IMU_Init），
 *         输出 [QR] 初始化日志。上电调用一次即可，扫码此后台常驻运行。
 */
void QR_Init(void);

/**
 * @brief  扫码主循环节拍（非阻塞）
 * @param  无
 * @retval 无
 * @note   主循环调度。检查帧超时：接收中且距上一字节 > QR_FRAME_TIMEOUT_MS
 *         则调 QR_ParseFrame 成帧解析。禁止在中断中调用，禁止 HAL_Delay。
 */
void QR_Update(void);

/**
 * @brief  获取当前任务码结构体指针
 * @param  无
 * @retval 任务码结构体指针（valid=0 表示未扫到/非法）
 * @note   返回内部静态结构体指针，调用方只读不写。
 */
const QR_TaskCode_t *QR_GetCode(void);

/**
 * @brief  查询自上次调用后是否收到新合法帧
 * @param  无
 * @retval 1=有新合法帧；0=无。调用后标志自动清零（读后自清零）。
 */
uint8_t QR_IsNew(void);

/**
 * @brief  查询模块是否在线
 * @param  无
 * @retval 1=UART4 收到过字节（在线）；0=从未收到（离线）。
 */
uint8_t QR_IsOnline(void);

/**
 * @brief  获取扫码状态（供测试页显示 OFFLINE/WAIT/OK/BAD）
 * @param  无
 * @retval QR_STAT_OFFLINE / QR_STAT_WAIT / QR_STAT_OK / QR_STAT_BAD
 */
QR_Status_t QR_GetStatus(void);

/**
 * @brief  复位当前结果（KEY3 手动重扫触发）
 * @param  无
 * @retval 无
 * @note   清除已成帧标志与 valid，测试页显示回到 WAIT，等待下一帧；
 *         不影响 UART4 接收（扫码仍后台运行），不阻塞。
 */
void QR_ResetResult(void);

/**
 * @brief  UART4 单字节接收中断处理（HAL_UART_RxCpltCallback 中调用）
 * @param  b: 刚收到的字节
 * @retval 无
 * @note   中断上下文调用。只把字节存入接收缓冲并刷新计数/时间戳，
 *         不解析、不调 Display_SetTaskCode、不 printf。缓冲满则丢弃后续字节。
 */
void QR_RxHandler(uint8_t b);

/**
 * @brief  通知 UART4 接收错误（ORE/FE/NE/PE）
 * @param  无
 * @retval 无
 * @note   由 bsp_imu.c 的 HAL_UART_ErrorCallback 调用。只累加错误计数，
 *         实际重装 RX IT 由 ErrorCallback 完成。中断上下文，禁止阻塞。
 */
void QR_NotifyError(void);

#ifdef __cplusplus
}
#endif

/* ===================== UART4 接收缓冲（供 bsp_imu.c 回调重装 IT 用） ===================== */
/* 非静态：HAL_UART_Receive_IT 需在此变量地址写入收到的字节，bsp_imu.c 的
   HAL_UART_RxCpltCallback 中 extern 引用后重装。 */
extern uint8_t qrRxByte;

#endif /* __BSP_QR_H */
