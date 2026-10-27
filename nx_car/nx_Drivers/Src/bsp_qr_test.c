/**
 ******************************************************************************
 * @file    bsp_qr_test.c
 * @brief   OLED 扫码测试页实现（P5 GM65 扫码人工验证）
 * @author  nx_car
 * @date    2026-09-10
 ******************************************************************************
 * @note   页面状态由 QR 模块（bsp_qr）的查询接口推导：
 *         QR_GetStatus -> OFFLINE/WAIT/OK/BAD；QR_GetCode -> raw/解析。
 *         页面重绘走脏标志机制：新帧由 QR 模块在主循环成帧后调
 *         Display_QRRefresh 置脏；KEY3 重扫也置脏。无周期高频重绘。
 ******************************************************************************
 */
#include "bsp_qr_test.h"
#include "bsp_qr.h"
#include "bsp_oled_ui.h"
#include "bsp_test.h"
#include "bsp_debug.h"
#include <string.h>
#include <stdio.h>

/* ===================== 内部状态 ===================== */
static uint8_t qrTestActive = 0;   /* 本页活动标志：0=否，1=是 */

/* ===================== 内部工具 ===================== */

/**
 * @brief  颜色值 -> 缩写字符
 * @param  color: 颜色值 1~4
 * @retval 'R'/'B'/'G'/'Y'；非法值返回 '?'
 * @note   1=红(Red) 2=蓝(Blue) 3=绿(Green) 4=黄(Yellow)
 */
static char QRTest_ColorChar(uint8_t color)
{
    switch (color)
    {
        case 1U: return 'R';
        case 2U: return 'B';
        case 3U: return 'G';
        case 4U: return 'Y';
        default: return '?';
    }
}

/* ===================== 对外接口 ===================== */

/**
 * @brief  进入扫码测试页（测试页 QR 项 run 回调调用）
 * @param  无
 * @retval 无
 * @note   切到 PAGE_QR；扫码模块上电已启动，本页只读显示当前缓存结果。
 */
void QR_Test(void)
{
    qrTestActive = 1U;
    Display_SetPage(PAGE_QR);
    Debug_Log("QR", "qr test enter");
}

/**
 * @brief  退出扫码测试页
 * @param  无
 * @retval 无
 * @note   回测试页；不停 UART4 接收（扫码始终后台运行）；
 *         QR 测试项状态复位 IDLE，光标保持 QR。
 */
void QR_Test_Stop(void)
{
    qrTestActive = 0U;
    /* 复位测试页 QR 项状态（不复位光标，光标保持在 QR） */
    testItems[TEST_ID_QR].state = TEST_STATE_IDLE;
    Debug_Log("QR", "qr test exit");
}

/**
 * @brief  扫码测试页按键处理
 * @param  key: 按键 ID
 * @retval 无
 * @note   KEY3=手动重扫触发（清当前结果回 WAIT，等待下一帧）；
 *         KEY4/KEY5=退出回测试页；KEY1/KEY2 忽略。
 */
void QRTest_HandleKey(KeyID_t key)
{
    if (key == KEY_NONE)
    {
        return;
    }

    switch (key)
    {
        case KEY3:   /* 手动重扫：清当前结果，回到 WAIT 等待下一帧 */
            QR_ResetResult();
            Debug_Log("QR", "manual rescan");
            Display_QRRefresh();
            break;

        case KEY4:   /* 退出回测试页（统一退出） */
        case KEY5:   /* 退出回测试页（等效） */
            QR_Test_Stop();
            Display_SetPage(PAGE_TEST);
            break;

        default:
            /* KEY1/KEY2 本页不使用 */
            break;
    }
}

/**
 * @brief  查询扫码测试页是否活动
 * @param  无
 * @retval 1=本页活动，0=非本页
 */
uint8_t QRTest_IsActive(void)
{
    return qrTestActive;
}

/**
 * @brief  获取状态文本（页面第2行）
 * @param  无
 * @retval "WAIT" / "OK" / "BAD" / "OFFLINE"
 */
const char *QRTest_GetStatusText(void)
{
    switch (QR_GetStatus())
    {
        case QR_STAT_OFFLINE: return "OFFLINE";
        case QR_STAT_WAIT:   return "WAIT";
        case QR_STAT_OK:     return "OK";
        case QR_STAT_BAD:    return "BAD";
        default:             return "WAIT";
    }
}

/**
 * @brief  获取原始码字符串（页面第3行）
 * @param  无
 * @retval 原始码指针；无码时返回 "---"
 */
const char *QRTest_GetRawText(void)
{
    const QR_TaskCode_t *code = QR_GetCode();
    if (!QR_IsOnline() || code->raw[0] == '\0')
    {
        return "---";
    }
    return code->raw;
}

/**
 * @brief  获取第一批颜色顺序缩写串（页面第4行）
 * @param  无
 * @retval 静态缓冲指针，如 "1:R 2:B 3:G"；无码时返回 "---"
 */
const char *QRTest_GetBatch1Text(void)
{
    static char line[24];
    const QR_TaskCode_t *code = QR_GetCode();

    if (!code->valid)
    {
        return "---";
    }

    /* 格式：{放置位}:{颜色缩写} 用空格连接 3 项
       例：pos=1,color=2 -> "1:B"；3 项 -> "1:B 2:R 3:G" */
    (void)snprintf(line, sizeof(line),
                          "%u:%c %u:%c %u:%c",
                          (unsigned)code->batch1Pos[0], QRTest_ColorChar(code->batch1Colors[0]),
                          (unsigned)code->batch1Pos[1], QRTest_ColorChar(code->batch1Colors[1]),
                          (unsigned)code->batch1Pos[2], QRTest_ColorChar(code->batch1Colors[2]));
    return line;
}
