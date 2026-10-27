/**
 ******************************************************************************
 * @file    bsp_rpi_test.c
 * @brief   OLED 树莓派通讯测试页实现（P6 视觉链路人工验证，协议 v2.0）
 * @author  nx_car
 * @date    2026-09-15
 ******************************************************************************
 * @note   页面数据全部取自 bsp_rpi 的查询接口（RPi_GetRcp / RPi_GetRz /
 *         RPi_GetCcg / RPi_GetShot / RPi_GetVsMode / RPi_IsAlive / RPi_IsOnline /
 *         RPi_GetRxCount / RPi_GetBadCount）。
 *         重绘走脏标志机制：新帧到达或数据过期时由 bsp_rpi 调 Display_RPiRefresh
 *         置脏，本页切指令/发送时也置脏；无周期高频重绘。
 *         本页只验证链路与数据，**不驱动底盘、不联动机械臂**（v2.0 无此需求）。
 *
 *         页面四行 8x16 大字体（每行最多 16 字符）：
 *           第1行  "RPI v2.0" + 在线状态  RPI v2.0 ONLINE / NOHB / OFFLINE
 *           第2行  视觉端状态 + 帧计数  TURN RX:12 B:0
 *           第3行  $RZ 结果（位号:颜色）  RZ 1:1 2:3 3:4（无数据 RZ --）
 *           第4行  当前选中项 + $RCP/$CCG/$SHOT 摘要  CCG3 R42,-15 C3=1
 *         按键语义独立：
 *           KEY1/KEY2 = 上一项 / 下一项（RCP/RZ/CCG,1/CCG,2/CCG,3/SHOT/CODE 循环）；
 *           KEY3      = 发送当前选中项（v2.0 不做任何机械臂联动，直接发帧）；
 *           KEY4/KEY5 = 退出回测试页。
 ******************************************************************************
 */
#include "bsp_rpi_test.h"
#include "bsp_rpi.h"
#include "bsp_qr.h"          /* QR_GetCode：任务码项取 GM65 扫到的原文 */
#include "bsp_oled_ui.h"
#include "bsp_test.h"
#include "bsp_debug.h"
#include <string.h>
#include <stdio.h>

/* ===================== 可调参数 ===================== */

/* 指令项个数：RCP / RZ / CCG,1 / CCG,2 / CCG,3 / SHOT / CODE = 7 条（KEY1/KEY2 循环选择） */
#define RPI_TEST_CMD_COUNT       7U

/* ===================== 内部状态 ===================== */

static uint8_t rpiTestActive = 0U;   /* 本页活动标志：0=否，1=是 */
static uint8_t rpiTestCmdIdx = 0U;   /* 当前选中的指令项下标（0 ~ RPI_TEST_CMD_COUNT-1） */

/* 指令项短名（第4行前缀 + 日志；顺序即 KEY1/KEY2 切换顺序） */
static const char *const rpiTestCmdName[RPI_TEST_CMD_COUNT] = {
    "RCP",     /* $RCP        请求转盘中心偏移 */
    "RZ",      /* $RZ         请求转盘三位号/颜色 */
    "CCG1",    /* $CCG,1      请求 1 号位颜色 */
    "CCG2",    /* $CCG,2      请求 2 号位颜色 */
    "CCG3",    /* $CCG,3      请求 3 号位颜色 */
    "SHOT",    /* $SHOT       请求打靶区圆环数据 */
    "CODE"     /* $CODE,<码>  下发任务码（取 GM65 扫到的原文） */
};

/* ===================== 内部函数 ===================== */

/**
 * @brief  视觉端状态枚举 → 4 字符内文本（第2行显示用）
 * @param  m: RPi_VsMode_t
 * @retval "IDLE" / "TURN" / "SHOT" / "UNK"
 */
static const char *RPiTest_ModeText(RPi_VsMode_t m)
{
    switch (m)
    {
        case RPI_VS_IDLE: return "IDLE";
        case RPI_VS_TURN: return "TURN";
        case RPI_VS_SHOT: return "SHOT";
        default:          return "UNK";
    }
}

/**
 * @brief  发送当前选中的指令项
 * @param  无
 * @retval 无
 * @note   任务码项取 GM65 扫到的原文；没扫到码时传 NULL，
 *         由 bsp_rpi 改发占位码 "---+---+---+---"（避免发出畸形帧）。
 *         v2.0 下不做任何机械臂/底盘联动，直接发帧。
 */
static void RPiTest_SendCurrent(void)
{
    switch (rpiTestCmdIdx)
    {
        case 0U:  RPi_ReqRcp();        break;   /* $RCP */
        case 1U:  RPi_ReqRz();         break;   /* $RZ */
        case 2U:  RPi_ReqCcg(1U);      break;   /* $CCG,1 */
        case 3U:  RPi_ReqCcg(2U);      break;   /* $CCG,2 */
        case 4U:  RPi_ReqCcg(3U);      break;   /* $CCG,3 */
        case 5U:  RPi_ReqShot();       break;   /* $SHOT */
        case 6U:                                /* $CODE,<任务码> */
        {
            const QR_TaskCode_t *qr = QR_GetCode();
            RPi_SendCode(((qr != NULL) && (qr->valid != 0U)) ? qr->raw : NULL);
            break;
        }
        default: break;
    }
}

/* ===================== 对外接口 ===================== */

/**
 * @brief  进入树莓派测试页（测试页 RPI 项 run 回调）
 * @param  无
 * @retval 无
 * @note   切到 PAGE_RPI；不自动发送任何指令。
 */
void RPI_Test(void)
{
    rpiTestActive = 1U;
    Display_SetPage(PAGE_RPI);
    Debug_Log("RPi", "rpi test enter");
}

/**
 * @brief  树莓派测试页周期刷新（update 回调）
 * @param  无
 * @retval 无
 * @note   在线状态变化时置脏；数据行本身由 bsp_rpi 在"收到新帧 / 数据过期 /
 *         心跳超时"时驱动置脏，这里不重复周期置脏。
 */
void RPI_Test_Update(void)
{
    static uint8_t lastAlive = 0xFFU;   /* 0xFF = 尚未取过 */

    if (RPi_IsAlive() != lastAlive)
    {
        lastAlive = RPi_IsAlive();
        Display_RPiRefresh();
    }
}

/**
 * @brief  退出树莓派测试页
 * @param  无
 * @retval 无
 * @note   回测试页；不停 USART1 接收（通讯始终后台运行）；
 *         RPI 测试项状态复位 IDLE，光标保持 RPI。
 */
void RPI_Test_Stop(void)
{
    rpiTestActive = 0U;
    /* 复位测试页 RPI 项状态（不复位光标，光标保持在 RPI） */
    testItems[TEST_ID_RPI].state = TEST_STATE_IDLE;
    Debug_Log("RPi", "rpi test exit");
}

/**
 * @brief  树莓派测试页按键处理
 * @param  key: 按键 ID
 * @retval 无
 * @note   KEY1/KEY2=切换指令项；KEY3=发送当前项；KEY4/KEY5=退出回测试页。
 */
void RPiTest_HandleKey(KeyID_t key)
{
    if (key == KEY_NONE)
    {
        return;
    }

    switch (key)
    {
        case KEY1:   /* 上一条指令（循环） */
            rpiTestCmdIdx = (uint8_t)((rpiTestCmdIdx == 0U)
                                      ? (RPI_TEST_CMD_COUNT - 1U)
                                      : (rpiTestCmdIdx - 1U));
            Debug_Log("RPi", "sel=%u/%u %s", (unsigned)(rpiTestCmdIdx + 1U),
                      (unsigned)RPI_TEST_CMD_COUNT, rpiTestCmdName[rpiTestCmdIdx]);
            Display_RPiRefresh();
            break;

        case KEY2:   /* 下一条指令（循环） */
            rpiTestCmdIdx = (uint8_t)((rpiTestCmdIdx + 1U) % RPI_TEST_CMD_COUNT);
            Debug_Log("RPi", "sel=%u/%u %s", (unsigned)(rpiTestCmdIdx + 1U),
                      (unsigned)RPI_TEST_CMD_COUNT, rpiTestCmdName[rpiTestCmdIdx]);
            Display_RPiRefresh();
            break;

        case KEY3:   /* 发送当前选中的指令 */
            RPiTest_SendCurrent();
            Display_RPiRefresh();
            break;

        case KEY4:   /* 退出回测试页（统一退出键） */
        case KEY5:   /* 退出回测试页（兼容旧语义） */
            RPI_Test_Stop();
            Display_SetPage(PAGE_TEST);
            break;

        default:
            break;
    }
}

/**
 * @brief  查询树莓派测试页是否活动
 * @param  无
 * @retval 1=本页活动，0=非本页
 */
uint8_t RPiTest_IsActive(void)
{
    return rpiTestActive;
}

/**
 * @brief  获取第1行文本（标题 + 在线状态）
 * @param  无
 * @retval 静态字符串
 * @note   三态显示（判据见协议 §8）：
 *           ONLINE  = 心跳正常（收到过 $PING 且未超时）—— 视觉端在工作
 *           NOHB    = 收到过字节但心跳丢了 —— 接线通、视觉端可能挂了/没开心跳
 *           OFFLINE = 一个字节都没收到 —— 接线/共地/波特率问题，或视觉端没在发
 */
const char *RPiTest_GetTitleText(void)
{
    if (RPi_IsAlive() != 0U)
    {
        return "RPI v2.0 ONLINE";
    }
    return (RPi_IsOnline() != 0U) ? "RPI v2.0 NOHB" : "RPI v2.0 OFFLINE";
}

/**
 * @brief  获取第2行文本（视觉端状态 + 帧计数）
 * @param  无
 * @retval 静态缓冲指针，如 "TURN RX:12 B:0"
 * @note   RX=有效帧累计；B=坏帧累计（未知帧名不计入坏帧）。
 */
const char *RPiTest_GetColorText(void)
{
    static char line[24];

    (void)snprintf(line, sizeof(line), "%s RX:%lu B:%lu",
                   RPiTest_ModeText(RPi_GetVsMode()),
                   (unsigned long)RPi_GetRxCount(),
                   (unsigned long)RPi_GetBadCount());
    return line;
}

/**
 * @brief  获取第3行文本（$RZ 结果：位号:颜色）
 * @param  无
 * @retval 静态缓冲指针，如 "RZ 1:1 2:3 3:4"；无数据时 "RZ --"
 * @note   颜色编号：1红 2黄 3蓝 4绿；0=未识别。$RZ 过期后 byPos 已清零。
 */
const char *RPiTest_GetTargetText(void)
{
    static char line[24];
    const RPi_Rz_t *rz = RPi_GetRz();

    if (rz->valid != 0U)
    {
        (void)snprintf(line, sizeof(line), "RZ 1:%u 2:%u 3:%u",
                       (unsigned)rz->byPos[1], (unsigned)rz->byPos[2],
                       (unsigned)rz->byPos[3]);
    }
    else
    {
        (void)snprintf(line, sizeof(line), "RZ --");
    }
    return line;
}

/**
 * @brief  获取第4行文本（当前选中项 + $RCP/$CCG/$SHOT 摘要）
 * @param  无
 * @retval 静态缓冲指针，如 "CCG3 R42,-15 C3=1"
 * @note   前缀是 KEY3 将要发送的指令项短名；随后是最新有效的 $RCP（R dx,dy）、
 *         $CCG（C pos=color）、$SHOT（S dx,dy）摘要。三者各自过期/未收到则不显示；
 *         全无数据时显示 "---"。8x16 行宽 16 字符，三项同时有效时尾部可能被截断，
 *         完整值看 UART5 日志。
 */
const char *RPiTest_GetCmdText(void)
{
    static char line[28];
    static char rcpS[16];
    static char ccgS[12];
    static char shotS[16];
    const RPi_Rcp_t  *rcp  = RPi_GetRcp();
    const RPi_Ccg_t  *ccg  = RPi_GetCcg();
    const RPi_Shot_t *shot = RPi_GetShot();

    rcpS[0]  = '\0';
    ccgS[0]  = '\0';
    shotS[0] = '\0';

    if (rcp->valid != 0U)
    {
        (void)snprintf(rcpS, sizeof(rcpS), " R%d,%d", (int)rcp->dxPx, (int)rcp->dyPx);
    }
    if (ccg->valid != 0U)
    {
        (void)snprintf(ccgS, sizeof(ccgS), " C%d=%d", (int)ccg->reqPos, (int)ccg->color);
    }
    if (shot->valid != 0U)
    {
        (void)snprintf(shotS, sizeof(shotS), " S%d,%d", (int)shot->dxPx, (int)shot->dyPx);
    }
    if ((rcpS[0] == '\0') && (ccgS[0] == '\0') && (shotS[0] == '\0'))
    {
        (void)snprintf(rcpS, sizeof(rcpS), " ---");
    }

    (void)snprintf(line, sizeof(line), "%s%s%s%s",
                   rpiTestCmdName[rpiTestCmdIdx], rcpS, ccgS, shotS);
    return line;
}
