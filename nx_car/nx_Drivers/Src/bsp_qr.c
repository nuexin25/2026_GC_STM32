/**
 ******************************************************************************
 * @file    bsp_qr.c
 * @brief   GM65 二维码扫码模块应用层实现（P5，UART4 中断接收 + 帧超时定界）
 * @author  nx_car
 * @date    2026-09-10
 ******************************************************************************
 * @note   工作流程：
 *         (1) QR_Init 启动 UART4 单字节中断接收（HAL_UART_Receive_IT）；
 *         (2) 每收到一字节，bsp_imu.c 的 HAL_UART_RxCpltCallback 调 QR_RxHandler，
 *             把字节搬入接收缓冲、刷新计数与时间戳，然后重装下一字节接收；
 *         (3) 主循环 QR_Update 检查帧间空闲超时，超时则调 QR_ParseFrame 成帧，
 *             校验 + 解析填充结构体 -> 合法则同步 OLED 任务码页 + 日志。
 *         中断只搬字节，解析与显示刷新均在主循环上下文，符合硬约束。
 ******************************************************************************
 */
#include "bsp_qr.h"
#include "usart.h"           /* huart4，UART4 句柄 */
#include "bsp_debug.h"       /* Debug_Log 统一日志（UART5） */
#include "bsp_oled_ui.h"     /* Display_SetTaskCode 同步任务码页 */
#include "bsp_rpi.h"         /* RPi_SendCode：扫码成功后把任务码下发给视觉端（协议 §4） */
#include <string.h>

/* ===================== 内部状态 ===================== */
/* UART4 单字节接收缓冲（HAL 写入地址）。非 static：bsp_imu.c 回调中 extern 引用。 */
uint8_t qrRxByte = 0;

static char     qrRxBuf[QR_MAX_LEN];   /* 接收帧缓冲 */
static uint8_t  qrRxLen = 0;            /* 当前帧已收字节数 */
static uint32_t qrLastByteTick = 0;     /* 上一字节时间戳（HAL_GetTick） */
static uint8_t  qrReceiving = 0;       /* 是否正在接收一帧 */
static uint8_t  qrOnline = 0;          /* 在线标志：收到过字节置 1 */
static uint8_t  qrFrameDone = 0;       /* 已成帧标志（含合法/非法），复位时清零 */
static uint8_t  qrNewFlag = 0;         /* 新合法帧标志（读后自清零） */
static uint32_t qrRxCount = 0;         /* 累计收到字节数（诊断用，永不回落） */
static uint32_t qrErrCount = 0;        /* 累计接收错误次数（ErrorCallback 自增） */
static uint32_t qrLastStatTick = 0;     /* 上次状态心跳时间戳 */

/* 当前任务码（解析结果）。QR_GetCode 返回其指针。 */
static QR_TaskCode_t qrCode;

/* ===================== 内部函数声明 ===================== */
static uint8_t QR_ValidateFrame(const char *buf, uint8_t len);
static void QR_ParseFrame(void);

/* ===================== 内部函数 ===================== */

/**
 * @brief  校验任务码帧格式
 * @param  buf: 帧缓冲（已去尾缀）
 * @param  len: 长度（不含尾缀）
 * @retval 1=合法 0=非法
 * @note   规则：长度==QR_VALID_LEN(15)；4 组各 3 位数字；3 个 '+' 分隔符；
 *         每个数字在 QR_DIGIT_MIN~QR_DIGIT_MAX(1~4) 范围内 ——
 *         1~4 是"颜色 1~4"与"放置靶位号 1~3"的并集（赛题表3 的 1~6 环是
 *         一个靶位内部的同心圆环，不进任务码）；
 *         **颜色是否 ≤4 / 靶位是否 ≤3** 交给任务层（bsp_task 的 Tsk_ParseCode）再判。
 */
static uint8_t QR_ValidateFrame(const char *buf, uint8_t len)
{
    uint8_t i;

    if (len != QR_VALID_LEN)
    {
        return 0U;
    }

    /* 期望：DDD+DDD+DDD+DDD
       索引 3/7/11 为 '+'，其余 12 位为数字且在 1~6 范围 */
    for (i = 0; i < QR_VALID_LEN; i++)
    {
        if (i == 3U || i == 7U || i == 11U)
        {
            if (buf[i] != '+')
            {
                return 0U;
            }
        }
        else
        {
            if (buf[i] < '0' || buf[i] > '9')
            {
                return 0U;
            }
            /* 数字字面值范围校验（1~4：颜色 1~4 ∪ 放置靶位 1~3） */
            {
                uint8_t v = (uint8_t)(buf[i] - '0');
                if (v < QR_DIGIT_MIN || v > QR_DIGIT_MAX)
                {
                    return 0U;
                }
            }
        }
    }
    return 1U;
}

/**
 * @brief  成帧解析（主循环上下文调用）
 * @param  无
 * @retval 无
 * @note   先去尾缀（CR/LF/空格），再调 Validate，合法则解析填充结构体 ->
 *         valid=1 -> 置新帧标志 -> 同步 OLED -> 日志 ok；
 *         非法则 valid=0 -> 日志 bad。重置接收计数准备下一帧。
 */
static void QR_ParseFrame(void)
{
    uint8_t validLen = qrRxLen;

    /* 复制原始串并确保结尾 '\0'（raw 容量 QR_MAX_LEN 足够） */
    if (validLen >= QR_MAX_LEN)
    {
        validLen = (uint8_t)(QR_MAX_LEN - 1U);
    }
    memcpy(qrCode.raw, qrRxBuf, validLen);
    qrCode.raw[validLen] = '\0';

    /* 去尾缀：从末尾剥 CR/LF/空格，得到合法长度 */
    while (validLen > 0U)
    {
        char c = qrCode.raw[validLen - 1U];
        if (c == '\r' || c == '\n' || c == ' ')
        {
            validLen--;
            qrCode.raw[validLen] = '\0';
        }
        else
        {
            break;
        }
    }

    if (QR_ValidateFrame(qrCode.raw, validLen))
    {
        /* 合法：按结构体字段顺序解析 12 位数字
           组1(digits 0~2) -> batch1Colors[0..2]
           组2(digits 3~5, 跳过 '+') -> batch1Pos[0..2]
           组3(digits 6~8, 跳过 '+') -> batch2Colors[0..2]
           组4(digits 9~11, 跳过 '+') -> batch2Pos[0..2]
           实际 12 个数字在 raw 的下标：0,1,2,4,5,6,8,9,10,12,13,14 */
        qrCode.batch1Colors[0] = (uint8_t)(qrCode.raw[0]  - '0');
        qrCode.batch1Colors[1] = (uint8_t)(qrCode.raw[1]  - '0');
        qrCode.batch1Colors[2] = (uint8_t)(qrCode.raw[2]  - '0');
        qrCode.batch1Pos[0]    = (uint8_t)(qrCode.raw[4]  - '0');
        qrCode.batch1Pos[1]    = (uint8_t)(qrCode.raw[5]  - '0');
        qrCode.batch1Pos[2]    = (uint8_t)(qrCode.raw[6]  - '0');
        qrCode.batch2Colors[0] = (uint8_t)(qrCode.raw[8]  - '0');
        qrCode.batch2Colors[1] = (uint8_t)(qrCode.raw[9]  - '0');
        qrCode.batch2Colors[2] = (uint8_t)(qrCode.raw[10] - '0');
        qrCode.batch2Pos[0]    = (uint8_t)(qrCode.raw[12] - '0');
        qrCode.batch2Pos[1]    = (uint8_t)(qrCode.raw[13] - '0');
        qrCode.batch2Pos[2]    = (uint8_t)(qrCode.raw[14] - '0');

        qrCode.valid = 1U;
        qrNewFlag   = 1U;

        /* 同步 OLED 任务码页（主循环上下文，符合硬约束） */
        Display_SetTaskCode(qrCode.raw);
        /* 下发给视觉端（$CODE,<码>，协议 §4：扫码成功、进转盘区前发一次）。
           2026-10-08 按现场要求改为**扫到一次就发一次**（RPi_SendCode 不再按内容去重），
           GM65 连续识读的每一次识别输出都会各发一帧。 */
        RPi_SendCode(qrCode.raw);
        Debug_Log("QR", "raw=\"%s\" ok", qrCode.raw);
    }
    else
    {
        qrCode.valid = 0U;
        Debug_Log("QR", "raw=\"%s\" bad", qrCode.raw);
    }

    qrFrameDone = 1U;

    /* 通知扫码测试页刷新（若当前处于 PAGE_QR，Display_QRRefresh 置脏） */
    Display_QRRefresh();

    /* 重置接收状态，准备下一帧 */
    qrRxLen     = 0U;
    qrReceiving = 0U;
}

/* ===================== 对外接口 ===================== */

/**
 * @brief  扫码模块初始化
 * @param  无
 * @retval 无
 * @note   复位接收状态，启动 UART4 单字节中断接收。上电调用一次，
 *         扫码此后台常驻运行，页面进入/退出不影响接收。
 */
void QR_Init(void)
{
    HAL_StatusTypeDef st;

    qrRxLen        = 0U;
    qrReceiving    = 0U;
    qrOnline       = 0U;
    qrFrameDone    = 0U;
    qrNewFlag      = 0U;
    qrCode.valid   = 0U;
    qrCode.raw[0]  = '\0';
    qrRxCount      = 0U;
    qrErrCount     = 0U;
    qrLastStatTick = HAL_GetTick();
    qrLastByteTick = HAL_GetTick();

    /* 启动 UART4 单字节中断接收（回调在 bsp_imu.c HAL_UART_RxCpltCallback）。
       若返回非 HAL_OK（例如句柄未就绪/已被占用），日志报错，便于定位。 */
    st = HAL_UART_Receive_IT(&huart4, &qrRxByte, 1U);
    if (st != HAL_OK)
    {
        Debug_Log("QR", "init FAIL rx_it=%d", (int)st);
    }
    else
    {
        Debug_Log("QR", "init, baud=9600 rx_it=ok");
    }
}

/**
 * @brief  扫码主循环节拍（非阻塞）
 * @param  无
 * @retval 无
 * @note   帧超时定界：接收中且距上一字节 > QR_FRAME_TIMEOUT_MS 则成帧解析。
 */
void QR_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* 帧超时定界：接收中且距上一字节 > QR_FRAME_TIMEOUT_MS 则成帧解析 */
    if (qrReceiving && ((now - qrLastByteTick) >= QR_FRAME_TIMEOUT_MS))
    {
        QR_ParseFrame();
    }

    /* 状态心跳（每 2s 输出一次到 UART5）：online/rxLen/rxCount/errCount。
       用于定位"扫码器在发但收不到"类问题——rxCount 不增=UART4 接收链路断；
       errCount 增=发生 ORE/FE/NE/PE（ErrorCallback 已重装）。 */
    if ((now - qrLastStatTick) >= 2000U)
    {
        qrLastStatTick = now;
        Debug_Log("QR", "stat on=%u rxLen=%u rx=%lu err=%lu",
                  (unsigned)qrOnline, (unsigned)qrRxLen,
                  (unsigned long)qrRxCount, (unsigned long)qrErrCount);
        /* 在线但累计字节 > 0 时，顺带 dump 当前缓冲前若干字节（hex），
           让 UART5 直接看到扫码器扫到的原始数据，便于核对波特率/接线。 */
        if (qrOnline && qrRxLen > 0U)
        {
            Debug_Log("QR", "buf[%u]=%02X %02X %02X %02X %02X %02X %02X %02X",
                       (unsigned)qrRxLen,
                       (unsigned)(qrRxBuf[0] & 0xFF),
                       (qrRxLen > 1U) ? (unsigned)(qrRxBuf[1] & 0xFF) : 0U,
                       (qrRxLen > 2U) ? (unsigned)(qrRxBuf[2] & 0xFF) : 0U,
                       (qrRxLen > 3U) ? (unsigned)(qrRxBuf[3] & 0xFF) : 0U,
                       (qrRxLen > 4U) ? (unsigned)(qrRxBuf[4] & 0xFF) : 0U,
                       (qrRxLen > 5U) ? (unsigned)(qrRxBuf[5] & 0xFF) : 0U,
                       (qrRxLen > 6U) ? (unsigned)(qrRxBuf[6] & 0xFF) : 0U,
                       (qrRxLen > 7U) ? (unsigned)(qrRxBuf[7] & 0xFF) : 0U);
        }
    }
}

/**
 * @brief  获取当前任务码结构体指针
 * @param  无
 * @retval 任务码结构体指针（valid=0 表示未扫到/非法）
 */
const QR_TaskCode_t *QR_GetCode(void)
{
    return &qrCode;
}

/**
 * @brief  查询自上次调用后是否收到新合法帧（读后自清零）
 * @param  无
 * @retval 1=有新合法帧；0=无
 */
uint8_t QR_IsNew(void)
{
    uint8_t flag = qrNewFlag;
    qrNewFlag = 0U;
    return flag;
}

/**
 * @brief  查询模块是否在线
 * @param  无
 * @retval 1=收到过字节（在线）；0=从未收到（离线）
 */
uint8_t QR_IsOnline(void)
{
    return qrOnline;
}

/**
 * @brief  获取扫码状态（供测试页显示）
 * @param  无
 * @retval QR_STAT_OFFLINE / QR_STAT_WAIT / QR_STAT_OK / QR_STAT_BAD
 * @note   OFFLINE=从未收到字节；WAIT=在线但未成帧；OK=上一帧合法；BAD=上一帧非法。
 */
QR_Status_t QR_GetStatus(void)
{
    if (!qrOnline)
    {
        return QR_STAT_OFFLINE;
    }
    if (!qrFrameDone)
    {
        return QR_STAT_WAIT;
    }
    return qrCode.valid ? QR_STAT_OK : QR_STAT_BAD;
}

/**
 * @brief  复位当前结果（KEY3 手动重扫触发）
 * @param  无
 * @retval 无
 * @note   清除已成帧标志与 valid，测试页显示回到 WAIT 等待下一帧；
 *         不停 UART4 接收（扫码仍后台运行）。
 */
void QR_ResetResult(void)
{
    qrFrameDone    = 0U;
    qrCode.valid   = 0U;
    qrNewFlag      = 0U;
    qrCode.raw[0]  = '\0';
    /* 不复位 qrRxLen/qrReceiving：若正在接收半帧，让其自然超时成帧 */
}

/**
 * @brief  UART4 单字节接收中断处理
 * @param  b: 刚收到的字节
 * @retval 无
 * @note   中断上下文调用（由 bsp_imu.c 的 HAL_UART_RxCpltCallback 转发）。
 *         只搬字节入缓冲、刷新计数与时间戳；不解析、不调 Display、不 printf。
 *         缓冲满（含尾缀余量）则丢弃后续字节，等超时成帧后自动复位。
 */
void QR_RxHandler(uint8_t b)
{
    /* 在线标志：收到过字节即置 1（永不回落，离线判定交给上层） */
    qrOnline    = 1U;
    qrReceiving = 1U;
    qrLastByteTick = HAL_GetTick();
    qrRxCount++;   /* 累计字节计数（诊断用） */

    /* 缓冲未满则存入；满则丢弃（等超时成帧复位） */
    if (qrRxLen < QR_MAX_LEN)
    {
        qrRxBuf[qrRxLen] = (char)b;
        qrRxLen++;
    }
}

/**
 * @brief  通知 UART4 接收错误（ORE/FE/NE/PE）
 * @param  无
 * @retval 无
 * @note   由 bsp_imu.c 的 HAL_UART_ErrorCallback 调用。只累加错误计数，
 *         实际重装 RX IT 由 ErrorCallback 完成。中断上下文，禁止阻塞。
 */
void QR_NotifyError(void)
{
    qrErrCount++;
}
