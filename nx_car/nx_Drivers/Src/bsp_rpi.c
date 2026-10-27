/**
 ******************************************************************************
 * @file    bsp_rpi.c
 * @brief   树莓派视觉通讯模块实现（P6，USART1 中断收发）
 * @author  nx_car
 * @date    2026-09-15
 ******************************************************************************
 * @note   协议契约：《树莓派通讯协议.md》v2.0（项目根目录，唯一有效版本）。
 *         帧格式：$<内容>*<校验>\r\n
 *           校验 = 对 '$' 之后、'*' 之前每个字符逐字节 XOR，2 位大写十六进制；
 *           `**` 表示跳过校验。
 *         接收：$RCP,dx,dy / $RZ,p1,c1,p2,c2,p3,c3 / $CCG,c / $SHOT,q1..q3,dx,dy /
 *               $PING,<MODE>
 *         发送：$RCP / $RZ / $CCG,<pos> / $SHOT / $CODE,<任务码>
 *
 *         实现要点（与硬约束对应）：
 *           1) 中断只搬字节：RPi_RxHandler 只存缓冲、刷时间戳；
 *           2) 主循环按行成帧：RPi_Update 调 RPi_ProcessLines；
 *           3) 未知帧名静默忽略，**不计坏帧**（向前兼容，避免掩盖真实链路问题）；
 *              只有"已识别的帧名 + 段数不足/校验错"才算坏帧；
 *           4) 数值越界**钳位**（不丢整帧），只在段数不足/校验错时丢帧；
 *           5) 数据过期**必须清零数值**（不只是 valid=0），
 *              $RZ 还要清 byPos[]，否则消费方会拿陈旧数据动作（抓错料/对错位）；
 *           6) 换算只留一个内部入口 RPi_PxToBodyMm，整数运算且防除零；
 *           7) 日志禁 %f，像素与 mm 均为整数打印。
 ******************************************************************************
 */
#include "bsp_rpi.h"
#include "usart.h"          /* huart1，USART1 句柄 */
#include "bsp_debug.h"      /* Debug_Log 统一日志（UART5） */
#include "bsp_oled_ui.h"    /* Display_RPiRefresh 测试页置脏 */
#include <string.h>
#include <stdio.h>

/* ===================== 可调参数（本文件内部） ===================== */
/* 解析日志限流间隔，单位 ms；0 = 每帧都打。
   【为什么必须限流】Debug_Log 是逐字节阻塞式 UART5 输出（一行约 3~4ms）。
   数据帧多时若每帧都打日志，光串口就占用大量 CPU 并显著拖慢主循环。
   默认同一帧类型 500ms 最多一条，页面仍显示实时值。
   **不要随意调小**；做单帧联调时可临时改 0。 */
#ifndef RPI_LOG_PER_FRAME_MS
#define RPI_LOG_PER_FRAME_MS     500U
#endif

/* 解析日志限流通道（每类帧独立限流，避免互刷） */
typedef enum {
    RPI_LOG_CH_RCP = 0,   /* $RCP */
    RPI_LOG_CH_RZ,        /* $RZ */
    RPI_LOG_CH_CCG,       /* $CCG */
    RPI_LOG_CH_SHOT,      /* $SHOT */
    RPI_LOG_CH_CLAMP,     /* 数值钳位提示（各帧共用一条） */
    RPI_LOG_CH_MISC,      /* 杂项：超长行丢弃 / 心跳丢失 / 接收错误（各共用一条） */
    RPI_LOG_CH_COUNT
} RPi_LogCh_t;

/* ===================== 内部状态 ===================== */

/* USART1 单字节接收缓冲（HAL 写入地址）。非 static：bsp_imu.c 回调中 extern 引用。 */
uint8_t rpiRxByte = 0;

/* ---- 接收：中断写、主循环读的字节环形缓冲（按 0x0A 行定界）---- */
static uint8_t  rpiRxRing[RPI_RX_RING_SIZE]; /* 字节环形缓冲 */
static volatile uint16_t rpiRxHead = 0U;     /* 写入位置（**仅中断**修改） */
static volatile uint16_t rpiRxTail = 0U;     /* 读出位置（**仅主循环**修改） */
static uint32_t rpiRxDropCount = 0;          /* 环满丢弃字节数（主循环太慢/波特率不对时增长） */
static uint8_t  rpiRxDropLogged = 0;         /* 环满日志只报一次 */
static uint32_t rpiLineDropCount = 0;        /* 超长行丢弃条数（>RPI_MAX_LEN 仍无 '\n'） */
static char     rpiLine[RPI_MAX_LEN + 1U];   /* 行装配缓冲（主循环用） */
static uint8_t  rpiLineLen = 0U;             /* 当前行已装配字节数 */
static uint8_t  rpiLineOver = 0U;            /* 1=本行已超限，丢弃到下一个 '\n' */

/* ---- 状态 / 计数 ---- */
static uint8_t  rpiOnline = 0;              /* 线路级：收到过字节即 1（永不回落，仅供诊断） */
static uint8_t  rpiOfflineLogged = 0;       /* 上电 3s 无字节的日志只报一次 */
static uint8_t  rpiErrReported = 0;         /* 接收错误日志只报一次 */
static uint32_t rpiInitTick = 0;            /* 上电初始化时刻 */
static uint32_t rpiFrameCount = 0;          /* 有效帧累计计数（已知帧名且解析通过） */
static uint32_t rpiBadCount = 0;            /* 坏帧累计计数（校验失败/段数不足） */
static uint32_t rpiErrCount = 0;            /* USART1 接收错误累计（ORE/FE/NE/PE） */
static uint32_t rpiLogTick[RPI_LOG_CH_COUNT]; /* 日志限流时间戳 */
static char     rpiLastSummary[24];         /* 最近一帧摘要（供测试页/日志） */

/* ---- 心跳 $PING（协议 §4.5）---- */
static uint32_t     rpiPingTick = 0;                /* 最近一次 $PING 时刻（IsAlive 判据） */
static uint8_t      rpiPingSeen = 0;                /* 是否收到过 $PING */
static RPi_VsMode_t rpiVsMode   = RPI_VS_UNKNOWN;   /* 视觉端上报的状态（变化时打日志） */
static uint8_t      rpiHbLostLogged = 0;            /* 心跳丢失日志只报一次（恢复时清） */

/* ---- 数据（协议 §4）---- */
static RPi_Rcp_t  rpiRcp;                   /* $RCP 结果（转盘中心偏移） */
static RPi_Rz_t   rpiRz;                    /* $RZ 结果（三位号/颜色） */
static RPi_Ccg_t  rpiCcg;                   /* $CCG 结果（准备位颜色应答） */
static RPi_Shot_t rpiShot;                  /* $SHOT 结果（打靶区数据） */
static RPi_Shot_t rpiShot2;                 /* ★$SHOT2 结果（ROUGH STACK 用：中间物料颜色质心；与 rpiShot 独立） */
static RPi_Rp_t   rpiRp;                    /* ★$RP 结果（转盘圆盘平行度 q1，单位 0.1°） */
static RPi_Cfg_t  rpiCfg;                   /* 运行期换算配置（初值 = .h 宏） */
static uint8_t    rpiDenZeroWarned[2];      /* 分母为 0 的报警只报一次（下标 0=前进 1=右移） */

/* ===================== 内部函数声明 ===================== */
static uint8_t  RPi_HexVal(char c);
static uint8_t  RPi_FindStar(const char *buf, uint8_t len);
static uint8_t  RPi_ChecksumOk(const char *buf, uint8_t len, uint8_t starIdx);
static int32_t  RPi_ParseInt(const char *s, uint8_t *ok);
static uint8_t  RPi_LogAllowed(uint8_t ch);
static RPi_VsMode_t RPi_ParseVsMode(const char *s);
static int16_t  RPi_ClampI16(int32_t v, int32_t lo, int32_t hi, const char *lbl);
static int16_t  RPi_AxisToMm(int32_t px, uint8_t axis);
void            RPi_PxToBodyMm(int16_t dxPx, int16_t dyPx,
                               int16_t *outFwdMm, int16_t *outRightMm);
static uint8_t  RPi_ExpireRcp(uint32_t now);
static uint8_t  RPi_ExpireRz(uint32_t now);
static uint8_t  RPi_ExpireCcg(uint32_t now);
static uint8_t  RPi_ExpireShot(uint32_t now);
static uint8_t  RPi_ExpireShot2(uint32_t now);
static uint8_t  RPi_ExpireRp(uint32_t now);
static uint8_t  RPi_DoParse(char *line, uint8_t len);
static void     RPi_ProcessLines(void);
static void     RPi_DropLine(void);
static void     RPi_MarkBad(void);
static void     RPi_SendFrame(const char *content);

/* ===================== 内部函数 ===================== */

/**
 * @brief  十六进制字符 → 数值
 * @param  c: 字符
 * @retval 0~15；非法字符返回 0xFF
 */
static uint8_t RPi_HexVal(char c)
{
    if (c >= '0' && c <= '9') { return (uint8_t)(c - '0'); }
    if (c >= 'A' && c <= 'F') { return (uint8_t)(c - 'A' + 10); }
    if (c >= 'a' && c <= 'f') { return (uint8_t)(c - 'a' + 10); }
    return 0xFFU;
}

/**
 * @brief  查找帧尾 '*' 的下标
 * @param  buf: 帧缓冲（已去尾缀）
 * @param  len: 有效长度
 * @retval '*' 的下标；未找到返回 0xFF
 */
static uint8_t RPi_FindStar(const char *buf, uint8_t len)
{
    uint8_t i;
    for (i = 0U; i < len; i++)
    {
        if (buf[i] == '*') { return i; }
    }
    return 0xFFU;
}

/**
 * @brief  校验帧的 XOR 校验字段
 * @param  buf:     帧缓冲（已去尾缀）
 * @param  len:     有效长度
 * @param  starIdx: '*' 的下标
 * @retval 1=校验通过（或为 `**` 跳过校验）；0=不通过
 * @note   协议 §2：校验 = 对 '$' 之后、'*' 之前每个字符逐字节 XOR，
 *         写成 2 位大写十六进制；`**` 表示跳过校验。
 */
static uint8_t RPi_ChecksumOk(const char *buf, uint8_t len, uint8_t starIdx)
{
    uint8_t i;
    uint8_t x = 0U;
    uint8_t hi;
    uint8_t lo;

    /* '*' 之后紧跟 '*' → 跳过校验（串口助手调试用） */
    if (((uint16_t)starIdx + 1U) < (uint16_t)len && buf[starIdx + 1U] == '*')
    {
        return 1U;
    }

    /* 需要 2 位十六进制校验字段 */
    if (((uint16_t)starIdx + 2U) >= (uint16_t)len)
    {
        return 0U;
    }
    hi = RPi_HexVal(buf[starIdx + 1U]);
    lo = RPi_HexVal(buf[starIdx + 2U]);
    if (hi == 0xFFU || lo == 0xFFU)
    {
        return 0U;
    }

    /* 逐字节异或（跳过开头的 '$'） */
    for (i = 1U; i < starIdx; i++)
    {
        x ^= (uint8_t)buf[i];
    }

    return (x == (uint8_t)((hi << 4) | lo)) ? 1U : 0U;
}

/**
 * @brief  解析十进制整数（允许前导空白与正负号）
 * @param  s:  以 '\0' 结尾的字段串（分段后每段已单独截断）
 * @param  ok: 输出，1=解析成功 0=不是合法整数
 * @retval 解析出的数值（失败时返回 0）
 * @note   要求整段都是数字（可带符号与首尾空白），
 *         避免 "12abc" 这类脏数据被当作 12 接受。
 */
static int32_t RPi_ParseInt(const char *s, uint8_t *ok)
{
    int32_t v = 0;
    int8_t  sign = 1;
    uint8_t any = 0U;

    while (*s == ' ' || *s == '\t') { s++; }
    if (*s == '-') { sign = -1; s++; }
    else if (*s == '+') { s++; }

    while (*s >= '0' && *s <= '9')
    {
        v = (v * 10) + (int32_t)(*s - '0');
        if (v > 1000000) { v = 1000000; }   /* 防溢出；越界值随后会被钳位处理 */
        s++;
        any = 1U;
    }

    /* 尾部只允许空白，且必须到字符串结尾 */
    while (*s == ' ' || *s == '\t') { s++; }
    if ((any == 0U) || (*s != '\0'))
    {
        *ok = 0U;
        return 0;
    }
    *ok = 1U;
    return sign * v;
}

/**
 * @brief  日志限流判断（每类帧独立）
 * @param  ch: 限流通道（RPi_LogCh_t）
 * @retval 1=允许打印（并刷新该通道时间戳）；0=本拍被限流
 * @note   RPI_LOG_PER_FRAME_MS=0 时恒返回 1（每帧都打）。
 */
static uint8_t RPi_LogAllowed(uint8_t ch)
{
    uint32_t now = HAL_GetTick();

    if (ch >= (uint8_t)RPI_LOG_CH_COUNT) { return 0U; }
    if ((now - rpiLogTick[ch]) < RPI_LOG_PER_FRAME_MS) { return 0U; }
    rpiLogTick[ch] = now;
    return 1U;
}

/**
 * @brief  解析 $PING 的模式字段（协议 §4.5）
 * @param  s: 字段串（"IDLE" / "TURN" / "SHOT"）
 * @retval 对应枚举；不识别返回 RPI_VS_UNKNOWN
 */
static RPi_VsMode_t RPi_ParseVsMode(const char *s)
{
    if (strcmp(s, "IDLE") == 0) { return RPI_VS_IDLE; }
    if (strcmp(s, "TURN") == 0) { return RPI_VS_TURN; }
    if (strcmp(s, "SHOT") == 0) { return RPI_VS_SHOT; }
    return RPI_VS_UNKNOWN;
}

/**
 * @brief  把 int32 钳位到 [lo, hi]（协议 §8：数值越界钳位并记录，不丢整帧）
 * @param  v:   原值
 * @param  lo:  下界
 * @param  hi:  上界
 * @param  lbl: 日志标签（用于定位是哪一帧哪一字段）
 * @retval 钳位后的 int16
 */
static int16_t RPi_ClampI16(int32_t v, int32_t lo, int32_t hi, const char *lbl)
{
    int32_t c = v;

    if (c > hi) { c = hi; }
    if (c < lo) { c = lo; }

    if ((c != v) && (RPi_LogAllowed((uint8_t)RPI_LOG_CH_CLAMP) != 0U))
    {
        Debug_Log("RPi", "%s clamp %ld -> %ld", lbl, (long)v, (long)c);
    }
    return (int16_t)c;
}

/**
 * @brief  单个像素量 → 车体毫米（整数运算，防除零）
 * @param  px:   已做轴映射与符号处理后的像素值
 * @param  axis: 0=前进方向 1=右移方向（决定用哪组 px2mm 系数）
 * @retval 换算后的毫米值
 * @note   分母为 0 时该轴输出 0 并报警一次（在线标定时误填 0 不至于死机）。
 *         系数是运行期可改的，极端取值可能超出 int16，故做 ±30000 钳位。
 */
static int16_t RPi_AxisToMm(int32_t px, uint8_t axis)
{
    uint32_t num;
    uint32_t den;
    int32_t  mm;

    if (axis > 1U) { axis = 1U; }
    num = (uint32_t)rpiCfg.px2mmNum[axis];
    den = (uint32_t)rpiCfg.px2mmDen[axis];

    if (den == 0U)
    {
        if (rpiDenZeroWarned[axis] == 0U)
        {
            rpiDenZeroWarned[axis] = 1U;
            Debug_Log("RPi", "px2mm den[%u]=0 -> axis mm forced 0", (unsigned)axis);
        }
        return 0;
    }

    mm = (int32_t)((px * (int32_t)num) / (int32_t)den);

    if (mm > 30000)  { mm = 30000; }
    if (mm < -30000) { mm = -30000; }
    return (int16_t)mm;
}

/**
 * @brief  图像像素偏移 → 车体毫米（**模块内唯一换算入口**，供 $RCP/$SHOT 共用）
 * @param  dxPx:       图像 x 方向偏移（相对画面中心，右正），px
 * @param  dyPx:       图像 y 方向偏移（相对画面中心，下正），px
 * @param  outFwdMm:   输出，车体前进方向修正量 mm（可为 NULL）
 * @param  outRightMm: 输出，车体右移方向修正量 mm（可为 NULL）
 * @retval 无
 * @note   依次完成：锚点补偿 → 钳位 → 轴映射 → 符号 → 比例换算（整数，防除零）。
 *         锚点补偿用运行期画布中心与标定锚点之差，吸收"镜头光轴 ≠ 机械中心"的固定偏置。
 *         任务层一般用 `RPi_GetRcp()/GetShot()` 里的 dxMm/dyMm（解析时已算好）；
 *         需要**分轴**时（bsp_align 的 X/Y 分轴闭环）才直接调本函数、另一轴传 0。
 */
void RPi_PxToBodyMm(int16_t dxPx, int16_t dyPx,
                    int16_t *outFwdMm, int16_t *outRightMm)
{
    int32_t errX;
    int32_t errY;
    int32_t pxFwd = 0;
    int32_t pxRight = 0;

    /* 1) 锚点补偿：偏差 = 目标（中心 + dx）- 锚点 = dx + (中心 - 锚点) */
    errX = (int32_t)dxPx + ((int32_t)RPI_IMG_CENTER_X - (int32_t)rpiCfg.anchorX);
    errY = (int32_t)dyPx + ((int32_t)RPI_IMG_CENTER_Y - (int32_t)rpiCfg.anchorY);

    /* 2) 钳位：视觉偶尔乱跳，限制单轴像素量级 */
    if (errX >  (int32_t)rpiCfg.maxPx) { errX =  (int32_t)rpiCfg.maxPx; }
    if (errX < -(int32_t)rpiCfg.maxPx) { errX = -(int32_t)rpiCfg.maxPx; }
    if (errY >  (int32_t)rpiCfg.maxPx) { errY =  (int32_t)rpiCfg.maxPx; }
    if (errY < -(int32_t)rpiCfg.maxPx) { errY = -(int32_t)rpiCfg.maxPx; }

    /* 3) 轴映射 + 符号：把每个图像轴的分量累加到它对应的车体轴 */
    if (rpiCfg.imgXToBody == 0U) { pxFwd   += (errX * (int32_t)rpiCfg.imgXSign); }
    else                         { pxRight += (errX * (int32_t)rpiCfg.imgXSign); }

    if (rpiCfg.imgYToBody == 0U) { pxFwd   += (errY * (int32_t)rpiCfg.imgYSign); }
    else                         { pxRight += (errY * (int32_t)rpiCfg.imgYSign); }

    /* 4) 比例换算（整数运算，防除零在 RPi_AxisToMm 内处理） */
    if (outFwdMm   != NULL) { *outFwdMm   = RPi_AxisToMm(pxFwd,   0U); }
    if (outRightMm != NULL) { *outRightMm = RPi_AxisToMm(pxRight, 1U); }
}

/**
 * @brief  $RCP 过期检查并清零数值（主循环调用）
 * @param  now: 当前时间戳(ms)
 * @retval 1=本次发生过期（需刷新页面）；0=未过期或已无效
 */
static uint8_t RPi_ExpireRcp(uint32_t now)
{
    if (rpiRcp.valid == 0U) { return 0U; }
    if ((now - rpiRcp.lastRxTick) <= RPI_DATA_EXPIRE_MS) { return 0U; }

    rpiRcp.valid = 0U;
    rpiRcp.dxPx  = 0;
    rpiRcp.dyPx  = 0;
    rpiRcp.dxMm  = 0;
    rpiRcp.dyMm  = 0;
    return 1U;
}

/**
 * @brief  $RZ 过期检查并清零数值（含 byPos[]，主循环调用）
 * @param  now: 当前时间戳(ms)
 * @retval 1=本次发生过期（需刷新页面）；0=未过期或已无效
 * @note   ★ byPos[] 是"位号→颜色"查表，过期后若残留旧值，任务层会按旧颜色抓错料，
 *         故必须一并清零。
 */
static uint8_t RPi_ExpireRz(uint32_t now)
{
    if (rpiRz.valid == 0U) { return 0U; }
    if ((now - rpiRz.lastRxTick) <= RPI_DATA_EXPIRE_MS) { return 0U; }

    rpiRz.valid = 0U;
    rpiRz.pos[0] = 0U; rpiRz.pos[1] = 0U; rpiRz.pos[2] = 0U;
    rpiRz.color[0] = 0U; rpiRz.color[1] = 0U; rpiRz.color[2] = 0U;
    rpiRz.byPos[0] = 0U; rpiRz.byPos[1] = 0U;
    rpiRz.byPos[2] = 0U; rpiRz.byPos[3] = 0U;
    return 1U;
}

/**
 * @brief  $CCG 过期检查并清零数值（主循环调用）
 * @param  now: 当前时间戳(ms)
 * @retval 1=本次发生过期（需刷新页面）；0=未过期或已无效
 */
static uint8_t RPi_ExpireCcg(uint32_t now)
{
    if (rpiCcg.valid == 0U) { return 0U; }
    if ((now - rpiCcg.lastRxTick) <= RPI_DATA_EXPIRE_MS) { return 0U; }

    rpiCcg.valid  = 0U;
    rpiCcg.reqPos = 0U;
    rpiCcg.color  = 0U;
    return 1U;
}

/**
 * @brief  $SHOT 过期检查并清零数值（主循环调用）
 * @param  now: 当前时间戳(ms)
 * @retval 1=本次发生过期（需刷新页面）；0=未过期或已无效
 */
static uint8_t RPi_ExpireShot(uint32_t now)
{
    if (rpiShot.valid == 0U) { return 0U; }
    if ((now - rpiShot.lastRxTick) <= RPI_DATA_EXPIRE_MS) { return 0U; }

    rpiShot.valid = 0U;
    rpiShot.q[0]  = 0; rpiShot.q[1] = 0; rpiShot.q[2] = 0;
    rpiShot.dxPx  = 0; rpiShot.dyPx = 0;
    rpiShot.dxMm  = 0; rpiShot.dyMm = 0;
    return 1U;
}

/**
 * @brief  ★$SHOT2 过期检查并清零数值（主循环调用，与 RPi_ExpireShot 同口径）
 * @param  now: 当前时间戳(ms)
 * @retval 1=本次发生过期（需刷新页面）；0=未过期或已无效
 */
static uint8_t RPi_ExpireShot2(uint32_t now)
{
    if (rpiShot2.valid == 0U) { return 0U; }
    if ((now - rpiShot2.lastRxTick) <= RPI_DATA_EXPIRE_MS) { return 0U; }

    rpiShot2.valid = 0U;
    rpiShot2.q[0]  = 0; rpiShot2.q[1] = 0; rpiShot2.q[2] = 0;
    rpiShot2.dxPx  = 0; rpiShot2.dyPx = 0;
    rpiShot2.dxMm  = 0; rpiShot2.dyMm = 0;
    return 1U;
}

/**
 * @brief  ★$RP 过期检查并清零数值（主循环调用，与 RPi_ExpireShot 同口径）
 * @param  now: 当前时间戳(ms)
 * @retval 1=本次发生过期（需刷新页面）；0=未过期或已无效
 */
static uint8_t RPi_ExpireRp(uint32_t now)
{
    if (rpiRp.valid == 0U) { return 0U; }
    if ((now - rpiRp.lastRxTick) <= RPI_DATA_EXPIRE_MS) { return 0U; }

    rpiRp.valid = 0U;
    rpiRp.q1    = 0;
    return 1U;
}

/**
 * @brief  计入一个坏帧（校验失败 / 段数不足）
 * @param  无
 * @retval 无
 */
static void RPi_MarkBad(void)
{
    rpiBadCount++;
    (void)snprintf(rpiLastSummary, sizeof(rpiLastSummary), "BAD");
}

/**
 * @brief  组帧并发送一条指令（阻塞式，带超时保护）
 * @param  content: 帧体（不含 '$' 与 '*校验\r\n'），如 "RCP" / "CCG,3"
 * @retval 无
 * @note   校验 = content 逐字节 XOR。帧短（≤32 字节）+ RPI_TX_TIMEOUT_MS 超时保护，
 *         不会长期阻塞主循环。发送后打一条日志便于与串口助手核对。
 */
static void RPi_SendFrame(const char *content)
{
    char     buf[RPI_MAX_LEN + 8U];
    uint8_t  x = 0U;
    int      n;
    const char *p;

    for (p = content; *p != '\0'; p++)
    {
        x ^= (uint8_t)(*p);
    }

    n = snprintf(buf, sizeof(buf), "$%s*%02X\r\n", content, (unsigned)x);
    if ((n <= 0) || ((size_t)n >= sizeof(buf)))
    {
        Debug_Log("RPi", "tx too long");
        return;
    }

    (void)HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)n, RPI_TX_TIMEOUT_MS);
    Debug_Log("RPi", "tx %s", content);
}

/**
 * @brief  解析一行报文（主循环上下文，内部实现）
 * @param  line: 一行报文（已完成行定界、无 CR/LF，需以 '\0' 结尾，可原地修改）
 * @param  len:  行有效长度
 * @retval 0=坏帧（帧结构非法，或已识别帧名但段数不足/数值非法）
 *         1=已识别帧名且解析成功
 *         2=未知帧名 / 未识别模式串（静默忽略，不计任何计数）
 * @note   协议 v2.0 入站类型：RCP / RZ / CCG / SHOT / PING；
 *         其余一律"丢弃、不报错、不改状态"，也不计坏帧。
 */
static uint8_t RPi_DoParse(char *line, uint8_t len)
{
    uint8_t  starIdx;
    uint8_t  total = 0U;         /* 实际段数（TYPE + 字段） */
    /* field[] 必须装得下"类型 + 最多 6 个字段"（$RZ 有 6 个字段） */
    char    *field[8] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
    char    *p;
    int32_t  v;
    uint8_t  ok;

    /* ---- 0) 防御性收尾：剥掉可能残留的 CR/LF/空白（正常路径取行时已滤掉） ---- */
    if (len >= RPI_MAX_LEN) { len = (uint8_t)(RPI_MAX_LEN - 1U); }
    line[len] = '\0';
    while (len > 0U)
    {
        char c = line[len - 1U];
        if (c == '\r' || c == '\n' || c == ' ' || c == '\t')
        {
            len--;
            line[len] = '\0';
        }
        else
        {
            break;
        }
    }

    /* ---- 1) 帧头与最短长度 ---- */
    if ((len < 4U) || (line[0] != '$'))
    {
        Debug_Log("RPi", "bad frame head");
        return 0U;
    }

    /* ---- 2) 找 '*' ---- */
    starIdx = RPi_FindStar(line, len);
    if (starIdx == 0xFFU)
    {
        Debug_Log("RPi", "bad frame no star");
        return 0U;
    }

    /* ---- 3) 校验（`**` 跳过） ---- */
    if (RPi_ChecksumOk(line, len, starIdx) == 0U)
    {
        Debug_Log("RPi", "bad frame cs");
        return 0U;
    }

    /* ---- 4) 在 '*' 处截断，帧体从 line[1] 开始 ---- */
    line[starIdx] = '\0';

    /* ---- 5) 按 ',' 分段（多余段只计数，向前兼容） ---- */
    p = &line[1];
    field[0] = p;
    total = 1U;
    while (*p != '\0')
    {
        if (*p == ',')
        {
            *p = '\0';
            if (total < 8U)
            {
                field[total] = p + 1;
            }
            total++;
        }
        p++;
    }

    /* ---- 6) 帧名分发 ---- */

    /* $RCP,dx,dy：转盘中心偏移（段数 ≥ 3，取前 3） */
    if (strcmp(field[0], "RCP") == 0)
    {
        int16_t fwd;
        int16_t right;

        if (total < 3U)
        {
            Debug_Log("RPi", "RCP bad fields");
            return 0U;
        }
        v = RPi_ParseInt(field[1], &ok);
        if (ok == 0U) { Debug_Log("RPi", "RCP bad dx"); return 0U; }
        rpiRcp.dxPx = RPi_ClampI16(v, -(int32_t)RPI_PX_MAX, (int32_t)RPI_PX_MAX, "RCP dx");

        v = RPi_ParseInt(field[2], &ok);
        if (ok == 0U) { Debug_Log("RPi", "RCP bad dy"); return 0U; }
        rpiRcp.dyPx = RPi_ClampI16(v, -(int32_t)RPI_PX_MAX, (int32_t)RPI_PX_MAX, "RCP dy");

        /* ★ (0,0) = 树莓派"**未识别到转盘**"的哨兵值（协议 §4.1；2026-10-10 视觉端确认）。
           此时本帧**不算有效数据**（valid=0）—— 否则对位会把"没找到"当成
           "偏差已经是 0"，直接误判成"已对准"。
           注意："真正对准"在本系统里由死区判据（|偏差| ≤ 死区 px）认定，不依赖 (0,0)。 */
        if ((rpiRcp.dxPx == 0) && (rpiRcp.dyPx == 0))
        {
            rpiRcp.dxMm  = 0;
            rpiRcp.dyMm  = 0;
            rpiRcp.valid = 0U;
            rpiRcp.lastRxTick = HAL_GetTick();
            (void)snprintf(rpiLastSummary, sizeof(rpiLastSummary), "RCP 0,0 notfound");
            Debug_Log("RPi", "RCP 0,0 = turntable not found -> invalid");
            return 1U;
        }

        /* 随即换算：px 与 mm 在同一帧内算好，天然一致；任务层无需自己乘系数 */
        RPi_PxToBodyMm(rpiRcp.dxPx, rpiRcp.dyPx, &fwd, &right);
        rpiRcp.dxMm = fwd;
        rpiRcp.dyMm = right;
        rpiRcp.valid = 1U;
        rpiRcp.lastRxTick = HAL_GetTick();

        (void)snprintf(rpiLastSummary, sizeof(rpiLastSummary), "RCP %d,%d",
                       (int)rpiRcp.dxPx, (int)rpiRcp.dyPx);
        if (RPi_LogAllowed((uint8_t)RPI_LOG_CH_RCP) != 0U)
        {
            Debug_Log("RPi", "RCP px=%d,%d mm=%d,%d",
                      (int)rpiRcp.dxPx, (int)rpiRcp.dyPx,
                      (int)rpiRcp.dxMm, (int)rpiRcp.dyMm);
        }
        return 1U;
    }

    /* $RZ,p1,c1,p2,c2,p3,c3：转盘三位号/颜色（段数 ≥ 7，取前 7） */
    if (strcmp(field[0], "RZ") == 0)
    {
        int32_t p1, c1, p2, c2, p3, c3;
        uint8_t seen[4] = {0U, 0U, 0U, 0U};

        if (total < 7U)
        {
            Debug_Log("RPi", "RZ bad fields");
            return 0U;
        }
        p1 = RPi_ParseInt(field[1], &ok); if (ok == 0U) { Debug_Log("RPi", "RZ bad p1"); return 0U; }
        c1 = RPi_ParseInt(field[2], &ok); if (ok == 0U) { Debug_Log("RPi", "RZ bad c1"); return 0U; }
        p2 = RPi_ParseInt(field[3], &ok); if (ok == 0U) { Debug_Log("RPi", "RZ bad p2"); return 0U; }
        c2 = RPi_ParseInt(field[4], &ok); if (ok == 0U) { Debug_Log("RPi", "RZ bad c2"); return 0U; }
        p3 = RPi_ParseInt(field[5], &ok); if (ok == 0U) { Debug_Log("RPi", "RZ bad p3"); return 0U; }
        c3 = RPi_ParseInt(field[6], &ok); if (ok == 0U) { Debug_Log("RPi", "RZ bad c3"); return 0U; }

        /* ★ 位号必须是 1/2/3 的一个排列：越界/重复/缺项 → 坏帧（协议 §4.2） */
        if ((p1 < 1) || (p1 > 3) || (p2 < 1) || (p2 > 3) || (p3 < 1) || (p3 > 3))
        {
            Debug_Log("RPi", "RZ pos out of range");
            return 0U;
        }
        if ((seen[p1] != 0U) || (seen[p2] != 0U) || (seen[p3] != 0U))
        {
            Debug_Log("RPi", "RZ pos duplicated");
            return 0U;
        }
        seen[p1] = 1U; seen[p2] = 1U; seen[p3] = 1U;

        rpiRz.pos[0] = (uint8_t)p1;
        rpiRz.pos[1] = (uint8_t)p2;
        rpiRz.pos[2] = (uint8_t)p3;
        /* 颜色合法范围 0~4（0=未识别，1红 2黄 3蓝 4绿）；越界钳位（不丢整帧） */
        rpiRz.color[0] = (uint8_t)RPi_ClampI16(c1, 0, 4, "RZ c1");
        rpiRz.color[1] = (uint8_t)RPi_ClampI16(c2, 0, 4, "RZ c2");
        rpiRz.color[2] = (uint8_t)RPi_ClampI16(c3, 0, 4, "RZ c3");

        /* 建"位号→颜色"查表 */
        rpiRz.byPos[0] = 0U;
        rpiRz.byPos[1] = 0U;
        rpiRz.byPos[2] = 0U;
        rpiRz.byPos[3] = 0U;
        rpiRz.byPos[rpiRz.pos[0]] = rpiRz.color[0];
        rpiRz.byPos[rpiRz.pos[1]] = rpiRz.color[1];
        rpiRz.byPos[rpiRz.pos[2]] = rpiRz.color[2];

        rpiRz.valid = 1U;
        rpiRz.lastRxTick = HAL_GetTick();

        (void)snprintf(rpiLastSummary, sizeof(rpiLastSummary), "RZ %u,%u,%u",
                       (unsigned)rpiRz.byPos[1], (unsigned)rpiRz.byPos[2],
                       (unsigned)rpiRz.byPos[3]);
        if (RPi_LogAllowed((uint8_t)RPI_LOG_CH_RZ) != 0U)
        {
            Debug_Log("RPi", "RZ p1=%u c1=%u p2=%u c2=%u p3=%u c3=%u",
                      (unsigned)rpiRz.pos[0], (unsigned)rpiRz.color[0],
                      (unsigned)rpiRz.pos[1], (unsigned)rpiRz.color[1],
                      (unsigned)rpiRz.pos[2], (unsigned)rpiRz.color[2]);
        }
        return 1U;
    }

    /* $CCG,c：当前准备位的颜色（段数 ≥ 2） */
    if (strcmp(field[0], "CCG") == 0)
    {
        if (total < 2U)
        {
            Debug_Log("RPi", "CCG bad fields");
            return 0U;
        }
        v = RPi_ParseInt(field[1], &ok);
        if (ok == 0U) { Debug_Log("RPi", "CCG bad c"); return 0U; }
        rpiCcg.color = (uint8_t)RPi_ClampI16(v, 0, 4, "CCG c");
        rpiCcg.valid = 1U;
        rpiCcg.lastRxTick = HAL_GetTick();

        (void)snprintf(rpiLastSummary, sizeof(rpiLastSummary), "CCG %u",
                       (unsigned)rpiCcg.color);
        if (RPi_LogAllowed((uint8_t)RPI_LOG_CH_CCG) != 0U)
        {
            Debug_Log("RPi", "CCG req=%u c=%u",
                      (unsigned)rpiCcg.reqPos, (unsigned)rpiCcg.color);
        }
        return 1U;
    }

    /* $RP,q1：★转盘圆盘平行度（新增协议《新增协议_RP.md》，2026-10-15）
       - q1 单位 **0.1°**（(实测角度 − 零点)×10），正 = 顺时针；
       - 合法范围 **[-900, 900]**，越界按钳位处理（不丢整帧，与其它帧口径一致）；
       - ⚠ **q1=0 可能是"真对齐"也可能是"任一侧没检到红线"**，视觉端不给有效标志，
         所以这里统一记 valid=1；"0 可不可信"由 bsp_align 按"本次对位是否见过非 0"
         判断（见 bsp_align.c 的 alRpSeenNz）。 */
    if (strcmp(field[0], "RP") == 0)
    {
        if (total < 2U)
        {
            Debug_Log("RPi", "RP bad fields");
            return 0U;
        }
        v = RPi_ParseInt(field[1], &ok);
        if (ok == 0U)
        {
            Debug_Log("RPi", "RP bad q1");
            return 0U;
        }

        rpiRp.q1         = RPi_ClampI16(v, -900, 900, "RP q1");
        rpiRp.valid      = 1U;
        rpiRp.lastRxTick = HAL_GetTick();

        (void)snprintf(rpiLastSummary, sizeof(rpiLastSummary), "RP q=%d", (int)rpiRp.q1);
        if (RPi_LogAllowed((uint8_t)RPI_LOG_CH_SHOT) != 0U)
        {
            Debug_Log("RPi", "RP q=%d", (int)rpiRp.q1);
        }
        return 1U;
    }

    /* $SHOT,q1,q2,q3,dx,dy ：打靶区数据（段数 ≥ 6，取前 6）
       ★$SHOT2,q1,q2,q3,dx,dy ：**ROUGH STACK 用**（视觉组《SHOT2通讯对接.md》）——
         字段定义**完全一样**，但**测量目标不同**：
           `$SHOT`  瞄"中间**圆环**圆心"（色环靶位）；
           `$SHOT2` 瞄"三个横排物料里**中间那个的颜色质心**"。
         所以两套数据**各自独立存放**（rpiShot / rpiShot2），**互不通用**。 */
    if ((strcmp(field[0], "SHOT") == 0) || (strcmp(field[0], "SHOT2") == 0))
    {
        RPi_Shot_t *dst = (field[0][4] == '2') ? &rpiShot2 : &rpiShot;
        const char  *tag = (field[0][4] == '2') ? "SHOT2" : "SHOT";
        int32_t q1, q2, q3;
        int16_t fwd;
        int16_t right;

        if (total < 6U)
        {
            Debug_Log("RPi", "%s bad fields", tag);
            return 0U;
        }
        q1 = RPi_ParseInt(field[1], &ok); if (ok == 0U) { Debug_Log("RPi", "%s bad q1", tag); return 0U; }
        q2 = RPi_ParseInt(field[2], &ok); if (ok == 0U) { Debug_Log("RPi", "%s bad q2", tag); return 0U; }
        q3 = RPi_ParseInt(field[3], &ok); if (ok == 0U) { Debug_Log("RPi", "%s bad q3", tag); return 0U; }
        v  = RPi_ParseInt(field[4], &ok); if (ok == 0U) { Debug_Log("RPi", "%s bad dx", tag); return 0U; }
        dst->dxPx = RPi_ClampI16(v, -(int32_t)RPI_PX_MAX, (int32_t)RPI_PX_MAX, tag);
        v  = RPi_ParseInt(field[5], &ok); if (ok == 0U) { Debug_Log("RPi", "%s bad dy", tag); return 0U; }
        dst->dyPx = RPi_ClampI16(v, -(int32_t)RPI_PX_MAX, (int32_t)RPI_PX_MAX, tag);

        dst->q[0] = RPi_ClampI16(q1, -(int32_t)RPI_PX_MAX, (int32_t)RPI_PX_MAX, tag);
        dst->q[1] = RPi_ClampI16(q2, -(int32_t)RPI_PX_MAX, (int32_t)RPI_PX_MAX, tag);
        dst->q[2] = RPi_ClampI16(q3, -(int32_t)RPI_PX_MAX, (int32_t)RPI_PX_MAX, tag);

        /* ★ (dx,dy)=(0,0) = 树莓派"**未识别到目标**"的哨兵值
           （协议 §4.4；样例 `$SHOT,0,0,0,0,0` / `$SHOT2,0,0,0,0,0`）。
           同样**不算有效数据**，避免把"没找到"当成"已对准"。 */
        if ((dst->dxPx == 0) && (dst->dyPx == 0))
        {
            dst->dxMm  = 0;
            dst->dyMm  = 0;
            dst->valid = 0U;
            dst->lastRxTick = HAL_GetTick();
            (void)snprintf(rpiLastSummary, sizeof(rpiLastSummary), "%s 0,0 notfound", tag);
            Debug_Log("RPi", "%s 0,0 = target not found -> invalid", tag);
            return 1U;
        }

        RPi_PxToBodyMm(dst->dxPx, dst->dyPx, &fwd, &right);
        dst->dxMm = fwd;
        dst->dyMm = right;
        dst->valid = 1U;
        dst->lastRxTick = HAL_GetTick();

        (void)snprintf(rpiLastSummary, sizeof(rpiLastSummary), "%s %d,%d",
                       tag, (int)dst->dxPx, (int)dst->dyPx);
        if (RPi_LogAllowed((uint8_t)RPI_LOG_CH_SHOT) != 0U)
        {
            Debug_Log("RPi", "%s q=%d,%d,%d px=%d,%d mm=%d,%d",
                      tag, (int)dst->q[0], (int)dst->q[1], (int)dst->q[2],
                      (int)dst->dxPx, (int)dst->dyPx,
                      (int)dst->dxMm, (int)dst->dyMm);
        }
        return 1U;
    }

    /* $PING,<MODE>：心跳 1 Hz。刷新"视觉端在线"判据与上报状态，不产生数据。
       MODE ∈ {IDLE, TURN, SHOT}；未知模式串忽略（不改状态、不计坏帧）。 */
    if (strcmp(field[0], "PING") == 0)
    {
        RPi_VsMode_t m;

        if (total < 2U)
        {
            Debug_Log("RPi", "PING bad fields");
            return 0U;
        }
        m = RPi_ParseVsMode(field[1]);

        /* 只要收到结构合法的 $PING 就刷新心跳（"收到过 $PING"即在线） */
        rpiPingTick     = HAL_GetTick();
        rpiPingSeen     = 1U;
        rpiHbLostLogged = 0U;              /* 心跳恢复，允许下次再报一次丢失 */

        if (m == RPI_VS_UNKNOWN)
        {
            /* 未知模式串：静默忽略（不改状态、不计坏帧） */
            return 2U;
        }

        if (m != rpiVsMode)
        {
            rpiVsMode = m;
            Debug_Log("RPi", "ping mode=%s", field[1]);
        }

        (void)snprintf(rpiLastSummary, sizeof(rpiLastSummary), "PING %s", field[1]);
        return 1U;
    }

    /* 未知帧名：静默忽略，**不动任何计数**（协议 §8 向前兼容）。
       若确需定位"帧名是否拼错"，把 RPI_LOG_UNKNOWN_FRAME 置 1 可临时打出帧名。 */
#if (RPI_LOG_UNKNOWN_FRAME != 0)
    Debug_Log("RPi", "unknown type=%s", field[0]);
#endif
    return 2U;
}

/**
 * @brief  记一次"整行丢弃"（超长行）并限流打日志
 * @param  无
 * @retval 无
 */
static void RPi_DropLine(void)
{
    rpiLineDropCount++;
    rpiLineOver = 0U;
    rpiLineLen  = 0U;

    if (RPi_LogAllowed((uint8_t)RPI_LOG_CH_MISC) != 0U)
    {
        Debug_Log("RPi", "line too long, drop total=%lu",
                  (unsigned long)rpiLineDropCount);
    }
}

/**
 * @brief  行定界 + 逐行解析（主循环上下文调用）
 * @param  无
 * @retval 无
 * @note   协议 v2.0 行尾为 \r\n。本函数把环形缓冲里**所有**完整行一次抽干、逐行解析；
 *         行内遇 '\r' 直接丢弃；一行超过 RPI_MAX_LEN 仍不见 '\n' 则整行丢弃并计数。
 *         解析结果落到内部结构体；有效帧计数与页面置脏在此统一处理。
 */
static void RPi_ProcessLines(void)
{
    uint8_t b;

    while (rpiRxTail != rpiRxHead)
    {
        b = rpiRxRing[rpiRxTail];
        rpiRxTail = (uint16_t)((rpiRxTail + 1U) % RPI_RX_RING_SIZE);

        if (b == '\r')
        {
            continue;                       /* 行尾 CR 丢弃（协议行尾 \r\n） */
        }

        if (b == '\n')
        {
            if (rpiLineOver != 0U)
            {
                RPi_DropLine();             /* 超长行：整行丢弃并计数 */
            }
            else if (rpiLineLen > 0U)
            {
                uint8_t res;

                rpiLine[rpiLineLen] = '\0';
                res = RPi_DoParse(rpiLine, rpiLineLen);
                if (res == 1U)
                {
                    rpiFrameCount++;
                    Display_RPiRefresh();
                }
                else if (res == 0U)
                {
                    RPi_MarkBad();
                    Display_RPiRefresh();
                }
                /* res == 2：未知帧名 / 未识别模式，静默忽略，不动任何计数 */
            }
            /* 空行（连续 '\n'）直接跳过 */

            rpiLineLen  = 0U;
            rpiLineOver = 0U;
            continue;
        }

        if (rpiLineOver != 0U)
        {
            /* 超长行还没等到行尾，又出现了帧头 → 上一行是垃圾：就地放弃并计数、
               从这个 '$' 重新开始，这样紧随其后的正常帧不会被一起吃掉。 */
            if (b == '$')
            {
                RPi_DropLine();
                rpiLine[0]  = '$';
                rpiLineLen  = 1U;
            }
            continue;
        }

        if (rpiLineLen >= RPI_MAX_LEN)
        {
            rpiLineOver = 1U;               /* 超限：本行改判为丢弃 */
            continue;
        }

        rpiLine[rpiLineLen] = (char)b;
        rpiLineLen++;
    }
}

/* ===================== 对外接口 ===================== */

/**
 * @brief  树莓派通讯模块初始化
 * @param  无
 * @retval 无
 * @note   复位状态与配置，启动 USART1 单字节中断接收；**不发任何指令**。
 */
void RPi_Init(void)
{
    HAL_StatusTypeDef st;
    uint8_t i;

    rpiRxHead        = 0U;
    rpiRxTail        = 0U;
    rpiRxDropCount   = 0U;
    rpiRxDropLogged  = 0U;
    rpiLineLen       = 0U;
    rpiLineOver      = 0U;
    rpiLineDropCount = 0U;
    rpiOnline        = 0U;
    rpiOfflineLogged = 0U;
    rpiErrReported   = 0U;
    rpiFrameCount    = 0U;
    rpiBadCount      = 0U;
    rpiErrCount      = 0U;
    rpiLastSummary[0] = '\0';

    /* 心跳 / 视觉端状态 */
    rpiPingTick     = 0U;
    rpiPingSeen     = 0U;
    rpiVsMode       = RPI_VS_UNKNOWN;
    rpiHbLostLogged = 0U;

    memset(&rpiRcp,  0, sizeof(rpiRcp));
    memset(&rpiRz,   0, sizeof(rpiRz));
    memset(&rpiCcg,  0, sizeof(rpiCcg));
    memset(&rpiShot, 0, sizeof(rpiShot));
    memset(&rpiShot2, 0, sizeof(rpiShot2));
    memset(&rpiRp,   0, sizeof(rpiRp));

    RPi_CfgReset();

    rpiInitTick = HAL_GetTick();
    /* 让每类帧的第一条日志立刻可打（减去一个限流间隔） */
    for (i = 0U; i < (uint8_t)RPI_LOG_CH_COUNT; i++)
    {
        rpiLogTick[i] = rpiInitTick - (uint32_t)RPI_LOG_PER_FRAME_MS;
    }

    /* 启动 USART1 单字节中断接收（回调在 bsp_imu.c HAL_UART_RxCpltCallback）。 */
    st = HAL_UART_Receive_IT(&huart1, &rpiRxByte, 1U);
    if (st != HAL_OK)
    {
        Debug_Log("RPi", "init FAIL rx_it=%d", (int)st);
    }
    else
    {
        Debug_Log("RPi", "init v2.0, baud=115200 rx_it=ok");
    }
}

/**
 * @brief  通讯主循环节拍（非阻塞）
 * @param  无
 * @retval 无
 * @note   行定界解析 + 数据过期清零 + 心跳离线判定 + 接收错误/环溢出首次上报。
 */
void RPi_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* 1) 行定界：把环形缓冲里的完整行全部取出解析 */
    RPi_ProcessLines();

    /* 2) 各数据帧过期：valid=0 且数值全部清零（$RZ 含 byPos[]） */
    if (RPi_ExpireRcp(now)  != 0U) { Display_RPiRefresh(); }
    if (RPi_ExpireRz(now)   != 0U) { Display_RPiRefresh(); }
    if (RPi_ExpireCcg(now)  != 0U) { Display_RPiRefresh(); }
    if (RPi_ExpireShot(now) != 0U) { Display_RPiRefresh(); }
    if (RPi_ExpireShot2(now) != 0U) { Display_RPiRefresh(); }
    if (RPi_ExpireRp(now)    != 0U) { Display_RPiRefresh(); }

    /* 3) 心跳超时（与数据超时**独立**）：$PING 断 → 视觉端离线。
          只置日志、**不清数据、不停车** —— 怎么降级由上层安全策略决定。 */
    if ((rpiPingSeen != 0U) && ((now - rpiPingTick) > RPI_HB_TIMEOUT_MS))
    {
        if (rpiHbLostLogged == 0U)
        {
            rpiHbLostLogged = 1U;      /* 恢复时在 $PING 分支清零 */
            Debug_Log("RPi", "heartbeat lost (>%u ms)", (unsigned)RPI_HB_TIMEOUT_MS);
            Display_RPiRefresh();      /* 在线状态行需刷新 */
        }
    }

    /* 4) 上电起一直没收到任何字节 → 打一条 offline（**仅日志**，不是在线判据） */
    if ((rpiOnline == 0U) && (rpiOfflineLogged == 0U) &&
        ((now - rpiInitTick) >= RPI_OFFLINE_TIMEOUT_MS))
    {
        rpiOfflineLogged = 1U;
        Debug_Log("RPi", "offline (no byte since boot)");
    }

    /* 5) 接收错误（ORE/FE/NE/PE）首次出现时报告一次。 */
    if ((rpiErrCount != 0U) && (rpiErrReported == 0U))
    {
        rpiErrReported = 1U;
        Debug_Log("RPi", "rx err seen, count=%lu", (unsigned long)rpiErrCount);
    }

    /* 6) 环形缓冲溢出首次出现时报告一次：说明主循环太慢或波特率不对 */
    if ((rpiRxDropCount != 0U) && (rpiRxDropLogged == 0U))
    {
        rpiRxDropLogged = 1U;
        Debug_Log("RPi", "rx ring overflow, dropped=%lu (check loop load/baud)",
                  (unsigned long)rpiRxDropCount);
    }
}

/**
 * @brief  取 $RCP 结果（转盘中心偏移）
 * @param  无
 * @retval 结构体常量指针
 */
const RPi_Rcp_t *RPi_GetRcp(void)
{
    return &rpiRcp;
}

/**
 * @brief  取 $RZ 结果（转盘三位号/颜色）
 * @param  无
 * @retval 结构体常量指针
 */
const RPi_Rz_t *RPi_GetRz(void)
{
    return &rpiRz;
}

/**
 * @brief  取 $CCG 结果（准备位颜色应答）
 * @param  无
 * @retval 结构体常量指针
 */
const RPi_Ccg_t *RPi_GetCcg(void)
{
    return &rpiCcg;
}

/**
 * @brief  取 $SHOT 结果（打靶区数据）
 * @param  无
 * @retval 结构体常量指针
 */
const RPi_Shot_t *RPi_GetShot(void)
{
    return &rpiShot;
}

/**
 * @brief  取 $SHOT2 结果（★ROUGH STACK 用：瞄准"三个横排物料中间那个"的颜色质心）
 * @param  无
 * @retval 结构体常量指针（valid=0 时数值已清零）
 * @note   与 `RPi_GetShot()` 是**两套独立数据**（见视觉组《SHOT2通讯对接.md》）。
 */
const RPi_Shot_t *RPi_GetShot2(void)
{
    return &rpiShot2;
}

/**
 * @brief  取 $RP 结果（★转盘圆盘平行度）
 * @param  无
 * @retval 结构体常量指针（valid=0 时 q1 已清零）
 */
const RPi_Rp_t *RPi_GetRp(void)
{
    return &rpiRp;
}

/**
 * @brief  便捷查询：在最近一次 $RZ 结果里按颜色找位号
 * @param  color: 目标颜色编号（1红 2黄 3蓝 4绿）
 * @retval 该颜色所在位号 1~3；找不到 / $RZ 无效 返回 0
 */
uint8_t RPi_FindColorPos(uint8_t color)
{
    uint8_t p;

    if ((rpiRz.valid == 0U) || (color == 0U)) { return 0U; }
    for (p = 1U; p <= 3U; p++)
    {
        if (rpiRz.byPos[p] == color) { return p; }
    }
    return 0U;
}

/**
 * @brief  便捷查询：取某位号的颜色（由 $RZ 的 byPos 表）
 * @param  pos: 位号 1~3
 * @retval 该位颜色 1~4；0=未识别/无效/越界
 */
uint8_t RPi_GetColorAtPos(uint8_t pos)
{
    if ((pos < 1U) || (pos > 3U)) { return 0U; }
    return rpiRz.byPos[pos];
}

/**
 * @brief  查询链路是否在线
 * @param  无
 * @retval 1=收到过字节；0=从未收到
 * @note   它回答"线路通不通"，不回答"数据可不可用"（后者看各结构体 valid）。
 */
uint8_t RPi_IsOnline(void)
{
    return rpiOnline;
}

/**
 * @brief  查询视觉端是否在线（心跳判据）
 * @param  无
 * @retval 1=最近 RPI_HB_TIMEOUT_MS 内收到过 $PING；0=超时/从未收到
 */
uint8_t RPi_IsAlive(void)
{
    if (rpiPingSeen == 0U)
    {
        return 0U;
    }
    return ((HAL_GetTick() - rpiPingTick) <= RPI_HB_TIMEOUT_MS) ? 1U : 0U;
}

/**
 * @brief  取视觉端当前状态（$PING 上报）
 * @param  无
 * @retval RPi_VsMode_t
 */
RPi_VsMode_t RPi_GetVsMode(void)
{
    return rpiVsMode;
}

/**
 * @brief  取有效帧累计计数
 * @param  无
 * @retval 有效帧数（未知帧名不计入）
 */
uint32_t RPi_GetRxCount(void)
{
    return rpiFrameCount;
}

/**
 * @brief  取坏帧累计计数
 * @param  无
 * @retval 坏帧数（未知帧名不计入）
 */
uint32_t RPi_GetBadCount(void)
{
    return rpiBadCount;
}

/**
 * @brief  取最近一帧摘要文本
 * @param  无
 * @retval 静态缓冲指针；尚无帧时返回 "---"
 */
const char *RPi_GetLastSummary(void)
{
    if (rpiLastSummary[0] == '\0')
    {
        return "---";
    }
    return rpiLastSummary;
}

/**
 * @brief  发送 $RCP：请求转盘中心 xy 偏移
 * @param  无
 * @retval 无
 * @note   组帧 $RCP*41。
 */
void RPi_ReqRcp(void)
{
    RPi_SendFrame("RCP");
}

/**
 * @brief  发送 $RZ：请求转盘三个位号的位号/颜色
 * @param  无
 * @retval 无
 * @note   组帧 $RZ*08。
 */
void RPi_ReqRz(void)
{
    RPi_SendFrame("RZ");
}

/**
 * @brief  发送 $CCG,<pos>：请求指定位号的颜色
 * @param  pos: 位号，仅允许 1~3
 * @retval 无
 * @note   越界**不发送**并打日志；发送成功后记录 reqPos 供应答核对。
 */
void RPi_ReqCcg(uint8_t pos)
{
    char content[16];

    if ((pos < 1U) || (pos > 3U))
    {
        Debug_Log("RPi", "CCG pos=%u out of 1~3, not sent", (unsigned)pos);
        return;
    }

    (void)snprintf(content, sizeof(content), "CCG,%u", (unsigned)pos);
    rpiCcg.reqPos = pos;      /* 记下本次请求位号，供上行 $CCG,c 应答核对 */
    RPi_SendFrame(content);
}

/**
 * @brief  发送 $SHOT：进入打靶阶段，请求圆环数据
 * @param  无
 * @retval 无
 * @note   组帧 $SHOT*00。
 */
void RPi_ReqShot(void)
{
    RPi_SendFrame("SHOT");
}

/**
 * @brief  发送 $SHOT2：请求"中间物料颜色质心"的偏移 + 平行度（★ROUGH STACK 用）
 * @param  无
 * @retval 无
 * @note   组帧 `$SHOT2*32`（校验由 RPi_SendFrame 自动算）。
 *         视觉组《SHOT2通讯对接.md》：`$SHOT2` 与 `$SHOT` **不是同一个测量**
 *         （SHOT=中间圆环圆心 / SHOT2=中间物料的颜色质心），数据各自独立。
 */
void RPi_ReqShot2(void)
{
    RPi_SendFrame("SHOT2");
}

/**
 * @brief  发送 $RP：请求**转盘圆盘平行度**（新增协议《新增协议_RP.md》）
 * @param  无
 * @retval 无
 * @note   组帧 `$RP*02`（校验由 RPi_SendFrame 自动算：XOR('R','P')=0x02）。
 *         视觉端收到后切 TURN 模式（640×480）；之后 `$RCP/$RZ/$CCG` 同一视图。
 */
void RPi_ReqRp(void)
{
    RPi_SendFrame("RP");
}

/**
 * @brief  下发任务码 $CODE,<任务码>
 * @param  task_code: 任务码字符串
 * @retval 无
 * @note   **每次都发**（扫到一次就下发一次，不按内容去重）。协议本身幂等
 *         （树莓派收到同一码只刷新显示），重复下发无副作用。
 *         task_code 为 NULL 或长度不等于 RPI_TASK_CODE_LEN(15) 时改发占位码。
 */
void RPi_SendCode(const char *task_code)
{
    char content[32];

    if ((task_code == NULL) || (strlen(task_code) != (size_t)RPI_TASK_CODE_LEN))
    {
        Debug_Log("RPi", "code len != %u, use placeholder", (unsigned)RPI_TASK_CODE_LEN);
        task_code = RPI_TASK_CODE_PLACEHOLDER;
    }

    (void)snprintf(content, sizeof(content), "CODE,%s", task_code);
    RPi_SendFrame(content);
}

/**
 * @brief  取可调配置（在线标定直接改）
 * @param  无
 * @retval 可写配置指针
 */
RPi_Cfg_t *RPi_GetCfg(void)
{
    return &rpiCfg;
}

/**
 * @brief  配置恢复成 .h 宏默认值
 * @param  无
 * @retval 无
 */
void RPi_CfgReset(void)
{
    rpiCfg.anchorX    = (int16_t)RPI_IMG_CENTER_X;
    rpiCfg.anchorY    = (int16_t)RPI_IMG_CENTER_Y;
    rpiCfg.imgXToBody = (uint8_t)RPI_IMG_X_TO_BODY;
    rpiCfg.imgXSign   = (int8_t)RPI_IMG_X_SIGN;
    rpiCfg.imgYToBody = (uint8_t)RPI_IMG_Y_TO_BODY;
    rpiCfg.imgYSign   = (int8_t)RPI_IMG_Y_SIGN;
    rpiCfg.px2mmNum[0] = (uint16_t)RPI_PX2MM_NUM_FWD;
    rpiCfg.px2mmDen[0] = (uint16_t)RPI_PX2MM_DEN_FWD;
    rpiCfg.px2mmNum[1] = (uint16_t)RPI_PX2MM_NUM_RIGHT;
    rpiCfg.px2mmDen[1] = (uint16_t)RPI_PX2MM_DEN_RIGHT;
    rpiCfg.deadbandPx = (int16_t)RPI_PX_DEADBAND;
    rpiCfg.maxPx      = (int16_t)RPI_PX_MAX;

    rpiDenZeroWarned[0] = 0U;
    rpiDenZeroWarned[1] = 0U;
}

/**
 * @brief  USART1 单字节接收中断处理
 * @param  b: 刚收到的字节
 * @retval 无
 * @note   中断上下文：只搬字节、刷时间戳与在线标志；不解析、不换算、不打印。
 */
void RPi_RxHandler(uint8_t b)
{
    uint16_t next = (uint16_t)((rpiRxHead + 1U) % RPI_RX_RING_SIZE);

    rpiOnline = 1U;                        /* 线路级：收到过字节即 1（永不回落） */

    if (next == rpiRxTail)
    {
        /* 环满：丢弃并计数（主循环太慢或波特率不对；RPi_Update 会报一次） */
        rpiRxDropCount++;
        return;
    }

    rpiRxRing[rpiRxHead] = b;
    rpiRxHead = next;
}

/**
 * @brief  通知 USART1 接收错误（ORE/FE/NE/PE）
 * @param  无
 * @retval 无
 * @note   中断上下文，只累加计数；重装 RX IT 由 bsp_imu.c 的 ErrorCallback 负责。
 */
void RPi_NotifyError(void)
{
    rpiErrCount++;
}
