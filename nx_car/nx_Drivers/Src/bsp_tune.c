/**
 ******************************************************************************
 * @file    bsp_tune.c
 * @brief   在线调参模块实现（详见 bsp_tune.h 的命令说明）
 * @author  nx_car
 * @date    2026-09-23
 ******************************************************************************
 * @note   设计要点：
 *   1. **轮询收字节**：直接读 UART5 的 SR/DR，不装中断、不改 stm32f1xx_it.c；
 *      读 SR 再读 DR 会一并清掉 ORE/FE/NE/PE，收链路不会卡死。
 *   2. **逐行解析**：攒到 '\n'（或 '\r'）才解析一行；行太长就丢弃并报错，
 *      防止把后续命令粘在超长行尾。
 *   3. **参数表**：Tune_Init 时把 RouteCfg_t 的字段逐个登记（名字 + 类型 + 合法范围），
 *      越界一律**夹到合法值**（绝不接受 0 分母之类的致命值）。
 *   4. 全部非阻塞、禁 HAL_Delay；日志禁 %f（都是整数参数）。
 *   5. 改的是 RAM 值，掉电不保存（要固化就抄回 bsp_route.h 的宏）。
 ******************************************************************************
 */
#include "bsp_tune.h"
#include "bsp_route.h"
#include "bsp_align.h"        /* ★运行期 PID 参数表（三套：RCP/SHOT/SHOT2） */
#include "bsp_align_test.h"   /* ★命令行启动对位（AlnTest_StartSrc / StopCmd） */
#include "bsp_vel_test.h"     /* ★VT 速度模式测试页串口命令（VTCFG/VTRUN/VTSTOP...） */
#include "bsp_vel_route.h"    /* ★VR VELROUTE 跑图页命令（VRCFG/VRRUN/VRSTOP/VRTELE） */
#include "bsp_mission.h"      /* ★速度模式内核运行期参数（Mission_VelSetParam/MV_*） */
#include "bsp_task.h"         /* ★赛题页测试启动（Task_Enter / Task_HandleKey / Task_Stop） */
#include "bsp_debug.h"
#include "usart.h"      /* huart5：直接读 UART5 的 SR/DR 轮询收命令（不占中断） */
#include <string.h>

/* ===================== 可调项 ===================== */

#define TUNE_LINE_MAX   160U    /* 单行命令最大长度（含结尾）
                                   160 是为了容纳一整条 ALNCFG（9 个参数 ~95 字符） */

typedef struct {
    const char *name;   /* 参数名（命令里用的名字） */
    uint8_t     is8;    /* 1=uint8 字段；0=uint16 字段 */
    uint16_t    minV;   /* 合法下限（含） */
    uint16_t    maxV;   /* 合法上限（含） */
    uint16_t   *pu16;   /* is8=0 时有效 */
    uint8_t    *pu8;    /* is8=1 时有效 */
} TuneEnt_t;

static TuneEnt_t tuneTab[TUNE_ENT_MAX];
static uint8_t   tuneEntCnt = 0U;

static char     tuneLine[TUNE_LINE_MAX];
static uint8_t  tuneLineLen = 0U;
static uint8_t  tuneLineOvf = 0U;
static uint32_t tuneRxBytes = 0U;

/* ===== ★【2026-10-15】UART5 收命令改为 **中断 + 环形缓冲** =====
   【为什么必须改】日志口 UART5 的发送是**阻塞**的（Debug_Log 一行 40~60 字节，
   115200 下要 4~5ms）。这段时间里主循环根本不去读 RX，而 RX 只有 1 字节寄存器 ->
   命令字节直接**溢出丢失**。现场实测：同一条命令发 5 次只有约 1 次能完整解析，
   日志里全是 `err unknown cmd / err unknown name`（少了字符）。
   现在由 RXNE 中断（stm32f1xx_it.c 的 UART5_IRQHandler）只做"塞进环形缓冲"，
   解析仍在主循环 Tune_Update() 里做（ISR 里绝不打印日志）。 */
#define TUNE_RING_SZ    256U

static volatile uint8_t  tuneRing[TUNE_RING_SZ];
static volatile uint16_t tuneRingHead = 0U;
static volatile uint16_t tuneRingTail = 0U;
static volatile uint32_t tuneRingDrop = 0U;   /* 缓冲满丢弃的字节数（诊断用） */

/* ===================== 内部工具 ===================== */

/** @brief 登记一个 uint16 参数（名字 / 合法范围 / 变量地址） */
static void Tune_AddU16(const char *name, uint16_t minV, uint16_t maxV, uint16_t *p)
{
    if (tuneEntCnt >= TUNE_ENT_MAX)
    {
        return;
    }
    tuneTab[tuneEntCnt].name = name;
    tuneTab[tuneEntCnt].is8  = 0U;
    tuneTab[tuneEntCnt].minV = minV;
    tuneTab[tuneEntCnt].maxV = maxV;
    tuneTab[tuneEntCnt].pu16 = p;
    tuneTab[tuneEntCnt].pu8  = NULL;
    tuneEntCnt++;
}

/** @brief 登记一个 uint8 参数 */
static void Tune_AddU8(const char *name, uint8_t minV, uint8_t maxV, uint8_t *p)
{
    if (tuneEntCnt >= TUNE_ENT_MAX)
    {
        return;
    }
    tuneTab[tuneEntCnt].name = name;
    tuneTab[tuneEntCnt].is8  = 1U;
    tuneTab[tuneEntCnt].minV = (uint16_t)minV;
    tuneTab[tuneEntCnt].maxV = (uint16_t)maxV;
    tuneTab[tuneEntCnt].pu16 = NULL;
    tuneTab[tuneEntCnt].pu8  = p;
    tuneEntCnt++;
}

/** @brief 不区分大小写的字符串比较（命令关键字用） */
static uint8_t Tune_Eq(const char *a, const char *b)
{
    while ((*a != '\0') && (*b != '\0'))
    {
        char ca = *a;
        char cb = *b;

        if ((ca >= 'A') && (ca <= 'Z')) { ca = (char)(ca - 'A' + 'a'); }
        if ((cb >= 'A') && (cb <= 'Z')) { cb = (char)(cb - 'A' + 'a'); }
        if (ca != cb)
        {
            return 0U;
        }
        a++;
        b++;
    }

    return (uint8_t)((*a == '\0') && (*b == '\0'));
}

/** @brief 解析非负十进制整数；成功返回 1（负数/非数字/空串返回 0） */
static uint8_t Tune_ParseNum(const char *s, int32_t *out)
{
    int32_t v = 0;
    uint8_t n = 0U;

    while (*s == ' ')
    {
        s++;
    }

    while ((*s >= '0') && (*s <= '9'))
    {
        v = (v * 10) + (int32_t)(*s - '0');
        n++;
        s++;
        if (n > 6U)
        {
            return 0U;              /* 明显不是合法参数值 */
        }
    }

    while (*s == ' ')
    {
        s++;
    }

    if ((n == 0U) || (*s != '\0'))
    {
        return 0U;
    }

    *out = v;
    return 1U;
}

/* ===== ★【2026-10-15】对位调参命令的解析工具 ===== */

/** @brief 前缀匹配（不区分大小写，且前缀后必须是空格或行尾） */
static uint8_t Tune_Prefix(const char *s, const char *pre)
{
    while (*pre != '\0')
    {
        char ca = *s;
        char cb = *pre;

        if ((ca >= 'A') && (ca <= 'Z')) { ca = (char)(ca - 'A' + 'a'); }
        if ((cb >= 'A') && (cb <= 'Z')) { cb = (char)(cb - 'A' + 'a'); }
        if ((ca != cb) || (*s == '\0'))
        {
            return 0U;
        }
        s++;
        pre++;
    }

    return (uint8_t)((*s == '\0') || (*s == ' '));
}

/**
 * @brief  在 "k1=v1 k2=v2 ..." 里找 key，返回 "=" 后的值起点
 * @retval NULL=没找到
 */
static const char *Tune_FindKey(const char *line, const char *key)
{
    size_t      klen = strlen(key);
    const char *p    = line;

    while (*p != '\0')
    {
        while (*p == ' ')
        {
            p++;
        }
        if ((strncmp(p, key, klen) == 0) && (p[klen] == '='))
        {
            return p + klen + 1;
        }
        while ((*p != '\0') && (*p != ' '))
        {
            p++;
        }
    }

    return NULL;
}

/** @brief 解析非负整数（到空格/行尾为止）；成功返回 1 */
static uint8_t Tune_ParseUInt(const char *s, int32_t *out)
{
    int32_t v = 0;
    uint8_t n = 0U;

    while ((*s >= '0') && (*s <= '9'))
    {
        v = (v * 10) + (int32_t)(*s - '0');
        n++;
        s++;
        if (n > 7U)
        {
            return 0U;
        }
    }

    if (n == 0U)
    {
        return 0U;
    }

    *out = v;
    return 1U;
}

/** @brief 取 "key=值" 里的整数；缺失/格式错返回 0 */
static uint8_t Tune_KeyInt(const char *line, const char *key, int32_t *out)
{
    const char *v = Tune_FindKey(line, key);

    return (v == NULL) ? 0U : Tune_ParseUInt(v, out);
}

/** @brief 源名 -> 0=RCP 1=SHOT 2=SHOT2；失败返回 0 */
static uint8_t Tune_ParseSrc(const char *s, uint8_t *src)
{
    if (strncmp(s, "SHOT2", 5) == 0) { *src = 2U; return 1U; }
    if (strncmp(s, "SHOT", 4) == 0)  { *src = 1U; return 1U; }
    if (strncmp(s, "RCP", 3) == 0)   { *src = 0U; return 1U; }

    return 0U;
}

static const char *Tune_SrcName(uint8_t src)
{
    if (src == 1U) { return "SHOT"; }
    if (src == 2U) { return "SHOT2"; }
    return "RCP";
}

/**
 * @brief  打印某一套对位 PID 参数（整数化：增益×1000、RPM×10）
 */
static void Tune_DumpAlignOne(uint8_t src)
{
    const Align_PidCfg_t *c = Align_GetPidCfg(src);

    if (c == NULL)
    {
        return;
    }

    Debug_Log("TUNE", "alncfg src=%s kp=%d ki=%d kd=%d rpm=%d wheel=%d "
                      "ykp=%d yki=%d ykd=%d yrpm=%d",
              Tune_SrcName(src),
              (int)(c->kp * 1000.0f + 0.5f), (int)(c->ki * 1000.0f + 0.5f),
              (int)(c->kd * 1000.0f + 0.5f), (int)(c->rpmMax * 10.0f + 0.5f),
              (int)c->wheelMax,
              (int)(c->ykp * 1000.0f + 0.5f), (int)(c->yki * 1000.0f + 0.5f),
              (int)(c->ykd * 1000.0f + 0.5f), (int)(c->yrpmMax * 10.0f + 0.5f));
}

/** @brief 打印三套对位 PID 参数 */
static void Tune_DumpAlign(void)
{
    uint8_t i;

    for (i = 0U; i < 3U; i++)
    {
        Tune_DumpAlignOne(i);
    }
    Debug_Log("TUNE", "alndump end");
}

/**
 * @brief  ALNCFG：一次设置某一套的全部 9 个参数
 * @param  p: 命令原文（"ALNCFG src=... kp=..."）
 * @note   整数值定标：kp/ki/kd/ykp/yki/ykd ×1000，rpm/yrpm ×10，wheel 原值。
 *         这样 PC 端不用解析浮点，串口也不会出现小数点。
 */
static void Tune_ExecAlignCfg(const char *p)
{
    const char  *srcs;
    uint8_t      src = 0U;
    int32_t      kp, ki, kd, rpm, whl, ykp, yki, ykd, yrpm;
    Align_PidCfg_t c;

    srcs = Tune_FindKey(p, "src");
    if ((srcs == NULL) || (Tune_ParseSrc(srcs, &src) == 0U))
    {
        Debug_Log("TUNE", "err alncfg src=RCP|SHOT|SHOT2");
        return;
    }
    if ((Tune_KeyInt(p, "kp",   &kp)   == 0U) || (Tune_KeyInt(p, "ki",   &ki)   == 0U) ||
        (Tune_KeyInt(p, "kd",   &kd)   == 0U) || (Tune_KeyInt(p, "rpm",  &rpm)  == 0U) ||
        (Tune_KeyInt(p, "wheel", &whl) == 0U) || (Tune_KeyInt(p, "ykp",  &ykp)  == 0U) ||
        (Tune_KeyInt(p, "yki",  &yki)  == 0U) || (Tune_KeyInt(p, "ykd",  &ykd)  == 0U) ||
        (Tune_KeyInt(p, "yrpm", &yrpm) == 0U))
    {
        Debug_Log("TUNE", "err alncfg need kp ki kd rpm wheel ykp yki ykd yrpm");
        return;
    }

    c.kp       = (float)kp / 1000.0f;
    c.ki       = (float)ki / 1000.0f;
    c.kd       = (float)kd / 1000.0f;
    c.rpmMax   = (float)rpm / 10.0f;
    c.wheelMax = whl;
    c.ykp      = (float)ykp / 1000.0f;
    c.yki      = (float)yki / 1000.0f;
    c.ykd      = (float)ykd / 1000.0f;
    c.yrpmMax  = (float)yrpm / 10.0f;

    if (Align_SetPidCfg(src, &c) == 0U)
    {
        Debug_Log("TUNE", "err alncfg bad src");
        return;
    }

    /* 回读**实际生效**值（被夹取时 PC 能立刻发现） */
    Tune_DumpAlignOne(src);
}

/** @brief 取参数当前值 */
static uint16_t Tune_GetVal(const TuneEnt_t *e)
{
    return (e->is8 != 0U) ? (uint16_t)(*e->pu8) : (*e->pu16);
}

/** @brief 写参数（自动夹到合法范围）；返回夹取后的值 */
static uint16_t Tune_SetVal(const TuneEnt_t *e, int32_t v)
{
    uint16_t out;

    if (v < (int32_t)e->minV)
    {
        v = (int32_t)e->minV;
    }
    if (v > (int32_t)e->maxV)
    {
        v = (int32_t)e->maxV;
    }
    out = (uint16_t)v;

    if (e->is8 != 0U)
    {
        *e->pu8 = (uint8_t)out;
    }
    else
    {
        *e->pu16 = out;
    }

    return out;
}

/** @brief 按名字找参数；找不到返回 NULL */
static const TuneEnt_t *Tune_Find(const char *name)
{
    uint8_t i;

    for (i = 0U; i < tuneEntCnt; i++)
    {
        if (Tune_Eq(tuneTab[i].name, name) != 0U)
        {
            return &tuneTab[i];
        }
    }

    return NULL;
}

/** @brief 打印全部可调参数（每行一条，便于 PC 端直接入库） */
static void Tune_Dump(void)
{
    uint8_t i;

    for (i = 0U; i < tuneEntCnt; i++)
    {
        Debug_Log("TUNE", "%s=%u [%u..%u]",
                  tuneTab[i].name, (unsigned int)Tune_GetVal(&tuneTab[i]),
                  (unsigned int)tuneTab[i].minV, (unsigned int)tuneTab[i].maxV);
    }
    Debug_Log("TUNE", "dump end");
}

/**
 * @brief  执行一行命令
 * @param  line: 以 '\0' 结尾的命令行（已去掉行尾）
 * @retval 无
 */
static void Tune_ExecLine(char *line)
{
    char *p = line;
    char *eq;

    while (*p == ' ')
    {
        p++;
    }
    if (*p == '\0')
    {
        return;                     /* 空行 */
    }

    /* ---- ★对位调参命令（2026-10-15，见 bsp_tune.h 说明） ----
       注意顺序：ALNCFG / ALNDUMP 必须先判，否则会被 "ALN" 前缀吃掉。 */
    if (Tune_Prefix(p, "ALNCFG") != 0U)
    {
        Tune_ExecAlignCfg(p);
        return;
    }
    if (Tune_Prefix(p, "ALNDUMP") != 0U)
    {
        Tune_DumpAlign();
        return;
    }
    if (Tune_Prefix(p, "ALN") != 0U)
    {
        const char *q = p + 3;
        const char *srcs;
        uint8_t     src = 0U;

        while (*q == ' ')
        {
            q++;
        }
        if (Tune_Eq(q, "STOP") != 0U)
        {
            AlnTest_StopCmd();
            Debug_Log("TUNE", "aln stop");
            return;
        }
        if (Tune_Eq(q, "EXIT") != 0U)
        {
            AlnTest_ExitCmd();
            Debug_Log("TUNE", "aln exit");
            return;
        }
        srcs = Tune_FindKey(q, "src");
        if ((srcs == NULL) || (Tune_ParseSrc(srcs, &src) == 0U))
        {
            Debug_Log("TUNE", "err aln src=RCP|SHOT|SHOT2 | ALN STOP | ALN EXIT");
            return;
        }
        AlnTest_StartSrc(src);
        Debug_Log("TUNE", "aln start src=%s (state=%u)",
                  Tune_SrcName(src), (unsigned)AlnTest_GetStateCode());
        return;
    }

    /* ---- ★赛题页（MISSION）远程测试：MSRUN=启动 AUTO / MSSTOP=急停 ----
       用来在现场用串口对比"赛题页的转向"和 VELROUTE 的转向。 */
    if (Tune_Eq(p, "MSRUN") != 0U)
    {
        if (Task_IsActive() == 0U)
        {
            Task_Enter();                   /* 进页 + 里程计清零 */
        }
        Task_HandleKey(KEY5);               /* AUTO：从当前步一键启动 */
        Debug_Log("TUNE", "ms run");
        return;
    }
    if (Tune_Eq(p, "MSSTOP") != 0U)
    {
        Task_Stop();
        Debug_Log("TUNE", "ms stop");
        return;
    }
    if (Tune_Eq(p, "MSMODE") != 0U)     /* 切 STEP/AUTO（KEY3） */
    {
        Task_HandleKey(KEY3);
        Debug_Log("TUNE", "ms mode toggled");
        return;
    }
    if (Tune_Eq(p, "MSAUTO") != 0U)     /* 显式设 AUTO */
    {
        Task_CmdSetMode(1U);
        return;
    }
    if (Tune_Eq(p, "MSSTEP") != 0U)     /* 显式设 STEP */
    {
        Task_CmdSetMode(0U);
        return;
    }
    if (Tune_Eq(p, "MSPREV") != 0U)     /* 上一步（KEY1，只移指针） */
    {
        Task_HandleKey(KEY1);
        Debug_Log("TUNE", "ms prev");
        return;
    }
    if (Tune_Eq(p, "MSFIRST") != 0U)    /* 步号回到第 1 步（连打 KEY1，远程测试用） */
    {
        uint8_t i;

        for (i = 0U; i < 40U; i++)
        {
            Task_HandleKey(KEY1);
        }
        Debug_Log("TUNE", "ms step -> 1");
        return;
    }

    /* ---- ★VELROUTE 速度模式跑图（P26）：在线改运动参数 + 串口起停 + 遥测 ----
       VRCFG V=1000 A=600 FA=300 C=80 T=12 K=12 D=15 Y=40 M=200
       VRRUN / VRRUN=5 / VRSTOP / VRDUMP / VRTELE=20
       ★参数是速度模式内核的运行期变量（bsp_mission.c 的 mnvP_*），改完立刻生效。 */
    if ((p[0] == 'V') && (p[1] == 'R'))
    {
        const char *q = p + 2;
        int32_t     v;

        while (*q == ' ')
        {
            q++;
        }

        if (Tune_Prefix(q, "CFG") != 0U)
        {
            const char *c = q + 3;

            if (Tune_KeyInt(c, "V",  &v) != 0U) { (void)Mission_VelSetParam((uint8_t)MV_VEL,   v); }
            if (Tune_KeyInt(c, "A",  &v) != 0U) { (void)Mission_VelSetParam((uint8_t)MV_ACC,   v); }
            if (Tune_KeyInt(c, "FA", &v) != 0U) { (void)Mission_VelSetParam((uint8_t)MV_FACC,  v); }
            if (Tune_KeyInt(c, "C",  &v) != 0U) { (void)Mission_VelSetParam((uint8_t)MV_CREEP, v); }
            if (Tune_KeyInt(c, "T",  &v) != 0U) { (void)Mission_VelSetParam((uint8_t)MV_TOL,   v); }
            if (Tune_KeyInt(c, "K",  &v) != 0U) { (void)Mission_VelSetParam((uint8_t)MV_KP,    v); }
            if (Tune_KeyInt(c, "D",  &v) != 0U) { (void)Mission_VelSetParam((uint8_t)MV_KD,    v); }
            if (Tune_KeyInt(c, "Y",  &v) != 0U) { (void)Mission_VelSetParam((uint8_t)MV_YMAX,  v); }
            if (Tune_KeyInt(c, "M",  &v) != 0U) { (void)Mission_VelSetParam((uint8_t)MV_TMAX,  v); }
            if (Tune_KeyInt(c, "TK", &v) != 0U) { (void)Mission_VelSetParam((uint8_t)MV_TKP,   v); }
            if (Tune_KeyInt(c, "TD", &v) != 0U) { (void)Mission_VelSetParam((uint8_t)MV_TKD,   v); }
            if (Tune_KeyInt(c, "TA", &v) != 0U) { (void)Mission_VelSetParam((uint8_t)MV_TACC,  v); }

            Mission_VelDump();
            return;
        }

        if (strncmp(q, "RUN", 3) == 0)
        {
            const char *vs = Tune_FindKey(q, "RUN");
            int32_t     s  = 1;

            if ((vs != NULL) && (Tune_ParseUInt(vs, &s) == 0U))
            {
                s = 1;
            }
            if (s < 1)   { s = 1; }
            if (s > 255) { s = 255; }

            Debug_Log("TUNE", "vr run=%u", (unsigned int)VelRoute_CmdStart((uint8_t)s));
            return;
        }
        if (Tune_Eq(q, "STOP") != 0U)
        {
            VelRoute_CmdStop();
            Debug_Log("TUNE", "vr stop");
            return;
        }
        if (strncmp(q, "ROT", 3) == 0)
        {
            const char *vs = Tune_FindKey(q, "ROT");
            int32_t     d  = 90;

            if (vs != NULL)
            {
                int32_t sign = 1;

                while (*vs == ' ') { vs++; }
                if (*vs == '-') { sign = -1; vs++; }

                if (Tune_ParseUInt(vs, &d) != 0U)
                {
                    d *= sign;
                }
                else
                {
                    d = 90;
                }
            }
            (void)VelRoute_CmdRot(d);
            return;
        }
        if (Tune_Eq(q, "DUMP") != 0U)
        {
            VelRoute_CmdDump();
            return;
        }
        if (strncmp(q, "TELE", 4) == 0)
        {
            const char *vs = Tune_FindKey(q, "TELE");
            int32_t     t  = 20;

            if ((vs == NULL) || (Tune_ParseUInt(vs, &t) == 0U))
            {
                t = 20;
            }
            if (t < 0)    { t = 0; }
            if (t > 1000) { t = 1000; }
            VelRoute_SetTeleMs((uint16_t)t);
            Debug_Log("TUNE", "vr tele=%u ms", (unsigned int)VelRoute_GetTeleMs());
            return;
        }

        Debug_Log("TUNE", "err vr cmd: CFG/RUN/STOP/DUMP/TELE");
        return;
    }

    /* ---- ★速度模式测试页（P25）命令 ----
       VTCFG V=600 T1=1500 T2=1500 A=300 G=90 C=250 K=8 D=15
       VTRUN / VTSTOP / VTEXIT / VTDUMP / VTTELE=50
       ★不能用 Tune_Prefix(p,"VT")：它要求 VT 后是空格/行尾，而 "VTCFG" 紧跟着 'C'。 */
    if ((p[0] == 'V') && (p[1] == 'T'))
    {
        const char *q = p + 2;
        int32_t     v;

        while (*q == ' ')
        {
            q++;
        }

        if (Tune_Prefix(q, "CFG") != 0U)
        {
            const char *c = q + 3;

            if (Tune_KeyInt(c, "V",  &v) != 0U) { (void)VelTest_CmdSet((uint8_t)VT_CMD_V,  v); }
            if (Tune_KeyInt(c, "T1", &v) != 0U) { (void)VelTest_CmdSet((uint8_t)VT_CMD_T1, v); }
            if (Tune_KeyInt(c, "T2", &v) != 0U) { (void)VelTest_CmdSet((uint8_t)VT_CMD_T2, v); }
            if (Tune_KeyInt(c, "A",  &v) != 0U) { (void)VelTest_CmdSet((uint8_t)VT_CMD_A,  v); }
            if (Tune_KeyInt(c, "G",  &v) != 0U) { (void)VelTest_CmdSet((uint8_t)VT_CMD_G,  v); }
            if (Tune_KeyInt(c, "C",  &v) != 0U) { (void)VelTest_CmdSet((uint8_t)VT_CMD_VC, v); }
            if (Tune_KeyInt(c, "K",  &v) != 0U) { (void)VelTest_CmdSet((uint8_t)VT_CMD_KP, v); }
            if (Tune_KeyInt(c, "D",  &v) != 0U) { (void)VelTest_CmdSet((uint8_t)VT_CMD_KD, v); }
            if (Tune_KeyInt(c, "AC", &v) != 0U)
            {
                if (v < 0)   { v = 0; }
                if (v > 255) { v = 255; }
                VelTest_CmdSetAcc((uint8_t)v);
            }
            if (Tune_KeyInt(c, "TRK", &v) != 0U) { (void)VelTest_CmdSet((uint8_t)VT_CMD_TRK, v); }
            if (Tune_KeyInt(c, "TRR", &v) != 0U) { (void)VelTest_CmdSet((uint8_t)VT_CMD_TRR, v); }
            if (Tune_KeyInt(c, "TRM", &v) != 0U) { (void)VelTest_CmdSet((uint8_t)VT_CMD_TRM, v); }
            if (Tune_KeyInt(c, "TRA", &v) != 0U) { (void)VelTest_CmdSet((uint8_t)VT_CMD_TRA, v); }
            if (Tune_KeyInt(c, "TRD", &v) != 0U) { (void)VelTest_CmdSet((uint8_t)VT_CMD_TRD, v); }
            if (Tune_KeyInt(c, "SQ",  &v) != 0U) { (void)VelTest_CmdSet((uint8_t)VT_CMD_SQ, v); }
            if (Tune_KeyInt(c, "L",   &v) != 0U) { (void)VelTest_CmdSet((uint8_t)VT_CMD_LEG, v); }

            VelTest_CmdDump();
            return;
        }

        if (Tune_Eq(q, "RUN") != 0U)
        {
            Debug_Log("TUNE", "vt run=%u", (unsigned int)VelTest_CmdStart());
            return;
        }
        if (Tune_Eq(q, "STOP") != 0U)
        {
            VelTest_CmdStop();
            Debug_Log("TUNE", "vt stop");
            return;
        }
        if (Tune_Eq(q, "EXIT") != 0U)
        {
            VelTest_CmdExit();
            Debug_Log("TUNE", "vt exit");
            return;
        }
        if (Tune_Eq(q, "DUMP") != 0U)
        {
            VelTest_CmdDump();
            return;
        }
        if (strncmp(q, "TELE", 4) == 0)
        {
            const char *vs = Tune_FindKey(q, "TELE");
            int32_t     t  = 50;

            if ((vs == NULL) || (Tune_ParseUInt(vs, &t) == 0U))
            {
                t = 50;
            }
            if (t < 0)    { t = 0; }
            if (t > 1000) { t = 1000; }
            VelTest_SetTeleMs((uint16_t)t);
            Debug_Log("TUNE", "vt tele=%u ms", (unsigned int)VelTest_GetTeleMs());
            return;
        }

        Debug_Log("TUNE", "err vt cmd: CFG/RUN/STOP/EXIT/DUMP/TELE");
        return;
    }

    /* ---- 动作类命令 ---- */
    if (Tune_Eq(p, "RUN") != 0U)
    {
        Debug_Log("TUNE", "run");
        Route_Start();
        return;
    }
    if (Tune_Eq(p, "STOP") != 0U)
    {
        Debug_Log("TUNE", "stop");
        Route_Abort();
        return;
    }
    if (Tune_Eq(p, "DEFAULT") != 0U)
    {
        Route_CfgReset();
        Debug_Log("TUNE", "defaults restored");
        Tune_Dump();
        return;
    }
    if (Tune_Eq(p, "DUMP") != 0U)
    {
        Tune_Dump();
        return;
    }

    /* ---- ?name：读一个参数 ---- */
    if (*p == '?')
    {
        const TuneEnt_t *e = Tune_Find(p + 1);

        if (e == NULL)
        {
            Debug_Log("TUNE", "err unknown name");
        }
        else
        {
            Debug_Log("TUNE", "%s=%u", e->name, (unsigned int)Tune_GetVal(e));
        }
        return;
    }

    /* ---- name=value：改一个参数 ---- */
    eq = p;
    while ((*eq != '\0') && (*eq != '='))
    {
        eq++;
    }
    if (*eq == '=')
    {
        const TuneEnt_t *e;
        int32_t          v = 0;
        uint16_t         got;

        *eq = '\0';                 /* 截断出 name */
        eq++;                       /* value 起点 */

        e = Tune_Find(p);
        if (e == NULL)
        {
            Debug_Log("TUNE", "err unknown name");
            return;
        }
        if (Tune_ParseNum(eq, &v) == 0U)
        {
            Debug_Log("TUNE", "err bad value");
            return;
        }

        got = Tune_SetVal(e, v);
        Debug_Log("TUNE", "set %s=%u (req %d, range %u..%u)",
                  e->name, (unsigned int)got, (int)v,
                  (unsigned int)e->minV, (unsigned int)e->maxV);
        return;
    }

    Debug_Log("TUNE", "err unknown cmd");
}

/* ===================== 公开接口 ===================== */

/**
 * @brief  收一个字节（**中断里调用**，只入环形缓冲，绝不打印/解析）
 * @param  b: 收到的字节
 */
void Tune_RxByte(uint8_t b)
{
    uint16_t next = (uint16_t)((tuneRingHead + 1U) % TUNE_RING_SZ);

    if (next == tuneRingTail)
    {
        tuneRingDrop++;              /* 缓冲满：丢新字节（正常情况下不会发生） */
        return;
    }
    tuneRing[tuneRingHead] = b;
    tuneRingHead = next;
}

/**
 * @brief  主循环里解析一个字节（攒行 -> 执行）
 */
static void Tune_ExecByte(uint8_t b)
{
    tuneRxBytes++;

    if ((b == '\n') || (b == '\r'))
    {
        if (tuneLineOvf != 0U)
        {
            Debug_Log("TUNE", "err line too long");
            tuneLineOvf = 0U;
            tuneLineLen = 0U;
            return;
        }
        if (tuneLineLen == 0U)
        {
            return;                 /* 空行（连续换行）忽略 */
        }

        tuneLine[tuneLineLen] = '\0';
        Tune_ExecLine(tuneLine);
        tuneLineLen = 0U;
        return;
    }

    if (tuneLineOvf != 0U)
    {
        return;                     /* 已溢出：丢掉本行剩余字节 */
    }
    if (tuneLineLen >= (TUNE_LINE_MAX - 1U))
    {
        tuneLineOvf = 1U;
        return;
    }

    tuneLine[tuneLineLen] = (char)b;
    tuneLineLen++;
}

uint32_t Tune_GetRxBytes(void)
{
    return tuneRxBytes;
}

void Tune_Init(void)
{
    RouteCfg_t *c = Route_GetCfg();

    tuneEntCnt   = 0U;
    tuneLineLen  = 0U;
    tuneLineOvf  = 0U;
    tuneRxBytes  = 0U;
    tuneRingHead = 0U;
    tuneRingTail = 0U;
    tuneRingDrop = 0U;

    /* ---- 参数表：名字 / 合法范围 / 字段地址 ----
       范围就是"安全边界"：越界会被夹住，绝不放行 0 分母、0 容差这类致命值。 */
    Tune_AddU16("rotScaleNum",        50U,  200U, &c->rotScaleNum);
    Tune_AddU16("rotScaleDen",        50U,  200U, &c->rotScaleDen);
    Tune_AddU16("rotVel",             20U,  500U, &c->rotVel);
    Tune_AddU16("distScaleNum",       50U,  200U, &c->distScaleNum);
    Tune_AddU16("distScaleDen",       50U,  200U, &c->distScaleDen);
    Tune_AddU16("tableScalePct",       5U,  100U, &c->tableScalePct);
    Tune_AddU8 ("lapRepeat",           0U,  255U, &c->lapRepeat);
    Tune_AddU16("holdMs",              0U,  500U, &c->holdMs);
    Tune_AddU8 ("yawEnable",           0U,    1U, &c->yawEnable);
    Tune_AddU16("yawTolDeg",           0U,   30U, &c->yawTolDeg);
    Tune_AddU16("yawCorrPct",         20U,  100U, &c->yawCorrPct);
    Tune_AddU16("yawMaxCorrX10",      30U,  900U, &c->yawMaxCorrX10);
    Tune_AddU8 ("yawMaxRound",         1U,    5U, &c->yawMaxRound);
    Tune_AddU16("yawBudgetMs",       200U, 3000U, &c->yawBudgetMs);

    /* ★打开 UART5 RX 中断：日志发送是阻塞的，只有中断才收得住命令字节 */
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);

    Debug_Log("TUNE", "ready, %u params (UART5 RX irq, ring=%u)",
              (unsigned int)tuneEntCnt, (unsigned int)TUNE_RING_SZ);
}

void Tune_Update(void)
{
    uint8_t guard = 0U;

    /* 从环形缓冲取字节解析；一次最多 128 字节，避免长时间占住主循环。
       中断只负责入队（Tune_RxByte），日志打印/命令执行都留在主循环里。 */
    while ((tuneRingTail != tuneRingHead) && (guard < 128U))
    {
        uint8_t b = tuneRing[tuneRingTail];

        tuneRingTail = (uint16_t)((tuneRingTail + 1U) % TUNE_RING_SZ);
        Tune_ExecByte(b);
        guard++;
    }
}
