/**
 ******************************************************************************
 * @file    bsp_align_test.c
 * @brief   对位调参测试页实现（P24）：单独跑一次对位 + 逐帧输出调参数据
 * @author  nx_car
 * @date    2026-10-15
 ******************************************************************************
 * @note   本页只做三件事：
 *           ① 提供三选一的数据源（$RCP / $SHOT / $SHOT2），按 KEY3 单独跑一次对位；
 *           ② 对位运行中按 `ALNT_LOG_MS` 打印**逐帧调参日志**（误差 + PID 输出 + 帧间隔）；
 *           ③ 把关键量显示在 OLED 上，现场不接串口也能看个大概。
 *
 *         调参日志的一行示例（字段含义见 bsp_align_test.h）：
 *           [ATN] t=1230 st=2 f=12 dt=100 px=-12,34 mm=-24,68 q=-8 u=1820,-410,0 w=22 db=1
 *
 *         硬约束：非阻塞；禁止 HAL_Delay；日志禁 %f（RPM 用 ×100 整数表示）。
 ******************************************************************************
 */
#include "bsp_align_test.h"
#include "bsp_align.h"        /* Align_Start / StartShot / StartShot2 / Abort / GetDebug */
#include "bsp_rpi.h"          /* RPi_ReqRcp/ReqShot/ReqShot2 + RPi_Get*()->valid */
#include "bsp_motor.h"        /* Motor_Enable + MOTOR_ADDR_MIN/MAX */
#include "bsp_arm_action.h"   /* ★启动前先把手臂摆到**全局观察位**（ArmAct_GotoTtReady） */
#include "bsp_oled_ui.h"      /* Display_SetPage / Display_MenuEnter / Display_AlignTestRefresh */
#include "bsp_debug.h"
#include <stdio.h>

/* ===================== 内部状态 ===================== */

typedef enum {
    AT_IDLE = 0,   /* 等用户按 KEY3 */
    AT_ARM,        /* ★手臂正在摆到观察位（摆好才开跑，避免挡住相机/碰靶） */
    AT_WAIT,       /* 已发请求，等视觉数据 valid */
    AT_RUN,        /* 对位进行中（Align_IsBusy） */
    AT_DONE,       /* 收敛 */
    AT_FAIL        /* 失败（手臂没摆好 / 无数据 / 对位失败） */
} AlnTestState_t;

static uint8_t  atActive  = 0U;         /* 1=本页活动 */
static uint8_t  atSrc     = 0U;         /* 0=$RCP 1=$SHOT 2=$SHOT2 */
static uint8_t  atState   = AT_IDLE;
static uint32_t atStart   = 0U;         /* 本次启动时刻（等待超时/用时计算） */
static uint32_t atLogTick = 0U;         /* 调参日志节流 */
static uint32_t atReqTick = 0U;         /* 等待期间重发请求节流 */
static uint32_t atUiTick  = 0U;         /* OLED 刷新节流 */
static uint16_t atLogCnt  = 0U;         /* 已打印的调参日志行数（诊断用） */

static const char *const atSrcName[3] = { "RCP", "SHOT", "SHOT2" };

/* ===================== 内部工具 ===================== */

/**
 * @brief  按当前数据源发一次视觉请求
 */
static void AlnTest_SendReq(void)
{
    if (atSrc == 0U)
    {
        RPi_ReqRcp();
        RPi_ReqRp();                /* ★转盘平行度（新增协议 $RP）：与 $RCP 同时请求 */
    }
    else if (atSrc == 1U) { RPi_ReqShot();  }
    else                  { RPi_ReqShot2(); }
}

/**
 * @brief  当前数据源是否已有**有效**偏移帧
 * @retval 1=有效
 */
static uint8_t AlnTest_DataValid(void)
{
    if (atSrc == 0U)
    {
        return (RPi_GetRcp()->valid != 0U) ? 1U : 0U;
    }
    if (atSrc == 1U)
    {
        return (RPi_GetShot()->valid != 0U) ? 1U : 0U;
    }
    return (RPi_GetShot2()->valid != 0U) ? 1U : 0U;
}

/**
 * @brief  停止对位（四轮软停）+ 回到 IDLE
 * @param  why: 日志里的原因（"user" / "exit"）
 */
static void AlnTest_Stop(const char *why)
{
    Align_Abort();
    if ((atState == AT_ARM) || (atState == AT_RUN) || (atState == AT_WAIT))
    {
        Debug_Log("ATN", "stop (%s)", why);
    }
    atState = AT_IDLE;
}

/**
 * @brief  启动一次对位（先发请求，等数据有效再由 Update 真正开跑）
 */
static void AlnTest_Kick(void)
{
    atStart   = HAL_GetTick();
    atReqTick = atStart;
    atLogTick = atStart;
    atLogCnt  = 0U;

    /* ★【2026-10-15 用户要求】**测试开始先把手臂摆到观察位**：
       ① 把爪子移出相机视野（否则视觉测到的是爪子，偏移是假的）；
       ② 对位时车会动，手臂在观察位才不会碰到转盘 / 靶面。 */
    if (ArmAct_GotoTtReady() != 0U)
    {
        atState = AT_ARM;
        Debug_Log("ATN", "start src=%s -> arm to observe pos first", atSrcName[atSrc]);
        return;
    }

    /* 手臂正在跑别的动作（或该动作被拒）：不阻塞测试，打警告后直接等数据 */
    atState = AT_WAIT;
    AlnTest_SendReq();
    Debug_Log("ATN", "WARN arm observe refused (busy?) -> start src=%s anyway",
              atSrcName[atSrc]);
}

/**
 * @brief  把 RPM×100 打成 "18.2" / "-4.1"
 * @note   日志禁 %f，所以用整数拆分；这里只用于 OLED 显示。
 */
static void AlnTest_RpmStr(int16_t u100, char *buf, uint16_t len)
{
    int32_t v = (int32_t)u100;
    int32_t a = (v < 0) ? -v : v;

    snprintf(buf, (size_t)len, "%s%ld.%ld",
             (v < 0) ? "-" : "", (long)(a / 100), (long)((a % 100) / 10));
}

/* ===================== 公开接口 ===================== */

void AlnTest_Enter(void)
{
    uint8_t addr;

    /* 四轮使能**抱轴**（只抱不动）：避免对位页里车被推着滑，导致偏移一直变 */
    for (addr = (uint8_t)MOTOR_ADDR_MIN; addr <= (uint8_t)MOTOR_ADDR_MAX; addr++)
    {
        Motor_Enable(addr, 1U);
    }

    atActive  = 1U;
    atState   = AT_IDLE;
    atSrc     = 0U;
    atLogCnt  = 0U;
    atUiTick  = HAL_GetTick();

    Display_SetPage(PAGE_ALIGN_TEST);

    Debug_Log("ATN", "enter (wheels enabled hold). K1/K2=src K3=start/stop K4=exit K5=stop");
}

void AlnTest_Exit(void)
{
    AlnTest_Stop("exit");
    atActive = 0U;
    Debug_Log("ATN", "exit, log lines=%u", (unsigned)atLogCnt);
    Display_MenuEnter();
}

uint8_t AlnTest_IsActive(void)
{
    return atActive;
}

void AlnTest_Update(void)
{
    uint32_t    now;
    Align_Dbg_t d;

    if (atActive == 0U)
    {
        return;
    }

    now = HAL_GetTick();

    /* ---- AT_ARM：等手臂摆到观察位，摆好再去请求视觉数据 ---- */
    if (atState == AT_ARM)
    {
        if (ArmAct_IsBusy() == 0U)
        {
            atState   = AT_WAIT;
            atReqTick = now;
            AlnTest_SendReq();
            Debug_Log("ATN", "arm ready (%lu ms) -> wait data src=%s",
                      (unsigned long)(now - atStart), atSrcName[atSrc]);
        }
        else if ((now - atStart) > (uint32_t)ALNT_ARM_MS)
        {
            atState = AT_FAIL;
            Debug_Log("ATN", "FAIL: arm not ready in %u ms", (unsigned)ALNT_ARM_MS);
        }
    }
    /* ---- AT_WAIT：等视觉数据有效 -> 真正启动 ---- */
    else if (atState == AT_WAIT)
    {
        if ((now - atReqTick) >= (uint32_t)ALNT_REQ_MS)
        {
            atReqTick = now;
            AlnTest_SendReq();
        }

        if (AlnTest_DataValid() != 0U)
        {
            uint8_t ok;

            if (atSrc == 0U)      { ok = Align_Start();      }
            else if (atSrc == 1U) { ok = Align_StartShot();  }
            else                  { ok = Align_StartShot2(); }

            if (ok != 0U)
            {
                atState   = AT_RUN;
                atLogTick = now;
                Debug_Log("ATN", "run src=%s (data ok, %lu ms)", atSrcName[atSrc],
                          (unsigned long)(now - atStart));
            }
        }

        if ((atState == AT_WAIT) && ((now - atStart) > (uint32_t)ALNT_WAIT_MS))
        {
            atState = AT_FAIL;
            Debug_Log("ATN", "FAIL: no valid data in %u ms (src=%s)",
                      (unsigned)ALNT_WAIT_MS, atSrcName[atSrc]);
        }
    }
    /* ---- AT_RUN：推日志 + 判结束 ---- */
    else if (atState == AT_RUN)
    {
        if (Align_IsBusy() == 0U)
        {
            Align_GetDebug(&d);

            if (Align_IsDone() != 0U)
            {
                atState = AT_DONE;
                Debug_Log("ATN", "DONE %lu ms frames=%u iters=%u px=%d,%d pass=%u",
                          (unsigned long)d.elapsedMs, (unsigned)d.frames, (unsigned)d.iters,
                          (int)d.dxPx, (int)d.dyPx, (unsigned)d.pass);
            }
            else
            {
                atState = AT_FAIL;
                Debug_Log("ATN", "FAIL %lu ms frames=%u iters=%u px=%d,%d q=%d",
                          (unsigned long)d.elapsedMs, (unsigned)d.frames, (unsigned)d.iters,
                          (int)d.dxPx, (int)d.dyPx, (int)d.qRaw);
            }
        }
        else if ((now - atLogTick) >= (uint32_t)ALNT_LOG_MS)
        {
            atLogTick = now;

            Align_GetDebug(&d);
            atLogCnt++;

            /* ★调参数据行（字段见 bsp_align_test.h）：一帧一条，便于还原"误差->输出"曲线 */
            Debug_Log("ATN",
                      "t=%lu st=%u f=%u dt=%u px=%d,%d mm=%d,%d q=%d u=%d,%d,%d w=%d db=%lu",
                      (unsigned long)d.elapsedMs, (unsigned)d.state, (unsigned)d.frames,
                      (unsigned)d.frameMs, (int)d.dxPx, (int)d.dyPx,
                      (int)d.mmFwd, (int)d.mmRight, (int)d.qRaw,
                      (int)d.uF, (int)d.uR, (int)d.uW, (int)d.wheelMax,
                      (unsigned long)d.deadbandPx);
        }
    }
    else
    {
        /* IDLE / DONE / FAIL：不做事 */
    }

    /* OLED 刷新节流（重绘比日志慢，200ms 足够看清） */
    if ((now - atUiTick) >= (uint32_t)ALNT_UI_MS)
    {
        atUiTick = now;
        Display_AlignTestRefresh();
    }
}

void AlnTest_HandleKey(KeyID_t key)
{
    if ((atActive == 0U) || (key == KEY_NONE))
    {
        return;
    }

    switch (key)
    {
        case KEY1:   /* 上一个数据源（运行中不许换） */
            if ((atState == AT_IDLE) || (atState == AT_DONE) || (atState == AT_FAIL))
            {
                atSrc = (atSrc == 0U) ? 2U : (uint8_t)(atSrc - 1U);
                Debug_Log("ATN", "src -> %s", atSrcName[atSrc]);
            }
            break;

        case KEY2:   /* 下一个数据源 */
            if ((atState == AT_IDLE) || (atState == AT_DONE) || (atState == AT_FAIL))
            {
                atSrc = (atSrc >= 2U) ? 0U : (uint8_t)(atSrc + 1U);
                Debug_Log("ATN", "src -> %s", atSrcName[atSrc]);
            }
            break;

        case KEY3:   /* 启动 / 停止 */
            if ((atState == AT_ARM) || (atState == AT_RUN) || (atState == AT_WAIT))
            {
                AlnTest_Stop("user K3");
            }
            else
            {
                AlnTest_Kick();
            }
            break;

        case KEY4:   /* 退出（先停） */
            AlnTest_Exit();
            return;

        case KEY5:   /* 只停止，不退出 */
            AlnTest_Stop("user K5");
            break;

        default:
            return;
    }

    Display_AlignTestRefresh();
}

/* ===================== 命令行入口（UART5 在线调参 / PC 自动调参） ===================== */

void AlnTest_StartSrc(uint8_t src)
{
    if (atActive == 0U)
    {
        AlnTest_Enter();            /* 自动进页：四轮抱轴 + 切页 */
    }

    /* 上一轮还没结束：先停，保证"一指令 = 一轮" */
    if ((atState == AT_ARM) || (atState == AT_RUN) || (atState == AT_WAIT))
    {
        AlnTest_Stop("cmd restart");
    }

    atSrc = (src < 3U) ? src : 0U;
    Debug_Log("ATN", "cmd start src=%s", atSrcName[atSrc]);
    AlnTest_Kick();
    Display_AlignTestRefresh();
}

void AlnTest_StopCmd(void)
{
    AlnTest_Stop("cmd");
    Display_AlignTestRefresh();
}

void AlnTest_ExitCmd(void)
{
    AlnTest_Exit();
}

uint8_t AlnTest_GetStateCode(void)
{
    return (uint8_t)atState;
}

/* ===================== 页面文本 ===================== */

const char *AlnTest_GetTitleText(void)
{
    static char buf[24];

    snprintf(buf, sizeof(buf), "ALN TEST %u/3 %s",
             (unsigned)(atSrc + 1U), atSrcName[atSrc]);

    return buf;
}

const char *AlnTest_GetErrText(void)
{
    static char buf[34];
    Align_Dbg_t d;

    if (atState == AT_WAIT)
    {
        return "px=-- (wait data)";
    }

    Align_GetDebug(&d);
    snprintf(buf, sizeof(buf), "px=%d,%d q=%d",
             (int)d.dxPx, (int)d.dyPx, (int)d.qRaw);

    return buf;
}

const char *AlnTest_GetOutText(void)
{
    static char  buf[40];
    char         sf[10];
    char         sr[10];
    char         sw[10];
    Align_Dbg_t  d;

    Align_GetDebug(&d);
    AlnTest_RpmStr(d.uF, sf, (uint16_t)sizeof(sf));
    AlnTest_RpmStr(d.uR, sr, (uint16_t)sizeof(sr));
    AlnTest_RpmStr(d.uW, sw, (uint16_t)sizeof(sw));

    snprintf(buf, sizeof(buf), "u=%s,%s,%s w=%d", sf, sr, sw, (int)d.wheelMax);

    return buf;
}

const char *AlnTest_GetStateText(void)
{
    static char buf[40];
    static const char *const stName[6] = { "IDLE", "ARM", "WAIT", "RUN", "DONE", "FAIL" };
    Align_Dbg_t d;

    Align_GetDebug(&d);
    snprintf(buf, sizeof(buf), "%s f=%u %lums K3=%s",
             stName[atState], (unsigned)d.frames, (unsigned long)d.elapsedMs,
             ((atState == AT_ARM) || (atState == AT_RUN) || (atState == AT_WAIT))
                 ? "STOP" : "GO");

    return buf;
}
