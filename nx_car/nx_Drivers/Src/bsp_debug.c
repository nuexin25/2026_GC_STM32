/**
 ******************************************************************************
 * @file    bsp_debug.c
 * @brief   调试系统驱动实现：printf 重定向、统一日志、LED 流水、PWM 蜂鸣器、板载 BZ_LED
 * @author  nx_car
 * @date    2026-09-05
 ******************************************************************************
 * @note    printf 重定向采用重写 fputc 的方式。工程已启用 MicroLIB
 *         （uvprojx 中 <useUlib>1</useUlib>），无需补 __use_no_semihosting 桩。
 *         蜂鸣器 WHT101 为无源器件，PA7=TIM3_CH2 输出 ≈2.73kHz PWM 方波驱动；
 *         蜂鸣时序采用非阻塞状态机（BSP_Beep_Update），禁止 HAL_Delay。
 ******************************************************************************
 */
#include "bsp_debug.h"
#include "usart.h"
#include "tim.h"
#include <stdarg.h>

/* ===================== 内部状态变量 ===================== */
/* 外接 LED 流水状态 */
static LED_FlowIndex_t ledFlowIndex = LED_FLOW_G;   /* 当前点亮的 LED 索引 */
static uint32_t ledFlowLastTick = 0;                /* 上次切换的 SysTick 时间戳 */

/* 心跳状态 */
static uint32_t heartbeatLastTick = 0;              /* 上次心跳输出的 SysTick 时间戳 */
static uint8_t  bzLed1State = 0;                    /* BZ_LED1 当前电平（0=灭，1=亮） */

/* 外接 LED 引脚表：按 LED_FlowIndex_t 顺序排列，统一挂在 GPIOA 上 */
static const uint16_t ledPinTable[LED_FLOW_MAX] = {
    LED_G_Pin,   /* PA0 绿色 */
    LED_B_Pin,   /* PA1 蓝色 */
    LED_Y_Pin    /* PA2 黄色 */
};

/* 板载 BZ_LED 引脚表：PE12/PE13/PE14，统一挂在 GPIOE 上 */
static const uint16_t bzLedPinTable[BZ_LED_MAX] = {
    BZ_LED1_Pin,  /* PE12 心跳 */
    BZ_LED2_Pin,  /* PE13 蜂鸣同步 */
    BZ_LED3_Pin   /* PE14 预留 */
};

/* 蜂鸣器非阻塞状态机 */
typedef enum {
    BEEP_IDLE = 0,   /* 空闲：无鸣叫 */
    BEEP_ON,         /* 鸣响段：PWM 输出中 */
    BEEP_OFF         /* 间歇段：PWM 停止 */
} BeepPhase_t;

static struct {
    uint8_t     times;        /* 剩余鸣叫次数；0xFF 表示持续循环模式 */
    uint8_t     continuous;   /* 1=持续断续鸣叫（times=0 传入） */
    uint16_t    onMs;         /* 单次鸣响时长 ms */
    uint16_t    offMs;        /* 两次鸣响间隔 ms */
    BeepPhase_t phase;        /* 当前阶段 */
    uint32_t    phaseStart;   /* 当前阶段起始时间戳 */
    uint8_t     running;      /* 1=鸣叫序列进行中 */
} beepState = {0};

/**
 * @brief  板载 BZ_LED 写电平（内部按低电平点亮极性转换）
 * @param  id: 板载 LED 编号
 * @param  on: 1=点亮，0=熄灭
 * @retval 无
 */
static void BSP_BZLED_Write(uint8_t id, uint8_t on)
{
    if (id >= BZ_LED_MAX) return;
    GPIO_PinState lvl = on ? BZ_LED_ACTIVE_LEVEL : BZ_LED_INACTIVE_LEVEL;
    HAL_GPIO_WritePin(GPIOE, bzLedPinTable[id], lvl);
}

/**
 * @brief  启动蜂鸣器 PWM 输出并点亮 BZ_LED2
 * @param  无
 * @retval 无
 */
static void Beep_PwmStart(void)
{
    HAL_TIM_PWM_Start(&BEEP_TIM, BEEP_TIM_CHANNEL);
    BSP_BZLED_Write(BZ_LED2, 1);
}

/**
 * @brief  停止蜂鸣器 PWM 输出并熄灭 BZ_LED2
 * @param  无
 * @retval 无
 */
static void Beep_PwmStop(void)
{
    HAL_TIM_PWM_Stop(&BEEP_TIM, BEEP_TIM_CHANNEL);
    BSP_BZLED_Write(BZ_LED2, 0);
}

/**
 * @brief  调试系统初始化
 * @param  无
 * @retval 无
 * @note   熄灭全部 LED（外接+板载），按 BEEP_DUTY_PERCENT 重设 CCR，上电蜂鸣一次
 */
void BSP_Debug_Init(void)
{
    /* 熄灭全部外接 LED */
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, LED_INACTIVE_LEVEL);
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, LED_INACTIVE_LEVEL);
    HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, LED_INACTIVE_LEVEL);

    /* 熄灭全部板载 BZ_LED（上电应全灭） */
    BSP_BZLED_Write(BZ_LED1, 0);
    BSP_BZLED_Write(BZ_LED2, 0);
    BSP_BZLED_Write(BZ_LED3, 0);

    /* 按 BEEP_DUTY_PERCENT 重设 PWM 占空比（CCR = (ARR+1)*DUTY/100） */
    {
        uint32_t arr = (uint32_t)BEEP_TIM.Init.Period + 1U;
        uint32_t ccr = (arr * (uint32_t)BEEP_DUTY_PERCENT) / 100U;
        __HAL_TIM_SET_COMPARE(&BEEP_TIM, BEEP_TIM_CHANNEL, ccr);
    }

    /* 上电蜂鸣一声（非阻塞，由 BSP_Beep_Update 驱动） */
    BSP_Beep(1, BEEP_ON_MS, 0);

    /* 记录起始节拍 */
    ledFlowLastTick = HAL_GetTick();
    heartbeatLastTick = HAL_GetTick();
}

/**
 * @brief  printf 重定向到 UART5
 * @param  ch: 待发送字符
 * @param  f:  文件指针（未使用，仅为匹配 fputc 原型）
 * @retval 已发送的字符
 * @note   阻塞发送，超时 100ms；UART5 为调试串口，115200-8-N-1
 */
int fputc(int ch, FILE *f)
{
    (void)f;
    uint8_t data = (uint8_t)ch;
    HAL_UART_Transmit(&huart5, &data, 1, 100);
    return ch;
}

/**
 * @brief  统一日志输出接口
 * @param  tag: 日志标签（3~4 字符）
 * @param  fmt: printf 风格格式串
 * @retval 无
 * @note   输出格式：[TAG] 内容\r\n
 */
void Debug_Log(const char *tag, const char *fmt, ...)
{
    va_list args;
    printf("[%s] ", tag);
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\r\n");
}

/**
 * @brief  蜂鸣器鸣叫配置（非阻塞，立即返回）
 * @param  times:  鸣叫次数；0=持续断续鸣叫（循环模式，供故障报警），>0 鸣叫 times 次
 * @param  on_ms:  每次鸣响时长，单位 ms
 * @param  off_ms: 相邻两次鸣叫的间隔时长，单位 ms
 * @retval 无
 * @note   非阻塞：仅设置模式并启动节拍，实际 PWM 启停由 BSP_Beep_Update() 管理；
 *         持续模式须用 BSP_Beep_Stop() 停止
 */
void BSP_Beep(uint8_t times, uint16_t on_ms, uint16_t off_ms)
{
    if (on_ms == 0U)
    {
        return;   /* 鸣响时长为 0，不鸣叫 */
    }

    beepState.onMs       = on_ms;
    beepState.offMs      = off_ms;
    beepState.phase      = BEEP_ON;
    beepState.phaseStart = HAL_GetTick();
    beepState.running    = 1U;

    if (times == 0U)
    {
        beepState.continuous = 1U;
        beepState.times      = 0xFFU;   /* 持续模式标记 */
    }
    else
    {
        beepState.continuous = 0U;
        beepState.times      = times;
    }

    /* 立即启动 PWM（鸣响段） */
    Beep_PwmStart();
}

/**
 * @brief  立即停止蜂鸣器（静音）
 * @param  无
 * @retval 无
 */
void BSP_Beep_Stop(void)
{
    beepState.running    = 0U;
    beepState.phase      = BEEP_IDLE;
    beepState.continuous = 0U;
    Beep_PwmStop();
}

/**
 * @brief  蜂鸣器非阻塞节拍更新（主循环周期调用）
 * @param  无
 * @retval 无
 * @note   基于 HAL_GetTick() 管理 on/off 时序，同步控制 BZ_LED2
 */
void BSP_Beep_Update(void)
{
    if (beepState.running == 0U)
    {
        return;
    }

    uint32_t now = HAL_GetTick();

    if (beepState.phase == BEEP_ON)
    {
        if ((now - beepState.phaseStart) >= beepState.onMs)
        {
            if (beepState.offMs == 0U)
            {
                /* 无间歇：连续鸣响；非持续模式则计数 */
                if (beepState.continuous == 0U)
                {
                    if (beepState.times > 0U)
                    {
                        beepState.times--;
                    }
                    if (beepState.times == 0U)
                    {
                        BSP_Beep_Stop();
                        return;
                    }
                }
                beepState.phaseStart = now;   /* 保持 ON，重置计时 */
            }
            else
            {
                /* 进入间歇段 */
                Beep_PwmStop();
                if (beepState.continuous == 0U)
                {
                    if (beepState.times > 0U)
                    {
                        beepState.times--;
                    }
                    if (beepState.times == 0U)
                    {
                        beepState.running = 0U;
                        beepState.phase   = BEEP_IDLE;
                        return;
                    }
                }
                beepState.phase      = BEEP_OFF;
                beepState.phaseStart = now;
            }
        }
    }
    else if (beepState.phase == BEEP_OFF)
    {
        if ((now - beepState.phaseStart) >= beepState.offMs)
        {
            /* 间歇结束，进入下一次鸣响 */
            Beep_PwmStart();
            beepState.phase      = BEEP_ON;
            beepState.phaseStart = now;
        }
    }
}

/**
 * @brief  板载 LED 通用设置接口
 * @param  id: 板载 LED 编号（BZ_LED1~BZ_LED3）
 * @param  on: 1=点亮，0=熄灭
 * @retval 无
 * @note   BZ_LED1/BZ_LED2 由系统自动管理，BZ_LED3 预留供外部使用
 */
void BSP_LED_Set(uint8_t id, uint8_t on)
{
    BSP_BZLED_Write(id, on);
}

/**
 * @brief  外接 LED 流水灯非阻塞更新
 * @param  无
 * @retval 无
 * @note   每 LED_FLOW_PERIOD_MS 切换一个 LED，循环顺序 G→B→Y→G...
 */
void BSP_LED_Flow_Update(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - ledFlowLastTick) >= LED_FLOW_PERIOD_MS)
    {
        ledFlowLastTick = now;

        /* 先熄灭全部外接 LED，再点亮当前索引对应的 LED */
        HAL_GPIO_WritePin(GPIOA, LED_G_Pin | LED_B_Pin | LED_Y_Pin, LED_INACTIVE_LEVEL);
        HAL_GPIO_WritePin(GPIOA, ledPinTable[ledFlowIndex], LED_ACTIVE_LEVEL);

        /* 切换到下一个 LED */
        ledFlowIndex = (LED_FlowIndex_t)((ledFlowIndex + 1U) % LED_FLOW_MAX);
    }
}

/**
 * @brief  心跳日志非阻塞更新
 * @param  无
 * @retval 无
 * @note   每 HEARTBEAT_PERIOD_MS 输出一条 [SYS] alive, t=xxxxx ms，同时翻转 BZ_LED1
 */
void BSP_Heartbeat_Update(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - heartbeatLastTick) >= HEARTBEAT_PERIOD_MS)
    {
        heartbeatLastTick = now;
        Debug_Log("SYS", "alive, t=%lu ms", (unsigned long)now);

        /* BZ_LED1 心跳翻转，与 [SYS] alive 同步 */
        bzLed1State ^= 1U;
        BSP_BZLED_Write(BZ_LED1, bzLed1State);
    }
}
