/**
 ******************************************************************************
 * @file    bsp_key.c
 * @brief   按键扫描驱动实现：消抖、短按、长按检测
 * @author  nx_car
 * @date    2026-09-05
 ******************************************************************************
 * @note   扫描流程：
 *   1. 每 KEY_SCAN_PERIOD_MS 读取一次 5 个按键电平；
 *   2. 消抖：连续 KEY_DEBOUNCE_COUNT 次同电平才确认状态变化；
 *   3. 确认按下时记录起始时间；确认释放时若未触发长按则上报短按；
 *   4. 按住达到 KEY_LONG_PRESS_MS 时置长按标志，由 Key_GetLongPress 读取。
 ******************************************************************************
 */
#include "bsp_key.h"
#include "bsp_debug.h"

/* 按键引脚表（按下为低电平） */
static const uint16_t keyPinTable[5] = {
    KEY1_Pin,   /* PE7 */
    KEY2_Pin,   /* PE8 */
    KEY3_Pin,   /* PE9 */
    KEY4_Pin,   /* PE10 */
    KEY5_Pin    /* PE11 */
};

/* 每个按键的状态机 */
typedef struct {
    GPIO_PinState lastRead;          /* 上次原始读取电平 */
    GPIO_PinState stableState;       /* 消抖确认后的稳定状态 */
    uint8_t       debounceCnt;       /* 连续同电平计数 */
    uint32_t      pressStartTime;    /* 确认按下的时间戳 */
    uint8_t       longPressTriggered;/* 长按是否已触发 */
} KeyState_t;

static KeyState_t keyState[5];
static uint32_t   lastScanTick = 0;
static KeyID_t    longPressKey = KEY_NONE;   /* 待读取的长按按键 */

/* ===================== 公开接口 ===================== */

/**
 * @brief  按键初始化
 */
void Key_Init(void)
{
    for (uint8_t i = 0; i < 5; i++)
    {
        keyState[i].lastRead = KEY_RELEASED_LEVEL;
        keyState[i].stableState = KEY_RELEASED_LEVEL;
        keyState[i].debounceCnt = 0;
        keyState[i].pressStartTime = 0;
        keyState[i].longPressTriggered = 0;
    }
    longPressKey = KEY_NONE;
    lastScanTick = HAL_GetTick();
}

/**
 * @brief  按键扫描（主循环周期调用）
 */
KeyID_t Key_Scan(void)
{
    uint32_t now = HAL_GetTick();

    /* 按扫描周期节流 */
    if ((now - lastScanTick) < KEY_SCAN_PERIOD_MS)
    {
        return KEY_NONE;
    }
    lastScanTick = now;

    KeyID_t shortKey = KEY_NONE;

    /* 按优先级 KEY1~KEY5 扫描，同一时刻仅响应一个 */
    for (uint8_t i = 0; i < 5; i++)
    {
        GPIO_PinState read = HAL_GPIO_ReadPin(GPIOE, keyPinTable[i]);

        /* 消抖计数 */
        if (read == keyState[i].lastRead)
        {
            if (keyState[i].debounceCnt < KEY_DEBOUNCE_COUNT)
            {
                keyState[i].debounceCnt++;
            }
        }
        else
        {
            keyState[i].lastRead = read;
            keyState[i].debounceCnt = 1;
        }

        /* 达到消抖确认次数，判定状态变化 */
        if (keyState[i].debounceCnt >= KEY_DEBOUNCE_COUNT)
        {
            if (read != keyState[i].stableState)
            {
                keyState[i].stableState = read;

                if (read == KEY_PRESSED_LEVEL)
                {
                    /* 确认按下 */
                    keyState[i].pressStartTime = now;
                    keyState[i].longPressTriggered = 0;
                }
                else
                {
                    /* 确认释放 */
                    if (keyState[i].longPressTriggered == 0)
                    {
                        /* 未触发长按，即为短按 */
                        shortKey = (KeyID_t)(i + 1);
                    }
                }
            }
            else if (read == KEY_PRESSED_LEVEL && keyState[i].longPressTriggered == 0)
            {
                /* 持续按下，检测长按 */
                if ((now - keyState[i].pressStartTime) >= KEY_LONG_PRESS_MS)
                {
                    keyState[i].longPressTriggered = 1;
                    longPressKey = (KeyID_t)(i + 1);
                }
            }
        }

        /* 已找到短按按键，跳过后续（保证优先级） */
        if (shortKey != KEY_NONE)
        {
            break;
        }
    }

    return shortKey;
}

/**
 * @brief  获取长按事件
 */
KeyID_t Key_GetLongPress(void)
{
    KeyID_t key = longPressKey;
    longPressKey = KEY_NONE;
    return key;
}
