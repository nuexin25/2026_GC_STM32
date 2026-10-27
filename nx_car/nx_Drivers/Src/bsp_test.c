/**
 ******************************************************************************
 * @file    bsp_test.c
 * @brief   ģ���Լ���ʵ�֣�����ע�����ͳһ����ҳ�����ַ���SYS/KEY ����
 * @author  nx_car
 * @date    2026-09-07
 ******************************************************************************
 * @note    ��������룺��ģ����Ժ�����OLED_Test �� bsp_oled_ui.c��IMU_Test ��
 *          bsp_imu.c��ͨ�� bsp_test.h ������ע����Ǽ�һ�м��ɽ���? ******************************************************************************
 */
#include "bsp_test.h"
#include "bsp_oled_ui.h"
#include "bsp_debug.h"
#include "bsp_wheel_test.h"
#include "bsp_qr_test.h"
#include "bsp_route_test.h"
#include "bsp_rpi_test.h"
#include "bsp_arm_test.h"

/* ===================== ����ע���?===================== */
/* name ���� OLED ��ʾ����6 �ַ�����δʵ��ģ�� state=NA������ָ��=NULL */
TestItem_t testItems[TEST_ID_MAX] = {
    { "SYS",  TEST_STATE_IDLE, SYS_Test,        NULL,             NULL },
    { "OLED", TEST_STATE_IDLE, OLED_Test,       NULL,             NULL },
    { "KEY",  TEST_STATE_IDLE, KEY_Test,        KEY_Test_Update,  KEY_Test_Stop },
    { "IMU",  TEST_STATE_IDLE, IMU_Test,        IMU_Test_Update,  IMU_Test_Stop },
    { "MOT",  TEST_STATE_IDLE, MOT_Test,        NULL,             MOT_Test_Stop },
    { "CHS",  TEST_STATE_IDLE, CHS_Test,        NULL,             CHS_Test_Stop },
    { "QR",  TEST_STATE_IDLE, QR_Test,         NULL,             QR_Test_Stop },
    { "ROUTE", TEST_STATE_IDLE, ROUTE_Test,      NULL,             ROUTE_Test_Stop },
    { "ARM",  TEST_STATE_IDLE, ARM_Test,        NULL,             ARM_Test_Stop },
    { "RPI",  TEST_STATE_IDLE, RPI_Test,        RPI_Test_Update,  RPI_Test_Stop },
};

/* ===================== �ڲ�״̬ ===================== */
static uint8_t  testActive = 0;            /* ����ҳ�����־��?=�ǲ���ҳ��1=����ҳ */
static uint8_t  testCursor = 0;            /* ��ǰ������ڲ���������?*/
static uint8_t  testScrollOffset = 0;      /* ����ҳ����ƫ�ƣ��ɼ�������ʼ������ */

/* KEY �����ڲ�״̬ */
static uint8_t  keyTestActive = 0;         /* ����ʶ��ģʽ��־ */
static uint32_t keyTestStartTick = 0;      /* ����ʶ����ʼʱ���?*/

/* ===================== �ڲ����� ===================== */

/**
 * @brief  ���¹������ڣ���֤���ʼ���ڿɼ���Χ��? * @param  ��
 * @retval ��
 */
static void Test_UpdateScroll(void)
{
    if (testCursor < testScrollOffset)
    {
        testScrollOffset = testCursor;
    }
    else if (testCursor >= (testScrollOffset + TEST_VISIBLE_ROWS))
    {
        testScrollOffset = testCursor - (TEST_VISIBLE_ROWS - 1U);
    }
}

/* ===================== ��ܺ���?===================== */

/**
 * @brief  ���Կ�ܳ�ʼ��? * @param  ��
 * @retval ��
 * @note   ��ʵ��ģ�飨run!=NULL����λΪ IDLE��δʵ��ģ�鸴λΪ NA
 */
void Test_Init(void)
{
    uint8_t i;
    for (i = 0; i < TEST_ID_MAX; i++)
    {
        if (testItems[i].run != NULL)
        {
            testItems[i].state = TEST_STATE_IDLE;
        }
        else
        {
            testItems[i].state = TEST_STATE_NA;
        }
    }
    testActive = 0;
    testCursor = 0;
    testScrollOffset = 0;
    keyTestActive = 0;
}

/**
 * @brief  ��ѯ��ǰ�Ƿ��ڲ���ҳ
 * @param  ��
 * @retval 1=����ҳ���0=�ǲ���ҳ
 */
uint8_t Test_IsActive(void)
{
    return testActive;
}

/**
 * @brief  �������ҳ����λ��꣬�������л�ҳ�棩
 * @param  ��
 * @retval ��
 */
void Test_Enter(void)
{
    testActive = 1;
    testCursor = 0;
    testScrollOffset = 0;
}

/**
 * @brief  �˳�����ҳ��ֹͣ���� RUN ���������־��
 * @param  ��
 * @retval ��
 */
void Test_Exit(void)
{
    uint8_t i;
    for (i = 0; i < TEST_ID_MAX; i++)
    {
        if (testItems[i].state == TEST_STATE_RUN)
        {
            if (testItems[i].stop != NULL)
            {
                testItems[i].stop();
            }
            testItems[i].state = TEST_STATE_IDLE;
        }
    }
    testActive = 0;
    testCursor = 0;
    testScrollOffset = 0;
    keyTestActive = 0;
}

/**
 * @brief  ����ҳ�ڰ�������
 * @param  key: ���� ID
 * @param  longPress: 1=������0=�̰��������� main ���������������ԣ�
 * @retval ��
 * @note   KEY1=���ơ�KEY2=���ƣ�ѭ������KEY3=����/ֹͣ��KEY4=�˳�������ҳ��
 *          KEY ���� RUN �ڼ����ⰴ����������ʶ��ɹ�? */
void Test_HandleKey(KeyID_t key, uint8_t longPress)
{
    (void)longPress;
    if (key == KEY_NONE)
    {
        return;
    }

    /* KEY ���������ڼ䣺���ⰴ��������"����ʶ��"�ɹ�����ʾ���Ų� PASS�� */
    if (testItems[TEST_ID_KEY].state == TEST_STATE_RUN)
    {
        KEY_Test_Stop();
        testItems[TEST_ID_KEY].state = TEST_STATE_PASS;
        Debug_Log("TEST", "KEY id=%u ok", (unsigned int)key);
        return;
    }

    switch (key)
    {
        case KEY1:   /* ������ƣ�ѭ����?*/
            if (testCursor == 0)
            {
                testCursor = (uint8_t)(TEST_ID_MAX - 1U);
            }
            else
            {
                testCursor--;
            }
            Test_UpdateScroll();
            break;

        case KEY2:   /* ������ƣ�ѭ����?*/
            testCursor = (uint8_t)((testCursor + 1U) % TEST_ID_MAX);
            Test_UpdateScroll();
            break;

        case KEY3:   /* ����/ֹͣ��ǰѡ���� */
            if (testItems[testCursor].state == TEST_STATE_RUN)
            {
                /* RUN �� stop �� IDLE */
                if (testItems[testCursor].stop != NULL)
                {
                    testItems[testCursor].stop();
                }
                testItems[testCursor].state = TEST_STATE_IDLE;
                Debug_Log("TEST", "%s stop", testItems[testCursor].name);
            }
            else if (testItems[testCursor].state == TEST_STATE_IDLE ||
                     testItems[testCursor].state == TEST_STATE_FAIL ||
                     testItems[testCursor].state == TEST_STATE_PASS)
            {
                /* IDLE/FAIL/PASS �� RUN */
                if (testItems[testCursor].run != NULL)
                {
                    testItems[testCursor].state = TEST_STATE_RUN;
                    testItems[testCursor].run();
                }
            }
            /* NA ����� */
            break;

        case KEY4:   /* �˳�����ҳ���ز˵� */
            Test_Exit();
            Display_MenuEnter();
            break;

        default:
            break;
    }
}

/**
 * @brief  ���Կ����ѭ������? * @param  ��
 * @retval ��
 * @note   �������� RUN ����״̬�仯���ϱ�?[TEST] <ģ����> PASS/FAIL
 */
void Test_Update(void)
{
    uint8_t i;
    for (i = 0; i < TEST_ID_MAX; i++)
    {
        if (testItems[i].state == TEST_STATE_RUN && testItems[i].update != NULL)
        {
            TestState_t before = testItems[i].state;
            testItems[i].update();
            if (testItems[i].state != before)
            {
                if (testItems[i].state == TEST_STATE_PASS)
                {
                    Debug_Log("TEST", "%s PASS", testItems[i].name);
                }
                else if (testItems[i].state == TEST_STATE_FAIL)
                {
                    Debug_Log("TEST", "%s FAIL", testItems[i].name);
                }
            }
        }
    }
}

/**
 * @brief  ��ȡ��ǰ�������? * @param  ��
 * @retval ���������?~TEST_ID_MAX-1��
 */
uint8_t Test_GetCursor(void)
{
    return testCursor;
}

/**
 * @brief  ��ȡ����ҳ����ƫ��
 * @param  ��
 * @retval ����ƫ�ƣ�0 ��
 */
uint8_t Test_GetScroll(void)
{
    return testScrollOffset;
}

/* ===================== SYS ϵͳ�Լ� ===================== */

/**
 * @brief  SYS ϵͳ�Լ�������run �ص���
 * @param  ��
 * @retval ��
 * @note   ���?LED_G/B/Y ���ε������� + ����һ�� + BZ_LED ���Ƹ���һ�Σ�
 *          �ڲ� HAL_Delay ���޲���ҳ����ִ�У���ʱ�� < 1s��900ms�����ɹ��� PASS
 */
void SYS_Test(void)
{
    /* ���?LED��PA0/PA1/PA2�����ε��� 200ms ���� */
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, LED_ACTIVE_LEVEL);
    HAL_Delay(TEST_LED_BLINK_MS);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, LED_INACTIVE_LEVEL);

    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, LED_ACTIVE_LEVEL);
    HAL_Delay(TEST_LED_BLINK_MS);
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, LED_INACTIVE_LEVEL);

    HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, LED_ACTIVE_LEVEL);
    HAL_Delay(TEST_LED_BLINK_MS);
    HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, LED_INACTIVE_LEVEL);

    /* ���������� 1 ������������ */
    BSP_Beep(1, 100, 0);

    /* BZ_LED ���Ƹ���һ�Σ������� 100ms�����Ƶ�����ʱ�� < 1s�� */
    BSP_LED_Set(BZ_LED1, 1); HAL_Delay(TEST_LED_BLINK_MS / 2U); BSP_LED_Set(BZ_LED1, 0);
    BSP_LED_Set(BZ_LED2, 1); HAL_Delay(TEST_LED_BLINK_MS / 2U); BSP_LED_Set(BZ_LED2, 0);
    BSP_LED_Set(BZ_LED3, 1); HAL_Delay(TEST_LED_BLINK_MS / 2U); BSP_LED_Set(BZ_LED3, 0);

    /* ϵͳ�Լ�ɹ�����?FAIL ��֧������ʾ�� */
    testItems[TEST_ID_SYS].state = TEST_STATE_PASS;
    Debug_Log("TEST", "SYS self-test ok");
}

/* ===================== KEY ����ʶ���Լ� ===================== */

/**
 * @brief  KEY ����ʶ�����������run �ص���
 * @param  ��
 * @retval ��
 * @note   ���밴��ʶ��ģʽ���ȴ����ⰴ�������������?Test_HandleKey ����ɣ�? *          ��ʱ����ڱ�ģ��?update �����? */
void KEY_Test(void)
{
    keyTestActive = 1;
    keyTestStartTick = HAL_GetTick();
    Debug_Log("TEST", "KEY press any key...");
}

/**
 * @brief  KEY ����ʶ�����ˢ�£�update �ص���
 * @param  ��
 * @retval ��
 * @note   ����ⳬʱ��?0s �ް��� �� FAIL������������ PASS �� Test_HandleKey �д���
 */
void KEY_Test_Update(void)
{
    if (!keyTestActive)
    {
        return;
    }

    if ((HAL_GetTick() - keyTestStartTick) >= TEST_TIMEOUT_MS)
    {
        keyTestActive = 0;
        testItems[TEST_ID_KEY].state = TEST_STATE_FAIL;
    }
}

/**
 * @brief  KEY ����ʶ�����ֹͣ��stop �ص���
 * @param  ��
 * @retval ��
 */
void KEY_Test_Stop(void)
{
    keyTestActive = 0;
}

/* ===================== MOT motor single-wheel test (P3) ===================== */

/**
  * @brief  MOT test run callback: enter single-wheel jog page.
  * @param  none
  * @retval none
  * @note   Page switch to PAGE_WHEEL is handled inside WheelTest_Enter().
  */
void MOT_Test(void)
{
    WheelTest_Enter();
}

/**
  * @brief  MOT test stop callback: exit single-wheel jog page (stop all motors).
  * @param  none
  * @retval none
  */
void MOT_Test_Stop(void)
{
    WheelTest_Exit();
}

