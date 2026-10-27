/**
 ******************************************************************************
 * @file    bsp_test.h
 * @brief   ģ���Լ���ͷ�ļ�������ע���?+ ͳһ����ҳ�����ַ�
 * @author  nx_car
 * @date    2026-09-07
 ******************************************************************************
 * @note    ����չ�������룺����ÿ����һ��ģ�飬ֻ���ڸ�ģ���ļ�д���Ժ���
 *          ���� bsp_test.c ע����Ǽ�һ�У���ܴ�������Ķ���? *          ����ҳ PAGE_TEST �ɳ��� KEY1 ����/�˳���ҳ�ڰ����ɱ���ܶ�ռ��? ******************************************************************************
 */
#ifndef __BSP_TEST_H
#define __BSP_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bsp_key.h"

/* ===================== ������ ID ö�� ===================== */
typedef enum {
    TEST_ID_SYS = 0,   /* ϵͳ�Լ죺���?LED + ������ + BZ_LED */
    TEST_ID_OLED,      /* OLED ��Ļ�Լ� */
    TEST_ID_KEY,       /* ����ʶ���Լ� */
    TEST_ID_IMU,       /* �����ǣ�P2�� */
    TEST_ID_MOT,       /* motor single-wheel test (P3) */
    TEST_ID_CHS,       /* chassis motion test (P4: position mode + mecanum) */
    TEST_ID_QR,        /* ɨ�루P5��Ԥ���� */
    TEST_ID_ROUTE,     /* route run test (P7): waypoint sequence executor */
    TEST_ID_SRV,       /* �����P8��Ԥ���� */
    TEST_ID_RPI,       /* raspberry pi vision link (P6) */
    TEST_ID_MAX
} TestID_t;

/* ===================== ����״̬ö�� ===================== */
typedef enum {
    TEST_STATE_IDLE = 0,   /* δ���� */
    TEST_STATE_RUN,        /* ������ */
    TEST_STATE_PASS,       /* ͨ�� */
    TEST_STATE_FAIL,       /* ʧ�� */
    TEST_STATE_NA          /* δʵ��/δ��װ��ռλ�� */
} TestState_t;

/* ===================== ������ṹ��?===================== */
typedef struct {
    const char *name;         /* ģ������OLED ��ʾ����6 �ַ��� */
    TestState_t state;        /* ��ǰ״̬ */
    void (*run)(void);        /* �������ԣ������������� state=RUN���������������?*/
    void (*update)(void);     /* ����ˢ�£�RUN �ڼ���ѭ�����ã�������/��ʱ/ʧ�� */
    void (*stop)(void);       /* ֹͣ���ָ���ȫ״̬���� state=IDLE */
} TestItem_t;

/* ===================== �ɵ����� ===================== */
#define TEST_TIMEOUT_MS       10000U  /* ���Գ�ʱ��ֵ����λ ms���ɵ���Χ 2000~60000 */
#define TEST_LED_BLINK_MS     200U    /* SYS �Լ� LED ����ʱ������λ ms���ɵ���Χ 50~1000 */
#define TEST_OLED_FLASH_MS    100U    /* OLED �Լ�ˢ������/�ף�ʱ������λ ms���ɵ���Χ 20~500 */
#define TEST_IMU_SAMPLE_CNT   3U      /* IMU �Լ�������Ч�����������ɵ���Χ 1~10 */
#define TEST_VISIBLE_ROWS     4U      /* ����ҳ�ɼ�ģ�������������壬ÿ�� 2 ҳ�� */

/* ===================== ����ע���?===================== */
/* ע������飨state ����ʱ���޸ģ��ʷ� const������ bsp_test.c �ж��� */
extern TestItem_t testItems[TEST_ID_MAX];

/* ===================== ��ܺ�������?===================== */

/**
 * @brief  ���Կ�ܳ�ʼ�������в�����?state ��λΪ IDLE/NA
 * @param  ��
 * @retval ��
 * @note   ����ѭ��ǰ����һ�Σ�NA ռλ����ע�����ֵ�������˴�������? */
void Test_Init(void);

/**
 * @brief  ��ѯ��ǰ�Ƿ��ڲ���ҳ
 * @param  ��
 * @retval 1=����ҳ���0=�ǲ���ҳ
 * @note   �� main �����ַ��жϣ�����ҳ�ڰ����ɿ�ܶ��? */
uint8_t Test_IsActive(void);

/**
 * @brief  �������ҳ����λ��굽 0���������л�ҳ�棩
 * @param  ��
 * @retval ��
 * @note   ҳ���л��� main �� Display_SetPage(PAGE_TEST) ���? */
void Test_Enter(void);

/**
 * @brief  �˳�����ҳ��ֹͣ���� RUN ���������־��
 * @param  ��
 * @retval ��
 * @note   ҳ���л��� main �� Display_SetPage(PAGE_DATA) ���? */
void Test_Exit(void);

/**
 * @brief  ����ҳ�ڰ����������� main �ڲ���ҳ����ʱ�ַ���
 * @param  key: ���� ID��KEY1~KEY5��
 * @param  longPress: 1=������0=�̰�������ܽ������̰���������?main ������
 * @retval ��
 * @note   KEY1=���ơ�KEY2=���ƣ�ѭ������KEY3=����/ֹͣ��KEY4=�˳�������ҳ
 */
void Test_HandleKey(KeyID_t key, uint8_t longPress);

/**
 * @brief  ���Կ����ѭ�����ģ����ȵ��?RUN �ͬ�������?OLED
 * @param  ��
 * @retval ��
 * @note   ����ѭ�����ڵ��ã��ڲ��� HAL_Delay
 */
void Test_Update(void);

/**
 * @brief  ��ȡ��ǰ������ڲ���������? * @param  ��
 * @retval ���������?~TEST_ID_MAX-1��
 */
uint8_t Test_GetCursor(void);

/**
 * @brief  ��ȡ����ҳ����ƫ�ƣ��ɼ�������ʼ������
 * @param  ��
 * @retval ����ƫ�ƣ�0 ��
 */
uint8_t Test_GetScroll(void);

/* ===================== ��ģ����Ժ�������?===================== */

/* SYS ϵͳ�Լ죨bsp_test.c�� */
void SYS_Test(void);

/* OLED �Լ죨bsp_oled_ui.c�� */
void OLED_Test(void);

/* KEY ����ʶ���Լ죨bsp_test.c�� */
void KEY_Test(void);
void KEY_Test_Update(void);
void KEY_Test_Stop(void);

/* IMU �������Լ죨bsp_imu.c�� */
void IMU_Test(void);
void IMU_Test_Update(void);
void IMU_Test_Stop(void);

/* MOT motor single-wheel test (bsp_wheel_test.c) */
void MOT_Test(void);
void MOT_Test_Stop(void);

/* CHS chassis motion test (bsp_chassis_test.c) */
void CHS_Test(void);
void CHS_Test_Stop(void);

/* QR scan test (bsp_qr_test.c) */
void QR_Test(void);
void QR_Test_Stop(void);

/* ROUTE route run test (bsp_route_test.c) */
void ROUTE_Test(void);
void ROUTE_Test_Stop(void);

/* RPi link test (bsp_rpi_test.c, P6) */
void RPI_Test(void);
void RPI_Test_Update(void);
void RPI_Test_Stop(void);

/* ARM servo/arm actuator test (bsp_arm_test.c, P8) */
void ARM_Test(void);
void ARM_Test_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_TEST_H */
