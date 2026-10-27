/**
 ******************************************************************************
 * @file    bsp_imu.h
 * @brief   IMU Ӧ�ò�ͷ�ļ���HWT101CT ���������Ƿ�װ
 * @author  nx_car
 * @date    2026-09-07
 ******************************************************************************
 * @note    �ײ�����Ϊ hwt101ct.c��ά�عٷ�������������Э�� bug����
 *          (1) ���ٶȻ���ϵ�� 400��/s���� HWT101 �� 2000��/s����
 *          (2) ���ٶ�֡ Wz �ֽ�ƫ�ƣ�λ���ֽ� 6-7����
 *          USART2 = PD5(TX)/PD6(RX)���жϽ��գ�Ĭ�� 9600-8-N-1��
 ******************************************************************************
 */
#ifndef __BSP_IMU_H
#define __BSP_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* ===================== �ɵ����� ===================== */
/* ������ͳһ�꣺ͬʱ���� STM32 USART2 ��������ģ�����ನ���ʡ�
 * �Ķ�����ͬ���޸�ģ��ನ���ʣ��������ֲ� P2 �޸Ĳ��裩�� */
#define HWT101CT_UART_BAUD      115200U   /* Ĭ�� 9600���ɸ� 4800/19200/38400/57600/115200/230400 */

/* ===== IMU output-rate config (GBK file: English comments only) ==============
 * The HWT101CT module's factory default output rate is 10Hz -> the yaw value the
 * application sees can be up to 100ms old. That is too slow for the P7 route
 * angle loop (it over-corrects because it re-reads a stale yaw).
 * IMU_CFG_RATE_HZ: requested auto-report rate, written to the module at boot
 *   (0 = do not touch the module, keep factory setting).
 *   Bandwidth (9600-8-N-1): one 11-byte frame ~= 11.5ms, so
 *     10Hz = 23% , 20Hz = 46%  of the link  -> always safe
 *     50Hz = 115% IF the module sends both 0x52 (gyro) and 0x53 (angle) frames;
 *     50Hz =  57% if it only sends 0x53 -> fine.
 *   Because that is unknown up-front, IMU_CFG_RATE_HZ > 20 is applied together
 *   with a BOOT-TIME PROBE: we count the frames that actually arrive during
 *   IMU_CFG_RATE_PROBE_MS; if fewer than IMU_CFG_RATE_PROBE_PCT% of the
 *   requested rate show up, we automatically drop back to
 *   IMU_CFG_RATE_FALLBACK_HZ and log it. So asking for 50Hz can never leave the
 *   link saturated unknowingly (worst case: boot costs ~0.6s and we end at 20Hz).
 *   To go to 100Hz raise the baud first (HWT101CT_UART_BAUD=115200 + the module
 *   side, see tuning manual P2 3.1), otherwise the link cannot carry it.
 * Allowed values: 10 / 20 / 50 / 100 (Hz); anything else -> fallback.
 * NOTE: the vendor driver writes the register over UART with ~300ms of HAL_Delay,
 * so this is only done once in IMU_Init (boot), never inside the main loop. */
#define IMU_CFG_RATE_HZ          100U    /* requested rate (0 = leave module alone)
                                          ★【2026-10-24】0 -> 100：出厂 10Hz 时 yaw 每 100ms
                                          才更新一次，速度模式航向环（20ms）完全跟不上，
                                          现场表现就是"转弯冲过头 + 出弯蛇形"。
                                          链路已是 115200，100Hz 足够；探测不过会自动回落到 20Hz。 */
#define IMU_CFG_RATE_FALLBACK_HZ 20U    /* used when the boot probe fails */
#define IMU_CFG_RATE_PROBE_MS    300U   /* probe window, ms */
#define IMU_CFG_RATE_PROBE_PCT   60U    /* require >= this % of requested rate */

#define IMU_TIMEOUT_MS          500U    /* �������ʶȳ�ʱ��ֵ����λ ms���ɵ���Χ 200~5000 */
#define IMU_REPORT_PERIOD_MS    200U    /* yaw/gz �ϱ����ڣ���λ ms���ɵ���Χ 50~1000 */
#define IMU_DEBUG_RAW           0       /* ԭʼ֡ hex ��ӡĬ�Ͽ��أ�0=�أ�1=����Ҳ���� KEY4 ����ʱ�л��� */

/* ===================== �������� ===================== */

/**
 * @brief  IMU ��ʼ�������� USART2 �����ʡ���ʼ������������жϽ������ֽ�
 * @param  ��
 * @retval ��
 * @note   ����ѭ��ǰ����һ�Σ�Ĭ�ϲ�������дģ��ನ���ʣ����ֳ��� 9600��
 */
void IMU_Init(void);

/**
 * @brief  IMU ��ѭ�����ģ�����֡��ԭʼ֡��ӡ�����ʶȼ�⡢�����ϱ�
 * @param  ��
 * @retval ��
 * @note   ����ѭ�����ڵ��ã��������ж��е��ã��ڲ��� HAL_Delay
 */
void IMU_Update(void);

/**
 * @brief  ��ȡ��ǰ Z ��Ƕȣ�yaw��
 * @param  ��
 * @retval Z ��Ƕȣ���λ�ȣ���Χ ��180�㣩
 */
float IMU_GetYaw(void);

/**
 * @brief  一次性"静态置角"：让 IMU_GetYaw() 从此刻起读 refDeg
 * @param  refDeg: 期望的当前朝向（deg，-180~180）
 * @retval 1=已置角；0=被拒绝（小车仍在转，|gz| > 30 deg/s）
 * @note   只在**小车静止**时调用（|gz| > 30 deg/s 会被拒绝并打日志）。
 *         只加一个固定偏移，**不钉住、不冻结**：之后照常积分/显示新角度，
 *         所以后续绝对坐标全部建立在新基准上。
 *         任务点用法：TRAY PICK 对位成功=0，STORE BACK 开始=180，
 *         ROUGH PLACE / ROUGH STACK 对位成功=90。
 */
uint8_t IMU_StaticZeroTo(float refDeg);   /* 返回 1=已置角 / 0=车还在转被拒绝 */

/**
 * @brief  ��ȡ��ǰ Z ����ٶ�
 * @param  ��
 * @retval Z ����ٶȣ���λ ��/s����Χ ��400��/s��
 */
float IMU_GetGyroZ(void);

/**
 * @brief  Z ��Ƕȹ��㣨����+����+���棩
 * @param  ��
 * @retval ��
 * @note   �ڲ��� HWT101CT_ResetYaw���� HAL_Delay���������ڳ�ʼ��/�����¼��е��ã�
 *         ��ֹ����ѭ������
 */
void IMU_ResetYaw(void);

/**
 * @brief  ��ѯ IMU ����״̬
 * @param  ��
 * @retval 1=���ߣ�IMU_TIMEOUT_MS ������֡����0=����
 */
uint8_t IMU_IsOnline(void);

/**
 * @brief  �ֶ��޸Ĳ����ʷ�װ����ģ���Ҳ��Ϊ HWT101CT_UART_BAUD ��Ӧö��ֵ��
 * @param  ��
 * @retval ��
 * @note   Ĭ�ϳ�ʼ�����̡��������ã������û���ȷ��Ҫ���߻ش�����ʱ���ֲ��ֶ����á�
 *         �ڲ��� HAL_Delay������+д�Ĵ���+���棩����ֹ����ѭ������
 */
void IMU_SetBaudRate(void);

/**
 * @brief  ����ʱ�л�ԭʼ֡ hex ��ӡ���أ�KEY4 �̰�������
 * @param  ��
 * @retval ��
 * @note   �� IMU_DEBUG_RAW �����������ʱ����������Ч
 */
void IMU_ToggleDebugRaw(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_IMU_H */
