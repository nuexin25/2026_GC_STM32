/**
 ******************************************************************************
 * @file    bsp_imu.c
 * @brief   IMU application layer (HWT101CT): UART frame parse, online detect, report
 * @author  nx_car
 * @date    2026-09-07
 ******************************************************************************
 * @note    USART2 RX interrupt only moves one byte into the HWT101CT state machine;
 *          frame parse / freshness check / periodic report run in IMU_Update() on the
 *          main loop (no HAL_Delay here).
 *          On UART error (ORE/FE/NE/PE) HAL does not call RxCpltCallback, so the RX IT
 *          must be re-armed in HAL_UART_ErrorCallback for USART2 as well -- otherwise
 *          the yaw freezes and every heading-dependent feature (odometry rotation,
 *          route planning) silently goes wrong.
 ******************************************************************************
 */
#include "bsp_imu.h"
#include "hwt101ct.h"
#include "usart.h"
#include "bsp_debug.h"
#include "bsp_oled_ui.h"
#include "bsp_test.h"
#include "bsp_qr.h"
#include "bsp_rpi.h"
#include "bsp_servo.h"
#include <stdio.h>

/* ===================== �ڲ�״̬ ===================== */
static HWT101CT_HandleTypeDef imuDev;         /* HWT101CT �������� */
static uint8_t  imuRxByte = 0;                /* �жϽ��յ��ֽڻ��� */

static float    imuYaw = 0.0f;                /* Z ��Ƕȣ��ȣ���Χ ��180�㣩 */
static float    imuYawOffset = 0.0f;          /* one-shot static-zero offset (deg), see IMU_StaticZeroTo() */
#if (IMU_CFG_RATE_HZ != 0U)
static uint8_t  imuCfgRateHz = 10U;           /* report rate actually in use (Hz) */
#endif
static uint32_t imuErrCnt = 0;             /* USART2 RX error count (ORE/FE/NE/PE) */
static uint32_t imuCntGyro = 0;             /* parsed 0x52 (gyro) frames */
static uint32_t imuCntAngle = 0;            /* parsed 0x53 (angle/yaw) frames */
static uint32_t imuCntYawChg = 0;           /* times the reported yaw value changed */
static float    imuGyroZ = 0.0f;              /* Z ����ٶȣ���/s����Χ ��400��/s�� */
static uint8_t  imuOnline = 0;                /* ���߱�־��0=���ߣ�1=���� */
static uint8_t  imuDebugRaw = IMU_DEBUG_RAW;  /* ԭʼ֡��ӡ����ʱ���� */
static uint32_t imuLastFrameTick = 0;         /* ���һ����Ч֡ʱ��� */
static uint32_t imuLastReportTick = 0;
static uint8_t  imuTestSampleCnt = 0;      /* IMU �Լ��Ѳ������� */
static uint32_t imuTestStartTick = 0;      /* IMU �Լ���ʼʱ��� */
static uint32_t imuTestLastSampleTick = 0; /* IMU �Լ��ϴβ���ʱ��� */        /* ���һ���ϱ�ʱ��� */

/* ===================== HAL �ص������������壩 ===================== */

/**
 * @brief  UART ������ɻص������� HAL �����壩
 * @param  huart: UART ���
 * @retval ��
 * @note   ������ USART2��IMU�������ֽ��� HWT101CT ״̬���������������������ֽڽ��ա�
 *         �ж���ֻ���ֽڣ������ػ�
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        HWT101CT_UART_RxHandler(&imuDev, imuRxByte);
        HAL_UART_Receive_IT(&huart2, &imuRxByte, 1);
    }
    else if (huart == &huart4)
    {
        QR_RxHandler(qrRxByte);
        HAL_UART_Receive_IT(&huart4, &qrRxByte, 1);
    }
    else if (huart == &huart3)
    {
        /* Servo debug board (Bus-Servo-Controller) readback path.
           Only feed the byte into the servo RX state machine here; frame
           parsing is done in the main loop (Servo_Update). */
        Servo_RxHandler(servoRxByte);
        HAL_UART_Receive_IT(&huart3, &servoRxByte, 1);
    }
    else if (huart == &huart1)
    {
        /* RPi link (USART1, 115200). Feed the byte into the bsp_rpi framing
           state machine; parsing is done in the main loop (RPi_Update). */
        RPi_RxHandler(rpiRxByte);
        HAL_UART_Receive_IT(&huart1, &rpiRxByte, 1);
    }
}

/* On UART error (ORE/FE/NE/PE) HAL does not call RxCpltCallback, so RX IT
   would stall forever. Re-arm UART4 single-byte RX here to keep reception alive. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart4)
    {
        QR_NotifyError();
        (void)HAL_UART_Receive_IT(&huart4, &qrRxByte, 1);
    }
    else if (huart == &huart1)
    {
        /* RPi link (USART1) must be re-armed as well: a stalled RX IT would
           silently stop all vision frames. */
        RPi_NotifyError();
        (void)HAL_UART_Receive_IT(&huart1, &rpiRxByte, 1);
    }
    else if (huart == &huart2)
    {
        /* IMU (USART2) must be re-armed too: a stalled RX IT freezes the yaw,
           which silently breaks odometry rotation and route heading planning */
        imuErrCnt++;
        (void)HAL_UART_Receive_IT(&huart2, &imuRxByte, 1);
    }
    else if (huart == &huart3)
    {
        /* Servo debug board (USART3): same ORE/FE/NE/PE trap. Without re-arming
           the RX IT the servo readback path would stay dead forever. */
        Servo_NotifyError();
        (void)HAL_UART_Receive_IT(&huart3, &servoRxByte, 1);
    }
}

/* ===================== Ӧ�ýӿ� ===================== */

/**
 * @brief  Count the frames that really arrive within a time window (boot probe)
 * @param  ms: window length in milliseconds
 * @retval number of successfully parsed frames
 * @note   Used right after setting a high report rate: it tells us whether the
 *         UART link can actually carry that rate (the module may also send the
 *         0x52 gyro frame, which doubles the byte load). RX interrupt must be
 *         armed before calling this.
 */
#if (IMU_CFG_RATE_HZ != 0U)
static uint32_t IMU_ProbeFrames(uint16_t ms)
{
    uint32_t cnt = 0U;
    uint32_t t0  = HAL_GetTick();

    while ((HAL_GetTick() - t0) < (uint32_t)ms)
    {
        if (imuDev.frame_ready)
        {
            if (HWT101CT_ParseFrame(&imuDev))
            {
                cnt++;
            }
        }
        HAL_Delay(1);
    }

    return cnt;
}

/**
 * @brief  Set the module auto-report rate (boot only, see IMU_CFG_RATE_HZ)
 * @param  None
 * @retval None
 * @note   The module factory default is 10Hz: the yaw the application reads can be
 *         100ms old, which makes the route angle loop over-correct (it re-reads a
 *         stale angle right after a correction).
 *         Requested rate is IMU_CFG_RATE_HZ. For anything above 20Hz we verify it
 *         with IMU_ProbeFrames(): if the link cannot carry it (e.g. the module
 *         also sends gyro frames and 9600 baud saturates), we drop back to
 *         IMU_CFG_RATE_FALLBACK_HZ automatically and log it -- so asking for 50Hz
 *         can never leave the link silently saturated.
 *         The vendor driver writes the register over UART and uses ~300ms of
 *         HAL_Delay inside, so this is called ONCE from IMU_Init (boot), never in
 *         the main loop. On failure the module keeps its old rate (no harm).
 */
static void IMU_CfgOutputRate(void)
{
    HWT101CT_OutputRate_t r;
    uint32_t need;

    if (IMU_CFG_RATE_HZ == 10U)      { r = HWT101CT_RRATE_10HZ;  }
    else if (IMU_CFG_RATE_HZ == 50U) { r = HWT101CT_RRATE_50HZ;  }
    else if (IMU_CFG_RATE_HZ == 100U){ r = HWT101CT_RRATE_100HZ; }
    else                             { r = HWT101CT_RRATE_20HZ;  }

    if (HWT101CT_SetOutputRate(&imuDev, r) != HAL_OK)
    {
        Debug_Log("IMU", "out rate set FAIL, keep old rate");
        return;
    }

    /* The fallback rate is always safe on this link: no probe needed */
    if ((uint32_t)IMU_CFG_RATE_HZ <= (uint32_t)IMU_CFG_RATE_FALLBACK_HZ)
    {
        imuCfgRateHz = (IMU_CFG_RATE_HZ == 10U) ? 10U : (uint8_t)IMU_CFG_RATE_FALLBACK_HZ;
        Debug_Log("IMU", "out rate set=%uHz", (unsigned int)imuCfgRateHz);
        return;
    }

    /* Probe: expect ~IMU_CFG_RATE_HZ * window frames */
    need = ((uint32_t)IMU_CFG_RATE_HZ * (uint32_t)IMU_CFG_RATE_PROBE_MS *
            (uint32_t)IMU_CFG_RATE_PROBE_PCT) / 100000U;
    if (need == 0U)
    {
        need = 1U;
    }

    {
        uint32_t got = IMU_ProbeFrames((uint16_t)IMU_CFG_RATE_PROBE_MS);

        if (got >= need)
        {
            imuCfgRateHz = (uint8_t)IMU_CFG_RATE_HZ;
            Debug_Log("IMU", "out rate set=%uHz (probe ok %lu/%lu)",
                      (unsigned int)imuCfgRateHz, (unsigned long)got, (unsigned long)need);
            return;
        }

        Debug_Log("IMU", "rate %uHz too fast (probe %lu/%lu), fallback",
                  (unsigned int)IMU_CFG_RATE_HZ, (unsigned long)got, (unsigned long)need);
    }

    /* Fall back to IMU_CFG_RATE_FALLBACK_HZ (safe), do not probe again */
    if (HWT101CT_SetOutputRate(&imuDev, HWT101CT_RRATE_20HZ) == HAL_OK)
    {
        imuCfgRateHz = (uint8_t)IMU_CFG_RATE_FALLBACK_HZ;
    }
    else
    {
        imuCfgRateHz = (uint8_t)IMU_CFG_RATE_HZ;    /* write failed: module keeps it */
    }

    Debug_Log("IMU", "out rate set=%uHz (fallback)", (unsigned int)imuCfgRateHz);
}
#endif /* IMU_CFG_RATE_HZ != 0 */

void IMU_Init(void)
{
    /* ��ͳһ�긲�� USART2 �����ʲ����䣨HAL_UART_Init ������ MspInit�� */
    huart2.Init.BaudRate = HWT101CT_UART_BAUD;
    HAL_UART_Init(&huart2);

    /* ��ʼ�� HWT101CT ��� */
    HWT101CT_Init(&imuDev, &huart2);

    /* ��¼��ʼ���ģ���ʼΪ���� */
    imuLastFrameTick = HAL_GetTick();
    imuLastReportTick = HAL_GetTick();
    imuOnline = 0;

    /* �����жϽ������ֽڣ������� RxCpltCallback �����ӣ�
       (must be armed BEFORE the rate probe below, the probe counts real frames) */
    HAL_UART_Receive_IT(&huart2, &imuRxByte, 1);

#if (IMU_CFG_RATE_HZ != 0U)
    /* Set the module auto-report rate once at boot (English comments only here) */
    IMU_CfgOutputRate();
    Debug_Log("IMU", "init, baud=%lu rate=%uHz",
              (unsigned long)HWT101CT_UART_BAUD, (unsigned int)imuCfgRateHz);
#else
    /* Rate is configured on the module itself (user: 115200 + 1000Hz).
       1000Hz x 11B x 10bit = 110kbps ~= 95% of 115200; if frames drop, use
       500Hz on the module or raise the baud to 230400. */
    Debug_Log("IMU", "init, baud=%lu rate=module (untouched)",
              (unsigned long)HWT101CT_UART_BAUD);
#endif
}

/**
 * @brief  IMU ��ѭ������
 * @param  ��
 * @retval ��
 * @note   ����֡ �� ԭʼ֡��ӡ������ʱ���أ��� ���ʶȼ�� �� �����ϱ����ڲ��� HAL_Delay
 */
void IMU_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* 1. ��������֡����������������״̬ */
    if (imuDev.frame_ready)
    {
        if (HWT101CT_ParseFrame(&imuDev))
        {
            /* frame-type counters: if a=0 but g>0, the module sends only gyro frames
               (yaw never updates); if yc stops growing, the yaw value is frozen */
            if (imuDev.last_frame[1] == 0x52U)      { imuCntGyro++; }
            else if (imuDev.last_frame[1] == 0x53U) { imuCntAngle++; }
            if (imuDev.data.yaw != imuYaw)          { imuCntYawChg++; }
            imuYaw = imuDev.data.yaw;
            imuGyroZ = imuDev.data.gyro_z;
            imuLastFrameTick = now;
            if (!imuOnline)
            {
                imuOnline = 1;
                Debug_Log("IMU", "online");
            }
        }
    }

    /* 2. ԭʼ֡ hex ��ӡ���������ϱ����ڣ���������ץ���� */
    if (imuDebugRaw && imuDev.last_frame_valid)
    {
        imuDev.last_frame_valid = false;
        Debug_Log("IMU", "raw %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                  imuDev.last_frame[0], imuDev.last_frame[1], imuDev.last_frame[2],
                  imuDev.last_frame[3], imuDev.last_frame[4], imuDev.last_frame[5],
                  imuDev.last_frame[6], imuDev.last_frame[7], imuDev.last_frame[8],
                  imuDev.last_frame[9], imuDev.last_frame[10]);
    }

    /* 3. �������ʶȼ�⣺��ʱ�����ߣ������� P7 ��ȫ������ */
    if (imuOnline && (now - imuLastFrameTick) >= IMU_TIMEOUT_MS)
    {
        imuOnline = 0;
        Debug_Log("IMU", "timeout");
    }

    /* 4. �����ϱ� yaw/gz�������Ŵ��ӡ���� %f�� */
    if ((now - imuLastReportTick) >= IMU_REPORT_PERIOD_MS)
    {
        imuLastReportTick = now;

        int32_t yaw_x10 = (int32_t)(IMU_GetYaw() * 10.0f);
        int32_t gz_x10  = (int32_t)(imuGyroZ * 10.0f);
        int32_t yaw_i = yaw_x10 / 10;
        uint32_t yaw_f = (yaw_x10 < 0) ? (uint32_t)(-yaw_x10) % 10U : (uint32_t)yaw_x10 % 10U;
        int32_t gz_i = gz_x10 / 10;
        uint32_t gz_f = (gz_x10 < 0) ? (uint32_t)(-gz_x10) % 10U : (uint32_t)gz_x10 % 10U;

        Debug_Log("IMU", "yaw=%ld.%lu gz=%ld.%lu err=%lu g=%lu a=%lu yc=%lu",
                  (long)yaw_i, (unsigned long)yaw_f,
                  (long)gz_i, (unsigned long)gz_f,
                  (unsigned long)imuErrCnt, (unsigned long)imuCntGyro,
                  (unsigned long)imuCntAngle, (unsigned long)imuCntYawChg);

        /* OLED ˢ�� yaw��PAGE_DATA ҳ yaw ռλ��gz ���� OLED �ӿڣ����� P7 ��չ�� */
        Display_SetYaw((int16_t)(IMU_GetYaw() * 100.0f));
    }
}

/**
 * @brief  ��ȡ��ǰ Z ��Ƕȣ�yaw��
 * @param  ��
 * @retval Z ��Ƕȣ���λ�ȣ���Χ ��180�㣩
 */
float IMU_GetYaw(void)
{
    float y = imuYaw + imuYawOffset;

    while (y > 180.0f)  { y -= 360.0f; }
    while (y < -180.0f) { y += 360.0f; }
    return y;
}

/**
 * @brief  一次性"静态置角"：让 IMU_GetYaw() 从此刻起读 refDeg（deg）
 * @param  refDeg: 期望的当前朝向（如 TRAY PICK=0 / STORE BACK=180 / ROUGH=90）
 * @retval 无
 * @note   只在**小车静止**时调用（本函数检查 |gz|>30 deg/s 直接拒绝并打日志）。
 *         只是给 IMU 的原始 yaw 加一个固定偏移，**不钉住、不冻结**：
 *         之后 IMU_GetYaw() = 原始 yaw + 偏移，小车照常转动、照常被测到，
 *         所有绝对坐标/航向计算（Chassis_GetOdomYaw / Route_VelYaw /
 *         Task 的 Rotate）都自动建立在新基准上。
 *         这是"任务点重新对零"：把每段累积的零漂在静止时刻一次性抹掉。
 */
uint8_t IMU_StaticZeroTo(float refDeg)
{
    if ((imuGyroZ > 30.0f) || (imuGyroZ < -30.0f))
    {
        Debug_Log("IMU", "static zero reject, gz=%d", (int)(imuGyroZ * 10.0f));
        return 0U;
    }

    imuYawOffset = refDeg - imuYaw;
    Debug_Log("IMU", "static zero -> ref=%d raw=%d off=%d",
              (int)(refDeg * 10.0f), (int)(imuYaw * 10.0f),
              (int)(imuYawOffset * 10.0f));
    return 1U;
}

/**
 * @brief  ��ȡ��ǰ Z ����ٶ�
 * @param  ��
 * @retval Z ����ٶȣ���λ ��/s����Χ ��400��/s��
 */
float IMU_GetGyroZ(void)
{
    return imuGyroZ;
}

/**
 * @brief  Z ��Ƕȹ���
 * @param  ��
 * @retval ��
 * @note   �ڲ��� HWT101CT_ResetYaw������+����+���棬�� HAL_Delay����
 *         �������ڳ�ʼ��/�����¼��е��ã���ֹ����ѭ������
 */
void IMU_ResetYaw(void)
{
    HWT101CT_ResetYaw(&imuDev);
}

/**
 * @brief  ��ѯ IMU ����״̬
 * @param  ��
 * @retval 1=���ߣ�0=����
 */
uint8_t IMU_IsOnline(void)
{
    return imuOnline;
}

/**
 * @brief  �ֶ��޸Ĳ����ʷ�װ
 * @param  ��
 * @retval ��
 * @note   ��ģ�����Ϊ HWT101CT_UART_BAUD ��Ӧö��ֵ����ͬ���� STM32 �����䡣
 *         Ĭ�ϳ�ʼ�������ã��ڲ��� HAL_Delay����ֹ����ѭ������
 */
void IMU_SetBaudRate(void)
{
    HWT101CT_BaudRate_t b;
    switch (HWT101CT_UART_BAUD)
    {
        case 4800:   b = HWT101CT_BAUD_4800;   break;
        case 9600:   b = HWT101CT_BAUD_9600;   break;
        case 19200:  b = HWT101CT_BAUD_19200;  break;
        case 38400:  b = HWT101CT_BAUD_38400;  break;
        case 57600:  b = HWT101CT_BAUD_57600;  break;
        case 115200: b = HWT101CT_BAUD_115200; break;
        case 230400: b = HWT101CT_BAUD_230400; break;
        default:     b = HWT101CT_BAUD_9600;   break;
    }

    /* ��ģ��ನ���ʣ��ڲ�������+д�Ĵ���+����+HAL_Delay�� */
    HWT101CT_SetBaudRate(&imuDev, b);

    /* ͬ���� STM32 �ಢ������������ */
    huart2.Init.BaudRate = HWT101CT_UART_BAUD;
    HAL_UART_Init(&huart2);
    HAL_UART_Receive_IT(&huart2, &imuRxByte, 1);

    Debug_Log("IMU", "baud set=%lu", (unsigned long)HWT101CT_UART_BAUD);
}

/**
 * @brief  ����ʱ�л�ԭʼ֡ hex ��ӡ����
 * @param  ��
 * @retval ��
 */
void IMU_ToggleDebugRaw(void)
{
    imuDebugRaw = imuDebugRaw ? 0U : 1U;
    Debug_Log("IMU", "raw debug %s", imuDebugRaw ? "on" : "off");
}

void IMU_Test(void)
{
    imuTestSampleCnt = 0;
    imuTestStartTick = HAL_GetTick();
    imuTestLastSampleTick = HAL_GetTick();
    Debug_Log("TEST", "IMU self-test start");
}

void IMU_Test_Update(void)
{
    if (!imuOnline)
    {
        if ((HAL_GetTick() - imuTestStartTick) >= TEST_TIMEOUT_MS)
        {
            testItems[TEST_ID_IMU].state = TEST_STATE_FAIL;
        }
        return;
    }

    if ((HAL_GetTick() - imuTestLastSampleTick) >= (IMU_REPORT_PERIOD_MS / 2U))
    {
        imuTestLastSampleTick = HAL_GetTick();
        imuTestSampleCnt++;
        Debug_Log("TEST", "IMU self-test yaw=%d gz=%d", (int)imuYaw, (int)imuGyroZ);
    }

    if (imuTestSampleCnt >= TEST_IMU_SAMPLE_CNT)
    {
        testItems[TEST_ID_IMU].state = TEST_STATE_PASS;
    }
}

void IMU_Test_Stop(void)
{
    imuTestSampleCnt = 0;
}
