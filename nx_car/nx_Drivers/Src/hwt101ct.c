/**
 ******************************************************************************
 * @file    hwt101ct.c
 * @brief   HWT101CT单轴陀螺仪模块HAL库驱动源文件
 * @author  WitMotion
 * @version V1.0.0
 * @date    2026
 ******************************************************************************
 * @attention
 * 基于维特智能HWT101CT模块协议编写
 * 通信方式: UART串口
 * 默认波特率: 9600bps
 ******************************************************************************
 */

#include "hwt101ct.h"
#include <string.h>

/* Private function prototypes -----------------------------------------------*/
static uint8_t HWT101CT_CalcChecksum(uint8_t *data, uint8_t len);
static int16_t HWT101CT_BytesToInt16(uint8_t low, uint8_t high);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  初始化HWT101CT模块
 * @param  hdev: HWT101CT句柄
 * @param  huart: UART句柄
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_Init(HWT101CT_HandleTypeDef *hdev, UART_HandleTypeDef *huart)
{
    if (hdev == NULL || huart == NULL) {
        return HAL_ERROR;
    }
    
    /* 初始化句柄 */
    hdev->huart = huart;
    hdev->rx_index = 0;
    hdev->rx_state = HWT101CT_RX_IDLE;
    hdev->frame_ready = false;
    hdev->last_frame_valid = false;
    hdev->initialized = true;
    
    /* 清零数据 */
    memset(&hdev->data, 0, sizeof(HWT101CT_Data_t));
    memset(hdev->rx_buffer, 0, sizeof(hdev->rx_buffer));
    memset(hdev->last_frame, 0, sizeof(hdev->last_frame));
    
    return HAL_OK;
}

/**
 * @brief  串口接收数据处理（在UART中断中调用）
 * @param  hdev: HWT101CT句柄
 * @param  byte: 接收到的字节
 * @retval None
 */
void HWT101CT_UART_RxHandler(HWT101CT_HandleTypeDef *hdev, uint8_t byte)
{
    uint8_t i;
    uint8_t n;
    uint8_t sum;
    
    if (hdev == NULL || !hdev->initialized) {
        return;
    }
    
    /* 空闲态：等待帧头 0x55 */
    if (hdev->rx_state == HWT101CT_RX_IDLE) {
        if (byte == HWT101CT_HEADER_READ) {
            hdev->rx_buffer[0] = byte;
            hdev->rx_index = 1;
            hdev->rx_state = HWT101CT_RX_FILL;
        }
        return;
    }
    
    /* 收集中：填充缓冲区 */
    hdev->rx_buffer[hdev->rx_index++] = byte;
    if (hdev->rx_index < HWT101CT_FRAME_LEN) {
        return;
    }
    
    /* 收满一帧，校验和验证（校验和 = 前10字节累加和） */
    sum = HWT101CT_CalcChecksum(hdev->rx_buffer, 10);
    if (sum == hdev->rx_buffer[10]) {
        /* 校验通过：保存原始帧供调试打印，置帧就绪 */
        for (i = 0; i < HWT101CT_FRAME_LEN; i++) {
            hdev->last_frame[i] = hdev->rx_buffer[i];
        }
        hdev->last_frame_valid = true;
        hdev->frame_ready = true;
        hdev->rx_index = 0;
        hdev->rx_state = HWT101CT_RX_IDLE;
        return;
    }
    
    /* 校验失败：滑窗寻找下一个帧头 0x55（从第2字节起），自恢复避免永久错位 */
    for (i = 1; i < HWT101CT_FRAME_LEN; i++) {
        if (hdev->rx_buffer[i] == HWT101CT_HEADER_READ) {
            n = HWT101CT_FRAME_LEN - i;
            memmove(hdev->rx_buffer, &hdev->rx_buffer[i], n);
            hdev->rx_index = n;
            hdev->rx_state = HWT101CT_RX_FILL;
            return;
        }
    }
    
    /* 未找到帧头，重置为空闲态 */
    hdev->rx_index = 0;
    hdev->rx_state = HWT101CT_RX_IDLE;
}

/**
 * @brief  解析接收到的数据帧
 * @param  hdev: HWT101CT句柄
 * @retval true: 解析成功, false: 解析失败
 */
bool HWT101CT_ParseFrame(HWT101CT_HandleTypeDef *hdev)
{
    uint8_t checksum;
    uint8_t data_type;
    int16_t temp;
    
    if (hdev == NULL || !hdev->initialized || !hdev->frame_ready) {
        return false;
    }
    
    hdev->frame_ready = false;
    
    /* 校验和验证（帧在接收时已锁定到 last_frame，避免被后续接收覆盖） */
    checksum = HWT101CT_CalcChecksum(hdev->last_frame, 10);
    if (checksum != hdev->last_frame[10]) {
        return false;
    }
    
    /* 获取数据类型 */
    data_type = hdev->last_frame[1];
    
    switch (data_type) {
        case HWT101CT_TYPE_GYRO:  /* 角速度数据 0x52 */
            /* 角速度帧：0x55 0x52 WxL WxH WyL WyH WzL WzH TL TH SUM，Wz 位于字节 6-7 */
            temp = HWT101CT_BytesToInt16(hdev->last_frame[6], hdev->last_frame[7]);
            hdev->data.gyro_z = (float)temp * HWT101CT_GYRO_SCALE;
            /* HWT101CT 为单轴（仅 Z 轴有效），raw 与校准值同源 */
            hdev->data.gyro_z_raw = hdev->data.gyro_z;
            break;
            
        case HWT101CT_TYPE_ANGLE:  /* 角度数据 0x53 */
            /* 角度帧：0x55 0x53 RollL RollH PitchL PitchH YawL YawH TL TH SUM，Yaw 位于字节 6-7 */
            temp = HWT101CT_BytesToInt16(hdev->last_frame[6], hdev->last_frame[7]);
            hdev->data.yaw = (float)temp * HWT101CT_ANGLE_SCALE;
            
            /* 版本号: 字节8-9 */
            hdev->data.version = (uint16_t)HWT101CT_BytesToInt16(hdev->last_frame[8], hdev->last_frame[9]);
            break;
            
        default:
            return false;
    }
    
    /* 更新时间戳 */
    hdev->data.timestamp = HAL_GetTick();
    
    return true;
}

/**
 * @brief  获取最新数据
 * @param  hdev: HWT101CT句柄
 * @param  data: 数据结构体指针
 * @retval None
 */
void HWT101CT_GetData(HWT101CT_HandleTypeDef *hdev, HWT101CT_Data_t *data)
{
    if (hdev == NULL || data == NULL || !hdev->initialized) {
        return;
    }
    
    memcpy(data, &hdev->data, sizeof(HWT101CT_Data_t));
}

/**
 * @brief  解锁寄存器写保护
 * @param  hdev: HWT101CT句柄
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_Unlock(HWT101CT_HandleTypeDef *hdev)
{
    return HWT101CT_WriteReg(hdev, HWT101CT_REG_KEY, HWT101CT_UNLOCK_KEY);
}

/**
 * @brief  保存配置
 * @param  hdev: HWT101CT句柄
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_SaveConfig(HWT101CT_HandleTypeDef *hdev)
{
    return HWT101CT_WriteReg(hdev, HWT101CT_REG_SAVE, 0x0000);
}

/**
 * @brief  重启模块
 * @param  hdev: HWT101CT句柄
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_Reboot(HWT101CT_HandleTypeDef *hdev)
{
    return HWT101CT_WriteReg(hdev, HWT101CT_REG_SAVE, 0x00FF);
}

/**
 * @brief  恢复出厂设置
 * @param  hdev: HWT101CT句柄
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_RestoreFactory(HWT101CT_HandleTypeDef *hdev)
{
    return HWT101CT_WriteReg(hdev, HWT101CT_REG_SAVE, 0x0001);
}

/**
 * @brief  Z轴角度归零
 * @param  hdev: HWT101CT句柄
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_ResetYaw(HWT101CT_HandleTypeDef *hdev)
{
    HAL_StatusTypeDef status;
    
    /* 解锁 */
    status = HWT101CT_Unlock(hdev);
    if (status != HAL_OK) return status;
    
    HAL_Delay(200);
    
    /* 角度归零 */
    status = HWT101CT_WriteReg(hdev, HWT101CT_REG_CALIYAW, 0x0000);
    if (status != HAL_OK) return status;
    
    HAL_Delay(500);
    
    /* 保存 */
    status = HWT101CT_SaveConfig(hdev);
    
    return status;
}

/**
 * @brief  设置Z轴角度
 * @param  hdev: HWT101CT句柄
 * @param  angle: 角度值 (-180 ~ 180°)
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_SetYaw(HWT101CT_HandleTypeDef *hdev, float angle)
{
    HAL_StatusTypeDef status;
    int16_t reg_value;
    
    /* 范围限制 */
    if (angle > 180.0f) angle = 180.0f;
    if (angle < -180.0f) angle = -180.0f;
    
    /* 转换为寄存器值: angle * 32768 / 180 */
    reg_value = (int16_t)(angle * 32768.0f / 180.0f);
    
    /* 解锁 */
    status = HWT101CT_Unlock(hdev);
    if (status != HAL_OK) return status;
    
    HAL_Delay(200);
    
    /* 设置角度 */
    status = HWT101CT_WriteReg(hdev, HWT101CT_REG_CALIYAW, (uint16_t)reg_value);
    if (status != HAL_OK) return status;
    
    HAL_Delay(200);
    
    /* 保存 */
    status = HWT101CT_SaveConfig(hdev);
    
    return status;
}

/**
 * @brief  自动获取零偏（需保持模块静止）
 * @param  hdev: HWT101CT句柄
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_AutoCalibration(HWT101CT_HandleTypeDef *hdev)
{
    HAL_StatusTypeDef status;
    
    /* 解锁 */
    status = HWT101CT_Unlock(hdev);
    if (status != HAL_OK) return status;
    
    HAL_Delay(200);
    
    /* 启动自动校准 */
    status = HWT101CT_WriteReg(hdev, HWT101CT_REG_WORKMODE, 0x0001);
    if (status != HAL_OK) return status;
    
    /* 等待校准完成 (约5秒) */
    HAL_Delay(5000);
    
    /* 保存 */
    status = HWT101CT_SaveConfig(hdev);
    
    return status;
}

/**
 * @brief  设置输出速率
 * @param  hdev: HWT101CT句柄
 * @param  rate: 输出速率枚举
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_SetOutputRate(HWT101CT_HandleTypeDef *hdev, HWT101CT_OutputRate_t rate)
{
    HAL_StatusTypeDef status;
    
    /* 解锁 */
    status = HWT101CT_Unlock(hdev);
    if (status != HAL_OK) return status;
    
    HAL_Delay(200);
    
    /* 设置输出速率 */
    status = HWT101CT_WriteReg(hdev, HWT101CT_REG_RRATE, (uint16_t)rate);
    if (status != HAL_OK) return status;
    
    HAL_Delay(100);
    
    /* 保存 */
    status = HWT101CT_SaveConfig(hdev);
    
    return status;
}

/**
 * @brief  设置波特率
 * @param  hdev: HWT101CT句柄
 * @param  baud: 波特率枚举
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_SetBaudRate(HWT101CT_HandleTypeDef *hdev, HWT101CT_BaudRate_t baud)
{
    HAL_StatusTypeDef status;
    
    /* 解锁 */
    status = HWT101CT_Unlock(hdev);
    if (status != HAL_OK) return status;
    
    HAL_Delay(200);
    
    /* 设置波特率 */
    status = HWT101CT_WriteReg(hdev, HWT101CT_REG_BAUD, (uint16_t)baud);
    if (status != HAL_OK) return status;
    
    HAL_Delay(100);
    
    /* 保存 */
    status = HWT101CT_SaveConfig(hdev);
    
    return status;
}

/**
 * @brief  设置LED开关
 * @param  hdev: HWT101CT句柄
 * @param  off: true-关闭, false-打开
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_SetLED(HWT101CT_HandleTypeDef *hdev, bool off)
{
    return HWT101CT_WriteReg(hdev, HWT101CT_REG_LEDOFF, off ? 0x0001 : 0x0000);
}

/**
 * @brief  设置工作模式
 * @param  hdev: HWT101CT句柄
 * @param  mode: 工作模式枚举
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_SetWorkMode(HWT101CT_HandleTypeDef *hdev, HWT101CT_WorkMode_t mode)
{
    HAL_StatusTypeDef status;
    
    /* 解锁 */
    status = HWT101CT_Unlock(hdev);
    if (status != HAL_OK) return status;
    
    HAL_Delay(200);
    
    /* 设置工作模式 */
    status = HWT101CT_WriteReg(hdev, HWT101CT_REG_WORKMODE, (uint16_t)mode);
    if (status != HAL_OK) return status;
    
    HAL_Delay(100);
    
    /* 保存 */
    status = HWT101CT_SaveConfig(hdev);
    
    return status;
}

/**
 * @brief  设置静止阈值
 * @param  hdev: HWT101CT句柄
 * @param  threshold: 阈值 (°/s)
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_SetStaticThreshold(HWT101CT_HandleTypeDef *hdev, float threshold)
{
    uint16_t reg_value;
    
    /* 转换为寄存器值: threshold * 1000 */
    reg_value = (uint16_t)(threshold * 1000.0f);
    
    return HWT101CT_WriteReg(hdev, HWT101CT_REG_GSTATICTHRE, reg_value);
}

/**
 * @brief  设置稳定时间
 * @param  hdev: HWT101CT句柄
 * @param  time_ms: 时间 (ms)
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_SetStaticTime(HWT101CT_HandleTypeDef *hdev, uint16_t time_ms)
{
    return HWT101CT_WriteReg(hdev, HWT101CT_REG_GSTATICTIME, time_ms);
}

/**
 * @brief  写寄存器
 * @param  hdev: HWT101CT句柄
 * @param  reg: 寄存器地址
 * @param  value: 寄存器值
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_WriteReg(HWT101CT_HandleTypeDef *hdev, uint8_t reg, uint16_t value)
{
    uint8_t tx_buffer[5];
    
    if (hdev == NULL || !hdev->initialized) {
        return HAL_ERROR;
    }
    
    /* 构造写寄存器帧: 0xFF 0xAA ADDR DATAL DATAH */
    tx_buffer[0] = HWT101CT_HEADER_WRITE1;
    tx_buffer[1] = HWT101CT_HEADER_WRITE2;
    tx_buffer[2] = reg;
    tx_buffer[3] = (uint8_t)(value & 0x00FF);        /* 低字节 */
    tx_buffer[4] = (uint8_t)((value & 0xFF00) >> 8); /* 高字节 */
    
    /* 发送数据 */
    return HAL_UART_Transmit(hdev->huart, tx_buffer, 5, 100);
}

/**
 * @brief  读寄存器
 * @param  hdev: HWT101CT句柄
 * @param  reg: 寄存器地址
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_ReadReg(HWT101CT_HandleTypeDef *hdev, uint8_t reg)
{
    return HWT101CT_WriteReg(hdev, HWT101CT_REG_READADDR, (uint16_t)reg);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  计算校验和
 * @param  data: 数据缓冲区
 * @param  len: 数据长度
 * @retval 校验和
 */
static uint8_t HWT101CT_CalcChecksum(uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint8_t sum = 0;
    
    for (i = 0; i < len; i++) {
        sum += data[i];
    }
    
    return sum;
}

/**
 * @brief  字节转换为有符号16位整数
 * @param  low: 低字节
 * @param  high: 高字节
 * @retval 有符号16位整数
 */
static int16_t HWT101CT_BytesToInt16(uint8_t low, uint8_t high)
{
    return (int16_t)(((int16_t)high << 8) | low);
}
