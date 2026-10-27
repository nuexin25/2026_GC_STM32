/**
 ******************************************************************************
 * @file    hwt101ct.h
 * @brief   HWT101CT单轴陀螺仪模块HAL库驱动头文件
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

#ifndef __HWT101CT_H
#define __HWT101CT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"  /* 根据实际MCU型号修改 */
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief HWT101CT数据结构体
 */
typedef struct {
    float gyro_z;             /* Z轴角速度 (°/s) */
    float gyro_z_raw;         /* Z轴原始角速度 (°/s) */
    float yaw;                /* 航向角 (°) */
    uint16_t version;         /* 版本号 */
    uint32_t timestamp;       /* 时间戳 */
} HWT101CT_Data_t;

/**
 * @brief HWT101CT寄存器地址定义
 */
typedef enum {
    HWT101CT_REG_SAVE         = 0x00,    /* 保存/重启/恢复出厂 */
    HWT101CT_REG_CALSE        = 0x01,    /* 手动获取零偏 */
    HWT101CT_REG_RRATE        = 0x03,    /* 输出速率 */
    HWT101CT_REG_BAUD         = 0x04,    /* 串口波特率 */
    HWT101CT_REG_IICADDR      = 0x1A,    /* 设备地址 */
    HWT101CT_REG_LEDOFF       = 0x1B,    /* 关闭LED灯 */
    HWT101CT_REG_READADDR     = 0x27,    /* 读取寄存器 */
    HWT101CT_REG_VERSION      = 0x2E,    /* 版本号 */
    HWT101CT_REG_GZ           = 0x39,    /* 角速度Z */
    HWT101CT_REG_YAW          = 0x3F,    /* 航向角 */
    HWT101CT_REG_WORKMODE     = 0x48,    /* 工作模式 */
    HWT101CT_REG_GYROBAIS     = 0x4C,    /* Z轴零偏值 */
    HWT101CT_REG_GSTATICTHRE  = 0x4E,    /* Z轴静止阈值 */
    HWT101CT_REG_GSTATICTIME  = 0x4F,    /* Z轴稳定时间 */
    HWT101CT_REG_KEY          = 0x69,    /* 解锁 */
    HWT101CT_REG_GYROCALITHR  = 0x61,    /* 静态检测阈值 */
    HWT101CT_REG_GYROCALTIME  = 0x63,    /* 陀螺仪自动校准时间 */
    HWT101CT_REG_WZTIME       = 0x6E,    /* 角速度连续静止时间 */
    HWT101CT_REG_WZSTATIC     = 0x6F,    /* 角速度积分阈值 */
    HWT101CT_REG_CALIYAW      = 0x76,    /* Z轴角度归零 */
} HWT101CT_RegAddr_t;

/**
 * @brief 输出速率枚举
 */
typedef enum {
    HWT101CT_RRATE_0_2HZ    = 0x01,   /* 0.2Hz */
    HWT101CT_RRATE_0_5HZ    = 0x02,   /* 0.5Hz */
    HWT101CT_RRATE_1HZ      = 0x03,   /* 1Hz */
    HWT101CT_RRATE_2HZ      = 0x04,   /* 2Hz */
    HWT101CT_RRATE_5HZ      = 0x05,   /* 5Hz */
    HWT101CT_RRATE_10HZ     = 0x06,   /* 10Hz (默认) */
    HWT101CT_RRATE_20HZ     = 0x07,   /* 20Hz */
    HWT101CT_RRATE_50HZ     = 0x08,   /* 50Hz */
    HWT101CT_RRATE_100HZ    = 0x09,   /* 100Hz */
    HWT101CT_RRATE_200HZ    = 0x0B,   /* 200Hz */
    HWT101CT_RRATE_500HZ    = 0x0C,   /* 500Hz */
    HWT101CT_RRATE_1000HZ   = 0x0D,   /* 1000Hz */
} HWT101CT_OutputRate_t;

/**
 * @brief 波特率枚举
 */
typedef enum {
    HWT101CT_BAUD_4800      = 0x01,   /* 4800bps */
    HWT101CT_BAUD_9600      = 0x02,   /* 9600bps (默认) */
    HWT101CT_BAUD_19200     = 0x03,   /* 19200bps */
    HWT101CT_BAUD_38400     = 0x04,   /* 38400bps */
    HWT101CT_BAUD_57600     = 0x05,   /* 57600bps */
    HWT101CT_BAUD_115200    = 0x06,   /* 115200bps */
    HWT101CT_BAUD_230400    = 0x07,   /* 230400bps */
} HWT101CT_BaudRate_t;

/**
 * @brief 工作模式枚举
 */
typedef enum {
    HWT101CT_MODE_NORMAL     = 0x00,   /* 正常数据模式 */
    HWT101CT_MODE_PEAK       = 0x01,   /* 求峰峰值模式 */
    HWT101CT_MODE_BIAS       = 0x02,   /* 求零偏模式 */
    HWT101CT_MODE_SCALE      = 0x03,   /* 求标度因素模式 */
} HWT101CT_WorkMode_t;

/**
 * @brief HWT101CT接收状态机状态
 */
typedef enum {
    HWT101CT_RX_IDLE = 0,   /* 空闲：等待帧头0x55 */
    HWT101CT_RX_FILL        /* 收集中：填充rx_buffer */
} HWT101CT_RxState_t;

/**
 * @brief HWT101CT句柄结构体
 */
typedef struct {
    UART_HandleTypeDef *huart;         /* UART句柄 */
    HWT101CT_Data_t data;              /* 解析后的数据 */
    
    /* 接收缓冲区 */
    uint8_t rx_buffer[11];             /* 单帧接收缓冲区 */
    uint8_t rx_index;                  /* 接收索引 */
    HWT101CT_RxState_t rx_state;       /* 接收状态机状态 */
    bool frame_ready;                  /* 帧就绪标志 */
    
    /* 原始帧调试缓冲 */
    uint8_t last_frame[11];            /* 最近一次完整帧原始字节（供调试打印） */
    bool last_frame_valid;             /* 有新的完整帧待读取 */
    
    /* 状态 */
    bool initialized;                  /* 初始化标志 */
} HWT101CT_HandleTypeDef;

/* Exported constants --------------------------------------------------------*/

/* 协议头定义 */
#define HWT101CT_HEADER_READ       0x55    /* 读数据帧头 */
#define HWT101CT_HEADER_WRITE1     0xFF    /* 写数据帧头1 */
#define HWT101CT_HEADER_WRITE2     0xAA    /* 写数据帧头2 */

/* 数据类型定义 */
#define HWT101CT_TYPE_GYRO         0x52    /* 角速度数据 */
#define HWT101CT_TYPE_ANGLE        0x53    /* 角度数据 */

/* 帧长度 */
#define HWT101CT_FRAME_LEN         11      /* 单帧字节数：帧头0x55 + 类型 + 数据 + 校验和 */

/* 解锁密钥 */
#define HWT101CT_UNLOCK_KEY        0xB588  /* 解锁密钥 */

/* 量程转换系数（换算公式：物理量 = 原始int16 * 量程 / 32768） */
#define HWT101CT_GYRO_SCALE        400.0f / 32768.0f    /* 角速度转换系数：HWT101CT量程±400°/s（非HWT101的2000°/s） */
#define HWT101CT_ANGLE_SCALE       180.0f / 32768.0f    /* 角度转换系数：Z轴角度量程±180° */

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  初始化HWT101CT模块
 * @param  hdev: HWT101CT句柄
 * @param  huart: UART句柄
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_Init(HWT101CT_HandleTypeDef *hdev, UART_HandleTypeDef *huart);

/**
 * @brief  串口接收数据处理（在UART中断中调用）
 * @param  hdev: HWT101CT句柄
 * @param  byte: 接收到的字节
 * @retval None
 */
void HWT101CT_UART_RxHandler(HWT101CT_HandleTypeDef *hdev, uint8_t byte);

/**
 * @brief  解析接收到的数据帧
 * @param  hdev: HWT101CT句柄
 * @retval true: 解析成功, false: 解析失败
 */
bool HWT101CT_ParseFrame(HWT101CT_HandleTypeDef *hdev);

/**
 * @brief  获取最新数据
 * @param  hdev: HWT101CT句柄
 * @param  data: 数据结构体指针
 * @retval None
 */
void HWT101CT_GetData(HWT101CT_HandleTypeDef *hdev, HWT101CT_Data_t *data);

/**
 * @brief  解锁寄存器写保护
 * @param  hdev: HWT101CT句柄
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_Unlock(HWT101CT_HandleTypeDef *hdev);

/**
 * @brief  保存配置
 * @param  hdev: HWT101CT句柄
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_SaveConfig(HWT101CT_HandleTypeDef *hdev);

/**
 * @brief  重启模块
 * @param  hdev: HWT101CT句柄
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_Reboot(HWT101CT_HandleTypeDef *hdev);

/**
 * @brief  恢复出厂设置
 * @param  hdev: HWT101CT句柄
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_RestoreFactory(HWT101CT_HandleTypeDef *hdev);

/**
 * @brief  Z轴角度归零
 * @param  hdev: HWT101CT句柄
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_ResetYaw(HWT101CT_HandleTypeDef *hdev);

/**
 * @brief  设置Z轴角度
 * @param  hdev: HWT101CT句柄
 * @param  angle: 角度值 (-180 ~ 180°)
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_SetYaw(HWT101CT_HandleTypeDef *hdev, float angle);

/**
 * @brief  自动获取零偏（需保持模块静止）
 * @param  hdev: HWT101CT句柄
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_AutoCalibration(HWT101CT_HandleTypeDef *hdev);

/**
 * @brief  设置输出速率
 * @param  hdev: HWT101CT句柄
 * @param  rate: 输出速率枚举
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_SetOutputRate(HWT101CT_HandleTypeDef *hdev, HWT101CT_OutputRate_t rate);

/**
 * @brief  设置波特率
 * @param  hdev: HWT101CT句柄
 * @param  baud: 波特率枚举
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_SetBaudRate(HWT101CT_HandleTypeDef *hdev, HWT101CT_BaudRate_t baud);

/**
 * @brief  设置LED开关
 * @param  hdev: HWT101CT句柄
 * @param  off: true-关闭, false-打开
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_SetLED(HWT101CT_HandleTypeDef *hdev, bool off);

/**
 * @brief  设置工作模式
 * @param  hdev: HWT101CT句柄
 * @param  mode: 工作模式枚举
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_SetWorkMode(HWT101CT_HandleTypeDef *hdev, HWT101CT_WorkMode_t mode);

/**
 * @brief  设置静止阈值
 * @param  hdev: HWT101CT句柄
 * @param  threshold: 阈值 (°/s)
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_SetStaticThreshold(HWT101CT_HandleTypeDef *hdev, float threshold);

/**
 * @brief  设置稳定时间
 * @param  hdev: HWT101CT句柄
 * @param  time_ms: 时间 (ms)
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_SetStaticTime(HWT101CT_HandleTypeDef *hdev, uint16_t time_ms);

/**
 * @brief  写寄存器
 * @param  hdev: HWT101CT句柄
 * @param  reg: 寄存器地址
 * @param  value: 寄存器值
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_WriteReg(HWT101CT_HandleTypeDef *hdev, uint8_t reg, uint16_t value);

/**
 * @brief  读寄存器
 * @param  hdev: HWT101CT句柄
 * @param  reg: 寄存器地址
 * @retval HAL状态
 */
HAL_StatusTypeDef HWT101CT_ReadReg(HWT101CT_HandleTypeDef *hdev, uint8_t reg);

#ifdef __cplusplus
}
#endif

#endif /* __HWT101CT_H */
