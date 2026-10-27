/**
 ******************************************************************************
 * @file    bsp_oled_ui.h
 * @brief   OLED 显示应用层：页面框架、格式化显示、页面选择菜单
 * @author  nx_car
 * @date    2026-09-05
 ******************************************************************************
 * @note   本文件为 oled.c 的应用层封装，不修改 oled.c 底层驱动。
 *         为提升刷新速度，所有页面数据采用 HAL_I2C_Mem_Write 整页写入，
 *         替代 oled.c 逐字节 WriteDat 的高开销。
 ******************************************************************************
 */
#ifndef __BSP_OLED_UI_H
#define __BSP_OLED_UI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "bsp_key.h"

/* ===================== 页面 ID 枚举 ===================== */
typedef enum {
    PAGE_BOOT = 0,   /* 开机页：Logo + 运行状态 */
    PAGE_DATA,       /* 数据页：运行时间、角度/位置 yaw/pos 预留 */
    PAGE_TASK,       /* 任务码页：任务码占位 */
    PAGE_STAT,       /* 统计页：抓取/码放统计占位 */
    PAGE_TEST,       /* 模块测试页：各模块自检 */
    PAGE_WHEEL,      /* 单轮点动测试页：P3 电机单轮调试（由测试页 MOT 项进入） */
    PAGE_CHASSIS,    /* 车机测试页：P4 麦轮底盘位置模式调试（由测试页 CHS 项进入） */
    PAGE_QR,         /* 扫码测试页：P5 GM65 扫码人工验证（由测试页 QR 项进入） */
    PAGE_ROUTE,      /* 跑图测试页：P7 无障碍物跑图（由测试页 ROUTE 项进入） */
    PAGE_RPI,        /* 树莓派通讯测试页：P6 视觉链路人工验证（由测试页 RPI 项进入） */
    PAGE_ARM,        /* 机械臂测试页：P8 舵机/机械臂电机单独调试（由测试页 SRV 项进入） */
    PAGE_ARM_SEQ,    /* 手臂调试页：P8 联调，进入自动回原点 + 多关节动作序列（由主菜单 ARMSEQ 进入） */
    PAGE_ARM_PARA,   /* 手臂参数页：P8 联调，读电机5/6 位置与舵机最后下发值（由主菜单 ARMPARA 进入） */
    PAGE_MISSION,    /* 赛题页：P9 步骤1，单步跑点位（由主菜单 MISSION 进入） */
    PAGE_TASK_DBG,   /* 任务调试页：P9 任务点单点调试（由主菜单 TDBG 进入） */
    PAGE_MENU,       /* 页面选择菜单：选择确认进入 */
    PAGE_ALIGN_TEST, /* 对位调参页：P24 单独跑一次对位 + 逐帧调参日志（由主菜单 ALNT 进入） */
    PAGE_VELTEST,    /* 速度模式测试页：P25 纯陀螺仪 + 时间控距离（由主菜单 VELTEST 进入） */
    PAGE_VELROUTE,   /* 速度模式跑图页：P26 赛题路段表全程速度模式（由主菜单 VELROUTE 进入） */
    PAGE_MAX
} PageID_t;

/* ===================== 可调参数 ===================== */
#define REFRESH_PERIOD_MS     100U    /* 页面刷新周期，单位 ms，可调范围 20~1000 */
#define BOOT_DISPLAY_MS       2000U  /* 开机页显示时长，单位 ms，可调范围 500~5000 */

#define OLED_I2C_ADDR         0x78   /* OLED I2C 地址（8 位写地址，对应 7 位 0x3C） */
#define OLED_I2C_TIMEOUT      100U   /* I2C 超时，单位 ms，可调范围 10~1000 */
#define OLED_PAGE_BUF_SIZE    128U   /* 整页缓冲区大小（128 列） */

/* ===================== 函数声明 ===================== */

/**
 * @brief  OLED 显示系统初始化
 * @param  无
 * @retval 无
 */
void Display_Init(void);

/**
 * @brief  切换到指定页面
 * @param  page: 页面 ID（PageID_t）
 * @retval 无
 */
void Display_SetPage(uint8_t page);


/**
 * @brief  显示任务主循环节拍（非阻塞）
 * @param  无
 * @retval 无
 */
void Display_Task(void);

/**
 * @brief  更新任务码显示内容
 * @param  code: 任务码字符串，NULL 或 "" 表示占位
 * @retval 无
 */
void Display_SetTaskCode(const char *code);

/**
 * @brief  更新抓取/码放统计
 * @param  pick_ok: 抓取正确次数
 * @param  pick_total: 抓取总次数
 * @param  place_ok: 码放正确次数
 * @param  place_total: 码放总次数
 * @retval 无
 */
void Display_SetStat(uint16_t pick_ok, uint16_t pick_total,
                     uint16_t place_ok, uint16_t place_total);

/**
 * @brief  更新 yaw 值（数据页预留）
 * @param  yaw_x100: 航向角放大 100 的整数值，0x7FFF 表示无效
 * @retval 无
 */
void Display_SetYaw(int16_t yaw_x100);

/**
 * @brief  更新位置值（数据页预留）
 * @param  pos_x_mm: X 坐标 mm
 * @param  pos_y_mm: Y 坐标 mm
 * @retval 无
 */
void Display_SetPos(int16_t pos_x_mm, int16_t pos_y_mm);

/**
 * @brief  刷新单轮点动测试页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_WHEEL 页时置脏；单轮页状态只在按键后变化，禁止周期重绘。
 */
void Display_WheelTestRefresh(void);

/**
 * @brief  刷新车机测试页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_CHASSIS 页时置脏；车机模块运行中每拍调用以
 *         刷新进度行，按键切换动作/启停时也调用。实际重绘受
 *         REFRESH_PERIOD_MS 节拍限制，不会高频全量重绘。
 */
void Display_ChassisRefresh(void);

/**
 * @brief  刷新扫码测试页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_QR 页时置脏；新帧成帧后由 QR 模块调用，
 *         KEY3 手动重扫也调用。禁止周期高频全量重绘。
 */
void Display_QRRefresh(void);

/**
 * @brief  刷新树莓派通讯测试页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_RPI 页时置脏；收到新帧或数据过期时由 RPi 模块调用，
 *         本页切指令/发送指令也调用。禁止周期高频全量重绘。
 */
void Display_RPiRefresh(void);

/**
 * @brief  刷新跑图测试页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_ROUTE 页时置脏；跑图执行器在状态/路段变化时调用，
 *         按键启动/中止也调用。禁止周期高频全量重绘。
 */
void Display_RouteRefresh(void);

/**
 * @brief  刷新机械臂测试页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_ARM 页时置脏；机械臂测试页由本页主循环节拍与
 *         按键处调用（电机位置一直在变，需要持续刷新才能看到实时值）。
 *         实际重绘受 REFRESH_PERIOD_MS 节拍限制，不会高频全量重绘。
 */
void Display_ArmRefresh(void);

/**
 * @brief  刷新手臂调试页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_ARM_SEQ 页时置脏；回原点/动作序列状态变化、
 *         按键操作时调用。禁止周期高频全量重绘。
 */
void Display_ArmSeqRefresh(void);

/**
 * @brief  刷新手臂参数页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_ARM_PARA 页时置脏；读数刷新（节流）与按键时调用。
 */
void Display_ArmParaRefresh(void);

/**
 * @brief  刷新赛题页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_MISSION 页时置脏；赛题执行器在状态/阶段变化时调用，
 *         到点停留期间每拍调用以刷新 0.1s 倒计时，按键操作也调用。
 *         实际重绘受 REFRESH_PERIOD_MS 节流，不会高频全量重绘。
 */
void Display_MissionRefresh(void);

/**
 * @brief  刷新任务调试页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_TASK_DBG 页时置脏；任务调试状态机在阶段/状态变化时每拍调用，
 *         按键操作也调用。实际重绘受 REFRESH_PERIOD_MS 节流，不会高频全量重绘。
 */
void Display_TaskDbgRefresh(void);

/**
 * @brief  刷新对位调参页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_ALIGN_TEST 页时置脏；`AlnTest_Update()` 按 ALNT_UI_MS 节流调用，
 *         按键操作也调用。实际重绘受 REFRESH_PERIOD_MS 节流。
 */
void Display_AlignTestRefresh(void);

/**
 * @brief  刷新速度模式测试页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_VELTEST 页时置脏；`VelTest_Update()` 按 200ms 节流调用
 *         （运行中要显示实时航向），按键操作也调用。实际重绘受 REFRESH_PERIOD_MS 节流。
 */
void Display_VelTestRefresh(void);

/**
 * @brief  刷新速度模式跑图页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_VELROUTE 页时置脏；只在**状态/步号变化**和按键时调用，
 *         运行中不周期刷新（整页 8x16 重绘会阻塞 ~100ms，拖慢 20ms 控制环）。
 */
void Display_VelRouteRefresh(void);

/**
 * @brief  OLED 自检测试函数（run 回调）
 * @param  无
 * @retval 无
 */
void OLED_Test(void);

/**
 * @brief  进入页面选择菜单
 * @param  无
 * @retval 无
 */
void Display_MenuEnter(void);

/**
 * @brief  退出页面选择菜单（回到进入前页面）
 * @param  无
 * @retval 无
 */
void Display_MenuExit(void);

/**
 * @brief  查询是否处于页面选择菜单
 * @param  无
 * @retval 1=菜单页，0=非菜单页
 */
uint8_t Display_MenuIsActive(void);

/**
 * @brief  菜单内按键处理
 * @param  key: 按键 ID
 * @retval 无
 */
void Display_MenuHandleKey(KeyID_t key);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_OLED_UI_H */
