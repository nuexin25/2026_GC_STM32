/**
 ******************************************************************************
 * @file    bsp_tune.h
 * @brief   在线调参模块（P7 自动调车用）：从**日志串口 UART5** 收 ASCII 命令，
 *          改 bsp_route 的运行期参数，并回一行可解析的结果，PC 脚本据此自动调车
 * @author  nx_car
 * @date    2026-09-23
 ******************************************************************************
 * @note   【为什么用 UART5】UART5 是日志口（`bsp_debug` 只发不收），
 *         usart.c 里它本来就是 TX_RX，把这根 RX 接上 USB-TTL 的 TX 就能"一根线
 *         既看日志又发命令"；而且不用碰 stm32f1xx_it.c（本模块**轮询**收字节，
 *         不走中断）。若你的转接板只接了 TX（常见），命令收不到 —— 那就把
 *         RX 也接上（USB-TTL 的 TX ↔ 板子 UART5_RX）。
 *
 *         命令（ASCII 文本，行尾 \n 或 \r\n，忽略多余空格）：
 *           RUN               启动跑图（= 测试页 KEY3）
 *           STOP              中止跑图（= KEY3 再按一次）
 *           DEFAULT           所有运行期参数恢复 bsp_route.h 里的默认值
 *           DUMP              打印全部可调参数（每行 `[TUNE] name=value`，末尾 `[TUNE] dump end`）
 *           <name>=<value>    改一个参数（越界自动夹到合法范围），回 `[TUNE] set name=value`
 *           ?<name>           读一个参数，回 `[TUNE] name=value`
 *         典型用法（PC 端）：`tableScalePct=40` → `rotVel=100` → `lapRepeat=3` → `RUN`
 *
 *         ★【2026-10-15】对位（P23/P24）在线调参命令（配合 tools\align_autotune.py）：
 *           ALN src=RCP|SHOT|SHOT2   进页 + 选源 + 启动一次对位（手臂先去观察位）
 *                                    （等价于"进 ALNT 页 + KEY1/2 选源 + KEY3"）
 *           ALN STOP                 停止当前对位（不退出页）
 *           ALN EXIT                 退出对位页
 *           ALNCFG src=SHOT kp=120 ki=120 kd=15 rpm=250 wheel=40 \
 *                  ykp=150 yki=10 ykd=10 yrpm=120
 *                                    一次设置该数据源的全部 9 个 PID 参数，
 *                                    回 `[TUNE] alncfg src=.. kp=..`（实际生效值）
 *                                    定标：增益 ×1000、rpm/yrpm ×10、wheel 原值
 *           ALNDUMP                  打印三套参数（每套一行 `[TUNE] alncfg ...`）
 *                                   末尾 `[TUNE] alndump end`
 *         ⚠ 这些命令改的是 RAM，掉电不保存；最优值要抄回 bsp_align.h 的 `ALIGN_<X>_*`。
 *         ⚠ 命令通道是 UART5 的 **RX**：USB-TTL 的 TX 必须接到板子 UART5_RX。
 *
 *         【非阻塞】Tune_Update 只做"读空 UART5 收寄存器 + 攒行 + 解析"，
 *         不发阻塞等待、不用 HAL_Delay；参数改完下一段就生效。
 *         【掉电不保存】在线改的只是 RAM 里的值（见 bsp_route.h 的 RouteCfg_t），
 *         调好后要把最终值抄回宏里。
 ******************************************************************************
 */
#ifndef __BSP_TUNE_H
#define __BSP_TUNE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* ===================== 可调项数量（够用即可，加参数就改 bsp_tune.c 的表） ===================== */
#define TUNE_ENT_MAX     24U

/**
 * @brief  在线调参模块初始化（绑好参数表，清接收状态）
 * @param  无
 * @retval 无
 * @note   在 main 的初始化段调用一次（需在 UART5（bsp_debug）与 Route_Init 之后）。
 */
void Tune_Init(void);

/**
 * @brief  在线调参轮询（主循环调用，非阻塞）
 * @param  无
 * @retval 无
 * @note   ① 把 UART5 收到的字节读空；② 按行解析；③ 执行 RUN/STOP/DUMP/set/get。
 *         没有数据时开销极小（只读一次状态寄存器）。
 */
void Tune_Update(void);

/**
 * @brief  喂一个字节进解析器（供其它接收通道复用，可选）
 * @param  b: 收到的字节
 * @retval 无
 */
void Tune_RxByte(uint8_t b);

/**
 * @brief  取"累计收到的命令字节数"（诊断接线：一直是 0 说明 RX 没接对）
 * @param  无
 * @retval 累计字节数
 */
uint32_t Tune_GetRxBytes(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_TUNE_H */
