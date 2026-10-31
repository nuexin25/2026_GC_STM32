/**
 ******************************************************************************
 * @file    bsp_rpi.h
 * @brief   树莓派视觉通讯模块应用层头文件（P6，USART1 中断收发）
 * @author  nx_car
 * @date    2026-09-15
 ******************************************************************************
 * @note   硬件：树莓派接 USART1（PA9-TX / PA10-RX，115200-8-N-1），
 *         usart.c 已完成初始化，USART1_IRQn 已使能，硬件层无需改动。
 *         协议契约：《树莓派通讯协议.md》v2.0（项目根目录，唯一有效版本）。
 *         **v1.0（$COLOR/$TURNTABLE/$ROUGH/$FINE、$CMD,xxx）已全部作废删除。**
 *
 *         本模块只做"通信管道"：
 *           接收 6 种数据帧（$RCP / $RZ / $CCG / $SHOT / $SHOT2 / ★$RP），
 *           发送 6 种指令帧（$RCP / $RZ / $CCG,<pos> / $SHOT / $SHOT2 / ★$RP / $CODE,<任务码>），
 *           并把像素偏移按标定比例换算成车体 mm 供上层使用。
 *         不做任务流程调度（何时发哪条指令属任务层 P9，见后续开发）。
 *
 *         帧格式：$<内容>*<校验>\r\n；校验 = '$' 之后、'*' 之前逐字节 XOR，
 *         2 位大写十六进制；`**` 表示跳过校验。
 *
 *         硬约束：
 *           - 中断（RPi_RxHandler）只搬字节，解析/换算都在主循环 RPi_Update；
 *           - 全部非阻塞（发送除外：帧短 + 超时保护）；
 *           - 未知帧名静默忽略，不计坏帧（向前兼容）；
 *           - 已知帧名但段数不足 / 校验错 → 丢帧 + 坏帧计数；
 *             数值越界 → 钳位（不丢整帧）；
 *           - 数据过期必须清零数值（不只是置 valid=0）；
 *           - 日志禁 %f（像素与 mm 均为整数）。
 ******************************************************************************
 */
#ifndef __BSP_RPI_H
#define __BSP_RPI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* ===================== 可调参数宏 ===================== */

/* 画面分辨率（协议 §7：相机画面 640x480，原点左上，x 右正 y 下正） */
#define RPI_IMG_W                640
#define RPI_IMG_H                480
#define RPI_IMG_CENTER_X         320    /* 画面中心 x（像素） */
#define RPI_IMG_CENTER_Y         240    /* 画面中心 y（像素） */

/* 图像轴 → 车体轴 的映射 + 符号。
   ★ 现场实测规则（2026-09-28）：
       偏移 X > 0 -> 车**后退**      X < 0 -> 车**前进**
       偏移 Y > 0 -> 车**左移**      Y < 0 -> 车**右移**
     换算成车体量：前进 = -X、右移 = -Y，故：
       · 图像第 1 字段(dx) 喂给"车体前进"，符号 -1
       · 图像第 2 字段(dy) 喂给"车体右移"，符号 -1
   现场若发现方向反了：只改下面 4 个值（或运行期改 RPi_GetCfg()），**不要改逻辑**。 */
#define RPI_IMG_X_TO_BODY        0      /* 图像 x（第 1 字段）→ 车体前进方向 */
#define RPI_IMG_X_SIGN           (-1)   /* X>0 需后退 → 前进量为负 */
#define RPI_IMG_Y_TO_BODY        1      /* 图像 y（第 2 字段）→ 车体右移方向 */
#define RPI_IMG_Y_SIGN           (-1)   /* Y>0 需左移 → 右移量为负 */

/* 历史口径说明（保留供标定溯源）：早期视觉端把数据帧坐标按 0.75 画布缩放输出
   （数据帧 1 px = 4/3 原生 px）。若沿用"原生像素口径"的旧系数 k_native，
   折算到数据帧口径为 k_data = k_native × 4/3。v2.0 直接以数据帧像素为口径标定即可。 */
#define RPI_CANVAS_SCALE_NUM     3      /* 画布缩放分子（0.75 = 3/4） */
#define RPI_CANVAS_SCALE_DEN     4      /* 画布缩放分母 */

/* 像素 → 毫米 比例（**必须实测标定**；2/3 ≈ 0.667mm/px 是占位值）。
   前进方向与右移方向**两轴可能不同，必须分别标定**。
   标定方法：放一个已知尺寸 S(mm) 的物体 → 读视觉端报出的像素跨度 P → k = S / P。 */
#define RPI_PX2MM_NUM_FWD        2      /* 前进方向 mm/px 分子 */
#define RPI_PX2MM_DEN_FWD        3      /* 前进方向 mm/px 分母（2/3 ≈ 0.667） */
#define RPI_PX2MM_NUM_RIGHT      2      /* 右移方向 mm/px 分子 */
#define RPI_PX2MM_DEN_RIGHT      3      /* 右移方向 mm/px 分母 */

/* 像素死区：|偏移| ≤ 本值才算"已对准"（对位收敛判据）。
   ★★ **按任务点分档**（2026-10-10 用户要求）：两档的区别只在于"要多准才算到"，
      由 `bsp_align` 在启动对位时**按数据源自动选择**（`Al_Begin` 里写 `RPi_GetCfg()->deadbandPx`）：
        · **常规档**（转盘区 `$RCP`，即 `TRAY PICK`）：**7px**；
        · **严格档**（打靶/暂存区 `$SHOT`/`$SHOT2`）：**7px**；
          【2026-10-15 用户要求】"**位移死区 ±7px**" -> 两档统一 7px（原 5px）。
          【沿革】2026-10-29 曾临时放到 9px 提速；2026-10-30 曾收到 3px（±3px 放置精度）；
          ★2026-10-31 用户要求"对位精度改回原来的版本" -> **恢复 7px**。
      两档**都只在真正进死区时才判成功**（`bsp_align` 的"末端放宽出口"已于 2026-10-10 删除）。
      ⚠ 若打靶对位始终进不了死区（一直 `FAIL align fail`），先把 `RPI_PX_DEADBAND_STRICT`
      放到 8~10；再检查 `RPI_PX2MM_*` 标定 / `ALIGN_XY_RPM_MIN`（保底速度太小会卡住不动）。 */
#define RPI_PX_DEADBAND          7      /* 常规档死区（转盘 $RCP 对位用） */
#define RPI_PX_DEADBAND_STRICT   7      /* 严格档死区（$SHOT/$SHOT2 对位用；7px ≈ 4.6mm）
                                           ★2026-10-31 用户要求恢复原值（3 -> 7）。 */
#define RPI_PX_MAX               1000   /* 像素钳位上限（协议合法范围 ±1000），单位 px */

/* 帧定界 / 过期 / 超时 */
/* 成帧方式：**按行定界** —— 遇 0x0A 即认为一行结束，0x0D 直接丢弃。
   本宏仅为兼容旧文档保留，不再参与成帧。 */
#define RPI_FRAME_TIMEOUT_MS     50U    /* 【已停用·不用于成帧】帧间空闲超时(ms) */

/* 单行最大长度。协议最长入站帧 $SHOT,-1000,-1000,-1000,-1000,-1000*XX 约 45 字节，
   取 96 留足余量；**超限则丢弃整行并计数**（不静默截断，便于发现视觉端发脏数据）。 */
#define RPI_MAX_LEN              96U

/* 接收环形缓冲大小：中断按字节写入、主循环取整行解析。 */
#define RPI_RX_RING_SIZE         256U

/* 数据帧过期阈值(ms)：连续未收到 → 置于无效**并清零数值**（协议 §8 要求 500ms）。 */
#define RPI_DATA_EXPIRE_MS       500U

/* 心跳 $PING 超时(ms)：连续未收到 → 判视觉端离线（协议 §8 要求 2500ms）。
   与数据超时是**两个独立计时器**：心跳只回答"视觉端在不在"，不表示数据有效。 */
#define RPI_HB_TIMEOUT_MS        2500U

#define RPI_OFFLINE_TIMEOUT_MS   3000U  /* 【仅日志】上电 3s 内一个字节都没收到 → 打一条 offline */
#define RPI_TX_TIMEOUT_MS        100U   /* 发送阻塞超时保护(ms) */

/* 对位比例控制参数（**供后续任务层用，本模块只定义不调用**） */
#define RPI_ALIGN_KP_NUM         7      /* 步长比例分子 */
#define RPI_ALIGN_KP_DEN         10     /* 步长比例分母（7/10 = 0.7） */
#define RPI_ALIGN_MAX_STEP_MM    60     /* 单次最大步长(mm) */
#define RPI_ALIGN_MIN_STEP_MM    4      /* 单次最小步长(mm) */
#define RPI_ALIGN_MAX_ITER       6      /* 单点最多迭代次数 */
#define RPI_ALIGN_TIMEOUT_MS     3000U  /* 单点对位总超时(ms) */

/* 任务码长度："134+123+314+231" = 15 字符（固定） */
#define RPI_TASK_CODE_LEN        15U
/* 任务码占位串：扫不到码时发送，长度同为 RPI_TASK_CODE_LEN，便于树莓派侧判无效 */
#define RPI_TASK_CODE_PLACEHOLDER "---+---+---+---"

/* 颜色编号（协议 §4.2/§5）：1红 2黄 3蓝 4绿；0=未识别。
   ★ 注意：v2.0 起颜色编号与旧文档/旧代码（1红2蓝3绿4黄）不同，以本文为准。 */
#define RPI_COLOR_RED            1U
#define RPI_COLOR_YELLOW         2U
#define RPI_COLOR_BLUE           3U
#define RPI_COLOR_GREEN          4U

/* 未知帧名诊断开关：协议要求"未知帧静默忽略"（默认 0 = 不打印）。
   联调期怀疑"树莓派发的帧名拼错/STM32 不认"时，临时改 1 可把未知帧名打到 UART5，
   定位完请改回 0（它不影响任何计数，只是多一行日志）。 */
#ifndef RPI_LOG_UNKNOWN_FRAME
#define RPI_LOG_UNKNOWN_FRAME    0
#endif

/* ===================== 数据结构 ===================== */

/* $RCP —— 转盘中心偏移（像素，相对画面中心） */
typedef struct {
    int16_t  dxPx;       /* 转盘中心相对画面中心的像素偏移，右正 */
    int16_t  dyPx;       /* 同上，下正 */
    int16_t  dxMm;       /* 换算后车体前进方向修正量 mm */
    int16_t  dyMm;       /* 换算后车体右移方向修正量 mm */
    uint8_t  valid;      /* 1=有效(未过期) 0=过期/未收到（此时数值已清零） */
    uint32_t lastRxTick; /* 最后收到时刻(ms)，用于过期判断 */
} RPi_Rcp_t;

/* $RZ —— 转盘三个位号的位号/颜色 */
typedef struct {
    uint8_t  pos[3];     /* 原始顺序的位号（1~3） */
    uint8_t  color[3];   /* 与 pos[] 一一对应的颜色（0=未识别） */
    uint8_t  byPos[4];   /* 查表：byPos[1~3] = 该位号的颜色；0=无效（过期时清零） */
    uint8_t  valid;      /* 1=有效(未过期) 0=过期/未收到 */
    uint32_t lastRxTick; /* 最后收到时刻(ms) */
} RPi_Rz_t;

/* $CCG —— 指定位号的颜色（应答） */
typedef struct {
    uint8_t  reqPos;     /* 本端最近一次请求的位号（1~3），便于核对 */
    uint8_t  color;      /* 该位号颜色：1~4；0=未识别 */
    uint8_t  valid;      /* 1=有效(未过期) 0=过期/未收到 */
    uint32_t lastRxTick; /* 最后收到时刻(ms) */
} RPi_Ccg_t;

/* $SHOT —— 打靶区数据 */
typedef struct {
    int16_t  q[3];       /* 三个圆环的平行度（本阶段不使用，保留字段） */
    int16_t  dxPx;       /* 中间圆环中心偏移，右正 */
    int16_t  dyPx;       /* 下正 */
    int16_t  dxMm;       /* 换算后车体前进方向修正量 mm */
    int16_t  dyMm;       /* 换算后车体右移方向修正量 mm */
    uint8_t  valid;      /* 1=有效(未过期) 0=过期/未收到 */
    uint32_t lastRxTick; /* 最后收到时刻(ms) */
} RPi_Shot_t;

/* $RP —— ★转盘圆盘平行度（新增协议《新增协议_RP.md》，2026-10-15）
   帧：`$RP,q1*XX`，q1 单位 **0.1°**（(实测角度 − 零点) × 10），正 = 顺时针。
   ⚠ **q1 = 0 有两种含义**：① 真的对齐了 ② 任一侧没检到红线（= 无数据）。
     视觉端**不给有效标志**，协议建议"连发 2~3 次全 0 且盘上有红线 -> 视为未检出，不动作"。
     本模块只保证"帧到达且数值在范围内 = valid"；"0 到底可不可信"由上层按
     `RPi_RpSeenNonZero()` 处理（对位过程中见过非 0，就把 0 当"已对齐"）。 */
typedef struct {
    int16_t  q1;         /* 圆盘平行度，单位 0.1°（±900 内），正 = 顺时针 */
    uint8_t  valid;      /* 1=有效(未过期) 0=过期/未收到 */
    uint32_t lastRxTick; /* 最后收到时刻(ms) */
} RPi_Rp_t;

/* 视觉端状态：由 `$PING,<MODE>` 上报（协议 §4.5；本端不自行切换状态） */
typedef enum {
    RPI_VS_UNKNOWN = 0,   /* 尚未收到过 $PING */
    RPI_VS_IDLE,          /* IDLE：待机，不发数据帧 */
    RPI_VS_TURN,          /* TURN：转盘阶段，正在响应 $RCP/$RZ/$CCG */
    RPI_VS_SHOT           /* SHOT：打靶阶段，正在响应 $SHOT */
} RPi_VsMode_t;

/* 像素 → 车体毫米 的映射配置（**现场标定项**，做成运行期可调）。
   为什么用运行期结构体：px→mm 系数与轴映射需要现场反复试错，编译期宏每改一次
   都要重烧固件；做成变量后可在线改（RPi_GetCfg 取指针直接写），改完立即生效，
   初值 = 下面的宏，掉电不保存；标定好后把结果抄回宏。 */
typedef struct {
    int16_t  anchorX;      /* 锚点像素 x：期望把目标对到的位置，默认 320(画面中心) */
    int16_t  anchorY;      /* 锚点像素 y，默认 240 */
    /* 图像轴 → 车体轴 的对应关系（相机安装决定，必须实测标定）
       值：0 = 车体前进方向，1 = 车体右移方向；两者应互不相同 */
    uint8_t  imgXToBody;   /* 图像 x 对应车体哪一轴，默认 0（前进） */
    int8_t   imgXSign;     /* 图像 x 方向符号，默认 -1；方向反了取 +1 */
    uint8_t  imgYToBody;   /* 图像 y 对应车体哪一轴，默认 1（右移） */
    int8_t   imgYSign;     /* 图像 y 方向符号，默认 -1 */
    /* 像素→毫米比例：px2mmNum[0]/px2mmDen[0] = 前进方向 mm/px
                      px2mmNum[1]/px2mmDen[1] = 右移方向 mm/px */
    uint16_t px2mmNum[2];  /* 分子（下标 0=前进 1=右移） */
    uint16_t px2mmDen[2];  /* 分母，为 0 时该轴输出 0 并报警（防除零） */
    int16_t  deadbandPx;   /* 像素死区：|偏移px| ≤ 本值视为已对准 */
    int16_t  maxPx;        /* 像素钳位上限（防视觉乱跳） */
} RPi_Cfg_t;

/* ===================== 对外接口 ===================== */

/* ---- 生命周期 ---- */

/**
 * @brief  树莓派通讯模块初始化
 * @param  无
 * @retval 无
 * @note   复位接收状态与配置（配置取 .h 宏默认值），启动 USART1 单字节中断接收。
 *         **不在初始化阶段发送任何指令**（避免上电就给树莓派发指令）。
 *         上电调用一次，通讯此后台常驻，页面进入/退出不影响接收。
 */
void RPi_Init(void);

/**
 * @brief  通讯主循环节拍（非阻塞）
 * @param  无
 * @retval 无
 * @note   主循环调度：行定界解析 + 数据过期检查（过期清零数值）+ 心跳离线判定。
 *         禁止在中断中调用，禁止 HAL_Delay。
 */
void RPi_Update(void);

/* ---- 取数据（协议 §4） ---- */

/**
 * @brief  取 $RCP 结果（转盘中心偏移）
 * @param  无
 * @retval 结构体常量指针（valid=0 表示过期/未收到，此时数值已全部清零）
 */
const RPi_Rcp_t  *RPi_GetRcp(void);

/**
 * @brief  取 $RZ 结果（转盘三位号/颜色）
 * @param  无
 * @retval 结构体常量指针（valid=0 时 pos/color/byPos 均已清零）
 */
const RPi_Rz_t   *RPi_GetRz(void);

/**
 * @brief  取 $CCG 结果（指定位号的颜色应答）
 * @param  无
 * @retval 结构体常量指针（valid=0 时数值已清零）
 */
const RPi_Ccg_t  *RPi_GetCcg(void);

/**
 * @brief  取 $SHOT 结果（打靶区数据）
 * @param  无
 * @retval 结构体常量指针（valid=0 时数值已清零）
 */
const RPi_Shot_t *RPi_GetShot(void);

/**
 * @brief  取 $SHOT2 结果（★ROUGH STACK 用：瞄准"三个横排物料中间那个"的颜色质心）
 * @param  无
 * @retval 结构体常量指针（valid=0 时数值已清零）
 * @note   ★【视觉组《SHOT2通讯对接.md》】`$SHOT2` 与 `$SHOT` **不是同一个测量**：
 *         `$SHOT` 瞄中间**圆环圆心**；`$SHOT2` 瞄**中间物料的颜色质心**。
 *         两套数据各自独立、**互不通用**（rpiShot / rpiShot2）。
 *         字段定义与 `$SHOT` 完全一致（q1 单位 0.1°、q2/q3 恒 0、dx/dy 像素、`0,0,0,0,0`=未识别）。
 */
const RPi_Shot_t *RPi_GetShot2(void);

/**
 * @brief  取 $RP 结果（★转盘圆盘平行度，新增协议《新增协议_RP.md》）
 * @param  无
 * @retval 结构体常量指针（valid=0 表示过期/未收到，此时 q1 已清零）
 * @note   q1 单位 0.1°，正 = 顺时针。
 *         ⚠ **q1=0 可能是"真对齐"也可能是"没检到红线"**（视觉端不给有效标志），
 *         上层要按"这次对位里是否见过非 0"来判定 0 可不可信。
 */
const RPi_Rp_t   *RPi_GetRp(void);

/**
 * @brief  便捷查询：在最近一次 $RZ 结果里按颜色找位号
 * @param  color: 目标颜色编号（1红 2黄 3蓝 4绿）
 * @retval 该颜色所在位号 1~3；找不到 / $RZ 无效 返回 0
 * @note   用 byPos[] 查表，仅当 $RZ valid 时有效（过期已自动清零）。
 */
uint8_t RPi_FindColorPos(uint8_t color);

/**
 * @brief  便捷查询：取某位号的颜色（由 $RZ 的 byPos 表）
 * @param  pos: 位号 1~3
 * @retval 该位颜色 1~4；0=未识别/无效/越界
 */
uint8_t RPi_GetColorAtPos(uint8_t pos);

/**
 * @brief  像素偏差 -> 车体位移修正量（**唯一换算入口**，$RCP/$SHOT 内部也走它）
 * @param  dxPx:  图像 X 像素偏差（右正）
 * @param  dyPx:  图像 Y 像素偏差（下正）
 * @param  outFwdMm:   输出：车体**前进**方向修正量 mm（正=前进）
 * @param  outRightMm: 输出：车体**右移**方向修正量 mm（正=右移）
 * @retval 无
 * @note   【给分轴对位用】映射是"每个图像轴各自累加到它对应的车体轴"，可**只传一个轴**
 *         （另一轴传 0），结果与整对换算在该轴上的分量一致 —— `bsp_align` 的
 *         "X/Y 分轴"闭环就靠这个特性。
 *         内部含锚点补偿、±`maxPx` 钳位、轴映射与符号、px→mm 比例（整数运算、防除零）。
 */
void RPi_PxToBodyMm(int16_t dxPx, int16_t dyPx,
                    int16_t *outFwdMm, int16_t *outRightMm);

/* ---- 状态查询 ---- */

/**
 * @brief  查询 USART1 是否曾经收到过字节（**线路级**诊断，永不回落）
 * @param  无
 * @retval 1=收到过任何字节；0=从未收到
 * @note   不是"视觉端在线"的判据 —— 那个用 RPi_IsAlive()（心跳）。
 */
uint8_t RPi_IsOnline(void);

/**
 * @brief  查询视觉端是否在线（**心跳判据**，协议 §8）
 * @param  无
 * @retval 1=最近 RPI_HB_TIMEOUT_MS(2500ms) 内收到过 $PING；0=超时（视觉端离线）
 * @note   与"数据是否有效"是两件事：视觉端在线也可能因当前 IDLE 而不发数据帧。
 */
uint8_t RPi_IsAlive(void);

/**
 * @brief  取视觉端当前状态（由 `$PING,<MODE>` 上报）
 * @param  无
 * @retval RPi_VsMode_t；尚未收到过 $PING 时为 RPI_VS_UNKNOWN
 */
RPi_VsMode_t RPi_GetVsMode(void);

/**
 * @brief  取有效帧累计计数（测试页显示）
 * @param  无
 * @retval 已知帧名且解析通过的帧数（未知帧名不计入）
 */
uint32_t RPi_GetRxCount(void);

/**
 * @brief  取坏帧累计计数（测试页显示）
 * @param  无
 * @retval 校验失败或已识别帧名但格式非法（段数不足）的帧数
 * @note   未知帧名**不计入**坏帧（协议 §8）
 */
uint32_t RPi_GetBadCount(void);

/**
 * @brief  取最近一帧摘要文本（测试页用）
 * @param  无
 * @retval 静态缓冲指针，如 "RCP 42,-15" / "RZ 1,3,4" / "BAD"；无帧时返回 "---"
 */
const char *RPi_GetLastSummary(void);

/* ---- 发送指令（协议 §3） ---- */

/**
 * @brief  发送 $RCP：请求转盘中心 xy 偏移（小车到达转盘区停稳后）
 * @param  无
 * @retval 无
 * @note   组帧 $RCP*41（XOR 2 位大写 HEX），阻塞发送带 RPI_TX_TIMEOUT_MS 超时保护。
 */
void RPi_ReqRcp(void);

/**
 * @brief  发送 $RZ：请求转盘三个位号的位号/颜色（手臂进入观察位后）
 * @param  无
 * @retval 无
 * @note   组帧 $RZ*08。
 */
void RPi_ReqRz(void);

/**
 * @brief  发送 $CCG,<pos>：请求指定位号的颜色（手臂预置到准备位后轮询）
 * @param  pos: 位号，仅允许 1~3（越界则**不发送**并打日志）
 * @retval 无
 * @note   组帧 $CCG,<pos>*CS；同时记录 reqPos 供应答核对。
 */
void RPi_ReqCcg(uint8_t pos);

/**
 * @brief  发送 $SHOT：进入打靶阶段，请求圆环数据（到达作业区停稳后）
 * @param  无
 * @retval 无
 * @note   组帧 $SHOT*00。
 */
void RPi_ReqShot(void);

/**
 * @brief  发送 $SHOT2：请求"中间物料颜色质心"的偏移 + 平行度（★ROUGH STACK 用）
 * @param  无
 * @retval 无
 * @note   组帧 `$SHOT2*32`（校验由发送层自动算）。
 *         视觉组《SHOT2通讯对接.md》：`$SHOT2` 与 `$SHOT` **是两个不同的测量**，
 *         收到 `$SHOT2` 后视觉端进入 `SHOT2` 模式并回 `$SHOT2,q1,q2,q3,dx,dy`。
 */
void RPi_ReqShot2(void);

/**
 * @brief  发送 $RP：请求**转盘圆盘平行度**（新增协议《新增协议_RP.md》）
 * @param  无
 * @retval 无
 * @note   组帧 `$RP*02`（XOR('R','P')=0x02，由 RPi_SendFrame 自动算）。
 *         视觉端收到后切 **TURN 模式（640×480）**，后续 `$RCP/$RZ/$CCG` 同一视图、无切换延迟。
 *         回帧 `$RP,<q1>`（单位 0.1°）；q1=0 可能是"对齐"也可能是"没检到"。
 */
void RPi_ReqRp(void);

/**
 * @brief  下发任务码 $CODE,<任务码>
 * @param  task_code: 任务码字符串，如 "134+123+314+231"
 * @retval 无
 * @note   task_code 为 NULL 或长度不等于 RPI_TASK_CODE_LEN(15) 时，
 *         改发占位码 "---+---+---+---" 并打一条日志提醒（避免发出畸形帧）。
 */
void RPi_SendCode(const char *task_code);

/**
 * @brief  ★【2026-10-31 新赛规】**正确抓放计数**上报（`$CNT`）
 * @param  op: 0=新一轮清零(`$CNT,0`，屏幕"抓0 放0")；1=抓到(`$CNT,1`)；2=放好(`$CNT,2`)
 * @retval 无
 * @note   协议《抓取计数协议.md》：视觉端收到即回 `$CNT,<抓>,<放>*XX` 并刷新屏幕下 1/3，
 *         **本工程只发不管**（回帧可选核对，不在本模块的解析表里，会被静默忽略）。
 *         校验由 `RPi_SendFrame` 自动算：`CNT,0`=0x45 / `CNT,1`=0x44 / `CNT,2`=0x47。
 *         发送时机：清零=每轮开始（`QR()` 成功那拍）；抓=夹爪**夹紧确认**之后；
 *                   放=夹爪**松开且抬升完成**之后（见 bsp_arm_action 的 `actCntPlacePend`）。
 */
void RPi_ReqCnt(uint8_t op);

/* ---- 换算配置（供任务层标定）---- */

/**
 * @brief  取可调配置（在线标定直接改）
 * @param  无
 * @retval 可写配置指针；改完立即生效（下一帧解析时体现），掉电不保存
 */
RPi_Cfg_t *RPi_GetCfg(void);

/**
 * @brief  把配置恢复成 .h 宏默认值
 * @param  无
 * @retval 无
 * @note   上电时 RPi_Init 会调一次；在线标定试错后可调它一键回退。
 */
void RPi_CfgReset(void);

/* ---- 接收（中断中调，由 bsp_imu.c 回调转发）---- */

/**
 * @brief  USART1 单字节接收中断处理
 * @param  b: 刚收到的字节
 * @retval 无
 * @note   中断上下文调用。**只搬字节**入缓冲、刷新计数与时间戳，
 *         不解析、不换算、不调 Display、不 printf。缓冲满则丢弃后续字节。
 */
void RPi_RxHandler(uint8_t b);

/**
 * @brief  通知 USART1 接收错误（ORE/FE/NE/PE）
 * @param  无
 * @retval 无
 * @note   由 bsp_imu.c 的 HAL_UART_ErrorCallback 调用，本函数只累加错误计数；
 *         实际重装 RX IT 由 ErrorCallback 完成。中断上下文，禁止阻塞。
 */
void RPi_NotifyError(void);

#ifdef __cplusplus
}
#endif

/* ===================== USART1 接收缓冲（供 bsp_imu.c 回调重装 IT 用） ===================== */
/* 非 static：HAL_UART_Receive_IT 需在此变量地址写入收到的字节，bsp_imu.c 的
   HAL_UART_RxCpltCallback 中 extern 引用后重装。做法与 qrRxByte 保持一致。 */
extern uint8_t rpiRxByte;

#endif /* __BSP_RPI_H */
