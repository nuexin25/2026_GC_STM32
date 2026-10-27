/**
 ******************************************************************************
 * @file    bsp_align.h
 * @brief   视觉闭环定位（P9）：按树莓派回的像素偏移，把车挪到偏移 (0,0)
 * @author  nx_car
 * @date    2026-09-28
 ******************************************************************************
 * @note   【做什么】车停稳在作业点后，视觉端持续回偏移量；本模块把偏移收敛到死区内。
 *
 *         【控制律：★2026-10-15 起改为 **PID + 车轮速度模式**（快速定位）】
 *           · **三轴各一套 PID**（前进 / 横移 / 旋转），输出直接是**车轮 RPM**，
 *             逆解后四轮背靠背下发（`Align_VelMoveVel`）；
 *           · **X/Y 同时闭环**（`ALIGN_PID_XY_SIMUL=1`），不再"先 X 再 Y"走两趟；
 *           · **只在新视觉帧上更新一次 PID**（帧间保持速度；速度模式下电机自己保持）；
 *           · 积分消"打滑/间隙"残差、微分抑制过冲 => 可用更高速度、更少来回。
 *           参数与调法见下面 `ALIGN_PID_*` 宏块；`ALIGN_PID_ENABLE=0` 可退回旧控制律。
 *
 *         【旧控制律（备用，`ALIGN_PID_ENABLE=0`）：速度模式 + X/Y **分轴** + 像素→速度线性映射】
 *           ① 一轴一轴来：先把 X（图像 dx）消进死区，再消 Y（图像 dy）。
 *           ② 越近越慢、速度连续（`ALIGN_XY_PX_*` ↔ `ALIGN_XY_RPM_*` 线性插值）。
 *           ③ 收敛判据仍是**像素死区**（直接测量量，不受 mm 系数误差影响）。
 *              ★ 死区**按任务点分两档**：转盘 `$RCP` 用 `RPI_PX_DEADBAND`(6px)，
 *                打靶 `$SHOT` 用 `RPI_PX_DEADBAND_STRICT`(1px)；由 Al_Begin 按数据源自动选。
 *           ④ 到位前先**停轮**再静置，等车真停稳、视觉刷到新一帧才判成功
 *              （避免"边停边判"把滑行量算漏）。
 *         原"偏差 ≤ALIGN_FINE_PX 就切**位置模式小步**收尾"的精调段**已删除**：
 *         现在全程速度模式，靠"保底速度 + 短节拍"收进死区。
 *
 *         【方向约定】**按实测规则**（与"相对画面中心"的原始语义不同，别混）：
 *             偏移 X > 0 -> 车**后退**      X < 0 -> 车**前进**
 *             偏移 Y > 0 -> 车**左移**      Y < 0 -> 车**右移**
 *         即车体位移 = (−X, −Y)。这段换算**走 bsp_rpi 的唯一换算入口**
 *         `RPi_PxToBodyMm()`，方向与符号由 `RPi_GetCfg()` 里的
 *         `imgXToBody/imgXSign/imgYToBody/imgYSign` 决定（默认已按上面这条规则设好）。
 *         现场若发现方向反了：改那几个符号即可，**不要改本文件的逻辑**。
 *
 *         【车身角度】**平移段** dRot = 0（只平移、不旋转，车身朝向不变）；
 *         但**打靶档（`$SHOT`，STORE/ROUGH PLACE）会先做一段"车身矫正"（AL_YAW）**：
 *         用 `$SHOT` 回的**平行度** `q1/q2/q3` 把车**原地转正**，再进入平移段 —— 见 `ALIGN_YAW_*`。
 *
 *         【为什么不能"一步到位"】像素->毫米系数不准 + 麦轮打滑 + 视觉有滞后，
 *         一次算出的距离必然有残差。所以全程都是"边走边读"的闭环，收敛判据用的是
 *         **像素**（直接测量量），不受 mm 系数误差影响。
 *
 *         硬约束：全部非阻塞（禁止 HAL_Delay）；日志禁 %f；日志有限流。
 ******************************************************************************
 */
#ifndef __BSP_ALIGN_H
#define __BSP_ALIGN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* ===================== 可调参数 ===================== */
/* 【本轮的定位策略】**不限次数、不限总时长**：一直做闭环，直到偏移进死区才停。
   只保留一个"视觉完全没数据"的兜底（ALIGN_NO_DATA_FAIL_MS），其余超时全部取消。 */

/* 速度命令下发节拍，单位 ms。**两个档，按偏差大小自动切换**：
 *   ALIGN_TICK_MS      常规（该轴偏差在插值段/封顶档）：50，与 CAN 上"位置查询 + 四轮速度"的约定节拍一致；
 *   ALIGN_TICK_FINE_MS 末端（该轴偏差 ≤ ALIGN_XY_PX_MIN、速度为保底档）：20。
 * 【为什么末端要更快】**每拍位移 = 速度 × 节拍**。末端速度已经很低，
 * 节拍再长就是"一拍走过头、下一拍再走回来"，看着就是一顿一顿。
 * 例：3RPM ≈ 15.7mm/s —— 50ms 一拍走 0.79mm，而像素死区只有几 px(≈几 mm)，
 * 余量很小、容易冲过头；20ms 一拍只走 0.31mm，才能稳稳落进死区。 */
#define ALIGN_TICK_MS           50U
#define ALIGN_TICK_FINE_MS      20U

/* 对位期间"重新请求视觉数据"的周期，单位 ms，默认 100，可调 50~500。
 * 【为什么必须有】协议 v2.0 是**请求/应答式**（不再像 v1.0 那样由树莓派持续推送）：
 *   树莓派只在收到 `$RCP`/`$SHOT` 时回一帧，且数据 500ms 不更新就判无效。
 *   对位要来回迭代好几拍，所以**必须周期重发请求**；否则第一拍之后就没有新帧，
 *   对位会卡在"等新帧"上**完全不动**（现象：车一动不动 / 走到一半停住）。
 * 调法：比树莓派"识别 + 回帧"耗时略大即可；调小只是多占点串口，不影响精度。 */
#define ALIGN_REQ_POLL_MS       100U

/* 卡住保护：连续这么久"偏差没有变小"（轮子打滑/没转起来）就把速度**抬到上限档**，
 * 并打一条 stall 日志；偏差重新变小即自动复位。 */
#define ALIGN_STALL_MS          1500U

/* ===== 分轴 + **像素→速度连续映射**（2026-10-09 二次改）=====
   ★ 控制律：**越靠近目标越慢**，速度随偏差**逐级递减**（不再只有两档），
     所以动起来速度是连续的、不会"忽快忽慢"：
        该轴 |偏差| ≥ ALIGN_XY_PX_MAX  -> ALIGN_XY_RPM_MAX      （封顶）
        该轴 |偏差| ≤ ALIGN_XY_PX_MIN  -> ALIGN_XY_RPM_MIN      （保底，防低速堵转）
        两者之间                      -> **线性插值**
   四个旋钮就是"速度上下限 ↔ 像素上下限"：

   | 该轴像素偏差        | 下发速度 |
   |---|---|
   | ≥ `ALIGN_XY_PX_MAX` | `ALIGN_XY_RPM_MAX` |
   | …线性递减…          | … |
   | ≤ `ALIGN_XY_PX_MIN` | `ALIGN_XY_RPM_MIN` |

   现象与调法：
     - 远距离太慢         -> 调大 ALIGN_XY_RPM_MAX，或调小 ALIGN_XY_PX_MAX（更快到封顶）
     - 末端过冲 / 来回摆  -> 调小 ALIGN_XY_RPM_MIN，或调大 ALIGN_XY_PX_MIN（更早进保底速度）
     - 车"嗡嗡响但不走"   -> 调大 ALIGN_XY_RPM_MIN（低速堵转）
     - 想更细腻平顺       -> 把 RPM 上下限拉开（级数更多、阶梯更细）
     - 起步/收尾一顿      -> 调小 ALIGN_VEL_SLEW_RPM
   注：轮速最终按**整数 RPM** 下发，所以曲线是"逐级"的 —— 上下限差得越大、级数越多、越平顺。 */
#define ALIGN_XY_PX_MIN         0      /* 像素下限：≤ 本值 -> 用速度下限 */
#define ALIGN_XY_PX_MAX         180     /* 像素上限：≥ 本值 -> 用速度上限 */
#define ALIGN_XY_RPM_MIN        0      /* 速度下限（车轮 RPM） */
#define ALIGN_XY_RPM_MAX        15     /* 速度上限（车轮 RPM） */

/* 每个下发周期允许的轮速变化量，单位 RPM，默认 6（= 120RPM/s）。
 * 作用 = 软件限斜率：起步不会"啪"一下到速、收尾不会"哐"一下停（丝滑的关键之一）。
 * 太大 -> 利落但冲；太小 -> 软但肉；0 = 不限斜率。 */
#define ALIGN_VEL_SLEW_RPM      15

/* ===== ★★【2026-10-15 用户要求】对位改 **PID + 车轮速度模式**（快速定位） =====
 * 背景：原控制律是"X/Y **分轴** + 像素→速度**线性插值**"（无 I、无 D）：
 *   · 一次只消一个轴（X 消完再消 Y），等于要"走两趟"；
 *   · 无积分 -> 轮子打滑/机构间隙造成的稳态残差只能靠"再跑一遍"硬磨；
 *   · 无微分 -> 只能靠"低速度 + 低上限"防过冲，整体必然偏慢。
 * 改为：**三轴（前进/横移/旋转）各一套 PID**，输出直接是**车轮 RPM**（速度模式），
 *   逆解后四轮背靠背下发。积分消稳态残差、微分抑制过冲 => 可以用**更高速度**跑、更少来回。
 *
 * 【调参顺序】现场照这个来，一次只动一项：
 *   ① 先调 `ALIGN_PID_KP`：0.05 起往上加，加到"接近很快、但末端开始过冲"；
 *   ② 加 `ALIGN_PID_KD`（0.005~0.02）把过冲压住；
 *   ③ 最后加 `ALIGN_PID_KI`（0.002~0.02）消"总是差 1~3px"的稳态残差。
 * 【现象对照】
 *   - 接近慢 / 末端不动          -> KP 太小，或 RPM_MAX 太小，或静摩擦（加 KI）
 *   - 冲过头、来回摆             -> KP 太大 / KD 太小；也可降 RPM_MAX
 *   - 一直小幅抖（嗡嗡响）       -> KD 太大（把像素量化噪声放大了）或 D_FILTER 太小
 *   - 总差 1~3px 到不了 (0,0)    -> KI 太小，或 `ALIGN_PID_I_SEP_MM` 太小（积分没机会积累）
 *
 * 【单位】误差 = **像素**（`RPi_PxToBodyMm` 已把像素换成**车体 mm** 并带方向/轴映射，
 *   所以 K* 实际是"**每 mm 误差多少 RPM**"，mm/px 系数已经含在里面）。
 *   输出 = 车轮 RPM（与 `ALIGN_XY_RPM_*` 同一量纲，可直接对照）。 */
#define ALIGN_PID_ENABLE        1U      /* 1=PID（默认）；0=退回旧的"分轴线性查表" */
#define ALIGN_PID_XY_SIMUL      1U      /* 1=X/Y **同时**闭环（快）；0=先 X 后 Y（旧口径） */

/* ---- 三个数据源**共用**的量（一般不用动） ---- */
#define ALIGN_PID_I_MAX         4.0f    /* 平移积分项**输出**限幅（RPM），防积饱和 */
#define ALIGN_PID_I_SEP_MM      10.0f   /* 平移积分分离：|误差| ≤ 本值才积分（mm）
                                           【2026-10-15 由 60 改 10】60mm 时积分在整段接近里
                                           一直累积 -> 到点已经积到 4RPM，末端必冲过头；
                                           只允许"最后 1cm"积，专治静摩擦残差。 */
#define ALIGN_PID_D_FILTER      0.6f    /* 微分低通"新值权重"（0~1） */
#define ALIGN_PID_KICK_RPM      2.5f    /* 卡住踢一脚的抬速档（RPM）
                                           【2026-10-15 由 10 降到 2.5】10RPM ≈ 14mm/帧，
                                           一脚就把 1~2px 死区冲过 20 倍 -> 极限环 ±40mm。 */
#define ALIGN_PID_KICK_MAX_MS   300U    /* ★踢脚**最长持续**时间（ms）：到点自动撤回 PID。
                                           【为什么必须有】原实现 `alBoost` 一旦置 1 会**锁存**，
                                           只要误差没刷新得更小就每拍都按 KICK 档输出 ->
                                           车一直满速跑。现改为"限时踢"，最多踢这么久。 */
/* ★【2026-10-15 用户要求】"更快速对位 + 精度也高" -> **静摩擦补偿（最小有效输出）**
 *   现象：末端 PID 输出 0.1~0.5 RPM 时麦轮**根本不转**（日志 `u=0.3 w=0` 停 1.4s），
 *         只能等 ALIGN_STALL_MS 后踢一脚 -> 走走停停，既慢又不准。
 *   做法：只要 PID **想动**（|u| > 0），就至少给本值；方向不变。
 *   取值：0.8 RPM 单轴 ≈ 0.85px/帧。
 *   ⚠【2026-10-15 用户反馈】曾把本值提到 4.0（"一步到位"）-> 现场**更严重**，
 *     已按用户要求**退回 0.8**（与 16:41 版本一致）。要再提速请改用调参脚本整定，
 *     不要直接抬这个值。
 *   调法：还"嗡嗡响但不走" -> 1.2；末端来回抖 -> 0.5。0 = 关闭补偿。 */
#define ALIGN_PID_MIN_U         0.8f
#define ALIGN_PID_YAW_I_MAX     3.0f    /* 旋转积分项限幅（RPM） */

/* ===== ★★【2026-10-15 用户要求】**三套参数分开调**：$RCP / $SHOT / $SHOT2 =====
 *   `Al_Begin()` 按数据源自动选表（`alPidCfg[alSrc]`），**不用改代码**。
 *   每套 = 平移 5 个（KP/KI/KD/RPM_MAX/WHEEL_MAX）+ 旋转 4 个（YAW_KP/KI/KD/RPM_MAX）。
 *   在 ALNT 测试页选对应源跑、按 P24 判据表改**对应那一套**，三处互不影响。
 *   单位：KP=RPM/(mm)、KI=RPM/(mm·s)、KD=RPM/(mm/s)；旋转轴的误差单位是 **0.1°**。
 *   调参顺序仍是：先 KP -> 再 KD -> 最后 KI（见 P23 第 3 节）。 */

/* --- ① $RCP（转盘 TRAY PICK；死区 7px） ---
   ⚠【2026-10-15】现场反馈"提高 KP 提速"后要退回上一版 -> 本组恢复为
     0.120 / 0.008 / 0.010 / 25（与 18:19 版本一致）。要提速请用在线调参整定。 */
#define ALIGN_RCP_KP            0.160f
#define ALIGN_RCP_KI            0.000f
#define ALIGN_RCP_KD            0.013f
#define ALIGN_RCP_RPM_MAX       23.0f
#define ALIGN_RCP_WHEEL_MAX     40
#define ALIGN_RCP_YAW_KP        0.200f  /* $RP 平行度闭环（ALIGN_YAW_RCP=1 时才参与） */
#define ALIGN_RCP_YAW_KI        0.000f
#define ALIGN_RCP_YAW_KD        0.015f
#define ALIGN_RCP_YAW_RPM_MAX   12.0f

/* --- ② $SHOT（打靶 STORE / ROUGH PLACE；死区 5px，不限时） --- */
#define ALIGN_SHOT_KP           0.180f  /* 【2026-10-15】0.12 -> 0.18：死区放到 5px 后可以跑快点 */
#define ALIGN_SHOT_KI           0.090f  /* 末端静摩擦残差靠积分顶上去（配合 I_SEP_MM=10） */
#define ALIGN_SHOT_KD           0.014f  /* 提高 KP 后用 D 提前收油、压过冲 */
#define ALIGN_SHOT_RPM_MAX      30.0f   /* 25 -> 30：远距离接近更快（近端仍由 KP 自然降速） */
#define ALIGN_SHOT_WHEEL_MAX    40
#define ALIGN_SHOT_YAW_KP       0.175f
#define ALIGN_SHOT_YAW_KI       0.000f
#define ALIGN_SHOT_YAW_KD       0.005f
#define ALIGN_SHOT_YAW_RPM_MAX  12.0f

/* --- ③ $SHOT2（ROUGH STACK；死区 7px，不限时） --- */
#define ALIGN_SHOT2_KP          0.180f
#define ALIGN_SHOT2_KI          0.090f
#define ALIGN_SHOT2_KD          0.014f
#define ALIGN_SHOT2_RPM_MAX     30.0f
#define ALIGN_SHOT2_WHEEL_MAX   40
#define ALIGN_SHOT2_YAW_KP      0.175f
#define ALIGN_SHOT2_YAW_KI      0.000f
#define ALIGN_SHOT2_YAW_KD      0.005f
#define ALIGN_SHOT2_YAW_RPM_MAX 12.0f

/* 速度段"数据最大年龄"，单位 ms，默认 250。
 * 速度段是**连续运动**，按旧数据换算的速度会一路开过去（例：40RPM ≈ 0.21m/s，
 * 数据老 250ms 就已经多走约 52mm）。所以这里比 RPI_DATA_EXPIRE_MS(500ms) 更严：
 * 超过本值没收到新偏移就**先软停再等**，等到了再继续。
 * 【自动放宽】代码会实测视觉帧间隔，实际上限 = max(本值, 3×实测帧间隔, 上限 1000ms)，
 * 所以视觉帧率比本值慢时不会被误判成"过期"（那会变成走一段停一段）。 */
#define ALIGN_VEL_DATA_MS       250U

/* ===== 加减速档（与跑图/赛题完全独立）=====
   ALIGN_ACC：加减速档 0~255，**数值越大加减速越缓**（0 = 无斜坡、直接起停）。
            速度模式的两段速共用这个档（原来的"位置模式精调段"已删除）。
            【2026-10-09 由 80 改成 0】现场要求"对位的加速度给 0"：
            无斜坡直接起停，响应最直接，末端不会因为"还在减速"多滑一段。
            注：软件限斜率 ALIGN_VEL_SLEW_RPM 仍在（起步/收尾本身不会一下到速、一下停），
            所以 acc=0 不会变成"硬起硬停"。
            若现场觉得起步"啪"一下太冲，再往回调到 30~80。 */
#define ALIGN_ACC               1U

/* 停稳后的静置时间，单位 ms，默认 100（**上限**）。
 * 实际取 min(本值, max(40, 实测视觉帧间隔))：这里只是等"视觉拍到车已停住的那一帧"，
 * 视觉快就不用等满。等太久会让"软停→复测"之间出现可见的停顿。 */
#define ALIGN_SETTLE_MS         100U

/* ===== 【2026-10-10 已删除】"末端放宽出口"（原 `ALIGN_NEAR_PX` / `ALIGN_HOLD_MS_*` / `ALIGN_HOLD_TOL_PX`）=====
   原来有一条"偏差已经不再变小就当到位"的放宽出口：偏差 ≤ 16px 且连续一段时间没再变小 -> 直接判成功。
   现场表现为"**快靠近的时候停下、偏差还有十几像素，要再按一次才到位**"。
   按用户要求改为**一步到位**：这条出口已**整体删除**，对位现在**只有一条成功判据** ——
   两轴都真正进**死区**（`RPI_PX_DEADBAND` / `RPI_PX_DEADBAND_STRICT`，见 bsp_rpi.h）才判 DONE。
   唯一的失败判据仍是"连续收不到新偏移帧"（见 `ALIGN_NO_DATA_FAIL_MS`）。 */


/* 等"新一帧"的日志上限，单位 ms，默认 800：超过就限流打一条提示，
 * **但不会中止**（继续等）。 */
#define ALIGN_FRAME_TIMEOUT_MS  800U

/* 连续多久"一个新偏移帧都没收到"才判失败，单位 ms，默认 15000；**0 = 永不失败**。
 * 设 0 就是死等：视觉断了也会一直等下去（只能再按一次 KEY5 中止）。
 * 保留这个兜底是因为"车跑出视野/视觉端停了"继续等没有意义，需要明确报出来。 */
#define ALIGN_NO_DATA_FAIL_MS   15000U

/* ===== ★ 车身矫正（平行度）：用 `$SHOT` 的 **q1** 把车身转正 =====
 * 【★2026-10-10 晚 按视觉组《打靶SHOT对接.md》修正】
 *   帧：`$SHOT,q1,q2,q3,dx,dy`
 *     · `q1` = **打靶平行度**（三环连线的倾斜角），**单位 0.1°**，int16，范围约 ±900（=±90°）；
 *       **正 = 顺时针偏，负 = 逆时针偏**；基准 = 过画面中心的水平横线（0° = 完全水平）。
 *       例：q1 = -8 ⇒ -0.8°（逆时针偏 0.8°）。
 *     · `q2`、`q3` = **保留字段，暂恒 0**（后续才可能填"上下长边平行度"）。
 *       ⇒ ★**必须只用 q1**（`ALIGN_YAW_Q_MODE = 1`）：上一版按"三个取平均"实现，
 *         会把真实误差**除以 3**（(−8+0+0)/3 = −2.67），转向量只有 1/3、还容易被判成"已平行"。
 *     · `dx, dy` = **中间圆环（2 号靶）中心偏移**（相对打靶零点，**像素**）—— XY 平移段用。
 * 【怎么做】本模块**先原地把 q1 转进 `ALIGN_YAW_DEADBAND`，再做 dx/dy 平移**
 *   （用户 2026-10-10 指定：**先转正、再平移**）。控制律与 XY 段同源：
 *   |q1| 越近越慢（线性插值）、末端换短节拍、卡住抬到上限速度。
 * 【作用范围】`ALIGN_YAW_ENABLE = 1` 时**只对 `$SHOT`（打靶/暂存 STORE·ROUGH PLACE）生效**，
 *   转盘 `$RCP` 完全不受影响（该帧没有 q 字段）。
 * 【现场标定】
 *   · **方向反了**（q1 越转越大）-> 取反 `ALIGN_YAW_SIGN`（+1 / -1）；
 *     ★已于 2026-10-10 晚现场反馈"平行度反了" ⇒ 现为 **+1**。
 *   · 一直磨 / 停不准 -> 调 `ALIGN_YAW_DEADBAND`（现 5 = **0.5°**）；
 *   · 转太快/太慢      -> 调 `ALIGN_YAW_RPM_MIN/MAX` 与下面两个"速度插值区间"。
 * ⚠ 命名说明：下面两个 `_PX_` 宏是**早期按"q 是像素"猜的命名**，现在确认单位是 0.1°，
 *   值已按 0.1° 重新填（为免牵动多处引用，**保留宏名**，看注释为准）。
 * 日志：进本段打 `[ALN] yaw first: q=.. mode=.. sign=.. db=.. rpm=..`；
 *      转正完成打 `[ALN] yaw ok q=.. -> XY`；页面第 4 行前缀为 `Y`。 */
#define ALIGN_YAW_ENABLE        1U     /* 1=对 $SHOT 做车身矫正；0=关闭（退回"只平移"） */

/* ★【2026-10-15 用户要求】转盘区也做**平行度对齐**（新增协议 `$RP`）
 *   `$RP,q1`：q1 = 圆盘平行度，单位 0.1°。`$RCP` 本体没有 q 字段，所以转盘对位时
 *   要**额外**请求 `$RP`（视觉端同时切 TURN 模式，与 $RCP 同一视图、无切换延迟）。
 *   打开后：转盘对位的"到位"判据也要求 |q1| ≤ ALIGN_YAW_DEADBAND，并且
 *   旋转轴与 XY **同时闭环**（ALIGN_YAW_SIMUL）。
 *   ⚠ `$RP` 的 q1=0 **可能是"真对齐"也可能是"没检到红线"**（协议不给有效标志）：
 *     0 时本模块不动作（不转），与"已对齐"等效；安全。
 *   0 = 关闭（退回只做 $RCP 平移）。 */
#define ALIGN_YAW_RCP           1U

#define ALIGN_YAW_Q_MODE        1U     /* ★只用 q1（视觉组：q2/q3 是保留字段、恒 0） */
#define ALIGN_YAW_SIGN          (+1)   /* q1 符号 -> 转向；现场反馈"反了"后由 -1 改成 +1 */

/* ★★【2026-10-15 用户要求】**XY 与平行度同时调**（原来是"先转正、进 XY 就不管了"）
 *   旧行为：`AL_YAW` 段把 |q| 转进死区 -> 进 `AL_RUN` 段后 uW 恒 0。
 *           实测平移过程中打滑会让车身持续偏航（80s 漂了 25°，q 从 -1 变到 -225），
 *           XY 是"在旋转的坐标系里追"，永远收敛不干净。
 *   新行为（本宏 =1）：
 *     ① **跳过独立的 AL_YAW 段**，直接进 XY；
 *     ② XY 段每帧**同时**算旋转轴 PID（同一个 `alPidW`），三轴输出一起逆解到四轮；
 *     ③ "到位"判据**同时要求 |q| ≤ ALIGN_YAW_DEADBAND**，即 XY 与平行度都进死区才算完。
 *   想退回旧行为：本宏改 0。 */
#define ALIGN_YAW_SIMUL         1U

#define ALIGN_YAW_DEADBAND      3      /* ★【2026-10-15 用户要求】|q1| ≤ 本值算"已平行"
                                          （**单位 0.1°**，3 = **±0.3°**；0.2 现场太严） */
#define ALIGN_YAW_PX_MIN        3      /* q1 下限（0.1°）：≤ 本值 -> 用速度下限（3 = 0.3°） */
#define ALIGN_YAW_PX_MAX        100    /* q1 上限（0.1°）：≥ 本值 -> 用速度上限（100 = 10°） */
#define ALIGN_YAW_RPM_MIN       1      /* 旋转速度下限（车轮 RPM） */
#define ALIGN_YAW_RPM_MAX       10     /* 旋转速度上限（车轮 RPM） */

/* ===== ★【2026-10-10 晚 用户要求】STORE PLACE 对位"**跑两轮**" =====
 *   ★【2026-10-15 已按用户要求**关闭**（`ALIGN_PASS_COUNT=1`）：改为**一步到位**，
 *     直接朝严格死区（1px）冲，不再先粗后精。下面是原两轮制的说明，保留以便随时改回 2。
 *   第 1 轮：粗靠近（死区放大 `ALIGN_PASS1_DB_MUL` 倍）——快，只求靠近；
 *   第 2 轮：精定位（死区回到严格值 `RPI_PX_DEADBAND`）——保证精准。
 *   流程每轮都是"先车身矫正(q1) -> 再 dx/dy 平移"，与原来一致。
 * **只对 `$SHOT`（STORE/ROUGH PLACE）生效**；`$RCP`（TRAY PICK）仍单轮（它有 7s 时间预算）。 */
#define ALIGN_PASS_COUNT        1      /* ★【2026-10-15 用户要求】**一步到位**：1 = 单轮直冲严格死区（默认）；
                                          2 = 老的两轮制（第 1 轮粗靠近、第 2 轮精定位） */
#define ALIGN_PASS1_DB_MUL      4      /* 第 1 轮死区放大倍数（越大会越快、也越粗） */

/* ★车身矫正的**安全阀**：本段累计超过这么多 ms 还没转进死区 -> 停轮 + 判失败
 * （日志 `yaw TIMEOUT ... -> fail`；页面 `AFAIL`）。
 * 【为什么必须有】**转正是原地旋转**，一旦 `ALIGN_YAW_SIGN` 标反（q 越转越大）或视觉给的 q
 *   根本收敛不了，"不限时"就意味着**车一直原地打转**。加这个上限最坏只是失败一次。
 * 0 = 不限时（**不推荐**，除非已经现场确认方向正确）。 */
#define ALIGN_YAW_MAX_MS        15000U

/* ===================== 对位时间预算（"几秒内必须完成对位"） ===================== */
/* 背景：转盘停留时间是 6~10s 可调，对位必须在它转走之前结束。
 * 用法：开对位**之前**调 Align_SetBudget(ms)；0 = 不限时（默认，一直磨到死区）。
 * 设了预算后本模块做三件事：
 *   ① **提速**：粗调段速度上限 ×ALIGN_BUDGET_SPEED_PCT%（先把误差压下去）；
 *   ② **逐级放宽死区**：用掉 60% 时间 -> 死区 +ALIGN_BUDGET_RELAX_PX；
 *                        用掉 80% 时间 -> 再 +ALIGN_BUDGET_RELAX_PX（最多放宽两次）；
 *                        **本值现为 0 = 不放宽**（见下面的宏注释）；
 *   ③ **硬收工**：到点仍未进最终死区 -> 判失败（日志 `BUDGET ... used up`），
 *      进了就判成功（日志 `done by BUDGET`）。**不会假装成功**。
 * 想"设几秒就几秒结束"就把预算设成比转盘停留少 1~2s（留出收尾/抓取的余量）。 */

/* 有预算时的提速比例，单位 %，默认 100 = **不提速**。
 * 【2026-10-06 由 150 降到 100】提速会加大收尾惯性 -> 过冲；时间应该从"少走几个来回"
 * 里省（预算的另一半机制：逐级放宽 + 到点硬收工），不要从速度里省。
 * 确实想抢时间再往上调（120~150），但先确认没有过冲。 */
#define ALIGN_BUDGET_SPEED_PCT  100

/* 时间过半后每级放宽的死区，单位像素，**现为 0 = 不放宽**。
 * 【2026-10-06 由 4 改成 0】用户要求"最终判断到位的像素偏差就是死区那个值"，
 * 所以**不许为了赶时间把死区放宽**：有预算时死区恒等于 RPI_PX_DEADBAND（见 bsp_rpi.h）。
 * 放宽总量 = 2 × 本值（0 时两级判断都退化成同一个死区，即不放宽）。
 * 于是时间预算只剩"提速 + 到点硬收工"两件事：
 *   到了 TT_ALIGN_BUDGET_MS 还没进死区 -> **判失败**（不会放宽后假装成功）。
 * 若某个现场确实允许更松，再把它调回 2~4；放宽直接决定抓取精度，别设太大。 */
#define ALIGN_BUDGET_RELAX_PX   0

/* ===== 对位成功提示音（Al_OnAligned 里登记，主循环 BSP_Beep_Update 发声） =====
 * 只在对位**成功**时响；失败/中止不响。
 *   TIMES = 0 关闭；ON_MS 每声时长；OFF_MS 两声之间的间隔。
 * 注意：BSP_Beep 是非阻塞"登记"，不会拖慢对位收尾。 */
#define ALIGN_DONE_BEEP_TIMES   2U
#define ALIGN_DONE_BEEP_ON_MS   80U
#define ALIGN_DONE_BEEP_OFF_MS  80U

/**
 * @brief  设置对位时间预算
 * @param  budgetMs: 预算 ms；0 = 不限时
 */
void Align_SetBudget(uint32_t budgetMs);

/**
 * @brief  登记"本次对位成功后要把陀螺仪静态置到多少度"
 * @param  refDeg: 目标角（TRAY PICK=0 / ROUGH PLACE=90 / ROUGH STACK=90）
 * @retval 无
 * @note   任务点在进入任务点时调用；对位成功那一刻执行 IMU_StaticZeroTo()，
 *         之后绝对坐标都基于新角度。未登记则对位成功不置角。
 */
void Align_SetDoneYawRef(float refDeg);

/**
 * @brief  取消"对位成功后静态置角"（任务点不要求时调用）
 */
void Align_ClearDoneYawRef(void);

/* ===================== 对外接口 ===================== */

/* ===== ★【2026-10-15】运行期 PID 参数表（供 UART5 在线调参 / PC 自动调参使用） =====
   三套参数按数据源分开（0=$RCP 1=$SHOT 2=$SHOT2），`Align_SetPidCfg()` 只改 RAM，
   掉电不保存；调好后要把最优值抄回上面的 `ALIGN_<X>_*` 宏。
   单位：kp/ki/kd 见上面注释；rpmMax = 平移单轴输出上限（RPM）；wheelMax = 单轮硬上限（RPM）；
        ykp/yki/ykd = 旋转轴增益（误差单位 0.1°）；yrpmMax = 旋转输出上限（RPM）。 */
typedef struct {
    float   kp;
    float   ki;
    float   kd;
    float   rpmMax;
    int32_t wheelMax;
    float   ykp;
    float   yki;
    float   ykd;
    float   yrpmMax;
} Align_PidCfg_t;

/**
 * @brief  运行期设置某一套 PID 参数（自动夹到安全范围）
 * @param  src: 0=$RCP 1=$SHOT 2=$SHOT2
 * @param  cfg: 新参数（9 项）
 * @retval 1=成功；0=src 非法
 * @note   若 `src` 正是本次对位在用的那套，**立刻生效**（下一拍 PID 就用新增益），
 *         且不清积分/微分状态，方便"跑一跑再微调"。
 */
uint8_t Align_SetPidCfg(uint8_t src, const Align_PidCfg_t *cfg);

/**
 * @brief  读取某一套 PID 参数
 * @param  src: 0=$RCP 1=$SHOT 2=$SHOT2
 * @retval 参数表指针；src 非法返回 NULL
 */
const Align_PidCfg_t *Align_GetPidCfg(uint8_t src);

/* ===== ★【2026-10-15】调参用调试快照（供对位测试页 bsp_align_test 读取） =====
   目的：把"每一帧的误差 + PID 输出"原样吐出来，现场整定 PID 时用来判断
         "是走不到位、还是冲过头、还是稳态残差"。
   只读、不影响控制；对位没在跑时 active=0。
   单位：px=像素、mm=车体毫米、u*=RPM×100（例：uF=1234 表示 12.34 RPM）。 */
typedef struct {
    uint8_t  active;      /* 1=对位进行中 */
    uint8_t  state;       /* 0=IDLE 1=YAW 2=RUN 3=READ 4=SETTLE 5=DONE 6=FAILED */
    uint8_t  src;         /* 0=$RCP 1=$SHOT 2=$SHOT2 */
    uint8_t  pass;        /* 轮次（两轮制时 1/2；现默认单轮=1） */
    uint16_t frames;      /* 已喂给 PID 的视觉帧数 */
    uint16_t iters;       /* 已进入速度段的次数 */
    int16_t  dxPx;        /* 图像偏移 X（px） */
    int16_t  dyPx;        /* 图像偏移 Y（px） */
    int16_t  qRaw;        /* 平行度原值（0.1°；$RCP 恒 0） */
    int16_t  mmFwd;       /* 前进轴误差（车体 mm，带方向） */
    int16_t  mmRight;     /* 横移轴误差（车体 mm） */
    int16_t  uF;          /* 前进轴 PID 输出 ×100（RPM） */
    int16_t  uR;          /* 横移轴 PID 输出 ×100（RPM） */
    int16_t  uW;          /* 旋转轴 PID 输出 ×100（RPM） */
    int16_t  wheelMax;    /* 本拍最大单轮 RPM */
    uint16_t frameMs;     /* 本拍 dt（实测视觉帧间隔 ms） */
    uint32_t elapsedMs;   /* 本次对位已用时 ms */
    uint32_t deadbandPx;  /* 当前生效死区（px） */
} Align_Dbg_t;

/**
 * @brief  取对位调试快照
 * @param  out: 输出（调用方提供）
 * @retval 无
 */
void Align_GetDebug(Align_Dbg_t *out);

/**
 * @brief  开始闭环对位（**转盘区**：数据源 `$RCP` 转盘中心偏移）
 * @param  无
 * @retval 1=已开始；0=没开始（视觉数据无效/过期，或已在运行）
 * @note   非阻塞；开始后由 Align_Update() 推进。
 *         只在"车已停稳"时调用（本模块不做"先过去"，只管**原地微调**）。
 *         调本函数前应先发 `RPi_ReqRcp()` 并等 `RPi_GetRcp()->valid`（协议 §3.1）。
 */
uint8_t Align_Start(void);

/**
 * @brief  开始闭环对位（**打靶区**：数据源 `$SHOT` 中间圆环偏移）
 * @param  无
 * @retval 1=已开始；0=没开始（视觉数据无效/过期，或已在运行）
 * @note   【2026-10-09 新增，协议 v2.0】暂存区 / 粗加工区的对位用 `$SHOT`（协议 §3.2），
 *         与转盘区 `$RCP` 是**两条独立数据帧**，所以必须用这个入口而不是 `Align_Start()`。
 *         其余行为（X/Y 分轴 + 像素→速度连续映射、死区判据、超时兜底）与 `Align_Start()` 完全一致。
 *         调本函数前应先发 `RPi_ReqShot()` 并等 `RPi_GetShot()->valid`。
 */
uint8_t Align_StartShot(void);

/**
 * @brief  开始闭环对位（★ROUGH STACK：数据源 `$SHOT2`）
 * @param  无
 * @retval 1=已开始；0=没开始（视觉数据无效/过期，或已在运行）
 * @note   ★【视觉组《SHOT2通讯对接.md》】`$SHOT2` 与 `$SHOT` 是**两个不同的测量**：
 *           `$SHOT`  瞄"中间**圆环**圆心"（色环靶位 -> STORE / ROUGH PLACE）；
 *           `$SHOT2` 瞄"三个横排物料里**中间那个的颜色质心**"（-> **ROUGH STACK**）。
 *         数据**各自独立**（`RPi_GetShot2()`）；本入口除数据源外的行为
 *         （严格档死区 1px、不限时、两轮制、车身矫正 q1、冻结平移坐标）与 `Align_StartShot()` **完全一致**。
 *         调本函数前应先发 `RPi_ReqShot2()` 并等 `RPi_GetShot2()->valid`。
 */
uint8_t Align_StartShot2(void);

/**
 * @brief  闭环对位主循环节拍（非阻塞）
 * @param  无
 * @retval 无
 * @note   在 main 主循环里调度。**漏调度会卡住不动**。
 */
void Align_Update(void);

/**
 * @brief  是否正在对位
 * @retval 1=运行中
 */
uint8_t Align_IsBusy(void);

/**
 * @brief  是否已收敛到死区内（对位成功）
 * @retval 1=成功
 */
uint8_t Align_IsDone(void);

/**
 * @brief  是否失败（迭代用尽 / 总超时 / 视觉数据一直无效）
 * @retval 1=失败
 */
uint8_t Align_IsFailed(void);

/**
 * @brief  中止对位（四轮急停）
 * @param  无
 * @retval 无
 */
void Align_Abort(void);

/**
 * @brief  取状态摘要（页面/日志用）
 * @param  无
 * @retval 形如 "ALIGN 2/8  x=-12 y=30" / "ALGN OK x=-2 y=1" / "ALGN FAIL no data" / "ALGN IDLE"
 */
const char *Align_GetStateText(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ALIGN_H */
