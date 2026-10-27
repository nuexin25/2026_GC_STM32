/**
 ******************************************************************************
 * @file    bsp_tt.h
 * @brief   作业点序列（P10）：① 转盘取料（连抓 3 件放车载 1/2/3）② 打靶放置（STORE/ROUGH PLACE）
 * @author  nx_car
 * @date    2026-10-06
 ******************************************************************************
 * @note   【本文件放两条**作业点序列**】—— 调试页与赛题流程**共用同一份实现**：
 *           ① **转盘取料** `TT_*`（协议 v2.0 §6.1）：TRAY PICK；
 *           ② **打靶放置** `SP_*`：STORE PLACE / ROUGH PLACE。
 *
 *         【① 转盘取料 `TT_*` 干什么】按《树莓派通讯协议》v2.0 §6.1 自动完成这一串（全程非阻塞，主循环调度）：
 *           ① 手臂到**观察区对位**（`ArmAct_TrayReady(slot)`：M5 ‖ S1 观察位 + 舵机3 转车载号位）
 *              —— **必须最先做**：摄像头要先看到转盘/物料，$RCP/$RZ 才有意义；
 *           ② 发 `$RCP` → 等 `RPi_GetRcp()->valid` → 车身对位（`Align_Start()`，数据源 $RCP）；
 *              （一直没数据就向车身侧平移一小段重试，见 `TRAY_RCP_SHIFT_*`）
 *           ③ **周期**发 `$RZ`（单帧快照，需重发）→ 先锁一份**基准映射**（位号↔颜色，
 *              要求同一份映射连续两帧出现）；
 *           ④ 继续周期发 `$RZ`，等映射相对基准发生**完美循环移位**（整盘转了一步、连续两帧一致）
 *              → 把这次变化**首次出现的时刻记为 t0**，立刻去 t0 那帧里第 1 件目标色的位号抓；
 *           ⑤ 第 1 件：抓 → 放车载 → 记账（颜色 = 任务码要求色）；
 *           ⑥ 第 2/3 件：**不再回观察位、不再对位**，位号由"基准位号 + (现在 - t0) 计时"推算：
 *              `steps = 经过时间 / TRAY_ROT_PERIOD_MS`；
 *              ★周期内已过时间 < `TRAY_ROT_LEAD_MS`（默认 2s）→ 就地抓 steps 位；
 *                ≥ 该值 → 提前去 steps+1 位，**停在那儿等料转到位**（= t0+(steps+1)*T）再抓；
 *                ★等待位还要 `$CCG,<位号>` 颜色反馈确认"就是本件目标色"才抓；
 *           ⑦ 满 3 件 -> 完成。车载位顺序 `TT_SLOT_FIRST` 起（1->2->3）。
 *
 *         【★唯一实现 2026-10-10】TRAY PICK 只有这一份实现：任务调试页（`TDBG_TRAY_PICK`，
 *         `bsp_task_dbg.c`）与赛题流程（`PICK_TRAY_ALL()`，`bsp_task.c`）**都调本模块**。
 *
 *         【为什么是请求/轮询式】v1.0 的"视觉持续推送当前颜色 + 转盘偏移"语义已删除，
 *         v2.0 改为"STM32 主动发请求、树莓派应答"：$RCP/$RZ/$SHOT 全部按需请求；
 *         ★2026-10-24 起 `$CCG` **只在"等待位"用**（确认该位号已是本件目标色），
 *         **不再**"等目标色转到准备位"式轮询。
 *
 *         【机械臂动作怎么来】不在这里定义，也不走"动作编号"：直接调 bsp_arm_action 的
 *         `ArmAct_TrayReady(slot)`（观察区对位 + 舵盘选号）/
 *         `ArmAct_GotoTtPos(pos)`（转盘取料姿态）/ `ArmAct_TrayPickPlace(slot)`（抓紧 -> 放车载）；
 *         动作组与标定值集中在 bsp_arm_action.h 的 `ARM_CAL_LIST`。
 *
 *         硬约束：禁止 HAL_Delay；日志禁 %f；全部非阻塞；每个等待都有超时宏兜底。
 ******************************************************************************
 */
#ifndef __BSP_TT_H
#define __BSP_TT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* ===================== 可调参数 ===================== */

/* 抓几件（放置顺序：从车载 TT_SLOT_FIRST 号位起，逐件 +1，即 **1 -> 2 -> 3**）
 * ★【2026-10-23 用户要求】第一次按顺序上盘 1/2/3 号位，之后**一直按这次绑定**使用
 *   （颜色↔号位在 TT_PLACE 时记账，STORE PLACE / STORE BACK 都查这张表）。 */
#define TT_ITEM_COUNT           3U

/* 第 1 件放车载几号位（后续每件 +1） */
#define TT_SLOT_FIRST           1U

/* 车载号位个数（1~3） */
#define TT_SLOT_MAX             3U

/* 单次对位的外层兜底超时，单位 ms，默认 30000。
 * 对位本身已不限时（见 Align_SetBudget），这里只留一个"明显不正常"的兜底：
 * 真跑满 30s 还收不了口，说明不是时间问题（视觉/机构异常），该报错中止。 */
#define TT_ALIGN_TIMEOUT_MS     30000U

/* 对位时间预算，单位 ms，**0 = 不限时**（一直磨到真正进像素死区为止）。
 * 与它配套：转盘把料转走时不会白等，Al 模块按"偏差突然变大"自行弃料重来。
 * 若现场发现转盘停留时间不够、料总在对到一半被转走，宁可把转盘停留时间调长，
 * 也不要把本值改回有限值（改回就又会为了赶时间牺牲精度）。 */
#define TT_ALIGN_BUDGET_MS      0U

/* 每一段机械臂动作的超时，单位 ms，默认 30000 */
#define TT_ARM_TIMEOUT_MS       30000U

/* ===== 打靶放置序列（STORE PLACE / ROUGH PLACE）的宏 ===== */
/* 放几件（车载位 **1 -> 2 -> 3** 依次取件，**按任务码第 2 组的顺序**放到对应靶位；
 * 顺序硬编码在 bsp_tt.c 的 SP_Start()/SP_PICK（spSlot 从 1 递增到 3，**按任务码第 2 组的顺序**；
 * ★2026-10-24 起"车载取件+靶位放置"合并成一条连续动作 `ArmAct_RunCar2Tgt`） */
#define SP_ITEM_COUNT           3U

/* "把对位启起来"这一小段的外层兜底，单位 ms，默认 10000。
 * 【只兜这段】等 `$SHOT` 有效 + `Align_StartShot()` 成功；对位本身（SP_WAIT_ALIGN）
 * **不限时**（严格档 1px + 关闭放宽出口 —— "不到 (0,0) 不放过"）。 */
#define SP_ALIGN_START_TIMEOUT_MS   10000U

/* ===== ★【2026-10-10 晚 用户要求】STORE PLACE 的 `$SHOT` 时序（见 bsp_tt.c 的 SP_Update）=====
 * 用户口径："**先开启 $SHOT，再到达观察位** —— 因为现在存在到达观察位后**无响应**的情况，
 *           可能是**树莓派开启程序超时**造成。"
 * 已改成：`SP_REQ_SHOT`(先发 $SHOT) -> `SP_OBSERVE`(臂去观察位) -> `SP_WAIT_SHOT`(等有效)
 *   —— 手臂走这段的时间就是给树莓派启动/出首帧的**提前量**。 */

/* 进 `SP_WAIT_SHOT` 后等 `$SHOT,dx,dy` 有效的**总时长**，单位 ms，默认 10000。
 * 【为什么另开一个，不用通用 `SHOT_TIMEOUT_MS`(3000)】用户现场判断"树莓派开程序会超时"，
 *   3000ms 往往不够（树莓派冷启动/加载视觉脚本）。这里给足 10s，只影响 STORE/ROUGH PLACE。 */
#define SP_SHOT_TIMEOUT_MS      10000U

/* 等待期间**周期重发** `$SHOT` 的间隔，单位 ms，默认 500。
 * 【为什么必须重发】协议 v2.0 是**一问一答、无 ACK**：若首发那一刻树莓派**还没在监听**，
 *   那一帧就**永远丢了**（不是"没算完"，是"根本没收到"），不重发只能干等到超时失败。 */
#define SP_SHOT_REQ_MS          500U

/* ===== ★【2026-10-11 用户要求】ROUGH PLACE 的"收尾姿态"=====
 * 用户："粗加工区三个放置完后，手臂回到车载 3 号位**不抓**的状态，
 *       顺序是**先电机6 抬高，舵机1 收回，电机5 转向，舵机2 张开**"。
 * 只有 `SP_SetRoughMode(1)` 的序列才做这一段（**STORE PLACE 不做** —— 它要"只松抓"收尾、
 * 好和 STORE BACK 衔接）。实现在 `bsp_arm_action` 的 `ArmAct_GotoCarIdle()`。 */
#define SP_ARM_IDLE_SLOT        3U       /* 收尾回到车载几号位 */
#define SP_ARM_IDLE_TIMEOUT_MS  30000U   /* 该收尾动作的超时（与 TT_ARM_TIMEOUT_MS 同口径） */

/* ===== 打靶取回序列（**STORE BACK**）的宏 ===== */
/* 取回几件（**按放置的逆序**：车载位 1 -> 2 -> 3 —— 因为 STORE PLACE 现在是 3->2->1 取件，
 * 每件从它原先放的那个靶位抓回） */
#define SB_ITEM_COUNT           3U

/* **第一件**的静置时间，单位 ms，**默认 0 = 不等待、直接抓**。
 * 【2026-10-15 用户要求】"**不用等 1s 再抓取**，只要上个任务点（STORE PLACE）放置过后，
 *   **直接抓取，不用等**" —— 原来是 1000ms，现在直接抓（省 1s）。
 * 想恢复"等料稳一下"再把它调回 1000（可调范围 0~2000）。 */
#define SB_SETTLE_MS            0U

/* 每一段机械臂动作的超时，单位 ms，默认 30000（与 TT_ARM_TIMEOUT_MS 同值） */
#define SB_ARM_TIMEOUT_MS       30000U

/* ===================== 对外接口 ===================== */

/**
 * @brief  初始化（上电调用一次；只复位状态，不动电机）
 */
void TT_Init(void);

/**
 * @brief  一键启动（取任务码第 1 组颜色）
 * @retval 1=已启动；0=拒绝（已在跑 / 对位或机械臂正忙 / 任务码非法 / 颜色未知）
 * @note   非阻塞：启动后由 TT_Update() 在主循环里推进。
 *         颜色来源 = `Task_ColorOfBatch(1, i)`（任务码第 1 组）；需要手工指定颜色时用
 *         `TT_StartWithColors()`。
 */
uint8_t TT_Start(void);

/**
 * @brief  按指定颜色启动（本轮 3 件的目标颜色顺序）
 * @param  colors3: 长度 3 的颜色数组（1红 2黄 3蓝 4绿）
 * @retval 1=已启动；0=拒绝（colors3 为 NULL / 含 0 / 已在跑 / 对位或机械臂正忙）
 * @note   非阻塞，与 TT_Start 共用同一套序列。
 */
uint8_t TT_StartWithColors(const uint8_t *colors3);

/**
 * @brief  中止（会急停对位并中止机械臂动作）
 */
void TT_Abort(void);

/**
 * @brief  主循环节拍（**漏调度会卡住不动**）
 */
void TT_Update(void);

/**
 * @brief  是否正在跑这个序列
 */
uint8_t TT_IsBusy(void);

/**
 * @brief  是否已跑完（3 件全部上车）
 */
uint8_t TT_IsDone(void);

/**
 * @brief  是否失败中止（超时/动作被中止/标定缺失/目标色缺失）
 */
uint8_t TT_IsFailed(void);

/**
 * @brief  ★【2026-10-15 用户要求】是否已进入"**只剩多余动作**"的收尾段
 * @retval 1=关键作业已完成（**最后一件已抓到手里、舵机1 已回 0**），
 *          剩下只有"放回车载位"这类回位动作 —— **此时可以让底盘先动、手臂并行**；0=还没到
 * @note   与 `TT_IsDone()` 的区别：`IsDone` 要等**整段跑完**（含最后一件放上车），
 *         `TailOnly` 只要"最后一件到手且 S1 已回 0"就为 1（此刻车一动不会带歪电机5）。
 *         `TT_Update()` 仍会照常把收尾动作跑完，**不会被本函数打断**。
 *         用途：`bsp_task.c` 的任务点在等本序列时用它提前放行 => 底盘与手臂并行。
 */
uint8_t TT_TailOnly(void);

/**
 * @brief  取状态摘要（页面第 4 行/日志用）
 * @retval 形如 "TT req RCP" / "ALGN OK x=-2 y=1" / "TT ccg Q2 C3" / "3:C2 2:C4 1:C1"
 */
const char *TT_GetStateText(void);

/**
 * @brief  取车载某号位上物料的颜色（**颜色与位号绑定**，供后面放到加工区用）
 * @param  slot: 车载号位 1~3
 * @retval 颜色值 1~4；0=该位还没放料（或参数越界）
 */
uint8_t TT_GetSlotColor(uint8_t slot);

/**
 * @brief  取绑定关系文本（日志/页面用）
 * @retval 形如 "3:C2 2:C4 1:C1"（没放的显示 C0）
 */
const char *TT_GetMapText(void);

/* ===================== ② 打靶放置序列（STORE PLACE / ROUGH PLACE） ===================== */
/* 同一份 job 序列，**两个入口共用**：
 *   任务调试页 `TDBG_STORE_PLACE` / `TDBG_ROUGH_PLACE`（bsp_task_dbg.c）
 *   赛题流程 `T_B1_PLACE`（`PLACE_STORE_ALL()`，bsp_task.c）
 * 流程：臂进观察位 -> `$SHOT` -> 等有效 -> 精确对位（严格档、**不限时**） ->
 *       **车载位 1→2→3**：读绑定色 -> 任务码第1组查靶位 -> `ARM_OP_CAR_PICK` -> `ARM_OP_TGT_PLACE`。
 * ⚠ 前提：先跑过 TRAY PICK（颜色↔车载位号绑定），否则 `no binding` 失败。 */

/**
 * @brief  初始化（上电调用一次；只复位状态）
 */
void SP_Init(void);

/**
 * @brief  启动（从车载 1 号位开始，放 3 件）
 * @retval 1=已启动；0=拒绝（已在跑 / 对位或机械臂正忙）
 * @note   非阻塞：启动后由 `SP_Update()` 在主循环里推进（main.c 调度）。
 */
uint8_t SP_Start(void);

/**
 * @brief  ★设定"本段是 ROUGH PLACE"：放置 3 件后，手臂回车载 `SP_ARM_IDLE_SLOT` 号位、**不抓**
 * @param  on: 1=ROUGH PLACE（要做收尾姿态）；0=STORE PLACE（不做）
 * @note   ★【2026-10-11 用户要求】在 `SP_Start()` **之前**调用。
 *         赛题由 `case T_B1_RGH_PLACE`(1) / `case T_B1_PLACE`(0) 设置；
 *         调试页由 `Tdbg_Start()` 按选中的任务点设置。
 */
void SP_SetRoughMode(uint8_t on);

/**
 * @brief  ★设定本段按**任务码第几批**取颜色/靶位号
 * @param  batch: 1=第 1/2 组（第一批）；2=第 3/4 组（第二批）
 * @note   ★【2026-10-23 用户要求】第二批也要先在粗加工区按**第 4 组**放置一遍，
 *         所以 SP_ 不能再写死第 1 批。在 `SP_Start()` **之前**调用（缺省 1）。
 */
void SP_SetBatch(uint8_t batch);

/**
 * @brief  中止（急停对位 + 中止机械臂动作）
 */
void SP_Abort(void);

/**
 * @brief  主循环节拍（**漏调度会卡住不动**）
 */
void SP_Update(void);

/** @brief  是否正在跑 */
uint8_t SP_IsBusy(void);
/** @brief  是否已跑完（3 件全放完） */
uint8_t SP_IsDone(void);
/** @brief 是否失败中止 */
uint8_t SP_IsFailed(void);

/**
 * @brief  ★【2026-10-15 用户要求】是否已进入"**只剩多余动作**"的收尾段
 * @retval 1=3 件已全部放完、正在做"**回车载位不抓**"的收尾（`SP_ARM_IDLE`），
 *          且**舵机1 已回 0** —— 此时可以让底盘先动、手臂并行；0=还没到
 * @note   只有 `SP_SetRoughMode(1)`（ROUGH PLACE）才有这段收尾；
 *         STORE PLACE（rough=0）放完即 `SP_DONE`，本函数恒 0。
 *         `SP_Update()` 仍会照常把收尾动作跑完，**不会被本函数打断**。
 */
uint8_t SP_TailOnly(void);

/**
 * @brief  取状态摘要（页面第 3 行用）
 * @retval 形如 "SP arm observe" / "ALGN ..." / "SP put T2" / "SP DONE 3/3"
 */
const char *SP_GetStateText(void);

/**
 * @brief  取关键数据文本（页面第 4 行用）
 * @retval 形如 "S1 c3 t2 0/3"；空闲时 "---"
 */
const char *SP_GetDataText(void);

/* ===================== ③ 打靶取回序列（**STORE BACK**） ===================== */
/* 同一份 job 序列，**两个入口共用**：
 *   任务调试页 `TDBG_STORE_BACK`（bsp_task_dbg.c）
 *   赛题流程 `T_B1_PBACK`（`PLACE_BACK_ALL()`，bsp_task.c）
 * 流程：**按 STORE PLACE 的逆序** —— 车载位 **1→2→3**（STORE PLACE 是 3→2→1 取件）：
 *   读该位绑定色 -> 任务码**第 1 组**反查靶位 -> 从靶位取件 -> 放回车载该号位。
 *   **第一件**（= 最后放上去的那件）：臂**不上抬、不转向**，**直接抓**（`SB_SETTLE_MS` 默认 0）。
 * ⚠ 前提：先跑过 TRAY PICK（颜色↔车载位号绑定）+ STORE PLACE（料已在靶位上）；
 *   否则 `no binding` 失败。 */

/**
 * @brief  初始化（上电调用一次；只复位状态）
 */
void SB_Init(void);

/**
 * @brief  启动（从车载 3 号位起，按逆序取回 3 件）
 * @param  chained: 1 = **紧跟 STORE PLACE** 执行（臂正停在"最后一次放置的那个靶位"姿态上）
 *                  ⇒ 第一件走**不上抬、不转向**的短流程（`SB_SETTLE_MS` 默认 0 = **不再等 1s**）；
 *                  0 = **独立执行**（臂可能停在任何地方）⇒ 第一件也走完整流程，**更安全**。
 * @retval 1=已启动；0=拒绝（已在跑 / 对位或机械臂正忙）
 * @note   非阻塞：启动后由 `SB_Update()` 在主循环推进（main.c 调度）。
 *         ⚠ **两个入口现在都传 1**：赛题 `PLACE_BACK_ALL()`（紧跟 STORE PLACE）传 1；
 *           任务调试页 `TDBG_STORE_BACK` 也传 1（用户 2026-10-10 晚要求
 *           "**最开始不用回到观察位 / 不要先缩回再抬升，直接从抓取开始**"）。
 *           两者都必须**先跑过 STORE PLACE**（臂停在最后放置的靶位姿态）。
 *           只有"臂不在该姿态的冷启动"才传 **0**（走完整流程，先缩回+抬到安全高再转向靶位）。
 */
uint8_t SB_Start(uint8_t chained);

/**
 * @brief  ★设定本段按**任务码第几批**取颜色/靶位号
 * @param  batch: 1=第 1/2 组（第一批）；2=第 3/4 组（第二批）
 * @note   ★【2026-10-23】第二批"放置 -> 取回"里的取回段要用**第 3/4 组**，
 *         所以在 `SB_Start()` 之前调用（缺省 1）。
 */
void SB_SetBatch(uint8_t batch);

/**
 * @brief  中止（中止机械臂动作；本序列不动轮子，故无需停对位）
 */
void SB_Abort(void);

/**
 * @brief  主循环节拍（**漏调度会卡住不动**）
 */
void SB_Update(void);

/** @brief 是否正在跑 */
uint8_t SB_IsBusy(void);
/** @brief 是否已跑完（3 件全放回车载） */
uint8_t SB_IsDone(void);
/** @brief 是否失败中止 */
uint8_t SB_IsFailed(void);

/**
 * @brief  ★【2026-10-15 用户要求】是否已进入"**只剩多余动作**"的收尾段
 * @retval 1=**最后一件（第 3 件）已从靶位抓到手里、舵机1 已回 0**，
 *          剩下只有"放回车载 3 号位 + 抬回安全高"这类回位动作 —— 此时可以让
 *          底盘先动、手臂并行；0=还没到
 * @note   这正是用户举的例子："**抓到第三个物料时、舵机1 回退到 0 时，先让底盘动起来**"。
 *         `SB_Update()` 仍会照常把"放回车载"跑完，**不会被本函数打断**。
 */
uint8_t SB_TailOnly(void);

/**
 * @brief  取状态摘要（页面第 3 行用）
 * @retval 形如 "SB wait 1s" / "SB pick T3" / "SB put S3" / "SB DONE 3/3"
 */
const char *SB_GetStateText(void);

/**
 * @brief  取关键数据文本（页面第 4 行用）
 * @retval 形如 "S3 c2 t1 0/3"；空闲时 "---"
 */
const char *SB_GetDataText(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_TT_H */
