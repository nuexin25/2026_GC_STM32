/**
 ******************************************************************************
 * @file    bsp_servo.h
 * @brief   幻尔总线舵机「数值回读」模块（P8：舵机实际位置 + 调试板电压）
 * @author  nx_car
 * @date    2026-09-20
 ******************************************************************************
 * @note   【本模块解决什么问题】
 *         官方库 LobotServoController 只有"写"指令，拿不到舵机的实际位置；
 *         本模块补上"读"这一半：USART3 中断收字节 → 主循环解析 → 缓存实际位置，
 *         供手臂参数页显示真实角度、供动作序列做"回读到位判定"。
 *         **不改官方库任何函数签名与行为**，也**不改运动指令的发送方式**。
 *
 *         【协议必须分清 —— 这是以前"读不到"的根因】
 *         协议 A（**本模块使用**）：MCU ↔ 调试板，USART3(PD8=TX/PD9=RX)，9600-8-N-1
 *             帧：0x55 0x55 | Length | Cmd | 参数...        （**无校验和、无 ID 字段**）
 *             **Length = 参数个数 + 2；整帧字节数 = Length + 2**
 *             （即 Length 字段后面还有 Length-1 个字节：Cmd + 参数）
 *             运动 Cmd=0x03（官方库 moveServo）、读位置 Cmd=0x15、读电压 Cmd=0x0F
 *         协议 B（**禁止发给调试板**）：舵机原生协议，115200 半双工单总线
 *             帧：0x55 | ID | Length | Cmd | 参数 | CheckSum
 *             本地《幻尔科技总线舵机通信协议1.pdf》讲的是这一套（含 SERVO_POS_READ=28），
 *             它是"调试板 → 舵机"之间用的，调试板**不认**这种帧格式。
 *
 *         【读位置帧（Cmd 0x15）逐字节】
 *           发送：55 55 | N+3 | 15 | N | ID1 .. IDN        整帧 = N+5 字节
 *                 本项目 N=2（CLAW=1、GRIP=2）：55 55 05 15 02 01 02
 *           应答：55 55 | N*3+3 | 15 | N | ID1 pL pH | ... 整帧 = N*3+5 字节
 *                 位置为**小端 16 位**，范围 0~1000（对应 0~240°，1 单位 ≈ 0.24°）
 *                 N=2、位置都是 500 时应答：55 55 09 15 02 01 F4 01 02 F4 01
 *         【读电压帧（Cmd 0x0F）】
 *           发送：55 55 02 0F          应答：55 55 LL 0F vL vH（单位 mV）
 *
 *         【中断只搬字节】Servo_RxHandler() 在中断里只喂状态机；
 *         成帧后的解析在 Servo_Update()（主循环）里做，禁止在中断里 printf/延时。
 *
 *         硬约束：全部函数非阻塞；禁止 HAL_Delay；日志禁 %f；
 *                 初始化阶段只启动接收，不发任何舵机动作指令。
 ******************************************************************************
 */
#ifndef __BSP_SERVO_H
#define __BSP_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* ===================== 协议常量（按调试板官方协议固化，禁止随意改） ===================== */

#define SERVO_FRAME_HEAD        0x55U   /* 帧头字节（帧头是连续两个 0x55） */
#define SERVO_CMD_MULT_POS_READ 0x15U   /* 读多个舵机当前位置（应答含各舵机位置） */
#define SERVO_CMD_GET_VIN       0x0FU   /* 读调试板电压（应答 2 字节 mV，小端） */
#define SERVO_CMD_MULT_UNLOAD   0x14U   /* **卸载多个舵机**：关掉保持力矩，可用手自由扳动 */

/* 调试板会主动上报 0x06/0x07/0x08（动作组开始/停止/完成）帧；它们不是回读应答，
 * 解析时按"未知命令"直接丢弃，故不单独定义宏。 */

/* ===================== 可调参数宏（集中调参区） ===================== */

/* 回读周期，单位 ms，**现为 100**（下限），可调范围 100~1000。
 * 9600bps 下一帧 11 字节约 11.5ms，周期再小读请求会和运动指令挤在同一条慢速串口上。
 * 【它就是"舵机步之间的停顿"】动作库判"舵机步完成"必须等到一条**命令之后**的回读值，
 *   即至少要等 1 个本周期。一个抓取+放置循环有 5 个舵机步：
 *   本值 200 -> 停顿约 1s；本值 100 -> 约 0.5s（**取 100 就是为了少停顿**）。
 *   现场看 `stepN CLAW ok +NNN ms`，正常 ≈ 本值 + 10~30ms；若明显大于本值 +
 *   几十 ms，说明该步在**重发**（日志有 `no move ... -> resend`）。 */
#define SERVO_READ_PERIOD_MS    100U

/* 从发出读请求到收满整帧的等待上限，单位 ms，默认 50，可调范围 20~200。
 * 一帧 11 字节 @9600 约 11.5ms，50ms 足够；超时按 SERVO_READ_RETRY 重试。 */
#define SERVO_READ_TIMEOUT_MS   50U

/* 单次回读超时后的重试次数，默认 2，可调范围 0~3。
 * 用尽重试仍无应答：放弃本次请求，等下一个回读周期再来。 */
#define SERVO_READ_RETRY        2U

/* 接收缓冲字节数（装"帧头之后的 Length-1 个字节"），默认 32，可调范围 16~64。
 * 必须 ≥ N*3+2（N 个舵机的应答负载）：本项目 2 舵机 = 8，取 32 留足余量。 */
#define SERVO_RX_BUF_LEN        32U

/* 位置合法性上限（用于过滤坏帧），0~1000 对应 0~240°；下限恒为 0，不另设宏。 */
#define SERVO_POS_MAX           1000U

/* 位置数据有效期，单位 ms，默认 1000，可调范围 200~5000。
 * 最近一次成功回读超过本时长，Servo_GetPos 就报"无效"（避免用过时数据当真值）。 */
#define SERVO_POS_VALID_MS      1000U

/* 每隔多少个回读周期插一次"读电压"，默认 10，可调范围 0~100（0=不读电压）。
 * 10 × SERVO_READ_PERIOD_MS(200) = 每 2s 一次，用于确认串口回读通路与板子供电。 */
#define SERVO_VIN_EVERY_N       10U

/* 单帧发送超时，单位 ms，默认 50，可调范围 10~1000。只作兜底，正常不会等满。 */
#define SERVO_TX_TIMEOUT_MS     50U

/* ★【2026-10-23】收帧时"字节间隔"超时，单位 ms，默认 30（可调 10~100）。
 * 为什么要它：状态机按 Length 收 body，**一旦线上丢了一个字节**，它会一直等"够数"，
 * 把后面所有正常应答全吞进这一帧（最长吞 255 字节）—— 表现就是"回读全断、位置一直 --"，
 * 而读请求 pending 又一直不清，运动指令全被挡。超过本时长没有新字节 -> 复位找帧头。
 * 9600 下一个字节 ≈ 1.04ms，30ms 足够区分"帧内间隔"和"帧间空闲"。 */
#define SERVO_RX_FRAME_GAP_MS   30U

/* 回读请求与"舵机运动指令"之间的最小间隔，单位 ms，默认 50，可调范围 0~200。
 * 运动指令刚发出去就插读请求容易串帧，所以 Servo_NotifyMotionSent() 会记录
 * 运动指令时刻，Servo_Update() 在本间隔内不发读请求。
 * ★【2026-10-23】20 → 50：同一拍合并成一帧（十几~二十几字节）后，给调试板留足
 *   处理时间再发读请求，避免它"还在处理运动帧"就被问位置。 */
#define SERVO_MOVE_GAP_MS       50U

/* 位置缓存槽数（缓存最近回读过的舵机 ID 与位置），默认 4，可调范围 2~8。 */
#define SERVO_POS_CACHE_MAX     4U

/* 回读时默认要读的舵机 ID 列表（本项目 3 个：爪子伸缩 ID1、抓松 ID2、车载转盘 ID3）。
 * 与 bsp_arm.h 的 SERVO_ID_CLAW / SERVO_ID_GRIP / SERVO_ID_TURNTABLE 保持一致。 */
#define SERVO_READ_ID_COUNT     3U
#define SERVO_READ_ID1          1U   /* 舵机1：爪子前伸/后缩 */
#define SERVO_READ_ID2          2U   /* 舵机2：爪子抓紧/松开 */
#define SERVO_READ_ID3          3U   /* 舵机3：车载转盘（1/2/3 号盘选位） */

/* ===================== 对外函数 ===================== */

/**
 * @brief  回读模块初始化
 * @param  无
 * @retval 无
 * @note   清缓存/清统计，并**启动 USART3 单字节中断接收**。
 *         上电调用一次；**只启动接收，不发任何舵机动作指令**（防上电爪子突动）。
 *         必须在 main.c 把 USART3 运行期改成 9600 之后调用。
 */
void Servo_Init(void);

/**
 * @brief  回读主循环节拍（非阻塞）
 * @param  无
 * @retval 无
 * @note   在主循环里调度（放在 Arm_Update() 之后）。职责：
 *           1) 处理中断攒好的完整帧（**解析在主循环**，不在中断里做）；
 *           2) 按 SERVO_READ_PERIOD_MS 周期发一次读位置请求（每隔
 *              SERVO_VIN_EVERY_N 次插一次读电压）；
 *           3) 检查应答超时并重试，累计 timeout 统计，限频打告警日志。
 *         漏调度 → 回读永不刷新（位置一直显示 --）。
 */
void Servo_Update(void);

/**
 * @brief  主动发一次"读多个舵机位置"请求（Cmd 0x15）
 * @param  num: 舵机个数，范围 1~8（本项目用 2）
 * @param  ids: 舵机 ID 数组，各 ID 范围 1~31（官方库限制）
 * @retval 无
 * @note   用于需要立刻拿一次读数的场合（如参数页 KEY3 刷新）。
 *         同一时刻只允许一个未完成的读请求（调试板是一问一答），
 *         若上一个请求还没完成，本函数不会发新请求。
 */
void Servo_ReadPos(uint8_t num, const uint8_t *ids);

/**
 * @brief  卸载（关掉）多个舵机的保持力矩
 * @param  num: 舵机个数，范围 1~8（本项目用 2）
 * @param  ids: 舵机 ID 数组，各 ID 范围 1~31
 * @retval 无
 * @note   对应官方二次开发协议 `CMD_MULT_SERVO_UNLOAD = 0x14`：
 *         "被指定的舵机**失去保持力矩，可以用手自由转动**"。
 *         帧：`0x55 0x55 | N+3 | 0x14 | N | ID1..IDN`（N=2 时 = `55 55 05 14 02 01 02`）。
 *
 *         【怎么恢复力矩】协议里**没有**单独的"装载"指令 —— 再给该舵机发一条
 *         **运动指令**（`moveServo` / `Arm_ServoMove`）它就会重新出力并转到目标位置。
 *         本项目的做法：手臂参数页（ARMPARA）进入时卸载、退出时按"最后下发位置"
 *         再发一条运动指令把力矩恢复回来。
 */
void Servo_Unload(uint8_t num, const uint8_t *ids);

/**
 * @brief  取某舵机最近一次成功回读的实际位置
 * @param  servoId: 舵机 ID（1 / 2）
 * @param  out_pos: 输出位置 0~1000；**返回 0 时不会改写它**
 * @retval 1=有效（有回读且未超过 SERVO_POS_VALID_MS）；0=无有效回读
 */
uint8_t Servo_GetPos(uint8_t servoId, uint16_t *out_pos);

/**
 * @brief  取最近一次成功回读的时刻
 * @param  无
 * @retval 最近一次成功解析 0x15 应答时的 HAL_GetTick() 计数
 * @note   调用方可用 (int32_t)(Servo_GetPosTick() - 某时刻) >= 0 判断
 *         "这次回读是不是在某条命令之后拿到的"（动作序列用它避免用旧值提前放行）。
 */
uint32_t Servo_GetPosTick(void);

/**
 * @brief  ★【2026-10-27】取**某舵机 ID 自己**最近一次成功回读的时刻
 * @param  servoId: 舵机 ID
 * @retval 该 ID 最后一次回读的 HAL_GetTick()；无有效数据返回 0
 * @note   判"这条命令之后有没有新回读"**必须**用它，不能用全局 `Servo_GetPosTick()`：
 *         全局时刻会被**别的舵机**的回读刷新成新鲜，于是本 ID 的旧缓存被当成新值，
 *         上层会"用命令之前的旧位置提前放行/误判已到位"。这是"动作时有时无"的来源之一。
 *         用法：`(int32_t)(Servo_GetPosTickById(id) - 命令时刻) >= 0`。
 */
uint32_t Servo_GetPosTickById(uint8_t servoId);

/**
 * @brief  取最近一次回读到的调试板电压
 * @param  out_mv: 输出电压，单位 mV；**返回 0 时不会改写它**
 * @retval 1=有效；0=还没读到过
 */
uint8_t Servo_GetVin(uint16_t *out_mv);

/**
 * @brief  取回读统计（供日志与页面）
 * @param  rxFrames: 成功解析的回读帧数（可为 NULL）
 * @param  badFrames: 长度/范围校验失败的坏帧数（可为 NULL）
 * @param  timeouts: 读请求超时次数（可为 NULL）
 * @retval 无
 */
void Servo_GetStat(uint32_t *rxFrames, uint32_t *badFrames, uint32_t *timeouts);

/**
 * @brief  通知回读模块"刚发过一条舵机运动指令"
 * @param  无
 * @retval 无
 * @note   由 bsp_arm.c 的 Arm_ServoMove() 调用。回读模块会因此把下一条读请求
 *         推迟至少 SERVO_MOVE_GAP_MS，避免读请求和运动指令挤在一起串帧。
 *         主循环上下文调用。
 */
void Servo_NotifyMotionSent(void);

/* ===================== ★【2026-10-11 根治"舵机丢步"】总线仲裁 ===================== */
/* 【为什么需要】舵机**运动指令**（Cmd 0x03，官方库 moveServo）与**位置回读请求**
 *   （Cmd 0x15，本模块）走的是**同一条 9600 的 USART3**，而调试板是**一问一答**设备：
 *   在它正忙（解析/应答上一条回读）时插进去的那条运动指令**会被它丢掉**。
 *   旧的 `Servo_NotifyMotionSent()` 只能"把**下一条**读请求推迟 SERVO_MOVE_GAP_MS"，
 *   **根本无法收回已经在途的那一条读请求**，所以"两条挤在一起"的概率一直在
 *   （读周期 100ms、每帧约 12~25ms，抢线概率 20%~50%），现场表现就是
 *   **"某个关节偶尔整步没执行"（丢步），舵机尤其明显**（电机走 CAN，不受此影响）。
 *
 * 【★2026-10-23 按新判据重做】原来的判据是"距**请求发出**够久"，但应答迟到/重发时
 *   这个窗口是假的，板子还在应答就插指令 -> 丢。现在改为**物理静默判据**：
 *     · 本机发送完成（`huart3.gState == READY`）；
 *     · 不在收帧中途（`servoRxStep == HEAD1`）；
 *     · 在读请求的应答已到（`servoReqPending == 0`）；
 *     · 距**最后一个收到的字节** ≥ quietMs（`servoLastRxTick`，中断里更新）。
 *   调用方用 `Servo_WaitBusIdle()` 在**下发运动指令前**等这个条件（有硬上限兜底）。
 */
/** 发运动指令前要求的静默时长（距最后收到的字节），单位 ms，可调 10~100 */
#define SERVO_QUIET_BEFORE_MOVE_MS   25U
/** 等静默的硬上限，单位 ms，可调 20~200；到点强发，绝不为"总线忙"卡死
 *  【2026-10-23 由 45 提到 80】3 个 ID 的回读应答 ≈ 14.6ms + 调试板处理，
 *  45ms 有时仍不够，会在应答尾巴上插指令 -> 偶发"完全没动"。 */
#define SERVO_WAIT_IDLE_HARD_MS      80U

/**
 * @brief  【总线仲裁】让出总线给舵机运动指令（幂等，可连续调用）
 * @param  无
 * @retval 无
 * @note   ① 有在途读请求 -> **放弃它**（它的应答到了仍会被正常解析、写入位置缓存，
 *             所以只是放弃"重试逻辑"，不丢数据）；
 *         ② 读节拍与"运动指令间隔"从此刻重新计时，避免刚发完指令立刻又发读请求。
 *         ⚠ 只有在 `Servo_WaitBusIdle()` 超时（板子不应答）时才该调用它强发；
 *           正常路径不要先 Yield 再发（那等于"应答还在路上就插指令"）。
 */
void Servo_BusYieldForMotion(void);
/**
 * @brief  【总线仲裁】总线是否已"静默"（可以安全下发舵机运动指令）
 * @param  quietMs: 要求"距**最后收到的字节**"至少过了这么多 ms
 * @retval 1=静默；0=还有读请求在途 / 正在收帧 / 刚收到字节 / 本机还在发
 */
uint8_t Servo_BusQuiet(uint16_t quietMs);

/**
 * @brief  ★【2026-10-23】等总线真正静默（等应答收完），最多等 hardMs
 * @param  quietMs: 静默判定窗（建议 `SERVO_QUIET_BEFORE_MOVE_MS`）
 * @param  hardMs : 硬上限（建议 `SERVO_WAIT_IDLE_HARD_MS`），0 = 不限（不要这么用）
 * @retval 1=已静默（可发运动指令）；0=超时（板子不应答 / 一直在收，调用方可强发）
 * @note   等待期间会**顺便解析已收到的应答帧**（`Servo_ServiceRx`），
 *         否则 `servoReqPending` 永远清不掉、必然等满硬上限。
 *         主循环上下文调用；最长阻塞 hardMs（默认 45ms），不调用 HAL_Delay。
 */
uint8_t Servo_WaitBusIdle(uint16_t quietMs, uint16_t hardMs);

/**
 * @brief  ★【2026-10-23】处理"中断已攒好"的应答帧（主循环 / 等待总线时用）
 * @param  无
 * @retval 无
 * @note   与 `Servo_Update()` 开头那步相同；抽出来给 `Servo_WaitBusIdle` 复用。
 */
void Servo_ServiceRx(void);

/**
 * @brief  【总线仲裁】当前是否**有读请求在途**（调试板还没应答完）
 * @param  无
 * @retval 1=在途（此时发运动指令很可能被丢）
 * @note   用途：动作层"等不到静默就强发"时，用它兜住 —— 只要应答还在途，
 *         **宁可再等一拍也不发**，避免运动指令与应答撞在一起（现场"丢步"的主因之一）。
 */
uint8_t Servo_ReqPending(void);

/**
 * @brief  USART3 单字节接收中断处理（由 bsp_imu.c 的 RxCpltCallback 转发）
 * @param  b: 刚收到的字节
 * @retval 无
 * @note   **中断上下文**：只喂解析状态机（攒字节/定界），
 *         禁止 printf、禁止 HAL_Delay、禁止任何阻塞；
 *         成帧后只置"帧就绪"标志，真正的解析交给 Servo_Update()。
 */
void Servo_RxHandler(uint8_t b);

/**
 * @brief  通知 USART3 接收出错（ORE/FE/NE/PE）
 * @param  无
 * @retval 无
 * @note   由 bsp_imu.c 的 HAL_UART_ErrorCallback 调用。只累加错误计数；
 *         实际重装 RX 中断由 ErrorCallback 完成（不重装会永久收不到数据）。
 */
void Servo_NotifyError(void);

#ifdef __cplusplus
}
#endif

/* ===================== USART3 接收字节（供 bsp_imu.c 回调重装 IT 用） ===================== */
/* 非静态：HAL_UART_Receive_IT 需要往这个变量地址写收到的字节，
   bsp_imu.c 的 HAL_UART_RxCpltCallback 里 extern 引用后重新挂接收。
   与 qrRxByte 的现有做法保持一致（不用 volatile，
   因为 HAL_UART_Receive_IT 形参是 uint8_t*，加 volatile 会类型不匹配告警）。 */
extern uint8_t servoRxByte;

#endif /* __BSP_SERVO_H */
