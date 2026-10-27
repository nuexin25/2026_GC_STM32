/**
 ******************************************************************************
 * @file    bsp_servo.c
 * @brief   幻尔总线舵机「数值回读」实现（P8：位置 + 调试板电压）
 * @author  nx_car
 * @date    2026-09-20
 ******************************************************************************
 * @note   数据流（严格按"中断只搬字节、解析在主循环"的约束）：
 *
 *      USART3 收到 1 字节
 *        └─（中断）bsp_imu.c 的 HAL_UART_RxCpltCallback
 *              └─ Servo_RxHandler(b)        只喂状态机：攒字节、按 Length 定界
 *                    └─ 收满一帧 → servoRxReady = 1
 *        └─（主循环）Servo_Update()
 *              └─ Servo_FrameDispatch()     解析 Cmd / 校验长度 / 写位置缓存
 *
 *     发送侧：Servo_Update() 按 SERVO_READ_PERIOD_MS 周期发一条读请求，
 *             用"单发单收 + 超时重试"保证不会串帧。
 *
 *     协议与帧示例见 bsp_servo.h 头部注释（协议 A：MCU ↔ 调试板，9600、双 0x55、无校验）。
 *
 *     硬约束：禁止 HAL_Delay；日志禁 %f（位置/电压都用整数打印）；
 *             中断里不做解析、不 printf。
 ******************************************************************************
 */
#include "bsp_servo.h"
#include "usart.h"        /* huart3：USART3 句柄（PD8=TX / PD9=RX） */
#include "bsp_debug.h"

/* ===================== 内部常量 ===================== */

/* 接收状态机步骤（只在中断里推进） */
#define SERVO_RX_STEP_HEAD1   0U   /* 等第 1 个帧头 0x55 */
#define SERVO_RX_STEP_HEAD2   1U   /* 等第 2 个帧头 0x55 */
#define SERVO_RX_STEP_LEN     2U   /* 收 Length 字段 */
#define SERVO_RX_STEP_BODY    3U   /* 收 Length-1 个字节（Cmd + 参数） */

/* 发送缓冲长度：最大帧 = N(8) + 5 = 13，取 16 留余量 */
#define SERVO_TX_BUF_LEN      16U

/* 单个舵机 ID 上限（官方库 moveServo 的硬校验也是 31） */
#define SERVO_ID_MAX          31U

/* 一次读请求最多带几个舵机 ID（按协议 A 的定义取 8） */
#define SERVO_READ_NUM_MAX    8U

/* 位置日志与超时告警的最小打印间隔，单位 ms（防刷屏） */
#define SERVO_LOG_MIN_MS      1000U

/* 周期统计日志间隔，单位 ms */
#define SERVO_STAT_LOG_MS     5000U

/* ===================== 内部状态 ===================== */

/* ---- USART3 接收字节（非静态：bsp_imu.c 的回调要往它地址里收字节） ---- */
uint8_t servoRxByte = 0U;

/* ---- 接收状态机（只在中断上下文推进） ---- */
static volatile uint8_t servoRxStep = SERVO_RX_STEP_HEAD1;  /* 当前步骤（主循环也要读：判"正在收帧"） */
static uint8_t servoRxLen   = 0U;                       /* 本帧 Length 字段值 */
static uint8_t servoRxIdx   = 0U;                       /* body 已收字节数 */
static uint8_t servoRxBody[SERVO_RX_BUF_LEN];           /* body：下标 0 = Cmd，其后为参数 */
static uint8_t servoRxReady = 0U;                       /* 1=有一帧待主循环解析 */
/* ★【2026-10-23】最后一个**收到字节**的时刻（中断里更新）。
   总线静默判据不再用"请求发出时刻"，而用它的物理依据 —— 答复真的发完了。 */
static volatile uint32_t servoLastRxTick = 0U;

/* ---- 发送缓冲（重试时原样重发，所以存下来） ---- */
static uint8_t servoTxBuf[SERVO_TX_BUF_LEN];
static uint8_t servoTxLen = 0U;

/* ---- 读请求状态（单发单收） ---- */
static uint8_t  servoReqPending = 0U;                   /* 1=已发出请求，等应答 */
static uint32_t servoReqTick    = 0U;                   /* 本请求发出时刻 */
static uint8_t  servoRetryLeft  = 0U;                   /* 剩余重试次数 */
static uint32_t servoReqPeriodTick = 0U;                /* 上次发起请求的时刻（周期节拍） */
static uint16_t servoVinDiv     = 0U;                   /* 读电压分频计数 */
static uint32_t servoLastMotionTick = 0U;               /* 上次发舵机运动指令的时刻 */

/* ---- 回读结果缓存 ---- */
static uint8_t  servoPosId[SERVO_POS_CACHE_MAX];        /* 缓存槽对应的舵机 ID */
static uint16_t servoPosVal[SERVO_POS_CACHE_MAX];       /* 缓存槽的位置值 0~1000 */
static uint8_t  servoPosOk[SERVO_POS_CACHE_MAX];        /* 1=该槽有有效数据 */
static uint8_t  servoPosCnt = 0U;                       /* 已使用槽数 */
static uint32_t servoPosTick = 0U;                      /* 最近一次成功回读位置（任意 ID）的时刻 */
/* ★【2026-10-27 修"偶发丢动作"】**每个 ID 各自**的回读时刻。
   旧实现只有一个全局时刻：只要**任一**响应帧解析成功就把全局时刻刷新，
   于是某个 ID 那几帧被丢弃（长度/范围校验失败、只读了部分 ID、或它一直没答）时，
   它的**陈旧缓存**仍会被判成"有效" -> 上层据旧值误判"已到位"而**跳过下发**、
   或验证层误判"没动" -> 动作时有时无。 */
static uint32_t servoPosTickById[SERVO_POS_CACHE_MAX];  /* 各槽最后一次回读时刻 */
static uint16_t servoVinMv = 0U;                        /* 最近一次回读到的电压 mV */
static uint8_t  servoVinValid = 0U;                     /* 1=电压有效 */

/* ---- 统计 ---- */
static uint32_t servoRxFrames    = 0U;                  /* 成功解析的回读帧数 */
static uint32_t servoBadFrames   = 0U;                  /* 长度/范围校验失败帧数 */
static uint32_t servoTimeouts    = 0U;                  /* 读请求超时次数 */
static uint32_t servoOtherFrames = 0U;                  /* 主动上报/未知 Cmd 帧数 */
static uint32_t servoBusYielded  = 0U;                  /* ★为发运动指令而"让出"的在途读请求次数 */

/* ---- 日志限频 ---- */
static uint32_t servoLastPosLogTick = 0U;
static uint32_t servoLastToLogTick  = 0U;
static uint32_t servoLastStatTick   = 0U;

/* ===================== 位置缓存 ===================== */

/**
 * @brief  在缓存里查找某舵机的槽位
 * @param  id: 舵机 ID
 * @retval >=0 槽位下标；-1 未找到
 */
static int8_t Servo_CacheFind(uint8_t id)
{
    uint8_t i;

    for (i = 0U; i < servoPosCnt; i++)
    {
        if (servoPosId[i] == id)
        {
            return (int8_t)i;
        }
    }
    return -1;
}

/**
 * @brief  写入一个回读位置（新 ID 则占一个新槽）
 * @param  id : 舵机 ID
 * @param  pos: 位置 0~1000（调用方已做范围校验）
 * @retval 无
 * @note   槽位用满则直接丢弃本次数据（不覆盖别人的，避免读数错乱）。
 */
static void Servo_CachePut(uint8_t id, uint16_t pos)
{
    int8_t idx = Servo_CacheFind(id);

    if (idx < 0)
    {
        if (servoPosCnt >= (uint8_t)SERVO_POS_CACHE_MAX)
        {
            return;   /* 缓存满，丢弃 */
        }
        idx = (int8_t)servoPosCnt;
        servoPosId[servoPosCnt] = id;
        servoPosCnt++;
    }

    servoPosVal[(uint8_t)idx] = pos;
    servoPosOk[(uint8_t)idx]  = 1U;
    /* ★按槽记时刻：过期判断只认"这个 ID 自己"最近一次回读 */
    servoPosTickById[(uint8_t)idx] = HAL_GetTick();
}

/**
 * @brief  从缓存取某舵机的位置（不做"数据过期"判断）
 * @param  id     : 舵机 ID
 * @param  out    : 输出位置，可为 NULL
 * @param  out_tick: 输出该槽最后一次回读时刻，可为 NULL
 * @retval 1=取到；0=该 ID 没有有效数据
 */
static uint8_t Servo_CacheGet(uint8_t id, uint16_t *out, uint32_t *out_tick)
{
    int8_t idx = Servo_CacheFind(id);

    if (idx < 0)
    {
        return 0U;
    }
    if (servoPosOk[(uint8_t)idx] == 0U)
    {
        return 0U;
    }
    if (out != NULL)
    {
        *out = servoPosVal[(uint8_t)idx];
    }
    if (out_tick != NULL)
    {
        *out_tick = servoPosTickById[(uint8_t)idx];
    }
    return 1U;
}

/* ===================== 发送 ===================== */

/**
 * @brief  把当前发送缓冲里的帧发给调试板
 * @param  无
 * @retval 无
 * @note   servoTxLen 为 0 表示没有待发内容（组帧失败），直接返回。
 *         有界超时，不会卡死主循环。
 */
static void Servo_TxFlush(void)
{
    if (servoTxLen == 0U)
    {
        return;
    }
    (void)HAL_UART_Transmit(&huart3, servoTxBuf, (uint16_t)servoTxLen,
                            (uint32_t)SERVO_TX_TIMEOUT_MS);
}

/**
 * @brief  组一帧"读多个舵机位置"请求（Cmd 0x15）
 * @param  num: 需要读取的 ID 个数
 * @param  ids: ID 数组
 * @retval 无
 * @note   帧结构（协议 A）：
 *           [0][1] 帧头 0x55 0x55
 *           [2]    Length = 参数个数 + 2，参数 = 个数 N + N 个 ID，故 Length = N + 3
 *           [3]    Cmd = 0x15
 *           [4]    N（舵机个数）
 *           [5..]  ID1 .. IDN
 *           整帧字节数 = Length + 2 = N + 5
 *         示例（N=2、ID=1/2）：55 55 05 15 02 01 02
 *         非法 ID（不在 1~31）会被跳过；一个合法 ID 都没有时不组帧。
 */
static void Servo_BuildReadPosReq(uint8_t num, const uint8_t *ids)
{
    uint8_t i;
    uint8_t n = 0U;

    servoTxLen = 0U;

    if ((num == 0U) || (ids == NULL) || (num > (uint8_t)SERVO_READ_NUM_MAX))
    {
        return;
    }

    for (i = 0U; i < num; i++)
    {
        uint8_t id = ids[i];

        if ((id >= 1U) && (id <= (uint8_t)SERVO_ID_MAX))
        {
            servoTxBuf[5U + n] = id;
            n++;
        }
    }

    if (n == 0U)
    {
        return;   /* 没有合法 ID，不发 */
    }

    servoTxBuf[0] = SERVO_FRAME_HEAD;
    servoTxBuf[1] = SERVO_FRAME_HEAD;
    servoTxBuf[2] = (uint8_t)(n + 3U);              /* Length = N + 3 */
    servoTxBuf[3] = SERVO_CMD_MULT_POS_READ;        /* 0x15 */
    servoTxBuf[4] = n;
    servoTxLen    = (uint8_t)(n + 5U);              /* 整帧 = Length + 2 */
}

/**
 * @brief  组一帧"读调试板电压"请求（Cmd 0x0F）
 * @param  无
 * @retval 无
 * @note   帧：55 55 02 0F（共 4 字节），无参数。
 */
static void Servo_BuildVinReq(void)
{
    servoTxBuf[0] = SERVO_FRAME_HEAD;
    servoTxBuf[1] = SERVO_FRAME_HEAD;
    servoTxBuf[2] = 2U;                       /* Length = 参数个数(0) + 2 */
    servoTxBuf[3] = SERVO_CMD_GET_VIN;        /* 0x0F */
    servoTxLen    = 4U;                       /* 整帧 = Length + 2 */
}

/* ===================== 接收解析（主循环上下文） ===================== */

/**
 * @brief  位置回读日志（限频，避免刷屏）
 * @param  无
 * @retval 无
 * @note   格式：[SRV] pos id1=105 id2=990；无效的显示 -1。
 */
static void Servo_LogPosThrottled(void)
{
    uint32_t now = HAL_GetTick();
    uint16_t p1 = 0U;
    uint16_t p2 = 0U;
    uint8_t  ok1;
    uint8_t  ok2;

    if ((now - servoLastPosLogTick) < (uint32_t)SERVO_LOG_MIN_MS)
    {
        return;
    }
    servoLastPosLogTick = now;

    ok1 = Servo_CacheGet((uint8_t)SERVO_READ_ID1, &p1, NULL);
    ok2 = Servo_CacheGet((uint8_t)SERVO_READ_ID2, &p2, NULL);

    Debug_Log("SRV", "pos id%u=%d id%u=%d",
              (unsigned int)SERVO_READ_ID1, (ok1 != 0U) ? (int)p1 : -1,
              (unsigned int)SERVO_READ_ID2, (ok2 != 0U) ? (int)p2 : -1);
}

/**
 * @brief  读请求超时告警（限频）
 * @param  无
 * @retval 无
 * @note   格式：[SRV] read timeout x3（x 后面是累计超时次数）
 */
static void Servo_LogTimeoutThrottled(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - servoLastToLogTick) < (uint32_t)SERVO_LOG_MIN_MS)
    {
        return;
    }
    servoLastToLogTick = now;

    Debug_Log("SRV", "read timeout x%u", (unsigned int)servoTimeouts);
}

/**
 * @brief  周期统计日志
 * @param  无
 * @retval 无
 * @note   格式：[SRV] stat rx=120 bad=0 to=2 oth=0
 *         rx=成功回读帧数、bad=坏帧数、to=超时次数、oth=主动上报/未知帧数
 */
static void Servo_LogStatThrottled(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - servoLastStatTick) < (uint32_t)SERVO_STAT_LOG_MS)
    {
        return;
    }
    servoLastStatTick = now;

    Debug_Log("SRV", "stat rx=%u bad=%u to=%u oth=%u yl=%u",
              (unsigned int)servoRxFrames, (unsigned int)servoBadFrames,
              (unsigned int)servoTimeouts, (unsigned int)servoOtherFrames,
              (unsigned int)servoBusYielded);
}

/**
 * @brief  解析一帧完整应答（**主循环上下文**，不在中断里调用）
 * @param  无
 * @retval 无
 * @note   缓冲口径统一：servoRxBody[0] = Cmd，其后是参数；
 *         servoRxLen = Length 字段值 = body 字节数 + 1。
 *         只处理 0x15（位置）与 0x0F（电压）；调试板主动上报的
 *         0x06/0x07/0x08 与未知 Cmd 一律忽略（不计坏帧，计入 oth）。
 */
static void Servo_FrameDispatch(void)
{
    uint8_t cmd = servoRxBody[0];

    if (cmd == SERVO_CMD_MULT_POS_READ)
    {
        uint8_t n = servoRxBody[1];
        uint8_t i;

        /* 长度自洽校验：Length 应等于 N*3 + 3（body 字节数 = Length-1 = N*3+2） */
        if ((n == 0U) || ((uint16_t)servoRxLen != (((uint16_t)n * 3U) + 3U)))
        {
            servoBadFrames++;
            servoReqPending = 0U;   /* 答非所问，本次请求就此结束，不干等超时 */
            return;
        }

        for (i = 0U; i < n; i++)
        {
            /* body 布局：body[2+3i]=ID，body[3+3i]=位置低字节，body[4+3i]=位置高字节 */
            uint8_t  id  = servoRxBody[2U + (i * 3U)];
            uint16_t pos = (uint16_t)servoRxBody[3U + (i * 3U)]
                         | ((uint16_t)servoRxBody[4U + (i * 3U)] << 8);

            /* 位置范围校验：uint16 天然 >= 0，只需查上限 */
            if (pos > (uint16_t)SERVO_POS_MAX)
            {
                servoBadFrames++;
                continue;
            }

            Servo_CachePut(id, pos);
        }

        servoRxFrames++;
        servoPosTick    = HAL_GetTick();
        servoReqPending = 0U;

        Servo_LogPosThrottled();
    }
    else if (cmd == SERVO_CMD_GET_VIN)
    {
        /* 应答：body[0]=0x0F，body[1]=电压低字节，body[2]=电压高字节 */
        if (servoRxLen < 3U)
        {
            servoBadFrames++;
            servoReqPending = 0U;
            return;
        }

        servoVinMv    = ((uint16_t)servoRxBody[2] << 8) | (uint16_t)servoRxBody[1];
        servoVinValid = 1U;
        servoRxFrames++;
        servoReqPending = 0U;

        Debug_Log("SRV", "rx path ok, vin=%u mV", (unsigned int)servoVinMv);
    }
    else
    {
        /* 0x06/0x07/0x08 动作组主动上报，或未知 Cmd：忽略 */
        servoOtherFrames++;
    }
}

/* ===================== 中断回调 ===================== */

/**
 * @brief  USART3 单字节接收处理（**中断上下文**）
 * @param  b: 刚收到的字节
 * @retval 无
 * @note   只做"按双帧头 + Length 定界"的攒字节，收满一帧只置 servoRxReady；
 *         解析在主循环 Servo_Update() 里做（硬约束：中断只搬字节）。
 */
void Servo_RxHandler(uint8_t b)
{
    /* ★【2026-10-23】无论字节是否成帧，都记"最后一个收到的字节时刻"：
       总线仲裁用它判断"板子的应答真的发完了没有"（比"请求发出时刻"可靠）。 */
    servoLastRxTick = HAL_GetTick();

    switch (servoRxStep)
    {
        case SERVO_RX_STEP_HEAD1:
            if (b == SERVO_FRAME_HEAD)
            {
                servoRxStep = SERVO_RX_STEP_HEAD2;
            }
            break;

        case SERVO_RX_STEP_HEAD2:
            if (b == SERVO_FRAME_HEAD)
            {
                servoRxStep = SERVO_RX_STEP_LEN;
            }
            else
            {
                servoRxStep = SERVO_RX_STEP_HEAD1;   /* 不是帧头，重新找 */
            }
            break;

        case SERVO_RX_STEP_LEN:
            /* Length = (Cmd + 参数) 字节数 + 1，合法范围 2 ~ SERVO_RX_BUF_LEN-2 */
            if ((b >= 2U) && (b <= (uint8_t)(SERVO_RX_BUF_LEN - 2U)))
            {
                servoRxLen  = b;
                servoRxIdx  = 0U;
                servoRxStep = SERVO_RX_STEP_BODY;
            }
            else
            {
                servoRxStep = SERVO_RX_STEP_HEAD1;   /* 长度非法：丢弃重来 */
            }
            break;

        default:   /* SERVO_RX_STEP_BODY */
            servoRxBody[servoRxIdx] = b;
            servoRxIdx++;
            if (servoRxIdx >= (uint8_t)(servoRxLen - 1U))
            {
                servoRxReady = 1U;                  /* 成帧，交给主循环解析 */
                servoRxStep  = SERVO_RX_STEP_HEAD1; /* 立刻开始找下一帧 */
            }
            break;
    }
}

/**
 * @brief  通知 USART3 接收错误（ORE/FE/NE/PE）
 * @param  无
 * @retval 无
 * @note   只累加超时统计槽里的错误计数（复用为"串口异常次数"），
 *         实际重装 RX 中断由 bsp_imu.c 的 ErrorCallback 完成。
 */
void Servo_NotifyError(void)
{
    servoBadFrames++;
}

/* ===================== 公开接口 ===================== */

/**
 * @brief  回读模块初始化（只启动接收，不发任何舵机动作指令）
 * @param  无
 * @retval 无
 * @note   必须在 main.c 把 USART3 运行期改成 SERVO_BUS_BAUD(9600) 之后调用。
 */
void Servo_Init(void)
{
    uint8_t i;

    /* ---- 清缓存与统计 ---- */
    for (i = 0U; i < (uint8_t)SERVO_POS_CACHE_MAX; i++)
    {
        servoPosId[i]  = 0U;
        servoPosVal[i] = 0U;
        servoPosOk[i]  = 0U;
        servoPosTickById[i] = 0U;
    }
    servoPosCnt  = 0U;
    servoPosTick = 0U;

    servoVinMv    = 0U;
    servoVinValid = 0U;

    servoRxFrames    = 0U;
    servoBadFrames   = 0U;
    servoTimeouts    = 0U;
    servoOtherFrames = 0U;
    servoBusYielded  = 0U;

    /* ---- 清接收状态机与请求状态 ---- */
    servoRxStep  = SERVO_RX_STEP_HEAD1;
    servoRxLen   = 0U;
    servoRxIdx   = 0U;
    servoRxReady = 0U;
    /* 让启动后可以立刻发运动指令：把"最后收字节时刻"设成够久以前 */
    servoLastRxTick = HAL_GetTick() - (uint32_t)SERVO_QUIET_BEFORE_MOVE_MS;

    servoReqPending = 0U;
    servoRetryLeft  = 0U;
    servoVinDiv     = 0U;
    servoTxLen      = 0U;

    servoReqPeriodTick  = HAL_GetTick();
    /* 让启动后可以立刻发第一条读请求（相当于"距离上次运动指令已经很久"） */
    servoLastMotionTick = HAL_GetTick() - (uint32_t)SERVO_MOVE_GAP_MS;

    servoLastPosLogTick = HAL_GetTick();
    servoLastToLogTick  = HAL_GetTick();
    servoLastStatTick   = HAL_GetTick();

    /* ---- 启动 USART3 单字节中断接收（此后由 bsp_imu.c 的 RxCpltCallback 续接） ---- */
    if (HAL_UART_Receive_IT(&huart3, &servoRxByte, 1U) != HAL_OK)
    {
        Debug_Log("SRV", "init FAIL, rx IT not started");
        return;
    }

    Debug_Log("SRV", "init, baud=%u, read period=%ums",
              (unsigned int)huart3.Init.BaudRate, (unsigned int)SERVO_READ_PERIOD_MS);
}

/**
 * @brief  回读主循环节拍（非阻塞）
 * @param  无
 * @retval 无
 */
void Servo_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* ---- 1) 处理中断攒好的完整帧（解析在主循环） ---- */
    Servo_ServiceRx();

    /* ---- 1b) ★半帧超时复位（丢字节后状态机会卡在 BODY 等够数，
               把后面所有正常应答都吞掉 -> 回读全断、运动指令被 pending 挡住） ---- */
    if ((servoRxStep != SERVO_RX_STEP_HEAD1) &&
        ((now - servoLastRxTick) >= (uint32_t)SERVO_RX_FRAME_GAP_MS))
    {
        servoRxStep = SERVO_RX_STEP_HEAD1;
        servoRxIdx  = 0U;
        servoBadFrames++;
    }

    /* ---- 2) 有未完成的读请求：判超时 + 重试 ---- */
    if (servoReqPending != 0U)
    {
        if ((now - servoReqTick) >= (uint32_t)SERVO_READ_TIMEOUT_MS)
        {
            servoTimeouts++;

            if (servoRetryLeft > 0U)
            {
                servoRetryLeft--;
                servoReqTick = now;
                Servo_TxFlush();            /* 原样重发上一条请求 */
            }
            else
            {
                servoReqPending = 0U;       /* 放弃本次，等下一个周期 */
                Servo_LogTimeoutThrottled();
            }
        }

        Servo_LogStatThrottled();
        return;   /* 单发单收：请求没结束就不再发新请求 */
    }

    /* ---- 3) 周期发起新请求 ---- */
    if ((now - servoReqPeriodTick) < (uint32_t)SERVO_READ_PERIOD_MS)
    {
        Servo_LogStatThrottled();
        return;
    }
    servoReqPeriodTick = now;

    /* 运动指令刚发出去 SERVO_MOVE_GAP_MS 内不发读请求，避免挤在同一条慢串口上 */
    if ((now - servoLastMotionTick) < (uint32_t)SERVO_MOVE_GAP_MS)
    {
        return;
    }

    /* 每 SERVO_VIN_EVERY_N 个周期插一次"读电压"，其余周期读位置 */
    servoVinDiv++;
#if (SERVO_VIN_EVERY_N != 0U)
    if (servoVinDiv >= (uint16_t)SERVO_VIN_EVERY_N)
    {
        servoVinDiv = 0U;
        Servo_BuildVinReq();
    }
    else
#endif
    {
        static const uint8_t defaultIds[SERVO_READ_ID_COUNT] = {
            (uint8_t)SERVO_READ_ID1, (uint8_t)SERVO_READ_ID2, (uint8_t)SERVO_READ_ID3
        };
        Servo_BuildReadPosReq((uint8_t)SERVO_READ_ID_COUNT, defaultIds);
    }

    if (servoTxLen == 0U)
    {
        return;   /* 组帧失败（ID 非法），本次不发 */
    }

    servoReqPending = 1U;
    servoReqTick    = now;
    servoRetryLeft  = (uint8_t)SERVO_READ_RETRY;
    Servo_TxFlush();
}

/**
 * @brief  主动发一次"读多个舵机位置"请求
 * @param  num: 舵机个数 1~8
 * @param  ids: 舵机 ID 数组（各 ID 需在 1~31 内）
 * @retval 无
 */
void Servo_ReadPos(uint8_t num, const uint8_t *ids)
{
    if (servoReqPending != 0U)
    {
        return;   /* 单发单收：上一个请求还没结束 */
    }

    Servo_BuildReadPosReq(num, ids);
    if (servoTxLen == 0U)
    {
        return;
    }

    servoReqPending = 1U;
    servoReqTick    = HAL_GetTick();
    servoRetryLeft  = (uint8_t)SERVO_READ_RETRY;
    Servo_TxFlush();
}

/**
 * @brief  卸载（关掉）多个舵机的保持力矩
 * @param  num: 舵机个数 1~8
 * @param  ids: 舵机 ID 数组（各 ID 需在 1~31 内）
 * @retval 无
 * @note   帧（官方二次开发协议 CMD_MULT_SERVO_UNLOAD）：
 *           [0][1] 帧头 0x55 0x55
 *           [2]    Length = N + 3
 *           [3]    Cmd = 0x14
 *           [4]    N（舵机个数）
 *           [5..]  ID1 .. IDN
 *           整帧 = Length + 2 = N + 5
 *         示例（N=2、ID=1/2）：55 55 05 14 02 01 02
 *         卸载后舵机失去保持力矩、可用手自由扳动；要恢复力矩只需再发一条运动指令。
 *         本函数不参与"单发单收"的读请求状态（它不是读请求）。
 */
void Servo_Unload(uint8_t num, const uint8_t *ids)
{
    uint8_t i;
    uint8_t n = 0U;

    servoTxLen = 0U;

    if ((num == 0U) || (ids == NULL) || (num > (uint8_t)SERVO_READ_NUM_MAX))
    {
        return;
    }

    for (i = 0U; i < num; i++)
    {
        uint8_t id = ids[i];

        if ((id >= 1U) && (id <= (uint8_t)SERVO_ID_MAX))
        {
            servoTxBuf[5U + n] = id;
            n++;
        }
    }

    if (n == 0U)
    {
        return;   /* 没有合法 ID，不发 */
    }

    servoTxBuf[0] = SERVO_FRAME_HEAD;
    servoTxBuf[1] = SERVO_FRAME_HEAD;
    servoTxBuf[2] = (uint8_t)(n + 3U);        /* Length = N + 3 */
    servoTxBuf[3] = SERVO_CMD_MULT_UNLOAD;    /* 0x14 */
    servoTxBuf[4] = n;
    servoTxLen    = (uint8_t)(n + 5U);        /* 整帧 = Length + 2 */

    /* 让下一条读请求推迟 SERVO_MOVE_GAP_MS，避免和这条指令挤在一起 */
    Servo_NotifyMotionSent();

    Servo_TxFlush();

    Debug_Log("SRV", "unload n=%u", (unsigned int)n);
}

/**
 * @brief  取某舵机最近一次成功回读的位置
 * @param  servoId: 舵机 ID
 * @param  out_pos: 输出位置，可为 NULL
 * @retval 1=有效；0=无有效回读（含数据超过 SERVO_POS_VALID_MS 过期）
 */
uint8_t Servo_GetPos(uint8_t servoId, uint16_t *out_pos)
{
    uint16_t pos  = 0U;
    uint32_t tick = 0U;

    /* ★【2026-10-27】过期判断用**该 ID 自己**的回读时刻（见 servoPosTickById 说明） */
    if (Servo_CacheGet(servoId, &pos, &tick) == 0U)
    {
        return 0U;
    }
    if ((HAL_GetTick() - tick) > (uint32_t)SERVO_POS_VALID_MS)
    {
        return 0U;   /* 这个 ID 太久没回读到，视为无效，避免把过时值当实际值 */
    }

    if (out_pos != NULL)
    {
        *out_pos = pos;
    }
    return 1U;
}

/**
 * @brief  取最近一次成功回读位置的时刻
 * @param  无
 * @retval HAL_GetTick() 时刻
 * @note   全局时刻（任一 ID 回读成功都会刷新）。判"某条命令之后有没有新回读"
 *         请用 `Servo_GetPosTickById()`，否则别的 ID 的回读会把这里"刷新"成新鲜。
 */
uint32_t Servo_GetPosTick(void)
{
    return servoPosTick;
}

/**
 * @brief  ★【2026-10-27】取**某舵机 ID 自己**最近一次成功回读的时刻
 * @param  servoId: 舵机 ID
 * @retval 该 ID 最后一次回读的 HAL_GetTick()；无有效数据返回 0
 * @note   调用方用 `(int32_t)(Servo_GetPosTickById(id) - 某时刻) >= 0` 判断
 *         "这次回读是不是在这条命令之后拿到的"。用全局 `Servo_GetPosTick()`
 *         会被**别的舵机**的回读刷新成新鲜，导致用旧值提前放行 / 误判没动。
 */
uint32_t Servo_GetPosTickById(uint8_t servoId)
{
    uint32_t tick = 0U;

    if (Servo_CacheGet(servoId, NULL, &tick) == 0U)
    {
        return 0U;
    }
    return tick;
}

/**
 * @brief  取最近一次回读到的调试板电压
 * @param  out_mv: 输出电压 mV，可为 NULL
 * @retval 1=有效；0=还没读到过
 */
uint8_t Servo_GetVin(uint16_t *out_mv)
{
    if (servoVinValid == 0U)
    {
        return 0U;
    }
    if (out_mv != NULL)
    {
        *out_mv = servoVinMv;
    }
    return 1U;
}

/**
 * @brief  取回读统计
 * @param  rxFrames / badFrames / timeouts: 三个输出（均可为 NULL）
 * @retval 无
 */
void Servo_GetStat(uint32_t *rxFrames, uint32_t *badFrames, uint32_t *timeouts)
{
    if (rxFrames != NULL)
    {
        *rxFrames = servoRxFrames;
    }
    if (badFrames != NULL)
    {
        *badFrames = servoBadFrames;
    }
    if (timeouts != NULL)
    {
        *timeouts = servoTimeouts;
    }
}

/**
 * @brief  通知回读模块"刚发过一条舵机运动指令"
 * @param  无
 * @retval 无
 * @note   由 Arm_ServoMove() 调用，用来把下一条读请求推迟 SERVO_MOVE_GAP_MS。
 */
void Servo_NotifyMotionSent(void)
{
    servoLastMotionTick = HAL_GetTick();
}

/* ===================== ★ 总线仲裁（舵机运动指令 vs 位置回读请求） ===================== */

/**
 * @brief  【总线仲裁】总线是否已"静默"（可以安全下发舵机运动指令）
 * @param  quietMs: 距"上一次读请求发出"至少要过的毫秒数
 * @retval 1=静默；0=还有请求在途 / 刚发过读请求
 */
uint8_t Servo_BusQuiet(uint16_t quietMs)
{
    uint32_t now = HAL_GetTick();

    /* ① 本机还在发（HAL 发送状态非 READY）-> 不算静默 */
    if (huart3.gState != HAL_UART_STATE_READY)
    {
        return 0U;
    }
    /* ② 正在收一帧（双帧头之后、帧尾之前）-> 板子还有字节没发完 */
    if (servoRxStep != SERVO_RX_STEP_HEAD1)
    {
        return 0U;
    }
    /* ③ 读请求还没应答完：一问一答，此时插运动指令会被板子丢掉 */
    if (servoReqPending != 0U)
    {
        return 0U;
    }
    /* ④ 距**最后一个收到的字节**必须够久（真正留出静默窗） */
    return (((now - servoLastRxTick) >= (uint32_t)quietMs) ? 1U : 0U);
}

/**
 * @brief  处理"中断已攒好"的应答帧（主循环 / 等总线时用）
 * @param  无
 * @retval 无
 */
void Servo_ServiceRx(void)
{
    if (servoRxReady != 0U)
    {
        servoRxReady = 0U;
        Servo_FrameDispatch();
    }
}

/**
 * @brief  ★等总线真正静默（板子的应答收完），最多等 hardMs
 * @param  quietMs: 静默判定窗（建议 SERVO_QUIET_BEFORE_MOVE_MS）
 * @param  hardMs : 硬上限（建议 SERVO_WAIT_IDLE_HARD_MS）；传 0 按默认值处理
 * @retval 1=已静默；0=超时（调用方应 Yield 后强发）
 * @note   等的时候**顺便解析应答**，否则 servoReqPending 永远清不掉。
 */
uint8_t Servo_WaitBusIdle(uint16_t quietMs, uint16_t hardMs)
{
    uint32_t t0 = HAL_GetTick();
    uint32_t now;

    if (hardMs == 0U)
    {
        hardMs = (uint16_t)SERVO_WAIT_IDLE_HARD_MS;
    }

    for (;;)
    {
        now = HAL_GetTick();

        /* ① 解析已攒好的应答帧（否则 servoReqPending 永远清不掉） */
        Servo_ServiceRx();

        /* ② ★【2026-10-27 修"偶发丢动作"】**半帧超时复位**（与 Servo_Update 同一判据）。
              等待期间不跑这一步的话：RX 状态机一旦卡在 HEAD2/LEN/BODY（丢过字节），
              `Servo_BusQuiet()` 的第②条 `servoRxStep == HEAD1` **永远为假** ->
              必然耗满硬上限(80ms) -> 调用方 Yield 后强发 -> 运动指令正好撞上
              板子的迟到应答 -> **被丢**。现场表现就是"这个动作有时执行有时不执行"。 */
        if ((servoRxStep != SERVO_RX_STEP_HEAD1) &&
            ((now - servoLastRxTick) >= (uint32_t)SERVO_RX_FRAME_GAP_MS))
        {
            servoRxStep = SERVO_RX_STEP_HEAD1;
            servoRxIdx  = 0U;
            servoBadFrames++;
        }

        /* ③ ★【2026-10-27】**在途读请求超时就地放弃**（不重发）。
              读请求的重试链最长 `SERVO_READ_TIMEOUT_MS x (SERVO_READ_RETRY+1)` = 150ms，
              比本函数的硬上限(80ms)长。若这里不放弃，`servoReqPending` 会把总线
              **永远**判成"忙"，每次下发运动指令都走"Yield 强发"那条路 -> 撞应答 -> 丢。 */
        if ((servoReqPending != 0U) &&
            ((now - servoReqTick) >= (uint32_t)SERVO_READ_TIMEOUT_MS))
        {
            servoReqPending = 0U;
            servoTimeouts++;
            servoBusYielded++;
        }

        /* ④ 判静默 */
        if (Servo_BusQuiet(quietMs) != 0U)
        {
            return 1U;
        }

        if ((now - t0) >= (uint32_t)hardMs)
        {
            return 0U;
        }
    }
}

/**
 * @brief  【总线仲裁】当前是否还有读请求在途（调试板还没答完）
 * @param  无
 * @retval 1=在途
 */
uint8_t Servo_ReqPending(void)
{
    return servoReqPending;
}

/**
 * @brief  【总线仲裁】让出总线给舵机运动指令（幂等）
 * @param  无
 * @retval 无
 * @note   放弃在途读请求（**不丢数据**：它的应答到了照样按 0x15 解析、写位置缓存，
 *         只是不再走"超时重试"那条路），并把读节拍 / 运动间隔从此刻重算。
 *         反复调用只有第一次有作用 —— 否则"静默计时"会被一直推后、永远发不出指令。
 */
void Servo_BusYieldForMotion(void)
{
    if (servoReqPending != 0U)
    {
        servoReqPending = 0U;
        servoReqTick    = HAL_GetTick();   /* 静默计时从"这一刻"开始重算 */
        servoBusYielded++;
    }

    servoReqPeriodTick  = HAL_GetTick();   /* 读节拍重排：指令发完再读 */
    servoLastMotionTick = HAL_GetTick();
}
