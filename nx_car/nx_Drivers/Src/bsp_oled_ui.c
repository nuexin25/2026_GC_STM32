/**
 ******************************************************************************
 * @file    bsp_oled_ui.c
 * @brief   OLED 显示应用层实现：I2C 整页写入、页面框架、刷新耗时统计
 * @author  nx_car
 * @date    2026-09-05
 ******************************************************************************
 * @note   渲染优化说明：
 *   oled.c 的 WriteDat 逐字节调用 HAL_I2C_Mem_Write，每字节约 0.3ms，
 *   全屏 1024 字节约 300ms，远慢于 HAL_I2C_Mem_Write 一次发一页（写页），
 *   整页 128 字节，加上控制字节 0x40 + 数据，单页刷新约 1.5ms。
 *   切页时先 OLED_ClearAll 批量清屏再重绘，避免旧页残留。
 ******************************************************************************
 */
#include "bsp_oled_ui.h"
#include "oled.h"
#include "i2c.h"
#include "bsp_debug.h"
#include "bsp_test.h"
#include "bsp_wheel_test.h"
#include "bsp_chassis_test.h"  /* P4 车机测试页：动作名/状态/进度 */
#include "bsp_qr_test.h"       /* P5 扫码测试页：状态/原始码/第一批颜色缩写 */
#include "bsp_rpi_test.h"      /* P6 树莓派通讯测试页：在线状态/颜色/三路偏移/指令 */
#include "bsp_route_test.h"    /* P7 跑图测试页：路段进度/状态/当前段目标 */
#include "bsp_arm_test.h"      /* P8 机械臂测试页：执行器名/状态/按键提示 */
#include "bsp_arm_seq.h"       /* P8 联调 手臂调试页：回原点 + 动作序列（主菜单进入） */
#include "bsp_arm_para.h"      /* P8 联调 手臂参数页：读电机/舵机参数（主菜单进入） */
#include "bsp_mission.h"       /* P9 赛题页：旧点位流程（导航内核 Mission_Nav* 仍被 bsp_task 复用） */
#include "bsp_task.h"          /* P9 赛题页：流程进度显示（Task_Get*Text） */
#include "bsp_task_dbg.h"      /* P9 任务调试页：任务点单点调试（TaskDbg_*） */
#include "bsp_align_test.h"    /* P24 对位调参页：单独跑对位 + 逐帧调参日志（AlnTest_*） */
#include "bsp_vel_test.h"      /* P25 速度模式测试页：纯陀螺仪 + 时间控距离（VelTest_*） */
#include "bsp_vel_route.h"     /* P26 速度模式跑图页：赛题路段表全程 0xF6（VelRoute_*） */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* 字模声明（定义于 oledfont.h，oled.c 已包含；此处声明以避免重复定义） */
extern const unsigned char F6x8[][6];
extern const unsigned char F8X16[];

/* ===================== 内部状态变量 ===================== */
static uint8_t  oledOnline = 0;              /* OLED 在线标志：0=离线，1=在线 */
static PageID_t currentPage = PAGE_BOOT;     /* 当前显示页 ID */
static uint8_t  pageDirty = 1;               /* 页面脏标志：1=需全量重绘，0=仅动态行 */
static uint32_t lastRefreshTick = 0;         /* 上次刷新时间戳 */
static uint32_t lastCycleTick = 0;           /* 上次切页时间戳 */
static uint8_t  refreshFrameCount = 0;       /* 刷新帧计数，每 10 帧上报一次耗时 */

/* 数据页动态值 */
static int16_t  yawX100 = 0x7FFF;            /* 航向角放大 100 的值，0x7FFF 表示无效 */
static int16_t  posXmm = 0x7FFF;             /* X 坐标 mm，0x7FFF 表示无效 */
static int16_t  posYmm = 0x7FFF;             /* Y 坐标 mm，0x7FFF 表示无效 */

/* 任务码页数据 */
static char taskCodeBuf[16] = "---+---+---+---";

/* 统计页数据 */
static uint16_t statPickOk = 0;
static uint16_t statPickTotal = 0;
static uint16_t statPlaceOk = 0;
static uint16_t statPlaceTotal = 0;

/* 整页渲染缓冲（每页 128 字节） */

/* 页面选择菜单状态 */
static uint8_t menuCursor = 0;                 /* 菜单光标（选中项下标，返回菜单时保留） */
static uint8_t menuTop = 0;                    /* 菜单滚动窗口的起始项下标 */
static uint8_t menuReturnPage = PAGE_DATA;     /* 菜单返回页 */
static const struct {
    const char *name;
    PageID_t page;
} menuItems[] = {
    { "DATA", PAGE_DATA },
    { "TASK", PAGE_TASK },
    { "STAT", PAGE_STAT },
    { "TEST", PAGE_TEST },
    { "ARMSEQ", PAGE_ARM_SEQ },      /* P8 联调：手臂调试（回原点 + 动作序列） */
    { "ARMPARA", PAGE_ARM_PARA },    /* P8 联调：手臂参数（读电机/舵机参数） */
    { "MISSION", PAGE_MISSION },     /* P9 步骤1：赛题单步跑点位 */
    { "TDBG", PAGE_TASK_DBG },       /* P9：任务点调试（单点验证每个任务点） */
    { "ALNT", PAGE_ALIGN_TEST },     /* P24：对位调参（单独跑对位 + 逐帧 PID 数据） */
    { "VELTEST", PAGE_VELTEST },     /* P25：速度模式测试（纯陀螺仪 + 时间控距离） */
    { "VELROUTE", PAGE_VELROUTE },   /* P26：速度模式跑图（赛题路段表全程 0xF6） */
};
#define MENU_ITEM_COUNT  ((uint8_t)(sizeof(menuItems) / sizeof(menuItems[0])))

/* 菜单一屏可见项数：8x16 大字体每项占 2 个页（16 像素高），128x64 屏共 4 项。
   菜单项多于本值时靠 menuTop 组成的滚动窗口显示，光标始终保持在窗口内。 */
#define MENU_VISIBLE_ROWS  ((uint8_t)4)

/**
 * @brief  确保菜单光标落在可见窗口内（滚动窗口校正）
 * @param  无
 * @retval 无
 * @note   光标上移越过窗口顶 -> 窗口上移一项；光标下移越过窗口底 -> 窗口下移一项。
 *         菜单项超过 4 项后**必须**调用，否则光标所在项会画到屏幕外。
 */
static void MenuEnsureVisible(void)
{
    if (menuCursor < menuTop)
    {
        menuTop = menuCursor;
    }
    else if (menuCursor >= (uint8_t)(menuTop + MENU_VISIBLE_ROWS))
    {
        menuTop = (uint8_t)(menuCursor - MENU_VISIBLE_ROWS + 1U);
    }
    else
    {
        /* 已在窗口内，不用动 */
    }
}

/* ===================== 底层 I2C 整页写入 ===================== */

/**
 * @brief  设置页地址与列地址（起始）
 * @param  page: 页 0~7
 * @param  col:  起始列 0~127
 * @retval 无
 * @note   一次发送 3 字节命令（页地址、列高 4 位、列低 4 位）
 */
static void OLED_SetPosFast(uint8_t page, uint8_t col)
{
    uint8_t cmd[3];
    cmd[0] = 0xB0 | (page & 0x07);
    cmd[1] = 0x10 | ((col >> 4) & 0x0F);
    cmd[2] = 0x00 | (col & 0x0F);
    HAL_I2C_Mem_Write(&hi2c1, OLED_I2C_ADDR, 0x00,
                      I2C_MEMADD_SIZE_8BIT, cmd, 3, OLED_I2C_TIMEOUT);
}

/**
 * @brief  写入一整页数据（128 字节，从第 0 列开始）
 * @param  page: 页 0~7
 * @param  buf:  128 字节数据缓冲区
 * @retval 无
 */
static void OLED_WritePage(uint8_t page, const uint8_t *buf)
{
    OLED_SetPosFast(page, 0);
    HAL_I2C_Mem_Write(&hi2c1, OLED_I2C_ADDR, 0x40,
                      I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf,
                      OLED_PAGE_BUF_SIZE, OLED_I2C_TIMEOUT);
}

/**
 * @brief  清屏（批量写 8 页全 0x00，比 OLED_CLS 逐字节快约 25 倍）
 * @param  无
 * @retval 无
 */
static void OLED_ClearAll(void)
{
    uint8_t buf[OLED_PAGE_BUF_SIZE];
    uint8_t p;
    memset(buf, 0x00, sizeof(buf));
    for (p = 0; p < 8U; p++)
    {
        OLED_WritePage(p, buf);
    }
}

/* ===================== 字模渲染 ===================== */


/**
 * @brief  将 8x16 ASCII 字符串渲染进上下两个页缓冲区
 * @param  bufTop: 上半页（128 字节）
 * @param  bufBot: 下半页（128 字节）
 * @param  str: 以 '\0' 结尾的字符串
 * @retval 无
 */
static void OLED_Render8x16(uint8_t *bufTop, uint8_t *bufBot, const char *str)
{
    memset(bufTop, 0, OLED_PAGE_BUF_SIZE);
    memset(bufBot, 0, OLED_PAGE_BUF_SIZE);
    uint8_t col = 0;
    while (*str != '\0' && col <= (OLED_PAGE_BUF_SIZE - 8U))
    {
        uint8_t c = (uint8_t)(*str - 32);
        if (c > 94U) { c = 0; }
        for (uint8_t i = 0; i < 8U; i++)
        {
            bufTop[col + i] = F8X16[c * 16 + i];
            bufBot[col + i] = F8X16[c * 16 + i + 8];
        }
        col += 8U;
        str++;
    }
}


static void OLED_ShowLine8x16(uint8_t page, const char *str)
{
    uint8_t bufTop[OLED_PAGE_BUF_SIZE];
    uint8_t bufBot[OLED_PAGE_BUF_SIZE];
    if (!oledOnline) return;
    OLED_Render8x16(bufTop, bufBot, str);
    OLED_WritePage(page, bufTop);
    OLED_WritePage(page + 1U, bufBot);
}

/* ===================== 页面绘制 ===================== */

/**
 * @brief  绘制开机页
 * @param  无
 * @retval 无
 */
static void DrawPageBoot(void)
{
    OLED_ShowLine8x16(0, "NX-CAR 2026");
    OLED_ShowLine8x16(2, __DATE__);
    OLED_ShowLine8x16(4, "GC Smart Car");
}

/**
 * @brief  绘制数据页静态行（yaw、pos，全量绘制）
 * @param  无
 * @retval 无
 */
static void DrawPageDataFull(void)
{
    char line[24];

    if (yawX100 == 0x7FFF)
    {
        OLED_ShowLine8x16(2, "yaw=---");
    }
    else
    {
        int16_t deg = yawX100 / 100;
        int16_t frac = yawX100 % 100;
        if (frac < 0) frac = -frac;
        snprintf(line, sizeof(line), "yaw=%d.%02d", deg, (int)frac);
        OLED_ShowLine8x16(2, line);
    }

    if (posXmm == 0x7FFF || posYmm == 0x7FFF)
    {
        OLED_ShowLine8x16(4, "pos=---");
    }
    else
    {
        snprintf(line, sizeof(line), "pos=%d,%d", posXmm, posYmm);
        OLED_ShowLine8x16(4, line);
    }
}

/**
 * @brief  绘制数据页动态行（t=）
 * @param  无
 * @retval 无
 */
static void DrawPageDataDynamic(void)
{
    char line[24];
    uint32_t sec = HAL_GetTick() / 1000U;

    snprintf(line, sizeof(line), "t=%lus", (unsigned long)sec);
    OLED_ShowLine8x16(0, line);
}

/**
 * @brief  绘制任务码页
 * @param  无
 * @retval 无
 */
static void DrawPageTask(void)
{
    OLED_ShowLine8x16(0, "TASK CODE");
    OLED_ShowLine8x16(2, taskCodeBuf);
}

/**
 * @brief  绘制统计页
 * @param  无
 * @retval 无
 */
static void DrawPageStat(void)
{
    char line[24];
    snprintf(line, sizeof(line), "PICK:%u/%u", statPickOk, statPickTotal);
    OLED_ShowLine8x16(0, line);
    snprintf(line, sizeof(line), "PLACE:%u/%u", statPlaceOk, statPlaceTotal);
    OLED_ShowLine8x16(2, line);
}

/**
 * @brief  绘制模块测试页
 * @param  无
 * @retval 无
 */
static void DrawPageTest(void)
{
    uint8_t i;
    uint8_t scroll = Test_GetScroll();
    uint8_t cursor = Test_GetCursor();
    char line[24];

    for (i = 0; i < TEST_VISIBLE_ROWS; i++)
    {
        uint8_t idx = scroll + i;
        if (idx >= TEST_ID_MAX) break;

        char mark = (idx == cursor) ? '>' : ' ';
        char stateCh;
        switch (testItems[idx].state)
        {
            case TEST_STATE_IDLE: stateCh = '.'; break;
            case TEST_STATE_RUN:  stateCh = '*'; break;
            case TEST_STATE_PASS: stateCh = 'P'; break;
            case TEST_STATE_FAIL: stateCh = 'F'; break;
            case TEST_STATE_NA:   stateCh = 'x'; break;
            default:              stateCh = '?'; break;
        }
        snprintf(line, sizeof(line), "%c%s %c", mark, testItems[idx].name, stateCh);
        OLED_ShowLine8x16((uint8_t)(i * 2U), line);
    }
}

/**
 * @brief  绘制单轮点动测试页（P3，8x16 大字体四行）
 * @param  无
 * @retval 无
 * @note   第1行标题 WHEEL TEST；第2行当前电机 编号+位置缩写(FL/FR/RL/RR)；
 *         第3行当前方向 FWD/REV/STOP；第4行点动速度 RPM 值。
 *         数据取自 bsp_wheel_test 模块，页面仅在脏标志置位时重绘。
 */
static void DrawPageWheel(void)
{
    char line[24];
    uint8_t mot = WheelTest_GetMotor();
    WheelDir_t dir = WheelTest_GetDirState();
    const char *dirStr;

    switch (dir)
    {
        case WHEEL_DIR_FWD: dirStr = "FWD";  break;
        case WHEEL_DIR_REV: dirStr = "REV";  break;
        case WHEEL_DIR_STOP:
        default:            dirStr = "STOP"; break;
    }

    OLED_ShowLine8x16(0, "WHEEL TEST");
    snprintf(line, sizeof(line), "MOT:%u %s", (unsigned int)mot, WheelTest_MotorName(mot));
    OLED_ShowLine8x16(2, line);
    snprintf(line, sizeof(line), "DIR:%s", dirStr);
    OLED_ShowLine8x16(4, line);
    snprintf(line, sizeof(line), "RPM:%u", (unsigned int)WheelTest_GetRpm());
    OLED_ShowLine8x16(6, line);
}

/**
 * @brief  绘制车机测试页（P4，8x16 大字体四行）
 * @param  无
 * @retval 无
 * @note   第1行标题 CHASSIS TEST；
 *         第2行动作+状态 "A:FWD S:IDLE"（用短标签 A:/S: 才能在同行的
 *           16 字符里放下 RIGHT / ABORT 这类长名字）；
 *         第3行相对坐标 "X=已走mm Y=已走mm"（原点=进入本页时的位置，
 *           +x=当时车头方向，+y=当时车身右侧）；
 *         第4行 "H=朝向角 D=本次动作已走mm"（禁 %f，小数位用整数拼）。
 *         数据取自 bsp_chassis_test / bsp_chassis 查询接口，
 *         仅在脏标志置位时重绘（运动中由里程计每拍置脏）。
 */
static void DrawPageChassis(void)
{
    char  line[24];
    float hd = Chassis_GetOdomYaw();
    int   xi = (int)Chassis_GetOdomX();
    int   yi = (int)Chassis_GetOdomY();
    int   hi = (int)hd;
    int   hf;

    /* 本行只有 16 字符：坐标超 ±9999mm 就截断显示，避免溢出串行 */
    if (xi >  9999) { xi =  9999; }
    if (xi < -9999) { xi = -9999; }
    if (yi >  9999) { yi =  9999; }
    if (yi < -9999) { yi = -9999; }

    OLED_ShowLine8x16(0, "CHASSIS TEST");

    /* 第2行：动作 + 状态（用短标签 A:/S:，长名如 RIGHT/ABORT 也能放下） */
    snprintf(line, sizeof(line), "A:%s S:%s",
             Chassis_GetActName(ChassisTest_GetAct()), Chassis_GetStateText());
    OLED_ShowLine8x16(2, line);

    /* 第3行：相对坐标（原点=进入本页时的位置，+x=当时车头方向，+y=当时右侧） */
    snprintf(line, sizeof(line), "X=%d Y=%d", xi, yi);
    OLED_ShowLine8x16(4, line);

    /* 第4行：朝向角（1 位小数，禁 %f 用整数拼）+ 本次动作已走距离 */
    hf = (int)(hd * 10.0f) % 10;
    if (hf < 0)
    {
        hf = -hf;
    }
    snprintf(line, sizeof(line), "H=%d.%d D=%d", hi, hf, (int)Chassis_GetDistMm());
    OLED_ShowLine8x16(6, line);
}

/**
 * @brief  绘制扫码测试页（P5，8x16 大字体四行）
 * @param  无
 * @retval 无
 * @note   第1行标题 QR SCAN TEST；第2行状态 WAIT/OK/BAD/OFFLINE；
 *         第3行原始码（有则显示，无则 "---"）；第4行第一批颜色顺序缩写。
 *         数据取自 bsp_qr_test 查询接口，仅在脏标志置位时重绘。
 */
static void DrawPageQR(void)
{
    OLED_ShowLine8x16(0, "QR SCAN TEST");
    OLED_ShowLine8x16(2, QRTest_GetStatusText());
    OLED_ShowLine8x16(4, QRTest_GetRawText());
    OLED_ShowLine8x16(6, QRTest_GetBatch1Text());
}

/**
 * @brief  绘制树莓派通讯测试页（P6，8x16 大字体四行）
 * @param  无
 * @retval 无
 * @note   第1行 标题+在线状态（`RPI TEST ONLINE` / `RPI TEST OFFLINE`）；
 *         第2行 物料颜色 + 转盘 1 号位偏移（`C:1 TT:120,-30`）；
 *         第3行 粗加工/精加工中心靶位偏移（`G:-60,25 F:10,-5`，标签各 1 字符省宽度）；
 *         第4行 当前选中指令 + 按键提示（`K3:COLOR_START`）。
 *         数据过期时显示 `--,--`（与"数据就是 0"区分开）。
 *         每行仅 16 字符，偏移量都到 3 位数时第3行会被截断（完整值看 UART5 日志）。
 *         数据取自 bsp_rpi_test 查询接口，仅在脏标志置位时重绘。
 */
static void DrawPageRPi(void)
{
    OLED_ShowLine8x16(0, RPiTest_GetTitleText());
    OLED_ShowLine8x16(2, RPiTest_GetColorText());
    OLED_ShowLine8x16(4, RPiTest_GetTargetText());
    OLED_ShowLine8x16(6, RPiTest_GetCmdText());
}

/**
 * @brief  绘制跑图测试页（P7，8x16 大字体四行）
 * @param  无
 * @retval 无
 * @note   第1行标题 ROUTE TEST；第2行路段进度 SEG 2/4；
 *         第3行状态+阶段 RUN MOVE；第4行当前段目标 to(500,500)。
 *         数据取自 bsp_route_test 查询接口，仅在脏标志置位时重绘。
 */
static void DrawPageRoute(void)
{
    OLED_ShowLine8x16(0, "ROUTE TEST");
    OLED_ShowLine8x16(2, RouteTest_GetSegText());
    OLED_ShowLine8x16(4, RouteTest_GetStatusText());
    OLED_ShowLine8x16(6, RouteTest_GetTargetText());
}

/**
 * @brief  绘制速度模式测试页（P25，8x16 大字体四行）
 * @param  无
 * @retval 无
 * @note   第1行 标题+状态（如 "VELTEST RUN1" / "VELTEST CORNER"）；
 *         第2行 参数 V / T1（前面 `>` 表示当前选中项）；
 *         第3行 参数 T2 / A；
 *         第4行 参数 G / VC（运行中改为显示实时航向 "Y=.."）。
 *         参数与状态都取自 bsp_vel_test 的查询接口，本页无编码器数据。
 */
static void DrawPageVelTest(void)
{
    OLED_ShowLine8x16(0, VelTest_GetLine(0));
    OLED_ShowLine8x16(2, VelTest_GetLine(1));
    OLED_ShowLine8x16(4, VelTest_GetLine(2));
    OLED_ShowLine8x16(6, VelTest_GetLine(3));
}

/**
 * @brief  绘制速度模式跑图页（P26，8x16 大字体四行）
 * @param  无
 * @retval 无
 * @note   第1行 标题+状态+模式（"VELRT RUN A" = AUTO 连跑 / "S" = STEP 单步）；
 *         第2行 步号/总数 + 步名；第3行 本段目标（TGT x,y 或 TGT ROT deg）；
 *         第4行 当前里程计坐标与朝向（N x,y deg）。
 */
static void DrawPageVelRoute(void)
{
    OLED_ShowLine8x16(0, VelRoute_GetLine(0));
    OLED_ShowLine8x16(2, VelRoute_GetLine(1));
    OLED_ShowLine8x16(4, VelRoute_GetLine(2));
    OLED_ShowLine8x16(6, VelRoute_GetLine(3));
}

/**
 * @brief  绘制机械臂测试页（P8，8x16 大字体四行）
 * @param  无
 * @retval 无
 * @note   第1行标题 ARM TEST；第2行当前执行器 TURN 1/4 5=EXIT；
 *         第3行状态（电机 p=位置 H=回零 R=到位 / 舵机 pos=目标）；
 *         第4行按键提示 K3=.. K4=.. K5LH。
 *         数据取自 bsp_arm_test 查询接口，仅在脏标志置位时重绘。
 */
static void DrawPageArm(void)
{
    OLED_ShowLine8x16(0, "ARM TEST");
    OLED_ShowLine8x16(2, ArmTest_GetItemText());
    OLED_ShowLine8x16(4, ArmTest_GetStateText());
    OLED_ShowLine8x16(6, ArmTest_GetHintText());
}

/**
 * @brief  绘制手臂调试页（P8 联调，8x16 大字体四行）
 * @param  无
 * @retval 无
 * @note   第1行标题 ARM SEQ；第2行状态+动作名+序号（INIT n/4 / READY xxx a/b /
 *         RUN xxx n/m / DONE / ABORT）；第3行当前关节与参数（未运行时显示
 *         选中动作的步数）；第4行按键提示。
 */
static void DrawPageArmSeq(void)
{
    OLED_ShowLine8x16(0, "ARM SEQ");
    OLED_ShowLine8x16(2, ArmSeq_GetStateText());
    OLED_ShowLine8x16(4, ArmSeq_GetStepText());
    OLED_ShowLine8x16(6, ArmSeq_GetHintText());
}

/**
 * @brief  绘制手臂参数页（P8 联调，8x16 大字体四行）
 * @param  无
 * @retval 无
 * @note   第1行标题 `ARM PARA S3=xxx`（★2026-10-21：S3 = 车载转盘实际位置；
 *         长按 KEY3 仍 = 把当前位置设为电机5 的 0 点，见手册/日志）；
 *         第2/3行电机5/6 当前位置（读不到显示 ----）；
 *         第4行 S1/S2 的**真实回读位置**（由 bsp_servo 周期读回，读不到显示 --）。
 */
static void DrawPageArmPara(void)
{
    OLED_ShowLine8x16(0, ArmPara_GetTitleText());  /* 标题 + S3（车载转盘）实际位置 */
    OLED_ShowLine8x16(2, ArmPara_GetMotor5Text());
    OLED_ShowLine8x16(4, ArmPara_GetMotor6Text());
    OLED_ShowLine8x16(6, ArmPara_GetServoText());
}

/**
 * @brief  绘制赛题页（P9 流程页，8x16 大字体四行）
 * @param  无
 * @retval 无
 * @note   第1行 步骤序号/总数 + 步骤名      12/42 B1_RGH
 *         第2行 本步目标坐标与朝向（T=目标） T1200,300h90
 *         第3行 实测坐标与朝向（A=实际）     A1180,295h89
 *         第4行 状态 + 模式 + 按键提示       IDLE STEP K5=GO
 *         数据取自 bsp_task 查询接口，仅在脏标志置位时重绘。
 */
static void DrawPageMission(void)
{
    OLED_ShowLine8x16(0, Task_GetTitleText());
    OLED_ShowLine8x16(2, Task_GetTargetText());
    OLED_ShowLine8x16(4, Task_GetOdomText());
    OLED_ShowLine8x16(6, Task_GetStateText());
}

/**
 * @brief  绘制任务调试页（P9，8x16 大字体四行）
 * @param  无
 * @retval 无
 * @note   第1行 当前项序号/总数        TASK DBG 2/6
 *         第2行 当前任务点名            TRAY PICK
 *         第3行 状态 + 阶段说明          RUN ccg Q2 / IDLE / DONE / FAIL ...
 *         第4行 关键数据                RZ 1 3 4 / c1 P2 Q3 / c1->S3
 *         数据取自 bsp_task_dbg 查询接口，仅在脏标志置位时重绘。
 */
static void DrawPageTaskDbg(void)
{
    OLED_ShowLine8x16(0, TaskDbg_GetTitleText());
    OLED_ShowLine8x16(2, TaskDbg_GetNameText());
    OLED_ShowLine8x16(4, TaskDbg_GetStateText());
    OLED_ShowLine8x16(6, TaskDbg_GetDataText());
}

/**
 * @brief  绘制对位调参页（P24，8x16 大字体四行）
 * @param  无
 * @retval 无
 * @note   第1行 标题 + 数据源        ALN TEST 2/3 SHOT
 *         第2行 图像偏移 + 平行度     px=-12,34 q=-8
 *         第3行 三轴 PID 输出         u=18.2,-4.1,0.0 w=22
 *         第4行 状态 + 帧数 + 用时    RUN f=12 1830ms K3=STOP
 *         数据取自 bsp_align_test 的 AlnTest_Get*Text()，仅在脏标志置位时重绘。
 */
static void DrawPageAlignTest(void)
{
    OLED_ShowLine8x16(0, AlnTest_GetTitleText());
    OLED_ShowLine8x16(2, AlnTest_GetErrText());
    OLED_ShowLine8x16(4, AlnTest_GetOutText());
    OLED_ShowLine8x16(6, AlnTest_GetStateText());
}

/**
 * @brief  绘制页面选择菜单（4 行滚动窗口）
 * @param  无
 * @retval 无
 * @note   菜单项可能多于 4 项，所以只画 [menuTop, menuTop+3] 这一段窗口；
 *         窗口外的项不画。不足 4 项时多余行画空格，避免残留旧字符。
 */
static void DrawPageMenu(void)
{
    uint8_t row;
    char line[24];

    for (row = 0U; row < MENU_VISIBLE_ROWS; row++)
    {
        uint8_t idx = (uint8_t)(menuTop + row);

        if (idx >= MENU_ITEM_COUNT)
        {
            OLED_ShowLine8x16((uint8_t)(row * 2U), " ");   /* 空格清行，防残留 */
            continue;
        }

        char mark = (idx == menuCursor) ? '>' : ' ';
        snprintf(line, sizeof(line), "%c%u.%s", mark, (unsigned)(idx + 1U), menuItems[idx].name);
        OLED_ShowLine8x16((uint8_t)(row * 2U), line);
    }
}

/* ===================== 公开接口 ===================== */

/**
 * @brief  OLED 显示系统初始化
 * @param  无
 * @retval 无
 * @note   探测在线，在线则 OLED_Init 并清屏，离线则置离线标志
 */
void Display_Init(void)
{
    HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(&hi2c1, OLED_I2C_ADDR, 3, 100);
    if (st != HAL_OK)
    {
        oledOnline = 0;
        Debug_Log("OLED", "not found");
        return;
    }

    oledOnline = 1;
    OLED_Init();
    OLED_CLS();
    Debug_Log("OLED", "init ok");

    currentPage = PAGE_BOOT;
    pageDirty = 1;
    lastCycleTick = HAL_GetTick();
    lastRefreshTick = HAL_GetTick();
}

/**
 * @brief  切换到指定页面
 * @param  page: 页面 ID（PageID_t）
 * @retval 无
 * @note   切页时先批量清屏再重绘，避免旧页残留
 */
void Display_SetPage(uint8_t page)
{
    if (!oledOnline) return;
    if (page >= PAGE_MAX) page = PAGE_BOOT;
    OLED_ClearAll();
    currentPage = (PageID_t)page;
    pageDirty = 1;
    lastCycleTick = HAL_GetTick();
}


/**
 * @brief  显示任务主循环节拍（非阻塞）
 * @param  无
 * @retval 无
 * @note   开机页 BOOT_DISPLAY_MS 后自动进入页面选择菜单；按 REFRESH_PERIOD_MS 刷新
 */
void Display_Task(void)
{
    if (!oledOnline) return;

    uint32_t now = HAL_GetTick();

    /* 开机页显示 BOOT_DISPLAY_MS 后自动进入页面选择菜单 */
    if (currentPage == PAGE_BOOT && (now - lastCycleTick) >= BOOT_DISPLAY_MS)
    {
        Display_MenuEnter();
    }

    /* 按 REFRESH_PERIOD_MS 刷新 */
    if ((now - lastRefreshTick) < REFRESH_PERIOD_MS) return;
    lastRefreshTick = now;

    uint32_t t0 = HAL_GetTick();

    if (pageDirty)
    {
        switch (currentPage)
        {
            case PAGE_BOOT:
                DrawPageBoot();
                break;
            case PAGE_DATA:
                DrawPageDataFull();
                DrawPageDataDynamic();
                break;
            case PAGE_TASK:
                DrawPageTask();
                break;
            case PAGE_STAT:
                DrawPageStat();
                break;
            case PAGE_TEST:
                DrawPageTest();
                break;
            case PAGE_WHEEL:
                DrawPageWheel();
                break;
            case PAGE_CHASSIS:
                DrawPageChassis();
                break;
            case PAGE_QR:
                DrawPageQR();
                break;
            case PAGE_RPI:
                DrawPageRPi();
                break;
            case PAGE_ROUTE:
                DrawPageRoute();
                break;
            case PAGE_ARM:
                DrawPageArm();
                break;
            case PAGE_ARM_SEQ:
                DrawPageArmSeq();
                break;
            case PAGE_ARM_PARA:
                DrawPageArmPara();
                break;
            case PAGE_MISSION:
                DrawPageMission();
                break;
            case PAGE_TASK_DBG:
                DrawPageTaskDbg();
                break;
            case PAGE_ALIGN_TEST:
                DrawPageAlignTest();
                break;
            case PAGE_VELTEST:
                DrawPageVelTest();
                break;
            case PAGE_VELROUTE:
                DrawPageVelRoute();
                break;
            case PAGE_MENU:
                DrawPageMenu();
                break;
            default:
                break;
        }
        pageDirty = 0;
    }
    else
    {
        if (currentPage == PAGE_DATA)
        {
            DrawPageDataDynamic();
        }
        else if (currentPage == PAGE_TEST)
        {
            DrawPageTest();
        }

    }

    uint32_t elapsed = HAL_GetTick() - t0;

    refreshFrameCount++;
    if (refreshFrameCount >= 10U)
    {
        refreshFrameCount = 0;
        Debug_Log("OLED", "refresh=%lu ms", (unsigned long)elapsed);
    }
}

/**
 * @brief  更新任务码显示内容
 * @param  code: 任务码字符串（如 "134+123+314+231"），NULL 或 "" 表示占位
 * @retval 无
 */
void Display_SetTaskCode(const char *code)
{
    if (code == NULL || code[0] == '\0')
    {
        strncpy(taskCodeBuf, "---+---+---+---", sizeof(taskCodeBuf) - 1);
    }
    else
    {
        strncpy(taskCodeBuf, code, sizeof(taskCodeBuf) - 1);
    }
    taskCodeBuf[sizeof(taskCodeBuf) - 1] = '\0';
    pageDirty = 1;
}

/**
 * @brief  更新抓取/码放统计
 * @param  pick_ok: 抓取正确次数
 * @param  pick_total: 抓取总次数
 * @param  place_ok: 码放正确次数
 * @param  place_total: 码放总次数
 * @retval 无
 */
void Display_SetStat(uint16_t pick_ok, uint16_t pick_total,
                     uint16_t place_ok, uint16_t place_total)
{
    statPickOk = pick_ok;
    statPickTotal = pick_total;
    statPlaceOk = place_ok;
    statPlaceTotal = place_total;
    pageDirty = 1;
}

/**
 * @brief  更新 yaw 值（数据页预留）
 * @param  yaw_x100: 航向角放大 100 的整数值（如 -1234 表示 -12.34°），0x7FFF 表示无效
 * @retval 无
 */
void Display_SetYaw(int16_t yaw_x100)
{
    yawX100 = yaw_x100;
    pageDirty = 1;
}

/**
 * @brief  更新位置值（数据页预留）
 * @param  pos_x_mm: X 坐标 mm
 * @param  pos_y_mm: Y 坐标 mm
 * @retval 无
 */
void Display_SetPos(int16_t pos_x_mm, int16_t pos_y_mm)
{
    posXmm = pos_x_mm;
    posYmm = pos_y_mm;
    pageDirty = 1;
}

/**
 * @brief  刷新单轮点动测试页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_WHEEL 页时置脏；单轮页状态只在按键后变化，
 *         不做周期性全量重绘（避免阻塞主循环导致按键响应延迟）。
 */
void Display_WheelTestRefresh(void)
{
    if (currentPage == PAGE_WHEEL)
    {
        pageDirty = 1;
    }
}

/**
 * @brief  刷新车机测试页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_CHASSIS 页时置脏；车机模块运行中每拍调用刷新
 *         进度行，按键切换动作/启停时也调用。禁止周期高频全量重绘。
 */
void Display_ChassisRefresh(void)
{
    if (currentPage == PAGE_CHASSIS)
    {
        pageDirty = 1;
    }
}

/**
 * @brief  刷新扫码测试页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_QR 页时置脏；新帧成帧后由 QR 模块调用，
 *         KEY3 手动重扫也调用。扫码页状态变化由新帧驱动，禁止周期高频重绘。
 */
void Display_QRRefresh(void)
{
    if (currentPage == PAGE_QR)
    {
        pageDirty = 1;
    }
}

/**
 * @brief  刷新树莓派通讯测试页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_RPI 页时置脏；收到新帧或数据过期时由 RPi 模块调用，
 *         本页切指令/发送指令也调用。数据变化由新帧驱动，禁止周期高频重绘。
 */
void Display_RPiRefresh(void)
{
    if (currentPage == PAGE_RPI)
    {
        pageDirty = 1;
    }
}

/**
 * @brief  刷新跑图测试页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_ROUTE 页时置脏；跑图执行器在路段切换/状态变化时调用，
 *         按键启动/中止也调用。实际重绘受 REFRESH_PERIOD_MS 节拍限制，
 *         不会高频全量重绘。
 */
void Display_RouteRefresh(void)
{
    if (currentPage == PAGE_ROUTE)
    {
        pageDirty = 1;
    }
}

/**
 * @brief  刷新机械臂测试页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_ARM 页时置脏；本页主循环节拍 ArmTest_Update 与
 *         按键处理都会调用（电机位置一直在变，需要持续刷新才能看到实时值）。
 *         实际重绘受 REFRESH_PERIOD_MS 节拍限制，不会高频全量重绘。
 */
void Display_ArmRefresh(void)
{
    if (currentPage == PAGE_ARM)
    {
        pageDirty = 1;
    }
}

/**
 * @brief  刷新手臂调试页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_ARM_SEQ 页时置脏；回原点/动作序列状态变化与按键时调用。
 */
void Display_ArmSeqRefresh(void)
{
    if (currentPage == PAGE_ARM_SEQ)
    {
        pageDirty = 1;
    }
}

/**
 * @brief  刷新手臂参数页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_ARM_PARA 页时置脏；读数刷新（节流）与按键时调用。
 */
void Display_ArmParaRefresh(void)
{
    if (currentPage == PAGE_ARM_PARA)
    {
        pageDirty = 1;
    }
}

/**
 * @brief  刷新赛题页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_MISSION 页时置脏；赛题执行器在状态/阶段变化时调用，
 *         到点停留期间每拍调用以刷新 0.1s 倒计时，按键操作也调用。
 */
void Display_MissionRefresh(void)
{
    if (currentPage == PAGE_MISSION)
    {
        pageDirty = 1;
    }
}

/**
 * @brief  刷新任务调试页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_TASK_DBG 页时置脏；任务调试状态机在阶段/状态变化时每拍调用，
 *         按键操作也调用。实际重绘受 REFRESH_PERIOD_MS 节流。
 */
void Display_TaskDbgRefresh(void)
{
    if (currentPage == PAGE_TASK_DBG)
    {
        pageDirty = 1;
    }
}

/**
 * @brief  刷新对位调参页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_ALIGN_TEST 页时置脏；`AlnTest_Update()` 按 ALNT_UI_MS 节流调用。
 */
void Display_AlignTestRefresh(void)
{
    if (currentPage == PAGE_ALIGN_TEST)
    {
        pageDirty = 1;
    }
}

/**
 * @brief  刷新速度模式测试页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_VELTEST 页时置脏；`VelTest_Update()` 按 200ms 节流调用
 *         （运行中要显示实时航向），按键操作也调用。实际重绘受 REFRESH_PERIOD_MS 节流。
 */
void Display_VelTestRefresh(void)
{
    if (currentPage == PAGE_VELTEST)
    {
        pageDirty = 1;
    }
}

/**
 * @brief  刷新速度模式跑图页（设置脏标志触发下次重绘）
 * @param  无
 * @retval 无
 * @note   仅当前处于 PAGE_VELROUTE 页时置脏；只在状态/步号变化与按键时调用，
 *         运行中不周期刷新 —— 整页 8x16 重绘阻塞约 100ms，会把 20ms 控制环拖成大步跳。
 */
void Display_VelRouteRefresh(void)
{
    if (currentPage == PAGE_VELROUTE)
    {
        pageDirty = 1;
    }
}

/**
 * @brief  OLED 自检测试函数（run 回调）
 * @param  无
 * @retval 无
 * @note   批量写刷全屏黑/白各一次再清屏恢复；I2C 在线则 PASS，否则 FAIL
 */
void OLED_Test(void)
{
    uint8_t buf[OLED_PAGE_BUF_SIZE];
    uint8_t p;

    if (!oledOnline)
    {
        testItems[TEST_ID_OLED].state = TEST_STATE_FAIL;
        Debug_Log("TEST", "OLED offline");
        return;
    }

    memset(buf, 0x00, sizeof(buf));
    for (p = 0; p < 8U; p++) OLED_WritePage(p, buf);
    HAL_Delay(TEST_OLED_FLASH_MS);

    memset(buf, 0xFF, sizeof(buf));
    for (p = 0; p < 8U; p++) OLED_WritePage(p, buf);
    HAL_Delay(TEST_OLED_FLASH_MS);

    OLED_ClearAll();
    pageDirty = 1;

    testItems[TEST_ID_OLED].state = TEST_STATE_PASS;
    Debug_Log("TEST", "OLED self-test ok");
}

/**
 * @brief  进入页面选择菜单
 * @param  无
 * @retval 无
 * @note   从开机页进入时返回页默认为数据页，其余记录当前页
 */
void Display_MenuEnter(void)
{
    Display_SetPage(PAGE_MENU);

    /* 校正滚动窗口，保证光标（可能是上次离开时留下的位置）可见 */
    MenuEnsureVisible();

    DrawPageMenu();
}

/**
 * @brief  退出页面选择菜单（回到进入前的页面）
 * @param  无
 * @retval 无
 */
void Display_MenuExit(void)
{
    Display_SetPage(menuReturnPage);
}

/**
 * @brief  查询是否处于页面选择菜单
 * @param  无
 * @retval 1=菜单页，0=非菜单页
 */
uint8_t Display_MenuIsActive(void)
{
    return (currentPage == PAGE_MENU) ? 1U : 0U;
}

/**
 * @brief  菜单内按键处理
 * @param  key: 按键 ID
 * @retval 无
 * @note   KEY1=上移、KEY2=下移、KEY3=确认进入、KEY4=退出。
 *         菜单项多于 4 项时靠滚动窗口显示，光标移动后调用 MenuEnsureVisible()
 *         保证光标始终在可见窗口内。
 */
void Display_MenuHandleKey(KeyID_t key)
{
    switch (key)
    {
        case KEY1:
            if (menuCursor == 0)
            {
                menuCursor = (uint8_t)(MENU_ITEM_COUNT - 1U);
            }
            else
            {
                menuCursor--;
            }
            MenuEnsureVisible();
            pageDirty = 1;
            break;

        case KEY2:
            menuCursor = (uint8_t)((menuCursor + 1U) % MENU_ITEM_COUNT);
            MenuEnsureVisible();
            pageDirty = 1;
            break;

        case KEY3:
            if (menuItems[menuCursor].page == PAGE_TEST)
            {
                /* 测试页有"进入前初始化"（Test_Enter），不能只切页 */
                Test_Enter();
                Display_SetPage(PAGE_TEST);
            }
            else if (menuItems[menuCursor].page == PAGE_ARM_SEQ)
            {
                /* 手臂调试页：进入即启动回原点状态机（ArmSeq_Enter 内部切页） */
                ArmSeq_Enter();
            }
            else if (menuItems[menuCursor].page == PAGE_ARM_PARA)
            {
                /* 手臂参数页：进入即电机失能 + 刷新读数（ArmPara_Enter 内部切页） */
                ArmPara_Enter();
            }
            else if (menuItems[menuCursor].page == PAGE_MISSION)
            {
                /* 赛题页：进入即里程计清零（原点=出发点）+ 切页；
                   不自动执行（安全，进页后按 KEY5 执行当前步/一键启动）（Task_Enter 内部切页） */
                Task_Enter();
            }
            else if (menuItems[menuCursor].page == PAGE_TASK_DBG)
            {
                /* 任务调试页：进入即复位状态 + 切页；不自动执行（安全，进页后按 KEY3 执行当前任务点）
                   （TaskDbg_Enter 内部切页） */
                TaskDbg_Enter();
            }
            else if (menuItems[menuCursor].page == PAGE_ALIGN_TEST)
            {
                /* 对位调参页：进入即使能四轮抱轴 + 切页；**不自动开跑**（安全，进页后按 KEY3） */
                AlnTest_Enter();
            }
            else if (menuItems[menuCursor].page == PAGE_VELTEST)
            {
                /* 速度模式测试页：进入即使能四轮抱轴 + 切页；**不自动开跑**（安全，进页后按 KEY5） */
                VelTest_Enter();
            }
            else if (menuItems[menuCursor].page == PAGE_VELROUTE)
            {
                /* 速度模式跑图页：进入即里程计清零 + 使能四轮 + 切页；**不自动开跑**（按 KEY5） */
                VelRoute_Enter();
            }
            else
            {
                Display_SetPage(menuItems[menuCursor].page);
            }
            break;


        default:
            break;
    }
}
