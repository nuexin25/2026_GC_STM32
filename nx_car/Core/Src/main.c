/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_Emm_V5.h"
#include "hwt101ct.h"
#include "oled.h"
#include "Emm_V5.h"
#include "bsp_debug.h"
#include "bsp_oled_ui.h"
#include "bsp_key.h"
#include "bsp_imu.h"
#include "bsp_test.h"
#include "bsp_motor.h"
#include "bsp_wheel_test.h"
#include "bsp_mecanum.h"        /* P4 麦轮运动学：整车位移 -> 四轮目标脉冲 */
#include "bsp_chassis.h"        /* P4 车机运动序列：位置模式 + 到位判定 + 航向纠偏 */
#include "bsp_chassis_test.h"   /* P4 车机测试页控制（PAGE_CHASSIS） */
#include "bsp_qr.h"             /* P5 GM65 扫码模块：UART4 中断接收 + 帧超时定界 */
#include "bsp_qr_test.h"        /* P5 扫码测试页控制（PAGE_QR） */
#include "bsp_rpi.h"             /* P6 树莓派通讯模块：USART1 收发 + 4 收 5 发（协议 v1.0） */
#include "bsp_rpi_test.h"        /* P6 树莓派通讯测试页控制（PAGE_RPI） */
#include "bsp_route.h"           /* P7 跑图执行器：按路段表顺序自动行驶（位置模式） */
#include "bsp_arm_action.h"      /* P9 抓取动作库：标定表 + 固定流程 + 非阻塞执行器 */
#include "bsp_align.h"           /* P9 视觉闭环定位：按像素偏移迭代平移到位 */
#include "bsp_tt.h"              /* P10 转盘取料序列：对位 + 连抓 3 件放车载 1/2/3 */
#include "bsp_route_test.h"      /* P7 跑图测试页控制（PAGE_ROUTE） */
#include "bsp_tune.h"            /* P7 自动调车：UART5 在线调参命令（RUN/STOP/参数=值） */
#include "bsp_arm.h"             /* P8 机械臂电机：手臂转向/升降（CAN addr 5/6）+ 舵机参数 */
#include "LobotServoController.h" /* P8 幻尔总线舵机官方库（经调试板 TTL 口，USART3） */
#include "bsp_servo.h"           /* P8 舵机回读模块：实际位置 / 调试板电压 */
#include "bsp_arm_test.h"        /* P8 机械臂测试页控制（PAGE_ARM） */
#include "bsp_arm_seq.h"         /* P8 联调 手臂调试页：回原点 + 动作序列（PAGE_ARM_SEQ） */
#include "bsp_arm_para.h"        /* P8 联调 手臂参数页：读电机/舵机参数（PAGE_ARM_PARA） */
#include "bsp_mission.h"         /* P9 赛题页：旧点位流程（导航内核 Mission_Nav* 仍被 bsp_task 复用） */
#include "bsp_task.h"            /* P9 赛题流程引擎：Task_Update/HandleKey（PAGE_MISSION） */
#include "bsp_task_dbg.h"        /* P9 任务调试页：任务点单点调试（PAGE_TASK_DBG） */
#include "bsp_align_test.h"      /* P24 对位调参页：单独跑对位 + 逐帧 PID 数据（PAGE_ALIGN_TEST） */
#include "bsp_vel_test.h"        /* P25 速度模式测试页：纯陀螺仪 + 时间控距离（PAGE_VELTEST） */
#include "bsp_vel_route.h"       /* P26 速度模式跑图页：赛题路段表全程 0xF6（PAGE_VELROUTE） */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* CAN 滤波器初始化、启动 CAN、使能 RX0 接收中断（与参考工程 nx_car_cs 一致；
     can.c 的 MX_CAN_Init 不含滤波器/启动代码，必须在此调用） */
  USER_CAN1_Filter_Init();
  if(HAL_CAN_Start(&hcan) != HAL_OK) { Error_Handler(); }
  if(HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) { Error_Handler(); }

  /* 调试系统初始化：熄灭 LED、上电蜂鸣、记录起始节拍 */
  BSP_Debug_Init();

  /* 上电横幅：项目名、编译时间、系统时钟频率 */
  printf("\r\n========================================\r\n");
  printf("  nx_car 智能搬运机器人\r\n");
  printf("  Build: %s %s\r\n", __DATE__, __TIME__);
  printf("  SYSCLK: %lu MHz\r\n", (unsigned long)(HAL_RCC_GetSysClockFreq() / 1000000UL));
  printf("========================================\r\n");
  Debug_Log("SYS", "boot done, enter main loop");

  /* OLED 显示系统初始化（检测在线+开机页） */
  Display_Init();
  /* 按键初始化（GPIO 已由 CubeMX 配置为上拉输入） */
  Key_Init();

  /* IMU 初始化：USART2 中断接收，波特率由 HWT101CT_UART_BAUD 宏决定 */
  IMU_Init();
  /* 模块自检框架初始化（注册表复位；电机使能/转动由单轮测试页 WheelTest_Enter 触发，上电不直发电机命令） */
  Test_Init();
  /* 电机总线自检启动（P3）：只读不写——依次读 1~4 号电机实时位置 S_CPOS，
     初始化阶段禁止发使能/速度指令，防上电车轮突转；应答由主循环 Motor_Update 轮询推进 */
  Motor_Init();
  /* 麦轮运动学初始化（P4）：只打印几何参数便于核对标定值，不发任何电机命令；
     上电阶段禁止使能/转动，四轮使能由车机测试页启动动作时触发 */
  Mecanum_Init();
  /* 车机运动模块初始化（P4）：只复位状态机，不发电机命令 */
  Chassis_Init();
  /* GM65 扫码模块初始化（P5）：启动 UART4 单字节中断接收，扫码此后台常驻运行，
     页面进入/退出不影响接收；合法任务码自动同步到任务码页（PAGE_TASK） */
  QR_Init();
  /* 树莓派通讯模块初始化（P6）：启动 USART1 单字节中断接收，通讯此后台常驻运行，
     页面进入/退出不影响接收；**初始化阶段不发任何指令**
     （$RCP/$RZ/$CCG/$SHOT/$CODE 由 P9 任务层或测试页手工触发） */
  RPi_Init();
  /* 转盘取料序列初始化（P10）：只复位状态机，不动电机；
     一键序列由 PICK_TRAY_ALL() 或任务调试页 TRAY PICK 启动（TT_Start） */
  TT_Init();
  /* 打靶放置序列初始化（P10 的 SP_，与 TRAY PICK 同源、同在 bsp_tt）：只复位状态机，不动电机；
     由任务调试页 STORE/ROUGH PLACE 或赛题流程 T_B1_PLACE（PLACE_STORE_ALL）启动（SP_Start） */
  SP_Init();
  /* 打靶取回序列初始化（P10 的 SB_，STORE BACK）：只复位状态机，不动电机；
     由任务调试页 STORE BACK 或赛题流程 T_B1_PBACK（PLACE_BACK_ALL）启动（SB_Start） */
  SB_Init();
  /* 跑图执行器初始化（P7）：只复位状态机，不发电机命令；
     四轮使能/下发位置命令由跑图测试页按 KEY3 启动时触发 */
  Route_Init();
  /* 在线调参模块（P7 自动调车）：从**日志串口 UART5 的 RX**轮询收 ASCII 命令
     （RUN/STOP/DUMP/DEFAULT/参数=值），改 bsp_route 的运行期参数；只收不发，
     不占中断、不改 stm32f1xx_it.c。PC 端脚本 auto_tune.py 靠它自动调车。 */
  Tune_Init();
  /* 舵机调试板串口适配（P8）：官方库 LobotServoController 假定串口已配好，
     而幻尔调试板的 TTL 口固定 9600，usart.c（CubeMX 生成、禁止改动）却把
     USART3 配成了 115200，故在此**运行期重设波特率**（8-N-1 等其它参数沿用
     usart.c 的配置）。只改串口参数，不发任何舵机动作指令（防上电爪子突动）。 */
  huart3.Init.BaudRate = (uint32_t)SERVO_BUS_BAUD;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Debug_Log("SRV", "baud=%u set FAIL", (unsigned int)SERVO_BUS_BAUD);
  }
  Debug_Log("SRV", "init, baud=%u", (unsigned int)SERVO_BUS_BAUD);
  /* 舵机回读模块初始化（P8）：启动 USART3 单字节中断接收，只收不发
     （必须在上面把 USART3 改成 9600 之后调用；初始化阶段不发任何动作指令） */
  Servo_Init();
  /* 机械臂电机初始化（P8）：只复位状态并启动周期查询，不发使能/动作指令；
     电机 5/6 使能由机械臂测试页进入时触发 */
  Arm_Init();
  /* 赛题流程模块初始化（P9）：只复位状态机，不发电机命令、不清零里程计、
     不使能四轮；里程计清零与四轮使能分别在进赛题页、按 KEY5 执行/启动时触发 */
  Task_Init();
  /* 任务点调试页初始化（P9）：只复位状态机，不发电机/机械臂命令；
     按键进入/执行由主菜单 TDBG 项与调试页自身按键触发 */
  TaskDbg_Init();

#if DEBUG_BEEP_TEST
  /* 临时验证：持续断续鸣叫(200ms on / 200ms off)，BZ_LED2 同步；调用 BSP_Beep_Stop() 立即静音 */
  BSP_Beep(0, 200, 200);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* LED 流水（非阻塞，每 LED_FLOW_PERIOD_MS 切换） */
    BSP_LED_Flow_Update();
    /* 心跳日志（非阻塞，每 HEARTBEAT_PERIOD_MS 输出） */
    BSP_Heartbeat_Update();
    /* OLED display task (non-blocking refresh + cycle) */
    Display_Task();
    /* 按键扫描：KEY1=下一页，KEY2=上一页，手动切页关闭轮播 */
    {
        KeyID_t lp = Key_GetLongPress();
        /* 单轮点动页/车机测试页/扫码测试页/树莓派页/跑图页/机械臂页/手臂调试页/手臂参数页/赛题页
           活动时屏蔽长按 KEY2 进菜单：子页与顶层页只能由 KEY4/KEY5 退出，
           否则页面切走后状态错乱（电机仍转/动作状态机卡在半途/显示残留） */
        if (lp == KEY2 && !Display_MenuIsActive() &&
            !WheelTest_IsActive() &&
            !ChassisTest_IsActive() &&
            !QRTest_IsActive() && !RPiTest_IsActive() &&
            !RouteTest_IsActive() && !ArmTest_IsActive() &&
            !ArmSeq_IsActive() && !ArmPara_IsActive() &&
            !Task_IsActive() && !TaskDbg_IsActive() &&
            !VelTest_IsActive() && !VelRoute_IsActive())
        {
            Display_MenuEnter();
        }
        else if ((lp != KEY_NONE) && ArmTest_IsActive())
        {
            /* 机械臂测试页的长按由本页处理（KEY5=让本执行器归到0点、KEY3=电机5手动回零）：
               长按事件被 Key_GetLongPress() 消费一次，且 bsp_key 的长按不产生短按事件，
               所以必须在这里派发，不能靠下面短按路由的形参带进去 */
            ArmTest_HandleLongKey(lp);
        }
        else if ((lp != KEY_NONE) && ArmPara_IsActive())
        {
            /* 手臂参数页的长按由本页处理（KEY3=把电机5 当前位置设为绝对0点，存电机 flash） */
            ArmPara_HandleLongKey(lp);
        }
        else if ((lp != KEY_NONE) && Task_IsActive())
        {
            /* 赛题页的长按由本页处理（长按 KEY5=里程计清零；短按 KEY5 已被
               "执行当前步 / 一键启动"占用） */
            Task_HandleLongKey(lp);
        }
        else if ((lp != KEY_NONE) && VelTest_IsActive())
        {
            /* 速度模式测试页：长按 KEY4/KEY5 = 停止并退出
               （短按 KEY4 是"参数减一档"，所以退出放长按） */
            VelTest_HandleLongKey(lp);
        }
        else if ((lp != KEY_NONE) && VelRoute_IsActive())
        {
            /* 速度模式跑图页：长按 KEY4/KEY5 = 停止并退出 */
            VelRoute_HandleLongKey(lp);
        }
        else if (lp != KEY_NONE)
        {
            Debug_Log("KEY", "id=%u event=long", (unsigned int)lp);
        }

        KeyID_t key = Key_Scan();
        if (key != KEY_NONE)
        {
            if (Test_IsActive())
            {
                /* 测试页活动内再分一层：单轮点动页活动时按键交对应子页处理
                   （子页按键语义独立）；否则维持测试页路由 */
                if (WheelTest_IsActive())
                {
                    WheelTest_HandleKey(key);
                }
                else if (ChassisTest_IsActive())
                {
                    /* 车机测试页：KEY1/KEY2 选动作、KEY3 启动/中止、KEY4/KEY5 退出 */
                    ChassisTest_HandleKey(key);
                }
                else if (QRTest_IsActive())
                {
                    /* 扫码测试页：KEY3 手动重扫、KEY4/KEY5 退出回测试页 */
                    QRTest_HandleKey(key);
                }
                else if (RPiTest_IsActive())
                {
                    /* 树莓派测试页：KEY1/KEY2 选指令项、KEY3 发送、KEY4/KEY5 退出回测试页 */
                    RPiTest_HandleKey(key);
                }
                else if (RouteTest_IsActive())
                {
                    /* 跑图测试页：KEY3 启动/中止跑图、KEY4/KEY5 退出回测试页 */
                    RouteTest_HandleKey(key);
                }
                else if (ArmTest_IsActive())
                {
                    /* 机械臂测试页：KEY1/KEY2 选执行器、KEY3/KEY4 主/副动作、
                       KEY5 停止全部并退出回测试页（本页 KEY4 不是退出键） */
                    ArmTest_HandleKey(key);
                }
                else
                {
                    Test_HandleKey(key, 0);
                }
            }
            else if (Display_MenuIsActive())
            {
                Display_MenuHandleKey(key);
            }
            else if (ArmSeq_IsActive())
            {
                /* 手臂调试页（主菜单进入的顶层页）：
                   KEY1/KEY2 选动作、KEY3 重新回原点、KEY4 退出、KEY5 启动/中止 */
                ArmSeq_HandleKey(key);
            }
            else if (AlnTest_IsActive())
            {
                /* 对位调参页（主菜单进入的顶层页）：
                   KEY1/KEY2 选数据源、KEY3 启动/停止、KEY4 退出、KEY5 停止 */
                AlnTest_HandleKey(key);
            }
            else if (VelTest_IsActive())
            {
                /* 速度模式测试页（主菜单进入的顶层页）：
                   KEY1/KEY2 选参数、KEY3 加一档、KEY4 减一档、KEY5 启动/停止；
                   长按 KEY4/KEY5 = 停止并退出（见上方长按路由） */
                VelTest_HandleKey(key);
            }
            else if (VelRoute_IsActive())
            {
                /* 速度模式跑图页（主菜单进入的顶层页）：
                   KEY1/KEY2 上/下一段、KEY3 切 AUTO/STEP、KEY4 退出、KEY5 启动/停止 */
                VelRoute_HandleKey(key);
            }
            else if (ArmPara_IsActive())
            {
                /* 手臂参数页（主菜单进入的顶层页）：KEY3 刷新读数、KEY4 退出 */
                ArmPara_HandleKey(key);
            }
            else if (Task_IsActive())
            {
                /* 赛题页（主菜单进入的顶层页）：KEY1/KEY2 上/下一步、KEY3 切 STEP/AUTO、
                   KEY4 退出、KEY5 执行当前步（AUTO 下=一键启动）/ 运行中=急停；
                   长按 KEY5 = 里程计清零（见上方长按路由） */
                Task_HandleKey(key);
            }
            else if (TaskDbg_IsActive())
            {
                /* 任务调试页（主菜单进入的顶层页）：KEY1/KEY2 上/下任务点、KEY3 执行/中止、
                   KEY4 退出、KEY5 中止+退出 */
                TaskDbg_HandleKey(key);
            }
            else
            {
                if (key == KEY4) { Display_MenuEnter(); }
                else if (key == KEY3) { IMU_ResetYaw(); Debug_Log("IMU", "yaw reset ok"); }
                Debug_Log("KEY", "id=%u event=short", (unsigned int)key);
            }
        }
    }
    /* 电机总线自检轮询（非阻塞）：推进收发状态机，收应答记 ok / 超时记 no reply */
    Motor_Update();
    /* 车机运动状态机推进（P4，非阻塞）：读四轮位置算与目标脉冲的误差判到位，
       按阶段推进（主运动段 -> 纠偏段/ARC 第二段 -> DONE）并刷新进度；
       漏调度会导致状态机一直停在 RUN 直到超时 */
    Chassis_Update();
    /* 相对坐标（里程计）每拍累加（P4，非阻塞）：四轮脉冲增量 -> 麦轮正解 ->
       按当前陀螺仪朝向旋转回原点坐标系后累加 x/y；与运动状态无关，
       手动推车也会计入。漏调度则车机测试页的坐标不更新 */
    Chassis_OdomUpdate();
    /* 跑图状态机推进（P7，非阻塞）：按路段表逐段执行（先转角度 -> 再到坐标点），
       到位判定复用电机到位回传 + 脉冲容差兜底；
       放在 OdomUpdate 之后，保证本拍用的是最新相对坐标算增量 */
    Route_Update();

    /* 赛题流程状态机推进（P9，非阻塞）：按 bsp_task.c 的步骤表逐段推进
       （先按当前朝向 GO 到点 -> 再 Rotate 转到目标朝向 -> 到点执行作业点）；
       导航内核复用 bsp_mission 已验证的转角闭环/同拍停稳/到位判定。
       放在 OdomUpdate 之后，保证本拍用的是最新相对坐标算增量 */
    Task_Update();

    /* 任务点调试页状态机推进（P9，非阻塞）：按三段式推进当前任务点的调试程序
       （发指令 -> 等数据/对位 -> 动作收尾；机械臂动作留桩）；漏调度会卡在某一阶段 */
    TaskDbg_Update();

    /* 在线调参轮询（P7 自动调车）：读空 UART5 RX 并执行命令；无数据时开销极小。
       放在 Route_Update 之后，保证"改参数→下一段生效"的时序直观。 */
    Tune_Update();
    /* 机械臂电机状态查询节拍（P8，非阻塞）：周期发 S_CPOS/S_FLAG/S_OFLAG
       刷新 M5/M6 的位置与"到位/回零完成"标志，并判定回零完成与超时；
       漏调度会导致到位/回零完成永远判不出来（回调只负责存，不负责查） */
    Arm_Update();
    /* ★【2026-10-23 舵机丢动作修复】**所有会发舵机运动指令的节拍放在
       `Servo_Update()` 之前** —— 同一拍内先下运动指令、最后才发位置回读请求，
       避免"刚发出的读请求占着总线、后面的运动指令被调试板丢掉"。
       （另有 `Servo_WaitBusIdle()` 在 Arm_ServoMove 内做物理静默判定兜底。） */
    /* 机械臂测试页刷新节拍（P8，非阻塞）：只置脏标志，实际重绘由 OLED 侧节流 */
    ArmTest_Update();
    /* 手臂调试页状态机（P8 联调，非阻塞）：推进"串行回原点 + 动作序列"；
       漏调度会一直卡在 INIT / 等待到位 */
    ArmSeq_Update();
    /* 手臂参数页读数刷新（P8 联调，非阻塞，内部按 ARM_PARA_REFRESH_MS 节流） */
    ArmPara_Update();
    /* 抓取动作主循环节拍（P9 机械臂动作库，非阻塞）：逐步推进当前抓取/放置动作，
       必要时自动衔接第二段（抓→放）；**漏调度会卡在某一步不动** */
    ArmAct_Update();
    /* ★【2026-10-23】把本拍积攒的舵机命令**合并成一帧**发出去（同一拍多个关节只发 1 帧） */
    Arm_ServoFlush();
    /* 舵机回读主循环节拍（P8，非阻塞）：处理中断攒好的应答帧（解析在主循环）、
       周期发读位置请求、超时重试与统计；漏调度则回读永不刷新（位置一直显示 --） */
    Servo_Update();
    /* 扫码模块主循环节拍（P5，非阻塞）：帧超时成帧解析 + 同步任务码页；
       漏调度则接收帧不会解析、OLED 任务码不更新 */
    QR_Update();
    /* 树莓派通讯主循环节拍（P6，非阻塞）：帧超时成帧解析 + 数据过期清零 + 离线判定；
       漏调度则收帧不解析、颜色/靶位数据不更新 */
    RPi_Update();
    /* 视觉闭环定位节拍（P9，非阻塞）：读 $RCP（转盘区）/ $SHOT（打靶区）偏移 ->
       平移一小步 -> 等停稳 -> 再读；收敛判据是像素死区（dRot 恒为 0，车身角度不歪）；
       漏调度会卡在对位中间 */
    Align_Update();
    /* ★对位调参页（P24，非阻塞）：推进"等数据 -> 开跑 -> 结束"，
       运行中按 ALNT_LOG_MS 打印逐帧调参日志；**漏调度不会开跑/不打日志** */
    AlnTest_Update();
    /* ★速度模式测试页（P25，非阻塞）：纯陀螺仪 + 时间控距离的两段直行 + 左转；
       推进速度曲线/转向闭环；**漏调度会停在某个状态不动** */
    VelTest_Update();
    VelRoute_Update();
    /* 转盘取料序列节拍（P10，非阻塞）：等物料停稳 -> 对位 -> 抓转盘1号位 -> 放车载1/2/3；
       由 PICK_TRAY_ALL()/任务调试页启动，**漏调度会卡在序列中间** */
    TT_Update();
    /* 打靶放置序列节拍（P10 的 SP_，非阻塞）：观察位 -> $SHOT 对位（严格档、不限时）->
       车载 1/2/3 逐件取件放靶位；由 PLACE_STORE_ALL()/任务调试页启动，
       **漏调度会卡在序列中间**（现象 = 任务点按了 KEY5 不动） */
    SP_Update();
    /* 打靶取回序列节拍（P10 的 SB_，STORE BACK，非阻塞）：按放置逆序 车载3→2→1 搬回；
       由 PLACE_BACK_ALL()/任务调试页启动，**漏调度会卡在序列中间** */
    SB_Update();
    /* 模块自检框架节拍：调度 RUN 项（含 KEY/IMU 测试刷新） */
    Test_Update();
    /* 蜂鸣器非阻塞节拍更新（PWM 启停 + BZ_LED2 同步），禁止阻塞 */
    BSP_Beep_Update();
    /* IMU 主循环节拍：解析帧/新鲜度检测/周期上报 */
    IMU_Update();
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
