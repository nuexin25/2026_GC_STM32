# 速度模式跑图页加入真实任务点（绝对坐标路段表）

## Context（背景与目标）

速度模式跑图页 [bsp_vel_route.c](file:///c:/Users/Nuexin/Desktop/2026_GC_STM32/nx_car/nx_Drivers/Src/bsp_vel_route.c) 目前只跑**移动段**（ROT 转向 / GO 平移）。用户要求把赛题的**任务点（作业点）**也放进来，跑到任务点后**真实执行作业**（扫码 / 取料 / 放置 / 取回 / 码垛 / 统计），让本页能完整跑完一整场。

**用户明确约束（本轮反馈）：**
1. 路段表要**和位置模式类似**——用**绝对坐标**（GO 存目标绝对 x,y；ROT 存绝对朝向），不是相对腿。
2. **路段表坐标由用户自己改**——我只搭机制与骨架，不替用户换算相对腿。

## 关键事实（已调研确认）

- 作业函数**已全部 public**，在 [bsp_task.h](file:///c:/Users/Nuexin/Desktop/2026_GC_STM32/nx_car/nx_Drivers/Inc/bsp_task.h) 可直接调用：`QR(void)`（L339）、`PICK_TRAY_ALL(uint8_t batch)`（L366）、`PLACE_STORE_ALL(void)`（L382）、`PLACE_BACK_ALL(void)`（L399）、`PLACE_ROUGH(uint8_t i)`（L408）、`STACK_STORE(uint8_t i)`（L433）、`STAT(void)`（L440）。
- 序列控制/中止在 [bsp_tt.h](file:///c:/Users/Nuexin/Desktop/2026_GC_STM32/nx_car/nx_Drivers/Inc/bsp_tt.h) 已 public：`SP_SetBatch`(L243)、`SP_SetRoughMode`(L235)、`SB_SetBatch`(L320)、`TT_Abort`(L156)、`SP_Abort`(L248)、`SB_Abort`(L325)。
- main.c 已**无条件每拍**调度 `ArmAct_Update / TT_Update / SP_Update / SB_Update / Align_Update`，作业序列启动后自动推进，本页无需额外调度。
- 任务点顺序与坐标蓝本：bsp_task.c 的 `Task_Update()` switch（L1129-1239），`GO(x,y)` 全为绝对坐标。

## 方案

### 1. 路段表结构（bsp_vel_route.c）——绝对坐标 + 任务点

`VrKind_t` 增加 `VR_TASK`；`VrStep_t` 保持绝对坐标语义并加任务字段：

```c
typedef enum {
    VR_ROT = 0,   /* 原地转到绝对朝向 deg */
    VR_GO,        /* 走到绝对坐标 (x,y) */
    VR_TASK       /* 任务点：原地执行真实作业 task */
} VrKind_t;

typedef enum {
    VR_TASK_QR = 0, VR_TASK_PICK_B1, VR_TASK_PLACE_B1, VR_TASK_PBACK_B1,
    VR_TASK_RGHPLACE_B1, VR_TASK_PICK_B2, VR_TASK_PLACE_B2, VR_TASK_PBACK_B2,
    VR_TASK_STACK_B2, VR_TASK_STAT
} VrTask_t;

typedef struct {
    uint8_t     kind;   /* VR_ROT / VR_GO / VR_TASK */
    int32_t     x;      /* GO：绝对 X（mm，前正） */
    int32_t     y;      /* GO：绝对 Y（mm，右正） */
    int16_t     deg;    /* ROT：绝对朝向（deg）；GO：先走腿 0=先前后 1=先左右 */
    uint8_t     task;   /* VR_TASK：VrTask_t 作业类型 */
    const char *name;   /* 页面显示（≤8 字符） */
} VrStep_t;
```

### 2. GO 内核改回「绝对坐标 + 段首分解」（bsp_mission.c）

`Mnv_Go(int32_t x, int32_t y, uint8_t first)` 从当前的「相对腿直接赋值」改回**绝对坐标**：

- 段首读一次里程计 `Chassis_GetOdomX/Y()` 与当前朝向 `Chassis_GetOdomYawTrue()`，算 `dX = x - OdomX`、`dY = y - OdomY`。
- 用段首朝向旋到车身系：`f = dX·cosθ - dY·sinθ`（前后腿）、`l = dX·sinθ + dY·cosθ`（左右腿）。
- 之后**正交两段腿**纯时间开环走（先前前后后左右，`first`/`deg` 决定顺序），过程中不纠偏、wz 由 `MNV_YAW_HOLD` 航向环叠加——即回到本页最早那版「段首读一次、过程开环」的形态。

> 说明：路段表是绝对坐标（好改），「先前后再左右」是为了纯时间开环下仍走直线，这段换算**在代码内部段首自动完成**，用户改坐标仍只管绝对 x/y。

### 3. 任务点分派（bsp_vel_route.c）

新增 `Vr_TaskUpdate(uint8_t task)`，每拍调用，返回 1 推进：

```c
switch (task) {
    case VR_TASK_QR:            return QR();
    case VR_TASK_PICK_B1:       return PICK_TRAY_ALL(1U);
    case VR_TASK_PLACE_B1:      SP_SetBatch(1U); SP_SetRoughMode(0U); return PLACE_STORE_ALL();
    case VR_TASK_PBACK_B1:      SB_SetBatch(1U);                         return PLACE_BACK_ALL();
    case VR_TASK_RGHPLACE_B1:   SP_SetBatch(1U); SP_SetRoughMode(1U); return PLACE_ROUGH(0);
    case VR_TASK_PICK_B2:       return PICK_TRAY_ALL(2U);
    case VR_TASK_PLACE_B2:      SP_SetBatch(2U); SP_SetRoughMode(0U); return PLACE_STORE_ALL();
    case VR_TASK_PBACK_B2:      SB_SetBatch(2U);                         return PLACE_BACK_ALL();
    case VR_TASK_STACK_B2:      return STACK_STORE(0);
    case VR_TASK_STAT:          return STAT();
    default:                    return 1U;
}
```

`VelRoute_Update()` 分发：`VR_ROT → Mission_VelRot(deg)`；`VR_GO → Mission_VelGo2(x,y,deg)`；`VR_TASK → Vr_TaskUpdate(task)`。头文件补 `#include "bsp_task.h"`、`#include "bsp_tt.h"`。

### 4. 路段表骨架（绝对坐标，用户自行修改）

`vrSteps[]` 按 bsp_task 的 `Task_Update()` switch 顺序给出**绝对坐标骨架**（初始值照抄现行 bsp_task.c，用户之后直接改每行的 x/y/deg 数字即可）。任务点显式插在对应位置：

```c
/* ROT 绝对朝向 / GO 绝对坐标 (x,y) / task 任务点 */
{ VR_ROT,    0,     0,    0, 0, "QR ROT"  },
{ VR_GO,   150,  -150,    0, 0, "QR GO"   },
{ VR_ROT,    0,     0,  -90, 0, "QR R1"   },
{ VR_GO,   150, -1300,    0, 0, "QR T1"   },
{ VR_TASK,   0,     0,    0, VR_TASK_QR, "QR DO" },
{ VR_GO,   150, -1060,    0, 0, "B1 T-GO" },
{ VR_ROT,    0,     0,    0, 0, "B1 T-R0" },
{ VR_GO,  1042, -1080,    0, 0, "B1 T-IN" },
{ VR_GO,  1042,  -109,    0, 0, "B1 T-AT" },
{ VR_TASK,   0,     0,    0, VR_TASK_PICK_B1, "B1 PICK" },
{ VR_GO,  1042,  -197,    0, 0, "B1 R-BK" },
{ VR_ROT,    0,     0,   90, 0, "B1 R-90" },
{ VR_GO,  1042, -1820,    0, 0, "B1 RGH"  },
{ VR_ROT,    0,     0,  180, 0, "B1 R-180" },
{ VR_TASK,   0,     0,    0, VR_TASK_PLACE_B1, "B1 PLACE" },
{ VR_TASK,   0,     0,    0, VR_TASK_PBACK_B1, "B1 PBACK" },
{ VR_GO,  1887, -1820,    0, 0, "B1 STO"  },
{ VR_ROT,    0,     0,   90, 0, "B1 S-90" },
{ VR_GO,  1887,  -970,    0, 0, "B1 S-GO" },
{ VR_TASK,   0,     0,    0, VR_TASK_RGHPLACE_B1, "B1 RGH" },
{ VR_GO,  1900,  -109,    0, 0, "B2 T-GO" },
{ VR_ROT,    0,     0,    0, 0, "B2 T-R0" },
{ VR_GO,  1024,  -109,    0, 0, "B2 T-IN" },
{ VR_TASK,   0,     0,    0, VR_TASK_PICK_B2, "B2 PICK" },
{ VR_GO,  1042,  -197,    0, 0, "B2 R-BK" },
{ VR_ROT,    0,     0,   90, 0, "B2 R-90" },
{ VR_GO,  1042, -1820,    0, 0, "B2 RGH"  },
{ VR_ROT,    0,     0,  180, 0, "B2 R-180" },
{ VR_TASK,   0,     0,    0, VR_TASK_PLACE_B2, "B2 PLACE" },
{ VR_TASK,   0,     0,    0, VR_TASK_PBACK_B2, "B2 PBACK" },
{ VR_GO,  1887, -1820,    0, 0, "B2 TOST" },
{ VR_ROT,    0,     0,   90, 0, "B2 T-R90" },
{ VR_GO,  1887,  -970,    0, 0, "B2 TOST2" },
{ VR_TASK,   0,     0,    0, VR_TASK_STACK_B2, "B2 STACK" },
{ VR_GO,  1800,  -950,    0, 0, "B2 BACK" },
{ VR_ROT,    0,     0,    0, 0, "B2 S-R0" },
{ VR_GO,   130,  -950,    0, 0, "B2 STO"  },
{ VR_ROT,    0,     0,    0, 0, "HOME R"  },
{ VR_GO,   -16,     0,    0, 0, "HOME GO" },
{ VR_TASK,   0,     0,    0, VR_TASK_STAT, "STAT" },
```

> 上面的 x/y/deg 数值照抄自当前 bsp_task.c，仅作初始骨架；**用户后续直接改这些绝对坐标即可**（与位置模式赛题页改法一致）。

### 5. 超时与清理

- 新增 `VR_TASK_TIMEOUT_MS`（建议 60000U）用于任务点整段；`VR_TIMEOUT_MS` 继续用于移动段。`VelRoute_Update()` 按 `kind==VR_TASK` 选超时。
- `VelRoute_Enter()` / `VelRoute_Exit()` 调 `TT_Abort()` / `SP_Abort()` / `SB_Abort()` 清理残留序列。

### 6. 说明性注释

[bsp_vel_route.h](file:///c:/Users/Nuexin/Desktop/2026_GC_STM32/nx_car/nx_Drivers/Inc/bsp_vel_route.h) 文件头「路段表只含移动段」改为「移动段（绝对坐标）+ 任务点（真实作业）」。

## 关键文件

- [bsp_vel_route.c](file:///c:/Users/Nuexin/Desktop/2026_GC_STM32/nx_car/nx_Drivers/Src/bsp_vel_route.c)：枚举/结构体、vrSteps 骨架、Update 三分支、Vr_TaskUpdate、超时、清理。
- [bsp_mission.c](file:///c:/Users/Nuexin/Desktop/2026_GC_STM32/nx_car/nx_Drivers/Src/bsp_mission.c)：`Mnv_Go` 改回「绝对坐标段首分解」。
- [bsp_vel_route.h](file:///c:/Users/Nuexin/Desktop/2026_GC_STM32/nx_car/nx_Drivers/Inc/bsp_vel_route.h)：文件头注释。
- 复用（不改）：[bsp_task.h](file:///c:/Users/Nuexin/Desktop/2026_GC_STM32/nx_car/nx_Drivers/Inc/bsp_task.h) 作业函数、[bsp_tt.h](file:///c:/Users/Nuexin/Desktop/2026_GC_STM32/nx_car/nx_Drivers/Inc/bsp_tt.h) 序列控制。

## 验证

1. Keil 编译，确认 0 Error / 0 Warning。
2. 烧录后进 VELROUTE 页，KEY3 切 STEP，KEY5 逐步跑：先核对移动段绝对坐标方向/距离，再验证任务点到位后机械臂执行真实作业并自动推进。
3. AUTO 连跑一遍，确认 40 段（30 移动 + 10 任务点）全部跑通，无卡死（超时兜底）。