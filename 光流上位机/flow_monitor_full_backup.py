# -*- coding: utf-8 -*-
"""
UP-F7-QX01 微距光流模块 上位机 (Upix 协议) + HWT101CT 陀螺融合
=============================================================
功能：
  1) 通过串口实时解析 UP-F7-QX01 光流模块输出的相对位移、速度与累计位移；
  2) 可选接入 HWT101CT 单轴陀螺仪，取航向角 θ，把每帧"机体系"位移旋转到
     "世界系"后累加，得到绝对坐标 (X, Y)，从而正确应对车体转向。

---------------------------- 光流协议 (Upix) ----------------------------
    帧结构 14 字节：
      [0] 0xFE            包头(固定)
      [1] 0x0A            光流数据结构体长度(固定 10)
      [2] flow_x_integral 低字节   )  X 方向累加位移(放大 10000 倍的 rad)
      [3] flow_x_integral 高字节   )
      [4] flow_y_integral 低字节   )  Y 方向累加位移(放大 10000 倍的 rad)
      [5] flow_y_integral 高字节   )
      [6] integration_timespan 低  )  两次发送之间的累计时间(us)
      [7] integration_timespan 高  )
      [8] ground_distance 低       )  预留(忽略)
      [9] ground_distance 高       )  预留(忽略)
      [10] valid                  状态/置信度  0=无效 255=最可靠
      [11] version                版本号(忽略)
      [12] XOR                    校验 = 结构体 10 字节(索引 2~11) 逐字节异或
      [13] 0x55                   包尾(固定)

    换算：
      X 角位移(rad) = flow_x_integral / 10000
      X 实际位移(mm) = X 角位移(rad) * 高度(mm)
      速度 = 实际位移 / 时间间隔(s)
    光流串口：115200, 8-N-1

---------------------- 陀螺协议 (HWT101CT / 维特智能) ----------------------
    帧结构 11 字节：
      [0] 0x55   包头
      [1] 0x53   角度帧类型(0x52 为角速度帧，本程序不使用)
      [2..9]     数据(角度帧: Yaw 在 byte6~7, 小端 int16)
      [10] SUM   校验 = 前 10 字节累加和 & 0xFF
    Yaw 换算：yaw(°) = int16 * 180 / 32768
    IMU 串口：9600, 8-N-1 (默认，可在界面改)

---------------------------- 坐标换算 ----------------------------
    机体系增量 (dx, dy) 为 mm；θ 为当前航向角(rad)：
      dX = dx*cosθ - dy*sinθ
      dY = dx*sinθ + dy*cosθ
    世界坐标累加：X += dX; Y += dY
    说明：无 IMU 数据时 θ 取 0，退化为"机体系累计"。

------------------------ ZUPT 零漂抑制(第一层) ------------------------
    长时间运行的零漂来自"零偏被积分"：陀螺 yaw 漂移误差 = b·t，光流零偏误差随路程累积。
    本程序用零速修正(ZUPT)压住它：
      静止判定：光流原始值 |x|,|y| <= STATIC_FLOW_TH_RAW 且 |yaw角速度| <= 阈值，
                连续 STATIC_FRAMES 帧安静 → 判定"静止"；连续 MOVE_FRAMES 帧有动作 → 判定"运动"(滞回)。
      静止期间：冻结坐标累加，并在线估计 光流零偏(原始单位) 与 yaw 漂移率(°/s)。
      运动期间：从原始光流中扣除零偏、从 yaw 增量中扣除漂移率后再积分。
    调参入口：本文件顶部 "ZUPT 零漂抑制参数" 一节。

------------------- 质量门控与滤波(第一层之②) -------------------
    协议里的 valid(0~255) 是置信度，低 valid 帧(纹理差/光照差/失焦)不可信：
      质量门控：valid < FLOW_QUALITY_MIN 的帧整帧丢弃，不参与累计/滤波/静止判定；
      去尖尖：只把"明显偏离中值的帧"替换掉，其余帧保留原值
              （不能整体取中值：光流是整数计数，慢速运动常输出 0,0,1,0,1... 的抖动序列，
                整体取中值会把它抹成 0 → 慢速位移丢失）；
      置信度加权：速度平滑系数按 valid/255 缩放，低质量帧对速度贡献更小。
    调参入口：本文件顶部 "质量门控与滤波参数" 一节 + 界面 "valid门限" 输入框。

------------------------ ZUPT 死区(慢速识别不出来) ------------------------
    STATIC_FLOW_TH_RAW 就是 ZUPT 的死区：小于该值的运动会被当成静止而冻结累加。
    死区速度(mm/s) ≈ STATIC_FLOW_TH_RAW / 10000 × 高度(mm) × 帧率，界面 ZUPT 行实时显示。
    想让更慢的运动也能被识别 → 调小 STATIC_FLOW_TH_RAW；
    若静止时判定不出"静止"(ZUPT 行一直显示"运动中") → 调大该值。

----------------------- 闭环校正(回到原点) -----------------------
    纯航迹推算的残差 ∝ 路程，回不到精确原点，必须引入绝对校正：
      - 自动吸附：车离开原点足够远(> 2×阈值)后，再次回到原点附近(<= 阈值)且判定"静止"时，
        自动把坐标吸附回 (0,0)，可同时把航向 θ 归零；
      - 手动校正：按界面 "原位校正" 按钮立刻吸附(确认车已在原点时用)；
      - 残差记录：每次吸附前记下偏差(mm)，即本次闭环的漂移量，用于评估光流精度与标定效果。
    调参入口：本文件顶部 "闭环校正参数" 一节 + 界面控件。

运行：python flow_monitor.py
依赖：pyserial (已安装)
"""

import math
import time
import queue
import threading
from collections import deque

import serial
import serial.tools.list_ports

import tkinter as tk
from tkinter import ttk
from tkinter import messagebox


# ============================ 光流(Upix)协议常量 ============================
FRAME_HEAD = 0xFE        # 包头
FRAME_TAIL = 0x55        # 包尾
STRUCT_LEN = 0x0A        # 结构体长度固定 10
FRAME_SIZE = 14          # 整帧长度 2(头+长度) + 10(结构体) + 1(校验) + 1(尾) = 14
FLOW_SCALE = 10000.0     # 协议中 flow 原始值是真实 rad 的 10000 倍
DEFAULT_FLOW_BAUD = 115200

# 速度平滑系数：0~1，越小越平滑（200/400Hz 原始速度抖动大，做轻度 EMA 便于读数）
SPEED_EMA_ALPHA = 0.25

# 默认模块到被测面高度(mm)。模块空气中有效测距 1.5~5.5cm，故默认 30mm。
DEFAULT_HEIGHT_MM = 30.0


# ============================ 陀螺(HWT101CT)协议常量 ============================
IMU_HEAD = 0x55          # 包头
IMU_TYPE_GYRO = 0x52     # 角速度帧(不使用)
IMU_TYPE_ANGLE = 0x53    # 角度帧(Yaw)
IMU_FRAME_LEN = 11       # 单帧长度
IMU_ANGLE_SCALE = 180.0 / 32768.0   # Yaw 量程 ±180°
DEFAULT_IMU_BAUD = 9600  # HWT101CT 出厂默认 9600

# 陀螺旋转方向：+1 表示 yaw 增大 = 车体逆时针(俯视)；装反/方向不对时改成 -1（或用界面勾选）
YAW_SIGN = 1

# 无 IMU 数据时，超过该时间(ms)未收到角度帧则视为"IMU 掉线"，回退 θ=0
IMU_STALE_MS = 500


# ============================ ZUPT 零漂抑制参数 ============================
# 思路：判定"静止"时冻结坐标累加，并在线估计两个零偏：
#   1) 光流零偏(原始单位) —— 静止时光流输出应≈0，其残差即为零点偏置；
#   2) yaw 漂移率(°/s)    —— 静止时 yaw 仍在缓慢变化，其变化率即陀螺零偏效应。
# 运动时把这两项扣掉再积分，可显著抑制长时间运行的零漂。
ZUPT_ENABLE_DEFAULT = True    # 默认启用
STATIC_FLOW_TH_RAW = 6        # 判定"安静"的光流原始值阈值(±)。
                              # 注意：这就是 ZUPT 的"死区"——小于该值的运动会被当成静止而冻结累加。
                              # 死区速度(mm/s) ≈ 阈值/10000 × 高度(mm) × 帧率，界面会实时显示。
                              # 慢速运动识别不出来时把这个值调小；静止时判定不出"静止"就调大。
STATIC_YAW_RATE_TH_DPS = 0.6  # 判定"安静"的 yaw 角速度阈值(°/s)
STATIC_FRAMES = 60            # 连续多少帧安静才判定为"静止"(阈值调小后需更长确认)
MOVE_FRAMES = 5               # 连续多少帧有动作才判定为"运动"(滞回，避免抖动)
BIAS_EMA_ALPHA = 0.03         # 零偏在线估计的平滑系数(越小越稳)


# ============================ 质量门控与滤波参数 ============================
# 光流协议里的 valid(0~255) 是置信度：太低说明纹理差/光照差/失焦，此时光流值不可信。
#   - 质量门控：valid < FLOW_QUALITY_MIN 的帧直接丢弃，不参与任何累计/滤波/静止判定；
#   - 中值滤波：抑制重复纹理(瓷砖、格栅)导致的"整周期跳变"尖尖；
#   - 置信度加权：速度平滑的权重按 valid 缩放，低质量帧对速度贡献更小。
FLOW_QUALITY_MIN = 30         # valid 门限(0~255)，低于此值丢帧
MEDIAN_WINDOW = 5             # 中值滤波窗口(帧数, 建议奇数)，越大越平滑但延迟越大
OUTLIER_TH_RAW = 3000         # 相对中值的偏差超过此值(原始单位)判为尖尖(仅统计)


# ============================ 闭环校正参数 ============================
# 纯航迹推算回不到精确原点，残差 ∝ 路程。工程解法是"绝对校正/闭环"：
#   - 自动吸附：车回到原点附近并停住(静止)时，把坐标吸附回 (0,0)，消除闭环残差；
#   - 手动校正：确认车已在原点时，按"原位校正"立刻吸附；
#   - 残差记录：吸附时记下校正前的偏差(mm)，用于评估光流精度/标定效果。
LOOP_AUTO_DEFAULT = True      # 默认开启自动吸附
LOOP_THRESH_MM = 150.0        # 自动吸附阈值：静止且距原点 < 该值(mm)时吸附
LOOP_RESET_YAW_DEFAULT = True # 吸附时是否同时把航向 θ 归零(原点处朝向=世界系 X 正向)


def _int16(lo, hi):
    """小端两字节 -> 有符号 16 位"""
    v = (hi << 8) | lo
    return v - 65536 if v >= 32768 else v


# ============================ 光流帧解析器 ============================
class UpixParser:
    """Upix 协议字节流解析器：喂入任意长度字节，吐出完整且校验通过的帧。"""

    def __init__(self):
        self.buf = bytearray()   # 接收缓冲
        self.ok = 0              # 校验通过帧数
        self.bad = 0             # 校验/长度错误帧数

    def feed(self, data):
        """喂入新数据，返回 [(x, y, ts_us, valid, version), ...]"""
        self.buf.extend(data)
        frames = []
        while True:
            # 1) 寻找包头
            idx = self.buf.find(bytes([FRAME_HEAD]))
            if idx < 0:
                self.buf.clear()          # 没有包头，丢弃全部
                break
            if idx > 0:
                del self.buf[:idx]        # 丢弃包头前的杂字节
            # 2) 数据不足一帧，等待
            if len(self.buf) < FRAME_SIZE:
                break
            # 3) 检查长度字节
            if self.buf[1] != STRUCT_LEN:
                self.bad += 1
                del self.buf[0]           # 假包头，后移一格继续找
                continue

            body = bytes(self.buf[2:12])  # 结构体 10 字节(索引 2~11)
            chk = self.buf[12]
            tail = self.buf[13]

            # 4) 校验：结构体逐字节异或 + 包尾
            calc = 0
            for b in body:
                calc ^= b

            if chk == calc and tail == FRAME_TAIL:
                x = _int16(body[0], body[1])
                y = _int16(body[2], body[3])
                ts = body[4] | (body[5] << 8)   # 无符号 16 位(us)
                valid = body[8]
                version = body[9]
                self.ok += 1
                del self.buf[:FRAME_SIZE]
                frames.append((x, y, ts, valid, version))
            else:
                self.bad += 1
                del self.buf[0]           # 校验失败，后移一格继续同步
        return frames


# ============================ 陀螺帧解析器 ============================
class WitMotionParser:
    """维特智能(HWT101CT)协议解析器：提取角度帧 Yaw。"""

    def __init__(self):
        self.buf = bytearray()
        self.ok = 0
        self.bad = 0
        self.yaw = None          # 最近一次 Yaw(°)

    def feed(self, data):
        """喂入新数据，返回本次解析出的 Yaw 列表(°)"""
        self.buf.extend(data)
        yaws = []
        while True:
            idx = self.buf.find(bytes([IMU_HEAD]))
            if idx < 0:
                self.buf.clear()
                break
            if idx > 0:
                del self.buf[:idx]
            if len(self.buf) < IMU_FRAME_LEN:
                break
            frame = bytes(self.buf[:IMU_FRAME_LEN])
            chk = 0
            for b in frame[:10]:
                chk = (chk + b) & 0xFF
            if chk == frame[10]:
                self.ok += 1
                del self.buf[:IMU_FRAME_LEN]
                if frame[1] == IMU_TYPE_ANGLE:
                    yaw = _int16(frame[6], frame[7]) * IMU_ANGLE_SCALE
                    self.yaw = yaw
                    yaws.append(yaw)
            else:
                self.bad += 1
                del self.buf[0]
        return yaws


# ============================ 串口读取线程 ============================
class SerialReader(threading.Thread):
    """后台线程：持续读串口 -> 解析 -> 把结果塞进队列（不阻塞 GUI）。"""

    def __init__(self, port, baud, out_queue, err_cb, parser):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.out_queue = out_queue
        self.err_cb = err_cb
        self.parser = parser
        self.ser = None
        self._stop = threading.Event()

    def run(self):
        try:
            # 8 位数据位 / 无校验 / 1 位停止位（串口默认值）
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except Exception as e:
            self.err_cb("打开串口 %s 失败: %s" % (self.port, e))
            return

        try:
            while not self._stop.is_set():
                try:
                    data = self.ser.read(512)      # 最多阻塞 0.1s
                except Exception as e:
                    self.err_cb("%s 串口读取异常: %s" % (self.port, e))
                    break
                if not data:
                    continue
                for item in self.parser.feed(data):
                    self.out_queue.put(item)
        finally:
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass

    def stop(self):
        self._stop.set()


# ============================ 主界面 ============================
class FlowMonitorApp:
    def __init__(self, root):
        self.root = root
        root.title("UP-F7-QX01 光流上位机 + HWT101CT 陀螺融合")
        root.resizable(False, False)

        self.flow_reader = None
        self.imu_reader = None
        self.flow_queue = queue.Queue()
        self.imu_queue = queue.Queue()

        # 机体系累计
        self.acc_x = 0.0          # 累计 X 位移(mm, 机体系)
        self.acc_y = 0.0
        # 世界系绝对坐标
        self.wx = 0.0             # 绝对坐标 X(mm)
        self.wy = 0.0
        # 速度平滑
        self.vel_x = 0.0
        self.vel_y = 0.0

        self.total_frames = 0
        self.ts_stamps = deque()  # 最近 1s 的帧时间戳(算帧率)

        # 航向角状态（零偏补偿后的 yaw 积分，可处理 ±180° 卷绕）
        self.yaw_last = None      # 上一次陀螺 Yaw 原始值(°)
        self.yaw_last_t = 0.0
        self.yaw_corr = 0.0       # 累积航向角(°)，启动时为 0
        self.yaw_rate_dps = 0.0   # 平滑后的 yaw 角速度(°/s)，用于静止判定
        self.yaw_drift_dps = 0.0  # 静止时估计出的 yaw 漂移率(°/s)
        self.yaw_stamp = 0.0      # 最近一次收到陀螺数据的时间

        # ZUPT 零漂抑制状态
        self.quiet_count = 0
        self.move_count = 0
        self.is_static = False
        self.flow_bias_x = 0.0    # 光流零点偏置(原始单位)
        self.flow_bias_y = 0.0

        # 质量门控与中值滤波
        self.win_x = deque(maxlen=MEDIAN_WINDOW)   # 最近 N 帧原始 X
        self.win_y = deque(maxlen=MEDIAN_WINDOW)
        self.rej_frames = 0       # 因 valid 过低被丢弃的帧数
        self.spike_frames = 0     # 尖尖(超过 OUTLIER_TH_RAW)帧数

        # 闭环校正
        self.moved_away = False   # 是否已离开原点足够远(避免一开机就吸附)
        self.last_res_mm = 0.0    # 最近一次校正前的残差(mm)
        self.max_res_mm = 0.0     # 历史最大残差(mm)
        self.loop_count = 0       # 闭环校正次数

        self.cur = dict(x=0, y=0, ts=0, valid=0, ver=0,
                        fmed_x=0, fmed_y=0, dx=0.0, dy=0.0)

        self._build_ui()
        self._refresh_ports()
        self.root.after(50, self._poll)   # 每 50ms 刷新一次界面

    # ---------------- UI 布局 ----------------
    def _build_ui(self):
        big = ("Consolas", 14, "bold")
        mid = ("Microsoft YaHei", 11)
        mono = ("Consolas", 12)

        # 顶部：光流串口设置
        fflow = ttk.LabelFrame(self.root, text="光流串口设置 (Upix 协议)")
        fflow.grid(row=0, column=0, sticky="ew", padx=10, pady=(8, 4))

        ttk.Label(fflow, text="串口:", font=mid).grid(row=0, column=0, padx=(8, 4), pady=6)
        self.flow_port_var = tk.StringVar()
        self.flow_port_cb = ttk.Combobox(fflow, textvariable=self.flow_port_var, width=26,
                                         state="readonly", font=mono)
        self.flow_port_cb.grid(row=0, column=1, padx=4, pady=6)
        ttk.Button(fflow, text="刷新", command=self._refresh_ports).grid(row=0, column=2, padx=4)

        ttk.Label(fflow, text="波特率:", font=mid).grid(row=0, column=3, padx=(12, 4))
        self.flow_baud_var = tk.StringVar(value=str(DEFAULT_FLOW_BAUD))
        ttk.Combobox(fflow, textvariable=self.flow_baud_var, width=8, font=mono,
                     values=["9600", "19200", "38400", "57600", "115200", "230400", "460800"]
                     ).grid(row=0, column=4, padx=4)

        self.btn_flow_open = ttk.Button(fflow, text="打开光流", command=self._open_flow)
        self.btn_flow_open.grid(row=0, column=5, padx=(12, 4))
        self.btn_flow_close = ttk.Button(fflow, text="关闭", command=self._close_flow, state="disabled")
        self.btn_flow_close.grid(row=0, column=6, padx=(4, 8))

        ttk.Label(fflow, text="模块高度(mm):", font=mid).grid(row=0, column=7, padx=(12, 4))
        self.height_var = tk.StringVar(value=str(int(DEFAULT_HEIGHT_MM)))
        ttk.Entry(fflow, textvariable=self.height_var, width=6, font=mono).grid(row=0, column=8, padx=(4, 8))

        ttk.Label(fflow, text="valid门限:", font=mid).grid(row=0, column=9, padx=(12, 4))
        self.qmin_var = tk.StringVar(value=str(FLOW_QUALITY_MIN))
        ttk.Entry(fflow, textvariable=self.qmin_var, width=6, font=mono).grid(row=0, column=10, padx=(4, 8))

        # IMU 串口设置
        fimu = ttk.LabelFrame(self.root, text="陀螺仪设置 (HWT101CT, 维特协议)")
        fimu.grid(row=1, column=0, sticky="ew", padx=10, pady=4)

        ttk.Label(fimu, text="串口:", font=mid).grid(row=0, column=0, padx=(8, 4), pady=6)
        self.imu_port_var = tk.StringVar()
        self.imu_port_cb = ttk.Combobox(fimu, textvariable=self.imu_port_var, width=26,
                                        state="readonly", font=mono)
        self.imu_port_cb.grid(row=0, column=1, padx=4, pady=6)

        ttk.Label(fimu, text="波特率:", font=mid).grid(row=0, column=3, padx=(12, 4))
        self.imu_baud_var = tk.StringVar(value=str(DEFAULT_IMU_BAUD))
        ttk.Combobox(fimu, textvariable=self.imu_baud_var, width=8, font=mono,
                     values=["4800", "9600", "19200", "38400", "57600", "115200", "230400"]
                     ).grid(row=0, column=4, padx=4)

        self.btn_imu_open = ttk.Button(fimu, text="打开陀螺", command=self._open_imu)
        self.btn_imu_open.grid(row=0, column=5, padx=(12, 4))
        self.btn_imu_close = ttk.Button(fimu, text="关闭", command=self._close_imu, state="disabled")
        self.btn_imu_close.grid(row=0, column=6, padx=(4, 8))

        self.yaw_var = tk.StringVar(value="θ --.-°")
        ttk.Label(fimu, textvariable=self.yaw_var, font=("Consolas", 13, "bold"),
                  foreground="#036").grid(row=0, column=7, padx=(12, 4))
        ttk.Button(fimu, text="θ 归零", command=self._zero_yaw).grid(row=0, column=8, padx=(4, 8))

        ttk.Label(fimu, text="安装偏置角(°):", font=mid).grid(row=1, column=0, columnspan=2,
                                                              sticky="w", padx=(8, 4), pady=(0, 8))
        self.offset_var = tk.StringVar(value="0")
        ttk.Entry(fimu, textvariable=self.offset_var, width=8, font=mono).grid(row=1, column=2,
                                                                              sticky="w", padx=4, pady=(0, 8))
        self.yaw_rev_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(fimu, text="θ 反向 (yaw 增大为顺时针)",
                        variable=self.yaw_rev_var).grid(row=1, column=3, columnspan=3,
                                                        sticky="w", padx=(12, 4), pady=(0, 8))

        self.zupt_var = tk.BooleanVar(value=ZUPT_ENABLE_DEFAULT)
        ttk.Checkbutton(fimu, text="启用 ZUPT 零漂抑制",
                        variable=self.zupt_var).grid(row=1, column=6, columnspan=3,
                                                     sticky="w", padx=(4, 8), pady=(0, 8))

        self.zupt_info_var = tk.StringVar(value="ZUPT: 待机")
        ttk.Label(fimu, textvariable=self.zupt_info_var, font=("Consolas", 10),
                  foreground="#666").grid(row=2, column=0, columnspan=9, sticky="w",
                                          padx=8, pady=(0, 8))

        # 闭环校正
        floop = ttk.LabelFrame(self.root, text="闭环校正 (回到原点)")
        floop.grid(row=2, column=0, sticky="ew", padx=10, pady=4)

        self.loop_auto_var = tk.BooleanVar(value=LOOP_AUTO_DEFAULT)
        ttk.Checkbutton(floop, text="自动吸附", variable=self.loop_auto_var
                        ).grid(row=0, column=0, padx=(8, 4), pady=6)

        ttk.Label(floop, text="吸附阈值(mm):", font=mid).grid(row=0, column=1, padx=(12, 4))
        self.loop_th_var = tk.StringVar(value=str(int(LOOP_THRESH_MM)))
        ttk.Entry(floop, textvariable=self.loop_th_var, width=6, font=mono).grid(row=0, column=2, padx=(4, 8))

        self.reset_yaw_var = tk.BooleanVar(value=LOOP_RESET_YAW_DEFAULT)
        ttk.Checkbutton(floop, text="同时归零 θ", variable=self.reset_yaw_var
                        ).grid(row=0, column=3, padx=(8, 4))

        ttk.Button(floop, text="原位校正", command=self._manual_loop
                   ).grid(row=0, column=4, padx=(12, 8))

        self.loop_info_var = tk.StringVar(value="上次残差 --.- mm  |  最大 --.- mm  |  次数 0")
        ttk.Label(floop, textvariable=self.loop_info_var, font=("Consolas", 11),
                  foreground="#036").grid(row=0, column=5, padx=(12, 8))

        # 状态行
        self.status_var = tk.StringVar(value="未连接")
        ttk.Label(self.root, textvariable=self.status_var, font=("Microsoft YaHei", 11),
                  foreground="#0a6").grid(row=3, column=0, sticky="w", padx=14)

        # 数据区（三列）
        body = ttk.Frame(self.root)
        body.grid(row=4, column=0, sticky="nsew", padx=10, pady=6)

        # 左：当前帧
        left = ttk.LabelFrame(body, text="当前帧 (原始 / 换算)")
        left.grid(row=0, column=0, sticky="n", padx=(0, 6))
        self.lb = {}
        for i, (name, key) in enumerate([
            ("flow_x_integral", "raw_x"),
            ("flow_y_integral", "raw_y"),
            ("timespan", "raw_ts"),
            ("valid(0-255)", "raw_valid"),
            ("去尖尖 X (raw)", "fmed_x"),
            ("去尖尖 Y (raw)", "fmed_y"),
            ("本帧位移 X (mm)", "dx"),
            ("本帧位移 Y (mm)", "dy"),
            ("速度 X (mm/s)", "vx"),
            ("速度 Y (mm/s)", "vy"),
        ]):
            ttk.Label(left, text=name + ":", font=mid).grid(row=i, column=0, sticky="w", padx=8, pady=3)
            self.lb[key] = ttk.Label(left, text="0", font=big, foreground="#036")
            self.lb[key].grid(row=i, column=1, sticky="e", padx=8, pady=3)

        # 中：机体系累计
        mid_col = ttk.LabelFrame(body, text="机体系累计 (车身轨迹)")
        mid_col.grid(row=0, column=1, sticky="n", padx=6)
        for i, (name, key) in enumerate([
            ("累计 X (mm)", "acc_x"),
            ("累计 Y (mm)", "acc_y"),
            ("合成距离 (mm)", "acc_d"),
        ]):
            ttk.Label(mid_col, text=name + ":", font=mid).grid(row=i, column=0, sticky="w", padx=8, pady=3)
            self.lb[key] = ttk.Label(mid_col, text="0", font=big, foreground="#666")
            self.lb[key].grid(row=i, column=1, sticky="e", padx=8, pady=3)

        # 右：世界系绝对坐标
        right = ttk.LabelFrame(body, text="世界系绝对坐标 (已做旋转补偿)")
        right.grid(row=0, column=2, sticky="n", padx=(6, 0))
        for i, (name, key) in enumerate([
            ("绝对 X (mm)", "wx"),
            ("绝对 Y (mm)", "wy"),
            ("距原点 (mm)", "wd"),
            ("航向角 θ (°)", "theta"),
            ("运动状态", "motion"),
            ("帧率 (Hz)", "fps"),
            ("总帧 / 坏帧", "cnt"),
            ("门控丢 / 尖尖", "qcnt"),
        ]):
            ttk.Label(right, text=name + ":", font=mid).grid(row=i, column=0, sticky="w", padx=8, pady=3)
            self.lb[key] = ttk.Label(right, text="0", font=big, foreground="#a30")
            self.lb[key].grid(row=i, column=1, sticky="e", padx=8, pady=3)

        # 坐标清 0 按钮：放在坐标面板正下方，方便随手清零(机体系 + 世界系位移)
        ttk.Button(right, text="坐标清 0", command=self._reset,
                   ).grid(row=i + 1, column=0, columnspan=2, sticky="ew", padx=8, pady=(10, 8))

        # 说明
        tip = ("位移 = flow/10000(rad) × 高度(mm)；世界系 = 机体系增量按航向角 θ 旋转后累加。\n"
               "质量门控：valid < 门限的帧直接丢弃；中值滤波抑制重复纹理导致的整周期跳变尖尖。\n"
               "ZUPT：判定静止时冻结累加，并在线估计光流零偏与 yaw 漂移率，运行越久越准。\n"
               "闭环：回到原点并停住时自动吸附回 (0,0)，残差即本次闭环的漂移量。\n"
               "θ 来自陀螺仪 Yaw：请把陀螺与光流分别接到两个 USB-TTL 串口。若无 θ，本程序退化为机体系累计。")
        ttk.Label(self.root, text=tip, font=("Microsoft YaHei", 9),
                  foreground="#666", justify="left").grid(row=5, column=0, sticky="w", padx=14, pady=(0, 8))

    # ---------------- 串口操作 ----------------
    def _refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        items = ["%s - %s" % (p.device, p.description) for p in ports]
        self.flow_port_cb["values"] = items
        self.imu_port_cb["values"] = items
        if items:
            if not self.flow_port_var.get():
                self.flow_port_var.set(items[0])
            if not self.imu_port_var.get():
                self.imu_port_var.set(items[min(1, len(items) - 1)])

    @staticmethod
    def _port_of(var):
        v = var.get()
        return v.split(" - ")[0].strip() if v else ""

    def _open_flow(self):
        port = self._port_of(self.flow_port_var)
        if not port:
            messagebox.showwarning("提示", "没有可用串口，请插入设备后点“刷新”。")
            return
        try:
            baud = int(self.flow_baud_var.get())
        except ValueError:
            messagebox.showwarning("提示", "波特率必须是数字。")
            return
        self.flow_queue = queue.Queue()
        self.flow_reader = SerialReader(port, baud, self.flow_queue, self._on_error, UpixParser())
        self.flow_reader.start()
        self.btn_flow_open.config(state="disabled")
        self.btn_flow_close.config(state="normal")
        self.status_var.set("光流已连接 %s @ %d" % (port, baud))

    def _close_flow(self):
        if self.flow_reader:
            self.flow_reader.stop()
            self.flow_reader = None
        self.btn_flow_open.config(state="normal")
        self.btn_flow_close.config(state="disabled")

    def _open_imu(self):
        port = self._port_of(self.imu_port_var)
        if not port:
            messagebox.showwarning("提示", "没有可用串口，请插入设备后点“刷新”。")
            return
        try:
            baud = int(self.imu_baud_var.get())
        except ValueError:
            messagebox.showwarning("提示", "IMU 波特率必须是数字。")
            return
        self.imu_queue = queue.Queue()
        self.imu_reader = SerialReader(port, baud, self.imu_queue, self._on_error, WitMotionParser())
        self.imu_reader.start()
        self.btn_imu_open.config(state="disabled")
        self.btn_imu_close.config(state="normal")

    def _close_imu(self):
        if self.imu_reader:
            self.imu_reader.stop()
            self.imu_reader = None
        self.btn_imu_open.config(state="normal")
        self.btn_imu_close.config(state="disabled")
        self.yaw_last = None
        self.yaw_corr = 0.0
        self.yaw_rate_dps = 0.0

    def _on_error(self, msg):
        # 线程回调，交给主线程弹窗
        self.root.after(0, lambda: (self.status_var.set(msg), messagebox.showerror("串口错误", msg)))

    def _zero_yaw(self):
        """把当前航向记为 0°（即世界系的 X 正方向）"""
        if self.yaw_last is None:
            messagebox.showinfo("提示", "还没收到陀螺数据，无法归零。")
            return
        self.yaw_corr = 0.0
        self.status_var.set("θ 已归零")

    def _reset(self):
        """坐标清 0：把机体系累计与世界系绝对坐标(及速度)归零，不影响航向角 θ 与已估出的零偏。"""
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.wx = 0.0
        self.wy = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.ts_stamps.clear()
        self.rej_frames = 0
        self.spike_frames = 0
        self.win_x.clear()
        self.win_y.clear()
        self.moved_away = False   # 清 0 后视为在原点

    def _height_mm(self):
        try:
            h = float(self.height_var.get())
            return h if h > 0 else DEFAULT_HEIGHT_MM
        except ValueError:
            return DEFAULT_HEIGHT_MM

    def _quality_min(self):
        """读取界面的 valid 门限(0~255)"""
        try:
            q = int(float(self.qmin_var.get()))
            return min(max(q, 0), 255)
        except (ValueError, AttributeError):
            return FLOW_QUALITY_MIN

    def _loop_thresh(self):
        """读取界面的闭环吸附阈值(mm)"""
        try:
            t = float(self.loop_th_var.get())
            return t if t > 0 else LOOP_THRESH_MM
        except (ValueError, AttributeError):
            return LOOP_THRESH_MM

    def _loop_closure(self):
        """闭环校正：把当前位置吸附回原点(可选同时归零航向)，返回吸附前残差(mm)。"""
        res = math.hypot(self.wx, self.wy)
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.wx = 0.0
        self.wy = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        if self.reset_yaw_var.get():
            self.yaw_corr = 0.0
        self.last_res_mm = res
        if res > self.max_res_mm:
            self.max_res_mm = res
        self.loop_count += 1
        self.moved_away = False
        self._update_loop_info()
        return res

    def _manual_loop(self):
        """手动"原位校正"：确认车已在原点时立刻吸附。"""
        r = self._loop_closure()
        self.status_var.set("原位校正: 吸附回原点 (残差 %.1f mm)" % r)

    def _update_loop_info(self):
        self.loop_info_var.set("上次残差 %.1f mm  |  最大 %.1f mm  |  次数 %d"
                               % (self.last_res_mm, self.max_res_mm, self.loop_count))

    def _theta_rad(self):
        """当前航向角(rad)。无有效陀螺数据时返回 0（退化为机体系）"""
        if self.yaw_last is None or (time.time() - self.yaw_stamp) * 1000 > IMU_STALE_MS:
            return 0.0, False
        sign = -1 if self.yaw_rev_var.get() else YAW_SIGN
        try:
            off = float(self.offset_var.get())
        except ValueError:
            off = 0.0
        deg = self.yaw_corr * sign + off
        return math.radians(deg), True

    # ---------------- 定时刷新 ----------------
    def _poll(self):
        height = self._height_mm()
        now = time.time()

        # 1) 吸收陀螺数据：逐样本求增量(解卷绕)，扣除漂移率后再积分
        while True:
            try:
                yaw = self.imu_queue.get_nowait()
            except queue.Empty:
                break
            t = time.time()
            if self.yaw_last is not None:
                dyaw = ((yaw - self.yaw_last + 180.0) % 360.0) - 180.0   # 解卷绕到 ±180
                dt = t - self.yaw_last_t
                if dt > 1e-4:
                    rate = dyaw / dt                                     # 瞬时 yaw 角速度(°/s)
                    self.yaw_rate_dps = 0.3 * rate + 0.7 * self.yaw_rate_dps
                    if self.zupt_var.get():
                        self.yaw_corr += dyaw - self.yaw_drift_dps * dt   # 扣掉漂移率再积分
                    else:
                        self.yaw_corr += dyaw
                    if self.is_static:                                   # 静止时才估计漂移率
                        self.yaw_drift_dps = BIAS_EMA_ALPHA * rate + (1 - BIAS_EMA_ALPHA) * self.yaw_drift_dps
            self.yaw_last = yaw
            self.yaw_last_t = t
            self.yaw_stamp = t

        # 2) 处理光流帧（质量门控 -> 中值滤波 -> 静止判定 -> 零偏扣除 -> 坐标累加）
        theta, theta_ok = self._theta_rad()
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        use_zupt = self.zupt_var.get()
        q_min = self._quality_min()

        while True:
            try:
                x, y, ts, valid, ver = self.flow_queue.get_nowait()
            except queue.Empty:
                break

            self.total_frames += 1

            # --- ① 质量门控：valid 太低的帧不可信，整帧丢弃 ---
            if valid < q_min:
                self.rej_frames += 1
                continue

            # --- ② 去尖尖：只把"明显偏离中值"的帧替换掉，其余帧保留原值 ---
            #   关键：不能整体取中值！光流积分是整数计数，慢速运动常输出 0,0,1,0,1... 这种抖动序列，
            #   整体取中值会把它们全部归零，导致慢速位移被抹掉。这里仅在超过阈值时替换。
            self.win_x.append(x)
            self.win_y.append(y)
            med_x = sorted(self.win_x)[len(self.win_x) // 2]
            med_y = sorted(self.win_y)[len(self.win_y) // 2]
            if abs(x - med_x) > OUTLIER_TH_RAW:
                self.spike_frames += 1
                fx_med = med_x
            else:
                fx_med = x
            if abs(y - med_y) > OUTLIER_TH_RAW:
                self.spike_frames += 1
                fy_med = med_y
            else:
                fy_med = y

            # --- 静止判定（滞回：静止需连续 STATIC_FRAMES 帧安静，运动需连续 MOVE_FRAMES 帧有动作）---
            quiet = (abs(fx_med) <= STATIC_FLOW_TH_RAW and abs(fy_med) <= STATIC_FLOW_TH_RAW
                     and abs(self.yaw_rate_dps) <= STATIC_YAW_RATE_TH_DPS)
            if quiet:
                self.quiet_count += 1
                self.move_count = 0
            else:
                self.move_count += 1
                self.quiet_count = 0
            if not self.is_static and self.quiet_count >= STATIC_FRAMES:
                self.is_static = True
            elif self.is_static and self.move_count >= MOVE_FRAMES:
                self.is_static = False

            # --- 静止且本帧安静时才在线估计光流零偏(避免启动瞬间/过渡帧污染估计) ---
            if use_zupt and self.is_static and quiet:
                self.flow_bias_x = (1 - BIAS_EMA_ALPHA) * self.flow_bias_x + BIAS_EMA_ALPHA * fx_med
                self.flow_bias_y = (1 - BIAS_EMA_ALPHA) * self.flow_bias_y + BIAS_EMA_ALPHA * fy_med

            fx = fx_med - (self.flow_bias_x if use_zupt else 0.0)
            fy = fy_med - (self.flow_bias_y if use_zupt else 0.0)

            # 换算：rad = raw/10000 ; mm = rad * height
            dx_mm = (fx / FLOW_SCALE) * height
            dy_mm = (fy / FLOW_SCALE) * height

            dt_s = (ts / 1e6) if ts > 0 else 0.005

            # 瞬时速度(mm/s) + 置信度加权平滑（低 valid 帧对速度贡献更小）
            vx = dx_mm / dt_s if dt_s > 0 else 0.0
            vy = dy_mm / dt_s if dt_s > 0 else 0.0
            a = SPEED_EMA_ALPHA * (valid / 255.0)
            self.vel_x = a * vx + (1 - a) * self.vel_x
            self.vel_y = a * vy + (1 - a) * self.vel_y

            # 仅在非静止时累加；ZUPT 生效且判定为静止时冻结累加(零速修正)
            if not (use_zupt and self.is_static):
                # 机体系累计
                self.acc_x += dx_mm
                self.acc_y += dy_mm
                # 世界系：把机体系增量旋转到世界系后累加
                self.wx += dx_mm * cos_t - dy_mm * sin_t
                self.wy += dx_mm * sin_t + dy_mm * cos_t

            self.cur = dict(x=x, y=y, ts=ts, valid=valid, ver=ver,
                            fmed_x=fx_med, fmed_y=fy_med, dx=dx_mm, dy=dy_mm)
            self.ts_stamps.append(now)

        # 清理 1s 之前的帧时间戳
        while self.ts_stamps and now - self.ts_stamps[0] > 1.0:
            self.ts_stamps.popleft()

        # 2.5) 闭环校正：走远后重新回到原点附近并停住 -> 自动吸附回 (0,0)
        dist = math.hypot(self.wx, self.wy)
        th = self._loop_thresh()
        if dist > th * 2.0:
            self.moved_away = True
        if (self.loop_auto_var.get() and self.moved_away and self.is_static
                and 0.0 < dist <= th):
            r = self._loop_closure()
            self.status_var.set("闭环校正: 残差 %.1f mm，已吸附回原点" % r)

        # 3) 更新界面
        c = self.cur
        self.lb["raw_x"].config(text=str(c["x"]))
        self.lb["raw_y"].config(text=str(c["y"]))
        self.lb["raw_ts"].config(text="%d us" % c["ts"])
        self.lb["raw_valid"].config(text=str(c["valid"]))
        self.lb["fmed_x"].config(text=str(c["fmed_x"]))
        self.lb["fmed_y"].config(text=str(c["fmed_y"]))
        self.lb["dx"].config(text="%+.3f" % c["dx"])
        self.lb["dy"].config(text="%+.3f" % c["dy"])
        self.lb["vx"].config(text="%+.2f" % self.vel_x)
        self.lb["vy"].config(text="%+.2f" % self.vel_y)

        self.lb["acc_x"].config(text="%+.3f" % self.acc_x)
        self.lb["acc_y"].config(text="%+.3f" % self.acc_y)
        self.lb["acc_d"].config(text="%.3f" % math.hypot(self.acc_x, self.acc_y))

        self.lb["wx"].config(text="%+.3f" % self.wx)
        self.lb["wy"].config(text="%+.3f" % self.wy)
        self.lb["wd"].config(text="%.3f" % math.hypot(self.wx, self.wy))
        self.lb["theta"].config(text="%+.1f" % math.degrees(theta))
        self.lb["motion"].config(text=("静止" if self.is_static else "运动"),
                                 foreground=("#0a6" if self.is_static else "#a30"))
        self.lb["fps"].config(text="%d" % len(self.ts_stamps))

        bad = self.flow_reader.parser.bad if self.flow_reader else 0
        self.lb["cnt"].config(text="%d / %d" % (self.total_frames, bad))
        self.lb["qcnt"].config(text="%d / %d" % (self.rej_frames, self.spike_frames))

        if not theta_ok:
            self.yaw_var.set("θ --.-°  (无陀螺)")
        else:
            self.yaw_var.set("θ %+.1f°" % math.degrees(theta))

        # ZUPT 状态、零偏与死区信息
        if not use_zupt:
            self.zupt_info_var.set("ZUPT: 已关闭 (不冻结累加，无死区)")
        else:
            fps = max(len(self.ts_stamps), 1)
            dead = STATIC_FLOW_TH_RAW / FLOW_SCALE * height * fps   # 死区速度(mm/s)
            self.zupt_info_var.set(
                "ZUPT: %s  | 光流零偏=(%+.1f, %+.1f) raw  | yaw 漂移 %+.3f °/s  | 死区 ≈ %.1f mm/s"
                % ("静止中" if self.is_static else "运动中",
                   self.flow_bias_x, self.flow_bias_y, self.yaw_drift_dps, dead))

        self.root.after(50, self._poll)

    def on_close(self):
        self._close_flow()
        self._close_imu()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = FlowMonitorApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
