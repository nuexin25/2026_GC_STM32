# -*- coding: utf-8 -*-
"""
数据采集器：串口读取 + 协议解析

职责：
  1. 在后台线程逐行读取 STM32 调试串口（UART5，115200）；
  2. 每行按协议解析（JSON 结构化数据 / 兼容纯文本日志 [TAG] msg）；
  3. 串口断开自动重连；
  4. 支持运行时切换串口（网页选择 COM 口后调用 switch_port）；
  5. 解析结果通过回调交给上层（app.py 负责存库 + WebSocket 广播）。

数据协议（详见《网站功能说明.md》第五节）：
  - JSON 行：{"t":"类型","ts":..., ...字段}
  - 文本日志：[TAG] 正文  → 转成 {"t":"log","tag":TAG,"msg":正文}
"""
import json
import re
import threading
import time

import serial
import serial.tools.list_ports

import config


# 文本日志 tag 提取正则：[TAG] 正文
_TAG_RE = re.compile(r"^\[([A-Za-z0-9_]+)\]\s*(.*)$")


def parse_line(line):
    """
    解析一行采集数据。

    :param line: 原始一行文本（含换行符也可）
    :return: (rec_type, tag, sub, payload_dict)；无法解析返回 None
    """
    line = line.strip()
    if not line:
        return None

    # 1) 优先尝试 JSON 结构化数据
    try:
        obj = json.loads(line)
        if isinstance(obj, dict) and "t" in obj:
            t = obj.get("t", "raw")
            tag = obj.get("tag", "")
            sub = obj.get("sub", "")
            return t, tag, sub, obj
    except (json.JSONDecodeError, ValueError):
        pass

    # 2) 兼容纯文本日志：[TAG] 正文
    m = _TAG_RE.match(line)
    if m:
        tag = m.group(1)
        msg = m.group(2)
        return "log", tag, "", {"tag": tag, "msg": msg}

    # 3) 无法提取 tag 的纯文本，归为 RAW
    return "log", "RAW", "", {"tag": "RAW", "msg": line}


def extract_structured(tag, msg):
    """
    从文本日志提取结构化数据（状态概览/实时曲线需要 JSON 类型）。

    STM32 当前只发文本日志（如 [IMU] yaw=0.6），不发 JSON 结构化数据。
    本函数是"文本日志 → 结构化数据"的桥接层：按已知日志格式提取数值，
    生成对应的结构化类型，让状态概览/实时曲线也能展示数据。
    STM32 固件未来直接发 JSON 后，本桥接可保留（作为兜底）或移除。

    :param tag: 日志标签（如 IMU/RPi/QR/MOT）
    :param msg: 日志正文（如 "yaw=0.6 gz=0.0"）
    :return: [(rec_type, tag, sub, payload), ...] 额外结构化数据列表
    """
    result = []

    # IMU：yaw 角
    if tag == "IMU":
        m = re.search(r"yaw=(-?[\d.]+)", msg)
        if m:
            result.append(("imu", "", "", {"t": "imu", "yaw": float(m.group(1))}))

    # 树莓派视觉：格式 [RPi] SUB key=value key=value ...
    elif tag == "RPi":
        parts = msg.split()
        if not parts:
            return result
        sub = parts[0]
        kv = {}
        for p in parts[1:]:
            if "=" in p:
                k, v = p.split("=", 1)
                try:
                    # 含小数点转 float，否则转 int
                    kv[k] = float(v) if ("." in v) else int(v)
                except ValueError:
                    kv[k] = v
        payload = {"t": "vision", "sub": sub}
        payload.update(kv)
        result.append(("vision", "", sub, payload))

    # 任务码：格式 [QR] raw="134+123+314+231" ok/bad
    elif tag == "QR":
        m = re.search(r'raw="([^"]+)"', msg)
        if m:
            valid = 1 if "ok" in msg else 0
            result.append(("task", "", "", {
                "t": "task", "code": m.group(1), "valid": valid,
            }))

    # 电机转速：格式 [MOT] addr=1 rpm=500（转速周期上报，同时有 addr 和 rpm 才提取）
    elif tag == "MOT":
        m = re.search(r"addr=(\d+)", msg)
        rm = re.search(r"rpm=(-?\d+)", msg)
        if m and rm:
            addr = int(m.group(1))
            rpm = int(rm.group(1))
            result.append(("motor", "", "", {
                "t": "motor", "addr": addr, "rpm": rpm,
                "pulse": 0, "state": "ok",
            }))

    return result


def list_ports():
    """
    列出当前可用串口。

    :return: [{"port": "COM3", "desc": "USB-SERIAL CH340"}, ...]
    """
    result = []
    for p in serial.tools.list_ports.comports():
        if p.device:
            result.append({"port": p.device, "desc": p.description or ""})
    return result


class Collector:
    """
    采集器主类。

    :param on_data: 数据回调函数，签名 on_data(rec_type, tag, sub, payload_dict)
    """

    def __init__(self, on_data):
        self.on_data = on_data
        self.running = False
        self.serial = None
        self.source = "none"          # 数据源：serial / simulate / none
        self.simulate = False         # 模拟数据开关（默认关，网页按钮开启）
        self._thread = None
        self._lock = threading.Lock()
        self._target_port = None      # 用户指定串口（None=自动探测）
        self._wakeup = threading.Event()   # 唤醒主循环的 sleep（switch/set_simulate 用）

    # ------------------------------------------------------------------
    # 对外接口
    # ------------------------------------------------------------------
    def start(self):
        """启动采集线程（守护线程，随主进程退出自动结束）。"""
        if self.running:
            return
        self.running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        """停止采集线程。"""
        self.running = False
        self._close_serial()

    def switch_port(self, port):
        """
        切换串口（网页选择 COM 口后调用）。

        :param port: 串口号（如 "COM3"），传 None/"" 表示回到自动探测
        """
        with self._lock:
            self._target_port = port if port else None
        # 关闭当前串口，唤醒主循环按新目标端口重连
        self._close_serial()
        self._wakeup.set()
        print(f"[collector] 切换串口: {port if port else '自动探测'}")

    def set_simulate(self, enabled):
        """
        开启/关闭模拟数据（网页按钮调用）。

        :param enabled: True=开启模拟，False=关闭模拟回到串口
        """
        with self._lock:
            self.simulate = enabled
        # 关闭当前串口，唤醒主循环按新状态切换
        self._close_serial()
        self._wakeup.set()
        print(f"[collector] 模拟数据: {'开启' if enabled else '关闭'}")

    def get_status(self):
        """返回当前数据源状态字符串（供 /api/status）。"""
        return self.source

    # ------------------------------------------------------------------
    # 主循环：模拟数据 或 连接串口
    # ------------------------------------------------------------------
    def _run(self):
        """采集主循环：开启模拟则生成模拟数据；否则连接串口。"""
        while self.running:
            # 模拟数据模式
            if self.simulate:
                self.source = "simulate"
                self._simulate_loop()
                continue

            # 串口模式
            port = self._resolve_port()
            if port is None:
                self.source = "none"
                self._wait_reconnect()
                continue

            if self._open_serial(port):
                self.source = "serial"
                self._serial_loop()          # 串口循环（含重连）
            else:
                self.source = "none"
                self._wait_reconnect()

    def _wait_reconnect(self):
        """可中断的等待（switch_port/set_simulate 会唤醒它）。"""
        self._wakeup.wait(config.RECONNECT_INTERVAL)
        self._wakeup.clear()

    def _resolve_port(self):
        """
        解析串口号：用户指定优先；否则用 config.SERIAL_PORT；"auto" 时自动探测。
        自动探测优先 USB 串口，跳过蓝牙串口（蓝牙串口打开可能阻塞）。
        :return: 串口名字符串或 None
        """
        with self._lock:
            if self._target_port:
                return self._target_port
        if config.SERIAL_PORT and config.SERIAL_PORT.lower() != "auto":
            return config.SERIAL_PORT
        ports = list(serial.tools.list_ports.comports())
        # 优先 USB 串口（STM32 经 USB 转串口接入）
        for p in ports:
            if p.device and "usb" in (p.description or "").lower():
                return p.device
        # 退而求其次：第一个非蓝牙串口
        for p in ports:
            if p.device and "bluetooth" not in (p.description or "").lower():
                return p.device
        return None

    def _open_serial(self, port):
        """打开串口，成功返回 True。"""
        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=config.SERIAL_BAUD,
                timeout=config.SERIAL_TIMEOUT,
            )
            print(f"[collector] 串口已打开: {port} @ {config.SERIAL_BAUD}")
            return True
        except Exception as e:
            print(f"[collector] 串口打开失败 {port}: {e}")
            return False

    def _close_serial(self):
        """安全关闭串口。"""
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
            except Exception:
                pass
        self.serial = None

    # ------------------------------------------------------------------
    # 串口读取循环
    # ------------------------------------------------------------------
    def _serial_loop(self):
        """
        串口读取循环（逐行）。

        串口断开、被关闭或读异常时不崩溃：关闭串口 → 返回主循环重连。
        注意：except 用 Exception（而非仅 SerialException/OSError），因为
        set_simulate/switch_port 关闭串口时 read 可能抛 TypeError 等其它异常，
        若漏捕会导致采集线程崩溃。
        """
        buf = b""
        while self.running:
            try:
                s = self.serial
                if s is None:            # 串口已被外部关闭
                    return
                data = s.read(256)       # 非阻塞（timeout=0.1）
                if data:
                    buf += data
                    while b"\n" in buf:
                        line_bytes, buf = buf.split(b"\n", 1)
                        self._handle_line(line_bytes)
            except Exception as e:
                print(f"[collector] 串口读异常: {e}")
                self._close_serial()
                return
        self._close_serial()

    def _handle_line(self, line_bytes):
        """处理一行字节数据：解码 + 解析 + 回调。"""
        try:
            line = line_bytes.decode("utf-8", errors="ignore")
        except Exception:
            return
        parsed = parse_line(line)
        if parsed is None:
            return
        rec_type, tag, sub, payload = parsed
        self.on_data(rec_type, tag, sub, payload)

        # 文本日志额外提取结构化数据，让状态概览/实时曲线也有值
        if rec_type == "log" and tag != "RAW":
            for extra in extract_structured(tag, payload.get("msg", "")):
                self.on_data(*extra)

    # ------------------------------------------------------------------
    # 模拟数据源（网页「模拟数据」按钮开启后运行）
    # ------------------------------------------------------------------
    def _simulate_loop(self):
        """
        模拟数据生成循环：定时生成符合协议的数据，走完整「解析→存储→展示」链路。
        用于 STM32 未接入时的开发与演示。simulate 开关关闭时退出。
        """
        import math
        import random

        t0 = time.time()
        yaw = 0.0
        x = 1200.0
        y = 800.0
        motor_pulse = [0, 0, 0, 0]
        motor_rpm = [0, 0, 0, 0]
        stage_names = ["IDLE", "GO_QR", "SCANNING", "TO_RAW", "PICK",
                       "TO_ROUGH", "PLACE1", "TO_STORE", "PLACE2",
                       "PICK2", "STACK", "RETURN", "DONE"]
        log_idx = 0

        while self.running and self.simulate:
            t = time.time() - t0

            # IMU yaw：正弦波动
            yaw = 180.0 * math.sin(t * 0.5)
            self.on_data("imu", "", "", {"t": "imu", "yaw": round(yaw, 1)})

            # 定位：随机游走
            x += random.uniform(-20, 20)
            y += random.uniform(-20, 20)
            x = max(0, min(2400, x))
            y = max(0, min(2400, y))
            self.on_data("pose", "", "", {
                "t": "pose", "x": round(x, 1), "y": round(y, 1), "yaw": round(yaw, 1),
            })

            # 电机：4 轮转速波动 + 脉冲递增
            for addr in range(1, 5):
                motor_rpm[addr - 1] = random.randint(-120, 120)
                motor_pulse[addr - 1] += abs(motor_rpm[addr - 1])
                self.on_data("motor", "", "", {
                    "t": "motor", "addr": addr,
                    "rpm": motor_rpm[addr - 1],
                    "pulse": motor_pulse[addr - 1],
                    "state": "ok",
                })

            # 视觉数据：随机生成一类
            vis_type = random.choice(["OBS", "PLA", "NAV", "BOX", "TRN"])
            if vis_type == "OBS":
                self.on_data("vision", "", "OBS", {
                    "t": "vision", "sub": "OBS",
                    "a": round(random.uniform(-180, 180), 1),
                    "d": random.randint(0, 3000), "side": random.randint(0, 1),
                    "clear": random.randint(0, 1),
                })
            elif vis_type == "PLA":
                self.on_data("vision", "", "PLA", {
                    "t": "vision", "sub": "PLA", "ring": random.randint(1, 6),
                    "dx": random.randint(-100, 100), "dy": random.randint(-100, 100),
                    "conf": random.randint(0, 100),
                })
            elif vis_type == "NAV":
                self.on_data("vision", "", "NAV", {
                    "t": "vision", "sub": "NAV",
                    "x": round(x), "y": round(y), "yaw": round(yaw, 1),
                    "conf": random.randint(0, 100),
                })
            elif vis_type == "BOX":
                self.on_data("vision", "", "BOX", {
                    "t": "vision", "sub": "BOX",
                    "has": random.randint(0, 1), "color": random.randint(0, 4),
                })
            else:
                self.on_data("vision", "", "TRN", {
                    "t": "vision", "sub": "TRN",
                    "c1": random.randint(1, 4), "a1": random.randint(-180, 180),
                    "c2": random.randint(1, 4), "a2": random.randint(-180, 180),
                    "c3": random.randint(1, 4), "a3": random.randint(-180, 180),
                })

            # 任务阶段：周期性切换
            stage_id = int(t * 2) % len(stage_names)
            self.on_data("stage", "", "", {
                "t": "stage", "id": stage_id,
                "name": stage_names[stage_id],
                "code": "134+123+314+231",
            })

            # 日志：周期性
            log_idx += 1
            tags = ["IMU", "MOT", "CAN", "QR", "RPi", "CHS"]
            self.on_data("log", tags[log_idx % len(tags)], "", {
                "tag": tags[log_idx % len(tags)],
                "msg": "simulate log #%d" % log_idx,
            })

            time.sleep(config.SIM_INTERVAL_MS / 1000.0)
