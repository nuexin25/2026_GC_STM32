#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NX-CAR P7 自动调车脚本（配合固件 bsp_tune 的在线调参协议）

【它做什么】
  1) 通过**日志串口 UART5** 看日志（同一根 USB-TTL 的 RX 线也能发命令给板子）；
  2) 命令板子用小圈连跑 N 圈（`tableScalePct` + `lapRepeat` + `RUN`）；
  3) 从日志里解析每圈的**转角误差、落点误差、闭环表现**，算一个代价 cost；
  4) 用**爬山法（坐标下降）**逐个调参数（旋转刻度 / 转速 / 平移刻度），
     保留让 cost 下降的方向，步长逐轮减半，直到收敛或达到迭代上限；
  5) 打印最优参数表并写 `best_params.txt`（把它抄回 bsp_route.h 的宏即可固化）。

【硬件连接】
  USB-TTL  ↔  STM32
    TX     →   UART5_RX（板子收命令；**这根线必须有**，只接 TX 的话收不到命令）
    RX     →   UART5_TX（板子发日志）
    GND    →   GND
  波特率与固件一致（默认 115200）。

【依赖】  pip install pyserial

【例子】
  python auto_tune.py --port COM5                    # 全自动调参（小圈 40%、3 圈/轮）
  python auto_tune.py --port COM5 --dry              # 只跑一轮看看数据，不改参数
  python auto_tune.py --port COM5 --set rotVel=120   # 手动改参数后退出
  python auto_tune.py --port COM5 --dump             # 打印板子当前全部可调参数
"""
import argparse
import re
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    print("缺少 pyserial：请先执行  pip install pyserial")
    sys.exit(2)

# ============================ 代价函数权重 ============================
# 目标：90° 转弯准（deg 级）+ 直行落点准（mm 级）+ 不要频繁 giveup
W_TURN_RMS = 1.0        # 每 1° 转角 RMS 误差算 1 分
W_POS_MM = 0.02         # 每 1mm 落点误差算 0.02 分（100mm 偏差 = 2 分）
PEN_ABORT = 50.0        # 任何 ABORT / 超时罚 50 分

# 要自动调的参数：(名字, 起始值, 步长, 最小, 最大)
#   注意 rotVel 只往**小**试：转速越低打滑越少、转角越准（代价是慢）
TUNE_PLAN = [
    ("rotScaleNum", 100, 8,   80,  160),   # 旋转刻度：90° 转不够就加大
    ("rotVel",      150, -25, 50,  200),   # 原地转速度：只降不升
    ("distScaleNum", 100, 4,  85,  130),   # 平移刻度：走短了加大
]

# ============================ 日志解析 ============================
RE_LAP_END = re.compile(r"\[ROUTE\] lap end x=(-?\d+) y=(-?\d+) yaw=(-?\d+) lap=(\d+)/(\d+)")
RE_ALL_DONE = re.compile(r"\[ROUTE\] all done")
RE_ABORT = re.compile(r"\[ROUTE\] (abort|seg\d+ timeout)")
RE_SETOK = re.compile(r"\[TUNE\] set ([A-Za-z0-9_]+)=(\d+) \(req (-?\d+)")
RE_TURN = re.compile(r"\[ROUTE\] seg(\d+) turn done, yaw d=(-?\d+) \(cmd=(-?\d+)\)")
RE_POSERR = re.compile(r"\[ROUTE\] seg(\d+) pos err x=(-?\d+) y=(-?\d+)")
RE_LAPSTART = re.compile(r"\[ROUTE\] lap (\d+)/(\d+) start")


class Metrics(object):
    """一轮（1..N 圈）的统计结果"""

    def __init__(self):
        self.turn_err = []      # 每个转弯的角度误差（deg，带符号）
        self.pos_err = []       # 每次落点误差（mm，取 x/y 的最大绝对值）
        self.turns = 0
        self.laps = 0
        self.abort = False
        self.timed_out = False
        self.raw_lines = []

    def cost(self):
        c = 0.0
        if self.turn_err:
            rms = (sum(e * e for e in self.turn_err) / len(self.turn_err)) ** 0.5
            c += W_TURN_RMS * rms
        if self.pos_err:
            c += W_POS_MM * max(self.pos_err)
        if self.abort:
            c += PEN_ABORT
        if self.timed_out:
            c += PEN_ABORT
        if self.laps == 0:
            c += PEN_ABORT
        return c

    def turn_rms(self):
        if not self.turn_err:
            return float("nan")
        return (sum(e * e for e in self.turn_err) / len(self.turn_err)) ** 0.5

    def summary(self):
        return ("转弯RMS={:.2f}deg 最大转角={:.2f} 落点max={}mm 圈数={} 转弯次数={} {}").format(
            self.turn_rms(), max((abs(e) for e in self.turn_err), default=0.0),
            max(self.pos_err) if self.pos_err else "-", self.laps, self.turns,
            "ABORT!" if self.abort else ("TIMEOUT!" if self.timed_out else ""))


class Board(object):
    """串口 + 命令 + 日志收集"""

    def __init__(self, port, baud, echo=False):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.echo = echo
        self.buf = ""
        self.log_fp = None

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass
        if self.log_fp:
            self.log_fp.close()

    def set_log(self, path):
        self.log_fp = open(path, "w", encoding="utf-8")

    def send(self, cmd):
        self.ser.write((cmd + "\n").encode("ascii", "ignore"))
        self.ser.flush()

    def pump(self, seconds, stop_when=None):
        """读串口 seconds 秒；stop_when(lines) 返回 True 就提前结束。返回期间收到的行"""
        lines = []
        t0 = time.time()
        while (time.time() - t0) < seconds:
            data = self.ser.read(4096)
            if data:
                self.buf += data.decode("utf-8", "replace")
                while "\n" in self.buf:
                    line, self.buf = self.buf.split("\n", 1)
                    line = line.strip("\r")
                    if line:
                        lines.append(line)
                        if self.log_fp:
                            self.log_fp.write(line + "\n")
                        if self.echo:
                            print("   |", line)
            if stop_when and stop_when(lines):
                break
        return lines

    def wait_tune(self, timeout=2.0):
        """等一条 [TUNE] 回复（用来确认 RX 接对、固件在线）"""
        got = []
        self.send("DUMP")
        lines = self.pump(timeout, stop_when=lambda ls: any("[TUNE]" in x for x in ls))
        got += [x for x in lines if "[TUNE]" in x]
        return got

    def dump(self):
        out = []
        self.send("DUMP")
        lines = self.pump(1.5, stop_when=lambda ls: any("dump end" in x for x in ls))
        for x in lines:
            m = re.search(r"\[TUNE\] ([A-Za-z0-9_]+)=(\d+)", x)
            if m:
                out.append((m.group(1), int(m.group(2))))
        return out

    def set(self, name, value):
        """下发一个参数；返回板子**实际生效**的值（被范围夹取时 != 请求值）。

        固件会回一行 `[TUNE] set X=实际 (req 请求, range lo..hi)`，
        这里解析出来，被夹取时直接警告 —— 否则爬山法会误以为"参数已设成 X"，
        实际板子用的是别的值，白跑几轮。
        """
        self.send("{}={}".format(name, int(value)))
        lines = self.pump(0.4)
        for line in lines:
            m = RE_SETOK.search(line)
            if m and m.group(1) == name:
                got = int(m.group(2))
                if got != int(value):
                    print("    !! {} 被夹取: 请求 {} -> 实际 {}".format(name, value, got))
                return got
        print("    !! 未收到 {}={} 的 [TUNE] 回复（命令丢失？）".format(name, value))
        return None

    def apply(self, params):
        for k, v in params.items():
            self.set(k, v)

    def run_laps(self, laps, timeout_s):
        """发 RUN 并收日志到"最后一圈结束/整体完成"，返回 Metrics"""
        m = Metrics()
        done = {"n": 0}

        def stop_when(lines):
            for line in lines:
                if RE_ABORT.search(line):
                    m.abort = True
                    return True
                if RE_ALL_DONE.search(line):
                    return True
                mm = RE_LAP_END.search(line)
                if mm:
                    m.laps = int(mm.group(4))
                    if m.laps >= laps:
                        return True
            return False

        self.send("RUN")
        lines = self.pump(timeout_s, stop_when=stop_when)
        for line in lines:
            self._parse(line, m)
        if not (m.abort or m.laps >= laps or any(RE_ALL_DONE.search(x) for x in lines)):
            m.timed_out = True
        self.send("STOP")
        self.pump(0.5)
        return m

    def _parse(self, line, m):
        mm = RE_TURN.search(line)
        if mm:
            d = int(mm.group(2))
            c = int(mm.group(3))
            m.turn_err.append((d - c) / 10.0)      # 单位 0.1deg -> deg
            m.turns += 1
        mm = RE_POSERR.search(line)
        if mm:
            m.pos_err.append(max(abs(int(mm.group(2))), abs(int(mm.group(3)))))


def hill_climb(board, args):
    """坐标下降爬山：逐个参数试 起始±步长，保留变好的；步长减半后重复"""
    current = {name: start for (name, start, _s, _lo, _hi) in TUNE_PLAN}
    best = dict(current)
    best_cost = None

    print("\n=== 第 0 轮：基线 ===")
    m = run_once(board, best, args)
    best_cost = m.cost()
    print("cost={:.2f}  {}".format(best_cost, m.summary()))

    step_scales = [1.0, 0.5, 0.25]
    for si, sc in enumerate(step_scales):
        for (name, start, step, lo, hi) in TUNE_PLAN:
            if si == 0 and abs(step) <= 0:
                continue
            for direction in (1, -1):
                cand = dict(best)
                newv = best[name] + int(round(step * sc)) * direction
                if newv < lo or newv > hi:
                    continue
                cand[name] = newv
                print("\n--- 试 {}={} (cost 当前 {:.2f}) ---".format(name, newv, best_cost))
                m = run_once(board, cand, args)
                c = m.cost()
                print("cost={:.2f}  {}".format(c, m.summary()))
                if c < best_cost - 1e-6:
                    best_cost = c
                    best = dict(cand)
                    print(">>> 采纳 {}={}".format(name, newv))
                    break       # 该方向有效，进入下一个参数
        print("\n=== 第 {} 轮结束，最优 cost={:.2f} ===".format(si + 1, best_cost))
    return best, best_cost


def run_once(board, params, args):
    """设参数 -> 连跑 N 圈 -> 返回 Metrics"""
    board.send("DEFAULT" if args.default_each else "DUMP")   # DEFAULT 时先复位再套参数
    board.pump(0.4)
    p = dict(params)
    p["tableScalePct"] = args.small
    p["lapRepeat"] = args.laps
    board.apply(p)
    print("    参数: " + " ".join("{}={}".format(k, v) for k, v in sorted(p.items())))
    to = args.laps * args.lap_timeout
    return board.run_laps(args.laps, to)


def main():
    ap = argparse.ArgumentParser(description="NX-CAR P7 自动调车（配合 bsp_tune）")
    ap.add_argument("--port", required=True, help="日志串口，如 COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--laps", type=int, default=3, help="每轮跑几圈（默认 3）")
    ap.add_argument("--small", type=int, default=40, help="调车用小圈：整表缩放%%（默认 40）")
    ap.add_argument("--lap-timeout", type=float, default=70.0, help="单圈超时秒（默认 70）")
    ap.add_argument("--iters", type=int, default=3, help="爬山轮数（默认 3）")
    ap.add_argument("--dry", action="store_true", help="只跑一轮看数据，不改参数")
    ap.add_argument("--dump", action="store_true", help="打印板子当前参数后退出")
    ap.add_argument("--set", action="append", default=[], help="手动改参数，如 --set rotVel=120（可多次）")
    ap.add_argument("--default-each", action="store_true", help="每轮先 DEFAULT 复位再套参数（推荐）")
    ap.add_argument("--echo", action="store_true", help="把串口日志也打到屏幕")
    ap.add_argument("--log", default="auto_tune_log.txt", help="日志落盘文件名")
    args = ap.parse_args()

    board = Board(args.port, args.baud, echo=args.echo)
    board.set_log(args.log)
    print("串口 {} 已打开，日志同时写入 {}".format(args.port, args.log))

    # 0) 确认能收到命令回复（验证 UART5 的 RX 已接）
    got = board.wait_tune(2.0)
    if not got:
        print("\n!! 没有收到 [TUNE] 回复：")
        print("   - 最常见原因：USB-TTL 的 TX 没接到板子的 UART5_RX（要把 RX 也接上）")
        print("   - 确认波特率一致（默认 115200）、GND 共地")
        print("   - 仍可用 --dry 只读日志（命令发不出，但日志能看）")
        if not args.dry:
            board.close()
            return 1
    else:
        print("板子在线，参数个数：{}".format(len(board.dump())))

    if args.dump:
        for k, v in board.dump():
            print("  {}={}".format(k, v))
        board.close()
        return 0

    if args.set:
        for kv in args.set:
            k, _, v = kv.partition("=")
            board.set(k.strip(), int(v))
            print("  已设 {}={}".format(k.strip(), v))
        board.close()
        return 0

    best = None
    best_cost = None
    try:
        if args.dry:
            m = run_once(board, {n: s for (n, s, _st, _lo, _hi) in TUNE_PLAN}, args)
            print("\n基线: cost={:.2f}  {}".format(m.cost(), m.summary()))
        else:
            best, best_cost = hill_climb(board, args)
            print("\n================ 最优结果 ================")
            print("cost = {:.2f}".format(best_cost))
            print("参数（抄回 bsp_route.h 或下次用 --set 传入）：")
            for k, v in sorted(best.items()):
                print("  {}={}".format(k, v))
            with open("best_params.txt", "w", encoding="utf-8") as fp:
                fp.write("# NX-CAR P7 自动调参最优值（cost={:.2f}）\n".format(best_cost))
                fp.write("# 已运行参数（每轮已通过串口下发，掉电不保存）\n")
                for k, v in sorted(best.items()):
                    fp.write("{}={}\n".format(k, v))
            print("已写 best_params.txt")
    finally:
        board.send("STOP")
        board.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
