#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NX-CAR 对位（P23/P24）调参助手：抓 UART5 日志 -> 逐轮算指标 -> 给下一组 PID 建议

【它做什么】
  1) 打开日志串口（COM8 / 115200），持续抓 `[ATN]` / `[ALN]` 行；
  2) 每跑完一次对位（`[ATN] DONE` 或 `[ATN] FAIL`），自动算这一轮的指标：
       起始误差 / 峰值 / 终值 / 换向次数 / 过冲量 / 输出饱和率 / 稳态残差 / 堵转拍数 / 帧间隔；
  3) 按《调参手册》P24 的判据给出**下一组**建议值，并打一行可机器读取的 `SUGGEST`；
  4) 把每轮结果追加到 `align_runs.json`，并按数据源（RCP/SHOT/SHOT2）排出最优的一轮。

【它不做什么】
  不往板子发命令（对位 PID 没有在线调参协议）。参数改法：把 `SUGGEST` 里那 9 个值
  抄进 `nx_Drivers/Inc/bsp_align.h` 对应那一套（`ALIGN_RCP_*` / `ALIGN_SHOT_*` / `ALIGN_SHOT2_*`），
  重新编译烧录，再跑一轮。脚本会跨轮次记住历史，最后给出每个源的最优参数。

【用法】
  python tools\\align_tune.py --port COM8                 # 一直抓，按 Ctrl+C 结束
  python tools\\align_tune.py --port COM8 --seconds 120   # 抓 120 秒后自动结束
  python tools\\align_tune.py --port COM8 --echo          # 顺带把原始日志打到屏幕
  python tools\\align_tune.py --replay tools\\align_log.txt  # 复盘已有日志，不开串口

【依赖】pip install pyserial
"""
import argparse
import json
import os
import re
import sys
import time

try:
    import serial
except ImportError:
    print("缺少 pyserial：请先执行  pip install pyserial")
    sys.exit(2)

BAUD = 115200

# ===================== 固件默认值（只在没抓到 `[ALN] pid cfg` 时兜底） =====================
DEFAULT_CFG = {
    "KP": 0.120, "KI": 0.008, "KD": 0.010, "RPM_MAX": 25.0, "WHEEL_MAX": 40,
    "YAW_KP": 0.150, "YAW_KI": 0.010, "YAW_KD": 0.010, "YAW_RPM_MAX": 12.0,
}

# ===================== 日志解析 =====================
RE_DATA = re.compile(
    r"\[ATN\] t=(\d+) st=(\d+) f=(\d+) dt=(\d+) px=(-?\d+),(-?\d+) "
    r"mm=(-?\d+),(-?\d+) q=(-?\d+) u=(-?\d+),(-?\d+),(-?\d+) w=(-?\d+) db=(\d+)")
RE_START = re.compile(r"\[ATN\] start src=(\w+)")
RE_RUN = re.compile(r"\[ATN\] run src=(\w+)")
RE_ARMOK = re.compile(r"\[ATN\] arm ready")
RE_WARNARM = re.compile(r"\[ATN\] WARN arm observe refused")
RE_DONE = re.compile(r"\[ATN\] DONE (\d+) ms frames=(\d+) iters=(\d+) px=(-?\d+),(-?\d+) pass=(\d+)")
RE_FAIL = re.compile(r"\[ATN\] FAIL (\d+) ms frames=(\d+) iters=(\d+) px=(-?\d+),(-?\d+) q=(-?\d+)")
RE_FAILMSG = re.compile(r"\[ATN\] FAIL[:\s](.*)")
RE_CFG = re.compile(
    r"\[ALN\] pid cfg src=(\w+) kp=(-?\d+) ki=(-?\d+) kd=(-?\d+) rpm=(-?\d+) "
    r"wheel=(-?\d+) yaw=(-?\d+),(-?\d+),(-?\d+),(-?\d+)")
RE_ATN = re.compile(r"\[ATN\] (.*)")
RE_ALN = re.compile(r"\[ALN\] (.*)")


# ===================== 数据结构 =====================
class Run(object):
    """一次对位（从 start 到 DONE/FAIL）"""

    def __init__(self, src, cfg, wall):
        self.src = src
        self.cfg = dict(cfg) if cfg else dict(DEFAULT_CFG)
        self.wall = wall
        self.samples = []
        self.result = "?"          # DONE / FAIL
        self.reason = ""
        self.dur_ms = 0
        self.frames = 0
        self.iters = 0
        self.final_px = (0, 0)
        self.deadband = 0
        self.arm_ok = False
        self.arm_warn = False

    # ---------- 单轴指标 ----------
    def axis(self, idx, uidx):
        """idx: 0=前进 1=横移；uidx: 2=uF 3=uR（samples 里 u 元组下标）"""
        e = [s["mm"][idx] for s in self.samples]
        u = [s["u"][uidx] for s in self.samples]
        n = len(e)
        out = {"n": n, "start": 0, "peak": 0, "final": 0, "signchg": 0,
               "overshoot": 0, "resid": 0, "sat": 0.0, "stall": 0}
        if n == 0:
            return out
        out["start"] = e[0]
        out["peak"] = max(abs(x) for x in e)
        out["final"] = e[-1]
        for i in range(1, n):
            if abs(e[i - 1]) >= 2 and abs(e[i]) >= 2 and e[i] * e[i - 1] < 0:
                out["signchg"] += 1
        # 过冲：首次反向之后的最大偏离
        if out["signchg"] > 0:
            for i in range(1, n):
                if abs(e[i - 1]) >= 2 and abs(e[i]) >= 2 and e[i] * e[i - 1] < 0:
                    out["overshoot"] = max(abs(x) for x in e[i:])
                    break
        tail = e[-3:] if n >= 3 else e
        out["resid"] = sum(abs(x) for x in tail) / float(len(tail))
        lim = self.cfg["RPM_MAX"] * 100.0 * 0.95
        big = [i for i in range(n) if abs(e[i]) > 8]
        if big:
            out["sat"] = sum(1 for i in big if abs(u[i]) >= lim) / float(len(big))
        out["stall"] = sum(1 for i in range(n)
                           if self.samples[i]["w"] == 0 and abs(u[i]) >= 100)
        return out

    def metrics(self):
        f = self.axis(0, 0)
        r = self.axis(1, 1)
        dts = [s["dt"] for s in self.samples if s["dt"] > 0]
        # 平行度（0.1°）：同一视觉帧会重复打印，按 f 去重后再统计
        qs = []
        lastf = -1
        for s in self.samples:
            if s["f"] != lastf:
                qs.append(s["q"])
                lastf = s["f"]
        return {
            "src": self.src, "result": self.result, "reason": self.reason,
            "dur_ms": self.dur_ms, "frames": self.frames, "iters": self.iters,
            "final_px": self.final_px, "deadband": self.deadband,
            "dt_avg": (sum(dts) / len(dts)) if dts else 0,
            "dt_max": max(dts) if dts else 0,
            "q_final": qs[-1] if qs else 0,
            "q_peak": max(abs(q) for q in qs) if qs else 0,
            "fwd": f, "right": r, "cfg": self.cfg,
        }

    def score(self):
        """越小越好：失败重罚；成功后比用时 + 精度"""
        m = self.metrics()
        if m["result"] != "DONE":
            return 1e6
        s = m["dur_ms"]
        s += 200.0 * max(m["fwd"]["overshoot"], m["right"]["overshoot"])
        s += 100.0 * max(m["fwd"]["resid"], m["right"]["resid"])
        s += 10.0 * max(m["fwd"]["signchg"], m["right"]["signchg"])
        return s


# ===================== 建议引擎 =====================
def clip(v, lo, hi):
    return max(lo, min(hi, v))


def suggest(run):
    """按《调参手册》P24 判据给出下一组 9 个参数 + 中文说明"""
    cfg = dict(run.cfg)
    m = run.metrics()
    notes = []
    f, r = m["fwd"], m["right"]

    if m["result"] != "DONE":
        if "no valid data" in m["reason"] or "arm not ready" in m["reason"]:
            notes.append("失败原因不在 PID（%s），先修视觉/手臂" % m["reason"])
        else:
            notes.append("本轮失败：%s" % (m["reason"] or "未知"))
            if max(f["final"], r["final"]) > 2:
                cfg["KI"] = round(clip(cfg["KI"] * 1.5, 0.002, 0.05), 4)
                notes.append("残差未消 -> KI x1.5")
    else:
        ov = max(f["overshoot"], r["overshoot"])
        sc = max(f["signchg"], r["signchg"])
        sat = max(f["sat"], r["sat"])
        resid = max(f["resid"], r["resid"])
        stall = f["stall"] + r["stall"]

        if sc >= 3:
            cfg["KP"] = round(clip(cfg["KP"] * 0.80, 0.03, 0.60), 4)
            cfg["KD"] = round(clip(cfg["KD"] * 1.30, 0.002, 0.05), 4)
            notes.append("来回摆 %d 次 -> KP x0.8, KD x1.3" % sc)
        elif ov > 8 and sc >= 2:
            if cfg["KD"] < 0.03:
                cfg["KD"] = round(clip(cfg["KD"] * 1.80, 0.002, 0.05), 4)
                notes.append("过冲 %dmm -> KD x1.8" % ov)
            else:
                cfg["KP"] = round(clip(cfg["KP"] * 0.85, 0.03, 0.60), 4)
                notes.append("过冲 %dmm 且 KD 已大 -> KP x0.85" % ov)

        if sat >= 0.50 and m["dur_ms"] > 1200:
            cfg["RPM_MAX"] = round(clip(cfg["RPM_MAX"] * 1.25, 10.0, 40.0), 1)
            cfg["WHEEL_MAX"] = int(clip(cfg["WHEEL_MAX"] * 1.15, 20, 60))
            notes.append("输出饱和 %.0f%% -> RPM_MAX x1.25" % (sat * 100))

        if resid >= 3 and sat < 0.5:
            cfg["KI"] = round(clip(cfg["KI"] * 1.60, 0.002, 0.05), 4)
            notes.append("稳态残差 %.1fmm -> KI x1.6" % resid)

        if stall >= 3:
            notes.append("有 %d 拍'有输出但不转' -> 查 VEL_SLEW_RPM/堵转" % stall)

        if m["dt_avg"] > 200:
            notes.append("视觉帧间隔 %.0fms 偏大 -> 先提帧率，否则 PID 必摆" % m["dt_avg"])

        if not notes:
            notes.append("指标已可接受，保持本组")

    return cfg, notes


# ===================== 输出 =====================
def print_run(run):
    m = run.metrics()
    f, r = m["fwd"], m["right"]
    print("")
    print("=== RUN %s src=%-5s result=%-4s dur=%dms frames=%d iters=%d "
          "dt(avg/max)=%d/%d db=%dpx final_px=%d,%d ===" % (
              time.strftime("%H:%M:%S"), m["src"], m["result"], m["dur_ms"],
              m["frames"], m["iters"], m["dt_avg"], m["dt_max"], m["deadband"],
              m["final_px"][0], m["final_px"][1]))
    print("axis   start  final  peak  signchg  overshoot  sat%   resid  stall")
    print("FWD    %5d  %5d  %4d  %7d  %9d  %4.0f  %5.1f  %5d" % (
        f["start"], f["final"], f["peak"], f["signchg"], f["overshoot"],
        f["sat"] * 100, f["resid"], f["stall"]))
    print("RIGHT  %5d  %5d  %4d  %7d  %9d  %4.0f  %5.1f  %5d" % (
        r["start"], r["final"], r["peak"], r["signchg"], r["overshoot"],
        r["sat"] * 100, r["resid"], r["stall"]))
    if m["reason"]:
        print("reason: %s" % m["reason"])

    cfg, notes = suggest(run)
    print("ADVICE: " + " | ".join(notes))
    print("SUGGEST src=%s KP=%.4f KI=%.4f KD=%.4f RPM_MAX=%.1f WHEEL_MAX=%d "
          "YAW_KP=%.4f YAW_KI=%.4f YAW_KD=%.4f YAW_RPM_MAX=%.1f" % (
              m["src"], cfg["KP"], cfg["KI"], cfg["KD"], cfg["RPM_MAX"],
              cfg["WHEEL_MAX"], cfg["YAW_KP"], cfg["YAW_KI"], cfg["YAW_KD"],
              cfg["YAW_RPM_MAX"]))
    print("  当前板上: KP=%.4f KI=%.4f KD=%.4f RPM_MAX=%.1f WHEEL_MAX=%d" % (
        run.cfg["KP"], run.cfg["KI"], run.cfg["KD"],
        run.cfg["RPM_MAX"], run.cfg["WHEEL_MAX"]))
    return cfg, notes


class Tracer(object):
    """日志状态机：把串口行变成 Run 对象"""

    def __init__(self, out_json):
        self.cur = None
        self.cfg = None
        self.history = []
        self.out_json = out_json
        self.cur_src = "RCP"

    def set_cfg_from_line(self, line):
        mm = RE_CFG.search(line)
        if mm:
            self.cfg = {
                "KP": int(mm.group(2)) / 1000.0, "KI": int(mm.group(3)) / 1000.0,
                "KD": int(mm.group(4)) / 1000.0, "RPM_MAX": float(mm.group(5)),
                "WHEEL_MAX": int(mm.group(6)),
                "YAW_KP": int(mm.group(7)) / 1000.0, "YAW_KI": int(mm.group(8)) / 1000.0,
                "YAW_KD": int(mm.group(9)) / 1000.0, "YAW_RPM_MAX": float(mm.group(10)),
            }
            print("[CFG] 板上参数 src=%s %s" % (mm.group(1), self.cfg))
            self.cur_src = mm.group(1)

    def feed(self, line):
        if not line:
            return
        self.set_cfg_from_line(line)
        mm = RE_START.search(line)
        if mm:
            self.cur = Run(mm.group(1), self.cfg, time.time())
            self.cur_src = mm.group(1)
            print("\n> 开始对位 src=%s%s" % (mm.group(1),
                  "（手臂先去观察位）" if "observe pos" in line else ""))
            return
        if self.cur is None:
            return
        if RE_ARMOK.search(line):
            self.cur.arm_ok = True
            print("> 手臂到观察位，等视觉数据")
            return
        if RE_WARNARM.search(line):
            self.cur.arm_warn = True
            print("> WARN 手臂动作被拒（正忙），直接等数据")
            return
        mm = RE_RUN.search(line)
        if mm:
            print("> 视觉数据有效，对位开跑 src=%s" % mm.group(1))
            return
        mm = RE_DATA.search(line)
        if mm:
            self.cur.samples.append({
                "t": int(mm.group(1)), "st": int(mm.group(2)), "f": int(mm.group(3)),
                "dt": int(mm.group(4)),
                "px": (int(mm.group(5)), int(mm.group(6))),
                "mm": (int(mm.group(7)), int(mm.group(8))),
                "q": int(mm.group(9)),
                "u": (int(mm.group(10)), int(mm.group(11)), int(mm.group(12))),
                "w": int(mm.group(13)), "db": int(mm.group(14)),
            })
            self.cur.deadband = int(mm.group(14))
            return
        mm = RE_DONE.search(line)
        if mm:
            self._finish("DONE", "", int(mm.group(1)), int(mm.group(2)), int(mm.group(3)),
                         (int(mm.group(4)), int(mm.group(5))))
            return
        mm = RE_FAIL.search(line)
        if mm:
            self._finish("FAIL", "", int(mm.group(1)), int(mm.group(2)), int(mm.group(3)),
                         (int(mm.group(4)), int(mm.group(5))))
            return
        mm = RE_FAILMSG.search(line)
        if mm and "FAIL" in line:
            if self.cur.result == "?":
                self._finish("FAIL", mm.group(1).strip(), 0, 0, 0, (0, 0))

    def _finish(self, result, reason, dur, frames, iters, px):
        if self.cur is None or self.cur.result != "?":
            return
        self.cur.result = result
        self.cur.reason = reason
        self.cur.dur_ms = dur or (self.cur.samples[-1]["t"] if self.cur.samples else 0)
        self.cur.frames = frames or len(self.cur.samples)
        self.cur.iters = iters
        self.cur.final_px = px
        print_run(self.cur)
        self.history.append(self.cur)
        self.save()
        best = self.best(self.cur.src)
        if best is not None:
            c = best.cfg
            print(">> %s 目前最优：%dms 过冲=%dmm 残差=%.1fmm -> KP=%.4f KI=%.4f KD=%.4f "
                  "RPM_MAX=%.1f WHEEL_MAX=%d" % (
                      self.cur.src, best.dur_ms,
                      max(best.metrics()["fwd"]["overshoot"],
                          best.metrics()["right"]["overshoot"]),
                      max(best.metrics()["fwd"]["resid"],
                          best.metrics()["right"]["resid"]),
                      c["KP"], c["KI"], c["KD"], c["RPM_MAX"], c["WHEEL_MAX"]))
        self.cur = None

    def best(self, src):
        cands = [x for x in self.history if x.src == src and x.result == "DONE"]
        if not cands:
            return None
        return min(cands, key=lambda x: x.score())

    def save(self):
        try:
            rows = []
            for x in self.history:
                d = x.metrics()
                d["wall"] = x.wall
                d["score"] = round(x.score(), 1)
                rows.append(d)
            with open(self.out_json, "w", encoding="utf-8") as fp:
                json.dump(rows, fp, ensure_ascii=False, indent=1)
        except Exception as exc:
            print("!! 写 %s 失败: %s" % (self.out_json, exc))

    def summary(self):
        print("\n================ 汇总 ================")
        for src in ("RCP", "SHOT", "SHOT2"):
            n = len([x for x in self.history if x.src == src])
            ok = len([x for x in self.history if x.src == src and x.result == "DONE"])
            b = self.best(src)
            if n == 0:
                print("%-6s 未跑" % src)
            elif b is None:
                print("%-6s 跑 %d 次，0 次成功" % (src, n))
            else:
                c = b.cfg
                m = b.metrics()
                print("%-6s 跑 %d 次，成功 %d 次；最优 %dms 过冲=%dmm 残差=%.1fmm -> "
                      "KP=%.4f KI=%.4f KD=%.4f RPM_MAX=%.1f WHEEL_MAX=%d YAW=%.4f/%.4f/%.4f/%.1f" % (
                          src, n, ok, b.dur_ms,
                          max(m["fwd"]["overshoot"], m["right"]["overshoot"]),
                          max(m["fwd"]["resid"], m["right"]["resid"]),
                          c["KP"], c["KI"], c["KD"], c["RPM_MAX"], c["WHEEL_MAX"],
                          c["YAW_KP"], c["YAW_KI"], c["YAW_KD"], c["YAW_RPM_MAX"]))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description="NX-CAR 对位调参助手（抓日志 + 逐轮建议）")
    ap.add_argument("--port", default="COM8", help="日志串口，默认 COM8")
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--seconds", type=float, default=0, help="抓多少秒后结束；0=一直抓")
    ap.add_argument("--echo", action="store_true", help="把原始串口行也打出来")
    ap.add_argument("--log", default=os.path.join(here, "align_log.txt"), help="原始日志落盘")
    ap.add_argument("--json", default=os.path.join(here, "align_runs.json"), help="每轮指标落盘")
    ap.add_argument("--replay", default="", help="复盘已有日志（不开串口）")
    args = ap.parse_args()

    tracer = Tracer(args.json)

    if args.replay:
        with open(args.replay, "r", encoding="utf-8", errors="replace") as fp:
            for line in fp:
                line = line.strip()
                if args.echo and line:
                    print("| " + line)
                tracer.feed(line)
        tracer.summary()
        return 0

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except Exception as exc:
        print("!! 打开 %s 失败: %s" % (args.port, exc))
        print("   - 串口助手/UartAssist 是否还占着该端口？（先关掉）")
        print("   - 设备管理器里确认端口号，或用 --port COMx 指定")
        return 1

    print("串口 %s 已打开 @%d，日志写入 %s" % (args.port, args.baud, args.log))
    print("现在去 ALNT 页：KEY1/KEY2 选源 -> KEY3 启动。Ctrl+C 结束。")
    print("（脚本只读日志，不向板子发命令；参数由人写入 bsp_align.h 后重新烧录）\n")

    buf = ""
    t0 = time.time()
    fp = open(args.log, "a", encoding="utf-8")
    try:
        while True:
            if args.seconds > 0 and (time.time() - t0) >= args.seconds:
                break
            data = ser.read(4096)
            if not data:
                continue
            buf += data.decode("utf-8", "replace")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip("\r")
                if not line:
                    continue
                fp.write(line + "\n")
                fp.flush()
                if args.echo:
                    print("| " + line)
                tracer.feed(line)
    except KeyboardInterrupt:
        print("\n(用户中断)")
    finally:
        fp.close()
        try:
            ser.close()
        except Exception:
            pass

    tracer.summary()
    print("\n指标已写入 %s；原始日志 %s" % (args.json, args.log))
    return 0


if __name__ == "__main__":
    sys.exit(main())
