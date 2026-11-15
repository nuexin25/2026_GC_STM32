#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NX-CAR 对位 PID **自动调参**（一条指令启动小车 + 在线改参数 + 爬山找最优）

【它做什么】
  1) 通过 UART5 发 `ALN src=SHOT` -> 板子自动进 ALNT 页、手臂去观察位、开跑对位；
     **不需要按任何键**，也不用人守着。
  2) 每轮开始前用 `ALNCFG src=... kp=.. ki=..` 在线改 PID（RAM，立即生效，不用重编译/烧录）；
  3) 抓 `[ATN]` 调参日志，算"用时 / 过冲 / 稳态残差 / 换向次数 / 终值像素"，算一个代价；
  4) **坐标下降爬山**：逐个参数试 ±步长，变好就收，步长逐轮减半；
  5) 结束后打印**最优 9 个参数**，并写成两种形式：
       · 一条 `ALNCFG ...` 命令（还想继续在线跑就直接发）
       · `bsp_align.h` 里的 `#define ALIGN_<X>_*`（**固化用，最终要抄进代码**）
     同时写 `tools\\best_params.txt`。

【接线】
  USB-TTL  RX <- 板子 UART5_TX（日志）
  USB-TTL  TX -> 板子 UART5_RX（命令；**这根必须有，否则脚本发不出去**）
  GND 共地，115200 8N1。

【用法】
  python tools\\align_autotune.py --port COM8 --src SHOT
  python tools\\align_autotune.py --port COM8 --src SHOT --rounds 3
  python tools\\align_autotune.py --port COM8 --src SHOT --dry      # 只跑一轮看指标
  python tools\\align_autotune.py --port COM8 --src RCP  --rounds 2

【安全】
  · 对位会让小车移动，跑之前把车放到该数据源能看到的位置、周围留空；
  · 脚本超时会自动发 `ALN STOP`，退出（Ctrl+C）也会发；
  · 想立刻停：按板子 KEY5，或关掉脚本（会发 STOP）。

【依赖】pip install pyserial
"""
import argparse
import os
import re
import sys
import time

try:
    import serial
except ImportError:
    print("缺少 pyserial：请先执行  pip install pyserial")
    sys.exit(2)

from align_tune import (Run, RE_DATA, RE_ARMOK, RE_WARNARM, RE_RUN,
                        RE_DONE, RE_FAIL, RE_FAILMSG)

# ===================== 固件回包解析 =====================
RE_ALNCFG = re.compile(
    r"\[TUNE\] alncfg src=(\w+) kp=(-?\d+) ki=(-?\d+) kd=(-?\d+) rpm=(-?\d+) "
    r"wheel=(-?\d+) ykp=(-?\d+) yki=(-?\d+) ykd=(-?\d+) yrpm=(-?\d+)")
RE_ALNSTART = re.compile(r"\[TUNE\] aln start src=(\w+)")
RE_ALNERR = re.compile(r"\[TUNE\] err (.*)")

SRC_ID = {"RCP": 0, "SHOT": 1, "SHOT2": 2}
CFG_KEYS = ["kp", "ki", "kd", "rpmMax", "wheelMax", "ykp", "yki", "ykd", "yrpmMax"]

# ===================== 爬山计划 =====================
# (显示名, cfg 键, 首轮步长, 下限, 上限)
PARAMS = [
    ("KP", "kp", 0.040, 0.030, 0.600),
    ("KI", "ki", 0.060, 0.000, 0.600),
    ("KD", "kd", 0.006, 0.000, 0.100),
    ("RPM_MAX", "rpmMax", 4.0, 8.0, 40.0),
]

# 平行度（旋转轴）单独一组：只有 --tune-yaw 时才参与爬山
PARAMS_YAW = [
    ("YAW_KP", "ykp", 0.050, 0.020, 0.800),
    ("YAW_KD", "ykd", 0.005, 0.000, 0.100),
    ("YAW_KI", "yki", 0.030, 0.000, 0.300),
]


def clip(v, lo, hi):
    return max(lo, min(hi, v))


def fmt_cfg(c):
    return ("kp=%.3f ki=%.3f kd=%.3f rpm=%.1f wheel=%d ykp=%.3f yki=%.3f ykd=%.3f yrpm=%.1f"
            % (c["kp"], c["ki"], c["kd"], c["rpmMax"], c["wheelMax"],
               c["ykp"], c["yki"], c["ykd"], c["yrpmMax"]))


def cfg_to_cmd(src, c):
    """转成固件 ALNCFG 命令（整数定标：增益×1000、rpm×10）"""
    return ("ALNCFG src=%s kp=%d ki=%d kd=%d rpm=%d wheel=%d "
            "ykp=%d yki=%d ykd=%d yrpm=%d" % (
                src, int(round(c["kp"] * 1000)), int(round(c["ki"] * 1000)),
                int(round(c["kd"] * 1000)), int(round(c["rpmMax"] * 10)),
                int(c["wheelMax"]), int(round(c["ykp"] * 1000)),
                int(round(c["yki"] * 1000)), int(round(c["ykd"] * 1000)),
                int(round(c["yrpmMax"] * 10))))


def score(run):
    """越小越好：失败重罚；成功后 = 用时 + 过冲 + 残差 + 振荡 + 终值像素 + 平行度"""
    m = run.metrics()
    if m["result"] != "DONE":
        return 1e6
    f, r = m["fwd"], m["right"]
    s = float(m["dur_ms"])
    s += 300.0 * max(f["overshoot"], r["overshoot"])
    s += 150.0 * max(f["resid"], r["resid"])
    s += 40.0 * max(f["signchg"], r["signchg"])
    s += 50.0 * max(abs(m["final_px"][0]), abs(m["final_px"][1]))
    s += 20.0 * abs(m["q_final"])          # 0.1°：5 = 0.5° -> 100 分
    return s


class Board(object):
    """串口 + 命令 + 日志"""

    def __init__(self, port, baud, log_path, echo=False):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.fp = open(log_path, "a", encoding="utf-8")
        self.buf = ""
        self.echo = echo
        self.infra_fail = 0

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass
        try:
            self.fp.close()
        except Exception:
            pass

    def send(self, cmd):
        self.ser.write((cmd + "\n").encode("ascii", "ignore"))
        self.ser.flush()

    def pump(self, seconds, stop=None):
        """读 seconds 秒；stop(line) 返回 True 提前结束。返回收到的行"""
        lines = []
        t0 = time.time()
        while (time.time() - t0) < seconds:
            data = self.ser.read(4096)
            if not data:
                continue
            self.buf += data.decode("utf-8", "replace")
            while "\n" in self.buf:
                line, self.buf = self.buf.split("\n", 1)
                line = line.strip("\r")
                if not line:
                    continue
                lines.append(line)
                self.fp.write(line + "\n")
                self.fp.flush()
                if self.echo:
                    print("   | " + line)
                if stop is not None and stop(line):
                    return lines
        return lines

    def wait_reply(self, pattern, timeout=2.0, src=None):
        """等一条匹配的回包，返回 match 对象或 None"""
        box = {"m": None}

        def stop(line):
            m = pattern.search(line)
            if m is not None:
                if src is None or m.group(1) == src:
                    box["m"] = m
                    return True
            return False

        self.pump(timeout, stop)
        return box["m"]

    def dump_cfg(self, tries=12):
        """读三套参数，返回 {src: cfg dict}。
        ★实测 PC->板 UART5 RX 误码严重（同一条命令约 1/5 能完整到达），所以这里重发。"""
        end_re = re.compile(r"\[TUNE\] alndump end")

        for _ in range(tries):
            self.send("ALNDUMP")
            lines = self.pump(0.8, lambda ln: end_re.search(ln) is not None)
            out = {}
            for line in lines:
                m = RE_ALNCFG.search(line)
                if m:
                    out[m.group(1)] = {
                        "kp": int(m.group(2)) / 1000.0,
                        "ki": int(m.group(3)) / 1000.0,
                        "kd": int(m.group(4)) / 1000.0,
                        "rpmMax": int(m.group(5)) / 10.0,
                        "wheelMax": int(m.group(6)),
                        "ykp": int(m.group(7)) / 1000.0,
                        "yki": int(m.group(8)) / 1000.0,
                        "ykd": int(m.group(9)) / 1000.0,
                        "yrpmMax": int(m.group(10)) / 10.0,
                    }
            if out and end_re.search("\n".join(lines)):
                return out
        return None

    def set_cfg(self, src, cfg, tries=10):
        """下发一组参数，返回固件实际生效值（被夹取时 != 请求值）；带重发"""
        for _ in range(tries):
            self.send(cfg_to_cmd(src, cfg))
            m = self.wait_reply(RE_ALNCFG, 0.8, src)
            if m is not None:
                return {
                    "kp": int(m.group(2)) / 1000.0, "ki": int(m.group(3)) / 1000.0,
                    "kd": int(m.group(4)) / 1000.0, "rpmMax": int(m.group(5)) / 10.0,
                    "wheelMax": int(m.group(6)), "ykp": int(m.group(7)) / 1000.0,
                    "yki": int(m.group(8)) / 1000.0, "ykd": int(m.group(9)) / 1000.0,
                    "yrpmMax": int(m.group(10)) / 10.0,
                }
        return None

    def start_align(self, src, tries=15):
        """发启动命令，直到看到 `[ATN] cmd start`（或 TUNE 回包）为止。

        ⚠ 不能用"重发到有回复"的写法：重复的 ALN 会把已经跑起来的对位**重启**。
        所以判据用板子自己打的 `[ATN] cmd start src=..`（命令确实被解析了才会出现）。"""
        for _ in range(tries):
            self.send("ALN src=%s" % src)
            lines = self.pump(1.0, lambda ln: ("[ATN] cmd start" in ln) or
                                              (RE_ALNSTART.search(ln) is not None))
            for line in lines:
                if "[ATN] cmd start" in line or RE_ALNSTART.search(line):
                    return True
        return False

    def stop_align(self):
        # 停止是幂等的，多按几次，确保这条命中也到达
        for _ in range(4):
            self.send("ALN STOP")
            if self.wait_reply(re.compile(r"\[TUNE\] aln stop"), 0.3) is not None:
                break

    def run_once(self, src, cfg, timeout_s, verbose=False):
        """改参数 -> 启动 -> 等到 DONE/FAIL -> 返回 Run（已填好样本）"""
        got = self.set_cfg(src, cfg)
        if got is None:
            print("   !! 参数没设进去（命令通道不通？）")
            return None

        # align_tune.Run 的 cfg 用大写键（RPM_MAX 等），这里做一次映射
        run_cfg = {
            "KP": cfg["kp"], "KI": cfg["ki"], "KD": cfg["kd"],
            "RPM_MAX": cfg["rpmMax"], "WHEEL_MAX": cfg["wheelMax"],
            "YAW_KP": cfg["ykp"], "YAW_KI": cfg["yki"],
            "YAW_KD": cfg["ykd"], "YAW_RPM_MAX": cfg["yrpmMax"],
        }
        run = Run(src, run_cfg, time.time())

        def on_line(line):
            if RE_ARMOK.search(line):
                print("     . 手臂到观察位")
            elif RE_WARNARM.search(line):
                print("     ! 手臂动作被拒，直接等数据")
            elif RE_RUN.search(line):
                print("     . 数据有效，开跑")
            m = RE_DATA.search(line)
            if m:
                run.samples.append({
                    "t": int(m.group(1)), "st": int(m.group(2)), "f": int(m.group(3)),
                    "dt": int(m.group(4)),
                    "px": (int(m.group(5)), int(m.group(6))),
                    "mm": (int(m.group(7)), int(m.group(8))),
                    "q": int(m.group(9)),
                    "u": (int(m.group(10)), int(m.group(11)), int(m.group(12))),
                    "w": int(m.group(13)), "db": int(m.group(14)),
                })
                run.deadband = int(m.group(14))
                if verbose and (len(run.samples) % 5 == 0):
                    print("     t=%5d px=%3d,%3d mm=%4d,%4d u=%4d,%4d q=%3d db=%d" % (
                        run.samples[-1]["t"], run.samples[-1]["px"][0],
                        run.samples[-1]["px"][1], run.samples[-1]["mm"][0],
                        run.samples[-1]["mm"][1], run.samples[-1]["u"][0],
                        run.samples[-1]["u"][1], run.samples[-1]["q"], run.deadband))
            mm = RE_DONE.search(line)
            if mm:
                run.result = "DONE"
                run.dur_ms = int(mm.group(1))
                run.frames = int(mm.group(2))
                run.iters = int(mm.group(3))
                run.final_px = (int(mm.group(4)), int(mm.group(5)))
                return True
            mm = RE_FAIL.search(line)
            if mm:
                run.result = "FAIL"
                run.dur_ms = int(mm.group(1))
                run.frames = int(mm.group(2))
                run.iters = int(mm.group(3))
                run.final_px = (int(mm.group(4)), int(mm.group(5)))
                return True
            mm = RE_FAILMSG.search(line)
            if mm and "[ATN]" in line:
                run.result = "FAIL"
                run.reason = mm.group(1).strip()
                return True
            return False

        print("   >>> ALN src=%s  %s" % (src, fmt_cfg(cfg)))
        if not self.start_align(src):
            print("   !! 没收到 [TUNE] aln start：UART5 RX 可能没接（USB-TTL TX -> 板 UART5_RX）")
            return None

        self.pump(timeout_s, on_line)
        if run.result == "?":
            self.stop_align()
            run.result = "FAIL"
            run.reason = "timeout %ds (stopped)" % timeout_s
        elif run.result == "FAIL" and not run.dur_ms:
            run.dur_ms = run.samples[-1]["t"] if run.samples else 0
        return run


def print_metrics(run):
    m = run.metrics()
    f, r = m["fwd"], m["right"]
    print("     %-4s %5.1fs  fwd: start=%4d final=%3d peak=%3d ovs=%3d sc=%d sat=%.0f%% resid=%.1f | "
          "right: final=%3d ovs=%3d | px=%d,%d q=%d(peak %d) | %s" % (
              m["result"], m["dur_ms"] / 1000.0, f["start"], f["final"], f["peak"],
              f["overshoot"], f["signchg"], f["sat"] * 100, f["resid"],
              r["final"], r["overshoot"], m["final_px"][0], m["final_px"][1],
              m["q_final"], m["q_peak"], m["reason"]))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description="NX-CAR 对位 PID 自动调参（指令启动 + 在线改参 + 爬山）")
    ap.add_argument("--port", default="COM8")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--src", default="SHOT", choices=["RCP", "SHOT", "SHOT2"])
    ap.add_argument("--rounds", type=int, default=2, help="爬山轮数（步长每轮减半）")
    ap.add_argument("--run-timeout", type=float, default=90.0, help="单次对位超时秒")
    ap.add_argument("--dry", action="store_true", help="只跑一轮基线，不调参")
    ap.add_argument("--tune-yaw", action="store_true",
                    help="同时调平行度（旋转轴 YAW_KP/KD/KI）；默认只调平移 4 项")
    ap.add_argument("--echo", action="store_true", help="打印全部串口原始行")
    ap.add_argument("--log", default=os.path.join(here, "autotune_log.txt"))
    ap.add_argument("--out", default=os.path.join(here, "best_params.txt"))
    args = ap.parse_args()

    src = args.src
    best = None
    best_score = None
    board = None

    try:
        board = Board(args.port, args.baud, args.log, echo=args.echo)
    except Exception as exc:
        print("!! 打开 %s 失败: %s" % (args.port, exc))
        print("   - 串口助手是否还占着？先关掉；或换 --port COMx")
        return 1

    print("串口 %s @%d 已打开；日志 %s" % (args.port, args.baud, args.log))

    try:
        # ---- 0) 验证命令通道（UART5 RX 有没有接） ----
        allcfg = board.dump_cfg()
        if allcfg is None:
            print("!! 收不到 ALNDUMP 回包。检查：")
            print("   · USB-TTL 的 TX 是否接到板子 UART5_RX（只接 RX 只能看日志、发不出命令）")
            print("   · 波特率 115200、GND 共地")
            print("   · 固件是否已烧录带 ALN/ALNCFG 命令的版本")
            return 1
        if src not in allcfg:
            print("!! 回包里没有 src=%s，实际有：%s" % (src, list(allcfg.keys())))
            return 1

        best = dict(allcfg[src])
        print("板上当前 %s: %s" % (src, fmt_cfg(best)))
        print("提示：对位会让小车移动；Ctrl+C 会发 ALN STOP。\n")

        # ---- 1) 基线 ----
        print("=== 基线 ===")
        run = board.run_once(src, best, args.run_timeout, verbose=True)
        if run is None:
            return 1
        print_metrics(run)
        best_score = score(run)
        if run.result != "DONE" and ("no valid data" in run.reason or "arm not ready" in run.reason):
            print("!! 失败原因不在 PID（%s）—— 先修视觉/手臂，自动调参已停止。" % run.reason)
            return 1
        print("基线代价 = %.1f" % best_score)

        if args.dry:
            print("\n(--dry：不继续爬山)")
        else:
            # ---- 2) 坐标下降 ----
            params = list(PARAMS) + (list(PARAMS_YAW) if args.tune_yaw else [])
            steps = {name: step for (name, key, step, _lo, _hi) in params}
            for rnd in range(args.rounds):
                print("\n=== 第 %d 轮（步长 %s）===" % (
                    rnd + 1, " ".join("%s=%.3f" % (k, v) for k, v in steps.items())))
                for (name, key, _step, lo, hi) in params:
                    for sign in (1, -1):
                        cand = dict(best)
                        cand[key] = round(clip(cand[key] + sign * steps[name], lo, hi), 4)
                        if abs(cand[key] - best[key]) < 1e-9:
                            continue
                        print("\n-- 试 %s=%.4f（当前代价 %.1f）" % (name, cand[key], best_score))

                        run = board.run_once(src, cand, args.run_timeout)
                        if run is None:
                            continue
                        print_metrics(run)
                        sc = score(run)
                        if run.result != "DONE":
                            if ("no valid data" in run.reason) or ("arm not ready" in run.reason):
                                board.infra_fail += 1
                                if board.infra_fail >= 2:
                                    print("!! 连续基础设施失败（视觉/手臂），停止自动调参")
                                    raise KeyboardInterrupt
                            print("   -> 失败，不收")
                            continue
                        board.infra_fail = 0
                        if sc < best_score - 1.0:
                            best, best_score = cand, sc
                            print("   -> 采纳（代价 %.1f）" % sc)
                            break
                        print("   -> 变差（代价 %.1f），回退" % sc)
                for k in steps:
                    steps[k] *= 0.5

    except KeyboardInterrupt:
        print("\n(中断)")
    finally:
        if board is not None:
            try:
                board.stop_align()
            except Exception:
                pass
            board.close()

    if best is None:
        print("没有可用结果")
        return 1

    # ---- 3) 输出最优 ----
    print("\n================ 最优 ================")
    print("src      = %s" % src)
    print("参数     : %s" % fmt_cfg(best))
    print("代价     : %.1f" % best_score)
    print("\n# 还想继续在线跑（不用重编译）：")
    print(cfg_to_cmd(src, best))
    print("\n# 固化到 nx_car/nx_Drivers/Inc/bsp_align.h：")
    pre = {"RCP": "ALIGN_RCP", "SHOT": "ALIGN_SHOT", "SHOT2": "ALIGN_SHOT2"}[src]
    print("#define %s_KP           %.3ff" % (pre, best["kp"]))
    print("#define %s_KI           %.3ff" % (pre, best["ki"]))
    print("#define %s_KD           %.3ff" % (pre, best["kd"]))
    print("#define %s_RPM_MAX      %.1ff" % (pre, best["rpmMax"]))
    print("#define %s_WHEEL_MAX    %d" % (pre, best["wheelMax"]))
    print("#define %s_YAW_KP       %.3ff" % (pre, best["ykp"]))
    print("#define %s_YAW_KI       %.3ff" % (pre, best["yki"]))
    print("#define %s_YAW_KD       %.3ff" % (pre, best["ykd"]))
    print("#define %s_YAW_RPM_MAX  %.1ff" % (pre, best["yrpmMax"]))

    with open(args.out, "w", encoding="utf-8") as fp:
        fp.write("# NX-CAR 对位自动调参最优（src=%s, 代价=%.1f）\n" % (src, best_score))
        fp.write("%s\n\n" % cfg_to_cmd(src, best))
        for k, v in best.items():
            fp.write("%s=%.4f\n" % (k, v))
    print("\n已写 %s" % args.out)
    print("注意：在线改的是 RAM，**掉电不保存**；确认效果后请把上面 #define 抄进 bsp_align.h 并重新烧录。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
