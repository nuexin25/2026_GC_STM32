#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VELROUTE 速度模式跑图：串口在线调参小工具

用法（COM8 = 日志/命令口，115200 8N1）：
  python vr_tune.py --cmd "VRTELE=50" --cmd "VRRUN=1" --listen 8
  python vr_tune.py --cmd "VRCFG A=500" --cmd "VRDUMP"
  python vr_tune.py --cmd "VRSTOP"

只做“发几条命令 + 收一段时间日志”，不做自动爬山（参数由人/助手看着数据决定）。
--listen 期间收到的行会原样打印，并按需过滤保存。
"""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("缺少 pyserial：pip install pyserial")
    sys.exit(2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM8")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--cmd", action="append", default=[], help="要发的命令（可多次）")
    ap.add_argument("--listen", type=float, default=3.0, help="发完后听多少秒")
    ap.add_argument("--filter", default="", help="只打印包含该子串的行（空=全部）")
    ap.add_argument("--out", default="", help="把收到的行追加写到文件")
    ap.add_argument("--stop", action="store_true",
                    help="听够 --listen 秒后自动发 VRSTOP（跑车时必带，别让车自己跑到底）")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    time.sleep(0.3)
    ser.reset_input_buffer()

    for c in args.cmd:
        ser.write((c + "\r\n").encode("ascii"))
        ser.flush()
        time.sleep(0.12)          # 命令之间留点间隔，避免冲掉

    t0 = time.time()
    lines = []
    while (time.time() - t0) < args.listen:
        raw = ser.readline()
        if not raw:
            continue
        try:
            s = raw.decode("utf-8", "replace").rstrip()
        except Exception:
            continue
        lines.append(s)
        if (not args.filter) or (args.filter in s):
            print(s)

    if args.stop:
        ser.write(b"VRSTOP\r\n")
        ser.flush()
        print("--- sent VRSTOP ---")

    if args.out:
        with open(args.out, "a", encoding="utf-8") as f:
            for s in lines:
                f.write(s + "\n")

    ser.close()


if __name__ == "__main__":
    main()
