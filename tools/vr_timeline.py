#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 run_*.txt 里没有时间戳的日志行，用 [SYS] alive, t=NNN ms 做锚点，
按行号线性插值出近似时间，输出关键事件时间轴。"""
import re
import sys

path = sys.argv[1] if len(sys.argv) > 1 else "run_base.txt"
pat = sys.argv[2] if len(sys.argv) > 2 else r"^\[(TASK|TT|SP|SB|ALN|VR|MS)\]"

anchor = re.compile(r"\[SYS\] alive, t=(\d+) ms")
rows = []
with open(path, encoding="utf-8", errors="replace") as f:
    for line in f:
        rows.append(line.rstrip("\n"))

anchors = []
for i, s in enumerate(rows):
    m = anchor.search(s)
    if m:
        anchors.append((i, int(m.group(1))))

if len(anchors) < 2:
    print("no anchors")
    sys.exit(0)


def est(i):
    if i <= anchors[0][0]:
        return anchors[0][1]
    if i >= anchors[-1][0]:
        return anchors[-1][1]
    for k in range(len(anchors) - 1):
        i0, t0 = anchors[k]
        i1, t1 = anchors[k + 1]
        if i0 <= i <= i1:
            return t0 + (t1 - t0) * (i - i0) / max(1, (i1 - i0))
    return anchors[-1][1]


flt = re.compile(pat)
prev = None
for i, s in enumerate(rows):
    if not flt.search(s):
        continue
    t = est(i)
    d = 0 if prev is None else t - prev
    prev = t
    print(f"{t/1000:7.2f}s (+{d:7.0f}ms) {s}")
