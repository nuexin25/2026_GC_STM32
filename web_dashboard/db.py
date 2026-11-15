# -*- coding: utf-8 -*-
"""
SQLite 存取封装

统一存储所有采集到的数据（JSON 类型 + 文本日志），字段：
    id      自增主键
    ts      采集器本机时间戳（毫秒，int）
    type    数据类型：log / imu / pose / motor / task / vision / stage
    tag     日志 tag（如 IMU/CAN/MOT，仅 log 类型有值，其余为空字符串）
    sub     视觉子类型（OBS/TRN/PLA/NAV/BOX，仅 vision 类型有值）
    payload 原始 JSON 字符串（完整数据原文，绘图时按需解析）

线程安全：SQLite 默认 check_same_thread=False + 每操作独立连接，
配合采集线程和 Flask 主线程的并发访问。
"""
import sqlite3
import json
import time
import os

import config


def _connect():
    """建立数据库连接，自动创建 data 目录。"""
    os.makedirs(os.path.dirname(config.DB_PATH), exist_ok=True)
    conn = sqlite3.connect(config.DB_PATH, check_same_thread=False)
    conn.row_factory = sqlite3.Row  # 查询结果可用 dict 方式访问
    return conn


def init_db():
    """初始化数据库：建表 + 建索引 + 按保留策略清理。"""
    conn = _connect()
    try:
        conn.execute("""
            CREATE TABLE IF NOT EXISTS telemetry (
                id      INTEGER PRIMARY KEY AUTOINCREMENT,
                ts      INTEGER NOT NULL,
                type    TEXT    NOT NULL,
                tag     TEXT    DEFAULT '',
                sub     TEXT    DEFAULT '',
                payload TEXT    NOT NULL
            )
        """)
        # ts 索引：历史回放按时间范围查询的关键
        conn.execute("CREATE INDEX IF NOT EXISTS idx_ts ON telemetry(ts)")
        conn.execute("CREATE INDEX IF NOT EXISTS idx_type ON telemetry(type)")
        conn.commit()
    finally:
        conn.close()
    # 启动时按保留策略清理过期数据
    cleanup()


def insert(rec_type, payload, tag="", sub=""):
    """
    插入一条数据。

    :param rec_type: 数据类型（log/imu/pose/motor/task/vision/stage）
    :param payload:  数据字典（会被 json.dumps 后存 payload 字段）
    :param tag:      日志 tag（可选）
    :param sub:      视觉子类型（可选）
    :return: 插入后的记录 dict（含 id 和 ts，供 WebSocket 广播）
    """
    ts = int(time.time() * 1000)  # 本机毫秒时间戳
    payload_str = json.dumps(payload, ensure_ascii=False)
    conn = _connect()
    try:
        cur = conn.execute(
            "INSERT INTO telemetry (ts, type, tag, sub, payload) VALUES (?,?,?,?,?)",
            (ts, rec_type, tag, sub, payload_str),
        )
        conn.commit()
        rid = cur.lastrowid
    finally:
        conn.close()
    return {"id": rid, "ts": ts, "type": rec_type, "tag": tag,
            "sub": sub, "payload": payload}


def query_history(rec_type=None, tag=None, sub=None, start=None, end=None, limit=1000):
    """
    按条件查询历史数据（用于历史回放曲线）。

    :param rec_type: 按类型过滤（可选，None=不过滤）
    :param tag:      按日志 tag 过滤（可选）
    :param sub:      按视觉子类型过滤（可选）
    :param start:    起始时间戳毫秒（可选）
    :param end:      结束时间戳毫秒（可选）
    :param limit:    最多返回条数（默认 1000）
    :return: 记录 dict 列表（已解析 payload 为 dict）
    """
    sql = "SELECT id, ts, type, tag, sub, payload FROM telemetry WHERE 1=1"
    args = []
    if rec_type:
        sql += " AND type=?"
        args.append(rec_type)
    if tag:
        sql += " AND tag=?"
        args.append(tag)
    if sub:
        sql += " AND sub=?"
        args.append(sub)
    if start is not None:
        sql += " AND ts>=?"
        args.append(start)
    if end is not None:
        sql += " AND ts<=?"
        args.append(end)
    sql += " ORDER BY ts ASC LIMIT ?"
    args.append(limit)

    conn = _connect()
    try:
        rows = conn.execute(sql, args).fetchall()
    finally:
        conn.close()

    result = []
    for r in rows:
        item = dict(r)
        try:
            item["payload"] = json.loads(item["payload"])
        except (json.JSONDecodeError, TypeError):
            item["payload"] = {}
        result.append(item)
    return result


def query_logs(keyword=None, tag=None, start=None, end=None, limit=500):
    """
    查询日志流（支持关键字/tag/时间范围过滤）。

    :return: 记录 dict 列表
    """
    sql = "SELECT id, ts, type, tag, payload FROM telemetry WHERE type='log'"
    args = []
    if tag:
        sql += " AND tag=?"
        args.append(tag)
    if keyword:
        sql += " AND payload LIKE ?"
        args.append("%" + keyword + "%")
    if start is not None:
        sql += " AND ts>=?"
        args.append(start)
    if end is not None:
        sql += " AND ts<=?"
        args.append(end)
    sql += " ORDER BY ts DESC LIMIT ?"
    args.append(limit)

    conn = _connect()
    try:
        rows = conn.execute(sql, args).fetchall()
    finally:
        conn.close()

    result = []
    for r in rows:
        item = dict(r)
        try:
            item["payload"] = json.loads(item["payload"])
        except (json.JSONDecodeError, TypeError):
            item["payload"] = {}
        result.append(item)
    return result


def get_tags():
    """返回所有出现过的日志 tag（供前端过滤下拉）。"""
    conn = _connect()
    try:
        rows = conn.execute(
            "SELECT DISTINCT tag FROM telemetry WHERE type='log' AND tag!='' ORDER BY tag"
        ).fetchall()
    finally:
        conn.close()
    return [r["tag"] for r in rows]


def get_latest_summary():
    """返回各类型最新一条数据摘要（供状态概览页）。"""
    types = ["imu", "pose", "motor", "task", "vision", "stage"]
    summary = {}
    conn = _connect()
    try:
        for t in types:
            row = conn.execute(
                "SELECT ts, payload FROM telemetry WHERE type=? ORDER BY ts DESC LIMIT 1",
                (t,),
            ).fetchone()
            if row:
                try:
                    summary[t] = {"ts": row["ts"], "data": json.loads(row["payload"])}
                except (json.JSONDecodeError, TypeError):
                    summary[t] = {"ts": row["ts"], "data": {}}
    finally:
        conn.close()
    return summary


def cleanup():
    """按保留天数清理过期数据（RETENTION_DAYS=0 表示不清理）。"""
    if config.RETENTION_DAYS <= 0:
        return
    cutoff = int(time.time() * 1000) - config.RETENTION_DAYS * 24 * 3600 * 1000
    conn = _connect()
    try:
        conn.execute("DELETE FROM telemetry WHERE ts < ?", (cutoff,))
        conn.commit()
    finally:
        conn.close()
