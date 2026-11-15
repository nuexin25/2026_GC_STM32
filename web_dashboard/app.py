# -*- coding: utf-8 -*-
"""
nx_car 本地数据看板 —— Flask 主程序

职责：
  1. 启动后台串口采集线程（collector.Collector）；
  2. 提供 Web 服务（静态页面 + REST API）；
  3. 提供 WebSocket 实时推送（/ws）。

数据流向（全部本机）：
  STM32(UART5) → 采集线程 → 存 SQLite + WebSocket 广播 → 浏览器

启动：python app.py
访问：http://127.0.0.1:<WEB_PORT>
"""
import json
import threading
import time

from flask import Flask, request, jsonify, send_from_directory
from flask_sock import Sock

import config
import db
import collector

# Flask 应用：static_folder 指向 static 目录，静态资源走 /static/...
app = Flask(__name__, static_folder="static", static_url_path="/static")
sock = Sock(app)

# ---- WebSocket 客户端集合（线程安全） ----
_clients = set()
_clients_lock = threading.Lock()

# ---- 采集器实例（全局，供 /api/status 查询数据源） ----
_collector = None


# ----------------------------------------------------------------------
# WebSocket 广播
# ----------------------------------------------------------------------
def broadcast(record):
    """
    向所有已连接的浏览器推送一条数据记录。

    :param record: db.insert 返回的记录 dict（含 id/ts/type/tag/sub/payload）
    """
    msg = json.dumps(record, ensure_ascii=False)
    with _clients_lock:
        dead = []
        for ws in _clients:
            try:
                ws.send(msg)
            except Exception:
                dead.append(ws)
        for ws in dead:
            _clients.discard(ws)


# ----------------------------------------------------------------------
# 采集器数据回调
# ----------------------------------------------------------------------
def on_data(rec_type, tag, sub, payload):
    """
    采集器回调：收到一条数据 → 存库 + 广播。
    （本函数由采集线程调用，db.insert 内部每操作独立连接，线程安全）
    """
    try:
        record = db.insert(rec_type, payload, tag=tag, sub=sub)
        broadcast(record)
    except Exception as e:
        print(f"[app] 数据处理异常: {e}")


# ----------------------------------------------------------------------
# WebSocket 路由
# ----------------------------------------------------------------------
@sock.route("/ws")
def ws_handler(ws):
    """
    WebSocket 连接处理：连接建立后加入客户端集合，阻塞等待断开。
    """
    with _clients_lock:
        _clients.add(ws)
    try:
        while True:
            # receive() 阻塞；客户端断开时会抛异常或返回 None
            ws.receive()
    except Exception:
        pass
    finally:
        with _clients_lock:
            _clients.discard(ws)


# ----------------------------------------------------------------------
# REST API
# ----------------------------------------------------------------------
@app.route("/")
def index():
    """返回单页主页。"""
    return send_from_directory("static", "index.html")


@app.route("/api/history")
def api_history():
    """
    查询历史数据（供历史回放曲线）。

    查询参数：
        type  数据类型（log/imu/pose/motor/task/vision/stage）
        tag   日志 tag（可选）
        sub   视觉子类型（可选）
        start/end 时间戳毫秒（可选）
        limit 条数上限（默认 1000）
    """
    rec_type = request.args.get("type")
    tag = request.args.get("tag")
    sub = request.args.get("sub")
    start = request.args.get("start", type=int)
    end = request.args.get("end", type=int)
    limit = request.args.get("limit", default=1000, type=int)

    rows = db.query_history(
        rec_type=rec_type, tag=tag, sub=sub,
        start=start, end=end, limit=limit,
    )
    return jsonify({"ok": True, "count": len(rows), "data": rows})


@app.route("/api/logs")
def api_logs():
    """
    查询日志流（支持关键字/tag/时间范围）。

    查询参数：keyword、tag、start、end、limit
    """
    keyword = request.args.get("keyword")
    tag = request.args.get("tag")
    start = request.args.get("start", type=int)
    end = request.args.get("end", type=int)
    limit = request.args.get("limit", default=500, type=int)

    rows = db.query_logs(keyword=keyword, tag=tag, start=start, end=end, limit=limit)
    return jsonify({"ok": True, "count": len(rows), "data": rows})


@app.route("/api/tags")
def api_tags():
    """返回所有出现过的日志 tag（供过滤下拉）。"""
    tags = db.get_tags()
    return jsonify({"ok": True, "data": tags})


@app.route("/api/status")
def api_status():
    """返回数据源状态 + 最新数据摘要（供状态概览页）。"""
    source = _collector.get_status() if _collector else "none"
    summary = db.get_latest_summary()
    return jsonify({
        "ok": True,
        "source": source,           # serial / none
        "time": int(time.time() * 1000),
        "summary": summary,
    })


@app.route("/api/ports")
def api_ports():
    """返回当前可用串口列表（供前端 COM 口下拉选择）。"""
    ports = collector.list_ports()
    return jsonify({"ok": True, "data": ports})


@app.route("/api/select_port", methods=["POST"])
def api_select_port():
    """
    切换串口（网页选择 COM 口后调用）。

    body: {"port": "COM3"} 或 {"port": ""}（空 = 自动探测）
    """
    data = request.get_json(silent=True) or {}
    port = data.get("port", "")
    if _collector:
        _collector.switch_port(port)
    return jsonify({"ok": True, "port": port})


@app.route("/api/simulate", methods=["POST"])
def api_simulate():
    """
    开启/关闭模拟数据（网页「模拟数据」按钮调用）。

    body: {"enabled": true/false}
    """
    data = request.get_json(silent=True) or {}
    enabled = bool(data.get("enabled", False))
    if _collector:
        _collector.set_simulate(enabled)
    return jsonify({"ok": True, "enabled": enabled})


@app.route("/api/param", methods=["POST"])
def api_param():
    """
    赛题参数下发接口（预留）。

    本阶段不实现实际下发，仅返回"暂未开放"。
    """
    return jsonify({"ok": False, "msg": "暂未开放，待固件支持"})


# ----------------------------------------------------------------------
# 入口
# ----------------------------------------------------------------------
def main():
    """主入口：初始化数据库 → 启动采集线程 → 启动 Web 服务。"""
    global _collector

    # 确保 data 目录存在并建表
    db.init_db()

    # 启动采集器（后台守护线程）
    _collector = collector.Collector(on_data)
    _collector.start()

    print(f"[app] 数据看板启动: http://{config.WEB_HOST}:{config.WEB_PORT}")
    app.run(host=config.WEB_HOST, port=config.WEB_PORT, debug=False)


if __name__ == "__main__":
    main()
