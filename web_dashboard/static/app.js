/* nx_car 数据看板 —— 前端逻辑
 *
 * 功能：
 *   1. 标签页切换
 *   2. WebSocket 连接 + 自动重连
 *   3. 实时数据接收 → 状态概览 + 实时曲线 + 日志流
 *   4. 历史回放（选类型 + 时间范围 → 曲线 + CSV 导出）
 */

// ==================== 全局状态 ====================
const CHART_MAX_POINTS = 300;   // 实时曲线保留点数（与后端 config 一致）

// 实时数据缓冲（各类型保留最近 N 点）
const buffers = {
  yaw: [],      // {x: ts, y: yaw}
  pose: [],     // {x: px, y: py}
  motor: { 1: [], 2: [], 3: [], 4: [] },  // 每轮 {x: ts, y: rpm}
};

let ws = null;                 // WebSocket 实例
let wsConnected = false;
let bootTs = Date.now();       // 页面加载时刻（用于运行时间估算）

// 日志相关
let logPaused = false;
let logLines = [];             // 当前显示日志缓存（最多 500 条）
const LOG_MAX = 500;

// ==================== 工具函数 ====================
/** 格式化时间戳为 HH:MM:SS */
function fmtTime(ts) {
  const d = new Date(ts);
  const p = (n) => String(n).padStart(2, "0");
  return `${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}`;
}

/** 格式化运行时长 */
function fmtUptime(ms) {
  const s = Math.floor(ms / 1000);
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  const sec = s % 60;
  const p = (n) => String(n).padStart(2, "0");
  return `${p(h)}:${p(m)}:${p(sec)}`;
}

// ==================== 标签页切换 ====================
function initTabs() {
  document.querySelectorAll(".tab-btn").forEach((btn) => {
    btn.addEventListener("click", () => {
      document.querySelectorAll(".tab-btn").forEach((b) => b.classList.remove("active"));
      document.querySelectorAll(".tab-panel").forEach((p) => p.classList.remove("active"));
      btn.classList.add("active");
      document.getElementById("tab-" + btn.dataset.tab).classList.add("active");
    });
  });
}

// ==================== WebSocket ====================
function connectWS() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${proto}://${location.host}/ws`);

  ws.onopen = () => {
    wsConnected = true;
    setBadge("ws-status", "WS 已连接", "badge-on");
  };

  ws.onmessage = (ev) => {
    try {
      const rec = JSON.parse(ev.data);
      handleRecord(rec);
    } catch (e) { /* 忽略解析错误 */ }
  };

  ws.onclose = () => {
    wsConnected = false;
    setBadge("ws-status", "WS 断开，重连中…", "badge-off");
    setTimeout(connectWS, 2000);   // 断线自动重连
  };

  ws.onerror = () => { try { ws.close(); } catch (e) {} };
}

/** 更新顶部徽章 */
function setBadge(id, text, cls) {
  const el = document.getElementById(id);
  el.textContent = text;
  el.className = "badge " + cls;
}

// ==================== 数据分发 ====================
function handleRecord(rec) {
  const { ts, type, tag, sub, payload } = rec;
  switch (type) {
    case "log":      handleLog(ts, payload); break;
    case "imu":      handleImu(ts, payload); break;
    case "pose":     handlePose(ts, payload); break;
    case "motor":    handleMotor(ts, payload); break;
    case "task":     handleTask(ts, payload); break;
    case "vision":   handleVision(ts, payload); break;
    case "stage":    handleStage(ts, payload); break;
  }
}

// ---- 各类型处理 ----
function handleLog(ts, p) {
  if (logPaused) return;
  const line = { ts, tag: p.tag || "RAW", msg: p.msg || "" };
  logLines.push(line);
  if (logLines.length > LOG_MAX) logLines.shift();
  renderLogs();
}

function handleImu(ts, p) {
  buffers.yaw.push({ x: ts, y: p.yaw });
  if (buffers.yaw.length > CHART_MAX_POINTS) buffers.yaw.shift();
  updateChart("yaw");
  document.getElementById("ov-yaw").textContent = p.yaw.toFixed(1);
}

function handlePose(ts, p) {
  buffers.pose.push({ x: p.x, y: p.y });
  if (buffers.pose.length > CHART_MAX_POINTS) buffers.pose.shift();
  updateChart("pose");
  document.getElementById("ov-x").textContent = p.x.toFixed(1);
  document.getElementById("ov-y").textContent = p.y.toFixed(1);
}

function handleMotor(ts, p) {
  const arr = buffers.motor[p.addr] || (buffers.motor[p.addr] = []);
  arr.push({ x: ts, y: p.rpm });
  if (arr.length > CHART_MAX_POINTS) arr.shift();
  updateChart("motor");
  // 电机卡片
  const el = document.getElementById("motor-" + p.addr);
  if (el) {
    el.className = "motor-item " + (p.state === "ok" ? "ok" : "bad");
    el.querySelector("b").textContent = p.rpm + " RPM";
  }
}

function handleTask(ts, p) {
  document.getElementById("ov-task").textContent = p.code + (p.valid ? "" : "（非法）");
}

function handleStage(ts, p) {
  document.getElementById("ov-stage").textContent = `${p.name} (${p.id})`;
}

// 视觉数据缓存：每类最新值（标签固定，值动态，避免标签跳动）
const visionCache = {};

function handleVision(ts, p) {
  visionCache[p.sub] = p;
  renderVision();
}

function fmtObs(p) { return p ? `距离 ${p.d}mm · 方位 ${p.a}° · ${p.side ? "右绕" : "左绕"} · ${p.clear ? "安全" : "有障"}` : ""; }
function fmtPla(p) { return p ? `环 ${p.ring} · 偏差 ${p.dx}/${p.dy}mm · 置信 ${p.conf}%` : ""; }
function fmtNav(p) { return p ? `x ${p.x} y ${p.y} yaw ${p.yaw}° · 置信 ${p.conf}%` : ""; }
function fmtBox(p) { return p ? `${p.has ? "有料" : "无料"} · 色 ${p.color}` : ""; }
function fmtTrn(p) { return p ? `色${p.c1}@${p.a1}° 色${p.c2}@${p.a2}° 色${p.c3}@${p.a3}°` : ""; }

function renderVision() {
  // 固定 5 类标签，值动态更新，标签名称不跳动
  const rows = [
    ["OBS 障碍物", fmtObs(visionCache.OBS)],
    ["PLA 对位", fmtPla(visionCache.PLA)],
    ["NAV 导航", fmtNav(visionCache.NAV)],
    ["BOX 物料", fmtBox(visionCache.BOX)],
    ["TRN 转盘", fmtTrn(visionCache.TRN)],
  ];
  const html = rows
    .map((r) => `<div class="kv"><span>${r[0]}</span><span>${r[1] || "—"}</span></div>`)
    .join("");
  document.getElementById("ov-vision").innerHTML = html;
  document.getElementById("rt-vision").innerHTML = html;
}

// ==================== 日志渲染 ====================
function renderLogs() {
  const tagFilter = document.getElementById("log-tag").value;
  const keyword = document.getElementById("log-keyword").value.toLowerCase();
  const list = document.getElementById("log-list");
  const html = logLines
    .filter((l) => !tagFilter || l.tag === tagFilter)
    .filter((l) => !keyword || l.msg.toLowerCase().includes(keyword))
    .slice(-LOG_MAX)
    .map((l) =>
      `<div class="log-line"><span class="log-time">${fmtTime(l.ts)}</span>` +
      `<span class="log-tag">[${l.tag}]</span> ${escapeHtml(l.msg)}</div>`
    )
    .join("");
  list.innerHTML = html;
  list.scrollTop = list.scrollHeight;   // 自动滚动到底部
}

function escapeHtml(s) {
  return s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
}

// ==================== Chart.js 图表 ====================
const charts = {};

function initCharts() {
  if (typeof Chart === "undefined") {
    console.error("Chart.js 未加载");
    return;
  }
  Chart.defaults.color = "#8a94a6";
  Chart.defaults.borderColor = "#e2e6ec";

  // IMU yaw 实时曲线
  charts.yaw = new Chart(document.getElementById("chart-yaw"), {
    type: "line",
    data: { datasets: [{ label: "Yaw (deg)", data: [], borderColor: "#4da3ff", tension: 0.3 }] },
    options: chartOptions("Yaw 角 (deg)"),
  });

  // 定位 XY 轨迹（散点连线）
  charts.pose = new Chart(document.getElementById("chart-pose"), {
    type: "scatter",
    data: { datasets: [{ label: "轨迹", data: [], backgroundColor: "#3ecf8e", pointRadius: 2 }] },
    options: {
      ...chartOptions("场地坐标 (mm)"),
      scales: {
        x: { type: "linear", title: { display: true, text: "X (mm)" } },
        y: { type: "linear", title: { display: true, text: "Y (mm)" } },
      },
    },
  });

  // 四轮转速曲线
  charts.motor = new Chart(document.getElementById("chart-motor"), {
    type: "line",
    data: { datasets: [
      { label: "前左", data: [], borderColor: "#4da3ff", tension: 0.2 },
      { label: "前右", data: [], borderColor: "#3ecf8e", tension: 0.2 },
      { label: "后左", data: [], borderColor: "#f5a623", tension: 0.2 },
      { label: "后右", data: [], borderColor: "#f05555", tension: 0.2 },
    ]},
    options: chartOptions("转速 (RPM)"),
  });

  // 历史回放曲线（初始空）
  charts.history = new Chart(document.getElementById("chart-history"), {
    type: "line",
    data: { datasets: [] },
    options: chartOptions("历史数据"),
  });
}

function chartOptions(title) {
  return {
    responsive: true,
    maintainAspectRatio: false,
    animation: false,   // 实时数据不带动画，避免闪烁
    plugins: { legend: { labels: { color: "#2b3440" } } },
    scales: {
      x: { type: "linear", title: { display: true, text: "时间" }, ticks: { maxTicksLimit: 8 } },
      y: { title: { display: true, text: title } },
    },
  };
}

/** 更新实时图表数据 */
function updateChart(kind) {
  if (kind === "yaw" && charts.yaw) {
    charts.yaw.data.datasets[0].data = buffers.yaw;
    charts.yaw.update("none");
  } else if (kind === "pose" && charts.pose) {
    charts.pose.data.datasets[0].data = buffers.pose;
    charts.pose.update("none");
  } else if (kind === "motor" && charts.motor) {
    [1, 2, 3, 4].forEach((addr, i) => {
      charts.motor.data.datasets[i].data = buffers.motor[addr] || [];
    });
    charts.motor.update("none");
  }
}

// ==================== 历史回放 ====================
function initHistory() {
  document.getElementById("hist-load").addEventListener("click", loadHistory);
  document.getElementById("hist-export").addEventListener("click", exportCSV);

  // 时间范围默认最近 1 分钟
  const now = Date.now();
  document.getElementById("hist-start").value = toLocalInput(now - 60000);
  document.getElementById("hist-end").value = toLocalInput(now);
}

function toLocalInput(ts) {
  const d = new Date(ts);
  const p = (n) => String(n).padStart(2, "0");
  return `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())}T${p(d.getHours())}:${p(d.getMinutes())}`;
}

function fromLocalInput(v) {
  return v ? new Date(v).getTime() : null;
}

async function loadHistory() {
  const type = document.getElementById("hist-type").value;
  const sub = document.getElementById("hist-sub").value;
  const start = fromLocalInput(document.getElementById("hist-start").value);
  const end = fromLocalInput(document.getElementById("hist-end").value);

  const params = new URLSearchParams({ type, limit: 2000 });
  if (sub) params.set("sub", sub);
  if (start) params.set("start", start);
  if (end) params.set("end", end);

  const resp = await fetch("/api/history?" + params.toString());
  const json = await resp.json();
  if (!json.ok) return;

  const datasets = extractSeries(type, json.data);
  charts.history.data.datasets = datasets;
  charts.history.update();
}

/** 根据类型从历史数据提取可绘图序列 */
function extractSeries(type, rows) {
  if (type === "imu") {
    return [{ label: "Yaw", data: rows.map(r => ({ x: r.ts, y: r.payload.yaw })), borderColor: "#4da3ff" }];
  }
  if (type === "pose") {
    return [{ label: "X", data: rows.map(r => ({ x: r.ts, y: r.payload.x })), borderColor: "#4da3ff" },
            { label: "Y", data: rows.map(r => ({ x: r.ts, y: r.payload.y })), borderColor: "#3ecf8e" }];
  }
  if (type === "motor") {
    const colors = ["#4da3ff", "#3ecf8e", "#f5a623", "#f05555"];
    return [1, 2, 3, 4].map((addr, i) => ({
      label: "轮" + addr,
      data: rows.filter(r => r.payload.addr === addr).map(r => ({ x: r.ts, y: r.payload.rpm })),
      borderColor: colors[i],
    }));
  }
  if (type === "vision") {
    // 视觉数据：按 sub 提取主字段
    const sub = rows[0]?.sub || "";
    if (sub === "OBS") return [{ label: "距离", data: rows.map(r => ({ x: r.ts, y: r.payload.d })), borderColor: "#f5a623" }];
    if (sub === "NAV") return [{ label: "X", data: rows.map(r => ({ x: r.ts, y: r.payload.x })), borderColor: "#4da3ff" }];
    if (sub === "PLA") return [{ label: "dx", data: rows.map(r => ({ x: r.ts, y: r.payload.dx })), borderColor: "#3ecf8e" }];
    return [{ label: "值", data: rows.map(r => ({ x: r.ts, y: 0 })), borderColor: "#7a8496" }];
  }
  if (type === "stage") {
    return [{ label: "阶段", data: rows.map(r => ({ x: r.ts, y: r.payload.id })), borderColor: "#f5a623", stepped: true }];
  }
  return [];
}

async function exportCSV() {
  const type = document.getElementById("hist-type").value;
  const sub = document.getElementById("hist-sub").value;
  const start = fromLocalInput(document.getElementById("hist-start").value);
  const end = fromLocalInput(document.getElementById("hist-end").value);
  const params = new URLSearchParams({ type, limit: 5000 });
  if (sub) params.set("sub", sub);
  if (start) params.set("start", start);
  if (end) params.set("end", end);

  const resp = await fetch("/api/history?" + params.toString());
  const json = await resp.json();
  const rows = json.data || [];

  let csv = "ts,type,payload\n";
  rows.forEach(r => {
    csv += `${r.ts},${r.type},"${JSON.stringify(r.payload).replace(/"/g, '""')}"\n`;
  });

  const blob = new Blob(["\ufeff" + csv], { type: "text/csv;charset=utf-8" });
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = `history_${type}_${Date.now()}.csv`;
  a.click();
  URL.revokeObjectURL(a.href);
}

// ==================== 日志页交互 ====================
function initLogs() {
  document.getElementById("log-tag").addEventListener("change", renderLogs);
  document.getElementById("log-keyword").addEventListener("input", renderLogs);
  document.getElementById("log-pause").addEventListener("click", (e) => {
    logPaused = !logPaused;
    e.target.textContent = logPaused ? "继续" : "暂停";
  });
  document.getElementById("log-clear").addEventListener("click", () => {
    logLines = [];
    renderLogs();
  });

  // 加载 tag 列表
  fetch("/api/tags").then(r => r.json()).then(json => {
    const sel = document.getElementById("log-tag");
    (json.data || []).forEach(tag => {
      const opt = document.createElement("option");
      opt.value = tag; opt.textContent = tag;
      sel.appendChild(opt);
    });
  });
}

// ==================== 状态概览：运行时间/心跳 + 数据源 ====================
function initStatus() {
  // 每 1s 刷新运行时间
  setInterval(() => {
    document.getElementById("ov-uptime").textContent = fmtUptime(Date.now() - bootTs);
  }, 1000);

  // 定时查询数据源状态
  setInterval(async () => {
    try {
      const resp = await fetch("/api/status");
      const json = await resp.json();
      const src = json.source;
      setBadge("source-status",
        src === "serial" ? "数据源：串口" : src === "simulate" ? "数据源：模拟数据" : "数据源：未连接",
        src === "serial" ? "badge-on" : src === "simulate" ? "badge-sim" : "badge-off");
    } catch (e) { /* 忽略 */ }
  }, 3000);
}

// ==================== COM 口选择 ====================
function initComSelect() {
  const sel = document.getElementById("com-select");

  // 加载可用串口列表
  function loadPorts() {
    fetch("/api/ports").then(r => r.json()).then(json => {
      const cur = sel.value;
      sel.innerHTML = '<option value="">自动探测</option>';
      (json.data || []).forEach(p => {
        const opt = document.createElement("option");
        opt.value = p.port;
        opt.textContent = p.port + (p.desc ? " · " + p.desc : "");
        sel.appendChild(opt);
      });
      sel.value = cur;
    }).catch(() => {});
  }
  loadPorts();
  setInterval(loadPorts, 3000);   // 定时刷新，支持 COM 热插拔

  // 选择后切换串口
  sel.addEventListener("change", () => {
    fetch("/api/select_port", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ port: sel.value }),
    });
  });
}

// ==================== 模拟数据开关 ====================
function initSimBtn() {
  const btn = document.getElementById("sim-btn");
  let enabled = false;

  btn.addEventListener("click", () => {
    enabled = !enabled;
    // 切换按钮样式
    btn.textContent = enabled ? "关闭模拟数据" : "开启模拟数据";
    btn.classList.toggle("on", enabled);
    // 调后端开关
    fetch("/api/simulate", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ enabled }),
    });
  });
}

// ==================== 启动 ====================
document.addEventListener("DOMContentLoaded", () => {
  initTabs();
  initCharts();
  initHistory();
  initLogs();
  initStatus();
  initComSelect();
  initSimBtn();
  connectWS();
});
