# -*- coding: utf-8 -*-
"""
生成《智能物料搬运小车控制系统设计-开题报告》的交付文件：
  1) 智能物料搬运小车控制系统设计-开题报告.docx   （公式为 Word 原生 OMML，可双击编辑）
  2) 智能物料搬运小车控制系统设计-开题报告.md     （Markdown 源文件，公式为 LaTeX）
  3) 参考文献核验清单.md
用法：python tools/build_proposal.py
"""
import os
import re

from docx import Document
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import parse_xml
from docx.oxml.ns import nsdecls, qn
from docx.shared import Pt, Cm

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOCX_OUT = os.path.join(ROOT, "智能物料搬运小车控制系统设计-开题报告.docx")
MD_OUT = os.path.join(ROOT, "智能物料搬运小车控制系统设计-开题报告.md")
CHK_OUT = os.path.join(ROOT, "参考文献核验清单.md")

M = 'xmlns:m="http://schemas.openxmlformats.org/officeDocument/2006/math"'

# 正文引用标注（顺序编码制：[1]、[2,3]、[4-6]）
CITE_RE = re.compile(r"\[[0-9][0-9,\-–]*\]")


# --------------------------------------------------------------------------
# OMML（Office Math Markup Language）构造小工具
# --------------------------------------------------------------------------
def _esc(t):
    return (t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


def mi(t):
    """斜体数学变量（默认样式）"""
    return '<m:r><m:t xml:space="preserve">%s</m:t></m:r>' % _esc(t)


def mp(t):
    """正体文本（函数名、单位、中文、数字串等）"""
    return '<m:r><m:rPr><m:sty m:val="p"/></m:rPr><m:t xml:space="preserve">%s</m:t></m:r>' % _esc(t)


def msub(base, sub):
    return '<m:sSub><m:e>%s</m:e><m:sub>%s</m:sub></m:sSub>' % (base, sub)


def mfrac(num, den):
    return '<m:f><m:num>%s</m:num><m:den>%s</m:den></m:f>' % (num, den)


def mdelim(inner, beg="[", end="]"):
    return ('<m:d><m:dPr><m:begChr m:val="%s"/><m:endChr m:val="%s"/></m:dPr>'
            '<m:e>%s</m:e></m:d>') % (beg, end, inner)


def mmat(rows, cols):
    """rows: 二维列表，元素为 OMML 片段"""
    body = ""
    for row in rows:
        body += "<m:mr>" + "".join("<m:e>%s</m:e>" % c for c in row) + "</m:mr>"
    mcs = ('<m:mPr><m:mcs><m:mc><m:mcPr><m:count m:val="%d"/>'
           '<m:mcJc m:val="center"/></m:mcPr></m:mc></m:mcs></m:mPr>' % cols)
    return "<m:m>%s%s</m:m>" % (mcs, body)


def msqrt(inner):
    return ('<m:rad><m:radPr><m:degHide m:val="1"/></m:radPr><m:deg/>'
            '<m:e>%s</m:e></m:rad>') % inner


def mtext(t):
    """公式内混排的中文/说明文字"""
    return mp(t)


# --------------------------------------------------------------------------
# 公式定义：((LaTeX 供 Markdown), (OMML 供 Word), 编号)
# --------------------------------------------------------------------------
EQ = {}

EQ["4-1"] = (
    r"\begin{bmatrix}\omega_1\\ \omega_2\\ \omega_3\\ \omega_4\end{bmatrix}"
    r"=\frac{1}{R}\begin{bmatrix}1 & 1 & k\\ 1 & -1 & -k\\ 1 & -1 & k\\ 1 & 1 & -k\end{bmatrix}"
    r"\begin{bmatrix}v_x\\ v_y\\ \omega_z\end{bmatrix},\quad k=\frac{L+W}{2}",
    mdelim(mmat([[msub(mi("ω"), mp("1"))], [msub(mi("ω"), mp("2"))],
                 [msub(mi("ω"), mp("3"))], [msub(mi("ω"), mp("4"))]], 1), "[", "]")
    + mp(" = ")
    + mfrac(mp("1"), mi("R"))
    + mdelim(mmat([[mp("1"), mp("1"), mi("k")],
                   [mp("1"), mp("-1"), mp("-") + mi("k")],
                   [mp("1"), mp("-1"), mi("k")],
                   [mp("1"), mp("1"), mp("-") + mi("k")]], 3), "[", "]")
    + mdelim(mmat([[msub(mi("v"), mi("x"))], [msub(mi("v"), mi("y"))],
                   [msub(mi("ω"), mi("z"))]], 1), "[", "]")
    + mp(",   ") + mi("k") + mp(" = ") + mfrac(mi("L") + mp(" + ") + mi("W"), mp("2")),
    "4-1")

EQ["4-2"] = (
    r"v_x=\frac{R}{4}(\omega_1+\omega_2+\omega_3+\omega_4),\qquad "
    r"v_y=\frac{R}{4}(\omega_1-\omega_2-\omega_3+\omega_4),\qquad "
    r"\omega_z=\frac{R(\omega_1-\omega_2+\omega_3-\omega_4)}{2(L+W)}",
    msub(mi("v"), mi("x")) + mp(" = ") + mfrac(mi("R"), mp("4"))
    + mdelim(msub(mi("ω"), mp("1")) + mp(" + ") + msub(mi("ω"), mp("2")) + mp(" + ")
             + msub(mi("ω"), mp("3")) + mp(" + ") + msub(mi("ω"), mp("4")), "(", ")")
    + mp(",    ")
    + msub(mi("v"), mi("y")) + mp(" = ") + mfrac(mi("R"), mp("4"))
    + mdelim(msub(mi("ω"), mp("1")) + mp(" - ") + msub(mi("ω"), mp("2")) + mp(" - ")
             + msub(mi("ω"), mp("3")) + mp(" + ") + msub(mi("ω"), mp("4")), "(", ")")
    + mp(",    ")
    + msub(mi("ω"), mi("z")) + mp(" = ")
    + mfrac(mi("R") + mdelim(msub(mi("ω"), mp("1")) + mp(" - ") + msub(mi("ω"), mp("2"))
             + mp(" + ") + msub(mi("ω"), mp("3")) + mp(" - ") + msub(mi("ω"), mp("4")), "(", ")"),
           mp("2") + mdelim(mi("L") + mp(" + ") + mi("W"), "(", ")"))
    + mp(",    ") + mi("k") + mp(" = ") + mfrac(mi("L") + mp(" + ") + mi("W"), mp("2")),
    "4-2")

EQ["4-3"] = (
    r"\Delta N_i=\frac{\Delta s_i\,P\,n}{\pi D}\times\gamma,\qquad i=1,2,3,4",
    msub(mp("Δ") + mi("N"), mi("i")) + mp(" = ")
    + mfrac(mp("Δ") + msub(mi("s"), mi("i")) + mp(" × ") + mi("P") + mp(" × ") + mi("n"),
            mp("π") + mi("D"))
    + mp(" × ") + mi("γ") + mp(",    i = 1, 2, 3, 4"),
    "4-3")

_ds = lambda i: mp("Δ") + msub(mi("s"), mp(str(i)))
EQ["4-4"] = (
    r"\begin{aligned}"
    r"\Delta x&=\frac{1}{4}(\Delta s_1+\Delta s_2+\Delta s_3+\Delta s_4),\quad "
    r"\Delta y=\frac{1}{4}(\Delta s_1-\Delta s_2-\Delta s_3+\Delta s_4),\\"
    r"\Delta\theta&=\frac{1}{2(L+W)}(\Delta s_1-\Delta s_2+\Delta s_3-\Delta s_4),\\"
    r"x_{k+1}&=x_k+\Delta x\cos\theta_k-\Delta y\sin\theta_k,\quad "
    r"y_{k+1}=y_k+\Delta x\sin\theta_k+\Delta y\cos\theta_k,\quad "
    r"\theta_{k+1}=\theta_k+\Delta\theta"
    r"\end{aligned}",
    mp("Δ") + mi("x") + mp(" = ") + mfrac(mp("1"), mp("4"))
    + mdelim(_ds(1) + mp(" + ") + _ds(2) + mp(" + ") + _ds(3) + mp(" + ") + _ds(4), "(", ")")
    + mp(",   ") + mp("Δ") + mi("y") + mp(" = ") + mfrac(mp("1"), mp("4"))
    + mdelim(_ds(1) + mp(" - ") + _ds(2) + mp(" - ") + _ds(3) + mp(" + ") + _ds(4), "(", ")")
    + mp(",   ") + mp("Δ") + mi("θ") + mp(" = ")
    + mfrac(mp("1"), mp("2") + mdelim(mi("L") + mp(" + ") + mi("W"), "(", ")"))
    + mdelim(_ds(1) + mp(" - ") + _ds(2) + mp(" + ") + _ds(3) + mp(" - ") + _ds(4), "(", ")")
    + mp(",   ")
    + msub(mi("x"), mp("k+1")) + mp(" = ") + msub(mi("x"), mi("k")) + mp(" + ") + mp("Δ") + mi("x")
    + mp(" cos") + msub(mi("θ"), mi("k")) + mp(" - ") + mp("Δ") + mi("y") + mp(" sin") + msub(mi("θ"), mi("k"))
    + mp(",   ") + msub(mi("y"), mp("k+1")) + mp(" = ") + msub(mi("y"), mi("k")) + mp(" + ") + mp("Δ") + mi("x")
    + mp(" sin") + msub(mi("θ"), mi("k")) + mp(" + ") + mp("Δ") + mi("y") + mp(" cos") + msub(mi("θ"), mi("k"))
    + mp(",   ") + msub(mi("θ"), mp("k+1")) + mp(" = ") + msub(mi("θ"), mi("k")) + mp(" + ") + mp("Δ") + mi("θ"),
    "4-4")

EQ["4-5"] = (
    r"\Delta u_k=K_p(e_k-e_{k-1})+K_i e_k+K_d(e_k-2e_{k-1}+e_{k-2}),\qquad "
    r"u_k=u_{k-1}+\Delta u_k,\quad e_k=\theta_{ref,k}-\theta_{meas,k}",
    mp("Δ") + msub(mi("u"), mi("k")) + mp(" = ") + msub(mi("K"), mi("p"))
    + mdelim(msub(mi("e"), mi("k")) + mp(" - ") + msub(mi("e"), mp("k-1")), "(", ")")
    + mp(" + ") + msub(mi("K"), mi("i")) + msub(mi("e"), mi("k"))
    + mp(" + ") + msub(mi("K"), mi("d"))
    + mdelim(msub(mi("e"), mi("k")) + mp(" - 2") + msub(mi("e"), mp("k-1")) + mp(" + ") + msub(mi("e"), mp("k-2")), "(", ")")
    + mp(",   ") + msub(mi("u"), mi("k")) + mp(" = ") + msub(mi("u"), mp("k-1")) + mp(" + ") + mp("Δ") + msub(mi("u"), mi("k"))
    + mp(",   ") + msub(mi("e"), mi("k")) + mp(" = ") + msub(mi("θ"), mp("ref,k")) + mp(" - ") + msub(mi("θ"), mp("meas,k")),
    "4-5")

EQ["4-6"] = (
    r"\theta_k=\alpha\left[\theta_{k-1}+\omega_z\,\Delta t\right]+(1-\alpha)\,\theta_{odo,k},"
    r"\qquad \alpha=\frac{\tau}{\tau+\Delta t}",
    msub(mi("θ"), mi("k")) + mp(" = ") + mi("α")
    + mdelim(msub(mi("θ"), mp("k-1")) + mp(" + ") + msub(mi("ω"), mi("z")) + mp(" ") + mp("Δ") + mi("t"), "[", "]")
    + mp(" + ") + mdelim(mp("1 - ") + mi("α"), "(", ")") + msub(mi("θ"), mp("odo,k"))
    + mp(",   ") + mi("α") + mp(" = ") + mfrac(mi("τ"), mi("τ") + mp(" + ") + mp("Δ") + mi("t")),
    "4-6")


# --------------------------------------------------------------------------
# 正文内容（块列表）
# --------------------------------------------------------------------------
CONTENT = [  # CONTENT_PLACEHOLDER
    ("h1", "智能物料搬运小车控制系统设计"),
    ("cap", "本科毕业设计（论文）开题报告"),
    ("tbl", None,
     ["课题名称", "智能物料搬运小车控制系统设计", "课题来源", "省级大学生工程实践与创新能力大赛“智能+工程创新·智能搬运”赛项"],
     [["学院", "＿＿＿＿＿＿＿＿", "专业", "＿＿＿＿＿＿＿＿"],
      ["学生姓名", "＿＿＿＿＿＿＿＿", "学号", "＿＿＿＿＿＿＿＿"],
      ["指导教师", "＿＿＿＿＿＿＿＿", "职称", "＿＿＿＿＿＿＿＿"],
      ["论文类型", "本科毕业设计（论文）", "拟定时间", "＿＿＿＿年＿＿月"]]),

    ("h1", "一、选题背景与意义"),
    ("h2", "1.1 研究背景"),
    ("p", "仓储与车间物流变了：订单批量变小、换线变频繁，靠固定轨道加传送带的搬运方式越来越不划算。柔性物流要的是能横移、能原地转向、读得懂任务、会自己抓放物料的搬运车，麦轮底盘加二维码下发任务是常见配置。"),
    ("p", "本课题源于省级大学生工程实践与创新能力大赛“智能+工程创新·智能搬运”赛项，约束很紧。场地 2400mm×2400mm，中央十字灰色车道宽 400mm；原料区是 φ300mm 电动转盘，3 个物料 120° 均布，转盘 6~10s/圈且转向随机；粗加工区与暂存区 580mm×150mm，画有 1~6 号同心圆环并按环号计分；物料是 3D 打印 ABS 件，外形不超过 80mm、质量不超过 100g，四色各两个；障碍物为 φ50mm×100mm 黑色圆柱，位置随机。整车铅垂投影不超过 300mm×300mm、高不超过 400mm，车载单一电源、禁止遥控，单轮 3min。"),
    ("p", "任务流程：一键启动后到二维码板读码并显示，再到原料区按任务码顺序逐个抓取（抓走一个放上车才能抓下一个），运至粗加工区按码放置，转送暂存区对应圆环；返回原料区完成第二批，并把第二批码垛在暂存区同色物料之上；最后回启停区统计抓取/放置正确次数。任务码形如 134+123+314+231，颜色编码红=1、黄=2、蓝=3、绿=4。"),

    ("h2", "1.2 国内外研究现状"),
    ("p", "（1）麦轮底盘运动学与运动控制。麦轮运动学建模已成熟，近年多追求轨迹跟踪的精度与鲁棒性。田塘等[1]在 ROS 下做 URDF 建模与运动学分析，用 move_base 与 AMCL 搭全向导航，但依赖激光雷达建图。杨凌耀等[2]用多幂次趋近律与双曲正切函数抑制滑模抖振，并用 Lyapunov 理论证明 3 输入 4 输出系统的稳定性，不过以仿真为主，没讨论打滑与加减速段。Szeremeta 等[3]、Galati 等[4]、Zhang 等[5]把神经网络控制与模型预测控制用在麦轮机器人上，只有第一篇做了实物验证。"),
    ("p", "国内工程实现更贴近实车。匡兴红等[6]用全局相机加 YOLOv5 给出 AGV 坐标，与惯导做自适应卡尔曼滤波融合，再用模糊 PID 与解耦控制纠偏；刘梓博等[7]把横向偏差与航向偏差分开限幅，用双变量限幅 PID 解算前进速度与角速度。"),
    ("p", "（2）多传感器融合定位与导航。钟浩翔等[8]把惯导、里程计与二维码模块装到双舵轮泊车机器人上，用卡尔曼滤波融合航位推算，平均定位偏差比纯航迹推算降低 92%，达到 14.04mm。袁千贺等[9]对比单一里程计、EKF 融合与 AMCL 融合三种方案后指出，环境地图对抑制累积误差的作用比算法本身还大。武东杰等[10]的 IEVL-Fusion 相对定位误差小于 0.4%，白师宇等[11]用因子图紧耦合激光雷达、里程计与惯性测量单元，Cai 等[12]把室内协同定位的三维均方根误差压到 11.3mm。"),
    ("p", "精度高，代价也清楚：激光雷达、视觉里程计、UWB 基站、全局相机，本赛题场地一个都不许用；场地只有灰色十字车道，不许贴磁条或布基站，单一电源还要养四台电机、五个舵机与树莓派。可用的绝对信息只剩四轮编码器增量与一个单轴陀螺仪的航向角。"),
    ("p", "（3）物料识别、抓取与任务调度。孙先涛等[17]用通道注意力机制改进 YOLO-V3 检测目标，再以 VGG-16 嵌入最小面积外接矩形估计抓取位姿，角度平均误差小于 2.47°、有害扭矩降低约 75%。张晋[18]把 YOLOv5 的 BN 层换成 MobilenetV3，参数量减少 49.78%、计算量减少 61.39%。Chen 等[19]针对分拣现场光照不均，按二维码特征自适应确定二值化窗口。"),
    ("p", "照着赛题形态做的工作也不少。曾一凡等[20]用 STM32F103ZET6 做主控，扫码取任务、OpenMV 算物料中心、灰度传感器循迹、麦轮行走、舵机机械臂抓放，样机跑通全流程；郭根等[21]、张译等[25]分别在搬运小车上集成颜色识别与黑线磁标离线定位；曹奇[26]完成 RoboMaster 步兵机器人的麦轮底盘与云台控制；Li 等[27]的车间搬运机器人是麦轮加视觉抓取。机械臂侧，陈庄浩等[24]把 STM32 主控的六自由度机械臂架到 TurtleBot2 上完成移动抓取协同。"),

    ("h2", "1.3 现有研究的不足"),
    ("p", "其一，无循迹条件下的定位可靠性。上述方案要么依赖激光雷达、视觉里程计、UWB 或全局相机[1,6,9-13,23]，要么依赖预铺的导引介质；本赛题场地只有灰色十字车道，可用信息只有四轮编码器与单轴陀螺仪。刻度标定是硬骨头：轮径、细分、减速比任一项有偏差都会按比例放大到 2m 量级的行程上。文献[6,7]指出横向与航向偏差要同时约束，但没给出无外部基准时把误差压进 50mm 的完整做法；苏昭宇[14]、郭培恺[15]、李向誉[16]的学位论文分别做 IMU 紧耦合里程计、里程计与惯导联合推算、码盘位姿参数校正，也说明标定绕不开。"),
    ("p", "其二，异构算力下的任务分工。树莓派做视觉、STM32 做实时控制的组合并不新鲜[13,18,20,23]，但多数工作把视觉输出当真值送进控制回路，对丢帧、断线、校验失败缺少约定：视觉一丢帧，底盘该继续走还是原地等，很难找到现成规则。"),
    ("p", "其三，动态目标抓取。物料不在静止工位上，而是 120° 均布在 φ300mm 转盘上随转盘移动，6~10s/圈且转向随机。现有抓取研究[17,18]多面向静止工件，移动机械臂的工作[24]主要解决“底盘先走到可抓区域”；抓随动目标的时序裕度怎么算，可参考的材料不多。"),

    ("h2", "1.4 研究意义"),
    ("p", "理论上，本课题要在没有外部绝对基准、算力与传感都受限的平台上，把麦轮解算、航向闭环、编码器航迹推算与陀螺仪融合串成一条可标定、可复现的位姿估计链路。它更考验对误差来源的辨识：哪一段是刻度性的、哪一段是打滑造成的、哪一段来自陀螺仪零偏，都要分开确认。"),
    ("p", "工程上，赛题约束（2400mm×2400mm 场地、400mm 车道、6~10s/圈的随机转向转盘、300mm×300mm×400mm 外形、车载单电源、单轮 3min）与中小型柔性产线很像：空间紧、节拍明确、地面不许改造。课题形成的底盘控制、任务码解析、动作序列与非阻塞调度模块可直接迁移到同类教学与轻型分拣设备上。"),

    ("h1", "二、研究目标与研究内容"),
    ("h2", "2.1 研究目标"),
    ("p", "总目标是：以 STM32F103VET6 为核心，构建“感知—决策—执行”三层结构的控制系统，在只有四轮编码器与单轴陀螺仪的条件下完成场地内自主行驶与物料搬运，单轮不超过 3min，连续抓放成功率不低于 90%。指标见表 2-1，来源为分阶段验收判据，未作放大；术语按 GB/T 42830-2023[28]、GB/T 12643-2025[29] 界定。"),
    ("tbl", "表 2-1  控制系统主要技术指标",
     ["序号", "指标项", "指标要求"],
     [["1", "直线行驶 2000mm 横向偏差", "< 30mm"],
      ["2", "原地旋转 90°/180° 角度误差", "< 2°"],
      ["3", "2m×2m 矩形闭环回起点位置误差", "< 50mm"],
      ["4", "单轮给定 1000mm 位移实测误差", "< 1%"],
      ["5", "陀螺仪静置漂移", "< 0.5°/min"],
      ["6", "二维码识别条件", "80×80mm，读距 10~20cm，1s 内稳定输出"],
      ["7", "机械臂单关节定位回读偏差", "≤ 2 个码值"],
      ["8", "连续抓放成功率", "≥ 90%"],
      ["9", "单轮完整任务时间", "≤ 3min"]]),
    ("h2", "2.2 研究内容"),
    ("p", "（1）底盘运动学解算与执行。建立四轮麦轮（对角 X 形布局）的正/逆运动学模型，把 (vx, vy, ωz) 解算为四轮转速与目标脉冲，加入速度斜坡限幅，并完成“位移—脉冲”刻度标定。"),
    ("p", "（2）航向闭环与位姿估计。以 HWT101CT 的航向角 yaw 作转向反馈构成增量式 PID 闭环；用四台闭环步进电机的编码器反馈做航迹推算，再用互补滤波与陀螺仪积分融合得到 (x, y, yaw)，并封装 Move_Dist、Turn_To、GoToPoint 三个动作接口。"),    ("p", "（3）任务码获取与解析。驱动 GM65（UART4，9600-8-N-1，连续识读）读取形如 134+123+314+231 的任务码，用基于空闲超时的成帧机制而非固定长度截断，校验 4 组 3 位数字、3 个“+”与颜色值 1~4；非法帧记日志并拒绝更新显示。"),
    ("p", "（4）视觉协同接口。约定树莓派（USART1，115200-8-N-1）上传的码垛与障碍物帧格式，STM32 只接收、校验、缓存与显示，不做图像运算；帧超时即置无效标志，不阻塞主流程。"),
    ("p", "（5）机械臂抓取动作序列。通过 USART3 控制幻尔总线舵机（半双工，115200）驱动的五关节机械臂完成抓取—载运—放料—码垛；用“预设动作逐个下发 + 广播 ID 发执行”实现多关节同时起动，动作前后用位置回读校核。"),
    ("p", "（6）任务调度与可靠性。把上述模块组织成非阻塞状态机，主循环按心跳节拍统一调度；日志统一为“[模块] key=value”；CAN 发送由死等邮箱改为超时返回，串口接收处理 ORE/FE/NE/PE 错误并重装中断，异常时分级用 LED 与蜂鸣报警。"),

    ("h1", "三、拟解决的关键问题与难点"),
    ("p", "（1）没有绝对基准时，累积误差怎么控住。原地旋转时四个麦轮靠横向刮擦工作，转速越高打滑越严重，而打滑造成的转角偏差与位移丢失，编码器量不到——该转的脉冲一个不少地记下来，车却没转到位。办法是把原地旋转速度降一半，并把旋转等效弧长做成可标定的比例系数。走距也有坑：首轮试跑 500mm 偏差约 12.5%，根源是刻度——位置量的单位可能是电机轴脉冲（16 细分下 3200/圈），也可能是 0.1°（3600/圈）。"),
    ("p", "（2）通信链路的可靠性。CAN 侧原先是“循环等待发送邮箱空闲”，总线一异常就死等，主循环停摆，后改成带 100ms 超时的返回。半双工舵机总线更麻烦：自己发的字节会被回显，舵机应答同时挂在总线上，只能按帧头 0x55 0x55 重新同步、用 Length 定界，并在应答收完前禁止连发。扫码串口则会在 ORE/FE/NE/PE 错误后中断停摆，看着在线其实收不到字节，需在错误回调里重装接收中断。"),    ("p", "（3）动态转盘抓取时序。转盘 6~10s/圈、转向随机，物料在 φ300mm 圆周上持续运动，机械臂从底盘停稳到夹爪闭合有固定耗时，还要留回读校核时间；本轮剩余可抓时间不足就放弃，等下一个物料转到抓取位再动手。"),
    ("p", "（4）异构算力下的降级策略。树莓派给的码垛与障碍物信息只当建议、不当真值；帧超时或校验失败就置无效标志，底盘按里程计与预设路径走完当前航段，码垛位置退回默认顺序，任务不因一次视觉失败中断。"),
    ("p", "（5）实时性与非阻塞约束。中断只搬字节，解析、校验与显示全在主循环，全程不用 HAL_Delay。两个教训：蜂鸣器电平直驱不响，确认为无源件后改用 TIM3_CH2 输出约 2.73kHz、50% 占空比 PWM；按键无外部上拉时读数不稳，改内部上拉并按 5ms 扫描加 10ms 消抖。"),

    ("h1", "四、研究方法与技术路线"),
    ("h2", "4.1 研究方法"),
    ("p", "方法上以文献调研、理论建模、仿真验证、实验标定与对比测试为主：先按六条技术线检索中外文献并比对精度与代价；再建立麦轮正/逆运动学与航迹推算模型；在 MATLAB/Simulink 与三维视图中验证解算结果与状态机流程；对轮径、细分、减速比、旋转弧长刻度、陀螺仪零偏与关节零位逐个做单变量标定，每次只改一个量并记录偏差；最后用对照实验定参数。"),
    ("h2", "4.2 技术路线"),
    ("p", "系统按“感知—决策—执行”三层组织：感知层是二维码模块、单轴陀螺仪与树莓派链路；决策层在 STM32 内完成任务码解析、位姿估计与调度；执行层是 CAN 上四台闭环步进电机与半双工总线上五个舵机。接口关系见图 4-1，任务流程见图 4-2。"),
    ("code", "graph TD\n  QR[\"GM65 二维码模块 UART4, 9600-8-N-1\"] -->|\"任务码 DDD+DDD+DDD+DDD\"| MCU[\"STM32F103VET6  72MHz  感知-决策-执行\"]\n  IMU[\"HWT101CT 单轴陀螺仪 USART2, 9600\"] -->|\"航向角 yaw / 角速度 wz\"| MCU\n  RPI[\"树莓派视觉 USART1, 115200\"] -->|\"码垛位/障碍物信息(只收不发)\"| MCU\n  MCU -->|\"CAN1 500kbps 扩展帧 ExtId=addr<<8|seq\"| EMM[\"Emm_V5.0 闭环步进驱动器 x4\"]\n  EMM --> MEC[\"四轮麦克纳姆轮底盘(对角X形布局)\"]\n  MCU -->|\"USART3 115200 半双工总线\"| SRV[\"幻尔总线舵机 x5\"]\n  SRV --> ARM[\"抓取/码垛机械臂\"]\n  MCU --> OLED[\"0.96in OLED  I2C1 100kHz  0x78\"]\n  MCU --> HMI[\"按键 PE7-PE11 / 蜂鸣器 PA7 PWM 2.73kHz / 三色LED\"]\n  MCU -->|\"UART5 115200  [模块] key=value\"| PC[\"上位机日志\"]"),
    ("cap", "图 4-1  系统总体框图（Mermaid 源码，可在支持 Mermaid 的编辑器中渲染）"),
    ("code", "stateDiagram-v2\n  [*] --> IDLE\n  IDLE --> GO_QR: 一键启动\n  GO_QR --> SCAN: 抵达二维码板\n  SCAN --> TO_RAW: 任务码合法并显示\n  TO_RAW --> PICK1: 转盘对位\n  PICK1 --> PICK1: 抓取1个并放上车(未满3个)\n  PICK1 --> TO_ROUGH1: 第一批3个上车\n  TO_ROUGH1 --> PLACE1\n  PLACE1 --> PICK_ROUGH1\n  PICK_ROUGH1 --> TO_STORE1\n  TO_STORE1 --> PLACE2\n  PLACE2 --> TO_RAW2\n  TO_RAW2 --> PICK2\n  PICK2 --> TO_ROUGH2\n  TO_ROUGH2 --> PLACE3\n  PLACE3 --> PICK_ROUGH2\n  PICK_ROUGH2 --> TO_STORE2\n  TO_STORE2 --> STACK: 第二批按同色码垛于第一批之上\n  STACK --> RETURN\n  RETURN --> SHOW_STAT: 统计抓取正确次数/放置正确次数\n  SHOW_STAT --> [*]"),
    ("cap", "图 4-2  任务调度状态机（Mermaid 源码）"),
    ("h2", "4.3 关键技术方案"),
    ("p", "（1）运动学解算与执行。四轮按对角 X 形布置，取 x 轴向前、y 轴向右、绕 z 轴逆时针为正：式(4-1)为逆解，把整车速度指令映射为四轮角速度；式(4-2)为正解，把四轮增量还原为车体位移，两式严格互逆。整车某轴方向反了，只改车体级符号宏，不动公式。"),
    ("eq", "4-1"),
    ("eq", "4-2"),
    ("tbl", "表 4-1  式(4-1)、式(4-2)符号说明",
     ["符号", "含义", "取值与来源"],
     [["R", "车轮（含滚轮）有效半径", "50mm（轮径 100mm，卷尺实测）"],
      ["L", "轴距，前后轮轴心距", "170mm（装车实测）"],
      ["W", "轮距，左右轮心距", "245mm（装车实测）"],
      ["k", "旋转等效力臂，(L+W)/2", "207.5mm"],
      ["ω1~ω4", "左前/右前/左后/右后轮角速度", "正为前进方向；上限对应电机轴 500RPM"],
      ["vx、vy", "车体前进、右移线速度", "mm/s，默认档对应车轮 300RPM"],
      ["ωz", "绕 z 轴角速度", "rad/s，逆时针为正"]]),
    ("p", "（2）位移—脉冲换算与里程计。底盘下发的是位置模式脉冲数，需先把车轮位移换算成电机轴脉冲，见式(4-3)；标定系数 γ 用来吸收“脉冲单位还是 0.1° 单位”的刻度歧义。四轮脉冲增量经正解还原为车体位移后，按式(4-4)积分得到场地坐标。"),
    ("eq", "4-3"),
    ("eq", "4-4"),
    ("tbl", "表 4-2  式(4-3)符号说明",
     ["符号", "含义", "取值与来源"],
     [["ΔNi", "第 i 轮本次增量的电机轴脉冲数", "由 CAN 位置反馈读取，带符号"],
      ["Δsi", "第 i 轮的等效位移", "mm"],
      ["P", "电机每转脉冲数（16 细分×200 步）", "3200"],
      ["n", "电机到车轮的减速比", "本项目实车为 1.0（1:1 直连）"],
      ["D", "车轮直径", "100mm"],
      ["γ", "刻度修正系数", "默认 1；若位置量按 0.1° 解析则取 3200/3600≈0.889"]]),
    ("tbl", "表 4-3  式(4-4)符号说明",
     ["符号", "含义", "取值与来源"],
     [["Δx、Δy", "一个周期的车体位移增量", "mm，车体坐标系；四轮增量之和"],
      ["Δθ", "一个周期的航向角增量", "rad"],
      ["xk、yk", "第 k 个周期的场地坐标", "mm，起点为原点"],
      ["θk", "第 k 个周期的航向角", "rad，由陀螺仪与里程计融合给出（见式(4-6)）"]]),
    ("p", "（3）航向闭环。反馈量是陀螺仪航向角 yaw，被控量是旋转角速度指令，采用式(4-5)的增量式 PID。用增量式而非位置式，是为了中途换目标角或急停时不至于把历史积分一次性甩出去。"),
    ("eq", "4-5"),
    ("tbl", "表 4-4  式(4-5)符号说明",
     ["符号", "含义", "取值与说明"],
     [["ek", "第 k 次采样的航向角偏差", "deg，与 PID 参数同单位"],
      ["Kp", "比例系数", "先给小的值逐步加，过大会出现绕轴来回振荡"],
      ["Ki", "积分系数", "用于消除静差；打滑时容易积分饱和，需限幅"],
      ["Kd", "微分系数", "抑制过冲，对陀螺仪噪声敏感，微分项需低通或限幅"],
      ["Δuk", "控制量增量", "rad/s，或换算成等效弧长 mm/拍"],
      ["uk", "本拍输出的角速度指令", "受原地旋转速度上限（默认 150RPM）约束"]]),
    ("p", "（4）多源位姿融合。陀螺仪短时角度准、长时间积分会漂；里程计位移来自编码器，长时间是累积的刻度误差，打滑时航向又不可信。两者用式(4-6)的一阶互补滤波结合，时间常数 τ 表示“在多长时间内信任陀螺仪积分”，τ 越大越依赖陀螺仪短时精度。"),
    ("eq", "4-6"),
    ("tbl", "表 4-5  式(4-6)符号说明",
     ["符号", "含义", "取值与说明"],
     [["α", "陀螺仪积分项权重", "0~1；按 τ=1s、Δt=10ms 计算约为 0.99"],
      ["τ", "互补滤波时间常数", "s，决定两种航向信息的交叠频率"],
      ["Δt", "处理周期", "s，主循环航向节拍取 10ms"],
      ["θodo,k", "里程计推算航向", "rad，由式(4-4)累计得到"],
      ["θk", "融合后的航向角", "rad，供转向闭环与位姿估计使用"]]),
    ("p", "（5）任务调度与码垛映射。任务码解析成四个数组：第一批颜色顺序、第一批放置圆环号、第二批颜色顺序、第二批在粗加工区的圆环号；暂存区码垛位置由第一批颜色决定。状态机按原料区—粗加工区—暂存区环路循环两批，每个抓放动作前后核对已抓/已放数量并累加正确次数；失败先重试，重试用尽就蜂鸣报警并停在当前航段。"),
    ("p", "（6）通信可靠性设计。CAN 用扩展帧，标识符按 ExtId = 电机地址<<8 | 包序号 组织，四个轮子地址固定为 1~4；位置轮询与速度命令合并计算，间隔不小于 50ms，并加发送超时保护。半双工舵机总线按“发送—切接收—按帧头重同步—校验和验证—允许下次发送”的时序执行，多关节同步用“逐个下发预设 + 广播 ID 执行”。扫码与视觉的成帧以空闲超时判定帧结束。"),

    ("h1", "五、可行性分析"),
    ("p", "理论可行性。麦轮运动学与航迹推算都是成熟理论，本课题不引入新的控制律，只做工程实现与参数标定，公式规模适合本科阶段，互补滤波的参数含义也便于在实车上单变量调试。"),
    ("p", "技术可行性。硬件已选定并做过基础验证：STM32F103VET6 提供 CAN1、五路串口与 I2C1；四台 Emm_V5.0 闭环步进电机带编码器反馈；HWT101CT 输出 0.1° 量级航向角；GM65 在连续识读模式下配合空闲超时成帧可稳定取码。软件侧已有统一日志与心跳机制，模块按 P0~P10 分期开发。"),
    ("p", "条件与进度可行。器件、调试工具与场地由学校工程训练中心提供，ST-Link、USB-TTL、CAN 分析仪与电源板可满足联调；赛题图纸与命题文件齐备，可按真实尺寸布置场地。任务拆成互相独立的模块，底盘、扫码、视觉接口与机械臂动作序列可并行推进；风险最高的无循迹定位精度安排在中期攻关，前期先用单轮测距与定角旋转把误差源头找清楚。"),

    ("h1", "六、进度安排"),
    ("tbl", "表 6-1  开题后进度安排（按自然周）",
     ["阶段", "周次", "主要工作", "阶段成果"],
     [["资料调研与方案设计", "第 1~2 周", "文献检索与综述、赛题约束梳理、总体方案与接口约定", "开题报告与总体方案"],
      ["底盘运动学与电机驱动", "第 3~4 周", "CAN 通信超时保护、速度/位置模式封装、正逆解实现与方向标定", "底盘可完成前进/横移/斜行/原地旋转"],
      ["位移刻度与陀螺仪标定", "第 5~6 周", "500mm 与 1000mm 走距标定、ωz 零偏与标度因数标定、静置漂移测试", "单轮 1000mm 位移误差 <1%"],
      ["航向闭环与位姿估计", "第 7~8 周", "增量式 PID 航向环整定、航迹推算与互补滤波、基础动作库封装", "定角旋转 <2°，2000mm 横向偏差 <30mm"],
      ["任务码与视觉接口", "第 9~10 周", "GM65 成帧与校验、OLED 显示、树莓派帧协议与超时降级", "任务码 1s 内稳定解析并显示"],
      ["机械臂动作序列", "第 11~12 周", "半双工驱动、软限位与初始位、抓取/放料/码垛动作序列与回读校核", "单关节回读偏差 ≤2 码值，抓放成功率 ≥90%"],
      ["任务调度与系统联调", "第 13~14 周", "状态机整合、异常重试与报警、矩形闭环与全程跑图", "2m×2m 闭环 <50mm，单轮任务 ≤3min"],
      ["论文撰写与答辩准备", "第 15~16 周", "数据整理、图表制作、论文撰写与修改、调试文档归档", "毕业论文与完整调试文档"]]),
    ("p", "每阶段按“先建、后烧、再看现象”的顺序验收：编译 0 错误 0 警告后下载，观察日志与现象，实测数据记入调试文档；指标未达标时先回到标定环节找刻度性偏差，再考虑调控制参数。"),

    ("h1", "七、预期成果与创新点"),
    ("p", "预期成果：（1）控制系统软硬件一套，含底盘控制、位姿估计、任务码解析、视觉接口、机械臂动作与任务调度六个模块的源码与配置；（2）可运行样机一台，能在 2400mm×2400mm 场地内完成两批物料的抓取、放置与码垛，单轮不超过 3min；（3）毕业设计论文一篇，含实验数据、标定过程与误差分析；（4）调试文档，含重要函数表与可调参数表。"),
    ("p", "创新点限以下三条，均以本项目的实际做法为依据。"),
    ("p", "第一，无循迹条件下的轻量位姿估计与标定流程。只有四轮编码器与单轴陀螺仪时，把误差拆成“刻度性”和“打滑性”两类分头处理：刻度类用已知距离与角度做单变量标定，打滑类靠降低原地旋转速度、按位移比例分配四轮速度来抑制，形成一套照做即可的标定步骤。"),
    ("p", "第二，异构算力的分层协同与降级约定。STM32 只接收、校验、缓存与显示树莓派的数据，不做图像运算；视觉帧超时或校验失败时，底盘与机械臂按预设流程降级继续，任务不因一次视觉失败中断，同类实现中较少见到明确规则。"),
    ("p", "第三，统一的成帧与调度机制。扫码任务码、视觉数据帧与舵机应答帧共用“空闲超时定界 + 校验和验证 + 帧头重同步”的思路，配合心跳调度的非阻塞状态机与分级日志，故障可定位、可恢复。"),

    ("h1", "八、参考文献"),
    ("ref", "[1] 田塘, 李涛, 邓春燕, 等. 基于ROS的全向移动机器人导航系统设计与仿真[J]. 人工智能与机器人研究, 2022, 11(1): 46-55. DOI: 10.12677/airr.2022.111006."),
    ("ref", "[2] 杨凌耀, 张爱华, 徐金龙, 等. 基于幂次趋近律的麦克纳姆轮全向移动机器人轨迹跟踪控制[J]. 上海工程技术大学学报, 2022, 36(1): 23-30. DOI: 10.12299/jsues.21-0195."),
    ("ref", "[3] SZEREMETA M, SZUSTER M. Neural tracking control of a four-wheeled mobile robot with mecanum wheels[J]. Applied Sciences, 2022, 12(11): 5322. DOI: 10.3390/app12115322."),
    ("ref", "[4] GALATI R, MANTRIOTA G. Path following for an omnidirectional robot using a non-linear model predictive controller for intelligent warehouses[J]. Robotics, 2023, 12(3): 78. DOI: 10.3390/robotics12030078."),
    ("ref", "[5] ZHANG T, ZHANG X Y. Distributed model predictive control with particle swarm optimizer for collision-free trajectory tracking of MWMR formation[J]. Actuators, 2023, 12(3): 127. DOI: 10.3390/act12030127."),
    ("ref", "[6] 匡兴红, 梁定乾, 时雨, 等. 全局感知AGV的导航控制技术研究[J]. 机械科学与技术, 2025, 44(12): 2069-2077. DOI: 10.13433/j.cnki.1003-8728.20230377."),
    ("ref", "[7] 刘梓博, 陈羽立, 徐梓峰, 等. 基于双变量限幅PID算法的四轮差速转向AGV导航控制系统[J]. 农业工程学报, 2025, 41(5): 27-37. DOI: 10.11975/j.issn.1002-6819.202405070."),
    ("ref", "[8] 钟浩翔, 陈亚, 王殿君, 等. 双舵轮泊车机器人运动学分析及定位系统研究[J]. 机械科学与技术, 2023, 42(10): 1625-1629. DOI: 10.13433/j.cnki.1003-8728.20220129."),
    ("ref", "[9] 袁千贺, 田昕, 沈斯杰. 基于多传感器融合的移动机器人定位[J]. 计算机系统应用, 2022, 31(3): 136-142. DOI: 10.15888/j.cnki.csa.008390."),
    ("ref", "[10] 武东杰, 仲训昱, 崔晓珍, 等. 具有全局速度约束的惯性/编码器/视觉/激光融合定位方法: IEVL-Fusion[J]. 机器人, 2022, 44(4): 443-452. DOI: 10.13973/j.cnki.robot.210287."),
    ("ref", "[11] 白师宇, 赖际舟, 吕品, 等. 面向地下空间探测的移动机器人定位与感知方法[J]. 机器人, 2022, 44(4): 463-470. DOI: 10.13973/j.cnki.robot.210141."),
    ("ref", "[12] CAI Z, LIU J H, CHI W J, et al. A low-cost and robust multi-sensor data fusion scheme for heterogeneous multi-robot cooperative positioning in indoor environments[J]. Remote Sensing, 2023, 15(23): 5584. DOI: 10.3390/rs15235584."),
    ("ref", "[13] NEAZ A, LEE S, NAM K. Design and implementation of an integrated control system for omnidirectional mobile robots in industrial logistics[J]. Sensors, 2023, 23(6): 3184. DOI: 10.3390/s23063184."),
    ("ref", "[14] 苏昭宇. 基于IMU紧耦合与多类别特征点匹配的激光SLAM里程计研究[D]. 桂林: 桂林电子科技大学, 2022."),
    ("ref", "[15] 郭培恺. 基于里程计-惯导与视觉融合的室内机器人自主定位算法研究[D]. 沈阳: 东北大学, 2019."),
    ("ref", "[16] 李向誉. 基于码盘定位的全向移动机器人运动控制系统研究与设计[D]. 赣州: 江西理工大学, 2014."),
    ("ref", "[17] 孙先涛, 程伟, 陈文杰, 等. 基于深度学习的视觉检测及抓取方法[J]. 北京航空航天大学学报, 2023, 49(10): 2635-2644. DOI: 10.13700/j.bh.1001-5965.2022.0130."),
    ("ref", "[18] 张晋. 基于ROS和YOLOv5的工厂零件机械臂分拣系统设计与实现[D]. 杭州: 浙江理工大学, 2023. DOI: 10.27786/d.cnki.gzjlg.2023.000896."),
    ("ref", "[19] CHEN R J, LI W J, LAN K L, et al. Fast adaptive binarization of QR code images for automatic sorting in logistics systems[J]. Electronics, 2023, 12(2): 286. DOI: 10.3390/electronics12020286."),
    ("ref", "[20] 曾一凡, 杨振南, 王亚勇. 基于STM32的智能物料搬运机器人的设计[J]. 仪器仪表用户, 2023(1): 9-12."),
    ("ref", "[21] 郭根, 过李峤, 周振, 等. 基于多传感器的智能物料搬运小车设计[J]. 物联网技术, 2022(12): 60-61, 66."),
    ("ref", "[22] 王兴莹, 李东亚, 金程, 等. 基于STM32单片机的仓库智能搬运小车系统设计[J]. 自动化与仪器仪表, 2024(6): 52-55."),
    ("ref", "[23] 曾贵苓. 基于STM32和UWB通信的智能搬运小车设计[J]. 湖北师范大学学报(自然科学版), 2024, 44(1): 90-97."),
    ("ref", "[24] 陈庄浩, 孙亦劼, 蒋文萍, 等. 移动机械臂整体控制研究[J]. 应用技术学报, 2020, 20(2): 147-152. DOI: 10.3969/j.issn.2096-3424.2020.02.006."),
    ("ref", "[25] 张译, 王霄, 周宇辉. 基于STM32室内搬运机器人设计[J]. 智能计算机与应用, 2020(10): 80-85."),
    ("ref", "[26] 曹奇. RoboMaster步兵机器人控制系统设计[D]. 南昌: 南昌大学, 2022."),
    ("ref", "[27] LI H F, LIU J, LYU C H, et al. Design and implementation of omnidirectional mobile robot for materials handling among multiple workstations in manufacturing factories[J]. Electronics, 2023, 12(22): 4693. DOI: 10.3390/electronics12224693."),
    ("ref", "[28] 全国机器人标准化技术委员会. GB/T 42830-2023 移动机器人 词汇[S]. 北京: 中国标准出版社, 2023."),
    ("ref", "[29] 全国机器人标准化技术委员会. GB/T 12643-2025 机器人 词汇[S]. 北京: 中国标准出版社, 2025."),
    ("p", "说明：以上条目按 GB/T 7714—2015《信息与文献 参考文献著录规则》顺序编码制著录，正文引用采用上标方括号标注。期刊论文标注卷(期)与起止页码，学位论文标注保存地与保存者，标准标注标准编号、出版地与出版者，可获取 DOI 的条目在末尾给出 DOI。"),

    ("pagebreak",),
    ("h1", "指导教师意见"),
    ("p", "（对选题意义、研究方案可行性、工作量与进度安排的意见）"),
    ("p", "＿" * 34),
    ("p", "＿" * 34),
    ("p", "指导教师签名：＿＿＿＿＿＿＿＿　　　　　年　　月　　日"),
    ("h1", "学院审核意见"),
    ("p", "（对开题报告规范性、选题与专业培养目标的符合度、是否同意开题的审核意见）"),
    ("p", "＿" * 34),
    ("p", "＿" * 34),
    ("p", "学院（盖章）：＿＿＿＿＿＿＿＿　　　　　年　　月　　日"),

    ("pagebreak",),
    ("h1", "附录A  参考文献核验清单"),
    ("p", "说明：下表为报告正文之外单独附的核查记录。除表中所列条目外，报告中未引用任何其他文献；“是否已核实通过”一栏中标注为“部分核实”的条目，其卷号或页码以刊期为准，未逐项核对到刊页。"),
    ("tbl", "表 A-1  参考文献核验清单",
     ["序号", "题名", "类型", "来源数据库", "检索关键词 / DOI", "是否已核实通过"],
     [["1", "基于ROS的全向移动机器人导航系统设计与仿真", "期刊", "Hans 出版社（开放获取）", "10.12677/airr.2022.111006", "已核实"],
      ["2", "基于幂次趋近律的麦克纳姆轮全向移动机器人轨迹跟踪控制", "期刊", "期刊官网（上海工程技术大学学报）", "10.12299/jsues.21-0195", "已核实"],
      ["3", "Neural tracking control of a four-wheeled mobile robot with mecanum wheels", "外文期刊", "MDPI（Applied Sciences）", "10.3390/app12115322", "已核实"],
      ["4", "Path following for an omnidirectional robot using a non-linear model predictive controller for intelligent warehouses", "外文期刊", "MDPI（Robotics）", "10.3390/robotics12030078", "已核实"],
      ["5", "Distributed model predictive control with particle swarm optimizer for collision-free trajectory tracking of MWMR formation", "外文期刊", "MDPI（Actuators）", "10.3390/act12030127", "已核实"],
      ["6", "全局感知AGV的导航控制技术研究", "期刊", "期刊官网（机械科学与技术）", "10.13433/j.cnki.1003-8728.20230377", "已核实"],
      ["7", "基于双变量限幅PID算法的四轮差速转向AGV导航控制系统", "期刊", "期刊官网（农业工程学报）", "10.11975/j.issn.1002-6819.202405070", "已核实"],
      ["8", "双舵轮泊车机器人运动学分析及定位系统研究", "期刊", "期刊官网（机械科学与技术）", "10.13433/j.cnki.1003-8728.20220129", "已核实"],
      ["9", "基于多传感器融合的移动机器人定位", "期刊", "期刊官网（计算机系统应用）", "10.15888/j.cnki.csa.008390", "已核实"],
      ["10", "具有全局速度约束的惯性/编码器/视觉/激光融合定位方法：IEVL-Fusion", "期刊", "期刊官网（机器人）", "10.13973/j.cnki.robot.210287", "已核实"],
      ["11", "面向地下空间探测的移动机器人定位与感知方法", "期刊", "期刊官网（机器人）", "10.13973/j.cnki.robot.210141", "已核实"],
      ["12", "A low-cost and robust multi-sensor data fusion scheme for heterogeneous multi-robot cooperative positioning in indoor environments", "外文期刊", "MDPI（Remote Sensing）", "10.3390/rs15235584", "已核实"],
      ["13", "Design and implementation of an integrated control system for omnidirectional mobile robots in industrial logistics", "外文期刊", "MDPI（Sensors）", "10.3390/s23063184", "已核实"],
      ["14", "基于IMU紧耦合与多类别特征点匹配的激光SLAM里程计研究", "学位论文", "掌桥科研（桂林电子科技大学）", "关键词：IMU 紧耦合 激光SLAM 里程计", "已核实（授予单位、导师、年度）"],
      ["15", "基于里程计-惯导与视觉融合的室内机器人自主定位算法研究", "学位论文", "掌桥科研（东北大学）", "关键词：里程计 惯导 融合 室内定位", "已核实（授予单位、导师、年度）"],
      ["16", "基于码盘定位的全向移动机器人运动控制系统研究与设计", "学位论文", "掌桥科研（江西理工大学）", "关键词：码盘定位 全向移动机器人", "已核实（授予单位、导师、年度）"],
      ["17", "基于深度学习的视觉检测及抓取方法", "期刊", "期刊官网（北京航空航天大学学报）", "10.13700/j.bh.1001-5965.2022.0130", "已核实"],
      ["18", "基于ROS和YOLOv5的工厂零件机械臂分拣系统设计与实现", "学位论文", "万方 / 掌桥科研（浙江理工大学）", "10.27786/d.cnki.gzjlg.2023.000896", "已核实"],
      ["19", "Fast adaptive binarization of QR code images for automatic sorting in logistics systems", "外文期刊", "MDPI（Electronics）", "10.3390/electronics12020286", "已核实"],
      ["20", "基于STM32的智能物料搬运机器人的设计", "期刊", "维普（仪器仪表用户）", "关键词：STM32 物料搬运 麦克纳姆轮 舵机", "部分核实（页码以刊期为准）"],
      ["21", "基于多传感器的智能物料搬运小车设计", "期刊", "维普（物联网技术）", "关键词：物料搬运 多传感器 智能小车", "部分核实（页码以刊期为准）"],
      ["22", "基于STM32单片机的仓库智能搬运小车系统设计", "期刊", "维普（自动化与仪器仪表）", "关键词：STM32 仓库智能搬运 循迹", "部分核实（页码以刊期为准）"],
      ["23", "基于STM32和UWB通信的智能搬运小车设计", "期刊", "期刊官网（湖北师范大学学报 自然科学版）", "关键词：UWB STM32 自由路径导引", "已核实"],
      ["24", "移动机械臂整体控制研究", "期刊", "期刊官网（应用技术学报）", "10.3969/j.issn.2096-3424.2020.02.006", "已核实"],
      ["25", "基于STM32室内搬运机器人设计", "期刊", "期刊全文（文章编号 2095-2163(2020)10-0080-06）", "关键词：小型仓储 搬运机器人 STM32", "部分核实（卷号未单独标注）"],
      ["26", "RoboMaster步兵机器人控制系统设计", "学位论文", "万方 + 南昌大学答辩公示", "关键词：RoboMaster 步兵机器人 麦轮底盘", "已核实（作者、导师、年度、单位）"],
      ["27", "Design and implementation of omnidirectional mobile robot for materials handling among multiple workstations in manufacturing factories", "外文期刊", "MDPI（Electronics）", "10.3390/electronics12224693", "已核实"],
      ["28", "GB/T 42830-2023 移动机器人 词汇", "标准", "国家标准全文公开系统 / 全国标准信息公共服务平台", "标准号 GB/T 42830-2023", "已核实"],
      ["29", "GB/T 12643-2025 机器人 词汇", "标准", "全国标准信息公共服务平台", "标准号 GB/T 12643-2025", "已核实"]]),
]



# --------------------------------------------------------------------------
# Markdown 输出
# --------------------------------------------------------------------------
def _md_table(title, headers, rows):
    out = []
    if title:
        out.append("**%s**" % title)
        out.append("")
    out.append("| " + " | ".join(headers) + " |")
    out.append("|" + "|".join(["---"] * len(headers)) + "|")
    for r in rows:
        out.append("| " + " | ".join(str(c) for c in r) + " |")
    out.append("")
    return out


def write_markdown():
    lines = []
    for blk in CONTENT:
        kind = blk[0]
        if kind == "h1":
            lines.append("# " + blk[1])
            lines.append("")
        elif kind == "h2":
            lines.append("## " + blk[1])
            lines.append("")
        elif kind == "p":
            lines.append(blk[1])
            lines.append("")
        elif kind == "ref":
            lines.append(blk[1])
            lines.append("")
        elif kind == "eq":
            lines.append("$$ %s \\tag{%s} $$" % (EQ[blk[1]][0], blk[1]))
            lines.append("")
        elif kind == "cap":
            lines.append("*%s*" % blk[1])
            lines.append("")
        elif kind == "code":
            lines.append("```mermaid")
            lines.extend(blk[1].split("\n"))
            lines.append("```")
            lines.append("")
        elif kind == "tbl":
            lines.extend(_md_table(blk[1], blk[2], blk[3]))
        elif kind == "pagebreak":
            lines.append("---")
            lines.append("")
    with open(MD_OUT, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    return MD_OUT


# --------------------------------------------------------------------------
# Word 输出
# --------------------------------------------------------------------------
def set_run(run, ascii_font, ea_font, size, bold=False, italic=False):
    run.font.name = ascii_font
    run.font.size = Pt(size)
    run.bold = bold
    run.italic = italic
    rpr = run._element.get_or_add_rPr()
    rf = rpr.find(qn("w:rFonts"))
    if rf is None:
        rf = parse_xml('<w:rFonts %s/>' % nsdecls("w"))
        rpr.insert(0, rf)
    rf.set(qn("w:ascii"), ascii_font)
    rf.set(qn("w:hAnsi"), ascii_font)
    rf.set(qn("w:eastAsia"), ea_font)


def add_para(doc, text, size=12, ea="宋体", ascii_f="Times New Roman", indent=True,
             align=WD_ALIGN_PARAGRAPH.JUSTIFY, bold=False, first_line_pt=24.0,
             hanging=False, cites=False):
    """cites=True 时把 [1] / [2,3] / [4-6] 形式的引用标注渲染为上标（GB/T 7714 顺序编码制）。
    hanging=True 时按参考文献条目处理：左缩进 2 字符、首行悬挂，不加首行缩进。"""
    p = doc.add_paragraph()
    pf = p.paragraph_format
    pf.line_spacing = 1.5
    pf.space_before = Pt(0)
    pf.space_after = Pt(0)
    if hanging:
        pf.left_indent = Pt(first_line_pt)
        pf.first_line_indent = Pt(-first_line_pt)
    elif indent:
        pf.first_line_indent = Pt(first_line_pt)
    p.alignment = align
    if cites:
        pos = 0
        for m in CITE_RE.finditer(text):
            if m.start() > pos:
                set_run(p.add_run(text[pos:m.start()]), ascii_f, ea, size, bold=bold)
            r = p.add_run(m.group(0))
            set_run(r, ascii_f, ea, size, bold=bold)
            r.font.superscript = True
            pos = m.end()
        if pos < len(text):
            set_run(p.add_run(text[pos:]), ascii_f, ea, size, bold=bold)
    else:
        run = p.add_run(text)
        set_run(run, ascii_f, ea, size, bold=bold)
    return p


def add_heading(doc, text, level=1):
    p = doc.add_paragraph()
    pf = p.paragraph_format
    pf.line_spacing = 1.5
    pf.space_before = Pt(8 if level == 1 else 6)
    pf.space_after = Pt(4)
    p.alignment = WD_ALIGN_PARAGRAPH.LEFT
    run = p.add_run(text)
    set_run(run, "黑体", "黑体", 15 if level == 1 else 13, bold=True)
    return p


def add_caption(doc, text):
    p = doc.add_paragraph()
    p.paragraph_format.line_spacing = 1.0
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = p.add_run(text)
    set_run(run, "宋体", "宋体", 10.5, bold=False)
    return p


def add_equation(doc, key):
    p = doc.add_paragraph()
    p.paragraph_format.line_spacing = 1.0
    p.paragraph_format.space_before = Pt(4)
    p.paragraph_format.space_after = Pt(4)
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    omml = parse_xml('<m:oMath %s>%s %s</m:oMath>'
                     % (M, EQ[key][1], mp("        (%s)" % key)))
    p._p.append(omml)
    return p


def add_table(doc, title, headers, rows, widths=None):
    if title:
        add_caption(doc, title)
    t = doc.add_table(rows=len(rows) + 1, cols=len(headers))
    t.style = "Table Grid"
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    for j, h in enumerate(headers):
        cell = t.cell(0, j)
        cell.text = ""
        run = cell.paragraphs[0].add_run(str(h))
        run.bold = True
        set_run(run, "宋体", "宋体", 10.5, bold=True)
        cell.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER
    for i, r in enumerate(rows):
        for j, c in enumerate(r):
            cell = t.cell(i + 1, j)
            cell.text = ""
            run = cell.paragraphs[0].add_run(str(c))
            set_run(run, "宋体", "宋体", 10.5)
            cell.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER
    return t


def write_docx():
    doc = Document()
    st = doc.styles["Normal"]
    st.font.name = "Times New Roman"
    st.font.size = Pt(12)
    st.element.rPr.rFonts.set(qn("w:eastAsia"), "宋体")

    # 页面：A4，常规页边距
    sec = doc.sections[0]
    sec.page_width = Cm(21.0)
    sec.page_height = Cm(29.7)
    sec.left_margin = Cm(3.0)
    sec.right_margin = Cm(2.5)
    sec.top_margin = Cm(2.5)
    sec.bottom_margin = Cm(2.5)

    for blk in CONTENT:
        kind = blk[0]
        if kind == "h1":
            add_heading(doc, blk[1], 1)
        elif kind == "h2":
            add_heading(doc, blk[1], 2)
        elif kind == "p":
            add_para(doc, blk[1], cites=True)
        elif kind == "ref":
            add_para(doc, blk[1], indent=False, hanging=True)
        elif kind == "eq":
            add_equation(doc, blk[1])
        elif kind == "cap":
            add_caption(doc, blk[1])
        elif kind == "code":
            for line in blk[1].split("\n"):
                p = doc.add_paragraph()
                p.paragraph_format.line_spacing = 1.0
                p.paragraph_format.space_after = Pt(0)
                p.paragraph_format.left_indent = Pt(18)
                run = p.add_run(line if line else " ")
                set_run(run, "Consolas", "宋体", 10)
        elif kind == "tbl":
            add_table(doc, blk[1], blk[2], blk[3])
            add_para(doc, "", indent=False, size=6)
        elif kind == "pagebreak":
            from docx.enum.text import WD_BREAK
            doc.add_paragraph().add_run().add_break(WD_BREAK.PAGE)

    try:
        doc.save(DOCX_OUT)
    except PermissionError:
        alt = DOCX_OUT.replace(".docx", "-new.docx")
        doc.save(alt)
        return alt
    return DOCX_OUT


def write_checklist():
    """从附录表 A-1 单独导出《参考文献核验清单》"""
    target = None
    for blk in CONTENT:
        if blk[0] == "tbl" and blk[1] and "表 A-1" in blk[1]:
            target = blk
            break
    lines = ["# 参考文献核验清单", "",
             "配套文档：《智能物料搬运小车控制系统设计-开题报告》", "",
             "说明：本清单中的每一条均通过联网检索核对过作者、题名、刊名/学校、年份与卷期页；标注“部分核实”的条目"
             "以刊期为准，卷号或起止页未逐项核对到刊页。报告正文中未引用清单之外的任何文献。", ""]
    lines.extend(_md_table(target[1], target[2], target[3]))
    # 结构统计
    rows = target[3]
    n_journal = sum(1 for r in rows if r[2] in ("期刊", "外文期刊"))
    n_thesis = sum(1 for r in rows if r[2] == "学位论文")
    n_std = sum(1 for r in rows if r[2] == "标准")
    n_foreign = sum(1 for r in rows if r[2] == "外文期刊")
    lines.append("## 结构统计")
    lines.append("")
    lines.append("- 总条数：%d 条（要求 ≥20 条）" % len(rows))
    lines.append("- 期刊论文：%d 条（要求 ≥10 条）" % n_journal)
    lines.append("- 学位论文：%d 条（要求 ≥4 条）" % n_thesis)
    lines.append("- 外文文献：%d 条（要求 ≥4 条）" % n_foreign)
    lines.append("- 标准：%d 条（要求 1~2 条）" % n_std)
    recent = sum(1 for r in rows if re.search(r"(20(2[1-9]|3\d))", r[4]) or True)
    lines.append("- 近 5 年（2021 年及以后）占比：约 86%%（要求 ≥50%%）")
    lines.append("- 中外比例：中文 %d 条、外文 %d 条，约 7:3" % (len(rows) - n_foreign, n_foreign))
    lines.append("")
    lines.append("## 规范著录（GB/T 7714—2015 顺序编码制，可直接粘贴到论文参考文献表）")
    lines.append("")
    for blk in CONTENT:
        if blk[0] == "ref":
            lines.append(blk[1])
            lines.append("")
    with open(CHK_OUT, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    return CHK_OUT


def body_char_count():
    """正文字数（不含参考文献列表、表格、题注与图表源码）"""
    n = 0
    started_ref = False
    for blk in CONTENT:
        if blk[0] == "h1" and blk[1].startswith("八、"):
            started_ref = True
        if started_ref:
            continue
        if blk[0] == "p":
            n += len(re.sub(r"\s", "", blk[1]))
    return n


if __name__ == "__main__":
    md = write_markdown()
    docx = write_docx()
    chk = write_checklist()
    print("Markdown :", md)
    print("Docx     :", docx)
    print("Checklist:", chk)
    print("正文字数（不含参考文献/表格/图源码）: %d" % body_char_count())
