# -*- coding: utf-8 -*-
"""生成项目功能与实现总结 docx。"""
import os
from docx import Document
from docx.shared import Pt, RGBColor, Cm
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml.ns import qn

# 输出到本脚本上一级目录(docs/),与脚本一起随仓库搬动。
OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "项目功能与实现总结_2026-10-12.docx")

doc = Document()


def _ea(run, name="Microsoft YaHei"):
    """设置字体（含中文），必须在设置过 font.name 之后调用。"""
    run.font.name = name
    rpr = run._element.get_or_add_rPr()
    rf = rpr.find(qn("w:rFonts"))
    if rf is None:
        rf = rpr.makeelement(qn("w:rFonts"), {})
        rpr.append(rf)
    rf.set(qn("w:eastAsia"), name)
    rf.set(qn("w:ascii"), name)
    rf.set(qn("w:hAnsi"), name)
    return run

# ---- 全局样式 ----
st = doc.styles["Normal"]
st.font.name = "Microsoft YaHei"
st.font.size = Pt(10.5)
_ea(st)
st.paragraph_format.space_after = Pt(4)
st.paragraph_format.line_spacing = 1.25

for name, size, color in (("Heading 1", 16, 0x1F3864), ("Heading 2", 13, 0x2E5395),
                          ("Heading 3", 11.5, 0x404040)):
    s = doc.styles[name]
    s.font.name = "Microsoft YaHei"
    s.font.size = Pt(size)
    s.font.color.rgb = RGBColor((color >> 16) & 255, (color >> 8) & 255, color & 255)
    _ea(s)
    s.paragraph_format.space_before = Pt(10)
    s.paragraph_format.space_after = Pt(5)


def _rich(par, text, base_bold=False):
    """把 **xxx** 渲染为加粗 run。"""
    import re as _re
    for i, seg in enumerate(_re.split(r'\*\*(.+?)\*\*', text)):
        if not seg:
            continue
        r = par.add_run(seg)
        r.bold = base_bold or (i % 2 == 1)
        _ea(r)
    return par


def p(text, bold=False, indent=0, size=None, color=None):
    par = doc.add_paragraph()
    par.paragraph_format.left_indent = Cm(0.5 * indent)
    _rich(par, text, base_bold=bold)
    if size or color:
        for r in par.runs:
            if size:
                r.font.size = Pt(size)
            if color:
                r.font.color.rgb = RGBColor(*color)
    return par


def bullets(items, indent=0):
    for it in items:
        if isinstance(it, tuple):          # (加粗前缀, 正文)
            par = doc.add_paragraph(style="List Bullet")
            par.paragraph_format.left_indent = Cm(0.75 + 0.5 * indent)
            r = par.add_run(it[0]); r.bold = True; _ea(r)
            _rich(par, it[1])
        else:
            par = doc.add_paragraph(style="List Bullet")
            par.paragraph_format.left_indent = Cm(0.75 + 0.5 * indent)
            _rich(par, it)


def table(headers, rows, widths=None):
    t = doc.add_table(rows=1, cols=len(headers))
    t.style = "Table Grid"
    hdr = t.rows[0].cells
    for i, h in enumerate(headers):
        hdr[i].text = ""
        r = hdr[i].paragraphs[0].add_run(h)
        r.bold = True
        r.font.size = Pt(10)
        _ea(r)
    for row in rows:
        cells = t.add_row().cells
        for i, v in enumerate(row):
            cells[i].text = ""
            r = cells[i].paragraphs[0].add_run(str(v))
            r.font.size = Pt(9.5)
            _ea(r)
    if widths:
        for row in t.rows:
            for i, w in enumerate(widths):
                row.cells[i].width = Cm(w)
    doc.add_paragraph()
    return t


def code(lines):
    for i, ln in enumerate(lines):
        par = doc.add_paragraph()
        par.paragraph_format.left_indent = Cm(0.8)
        par.paragraph_format.space_after = Pt(0)
        par.paragraph_format.line_spacing = 1.0
        r = par.add_run(ln)
        r.font.name = "Consolas"
        rpr = r._element.get_or_add_rPr(); rf = rpr.makeelement(qn("w:rFonts"), {}); rf.set(qn("w:eastAsia"), "Consolas"); rf.set(qn("w:ascii"), "Consolas"); rf.set(qn("w:hAnsi"), "Consolas"); rpr.append(rf)
        r.font.size = Pt(9)
    doc.add_paragraph()


# ================= 封面 =================
title = doc.add_paragraph()
title.alignment = WD_ALIGN_PARAGRAPH.CENTER
tr = title.add_run("ROS2 远程操控上位机")
tr.bold = True
tr.font.size = Pt(24)
tr.font.color.rgb = RGBColor(0x1F, 0x38, 0x64)
_ea(tr)

sub = doc.add_paragraph()
sub.alignment = WD_ALIGN_PARAGRAPH.CENTER
sr = sub.add_run("项目功能与实现总结")
sr.font.size = Pt(15)
_ea(sr)

info = doc.add_paragraph()
info.alignment = WD_ALIGN_PARAGRAPH.CENTER
ir = info.add_run("版本：2026-10-12    适用平台：ROS2 Humble + Qt5 + Docker\n"
                  "上位机：Linux PC（x86_64）    下位机：MentorPi 小车（ARM64 / Raspberry Pi）")
ir.font.size = Pt(10)
ir.font.color.rgb = RGBColor(0x59, 0x59, 0x59)
_ea(ir)

doc.add_paragraph()

# ================= 1. 项目概览 =================
doc.add_heading("一、项目概览", level=1)
p("本项目是一套面向 ROS2 移动机器人的**远程操控上位机**：在 PC 上运行一个 Qt 图形界面，"
  "通过 ZeroTier 虚拟局域网跨城连接 Raspberry Pi 小车，完成建图、导航、遥控、地图编辑与算法调试的全流程操作。"
  "项目已实现从零搭建到跨城端到端跑通（2026-10-12 验证通过）。")
p("核心设计原则：", bold=True)
bullets([
    "上位机只负责「显示 + 人机交互 + 决策」，不直接驱动硬件；",
    "所有需要长期运行、贴近硬件的逻辑，做成小车端常驻服务；",
    "两端只通过自定义 topic 与 ROS2 action 通信，接口最小化；",
    "跨城网络靠 ZeroTier 打底、FastDDS Discovery Server 解决节点发现问题。",
])

doc.add_heading("1.1 系统结构", level=2)
table(
    ["层级", "位置", "组成", "职责"],
    [
        ["上位机", "PC / Docker 容器 rcj_gui_humble", "Qt5 界面 + rclcpp 节点",
         "界面渲染、人机交互、订阅话题、下发指令、地图存取"],
        ["网络", "ZeroTier 虚拟局域网", "FastDDS + Discovery Server",
         "跨城节点发现与数据传输"],
        ["下位机", "MentorPi 小车 / Docker 容器 MentorPi", "4 个 rclpy 常驻服务 + ROS2 节点",
         "建图、Nav2 导航、速度看门狗、彩色相机中继"],
        ["硬件", "Raspberry Pi + 传感器", "激光雷达、USB 摄像头、底盘",
         "感知与执行"],
    ],
    widths=[1.8, 4.0, 4.5, 6.0],
)

# ================= 2. 各功能实现 =================
doc.add_heading("二、各功能实现说明", level=1)
p("本章逐个说明每个功能「是怎么实现的」——采用什么机制、为什么这么做、以及踩过的坑。")

doc.add_heading("2.1 程序骨架与连接生命周期", level=2)
p("GUI 是 Qt5 与 rclcpp 运行在同一个进程内：main() 先创建 QApplication，随后在独立线程中"
  "运行 MultiThreadedExecutor 进行 spin；Qt 主线程负责界面。两边通过信号槽通信——ROS 回调中不直接操作控件，"
  "一律发信号交由 UI 线程处理。")
bullets([
    ("跨线程类型必须注册：", "传递给信号的自定义类型（代价地图、粒子云等容器）需在 main() 中 qRegisterMetaType，"
     "否则信号会被 Qt 静默丢弃，表现为界面永不刷新。"),
    ("延迟初始化：", "不点「连接」就不创建 ROS 节点，连接时才一次性建好订阅、发布者与 action client，"
     "断开时整体销毁，避免半连接状态。"),
    ("话题名不写死：", "全部从 robots/<名称>.yaml 配置档读取（经 RobotProfile 解析），换车只改 yaml，不改代码。"),
])

doc.add_heading("2.2 遥控（控制页）", level=2)
bullets([
    ("速度合成：", "摇杆/键盘产生归一化的线速度与角速度 (vx, vy, wz)，乘以「速度档位」滑块换算为真实 m/s，"
     "封装为 geometry_msgs/Twist 发布到 cmd_vel。"),
    ("10Hz 持续重发：", "不是按下才发。GUI 侧定时器 10Hz 重发，且只在当前值非零时发（零值不发，防止把小车钉死）。"
     "这是「停不下来」故障的修法之一——原先单发一次，网络抖动会让小车持续运动。"),
    ("小车侧看门狗：", "cmd_vel_watchdog.py 常驻监听 cmd_vel，超过阈值时间没有新指令就自动发零速刹车。"
     "GUI 崩溃或网络断开时小车都会停。"),
    ("急停：", "急停按钮直接下发零速，并同时清零 GUI 侧的重发目标值，避免定时器下一拍又把它推走。"),
])

doc.add_heading("2.3 相机画面", level=2)
bullets([
    ("基本链路：", "订阅 CompressedImage（SensorDataQoS / best_effort），回调中用 QImage::loadFromData 解析 JPEG，"
     "交给 QLabel 缩放显示，并统计帧率。"),
    ("彩色修复：", "根因不在 GUI，而在小车侧——compressed_image_transport 对 yuv422_yuy2 只提取 Y 通道，"
     "因此发布出的 JPEG 本身就是灰度的。解决方式是在小车上新增 camera_color_relay.py：订阅原始 YUYV 帧，"
     "用 OpenCV 转 BGR 再编码 JPEG，发布到新话题，GUI 配置档改订新话题。A/B 实测旧话题彩色像素占比 0.0%，新话题 39.1%。"),
    ("中环节流改造：", "采用「定时器 + 最新帧缓存」：回调只更新 latest，定时器按目标帧率取最新帧转换。"
     "最初在回调里做时间限速，会被摄像头约 20Hz 的帧网格量化成 6~7fps。"),
])

doc.add_heading("2.4 渲染与交互（中央视图）", level=2)
bullets([
    ("自绘而非场景图：", "视图控件自行维护「世界坐标 → 屏幕像素」变换（平移、缩放、Y 轴翻转），"
     "在 paintEvent 中按图层顺序绘制：地图 → 代价地图 → 激光点 → 粒子云 → 机器人位姿 → 目标点连线。"),
    ("图层开关：", "图层以位标志表示，勾选框改标志位后触发重绘，关闭某层零额外开销。"),
    ("鼠标交互：", "左键平移、滚轮缩放、Shift+左键选点、中键拖拽、右键菜单。"
     "「设为初始位姿」需要点两次才生效，是已知未修问题（第一次点击被当作进入画点模式消耗掉）。"),
    ("地图 QoS：", "/map 使用 transient_local，后连接上的 GUI 也能立即拿到最后一帧地图。"),
])

doc.add_heading("2.5 建图（SLAM）", level=2)
bullets([
    ("职责划分：", "GUI 发布自定义主题 /start_slam（携带算法名、分辨率、是否清图），小车端 slam_controller.py 接收后"
     "以 subprocess 拉起真正的 slam_toolbox / cartographer 节点，并通过 /slam_status 回传状态。"
     "如此设计是因为小车端进程独立更易排查，且 ARM 平台上不便在 GUI 侧运行 launch。"),
    ("四种建图方式：", "服务内部按 slam_toolbox 在线/离线、cartographer 在线/离线分支，拼接不同的 launch 参数。"),
    ("存图在 GUI 侧：", "GUI 收到 /map 后已持有完整栅格，保存时由 GUI 侧把 OccupancyGrid 写成 PGM + YAML，"
     "因此小车服务退出后仍可保存已获取的地图。"),
])

doc.add_heading("2.6 导航（Nav2）", level=2)
bullets([
    ("服务拉起 Nav2：", "小车端 nav_controller.py 负责拉起与停止 Nav2（robot_nav2.sh）；GUI 通过自定义主题请求"
     "「启动 / 停止 / 应用参数」，服务执行完毕后回传状态。"),
    ("设置目标点：", "在视图上 Shift+左键选点，GUI 通过 NavigateToPose action client 发送目标，"
     "action 的 feedback 回调用于更新进度与状态文字；取消即调用 cancel_goal，而非发零速。"),
    ("初始位姿：", "选区后换算为 PoseWithCovarianceStamped 发布到 /initialpose，由 AMCL 接收。"),
    ("参数与算法切换：", "面板参数写入 nav2_params_gui.yaml 并下发到小车后重启 Nav2；切换导航算法走同一套流程。"
     "该链路存在已知问题：重启偶发卡死，表现为僵尸容器 + Discovery Server 幽灵条目，清理残留即可恢复。"),
])

doc.add_heading("2.7 代价地图与粒子云", level=2)
bullets([
    ("实现方式：", "订阅两个话题——OccupancyGrid（代价地图）与 PoseArray（AMCL 粒子），转换为 Qt 侧结构后自绘。"),
    ("元类型注册：", "由于经信号跨线程传递自定义容器，必须注册元类型，否则一条都画不出来。"),
    ("已知未修：", "停止 Nav2 后这两个图层不会清空，仍显示最后一帧——缺少「对端已消失」事件，只能靠超时清除。"),
])

doc.add_heading("2.8 SLAM 调参面板", level=2)
bullets([
    ("请求/响应式：", "不维持长连接。「读取」发送请求，服务读取当前 slam 节点参数后回传；「应用」再写回。"),
    ("取舍：", "服务无需维护状态；代价是每次都要等待往返，因此面板带超时与「正在读取」状态提示。"),
    ("生效方式：", "多数参数（如 map_update_interval）修改后立即生效，无需重启节点。"),
])

doc.add_heading("2.9 自定义算法插件", level=2)
bullets([
    ("生成骨架：", "在 custom_algo 页面填写名称、选择模板类型后生成一个 ROS2 功能包骨架，"
     "内含 plugin.xml 与继承 Nav2 planner/controller 接口的 C++ 类。"),
    ("远程编译：", "「部署」调用 deploy_algo.sh：打包 → scp 到小车 → 在小车容器内 colcon build（ARM64 目标，"
     "PC 为 x86_64，不能本地交叉编译）→ source 新 workspace。"),
    ("易错点：", "plugin.xml 的安装路径必须与 nav2_params_gui.yaml 中的插件名严格对应，否则 Nav2 无法加载。"),
])

doc.add_heading("2.10 地图编辑", level=2)
bullets([
    ("纯函数核心：", "地图编辑算法抽取为不依赖 Qt 与 ROS 的纯函数模块：输入 OccupancyGrid + 笔刷（圆形/矩形）+ 操作点，"
     "输出新栅格。因此可以独立编写单元测试。"),
    ("先冻结后编辑：", "建图过程中点击编辑会先对当前 /map 做快照作为底图，在快照上擦除或补画，"
     "点击保存才写出 PGM + YAML，避免编辑结果被下一帧实时地图覆盖。"),
])

doc.add_heading("2.11 日志面板", level=2)
bullets([
    ("数据来源：", "订阅 /rosout（本机与小车节点日志皆在此），按级别过滤、按来源分类显示；"
     "GUI 自身的操作事件也写入同一模型，便于将「导航失败」与之前的「下发目标点」对应起来查看。"),
    ("导出：", "导出 CSV 为另行写出表格内容，尚未验证真实落盘。"),
    ("已知问题：", "时间戳比本地时间早 8 小时（容器未设置时区），修法是给容器加 -e TZ=Asia/Shanghai。"),
])

doc.add_heading("2.12 设置页与配置档", level=2)
bullets([
    ("功能：", "选择小车 IP（支持列表与自动探测，优先尝试 ZeroTier 虚拟 IP）、切换配置档、保存配置。"),
    ("已知问题一：", "「测试 ROS 连接」以 QProcess 执行 ping；容器内无 ping，且 QProcess 启动失败只触发 errorOccurred"
     "而不触发 finished，界面会永久停留在「正在 ping」。"),
    ("已知问题二：", "保存时 ip_combo 为局部变量，取不到实际值，导致 network/robot_ip 写入旧值。"),
])

doc.add_heading("2.13 部署与一键运行", level=2)
bullets([
    ("镜像内编译：", "Dockerfile.humble 基于 Ubuntu 22.04 + ROS2 Humble + Qt5 及全部依赖，"
     "colcon build 在镜像构建阶段完成，因此现场运行 GUI 无需编译。"),
    ("一键启动 GUI：", "run_humble_gui.sh 完成：处理 xauth → 同步小车时钟 → 删除旧容器 → 以 --net=host 启动新容器"
     "（挂载 maps/、robots/、.gui_home/、custom_algo/）→ 传入自动连接 IP。"),
    ("一键起小车服务：", "start_robot_services.sh 在 PC 上执行，通过 ssh + docker exec -d 拉起小车 4 个常驻服务。"
     "必须使用 -d，普通 nohup & 会在 exec 退出时被一并回收。"),
    ("服务代码更新：", "服务文件既未挂载也无热加载，修改后必须「写临时文件 → py_compile → md5 对比 → 重启进程」，"
     "否则改动不生效。注意 pkill -f xxx.py 写在 bash -c 内会匹配到自身命令行而杀掉该 shell，"
     "导致后续命令全部不执行但 ssh 仍返回 0——必须用 md5 校验确认真实落盘。"),
])

doc.add_heading("2.14 跨城网络", level=2)
bullets([
    ("组网：", "小车与 PC 加入同一 ZeroTier 网络（小车 10.132.156.106，PC 10.132.156.182），物理上跨城。"),
    ("发现机制：", "不使用默认多播（跨城多播易被网络设备阻断），两端统一指向小车上的 Discovery Server："
     "小车容器 fastdds_discovery 监听 11811 端口，小车自身节点使用 127.0.0.1:11811，PC 侧使用 10.132.156.106:11811，"
     "由 setup_robot_dds.sh 统一管理。优化后 odom 首帧时延由 13.1s 降至 0.9s。"),
    ("注意事项：", "Discovery Server 内部表曾被客户端条目扰乱（数据面通但节点数暴跌），需做完整 DDS 复位；"
     "ros2 topic list 在小车容器内会卡死，判断链路必须使用 rclpy 探针脚本。"),
])

# ================= 3. 工作量 =================
doc.add_heading("三、工作量统计", level=1)
table(
    ["模块", "内容", "规模"],
    [
        ["上位机 GUI", "Qt 界面 + ROS 封装，50 个 cpp/h 文件", "6748 行"],
        ["小车端服务", "slam_controller / nav_controller / cmd_vel_watchdog / camera_color_relay", "798 行"],
        ["脚本与工具", "启动、部署、DDS、探针等 shell 与 Python 工具", "1011 行"],
        ["自定义算法插件", "Nav2 插件模板与示例", "193 行"],
        ["测试", "地图编辑等单元测试", "278 行"],
        ["文档", "README / NAV2 / SLAM_CONTROLLER / REMOTE_NETWORK / 使用说明 / 进度报告", "6 份"],
    ],
    widths=[3.5, 9.0, 3.0],
)
p("项目始于 2026-10-03，主体开发集中在 2026-10-04（7 次提交），其后为联调与稳定性修复阶段。")

# ================= 4. 问题与解决 =================
doc.add_heading("四、典型问题与解决方式", level=1)
table(
    ["问题现象", "根本原因", "解决方式"],
    [
        ["遥操作松手后小车不停", "单发 cmd_vel 无持续信号，网络抖动即失控",
         "GUI 侧 10Hz 重发（仅非零速）+ 小车端 cmd_vel_watchdog 兜底刹车"],
        ["相机画面为黑白", "compressed_image_transport 对 yuv422_yuy2 只取 Y 通道",
         "小车端新增 camera_color_relay，转 BGR 后重新编码 JPEG"],
        ["相机帧率仅 6~7fps", "在回调内做时间限速，被约 20Hz 帧网格量化",
         "改为定时器 + 最新帧缓存的两级结构"],
        ["重连后 GUI 死机", "跨 ROS2 发行版（Jazzy 混用）导致 DDS 不兼容",
         "上位机统一走 Humble 容器，跨版本方案已废弃"],
        ["跨城发现慢 / 时通时断", "默认多播跨城不可达", "两端统一 FastDDS Discovery Server"],
        ["激光数据被 SLAM 拒收", "雷达帧长抖动，超出 SLAM 容忍范围", "已修复（帧长规范化）"],
        ["服务改动后无效果", "服务无热加载，且 pkill 自匹配导致部署静默失败",
         "改为「写文件 → 编译校验 → md5 对比 → 重启」流程，pkill 使用方括号规避自匹配"],
        ["切换算法重启 Nav2 卡死", "僵尸容器 + Discovery Server 幽灵条目", "清理残留容器与 DDS 状态后恢复"],
        ["小车每 60 秒自动重启", "厂家预置 wifi.service 定时重启网络",
         "在 SD 卡上停用其自启动，并补回热点/有线配置"],
        ["磁盘写满", "Docker 镜像与容器占用根分区", "清理 Docker 资源（需先列出清单确认）"],
        ["/tmp 堆积十万余文件", "launch 参数临时文件未清理", "已清理，待加入定期清理机制"],
    ],
    widths=[4.5, 5.5, 6.0],
)

# ================= 5. 现状 =================
doc.add_heading("五、当前状态与待办", level=1)
doc.add_heading("5.1 已验证可用", level=2)
bullets([
    "跨城 ZeroTier + Discovery Server 链路：odom 首帧 0.9s，2026-10-12 端到端导航通过；",
    "小车端 4 个常驻服务全部正常；",
    "相机彩色画面正常，跨城实测 10.0fps；",
    "建图、导航、地图编辑、日志、算法切换主流程可用。",
])

doc.add_heading("5.2 待修问题", level=2)
table(
    ["编号", "问题", "位置与修法"],
    [
        ["A", "日志时间戳早 8 小时", "run_humble_gui.sh 增加 -e TZ=Asia/Shanghai 与本地时间挂载"],
        ["B", "「测试 ROS 连接」永久卡在「正在 ping」", "容器无 ping，且 QProcess 失败不发 finished，需补 errorOccurred 处理"],
        ["D", "保存配置串写 network/robot_ip", "ip_combo 为局部变量，取不到实际值"],
        ["E", "断开后重连右侧地图视图空白", "重连时的判定条件与视图重置逻辑需修正"],
        ["—", "「设为初始位姿」首点必丢", "进入画点模式的首次点击被消耗，需区分模式切换与选点"],
        ["—", "停止 Nav2 后代价地图/粒子云不清空", "缺少对端消失事件，需加超时清除"],
        ["—", "日志导出 CSV 未验证落盘", "需实测导出文件"],
        ["—", "摇杆未经真实推动验证", "需在安全前提下实测"],
    ],
    widths=[1.5, 6.5, 7.5],
)

doc.add_heading("5.3 一句话总结", level=2)
p("上位机只做「显示 + 人机 + 决策」，所有需要长期运行或贴近硬件的逻辑都做成小车端常驻服务；"
  "两端仅通过自定义 topic 与 action 通信；跨城依赖 ZeroTier 打底、Discovery Server 解决节点发现。"
  "因此调试时最常见的判断是——问题出在 GUI 侧，还是小车端服务没有起来。")

doc.save(OUT)
print("✅ 已生成:", OUT)
print("大小: %.1f KB" % (os.path.getsize(OUT) / 1024))
