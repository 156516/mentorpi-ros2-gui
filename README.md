# MentorPi ROS2 上位机（robot_control_gui）

一个 **C++ / Qt5 + ROS2** 的机器人上位机（控制台），用于 **MentorPi（树莓派麦轮小车）** 的
遥控、建图、地图编辑与 Nav2 自主导航。

界面全部由 Qt 自绘（不内嵌 RViz），PC 端通过 **DDS 直连** 小车的 ROS2 网络：
速度、激光、里程计、地图、代价地图、粒子云、导航 action、SLAM/导航服务，全部走 ROS2
话题 / 服务 / action，**不需要 SSH 隧道，也不需要 rosbridge**（同网段直连）。

> 本项目是 ROS1 Noetic + Qt5 版本（`robot_control_gui-master`）的 ROS2 复刻，
> 面向 ROS2 Humble + Nav2 + slam_toolbox 技术栈。

---

## 目录

- [1. 功能概览](#1-功能概览)
- [2. 系统架构](#2-系统架构)
- [3. 技术方案](#3-技术方案)
- [4. 在一台新 PC 上部署](#4-在一台新-pc-上部署)
- [5. 接入小车](#5-接入小车)
- [6. 使用流程（建图 / 导航）](#6-使用流程建图--导航)
- [7. 换一台机器人](#7-换一台机器人)
- [8. 自定义 Nav2 算法](#8-自定义-nav2-算法)
- [9. 单元测试](#9-单元测试)
- [10. 已知限制与踩坑记录](#10-已知限制与踩坑记录)

---

## 1. 功能概览

GUI 分为 **6 个页面**，工具栏提供连接/断开、重置视图、视角角度、内置图文帮助：

| 页面 | 能力 |
|---|---|
| **控制** | 双虚拟摇杆 + 键盘 `W/A/S/D` + 空格急停；速度仪表盘；底盘速度实时下发 |
| **导航** | 地图上拖拽设目标点+朝向 → 开始导航；取消导航；**代价地图**开关（全局/局部，RViz 配色）；**AMCL 粒子云**开关；导航参数面板（规划器/控制器下拉、车半径、膨胀半径、读取/应用并重启）；算法参数查询/修改；📍用当前位置设为初始位姿 |
| **建图** | slam_toolbox 起停；SLAM 参数面板（5 个常用参数，带 `*` 的需"应用并重启 SLAM"）；Save map 存 PGM+YAML 到 PC |
| **地图编辑** | 编辑当前图 / 导入 PGM+YAML；补墙·擦除笔刷；撤销重做；另存为 |
| **状态** | odom / 电池 / diagnostics / 诊断；机器人位姿与速度 |
| **设置** | 机器人 IP、ROS_DOMAIN_ID、速度上限（`QSettings` 持久化）；ping / ssh 连通性测试；**机器人配置档**下拉 |

**日志页**：订阅小车 `/rosout` + GUI 事件，按 **分类**（定位/导航/建图/参数/位姿/传感器/GUI）
与 **级别** 筛选，按时间段导出 CSV。

**视图交互**（`RobotView`，QGraphicsView 自绘）：滚轮缩放 · 中键平移 · **右键拖动旋转** ·
**左键点 = 目标点** · **Shift+左键 = 初始位姿**。

---

## 2. 系统架构

```
┌──────────────────────── PC (Ubuntu) ─────────────────────────┐
│                                                              │
│   Docker 容器  rcj_gui_humble                                 │
│   osrf/ros:humble-desktop  +  编译好的 robot_control_gui      │
│   ┌──────────────────────────────────────────────────────┐   │
│   │  Qt5 UI 线程            rclcpp MultiThreadedExecutor  │   │
│   │  main_window / 6 页面    ├─ RobotController (话题)     │   │
│   │  RobotView (自绘)  ◄───  ├─ DiagnosticsWatcher        │   │
│   │  MjpegStream             ├─ SlamParamClient (服务)     │   │
│   │                          └─ NavigateToPose (action)   │   │
│   └──────────────────────────────────────────────────────┘   │
│              ▲ X11 窗口              ▲ DDS (--net=host)       │
│   run_humble_gui.sh：xauth/X11 cookie │ 时钟同步 │ scan relay │
└──────────────────────────────────────┼───────────────────────┘
                                       │  Wi-Fi 热点 192.168.149.1
                                       │  ROS_DOMAIN_ID=0
┌──────────────────────────────────────┼───────────────────────┐
│ MentorPi 小车 (Raspberry Pi, Ubuntu 22.04)                    │
│   Docker 容器  MentorPi  (ros-humble)                         │
│   ├─ bringup：底盘 / 雷达 / 相机 / IMU / odom_publisher        │
│   ├─ slam_controller.py   ← /start_slam /stop_slam /is_mapping│
│   ├─ nav_controller.py    ← /apply_nav_params /nav_map_upload │
│   └─ Nav2 (bringup_launch.py) ← nav_controller 拉起            │
└──────────────────────────────────────────────────────────────┘
```

**两条控制路径**：

- **数据面（DDS 直连）**：GUI 订阅/发布话题、调用服务、发送导航 action，全部走 ROS2 DDS。
  这是主路径，SSH 只用于部署与运维脚本。
- **运维面（SSH）**：`start_robot_services.sh` / `robot_nav2.sh` / `deploy_algo.sh`
  通过 `sshpass + docker exec` 在小车容器里起常驻节点、部署自定义插件。

**目录结构**：

```
robot_control_gui_ros2/
├── README.md                    本文件
├── Dockerfile.humble            把 GUI 整个编进 Humble 容器的镜像定义
├── run_humble_gui.sh            ★ 一键：时钟同步 → 起容器 → 起 scan relay
├── start_robot_services.sh      ★ 一键：在小车端起 slam_controller + nav_controller
├── sync_robot_clock.sh          校准小车时钟（TF 依赖，非常重要）
├── robot_nav2.sh                在小车端直接起/停/查 Nav2（备用）
├── deploy_algo.sh               编译部署自定义 Nav2 插件到小车（ARM64）
├── setup_sd_card.sh             （可选）改 SD 卡使 slam_controller 开机自启
│
├── ros2_ws_humble/src/robot_control_gui_humble/   ← GUI 源码（Humble 版，主线）
│   ├── include|src/ui/           Qt 界面：主窗口 + 6 页面 + 自绘视图
│   ├── include|src/ros/          ROS 封装：RobotController / Profile /
│   │                             DiagnosticsWatcher / SlamParamClient /
│   │                             MapEditor（纯函数）/ topic_names
│   ├── launch/                   ros2 launch 入口
│   └── test/                     2 组 gtest（纯 C++，无需 ROS 运行时）
│
├── src/robot_control_gui_jazzy/  ← 同源 Jazzy 变体（遗留，不推荐直连 Humble 车）
├── slam_controller_ws/src/       小车端：SLAM 起停服务（rclpy）
├── nav_controller_ws/src/        小车端：Nav2 参数/算法管理（rclpy）
├── robots/mentorpi.yaml          机器人配置档（换车不改代码）
├── robots/_template.yaml         新机器人模板
├── custom_algo/my_aster|my_pid/  自定义 Nav2 插件示例（规划器 / 控制器）
└── maps/                         保存的地图（PC 侧，挂进容器 /maps）
```

---

## 3. 技术方案

### 3.1 为什么用 Humble 容器跑 GUI？

**PC 是 Ubuntu 24.04（只有 ROS2 Jazzy），小车是 Ubuntu 22.04（ROS2 Humble）。**

最初直接在 PC 上用 Jazzy 编译 GUI 并直连 Humble 小车，会踩 **跨 LTS 的 DDS 序列化不兼容**：
Jazzy 侧发出的数据小车解析不了，**每次重连会把底盘状态机带崩**（一度误以为是"小车缺陷、
必须每次冷启"）。

解决方案：把整个 GUI 编进 `osrf/ros:humble-desktop` 容器里运行，**两侧 distro 完全一致**。
实测（2026-10-04）**断线重连完全正常**，不再需要冷启小车。

- `Dockerfile.humble` 只 copy `ros2_ws_humble/` 源码，`colcon build` 后把 install 目录
  落到镜像里的 `/opt/robot_control_gui_humble`。
- 容器以 **宿主用户身份**（`--user $(id -u):$(id -g)`）运行，产生的文件归你所有，
  不会留 root 文件。
- `--net=host` 打通 DDS；X11 通过 `/tmp/.X11-unix` + 真实 xauth cookie 挂进容器
  （Wayland 下 `~/.Xauthority` 是空目录，脚本会用 `xauth info` 找真实 cookie 再拷进容器）。

> ⚠️ **不要用"FROM 上个镜像"的增量方式反复构建**：每层 `cp` 整个 install 目录会叠加，
> 镜像会从 5.25GB 涨到 10GB+。改源码就重跑完整 `Dockerfile.humble`（apt 层有缓存，不慢）。

### 3.2 GUI 侧

- **Qt5 Widgets** 做全部界面；**QGraphicsView 自绘**地图 / 机器人 / 激光点 / 路径 /
  代价地图 / 粒子云，替代内嵌 RViz（跨版本 RViz 依赖太重且难定制）。
- **rclcpp `MultiThreadedExecutor`** 跑 ROS2；ROS 回调在独立线程，通过 **Qt 信号跨线程**
  投递到 UI 线程。
  - ⚠️ 跨线程信号必须 `qRegisterMetaType<T>()`（见 `main.cpp`），否则槽函数**静默不执行**。
  - ⚠️ `create_subscription` 的返回值必须存成成员 `SharedPtr`，否则订阅**立刻失效**。
- **默认不连 ROS**：GUI 启动时保持静默，点"🔌 连接小车"才 `init` rclcpp 并建订阅
  （节点名带 PID 后缀，避免重复启动冲突）。
- **坐标与朝向**：Qt 场景 y 轴向下，而世界 y 轴向上 → `RobotView::fitWithYUp()` 用
  `setTransform(单位阵) → fitInView → scale(1,-1)` 修正；且**只在第一帧地图 fit**，
  否则实时 1Hz 的 `/map` 会一直重置用户视角。
- **代价地图配色**用 Nav2 标准约定：`0`=空闲，`1..98`=膨胀代价，`99`=内切(INSCRIBED)，
  `100`=致命障碍(LETHAL)，`-1`=未知。

### 3.3 机器人配置档（换车不改代码）

`robots/*.yaml` 描述话题名、电池类型、坐标系、能力开关；`topic_names.h` 全部从
`RobotProfile::cur()` 读取。设置页有"机器人配置"下拉，**加新车 = 复制模板改值 + 重启 GUI**。

```yaml
name: MentorPi (默认)
robot_ip: 192.168.149.1
cmd_vel:  /controller/cmd_vel
scan:     /scan_raw
odom:     /odom
map:      /map
plan:     /plan
battery:  /ros_robot_controller/battery
battery_type: uint16            # uint16 | battery_state
map_frame:  map
odom_frame: odom
base_frame: base_footprint
lidar_frame: lidar_frame
has_nav2: true
has_slam_controller: true
has_camera: true
camera_topic: /ascamera/camera_publisher/rgb0/image_compressed
```

### 3.4 小车侧常驻服务

两个 `rclpy` 节点跑在小车容器里，让 GUI 无需 SSH 即可控制 SLAM 与 Nav2 生命周期：

- **`slam_controller.py`** — 服务：`/start_slam`、`/stop_slam`、`/is_mapping`；
  发布 `/slam_status`。按方法拉起对应 launch（`slam_toolbox` / `rtabmap` / `hector` /
  `cartographer`）。
- **`nav_controller.py`** — 服务：`/apply_nav_params`、`/stop_nav`、`/is_nav`、
  `/list_nav_plugins`；订阅 `/nav_params`（JSON）、`/nav_map_upload`（`OccupancyGrid`,
  latched）；发布 `/nav_status`。它把 GUI 选择写进 `nav2_params_gui.yaml` 并重启 Nav2，
  扫描 `custom_ws` 下的自定义插件并入下拉列表。

两个脚本都会 **自 re-exec**：先 `source` 小车 overlay 再 `exec python3 自己`，
因此从 `bash -c` / `docker exec` / `nohup` 任何上下文启动都能拿到正确的
`AMENT_PREFIX_PATH` / `LD_LIBRARY_PATH`。

### 3.5 关键话题 / 服务 / action（mentorpi）

| 用途 | 名称 | 说明 |
|---|---|---|
| 底盘速度 | `/cmd_vel`（GUI 发 `/controller/cmd_vel`） | Nav2 发 `/cmd_vel` 直接可用；mentorpi 的 `odom_publisher` 同时订阅 `/app/cmd_vel`、`/cmd_vel`、`/controller/cmd_vel` |
| 雷达 | `/scan_raw` | mentorpi 不做 `laser_filters`，没有 `/scan` → 容器里 `scan_relay.py` 转发 |
| 地图 | `/map` | QoS 必须 `transient_local`，否则被静默丢弃 |
| 电池 | `/ros_robot_controller/battery` | `std_msgs/UInt16`（毫伏×100），**不是** `BatteryState` |
| 导航 | `/navigate_to_pose` | Nav2 启动后才有 |
| 存图 | `/slam_toolbox/save_map` | 注意不是 `/map_saver/save_map` |

---

## 4. 在一台新 PC 上部署

### 4.1 前置条件

- Ubuntu（22.04 / 24.04 均可，**PC 的 ROS 版本不重要**——GUI 跑在容器里）。
- 已安装 **Docker**（能用 `docker run --net=host`）。
- 宿主具备 X11 / Wayland 图形环境。
- 若要用运维脚本：`sshpass`、`xauth`（`sudo apt install sshpass x11-xserver-utils`）。

```bash
# 验证 docker 可用
docker run --rm hello-world
```

### 4.2 克隆并构建镜像

```bash
git clone https://github.com/<你的用户名>/mentorpi-ros2-gui.git
cd mentorpi-ros2-gui

# 构建 Humble 容器镜像（apt 层会缓存，首次较慢）
docker build -f Dockerfile.humble -t robot_control_gui_humble:latest .
```

### 4.3 一键启动

```bash
# ① 起 GUI（自动：校准小车时钟 → 起容器 → 起 /scan_raw→/scan relay）
bash run_humble_gui.sh

# ② 另开一个终端，在小车端起 slam/nav 两个常驻服务
bash start_robot_services.sh
```

GUI 窗口出现后，点 **「🔌 连接小车」** → 输入小车 IP（默认 `192.168.149.1`）→ 确定。

> **只看界面 / 无小车调试**：直接 `bash run_humble_gui.sh` 后**不要点连接**，
> GUI 保持完全静默，可安全查看 UI 布局。也可设环境变量
> `RCJ_AUTOCONNECT_IP=<ip>` 自动连接（跳过弹框），`RCJ_SHOW_HELP=1` 启动即弹帮助。

### 4.4 开发（改代码后重新构建）

```bash
# 改 ros2_ws_humble/ 下源码后，重跑完整构建（不要用增量 FROM 旧镜像）
docker build -f Dockerfile.humble -t robot_control_gui_humble:latest .
bash run_humble_gui.sh          # 重启容器即生效
```

若只想快速验证编译：进入一个 Humble 容器内 `colcon build`，无需每次全量重建镜像。

### 4.5 环境变量

| 变量 | 作用 |
|---|---|
| `DISPLAY` | 传给容器（默认 `:0`） |
| `ROS_DOMAIN_ID` | 必须与小车一致（mentorpi 默认 `0`） |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | `SUBNET`（默认） |
| `RCJ_AUTOCONNECT_IP` | 设了则自动连接该 IP，跳过弹框（便于自动化测试） |
| `RCJ_SHOW_HELP` | 启动即弹出内置使用说明 |
| `RCJ_ROBOTS_DIR` | 配置档目录（`run_humble_gui.sh` 挂载为 `/robots`） |

---

## 5. 接入小车

以 **MentorPi** 为例（其它车型见 [§7](#7-换一台机器人)）。

### 5.1 网络

1. 小车通电，**等约 30 秒听「滴」一声**（主板自检正常，否则 `/imu`、`/battery` 无数据）。
2. PC 连小车 Wi-Fi 热点（SSID 一般 `MentorPi_xxx` / `HW-xxxx`，密码见机身）。
3. 验证：`ping 192.168.149.1` 通即可。

### 5.2 同步小车时钟（**必做，否则一堆"玄学问题"**）

树莓派没有 RTC 也不一定连 NTP，重启后时钟可能差数天（实测差过 **6.5 天**）。
ROS2 的 TF 查询按时间戳做，时间不对会让 GUI 显示"未定位"、AMCL 报
`extrapolation into the future`、导航发不出去。

```bash
bash sync_robot_clock.sh        # run_humble_gui.sh 已自动调用
```

### 5.3 起小车端服务

```bash
bash start_robot_services.sh
```

它通过 `sshpass + docker exec -d -u ubuntu MentorPi` 起 `slam_controller` 和
`nav_controller`（**必须 detached**；普通 `exec` 一退出进程就被杀）。这两个服务
**不是开机自启的，每次小车重启后都要重起**。

> 想让 `slam_controller` 开机自启：`bash setup_sd_card.sh /media/$USER/rootfs`
> （挂载 SD 卡后跑，会改 `start.sh`）。

### 5.4 依赖与默认凭据

小车侧默认：`ssh pi@192.168.149.1`，密码 `raspberrypi`，容器名 `MentorPi`，
ROS 工作区 `/home/ubuntu/ros2_ws/install`，自定义插件工作区
`/home/ubuntu/custom_ws`。**换车时这些值写在 `start_robot_services.sh` /
`robot_nav2.sh` / `deploy_algo.sh` 顶部，按需修改。**

---

## 6. 使用流程（建图 / 导航）

### 6.1 建图

1. **建图页** → 方法保持 `slam_toolbox` → **开始建图**。
2. 切**控制页**，开车绕一圈，地图实时长出。
3. 回建图页 → **停止** → **Save map**：路径填 `/maps/<名字>` → 存到 PC 的
   `robot_control_gui_ros2/maps/`。
4. 想修图：**地图编辑页** → 导入 → 补墙/擦除 → 另存为。

### 6.2 导航

> Nav2 与 slam_toolbox **都发 `map→odom`，不能同时开**。顺序：建图存图 → 停 SLAM → 起 Nav2。

1. **导航页** → 上传当前地图给小车（或直接用小车上的图）→ **应用并重启导航**，等 10~20 秒。
2. **设初始位姿**（关键）：点 **📍 用当前位置设为初始位姿**（或地图上 Shift+左键）。
   - 看 **粒子云** 判断准不准：聚成小团 = 准，散开 = 不准，跑到别处 = 收错。
3. 地图上 **左键按住拖动**（拖的方向 = 目标朝向）→ 出现绿点+绿箭头。
4. 点 **▶ 开始导航**，车出发；中断点 **✗ 取消导航**。

### 6.3 换算法 / 调参数

- **换规划器/控制器**：导航页下拉（首次先点"读取当前值"让自定义算法出现）→ 应用并重启导航。
- **调参数**：点 🔧 算法参数 → 选规划器/控制器 → 双击值改 → 应用修改。
- **加自己的算法**：见 [§8](#8-自定义-nav2-算法)。

### 6.4 收工

1. GUI 里点 **⏏ 断开**（重要：别直接关窗口）。
2. 再关窗口。
3. 停容器：`docker rm -f rcj_gui_humble`。

---

## 7. 换一台机器人

1. `cp robots/_template.yaml robots/myrobot.yaml`，按车改话题名 / 坐标系 / 能力开关。
2. 重启 GUI（或用 `RCJ_ROBOTS_DIR` 指向你的配置档目录）。
3. 设置页 → **机器人配置** 下拉选 `myrobot.yaml`。

**不用改任何代码。** 若新车没有小车端 `slam_controller`，把 `has_slam_controller`
设为 `false`（建图页会相应变化）。

---

## 8. 自定义 Nav2 算法

内置示例：`custom_algo/my_aster`（全局规划器）、`custom_algo/my_pid`（控制器）。

1. GUI 导航页 → ➕ 添加自定义算法 → 生成模板。
2. 改 `custom_algo/<名字>/src/*.cpp` 与 `plugins.xml`（**插件类名以 `plugins.xml` 为准**）。
3. 部署：

```bash
bash deploy_algo.sh <名字>
# 1/4 传到小车 → 2/4 放进容器 → 3/4 在小车(ARM64)上 colcon build → 4/4 完成
```

4. 回 GUI 导航页 → 点「读取当前值」→ 下拉里选它 →「应用并重启导航」。

> ⚠️ **插件必须在 ARM64 小车上编译**（PC 是 x86_64，编出的 `.so` 小车加载不了）。
> ⚠️ Nav2 启动前必须 `source /home/ubuntu/custom_ws/install/setup.bash`，
> 否则报 `class ... does not exist`（`nav_controller.py` 已处理）。

---

## 9. 单元测试

两组**纯 C++ gtest**（不依赖 ROS 运行时），覆盖 `occupancyGrid` / `laserScan` /
`path` 转换与地图编辑 / PGM-YAML 序列化：

```bash
# 在 Humble 容器内（或 Humble 环境）：
cd ros2_ws_humble && source /opt/ros/humble/setup.bash
colcon build --packages-select robot_control_gui_humble \
    --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select robot_control_gui_humble
colcon test-result --verbose
```

---

## 10. 已知限制与踩坑记录

| 现象 | 原因 / 处理 |
|---|---|
| 点开始导航**车不动** | ① Nav2 没起 → 应用并重启导航；② **没设初始位姿** → 点 📍 |
| 「定位」❌ 未定位 | 点 📍 或图上 Shift+左键设位置 |
| 定位 ✅ 但位置不对 | AMCL 从 (0,0,0) 收敛错 → **重设初始位姿** |
| 导航发不出 / TF 全查不到 | **小车时钟不对** → `bash sync_robot_clock.sh` |
| 代价地图起不来 | 没定位（无 `map→odom`）→ 先设初始位姿 |
| 建图收不到数据、`/map` 空 | mentorpi 只发 `/scan_raw` → 靠容器里的 `scan_relay.py` 转 `/scan` |
| 自定义算法选不到 | 没跑 `deploy_algo.sh`，或 `nav_controller` 没重启 |
| 换车 | 改 `robots/*.yaml`，设置页选它，重启 GUI |
| 镜像越来越大 | 别用增量 `FROM` 旧镜像构建，重跑完整 Dockerfile |

**其它限制**：

- **横向平移（A/D）**：GUI 发 `linear.y`，但 mentorpi 麦轮 holonomic 是否响应取决于底盘
  驱动配置（当前实测只能前后 + 旋转）。
- **MJPEG 摄像头**：仅占位，未实现实际拉流。
- **地图持久化**：保存的路径交给小车端服务，GUI 不自动下载到本地（PC 侧 `maps/` 是
  GUI 另存/导入用的）。
- **机器人显示位姿**：优先用 TF 的 `map` 帧（`tryGetRobotPoseInMap`），TF 不可用时回退
  `odom`——只用 odom 会与地图差一个 `map→odom` 旋转（实测约 16°）。
- **跨网段**：当前仅支持 DDS 直连（同网段）。跨网段需改用 rosbridge/WebSocket。

---

## 许可

MIT
