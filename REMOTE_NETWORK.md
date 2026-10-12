# 跨城使用 GUI —— ZeroTier + FastDDS Discovery Server

PC 和小车不在同一局域网时(家里 ↔ 实验室,跨城等),走 ZeroTier 虚拟网络打洞,
两端 RMW 统一用 **FastDDS**(`rmw_fastrtps_cpp`),发现走 **Discovery Server**(不走组播)。

> 2026-10-12 改版。本文先前记录的是 **CycloneDDS + 单播 peer** 方案,已完全作废 ——
> 现在两端都是 FastDDS。那个时代的遗留文件(`cds.xml` / `cds_robot.xml` /
> `setup_zerotier_robot.sh`)已随 `_retired/` 目录一并删除,不再被任何脚本引用。

## 拓扑

```
  PC (Ubuntu)                                RPi (树莓派)
  ┌────────────────┐                        ┌──────────────────────────────┐
  │ Docker GUI     │                        │ MentorPi 容器(host 网络)      │
  │ (Humble)       │    FastDDS             │  bringup / nav / slam / 看门狗 │
  │                │ ◄──────────────────►   │                              │
  │ ROS_DISCOVERY_ │  10.132.156.182        │  ROS_DISCOVERY_SERVER        │
  │ SERVER=        │  ↕ ZeroTier 虚拟网      │   =127.0.0.1:11811           │
  │ 10.132.156.106 │                        │        │                     │
  └────────────────┘                        │        ▼                     │
                                            │  ┌────────────────────────┐  │
                                            │  │ fastdds_discovery 容器 │  │
                                            │  │ (SERVER, :11811)       │  │
                                            │  └────────────────────────┘  │
                                            └──────────────────────────────┘
```

**为什么服务器放小车、小车节点用 `127.0.0.1` 连它**:
PC 关机、ZT 断线都不影响小车**内部**的 ROS 图(odom ↔ ros_robot_controller ↔ 雷达/底盘),
底盘链路不会因为网络问题一起失效。

## 当前状态

| 端 | ZeroTier 状态 | Node ID | 虚拟 IP |
|---|---|---|---|
| PC | `active (running)` | `e768796129` | `10.132.156.182` |
| RPi | `active (running)` | `6501e14775` | `10.132.156.106` |

网络 ID `166359304e8a1001`。小车侧还有有线兜底 `eth0 = 192.168.1.112`(静态,never-default)。

## 怎么配的(两处)

### 小车端

1. **发现服务器容器**(SD 卡持久,`--restart unless-stopped` 随 docker 开机自启):
   ```bash
   docker run -d --name fastdds_discovery --restart unless-stopped --net host \
     ros:humble bash -lc 'source /opt/ros/humble/setup.bash; exec fastdds discovery -i 0 -p 11811'
   ```

2. **环境变量**写进 SD 卡上的 `.typerc`(权威副本 = 宿主机 `/home/pi/docker/tmp/.typerc`,
   容器内 `/home/ubuntu/shared/.typerc`,由 sync 服务自动推进容器):
   ```bash
   export ROS_DISCOVERY_SERVER=127.0.0.1:11811
   ```
   小车端 ROS 环境**不来自 docker env**(容器 `Config.Env` 里没有任何 ROS 变量),而来自
   shell rc 链 `~/.zshrc → ros2_ws/.zshrc → .robotrc → .typerc` —— 所以必须改 `.typerc`。

   ⚠️ **只对新拉起的进程生效**。要让 bringup 那批厂家节点吃到,必须:
   ```bash
   ssh pi@10.132.156.106 'sudo systemctl restart start_node.service'
   ```
   (`start_node.sh` 自己会先 `~/.stop_ros.sh` 再重起 bringup,是它的正常重启路径。)

3. 我们自己的三个脚本(`slam_controller.py` / `nav_controller.py` / `cmd_vel_watchdog.py`)
   **不读 `.typerc`**,靠 `start_robot_services.sh` 里的 `docker exec -e` 传;
   `robot_nav2.sh` 同理。

### PC 端

`run_humble_gui.sh` 里两处(**必须同时改**,否则 GUI 与 relay 的发现配置不一致):
- GUI 容器的 `docker run -e ROS_DISCOVERY_SERVER="$ROS_DISCOVERY_SERVER"`
- scan relay 的 `docker exec`(其 `export` 行)

默认值 `10.132.156.106:11811`,可用 `ROS_DISCOVERY_SERVER=...` 覆盖。

### 一键脚本

```bash
bash setup_robot_dds.sh start    # 起服务器容器 + 幂等写入 .typerc(先备份 .bak-ds)
bash setup_robot_dds.sh status   # 看服务器 / 端口 / .typerc 状态
bash setup_robot_dds.sh stop     # 回滚:还原 .typerc + 删容器
```
`start` / `stop` 之后都要 `sudo systemctl restart start_node.service` 才生效。

## 实测效果(2026-10-12)

改之前在 ZT 上靠默认组播,发现极慢;改后跨城速率与小车**机内**速率一致:

| 话题 | 改前(组播) | 改后(Discovery Server) |
|---|---|---|
| `/odom` | 首帧 **13.1 s** · ≈4 Hz | 首帧 **0.9 s** · **28 Hz** |
| `/tf` | 首帧 3.4 s · 4.4 Hz | 首帧 **1.0 s** · **32 Hz** |
| `/scan_raw` | 首帧 **13.7 s** · 1.2 Hz | 首帧 **1.4 s** · **9.3 Hz** |
| PC 侧可见小车节点 | **0 个** | **26 个**(全部) |

## 验证跨城通信

⚠️ **别用 `ros2 node list` / `ros2 topic list` 判断死活** —— 在链路上它们会返回空或直接挂死,
看起来像 DDS 死了,其实数据是通的。**一律用 rclpy 探针**(`tools/dds_probe/`)。

```bash
cd ~/ros2_gui/robot_control_gui_ros2/tools/dds_probe
docker run --rm --net=host --entrypoint /bin/bash \
  -v $PWD/probe_one.py:/tmp/probe_one.py:ro \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e ROS_DISCOVERY_SERVER=10.132.156.106:11811 \
  robot_control_gui_humble:latest \
  -c 'source /opt/ros/humble/setup.bash && python3 /tmp/probe_one.py /odom 20'
```
判据:默认传输就有数据、首帧 1~2 秒即到。`TL=0` 是正常的(`/odom` 是 VOLATILE,本就不该匹配 TRANSIENT_LOCAL)。

## 常见坑

- **时钟差**:RPi 没 RTC,启动 GUI 前要 `bash sync_robot_clock.sh`(`run_humble_gui.sh` 已自动跑)。
  时间差 >1 s 会让 TF 查询全部失败,表现成「未定位 / 导航发不出去」。
- **链路拥塞 ≠ DDS 半死**:相机流占满手机热点上行时,发现会退化(`ros2 node list` 看不到东西、
  首帧拖到十几秒),但**数据面没死**—— 探针只要有首帧、速率>0 就是「慢」不是「死」。
- **相机流一开,ssh 基本挤不进去**(`Connection timed out during banner exchange`)。
  要 ssh 上小车做维护,先让 GUI 断开。`start_robot_services.sh` 的 `ConnectTimeout=5`
  在这种状态下必挂,得放宽到 20~25 s。
- **窗口标题「已连接」不代表图是活的** —— 那只是 GUI 自己的连接状态。
- **ZeroTier relay**:两端都在 NAT 后时会落到 Planet 中继(走境外),比直连慢,但一般够用;
   改完网络后 ZT 要 1~3 分钟才重新协商出路径,别急着判定失败。
- **节点 ID 记错**:网页里 Member 的 `Node ID` 和 `Address` 是两回事,Auth 勾的是 Node ID。
