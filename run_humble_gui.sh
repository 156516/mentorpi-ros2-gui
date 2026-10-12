#!/bin/bash
# 启动 Humble 容器版 GUI(含 X11 + /scan_raw->/scan relay)
# 用法: bash run_humble_gui.sh
set -e
IMG=robot_control_gui_humble:latest
NAME=rcj_gui_humble
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$HERE/maps"        # 地图导入/导出(挂进容器 /maps)
mkdir -p "$HERE/.gui_home"   # 容器内 HOME(存 QSettings)
mkdir -p "$HERE/robots"      # 机器人配置档(挂进容器 /robots)
mkdir -p "$HERE/custom_algo" # 自定义算法模板(挂进容器 /algo)
export DISPLAY=${DISPLAY:-:0}

# 跨城发现:所有参与者都指向小车上的 FastDDS Discovery Server(不走组播)。
# 默认取小车 ZeroTier IP;小车侧自己的节点指向 127.0.0.1:11811(见小车 .typerc)。
export ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER:-10.132.156.106:11811}
echo "[dds] ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER"

# 1) Xauthority(Wayland 下 ~/.Xauthority 是空目录,需取真实 cookie)
XAUTH_SRC="$(xauth info 2>/dev/null | awk -F': ' '/Authority file/ {print $2; exit}' | sed 's/^[[:space:]]*//')"
if [[ -n "$XAUTH_SRC" && -s "$XAUTH_SRC" ]]; then
  cp -f "$XAUTH_SRC" /tmp/.docker-xauth; chmod 644 /tmp/.docker-xauth
  echo "[x11] Xauthority: $XAUTH_SRC"
else
  echo "[x11] 警告: 找不到 Xauthority,窗口可能弹不出"
fi
xhost +local:docker >/dev/null 2>&1 || true

# 1.5) 校准小车时钟(时间不对会让 TF 全查不到 —— 实测差过 6.5 天)
bash "$HERE/sync_robot_clock.sh" 2>/dev/null || echo "[clock] 跳过(小车没连?)"

# 2) 起 GUI 容器(以你的用户身份跑 -> 它创建的文件归你,不再是 root)
#    注意 --entrypoint /bin/bash:镜像 :latest 里烘焙了一个坏 entrypoint
#    (ENTRYPOINT=[/bin/bash] CMD=[-c sleep 1800]),会把整条命令当文件名 -> exit 127
docker rm -f "$NAME" >/dev/null 2>&1 || true
docker run -d --name "$NAME" --net=host \
  --user "$(id -u):$(id -g)" -e HOME=/gui_home \
  -e DISPLAY="$DISPLAY" -e XAUTHORITY=/tmp/.docker-xauth \
  -e ROS_DOMAIN_ID=0 -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET -e QT_X11_NO_MITSHM=1 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e ROS_DISCOVERY_SERVER="$ROS_DISCOVERY_SERVER" \
  -e TZ=Asia/Shanghai \
  -v /etc/localtime:/etc/localtime:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /tmp/.docker-xauth:/tmp/.docker-xauth:ro \
  -v "$HERE/maps:/maps:rw" \
  -v "$HERE/custom_algo:/algo:rw" \
  -v "$HERE/.gui_home:/gui_home:rw" \
  -v "$HERE/robots:/robots:ro" \
  -e RCJ_ROBOTS_DIR=/robots \
  -e RCJ_AUTOCONNECT_IP="${RCJ_AUTOCONNECT_IP:-}" \
  -e RCJ_SHOW_HELP="${RCJ_SHOW_HELP:-}" \
  -e RCJ_DUMP_LAYOUT="${RCJ_DUMP_LAYOUT:-}" \
  -e RCJ_SELFTEST_RECONNECT="${RCJ_SELFTEST_RECONNECT:-}" \
  --entrypoint /bin/bash \
  "$IMG" \
  -c "source /opt/ros/humble/setup.bash && source /opt/robot_control_gui_humble/setup.bash && exec ros2 run robot_control_gui_humble robot_control_gui_humble_node" >/dev/null
echo "[gui] 容器 $NAME 已起"

# 3) scan relay: mentorpi 只发 /scan_raw,而 slam_toolbox 订 /scan
sleep 3
docker cp "$HERE/scan_relay.py" "$NAME":/tmp/scan_relay.py
docker exec -d "$NAME" bash -lc "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER; python3 /tmp/scan_relay.py > /tmp/relay.log 2>&1"
echo "[relay] /scan_raw -> /scan 已起"
echo "完成。窗口里点「连接小车」→ 跨城填 ZeroTier IP 10.132.156.106(本地热点 192.168.149.1)→ 建图页 → 开始建图"
