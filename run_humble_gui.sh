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
docker rm -f "$NAME" >/dev/null 2>&1 || true
docker run -d --name "$NAME" --net=host \
  --user "$(id -u):$(id -g)" -e HOME=/gui_home \
  -e DISPLAY="$DISPLAY" -e XAUTHORITY=/tmp/.docker-xauth \
  -e ROS_DOMAIN_ID=0 -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /tmp/.docker-xauth:/tmp/.docker-xauth:ro \
  -v "$HERE/maps:/maps:rw" \
  -v "$HERE/custom_algo:/algo:rw" \
  -v "$HERE/.gui_home:/gui_home:rw" \
  -v "$HERE/robots:/robots:ro" \
  -e RCJ_ROBOTS_DIR=/robots \
  "$IMG" \
  "source /opt/ros/humble/setup.bash && source /opt/robot_control_gui_humble/setup.bash && exec ros2 run robot_control_gui_humble robot_control_gui_humble_node" >/dev/null
echo "[gui] 容器 $NAME 已起"

# 3) scan relay: mentorpi 只发 /scan_raw,而 slam_toolbox 订 /scan
sleep 3
docker cp "$HERE/scan_relay.py" "$NAME":/tmp/scan_relay.py
docker exec -d "$NAME" bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; python3 /tmp/scan_relay.py > /tmp/relay.log 2>&1'
echo "[relay] /scan_raw -> /scan 已起"
echo "完成。窗口里点「连接小车」(IP 192.168.149.1)→ 建图页 → 开始建图"
