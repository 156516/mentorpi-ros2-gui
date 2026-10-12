#!/bin/bash
# start_robot_services.sh —— 小车启动后,在 PC 上一条命令起好小车的两个常驻服务。
#
#   slam_controller : 建图(收 GUI 的 /start_slam)
#   nav_controller  : 导航参数/算法切换(收 /apply_nav_params、/nav_map_upload)
#
# 用:  bash start_robot_services.sh
#
# 跨城/跨网改造(2026-10-10):
#   - 小车 IP 优先走 ZeroTier(10.132.156.106);本地调试依次回退有线 192.168.1.112、AP 192.168.149.1
#   - 全链路统一 FastDDS(rmw_fastrtps_cpp),不再用 CycloneDDS / /etc/cds.xml
#     (GUI 容器和 scan relay 也是 fastrtps,见 run_humble_gui.sh)
#
# 注意:必须用 `docker exec -d`(detached)。用普通 exec + nohup & 的话,
#       exec 一退出进程就被杀 —— 踩过。
set -e
PASS=raspberrypi
SSH_OPTS="-o ConnectTimeout=5 -o StrictHostKeyChecking=no"

# --- 1. 选通的小车 IP:ZeroTier 优先,本地调试依次回退有线 / AP ---
PI=""
for ip in \
  "${RPI_ZEROTIER_IP:-10.132.156.106}" \
  "192.168.1.112" \
  "192.168.149.1"; do
  [[ -z "$ip" ]] && continue
  if sshpass -p "$PASS" ssh $SSH_OPTS "pi@$ip" 'true' 2>/dev/null; then
    PI="pi@$ip"
    case "$ip" in
      10.132.*)      echo ">>> 走 ZeroTier 虚拟 IP: $PI" ;;
      192.168.1.*)   echo ">>> 走有线兜底: $PI" ;;
      *)             echo ">>> 走 AP 热点: $PI" ;;
    esac
    break
  fi
done
[[ -z "$PI" ]] && { echo "❌ ZeroTier(10.132.156.106) / 有线(192.168.1.112) / AP(192.168.149.1) 都连不上"; exit 1; }
SSH="sshpass -p $PASS ssh $SSH_OPTS $PI"

echo ">>> 检查小车"
$SSH "echo -n '小车在线: '; date -u" || { echo "❌ ssh 不通"; exit 1; }

# --- 2. 起服务(统一 FastDDS,与 GUI 容器一致)---
start_one() {   # $1 = 脚本名
  local name="$1"
  if $SSH "docker exec -u ubuntu MentorPi pgrep -f ${name}.py >/dev/null"; then
    echo "  ${name} 已在跑,跳过"
  else
    $SSH "docker exec -d -u ubuntu \
      -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
      -e ROS_DOMAIN_ID=0 \
      -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
      -e ROS_DISCOVERY_SERVER=${RPI_DISCOVERY_SERVER:-127.0.0.1:11811} \
      -e PYTHONUNBUFFERED=1 \
      MentorPi bash -lc 'source /opt/ros/humble/setup.bash; python3 /home/ubuntu/ros2_ws/src/${name}.py > /tmp/${name}.log 2>&1'"
    echo "  ${name} 已启动(FastDDS)"
  fi
}

echo ">>> 起小车端服务"
start_one slam_controller
start_one nav_controller
start_one cmd_vel_watchdog   # 速度看门狗:链路断/GUI 挂时自动刹车
start_one camera_color_relay # 彩色相机中继:usb_cam 的 YUYV 原图 → 彩色 JPEG(GUI 相机面板用)

sleep 5
echo ">>> 验证(进程是否存活)"
$SSH "docker exec -u ubuntu MentorPi bash -lc '
  pgrep -f slam_controller.py >/dev/null && echo \"  slam_controller ✅\" || echo \"  slam_controller ❌\"
  pgrep -f nav_controller.py  >/dev/null && echo \"  nav_controller  ✅\" || echo \"  nav_controller  ❌\"
  pgrep -f cmd_vel_watchdog.py >/dev/null && echo \"  cmd_vel_watchdog ✅\" || echo \"  cmd_vel_watchdog ❌\"
  pgrep -f camera_color_relay.py >/dev/null && echo \"  camera_color_relay ✅\" || echo \"  camera_color_relay ❌\"'"
echo ">>> 完事。回 GUI 点「🔌 连接小车」→ 跨城填 10.132.156.106"
