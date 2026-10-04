#!/bin/bash
# start_robot_services.sh —— 小车重启后,在 PC 上一条命令起好小车的两个常驻服务。
#
#   slam_controller : 建图(收 GUI 的 /start_slam)
#   nav_controller  : 导航参数/算法切换(收 /apply_nav_params、/nav_map_upload)
#
# 用:  bash start_robot_services.sh
#
# 注意:必须用 `docker exec -d`(detached)。用普通 exec + nohup & 的话,
#       exec 一退出进程就被杀 —— 踩过。
set -e
PI=pi@192.168.149.1
PASS=raspberrypi
SSH="sshpass -p $PASS ssh -o ConnectTimeout=6 -o StrictHostKeyChecking=no $PI"

echo ">>> 检查小车"
$SSH "echo -n '小车在线: '; date -u" || { echo "❌ ssh 不通,先连小车热点"; exit 1; }

start_one() {   # $1 = 脚本名
  local name="$1"
  if $SSH "docker exec -u ubuntu MentorPi pgrep -f ${name}.py >/dev/null"; then
    echo "  ${name} 已在跑,跳过"
  else
    $SSH "docker exec -d -u ubuntu MentorPi bash -lc 'source /opt/ros/humble/setup.bash; python3 /home/ubuntu/ros2_ws/src/${name}.py > /tmp/${name}.log 2>&1'"
    echo "  ${name} 已启动"
  fi
}

echo ">>> 起小车端服务"
start_one slam_controller
start_one nav_controller

sleep 5
echo ">>> 验证"
$SSH "docker exec -u ubuntu MentorPi bash -lc '
  source /opt/ros/humble/setup.bash
  pgrep -f slam_controller.py >/dev/null && echo \"  slam_controller ✅\" || echo \"  slam_controller ❌\"
  pgrep -f nav_controller.py  >/dev/null && echo \"  nav_controller  ✅\" || echo \"  nav_controller  ❌\"
  echo -n \"  服务: \"; ros2 service list 2>/dev/null | grep -cE \"start_slam|apply_nav_params\" | xargs -I{} echo \"{}\/2 就绪\"'"
echo ">>> 完事。回 GUI 点「🔌 连接小车」"
