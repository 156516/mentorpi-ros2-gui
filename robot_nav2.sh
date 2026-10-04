#!/bin/bash
# robot_nav2.sh —— 在小车上启动 Nav2(导航),让 GUI 的「导航」页能用。
#
# 用法:
#   bash robot_nav2.sh start <地图名>    # 启动:加载地图 + AMCL 定位 + 导航
#   bash robot_nav2.sh stop              # 停掉 Nav2
#   bash robot_nav2.sh status            # 看节点/服务在不在
#
# <地图名> 是不带扩展名的路径(小车上的),例如 /home/ubuntu/ros2_ws/maps/my_map
#   需要该目录下有 <地图名>.yaml + <地图名>.pgm
#   (用 GUI 的「建图页 Save」或「地图编辑页 另存为」生成,再拷到小车上)
#
# 速度话题:Nav2 的 controller 发布 /cmd_vel;mentorpi 的 odom_publisher
# 同时订阅 /app/cmd_vel、/cmd_vel、/controller/cmd_vel 并驱动电机
# —— 所以 /cmd_vel 直接可用,不需要任何重映射(已实测确认)。
#
# ⚠️ 本脚本尚未在真车上验证过(小车充电中)。第一次跑请开两个终端:
#    一个跑 start,一个 `docker exec ... ros2 topic list` 看有没有 /navigate_to_pose。

set -e
PI=pi@192.168.149.1
PASS=raspberrypi
DOCKER="docker exec -u ubuntu MentorPi bash -lc"

usage() { echo "用法: $0 start <地图名> | stop | status"; exit 1; }

run_on_robot() {
  sshpass -p "$PASS" ssh -o ConnectTimeout=6 -o StrictHostKeyChecking=no "$PI" "$1"
}

case "${1:-}" in
  start)
    MAP="${2:-}"
    [ -z "$MAP" ] && usage
    echo ">>> 在小车上启动 Nav2(后台),地图: $MAP"
    # 注意:不用重映射 —— Nav2 发 /cmd_vel,mentorpi 的 odom_publisher 已订阅
    # /app/cmd_vel、/cmd_vel、/controller/cmd_vel 并驱动电机(实测确认)。
    run_on_robot "docker exec -d -u ubuntu MentorPi bash -lc 'source /opt/ros/humble/setup.bash; \
      ros2 launch nav2_bringup bringup_launch.py \
        use_sim_time:=false \
        map:=$MAP.yaml \
        params_file:=/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml \
        > /tmp/nav2.log 2>&1'"
    echo ">>> 已后台启动,等 12 秒让它起来..."
    sleep 12
    run_on_robot "$DOCKER \"source /opt/ros/humble/setup.bash; \
      echo '--- nav2 节点 ---'; ros2 node list 2>/dev/null | grep -iE 'amcl|controller_server|planner_server|bt_navigator' | head; \
      echo '--- 导航 action ---'; ros2 action list 2>/dev/null | grep navigate_to_pose; \
      echo '--- 日志尾 ---'; tail -5 /tmp/nav2.log\""
    ;;
  stop)
    echo ">>> 停掉 Nav2"
    run_on_robot "$DOCKER \"pkill -f nav2_bringup; pkill -f 'nav2_.*_node'; echo stopped\""
    ;;
  status)
    run_on_robot "$DOCKER \"source /opt/ros/humble/setup.bash; \
      echo '--- nav2 节点 ---'; ros2 node list 2>/dev/null | grep -i nav2 | head; \
      echo '--- 导航 action ---'; ros2 action list 2>/dev/null | grep navigate_to_pose; \
      echo '--- AMCL ---'; ros2 node list 2>/dev/null | grep -i amcl\""
    ;;
  *)
    usage
    ;;
esac
