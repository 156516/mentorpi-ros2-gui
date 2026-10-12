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
PASS=raspberrypi
SSH_OPTS="-o ConnectTimeout=20 -o StrictHostKeyChecking=no"

usage() { echo "用法: $0 start <地图名> | stop | status"; exit 1; }

# 小车 IP:ZeroTier 优先,依次回退有线 / AP(与 start_robot_services.sh 一致)。
# 原来这里硬编码 192.168.149.1(AP 热点),跨城根本用不了。
PI=""
for ip in "${RPI_ZEROTIER_IP:-10.132.156.106}" "192.168.1.112" "192.168.149.1"; do
  [[ -z "$ip" ]] && continue
  if sshpass -p "$PASS" ssh $SSH_OPTS "pi@$ip" 'true' 2>/dev/null; then
    PI="pi@$ip"; echo ">>> 小车: $PI"; break
  fi
done
[[ -z "$PI" ]] && { echo "❌ 三个 IP 都连不上"; exit 1; }

# Nav2 也走小车上那个 FastDDS Discovery Server(小车侧用 127.0.0.1)。
# 不显式传的话会退回默认组播发现,和已切到 SERVER 的其它节点互相看不见。
DS_ENV="-e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e ROS_DISCOVERY_SERVER=${RPI_DISCOVERY_SERVER:-127.0.0.1:11811}"
DOCKER="docker exec -u ubuntu $DS_ENV MentorPi bash -lc"

run_on_robot() {
  sshpass -p "$PASS" ssh $SSH_OPTS "$PI" "$1"
}

case "${1:-}" in
  start)
    MAP="${2:-}"
    [ -z "$MAP" ] && usage
    echo ">>> 在小车上启动 Nav2(后台),地图: $MAP"
    # 注意:不用重映射 —— Nav2 发 /cmd_vel,mentorpi 的 odom_publisher 已订阅
    # /app/cmd_vel、/cmd_vel、/controller/cmd_vel 并驱动电机(实测确认)。
    run_on_robot "docker exec -d -u ubuntu $DS_ENV MentorPi bash -lc 'source /opt/ros/humble/setup.bash; \
      ros2 launch nav2_bringup bringup_launch.py \
        use_sim_time:=false \
        map:=$MAP.yaml \
        params_file:=/home/ubuntu/ros2_ws/nav2_params_gui.yaml \
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
    # 必须连 component_container_isolated 一起杀:Nav2 的节点(amcl/map_server/costmap...)
    # 全是作为插件装在这个容器里的,只杀 nav2_bringup 会留下容器 + 全部节点继续跑。
    # ⚠️ 模式必须用 [n]av2_... / [c]omponent_...:裸的 pkill -f PATTERN 会**把自己跑的
    # bash -lc 进程一起匹配杀掉**(bash 自己的命令行就含 PATTERN 字面),后面的就全跑不到了。
    run_on_robot "$DOCKER \"pkill -f '[n]av2_bringup'; pkill -f '[n]av2_.*_node'; pkill -f '[c]omponent_container'; echo stopped\""
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
