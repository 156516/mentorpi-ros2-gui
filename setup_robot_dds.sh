#!/bin/bash
# setup_robot_dds.sh —— 在小车上落地 / 回滚「FastDDS Discovery Server」跨城发现配置。
#
# 用法:
#   bash setup_robot_dds.sh start    # 起服务器容器 + 把 ROS_DISCOVERY_SERVER 写进 .typerc
#   bash setup_robot_dds.sh stop     # 回滚:移除 .typerc 里那行 + 停掉服务器容器
#   bash setup_robot_dds.sh status   # 看服务器/端口/.typerc 状态
#
# 为什么是 Discovery Server:
#   跨 ZeroTier 时 FastDDS 默认的 multicast 发现极慢(实测首帧 13.1 s、速率掉到 4 Hz)。
#   官方推荐的 WAN 方案是 Discovery Server:所有参与者只跟服务器打交道,不走组播。
#
# 落点为什么是 .typerc:
#   小车端 ROS 环境不来自 docker env(容器 Config.Env 里没有任何 ROS 变量),而来自
#   shell rc 链  ~/.zshrc → ros2_ws/.zshrc → .robotrc → .typerc。
#   而 .typerc 的权威副本在 SD 卡上(宿主机 /home/pi/docker/tmp/.typerc,容器内
#   /home/ubuntu/shared/.typerc),由两个 sync 服务自动推进容器 —— 重启/容器重建都在。
#
# ⚠️ 改完 .typerc 只对新拉起的进程生效,要让 bringup 那批厂家节点吃到,必须:
#      ssh pi@<车> 'sudo systemctl restart start_node.service'
#   我们自己的三个脚本(slam/nav/watchdog)不读 .typerc,靠 start_robot_services.sh 的 -e 传。
#
# ⚠️ 服务器跑在小车上、小车节点用 127.0.0.1 连它 —— 这样 ZT 断了也不影响小车内部底盘链路。

set -e
PASS=raspberrypi
SSH_OPTS="-o ConnectTimeout=20 -o StrictHostKeyChecking=no"
DS_PORT=11811
SRV_NAME=fastdds_discovery
SRV_IMAGE=ros:humble
TYPERC_HOST=/home/pi/docker/tmp/.typerc              # SD 卡上的权威副本
TYPERC_CONTAINER=/home/ubuntu/shared/.typerc         # 容器内同一文件(bind mount)
LINEMARK='# rcj-added: FastDDS Discovery Server(由 setup_robot_dds.sh 管理,勿手改)'
TARGET_LINE="export ROS_DISCOVERY_SERVER=127.0.0.1:${DS_PORT}"

# --- 选通的小车 IP:ZeroTier 优先(与 start_robot_services.sh 一致)---
PI=""
for ip in "${RPI_ZEROTIER_IP:-10.132.156.106}" "192.168.1.112" "192.168.149.1"; do
  [[ -z "$ip" ]] && continue
  if sshpass -p "$PASS" ssh $SSH_OPTS "pi@$ip" 'true' 2>/dev/null; then
    PI="pi@$ip"; echo ">>> 小车: $PI"; break
  fi
done
[[ -z "$PI" ]] && { echo "❌ ZeroTier(10.132.156.106)/有线(192.168.1.112)/AP(192.168.149.1) 都连不上"; exit 1; }

R() { sshpass -p "$PASS" ssh $SSH_OPTS "$PI" "$1"; }

case "${1:-}" in
  start)
    echo ">>> 1/3 起发现服务器容器(${SRV_NAME})"
    R "docker rm -f ${SRV_NAME} >/dev/null 2>&1 || true
       docker run -d --name ${SRV_NAME} --restart unless-stopped --net host \
         ${SRV_IMAGE} bash -lc 'source /opt/ros/humble/setup.bash; exec fastdds discovery -i 0 -p ${DS_PORT}'
       sleep 3
       docker ps --filter name=${SRV_NAME} --format '  {{.Names}} {{.Status}}'"

    echo ">>> 2/3 写 .typerc(幂等;先备份 .bak-ds)"
    R "test -f ${TYPERC_HOST}.bak-ds || cp -p ${TYPERC_HOST} ${TYPERC_HOST}.bak-ds
       if grep -qE '^[[:space:]]*export[[:space:]]+ROS_DISCOVERY_SERVER=' ${TYPERC_HOST}; then
         sed -i -E 's|^[[:space:]]*export[[:space:]]+ROS_DISCOVERY_SERVER=.*|${TARGET_LINE}|' ${TYPERC_HOST}
         echo '  (已存在,就地更新)'
       else
         printf '\n%s\n%s\n' '${LINEMARK}' '${TARGET_LINE}' >> ${TYPERC_HOST}
         echo '  (新增)'
       fi
       grep -n 'ROS_DISCOVERY_SERVER' ${TYPERC_HOST}"

    echo ">>> 3/3 确认已同步进容器(sync 服务是 inotifywait,给它 3 秒)"
    sleep 3
    R "docker exec -u ubuntu MentorPi bash -lc 'grep -n ROS_DISCOVERY_SERVER /home/ubuntu/shared/.typerc /home/ubuntu/ros2_ws/.typerc'"

    cat <<EOF

✅ 服务器 + .typerc 就位。**还差最后一步(只对新进程生效)**:
     ssh pi@${PI#pi@} 'sudo systemctl restart start_node.service'
   然后回 PC 重跑 start_robot_services.sh 与 run_humble_gui.sh。
EOF
    ;;

  stop)
    echo ">>> 回滚:移除 .typerc 里的行 + 停掉服务器容器"
    R "if [ -f ${TYPERC_HOST}.bak-ds ]; then
         cp -p ${TYPERC_HOST}.bak-ds ${TYPERC_HOST}
         echo '  .typerc 已从 .bak-ds 还原'
       else
         sed -i '/ROS_DISCOVERY_SERVER/d' ${TYPERC_HOST}
         echo '  .typerc 已删除相关行(无备份,按行删除)'
       fi
       grep -n ROS_DISCOVERY_SERVER ${TYPERC_HOST} || echo '  (已无该变量)'
       docker rm -f ${SRV_NAME} >/dev/null 2>&1 && echo '  容器已删' || echo '  容器本就不存在'"
    echo
    echo "⚠️ 记得重启小车 ROS 栈让它生效: ssh pi@${PI#pi@} 'sudo systemctl restart start_node.service'"
    ;;

  status)
    echo ">>> 发现服务器容器"
    R "docker ps -a --filter name=${SRV_NAME} --format '  {{.Names}}\t{{.Status}}\t{{.Image}}' || true
       echo '--- 监听端口 ---'
       (ss -lunp 2>/dev/null | grep ${DS_PORT}) || echo '  (宿主机 ss 看不到,见容器日志)'
       echo '--- 容器日志尾 ---'
       docker logs ${SRV_NAME} 2>&1 | tail -6 || true"
    echo ">>> .typerc"
    R "grep -n ROS_DISCOVERY_SERVER ${TYPERC_HOST} || echo '  (未配置)'
       echo '--- 容器内副本 ---'
       docker exec -u ubuntu MentorPi bash -lc 'grep -n ROS_DISCOVERY_SERVER /home/ubuntu/ros2_ws/.typerc || echo \"  容器内未同步\"'"
    echo ">>> 跨 ZT 实测(在 PC 上跑):"
    echo "  docker run --rm --net=host -v \$PWD/tools/dds_probe/probe_one.py:/tmp/probe_one.py:ro \\"
    echo "    -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \\"
    echo "    -e ROS_DISCOVERY_SERVER=10.132.156.106:${DS_PORT} robot_control_gui_humble:latest \\"
    echo "    -c 'source /opt/ros/humble/setup.bash && python3 /tmp/probe_one.py /odom 20'"
    ;;

  *)
    echo "用法: $0 start|stop|status"; exit 1;;
esac
