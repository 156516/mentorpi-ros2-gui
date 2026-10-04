#!/bin/bash
# deploy_algo.sh <算法名> —— 把你改好的自定义 Nav2 插件编译并部署到小车(ARM64)。
# 用法: bash deploy_algo.sh my_planner
set -e
NAME="${1:-}"
[ -z "$NAME" ] && { echo "用法: $0 <算法名>"; exit 1; }
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/custom_algo/$NAME"
[ -d "$SRC" ] || { echo "找不到 $SRC(先在 GUI 里点「➕ 添加自定义算法」生成模板)"; exit 1; }

PI=pi@192.168.149.1; PASS=raspberrypi
echo ">>> 1/4 传到小车"
tar czf /tmp/_algo.tgz -C "$HERE/custom_algo" "$NAME"
sshpass -p "$PASS" scp -o StrictHostKeyChecking=no /tmp/_algo.tgz "$PI:/home/pi/" >/dev/null

echo ">>> 2/4 放进容器并解包"
sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no "$PI" \
  "docker cp /home/pi/_algo.tgz MentorPi:/tmp/ && \
   docker exec -u ubuntu MentorPi bash -lc 'mkdir -p /home/ubuntu/custom_ws/src && \
     tar xzf /tmp/_algo.tgz -C /home/ubuntu/custom_ws/src'"

echo ">>> 3/4 在小车(ARM64)上编译(约 1-2 分钟)"
sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no "$PI" \
  "docker exec -u ubuntu MentorPi bash -lc 'source /opt/ros/humble/setup.bash && \
     cd /home/ubuntu/custom_ws && colcon build --packages-select $NAME 2>&1 | tail -8'"

echo ">>> 4/4 完成"
# 从 plugins.xml 里读出真实的插件类名(不要猜)
CLASSES=$(grep -o 'name="[^"]*"' "$SRC/plugins.xml" 2>/dev/null | sed 's/name="//;s/"//' | tr '\n' ' ')
echo "插件类名: ${CLASSES:-（未在 plugins.xml 里找到）}"
echo
echo "下一步:"
echo "  1. 若是第一次添加:重启 nav_controller 让它扫描到(bash robot_nav2.sh 里会重启)"
echo "  2. 在 GUI 导航页点「读取当前值」→ 下拉里选它 →「应用并重启导航」"
echo "  3. 若是改了已有算法: 上面两步都做一遍(Nav2 重启才会加载新的 .so)"
