#!/bin/bash
# sync_robot_clock.sh —— 把小车的系统时钟校准到 PC 的时间。
#
# 为什么需要:小车(树莓派)没有 RTC 也不一定连了 NTP,重启后时间可能差很多
# (实测差过 6.5 天)。时间不对会让 ROS2 的 TF 查询**全部失败**
# (按时间戳查,数据"来自未来"或"过期"),表现为:GUI 显示未定位、
# AMCL 报 extrapolation into the future、导航发不出去等一堆玄学问题。
set -e
PI=pi@192.168.149.1
PASS=raspberrypi
NOW_UTC="$(date -u '+%Y-%m-%d %H:%M:%S')"
echo ">>> 校准小车时钟到 UTC $NOW_UTC"
sshpass -p "$PASS" ssh -o ConnectTimeout=6 -o StrictHostKeyChecking=no "$PI" \
  "sudo date -u -s '$NOW_UTC' >/dev/null && echo -n '小车时间: ' && date -u"
