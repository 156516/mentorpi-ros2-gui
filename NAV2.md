# GUI 导航(Nav2)使用说明

> 导航 = 让小车自己沿着规划路径走到你点的目标点。
> 需要小车端跑 **Nav2**(定位 AMCL + 导航),GUI 这边只负责发目标点。

---

## 1. 先有一张地图

Nav2 要一张**已建好的栅格地图**(`<名字>.yaml` + `<名字>.pgm`):

- 用 GUI 建图 → 「建图页 → Save map」或「地图编辑页 → 💾 另存为」→ 存到 PC 的
  `ros2_gui/robot_control_gui_ros2/maps/`
- 拷到小车(不用编辑的话直接拷):
  ```bash
  sshpass -p raspberrypi scp maps/my_map.{pgm,yaml} pi@192.168.149.1:/home/pi/
  sshpass -p raspberrypi ssh pi@192.168.149.1 \
    "docker cp /home/pi/my_map.pgm MentorPi:/home/ubuntu/ros2_ws/maps/ && \
     docker cp /home/pi/my_map.yaml MentorPi:/home/ubuntu/ros2_ws/maps/"
  ```

## 2. 在小车上启动 Nav2

```bash
bash robot_nav2.sh start /home/ubuntu/ros2_ws/maps/my_map
```

> ⚠️ **关键坑**:Nav2 默认往 `/cmd_vel` 发速度,而 mentorpi 底盘订阅
> `/controller/cmd_vel`。脚本里已经 `remappings:="/cmd_vel:=/controller/cmd_vel"`,
> 少了这步车不会动。

检查是否起来了:

```bash
bash robot_nav2.sh status
# 期望看到 nav2 节点、/navigate_to_pose action、amcl
```

## 3. GUI 端操作

1. 连接小车(已经连着就不用)
2. **告诉 AMCL 小车现在在哪** ← 必须先做,否则定位是散的
   - 切到「导航」页或任意页面,**Shift+左键** 在地图上点小车实际所在位置
   - (现在这一步已经接通了;以前点了没反应)
3. **在地图上左键点一个目标点** → 立即下发导航
4. 看「导航」页的状态:导航中 / 已到达 / 失败;剩余距离实时刷新
5. 想中断:点「导航」页的 **✗ 取消导航**

## 4. 小车上「原地建图 ↔ 导航」不能同时

- 建图(slam_toolbox)和导航(Nav2)都要发 `map→odom` TF,**同时开会打架**。
- 正确顺序:**先建图存图 → 停 SLAM → 再起 Nav2 导航**。

---

## 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| 点目标没反应 | 小车端 Nav2 没起 → 先 `robot_nav2.sh status` 看 `/navigate_to_pose` 在不在 |
| 车不动但状态是"导航中" | `/cmd_vel` 没重映射到 `/controller/cmd_vel` |
| 定位乱跳 / 路线穿墙 | 没设初始位姿 → 地图上 Shift+左键点一下小车位置 |
| 地图加载失败 | 地图路径不对,或 `.pgm` 和 `.yaml` 不在同一目录 |
| 建图/导航互相干扰 | 两者都发 `map→odom`;不要同时开 |

---

## 备注

- 脚本 `robot_nav2.sh` **还没在真车上验证过**(小车充电中)。第一次跑建议:
  一个终端 `bash robot_nav2.sh start ...`,另一个 `bash robot_nav2.sh status`。
- mentorpi 自带的 Nav2 参数文件在 `/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml`,
  默认参数够跑;要调(机器人半径、速度上限等)再复制一份改。
