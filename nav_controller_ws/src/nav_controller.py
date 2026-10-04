#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# nav_controller.py — 小车端 Nav2 管理器(和 slam_controller.py 一个套路)
#
# 让 GUI 可以在界面上:
#   1. 选 全局规划器(planner)  和 轨迹跟踪控制器(controller)
#   2. 改几个通用参数(车半径 / 膨胀半径 / 额外参数)
#   3. 一键写参数文件并重启 Nav2
#
# 接口:
#   /nav_params        (std_msgs/String, JSON)  —— 暂存选择
#   /apply_nav_params  (std_srvs/Trigger)       —— 写配置 + 重启 Nav2
#   /stop_nav          (std_srvs/Trigger)
#   /is_nav            (std_srvs/Trigger)
#   /nav_status        (std_msgs/String, 发布)
#
# JSON 形如:
#   {"planner":"smac_hybrid", "controller":"rpp", "map":"/home/ubuntu/ros2_ws/maps/test2",
#    "robot_radius":0.20, "inflation_radius":0.55,
#    "extra":{"controller_server":{"FollowPath":{"max_vel_x":0.4}}}}
#
# 依赖:python3-yaml(ros-humble 自带)

import json
import os
import signal
import subprocess
import sys
import time
from typing import Optional

if not os.environ.get('_RCJ_NAV_RESPAWNED'):
    overlay = ' && '.join([
        'source /home/ubuntu/ros2_ws/install/setup.bash',
        'source /opt/ros/humble/setup.bash',
        'source /home/ubuntu/.bashrc 2>/dev/null',
        'export _RCJ_NAV_RESPAWNED=1',
        'exec /usr/bin/python3 /home/ubuntu/ros2_ws/src/nav_controller.py "$@"',
    ])
    os.execvp('/bin/bash', ['/bin/bash', '-c', overlay])
    sys.exit('failed to respawn')

import glob
import math
import xml.etree.ElementTree as ET
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import Trigger

STOCK_PARAMS = '/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml'
# GUI 通过 /nav_map_upload 传上来的地图存这里;导航默认用它。
GUI_MAP_BASE = '/home/ubuntu/ros2_ws/maps/gui_map'
GUI_PARAMS   = '/home/ubuntu/ros2_ws/nav2_params_gui.yaml'
CUSTOM_WS    = '/home/ubuntu/custom_ws/install'   # deploy_algo.sh 部署到这里

# 可选算法(名字 → 插件类)。GUI 下拉用的 key 必须和这里一致。
PLANNERS = {
    'navfn':        'nav2_navfn_planner/NavfnPlanner',
    'smac2d':       'nav2_smac_planner/SmacPlanner2D',
    'smac_hybrid':  'nav2_smac_planner/SmacPlannerHybrid',
    'smac_lattice': 'nav2_smac_planner/SmacPlannerLattice',
    'theta_star':   'nav2_theta_star_planner/ThetaStarPlanner',
}
CONTROLLERS = {
    'dwb':  'dwb_core::DWBLocalPlanner',
    'rpp':  'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
    'mppi': 'nav2_mppi_controller::MPPIController',
}
DEFAULT_CFG = {
    'planner': 'navfn',
    'controller': 'dwb',
    'map': '',   # 空 = 用 GUI 上传的 gui_map
    'robot_radius': 0.20,
    'inflation_radius': 0.55,
    'extra': {},
}


class NavController(Node):
    def __init__(self):
        super().__init__('nav_controller')
        self._child: Optional[subprocess.Popen] = None
        self._cfg = dict(DEFAULT_CFG)

        self._scan_custom_plugins()
        self.create_service(Trigger, '/apply_nav_params', self._on_apply)
        self.create_service(Trigger, '/list_nav_plugins', self._on_list)
        self.create_service(Trigger, '/stop_nav', self._on_stop)
        self.create_service(Trigger, '/is_nav', self._on_is)
        self.create_subscription(String, '/nav_params', self._on_params, 10)
        # GUI 上传地图(latched;晚订阅也能收到)
        self.create_subscription(OccupancyGrid, '/nav_map_upload', self._on_map,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._status_pub = self.create_publisher(String, '/nav_status', 10)
        self.create_timer(0.5, self._check_child)

        self._publish(f'idle (planners={list(PLANNERS)}, controllers={list(CONTROLLERS)})')
        self.get_logger().info('nav_controller ready. planners=%s controllers=%s'
                               % (list(PLANNERS), list(CONTROLLERS)))

    # ---------- 自定义插件扫描 ----------
    def _scan_custom_plugins(self):
        """扫 CUSTOM_WS 下所有 plugins.xml,把自定义的规划器/控制器并进可选列表。"""
        n = 0
        for path in glob.glob(CUSTOM_WS + '/*/share/*/plugins.xml'):
            try:
                for lib in ET.parse(path).getroot().iter('class'):
                    cls = lib.get('name')
                    base = lib.get('base_class_type') or ''
                    if not cls: continue
                    if 'GlobalPlanner' in base:
                        PLANNERS[cls] = cls; n += 1
                    elif 'Controller' in base:
                        CONTROLLERS[cls] = cls; n += 1
            except Exception as e:
                self.get_logger().warn('解析 %s 失败: %s' % (path, e))
        if n:
            self.get_logger().info('发现 %d 个自定义插件: planners=%s controllers=%s'
                                   % (n, [k for k in PLANNERS if '/' in k],
                                      [k for k in CONTROLLERS if '/' in k]))

    def _on_list(self, _req, resp):
        resp.success = True
        resp.message = json.dumps({'planners': sorted(PLANNERS), 'controllers': sorted(CONTROLLERS)})
        return resp

    # ---------- 接口 ----------
    def _on_params(self, msg):
        try:
            d = json.loads(msg.data or '{}')
        except ValueError as e:
            self.get_logger().warn('/nav_params: bad JSON (%s)' % e)
            return
        if not isinstance(d, dict):
            return
        for k in ('planner', 'controller', 'map'):
            if k in d and isinstance(d[k], str):
                self._cfg[k] = d[k]
        for k in ('robot_radius', 'inflation_radius'):
            if k in d and isinstance(d[k], (int, float)):
                self._cfg[k] = float(d[k])
        if isinstance(d.get('extra'), dict):
            self._cfg['extra'] = d['extra']
        self.get_logger().info('staged nav cfg: %s' % json.dumps(self._cfg, ensure_ascii=False))

    def _on_map(self, g):
        """GUI 把地图通过 ROS 话题发过来 -> 存成 PGM+YAML,并设为导航用图。"""
        try:
            self._write_map_files(g)
        except Exception as e:
            self.get_logger().error('写地图失败: %s' % e)
            return
        self._cfg['map'] = GUI_MAP_BASE
        self._publish('map received from GUI (%dx%d)' % (g.info.width, g.info.height))
        self.get_logger().info('GUI map saved: %s.pgm/.yaml (%dx%d)'
                               % (GUI_MAP_BASE, g.info.width, g.info.height))

    def _write_map_files(self, g):
        w, h = int(g.info.width), int(g.info.height)
        res = float(g.info.resolution)
        ox = float(g.info.origin.position.x)
        oy = float(g.info.origin.position.y)
        q = g.info.origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        os.makedirs(os.path.dirname(GUI_MAP_BASE), exist_ok=True)
        # PGM P5:行从上往下(地图最后一行 = 最高 y);100->黑(0), 0->白(254), -1->205
        with open(GUI_MAP_BASE + '.pgm', 'wb') as f:
            f.write(b'P5\n%d %d\n255\n' % (w, h))
            for r in range(h - 1, -1, -1):
                row = bytearray(w)
                base = r * w
                for c in range(w):
                    v = g.data[base + c]
                    row[c] = 205 if v < 0 else (0 if v >= 50 else 254)
                f.write(bytes(row))
        with open(GUI_MAP_BASE + '.yaml', 'w') as f:
            f.write('image: gui_map.pgm\nresolution: %.6f\norigin: [%.6f, %.6f, %.6f]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
                    % (res, ox, oy, yaw))

    def _on_apply(self, _req, resp):
        planner = self._cfg.get('planner', 'navfn')
        controller = self._cfg.get('controller', 'dwb')
        # 允许直接给插件类名(如 my_aster/MyPlanner)—— 不在内置表里就原样用
        pclass = PLANNERS.get(planner, planner if '/' in planner else None)
        cclass = CONTROLLERS.get(controller, controller if '/' in controller else None)
        if pclass is None:
            resp.success = False; resp.message = 'unknown planner: %s' % planner; return resp
        if cclass is None:
            resp.success = False; resp.message = 'unknown controller: %s' % controller; return resp
        if not self._cfg.get('map'):
            if os.path.exists(GUI_MAP_BASE + '.yaml'):
                self._cfg['map'] = GUI_MAP_BASE
            else:
                resp.success = False
                resp.message = ('没有可用地图:GUI 还没上传过,也没指定地图路径')
                return resp
        try:
            self._write_params(pclass, cclass)
        except Exception as e:
            resp.success = False; resp.message = '写参数文件失败: %s' % e; return resp

        self._stop_child()
        ok, msg = self._start_child()
        resp.success = ok
        resp.message = msg
        self._publish('running' if ok else 'failed: %s' % msg)
        return resp

    def _on_stop(self, _req, resp):
        self._stop_child()
        self._publish('idle')
        resp.success = True; resp.message = 'stopped'
        return resp

    def _on_is(self, _req, resp):
        alive = self._child is not None and self._child.poll() is None
        resp.success = alive
        resp.message = 'navigating' if alive else 'idle'
        return resp

    # ---------- 内部 ----------
    def _write_params(self, planner, controller):  # 参数已是插件类名
        with open(STOCK_PARAMS) as f:
            p = yaml.safe_load(f)

        # 规划器
        p.setdefault('planner_server', {}).setdefault('ros__parameters', {})
        pp = p['planner_server']['ros__parameters']
        pp['planner_plugins'] = ['GridBased']
        pp.setdefault('GridBased', {})
        pp['GridBased']['plugin'] = planner      # 已经是类名

        # 控制器
        cp = p['controller_server']['ros__parameters']
        cp['controller_plugins'] = ['FollowPath']
        cp.setdefault('FollowPath', {})
        cp['FollowPath']['plugin'] = controller   # 已经是类名

        # 通用:车半径 + 膨胀半径(两个代价地图都设)
        rr = self._cfg.get('robot_radius')
        ir = self._cfg.get('inflation_radius')
        for cm in ('global_costmap', 'local_costmap'):
            c = p.get(cm, {}).get('ros__parameters')
            if not isinstance(c, dict):
                continue
            c['robot_radius'] = rr
            if isinstance(c.get('inflation_layer'), dict):
                c['inflation_layer']['inflation_radius'] = ir

        # 额外(高级)参数:深合并,例如 {"controller_server":{"FollowPath":{"max_vel_x":0.4}}}
        extra = self._cfg.get('extra') or {}
        for node_name, sub in extra.items():
            if node_name not in p or not isinstance(sub, dict):
                continue
            target = p[node_name].setdefault('ros__parameters', {})
            self._deep_merge(target, sub)

        map_path = self._cfg.get('map', DEFAULT_CFG['map'])
        p['map_server'] = p.get('map_server') or {}
        p['map_server'].setdefault('ros__parameters', {})['yaml_filename'] = map_path + '.yaml'

        os.makedirs(os.path.dirname(GUI_PARAMS), exist_ok=True)
        with open(GUI_PARAMS, 'w') as f:
            yaml.safe_dump(p, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
        self.get_logger().info('wrote %s (planner=%s controller=%s)' % (GUI_PARAMS, planner, controller))

    @staticmethod
    def _deep_merge(dst, src):
        for k, v in src.items():
            if isinstance(v, dict) and isinstance(dst.get(k), dict):
                NavController._deep_merge(dst[k], v)
            else:
                dst[k] = v

    def _start_child(self):
        map_path = self._cfg.get('map', DEFAULT_CFG['map'])
        # 先 source 自定义算法工作区(deploy_algo.sh 部署的插件在这),
        # 否则 Nav2 的插件表里没有你的算法 -> "class ... does not exist"
        cmd = ('source /opt/ros/humble/setup.bash && '
               'if [ -f /home/ubuntu/custom_ws/install/setup.bash ]; then '
               '  source /home/ubuntu/custom_ws/install/setup.bash; '
               'fi && '
               'ros2 launch nav2_bringup bringup_launch.py '
               'use_sim_time:=false '
               'map:=%s.yaml '
               'params_file:=%s' % (map_path, GUI_PARAMS))
        self.get_logger().info('starting Nav2: %s' % cmd)
        log = open('/tmp/nav_controller_child.log', 'a')
        try:
            self._child = subprocess.Popen(
                ['/bin/bash', '-c', cmd], stdout=log, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL, start_new_session=True, close_fds=True)
        except OSError as e:
            log.close()
            return False, 'failed to spawn: %s' % e
        log.close()
        time.sleep(3)
        return True, 'started pid=%s' % self._child.pid

    def _stop_child(self):
        if self._child is None:
            return
        pid = self._child.pid
        try:
            os.killpg(os.getpgid(pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline and self._child.poll() is None:
            time.sleep(0.1)
        if self._child.poll() is None:
            try:
                os.killpg(os.getpgid(pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        self._child = None
        # 保险:清掉残留的 nav2 进程
        subprocess.call(['/bin/bash', '-c',
                         "pkill -f 'nav2_.*_node' 2>/dev/null; pkill -f nav2_bringup 2>/dev/null; true"])

    def _check_child(self):
        if self._child is not None and self._child.poll() is not None:
            self.get_logger().warn('Nav2 child exited, resetting')
            self._child = None
            self._publish('idle (child exited)')

    def _publish(self, s):
        m = String(); m.data = s
        self._status_pub.publish(m)


def main():
    rclpy.init()
    node = NavController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._stop_child()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
