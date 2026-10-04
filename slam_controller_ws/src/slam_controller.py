#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# slam_controller_node.py
#
# A long-lived ROS2 node that lives on the mentorpi robot and exposes
# three services so a remote GUI can drive the SLAM lifecycle without
# needing SSH:
#
#   /start_slam  (std_srvs/srv/Trigger)  - trigger SLAM start; method
#                                           comes from the last message on
#                                           /slam_request_method
#   /stop_slam   (std_srvs/srv/Empty)     - kill the current SLAM subprocess
#   /is_mapping  (std_srvs/srv/Trigger)   - true if a SLAM child is running
#
# Also publishes /slam_status (std_msgs/String) with the current state.
#
# Supported methods map to launch files that must already be available
# on the robot's ROS install:
#
#   rtabmap         -> ros2 launch slam rtabmap_slam.launch.py
#   slam_toolbox     -> ros2 launch slam_toolbox online_sync_launch.py
#   hector           -> ros2 launch hector_mapping mapping.launch
#   cartographer     -> ros2 launch cartographer_ros occupancy_grid_node.launch.py
#
# Designed to be run inside the MentorPi Docker container:
#
#   source /opt/ros/humble/setup.bash
#   python3 /home/ubuntu/ros2_ws/src/slam_controller.py
#
# Compatible with both ROS2 Humble (robot) and Jazzy (dev PC) because
# we only depend on std_srvs / std_msgs / rclpy which have stable
# message definitions across distributions.

import json
import os
import signal
import subprocess
import sys
import time
from typing import Optional

# The hard part: mentorpi's ROS2 (rclpy, std_srvs, ...) lives under
# /opt/ros/humble, and the mentorpi workspace overlay at
# /home/ubuntu/ros2_ws/install. The Python interpreter in this container
# won't find them unless the launcher's environment already has the
# AMENT_PREFIX_PATH and LD_LIBRARY_PATH set up. To make this script run
# from ANY context (bash -c, docker exec, systemd, nohup, ssh) we re-
# exec ourselves under bash with `source` applied, then exec python3.

if not os.environ.get('_RCJ_RESPAWNED'):
    overlay_bash = ' && '.join([
        'source /home/ubuntu/ros2_ws/install/setup.bash',
        'source /opt/ros/humble/setup.bash',
        f'source /home/ubuntu/.bashrc 2>/dev/null',
        f'export _RCJ_RESPAWNED=1',
        f'exec /usr/bin/python3 /home/ubuntu/ros2_ws/src/slam_controller.py "$@"',
    ])
    os.execvp('/bin/bash', ['/bin/bash', '-c', overlay_bash])
    sys.exit('failed to respawn')

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger


# Launch-time params the GUI can tune. slam_toolbox reads these ONCE at
# configure time, so changing them requires a restart -> the GUI writes them
# here (via /slam_params + /apply_slam_params) and we relaunch.
SLAM_PARAMS_PATH = '/home/ubuntu/ros2_ws/slam_params.yaml'
DEFAULT_SLAM_PARAMS = {
    'map_update_interval': 5.0,
    'resolution': 0.05,
    'max_laser_range': 20.0,
    'transform_publish_period': 0.02,
    'minimum_travel_distance': 0.5,
}

SLAM_COMMANDS = {
    # The mentorpi 'slam' workspace package is marked NEED_COMPILE by
    # colcon (its build appears incomplete) — its launch files can't be
    # found via ros2 launch. Use the upstream apt packages instead:
    # ros-humble-rtabmap, ros-humble-slam-toolbox, ros-humble-hector-
    # mapping, ros-humble-cartographer — all of which we expect to be
    # installed on the robot. This is the path that actually starts.
    'rtabmap':
        'source /opt/ros/humble/setup.bash && '
        'ros2 launch rtabmap_launch rtabmap.launch.py '
        'rtabmapviz:=false',
    'slam_toolbox':
        'source /opt/ros/humble/setup.bash && '
        'ros2 launch slam_toolbox online_sync_launch.py '
        'use_sim_time:=false '
        f'slam_params_file:={SLAM_PARAMS_PATH}',
    'hector':
        'source /opt/ros/humble/setup.bash && '
        'ros2 launch hector_mapping mapping.launch '
        'use_sim_time:=false',
    'cartographer':
        'source /opt/ros/humble/setup.bash && '
        'ros2 launch cartographer_ros occupancy_grid_node.launch.py '
        'use_sim_time:=false',
}


class SlamController(Node):
    def __init__(self):
        super().__init__('slam_controller')
        self._child: Optional[subprocess.Popen] = None
        self._method: str = ''
        self._pending_method: str = ''

        self._slam_params = dict(DEFAULT_SLAM_PARAMS)

        self.create_service(Trigger, '/start_slam', self._on_start)
        self.create_service(Empty,   '/stop_slam',  self._on_stop)
        self.create_service(Trigger, '/is_mapping', self._on_is_mapping)
        self.create_service(Trigger, '/apply_slam_params', self._on_apply_params)

        self._status_pub = self.create_publisher(String, '/slam_status', 10)
        self.create_subscription(String, '/slam_request_method', self._on_method, 10)
        self.create_subscription(String, '/slam_params', self._on_params, 10)

        # Make sure the launch file has a params file to read at startup.
        self._write_params_yaml()

        # Watchdog timer: poll the child process every 0.5s and reset
        # state if it died.
        self._watchdog = self.create_timer(0.5, self._check_child)

        self._publish_status('idle')
        self.get_logger().info(
            'slam_controller ready on %s. Supported: %s'
            % (os.environ.get('ROS_DOMAIN_ID', '0'), list(SLAM_COMMANDS))
        )

    def _on_method(self, msg):
        m = (msg.data or '').strip()
        if m:
            self._pending_method = m
            self.get_logger().info(f'pending method set: {m}')

    def _on_params(self, msg):
        try:
            data = json.loads(msg.data or '{}')
        except ValueError as e:
            self.get_logger().warn(f'/slam_params: bad JSON ({e})')
            return
        if not isinstance(data, dict):
            self.get_logger().warn('/slam_params: expected a JSON object')
            return
        for k, v in data.items():
            if k in DEFAULT_SLAM_PARAMS and isinstance(v, (int, float)):
                self._slam_params[k] = float(v)
        self.get_logger().info(f'staged SLAM params: {self._slam_params}')

    def _write_params_yaml(self):
        lines = ['slam_toolbox:', '  ros__parameters:']
        for k in DEFAULT_SLAM_PARAMS:
            lines.append(f'    {k}: {self._slam_params.get(k, DEFAULT_SLAM_PARAMS[k])}')
        try:
            with open(SLAM_PARAMS_PATH, 'w') as f:
                f.write('\n'.join(lines) + '\n')
            return True
        except OSError as e:
            self.get_logger().error(f'cannot write {SLAM_PARAMS_PATH}: {e}')
            return False

    def _on_apply_params(self, _request, response):
        if not self._write_params_yaml():
            response.success = False
            response.message = f'cannot write {SLAM_PARAMS_PATH}'
            return response
        was_running = self._child is not None and self._child.poll() is None
        method = self._method
        if was_running:
            self._stop_child()
            if method == 'slam_toolbox':
                self._pending_method = method
                sub = Trigger.Response()
                self._on_start(Trigger.Request(), sub)
                response.success = sub.success
                response.message = f'params written; restarted: {sub.message}'
                self._publish_status(f'params applied, restarted ({method})')
                return response
        response.success = True
        response.message = ('params written (SLAM not running; applies on next start)'
                            if not was_running else 'params written (method is not slam_toolbox)')
        self._publish_status('params applied')
        return response

    def _on_start(self, _request, response):
        if self._child is not None and self._child.poll() is None:
            response.success = False
            response.message = (
                f'SLAM already running (method={self._method}, pid={self._child.pid})'
            )
            return response

        if not self._pending_method:
            response.success = False
            response.message = (
                'no method set: publish a String on /slam_request_method first'
            )
            return response
        method = self._pending_method

        if method not in SLAM_COMMANDS:
            response.success = False
            response.message = (
                f"unknown method '{method}'. Supported: {list(SLAM_COMMANDS)}"
            )
            self.get_logger().error(response.message)
            self._pending_method = ''
            return response

        cmd = SLAM_COMMANDS[method]
        self.get_logger().info(f'starting SLAM: {cmd}')
        self._publish_status(f'starting: {method}')

        log_path = '/tmp/slam_controller_child.log'
        log_fp = open(log_path, 'a')
        try:
            self._child = subprocess.Popen(
                ['/bin/bash', '-c', cmd],
                stdout=log_fp,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                start_new_session=True,
                close_fds=True,
            )
        except OSError as e:
            response.success = False
            response.message = f'failed to spawn child: {e}'
            self.get_logger().error(response.message)
            self._child = None
            return response
        finally:
            log_fp.close()

        self._method = method
        response.success = True
        response.message = f'started pid={self._child.pid}'
        self._publish_status(
            f'running: {method} (pid={self._child.pid})'
        )
        self._pending_method = ''
        self.get_logger().info(response.message)
        return response

    def _on_stop(self, _request, response):
        if self._child is None:
            self._publish_status('idle')
            return response
        self._stop_child()
        self._publish_status('idle')
        self._method = ''
        return response

    def _check_child(self):
        """Periodic watchdog: if the SLAM subprocess died without us
        noticing (launch failure, segfault, OOM), reset state so the GUI
        can start a new one."""
        if self._child is None:
            return
        rc = self._child.poll()
        if rc is not None and rc != 0:
            self.get_logger().warn(
                f'SLAM child exited with code {rc}, resetting state'
            )
            self._child = None
            self._method = ''
            self._publish_status(f'crashed (exit={rc})')

    def _on_is_mapping(self, _request, response):
        alive = self._child is not None and self._child.poll() is None
        response.success = alive
        response.message = (
            f'mapping (method={self._method})' if alive else 'idle'
        )
        return response

    def _stop_child(self):
        if self._child is None:
            return
        pid = self._child.pid
        self.get_logger().info(
            f'stopping SLAM child pid={pid} (method={self._method})'
        )
        try:
            os.killpg(os.getpgid(pid), signal.SIGTERM)
        except ProcessLookupError:
            pass

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if self._child.poll() is not None:
                break
            time.sleep(0.1)

        if self._child.poll() is None:
            self.get_logger().warn('child did not exit, sending SIGKILL')
            try:
                os.killpg(os.getpgid(pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                self._child.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                pass

        self._child = None
        self._method = ''

    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)


def main():
    rclpy.init()
    node = SlamController()
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