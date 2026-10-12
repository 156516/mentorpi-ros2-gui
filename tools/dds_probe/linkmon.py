#!/usr/bin/env python3
# 后台链路监控:每 5 秒记录 ZT peer 的 NAT 端口 / 延迟 / 到小车的 TCP RTT /
# 到手机校园 IP 的 ping。目的:验证「端口 41383 <-> 高延迟」的相关性,
# 并区分故障发生在 手机NAT/中继 还是 小车<->手机 Wi-Fi 那一跳。
# 只读,不驱动小车。
import datetime
import socket
import subprocess
import time

PEER = '6501e14775'
ROBOT = ('10.132.156.106', 22)   # 经 ZT
PHONE = '10.239.16.102'          # 手机校园网 IP,不经 ZT
DURATION = 3600                  # 跑 1 小时


def zt():
    try:
        out = subprocess.run(['sudo', '-n', 'zerotier-cli', 'peers'],
                             capture_output=True, text=True, timeout=8).stdout
        for line in out.splitlines():
            if line.startswith(PEER):
                f = line.split()
                return f[3], f[5], f[7]      # latency, link, path
    except Exception:
        pass
    return '?', '?', '?'


def tcp(dest, t=3.0):
    s = socket.socket()
    s.settimeout(t)
    a = time.perf_counter()
    try:
        s.connect(dest)
        return '%.0f' % ((time.perf_counter() - a) * 1000)
    except Exception:
        return 'FAIL'
    finally:
        s.close()


def ping(host):
    try:
        r = subprocess.run(['ping', '-c', '1', '-W', '2', host],
                           capture_output=True, text=True, timeout=8)
        if r.returncode == 0:
            for line in r.stdout.splitlines():
                if 'time=' in line:
                    return line.split('time=')[1].split()[0]
    except Exception:
        pass
    return 'FAIL'


print('时间      ZT延迟 链路   端口    TCP_RTT  手机ping   路径', flush=True)
t0 = time.time()
while time.time() - t0 < DURATION:
    lat, link, path = zt()
    port = path.rsplit('/', 1)[-1] if '/' in path else '?'
    rtt = tcp(ROBOT)
    ph = ping(PHONE)
    bad = ''
    if rtt == 'FAIL':
        bad = '   <<<< 断'
    elif rtt.isdigit() and int(rtt) > 300:
        bad = '   <<<< 劣化'
    print('%s %7s %-6s %-6s %-8s %-10s %s%s' % (
        datetime.datetime.now().strftime('%H:%M:%S'),
        lat, link, port, rtt, ph, path, bad), flush=True)
    time.sleep(5)
