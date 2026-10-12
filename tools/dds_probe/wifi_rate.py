#!/usr/bin/env python3
# 小车无线口实际收发速率(10 秒平均),以及 ZT 隧道口速率。
import time

IFACES = ('wlan0', 'zteyw5gicp')


def snap():
    d = {}
    with open('/proc/net/dev') as f:
        for l in f:
            if ':' not in l:
                continue
            name, rest = l.split(':')
            f8 = rest.split()
            d[name.strip()] = (int(f8[0]), int(f8[8]))   # rx_bytes, tx_bytes
    return d


a = snap()
t0 = time.time()
time.sleep(10)
b = snap()
dt = time.time() - t0

print("  接口          RX(下行)      TX(上行)     [10 秒平均]")
for k in IFACES:
    if k in a and k in b:
        rx = (b[k][0] - a[k][0]) * 8 / 1e6 / dt
        tx = (b[k][1] - a[k][1]) * 8 / 1e6 / dt
        print("  %-11s %7.2f Mbit/s  %7.2f Mbit/s" % (k, rx, tx))
    else:
        print("  %-11s 不存在" % k)
