#!/usr/bin/env python3
"""Debug nav: monitor odom, path, cmd_vel, way_point simultaneously."""

import math
import time
import lcm  # type: ignore[import-untyped]

from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.PointStamped import PointStamped

LCM_URL = "udpm://239.255.76.67:7667?ttl=0"
lc = lcm.LCM(LCM_URL)
last = {}

def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def on_odom(_ch, data):
    now = time.time()
    if now - last.get('odom', 0) < 1.0:
        return
    last['odom'] = now
    odom = Odometry.lcm_decode(data)
    p = odom.pose.position
    yaw = quat_to_yaw(odom.pose.orientation)
    print(f"[ODOM] pos=({p.x:+.3f}, {p.y:+.3f}) yaw={math.degrees(yaw):+.1f}°")


def on_path(_ch, data):
    now = time.time()
    if now - last.get('path', 0) < 2.0:
        return
    last['path'] = now
    path = Path.lcm_decode(data)
    n = len(path.poses) if hasattr(path, 'poses') else 0
    if n > 0:
        ps0 = path.poses[0]
        psn = path.poses[n-1]
        p0 = ps0.pose.position if hasattr(ps0, 'pose') else ps0.position
        pn = psn.pose.position if hasattr(psn, 'pose') else psn.position
        # Path direction (vehicle frame)
        if n > 1:
            dx = pn.x - p0.x
            dy = pn.y - p0.y
            ang = math.degrees(math.atan2(dy, dx))
        else:
            ang = 0
        print(f"[PATH] {n} pts: first=({p0.x:+.2f},{p0.y:+.2f}) last=({pn.x:+.2f},{pn.y:+.2f}) dir={ang:+.1f}°")
    else:
        print("[PATH] empty")


def on_cmd(_ch, data):
    now = time.time()
    if now - last.get('cmd', 0) < 1.0:
        return
    last['cmd'] = now
    t = Twist.lcm_decode(data)
    speed = math.sqrt(t.linear.x**2 + t.linear.y**2)
    if speed > 0.01:
        heading = math.degrees(math.atan2(t.linear.y, t.linear.x))
    else:
        heading = 0
    print(f"[CMD]  linear=({t.linear.x:+.3f}, {t.linear.y:+.3f}) ang_z={t.angular.z:+.3f} speed={speed:.3f} heading={heading:+.1f}°")


def on_wp(_ch, data):
    wp = PointStamped.lcm_decode(data)
    now = time.time()
    if now - last.get('wp', 0) < 2.0:
        return
    last['wp'] = now
    print(f"[WP]   way_point=({wp.x:+.2f}, {wp.y:+.2f})")


lc.subscribe("/odometry#nav_msgs.Odometry", on_odom)
lc.subscribe("/path#nav_msgs.Path", on_path)
lc.subscribe("/nav_cmd_vel#geometry_msgs.Twist", on_cmd)
lc.subscribe("/way_point#geometry_msgs.PointStamped", on_wp)

print("Listening for odom, path, cmd_vel, way_point...")
while True:
    lc.handle_timeout(100)
