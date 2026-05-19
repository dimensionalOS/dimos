#!/usr/bin/env python3
"""Speed test: send constant cmd_vel for N seconds, measure actual displacement via odom.

Usage (from dimos venv):
    python DimSim/scripts/speed-test.py [--speed 0.5] [--duration 5]

Requires: dimos environment with LCM working.
Run while DimSim bridge is active (browser tab open).
"""

import argparse
import math
import struct
import threading
import time

# LCM multicast constants
LCM_ADDR = "239.255.76.67"
LCM_PORT = 7667
MAGIC = 0x4C433032

# ── Minimal LCM encode/decode (no dimos import needed) ─────────────────────


def _encode_lcm_packet(channel: str, payload: bytes, seq: int) -> bytes:
    ch_bytes = channel.encode("utf-8")
    header = struct.pack(">II", MAGIC, seq)
    return header + ch_bytes + b"\x00" + payload


def _decode_lcm_packet(data: bytes):
    if len(data) < 8:
        return None, None
    magic, seq = struct.unpack(">II", data[:8])
    if magic != MAGIC:
        return None, None
    rest = data[8:]
    null_idx = rest.index(0)
    channel = rest[:null_idx].decode("utf-8")
    payload = rest[null_idx + 1 :]
    return channel, payload


# ── Twist encoding (geometry_msgs.Twist LCM) ───────────────────────────────
# Layout: hash(8) + 6 x float64 (linear.x,y,z, angular.x,y,z)


def encode_twist(linear_x=0.0, linear_y=0.0, linear_z=0.0,
                 angular_x=0.0, angular_y=0.0, angular_z=0.0) -> bytes:
    from dimos.msgs.geometry_msgs import Twist, Vector3
    t = Twist(
        linear=Vector3(x=linear_x, y=linear_y, z=linear_z),
        angular=Vector3(x=angular_x, y=angular_y, z=angular_z),
    )
    return t.lcm_encode()


# ── PoseStamped decoding ───────────────────────────────────────────────────


def decode_pose_stamped(data: bytes):
    """Decode geometry_msgs.PoseStamped using dimos LCM decoder."""
    from dimos.msgs.geometry_msgs import PoseStamped
    msg = PoseStamped.lcm_decode(data)
    p = msg.position
    q = msg.orientation
    return p.x, p.y, p.z, q.x, q.y, q.z, q.w


# ── Main ────────────────────────────────────────────────────────────────────


def main():
    import socket

    parser = argparse.ArgumentParser(description="DimSim speed test")
    parser.add_argument("--speed", type=float, default=0.5, help="linear.x m/s (default 0.5)")
    parser.add_argument("--duration", type=float, default=5.0, help="seconds to drive (default 5)")
    args = parser.parse_args()

    speed = args.speed
    duration = args.duration

    # ── Setup UDP multicast socket ──
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    sock.bind(("", LCM_PORT))

    mreq = struct.pack("4s4s", socket.inet_aton(LCM_ADDR), socket.inet_aton("0.0.0.0"))
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    sock.settimeout(0.1)

    # Send socket (separate so we can send without receiving our own)
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    send_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)

    # ── Collect initial odom ──
    print(f"Waiting for odom...")
    odom_channel = "/odom#geometry_msgs.PoseStamped"
    start_pose = None

    for _ in range(200):  # up to 20s
        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            continue
        ch, payload = _decode_lcm_packet(data)
        if ch == odom_channel:
            start_pose = decode_pose_stamped(payload)
            break

    if start_pose is None:
        print("ERROR: No odom received. Is the bridge running?")
        return

    sx, sy, sz = start_pose[0], start_pose[1], start_pose[2]
    print(f"Start pose: ({sx:.3f}, {sy:.3f}, {sz:.3f})")
    print(f"Sending cmd_vel: linear.x={speed} m/s for {duration}s")
    print(f"Expected displacement: {speed * duration:.2f}m")
    print()

    # ── Drive: send cmd_vel at 10 Hz, collect odom ──
    seq = 1000
    stop = threading.Event()
    poses = []

    def send_cmd_vel():
        nonlocal seq
        while not stop.is_set():
            twist_data = encode_twist(linear_x=speed)
            packet = _encode_lcm_packet("/cmd_vel#geometry_msgs.Twist", twist_data, seq)
            send_sock.sendto(packet, (LCM_ADDR, LCM_PORT))
            seq += 1
            time.sleep(0.1)  # 10 Hz, matches planner rate

    sender = threading.Thread(target=send_cmd_vel, daemon=True)
    sender.start()

    t0 = time.time()
    while time.time() - t0 < duration:
        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            continue
        ch, payload = _decode_lcm_packet(data)
        if ch == odom_channel:
            pose = decode_pose_stamped(payload)
            elapsed = time.time() - t0
            poses.append((elapsed, pose))

    # Stop sending, send zero velocity
    stop.set()
    sender.join()
    for _ in range(5):
        twist_data = encode_twist()
        packet = _encode_lcm_packet("/cmd_vel#geometry_msgs.Twist", twist_data, seq)
        send_sock.sendto(packet, (LCM_ADDR, LCM_PORT))
        seq += 1
        time.sleep(0.05)

    # ── Results ──
    if not poses:
        print("ERROR: No odom received during test")
        return

    last_pose = poses[-1][1]
    ex, ey, ez = last_pose[0], last_pose[1], last_pose[2]
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    expected = speed * duration
    ratio = dist / expected if expected > 0 else 0

    print(f"{'─' * 50}")
    print(f"End pose:    ({ex:.3f}, {ey:.3f}, {ez:.3f})")
    print(f"Displacement: {dist:.3f}m")
    print(f"Expected:     {expected:.3f}m")
    print(f"Ratio:        {ratio:.2f}x  {'(OK)' if 0.85 < ratio < 1.15 else '(SLOW!)' if ratio < 0.85 else '(FAST!)'}")
    print(f"Odom samples: {len(poses)} ({len(poses)/duration:.0f} Hz)")
    print(f"{'─' * 50}")

    # Show velocity over time
    print(f"\nVelocity trace (sampled):")
    prev_t, prev_p = 0, start_pose
    for i, (t, p) in enumerate(poses):
        if i % max(1, len(poses) // 10) != 0:
            continue
        dt = t - prev_t
        if dt > 0:
            d = math.sqrt((p[0]-prev_p[0])**2 + (p[1]-prev_p[1])**2 + (p[2]-prev_p[2])**2)
            v = d / dt
            print(f"  t={t:5.2f}s  v={v:.3f} m/s  pos=({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")
        prev_t, prev_p = t, p

    sock.close()
    send_sock.close()


if __name__ == "__main__":
    main()
