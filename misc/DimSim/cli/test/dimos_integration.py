#!/usr/bin/env python3
"""
DimSim ↔ dimos Integration Test (UDP Multicast)

Validates end-to-end connectivity between DimSim (browser sim) and dimos
(Python robotics stack) via LCM UDP multicast through the bridge server.

Data flow:
  Python  ──UDP multicast──▶  Bridge Server  ──WebSocket──▶  Browser (DimSim)
  Python  ◀──UDP multicast──  Bridge Server  ◀──WebSocket──  Browser (DimSim)

This script:
  1. Joins the LCM multicast group (239.255.76.67:7667)
  2. Publishes /cmd_vel Twist commands as LCM packets via UDP multicast → agent moves
  3. Listens for /odom, /camera/image, /camera/depth, /lidar/points on multicast
  4. Reports what it receives; SUCCESS when all 4 channels are live

Prerequisites:
  1. Start DimSim bridge:
       ~/.deno/bin/deno run --allow-all --unstable-net cli/cli.ts dev
  2. Open http://localhost:8090 in Chrome (scene must load)
  3. Run this script from the dimos venv:
       /path/to/dimos/.venv/bin/python cli/test/dimos_integration.py

Options:
  --timeout N    Timeout in seconds (default: 30)
  --rate N       cmd_vel publish rate in Hz (default: 10)
"""

import argparse
import socket
import struct
import sys
import threading
import time

# dimos message types for encoding cmd_vel
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3

# -- LCM constants ------------------------------------------------------------
LCM_MAGIC = 0x4C433032  # "LC02" in ASCII / big-endian
MCAST_GRP = "239.255.76.67"
MCAST_PORT = 7667
_seq = 0

# -- LCM packet codec (matches @dimos/msgs encodePacket / decodePacket) --------


def encode_lcm_packet(channel: str, payload: bytes) -> bytes:
    """Encode an LCM binary packet (same format as @dimos/msgs encodePacket)."""
    global _seq
    ch_bytes = channel.encode("utf-8")
    buf = struct.pack(">II", LCM_MAGIC, _seq) + ch_bytes + b"\x00" + payload
    _seq += 1
    return buf


def decode_lcm_packet(data: bytes) -> tuple[str, bytes]:
    """Decode an LCM packet → (channel, payload). Raises ValueError on bad packet."""
    if len(data) < 9:
        raise ValueError("Packet too short")
    magic = struct.unpack_from(">I", data, 0)[0]
    if magic != LCM_MAGIC:
        raise ValueError(f"Bad magic: 0x{magic:08x}")
    null_pos = data.index(0, 8)
    channel = data[8:null_pos].decode("utf-8")
    payload = data[null_pos + 1 :]
    return channel, payload


# -- Channel names (must match DimSim's dimosBridge.ts) ------------------------
CH_CMD_VEL = "/cmd_vel#geometry_msgs.Twist"
CH_ODOM = "/odom#geometry_msgs.PoseStamped"
CH_IMAGE = "/camera/image#sensor_msgs.Image"
CH_DEPTH = "/camera/depth#sensor_msgs.Image"
CH_LIDAR = "/lidar/points#sensor_msgs.PointCloud2"


def create_mcast_recv_socket() -> socket.socket:
    """Create a UDP socket joined to the LCM multicast group for receiving."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except AttributeError:
        pass
    sock.bind(("127.0.0.1", MCAST_PORT))
    mreq = struct.pack("4s4s", socket.inet_aton(MCAST_GRP), socket.inet_aton("0.0.0.0"))
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)
    sock.settimeout(1.0)
    return sock


def create_mcast_send_socket() -> socket.socket:
    """Create a UDP socket for sending to the LCM multicast group."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)
    return sock


def main():
    parser = argparse.ArgumentParser(description="DimSim ↔ dimos integration test (UDP multicast)")
    parser.add_argument("--timeout", type=int, default=30, help="Timeout in seconds")
    parser.add_argument("--rate", type=int, default=10, help="cmd_vel publish rate (Hz)")
    args = parser.parse_args()

    received = {"odom": 0, "image": 0, "depth": 0, "lidar": 0}
    tick = 0
    success = False
    running = True

    recv_sock = create_mcast_recv_socket()
    send_sock = create_mcast_send_socket()

    print(f"[integration] LCM multicast {MCAST_GRP}:{MCAST_PORT}")
    print(f"[integration] Publishing /cmd_vel at {args.rate} Hz")
    print("[integration] Listening for sensor data on multicast")
    print(f"[integration] Timeout: {args.timeout}s\n")

    # -- Receive thread --------------------------------------------------------
    def recv_loop():
        while running:
            try:
                data, addr = recv_sock.recvfrom(65536)
            except TimeoutError:
                continue
            except OSError:
                break

            try:
                channel, payload = decode_lcm_packet(data)
            except (ValueError, IndexError):
                continue

            if "/odom" in channel:
                received["odom"] += 1
                if received["odom"] <= 3 or received["odom"] % 10 == 0:
                    print(f"[integration] Got odom #{received['odom']} ({len(payload)}B)")
            elif "/camera/image" in channel:
                received["image"] += 1
                if received["image"] <= 3 or received["image"] % 10 == 0:
                    print(f"[integration] Got RGB #{received['image']} ({len(payload)}B)")
            elif "/camera/depth" in channel:
                received["depth"] += 1
                if received["depth"] <= 3 or received["depth"] % 10 == 0:
                    print(f"[integration] Got depth #{received['depth']} ({len(payload)}B)")
            elif "/lidar/points" in channel:
                received["lidar"] += 1
                if received["lidar"] <= 3 or received["lidar"] % 10 == 0:
                    print(f"[integration] Got LiDAR #{received['lidar']} ({len(payload)}B)")

    recv_thread = threading.Thread(target=recv_loop, daemon=True)
    recv_thread.start()

    # -- Send loop -------------------------------------------------------------
    interval = 1.0 / args.rate
    start_time = time.time()

    try:
        while time.time() - start_time < args.timeout:
            # Build Twist — Three.js identity: z=forward, y=yaw
            twist = Twist(
                linear=Vector3(0, 0, 0.5),
                angular=Vector3(0, 0.3, 0),
            )
            payload = twist.lcm_encode()
            packet = encode_lcm_packet(CH_CMD_VEL, payload)
            send_sock.sendto(packet, (MCAST_GRP, MCAST_PORT))
            tick += 1

            if tick <= 3 or tick % 20 == 0:
                print(f"[integration] Sent cmd_vel #{tick}")

            # Status check every 5s
            elapsed = time.time() - start_time
            if tick > 1 and (tick % (args.rate * 5) == 0):
                print(
                    f"\n[integration] STATUS ({elapsed:.0f}s): "
                    f"cmd_sent={tick} odom={received['odom']} "
                    f"rgb={received['image']} depth={received['depth']} "
                    f"lidar={received['lidar']}"
                )

                if all(v > 0 for v in received.values()):
                    success = True
                    print("\n========================================")
                    print("  SUCCESS: All channels working!")
                    print("  DimSim ↔ dimos LCM multicast verified.")
                    print("========================================\n")
                    break

                if received["odom"] == 0 and elapsed > 10:
                    print("[integration] No sensor data on multicast. Check:")
                    print("  1. Bridge running with vendored @dimos/lcm (joinMulticastV4)")
                    print("  2. Browser open at localhost:8090 with scene loaded")
                print()

            time.sleep(interval)

        if not success:
            print(f"\n[integration] TIMEOUT after {args.timeout}s")
            print(
                f"[integration] Final: cmd_sent={tick} odom={received['odom']} "
                f"rgb={received['image']} depth={received['depth']} "
                f"lidar={received['lidar']}"
            )

    except KeyboardInterrupt:
        print("\n[integration] Interrupted by user")

    finally:
        running = False
        # Send zero velocity (safety stop)
        try:
            stop_twist = Twist()
            stop_pkt = encode_lcm_packet(CH_CMD_VEL, stop_twist.lcm_encode())
            send_sock.sendto(stop_pkt, (MCAST_GRP, MCAST_PORT))
        except Exception:
            pass

        recv_sock.close()
        send_sock.close()
        print("[integration] Done.")

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
