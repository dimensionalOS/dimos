# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The sim as a motor bus over TCP, for the Rust controller's MuJoCo backend: one 1 ms step per command.

    python -m dimos.simulation.m20_policy_sim.server [--port 43921]

Little-endian. Request: u8 op, then
  1 STEP   80 f64 (kp, q, kd, dq, tau; 16 each, robot order)   -> STATE after one 1 ms physics step
  2 STATE  nothing                                              -> STATE
  3 RESET  u32 length + JSON {"terrain": {...Terrain}, "pose": [16] | "lie", "height_map": {rows, cols}}
                                                                -> STATE
  4 TRUTH  nothing  -> u32 10 + 10 f64: base x, y, z, body vx, vy, yaw rate, tilt (rad), trunk contact (0/1), time,
                     yaw (rad)
  5 LIDAR  u32 length + JSON {"mount_xyz": [3], "mount_rpy": [3], "elevations_deg": [..], "azimuths_deg": [..],
                     "range_noise_m": f, "seed": n}
                                                -> u32 4n + n x (x, y, z, intensity) f64, in the lidar's frame
STATE reply: u32 n + n f64: stamp, quat (w, x, y, z), gyro 3, accel 3, q 16, dq 16, tau 16, then the height map.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import struct

import numpy as np

from dimos.simulation.m20_policy_sim.config import load_config
from dimos.simulation.m20_policy_sim.controller import JointCommand, leg_pose_for_height
from dimos.simulation.m20_policy_sim.runner import projected_gravity
from dimos.simulation.m20_policy_sim.sequences import HEIGHT_MAP_GRID
from dimos.simulation.m20_policy_sim.world import Terrain, World

DEFAULT_PORT = 43921


def recv_exact(conn: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("client closed")
        buf += chunk
    return bytes(buf)


def pack(values: np.ndarray) -> bytes:
    values = np.asarray(values, "<f8")
    return struct.pack("<I", len(values)) + values.tobytes()


class Session:
    def __init__(self, cfg: dict) -> None:
        self.cfg = cfg
        self.height_map: dict | None = None
        self.rng: np.random.Generator | None = None
        self.seed: int | None = None
        self.world = World(Terrain(), torque_limit=np.tile(cfg["robot"]["torque_limit"], 4))
        self.reset({})

    def reset(self, req: dict) -> None:
        terrain = Terrain(**req.get("terrain", {}))
        if terrain != self.world.terrain:
            self.world = World(terrain, torque_limit=np.tile(self.cfg["robot"]["torque_limit"], 4))
        pose = req.get("pose", "lie")
        if pose == "lie":
            pose = leg_pose_for_height(
                self.cfg["robot"], self.cfg["states"]["lie_down"]["lie_height"], 0.0
            )
        self.world.reset(np.asarray(pose, np.float64))
        self.height_map = req.get("height_map")
        self.ticks = 0
        self.heights = None

    def state(self) -> bytes:
        s = self.world.state()
        parts = [[s.stamp], s.quat, s.gyro, s.accel, s.q, s.dq, s.tau]
        if self.height_map:
            if self.heights is None or self.ticks % self.cfg["timing"]["policy_decimation"] == 0:
                hm = self.height_map
                self.heights = self.world.height_map(hm["rows"], hm["cols"], **HEIGHT_MAP_GRID)
            parts.append(self.heights)
        return pack(np.concatenate([np.asarray(p, np.float64).ravel() for p in parts]))

    def truth(self) -> bytes:
        pos, quat = self.world.base_pose()
        v = self.world.base_velocity_body()
        g = projected_gravity(quat)
        tilt = math.acos(max(-1.0, min(1.0, -g[2])))
        contact = float(self.world.body_contacts_ground())
        w, x, y, z = quat
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return pack(np.array([*pos, *v, tilt, contact, self.world.time, yaw]))

    def lidar(self, req: dict) -> bytes:
        if self.rng is None or ("seed" in req and req["seed"] != self.seed):
            self.seed = req.get("seed", 0)
            self.rng = np.random.default_rng(self.seed)
        points = self.world.lidar_scan(
            tuple(req["mount_xyz"]),
            tuple(req["mount_rpy"]),
            np.asarray(req["elevations_deg"], np.float64),
            np.asarray(req["azimuths_deg"], np.float64),
            float(req.get("range_noise_m", 0.0)),
            self.rng,
        )
        return pack(points.ravel())

    def handle(self, conn: socket.socket) -> None:
        while True:
            op = recv_exact(conn, 1)[0]
            if op == 1:
                v = np.frombuffer(recv_exact(conn, 80 * 8), "<f8").reshape(5, 16)
                self.world.step(
                    JointCommand(v[0].copy(), v[1].copy(), v[2].copy(), v[3].copy(), v[4].copy())
                )
                self.ticks += 1
                conn.sendall(self.state())
            elif op == 2:
                conn.sendall(self.state())
            elif op == 3:
                (n,) = struct.unpack("<I", recv_exact(conn, 4))
                self.reset(json.loads(recv_exact(conn, n)))
                conn.sendall(self.state())
            elif op == 4:
                conn.sendall(self.truth())
            elif op == 5:
                (n,) = struct.unpack("<I", recv_exact(conn, 4))
                conn.sendall(self.lidar(json.loads(recv_exact(conn, n))))
            else:
                raise ValueError(f"unknown op {op}")


def serve(port: int, once: bool = False) -> None:
    cfg = load_config()
    session = Session(cfg)
    with socket.create_server(("127.0.0.1", port)) as srv:
        # port 0 takes a free one; the line says which, so parallel clients never collide
        print(f"m20 sim listening on 127.0.0.1:{srv.getsockname()[1]}", flush=True)
        while True:
            conn, _ = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            try:
                session.handle(conn)
            except ConnectionError:
                pass
            finally:
                conn.close()
            if once:
                return


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="0 picks a free port")
    parser.add_argument(
        "--once", action="store_true", help="exit after the first client disconnects"
    )
    args = parser.parse_args()
    serve(args.port, args.once)


if __name__ == "__main__":
    main()
