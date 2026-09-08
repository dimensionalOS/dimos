#!/usr/bin/env python
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

"""Habitat-sim native module: drives a scene from Twist, publishes on zenoh.

habitat-sim ships python 3.9 conda builds only, so this runs in its own env
(``nix/env``) as a NativeModule subprocess. It must stay importable under 3.9
with only numpy, zenoh and dimos_lcm: no dimos imports, no 3.10+ syntax.
``test_server.py`` enforces both.

``dimos_lcm`` is the standalone message package and carries the same
``lcm_encode`` payloads ZenohTransport puts on the wire, so this speaks dimos
without being dimos. The LCM *runtime* is deliberately absent -- see nix/install.sh.

Launch contract (NativeModule ``stdin_config``): one JSON line on stdin holding
``topics`` (port name -> zenoh key expression), ``config`` and ``session``.
"""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

from dimos_lcm.geometry_msgs.Quaternion import Quaternion
from dimos_lcm.geometry_msgs.Transform import Transform
from dimos_lcm.geometry_msgs.TransformStamped import TransformStamped
from dimos_lcm.geometry_msgs.Twist import Twist as LCMTwist
from dimos_lcm.geometry_msgs.Vector3 import Vector3
from dimos_lcm.nav_msgs.Odometry import Odometry as LCMOdometry
from dimos_lcm.sensor_msgs.CameraInfo import CameraInfo as LCMCameraInfo
from dimos_lcm.sensor_msgs.Image import Image as LCMImage
from dimos_lcm.sensor_msgs.PointCloud2 import PointCloud2 as LCMPointCloud2
from dimos_lcm.sensor_msgs.PointField import PointField
from dimos_lcm.std_msgs.Header import Header
from dimos_lcm.tf2_msgs.TFMessage import TFMessage as LCMTFMessage
import numpy as np
import zenoh

_spec = importlib.util.spec_from_file_location(
    "_habitat_frames", Path(__file__).parent / "frames.py"
)
assert _spec is not None and _spec.loader is not None
frames = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(frames)


def log(msg: str) -> None:
    """Plain-text line for NativeModule's log reader (LogFormat.TEXT)."""
    sys.stderr.write(f"[habitat] {msg}\n")
    sys.stderr.flush()


def _stamp(header: Header, ts: float) -> None:
    header.seq = 0
    header.stamp.sec = int(ts)
    header.stamp.nsec = int((ts - int(ts)) * 1e9)


def image_msg(array: np.ndarray, encoding: str, frame_id: str, ts: float) -> bytes:
    m = LCMImage()
    m.header = Header()
    _stamp(m.header, ts)
    m.header.frame_id = frame_id
    m.height, m.width = int(array.shape[0]), int(array.shape[1])
    m.encoding = encoding
    m.is_bigendian = False
    channels = 1 if array.ndim == 2 else array.shape[2]
    m.step = m.width * array.dtype.itemsize * channels
    view = memoryview(np.ascontiguousarray(array)).cast("B")
    m.data_length = len(view)
    m.data = view
    return bytes(m.lcm_encode())


def camera_info_msg(k: dict[str, float], frame_id: str, ts: float) -> bytes:
    m = LCMCameraInfo()
    m.header = Header()
    _stamp(m.header, ts)
    m.header.frame_id = frame_id
    m.width, m.height = int(k["width"]), int(k["height"])
    m.distortion_model = "plumb_bob"
    m.D = [0.0] * 5
    m.D_length = 5
    fx, fy, cx, cy = k["fx"], k["fy"], k["cx"], k["cy"]
    m.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    m.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    m.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    m.binning_x = m.binning_y = 0
    return bytes(m.lcm_encode())


def _xyzrgb_fields() -> list[Any]:
    fields = []
    for i, name in enumerate(["x", "y", "z"]):
        f = PointField()
        f.name, f.offset, f.datatype, f.count = name, i * 4, 7, 1
        fields.append(f)
    f = PointField()
    f.name, f.offset, f.datatype, f.count = "rgb", 12, 7, 1
    fields.append(f)
    return fields


def cloud_msg(points: np.ndarray, colors: np.ndarray, frame_id: str, ts: float) -> bytes:
    """xyz + packed-rgb PointCloud2, matching dimos PointCloud2.lcm_encode's layout."""
    m = LCMPointCloud2()
    m.header = Header()
    _stamp(m.header, ts)
    m.header.frame_id = frame_id
    m.fields = _xyzrgb_fields()
    m.fields_length = 4
    m.point_step = 16
    m.is_bigendian = False
    m.is_dense = True
    m.height = 1
    m.width = len(points)
    if len(points) == 0:
        m.height = 0
        m.row_step = 0
        m.data_length = 0
        m.data = b""
        return bytes(m.lcm_encode())

    # ROS convention: rgb is a float32 whose bytes are [padding, r, g, b].
    rgb_u32 = (
        (colors[:, 0].astype(np.uint32) << 16)
        | (colors[:, 1].astype(np.uint32) << 8)
        | colors[:, 2].astype(np.uint32)
    )
    data = np.column_stack([points.astype(np.float32), rgb_u32.view(np.float32)]).astype(np.float32)
    view = memoryview(np.ascontiguousarray(data)).cast("B")
    m.row_step = m.point_step * m.width
    m.data_length = len(view)
    m.data = view
    return bytes(m.lcm_encode())


def odometry_msg(
    position: np.ndarray,
    quat_xyzw: np.ndarray,
    twist: tuple[float, float, float],
    frame_id: str,
    child_frame_id: str,
    ts: float,
) -> bytes:
    m = LCMOdometry()
    m.header = Header()
    _stamp(m.header, ts)
    m.header.frame_id = frame_id
    m.child_frame_id = child_frame_id
    p = m.pose.pose.position
    p.x, p.y, p.z = [float(v) for v in position]
    q = m.pose.pose.orientation
    q.x, q.y, q.z, q.w = [float(v) for v in quat_xyzw]
    m.pose.covariance = [0.0] * 36
    m.twist.twist.linear.x = float(twist[0])
    m.twist.twist.linear.y = float(twist[1])
    m.twist.twist.angular.z = float(twist[2])
    m.twist.covariance = [0.0] * 36
    return bytes(m.lcm_encode())


def tf_msg(links: list[tuple[str, str, Any, Any]], ts: float) -> bytes:
    """links: (parent, child, translation xyz, rotation xyzw)."""
    m = LCMTFMessage()
    out = []
    for parent, child, xyz, quat in links:
        t = TransformStamped()
        # Fresh Header per link: the generated bindings hand out a shared default,
        # so reusing it makes every transform alias the last one's frame_id.
        t.header = Header()
        _stamp(t.header, ts)
        t.header.frame_id = parent
        t.child_frame_id = child
        # Same story one level down: Transform().translation is shared between
        # instances, so mutating it in place makes all three links serialize the
        # last one's pose -- which reads as the whole map pointing up.
        transform = Transform()
        transform.translation = Vector3()
        transform.rotation = Quaternion()
        (transform.translation.x, transform.translation.y, transform.translation.z) = (
            float(v) for v in xyz
        )
        (
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ) = (float(v) for v in quat)
        t.transform = transform
        out.append(t)
    m.transforms = out
    m.transforms_length = len(out)
    return bytes(m.lcm_encode())


def unproject(
    depth: np.ndarray, rgb: np.ndarray, k: dict[str, float], trunc: float, stride: int
) -> tuple[np.ndarray, np.ndarray]:
    """Depth + colour -> points in the camera OPTICAL frame (x right, y down, z fwd).

    Plain numpy rather than open3d: the py3.9 env has no use for a second heavy
    dependency when the unprojection is six lines.
    """
    d = depth[::stride, ::stride]
    c = rgb[::stride, ::stride]
    fx, fy = k["fx"] / stride, k["fy"] / stride
    cx, cy = k["cx"] / stride, k["cy"] / stride
    h, w = d.shape
    u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    valid = np.isfinite(d) & (d > 0.0) & (d < trunc)
    z = d[valid]
    pts = np.stack([(u[valid] - cx) * z / fx, (v[valid] - cy) * z / fy, z], axis=1)
    return pts.astype(np.float32), c[valid].astype(np.uint8)


class HabitatHost:
    """Owns the Simulator and applies twist commands against the navmesh."""

    def __init__(self, cfg: dict[str, Any]) -> None:
        import habitat_sim  # type: ignore[import-not-found]

        self.hs = habitat_sim
        self.cfg = cfg
        self.width = int(cfg["width"])
        self.height = int(cfg["height"])
        self.hfov = float(cfg["hfov_deg"])
        self.camera_height = float(cfg["camera_height_m"])
        self.publish_semantic = bool(cfg.get("publish_semantic", False))
        self._sim: Any = None
        self._agent: Any = None
        self.yaw = 0.0
        self._build(cfg["scene_id"])

    def _build(self, scene_id: str) -> None:
        hs = self.hs
        backend = hs.SimulatorConfiguration()
        backend.scene_dataset_config_file = self.cfg["scene_dataset_config"]
        backend.scene_id = scene_id
        backend.enable_physics = False
        backend.random_seed = int(self.cfg.get("seed", 0))

        def cam(uuid: str, sensor_type: Any) -> Any:
            spec = hs.CameraSensorSpec()
            spec.uuid = uuid
            spec.sensor_type = sensor_type
            spec.resolution = [self.height, self.width]
            spec.position = [0.0, self.camera_height, 0.0]
            spec.hfov = self.hfov
            return spec

        specs = [cam("rgb", hs.SensorType.COLOR), cam("depth", hs.SensorType.DEPTH)]
        if self.publish_semantic:
            specs.append(cam("semantic", hs.SensorType.SEMANTIC))

        agent_cfg = hs.agent.AgentConfiguration()
        agent_cfg.sensor_specifications = specs
        # Actions are unused: motion is applied as a twist against the navmesh.
        agent_cfg.action_space = {}

        if self._sim is not None:
            self._sim.close()
        self._sim = hs.Simulator(hs.Configuration(backend, [agent_cfg]))
        self._agent = self._sim.initialize_agent(0)
        self.reset_pose()

    def reset_pose(self) -> None:
        self._sim.pathfinder.seed(int(self.cfg.get("seed", 0)))
        state = self._agent.get_state()
        state.position = self._sim.pathfinder.get_random_navigable_point()
        self.yaw = 0.0
        state.rotation = self.hs.utils.common.quat_from_angle_axis(0.0, np.array([0.0, 1.0, 0.0]))
        self._agent.set_state(state)

    def intrinsics(self) -> dict[str, float]:
        """Pinhole intrinsics implied by hfov; square pixels, centred principal point."""
        fx = (self.width / 2.0) / math.tan(math.radians(self.hfov) / 2.0)
        return {
            "fx": float(fx),
            "fy": float(fx),
            "cx": self.width / 2.0,
            "cy": self.height / 2.0,
            "width": float(self.width),
            "height": float(self.height),
        }

    def step(
        self, vx: float, vy: float, wz: float, dt: float
    ) -> tuple[np.ndarray, float, dict[str, np.ndarray]]:
        """Integrate a twist for dt, sliding along the navmesh, then render."""
        state = self._agent.get_state()
        start = np.asarray(state.position, dtype=np.float64)

        self.yaw = float(np.arctan2(np.sin(self.yaw + wz * dt), np.cos(self.yaw + wz * dt)))
        forward, left = frames.habitat_heading(self.yaw)
        target = start + forward * (vx * dt) + left * (vy * dt)

        # try_step slides along geometry instead of teleporting through it. The
        # achieved position is what gets published: never dead-reckon the command.
        achieved = np.asarray(self._sim.pathfinder.try_step(start, target), dtype=np.float64)

        state.position = achieved
        state.rotation = self.hs.utils.common.quat_from_angle_axis(
            self.yaw, np.array([0.0, 1.0, 0.0])
        )
        self._agent.set_state(state)

        return frames.position_to_ros(achieved), self.yaw, self._sim.get_sensor_observations()


def zenoh_session(session: dict[str, Any]) -> Any:
    """Open a zenoh session on the settings NativeModule handed us."""
    cfg = zenoh.Config()
    if session.get("mode"):
        cfg.insert_json5("mode", json.dumps(session["mode"]))
    if session.get("connect"):
        cfg.insert_json5("connect/endpoints", json.dumps(session["connect"]))
    if session.get("listen"):
        cfg.insert_json5("listen/endpoints", json.dumps(session["listen"]))
    if session.get("multicast") is not None:
        cfg.insert_json5("scouting/multicast/enabled", json.dumps(bool(session["multicast"])))
    return zenoh.open(cfg)


def main() -> None:
    launch = json.loads(sys.stdin.readline())
    topics: dict[str, str] = launch["topics"]
    cfg: dict[str, Any] = launch.get("config") or {}
    session = launch.get("session") or {}

    log(f"topics: {sorted(topics)}")
    z = zenoh_session(session)
    pubs = {name: z.declare_publisher(key) for name, key in topics.items() if name != "cmd_vel"}

    cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0}

    def on_cmd_vel(sample: Any) -> None:
        try:
            t = LCMTwist.lcm_decode(bytes(sample.payload.to_bytes()))
            cmd["vx"], cmd["vy"], cmd["wz"] = t.linear.x, t.linear.y, t.angular.z
        except Exception as exc:
            log(f"bad cmd_vel: {exc}")

    if "cmd_vel" in topics:
        z.declare_subscriber(topics["cmd_vel"], on_cmd_vel)

    host = HabitatHost(cfg)
    k = host.intrinsics()
    log("ready: {} fx={:.1f}".format(cfg.get("scene_id"), k["fx"]))

    dt = 1.0 / float(cfg.get("sim_rate_hz", 10.0))
    stride = int(cfg.get("scan_stride", 2))
    trunc = float(cfg.get("max_depth_m", 5.0))
    cam_h = float(cfg.get("camera_height_m", 0.45))
    scan_enabled = bool(cfg.get("publish_scan", True))
    scan_frame = str(cfg.get("scan_frame", "world"))
    optical = frames.optical_to_body_matrix()

    def put(name: str, payload: bytes) -> None:
        pub = pubs.get(name)
        if pub is not None:
            pub.put(payload)

    prev: tuple[np.ndarray, float, float] | None = None
    while True:
        tick = time.time()
        position, yaw, obs = host.step(cmd["vx"], cmd["vy"], cmd["wz"], dt)
        now = time.time()

        rgb = np.ascontiguousarray(obs["rgb"][:, :, :3])
        depth = np.ascontiguousarray(obs["depth"].astype(np.float32))
        put("color_image", image_msg(rgb, "rgb8", "camera_optical", now))
        put("depth_image", image_msg(depth, "32FC1", "camera_optical", now))
        put("camera_info", camera_info_msg(k, "camera_optical", now))
        if "semantic" in obs:
            put(
                "semantic_image",
                image_msg(
                    np.ascontiguousarray(obs["semantic"].astype(np.uint16)),
                    "16UC1",
                    "camera_optical",
                    now,
                ),
            )

        quat = np.array([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)])

        # Achieved velocity, not the command: try_step slides along walls, so the
        # two differ whenever the robot is in contact with geometry.
        if prev is None:
            vel = (0.0, 0.0, 0.0)
        else:
            p0, y0, t0 = prev
            span = max(now - t0, 1e-6)
            d = position - p0
            c, s = math.cos(y0), math.sin(y0)
            vel = (
                float((c * d[0] + s * d[1]) / span),
                float((-s * d[0] + c * d[1]) / span),
                float(math.atan2(math.sin(yaw - y0), math.cos(yaw - y0)) / span),
            )
        prev = (position.copy(), yaw, now)

        put("odometry", odometry_msg(position, quat, vel, "world", "base_link", now))
        put(
            "tf",
            tf_msg(
                [
                    ("world", "base_link", position, quat),
                    ("base_link", "camera", (0.0, 0.0, cam_h), (0.0, 0.0, 0.0, 1.0)),
                    # Body -> optical. Images hang off camera_optical: a pinhole
                    # points down its frame's +z, which in the body frame is up.
                    ("camera", "camera_optical", (0.0, 0.0, 0.0), frames.OPTICAL_QUAT_XYZW),
                ],
                now,
            ),
        )

        if scan_enabled and "registered_scan" in pubs:
            pts, colors = unproject(depth, rgb, k, trunc, stride)
            if scan_frame == "world":
                world = frames.pose_matrix(position + np.array([0.0, 0.0, cam_h]), yaw) @ optical
                pts = pts @ world[:3, :3].T + world[:3, 3]
            put("registered_scan", cloud_msg(pts, colors, scan_frame, now))

        time.sleep(max(0.0, dt - (time.time() - tick)))


if __name__ == "__main__":
    main()
