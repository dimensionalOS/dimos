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

from __future__ import annotations

import time
from typing import Any

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu


def encode_call(request_id: str, name: str, arguments: dict[str, Any]) -> dict[str, Any]:
    return {
        "title": name,
        "timestamp": int(time.time() * 1000),
        "guid": request_id,
        "data": arguments,
    }


def encode_response(request_id: str, result: Any) -> dict[str, Any]:
    return {"type": "response", "id": request_id, "result": result}


def encode_error(request_id: str, code: int, message: str) -> dict[str, Any]:
    return {"type": "error", "id": request_id, "error": {"code": code, "message": message}}


def is_response(msg: dict[str, Any]) -> bool:
    if msg.get("type") in ("response", "error") and "id" in msg:
        return True
    title = msg.get("title")
    guid = msg.get("guid")
    return isinstance(title, str) and title.startswith("response_") and isinstance(guid, str)


def is_notify(msg: dict[str, Any]) -> bool:
    if msg.get("type") == "notify" and "name" in msg:
        return True
    title = msg.get("title")
    return isinstance(title, str) and title.startswith("notify_")


def notify_name(msg: dict[str, Any]) -> str:
    if "title" in msg:
        return str(msg.get("title", ""))
    return str(msg.get("name", ""))


def notify_payload(msg: dict[str, Any]) -> dict[str, Any]:
    if "title" in msg:
        out: dict[str, Any] = {}
        if "timestamp" in msg:
            out["timestamp"] = msg["timestamp"]
        if "accid" in msg:
            out["accid"] = msg["accid"]
        data = msg.get("data") or {}
        if isinstance(data, dict):
            out.update(data)
            return out
        out["value"] = data
        return out

    payload = msg.get("data") or msg.get("payload") or msg.get("arguments") or {}
    return payload if isinstance(payload, dict) else {"value": payload}


def build_request_stand_mode() -> dict[str, Any]:
    return {}


def build_request_walk_mode() -> dict[str, Any]:
    return {}


def build_request_sitdown() -> dict[str, Any]:
    return {}


def build_request_recover() -> dict[str, Any]:
    return {}


def build_request_emgy_stop() -> dict[str, Any]:
    return {}


def build_request_enable_odom(enable: bool = True) -> dict[str, Any]:
    return {"enable": enable}


def build_request_enable_imu(enable: bool = True) -> dict[str, Any]:
    return {"enable": enable}


def build_request_base_height(height: float) -> dict[str, Any]:
    return {"height": height}


def build_request_light_effect(effect: str) -> dict[str, Any]:
    return {"effect": effect}


def build_request_twist(
    twist: Twist,
    max_vx: float = 1.0,
    max_vy: float = 1.0,
    max_yaw: float = 1.0,
) -> dict[str, Any]:
    def _ratio(value: float, denom: float) -> float:
        d = denom if denom > 0.0 else 1.0
        r = value / d
        if r > 1.0:
            return 1.0
        if r < -1.0:
            return -1.0
        return float(r)

    return {
        "x": _ratio(twist.linear.x, max_vx),
        "y": _ratio(twist.linear.y, max_vy),
        "z": _ratio(twist.angular.z, max_yaw),
    }


def parse_notify_imu(payload: dict[str, Any], frame_id: str = "imu_link") -> Imu:
    timestamp = payload.get("timestamp")
    if isinstance(timestamp, (int, float)):
        ts = float(timestamp) / 1000.0
    else:
        ts = float(payload.get("ts") or payload.get("stamp") or time.time())

    orientation_raw = payload.get("orientation") or payload.get("quat") or {}
    gyro_raw = payload.get("gyro") or payload.get("angular_velocity") or {}
    acc_raw = payload.get("acc") or payload.get("linear_acceleration") or {}

    def _vec3(v: Any) -> Vector3:
        if isinstance(v, dict):
            return Vector3(float(v.get("x", 0.0)), float(v.get("y", 0.0)), float(v.get("z", 0.0)))
        if isinstance(v, (list, tuple)) and len(v) >= 3:
            return Vector3(float(v[0]), float(v[1]), float(v[2]))
        return Vector3(0.0, 0.0, 0.0)

    def _quat(v: Any) -> Quaternion:
        if isinstance(v, dict):
            return Quaternion(
                float(v.get("x", 0.0)),
                float(v.get("y", 0.0)),
                float(v.get("z", 0.0)),
                float(v.get("w", 1.0)),
            )
        if isinstance(v, (list, tuple)) and len(v) >= 4:
            w = float(v[0])
            x = float(v[1])
            y = float(v[2])
            z = float(v[3])
            if w == 0.0 and x == 0.0 and y == 0.0 and z == 0.0:
                return Quaternion(0.0, 0.0, 0.0, 1.0)
            return Quaternion(x, y, z, w)
        return Quaternion(0.0, 0.0, 0.0, 1.0)

    return Imu(
        ts=ts,
        frame_id=frame_id,
        orientation=_quat(orientation_raw),
        angular_velocity=_vec3(gyro_raw),
        linear_acceleration=_vec3(acc_raw),
    )


def parse_notify_odom(
    payload: dict[str, Any],
    frame_id: str = "odom",
    child_frame_id: str = "base_link",
) -> tuple[Odometry, PoseStamped]:
    timestamp = payload.get("timestamp")
    if isinstance(timestamp, (int, float)):
        ts = float(timestamp) / 1000.0
    else:
        ts = float(payload.get("ts") or payload.get("stamp") or time.time())

    pos_raw = payload.get("pose_position") or payload.get("position") or payload.get("pos") or {}
    ori_raw = payload.get("pose_orientation") or payload.get("orientation") or payload.get("quat") or {}
    lin_vel_raw = payload.get("twist_linear") or payload.get("linear_velocity") or payload.get("vel") or payload.get("linear") or {}
    ang_vel_raw = payload.get("twist_angular") or payload.get("angular_velocity") or payload.get("angular") or {}

    def _vec3(v: Any) -> Vector3:
        if isinstance(v, dict):
            return Vector3(float(v.get("x", 0.0)), float(v.get("y", 0.0)), float(v.get("z", 0.0)))
        if isinstance(v, (list, tuple)) and len(v) >= 3:
            return Vector3(float(v[0]), float(v[1]), float(v[2]))
        return Vector3(0.0, 0.0, 0.0)

    def _quat(v: Any) -> Quaternion:
        if isinstance(v, dict):
            return Quaternion(
                float(v.get("x", 0.0)),
                float(v.get("y", 0.0)),
                float(v.get("z", 0.0)),
                float(v.get("w", 1.0)),
            )
        if isinstance(v, (list, tuple)) and len(v) >= 4:
            x = float(v[0])
            y = float(v[1])
            z = float(v[2])
            w = float(v[3])
            if w == 0.0 and x == 0.0 and y == 0.0 and z == 0.0:
                return Quaternion(0.0, 0.0, 0.0, 1.0)
            return Quaternion(x, y, z, w)
        return Quaternion(0.0, 0.0, 0.0, 1.0)

    pose = Pose(position=_vec3(pos_raw), orientation=_quat(ori_raw))
    twist = Twist(linear=_vec3(lin_vel_raw), angular=_vec3(ang_vel_raw))

    odom = Odometry(ts=ts, frame_id=frame_id, child_frame_id=child_frame_id, pose=pose, twist=twist)
    pose_stamped = PoseStamped(ts=ts, frame_id=frame_id, position=pose.position, orientation=pose.orientation)
    return odom, pose_stamped
