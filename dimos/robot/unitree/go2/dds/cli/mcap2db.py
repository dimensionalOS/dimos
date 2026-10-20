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

"""go2web recording (.mcap) -> <stem>.db: every recorded stream, decoded, plus standard ROS views.

* every mcap channel with a codec, under its store name (Go2 DDS, mid360/imu, policy/lowcmd, …)
* mid360_lidar: the MID360 cloud, regenerated bit-exact from the lossless mid360/raw_lvxz
  channel by go2web's `dimos-helper restore` (which also leaves a Point-LIO-replayable .pcap)
* joint_state / motor_state / joint_command / policy_joint_command / body_imu / battery:
  lowstate and lowcmd as sensor_msgs and control_msgs types; the raw streams stay alongside

Conversion only — nothing is estimated or calibrated here. Standalone, not wired into the dimos CLI:

    uv run python -m dimos.robot.unitree.go2.dds.cli.mcap2db recordings/*.mcap
"""

import os
from pathlib import Path
import subprocess
import sys
from typing import TYPE_CHECKING, Any

from dimos.memory.store.mcap import McapStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.dds import ros
from dimos.robot.unitree.go2.dds.codec import GO2_CODECS, CdrStructCodec, FnCodec
from dimos.robot.unitree.go2.dds.msgs.LowCmd import LowCmd
from dimos.robot.unitree.go2.dds.msgs.LowState import LowState
from dimos.robot.unitree.go2.dds.store import STREAMS

if TYPE_CHECKING:
    from dimos.memory.stream import Stream


# go2web's recorder companion: regenerates the mid360 cloud (+ .pcap) from mid360/raw_lvxz.
# On PATH, or wherever DIMOS_HELPER points.
HELPER = os.environ.get("DIMOS_HELPER", "dimos-helper")
CODECS = {
    **GO2_CODECS,
    "mid360/imu": FnCodec(Imu, ros.decode_imu),
    "mid360/cloud": FnCodec(PointCloud2, ros.decode_pointcloud2),
    "policy/lowcmd": CdrStructCodec(LowCmd),
}
EXTRA_STREAMS = {
    "mid360_imu": "mid360/imu",
    "mid360_lidar": "mid360/cloud",
    "policy_lowcmd": "policy/lowcmd",
}


def copy_streams(mcap: Path, dst: SqliteStore) -> None:
    src = McapStore(path=str(mcap), codecs=CODECS, streams={**STREAMS, **EXTRA_STREAMS})
    for name in src.list_streams():
        s: Stream[Any] = src.stream(name)
        first = s.first() if s.count() else None
        if first is None:
            continue
        n = len(s.order_by("ts").save(dst.stream(name, type(first.data))).to_list())
        print(f"[mcap2db]   {name:20s} {type(first.data).__name__:16s} {n}", flush=True)


# Unitree SDK motor order (FR, FL, RR, RL; hip, thigh, calf), Go2 URDF joint names.
LEG_JOINTS = [
    f"{leg}_{j}_joint" for leg in ("FR", "FL", "RR", "RL") for j in ("hip", "thigh", "calf")
]
FEET = [f"{leg}_foot" for leg in ("FR", "FL", "RR", "RL")]
STD_STREAMS = (
    "joint_state",
    "motor_state",
    "joint_command",
    "policy_joint_command",
    "body_imu",
    "battery",
)


def _dynamic_joint_state(ts: float, rows: list[tuple[str, dict[str, float]]]) -> object:
    """ros2_control control_msgs/DynamicJointState: per joint, named interface values."""
    from dimos_lcm.control_msgs import DynamicJointState, InterfaceValue

    msg = DynamicJointState()
    msg.header.stamp.sec = int(ts)
    msg.header.stamp.nsec = int((ts - int(ts)) * 1e9)
    msg.joint_names = [name for name, _ in rows]
    msg.joint_names_length = len(rows)
    msg.interface_values = []
    for _, vals in rows:
        iv = InterfaceValue()
        iv.interface_names, iv.values = list(vals), [float(v) for v in vals.values()]
        iv.interface_names_length = iv.values_length = len(vals)
        msg.interface_values.append(iv)
    msg.interface_values_length = len(rows)
    return msg


def add_std_streams(store: SqliteStore) -> None:
    """Standard ROS-typed views of lowstate/lowcmd. The raw Go2 streams stay as the lossless source."""
    from dimos_lcm.control_msgs import DynamicJointState

    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.sensor_msgs.BatteryState import BatteryState
    from dimos.msgs.sensor_msgs.JointState import JointState

    js, ms = store.stream("joint_state", JointState), store.stream("motor_state", DynamicJointState)
    imu, bat = store.stream("body_imu", Imu), store.stream("battery", BatteryState)
    for o in store.stream("lowstate", LowState).order_by("ts"):
        ls, ts = o.data, o.ts
        legs = ls.motor_state[:12]
        js.append(
            JointState(
                ts=ts,
                name=LEG_JOINTS,
                position=[m.q for m in legs],
                velocity=[m.dq for m in legs],
                effort=[m.tau_est for m in legs],
            ),
            ts=ts,
        )
        motors = [
            (
                n,
                {
                    "position": m.q,
                    "velocity": m.dq,
                    "acceleration": m.ddq,
                    "effort": m.tau_est,
                    "temperature": m.temperature,
                    "mode": m.mode,
                    "lost": m.lost,
                },
            )
            for n, m in zip(LEG_JOINTS, legs, strict=True)
        ]
        feet = [
            (n, {"force": f, "force_est": fe})
            for n, f, fe in zip(FEET, ls.foot_force, ls.foot_force_est, strict=True)
        ]
        ms.append(_dynamic_joint_state(ts, motors + feet), ts=ts)
        q = ls.imu_state.quaternion  # w x y z
        imu.append(
            Imu(
                angular_velocity=Vector3(*map(float, ls.imu_state.gyroscope)),
                linear_acceleration=Vector3(*map(float, ls.imu_state.accelerometer)),
                orientation=Quaternion(float(q[1]), float(q[2]), float(q[3]), float(q[0])),
                frame_id="imu",
                ts=ts,
            ),
            ts=ts,
        )
        bms = ls.bms_state
        bat.append(
            BatteryState(
                ts=ts,
                voltage=ls.power_v,
                current=ls.power_a,
                percentage=bms.soc / 100,
                temperature=float(max(bms.bq_ntc)),
                cell_voltage=[v / 1000 for v in bms.cell_vol],
            ),
            ts=ts,
        )
    for src, out in (("lowcmd", "joint_command"), ("policy_lowcmd", "policy_joint_command")):
        if src not in store.list_streams():
            continue
        dst = store.stream(out, DynamicJointState)
        for oc in store.stream(src, LowCmd).order_by("ts"):
            rows = [
                (
                    n,
                    {
                        "position": c.q,
                        "velocity": c.dq,
                        "effort": c.tau,
                        "stiffness": c.kp,
                        "damping": c.kd,
                        "mode": c.mode,
                    },
                )
                for n, c in zip(LEG_JOINTS, oc.data.motor_cmd[:12], strict=True)
            ]
            dst.append(_dynamic_joint_state(oc.ts, rows), ts=oc.ts)
    for name in STD_STREAMS:
        if name in store.list_streams():
            print(f"[mcap2db]   {name:20s} {store.stream(name).count()}", flush=True)


def convert(mcap: Path) -> SqliteStore:
    """<stem>.mcap -> <stem>.db (plus the restored <stem>.pcap). Returns the open store."""
    out, stem = mcap.parent, mcap.stem
    cloud, db = out / f"{stem}.cloud.mcap", out / f"{stem}.db"
    if db.exists():
        sys.exit(f"{db} exists, delete it to rebuild")
    subprocess.run(["nice", "-n", "19", HELPER, "restore", str(mcap), str(out)], check=True)
    store = SqliteStore(path=str(db))
    copy_streams(mcap, store)
    copy_streams(cloud, store)
    add_std_streams(store)
    cloud.unlink()
    return store


if __name__ == "__main__":
    for p in sys.argv[1:]:
        convert(Path(p).resolve()).stop()
