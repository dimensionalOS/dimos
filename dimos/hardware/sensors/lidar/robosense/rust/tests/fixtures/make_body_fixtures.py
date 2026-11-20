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

"""gen OUT_DIR VOXEL: config for body_filter_replay; compare OUT_DIR: RoboPlan (the Python filter's backend) vs the Rust masks."""

import json
from pathlib import Path
import struct
import sys
import time

import numpy as np


def gen(out: Path, voxel: float) -> None:
    from dimos.robot.deeprobotics.m20 import constants
    from dimos.robot.deeprobotics.m20.model import M20_MODEL, STAND_JOINT_POSITIONS

    desc = M20_MODEL.load()
    (out / "m20.urdf").write_text(desc.xml)
    rng = np.random.default_rng(7)
    sets = [dict(STAND_JOINT_POSITIONS)]
    limits = {j.name: (j.lower, j.upper) for j in desc.joints}
    for _ in range(11):
        s = {}
        for name in STAND_JOINT_POSITIONS:
            lo, hi = limits[name]
            lo, hi = (-np.pi, np.pi) if lo is None or hi is None or hi <= lo else (lo, hi)
            s[name] = float(rng.uniform(lo, hi))
        sets.append(s)
    lidars = {
        constants.FRONT_LIDAR_FRAME: dict(
            msop_port=constants.FRONT_LIDAR_PORTS[0],
            difop_port=constants.FRONT_LIDAR_PORTS[1],
            xyz=list(constants.FRONT_LIDAR_XYZ_M),
            rpy=list(constants.FRONT_LIDAR_RPY),
        ),
        constants.REAR_LIDAR_FRAME: dict(
            msop_port=constants.REAR_LIDAR_PORTS[0],
            difop_port=constants.REAR_LIDAR_PORTS[1],
            xyz=list(constants.REAR_LIDAR_XYZ_M),
            rpy=list(constants.REAR_LIDAR_RPY),
        ),
    }
    cfg = dict(
        urdf_path=str(out / "m20.urdf"),
        padding_m=0.01,
        links=None,
        min_distance=0.1,
        max_distance=60.0,
        voxel=voxel,
        lidars=lidars,
        joint_sets=sets,
    )
    (out / "config.json").write_text(json.dumps(cfg, indent=1))


def read_scans(path: Path):
    b = path.read_bytes()
    at = 0
    out = []
    while at < len(b):
        (n,) = struct.unpack_from("<I", b, at)
        at += 4
        frame = b[at : at + n].decode()
        at += n
        (stamp, count) = struct.unpack_from("<dI", b, at)
        at += 12
        pts = np.frombuffer(b, dtype="<f4", count=count * 3, offset=at).reshape(-1, 3)
        at += count * 12
        out.append((frame, stamp, pts))
    return out


def read_masks(path: Path):
    b = path.read_bytes()
    at = 0
    out = []
    while at < len(b):
        (s, ms, n) = struct.unpack_from("<IdI", b, at)
        at += 16
        out.append((s, ms, np.frombuffer(b, dtype=np.uint8, count=n, offset=at).astype(bool)))
        at += n
    return out


def compare(out: Path) -> None:
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.robot.deeprobotics.m20.model import M20_MODEL
    from dimos.robot.roboplan_body_filter import RoboPlanBodyFilter

    cfg = json.loads((out / "config.json").read_text())
    rp = RoboPlanBodyFilter(M20_MODEL.load(), cfg["padding_m"])
    # the mount exactly as the Python filter gets it from tf (Quaternion.from_euler, Transform.to_matrix)
    mounts = {
        f: Transform(
            translation=Vector3(*l["xyz"]),
            rotation=Quaternion.from_euler(Vector3(*l["rpy"])),
            frame_id="base_link",
            child_frame_id=f,
        ).to_matrix()
        for f, l in cfg["lidars"].items()
    }
    scans, masks = read_scans(out / "scans.bin"), read_masks(out / "masks.bin")
    assert len(scans) == len(masks)
    tot = removed = diff = 0
    diff_scans = 0
    py_ms = []
    rs_ms = []
    per_set = {}
    worst = []
    for (frame, stamp, pts), (s, ms, rust_on_body) in zip(scans, masks, strict=False):
        T = mounts[frame]
        base = np.asarray(pts @ T[:3, :3].T + T[:3, 3], dtype=np.float64)
        t0 = time.perf_counter()
        keep = rp.keep_mask(base, cfg["joint_sets"][s], {})
        py_ms.append((time.perf_counter() - t0) * 1e3)
        py_on_body = ~keep
        d = int(np.count_nonzero(py_on_body != rust_on_body))
        tot += len(pts)
        removed += int(py_on_body.sum())
        diff += d
        diff_scans += d > 0
        rs_ms.append(ms)
        ps = per_set.setdefault(s, [0, 0, 0])
        ps[0] += 1
        ps[1] += int(py_on_body.sum())
        ps[2] += d
        if d:
            worst.append((frame, stamp, s, d, base[py_on_body != rust_on_body][:3].tolist()))
    print(
        json.dumps(
            dict(
                scans=len(scans),
                points=tot,
                python_removed=removed,
                mismatched_points=diff,
                mismatched_scans=diff_scans,
                python_ms_median=float(np.median(py_ms)),
                rust_ms_median=float(np.median(rs_ms)),
                rust_ms_p95=float(np.percentile(rs_ms, 95)),
                per_joint_set={
                    k: dict(scans=v[0], removed=v[1], mismatched=v[2])
                    for k, v in sorted(per_set.items())
                },
                first_mismatches=worst[:5],
            ),
            indent=1,
        )
    )


def fixture(src: Path, dest: Path) -> None:
    """A few real scans (the points near the robot, plus a sparse sample of the rest) with RoboPlan's body mask."""
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.robot.deeprobotics.m20.model import M20_MODEL
    from dimos.robot.roboplan_body_filter import RoboPlanBodyFilter

    cfg = json.loads((src / "config.json").read_text())
    rp = RoboPlanBodyFilter(M20_MODEL.load(), cfg["padding_m"])
    mounts = {
        f: Transform(
            translation=Vector3(*l["xyz"]),
            rotation=Quaternion.from_euler(Vector3(*l["rpy"])),
            frame_id="base_link",
            child_frame_id=f,
        ).to_matrix()
        for f, l in cfg["lidars"].items()
    }
    scans = read_scans(src / "scans.bin")
    masks = read_masks(src / "masks.bin")
    # the scans with the most body points: per lidar, the top one at the standing pose and the top one at another
    picks = []
    for frame in cfg["lidars"]:
        for standing in (True, False):
            ranked = sorted(
                (int(m[2].sum()), i)
                for i, m in enumerate(masks)
                if scans[i][0] == frame and (m[0] == 0) == standing
            )
            picks += [i for _, i in ranked[-1:]]
    out = bytearray()
    summary = []
    for i in picks:
        frame, stamp, pts = scans[i]
        T = mounts[frame]
        joints = cfg["joint_sets"][masks[i][0]]
        body = ~rp.keep_mask(np.asarray(pts @ T[:3, :3].T + T[:3, 3], dtype=np.float64), joints, {})
        # every body point, a fortieth of the others within 1 m of the lidar, a sparse sample of the rest
        index = np.arange(len(pts))
        sel = body | ((np.linalg.norm(pts, axis=1) < 1.0) & (index % 40 == 0)) | (index % 2000 == 0)
        pts, body = np.ascontiguousarray(pts[sel]), body[sel]
        out += (
            struct.pack("<I", len(frame)) + frame.encode() + struct.pack("<dI", stamp, len(joints))
        )
        for name, q in joints.items():
            out += struct.pack("<I", len(name)) + name.encode() + struct.pack("<d", q)
        out += (
            struct.pack("<I", len(pts))
            + pts.astype("<f4").tobytes()
            + body.astype(np.uint8).tobytes()
        )
        summary.append((frame, len(pts), int(body.sum())))
    dest.write_bytes(bytes(out))
    print(len(out), summary)


def synthetic(src: Path, dest: Path) -> None:
    """Points sampled inside every M20 collision primitive at the standing pose (posed by dimos's own URDF joint
    math, not the filter's), in each lidar's frame; all of them are body."""
    import xml.etree.ElementTree as ET

    from scipy.spatial.transform import Rotation

    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.robot.deeprobotics.m20.model import M20_MODEL, STAND_JOINT_POSITIONS

    cfg = json.loads((src / "config.json").read_text())
    desc = M20_MODEL.load()
    joints = {j.child_link: j for j in desc.joints}

    def root_from(link):
        if link not in joints:
            return np.eye(4)
        j = joints[link]
        return root_from(j.parent_link) @ j.parent_from_child(
            STAND_JOINT_POSITIONS.get(j.name, 0.0)
        )

    rng = np.random.default_rng(3)
    body = []
    for link in ET.fromstring(desc.xml).iter("link"):
        for c in link.findall("collision"):
            o = c.find("origin")
            g = c.find("geometry")[0]
            T = np.eye(4)
            if o is not None:
                T[:3, :3] = Rotation.from_euler(
                    "xyz", [float(v) for v in o.get("rpy", "0 0 0").split()]
                ).as_matrix()
                T[:3, 3] = [float(v) for v in o.get("xyz", "0 0 0").split()]
            u = rng.uniform(-1, 1, size=(100, 3)) * 0.999
            if g.tag == "box":
                local = u * np.array([float(v) for v in g.get("size").split()]) / 2
            else:
                r, half = float(g.get("radius")), float(g.get("length")) / 2
                u = u[np.linalg.norm(u[:, :2], axis=1) <= 1]
                local = np.c_[u[:, 0] * r, u[:, 1] * r, u[:, 2] * half]
            W = root_from(link.get("name")) @ T
            body.append(local @ W[:3, :3].T + W[:3, 3])
    body = np.concatenate(body)
    out = bytearray()
    for frame, l in cfg["lidars"].items():
        M = Transform(
            translation=Vector3(*l["xyz"]),
            rotation=Quaternion.from_euler(Vector3(*l["rpy"])),
            frame_id="base_link",
            child_frame_id=frame,
        ).to_matrix()
        inv = np.linalg.inv(M)
        pts = (body @ inv[:3, :3].T + inv[:3, 3]).astype("<f4")
        out += (
            struct.pack("<I", len(frame))
            + frame.encode()
            + struct.pack("<dI", 0.0, len(STAND_JOINT_POSITIONS))
        )
        for name, q in STAND_JOINT_POSITIONS.items():
            out += struct.pack("<I", len(name)) + name.encode() + struct.pack("<d", q)
        out += struct.pack("<I", len(pts)) + pts.tobytes() + np.ones(len(pts), np.uint8).tobytes()
    dest.write_bytes(bytes(out))
    print(len(body), "synthetic body points per lidar")


if __name__ == "__main__":
    if sys.argv[1] == "synthetic":
        synthetic(Path(sys.argv[2]), Path(sys.argv[3]))
    elif sys.argv[1] == "fixture":
        fixture(Path(sys.argv[2]), Path(sys.argv[3]))
    elif sys.argv[1] == "gen":
        gen(Path(sys.argv[2]), float(sys.argv[3]))
    else:
        compare(Path(sys.argv[2]))
