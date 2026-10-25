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

"""Ground-truth pose per keyframe, as a world <- world correction applied to the keyframe's raw (odometry) pose.
Preferred source: the Map Editor icp_stitch tf edge <parent> -> <child> (e.g. map -> odom) written into the recording's
tf stream, which IS the correction C(t). Fallback: a corrected odometry stream (recordings.sh `gt=`), C(t) =
corrected(t) * odometry(t)^-1. Either way gt(kf) = C(t0)^-1 * C(t_kf) * raw(kf), re-anchored at the first sample so
GT starts where the methods' maps start. Returns None when the recording has no GT.

gt spec (manifest entry "gt"): {"db": path, "tf": ["map", "odom"]} and/or {"db": path, "stream": "fastlio_odometry_corrected",
"odom_stream": "fastlio_odometry", "odom_db": optional path}; a list of specs is tried in order."""

from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

from dimos.memory.store.sqlite import SqliteStore


def _table(rows):
    a = np.array(rows, dtype=float)
    return a[np.unique(a[:, 0], return_index=True)[1]]


def _interp(a, ts):
    ts = np.clip(ts, a[0, 0], a[-1, 0])
    rotation = (
        Slerp(a[:, 0], R.from_quat(a[:, 4:]))(ts)
        if len(a) > 1
        else R.from_quat(np.repeat(a[:, 4:], len(ts), 0))
    )
    return rotation, np.stack([np.interp(ts, a[:, 0], a[:, k]) for k in (1, 2, 3)], 1)


def _pose_rows(stream):
    rows = []
    for o in stream.order_by("ts"):
        p = o.data.pose if hasattr(o.data, "pose") and hasattr(o.data.pose, "position") else o.data
        rows.append(
            (
                o.ts,
                p.position.x,
                p.position.y,
                p.position.z,
                p.orientation.x,
                p.orientation.y,
                p.orientation.z,
                p.orientation.w,
            )
        )
    return _table(rows) if rows else None


def _correction_from_tf(spec):
    parent, child = spec["tf"]
    rows = []
    with SqliteStore(path=str(spec["db"])) as store:
        for message in store.stream("tf").tags(child_frame=child).order_by("ts"):
            for tr in message.data.transforms:
                if tr.frame_id.lstrip("/") == parent and tr.child_frame_id.lstrip("/") == child:
                    t, q = tr.translation, tr.rotation
                    rows.append((message.ts, t.x, t.y, t.z, q.x, q.y, q.z, q.w))
    if len(rows) < 2:
        return None
    return _table(rows), f"tf {parent}->{child} ({len(rows)} samples)"


def _correction_from_stream(spec):
    with SqliteStore(path=str(spec["db"])) as store:
        if spec["stream"] not in store.list_streams():
            return None
        corrected = _pose_rows(store.stream(spec["stream"]))
    with SqliteStore(path=str(spec.get("odom_db", spec["db"]))) as store:
        odom = _pose_rows(store.stream(spec["odom_stream"]))
    if corrected is None or odom is None:
        return None
    odom_rotation, odom_translation = _interp(odom, corrected[:, 0])
    rotation = R.from_quat(corrected[:, 4:]) * odom_rotation.inv()
    translation = corrected[:, 1:4] - rotation.apply(odom_translation)
    return np.concatenate(
        [corrected[:, :1], translation, rotation.as_quat()], 1
    ), f"stream {spec['stream']} ({len(corrected)} samples)"


def correction(gt_specs):
    """(table of ts, xyz, xyzw correction samples, description) from the first spec that works, or None."""
    specs = gt_specs if isinstance(gt_specs, list) else [gt_specs] if gt_specs else []
    for spec in specs:
        spec = {k: str(Path(v).expanduser()) if k.endswith("db") else v for k, v in spec.items()}
        if not Path(spec["db"]).exists():
            continue
        found = _correction_from_tf(spec) if "tf" in spec else _correction_from_stream(spec)
        if found is not None:
            return found
    return None


def gt_poses(gt_specs, ts, raw, found=None):
    """(N, 7) GT xyz + xyzw for keyframes at ts with raw poses (N, 7), and a description; (None, None) without GT.
    Keyframes outside the GT's time span get NaN."""
    found = found or correction(gt_specs)
    if found is None:
        return None, None
    table, description = found
    c_rotation, c_translation = _interp(table, np.asarray(ts, float))
    anchor_rotation, anchor_translation = _interp(table, table[:1, 0])
    anchor_inv = anchor_rotation.inv()
    rotation = anchor_inv * c_rotation * R.from_quat(raw[:, 3:])
    xyz = anchor_inv.apply(c_rotation.apply(raw[:, :3]) + c_translation - anchor_translation)
    out = np.concatenate([xyz, rotation.as_quat()], 1)
    outside = (np.asarray(ts) < table[0, 0] - 1.0) | (np.asarray(ts) > table[-1, 0] + 1.0)
    out[outside] = np.nan
    return out, description
