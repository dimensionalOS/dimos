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

import numpy as np
import pytest

pytest.importorskip("dimos_mls_planner")

from dimos_lcm.sensor_msgs import PointCloud2, PointField

from dimos.navigation.global_planner.mls_planner.mls_planner import MLSPlanner

VOXEL_SIZE = 0.2
# GscPGO's packed record layouts, by field name: new_keyframe, then loop_closure_event.
KEYFRAME_DTYPE = np.dtype(
    [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("node_id", "<u4"), ("ts", "<f8")]
    + [(n, "<f4") for n in ("qx", "qy", "qz", "qw")]
    + [(n, "<f4") for n in ("min_x", "min_y", "min_z", "max_x", "max_y", "max_z")]
)
EVENT_DTYPE = np.dtype(
    [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("node_id", "<u4"), ("ts", "<f8")]
    + [(n, "<f4") for n in ("qx", "qy", "qz", "qw", "dx", "dy", "dz", "dqx", "dqy", "dqz", "dqw")]
    + [("version", "<u4"), ("closed_with", "<u4")]
)
NO_PARTNER = 0xFFFFFFFF
DATATYPES = {"<f4": PointField.FLOAT32, "<f8": PointField.FLOAT64, "<u4": PointField.UINT32}


def cloud_bytes(records: np.ndarray) -> bytes:
    fields = [
        PointField(name=name, offset=offset, datatype=DATATYPES[dt.str], count=1)
        for name, (dt, offset) in records.dtype.fields.items()
    ]
    data = records.tobytes()
    msg = PointCloud2(
        fields_length=len(fields),
        fields=fields,
        height=1,
        width=len(records),
        point_step=records.dtype.itemsize,
        row_step=len(data),
        data_length=len(data),
        data=data,
        is_dense=True,
    )
    return msg.lcm_encode()


def floor():
    coords = np.arange(-3.0, 3.0, VOXEL_SIZE, dtype=np.float32) + VOXEL_SIZE / 2
    xs, ys = np.meshgrid(coords, coords)
    return np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, VOXEL_SIZE / 2, np.float32)], axis=1)


def keyframes():
    records = np.zeros(2, KEYFRAME_DTYPE)
    records["x"] = [-2.0, 2.0]
    records["node_id"] = [0, 1]
    records["ts"] = [0.0, 1.0]
    records["qw"] = 1.0
    records["min_x"], records["min_y"], records["min_z"] = -3.0, -3.0, -1.0
    records["max_x"], records["max_y"], records["max_z"] = 3.0, 3.0, 1.0
    return records


def closure(keyframe_records, dx, version, closed_with=(NO_PARTNER, NO_PARTNER)):
    event = np.zeros(len(keyframe_records), EVENT_DTYPE)
    for name in ("x", "y", "z", "node_id", "ts", "qx", "qy", "qz", "qw"):
        event[name] = keyframe_records[name]
    event["dx"], event["dqw"], event["version"] = dx, 1.0, version
    event["closed_with"] = closed_with
    return cloud_bytes(event)


def make_planner(**kwargs):
    return MLSPlanner(voxel_size=VOXEL_SIZE, robot_height=1.0, reform={}, **kwargs)


def region(planner, version=None):
    return planner.update_region(floor(), (0.0, 0.0), 3.0, -1.0, 2.0, 0.5, version=version)


def test_keyframes_and_a_closing_event_start_a_layer():
    planner = make_planner()
    records = keyframes()
    assert planner.add_keyframes(cloud_bytes(records)) == 2
    assert region(planner)
    outcome = planner.on_closure(closure(records, 0.5, 1, closed_with=(1, 0)))
    assert outcome == {
        "version": 1,
        "closing": [0, 1],
        "new_layer": True,
        "layers": outcome["layers"],
    }
    assert planner.reform_version() == 1


def test_a_cloud_stamped_with_a_closure_not_received_is_refused_for_holding():
    planner = make_planner()
    planner.add_keyframes(cloud_bytes(keyframes()))
    assert region(planner, version=1) is False
    assert region(planner, version=0)


def test_an_unknown_reform_setting_is_rejected():
    with pytest.raises(ValueError):
        MLSPlanner(voxel_size=VOXEL_SIZE, robot_height=1.0, reform={"tile_size": 5.0})
