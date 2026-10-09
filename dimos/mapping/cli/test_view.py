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

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import encode as cdr_encode
import numpy as np
import rerun as rr

from dimos.mapping.cli.view import main
from dimos.msgs.pointcloud import pointcloud_from_xyz


def test_view_cdr_cloud_writes_offline_recording(tmp_path: Path, capsys):
    pc2 = tmp_path / "map.pc2.cdr"
    cloud = pointcloud_from_xyz(
        np.random.default_rng(0).random((256, 3)),
        header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0)),
    )
    pc2.write_bytes(cdr_encode(cloud))
    out = tmp_path / "view.rrd"
    try:
        main(pc2, voxel=0.05, bottom_cutoff=None, out=out)
    finally:
        rr.disconnect()
    assert "256 points in 'world'" in capsys.readouterr().out
    assert out.exists() and out.stat().st_size > 0
