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

"""Inspect CDR point-cloud self exclusion as a modeled arm changes position."""

from pathlib import Path
from tempfile import TemporaryDirectory

from dimos_generated.geometry_msgs.msg import Transform, TransformStamped, Vector3
from dimos_generated.sensor_msgs.msg import PointCloud2
from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.manipulation.planning.utils.point_cloud_self_filter import PointCloudSelfFilter
from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_xyz
from dimos.msgs.time import time_from_nanoseconds, to_nanoseconds
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.robot.assets.model import RobotModel

_URDF = """<robot name="self_filter_demo">
<link name="base"/>
<link name="arm"><collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision></link>
<joint name="mount" type="fixed"><parent link="base"/><child link="arm"/></joint>
</robot>"""


def main() -> None:
    with TemporaryDirectory(prefix="dimos-cdr-self-filter-") as directory:
        urdf = Path(directory) / "arm.urdf"
        urdf.write_text(_URDF)
        module = PointCloudSelfFilter(model=RobotModel.from_file(urdf), tf_forward_tolerance_s=0.0)
        buffer = MultiTBuffer()
        module.__dict__["_tf"] = buffer
        try:
            for tick, arm_x in enumerate([0.0, 1.0, 2.0]):
                stamp = time_from_nanoseconds(1700000000123456789 + tick * 1000000000)
                for frame in ("camera", "world"):
                    buffer.receive_transform(
                        TransformStamped(
                            header=Header(frame_id=frame, stamp=stamp),
                            child_frame_id="arm",
                            transform=Transform(translation=Vector3(x=arm_x)),
                        )
                    )
                source = pointcloud_from_xyz(
                    np.array([[arm_x, 0, 0], [5.0, 0, 0]]),
                    header=Header(frame_id="camera", stamp=stamp),
                )
                result = module.filter_cloud(PointCloud2.decode(source.encode()))
                assert result is not None
                filtered, mask = (PointCloud2.decode(value.encode()) for value in result)
                np.testing.assert_allclose(pointcloud_xyz(filtered), [[5.0, 0, 0]])
                assert filtered.header == source.header
                assert mask.header.stamp == stamp and mask.header.frame_id == "world"
                print(
                    f"arm x={arm_x}: input=2 points, retained={filtered.width}, clear-mask cells={mask.width}, source ns={to_nanoseconds(stamp)}"
                )
        finally:
            module.__dict__["_tf"] = None
            module.dispose()
    print("PASS: robot returns excluded; generated mask and cloud preserve exact stamps")


if __name__ == "__main__":
    main()
