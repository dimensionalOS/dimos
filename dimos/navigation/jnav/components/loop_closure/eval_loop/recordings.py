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

"""The benchmark's recordings, in priority order, and where each one is downloaded from.

Cloud recordings come from console.dimensional.org (`dimos login` first) and need `prep.py`;
LFS recordings already have the pose-stamped go2 `lidar` stream both PGOs take.
"""

from dataclasses import dataclass
import os
from pathlib import Path

CACHE = Path(os.environ.get("EVAL_LOOP_DATA", Path.home() / ".cache/dimos/eval_loop")).expanduser()


@dataclass(frozen=True)
class Dataset:
    name: str
    cloud_id: str = ""  # console.dimensional.org upload id; empty = LFS
    lidar: str = "lidar"
    odom: str = ""
    # camera for Ivan's AprilTag metric: "go2" = his hard-coded go2 front intrinsics, "stream" = the
    # recording's own `camera_info`, "" = no camera
    tag_camera: str = ""
    # odometry the camera's poses were recorded against, when it isn't `odom`: markers are re-expressed
    # from its world into `odom`'s at each sighting
    tag_odom: str = ""
    # seconds `odom`'s clock runs ahead of the camera's
    clock_offset_s: float = 0.0
    # camera frame in the recording's tf: when set, each image's camera pose is the scan pose at its
    # time times the static tf to this frame, and the images' own stored poses are not used
    tag_camera_frame: str = ""
    # the frame whose pose `odom` really reports, when its child_frame_id says otherwise (passed to prep.py)
    odom_pose_of: str = ""
    # marker ids on something that moves (not a landmark), left out of the tag spread
    tag_ids_moving: tuple[int, ...] = ()
    # a local recording (an .mcap's lidar/odom/tf copied into a .db with `dtk data to_db`), for ones not uploaded
    local: str = ""
    # voxel leaf (m) prep.py downsamples scans to, for a recording that stored them undownsampled
    voxel_m: float = 0.0

    def source(self) -> Path:
        """Path of the recording to benchmark, downloading it on first use."""
        if self.local:
            return Path(self.local).expanduser()
        if not self.cloud_id:
            from dimos.utils.data import get_data

            return get_data(f"{self.name}.db")
        out = CACHE / f"{self.cloud_id}.db"
        if not out.exists():
            from dimos.cloud.data import CloudData

            print(f"{self.name}: downloading {self.cloud_id} -> {out}", flush=True)
            CloudData().pull(self.cloud_id, out)
        return out


# id 17 is on the back of a robot dog that moves around the course
HUGE_LOOP_MOVING_TAGS = (17,)
SMALL_LOOP = "cd84b786a09c41c8a5435a02534a4355"
HUGE_LOOP = "b534a3db43424661b8f6710ef73dd4e9"
# the same 2026-06-04 walk recorded on the go2 (go2 lidar + odom, and a Mid-360 run through both LIOs)
HUGE_LOOP_GO2 = "dbf5283feeb446d49f065a100985a78e"

DATASETS = [
    Dataset("small_loop_go2", SMALL_LOOP, "lidar", "odom", "go2"),
    # its LIO odometry says child base_link but reports the Mid-360's pose (the 44 deg mount is
    # already in it), so prep.py takes it as mid360_link's pose.
    # the go2 db's images carry old_fastlio_odometry poses, a FAST-LIO build with velocity spikes, so
    # camera poses come from each dataset's own odometry and the tf to camera_optical instead
    Dataset(
        "huge_loop_go2", HUGE_LOOP_GO2, "lidar", "odom", "stream", tag_camera_frame="camera_optical"
    ),
    Dataset(
        "huge_loop_go2_pointlio",
        HUGE_LOOP_GO2,
        "pointlio_lidar",
        "pointlio_odometry",
        "stream",
        tag_camera_frame="camera_optical",
        odom_pose_of="mid360_link",
    ),
    Dataset(
        "huge_loop_go2_fastlio",
        HUGE_LOOP_GO2,
        "fastlio_lidar",
        "fastlio_odometry",
        "stream",
        tag_camera_frame="camera_optical",
        odom_pose_of="mid360_link",
    ),
    # pointlio was re-run on the pcap on 2026-07-11 and stamped with that day's clock; the offset
    # lines its speed profile up with fastlio's (path RMS 1.9 m over the first 10 min)
    Dataset(
        "huge_loop_pointlio",
        HUGE_LOOP,
        "pointlio_lidar",
        "pointlio_odometry",
        "stream",
        tag_odom="fastlio_odometry",
        clock_offset_s=3216586.885,
        tag_ids_moving=HUGE_LOOP_MOVING_TAGS,
    ),
    Dataset(
        "huge_loop_fastlio",
        HUGE_LOOP,
        "fastlio_lidar",
        "fastlio_odometry",
        "stream",
        tag_ids_moving=HUGE_LOOP_MOVING_TAGS,
    ),
    # held out: Ivan's LFS recording, no sign in git that gsc was ever run on it
    Dataset("mid360_athens_stairs", lidar="pointlio_lidar", odom="pointlio_odometry"),
    # never seen by gsc during its development; their scans were stored undownsampled, so prep.py
    # downsamples them the way Point-LIO publishes them (its 0.2 m filter_size_surf).: the ROSCon booth setup walk and the SF 555 office (two floors),
    # both on the realsense + Mid-360 rig through Point-LIO
    Dataset(
        "roscon",
        lidar="pointlio_lidar",
        odom="pointlio_odometry",
        local="~/.cache/dimos/eval_loop/roscon.db",
        voxel_m=0.2,
    ),
    Dataset(
        "office555",
        lidar="pointlio_lidar",
        odom="pointlio_odometry",
        local="~/.cache/dimos/eval_loop/office555.db",
        voxel_m=0.2,
    ),
    Dataset("go2_hongkong_office", tag_camera="go2"),
    *(Dataset(f"hk_village{i}", tag_camera="go2") for i in range(1, 7)),
]
BY_NAME = {dataset.name: dataset for dataset in DATASETS}
