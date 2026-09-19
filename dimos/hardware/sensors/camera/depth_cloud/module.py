# Copyright 2025-2026 Dimensional Inc.
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

"""Unproject a depth image into a PointCloud2 using its CameraInfo intrinsics.

Geometry only — :meth:`PointCloud2.from_rgbd` covers the coloured case but needs a
size-matched colour frame alongside the depth one. Mapping consumers only want the
points, so this skips the colour stream entirely.

Points come out in the optical frame the intrinsics describe (x right, y down,
z forward), tagged with the ``CameraInfo``'s ``frame_id`` so a downstream
consumer resolving it through tf places them correctly. The depth image's own
``frame_id`` is the fallback: a vendor driver often stamps depth with a frame
nobody publishes a transform for, while the intrinsics can be given the link
name the robot actually puts on tf.

The unprojection itself lives in ``rust/src/unproject.rs``. A 1280x720 frame is
920k pixels to range-gate and divide, per frame, on the critical path of a map
update.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from pydantic import Field, field_validator

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class DepthCloudConfig(NativeModuleConfig):
    # Built through the workspace, like the other Rust native modules.
    cwd: str | None = "rust"
    executable: str = str(DIMOS_PROJECT_ROOT / "target" / "release" / "depth_cloud")
    build_command: str | None = "cargo build --release"
    stdin_config: bool = True
    base_fields: frozenset[str] = frozenset({"frame_id"})

    # Keep every Nth pixel on each axis. A 1280x720 depth frame is 920k points;
    # at 4 that is 57k, which is the range the voxel map ray-caster is sized for.
    decimation: int = Field(default=4, ge=1)
    # Depth below this is the sensor's blind zone; above it, stereo range error
    # grows past the voxel size and smears obstacles.
    min_range_m: float = Field(default=0.2, ge=0.0)
    max_range_m: float = Field(default=6.0, gt=0.0)
    # Multiplier onto metres. uint16 depth is millimetres, so 0.001; float32
    # depth is already metres, so the Rust side ignores it for float input.
    depth_scale: float = Field(default=0.001, gt=0.0)
    # Empty rather than None so the native config takes a plain string. Falsy
    # either way, so Module.frame_id still falls back to the class name.
    frame_id: str | None = ""


class DepthCloud(NativeModule):
    """Depth image + intrinsics -> PointCloud2 in the camera's optical frame."""

    config: DepthCloudConfig

    depth: In[Image]
    camera_info: In[CameraInfo]
    cloud: Out[PointCloud2]


if TYPE_CHECKING:
    DepthCloud()


class StereoCloudConfig(NativeModuleConfig):
    # Same crate as DepthCloud; the two binaries land side by side in target/.
    cwd: str | None = "rust"
    executable: str = str(DIMOS_PROJECT_ROOT / "target" / "release" / "stereo_cloud")
    build_command: str | None = "cargo build --release"
    stdin_config: bool = True
    # The height bounds are listed so a None crosses to Rust as an explicit
    # null: the native config forbids absent keys, and "no bound" has to be
    # said rather than left out.
    base_fields: frozenset[str] = frozenset({"frame_id", "min_height_m", "max_height_m"})

    # Distance between the two head cameras. The default is the R1 Pro's URDF
    # geometry: camera_head_left_joint sits at y +0.059919 and the right one at
    # y -0.060276, with identical xyz and rpy otherwise. That is a nominal
    # design figure rather than a measured one, and depth scales linearly with
    # it, so this is the knob to turn when the cloud sits consistently nearer or
    # further than the lidar.
    baseline_m: float = Field(default=0.120195, gt=0.0)
    # 1920x1536 per eye is far more than matching can afford on the robot's Orin
    # while it also runs the whole driver stack, and navigation does not need
    # it. At 4 the search still resolves obstacles to a few centimetres.
    downscale: int = Field(default=4, ge=1, le=16)
    # With downscale 4 and the R1's geometry this covers roughly 0.3 m to 7.5 m.
    disparity_range: int = Field(default=96, ge=8, le=512)
    p1: int = Field(default=8, ge=0, le=255)
    p2: int = Field(default=120, ge=1, le=4096)
    # Raising this throws away more of the textureless regions where stereo
    # invents surfaces — the failure that matters most for navigation, since an
    # invented surface is an obstacle that is not there.
    uniqueness: float = Field(default=0.10, ge=0.0, le=1.0)
    max_lr_difference: float = Field(default=1.5)
    # De-noising. Stereo invents small isolated patches on textureless surfaces
    # that survive the occlusion and uniqueness checks; to a voxel map a
    # floating patch is an obstacle in empty space, so the robot refuses to
    # drive through nothing. ~0.2% of a 480x384 frame: big enough to clear those
    # blobs, small enough to keep a chair leg. 0 or 1 turns it off.
    min_region: int = Field(default=350, ge=0)
    speckle_max_step: float = Field(default=1.5, ge=0.0)
    # Aggregate along the four diagonals as well as the four axes. Doubles the
    # matcher's cost, and buys support where a surface has nothing of its own to
    # match on -- a floor being exactly that, and the one navigation needs.
    diagonal_paths: bool = False
    # Below this, metres-per-pixel grows without bound and noise plants
    # obstacles across the whole map.
    min_disparity_px: float = Field(default=1.0, ge=0.0)
    decimation: int = Field(default=2, ge=1)
    min_range_m: float = Field(default=0.3, ge=0.0)
    max_range_m: float = Field(default=7.0, gt=0.0)
    frame_id: str | None = ""
    # The head cameras free-run rather than being hardware synchronised, so
    # some skew is expected; too much smears the disparity of anything moving.
    max_pair_skew_s: float = Field(default=0.05, ge=0.0, le=1.0)
    # How the right eye is aimed relative to the left. A robot publishing two
    # *monocular* CameraInfos has said nothing about this, and zero assumes the
    # eyes are perfectly parallel, which no real rig is: a relative yaw offsets
    # every disparity by the same amount, and a relative pitch puts the two
    # pictures on different rows where a matcher searching along a row finds
    # nothing. Fit them with dimos.robot.galaxea.r1pro.fit_rectification; they
    # are properties of the rig and change only when something is unbolted.
    right_roll_rad: float = Field(default=0.0, ge=-0.2, le=0.2)
    right_pitch_rad: float = Field(default=0.0, ge=-0.2, le=0.2)
    right_yaw_rad: float = Field(default=0.0, ge=-0.2, le=0.2)
    # The depth denoise chain, run on the metric depth before it is
    # unprojected: filters joined by "+", each "name" or "name:arg" (plane
    # takes "plane:radius:weight"); see rust/src/denoise.rs for what each one
    # does. "none" turns it off. The default is what a week of scoring chains
    # against the lidar map of the same scene settled on: the median takes the
    # thorns off, the plane fit flattens the floor and walls the matcher had
    # rippled, and the fill then has honest neighbours to close small holes.
    denoise: str = Field(default="median:8+plane:16:1+fill:8")
    # Keep only cloud points whose height in the robot base frame lies inside
    # [min_height_m, max_height_m]; None on a side is no bound there, None on
    # both is no gate. The depth image is not gated -- it is the camera's
    # answer, the cloud is what the map may use.
    min_height_m: float | None = None
    max_height_m: float | None = None
    # The pose of the camera's OPTICAL frame (x right, y down, z forward) in
    # the robot base frame, so a camera point p sits at R(rpy) @ p + xyz in
    # base and its height is that vector's z. rpy is roll about x, then pitch
    # about y, then yaw about z, all about the fixed base axes (extrinsic XYZ,
    # the same as intrinsic Z-Y'-X''): Quaternion.from_euler's convention, and
    # a URDF's. Only read when a height bound is set.
    base_from_camera_xyz_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    base_from_camera_rpy_rad: tuple[float, float, float] = (0.0, 0.0, 0.0)

    @field_validator("denoise")
    @classmethod
    def _denoise_parses(cls, chain: str) -> str:
        # Mirrors Chain::parse in rust/src/denoise.rs, so a typo fails at
        # blueprint time with the token named, not on the robot at start.
        if chain in ("", "none"):
            return chain
        for token in chain.split("+"):
            name, _, argument = token.partition(":")
            if name not in _DENOISE_FILTERS:
                raise ValueError(
                    f"denoise={token!r}: {name!r} is not one of {sorted(_DENOISE_FILTERS)}"
                )
            numbers = argument.split(":") if argument else []
            if len(numbers) > (2 if name == "plane" else 1):
                raise ValueError(f"denoise={token!r}: too many arguments for {name!r}")
            for number in numbers:
                try:
                    float(number)
                except ValueError:
                    raise ValueError(f"denoise={token!r}: {number!r} is not a number") from None
        return chain


# The filters rust/src/denoise.rs knows, by their chain names.
_DENOISE_FILTERS = frozenset(
    {"none", "median", "speckle", "bilateral", "coarse", "mean", "steep", "plane", "fill"}
)


class StereoCloud(NativeModule):
    """Two RGB head frames -> depth map + PointCloud2, by semi-global matching.

    :class:`DepthCloud` is the right module when something upstream already
    produces depth, which is true of the wrist RealSenses. It is not true of the
    R1 Pro's head: Galaxea's own spec calls it "1x pure binocular RGB camera"
    and the robot publishes no head depth topic, so the matching happens here.

    Both modules share ``rust/src/unproject.rs`` for the depth -> cloud half.

    ``depth`` is published alongside the cloud because a wrong cloud and a wrong
    calibration look identical once the points are in 3D; the depth map is where
    a bad rectification is actually visible.
    """

    config: StereoCloudConfig

    left: In[CompressedImage]
    right: In[CompressedImage]
    left_info: In[CameraInfo]
    right_info: In[CameraInfo]
    cloud: Out[PointCloud2]
    depth: Out[Image]


if TYPE_CHECKING:
    StereoCloud()
