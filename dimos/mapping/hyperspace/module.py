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

from typing import TYPE_CHECKING

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

# A colour frame pairs with the depth frame nearest it in time; a RealSense
# publishes both within a frame period of each other.
DEPTH_MAX_DT_S = 0.05


class HyperspaceConfig(NativeModuleConfig):
    cwd: str | None = "rust"
    # The crate is a workspace member, so cargo builds into the repo-root target dir.
    executable: str = str(DIMOS_PROJECT_ROOT / "target" / "release" / "hyperspace")
    build_command: str | None = "cargo build --release"
    stdin_config: bool = True

    # Voxel edge length of the answer raster, in meters.
    voxel_size: float = 0.10
    # Frame queries are answered in unless a request names another.
    world_frame: str = "odom"
    # Frame the quality gate measures camera motion against.
    motion_reference_frame: str = "odom"

    # SigLIP2 snapshot directory (config.json, tokenizer.json, *.safetensors).
    # Empty runs a stub embedder: the pipeline works end to end, the answers do
    # not mean anything. Needs a binary built with the `siglip` cargo feature.
    model_dir: str = ""
    # depth2depth weights directory (dinov2_vits14 + da2_head_vits safetensors).
    # Empty uses the raw sensor depth. Needs the `depth2depth` cargo feature.
    depth_weights_dir: str = ""
    # Run the models on CUDA. Needs the `cuda` cargo feature.
    cuda: bool = False

    # Frames held before the middle one is judged, against the 5 before and 5
    # after it. Odd; 11 at 5 Hz is about 2 seconds.
    buffer_len: int = 11
    # Mean per-patch (1 - cosine) against the last kept keyframe to keep one.
    novelty_threshold: float = 0.05
    # Single-patch change that keeps a frame even when the view barely moved,
    # which is how a new object entering a static view is caught. Negative disables.
    patch_novelty_threshold: float = 0.5
    # Motion-blur gate: drop frames whose camera turns faster than this (rad/s).
    # Negative disables it, which is right for a global-shutter camera.
    max_angular_velocity: float = 1.5
    # Drop frames whose camera moves faster than this (m/s). Negative disables.
    max_linear_velocity: float = -1.0
    # Drop frames with more than this fraction of near-black pixels. Negative disables.
    max_dark_fraction: float = 0.6
    # Drop frames with more than this fraction of near-white pixels. Negative disables.
    max_bright_fraction: float = -1.0
    # Never keep two keyframes closer together than this (s). Negative disables.
    min_keyframe_interval: float = 0.1

    # Depth readings beyond this many meters are treated as holes. RealSense
    # frames carry 65535 mm "no reading" sentinels and occasional 20-40 m glitches.
    max_depth_m: float = 10.0
    # A colour frame pairs with the depth frame within this many seconds of it.
    depth_max_dt: float = DEPTH_MAX_DT_S
    # Depth frames buffered per sensor while waiting for their colour frame.
    depth_history: int = 64
    # Stride of the depth thumbnail kept per keyframe and rendered as scene_map.
    # Zero keeps none, and then scene_map stays empty.
    depth_thumbnail_stride: int = 4

    # Patch score (query minus best background prompt) needed to be "hot".
    hot_threshold: float = 0.02
    # Ceiling on hot patches per query; the highest scoring ones win.
    max_hot_patches: int = 6000
    # Each hot patch's pyramid is cut at these fractions of its fused depth.
    cap_near: float = 0.9
    cap_far: float = 1.1
    # Prompts contrasted against the query, comma separated. Empty uses the
    # crate's indoor defaults.
    background_prompts: str = ""

    # Publish scene_map every Nth kept keyframe. Zero never publishes it.
    scene_emit_every: int = 10
    # Depth samples a voxel needs before it appears in scene_map.
    scene_min_samples: int = 3


class Hyperspace(NativeModule):
    """Open-vocabulary 3D querying: ask for "a chair", get the voxels back.

    Colour, depth and tf go in continuously. A query arrives on ``query`` as
    JSON (``{"id": 7, "text": "a chair"}``) or as bare text, and the answer
    comes back on ``query_result`` as a scored voxel cloud whose ``header.seq``
    is the request's id. That id is what pairs an answer with its request, so
    querying is plain pub/sub with no RPC and no blocking call.
    """

    config: HyperspaceConfig

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    depth_camera_info: In[CameraInfo]
    tf: In[TFMessage]
    # {"id": 7, "text": "a chair", "frame": "odom"}, or bare text for id 0.
    query: In[String]

    # Voxel centers with a `score` field, `header.seq` = the id that asked.
    query_result: Out[PointCloud2]
    # Occupied voxels from the kept keyframes' depth, for context in a viewer.
    scene_map: Out[PointCloud2]


if TYPE_CHECKING:
    Hyperspace()
