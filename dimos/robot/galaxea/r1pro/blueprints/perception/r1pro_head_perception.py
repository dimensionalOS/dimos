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

"""Camera-frame perception only. No manipulation, motor driver or base TF."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.galaxea.r1pro.blueprints.perception.r1pro_head_stereo import r1pro_head_stereo
from dimos.robot.galaxea.r1pro.head_perception_preflight import head_perception_ready
from dimos.visualization.perception.viser_module import PerceptionViser

r1pro_head_perception = (
    autoconnect(
        r1pro_head_stereo,
        ObjectSceneRegistrationModule.blueprint(
            target_frame="r1pro_head_left_rectified_optical",
            detector_backend="moondream",
            segmentation_backend="edgetam",
            detect_on_request=True,
            min_detections_for_permanent=1,
            snapshot_only=True,
            max_input_age_s=0.5,
            max_result_age_s=15.0,
            max_pair_skew_s=0.002,
            require_segmentation=True,
            min_valid_depth_fraction=0.3,
            min_valid_depth_pixels=100,
            use_aabb=True,
            max_obstacle_width=0,
        ),
        PerceptionViser.blueprint(),
    )
    .remappings(
        [
            (ObjectSceneRegistrationModule, "color_image", "head_rgbd_color"),
            (ObjectSceneRegistrationModule, "depth_image", "head_rgbd_depth"),
            (ObjectSceneRegistrationModule, "camera_info", "head_rgbd_info"),
        ]
    )
    .global_config(n_workers=5, viewer="none", transport="zenoh")
    .requirements(head_perception_ready)
)
