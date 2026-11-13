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

"""One distributed hardware run: laptop leaders/model and R1 Pro control/cameras.

Copy the installed vendor R1_PRO directory into
CACHE_DIR/r1pro-hardware/mobiman/urdf/R1_PRO on the operator machine. Geometry, frames, axes and SI inertias stay exactly as installed.
Only command joint names and the gripper opening-to-finger relation are adapted:
total opening in metres drives the two vendor +/-50 mm fingers at +/- one half.
The added opening links are invisible measurement frames, not physical bodies.
"""

from dimos.constants import CACHE_DIR
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import ZenohTransport
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_LATEST_WINS
from dimos.robot.assets.model import JointDescription, RobotModel
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_joylo_inputs import _operator_blueprint
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_joylo_real import make_joylo_real
from dimos.robot.galaxea.r1pro.collection import r1pro_collection
from dimos.robot.galaxea.r1pro.head_cameras import HeadCameraInfo, HeadLeftCamera, HeadRightCamera
from dimos.robot.galaxea.r1pro.joylo import UPPER
from dimos.robot.galaxea.r1pro.spatial_sensors import R1SpatialSensors
from dimos.robot.usd_manager import UsdManager

_vendor = CACHE_DIR / "r1pro-hardware/mobiman"
_model = RobotModel.from_file(
    _vendor / "urdf/R1_PRO/urdf/r1_pro.urdf",
    package_paths={"mobiman": _vendor},
).with_renamed_joints({name.removeprefix("r1pro/"): name for name in UPPER})
for _side in ("left", "right"):
    _opening = f"r1pro/{_side}_gripper"
    _model = _model.with_added_joints(
        JointDescription(
            name=_opening,
            type="prismatic",
            parent_link=f"{_side}_gripper_link",
            child_link=f"{_side}_gripper_opening",
            lower=0.0,
            upper=0.1,
            velocity=0.5,
            effort=100.0,
        )
    )
    for _index, _multiplier in ((1, 0.5), (2, -0.5)):
        _model = _model.with_mimic_joint(
            f"{_side}_gripper_finger_joint{_index}",
            _opening,
            _multiplier,
        )

_vision = (
    autoconnect(
        HeadLeftCamera.blueprint(jpeg_quality=70, build_command=None),
        HeadRightCamera.blueprint(jpeg_quality=70, build_command=None),
        HeadCameraInfo.blueprint(),
        R1SpatialSensors.blueprint(),
    )
    .remappings(
        [
            (HeadLeftCamera, "jpeg_out", "head_left_color"),
            (HeadRightCamera, "jpeg_out", "head_right_color"),
            (HeadCameraInfo, "left_info", "head_left_color_info"),
            (HeadCameraInfo, "right_info", "head_right_color_info"),
        ]
    )
    .transports(
        {
            (name, CompressedImage): ZenohTransport.spec(
                f"dimos/{name}", CompressedImage, qos=QOS_LATEST_WINS
            )
            for name in (
                "head_left_color",
                "head_right_color",
                "color_image",
                "left_wrist",
                "right_wrist",
            )
        }
    )
)

r1pro_openarm_joylo_real = autoconnect(
    autoconnect(make_joylo_real(publish_odom_tf=True, enable_wrist_color=False), _vision).hosted(
        tags={"r1pro": True}
    ),
    _operator_blueprint().hosted(local=True),
    UsdManager.blueprint(model=_model, tf_source="joints", unchanged_tf_hz=15).hosted(local=True),
).global_config(viewer="none", transport="zenoh", n_workers=7)


# The recorder lives beside control and cameras, so laptop/network loss cannot
# remove the robot-side evidence. It starts first and shuts down last.
r1pro_openarm_joylo_collect = autoconnect(
    autoconnect(
        r1pro_collection(), make_joylo_real(publish_odom_tf=True, enable_wrist_color=False), _vision
    ).hosted(tags={"r1pro": True}),
    _operator_blueprint().hosted(local=True),
    UsdManager.blueprint(model=_model, tf_source="joints", unchanged_tf_hz=15).hosted(local=True),
).global_config(viewer="none", transport="zenoh", n_workers=9)
