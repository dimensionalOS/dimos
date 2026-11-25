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
from dimos.core.coordination.blueprints import Blueprint, MetaBlueprint, autoconnect
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.robot.assets.model import JointDescription, RobotModel
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_joylo_inputs import (
    _camera_transports,
    _operator_blueprint,
    _webxr_blueprint,
    _webxr_joystick_blueprint,
)
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_joylo_real import make_joylo_real
from dimos.robot.galaxea.r1pro.collection import HARDWARE_WRIST_SHAPE, r1pro_collection
from dimos.robot.galaxea.r1pro.external_camera import R1ProExternalCamera
from dimos.robot.galaxea.r1pro.head_cameras import (
    HEAD_JPEG_QUALITY,
    HeadCameraInfo,
    HeadLeftCamera,
    HeadRightCamera,
)
from dimos.robot.galaxea.r1pro.joylo import UPPER
from dimos.robot.galaxea.r1pro.spatial_sensors import R1SpatialSensors
from dimos.robot.usd_manager import UsdManager

# Arm speed at start and the most the operator may raise it to, in rad/s.
_ARM_RATE, _ARM_CEILING = 1.5, 2.0
# Speed cap sent to the robot's own joint tracker, in rad/s.
_TRACKING_SPEED = 2.0
# Low-pass cutoff on the leader readings, in Hz, to smooth hand tremor.
_LEADER_FILTER_CUTOFF_HZ = 10.0

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
        HeadLeftCamera.blueprint(jpeg_quality=HEAD_JPEG_QUALITY, build_command=None),
        HeadRightCamera.blueprint(jpeg_quality=HEAD_JPEG_QUALITY, build_command=None),
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
    .transports(_camera_transports)
)


def _external_camera() -> Blueprint:
    """The operator laptop's camera filming the robot, as external_color JPEG."""
    return (
        R1ProExternalCamera.blueprint()
        .remappings([(R1ProExternalCamera, "color", "external_color")])
        .transports(_camera_transports)
    )


def make_joylo_desktop(*, recording: bool = False) -> MetaBlueprint:
    """Hardware teleop: control and cameras on the robot, leaders and Joy-Cons here.

    Args:
        recording: Add the recorder and episode monitor on the robot, and the
            laptop's external camera. The run then needs --recorder.recording
            (a new folder on the robot) and --episodes.task.
    """
    collection = r1pro_collection(
        wrist_shape=HARDWARE_WRIST_SHAPE, extra_inputs={"external_color": CompressedImage}
    )
    operator = autoconnect(
        _operator_blueprint(filter_cutoff_hz=_LEADER_FILTER_CUTOFF_HZ),
        *([_external_camera()] if recording else []),
    )
    return autoconnect(
        autoconnect(
            # When requested, the recorder starts first and shuts down last.
            *([collection] if recording else []),
            # Operator video uses head_*_color / wrist_*_color independently of
            # the paired, rectified RGB-D streams used by the mesh and recorder.
            make_joylo_real(
                publish_odom_tf=True,
                enable_wrist_color=True,
                webxr=True,
                arm_rate=_ARM_RATE,
                arm_ceiling=_ARM_CEILING,
                tracking_speed=_TRACKING_SPEED,
            ),
            # Retarget beside feedback and IK, where headset poses arrive.
            _webxr_blueprint(),
            _webxr_joystick_blueprint(),
            _vision,
        ).hosted(tags={"r1pro": True}),
        operator.hosted(local=True),
        UsdManager.blueprint(model=_model, unchanged_tf_hz=15).hosted(local=True),
    ).global_config(viewer="none", transport="zenoh", n_workers=11)


# autoconnect(...) is what the registry generator recognises as a runnable blueprint.
r1pro_openarm_joylo_real = autoconnect(make_joylo_desktop())
r1pro_openarm_joylo_collect = autoconnect(make_joylo_desktop(recording=True))
