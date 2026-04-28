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

"""Unitree G1 GR00T whole-body-control blueprint — MuJoCo sim.

Sim counterpart to ``unitree_g1_groot_wbc``.  Same coordinator + tasks,
swap the real DDS adapter for the MuJoCo sim adapter, drop the
real-hw safety ritual (no operator → auto-arm, no ramp, no dry-run),
add the viser viewer + splat-rendered head camera so perception /
memory / agents can consume the same wire format real cameras produce.

Architecture:
    dashboard WASD ──▶ WebsocketVisModule ──▶ LCM /g1/cmd_vel
                                                       │
                              coordinator twist_command ──▶ GrootWBCTask
                                                       │
    ControlCoordinator ──joint_state, odom──▶ LCM
                              │
                              ▼
                  ViserRenderModule (browser at :8082)
                              │
                              ▼
                  SplatCameraModule ──▶ /splat/color_image
                                        /splat/camera_info

Splat + alignment YAML are pulled via the standard Git-LFS data flow:
``get_data("dimos_office")`` triggers a one-time pull of
``data/.lfs/dimos_office.tar.gz`` and decompresses to
``data/dimos_office/``.  YAML schema: ``SplatAlignment`` in
``dimos/visualization/viser/splat.py``.  If the pull fails (no
Git-LFS, offline, etc.) the viser modules are skipped and the rest
of the blueprint runs unchanged.

Usage:
    dimos run unitree-g1-groot-wbc-sim
"""

from __future__ import annotations

import os

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Bool import Bool as DimosBool
from dimos.robot.unitree.g1.blueprints.basic._groot_wbc_common import (
    ARM_DEFAULT_POSE,
    G1_GROOT_KD,
    G1_GROOT_KP,
    g1_arms,
    g1_joints,
    g1_legs_waist,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger
from dimos.visualization.viser import SplatCameraModule, ViserRenderModule
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

logger = setup_logger()

# MJCF the GR00T policies were trained against — torque actuators, the
# subprocess (and now the in-process MujocoEngine) computes PD itself.
_MJCF_PATH = "data/mujoco_sim/g1_gear_wbc.xml"

_g1_coordinator = (
    ControlCoordinator.blueprint(
        tick_rate=500.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[
            HardwareComponent(
                hardware_id="g1",
                hardware_type=HardwareType.WHOLE_BODY,
                joints=g1_joints,
                # In-process engine via MujocoSimModule — adapter and
                # engine share state through SHM keyed on the MJCF path.
                adapter_type="sim_mujoco_g1",
                address=_MJCF_PATH,
                domain_id=0,
                auto_enable=True,
                wb_config=WholeBodyConfig(kp=tuple(G1_GROOT_KP), kd=tuple(G1_GROOT_KD)),
            ),
        ],
        tasks=[
            TaskConfig(
                name="groot_wbc",
                type="groot_wbc",
                joint_names=g1_legs_waist,
                priority=50,
                model_path=os.getenv("GROOT_MODEL_DIR", str(get_data("groot"))),
                hardware_id="g1",
                auto_start=True,
                # Sim convenience: the MuJoCo subprocess holds the MJCF
                # init pose until the first command arrives, so no
                # operator-arm ritual or ramp is needed.  Dry-run off.
                auto_arm=True,
                auto_dry_run=False,
                default_ramp_seconds=0.0,
            ),
            TaskConfig(
                name="servo_arms",
                type="servo",
                joint_names=g1_arms,
                priority=10,
                default_positions=ARM_DEFAULT_POSE,
                auto_start=True,
            ),
        ],
    )
    .transports(
        {
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
            ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
            ("joint_command", JointState): LCMTransport("/g1/joint_command", JointState),
            ("twist_command", Twist): LCMTransport("/g1/cmd_vel", Twist),
            ("activate", DimosBool): LCMTransport("/g1/activate", DimosBool),
            ("dry_run", DimosBool): LCMTransport("/g1/dry_run", DimosBool),
        }
    )
    .global_config(
        # global_config.simulation gates real-vs-sim adapter selection
        # in some upstream blueprints; harmless to set even though the
        # in-process engine + adapter pair don't read robot_model anymore.
        robot_model="unitree_g1",
    )
)

# In-process MuJoCo engine.  Owns the MujocoEngine (single thread, no
# subprocess) and publishes joint state + IMU into SHM for the WB
# adapter, plus camera/lidar/pointcloud streams for downstream consumers.
# The G1 GR00T MJCF has no head_camera so we point camera_name at the
# torso lidar instead — a separate splat camera handles RGB perception.
_g1_engine = MujocoSimModule.blueprint(
    address=_MJCF_PATH,
    headless=False,
    dof=29,
    enable_depth=True,
    enable_pointcloud=True,
    pointcloud_fps=2.0,
    camera_name="lidar_front_camera",
    # G1 GR00T MJCF references meshes by bare filename (menagerie convention);
    # without the legacy asset injection MjModel.from_xml_path can't find them.
    inject_legacy_assets=True,
)

# WASD teleop dashboard at http://localhost:7779/.
_g1_ws_vis = WebsocketVisModule.blueprint().transports(
    {
        ("cmd_vel", Twist): LCMTransport("/g1/cmd_vel", Twist),
        ("activate", DimosBool): LCMTransport("/g1/activate", DimosBool),
        ("dry_run", DimosBool): LCMTransport("/g1/dry_run", DimosBool),
    },
)

# Splat viewer + splat-rendered head camera, gated on the splat asset
# being pullable.  When the LFS pull fails the modules are silently
# skipped and the rest of the sim runs as a flat-floor MuJoCo viewer.
_viser_modules: tuple = ()
try:
    _splat_dir = get_data("dimos_office")
    _splat_path = _splat_dir / "dimos_office.ply"
    _alignment_yaml = _splat_dir / "dimos_office.yaml"
except Exception as e:
    logger.warning(f"Splat asset unavailable: {e}; viser viewer + splat camera disabled")
    _splat_path = None
if _splat_path is not None and _splat_path.exists():
    _g1_viser = ViserRenderModule.blueprint(
        splat_path=str(_splat_path),
        mjcf_path=_MJCF_PATH,
        alignment_yaml=str(_alignment_yaml) if _alignment_yaml.exists() else None,
        port=8082,
    ).transports(
        {
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
            ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
        },
    )
    _g1_splat_cam = SplatCameraModule.blueprint(
        splat_path=str(_splat_path),
        mjcf_path=_MJCF_PATH,
        alignment_yaml=str(_alignment_yaml) if _alignment_yaml.exists() else None,
        render_hz=10.0,
    ).transports(
        {
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
            ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
            ("color_image", Image): LCMTransport("/splat/color_image", Image),
            ("camera_info", CameraInfo): LCMTransport("/splat/camera_info", CameraInfo),
        },
    )
    _viser_modules = (_g1_viser, _g1_splat_cam)

unitree_g1_groot_wbc_sim = autoconnect(_g1_coordinator, _g1_engine, _g1_ws_vis, *_viser_modules)

__all__ = ["unitree_g1_groot_wbc_sim"]
