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

"""YOLOE + Go2 keyboard teleop blueprint.

Combines YOLOE open-vocabulary detection with keyboard-controlled movement so
you can manually drive the Go2 (in simulation or on real hardware) while
verifying YOLOE detection quality in the Rerun viewer.

Usage:
    # Simulation (recommended for capability checking)
    dimos --simulation run yoloe-keyboard-teleop

    # Replay (detection-only, robot does not move)
    dimos --replay run yoloe-keyboard-teleop

    # Real Go2
    dimos --robot-ip 192.168.123.161 --rerun-open native run yoloe-keyboard-teleop

Keyboard controls (pygame window must have focus):
    W / S       — forward / backward
    A / D       — rotate left / right
    Shift       — speed boost  (2×)
    Ctrl        — slow mode    (0.5×)
    Space       — publish zero Twist (stop)
    Esc / Q     — quit keyboard window
"""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.custom.modules.yoloe_tracking_module import (
    YoloeTrackingModule,
    _require_yoloe_lrpc_model,
)
from dimos.robot.custom.visualization.detection2d_overlay import yoloe_overlay
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import (
    rerun_config as go2_rerun_config,
    unitree_go2_basic,
)
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.visualization.vis_module import vis_module

_YOLOE_DETECTIONS_TOPIC = "/color_image/yoloe_detections"
_YOLOE_DETECTIONS_ENTITY = "world/color_image/yoloe_detections"


def _yoloe_keyboard_rerun_blueprint() -> Any:
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(
                origin="world/color_image",
                contents=["world/color_image/**"],
                name="Camera",
            ),
            rrb.Spatial3DView(
                origin="world",
                contents=[
                    "world/**",
                    f"-{_YOLOE_DETECTIONS_ENTITY}",  # 在 3D 视图中隐藏 YOLOE 2D bbox overlay
                ],
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.5),
                ),
                overrides={
                    "world/lidar": rrb.EntityBehavior(visible=False),
                },
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


_yoloe_keyboard_rerun_config = {
    **go2_rerun_config,
    "blueprint": _yoloe_keyboard_rerun_blueprint,
    "visual_override": {
        **go2_rerun_config["visual_override"],
        _YOLOE_DETECTIONS_ENTITY: yoloe_overlay,  # 对 YOLOE detections 做青色 Camera overlay
    },
}

_yoloe_keyboard_vis = vis_module(
    viewer_backend=global_config.viewer,
    rerun_config=_yoloe_keyboard_rerun_config,
)

# 在 simulation 模式下 MuJoCo 的 locomotion policy 已占用 CoreMLExecutionProvider，
# 强制 YOLOE 使用 CPU 避免 Metal/MPS 资源竞争导致 worker 崩溃。
# 在 replay / 真机模式下保持 device=None（自动选择 GPU）。
# _yoloe_device = "cpu" if global_config.simulation else None
_yoloe_device = None

yoloe_keyboard_teleop = (
    autoconnect(
        unitree_go2_basic,
        _yoloe_keyboard_vis,
        YoloeTrackingModule.blueprint(device=_yoloe_device),
        # MovementManager 作为 velocity hub：
        #   tele_cmd_vel (来自 KeyboardTeleop) → MovementManager → cmd_vel → GO2Connection
        # 与 unitree_go2 的控制路径保持一致，simulation 模式下 MujocoConnection 期望此路径。
        MovementManager.blueprint(),
        # publish_only_when_active=True: 松开按键后只发一次零 Twist 再静默，
        # 不持续刷 cmd_vel，不干扰 MovementManager 的 nav/tele 优先级逻辑。
        KeyboardTeleop.blueprint(publish_only_when_active=True),
    )
    .global_config(
        n_workers=8,  # basic(4) + YoloeTracking + MovementManager + KeyboardTeleop + vis
        robot_model="unitree_go2",
    )
    .remappings(
        [
            # KeyboardTeleop 输出 cmd_vel，需重命名为 tele_cmd_vel 才能连接到 MovementManager
            (KeyboardTeleop, "cmd_vel", "tele_cmd_vel"),
        ]
    )
    .transports(
        {
            ("detections", Detection2DArray): LCMTransport(
                _YOLOE_DETECTIONS_TOPIC,
                Detection2DArray,
            ),
        }
    )
    .requirements(
        _require_yoloe_lrpc_model,
    )
)

__all__ = ["yoloe_keyboard_teleop"]
