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

"""Opt-in dual-Metal PPO policy station over an ideal, no-CAN connection.

This is the deployment composition for the accepted learned controller. It
never constructs :class:`DualMetalConnection`, so the fake launch path cannot
open a CAN bus even if a hardware address is configured elsewhere. The two
jaws keep their own gripper tasks; the policy claims the 12 arm joints only.

The learned artifacts are not committed. Point the blueprint at a directory
containing ``metal_arm_policy.manifest.json``, the exported ``.onnx`` and its
``.normalizer.npz`` through ``--model-dir`` / ``--manifest-path`` or
``DIMOS_METAL_POLICY_MODEL_DIR`` / ``DIMOS_METAL_POLICY_MANIFEST``.
"""

from __future__ import annotations

import os
from pathlib import Path

from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.robot.manipulators.damiao.damiao_blueprint import DamiaoTeleopCoordinator
from dimos.robot.manipulators.dual_makermods_metal.config import (
    ALL_ARM_JOINTS,
    FINGER_JOINTS,
    SIDES,
    dual_metal_model_config,
)
from dimos.robot.manipulators.dual_makermods_metal.policy.contract import HOME

#: The trained station mounts differ from the repo's example bench layout.
POLICY_MODEL = dual_metal_model_config(
    left_xyz=(0.0, -0.345, 0.008),
    right_xyz=(0.0, 0.345, 0.008),
    robot="arm",
)

DEFAULT_MODEL_DIR = Path(
    os.environ.get(
        "DIMOS_METAL_POLICY_MODEL_DIR",
        str(Path("~/.local/share/dimos/metal_policy").expanduser()),
    )
)
DEFAULT_MANIFEST_PATH = Path(
    os.environ.get(
        "DIMOS_METAL_POLICY_MANIFEST",
        str(DEFAULT_MODEL_DIR / "metal_arm_policy.manifest.json"),
    )
)


def _initial_positions() -> dict[str, float]:
    """The trained home pose with both jaws closed; the task stays inert."""
    positions = dict(zip((f"arm/{joint}" for joint in ALL_ARM_JOINTS), HOME, strict=True))
    for side in SIDES:
        positions[f"arm/{FINGER_JOINTS[side][0]}"] = 0.0
    return positions


def dual_metal_policy_blueprint(
    *,
    model_dir: str | Path | None = None,
    manifest_path: str | Path | None = None,
    teleop: bool = False,
    dry_run: bool = False,
) -> Blueprint:
    """Compose the policy task, both jaws and an ideal joint connection.

    Args:
        model_dir: Directory holding the exported ONNX actor, normalizer and hash.
        manifest_path: Path to the policy manifest that pins them.
        teleop: Also accept the WebXR deadman and controller streams.
        dry_run: Compute actions without emitting joint commands.
    """
    resolved_model_dir = Path(model_dir) if model_dir is not None else DEFAULT_MODEL_DIR
    resolved_manifest = Path(manifest_path) if manifest_path is not None else DEFAULT_MANIFEST_PATH
    tasks = [
        TaskConfig(
            name="metal_arm_policy",
            type="metal_arm_policy",
            joint_names=[f"arm/{joint}" for joint in ALL_ARM_JOINTS],
            priority=40,
            params={
                "model_dir": str(resolved_model_dir),
                "manifest_path": str(resolved_manifest),
                "feedback_timeout_s": 0.2,
                "intent_timeout_s": 0.5,
                "goal_timeout_s": 0.25 if teleop else None,
                "teleop": teleop,
                "dry_run": dry_run,
            },
        ),
        *[
            TaskConfig(
                name=f"{side}_arm_gripper",
                type="gripper",
                joint_names=[f"arm/{FINGER_JOINTS[side][0]}"],
                priority=20,
                params={"open_at": "lower"},
                stream_bind={"gripper_command": f"{side}_gripper_command"},
            )
            for side in SIDES
        ],
    ]
    return autoconnect(
        DamiaoTeleopCoordinator.blueprint(
            instance_name="ControlCoordinator",
            model=POLICY_MODEL.model,
            tasks=tasks,
        ),
        MockJointConnection.blueprint(
            source="arm",
            model=POLICY_MODEL.model,
            initial_positions=_initial_positions(),
        ),
    ).remappings(
        [
            (MockJointConnection, "position_command", "arm_position_command"),
            (DamiaoTeleopCoordinator, "arm_joint_state", "joint_state"),
        ]
    )


#: Registered, opt-in fake/no-CAN station. Inert until its task is engaged.
dual_makermods_metal_policy_fake = dual_metal_policy_blueprint().remappings([])
