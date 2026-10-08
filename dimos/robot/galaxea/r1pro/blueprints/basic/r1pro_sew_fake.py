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

"""R1 upper-body WebXR experiment. Always in-memory hardware, never ROS."""

from __future__ import annotations

from pathlib import Path
import threading
from typing import Any, Literal, Protocol

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinatorConfig, TaskConfig
from dimos.control.tasks.sew_teleop_task.task import SewTeleopTask
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.galaxea.r1pro.joints import PASSIVE_JOINTS
from dimos.robot.galaxea.r1pro.sew_model import R1SewModel
from dimos.spec.utils import Spec
from dimos.teleop.webxr.body_tracking import BodyTrackingSnapshot
from dimos.teleop.webxr.controller_types import WebXRControllerState
from dimos.teleop.webxr.extensions import ArmTeleopModule


class R1FakeConfig(ControlCoordinatorConfig):
    urdf_path: str = ""
    solver_backend: Literal["pink", "sew"] = "pink"


class R1FakeCoordinator(TeleopControlCoordinator):
    config: R1FakeConfig
    body_tracking: In[BodyTrackingSnapshot]

    def _setup_from_config(self) -> None:
        if not self.config.urdf_path:
            raise ValueError("Pass --urdf-path pointing to the pinned R1 Pro robot description")
        path = Path(self.config.urdf_path).resolve()
        model = R1SewModel(path)
        names = [f"r1pro/{n}" for n in model.names]
        home = [
            0.3,
            -0.6,
            -0.3,
            0.0,
            0.0,
            0.3,
            0.0,
            -0.6,
            0.0,
            0.0,
            0.0,
            0.0,
            -0.3,
            0.0,
            -0.6,
            0.0,
            0.0,
            0.0,
        ]
        self.config.hardware = [
            HardwareComponent(
                hardware_id="r1pro",
                hardware_type=HardwareType.WHOLE_BODY,
                joints=names,
                adapter_type="mock_whole_body",
                adapter_kwargs={"initial_positions": home},
            )
        ]
        if self.config.solver_backend == "sew":
            task = TaskConfig(
                name="upper_body",
                type="sew_teleop",
                joint_names=names,
                params={"urdf_path": str(path)},
            )
        else:
            robot = (
                RobotModel.from_file(path)
                .with_fixed_joints(*PASSIVE_JOINTS)
                .with_default_joint_acceleration_limit(2.0)
                .with_renamed_joints({n: f"r1pro/{n}" for n in model.names})
            )
            config = RobotModelConfig(model=robot, joint_names=names, base_link="base_link")
            task = TaskConfig(
                name="upper_body",
                type="teleop_ik",
                joint_names=names[4:],
                params={
                    "robot_model": config,
                    "bindings": [
                        {"hand": s, "target_frame": f"{s}_gripper_link"} for s in ("left", "right")
                    ],
                },
            )
        self.config.tasks = [task]
        super()._setup_from_config()

    @rpc
    def sew_status(self) -> dict[str, Any]:
        task = self.get_task("upper_body")
        if isinstance(task, SewTeleopTask):
            with task.lock:
                return dict(task.status)
        return {
            "state": "PINK" if task is not None else "NOT_STARTED",
            "reason": "Relative hand pose mode; torso held",
        }


class _StatusSpec(Spec, Protocol):
    def sew_status(self) -> dict[str, Any]: ...


class R1FakeWebXR(ArmTeleopModule):
    _coordinator: _StatusSpec

    def _publish_button_state(
        self, left: WebXRControllerState | None, right: WebXRControllerState | None
    ) -> None:
        # Missing controller packets must expire at the task, never masquerade
        # as the physical release required to clear its rearm latch.
        if left is not None and right is not None:
            super()._publish_button_state(left, right)

    def _webxr_client_config(self) -> dict[str, Any]:
        result = super()._webxr_client_config()
        result["session_modes"] = ["immersive-vr", "immersive-ar"]
        result["sew_status_url"] = "/teleop/sew-status"
        return result

    def _setup_routes(self) -> None:
        super()._setup_routes()
        assert self._web_server is not None

        @self._web_server.app.get("/teleop/sew-status")
        async def sew_status() -> dict[str, Any]:
            return self._coordinator.sew_status()

    def _start_server(self) -> None:
        # No auto-generated certificates or LAN exposure. A trusted HTTPS
        # deployment is a separate explicit setup step for a real Quest.
        assert self._web_server is not None
        self._web_server.host = "127.0.0.1"
        self._web_server_thread = threading.Thread(
            target=self._web_server.run, kwargs={"ssl": False}, daemon=True
        )
        self._web_server_thread.start()


teleop_webxr_r1pro_fake = (
    autoconnect(
        R1FakeWebXR.blueprint(body_tracking_mode="optional"),
        R1FakeCoordinator.blueprint(
            instance_name="ControlCoordinator", tasks=[TaskConfig(name="upper_body")]
        ),
    )
    .remappings(
        [
            (R1FakeWebXR, "left_controller_output", "left_cartesian_command"),
            (R1FakeWebXR, "right_controller_output", "right_cartesian_command"),
        ]
    )
    .global_config(skip_system_configuration=True)
)
