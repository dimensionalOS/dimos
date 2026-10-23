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

"""Simulated xArm7 pick-and-place stack that an operator can hold.

The same modules as xarm-perception-sim, on a coordinator that also runs the
operator hold and takes hold and e-stop commands and sends their status as
streams. OperatorCommandsModule runs scan, click-to-target and pick commands.
The camera, a point cloud of the scene, every status and every command also go
to the page in web/hybrid-teleop: run
`dimos run hybrid-teleop-sim --serve-dir web/hybrid-teleop`.
"""

from __future__ import annotations

from collections.abc import Callable
import time
from typing import Any

from dimos_lcm.std_msgs import Bool
from reactivex.disposable import Disposable

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.task import ControlTask
from dimos.control.tasks.operator_hold_task.operator_hold_task import (
    OPERATOR_HOLD_TASK_NAME,
    HoldRoute,
    OperatorEstopStatus,
    OperatorHoldCommand,
    OperatorHoldStatus,
    OperatorHoldTask,
    operator_hold_task,
)
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.operator_assist.operator_commands_module import (
    OperatorCommand,
    OperatorCommandsModule,
    OperatorState,
)
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.protocol.rpc.spec import DEFAULT_RPC_TIMEOUTS
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import (
    XARM7_SIM_PATH,
    make_xarm7_sim_hardware,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.utils.logging_config import setup_logger
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.web.cockpit import Channel, cockpit

logger = setup_logger()


class OperatorHoldCoordinator(ControlCoordinator):
    """Add the ports the operator_hold task card reads and reports on, plus an e-stop.

    ``operator_hold_command`` takes an OperatorHoldCommand that requests or
    acknowledges the hold. ``operator_hold_status`` carries every
    OperatorHoldStatus the hold task sends.

    ``operator_estop`` takes a Bool: True stops, False clears. Stop latches
    E-STOP on every task that has ``set_estop``, requests the hold (route
    "failsafe", reason "e-stop") so every joint is frozen where it is, and
    cancels the running trajectory. Clear only releases the E-STOP latch: the
    hold stays on until the operator acknowledges it. ``operator_estop_status``
    sends an OperatorEstopStatus once at start and on every ``set_estop``.

    Combine it with another coordinator subclass by inheriting from both, e.g.
    ``class MyCoordinator(OperatorHoldCoordinator, TeleopControlCoordinator)``.
    """

    operator_hold_command: In[OperatorHoldCommand]
    operator_hold_status: Out[OperatorHoldStatus]
    operator_estop: In[Bool]
    operator_estop_status: Out[OperatorEstopStatus]

    _estopped: bool = False

    @rpc
    def add_task(
        self,
        task: ControlTask,
        task_type: str | None = None,
        stream_bind: dict[str, str] | None = None,
    ) -> bool:
        """Register a task; a hold task's status also goes out on operator_hold_status."""
        added = super().add_task(task, task_type=task_type, stream_bind=stream_bind)
        if added and isinstance(task, OperatorHoldTask):
            task.set_status_publisher(self.operator_hold_status.publish)
        return added

    @rpc
    def start(self) -> None:
        """Start the coordinator, listen on operator_estop and send the current e-stop state."""
        super().start()
        try:
            self.register_disposable(
                Disposable(self.operator_estop.subscribe(self._on_estop_command))
            )
        except Exception:
            logger.warning(
                "Could not subscribe to operator_estop; e-stop works over the set_estop RPC only",
                exc_info=True,
            )
        self._publish_estop_status()

    @rpc
    def set_estop(self, estopped: bool) -> bool:
        """Latch (True) or clear (False) E-STOP on every task that has set_estop.

        The new state also goes out on operator_estop_status.
        """
        try:
            return super().set_estop(estopped)
        finally:
            self._estopped = estopped
            if not estopped:
                logger.info("E-STOP cleared at coordinator")
            self._publish_estop_status()

    def _on_estop_command(self, msg: Bool) -> None:
        """Stop (True) or clear (False) from one operator_estop message."""
        if msg.data:
            # Latch first. The hold is requested before the trajectory is
            # cancelled so no tick passes with nothing commanding the arm.
            steps: list[tuple[str, Callable[[], Any]]] = [
                ("set_estop", lambda: self.set_estop(True)),
                (
                    "request hold",
                    lambda: self.task_invoke(
                        OPERATOR_HOLD_TASK_NAME,
                        "request",
                        {"route": HoldRoute.FAILSAFE.value, "reason": "e-stop"},
                    ),
                ),
                ("cancel trajectory", self.cancel_trajectory),
            ]
        else:
            steps = [("set_estop", lambda: self.set_estop(False))]
        for name, step in steps:
            try:
                step()
            except Exception:
                logger.exception("E-stop step failed", step=name, estop=msg.data)

    def _publish_estop_status(self) -> None:
        try:
            self.operator_estop_status.publish(
                OperatorEstopStatus(estopped=self._estopped, stamp=time.time())
            )
        except Exception:
            logger.exception("Publishing e-stop status failed")


_xarm7_sim_hw = make_xarm7_sim_hardware()

hybrid_teleop_sim = autoconnect(
    ManipulationModule.blueprint(
        model=make_xarm7_sim_robot_config(),
        planning_timeout=10.0,
        visualization={"backend": "viser"},
    ),
    ManipulationSkills.blueprint(),
    PickAndPlaceModule.blueprint(planning_frame="world"),
    HeuristicGraspModule.blueprint(),
    # A pick blocks until the object is held or every grasp failed: minutes.
    OperatorCommandsModule.blueprint(rpc_timeouts={**DEFAULT_RPC_TIMEOUTS, "pick_object": 600.0}),
    # The page's 3D view: rays from the wrist camera (80 x 60, about 1 cm
    # apart on the table) give world-frame points at 5 Hz. Renamed so they do
    # not mix with the object clouds ObjectSceneRegistrationModule sends on
    # "pointcloud".
    MujocoSimModule.blueprint(
        **make_xarm7_sim_module_kwargs(XARM7_SIM_PATH),
        enable_pointcloud=True,
        enable_mujoco_lidar=True,
        mujoco_lidar_raycast_width=80,
        mujoco_lidar_raycast_height=60,
        mujoco_lidar_voxel_size=0.01,
    ).remappings([(MujocoSimModule, "pointcloud", "scene_pointcloud")]),
    ObjectSceneRegistrationModule.blueprint(
        target_frame="world",
        detector_backend="moondream",
        segmentation_backend="edgetam",
        detect_on_request=True,
        # Detections closer than this (metres) count as one object. The objects
        # on the table are 10 to 17 cm apart; the 0.2 m default merges them.
        distance_threshold=0.05,
    ),
    coordinator(
        hardware=[_xarm7_sim_hw],
        tasks=[
            trajectory_task(_xarm7_sim_hw),
            TaskConfig(
                name="arm_gripper",
                type="gripper",
                joint_names=["arm/gripper"],
                priority=20,
            ),
            operator_hold_task(),
        ],
        cls=OperatorHoldCoordinator,
        instance_name="ControlCoordinator",
    ),
    RerunBridgeModule.blueprint(),
    cockpit(
        channels=[
            Channel(
                "color_image",
                Image,
                encoding="jpeg.v1",
                delivery="latest",
                max_hz=15.0,
                params={"quality": 75},
            ),
            # frame_id is for the page only; the encoder reads res (metres).
            Channel(
                "scene_pointcloud",
                PointCloud2,
                encoding="voxels.zlib.v1",
                delivery="latest",
                max_hz=5.0,
                params={"res": 0.01, "frame_id": "world"},
            ),
            Channel(
                "operator_hold_status",
                OperatorHoldStatus,
                max_hz=5.0,
                paced=True,
                resend_on_subscribe=True,
            ),
            Channel(
                "operator_hold_command",
                OperatorHoldCommand,
                dir="tx",
                publish="shared",
                encoding="operator_hold_command.json.v1",
                max_hz=5.0,
            ),
            Channel(
                "operator_estop",
                Bool,
                dir="tx",
                publish="shared",
                encoding="bool.json.v1",
                max_hz=5.0,
            ),
            Channel(
                "operator_estop_status",
                OperatorEstopStatus,
                max_hz=5.0,
                paced=True,
                resend_on_subscribe=True,
            ),
            Channel(
                "operator_command",
                OperatorCommand,
                dir="tx",
                publish="shared",
                encoding="operator_command.json.v1",
                max_hz=5.0,
            ),
            # Paced: a state that follows another within 0.2 s still reaches the page.
            Channel(
                "operator_state",
                OperatorState,
                max_hz=5.0,
                paced=True,
                resend_on_subscribe=True,
            ),
        ]
    ),
)
