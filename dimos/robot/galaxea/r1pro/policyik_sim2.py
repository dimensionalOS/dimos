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

"""Encoder-feedback host for the coordinated R1 PolicyIK task in Sim2."""

import json
from pathlib import Path
import threading
from typing import Any

from dimos_lcm.common_msgs import SampleHeader
from dimos_lcm.control_msgs import ActionCommand, JointCommand
from dimos_lcm.std_msgs import Header, Time

from dimos.control.policyik.controller import PERIOD_NS
from dimos.control.policyik.task import PolicyIKTask, PolicyIKTaskConfig
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.rpc_client import RPCClient
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseArray import PoseArray
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.sim2.module import SimModule


class PolicyIKSimConfig(ModuleConfig):
    bundle: Path
    simulation: str = "policyik_simulation"
    command_ttl_ns: int = 100_000_000


class PolicyIKSimModule(Module):
    """Use encoder q/dq plus URDF FK; no simulator body pose enters the actor."""

    config: PolicyIKSimConfig
    joint_state: In[JointState]
    hand_goals: In[PoseArray]
    actions: Out[ActionCommand]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._task = PolicyIKTask("r1pro_policyik", PolicyIKTaskConfig(self.config.bundle))
        self._lock = threading.RLock()
        self._clock_id: str | None = None
        self._last_ns = -1
        self._sequence = 0
        self._missed = 0
        self._manifest: dict[str, Any] = {}
        self._paused = False
        self._last_reference: list[float] | None = None

    @rpc
    def build(self) -> None:
        super().build()
        if not PERIOD_NS <= self.config.command_ttl_ns <= 1_000_000_000:
            raise ValueError("Command TTL must cover one control interval and be at most 1 s")
        self._manifest = json.loads((self.config.bundle / "manifest.json").read_text())
        simulation = RPCClient.remote(SimModule, self.config.simulation, rpc=self.rpc)
        try:
            self._clock_id = simulation.simulation_config()["sim_clock_id"]
        finally:
            simulation.stop_rpc_client()
        self._task.start()

    @rpc
    def set_goals(self, goals: list[list[float]], frame: str) -> None:
        self._task.set_goals(goals, frame)

    async def handle_hand_goals(self, value: PoseArray) -> None:
        rows = [
            [
                p.position.x,
                p.position.y,
                p.position.z,
                p.orientation.w,
                p.orientation.x,
                p.orientation.y,
                p.orientation.z,
            ]
            for p in value.poses
        ]
        self.set_goals(rows, value.frame_id)

    @rpc
    def set_estop(self, estopped: bool) -> None:
        with self._lock:
            self._task.set_estop(estopped)
            if estopped:
                self._last_reference = None

    @rpc
    def set_paused(self, paused: bool) -> None:
        """Freeze the last commanded reference, retaining its gravity offset.

        This pauses policy inference, not physics or the joint servos. It is
        not an emergency stop; command expiry still belongs to SimModule.
        """
        with self._lock:
            self._paused = paused

    async def handle_joint_state(self, sample: JointState) -> None:
        action = self.compute(sample)
        if action is not None:
            self.actions.publish(action)

    def compute(self, sample: JointState) -> ActionCommand | None:
        with self._lock:
            if self._clock_id is None:
                raise RuntimeError("Build policy before accepting measurements")
            stamp = round(sample.ts * 1e9)
            if stamp <= self._last_ns:
                return None
            if self._last_ns >= 0:
                self._missed += max(0, round((stamp - self._last_ns) / PERIOD_NS) - 1)
            state = CoordinatorState(
                joints=JointStateSnapshot(
                    joint_positions=dict(zip(sample.name, sample.position, strict=True)),
                    joint_velocities=dict(zip(sample.name, sample.velocity, strict=True)),
                    timestamp=sample.ts,
                ),
                t_now=sample.ts,
                dt=PERIOD_NS / 1e9,
            )
            result = None if self._paused else self._task.compute(state)
            self._last_ns = stamp
            if result is not None and result.positions is not None:
                self._last_reference = list(result.positions)
            elif not self._paused:
                return None
            if self._last_reference is None:
                return None
            self._sequence += 1
            names = self._manifest["joints"]
            return ActionCommand(
                header=SampleHeader(
                    clock_id=self._clock_id,
                    epoch=0,
                    time_ns=stamp,
                    source_id=self.config.instance_name or "policyik_sim",
                    sequence=self._sequence,
                    frame_id=self._manifest["frame"],
                ),
                based_on_time_ns=stamp,
                apply_at_ns=stamp,
                valid_until_ns=stamp + self.config.command_ttl_ns,
                joint_commands_length=5,
                joint_commands=[
                    JointCommand(
                        header=Header(stamp=Time(sec=stamp // 10**9, nsec=stamp % 10**9)),
                        joint_names_length=len(names),
                        joint_names=list(names),
                        interface_name=interface,
                        values_length=len(names),
                        values=list(values),
                    )
                    for interface, values in (
                        ("position", self._last_reference),
                        ("velocity", [0.0] * len(names)),
                        ("kp", self._manifest["kp"]),
                        ("kd", self._manifest["kd"]),
                        ("effort", [0.0] * len(names)),
                    )
                ],
                cbor_commands_length=0,
                cbor_commands=[],
            )

    @rpc
    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                **self._task.status(),
                "paused": self._paused,
                "missed_encoder_acquisitions": self._missed,
                "last_sensor_ns": self._last_ns,
                "published_commands": self._sequence,
            }

    @rpc
    def stop(self) -> None:
        self._task.stop()
        super().stop()
