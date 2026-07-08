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

"""Characterization tests for coordinator input-stream routing.

These pin the observable routing behavior of the coordinator's input
streams (joint_command, coordinator_cartesian_command,
coordinator_ee_twist_command, twist_command, teleop_buttons) so the
card-routing refactor can prove it preserves them. They intentionally
avoid coordinator internals: messages enter through the ports'
``subscribe`` seam and effects are observed on the tasks.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from typing import Any

import pytest

from dimos.control.components import (
    HardwareComponent,
    HardwareType,
    make_twist_base_joints,
)
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.task import (
    BaseControlTask,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.quest.quest_types import Buttons

ARM_JOINTS = ["arm/joint1", "arm/joint2"]

STREAMS = (
    "joint_command",
    "coordinator_cartesian_command",
    "coordinator_ee_twist_command",
    "twist_command",
    "teleop_buttons",
)


class RecordingTask(BaseControlTask):
    """Stub task that records every stream handler invocation."""

    def __init__(self, name: str, joints: frozenset[str] = frozenset()) -> None:
        self._name = name
        self._joints = frozenset(joints)
        self.cartesian_calls: list[tuple[Any, float]] = []
        self.ee_twist_calls: list[tuple[Any, float]] = []
        self.buttons_calls: list[Any] = []

    @property
    def name(self) -> str:
        return self._name

    def claim(self) -> ResourceClaim:
        return ResourceClaim(joints=self._joints)

    def is_active(self) -> bool:
        return False

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        _ = state
        return None

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        _ = by_task, joints

    def on_cartesian_command(self, pose: Any, t_now: float) -> bool:
        self.cartesian_calls.append((pose, t_now))
        return True

    def on_ee_twist_command(self, twist: Any, t_now: float) -> bool:
        self.ee_twist_calls.append((twist, t_now))
        return True

    def on_buttons(self, msg: Any) -> bool:
        self.buttons_calls.append(msg)
        return True

    def on_teleop_buttons(self, msg: Any, t_now: float) -> bool:
        # Mirrors TeleopIKTask: the uniform handler delegates to on_buttons.
        _ = t_now
        return self.on_buttons(msg)


class VelocityCapableTask(RecordingTask):
    """Stub that opts into the twist fan-out via set_velocity_command."""

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.velocity_commands: list[tuple[float, float, float, float]] = []

    def set_velocity_command(self, vx: float, vy: float, wz: float, t_now: float) -> None:
        self.velocity_commands.append((vx, vy, wz, t_now))


class PortTap:
    """Captures subscribe() on one coordinator port and replays messages."""

    def __init__(self, mocker: Any, port: Any, fail: bool = False) -> None:
        self.callbacks: list[Callable[[Any], None]] = []
        self.unsub = mocker.Mock()

        def _subscribe(cb: Callable[[Any], None]) -> Callable[[], None]:
            if fail:
                raise RuntimeError("no transport configured")
            self.callbacks.append(cb)
            return self.unsub

        mocker.patch.object(port, "subscribe", side_effect=_subscribe)

    @property
    def subscribed(self) -> bool:
        return bool(self.callbacks)

    def emit(self, msg: Any) -> None:
        assert self.callbacks, "port was never subscribed"
        for cb in list(self.callbacks):
            cb(msg)


@pytest.fixture
def make_coordinator(
    mocker,
) -> Iterator[Callable[..., tuple[ControlCoordinator, dict[str, PortTap]]]]:
    """Build a coordinator with all input ports tapped; stop all on teardown."""
    mocker.patch("dimos.control.coordinator.TickLoop")
    coordinators: list[ControlCoordinator] = []

    def make(
        *,
        stub_task_types: bool = False,
        fail_streams: tuple[str, ...] = (),
        **kwargs: Any,
    ) -> tuple[ControlCoordinator, dict[str, PortTap]]:
        coordinator = ControlCoordinator(publish_joint_state=False, **kwargs)
        if stub_task_types:
            coordinator._create_task_from_config = lambda cfg: RecordingTask(
                cfg.name, frozenset(cfg.joint_names)
            )
        taps = {
            stream: PortTap(mocker, getattr(coordinator, stream), fail=stream in fail_streams)
            for stream in STREAMS
        }
        coordinators.append(coordinator)
        return coordinator, taps

    try:
        yield make
    finally:
        for coordinator in coordinators:
            coordinator.stop()


def _streaming_coordinator(make_coordinator):
    coordinator, taps = make_coordinator(
        tasks=[
            TaskConfig(name="servo1", type="servo", joint_names=ARM_JOINTS),
            TaskConfig(name="vel1", type="velocity", joint_names=ARM_JOINTS),
        ]
    )
    coordinator.start()
    return coordinator, taps


class TestJointCommandRouting:
    def test_position_only_updates_servo_task(self, make_coordinator):
        coordinator, taps = _streaming_coordinator(make_coordinator)

        taps["joint_command"].emit(JointState(name=ARM_JOINTS, position=[0.1, 0.2]))

        assert coordinator.get_task("servo1")._target == [0.1, 0.2]
        assert coordinator.get_task("vel1")._velocities is None

    def test_velocity_only_updates_velocity_task(self, make_coordinator):
        coordinator, taps = _streaming_coordinator(make_coordinator)

        taps["joint_command"].emit(JointState(name=ARM_JOINTS, velocity=[0.5, 0.6]))

        assert coordinator.get_task("vel1")._velocities == [0.5, 0.6]
        assert coordinator.get_task("servo1")._target is None

    def test_position_wins_when_both_present(self, make_coordinator):
        coordinator, taps = _streaming_coordinator(make_coordinator)

        taps["joint_command"].emit(
            JointState(name=ARM_JOINTS, position=[0.1, 0.2], velocity=[0.5, 0.6])
        )

        assert coordinator.get_task("servo1")._target == [0.1, 0.2]
        assert coordinator.get_task("vel1")._velocities is None

    def test_unclaimed_joints_route_to_nobody(self, make_coordinator):
        coordinator, taps = _streaming_coordinator(make_coordinator)

        taps["joint_command"].emit(JointState(name=["other/joint9"], position=[1.0]))

        assert coordinator.get_task("servo1")._target is None
        assert coordinator.get_task("vel1")._velocities is None

    def test_empty_message_routes_to_nobody(self, make_coordinator):
        coordinator, taps = _streaming_coordinator(make_coordinator)

        taps["joint_command"].emit(JointState(name=[], position=[]))

        assert coordinator.get_task("servo1")._target is None
        assert coordinator.get_task("vel1")._velocities is None


class TestByTaskNameRouting:
    @staticmethod
    def _cartesian_coordinator(make_coordinator):
        coordinator, taps = make_coordinator(
            stub_task_types=True,
            tasks=[
                TaskConfig(name="cart_a", type="cartesian_ik", joint_names=ARM_JOINTS),
                TaskConfig(name="cart_b", type="cartesian_ik", joint_names=ARM_JOINTS),
            ],
        )
        coordinator.start()
        return coordinator, taps

    def test_cartesian_delivered_only_to_named_task(self, make_coordinator):
        coordinator, taps = self._cartesian_coordinator(make_coordinator)

        taps["coordinator_cartesian_command"].emit(PoseStamped(frame_id="cart_a"))

        cart_a = coordinator.get_task("cart_a")
        cart_b = coordinator.get_task("cart_b")
        assert len(cart_a.cartesian_calls) == 1
        msg, t_now = cart_a.cartesian_calls[0]
        assert msg.frame_id == "cart_a"
        assert isinstance(t_now, float)
        assert cart_b.cartesian_calls == []

    @pytest.mark.parametrize("frame_id", ["unknown_task", ""])
    def test_cartesian_unmatched_frame_id_delivers_nothing(self, make_coordinator, frame_id):
        coordinator, taps = self._cartesian_coordinator(make_coordinator)

        taps["coordinator_cartesian_command"].emit(PoseStamped(frame_id=frame_id))

        assert coordinator.get_task("cart_a").cartesian_calls == []
        assert coordinator.get_task("cart_b").cartesian_calls == []

    @staticmethod
    def _ee_twist_coordinator(make_coordinator):
        coordinator, taps = make_coordinator(
            stub_task_types=True,
            tasks=[
                TaskConfig(name="eef_a", type="eef_twist", joint_names=ARM_JOINTS),
                TaskConfig(name="eef_b", type="eef_twist", joint_names=ARM_JOINTS),
            ],
        )
        coordinator.start()
        return coordinator, taps

    def test_ee_twist_delivered_only_to_named_task(self, make_coordinator):
        coordinator, taps = self._ee_twist_coordinator(make_coordinator)

        taps["coordinator_ee_twist_command"].emit(
            TwistStamped(frame_id="eef_a", linear=[0.1, 0.0, 0.0], angular=[0.0, 0.0, 0.0])
        )

        assert len(coordinator.get_task("eef_a").ee_twist_calls) == 1
        assert coordinator.get_task("eef_b").ee_twist_calls == []

    @pytest.mark.parametrize("frame_id", ["unknown_task", ""])
    def test_ee_twist_unmatched_frame_id_delivers_nothing(self, make_coordinator, frame_id):
        coordinator, taps = self._ee_twist_coordinator(make_coordinator)

        taps["coordinator_ee_twist_command"].emit(
            TwistStamped(frame_id=frame_id, linear=[0.1, 0.0, 0.0], angular=[0.0, 0.0, 0.0])
        )

        assert coordinator.get_task("eef_a").ee_twist_calls == []
        assert coordinator.get_task("eef_b").ee_twist_calls == []


class TestButtonsRouting:
    def test_buttons_reach_teleop_task(self, make_coordinator):
        coordinator, taps = make_coordinator(
            stub_task_types=True,
            tasks=[TaskConfig(name="teleop1", type="teleop_ik", joint_names=ARM_JOINTS)],
        )
        coordinator.start()

        taps["teleop_buttons"].emit(Buttons())

        assert len(coordinator.get_task("teleop1").buttons_calls) == 1


def _base_component() -> HardwareComponent:
    return HardwareComponent(
        hardware_id="base",
        hardware_type=HardwareType.BASE,
        joints=make_twist_base_joints("base"),
        adapter_type="mock_twist_base",
    )


class TestTwistRouting:
    """Twist stays on the legacy path until B3; these guard that seam."""

    def test_base_twist_maps_to_virtual_joint_velocities(self, make_coordinator):
        coordinator, taps = make_coordinator(
            hardware=[_base_component()],
            tasks=[
                TaskConfig(
                    name="basevel",
                    type="velocity",
                    joint_names=make_twist_base_joints("base"),
                )
            ],
        )
        coordinator.start()

        taps["twist_command"].emit(Twist(linear=[1.0, 2.0, 0.0], angular=[0.0, 0.0, 3.0]))

        assert coordinator.get_task("basevel")._velocities == [1.0, 2.0, 3.0]

    def test_twist_fans_out_to_velocity_capable_tasks(self, make_coordinator):
        coordinator, taps = make_coordinator()
        capable = VelocityCapableTask("capable")
        plain = RecordingTask("plain")
        coordinator.add_task(capable)
        coordinator.add_task(plain)
        coordinator.start()

        taps["twist_command"].emit(Twist(linear=[1.0, 2.0, 0.0], angular=[0.0, 0.0, 3.0]))

        assert len(capable.velocity_commands) == 1
        vx, vy, wz, t_now = capable.velocity_commands[0]
        assert (vx, vy, wz) == (1.0, 2.0, 3.0)
        assert isinstance(t_now, float)

    def test_base_twist_both_maps_joints_and_fans_out(self, make_coordinator):
        coordinator, taps = make_coordinator(
            hardware=[_base_component()],
            tasks=[
                TaskConfig(
                    name="basevel",
                    type="velocity",
                    joint_names=make_twist_base_joints("base"),
                )
            ],
        )
        capable = VelocityCapableTask("capable")
        coordinator.add_task(capable)
        coordinator.start()

        taps["twist_command"].emit(Twist(linear=[1.0, 2.0, 0.0], angular=[0.0, 0.0, 3.0]))

        assert coordinator.get_task("basevel")._velocities == [1.0, 2.0, 3.0]
        assert len(capable.velocity_commands) == 1

    def test_twist_subscribed_for_base_hardware_without_tasks(self, make_coordinator):
        coordinator, taps = make_coordinator(hardware=[_base_component()])
        coordinator.start()

        assert taps["twist_command"].subscribed

    def test_twist_subscribed_for_velocity_capable_task_without_base(self, make_coordinator):
        coordinator, taps = make_coordinator()
        coordinator.add_task(VelocityCapableTask("capable"))
        coordinator.start()

        assert taps["twist_command"].subscribed

    def test_twist_not_subscribed_without_base_or_velocity_capable_task(self, make_coordinator):
        coordinator, taps = make_coordinator(
            tasks=[TaskConfig(name="traj", type="trajectory", joint_names=ARM_JOINTS)]
        )
        coordinator.start()

        assert not taps["twist_command"].subscribed


class TestSubscriptionLifecycle:
    def test_streams_without_consumers_are_not_subscribed(self, make_coordinator):
        coordinator, taps = make_coordinator(
            tasks=[TaskConfig(name="traj", type="trajectory", joint_names=ARM_JOINTS)]
        )
        coordinator.start()

        for stream in STREAMS:
            assert not taps[stream].subscribed, stream

    def test_missing_transport_warns_and_start_completes(self, make_coordinator):
        coordinator, taps = make_coordinator(
            fail_streams=("joint_command",),
            tasks=[TaskConfig(name="servo1", type="servo", joint_names=ARM_JOINTS)],
        )

        coordinator.start()

        assert coordinator.get_task("servo1") is not None
        assert not taps["joint_command"].subscribed

    def test_stop_unsubscribes_all_streams(self, make_coordinator):
        coordinator, taps = make_coordinator(
            hardware=[_base_component()],
            tasks=[
                TaskConfig(name="servo1", type="servo", joint_names=ARM_JOINTS),
                TaskConfig(name="vel1", type="velocity", joint_names=ARM_JOINTS),
            ],
        )
        coordinator.start()
        assert taps["joint_command"].subscribed
        assert taps["twist_command"].subscribed

        coordinator.stop()

        taps["joint_command"].unsub.assert_called_once()
        taps["twist_command"].unsub.assert_called_once()


class CardlessStreamTask(RecordingTask):
    """Stub overriding the servo-side digest to observe (non-)delivery."""

    def __init__(self, name: str, joints: frozenset[str] = frozenset()) -> None:
        super().__init__(name, joints)
        self.position_targets: list[dict[str, float]] = []

    def set_target_by_name(self, positions: dict[str, float], t_now: float) -> bool:
        _ = t_now
        self.position_targets.append(positions)
        return True


class ProbeTask(RecordingTask):
    """Stub with a novel handler name, bindable only through a runtime card."""

    def __init__(self, name: str, joints: frozenset[str] = frozenset()) -> None:
        super().__init__(name, joints)
        self.probe_commands: list[tuple[Any, float]] = []

    def on_probe_command(self, msg: Any, t_now: float) -> bool:
        self.probe_commands.append((msg, t_now))
        return True


class RaisingProbeTask(ProbeTask):
    """Probe whose handler raises, to test per-task dispatch isolation."""

    def on_probe_command(self, msg: Any, t_now: float) -> bool:
        _ = msg, t_now
        raise RuntimeError("handler boom")


@pytest.fixture
def probe_card_type() -> Iterator[str]:
    """Register a claim_overlap card bound to on_probe_command; clean up after."""
    from dimos.control.tasks.registry import control_task_registry

    task_type = "routing_probe_task"
    control_task_registry.register_bindings(
        task_type,
        consumes={"joint_command": ("on_probe_command", "claim_overlap")},
    )
    try:
        yield task_type
    finally:
        control_task_registry._bindings.pop(task_type, None)
        control_task_registry._binding_sources.pop(task_type, None)


class TestCardRoutingContract:
    """Contract introduced by card routing: intentional deltas and new seams."""

    def test_buttons_skip_card_less_tasks(self, make_coordinator):
        coordinator, taps = make_coordinator(
            stub_task_types=True,
            tasks=[TaskConfig(name="teleop1", type="teleop_ik", joint_names=ARM_JOINTS)],
        )
        cardless = RecordingTask("cardless")
        coordinator.add_task(cardless)
        coordinator.start()

        taps["teleop_buttons"].emit(Buttons())

        assert len(coordinator.get_task("teleop1").buttons_calls) == 1
        assert cardless.buttons_calls == []

    def test_bare_add_task_gets_no_stream_routing(self, make_coordinator):
        coordinator, taps = _streaming_coordinator(make_coordinator)
        bare = CardlessStreamTask("bare", frozenset(ARM_JOINTS))
        coordinator.add_task(bare)

        taps["joint_command"].emit(JointState(name=ARM_JOINTS, position=[0.1, 0.2]))

        assert coordinator.get_task("servo1")._target == [0.1, 0.2]
        assert bare.position_targets == []

    def test_remove_task_prunes_its_routes(self, make_coordinator):
        coordinator, taps = _streaming_coordinator(make_coordinator)
        servo = coordinator.get_task("servo1")
        assert coordinator.remove_task("servo1")

        taps["joint_command"].emit(JointState(name=ARM_JOINTS, position=[0.1, 0.2]))
        taps["joint_command"].emit(JointState(name=ARM_JOINTS, velocity=[0.5, 0.6]))

        assert servo._target is None
        assert coordinator.get_task("vel1")._velocities == [0.5, 0.6]

    def test_runtime_add_task_with_type_activates_routing(self, make_coordinator):
        from dimos.control.tasks.servo_task.servo_task import (
            JointServoTask,
            JointServoTaskConfig,
        )

        coordinator, taps = make_coordinator()
        coordinator.start()
        assert not taps["joint_command"].subscribed

        task = JointServoTask("servo_rt", JointServoTaskConfig(joint_names=ARM_JOINTS))
        assert coordinator.add_task(task, task_type="servo")

        assert taps["joint_command"].subscribed
        taps["joint_command"].emit(JointState(name=ARM_JOINTS, position=[0.3, 0.4]))
        assert task._target == [0.3, 0.4]

    def test_runtime_registered_card_routes_with_zero_coordinator_edits(
        self, make_coordinator, probe_card_type
    ):
        coordinator, taps = make_coordinator()
        coordinator.start()
        probe = ProbeTask("probe1", frozenset(ARM_JOINTS))
        assert coordinator.add_task(probe, task_type=probe_card_type)

        taps["joint_command"].emit(JointState(name=ARM_JOINTS, position=[0.1, 0.2]))

        assert len(probe.probe_commands) == 1
        msg, t_now = probe.probe_commands[0]
        assert list(msg.name) == ARM_JOINTS
        assert isinstance(t_now, float)

    def test_claim_overlap_gate_blocks_non_overlapping_and_empty(
        self, make_coordinator, probe_card_type
    ):
        # A pure recorder (no self-filtering) pins the dispatcher's own overlap
        # gate — the real servo/velocity tasks would silently no-op and hide it.
        coordinator, taps = make_coordinator()
        coordinator.start()
        probe = ProbeTask("probe1", frozenset(ARM_JOINTS))
        assert coordinator.add_task(probe, task_type=probe_card_type)

        taps["joint_command"].emit(JointState(name=["other/joint9"], position=[1.0]))
        taps["joint_command"].emit(JointState(name=[], position=[]))
        assert probe.probe_commands == []

        taps["joint_command"].emit(JointState(name=ARM_JOINTS, position=[0.1, 0.2]))
        assert len(probe.probe_commands) == 1

    def test_dispatch_isolates_raising_handler_from_siblings(
        self, make_coordinator, probe_card_type
    ):
        coordinator, taps = make_coordinator()
        coordinator.start()
        raiser = RaisingProbeTask("raiser", frozenset(ARM_JOINTS))
        recorder = ProbeTask("recorder", frozenset(ARM_JOINTS))
        assert coordinator.add_task(raiser, task_type=probe_card_type)
        assert coordinator.add_task(recorder, task_type=probe_card_type)

        # raiser is first in the route list; its exception must neither abort
        # delivery to recorder nor propagate out of the port callback (emit).
        taps["joint_command"].emit(JointState(name=ARM_JOINTS, position=[0.1, 0.2]))

        assert len(recorder.probe_commands) == 1

    def test_removing_last_consumer_unsubscribes_stream(self, make_coordinator):
        coordinator, taps = make_coordinator(
            tasks=[
                TaskConfig(name="servo1", type="servo", joint_names=ARM_JOINTS),
                TaskConfig(name="vel1", type="velocity", joint_names=ARM_JOINTS),
            ]
        )
        coordinator.start()
        assert taps["joint_command"].subscribed

        assert coordinator.remove_task("servo1")
        taps["joint_command"].unsub.assert_not_called()  # vel1 still consumes

        assert coordinator.remove_task("vel1")
        taps["joint_command"].unsub.assert_called_once()  # last consumer gone

    def test_unknown_task_type_warns_and_sets_no_routing(self, make_coordinator, mocker):
        import dimos.control.coordinator as coord_mod

        warn = mocker.patch.object(coord_mod.logger, "warning")
        coordinator, taps = make_coordinator()
        coordinator.start()
        warn.reset_mock()  # ignore any start()-time warnings

        assert coordinator.add_task(
            RecordingTask("mystery", frozenset(ARM_JOINTS)), task_type="srvo"
        )

        assert any("srvo" in str(c.args[0]) for c in warn.call_args_list)
        assert not taps["joint_command"].subscribed

    def test_cardless_known_type_does_not_warn(self, make_coordinator, mocker):
        import dimos.control.coordinator as coord_mod

        warn = mocker.patch.object(coord_mod.logger, "warning")
        coordinator, _ = make_coordinator()
        coordinator.start()
        warn.reset_mock()

        # trajectory is a real type with an intentionally empty card.
        coordinator.add_task(RecordingTask("traj", frozenset(ARM_JOINTS)), task_type="trajectory")

        assert not any("unknown task_type" in str(c.args[0]) for c in warn.call_args_list)
