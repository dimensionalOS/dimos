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
import threading
from typing import Any

import pytest

from dimos.control._control_test_helpers import RecordingTask
from dimos.control.components import (
    HardwareComponent,
    HardwareType,
    make_twist_base_joints,
)
import dimos.control.coordinator as coord_mod
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.registry import control_task_registry
from dimos.control.tasks.servo_task.servo_task import JointServoTask, JointServoTaskConfig
from dimos.hardware.drive_trains.registry import twist_base_adapter_registry
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


class VelocityCapableTask(RecordingTask):
    """Bare stub with set_velocity_command; card routing gives it nothing."""

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.velocity_commands: list[tuple[float, float, float, float]] = []

    def set_velocity_command(self, vx: float, vy: float, wz: float, t_now: float) -> None:
        self.velocity_commands.append((vx, vy, wz, t_now))


class G1ShapedVelocityTask(VelocityCapableTask):
    """Stub carrying g1_groot_wbc's twist surface, registered under its type."""

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.twist_msgs: list[Any] = []

    def on_twist_command(self, msg: Any, t_now: float) -> None:
        # Mirrors G1GrootWBCTask: the uniform handler delegates to
        # set_velocity_command.
        self.twist_msgs.append(msg)
        self.set_velocity_command(msg.linear.x, msg.linear.y, msg.angular.z, t_now)


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
    """Base virtual-joint mapping is hardware-side; the fan-out is card-routed."""

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
        capable = G1ShapedVelocityTask("capable")
        coordinator.add_task(capable, task_type="g1_groot_wbc")
        coordinator.start()

        taps["twist_command"].emit(Twist(linear=[1.0, 2.0, 0.0], angular=[0.0, 0.0, 3.0]))

        assert coordinator.get_task("basevel")._velocities == [1.0, 2.0, 3.0]
        assert len(capable.velocity_commands) == 1

    def test_twist_subscribed_for_base_hardware_without_tasks(self, make_coordinator):
        coordinator, taps = make_coordinator(hardware=[_base_component()])
        coordinator.start()

        assert taps["twist_command"].subscribed

    def test_twist_not_subscribed_without_base_or_velocity_capable_task(self, make_coordinator):
        coordinator, taps = make_coordinator(
            tasks=[TaskConfig(name="traj", type="trajectory", joint_names=ARM_JOINTS)]
        )
        coordinator.start()

        assert not taps["twist_command"].subscribed

    def test_base_suffix_subset_maps_only_declared_joints(self, make_coordinator):
        carlike_joints = make_twist_base_joints("base", ["vx", "wz"])
        coordinator, taps = make_coordinator(
            hardware=[
                HardwareComponent(
                    hardware_id="base",
                    hardware_type=HardwareType.BASE,
                    joints=carlike_joints,
                    adapter_type="mock_twist_base",
                )
            ],
            tasks=[TaskConfig(name="basevel", type="velocity", joint_names=carlike_joints)],
        )
        coordinator.start()

        taps["twist_command"].emit(Twist(linear=[1.0, 2.0, 0.0], angular=[0.0, 0.0, 3.0]))

        assert coordinator.get_task("basevel")._velocities == [1.0, 3.0]

    def test_base_full_6d_twist_maps_all_axes(self, make_coordinator):
        # Drones and such: a base may declare all six twist axes.
        drone_joints = make_twist_base_joints("base", ["vx", "vy", "vz", "wx", "wy", "wz"])
        coordinator, taps = make_coordinator(
            hardware=[
                HardwareComponent(
                    hardware_id="base",
                    hardware_type=HardwareType.BASE,
                    joints=drone_joints,
                    adapter_type="mock_twist_base",
                )
            ],
            tasks=[TaskConfig(name="basevel", type="velocity", joint_names=drone_joints)],
        )
        coordinator.start()

        taps["twist_command"].emit(Twist(linear=[1.0, 2.0, 3.0], angular=[4.0, 5.0, 6.0]))

        assert coordinator.get_task("basevel")._velocities == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]

    def test_twist_mapping_ignores_non_base_hardware(self, make_coordinator):
        # A manipulator joint named like a twist axis must not be mapped.
        coordinator, taps = make_coordinator(
            hardware=[
                _base_component(),
                HardwareComponent(
                    hardware_id="arm",
                    hardware_type=HardwareType.MANIPULATOR,
                    joints=["arm/vx"],
                    adapter_type="mock",
                ),
            ],
            tasks=[
                TaskConfig(
                    name="basevel",
                    type="velocity",
                    joint_names=make_twist_base_joints("base"),
                ),
                TaskConfig(name="armvel", type="velocity", joint_names=["arm/vx"]),
            ],
        )
        coordinator.start()

        taps["twist_command"].emit(Twist(linear=[1.0, 2.0, 0.0], angular=[0.0, 0.0, 3.0]))

        assert coordinator.get_task("basevel")._velocities == [1.0, 2.0, 3.0]
        assert coordinator.get_task("armvel")._velocities is None

    def test_twist_reaches_declaring_velocity_task_without_base(self, make_coordinator):
        coordinator, taps = make_coordinator()
        task = G1ShapedVelocityTask("g1")
        coordinator.add_task(task, task_type="g1_groot_wbc")
        coordinator.start()

        assert taps["twist_command"].subscribed
        taps["twist_command"].emit(Twist(linear=[1.0, 2.0, 0.0], angular=[0.0, 0.0, 3.0]))

        assert len(task.velocity_commands) == 1
        vx, vy, wz, t_now = task.velocity_commands[0]
        assert (vx, vy, wz) == (1.0, 2.0, 3.0)
        assert isinstance(t_now, float)


class TestTwistCardContract:
    """Contract introduced by the twist migration: intentional deltas and new seams."""

    def test_card_routed_twist_delivers_raw_msg_to_on_twist_command(self, make_coordinator):
        coordinator, taps = make_coordinator()
        task = G1ShapedVelocityTask("g1")
        coordinator.add_task(task, task_type="g1_groot_wbc")
        coordinator.start()

        msg = Twist(linear=[1.0, 2.0, 0.0], angular=[0.0, 0.0, 3.0])
        taps["twist_command"].emit(msg)

        assert len(task.twist_msgs) == 1
        assert task.twist_msgs[0] is msg  # the raw message, not a digest

    def test_bare_set_velocity_command_stub_gets_nothing(self, make_coordinator):
        # Intentional delta: the fan-out narrowed to card-declared consumers.
        coordinator, taps = make_coordinator(hardware=[_base_component()])
        bare = VelocityCapableTask("bare")
        coordinator.add_task(bare)
        coordinator.start()
        assert taps["twist_command"].subscribed  # the base keeps the stream alive

        taps["twist_command"].emit(Twist(linear=[1.0, 2.0, 0.0], angular=[0.0, 0.0, 3.0]))

        assert bare.velocity_commands == []

    def test_runtime_base_add_remove_toggles_twist_subscription(self, make_coordinator):
        coordinator, taps = make_coordinator()
        coordinator.start()
        assert not taps["twist_command"].subscribed

        component = _base_component()
        adapter = twist_base_adapter_registry.create(
            "mock_twist_base", dof=len(component.joints), hardware_id="base"
        )
        assert adapter.connect()
        assert coordinator.add_hardware(adapter, component)
        assert taps["twist_command"].subscribed

        assert coordinator.remove_hardware("base")
        taps["twist_command"].unsub.assert_called_once()

    def test_twist_delivery_concurrent_with_remove_hardware(self, make_coordinator):
        coordinator, taps = make_coordinator(hardware=[_base_component()])
        coordinator.start()
        assert taps["twist_command"].subscribed

        msg = Twist(linear=[1.0, 0.0, 0.0], angular=[0.0, 0.0, 0.5])
        stop = threading.Event()

        def pump() -> None:
            while not stop.is_set():
                taps["twist_command"].emit(msg)

        removed: list[bool] = []
        pumper = threading.Thread(target=pump, daemon=True)
        remover = threading.Thread(
            target=lambda: removed.append(coordinator.remove_hardware("base")), daemon=True
        )
        pumper.start()
        remover.start()
        remover.join(timeout=5.0)
        stop.set()
        pumper.join(timeout=5.0)

        assert not remover.is_alive(), "remove_hardware deadlocked against twist delivery"
        assert not pumper.is_alive(), "twist delivery deadlocked against remove_hardware"
        assert removed == [True]


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
        raise RuntimeError("handler boom")


@pytest.fixture
def probe_card_type() -> Iterator[str]:
    """Register a claim_overlap card bound to on_probe_command; clean up after."""
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
        warn = mocker.patch.object(coord_mod.logger, "warning")
        coordinator, taps = make_coordinator()
        coordinator.start()
        warn.reset_mock()  # ignore any start()-time warnings

        assert coordinator.add_task(
            RecordingTask("mystery", frozenset(ARM_JOINTS)), task_type="srvo"
        )

        assert any("srvo" in str(c) for c in warn.call_args_list)
        assert not taps["joint_command"].subscribed

    def test_cardless_known_type_does_not_warn(self, make_coordinator, mocker):
        warn = mocker.patch.object(coord_mod.logger, "warning")
        coordinator, _ = make_coordinator()
        coordinator.start()
        warn.reset_mock()

        # trajectory is a real type with an intentionally empty card.
        coordinator.add_task(RecordingTask("traj", frozenset(ARM_JOINTS)), task_type="trajectory")

        assert not any("unknown task_type" in str(c.args[0]) for c in warn.call_args_list)
