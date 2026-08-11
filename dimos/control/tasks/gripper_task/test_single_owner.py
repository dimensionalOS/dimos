# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Single-owner audit: gripper joints are claimed only by gripper tasks.

Instantiates tasks and reads claim(); a TaskConfig scan misses claims widened
at runtime.
"""

from __future__ import annotations

import inspect
from pathlib import Path

import pytest

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.hardware_interface import ConnectedHardware
from dimos.control.tasks.registry import control_task_registry
from dimos.hardware.manipulators.mock.adapter import MockAdapter
from dimos.hardware.manipulators.spec import JointLimits

_SOURCE_ROOT = Path(__file__).resolve().parents[3]

# Blueprints that put a gripper on a device. Each must have exactly one owner.
_GRIPPER_BLUEPRINTS = [
    ("dimos.robot.manipulators.a1z.blueprints.teleop", "keyboard_teleop_a1z"),
    ("dimos.robot.manipulators.a1z.blueprints.teleop", "coordinator_teleop_a1z"),
    ("dimos.robot.manipulators.piper.blueprints.teleop", "keyboard_teleop_piper"),
    ("dimos.robot.manipulators.piper.blueprints.teleop", "coordinator_teleop_piper"),
    ("dimos.robot.manipulators.xarm.blueprints.teleop", "keyboard_teleop_xarm6"),
    ("dimos.robot.manipulators.xarm.blueprints.teleop", "keyboard_teleop_xarm7"),
    ("dimos.robot.manipulators.xarm.blueprints.teleop", "coordinator_teleop_xarm6"),
    ("dimos.robot.manipulators.xarm.blueprints.teleop", "coordinator_teleop_xarm7"),
]


def _blueprint(module: str, name: str):
    import importlib

    return getattr(importlib.import_module(module), name)


def _coordinator_kwargs(blueprint):
    return next(
        a.kwargs
        for a in blueprint.blueprints
        if isinstance(a.module, type) and issubclass(a.module, ControlCoordinator)
    )


def _fake_hardware(components) -> dict[str, ConnectedHardware]:
    """Mock adapters shaped like the real components, so tasks can be built."""
    hardware = {}
    for component in components:
        adapter = MockAdapter(
            dof=len(component.all_joints),
            limits=JointLimits(
                position_lower=[-3.14] * (len(component.all_joints) - 1) + [0.0],
                position_upper=[3.14] * (len(component.all_joints) - 1) + [0.08],
                velocity_max=[1.0] * len(component.all_joints),
            ),
        )
        adapter.connect()
        hardware[component.hardware_id] = ConnectedHardware(adapter, component)
    return hardware


@pytest.mark.parametrize(("module", "name"), _GRIPPER_BLUEPRINTS, ids=lambda v: str(v))
def test_no_task_config_declares_a_gripper_joint_but_the_gripper_task(
    module: str, name: str
) -> None:
    """Static half: every blueprint, including ones whose tasks need assets."""
    kwargs = _coordinator_kwargs(_blueprint(module, name))
    gripper_joints = {
        joint for cfg in kwargs["tasks"] if cfg.type == "gripper" for joint in cfg.joint_names
    }
    if not gripper_joints:
        pytest.skip(f"{name} has no gripper")

    owners = [
        f"{cfg.name} ({cfg.type})"
        for cfg in kwargs["tasks"]
        if set(cfg.joint_names) & gripper_joints
    ]
    assert owners, f"{name}: gripper {sorted(gripper_joints)} has no owner at all"
    assert all(o.endswith("(gripper)") for o in owners), f"{name}: claimed by {owners}"


@pytest.mark.parametrize(("module", "name"), _GRIPPER_BLUEPRINTS, ids=lambda v: str(v))
def test_gripper_joints_have_exactly_one_runtime_claimant(module: str, name: str) -> None:
    """Runtime half: unbuildable tasks must be structurally unable to claim."""
    kwargs = _coordinator_kwargs(_blueprint(module, name))
    components = kwargs["hardware"]
    gripper_joints = {
        joint for cfg in kwargs["tasks"] if cfg.type == "gripper" for joint in cfg.joint_names
    }
    if not gripper_joints:
        pytest.skip(f"{name} has no gripper")

    hardware = _fake_hardware(components)
    claimants: dict[str, list[str]] = {j: [] for j in gripper_joints}

    for cfg in kwargs["tasks"]:
        try:
            task = control_task_registry.create(cfg.type, cfg, hardware=hardware)
        except Exception:
            # Not buildable here (needs model assets). Prove it cannot be a
            # hidden second owner rather than passing over it in silence.
            assert not (set(cfg.joint_names) & gripper_joints), (
                f"{name}: {cfg.name!r} ({cfg.type}) declares gripper joints and could "
                "not be built to check its runtime claim"
            )
            assert not _augments_claim(cfg.type), (
                f"{name}: {cfg.name!r} ({cfg.type}) overrides claim() and could not be "
                "built — its runtime claim is unverifiable, so it must not override"
            )
            continue
        # The runtime claim, not the declared joint_names.
        for joint in task.claim().joints & gripper_joints:
            claimants[joint].append(f"{cfg.name} ({cfg.type})")

    for joint, owners in claimants.items():
        assert len(owners) == 1, f"{name}: {joint} claimed by {owners or 'nobody'}"
        assert owners[0].endswith("(gripper)"), (
            f"{name}: {joint} is owned by {owners[0]}, not a gripper task"
        )


def _augments_claim(task_type: str) -> bool:
    """True if the task type's module defines its own claim()."""
    import importlib

    path = control_task_registry._factory_paths[task_type.lower()]
    module_name, _factory = path.split(":", 1)
    module = importlib.import_module(module_name)
    return any(
        inspect.isclass(obj)
        # Defined here, not imported: a base class's claim() returns exactly
        # the configured joints, so only a local override can widen it.
        and obj.__module__ == module_name
        and "claim" in vars(obj)
        for obj in vars(module).values()
    )


class TestTheOldPathsAreGone:
    @staticmethod
    def _sources() -> list[Path]:
        """Production sources; tests legitimately name the deleted symbols."""
        return [
            p
            for p in _SOURCE_ROOT.rglob("*.py")
            if not p.name.startswith(("test_", "demo_")) and ".venv" not in p.parts
        ]

    @pytest.mark.parametrize(
        "symbol",
        [
            "claim_with_gripper",
            "append_gripper_position",
            "XARM_GRIPPER_PARAMS",
            "GripperTaskOverrides",
            "gripper_open_pos",
            "gripper_closed_pos",
            "gripper_open_position",
            "gripper_closed_position",
            "_normalized_to_physical",
            "_physical_to_normalized",
            "write_gripper_position",
        ],
    )
    def test_symbol_is_gone(self, symbol: str) -> None:
        hits = [
            f"{p.relative_to(_SOURCE_ROOT)}"
            for p in self._sources()
            if symbol in p.read_text(encoding="utf-8", errors="ignore")
        ]
        assert not hits, f"{symbol!r} should have been deleted; still in {hits}"

    def test_the_coordinator_has_no_gripper_rpc(self) -> None:
        assert not hasattr(ControlCoordinator, "set_gripper_position")
        assert not hasattr(ControlCoordinator, "get_gripper_position")

    def test_the_adapter_protocol_has_no_scalar_gripper_api(self) -> None:
        from dimos.hardware.manipulators.spec import ManipulatorAdapter

        members = dict(inspect.getmembers(ManipulatorAdapter))
        assert "read_gripper_position" not in members
        assert "write_gripper_position" not in members
        assert "get_gripper_dof" not in members


class TestSkillRoundTrip:
    """R26: one scale, so set_gripper(get_gripper()) is a no-op."""

    @staticmethod
    def _wired():
        from dimos.control.tasks.gripper_task.gripper_task import create_task

        component = HardwareComponent(
            hardware_id="arm",
            hardware_type=HardwareType.MANIPULATOR,
            all_joints=[*make_joints("arm", 6), "arm/gripper"],
        )
        adapter = MockAdapter(
            dof=7,
            limits=JointLimits(
                position_lower=[-3.14] * 6 + [0.0],
                position_upper=[3.14] * 6 + [0.08],
                velocity_max=[1.0] * 7,
            ),
        )
        adapter.connect()
        hardware = ConnectedHardware(adapter, component)
        task = create_task(
            TaskConfig(
                name="arm_gripper",
                type="gripper",
                joint_names=["arm/gripper"],
                priority=20,
            ),
            {"arm": hardware},
        )
        return task, hardware, adapter

    @staticmethod
    def _apply(task, hardware) -> float:
        """Run one tick and return what the adapter now holds."""
        from dimos.control.task import CoordinatorState, JointStateSnapshot

        positions = {n: s.position for n, s in hardware.read_state().items()}
        out = task.compute(
            CoordinatorState(joints=JointStateSnapshot(joint_positions=positions), t_now=0.0)
        )
        if out is not None:
            hardware.write_command(dict(zip(out.joint_names, out.positions, strict=True)), out.mode)
        return hardware.read_state()["arm/gripper"].position

    @pytest.mark.parametrize("fraction", [0.0, 0.25, 0.5, 1.0])
    def test_normalized_in_normalized_out(self, fraction: float) -> None:
        task, hardware, _ = self._wired()
        task.set_normalized([fraction], t_now=0.0)
        native = self._apply(task, hardware)

        assert native / 0.08 == pytest.approx(fraction)

    def test_open_and_close_land_on_the_sweep_endpoints(self) -> None:
        task, hardware, _ = self._wired()

        task.set_normalized([0.0], t_now=0.0)
        assert self._apply(task, hardware) == pytest.approx(0.0)

        task.set_normalized([1.0], t_now=0.0)
        assert self._apply(task, hardware) == pytest.approx(0.08)
