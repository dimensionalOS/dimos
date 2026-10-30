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

"""Collect every robot description and attach one world before module startup."""

from dataclasses import replace
from functools import partial
from types import MappingProxyType
from typing import Any, Protocol, runtime_checkable

from dimos.core.coordination.blueprint_config.fields import module_config_cls
from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom
from dimos.core.global_config import GlobalConfig
from dimos.core.module import ModuleBase, is_module_type
from dimos.sim2.backends.factory import create_backend
from dimos.sim2.blueprint import simulation
from dimos.sim2.device import SimulatedDeviceFactory
from dimos.sim2.module import FEEDBACK_PORTS, PORT_KINDS, SimulationModule
from dimos.sim2.physics import WorldState
from dimos.sim2.sensors.composition import Sensor
from dimos.sim2.sensors.lidar.spec import Lidar
from dimos.sim2.sensors.module import SensorModule
from dimos.sim2.sensors.observations import ObservationModule
from dimos.sim2.spec import RobotConnectionSpec, RobotInstanceSpec
from dimos.sim2.world.scene import scene_path


@runtime_checkable
class SensorConnectionSpec(Protocol):
    @staticmethod
    def simulated_sensor(config: Any) -> Sensor: ...


def has_simulation_binding(module: type[ModuleBase]) -> bool:
    """Whether a module contributes a physical instance at world construction."""
    return isinstance(module, RobotConnectionSpec | SensorConnectionSpec) or issubclass(
        module, SensorModule | ObservationModule
    )


def attach_simulation(
    blueprint: Blueprint, module_kwargs: dict[str, dict[str, Any]], config: GlobalConfig
) -> tuple[Blueprint, dict[str, dict[str, Any]]]:
    """Resolve all instances, then inject clients while retaining robot connections.

    Descriptions declare the command capability. Discovery does not depend on a
    particular connection base class, PD flag, robot family or policy.
    """
    atoms = blueprint.active_blueprints
    robot_atoms = [a for a in atoms if isinstance(a.module, RobotConnectionSpec)]
    replaced_sensors = [a for a in atoms if isinstance(a.module, SensorConnectionSpec)]
    if not robot_atoms:
        if replaced_sensors:
            raise ValueError("simulated hardware sensors require a described robot instance")
        return blueprint, module_kwargs
    world_atoms = [a for a in atoms if issubclass(a.module, SimulationModule)]
    if len(world_atoms) > 1:
        raise ValueError("automatic robot collection requires one shared simulation module")
    explicit_sensors = [a for a in atoms if issubclass(a.module, SensorModule | ObservationModule)]
    state_consumers = [
        atom
        for atom in atoms
        if any(stream.direction == "in" and stream.type is WorldState for stream in atom.streams)
    ]
    robots: dict[str, RobotInstanceSpec] = {}
    owners: dict[str, str] = {}
    kwargs = {name: dict(values) for name, values in module_kwargs.items()}
    robot_topics: dict[str, dict[str, str]] = {}
    for atom in robot_atoms:
        resolved = _resolved(atom, kwargs, config)
        assert isinstance(atom.module, RobotConnectionSpec)
        robot = atom.module.describe_simulated_robot(resolved)
        namespace = atom.name.rpartition("/")[0]
        robot = replace(
            robot,
            robot_id=f"{namespace}/{robot.robot_id}" if namespace else robot.robot_id,
            frame_id=(
                f"{resolved.frame_id_prefix}/{robot.frame_id}"
                if resolved.frame_id_prefix
                else robot.frame_id
            ),
        )
        if robot.robot_id in robots:
            raise ValueError(f"duplicate simulated robot id {robot.robot_id!r}")
        robots[robot.robot_id] = robot
        owners[robot.robot_id] = atom.name
        ports = {stream.name for stream in atom.streams}
        feedback = tuple(kind for kind in FEEDBACK_PORTS if kind in ports)
        command = "cmd_vel" if robot.command_kind == "velocity" else "motor_command"
        topics = {kind: f"_sim2/{atom.name}/{kind}" for kind in (command, *feedback)}
        # Ground truth not exposed by the connection remains robot-scoped when
        # several robots share a world. Preserve the single-robot wire names.
        truth = {
            kind: (kind if len(robot_atoms) == 1 else f"{robot.robot_id}/{kind}")
            for kind in FEEDBACK_PORTS
        }
        robot_topics[robot.robot_id] = {**truth, **topics}
        kwargs.setdefault(atom.name, {})["simulation_device"] = SimulatedDeviceFactory(
            robot=robot,
            topics=topics,
            g=config.model_copy(deep=True),
            feedback_ports=feedback,
            halt_damping=robot.halt_damping if robot.halt_damping is not None else 1.0,
        )

    defaults = {robot.default_scene for robot in robots.values()}
    if config.scene_package in (None, "none") and len(defaults) != 1:
        raise ValueError("robots specify different default scenes; select --scene-package")
    scene = scene_path(config.scene_package, next(iter(defaults)))
    sensors: list[tuple[str | None, Sensor]] = []
    replacements: list[tuple[str | None, Sensor]] = []
    names: dict[tuple[str | None, str], str] = {}
    sensor_owners: dict[str, str | None] = {}
    for atom in replaced_sensors:
        resolved = _resolved(atom, kwargs, config)
        assert isinstance(atom.module, SensorConnectionSpec)
        robot_id = _sensor_owner(atom, None, owners)
        sensor = replace(atom.module.simulated_sensor(resolved), name=atom.name.rsplit("/", 1)[-1])
        if isinstance(sensor, Lidar) and sensor.imu is not None:
            sensor = replace(sensor, imu=replace(sensor.imu, name=f"{sensor.name}_imu"))
        sensors.append((robot_id, sensor))
        replacements.append((robot_id, sensor))
        names[robot_id, sensor.name] = atom.name
        sensor_owners[atom.name] = robot_id
        kwargs.pop(atom.name, None)
    for atom in explicit_sensors:
        resolved = _resolved(atom, kwargs, config)
        robot_id = _sensor_owner(atom, resolved.robot_id, owners)
        sensors.append((robot_id, resolved.sensor))
        kwargs.setdefault(atom.name, {})["robot_id"] = robot_id
    for atom in state_consumers:
        if "robot_id" in module_config_cls(atom).model_fields:
            resolved = _resolved(atom, kwargs, config)
            kwargs.setdefault(atom.name, {})["robot_id"] = (
                None
                if resolved.robot_id is None
                else _sensor_owner(atom, resolved.robot_id, owners)
            )
    sensor_keys = [(owner, sensor.name) for owner, sensor in sensors]
    if len(sensor_keys) != len(set(sensor_keys)):
        raise ValueError("sensor names must be unique within each robot instance")

    backend = partial(
        create_backend,
        name=config.simulation,
        robots=robots,
        scene=scene,
        sensors=tuple(sensors),
    )
    generated = simulation(
        backend=backend,
        robots=robots,
        sensors=tuple(replacements),
        names=names,
        robot_topics=robot_topics,
    )
    generated_world = generated.blueprints[0]
    generated_world = replace(
        generated_world, kwargs={**generated_world.kwargs, "sensors": tuple(sensors)}
    )
    if world_atoms:
        world = world_atoms[0]
        world_config = _resolved(world, kwargs, config)
        if world_config.robots or world_config.robot_topics or world_config.sensors:
            raise ValueError(
                "a manually supplied world uses collected robot/sensor descriptions; "
                "do not also configure robots, robot_topics or sensors on it"
            )
        backend = partial(backend, snapshot_hz=world_config.snapshot_hz)
        kwargs.setdefault(world.name, {}).update(
            backend=world_config.backend or (None if world_config.environment else backend),
            robots=robots,
            robot_topics=robot_topics,
            sensors=tuple(sensors),
        )
        added = generated.blueprints[1:]
        snapshot_topics = {
            port: _effective(blueprint, world, port)
            for port in ("snapshots", "observations", "world_state")
        }
    else:
        added = (generated_world, *generated.blueprints[1:])
        snapshot_topics = {port: port for port in ("snapshots", "observations", "world_state")}
    swapped = {a.name: a for a in replaced_sensors}
    _check_module_refs(blueprint, swapped)
    kept = [a for a in blueprint.blueprints if a.name not in swapped]
    remaps = {key: value for key, value in blueprint.remapping_map.items() if key[0] not in swapped}
    remaps.update(generated.remapping_map)
    for atom in (*explicit_sensors, *generated.blueprints[1:]):
        for stream in atom.streams:
            if stream.direction == "in" and stream.name in snapshot_topics:
                remaps[atom.name, stream.name] = snapshot_topics[stream.name]
    for atom in state_consumers:
        for stream in atom.streams:
            if stream.direction == "in" and stream.type is WorldState:
                remaps[atom.name, stream.name] = snapshot_topics["world_state"]
    for atom in replaced_sensors:
        replacement = next(a for a in generated.blueprints if a.name == atom.name)
        driver_ports = {s.name for s in atom.streams}
        owner = sensor_owners[atom.name] or "world"
        for stream in replacement.streams:
            if stream.direction != "out" or stream.name == "tf":
                continue
            remaps[atom.name, stream.name] = (
                _effective(blueprint, atom, stream.name)
                if stream.name in driver_ports
                else f"{owner}/{atom.name.rsplit('/', 1)[-1]}/{stream.name}"
            )
    return replace(
        blueprint, blueprints=(*kept, *added), remapping_map=MappingProxyType(remaps)
    ), kwargs


def _sensor_owner(atom: BlueprintAtom, requested: str | None, owners: dict[str, str]) -> str | None:
    if requested == "world":
        return None
    namespace = atom.name.rpartition("/")[0]
    if requested is not None:
        relative = f"{namespace}/{requested}" if namespace else requested
        if relative in owners:
            return relative
        if requested in owners:
            return requested
        raise ValueError(f"{atom.name}: unknown simulated robot {requested!r}")
    candidates = [robot for robot, name in owners.items() if name.rpartition("/")[0] == namespace]
    if len(candidates) == 1:
        return candidates[0]
    if len(owners) == 1:
        return next(iter(owners))
    raise ValueError(f"{atom.name}: ambiguous robot ownership; specify robot_id")


def _resolved(
    atom: BlueprintAtom, module_kwargs: dict[str, dict[str, Any]], config: GlobalConfig
) -> Any:
    values = {**atom.kwargs, **module_kwargs.get(atom.name, {})}
    values.update(g=config, instance_name=atom.name)
    return module_config_cls(atom)(**values)


def _effective(blueprint: Blueprint, atom: BlueprintAtom, port: str) -> str:
    value = blueprint.remapping_map.get((atom.name, port), port)
    if not isinstance(value, str):
        raise ValueError(f"{atom.name}.{port} is remapped to a module, not a stream")
    return value


def _check_module_refs(blueprint: Blueprint, swapped: dict[str, BlueprintAtom]) -> None:
    classes = {a.module for a in swapped.values()}
    for atom in blueprint.active_blueprints:
        if atom.name in swapped:
            continue
        for ref in atom.module_refs:
            target = blueprint.remapping_map.get((atom.name, ref.name), ref.spec)
            if is_module_type(target) and target in classes:
                raise ValueError(
                    f"{atom.name}.{ref.name} calls {target.__name__} directly, which has no "
                    "simulated counterpart; depend on a Spec the simulation also provides"
                )


__all__ = ["PORT_KINDS", "attach_simulation", "has_simulation_binding"]
