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

"""Add a shared world while keeping each robot's connection in the blueprint.

Connections describe their simulated bodies. This build step resolves those
descriptions and supplies a simulated device factory; ConnectionModule chooses
that device at startup. Only standalone sensor drivers are replaced by sensor
workers. The connection keeps its public ports, RPCs and command validation.
"""

from dataclasses import replace
from functools import partial
from types import MappingProxyType
from typing import Any

from dimos.control.connection.connection_module import ConnectionModule
from dimos.core.coordination.blueprint_config.fields import module_config_cls
from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom
from dimos.core.global_config import GlobalConfig
from dimos.core.module import ModuleBase, is_module_type
from dimos.sim2.blueprint import simulation
from dimos.sim2.device import SimulatedDeviceFactory
from dimos.sim2.environment import MotorScene
from dimos.sim2.module import FEEDBACK_PORTS, PORT_KINDS
from dimos.sim2.sensors.composition import Sensor
from dimos.sim2.sensors.lidar.spec import Lidar
from dimos.sim2.spec import SimulatedRobot
from dimos.sim2.world.scene import scene_path

ROBOT_TWIN = "simulated_robot"
SENSOR_TWIN = "simulated_sensor"
DEFAULT_SCENE = "default_sim_scene"


def declares_twin(module: type[ModuleBase]) -> bool:
    return hasattr(module, ROBOT_TWIN) or hasattr(module, SENSOR_TWIN)


def simulate(
    blueprint: Blueprint, module_kwargs: dict[str, dict[str, Any]], config: GlobalConfig
) -> tuple[Blueprint, dict[str, dict[str, Any]]]:
    """Resolve simulation after CLI overrides, before modules are instantiated.

    The current automatic composition supports one robot and its sensors.
    Explicit MotorScene/SimulationModule composition can contain many robots.
    """
    atoms = blueprint.active_blueprints
    robot_atoms = [a for a in atoms if hasattr(a.module, ROBOT_TWIN)]
    sensor_atoms = [a for a in atoms if hasattr(a.module, SENSOR_TWIN)]
    if not robot_atoms and not sensor_atoms:
        return blueprint, module_kwargs
    if len(robot_atoms) != 1:
        found = sorted(a.name for a in robot_atoms)
        raise ValueError(f"sim2 collects exactly one robot connection for now; found {found}")
    robot_atom = robot_atoms[0]
    if not issubclass(robot_atom.module, ConnectionModule):
        raise TypeError(f"{robot_atom.name}.simulated_robot requires a ConnectionModule")
    resolved = _resolved(robot_atom, module_kwargs, config)
    if not resolved.pd:
        raise ValueError("the simulated motor device currently requires the PD command interface")
    ports = {s.name: s for s in robot_atom.streams}
    if not ("motor_command" in ports and "motor_states" in ports):
        raise ValueError(
            "a simulated motor connection requires motor_command and motor_states ports"
        )
    default_scene = getattr(robot_atom.module, DEFAULT_SCENE, None)
    if not isinstance(default_scene, str):
        raise TypeError(f"{robot_atom.name} must declare default_sim_scene next to simulated_robot")
    scene = scene_path(config.scene_package, default_scene)
    twin = getattr(robot_atom.module, ROBOT_TWIN)(resolved, scene)
    if not isinstance(twin, SimulatedRobot):
        raise TypeError(f"{robot_atom.name}.simulated_robot must return a SimulatedRobot")
    robot_id = twin.robot_id
    twin = replace(
        twin, halt_damping=resolved.halt_damping, command_timeout=resolved.deadman_timeout_s
    )
    sensors: list[tuple[str | None, Sensor]] = []
    names: dict[tuple[str | None, str], str] = {}
    for atom in sensor_atoms:
        sensor: Sensor = getattr(atom.module, SENSOR_TWIN)(_resolved(atom, module_kwargs, config))
        sensor = replace(sensor, name=atom.name)
        if isinstance(sensor, Lidar) and sensor.imu is not None:
            sensor = replace(sensor, imu=replace(sensor.imu, name=f"{atom.name}_imu"))
        sensors.append((robot_id, sensor))
        names[robot_id, atom.name] = atom.name

    # The world and its device use private streams; the original connection
    # remains the only publisher/consumer of its public motor interface.
    feedback = tuple(kind for kind in FEEDBACK_PORTS if kind in ports)
    topics = {kind: f"_sim2/{robot_atom.name}/{kind}" for kind in ("motor_command", *feedback)}
    # Preserve ground-truth streams that hardware does not provide.
    world_topics = {**{kind: kind for kind in FEEDBACK_PORTS}, **topics}
    environment = partial(
        MotorScene, scene=scene, models={robot_id: twin.model}, sensors=tuple(sensors)
    )
    generated = simulation(
        environment=environment,
        robots={robot_id: twin},
        sensors=tuple(sensors),
        names=names,
        robot_topics={robot_id: world_topics},
    )

    swapped = {a.name: a for a in sensor_atoms}
    _check_module_refs(blueprint, swapped)
    kept = [a for a in blueprint.blueprints if a.name not in swapped]
    remaps = {k: v for k, v in blueprint.remapping_map.items() if k[0] not in swapped}
    remaps.update(generated.remapping_map)
    for atom in sensor_atoms:
        replacement = next(a for a in generated.blueprints if a.name == atom.name)
        driver_ports = {s.name for s in atom.streams}
        for stream in replacement.streams:
            if stream.direction != "out" or stream.name == "tf":
                continue
            remaps[atom.name, stream.name] = (
                _effective(blueprint, atom, stream.name)
                if stream.name in driver_ports
                else f"{robot_id}/{atom.name}/{stream.name}"
            )

    composed = replace(
        blueprint,
        blueprints=(*kept, *generated.blueprints),
        remapping_map=MappingProxyType(remaps),
    )
    kwargs = {k: dict(v) for k, v in module_kwargs.items() if k not in swapped}
    kwargs.setdefault(robot_atom.name, {})["simulation_device"] = SimulatedDeviceFactory(
        robot=twin,
        topics=topics,
        g=config.model_copy(deep=True),
        feedback_ports=feedback,
        halt_damping=resolved.halt_damping,
    )
    return composed, kwargs


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


__all__ = ["PORT_KINDS", "declares_twin", "simulate"]
