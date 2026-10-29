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

"""Swap hardware modules for their simulated twins under ``--simulation mujoco``.

A hardware module opts in by describing its twin:

- A robot connection declares ``default_sim_scene: ClassVar[str]`` and a static
  ``simulated_robot(config, scene: Path) -> SimulatedRobot``: the robot's body
  placed in the scene, and the ``SimulationModule`` subclass whose ports
  ``<robot_id>_<port>`` mirror the connection's own ports.
- A sensor driver declares a static ``simulated_sensor(config) -> Sensor``: the
  device the world should attach at the driver's ``frame_id``, which must be a
  site or camera in the robot's MJCF.

``simulate`` replaces each such module with the twin under the same instance
name and the same stream names, so every other module in the blueprint, the
coordinator included, is wired exactly as on hardware. Nothing sim-specific is
added to the wire. Ground-truth ports the hardware never had (``odom``, ``tf``,
a lidar's world-frame ``pointcloud``) get plain or robot-qualified names.
"""

from __future__ import annotations

from dataclasses import replace
from types import MappingProxyType
from typing import Any

from dimos.core.coordination.blueprint_config.fields import module_config_cls
from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom
from dimos.core.global_config import GlobalConfig
from dimos.core.module import ModuleBase, is_module_type
from dimos.sim2.blueprint import simulation
from dimos.sim2.module import PORT_KINDS, robot_ports
from dimos.sim2.sensors.composition import Sensor
from dimos.sim2.sensors.lidar.spec import Lidar
from dimos.sim2.spec import SimulatedRobot, WorldConfig
from dimos.sim2.world.scene import scene_path

ROBOT_TWIN = "simulated_robot"
SENSOR_TWIN = "simulated_sensor"
DEFAULT_SCENE = "default_sim_scene"


def declares_twin(module: type[ModuleBase]) -> bool:
    return hasattr(module, ROBOT_TWIN) or hasattr(module, SENSOR_TWIN)


def simulate(
    blueprint: Blueprint, module_kwargs: dict[str, dict[str, Any]], config: GlobalConfig
) -> tuple[Blueprint, dict[str, dict[str, Any]]]:
    """Replace every twin-declaring module by its simulation.

    Module configs are resolved the way the coordinator resolves them, including
    CLI overrides in ``module_kwargs``, and no module is instantiated. The
    overrides of a swapped module are consumed by its twin and dropped from the
    returned kwargs, since the replacement under the same name has another
    config. A blueprint without twin declarations is returned unchanged.
    """
    atoms = blueprint.active_blueprints
    robot_atoms = [a for a in atoms if hasattr(a.module, ROBOT_TWIN)]
    sensor_atoms = [a for a in atoms if hasattr(a.module, SENSOR_TWIN)]
    if not robot_atoms and not sensor_atoms:
        return blueprint, module_kwargs
    if len(robot_atoms) != 1:
        found = sorted(a.name for a in robot_atoms)
        raise ValueError(
            f"sim2 swaps exactly one robot connection per blueprint for now; found {found}"
        )
    robot_atom = robot_atoms[0]
    default_scene = getattr(robot_atom.module, DEFAULT_SCENE, None)
    if not isinstance(default_scene, str):
        raise TypeError(f"{robot_atom.name} must declare default_sim_scene next to simulated_robot")
    scene = scene_path(config.scene_package, default_scene)
    twin = getattr(robot_atom.module, ROBOT_TWIN)(
        _resolved(robot_atom, module_kwargs, config), scene
    )
    if not isinstance(twin, SimulatedRobot):
        raise TypeError(f"{robot_atom.name}.simulated_robot must return a SimulatedRobot")
    robot_id, world_cls, instance = twin.robot_id, twin.world, twin.instance
    names: dict[tuple[str | None, str], str] = {}
    for atom in sensor_atoms:
        sensor: Sensor = getattr(atom.module, SENSOR_TWIN)(_resolved(atom, module_kwargs, config))
        sensor = replace(sensor, name=atom.name)
        if isinstance(sensor, Lidar) and sensor.imu is not None:
            sensor = replace(sensor, imu=replace(sensor.imu, name=f"{atom.name}_imu"))
        instance = replace(instance, config=instance.config.with_sensor(sensor))
        names[robot_id, atom.name] = atom.name
    world = WorldConfig(scene=scene, robots={robot_id: instance})
    generated = simulation(world=world, module=world_cls, names=names)

    swapped = {robot_atom.name: robot_atom, **{a.name: a for a in sensor_atoms}}
    _check_module_refs(blueprint, swapped)
    kept = [a for a in blueprint.blueprints if a.name not in swapped]
    remaps = {k: v for k, v in blueprint.remapping_map.items() if k[0] not in swapped}
    remaps.update({k: v for k, v in generated.remapping_map.items()})

    # The world's robot ports take the connection's stream names; truth ports it
    # never had keep the plain kind as their stream name.
    world_atom = generated.blueprints[0]
    connection_ports = {s.name: s for s in robot_atom.streams}
    for kind, attribute in robot_ports(world_cls, [robot_id])[robot_id].items():
        if kind in connection_ports:
            remaps[world_atom.name, attribute] = _effective(blueprint, robot_atom, kind)
        else:
            remaps[world_atom.name, attribute] = kind
    # A swapped sensor keeps the driver's stream names; its truth extras are qualified.
    for atom in sensor_atoms:
        replacement = next(a for a in generated.blueprints if a.name == atom.name)
        driver_ports = {s.name for s in atom.streams}
        for stream in replacement.streams:
            if stream.direction != "out" or stream.name == "tf":
                continue
            if stream.name in driver_ports:
                remaps[atom.name, stream.name] = _effective(blueprint, atom, stream.name)
            else:
                remaps[atom.name, stream.name] = f"{robot_id}/{atom.name}/{stream.name}"
    swapped_blueprint = replace(
        blueprint,
        blueprints=(*kept, *generated.blueprints),
        remapping_map=MappingProxyType(remaps),
    )
    return swapped_blueprint, {k: v for k, v in module_kwargs.items() if k not in swapped}


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
