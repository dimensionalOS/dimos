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

"""Collect resolved robot definitions before deploying one shared MuJoCo world."""

from __future__ import annotations

from dataclasses import replace
from typing import Any, cast

from dimos.core.coordination.blueprint_config.fields import module_config_cls
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import GlobalConfig
from dimos.sim2.blueprint import simulation
from dimos.sim2.module import SimulationModule, SimulationModuleConfig
from dimos.sim2.spec import RobotInstance, SimRobotSpec, WorldConfig
from dimos.sim2.world.scene import scene_path


def collect_simulation(
    blueprint: Blueprint,
    module_kwargs: dict[str, dict[str, Any]],
    config: GlobalConfig,
) -> tuple[Blueprint, dict[str, dict[str, Any]]]:
    """Expand opted-in connections after configuration, before transport/deployment.

    Module instance names are robot identities, including blueprint namespaces.
    No driver is instantiated and no hardware or simulator is opened here.
    An explicit SimulationModule supplies world settings; otherwise CLI scene
    selection and the stock logistics/workbench defaults are used.
    """
    if config.simulation != "mujoco":
        return blueprint, module_kwargs
    providers = [
        a for a in blueprint.active_blueprints if hasattr(a.module, "describe_simulated_robot")
    ]
    if not providers:
        return blueprint, module_kwargs
    for atom in providers:
        if not isinstance(atom.module, SimRobotSpec):
            raise TypeError(
                f"{atom.name} must declare default_sim_scene and describe_simulated_robot"
            )
    worlds = [a for a in blueprint.active_blueprints if issubclass(a.module, SimulationModule)]
    if len(worlds) > 1:
        raise ValueError("sim2 requires one shared SimulationModule, not one per robot namespace")
    if SimulationModule in blueprint.disabled_modules_tuple:
        raise ValueError("simulated robot connections require SimulationModule")
    if worlds:
        atom = worlds[0]
        values = {**atom.kwargs, **module_kwargs.get(atom.name, {})}
        settings = SimulationModuleConfig(**values)
        world_name = atom.name
    else:
        defaults = {cast("SimRobotSpec", a.module).default_sim_scene for a in providers}
        selected = config.scene_package
        if not selected and len(defaults) != 1:
            raise ValueError(
                "robots have different default scenes; select one shared --scene-package"
            )
        scene = scene_path(selected, sorted(defaults)[0])
        settings = SimulationModuleConfig(world=WorldConfig(scene, {}))
        world_name = SimulationModule.name

    robots: dict[str, RobotInstance] = dict(settings.world.robots)
    for atom in providers:
        values = {**atom.kwargs, **module_kwargs.get(atom.name, {})}
        values.update(g=config, instance_name=atom.name)
        resolved = module_config_cls(atom)(**values)
        instance = cast("SimRobotSpec", atom.module).describe_simulated_robot(
            resolved, settings.world.scene
        )
        if not isinstance(instance, RobotInstance):
            raise TypeError(f"{atom.name}.describe_simulated_robot must return RobotInstance")
        if atom.name in robots:
            raise ValueError(f"duplicate simulated robot definition {atom.name!r}")
        robots[atom.name] = instance
    hosted = {name for placement in blueprint.hosted_placements for name in placement.module_names}
    if hosted.intersection(robots):
        raise ValueError("sim2 robot device clients must run on the same host as physics")

    timestep = settings.world.timestep
    limit = min(robot.config.max_timestep for robot in robots.values())
    if worlds and timestep > limit:
        raise ValueError(f"world timestep {timestep} exceeds robot limit {limit}")
    world = replace(settings.world, robots=robots, timestep=min(timestep, limit))
    generated = simulation(
        world=world,
        sim_id=settings.sim_id,
        viewer=settings.viewer,
        instance_name=world_name,
        evidence_dir=settings.evidence_dir,
    )
    _, *sensors = generated.blueprints
    existing = {a.name: a for a in blueprint.blueprints}
    for sensor in sensors:
        original = existing.get(sensor.name)
        if original is not None and original.module is not sensor.module:
            raise ValueError(f"generated sim2 sensor conflicts with module {sensor.name!r}")
    # Explicit caller wiring and device settings take precedence. Only the world
    # robot map is replaced with the complete, validated set of definitions.
    expanded = autoconnect(generated, blueprint)
    kwargs = {name: dict(values) for name, values in module_kwargs.items()}
    kwargs[world_name] = {**kwargs.get(world_name, {}), "world": world}
    return expanded, kwargs
